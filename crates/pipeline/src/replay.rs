//! Versioned JSONL sensor logs, streamed verification with a mandatory count footer.
mod finite;
use crate::{DrivingPipeline, PipelineConfig, PipelineOutput, SensorFrame};
use serde::{Deserialize, Serialize};
use std::io::{BufRead, Write};
pub const LOG_SCHEMA_VERSION: u32 = 1;
const MAX_LINE_BYTES: u64 = 8 * 1024 * 1024;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LogHeader {
    pub schema_version: u32,
    pub source: String,
    pub config: PipelineConfig,
}
impl LogHeader {
    fn validate_metadata(&self) -> Result<(), String> {
        if self.schema_version != LOG_SCHEMA_VERSION || self.source.is_empty() {
            return Err("invalid/unsupported log header".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedTick {
    pub input: SensorFrame,
    pub expected: PipelineOutput,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LogRecord {
    Header { header: Box<LogHeader> },
    Tick { tick: Box<RecordedTick> },
    End { ticks: usize },
}
#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum LogRecordRef<'a> {
    Header { header: &'a LogHeader },
    Tick { tick: &'a RecordedTick },
    End { ticks: usize },
}
#[derive(Clone, Debug)]
pub struct SensorLog {
    pub header: LogHeader,
    pub ticks: Vec<RecordedTick>,
}
impl SensorLog {
    pub fn new(source: impl Into<String>, config: PipelineConfig) -> Self {
        Self {
            header: LogHeader {
                schema_version: LOG_SCHEMA_VERSION,
                source: source.into(),
                config,
            },
            ticks: vec![],
        }
    }
    pub fn record(&mut self, input: SensorFrame, expected: PipelineOutput) {
        self.ticks.push(RecordedTick { input, expected });
    }
    /// Each encoded line, including its newline, must fit the reader's 8 MiB limit.
    /// Nonfinite numbers, including optional values, are rejected before JSON encoding.
    /// Invalid/oversized records are not emitted; earlier complete records may remain.
    /// Frame clocks are preflighted before writing; the header must construct a pipeline.
    pub fn write(&self, mut writer: impl Write) -> Result<(), String> {
        if self.ticks.is_empty() {
            return Err("cannot write an empty sensor log".into());
        }
        validate_frame_times(&self.ticks)?;
        let mut buffer = Vec::with_capacity(MAX_LINE_BYTES as usize);
        write_record(
            &mut writer,
            &LogRecordRef::Header {
                header: &self.header,
            },
            &mut buffer,
        )
        .map_err(|e| format!("log header: {e}"))?;
        for (index, tick) in self.ticks.iter().enumerate() {
            write_record(&mut writer, &LogRecordRef::Tick { tick }, &mut buffer)
                .map_err(|e| format!("tick {index}: {e}"))?;
        }
        write_record(
            &mut writer,
            &LogRecordRef::End {
                ticks: self.ticks.len(),
            },
            &mut buffer,
        )
        .map_err(|e| format!("log footer: {e}"))?;
        writer.flush().map_err(|e| e.to_string())
    }
}
fn validate_frame_times(ticks: &[RecordedTick]) -> Result<(), String> {
    let mut previous = None;
    for (index, tick) in ticks.iter().enumerate() {
        let time = tick.input.time;
        if !time.is_finite() {
            return Err(format!("tick {index}: non-finite frame time"));
        }
        if time < 0.0 || previous.is_some_and(|last| time <= last) {
            return Err(format!(
                "tick {index}: frame time must be nonnegative and strictly increasing"
            ));
        }
        if !tick.expected.time.is_finite() {
            return Err(format!("tick {index}: non-finite expected output time"));
        }
        if tick.expected.time.to_bits() != time.to_bits() {
            return Err(format!(
                "tick {index}: expected output time must match frame time exactly"
            ));
        }
        previous = Some(time);
    }
    Ok(())
}
struct RecordBuffer<'a> {
    bytes: &'a mut Vec<u8>,
}
impl Write for RecordBuffer<'_> {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        // Reserve one byte for the final newline, which the reader also counts.
        if self.bytes.len().saturating_add(bytes.len()) >= MAX_LINE_BYTES as usize {
            return Err(std::io::Error::other(
                "sensor log record exceeds 8 MiB line size limit",
            ));
        }
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
fn write_record(
    writer: &mut impl Write,
    record: &LogRecordRef<'_>,
    buffer: &mut Vec<u8>,
) -> Result<(), String> {
    buffer.clear();
    record
        .serialize(finite::FiniteNumbers)
        .map_err(|e| e.to_string())?;
    serde_json::to_writer(&mut RecordBuffer { bytes: buffer }, record)
        .map_err(|e| e.to_string())?;
    if let LogRecordRef::Header { header } = record {
        header.validate_metadata()?;
        // Bound configuration collections before cloning and checking startup.
        header.config.validate()?;
        drop(DrivingPipeline::new(header.config.clone())?);
    }
    buffer.push(b'\n');
    writer.write_all(buffer).map_err(|e| e.to_string())
}
#[derive(Debug, Serialize)]
pub struct ReplayReport {
    pub source: String,
    pub ticks: usize,
    pub verified: bool,
}
/// Recompute every output from observations only; expected outputs never enter the pipeline.
/// Reject empty/truncated logs, schema changes, reordered clocks and output mismatches.
pub fn verify(mut reader: impl BufRead, mut outputs: impl Write) -> Result<ReplayReport, String> {
    let mut pipeline = None;
    let mut source = String::new();
    let mut ticks = 0;
    let mut ended = false;
    let mut line_number = 0;
    loop {
        let mut line = String::new();
        let length = std::io::Read::take(&mut reader, MAX_LINE_BYTES + 1)
            .read_line(&mut line)
            .map_err(|e| e.to_string())?;
        if length == 0 {
            break;
        }
        line_number += 1;
        if length as u64 > MAX_LINE_BYTES {
            return Err(format!("line {line_number} exceeds size limit"));
        }
        if ended {
            return Err("data after log footer".into());
        }
        let record: LogRecord =
            serde_json::from_str(&line).map_err(|e| format!("line {line_number}: {e}"))?;
        match record {
            LogRecord::Header { header } => {
                if pipeline.is_some() || line_number != 1 {
                    return Err("invalid/unsupported log header".into());
                }
                header.validate_metadata()?;
                source = header.source;
                pipeline = Some(DrivingPipeline::new(header.config)?);
            }
            LogRecord::Tick { tick } => {
                let stack = pipeline.as_mut().ok_or("tick before log header")?;
                let actual = stack
                    .step(&tick.input)
                    .map_err(|e| format!("tick {ticks}: {e}"))?;
                if serde_json::to_vec(&actual).map_err(|e| e.to_string())?
                    != serde_json::to_vec(&tick.expected).map_err(|e| e.to_string())?
                {
                    return Err(format!(
                        "replay mismatch at tick {ticks} (time {:.3})",
                        tick.input.time
                    ));
                }
                serde_json::to_writer(&mut outputs, &actual).map_err(|e| e.to_string())?;
                outputs.write_all(b"\n").map_err(|e| e.to_string())?;
                ticks += 1;
            }
            LogRecord::End { ticks: expected } => {
                if pipeline.is_none() || ticks == 0 || ticks != expected {
                    return Err("invalid footer or zero/truncated tick sequence".into());
                }
                ended = true;
            }
        }
    }
    if !ended {
        return Err("missing log footer (empty or truncated log)".into());
    }
    outputs.flush().map_err(|e| e.to_string())?;
    Ok(ReplayReport {
        source,
        ticks,
        verified: true,
    })
}
