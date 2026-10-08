//! Versioned JSONL sensor logs, streamed verification with a mandatory count footer.
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
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedTick {
    pub input: SensorFrame,
    pub expected: PipelineOutput,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LogRecord {
    Header { header: LogHeader },
    Tick { tick: RecordedTick },
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
    pub fn write(&self, mut writer: impl Write) -> Result<(), String> {
        if self.ticks.is_empty() {
            return Err("cannot write an empty sensor log".into());
        }
        write_record(
            &mut writer,
            &LogRecord::Header {
                header: self.header.clone(),
            },
        )?;
        for tick in &self.ticks {
            write_record(&mut writer, &LogRecord::Tick { tick: tick.clone() })?;
        }
        write_record(
            &mut writer,
            &LogRecord::End {
                ticks: self.ticks.len(),
            },
        )?;
        writer.flush().map_err(|e| e.to_string())
    }
}
fn write_record(writer: &mut impl Write, record: &LogRecord) -> Result<(), String> {
    serde_json::to_writer(&mut *writer, record).map_err(|e| e.to_string())?;
    writer.write_all(b"\n").map_err(|e| e.to_string())
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
                if pipeline.is_some()
                    || line_number != 1
                    || header.schema_version != LOG_SCHEMA_VERSION
                    || header.source.is_empty()
                {
                    return Err("invalid/unsupported log header".into());
                }
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
