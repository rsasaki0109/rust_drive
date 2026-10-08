# Contributing

Start with the working simulator and a small reproducible scenario. Read [architecture](docs/architecture.md), [capabilities](docs/capabilities.md), and [development](docs/development.md) before changing interfaces.

1. Build and run `cargo test --workspace --locked`.
2. Make a focused implementation with a meaningful regression test or independently scored scenario.
3. Run `bash scripts/check.sh`; update English documentation if contracts or demonstrated behavior change.
4. Include the trigger, before/after behavior, tests run and remaining limitations in a pull request.

Use SI units and explicit frames/timestamps. Avoid unchecked numeric inputs, hidden simulator truth, new global runtimes, or placeholders reported as implemented features. Do not commit generated `target/` or `artifacts/`. Refresh README media intentionally and retain provenance. Record license and provenance for any external code, model or dataset.

Original contributions are licensed under Apache-2.0. Planned real-vehicle work requires a separate safety engineering review; this repository currently targets simulation.
