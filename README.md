# PlayStation 1 Emulator in Rust

A work-in-progress PlayStation 1 emulator written in Rust, with an interactive desktop interface and a headless mode for debugging and hardware experiments.

The project explores CPU instruction execution, memory-mapped hardware, graphics, audio, controller protocols, and the timing interactions between console subsystems.

## Features

- CPU emulation with exception handling and instruction tracing.
- Memory bus, DMA transfers, interrupts, and CD-ROM handling.
- Graphics rendering, VRAM export, and MDEC video decoding.
- SPU audio synthesis and a Windows audio output backend.
- Digital and analog controller emulation, with Windows XInput and WinMM controller support.
- Desktop interface for configuring and running the emulator.
- Headless execution with cycle limits, state exports, and diagnostic output.

## Current status

This is an experimental emulator under active development. Game compatibility, performance, and hardware timing are incomplete; it is not cycle accurate, and full-game compatibility is not claimed.

Development investigations have exercised boot, gameplay, and video playback paths in selected games. These checks do not establish that an entire game is playable. Historical investigation notes and reproduction commands are in [diagnostics/README.md](diagnostics/README.md); generated diagnostic artifacts referenced there are kept locally and are not included in this repository.

## Build and run

Install the Rust toolchain and the native build tools required by your platform. Windows is the primary development platform; audio and host-controller backends use Windows APIs.

```powershell
git clone https://github.com/noahyoung47/ps1-emulator-rust.git
cd ps1-emulator-rust
cargo build --release
```

Launch the desktop interface:

```powershell
cargo run --release -- --window
```

Supply your own BIOS and disc image to boot a game:

```powershell
cargo run --release -- --bios "C:\path\to\SCPH1001.BIN" --cd-bin "C:\path\to\game.bin" --window
```

BIOS files, commercial game images, and downloaded test suites are not distributed with this project. They are excluded from version control.

## Debugging and testing

Run the automated tests:

```powershell
cargo test --release
```

Run a bounded headless session and export its state and VRAM:

```powershell
cargo run --release -- --bios "C:\path\to\SCPH1001.BIN" --cd-bin "C:\path\to\game.bin" --headless --max-cycles 1000000 --dump-state-json "state.json" --dump-vram "vram.ppm"
```

Additional options include `--exe` for PS-X executables, `--trace` for instruction tracing, and `--pad-buttons` for scripted controller input. The opt-in game probe requires local BIOS and disc files and is ignored by the default test run.

## Source guide

| File | Role |
| --- | --- |
| `src/cpu.rs` | CPU instruction execution and exceptions |
| `src/bus.rs` | Memory map, graphics, DMA, CD-ROM, and other device behavior |
| `src/spu.rs` | Audio synthesis and output |
| `src/gamepad.rs` | Emulated controllers and host-controller input |
| `src/ui.rs` | Desktop interface |
| `src/main.rs` | Application entry point, command-line options, and execution loop |
| `src/game_probe.rs` | Opt-in integration diagnostics using local game files |

## Development priorities

- Improve CPU and peripheral timing fidelity.
- Expand regression coverage for hardware behavior.
- Investigate remaining compatibility and playback issues.
- Improve real-time performance and desktop usability.
