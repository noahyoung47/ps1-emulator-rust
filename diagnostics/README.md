# Compatibility investigation — 2026-10-25

## Changes

The executable is rebuilt at `target/release/psx_ai_emu.exe`.

- CD sectors now arrive at 75 sectors/second or 150 in double-speed mode, instead of one every 2,048 CPU cycles. Command completion delays remain separate. This fixes the reproduced Crash asset-loading failure.
- ReadN/ReadS report the reading status bit (bit 5), rather than the CD-DA playback bit (bit 7).
- GetlocL returns the received sector's actual header and XA subheader. GetlocP uses the current drive position, rather than the next sector address. Seeks update that position too.
- XA audio/realtime sectors bypass the CPU's data-ready/movie parser when XA decoding is enabled. Previously, an audio sector interrupted FFVII's sequence of STR video chunks, preventing its first movie frame from reaching MDEC. XA audio playback/mixing remains unimplemented; this change routes/suppresses the audio data rather than producing sound.
- MDEC uses the correct inverse zigzag scan, bypasses zigzag for q-scale zero, uses the hardware scale-table values, and applies the scale table's 13 significant bits in both IDCT passes.
- Request-mode MDEC output DMA waits for decoded blocks, retains its remaining block count/address, and resumes when the decoder produces data. It no longer writes zeros and raises completion when the decoder is empty.
- Byte and halfword writes to DICR now update the addressed control/acknowledgement lanes while preserving other controls and pending flags. FFVII uses `LBU/SB DICR+2` to suppress DMA3 callbacks for intermediate STR sectors. The missing byte writes caused premature frame-complete callbacks and the first-frame freeze.
- `--trace` now emits instruction addresses, words, and registers to stderr; previously it set an unused flag.

Hardware references: [CD controller](https://psx-spx.consoledev.net/dev/ps1/cdr/cdromdrive/), [MDEC](https://psx-spx.consoledev.net/macroblockdecodermdec/), [DMA registers](https://psx-spx.consoledev.net/dmachannels/), [STR/XA interleaving](https://psx-spx.consoledev.net/cdromfileformats/).

## Results and remaining issues

- `cargo test --release`: **38 passed**, 0 failed; 1 opt-in local-disc probe ignored by default. Six regression tests added for sector cadence/location, XA routing, DMA waiting/resumption, IDCT, RLE scan order, and partial DICR writes. The old MDEC pixel expectation was corrected to the independently derived DC level.
- Crash: before the timing fix, starting gameplay produced address error `BadVAddr=0x00000006`, EPC `0x80032188`. The game had received `-10` from an unresolved asset lookup and used it as a pointer. After the fix, it reaches N. Sanity Beach and renders gameplay; a 1.5-billion-instruction run completed without that exception. See `crash-final/gameplay.png`.
- FFVII: **the reported movie freeze is fixed**. A final headless boot selected New Game, then released all buttons. It decoded **601 consecutive movie frames** (about 40 seconds of movie time), completed output DMA, and produced changing display snapshots without `MDEC_out_sync timeout`. See `ff7-verified/verified.png` and `ff7-verified.log`. The CPU/DMA/MDEC timing experiments were reverted: the final production fix for this follow-up is DICR partial writes, not a timing workaround. A full game playthrough is not claimed.
- Ridge Racer: a 1.3-billion-instruction headless regression run reaches the race without unexpected CPU exceptions (see `ridge-final/gameplay.png`). Scripted Start/Cross input also opens its pause menu. This checks boot/race progression, not real-time GUI performance.
- The supplied `psxtest_cpu.exe` ran for 100 million instructions, reached `Done`, and printed `Result: 00000101`. No reference result was established, so this is **not claimed as a passing CPU conformance test**. See `investigation/cpu_test.log`.
- BIOS/game VSync timeout messages still occur. CPU instruction timing, CD command/sector scheduling, partial/in-flight MDEC input, and other hardware behaviors remain simplified. No claim of cycle accuracy or full-game compatibility is made.

## Reproduce (PowerShell, from src_repo)

```powershell
cargo test --release
cargo build --release

$env:GAME_BIN = 'Crash Bandicoot (USA).bin'
$env:GAME_CYCLES = '1500000000'
$env:GAME_OUT = 'diagnostics/crash-final'
Remove-Item Env:GAME_UP -ErrorAction SilentlyContinue
cargo test --release game_probe -- --ignored --nocapture

$env:GAME_BIN = 'Final Fantasy VII (USA) (Disc 1).bin'
$env:GAME_CYCLES = '4000000000'
$env:GAME_OUT = 'diagnostics/ff7-verified'
$env:GAME_MIN_MOVIE_FRAMES = '600'
$env:GAME_UP = '1'
cargo test --release game_probe -- --ignored --nocapture

Remove-Item Env:GAME_MIN_MOVIE_FRAMES -ErrorAction SilentlyContinue
$env:GAME_BIN = 'Ridge Racer (Track 01).bin'
$env:GAME_CYCLES = '1300000000'
$env:GAME_OUT = 'diagnostics/ridge-final'
Remove-Item Env:GAME_UP -ErrorAction SilentlyContinue
cargo test --release game_probe -- --ignored --nocapture

.\target\release\psx_ai_emu.exe --bios SCPH1001.BIN --cd-bin 'Crash Bandicoot (USA).bin' --window
```

The opt-in probe sends brief Start/Cross presses after BIOS boot. With GAME_UP set, it switches to repeated Up/Circle presses after 600 million instructions to select FFVII's New Game, then releases the pad once streaming starts. GAME_MIN_MOVIE_FRAMES makes the probe fail if playback stalls before the requested number of frames; the probe also rejects an MDEC output timeout. Set GAME_HISTORY=1 to collect the last 32 instructions around an unexpected CPU exception. Snapshots include full VRAM and the active display; the display export correctly handles 24-bit movie mode.
