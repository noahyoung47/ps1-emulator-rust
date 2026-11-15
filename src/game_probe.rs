//! Opt-in local-disc diagnostic: GAME_BIN=... cargo test --release game_probe -- --ignored --nocapture
use crate::{bus::Bus, cpu::Cpu};

#[test]
#[ignore = "requires local BIOS and game image; runs hundreds of millions of instructions"]
fn game_probe() {
    let game = std::env::var("GAME_BIN").expect("set GAME_BIN to a local disc image");
    let limit: u64 = std::env::var("GAME_CYCLES")
        .unwrap_or("600000000".into())
        .parse()
        .unwrap();
    let output =
        std::path::PathBuf::from(std::env::var("GAME_OUT").unwrap_or("diagnostics/probe".into()));
    std::fs::create_dir_all(&output).unwrap();
    let mut bus = Bus::new();
    bus.load_bios("SCPH1001.BIN").unwrap();
    bus.load_cd_bin_file(game).unwrap();
    let mut cpu = Cpu::new();
    let mut next_report = 10_000_000;
    let mut next_snapshot = 100_000_000;
    let mut history = std::collections::VecDeque::new();
    let keep_history = std::env::var_os("GAME_HISTORY").is_some();
    let minimum_frames: u64 = std::env::var("GAME_MIN_MOVIE_FRAMES")
        .unwrap_or("0".into())
        .parse()
        .unwrap();
    let mut streaming_reached = false;
    let mut movie_frames = 0u64;
    let up = std::env::var_os("GAME_UP").is_some();
    let confirm = if up { 0xdfff } else { 0xbfff };
    let mut fault = false;
    while cpu.cycles < limit {
        streaming_reached |= bus.cd_mode & 0x40 != 0 && bus.cd_reading;
        if up && cpu.pc == 0x80042044 {
            movie_frames += 1;
        }
        // Press Start then Cross briefly every ~3 emulated seconds, after BIOS boot.
        let phase = cpu.cycles % 100_000_000;
        bus.pad1_buttons = if cpu.cycles < 100_000_000 {
            0xffff
        } else if phase < 2_000_000 {
            0xfff7
        } else if (20_000_000..22_000_000).contains(&phase) {
            confirm
        } else {
            0xffff
        };
        if cpu.cycles > 300_000_000 {
            if keep_history {
                history.push_back((cpu.pc, bus.read32(cpu.pc)));
                if history.len() > 32 {
                    history.pop_front();
                }
            }
            if (cpu.cop0[13] & 0x7c) != 0 && (cpu.cop0[13] & 0x7c) != 0x20 && cpu.pc == 0x80000080 {
                eprintln!("BAD EXECUTION {:08x?}", history);
                fault = true;
                break;
            }
        }
        if up && cpu.cycles > 600_000_000 {
            // FFVII: select New Game with Up/Circle, then release the pad
            // throughout playback so the movie regression cannot skip frames.
            bus.pad1_buttons = if streaming_reached {
                0xffff
            } else if (10_000_000..12_000_000).contains(&phase) {
                0xffef
            } else if (20_000_000..22_000_000).contains(&phase) {
                0xdfff
            } else {
                0xffff
            };
        }
        cpu.step(&mut bus);
        if minimum_frames > 0
            && movie_frames > minimum_frames
            && bus.dma[0].chcr & (1 << 24) == 0
            && bus.dma[1].chcr & (1 << 24) == 0
            && bus.mdec_words_remaining == 0
            && bus.mdec_out_fifo.is_empty()
        {
            break;
        }
        if cpu.cycles >= next_report {
            eprintln!("PROBE {} pc={:08x} ra={:08x} irq={:x}/{:x} cd={:x} mode={:x} lba={} seek={} pending={} mdec={} out={} dma1={:x} gpuwords={}", cpu.cycles, cpu.pc, cpu.gpr[31], bus.i_stat, bus.i_mask, bus.cd_irq_flags, bus.cd_mode, bus.read_lba, bus.seek_lba, bus.cd_pending_irqs.len(), bus.mdec_words_remaining, bus.mdec_out_fifo.len(), bus.dma[1].chcr, bus.gp0_command_log.len());
            next_report += 10_000_000;
            if cpu.cycles >= next_snapshot {
                bus.dump_vram_ppm(output.join(format!("vram_{}.ppm", next_snapshot)))
                    .unwrap();
                dump_display(&bus, next_snapshot, &output);
                next_snapshot += 100_000_000;
            }
        }
    }
    std::fs::write(
        output.join("state.json"),
        cpu.dump_state_json(&bus.tty_buffer),
    )
    .unwrap();
    eprintln!("REGS {:08x?}", cpu.gpr);
    eprintln!("BADVADDR {:08x}", cpu.cop0[8]);
    eprintln!("MOVIE FRAMES {}", movie_frames);
    dump_display(&bus, cpu.cycles, &output);
    std::fs::write(output.join("ram.bin"), &bus.ram).unwrap();
    for offset in 0..16 {
        let addr = cpu.pc.wrapping_add(offset * 4);
        eprintln!("{:08x}: {:08x}", addr, bus.read32(addr));
    }
    assert!(
        !fault,
        "unexpected CPU exception; see diagnostic state and instruction history"
    );
    assert!(
        movie_frames >= minimum_frames,
        "only {} movie frames, expected at least {}",
        movie_frames,
        minimum_frames
    );
    assert!(
        !bus.tty_buffer.contains("MDEC_out_sync timeout"),
        "MDEC output timed out"
    );
}

fn dump_display(bus: &Bus, cycles: u64, output: &std::path::Path) {
    let (w, h) = (bus.display_width as usize, bus.display_height as usize);
    let mut bytes = format!("P6\n{} {}\n255\n", w, h).into_bytes();
    for y in 0..h {
        let row = (bus.display_vram_y as usize + y) % 512 * 1024;
        for x in 0..w {
            if bus.gpustat & (1 << 21) != 0 {
                for c in 0..3 {
                    let offset = bus.display_vram_x as usize * 2 + x * 3 + c;
                    bytes.push((bus.vram[row + offset / 2 % 1024] >> (offset % 2 * 8)) as u8);
                }
            } else {
                let p = bus.vram[row + (bus.display_vram_x as usize + x) % 1024];
                bytes.extend_from_slice(&[
                    ((p & 31) << 3) as u8,
                    (((p >> 5) & 31) << 3) as u8,
                    (((p >> 10) & 31) << 3) as u8,
                ]);
            }
        }
    }
    std::fs::write(output.join(format!("display_{}.ppm", cycles)), bytes).unwrap();
}
