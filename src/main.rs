//! Real-time DuckStation-Style Interactive Frontend and Headless CLI Entrypoint
//! Supports --bios, --exe, --cd-bin, --pad-buttons, --max-cycles, --trace, --dump-state-json, --dump-vram, and --window

pub mod bus;
pub mod cpu;
pub mod gamepad;
pub mod spu;
pub mod ui;

#[cfg(test)]
mod game_probe;

use bus::Bus;
use cpu::Cpu;
use gamepad::host_gamepad::GamepadPoller;
use minifb::{Key, KeyRepeat, MouseButton, MouseMode, Scale, Window, WindowOptions};
use spu::backend::AudioOutput;
use std::env;
use std::fs;
use ui::{ActiveMenu, ActiveToolWindow, DuckStationUi};

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut bios_path: Option<String> = None;
    let mut exe_path: Option<String> = None;
    let mut cd_bin_path: Option<String> = None;
    let mut pad_buttons: u16 = 0xFFFF;
    let mut max_cycles: u64 = 1_000_000;
    let mut max_cycles_specified = false;
    let mut trace = false;
    let mut dump_json: Option<String> = None;
    let mut dump_vram: Option<String> = None;
    let mut window_mode_opt: Option<bool> = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--bios" => {
                i += 1;
                bios_path = args.get(i).cloned();
            }
            "--exe" => {
                i += 1;
                exe_path = args.get(i).cloned();
            }
            "--cd-bin" => {
                i += 1;
                cd_bin_path = args.get(i).cloned();
            }
            "--pad-buttons" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    let trimmed = val.trim_start_matches("0x").trim_start_matches("0X");
                    if let Ok(parsed) = u16::from_str_radix(trimmed, 16) {
                        pad_buttons = parsed;
                    }
                }
            }
            "--max-cycles" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    max_cycles = val.parse().unwrap_or(1_000_000);
                    max_cycles_specified = true;
                }
            }
            "--trace" => trace = true,
            "--dump-state-json" => {
                i += 1;
                dump_json = args.get(i).cloned();
            }
            "--dump-vram" => {
                i += 1;
                dump_vram = args.get(i).cloned();
            }
            "--window" | "--gui" => {
                window_mode_opt = Some(true);
            }
            "--headless" | "--no-window" => {
                window_mode_opt = Some(false);
            }
            _ => {}
        }
        i += 1;
    }

    let window_mode = match window_mode_opt {
        Some(w) => w,
        None => dump_vram.is_none(),
    };

    let mut bus = Bus::new();
    bus.pad1_buttons = pad_buttons;
    let mut cpu = Cpu::new();
    cpu.trace_enabled = trace;

    if let Some(ref path) = bios_path {
        if let Err(e) = bus.load_bios(path) {
            eprintln!("Error loading BIOS: {}", e);
            std::process::exit(1);
        }
    }

    if let Some(ref path) = cd_bin_path {
        if let Err(e) = bus.load_cd_bin_file(path) {
            eprintln!("Error loading CD-ROM BIN image: {}", e);
            std::process::exit(1);
        }
    }

    if let Some(ref path) = exe_path {
        let exe_bytes = match fs::read(path) {
            Ok(data) => data,
            Err(e) => {
                eprintln!("Error reading PS-EXE: {}", e);
                std::process::exit(1);
            }
        };
        if bios_path.is_some() {
            if let Err(e) = bus.queue_exe_bytes(&exe_bytes) {
                eprintln!("Error queuing PS-EXE for BIOS shell hook: {}", e);
                std::process::exit(1);
            }
        } else if let Err(e) = cpu.sideload_exe(&mut bus, &exe_bytes) {
            eprintln!("Error side-loading bare-metal PS-EXE: {}", e);
            std::process::exit(1);
        }
    }

    if window_mode {
        let mut win_w: usize = 960;
        let mut win_h: usize = 640;
        let mut window = Window::new(
            "psx_ai_emu — DuckStation Edition (PS1 Emulator)",
            win_w,
            win_h,
            WindowOptions {
                resize: true,
                scale: Scale::X1,
                ..WindowOptions::default()
            },
        )
        .unwrap_or_else(|e| {
            eprintln!("Failed to create window: {}", e);
            std::process::exit(1);
        });

        println!("[psx_ai_emu] DuckStation-Style Frontend Started.");
        println!("Hotkeys:");
        println!("  F1 / F3:        Quick Save State / Quick Load State (Active Slot)");
        println!("  F2:             Cycle Active Save Slot (1..5)");
        println!("  F5 / Guide:     Toggle DualShock Analog (0x73) <-> Digital (0x41)");
        println!("  F6 / Pause:     Pause / Resume Emulation");
        println!("  F7:             Frame Step (Advance 1 Video Frame)");
        println!("  F8:             Reset System");
        println!("  F10:            Save VRAM Screenshot (.ppm)");
        println!("  Tab:            Hold or Toggle Fast-Forward (Turbo)");
        println!("  Escape:         Close Modal / Toggle Game List & Pause");

        let mut audio_out = AudioOutput::new();
        if audio_out.is_some() {
            println!("[Audio] 44.1 kHz 16-bit Stereo WaveOut Stream Initialized (SPU + CD-XA + Reverb).");
        } else {
            println!("[Audio] Warning: Could not open host audio output device; running silent.");
        }
        let mut gamepad_poller = GamepadPoller::new();
        let mut ui = DuckStationUi::new(bios_path.clone(), cd_bin_path.clone(), exe_path.clone());

        // If auto-detected BIOS during game library scan and none was passed via CLI, load it into bus if idle
        if bios_path.is_none() {
            if let Some(ref auto_bios) = ui.bios_path {
                let _ = bus.load_bios(auto_bios);
            }
        }

        window.set_target_fps(60);
        let mut fb = vec![0u32; win_w * win_h];
        let mut audio_samples = Vec::with_capacity(2048);

        while window.is_open() && !ui.request_exit {
            if max_cycles_specified && cpu.cycles >= max_cycles {
                break;
            }

            let (cur_w, cur_h) = window.get_size();
            if cur_w >= 320 && cur_h >= 240 && (cur_w != win_w || cur_h != win_h) {
                win_w = cur_w;
                win_h = cur_h;
                fb.resize(win_w * win_h, 0);
            }

            // Handle Escape: close open dropdown/modal/tool window first, or toggle Game List / Pause
            if window.is_key_pressed(Key::Escape, KeyRepeat::No) {
                if ui.active_menu != ActiveMenu::None {
                    ui.active_menu = ActiveMenu::None;
                } else if ui.settings_open {
                    ui.settings_open = false;
                } else if ui.tool_window != ActiveToolWindow::None {
                    ui.tool_window = ActiveToolWindow::None;
                } else if ui.running {
                    ui.show_game_list = !ui.show_game_list;
                } else {
                    break;
                }
            }

            // Function-key hotkeys
            if window.is_key_pressed(Key::F1, KeyRepeat::No) {
                ui.save_state_slot(ui.selected_slot, &cpu, &bus);
            }
            if window.is_key_pressed(Key::F2, KeyRepeat::No) {
                ui.selected_slot = (ui.selected_slot + 1) % 5;
                ui.push_osd(format!("Selected Save State Slot {}", ui.selected_slot + 1));
            }
            if window.is_key_pressed(Key::F3, KeyRepeat::No) {
                ui.load_state_slot(ui.selected_slot, &mut cpu, &mut bus);
            }
            if window.is_key_pressed(Key::F6, KeyRepeat::No) {
                ui.paused = !ui.paused;
                ui.push_osd(if ui.paused { "Emulation Paused" } else { "Emulation Resumed" });
            }
            if window.is_key_pressed(Key::F7, KeyRepeat::No) {
                ui.paused = true;
                ui.step_one_frame = true;
                ui.push_osd("Stepped 1 Frame");
            }
            if window.is_key_pressed(Key::F8, KeyRepeat::No) {
                ui.reset_system(&mut cpu, &mut bus);
            }
            if window.is_key_pressed(Key::F9, KeyRepeat::No) {
                ui.post_shader = ui.post_shader.next();
                ui.push_osd(format!("Post-Processing Shader: {}", ui.post_shader.label()));
            }
            if window.is_key_pressed(Key::F10, KeyRepeat::No) {
                let _ = bus.dump_vram_ppm("vram_screenshot.ppm");
                ui.push_osd("Saved VRAM Screenshot to vram_screenshot.ppm");
            }
            if window.is_key_pressed(Key::F11, KeyRepeat::No) {
                ui.texture_filter = ui.texture_filter.next();
                ui.bilinear_filtering = matches!(
                    ui.texture_filter,
                    ui::TextureFilterMode::BilinearViewport | ui::TextureFilterMode::Bilinear3DAndViewport
                );
                ui.push_osd(format!("Texture Filter: {}", ui.texture_filter.label()));
            }
            if window.is_key_pressed(Key::Tab, KeyRepeat::No) {
                ui.fast_forward = !ui.fast_forward;
                window.set_target_fps(if ui.fast_forward { 240 } else { 60 });
                ui.push_osd(format!(
                    "Fast Forward (Turbo): {}",
                    if ui.fast_forward { "ON" } else { "OFF" }
                ));
            }

            // Poll host XInput / WinMM gamepad (and pass DualShock rumble state if enabled)
            let (rumble_s, rumble_l) = if ui.rumble_enabled && ui.running && !ui.paused {
                (bus.pad1.motor_small, bus.pad1.motor_large)
            } else {
                (0, 0)
            };
            let gp = gamepad_poller.poll(rumble_s, rumble_l);

            if window.is_key_pressed(Key::F5, KeyRepeat::No) || gp.toggle_analog_pressed {
                let mode = bus.pad1.toggle_analog_mode();
                ui.push_osd(format!(
                    "Port 1 Controller Mode: {}",
                    if mode { "DualShock Analog (0x73)" } else { "Digital Pad (0x41)" }
                ));
            }

            // Keyboard + Host Gamepad input mapping to active-low PS1 pad
            let mut buttons_low: u8 = (pad_buttons & 0xFF) as u8 & gp.buttons_low;
            let mut buttons_high: u8 = ((pad_buttons >> 8) & 0xFF) as u8 & gp.buttons_high;

            // Byte 1: [Select, L3, R3, Start, Up, Right, Down, Left]
            if window.is_key_down(Key::RightShift) || window.is_key_down(Key::Backspace) {
                buttons_low &= !(1 << 0); // Select
            }
            if window.is_key_down(Key::N) {
                buttons_low &= !(1 << 1); // L3
            }
            if window.is_key_down(Key::M) {
                buttons_low &= !(1 << 2); // R3
            }
            if window.is_key_down(Key::Enter) {
                buttons_low &= !(1 << 3); // Start
            }
            if window.is_key_down(Key::Up) || window.is_key_down(Key::W) {
                buttons_low &= !(1 << 4); // Up
            }
            if window.is_key_down(Key::Right) || window.is_key_down(Key::D) {
                buttons_low &= !(1 << 5); // Right
            }
            if window.is_key_down(Key::Down) || window.is_key_down(Key::S) {
                buttons_low &= !(1 << 6); // Down
            }
            if window.is_key_down(Key::Left) || window.is_key_down(Key::A) {
                buttons_low &= !(1 << 7); // Left
            }

            // Byte 2: [L2, R2, L1, R1, Triangle, Circle, Cross, Square]
            if window.is_key_down(Key::Key1) || window.is_key_down(Key::U) {
                buttons_high &= !(1 << 0); // L2
            }
            if window.is_key_down(Key::Key2) || window.is_key_down(Key::O) {
                buttons_high &= !(1 << 1); // R2
            }
            if window.is_key_down(Key::Q) || window.is_key_down(Key::Key3) {
                buttons_high &= !(1 << 2); // L1
            }
            if window.is_key_down(Key::E) || window.is_key_down(Key::Key4) {
                buttons_high &= !(1 << 3); // R1
            }
            if window.is_key_down(Key::V) || window.is_key_down(Key::I) {
                buttons_high &= !(1 << 4); // Triangle
            }
            if window.is_key_down(Key::X) || window.is_key_down(Key::L) {
                buttons_high &= !(1 << 5); // Circle
            }
            if window.is_key_down(Key::Z) || window.is_key_down(Key::K) || window.is_key_down(Key::Space) {
                buttons_high &= !(1 << 6); // Cross
            }
            if window.is_key_down(Key::C) || window.is_key_down(Key::J) {
                buttons_high &= !(1 << 7); // Square
            }

            // Analog stick axes (from host gamepad + keyboard T/F/G/H for right stick)
            let mut lx = gp.left_x;
            let mut ly = gp.left_y;
            let mut rx = gp.right_x;
            let mut ry = gp.right_y;

            if lx == 0x80 {
                if window.is_key_down(Key::Left) || window.is_key_down(Key::A) {
                    lx = 0x00;
                } else if window.is_key_down(Key::Right) || window.is_key_down(Key::D) {
                    lx = 0xFF;
                }
            }
            if ly == 0x80 {
                if window.is_key_down(Key::Up) || window.is_key_down(Key::W) {
                    ly = 0x00;
                } else if window.is_key_down(Key::Down) || window.is_key_down(Key::S) {
                    ly = 0xFF;
                }
            }
            if rx == 0x80 {
                if window.is_key_down(Key::F) {
                    rx = 0x00;
                } else if window.is_key_down(Key::H) {
                    rx = 0xFF;
                }
            }
            if ry == 0x80 {
                if window.is_key_down(Key::T) {
                    ry = 0x00;
                } else if window.is_key_down(Key::G) {
                    ry = 0xFF;
                }
            }

            ui.last_pad_low = buttons_low;
            ui.last_pad_high = buttons_high;
            ui.last_lx = lx;
            ui.last_ly = ly;
            ui.last_rx = rx;
            ui.last_ry = ry;

            bus.set_pad_state(buttons_low, buttons_high);
            bus.set_pad_analog(lx, ly, rx, ry);

            // Step Emulation if Running and Not Paused (or if single-stepping)
            if ui.running {
                if ui.step_one_instruction {
                    ui.step_one_instruction = false;
                    cpu.step(&mut bus);
                } else if !ui.paused || ui.step_one_frame {
                    ui.step_one_frame = false;
                    let frame_cycles = ui.cycles_per_frame_scaled();
                    let frame_limit = cpu.cycles + frame_cycles + 100_000;
                    while bus.vblank_triggered && cpu.cycles < frame_limit {
                        cpu.step_batch(&mut bus, 64);
                    }
                    while !bus.vblank_triggered && cpu.cycles < frame_limit {
                        cpu.step_batch(&mut bus, 64);
                    }
                    ui.update_fps();
                }
            }

            // Drain synthesized 44.1 kHz stereo audio samples and submit to low-latency WaveOut stream
            bus.drain_audio_samples_into(&mut audio_samples);
            if !audio_samples.is_empty() {
                ui.push_scope_samples(&audio_samples);
                let muted = !ui.audio_enabled || (ui.fast_forward && ui.mute_in_fast_forward);
                if let Some(ref mut out) = audio_out {
                    out.submit_samples(&audio_samples, ui.master_volume, muted);
                }
            }

            // Query Mouse Position, Left-Click, and Scroll Wheel for DuckStation UI
            let (mx, my) = window
                .get_mouse_pos(MouseMode::Clamp)
                .map(|(x, y)| (x as i32, y as i32))
                .unwrap_or((0, 0));
            let mouse_down = window.get_mouse_down(MouseButton::Left);
            let scroll_y = window.get_scroll_wheel().map(|(_, sy)| sy).unwrap_or(0.0);

            ui.render_and_interact(
                &mut fb,
                win_w,
                win_h,
                mx,
                my,
                mouse_down,
                scroll_y,
                &mut cpu,
                &mut bus,
            );

            if let Err(e) = window.update_with_buffer(&fb, win_w, win_h) {
                eprintln!("Window update error: {}", e);
                break;
            }
        }
    } else {
        while cpu.cycles < max_cycles {
            cpu.step(&mut bus);
        }
    }

    if let Some(json_path) = dump_json {
        let json = cpu.dump_state_json(&bus.tty_buffer);
        let _ = fs::write(json_path, json);
    }

    if let Some(vram_path) = dump_vram {
        println!(
            "[GPU] Final Display Viewport: {}x{} at ({}, {}) (24-bit: {})",
            bus.display_width,
            bus.display_height,
            bus.display_vram_x,
            bus.display_vram_y,
            (bus.gpustat & (1 << 21)) != 0
        );
        let _ = bus.dump_vram_ppm(vram_path);
    }
}
