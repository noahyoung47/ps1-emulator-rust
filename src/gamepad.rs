//! PS1 DualShock (SCPH-1200) / Digital Controller Emulation & Host XInput / WinMM Gamepad Backend.
//!
//! Features:
//! - Emulates both PS1 Standard Digital Pad (`ID = 0x415A`, 2 button bytes) and
//!   PS1 DualShock Analog Controller (`ID = 0x735A`, 2 button bytes + 4 analog thumbstick bytes: RX, RY, LX, LY)
//! - Supports Full DualShock Configuration Protocol (`0x42` Read/Rumble, `0x43` Enter/Exit Config Mode,
//!   `0x44` Set Analog/Digital Mode & LED Lock, `0x45` Query Model/LED Status, `0x46`/`0x47`/`0x4C` Capabilities,
//!   `0x4D` Map Rumble Motors)
//! - Dynamic Runtime Mode Toggle (F5 or Host Controller Guide/Back+Start) + Automatic Analog Activation when
//!   thumbsticks are deflected or when a game enables Analog Mode via `0x44`
//! - Zero-dependency Windows XInput (`xinput1_4.dll` / `xinput1_3.dll` / `xinput9_1_0.dll`) + `winmm` `joyGetPosEx`
//!   fallback for any DirectInput/XInput controller, including DualShock Rumble pass-through to XInput vibration motors!

#[derive(Clone, Debug)]
pub struct DualShockController {
    /// Active-low 16-bit button state:
    /// Low byte:  [Select, L3, R3, Start, Up, Right, Down, Left]
    /// High byte: [L2, R2, L1, R1, Triangle, Circle, Cross, Square]
    pub buttons: u16,
    /// Right stick X (`0x00` = Left, `0x80` = Center, `0xFF` = Right)
    pub right_x: u8,
    /// Right stick Y (`0x00` = Up, `0x80` = Center, `0xFF` = Down)
    pub right_y: u8,
    /// Left stick X (`0x00` = Left, `0x80` = Center, `0xFF` = Right)
    pub left_x: u8,
    /// Left stick Y (`0x00` = Up, `0x80` = Center, `0xFF` = Down)
    pub left_y: u8,
    /// Whether controller is in Analog (DualShock `0x73`) mode vs Digital (`0x41`) mode
    pub analog_mode: bool,
    /// Whether controller is currently inside `0x43` Configuration Mode (`0xF3`)
    pub config_mode: bool,
    /// Whether the physical Analog button is locked by command `0x44` (`0x03`)
    pub mode_locked: bool,
    /// Rumble motor mapping configured via command `0x4D` (default `[0xFF; 6]`)
    pub motor_map: [u8; 6],
    /// Small (right) fast-motor state (`0` = off, `1` = on)
    pub motor_small: u8,
    /// Large (left) variable-speed motor state (`0x00..=0xFF`)
    pub motor_large: u8,
    /// Current multi-byte packet response buffer for the active SIO0 transaction
    pub tx_response: Vec<u8>,
    /// Current command being processed in the active SIO0 transaction
    pub current_cmd: u8,
    /// Current sub-index parameter latch for commands `0x46` and `0x4C`
    pub sub_param: u8,
}

impl Default for DualShockController {
    fn default() -> Self {
        Self::new()
    }
}

impl DualShockController {
    pub fn new() -> Self {
        Self {
            buttons: 0xFFFF,
            right_x: 0x80,
            right_y: 0x80,
            left_x: 0x80,
            left_y: 0x80,
            analog_mode: false,
            config_mode: false,
            mode_locked: false,
            motor_map: [0xFF; 6],
            motor_small: 0,
            motor_large: 0,
            tx_response: Vec::with_capacity(8),
            current_cmd: 0,
            sub_param: 0,
        }
    }

    pub fn reset_transfer(&mut self) {
        self.tx_response.clear();
        self.current_cmd = 0;
        self.sub_param = 0;
    }

    pub fn toggle_analog_mode(&mut self) -> bool {
        if !self.mode_locked {
            self.analog_mode = !self.analog_mode;
            if !self.analog_mode {
                self.right_x = 0x80;
                self.right_y = 0x80;
                self.left_x = 0x80;
                self.left_y = 0x80;
            }
        }
        self.analog_mode
    }

    /// Process a byte after device address `0x01` has been selected.
    /// `step` starts at `1` (the command byte, e.g., `0x42`, `0x43`, etc.).
    /// Returns `(response_byte, has_more_bytes_with_ack)`.
    pub fn process_sio_byte(&mut self, step: u8, val: u8, pad1_buttons: u16) -> (u8, bool) {
        self.buttons = pad1_buttons;

        if step == 1 {
            self.current_cmd = val;
            self.sub_param = 0;
            self.tx_response.clear();

            let mode_id = if self.config_mode {
                0xF3
            } else if self.analog_mode {
                0x73
            } else {
                0x41
            };

            let btn_lo = (self.buttons & 0xFF) as u8;
            let btn_hi = ((self.buttons >> 8) & 0xFF) as u8;

            match val {
                0x42 => {
                    // Main Read Pad & Set Rumble command (valid in both normal and config modes)
                    if self.config_mode || self.analog_mode {
                        self.tx_response.extend_from_slice(&[
                            mode_id,
                            0x5A,
                            btn_lo,
                            btn_hi,
                            self.right_x,
                            self.right_y,
                            self.left_x,
                            self.left_y,
                        ]);
                    } else {
                        // Digital mode: if sticks are left neutral, L3/R3 bits (1 and 2) in digital mode are high (1)
                        let dig_lo = btn_lo | 0x06;
                        self.tx_response.extend_from_slice(&[mode_id, 0x5A, dig_lo, btn_hi]);
                    }
                    return (self.tx_response[0], true);
                }
                0x43 => {
                    // Enter / Exit Config Mode (Escape command)
                    if self.config_mode || self.analog_mode {
                        self.tx_response.extend_from_slice(&[
                            mode_id,
                            0x5A,
                            btn_lo,
                            btn_hi,
                            self.right_x,
                            self.right_y,
                            self.left_x,
                            self.left_y,
                        ]);
                    } else {
                        let dig_lo = btn_lo | 0x06;
                        self.tx_response.extend_from_slice(&[mode_id, 0x5A, dig_lo, btn_hi]);
                    }
                    return (self.tx_response[0], true);
                }
                0x44 if self.config_mode => {
                    // Set LED / Analog Mode state
                    self.tx_response
                        .extend_from_slice(&[0xF3, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
                    return (self.tx_response[0], true);
                }
                0x45 if self.config_mode => {
                    // Query Model & Analog LED status
                    let led_on = u8::from(self.analog_mode);
                    self.tx_response.extend_from_slice(&[
                        0xF3, 0x5A, 0x01, 0x02, led_on, 0x02, 0x01, 0x00,
                    ]);
                    return (self.tx_response[0], true);
                }
                0x46 | 0x4C if self.config_mode => {
                    // Query variable capabilities (filled dynamically at step 3)
                    self.tx_response
                        .extend_from_slice(&[0xF3, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
                    return (self.tx_response[0], true);
                }
                0x47 if self.config_mode => {
                    self.tx_response
                        .extend_from_slice(&[0xF3, 0x5A, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00]);
                    return (self.tx_response[0], true);
                }
                0x4D if self.config_mode => {
                    // Map rumble actuators; returns previous mapping
                    let m = self.motor_map;
                    self.tx_response
                        .extend_from_slice(&[0xF3, 0x5A, m[0], m[1], m[2], m[3], m[4], m[5]]);
                    return (self.tx_response[0], true);
                }
                _ => {
                    return (0xFF, false);
                }
            }
        }

        let idx = (step - 1) as usize;
        if idx >= self.tx_response.len() {
            return (0xFF, false);
        }

        // Handle command-specific incoming parameters on steps >= 3 (idx >= 2)
        match self.current_cmd {
            0x42 => {
                if idx >= 2 && (idx - 2) < 6 {
                    let slot = idx - 2;
                    match self.motor_map[slot] {
                        0x00 => self.motor_small = u8::from(val != 0),
                        0x01 => self.motor_large = val,
                        _ => {
                            // Default standard PS1 mapping if game didn't send 0x4D
                            if slot == 0 && self.motor_map == [0xFF; 6] {
                                self.motor_small = u8::from(val != 0);
                            } else if slot == 1 && self.motor_map == [0xFF; 6] {
                                self.motor_large = val;
                            }
                        }
                    }
                }
            }
            0x43 => {
                if idx == 2 {
                    self.config_mode = (val & 0x01) != 0;
                }
            }
            0x44 if self.config_mode => {
                if idx == 2 {
                    self.analog_mode = (val & 0x01) != 0;
                    if !self.analog_mode {
                        self.right_x = 0x80;
                        self.right_y = 0x80;
                        self.left_x = 0x80;
                        self.left_y = 0x80;
                    }
                } else if idx == 3 {
                    self.mode_locked = (val & 0x03) == 0x03;
                }
            }
            0x46 if self.config_mode => {
                if idx == 2 {
                    self.sub_param = val;
                    if val == 0x00 {
                        self.tx_response[4..8].copy_from_slice(&[0x01, 0x02, 0x00, 0x0A]);
                    } else if val == 0x01 {
                        self.tx_response[4..8].copy_from_slice(&[0x01, 0x01, 0x01, 0x14]);
                    }
                }
            }
            0x4C if self.config_mode => {
                if idx == 2 {
                    self.sub_param = val;
                    if val == 0x00 {
                        self.tx_response[5] = 0x04;
                    } else if val == 0x01 {
                        self.tx_response[5] = 0x07;
                    }
                }
            }
            0x4D if self.config_mode => {
                if idx >= 2 && (idx - 2) < 6 {
                    self.motor_map[idx - 2] = val;
                }
            }
            _ => {}
        }

        let resp_byte = self.tx_response[idx];
        let has_more = (idx + 1) < self.tx_response.len();
        if !has_more {
            self.tx_response.clear();
        }
        (resp_byte, has_more)
    }
}

/// Host Gamepad State (polled from XInput or WinMM Joystick API)
#[derive(Clone, Copy, Debug)]
pub struct HostGamepadInput {
    pub connected: bool,
    pub buttons_low: u8,
    pub buttons_high: u8,
    pub left_x: u8,
    pub left_y: u8,
    pub right_x: u8,
    pub right_y: u8,
    pub analog_deflected: bool,
    pub toggle_analog_pressed: bool,
}

impl Default for HostGamepadInput {
    fn default() -> Self {
        Self {
            connected: false,
            buttons_low: 0xFF,
            buttons_high: 0xFF,
            left_x: 0x80,
            left_y: 0x80,
            right_x: 0x80,
            right_y: 0x80,
            analog_deflected: false,
            toggle_analog_pressed: false,
        }
    }
}

#[cfg(target_os = "windows")]
pub mod host_gamepad {
    use super::HostGamepadInput;
    use std::ffi::c_void;
    use std::mem::{size_of, zeroed};

    const XINPUT_GAMEPAD_DPAD_UP: u16 = 0x0001;
    const XINPUT_GAMEPAD_DPAD_DOWN: u16 = 0x0002;
    const XINPUT_GAMEPAD_DPAD_LEFT: u16 = 0x0004;
    const XINPUT_GAMEPAD_DPAD_RIGHT: u16 = 0x0008;
    const XINPUT_GAMEPAD_START: u16 = 0x0010;
    const XINPUT_GAMEPAD_BACK: u16 = 0x0020;
    const XINPUT_GAMEPAD_LEFT_THUMB: u16 = 0x0040;
    const XINPUT_GAMEPAD_RIGHT_THUMB: u16 = 0x0080;
    const XINPUT_GAMEPAD_LEFT_SHOULDER: u16 = 0x0100;
    const XINPUT_GAMEPAD_RIGHT_SHOULDER: u16 = 0x0200;
    const XINPUT_GAMEPAD_GUIDE: u16 = 0x0400;
    const XINPUT_GAMEPAD_A: u16 = 0x1000;
    const XINPUT_GAMEPAD_B: u16 = 0x2000;
    const XINPUT_GAMEPAD_X: u16 = 0x4000;
    const XINPUT_GAMEPAD_Y: u16 = 0x8000;

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct XInputGamepad {
        w_buttons: u16,
        b_left_trigger: u8,
        b_right_trigger: u8,
        s_thumb_lx: i16,
        s_thumb_ly: i16,
        s_thumb_rx: i16,
        s_thumb_ry: i16,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct XInputState {
        dw_packet_number: u32,
        gamepad: XInputGamepad,
    }

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct XInputVibration {
        w_left_motor_speed: u16,
        w_right_motor_speed: u16,
    }

    type XInputGetStateFn = unsafe extern "system" fn(u32, *mut XInputState) -> u32;
    type XInputSetStateFn = unsafe extern "system" fn(u32, *const XInputVibration) -> u32;

    #[repr(C)]
    struct JoyInfoEx {
        dw_size: u32,
        dw_flags: u32,
        dw_xpos: u32,
        dw_ypos: u32,
        dw_zpos: u32,
        dw_rpos: u32,
        dw_upos: u32,
        dw_vpos: u32,
        dw_buttons: u32,
        dw_button_number: u32,
        dw_pov: u32,
        dw_reserved1: u32,
        dw_reserved2: u32,
    }

    const JOY_RETURNALL: u32 = 0x0000_00FF;

    #[link(name = "kernel32")]
    extern "system" {
        fn LoadLibraryA(lp_lib_file_name: *const u8) -> *mut c_void;
        fn GetProcAddress(h_module: *mut c_void, lp_proc_name: *const u8) -> *mut c_void;
    }

    #[link(name = "winmm")]
    extern "system" {
        fn joyGetPosEx(u_joy_id: u32, pji: *mut JoyInfoEx) -> u32;
    }

    pub struct GamepadPoller {
        get_state: Option<XInputGetStateFn>,
        set_state: Option<XInputSetStateFn>,
        active_port: u32,
        was_connected: bool,
        prev_guide: bool,
        last_rumble: (u8, u8),
        probe_cooldown: u32,
        cached_input: HostGamepadInput,
    }

    impl Default for GamepadPoller {
        fn default() -> Self {
            Self::new()
        }
    }

    impl GamepadPoller {
        pub fn new() -> Self {
            let dll_names: [&[u8]; 3] = [
                b"xinput1_4.dll\0",
                b"xinput1_3.dll\0",
                b"xinput9_1_0.dll\0",
            ];
            let mut get_state = None;
            let mut set_state = None;

            for name in dll_names {
                unsafe {
                    let h = LoadLibraryA(name.as_ptr());
                    if !h.is_null() {
                        // Ordinal 100 in xinput1_4 / xinput1_3 reads the Guide button as well; fallback to XInputGetState
                        let p_get_ex = GetProcAddress(h, 100 as *const u8);
                        let p_get = if !p_get_ex.is_null() {
                            p_get_ex
                        } else {
                            GetProcAddress(h, b"XInputGetState\0".as_ptr())
                        };
                        let p_set = GetProcAddress(h, b"XInputSetState\0".as_ptr());
                        if !p_get.is_null() {
                            get_state = Some(std::mem::transmute::<*mut c_void, XInputGetStateFn>(
                                p_get,
                            ));
                        }
                        if !p_set.is_null() {
                            set_state = Some(std::mem::transmute::<*mut c_void, XInputSetStateFn>(
                                p_set,
                            ));
                        }
                        if get_state.is_some() {
                            break;
                        }
                    }
                }
            }

            Self {
                get_state,
                set_state,
                active_port: 0,
                was_connected: false,
                prev_guide: false,
                last_rumble: (0, 0),
                probe_cooldown: 0,
                cached_input: HostGamepadInput::default(),
            }
        }

        #[inline]
        fn apply_deadzone(val: i16, deadzone: i32) -> (u8, bool) {
            let v = val as i32;
            if v.abs() <= deadzone {
                (0x80, false)
            } else {
                let sign = v.signum();
                let mag = ((v.abs() - deadzone) * 127) / (32767 - deadzone);
                let scaled = (128 + sign * mag.clamp(0, 127)).clamp(0, 255) as u8;
                (scaled, true)
            }
        }

        pub fn poll(&mut self, motor_small: u8, motor_large: u8) -> HostGamepadInput {
            if !self.was_connected && self.probe_cooldown > 0 {
                self.probe_cooldown -= 1;
                return self.cached_input;
            }
            let mut out = HostGamepadInput::default();

            // 1. Try XInput first (check active_port first; only scan other ports when disconnected)
            if let Some(get_state) = self.get_state {
                let ports_to_check = if self.was_connected { 1 } else { 4 };
                for port_offset in 0..ports_to_check {
                    let port = (self.active_port + port_offset) % 4;
                    let mut state: XInputState = unsafe { zeroed() };
                    let res = unsafe { get_state(port, &mut state) };
                    if res == 0 {
                        self.active_port = port;
                        out.connected = true;
                        if !self.was_connected {
                            println!(
                                "[Gamepad] XInput Controller Connected on Port {}.",
                                port + 1
                            );
                            self.was_connected = true;
                        }

                        let gp = state.gamepad;
                        let b = gp.w_buttons;

                        // Byte 1: [Select, L3, R3, Start, Up, Right, Down, Left]
                        if (b & XINPUT_GAMEPAD_BACK) != 0 {
                            out.buttons_low &= !(1 << 0); // Select
                        }
                        if (b & XINPUT_GAMEPAD_LEFT_THUMB) != 0 {
                            out.buttons_low &= !(1 << 1); // L3
                        }
                        if (b & XINPUT_GAMEPAD_RIGHT_THUMB) != 0 {
                            out.buttons_low &= !(1 << 2); // R3
                        }
                        if (b & XINPUT_GAMEPAD_START) != 0 {
                            out.buttons_low &= !(1 << 3); // Start
                        }
                        if (b & XINPUT_GAMEPAD_DPAD_UP) != 0 {
                            out.buttons_low &= !(1 << 4); // Up
                        }
                        if (b & XINPUT_GAMEPAD_DPAD_RIGHT) != 0 {
                            out.buttons_low &= !(1 << 5); // Right
                        }
                        if (b & XINPUT_GAMEPAD_DPAD_DOWN) != 0 {
                            out.buttons_low &= !(1 << 6); // Down
                        }
                        if (b & XINPUT_GAMEPAD_DPAD_LEFT) != 0 {
                            out.buttons_low &= !(1 << 7); // Left
                        }

                        // Byte 2: [L2, R2, L1, R1, Triangle, Circle, Cross, Square]
                        if gp.b_left_trigger > 30 {
                            out.buttons_high &= !(1 << 0); // L2
                        }
                        if gp.b_right_trigger > 30 {
                            out.buttons_high &= !(1 << 1); // R2
                        }
                        if (b & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0 {
                            out.buttons_high &= !(1 << 2); // L1
                        }
                        if (b & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0 {
                            out.buttons_high &= !(1 << 3); // R1
                        }
                        if (b & XINPUT_GAMEPAD_Y) != 0 {
                            out.buttons_high &= !(1 << 4); // Triangle
                        }
                        if (b & XINPUT_GAMEPAD_B) != 0 {
                            out.buttons_high &= !(1 << 5); // Circle
                        }
                        if (b & XINPUT_GAMEPAD_A) != 0 {
                            out.buttons_high &= !(1 << 6); // Cross
                        }
                        if (b & XINPUT_GAMEPAD_X) != 0 {
                            out.buttons_high &= !(1 << 7); // Square
                        }

                        // Thumbsticks (with radial/axial deadzone 5500)
                        // Note: XInput Y is +Up/-Down, while PS1 DualShock Y is 0x00=Up, 0xFF=Down
                        let (lx, dlx) = Self::apply_deadzone(gp.s_thumb_lx, 5500);
                        let (ly, dly) = Self::apply_deadzone(gp.s_thumb_ly.saturating_neg(), 5500);
                        let (rx, drx) = Self::apply_deadzone(gp.s_thumb_rx, 5500);
                        let (ry, dry) = Self::apply_deadzone(gp.s_thumb_ry.saturating_neg(), 5500);

                        out.left_x = lx;
                        out.left_y = ly;
                        out.right_x = rx;
                        out.right_y = ry;
                        out.analog_deflected = dlx || dly || drx || dry;

                        // Also map Left Stick to D-Pad when deflected past 50% so digital-only PS1 games work seamlessly with Left Stick!
                        if gp.s_thumb_ly > 16384 {
                            out.buttons_low &= !(1 << 4); // Up
                        } else if gp.s_thumb_ly < -16384 {
                            out.buttons_low &= !(1 << 6); // Down
                        }
                        if gp.s_thumb_lx > 16384 {
                            out.buttons_low &= !(1 << 5); // Right
                        } else if gp.s_thumb_lx < -16384 {
                            out.buttons_low &= !(1 << 7); // Left
                        }

                        let guide_now = (b & XINPUT_GAMEPAD_GUIDE) != 0;
                        if guide_now && !self.prev_guide {
                            out.toggle_analog_pressed = true;
                        }
                        self.prev_guide = guide_now;

                        // Forward PS1 DualShock Rumble to XInput vibration motors
                        if let Some(set_state) = self.set_state {
                            if self.last_rumble != (motor_small, motor_large) {
                                self.last_rumble = (motor_small, motor_large);
                                let vib = XInputVibration {
                                    w_left_motor_speed: (motor_large as u16) * 257,
                                    w_right_motor_speed: if motor_small != 0 {
                                        0xFFFF
                                    } else {
                                        0
                                    },
                                };
                                unsafe {
                                    let _ = set_state(port, &vib);
                                }
                            }
                        }

                        self.cached_input = out;
                        return out;
                    }
                }
            }

            // 2. Fallback to WinMM joyGetPosEx for generic DirectInput USB gamepads
            let mut ji: JoyInfoEx = unsafe { zeroed() };
            ji.dw_size = size_of::<JoyInfoEx>() as u32;
            ji.dw_flags = JOY_RETURNALL;
            let res = unsafe { joyGetPosEx(0, &mut ji) };
            if res == 0 {
                out.connected = true;
                if !self.was_connected {
                    println!("[Gamepad] WinMM USB Gamepad Connected.");
                    self.was_connected = true;
                }
                let btns = ji.dw_buttons;
                // Standard DirectInput mapping:
                // 0: Square/X, 1: Cross/A, 2: Circle/B, 3: Triangle/Y, 4: L1, 5: R1, 6: L2, 7: R2, 8: Select, 9: Start, 10: L3, 11: R3
                if (btns & (1 << 8)) != 0 {
                    out.buttons_low &= !(1 << 0); // Select
                }
                if (btns & (1 << 10)) != 0 {
                    out.buttons_low &= !(1 << 1); // L3
                }
                if (btns & (1 << 11)) != 0 {
                    out.buttons_low &= !(1 << 2); // R3
                }
                if (btns & (1 << 9)) != 0 {
                    out.buttons_low &= !(1 << 3); // Start
                }
                if (btns & (1 << 6)) != 0 {
                    out.buttons_high &= !(1 << 0); // L2
                }
                if (btns & (1 << 7)) != 0 {
                    out.buttons_high &= !(1 << 1); // R2
                }
                if (btns & (1 << 4)) != 0 {
                    out.buttons_high &= !(1 << 2); // L1
                }
                if (btns & (1 << 5)) != 0 {
                    out.buttons_high &= !(1 << 3); // R1
                }
                if (btns & (1 << 3)) != 0 {
                    out.buttons_high &= !(1 << 4); // Triangle
                }
                if (btns & (1 << 2)) != 0 {
                    out.buttons_high &= !(1 << 5); // Circle
                }
                if (btns & (1 << 1)) != 0 {
                    out.buttons_high &= !(1 << 6); // Cross
                }
                if (btns & (1 << 0)) != 0 {
                    out.buttons_high &= !(1 << 7); // Square
                }

                // POV Hat (in hundredths of degrees, 0xFFFF = centered)
                let pov = ji.dw_pov;
                if pov < 36000 {
                    if !(4500..=13500).contains(&pov) && (pov <= 4500 || pov >= 31500) {
                        out.buttons_low &= !(1 << 4); // Up
                    }
                    if (4500..=13500).contains(&pov) {
                        out.buttons_low &= !(1 << 5); // Right
                    }
                    if (13500..=22500).contains(&pov) {
                        out.buttons_low &= !(1 << 6); // Down
                    }
                    if (22500..=31500).contains(&pov) {
                        out.buttons_low &= !(1 << 7); // Left
                    }
                }

                let lx = (ji.dw_xpos >> 8).clamp(0, 255) as u8;
                let ly = (ji.dw_ypos >> 8).clamp(0, 255) as u8;
                let rx = (ji.dw_zpos >> 8).clamp(0, 255) as u8;
                let ry = (ji.dw_rpos >> 8).clamp(0, 255) as u8;

                out.left_x = if lx.abs_diff(0x80) < 20 { 0x80 } else { lx };
                out.left_y = if ly.abs_diff(0x80) < 20 { 0x80 } else { ly };
                out.right_x = if rx.abs_diff(0x80) < 20 { 0x80 } else { rx };
                out.right_y = if ry.abs_diff(0x80) < 20 { 0x80 } else { ry };

                if out.left_y < 64 {
                    out.buttons_low &= !(1 << 4); // Up
                } else if out.left_y > 192 {
                    out.buttons_low &= !(1 << 6); // Down
                }
                if out.left_x > 192 {
                    out.buttons_low &= !(1 << 5); // Right
                } else if out.left_x < 64 {
                    out.buttons_low &= !(1 << 7); // Left
                }

                self.cached_input = out;
                return out;
            }

            if self.was_connected {
                println!("[Gamepad] Controller Disconnected.");
                self.was_connected = false;
            }
            // Wait ~60 frames (1 second) before re-probing disconnected XInput/WinMM ports
            // to avoid the ~3ms/frame Windows HID device enumeration stall!
            self.probe_cooldown = 60;
            self.cached_input = out;
            out
        }
    }
}

#[cfg(not(target_os = "windows"))]
pub mod host_gamepad {
    use super::HostGamepadInput;

    #[derive(Default)]
    pub struct GamepadPoller;

    impl GamepadPoller {
        pub fn new() -> Self {
            Self
        }
        pub fn poll(&mut self, _motor_small: u8, _motor_large: u8) -> HostGamepadInput {
            HostGamepadInput::default()
        }
    }
}

