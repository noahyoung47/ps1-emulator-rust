//! DuckStation-Inspired Interactive Frontend, Settings Suite, Game Library,
//! Native File Dialogs, and Real-Time Hardware Debugging Tools (VRAM, CPU/GTE, SPU, Memory/TTY).

use crate::bus::{Bus, CYCLES_PER_FRAME, VRAM_HEIGHT, VRAM_WIDTH};
use crate::cpu::Cpu;
use crate::spu::AdsrPhase;
use std::collections::VecDeque;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub const UI_MENU_HEIGHT: usize = 24;
pub const UI_TOOLBAR_HEIGHT: usize = 34;
pub const UI_STATUS_HEIGHT: usize = 22;

// DuckStation Dark Slate & Blue Color Palette (0x00RRGGBB)
pub const COL_BG_DARK: u32 = 0x14171D;
pub const COL_BG_PANEL: u32 = 0x1E222A;
pub const COL_BG_ELEVATED: u32 = 0x262B36;
pub const COL_BG_INPUT: u32 = 0x181B22;
pub const COL_BG_HOVER: u32 = 0x323949;
pub const COL_ACCENT: u32 = 0x3B82F6;
pub const COL_ACCENT_HOVER: u32 = 0x5294FF;
pub const COL_ACCENT_DIM: u32 = 0x1E3A6D;
pub const COL_BORDER: u32 = 0x353C4B;
pub const COL_TEXT: u32 = 0xE5E9F0;
pub const COL_TEXT_DIM: u32 = 0x9CA3AF;
pub const COL_TEXT_ACCENT: u32 = 0x60A5FA;
pub const COL_GREEN: u32 = 0x22C55E;
pub const COL_YELLOW: u32 = 0xEAB308;
pub const COL_RED: u32 = 0xEF4444;
pub const COL_PURPLE: u32 = 0xA855F7;

/// Native Win32 `comdlg32.dll` Open File Dialog (`GetOpenFileNameA`)
#[cfg(target_os = "windows")]
pub fn open_file_dialog(title: &str, filter: &[u8]) -> Option<String> {
    use std::ffi::c_void;
    use std::mem::size_of;
    use std::ptr::{null, null_mut};

    const OFN_PATHMUSTEXIST: u32 = 0x0000_0800;
    const OFN_FILEMUSTEXIST: u32 = 0x0000_1000;
    const OFN_NOCHANGEDIR: u32 = 0x0000_0008;
    const OFN_EXPLORER: u32 = 0x0008_0000;

    #[repr(C)]
    struct OpenFileNameA {
        l_struct_size: u32,
        hwnd_owner: *mut c_void,
        h_instance: *mut c_void,
        lpstr_filter: *const u8,
        lpstr_custom_filter: *mut u8,
        n_max_cust_filter: u32,
        n_filter_index: u32,
        lpstr_file: *mut u8,
        n_max_file: u32,
        lpstr_file_title: *mut u8,
        n_max_file_title: u32,
        lpstr_initial_dir: *const u8,
        lpstr_title: *const u8,
        flags: u32,
        n_file_offset: u16,
        n_file_extension: u16,
        lpstr_def_ext: *const u8,
        l_cust_data: usize,
        lpfn_hook: *mut c_void,
        lp_template_name: *const u8,
        pv_reserved: *mut c_void,
        dw_reserved: u32,
        flags_ex: u32,
    }

    #[link(name = "comdlg32")]
    extern "system" {
        fn GetOpenFileNameA(lpofn: *mut OpenFileNameA) -> i32;
    }

    let mut file_buf = vec![0u8; 1024];
    let mut title_c = title.as_bytes().to_vec();
    title_c.push(0);

    let mut ofn = OpenFileNameA {
        l_struct_size: size_of::<OpenFileNameA>() as u32,
        hwnd_owner: null_mut(),
        h_instance: null_mut(),
        lpstr_filter: filter.as_ptr(),
        lpstr_custom_filter: null_mut(),
        n_max_cust_filter: 0,
        n_filter_index: 1,
        lpstr_file: file_buf.as_mut_ptr(),
        n_max_file: file_buf.len() as u32,
        lpstr_file_title: null_mut(),
        n_max_file_title: 0,
        lpstr_initial_dir: null(),
        lpstr_title: title_c.as_ptr(),
        flags: OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR | OFN_EXPLORER,
        n_file_offset: 0,
        n_file_extension: 0,
        lpstr_def_ext: null(),
        l_cust_data: 0,
        lpfn_hook: null_mut(),
        lp_template_name: null(),
        pv_reserved: null_mut(),
        dw_reserved: 0,
        flags_ex: 0,
    };

    let ok = unsafe { GetOpenFileNameA(&mut ofn) };
    if ok != 0 {
        let len = file_buf.iter().position(|&b| b == 0).unwrap_or(0);
        if len > 0 {
            return String::from_utf8(file_buf[..len].to_vec()).ok();
        }
    }
    None
}

#[cfg(not(target_os = "windows"))]
pub fn open_file_dialog(_title: &str, _filter: &[u8]) -> Option<String> {
    None
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AspectRatioMode {
    Auto43,
    Force169,
    Pixel1To1,
    Stretch,
}

impl AspectRatioMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Auto43 => "4:3 (Standard TV)",
            Self::Force169 => "16:9 (Widescreen)",
            Self::Pixel1To1 => "1:1 (Native Pixel)",
            Self::Stretch => "Stretch to Fill",
        }
    }
    pub fn next(self) -> Self {
        match self {
            Self::Auto43 => Self::Force169,
            Self::Force169 => Self::Pixel1To1,
            Self::Pixel1To1 => Self::Stretch,
            Self::Stretch => Self::Auto43,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TextureFilterMode {
    Nearest,
    BilinearViewport,
    Bilinear3DAndViewport,
    XbrzSmooth,
    Jinc2Sharp,
}

impl TextureFilterMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Nearest => "Nearest-Neighbor (Sharp)",
            Self::BilinearViewport => "Bilinear (Viewport)",
            Self::Bilinear3DAndViewport => "Bilinear (3D + Viewport)",
            Self::XbrzSmooth => "xBRZ / Scale2x Smooth",
            Self::Jinc2Sharp => "JINC2 Sharp Bilinear",
        }
    }
    pub fn next(self) -> Self {
        match self {
            Self::Nearest => Self::BilinearViewport,
            Self::BilinearViewport => Self::Bilinear3DAndViewport,
            Self::Bilinear3DAndViewport => Self::XbrzSmooth,
            Self::XbrzSmooth => Self::Jinc2Sharp,
            Self::Jinc2Sharp => Self::Nearest,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PostShaderPreset {
    None,
    CrtTrinitron,
    CrtLottesArcade,
    LcdGrid,
    VhsComposite,
    CasSharpen,
    KawaseBloom,
    MonochromeNoir,
}

impl PostShaderPreset {
    pub fn label(self) -> &'static str {
        match self {
            Self::None => "None (Raw Output)",
            Self::CrtTrinitron => "CRT Sony Trinitron",
            Self::CrtLottesArcade => "CRT Lottes ShadowMask",
            Self::LcdGrid => "PSone LCD Subpixel Grid",
            Self::VhsComposite => "VHS / NTSC Composite",
            Self::CasSharpen => "AMD FidelityFX CAS",
            Self::KawaseBloom => "Kawase HDR Glow",
            Self::MonochromeNoir => "Sepia / Retro Film",
        }
    }
    pub fn next(self) -> Self {
        match self {
            Self::None => Self::CrtTrinitron,
            Self::CrtTrinitron => Self::CrtLottesArcade,
            Self::CrtLottesArcade => Self::LcdGrid,
            Self::LcdGrid => Self::VhsComposite,
            Self::VhsComposite => Self::CasSharpen,
            Self::CasSharpen => Self::KawaseBloom,
            Self::KawaseBloom => Self::MonochromeNoir,
            Self::MonochromeNoir => Self::None,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ActiveMenu {
    None,
    System,
    Settings,
    View,
    Tools,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SettingsTab {
    Console,
    Bios,
    Display,
    Enhancements,
    Shaders,
    Audio,
    Controllers,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ActiveToolWindow {
    None,
    VramViewer,
    CpuDebugger,
    SpuInspector,
    MemoryTtyViewer,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum MemRegion {
    MainRam,
    Scratchpad,
    BiosRom,
    SpuRam,
}

#[derive(Clone)]
pub struct GameEntry {
    pub title: String,
    pub file_name: String,
    pub path: String,
    pub is_exe: bool,
    pub size_bytes: u64,
}

#[derive(Clone)]
pub struct SaveStateSlot {
    pub cpu: Cpu,
    pub bus: Bus,
    pub label: String,
}

pub struct DuckStationUi {
    // System & Emulation State
    pub running: bool,
    pub paused: bool,
    pub fast_forward: bool,
    pub step_one_frame: bool,
    pub step_one_instruction: bool,
    pub request_exit: bool,

    // Loaded Media Info
    pub bios_path: Option<String>,
    pub current_game_title: String,
    pub current_disc_path: Option<String>,
    pub current_exe_path: Option<String>,

    // UI Chrome & View Toggles
    pub show_toolbar: bool,
    pub show_status_bar: bool,
    pub show_game_list: bool,
    pub show_osd: bool,
    pub show_fps_overlay: bool,
    pub full_vram_display: bool,

    // Active Menus, Modals & Tool Windows
    pub active_menu: ActiveMenu,
    pub settings_open: bool,
    pub settings_tab: SettingsTab,
    pub tool_window: ActiveToolWindow,
    pub mem_region: MemRegion,
    pub mem_view_offset: usize,

    // Console & Enhancement Settings
    pub cpu_overclock_pct: u32, // 50, 100, 150, 200, 300
    pub fast_boot: bool,
    pub aspect_ratio: AspectRatioMode,
    pub texture_filter: TextureFilterMode,
    pub post_shader: PostShaderPreset,
    pub scanline_strength: u8,     // 0, 25, 50, 75
    pub color_vibrance_pct: u32,   // 100, 120, 140
    pub brightness_offset: i32,    // -16, 0, 16, 32
    pub bilinear_filtering: bool,
    pub scanlines: bool,
    pub crt_vignette: bool,
    pub color_dithering: bool,
    pub force_ntsc_60: bool,
    pub crop_overscan: bool,
    pub integer_upscaling: bool,
    pub widescreen_gte_hack: bool,
    pub pgxp_culling_correction: bool,
    pub cd_fast_load: bool,
    pub gpu_hardware_dither: bool,
    pub deinterlace_480i: bool,

    // Audio Settings
    pub audio_enabled: bool,
    pub master_volume: i32,
    pub reverb_enabled: bool,
    pub cd_audio_enabled: bool,
    pub mute_in_fast_forward: bool,
    pub scope_samples: VecDeque<(i16, i16)>,

    // Controller Settings
    pub rumble_enabled: bool,
    pub analog_deadzone_pct: u8,
    pub last_pad_low: u8,
    pub last_pad_high: u8,
    pub last_lx: u8,
    pub last_ly: u8,
    pub last_rx: u8,
    pub last_ry: u8,

    // Save States (5 slots)
    pub save_slots: [Option<SaveStateSlot>; 5],
    pub selected_slot: usize,

    // Game Library
    pub game_entries: Vec<GameEntry>,
    pub selected_game_idx: usize,
    pub game_list_scroll: usize,

    // OSD Toast Notifications
    pub osd_messages: VecDeque<(String, Instant)>,

    // Performance Metrics
    pub fps_counter: u32,
    pub current_fps: f32,
    pub last_fps_instant: Instant,
    pub mouse_prev_down: bool,
    pub mouse_x: i32,
    pub mouse_y: i32,
}

impl DuckStationUi {
    pub fn new(
        bios_path: Option<String>,
        cd_bin_path: Option<String>,
        exe_path: Option<String>,
    ) -> Self {
        let has_initial_target = bios_path.is_some() || cd_bin_path.is_some() || exe_path.is_some();
        let current_game_title = if let Some(ref p) = exe_path {
            Path::new(p)
                .file_name()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_else(|| "PS-X EXE".to_string())
        } else if let Some(ref p) = cd_bin_path {
            Path::new(p)
                .file_name()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_else(|| "CD-ROM Disc".to_string())
        } else if bios_path.is_some() {
            "PlayStation BIOS Shell".to_string()
        } else {
            "No Game Loaded".to_string()
        };

        let mut ui = Self {
            running: has_initial_target,
            paused: false,
            fast_forward: false,
            step_one_frame: false,
            step_one_instruction: false,
            request_exit: false,
            bios_path,
            current_game_title,
            current_disc_path: cd_bin_path,
            current_exe_path: exe_path,
            show_toolbar: true,
            show_status_bar: true,
            show_game_list: !has_initial_target,
            show_osd: true,
            show_fps_overlay: true,
            full_vram_display: false,
            active_menu: ActiveMenu::None,
            settings_open: false,
            settings_tab: SettingsTab::Console,
            tool_window: ActiveToolWindow::None,
            mem_region: MemRegion::MainRam,
            mem_view_offset: 0x0001_0000,
            cpu_overclock_pct: 100,
            fast_boot: false,
            aspect_ratio: AspectRatioMode::Auto43,
            texture_filter: TextureFilterMode::Nearest,
            post_shader: PostShaderPreset::None,
            scanline_strength: 0,
            color_vibrance_pct: 100,
            brightness_offset: 0,
            bilinear_filtering: false,
            scanlines: false,
            crt_vignette: false,
            color_dithering: true,
            force_ntsc_60: true,
            crop_overscan: false,
            integer_upscaling: false,
            widescreen_gte_hack: false,
            pgxp_culling_correction: false,
            cd_fast_load: false,
            gpu_hardware_dither: false,
            deinterlace_480i: false,
            audio_enabled: true,
            master_volume: 100,
            reverb_enabled: true,
            cd_audio_enabled: true,
            mute_in_fast_forward: false,
            scope_samples: VecDeque::with_capacity(512),
            rumble_enabled: true,
            analog_deadzone_pct: 12,
            last_pad_low: 0xFF,
            last_pad_high: 0xFF,
            last_lx: 0x80,
            last_ly: 0x80,
            last_rx: 0x80,
            last_ry: 0x80,
            save_slots: [None, None, None, None, None],
            selected_slot: 0,
            game_entries: Vec::new(),
            selected_game_idx: 0,
            game_list_scroll: 0,
            osd_messages: VecDeque::new(),
            fps_counter: 0,
            current_fps: 60.0,
            last_fps_instant: Instant::now(),
            mouse_prev_down: false,
            mouse_x: 0,
            mouse_y: 0,
        };
        ui.scan_workspace_games();
        ui.push_osd("DuckStation-Style Frontend Ready (Click Menus or Toolbar)");
        ui
    }

    pub fn push_osd(&mut self, msg: impl Into<String>) {
        if self.osd_messages.len() >= 5 {
            self.osd_messages.pop_front();
        }
        self.osd_messages.push_back((msg.into(), Instant::now()));
    }

    pub fn scan_workspace_games(&mut self) {
        self.game_entries.clear();
        let mut search_dirs = Vec::new();
        if let Ok(cwd) = std::env::current_dir() {
            search_dirs.push(cwd.clone());
            if let Some(parent) = cwd.parent() {
                search_dirs.push(parent.to_path_buf());
                search_dirs.push(parent.join("Test Rom"));
            }
            search_dirs.push(cwd.join("Test Rom"));
        }

        let mut seen = std::collections::HashSet::new();
        for dir in search_dirs {
            Self::scan_dir_recursive(&dir, 0, &mut self.game_entries, &mut seen, &mut self.bios_path);
        }
        self.game_entries.sort_by(|a, b| a.title.cmp(&b.title));
    }

    fn scan_dir_recursive(
        dir: &Path,
        depth: usize,
        out: &mut Vec<GameEntry>,
        seen: &mut std::collections::HashSet<PathBuf>,
        bios_out: &mut Option<String>,
    ) {
        if depth > 2 {
            return;
        }
        let entries = match fs::read_dir(dir) {
            Ok(e) => e,
            Err(_) => return,
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                let name = path
                    .file_name()
                    .and_then(|s| s.to_str())
                    .unwrap_or("")
                    .to_ascii_lowercase();
                if name != "target" && name != ".git" && name != "src" {
                    Self::scan_dir_recursive(&path, depth + 1, out, seen, bios_out);
                }
                continue;
            }
            let canonical = fs::canonicalize(&path).unwrap_or_else(|_| path.clone());
            if seen.contains(&canonical) {
                continue;
            }
            let ext = path
                .extension()
                .and_then(|s| s.to_str())
                .unwrap_or("")
                .to_ascii_lowercase();
            let file_name = path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("")
                .to_string();
            let meta_len = fs::metadata(&path).map(|m| m.len()).unwrap_or(0);

            // Auto-detect BIOS ROM if not yet configured (512KB SCPH*.BIN)
            if bios_out.is_none()
                && meta_len == 512 * 1024
                && (file_name.to_ascii_uppercase().starts_with("SCPH")
                    || file_name.to_ascii_lowercase().contains("bios"))
            {
                *bios_out = Some(path.to_string_lossy().to_string());
            }

            if ext == "exe" || ext == "psx" {
                if meta_len >= 0x800 {
                    if let Ok(bytes) = fs::read(&path) {
                        if bytes.len() >= 8 && &bytes[0..8] == b"PS-X EXE" {
                            seen.insert(canonical);
                            out.push(GameEntry {
                                title: file_name.clone(),
                                file_name,
                                path: path.to_string_lossy().to_string(),
                                is_exe: true,
                                size_bytes: meta_len,
                            });
                        }
                    }
                }
            } else if (ext == "bin" || ext == "iso" || ext == "img") && meta_len > 600 * 1024 {
                seen.insert(canonical);
                let title = path
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or(&file_name)
                    .to_string();
                out.push(GameEntry {
                    title,
                    file_name,
                    path: path.to_string_lossy().to_string(),
                    is_exe: false,
                    size_bytes: meta_len,
                });
            }
        }
    }

    pub fn boot_file_path(&mut self, path: &str, cpu: &mut Cpu, bus: &mut Bus) {
        let p = Path::new(path);
        let ext = p
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        let file_name = p
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or(path)
            .to_string();

        // Check if file is a 512KB BIOS ROM
        if let Ok(meta) = fs::metadata(p) {
            if meta.len() == 512 * 1024
                && (file_name.to_ascii_uppercase().starts_with("SCPH")
                    || file_name.to_ascii_lowercase().contains("bios"))
            {
                self.bios_path = Some(path.to_string());
                self.push_osd(format!("Configured BIOS: {}", file_name));
                return;
            }
        }

        let mut new_bus = Bus::new();
        new_bus.spu.reverb_enabled = self.reverb_enabled;
        new_bus.spu.cd_audio_enabled = self.cd_audio_enabled;
        new_bus.spu.master_volume = if self.audio_enabled { self.master_volume } else { 0 };
        let mut new_cpu = Cpu::new();

        if let Some(ref bpath) = self.bios_path {
            let _ = new_bus.load_bios(bpath);
        }

        // Check if PS-X EXE
        let is_ps_exe = if ext == "exe" || ext == "psx" {
            true
        } else if let Ok(data) = fs::read(p) {
            data.len() >= 8 && &data[0..8] == b"PS-X EXE"
        } else {
            false
        };

        if is_ps_exe {
            match fs::read(p) {
                Ok(bytes) => {
                    if self.bios_path.is_some() && !self.fast_boot {
                        if let Err(e) = new_bus.queue_exe_bytes(&bytes) {
                            self.push_osd(format!("EXE Error: {}", e));
                            return;
                        }
                    } else if let Err(e) = new_cpu.sideload_exe(&mut new_bus, &bytes) {
                        self.push_osd(format!("EXE Error: {}", e));
                        return;
                    }
                    self.current_exe_path = Some(path.to_string());
                    self.current_disc_path = None;
                    self.current_game_title = file_name.clone();
                    self.running = true;
                    self.paused = false;
                    self.show_game_list = false;
                    *bus = new_bus;
                    *cpu = new_cpu;
                    self.push_osd(format!("Booted PS-EXE: {}", file_name));
                }
                Err(e) => {
                    self.push_osd(format!("Read failed: {}", e));
                }
            }
        } else {
            if self.bios_path.is_none() {
                self.push_osd("Note: Select a PS1 BIOS in Settings -> BIOS to boot CD-ROM discs!");
            }
            match new_bus.load_cd_bin_file(path) {
                Ok(_) => {
                    self.current_disc_path = Some(path.to_string());
                    self.current_exe_path = None;
                    self.current_game_title = file_name.clone();
                    self.running = true;
                    self.paused = false;
                    self.show_game_list = false;
                    *bus = new_bus;
                    *cpu = new_cpu;
                    self.push_osd(format!("Booted Disc: {}", file_name));
                }
                Err(e) => {
                    self.push_osd(format!("Disc Load Error: {}", e));
                }
            }
        }
    }

    pub fn boot_bios_only(&mut self, cpu: &mut Cpu, bus: &mut Bus) {
        let Some(ref bpath) = self.bios_path.clone() else {
            self.push_osd("No BIOS configured! Click Settings -> BIOS to choose SCPH1001.BIN.");
            self.settings_open = true;
            self.settings_tab = SettingsTab::Bios;
            return;
        };
        let mut new_bus = Bus::new();
        if let Err(e) = new_bus.load_bios(bpath) {
            self.push_osd(format!("BIOS Error: {}", e));
            return;
        }
        new_bus.spu.reverb_enabled = self.reverb_enabled;
        new_bus.spu.cd_audio_enabled = self.cd_audio_enabled;
        new_bus.spu.master_volume = if self.audio_enabled { self.master_volume } else { 0 };
        *bus = new_bus;
        *cpu = Cpu::new();
        self.current_disc_path = None;
        self.current_exe_path = None;
        self.current_game_title = "PlayStation BIOS Shell".to_string();
        self.running = true;
        self.paused = false;
        self.show_game_list = false;
        self.push_osd("Booted PlayStation BIOS Shell");
    }

    pub fn reset_system(&mut self, cpu: &mut Cpu, bus: &mut Bus) {
        if let Some(path) = self.current_exe_path.clone() {
            self.boot_file_path(&path, cpu, bus);
            self.push_osd("System Reset (PS-EXE)");
        } else if let Some(path) = self.current_disc_path.clone() {
            self.boot_file_path(&path, cpu, bus);
            self.push_osd("System Reset (CD-ROM)");
        } else if self.bios_path.is_some() {
            self.boot_bios_only(cpu, bus);
            self.push_osd("System Reset (BIOS)");
        } else {
            self.push_osd("Nothing loaded to reset");
        }
    }

    pub fn save_state_slot(&mut self, slot: usize, cpu: &Cpu, bus: &Bus) {
        if slot >= 5 {
            return;
        }
        self.save_slots[slot] = Some(SaveStateSlot {
            cpu: cpu.clone(),
            bus: bus.clone(),
            label: format!("PC: {:08X} | {}", cpu.pc, self.current_game_title),
        });
        self.selected_slot = slot;
        self.push_osd(format!("Saved State to Slot {} (PC=0x{:08X})", slot + 1, cpu.pc));
    }

    pub fn load_state_slot(&mut self, slot: usize, cpu: &mut Cpu, bus: &mut Bus) {
        if slot >= 5 {
            return;
        }
        if let Some(ref saved) = self.save_slots[slot] {
            *cpu = saved.cpu.clone();
            *bus = saved.bus.clone();
            self.running = true;
            self.show_game_list = false;
            self.selected_slot = slot;
            self.push_osd(format!("Loaded State from Slot {} (PC=0x{:08X})", slot + 1, cpu.pc));
        } else {
            self.push_osd(format!("Save State Slot {} is empty!", slot + 1));
        }
    }

    pub fn update_fps(&mut self) {
        self.fps_counter += 1;
        let elapsed = self.last_fps_instant.elapsed().as_secs_f32();
        if elapsed >= 0.5 {
            self.current_fps = (self.fps_counter as f32) / elapsed;
            self.fps_counter = 0;
            self.last_fps_instant = Instant::now();
        }
    }

    pub fn push_scope_samples(&mut self, samples: &[(i16, i16)]) {
        for &(l, r) in samples.iter().step_by(4) {
            if self.scope_samples.len() >= 256 {
                self.scope_samples.pop_front();
            }
            self.scope_samples.push_back((l, r));
        }
    }

    pub fn cycles_per_frame_scaled(&self) -> u64 {
        ((CYCLES_PER_FRAME as u64) * (self.cpu_overclock_pct as u64)) / 100
    }
}

/// Complete 8x8 ASCII Bitmap Font (ASCII 0x20..=0x7E)
pub const FONT_8X8: [[u8; 8]; 95] = [
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], // ' '
    [0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00], // '!'
    [0x66, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00], // '"'
    [0x6C, 0x6C, 0xFE, 0x6C, 0xFE, 0x6C, 0x6C, 0x00], // '#'
    [0x18, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x18, 0x00], // '$'
    [0x00, 0xC6, 0xCC, 0x18, 0x30, 0x66, 0xC6, 0x00], // '%'
    [0x38, 0x6C, 0x38, 0x76, 0xDC, 0xCC, 0x76, 0x00], // '&'
    [0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00], // '\''
    [0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00], // '('
    [0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00], // ')'
    [0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00], // '*'
    [0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00], // '+'
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30], // ','
    [0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00], // '-'
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00], // '.'
    [0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x80, 0x00], // '/'
    [0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00], // '0'
    [0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00], // '1'
    [0x3C, 0x66, 0x06, 0x1C, 0x30, 0x66, 0x7E, 0x00], // '2'
    [0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00], // '3'
    [0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x1E, 0x00], // '4'
    [0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00], // '5'
    [0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00], // '6'
    [0x7E, 0x66, 0x06, 0x0C, 0x18, 0x18, 0x18, 0x00], // '7'
    [0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00], // '8'
    [0x3C, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00], // '9'
    [0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00], // ':'
    [0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x30], // ';'
    [0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x00], // '<'
    [0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00], // '='
    [0x30, 0x18, 0x0C, 0x06, 0x0C, 0x18, 0x30, 0x00], // '>'
    [0x3C, 0x66, 0x06, 0x0C, 0x18, 0x00, 0x18, 0x00], // '?'
    [0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x60, 0x3E, 0x00], // '@'
    [0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x00], // 'A'
    [0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00], // 'B'
    [0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00], // 'C'
    [0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00], // 'D'
    [0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x7E, 0x00], // 'E'
    [0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x60, 0x00], // 'F'
    [0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3E, 0x00], // 'G'
    [0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00], // 'H'
    [0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00], // 'I'
    [0x1E, 0x0C, 0x0C, 0x0C, 0xCC, 0xCC, 0x78, 0x00], // 'J'
    [0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00], // 'K'
    [0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00], // 'L'
    [0xC6, 0xEE, 0xFE, 0xD6, 0xC6, 0xC6, 0xC6, 0x00], // 'M'
    [0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00], // 'N'
    [0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00], // 'O'
    [0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00], // 'P'
    [0x3C, 0x66, 0x66, 0x66, 0x6A, 0x6C, 0x36, 0x00], // 'Q'
    [0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x00], // 'R'
    [0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00], // 'S'
    [0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00], // 'T'
    [0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00], // 'U'
    [0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00], // 'V'
    [0xC6, 0xC6, 0xC6, 0xD6, 0xFE, 0xEE, 0xC6, 0x00], // 'W'
    [0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00], // 'X'
    [0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00], // 'Y'
    [0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00], // 'Z'
    [0x3C, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3C, 0x00], // '['
    [0xC0, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x02, 0x00], // '\\'
    [0x3C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3C, 0x00], // ']'
    [0x10, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00, 0x00], // '^'
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF], // '_'
    [0x30, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00], // '`'
    [0x00, 0x00, 0x3C, 0x06, 0x3E, 0x66, 0x3E, 0x00], // 'a'
    [0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x7C, 0x00], // 'b'
    [0x00, 0x00, 0x3C, 0x66, 0x60, 0x66, 0x3C, 0x00], // 'c'
    [0x06, 0x06, 0x3E, 0x66, 0x66, 0x66, 0x3E, 0x00], // 'd'
    [0x00, 0x00, 0x3C, 0x66, 0x7E, 0x60, 0x3C, 0x00], // 'e'
    [0x1C, 0x36, 0x30, 0x78, 0x30, 0x30, 0x30, 0x00], // 'f'
    [0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x3C], // 'g'
    [0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00], // 'h'
    [0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x3C, 0x00], // 'i'
    [0x06, 0x00, 0x06, 0x06, 0x06, 0x66, 0x66, 0x3C], // 'j'
    [0x60, 0x60, 0x66, 0x6C, 0x78, 0x6C, 0x66, 0x00], // 'k'
    [0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00], // 'l'
    [0x00, 0x00, 0xEC, 0xFE, 0xD6, 0xC6, 0xC6, 0x00], // 'm'
    [0x00, 0x00, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00], // 'n'
    [0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00], // 'o'
    [0x00, 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60], // 'p'
    [0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x06], // 'q'
    [0x00, 0x00, 0x7C, 0x66, 0x60, 0x60, 0x60, 0x00], // 'r'
    [0x00, 0x00, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x00], // 's'
    [0x30, 0x30, 0x7C, 0x30, 0x30, 0x36, 0x1C, 0x00], // 't'
    [0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3E, 0x00], // 'u'
    [0x00, 0x00, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00], // 'v'
    [0x00, 0x00, 0xC6, 0xC6, 0xD6, 0xFE, 0x6C, 0x00], // 'w'
    [0x00, 0x00, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x00], // 'x'
    [0x00, 0x00, 0x66, 0x66, 0x66, 0x3E, 0x06, 0x3C], // 'y'
    [0x00, 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x7E, 0x00], // 'z'
    [0x0E, 0x18, 0x18, 0x70, 0x18, 0x18, 0x0E, 0x00], // '{'
    [0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00], // '|'
    [0x70, 0x18, 0x18, 0x0E, 0x18, 0x18, 0x70, 0x00], // '}'
    [0x76, 0xDC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], // '~'
];

pub struct Canvas<'a> {
    pub fb: &'a mut [u32],
    pub width: usize,
    pub height: usize,
}

impl<'a> Canvas<'a> {
    pub fn new(fb: &'a mut [u32], width: usize, height: usize) -> Self {
        Self { fb, width, height }
    }

    #[inline]
    pub fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, color: u32) {
        if w <= 0 || h <= 0 {
            return;
        }
        let x0 = x.max(0) as usize;
        let y0 = y.max(0) as usize;
        let x1 = (x + w).min(self.width as i32).max(0) as usize;
        let y1 = (y + h).min(self.height as i32).max(0) as usize;
        for py in y0..y1 {
            let row = py * self.width;
            self.fb[row + x0..row + x1].fill(color);
        }
    }

    #[inline]
    pub fn fill_rect_alpha(&mut self, x: i32, y: i32, w: i32, h: i32, color: u32, alpha: u8) {
        if w <= 0 || h <= 0 {
            return;
        }
        let a = alpha as u32;
        let inv = 255 - a;
        let sr = (color >> 16) & 0xFF;
        let sg = (color >> 8) & 0xFF;
        let sb = color & 0xFF;

        let x0 = x.max(0) as usize;
        let y0 = y.max(0) as usize;
        let x1 = (x + w).min(self.width as i32).max(0) as usize;
        let y1 = (y + h).min(self.height as i32).max(0) as usize;
        for py in y0..y1 {
            let row = py * self.width;
            for px in x0..x1 {
                let dst = self.fb[row + px];
                let dr = (dst >> 16) & 0xFF;
                let dg = (dst >> 8) & 0xFF;
                let db = dst & 0xFF;
                let r = (sr * a + dr * inv) / 255;
                let g = (sg * a + dg * inv) / 255;
                let b = (sb * a + db * inv) / 255;
                self.fb[row + px] = (r << 16) | (g << 8) | b;
            }
        }
    }

    pub fn draw_border(&mut self, x: i32, y: i32, w: i32, h: i32, color: u32) {
        self.fill_rect(x, y, w, 1, color);
        self.fill_rect(x, y + h - 1, w, 1, color);
        self.fill_rect(x, y, 1, h, color);
        self.fill_rect(x + w - 1, y, 1, h, color);
    }

    pub fn draw_panel(&mut self, x: i32, y: i32, w: i32, h: i32, bg: u32, border: u32) {
        self.fill_rect(x, y, w, h, bg);
        self.draw_border(x, y, w, h, border);
    }

    pub fn draw_char(&mut self, x: i32, y: i32, ch: char, color: u32) {
        let code = ch as usize;
        if !(0x20..=0x7E).contains(&code) {
            return;
        }
        let glyph = &FONT_8X8[code - 0x20];
        for (row_idx, &bits) in glyph.iter().enumerate() {
            let py = y + row_idx as i32;
            if py < 0 || py >= self.height as i32 {
                continue;
            }
            let row_off = (py as usize) * self.width;
            for col_idx in 0..8 {
                if (bits & (0x80 >> col_idx)) != 0 {
                    let px = x + col_idx;
                    if px >= 0 && px < self.width as i32 {
                        self.fb[row_off + px as usize] = color;
                    }
                }
            }
        }
    }

    pub fn draw_text(&mut self, x: i32, y: i32, text: &str, color: u32) {
        let mut cx = x;
        for ch in text.chars() {
            self.draw_char(cx, y, ch, color);
            cx += 8;
        }
    }

    pub fn draw_text_clipped(&mut self, x: i32, y: i32, max_chars: usize, text: &str, color: u32) {
        let mut cx = x;
        for (i, ch) in text.chars().enumerate() {
            if i >= max_chars {
                break;
            }
            self.draw_char(cx, y, ch, color);
            cx += 8;
        }
    }

    pub fn draw_line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, color: u32) {
        let mut x = x0;
        let mut y = y0;
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            if x >= 0 && x < self.width as i32 && y >= 0 && y < self.height as i32 {
                self.fb[(y as usize) * self.width + (x as usize)] = color;
            }
            if x == x1 && y == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }
}

pub const GPR_NAMES: [&str; 32] = [
    "r0/zero", "r1/at", "r2/v0", "r3/v1", "r4/a0", "r5/a1", "r6/a2", "r7/a3", "r8/t0", "r9/t1",
    "r10/t2", "r11/t3", "r12/t4", "r13/t5", "r14/t6", "r15/t7", "r16/s0", "r17/s1", "r18/s2",
    "r19/s3", "r20/s4", "r21/s5", "r22/s6", "r23/s7", "r24/t8", "r25/t9", "r26/k0", "r27/k1",
    "r28/gp", "r29/sp", "r30/fp", "r31/ra",
];

/// Disassemble a single 32-bit MIPS R3000A instruction for the DuckStation CPU Debugger
pub fn disassemble_mips(instr: u32, pc: u32) -> String {
    if instr == 0 {
        return "nop".to_string();
    }
    let op = instr >> 26;
    let rs = ((instr >> 21) & 0x1F) as usize;
    let rt = ((instr >> 16) & 0x1F) as usize;
    let rd = ((instr >> 11) & 0x1F) as usize;
    let shamt = (instr >> 6) & 0x1F;
    let funct = instr & 0x3F;
    let imm = (instr & 0xFFFF) as u16;
    let simm = imm as i16 as i32;
    let target = (pc.wrapping_add(4) & 0xF000_0000) | ((instr & 0x03FF_FFFF) << 2);
    let branch_dst = pc.wrapping_add(4).wrapping_add((simm as u32) << 2);

    match op {
        0x00 => match funct {
            0x00 => format!("sll     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rt], shamt),
            0x02 => format!("srl     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rt], shamt),
            0x03 => format!("sra     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rt], shamt),
            0x08 => format!("jr      {}", GPR_NAMES[rs]),
            0x09 => format!("jalr    {}, {}", GPR_NAMES[rd], GPR_NAMES[rs]),
            0x0C => "syscall".to_string(),
            0x0D => "break".to_string(),
            0x10 => format!("mfhi    {}", GPR_NAMES[rd]),
            0x12 => format!("mflo    {}", GPR_NAMES[rd]),
            0x18 => format!("mult    {}, {}", GPR_NAMES[rs], GPR_NAMES[rt]),
            0x19 => format!("multu   {}, {}", GPR_NAMES[rs], GPR_NAMES[rt]),
            0x1A => format!("div     {}, {}", GPR_NAMES[rs], GPR_NAMES[rt]),
            0x1B => format!("divu    {}, {}", GPR_NAMES[rs], GPR_NAMES[rt]),
            0x20 => format!("add     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x21 => format!("addu    {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x22 => format!("sub     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x23 => format!("subu    {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x24 => format!("and     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x25 => format!("or      {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x26 => format!("xor     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x27 => format!("nor     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x2A => format!("slt     {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            0x2B => format!("sltu    {}, {}, {}", GPR_NAMES[rd], GPR_NAMES[rs], GPR_NAMES[rt]),
            _ => format!("special 0x{:02X}", funct),
        },
        0x01 => match rt {
            0x00 => format!("bltz    {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
            0x01 => format!("bgez    {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
            0x10 => format!("bltzal  {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
            0x11 => format!("bgezal  {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
            _ => format!("bcond   {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
        },
        0x02 => format!("j       0x{:08X}", target),
        0x03 => format!("jal     0x{:08X}", target),
        0x04 => format!("beq     {}, {}, 0x{:08X}", GPR_NAMES[rs], GPR_NAMES[rt], branch_dst),
        0x05 => format!("bne     {}, {}, 0x{:08X}", GPR_NAMES[rs], GPR_NAMES[rt], branch_dst),
        0x06 => format!("blez    {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
        0x07 => format!("bgtz    {}, 0x{:08X}", GPR_NAMES[rs], branch_dst),
        0x08 => format!("addi    {}, {}, {}", GPR_NAMES[rt], GPR_NAMES[rs], simm),
        0x09 => format!("addiu   {}, {}, {}", GPR_NAMES[rt], GPR_NAMES[rs], simm),
        0x0A => format!("slti    {}, {}, {}", GPR_NAMES[rt], GPR_NAMES[rs], simm),
        0x0B => format!("sltiu   {}, {}, {}", GPR_NAMES[rt], GPR_NAMES[rs], simm),
        0x0C => format!("andi    {}, {}, 0x{:04X}", GPR_NAMES[rt], GPR_NAMES[rs], imm),
        0x0D => format!("ori     {}, {}, 0x{:04X}", GPR_NAMES[rt], GPR_NAMES[rs], imm),
        0x0E => format!("xori    {}, {}, 0x{:04X}", GPR_NAMES[rt], GPR_NAMES[rs], imm),
        0x0F => format!("lui     {}, 0x{:04X}", GPR_NAMES[rt], imm),
        0x10 => match rs {
            0x00 => format!("mfc0    {}, cop0r{}", GPR_NAMES[rt], rd),
            0x04 => format!("mtc0    {}, cop0r{}", GPR_NAMES[rt], rd),
            0x10 => "rfe".to_string(),
            _ => format!("cop0    0x{:08X}", instr),
        },
        0x12 => {
            if (instr & (1 << 25)) != 0 {
                format!("gte     cmd 0x{:02X}", instr & 0x3F)
            } else {
                match rs {
                    0x00 => format!("mfc2    {}, gte_d{}", GPR_NAMES[rt], rd),
                    0x02 => format!("cfc2    {}, gte_c{}", GPR_NAMES[rt], rd),
                    0x04 => format!("mtc2    {}, gte_d{}", GPR_NAMES[rt], rd),
                    0x06 => format!("ctc2    {}, gte_c{}", GPR_NAMES[rt], rd),
                    _ => format!("cop2    0x{:08X}", instr),
                }
            }
        }
        0x20 => format!("lb      {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x21 => format!("lh      {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x23 => format!("lw      {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x24 => format!("lbu     {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x25 => format!("lhu     {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x28 => format!("sb      {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x29 => format!("sh      {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x2B => format!("sw      {}, {}({})", GPR_NAMES[rt], simm, GPR_NAMES[rs]),
        0x32 => format!("lwc2    gte_d{}, {}({})", rt, simm, GPR_NAMES[rs]),
        0x3A => format!("swc2    gte_d{}, {}({})", rt, simm, GPR_NAMES[rs]),
        _ => format!("op 0x{:02X} (0x{:08X})", op, instr),
    }
}

#[inline]
fn rect_contains(mx: i32, my: i32, x: i32, y: i32, w: i32, h: i32) -> bool {
    mx >= x && mx < x + w && my >= y && my < y + h
}

impl DuckStationUi {
    /// Render the complete DuckStation Frontend (Game Viewport or Game Library, Top Menu Bar,
    /// Toolbar, Status Bar, OSD Overlays, Settings Modal, and Hardware Inspector Tools)
    /// and handle mouse clicks.
    #[allow(clippy::too_many_arguments)]
    pub fn render_and_interact(
        &mut self,
        fb: &mut [u32],
        win_w: usize,
        win_h: usize,
        mouse_x: i32,
        mouse_y: i32,
        mouse_down: bool,
        scroll_y: f32,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        let clicked = mouse_down && !self.mouse_prev_down;
        self.mouse_prev_down = mouse_down;
        self.mouse_x = mouse_x;
        self.mouse_y = mouse_y;

        // Sync audio & enhancement settings to SPU, Bus, and CPU/GTE
        bus.spu.reverb_enabled = self.reverb_enabled;
        bus.spu.cd_audio_enabled = self.cd_audio_enabled;
        bus.spu.master_volume = if self.audio_enabled { self.master_volume } else { 0 };
        bus.gpu_dither_enabled = self.gpu_hardware_dither;
        bus.gpu_tex_bilinear = self.texture_filter == TextureFilterMode::Bilinear3DAndViewport;
        bus.gpu_force_480i_deinterlace = self.deinterlace_480i;
        bus.cd_fast_load = self.cd_fast_load;
        cpu.widescreen_hack = self.widescreen_gte_hack;
        cpu.pgxp_culling_fix = self.pgxp_culling_correction;

        let top_bar_h = UI_MENU_HEIGHT + if self.show_toolbar { UI_TOOLBAR_HEIGHT } else { 0 };
        let bottom_bar_h = if self.show_status_bar { UI_STATUS_HEIGHT } else { 0 };
        let content_y = top_bar_h as i32;
        let content_h = (win_h as i32 - top_bar_h as i32 - bottom_bar_h as i32).max(100);

        let mut canvas = Canvas::new(fb, win_w, win_h);

        let modal_blocking = self.active_menu != ActiveMenu::None
            || self.settings_open
            || self.tool_window != ActiveToolWindow::None;

        // 1. Main Content Area: Either Game Library Dashboard or PS1 Display Viewport
        if self.show_game_list || !self.running {
            canvas.fill_rect(0, 0, win_w as i32, win_h as i32, COL_BG_DARK);
            self.draw_game_library(
                &mut canvas,
                0,
                content_y,
                win_w as i32,
                content_h,
                if modal_blocking { false } else { clicked },
                scroll_y,
                cpu,
                bus,
            );
        } else {
            self.draw_ps1_viewport(&mut canvas, 0, content_y, win_w as i32, content_h, bus);
        }

        // 2. OSD Toast Messages & FPS / Speed Overlay
        if self.show_osd {
            self.draw_osd_messages(&mut canvas, 12, content_y + 10);
        }
        if self.show_fps_overlay && self.running && !self.show_game_list {
            self.draw_fps_overlay(&mut canvas, win_w as i32 - 230, content_y + 10, cpu, bus);
        }

        // 3. Top Toolbar (if enabled)
        if self.show_toolbar {
            self.draw_toolbar(
                &mut canvas,
                0,
                UI_MENU_HEIGHT as i32,
                win_w as i32,
                UI_TOOLBAR_HEIGHT as i32,
                if self.active_menu == ActiveMenu::None { clicked } else { false },
                cpu,
                bus,
            );
        }

        // 4. Bottom Status Bar (if enabled)
        if self.show_status_bar {
            self.draw_status_bar(
                &mut canvas,
                0,
                win_h as i32 - UI_STATUS_HEIGHT as i32,
                win_w as i32,
                UI_STATUS_HEIGHT as i32,
                cpu,
                bus,
            );
        }

        // 5. Active Tool Window (VRAM Viewer, CPU/GTE Debugger, SPU Voice Scope, Memory/TTY)
        if self.tool_window != ActiveToolWindow::None {
            self.draw_tool_window(
                &mut canvas,
                win_w as i32,
                win_h as i32,
                if self.active_menu == ActiveMenu::None { clicked } else { false },
                scroll_y,
                cpu,
                bus,
            );
        }

        // 6. Tabbed Settings Modal Window
        if self.settings_open {
            self.draw_settings_modal(
                &mut canvas,
                win_w as i32,
                win_h as i32,
                if self.active_menu == ActiveMenu::None { clicked } else { false },
                cpu,
                bus,
            );
        }

        // 7. Top Menu Bar & Active Dropdown Menu (drawn last so dropdowns sit on top)
        self.draw_menu_bar(&mut canvas, win_w as i32, clicked, cpu, bus);
    }

    fn draw_ps1_viewport(
        &self,
        canvas: &mut Canvas,
        vx: i32,
        vy: i32,
        vw: i32,
        vh: i32,
        bus: &Bus,
    ) {
        if vw <= 0 || vh <= 0 {
            return;
        }
        let is_24bit = (bus.gpustat & (1 << 21)) != 0;
        let mut src_w = if self.full_vram_display {
            1024usize
        } else if bus.display_width > 0 {
            bus.display_width as usize
        } else {
            320usize
        };
        let mut src_h = if self.full_vram_display {
            512usize
        } else if bus.display_height > 0 {
            bus.display_height as usize
        } else {
            240usize
        };
        let mut start_x = if self.full_vram_display {
            0
        } else {
            (bus.display_vram_x as usize) % VRAM_WIDTH
        };
        let mut start_y = if self.full_vram_display {
            0
        } else {
            (bus.display_vram_y as usize) % VRAM_HEIGHT
        };

        // Optional Overscan Cropping (crops 8px top/bottom & 8px left/right border garbage)
        if self.crop_overscan && !self.full_vram_display && src_w > 32 && src_h > 32 {
            start_x = (start_x + 8) % VRAM_WIDTH;
            start_y = (start_y + 8) % VRAM_HEIGHT;
            src_w -= 16;
            src_h -= 16;
        }

        // Compute destination rectangle inside (vx, vy, vw, vh) based on AspectRatioMode
        let (mut dst_w, mut dst_h) = if self.full_vram_display {
            let scale_w = vw;
            let scale_h = (vw / 2).min(vh);
            (scale_w.min(scale_h * 2), scale_h)
        } else {
            match self.aspect_ratio {
                AspectRatioMode::Auto43 => {
                    if vw * 3 > vh * 4 {
                        ((vh * 4) / 3, vh)
                    } else {
                        (vw, (vw * 3) / 4)
                    }
                }
                AspectRatioMode::Force169 => {
                    if vw * 9 > vh * 16 {
                        ((vh * 16) / 9, vh)
                    } else {
                        (vw, (vw * 9) / 16)
                    }
                }
                AspectRatioMode::Pixel1To1 => {
                    let mult_x = (vw / (src_w as i32)).max(1);
                    let mult_y = (vh / (src_h as i32)).max(1);
                    let mult = mult_x.min(mult_y);
                    ((src_w as i32 * mult).min(vw), (src_h as i32 * mult).min(vh))
                }
                AspectRatioMode::Stretch => (vw, vh),
            }
        };

        // Optional Integer Upscaling (snap height to exact integer multiple of source resolution)
        if self.integer_upscaling && !self.full_vram_display && dst_h >= src_h as i32 {
            let int_mult = (dst_h / (src_h as i32)).max(1);
            let snapped_h = (src_h as i32) * int_mult;
            dst_w = (dst_w * snapped_h) / dst_h.max(1);
            dst_h = snapped_h;
        }

        let dst_x0 = vx + (vw - dst_w) / 2;
        let dst_y0 = vy + (vh - dst_h) / 2;
        let uw = dst_w.max(1) as usize;
        let uh = dst_h.max(1) as usize;

        // Clear only the letterbox / pillarbox bars around the active viewport
        if dst_y0 > vy {
            canvas.fill_rect(vx, vy, vw, dst_y0 - vy, COL_BG_DARK);
        }
        let dst_y1 = dst_y0 + dst_h;
        if dst_y1 < vy + vh {
            canvas.fill_rect(vx, dst_y1, vw, (vy + vh) - dst_y1, COL_BG_DARK);
        }
        if dst_x0 > vx {
            canvas.fill_rect(vx, dst_y0, dst_x0 - vx, dst_h, COL_BG_DARK);
        }
        let dst_x1 = dst_x0 + dst_w;
        if dst_x1 < vx + vw {
            canvas.fill_rect(dst_x1, dst_y0, (vx + vw) - dst_x1, dst_h, COL_BG_DARK);
        }

        let use_bilerp = self.bilinear_filtering
            || matches!(
                self.texture_filter,
                TextureFilterMode::BilinearViewport
                    | TextureFilterMode::Bilinear3DAndViewport
                    | TextureFilterMode::Jinc2Sharp
            );
        let use_xbrz = self.texture_filter == TextureFilterMode::XbrzSmooth;
        let use_jinc2 = self.texture_filter == TextureFilterMode::Jinc2Sharp;

        let step_x_fp = (src_w << 16) / uw;
        let step_y_fp = (src_h << 16) / uh;
        let step_x_bilerp = (src_w << 8) / uw;
        let step_y_bilerp = (src_h << 8) / uh;
        let mut y_fp = 0usize;
        let mut y_bilerp = 0usize;

        let eff_scanline_str = if self.scanline_strength > 0 {
            self.scanline_strength
        } else if self.scanlines {
            25
        } else {
            0
        };

        for dy in 0..uh {
            let py = dst_y0 + dy as i32;
            if py < vy || py >= vy + vh || py < 0 || py >= canvas.height as i32 {
                y_fp += step_y_fp;
                y_bilerp += step_y_bilerp;
                continue;
            }
            let scanline_mul: u32 = if eff_scanline_str > 0 && (dy & 1) == 1 {
                ((100u32.saturating_sub(eff_scanline_str as u32)) * 256) / 100
            } else {
                256
            };
            let fb_row = (py as usize) * canvas.width;

            if is_24bit && !self.full_vram_display {
                let vram_y = (start_y + (y_fp >> 16)) % VRAM_HEIGHT;
                let vram_row = vram_y * VRAM_WIDTH;
                let mut x_fp = 0usize;
                for dx in 0..uw {
                    let px = dst_x0 + dx as i32;
                    let sample_x = x_fp >> 16;
                    x_fp += step_x_fp;
                    if px < vx || px >= vx + vw || px < 0 || px >= canvas.width as i32 {
                        continue;
                    }
                    let byte_offset = start_x * 2 + sample_x * 3;
                    let word_idx0 = (byte_offset >> 1) & 1023;
                    let word_idx1 = (word_idx0 + 1) & 1023;
                    let w0 = bus.vram[vram_row + word_idx0];
                    let w1 = bus.vram[vram_row + word_idx1];
                    let (mut r, mut g, mut b) = if (byte_offset & 1) == 0 {
                        (
                            (w0 & 0xFF) as u32,
                            ((w0 >> 8) & 0xFF) as u32,
                            (w1 & 0xFF) as u32,
                        )
                    } else {
                        (
                            ((w0 >> 8) & 0xFF) as u32,
                            (w1 & 0xFF) as u32,
                            ((w1 >> 8) & 0xFF) as u32,
                        )
                    };
                    if scanline_mul < 256 {
                        r = (r * scanline_mul) >> 8;
                        g = (g * scanline_mul) >> 8;
                        b = (b * scanline_mul) >> 8;
                    }
                    canvas.fb[fb_row + px as usize] = (r << 16) | (g << 8) | b;
                }
            } else if use_xbrz && !self.full_vram_display {
                // EPX / Scale2x / xBRZ-lite edge-directed pixel art filter
                let sy = (start_y + (y_fp >> 16)) & 511;
                let sy_up = sy.saturating_sub(1) & 511;
                let sy_dn = (sy + 1) & 511;
                let sub_y = (y_fp >> 15) & 1;
                let mut x_fp = 0usize;
                for dx in 0..uw {
                    let px = dst_x0 + dx as i32;
                    let sx = (start_x + (x_fp >> 16)) & 1023;
                    let sub_x = (x_fp >> 15) & 1;
                    x_fp += step_x_fp;
                    if px < vx || px >= vx + vw || px < 0 || px >= canvas.width as i32 {
                        continue;
                    }
                    let sx_lt = sx.saturating_sub(1) & 1023;
                    let sx_rt = (sx + 1) & 1023;
                    let p = bus.vram[sy * 1024 + sx];
                    let a = bus.vram[sy_up * 1024 + sx];
                    let b_px = bus.vram[sy * 1024 + sx_rt];
                    let c = bus.vram[sy * 1024 + sx_lt];
                    let d = bus.vram[sy_dn * 1024 + sx];

                    let chosen = if c != b_px && a != d {
                        match (sub_x, sub_y) {
                            (0, 0) if c == a => c,
                            (1, 0) if a == b_px => b_px,
                            (0, 1) if d == c => c,
                            (1, 1) if b_px == d => b_px,
                            _ => p,
                        }
                    } else {
                        p
                    } as u32;

                    let mut r = (chosen & 0x1F) << 3;
                    let mut g = ((chosen >> 5) & 0x1F) << 3;
                    let mut b = ((chosen >> 10) & 0x1F) << 3;
                    if self.color_dithering {
                        r |= r >> 5;
                        g |= g >> 5;
                        b |= b >> 5;
                    }
                    if scanline_mul < 256 {
                        r = (r * scanline_mul) >> 8;
                        g = (g * scanline_mul) >> 8;
                        b = (b * scanline_mul) >> 8;
                    }
                    canvas.fb[fb_row + px as usize] = (r << 16) | (g << 8) | b;
                }
            } else if use_bilerp && !self.full_vram_display {
                let sy0 = (start_y + (y_bilerp >> 8)) & 511;
                let sy1 = (sy0 + 1) & 511;
                let mut wy = (y_bilerp & 0xFF) as u32;
                if use_jinc2 {
                    // JINC2 Sharp Bilinear S-curve remap: crisp texel centers, smooth transitions
                    wy = (wy * wy * (768 - 2 * wy)) >> 16;
                }
                let row0 = sy0 * 1024;
                let row1 = sy1 * 1024;
                let mut x_bilerp = 0usize;

                for dx in 0..uw {
                    let px = dst_x0 + dx as i32;
                    let fx_fp = x_bilerp;
                    x_bilerp += step_x_bilerp;
                    if px < vx || px >= vx + vw || px < 0 || px >= canvas.width as i32 {
                        continue;
                    }
                    let sx0 = (start_x + (fx_fp >> 8)) & 1023;
                    let sx1 = (sx0 + 1) & 1023;
                    let mut wx = (fx_fp & 0xFF) as u32;
                    if use_jinc2 {
                        wx = (wx * wx * (768 - 2 * wx)) >> 16;
                    }

                    let p00 = bus.vram[row0 + sx0];
                    let p10 = bus.vram[row0 + sx1];
                    let p01 = bus.vram[row1 + sx0];
                    let p11 = bus.vram[row1 + sx1];

                    let bilerp = |shift: usize| -> u32 {
                        let c00 = (((p00 >> shift) & 0x1F) as u32) << 3;
                        let c10 = (((p10 >> shift) & 0x1F) as u32) << 3;
                        let c01 = (((p01 >> shift) & 0x1F) as u32) << 3;
                        let c11 = (((p11 >> shift) & 0x1F) as u32) << 3;
                        let top = (c00 * (256 - wx) + c10 * wx) >> 8;
                        let bot = (c01 * (256 - wx) + c11 * wx) >> 8;
                        (top * (256 - wy) + bot * wy) >> 8
                    };
                    let mut r = bilerp(0);
                    let mut g = bilerp(5);
                    let mut b = bilerp(10);
                    if scanline_mul < 256 {
                        r = (r * scanline_mul) >> 8;
                        g = (g * scanline_mul) >> 8;
                        b = (b * scanline_mul) >> 8;
                    }
                    canvas.fb[fb_row + px as usize] = (r << 16) | (g << 8) | b;
                }
            } else {
                let vram_y = (start_y + (y_fp >> 16)) & 511;
                let vram_row_slice = &bus.vram[vram_y * 1024..(vram_y + 1) * 1024];
                let dither = self.color_dithering;
                let vignette = self.crt_vignette;
                let deinterlace = self.deinterlace_480i && src_h >= 400;
                let vram_y_next = (vram_y + 1) & 511;
                let vram_next_slice = &bus.vram[vram_y_next * 1024..(vram_y_next + 1) * 1024];

                // Fast-path when destination row is completely inside canvas bounds
                if dst_x0 >= 0 && (dst_x0 as usize + uw) <= canvas.width && !vignette && !deinterlace {
                    let fb_slice = &mut canvas.fb[fb_row + dst_x0 as usize..fb_row + dst_x0 as usize + uw];
                    let mut x_fp = 0usize;
                    if dither && scanline_mul == 256 {
                        for out_px in fb_slice.iter_mut() {
                            let vram_x = (start_x + (x_fp >> 16)) & 1023;
                            x_fp += step_x_fp;
                            let pixel_15 = vram_row_slice[vram_x] as u32;
                            let r5 = (pixel_15 & 0x1F) << 3;
                            let g5 = ((pixel_15 >> 5) & 0x1F) << 3;
                            let b5 = ((pixel_15 >> 10) & 0x1F) << 3;
                            let r = r5 | (r5 >> 5);
                            let g = g5 | (g5 >> 5);
                            let b = b5 | (b5 >> 5);
                            *out_px = (r << 16) | (g << 8) | b;
                        }
                    } else {
                        for out_px in fb_slice.iter_mut() {
                            let vram_x = (start_x + (x_fp >> 16)) & 1023;
                            x_fp += step_x_fp;
                            let pixel_15 = vram_row_slice[vram_x] as u32;
                            let mut r = (pixel_15 & 0x1F) << 3;
                            let mut g = ((pixel_15 >> 5) & 0x1F) << 3;
                            let mut b = ((pixel_15 >> 10) & 0x1F) << 3;
                            if dither {
                                r |= r >> 5;
                                g |= g >> 5;
                                b |= b >> 5;
                            }
                            if scanline_mul < 256 {
                                r = (r * scanline_mul) >> 8;
                                g = (g * scanline_mul) >> 8;
                                b = (b * scanline_mul) >> 8;
                            }
                            *out_px = (r << 16) | (g << 8) | b;
                        }
                    }
                } else {
                    let mut x_fp = 0usize;
                    for dx in 0..uw {
                        let px = dst_x0 + dx as i32;
                        let vram_x = (start_x + (x_fp >> 16)) & 1023;
                        x_fp += step_x_fp;
                        if px < vx || px >= vx + vw || px < 0 || px >= canvas.width as i32 {
                            continue;
                        }
                        let pixel_15 = vram_row_slice[vram_x] as u32;
                        let mut r = (pixel_15 & 0x001F) << 3;
                        let mut g = ((pixel_15 >> 5) & 0x001F) << 3;
                        let mut b = ((pixel_15 >> 10) & 0x001F) << 3;
                        if deinterlace {
                            let p_next = vram_next_slice[vram_x] as u32;
                            r = (r + ((p_next & 0x1F) << 3)) >> 1;
                            g = (g + (((p_next >> 5) & 0x1F) << 3)) >> 1;
                            b = (b + (((p_next >> 10) & 0x1F) << 3)) >> 1;
                        }
                        if dither {
                            r |= r >> 5;
                            g |= g >> 5;
                            b |= b >> 5;
                        }
                        if scanline_mul < 256 {
                            r = (r * scanline_mul) >> 8;
                            g = (g * scanline_mul) >> 8;
                            b = (b * scanline_mul) >> 8;
                        }
                        if vignette {
                            let nx = (dx as i32 * 2 - uw as i32).abs();
                            let ny = (dy as i32 * 2 - uh as i32).abs();
                            if nx * 10 > uw as i32 * 9 || ny * 10 > uh as i32 * 9 {
                                r = (r * 7) >> 3;
                                g = (g * 7) >> 3;
                                b = (b * 7) >> 3;
                            }
                        }
                        canvas.fb[fb_row + px as usize] = (r << 16) | (g << 8) | b;
                    }
                }
            }
            y_fp += step_y_fp;
            y_bilerp += step_y_bilerp;
        }

        // Apply DuckStation Post-Processing Shader Preset & Color Grading (only if enabled)
        if self.post_shader != PostShaderPreset::None
            || self.color_vibrance_pct != 100
            || self.brightness_offset != 0
        {
            self.apply_post_processing_shader(canvas, dst_x0, dst_y0, dst_w, dst_h);
        }

        // Subtle viewport border
        canvas.draw_border(dst_x0, dst_y0, dst_w, dst_h, COL_BORDER);
    }

    fn apply_post_processing_shader(
        &self,
        canvas: &mut Canvas,
        dst_x0: i32,
        dst_y0: i32,
        dst_w: i32,
        dst_h: i32,
    ) {
        let x0 = dst_x0.max(0) as usize;
        let y0 = dst_y0.max(0) as usize;
        let x1 = (dst_x0 + dst_w).min(canvas.width as i32).max(0) as usize;
        let y1 = (dst_y0 + dst_h).min(canvas.height as i32).max(0) as usize;
        if x0 >= x1 || y0 >= y1 {
            return;
        }

        let shader = self.post_shader;
        let vib = self.color_vibrance_pct as i32;
        let brt = self.brightness_offset;
        let cw = canvas.width;

        // Pass 1: Spatial Neighbor Shaders (CAS Sharpen, Kawase Bloom, VHS Chroma Bleed)
        match shader {
            PostShaderPreset::CasSharpen => {
                let mut prev_row = canvas.fb[y0 * cw + x0..y0 * cw + x1].to_vec();
                for py in (y0 + 1)..(y1.saturating_sub(1)) {
                    let cur_row = canvas.fb[py * cw + x0..py * cw + x1].to_vec();
                    let next_row_off = (py + 1) * cw;
                    let cur_row_off = py * cw + x0;
                    for i in 1..(cur_row.len().saturating_sub(1)) {
                        let c = cur_row[i];
                        let n = prev_row[i];
                        let s = canvas.fb[next_row_off + x0 + i];
                        let w = cur_row[i - 1];
                        let e = cur_row[i + 1];
                        let sharp_ch = |shift: usize| -> u32 {
                            let cc = ((c >> shift) & 0xFF) as i32;
                            let avg = ((((n >> shift) & 0xFF)
                                + ((s >> shift) & 0xFF)
                                + ((w >> shift) & 0xFF)
                                + ((e >> shift) & 0xFF))
                                >> 2) as i32;
                            (cc + ((cc - avg) >> 1)).clamp(0, 255) as u32
                        };
                        canvas.fb[cur_row_off + i] = (sharp_ch(16) << 16) | (sharp_ch(8) << 8) | sharp_ch(0);
                    }
                    prev_row = cur_row;
                }
            }
            PostShaderPreset::KawaseBloom => {
                for py in y0..y1 {
                    let row_slice = &mut canvas.fb[py * cw + x0..py * cw + x1];
                    let mut glow_r = 0i32;
                    let mut glow_g = 0i32;
                    let mut glow_b = 0i32;
                    for px_val in row_slice.iter_mut() {
                        let r = ((*px_val >> 16) & 0xFF) as i32;
                        let g = ((*px_val >> 8) & 0xFF) as i32;
                        let b = (*px_val & 0xFF) as i32;
                        let lum = (r * 77 + g * 150 + b * 29) >> 8;
                        let bright = (lum - 135).max(0);
                        glow_r = (glow_r * 3 + ((r * bright) >> 7)) >> 2;
                        glow_g = (glow_g * 3 + ((g * bright) >> 7)) >> 2;
                        glow_b = (glow_b * 3 + ((b * bright) >> 7)) >> 2;
                        let nr = (r + glow_r).min(255) as u32;
                        let ng = (g + glow_g).min(255) as u32;
                        let nb = (b + glow_b).min(255) as u32;
                        *px_val = (nr << 16) | (ng << 8) | nb;
                    }
                }
            }
            PostShaderPreset::VhsComposite => {
                for py in y0..y1 {
                    let row_slice = &mut canvas.fb[py * cw + x0..py * cw + x1];
                    let len = row_slice.len();
                    if len < 4 {
                        continue;
                    }
                    for i in (2..len - 1).rev() {
                        let r_shift = (row_slice[i - 2] >> 16) & 0xFF;
                        let g_cur = (row_slice[i] >> 8) & 0xFF;
                        let b_shift = row_slice[i + 1] & 0xFF;
                        let mut r = ((r_shift + ((row_slice[i] >> 16) & 0xFF)) >> 1) as i32;
                        let mut g = g_cur as i32;
                        let mut b = ((b_shift + (row_slice[i] & 0xFF)) >> 1) as i32;
                        if (py & 1) == 1 {
                            r = (r * 220) >> 8;
                            g = (g * 220) >> 8;
                            b = (b * 220) >> 8;
                        }
                        row_slice[i] = ((r as u32) << 16) | ((g as u32) << 8) | (b as u32);
                    }
                }
            }
            _ => {}
        }

        // Pass 2: Per-Pixel CRT / LCD Mask + Vibrance + Brightness
        for py in y0..y1 {
            let row_slice = &mut canvas.fb[py * cw + x0..py * cw + x1];
            for (dx, px_val) in row_slice.iter_mut().enumerate() {
                let mut r = ((*px_val >> 16) & 0xFF) as i32;
                let mut g = ((*px_val >> 8) & 0xFF) as i32;
                let mut b = (*px_val & 0xFF) as i32;

                match shader {
                    PostShaderPreset::CrtTrinitron => {
                        // Sony Trinitron Aperture Grille RGB subpixel columns + phosphor scanline
                        match dx % 3 {
                            0 => {
                                r = (r * 280) >> 8;
                                g = (g * 225) >> 8;
                                b = (b * 225) >> 8;
                            }
                            1 => {
                                r = (r * 225) >> 8;
                                g = (g * 280) >> 8;
                                b = (b * 225) >> 8;
                            }
                            _ => {
                                r = (r * 225) >> 8;
                                g = (g * 225) >> 8;
                                b = (b * 280) >> 8;
                            }
                        }
                        if (py & 1) == 1 {
                            let lum = (r + g + b) / 3;
                            let beam = (175 + (lum >> 2)).min(245);
                            r = (r * beam) >> 8;
                            g = (g * beam) >> 8;
                            b = (b * beam) >> 8;
                        }
                    }
                    PostShaderPreset::CrtLottesArcade => {
                        // Lottes staggered shadow-mask triad + arcade phosphor boost
                        let triad = (dx + (py & 1) * 2) % 3;
                        match triad {
                            0 => r = (r * 276) >> 8,
                            1 => g = (g * 276) >> 8,
                            _ => b = (b * 276) >> 8,
                        }
                        if (py % 3) == 2 {
                            r = (r * 195) >> 8;
                            g = (g * 195) >> 8;
                            b = (b * 195) >> 8;
                        }
                    }
                    PostShaderPreset::LcdGrid => {
                        // PSone Portable LCD 3x3 subpixel cell border
                        if (dx % 3) == 2 || (py % 3) == 2 {
                            r = (r * 196) >> 8;
                            g = (g * 204) >> 8;
                            b = (b * 196) >> 8;
                        }
                    }
                    PostShaderPreset::MonochromeNoir => {
                        // Warm Vintage Sepia / Noir Film Tone
                        let lum = (r * 77 + g * 150 + b * 29) >> 8;
                        r = (lum * 280) >> 8;
                        g = (lum * 245) >> 8;
                        b = (lum * 195) >> 8;
                    }
                    _ => {}
                }

                if vib != 100 {
                    let avg = (r + g + b) / 3;
                    r = avg + ((r - avg) * vib) / 100;
                    g = avg + ((g - avg) * vib) / 100;
                    b = avg + ((b - avg) * vib) / 100;
                }
                if brt != 0 {
                    r += brt;
                    g += brt;
                    b += brt;
                }

                let fr = r.clamp(0, 255) as u32;
                let fg = g.clamp(0, 255) as u32;
                let fb_c = b.clamp(0, 255) as u32;
                *px_val = (fr << 16) | (fg << 8) | fb_c;
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_game_library(
        &mut self,
        canvas: &mut Canvas,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        clicked: bool,
        scroll_y: f32,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        canvas.fill_rect(x, y, w, h, COL_BG_DARK);

        // Header banner
        let banner_h = 74;
        canvas.draw_panel(x + 16, y + 12, w - 32, banner_h, COL_BG_PANEL, COL_BORDER);
        // DuckStation-style emblem icon
        canvas.fill_rect(x + 30, y + 24, 48, 48, COL_ACCENT);
        canvas.draw_border(x + 30, y + 24, 48, 48, COL_ACCENT_HOVER);
        canvas.draw_text(x + 42, y + 40, "PSX", 0xFFFFFF);
        canvas.draw_text(x + 42, y + 52, "EMU", COL_YELLOW);

        canvas.draw_text(
            x + 92,
            y + 26,
            "DuckStation-Style PlayStation 1 Frontend",
            COL_TEXT,
        );
        let bios_status = match self.bios_path {
            Some(ref bp) => {
                let name = Path::new(bp)
                    .file_name()
                    .map(|s| s.to_string_lossy().to_string())
                    .unwrap_or_else(|| bp.clone());
                format!("BIOS Configured: {} (Ready)", name)
            }
            None => "BIOS: None selected (PS-EXEs boot directly; select SCPH1001.BIN for CD-ROMs)".to_string(),
        };
        canvas.draw_text_clipped(
            x + 92,
            y + 44,
            ((w - 380).max(20) / 8) as usize,
            &bios_status,
            if self.bios_path.is_some() { COL_GREEN } else { COL_YELLOW },
        );
        canvas.draw_text(
            x + 92,
            y + 60,
            "Supports: CD-ROM (.bin/.iso/.img), PS-X EXE (.exe), SPU 24-Voice Audio + DualShock",
            COL_TEXT_DIM,
        );

        // Quick Action Buttons in Banner
        let btn_open_x = x + w - 260;
        let btn_open_y = y + 24;
        let btn_w = 115;
        let btn_h = 22;
        if self.draw_button(canvas, btn_open_x, btn_open_y, btn_w, btn_h, "Open File...", true) && clicked {
            if let Some(path) = open_file_dialog(
                "Select PlayStation Disc (.bin/.iso) or PS-X EXE (.exe)",
                b"PS1 Disc / EXE (*.bin;*.iso;*.img;*.exe;*.psx)\0*.bin;*.iso;*.img;*.exe;*.psx\0All Files (*.*)\0*.*\0\0",
            ) {
                self.boot_file_path(&path, cpu, bus);
            }
        }

        let btn_bios_x = x + w - 138;
        if self.draw_button(canvas, btn_bios_x, btn_open_y, btn_w, btn_h, "Select BIOS...", false) && clicked {
            if let Some(path) = open_file_dialog(
                "Select PlayStation BIOS ROM (SCPH1001.BIN)",
                b"BIOS ROM (*.bin;*.rom)\0*.bin;*.rom\0All Files (*.*)\0*.*\0\0",
            ) {
                self.bios_path = Some(path.clone());
                self.push_osd(format!("Selected BIOS: {}", path));
            }
        }

        let btn_boot_bios_y = y + 52;
        if self.draw_button(canvas, btn_open_x, btn_boot_bios_y, btn_w, btn_h, "Boot BIOS", false) && clicked {
            self.boot_bios_only(cpu, bus);
        }
        if self.draw_button(canvas, btn_bios_x, btn_boot_bios_y, btn_w, btn_h, "Rescan Games", false) && clicked {
            self.scan_workspace_games();
            self.push_osd(format!("Found {} PS1 titles in workspace", self.game_entries.len()));
        }

        // Game Library Table
        let table_x = x + 16;
        let table_y = y + 98;
        let table_w = w - 32;
        let table_h = (h - 110).max(80);
        canvas.draw_panel(table_x, table_y, table_w, table_h, COL_BG_PANEL, COL_BORDER);

        // Table Header
        canvas.fill_rect(table_x + 1, table_y + 1, table_w - 2, 26, COL_BG_ELEVATED);
        canvas.draw_text(table_x + 14, table_y + 10, "TYPE", COL_TEXT_ACCENT);
        canvas.draw_text(table_x + 90, table_y + 10, "GAME / EXECUTABLE TITLE", COL_TEXT_ACCENT);
        canvas.draw_text(table_x + 360, table_y + 10, "SIZE", COL_TEXT_ACCENT);
        canvas.draw_text(table_x + 450, table_y + 10, "FILE PATH", COL_TEXT_ACCENT);
        canvas.draw_text(table_x + table_w - 92, table_y + 10, "ACTION", COL_TEXT_ACCENT);

        let row_h = 30;
        let max_rows = ((table_h - 32) / row_h).max(1) as usize;
        if scroll_y < 0.0 && self.game_list_scroll + max_rows < self.game_entries.len() {
            self.game_list_scroll += 1;
        } else if scroll_y > 0.0 && self.game_list_scroll > 0 {
            self.game_list_scroll -= 1;
        }

        let mut boot_idx: Option<usize> = None;
        if self.game_entries.is_empty() {
            canvas.draw_text(
                table_x + 24,
                table_y + 48,
                "No .bin/.iso/.exe files found in default folders. Click 'Open File...' above to load any file!",
                COL_TEXT_DIM,
            );
        } else {
            for (vis_i, entry_idx) in (self.game_list_scroll..self.game_entries.len())
                .take(max_rows)
                .enumerate()
            {
                let ry = table_y + 28 + (vis_i as i32) * row_h;
                let hovered = rect_contains(self.mouse_x, self.mouse_y, table_x + 2, ry, table_w - 4, row_h - 2);
                let selected = self.selected_game_idx == entry_idx;
                let row_bg = if selected {
                    COL_ACCENT_DIM
                } else if hovered {
                    COL_BG_HOVER
                } else if vis_i % 2 == 0 {
                    COL_BG_PANEL
                } else {
                    COL_BG_INPUT
                };
                canvas.fill_rect(table_x + 2, ry, table_w - 4, row_h - 2, row_bg);

                let entry = &self.game_entries[entry_idx];
                // Badge
                let (badge_txt, badge_col) = if entry.is_exe {
                    ("PS-EXE", COL_PURPLE)
                } else {
                    ("CD-ROM", COL_ACCENT)
                };
                canvas.fill_rect(table_x + 12, ry + 5, 58, 18, badge_col);
                canvas.draw_text(table_x + 17, ry + 10, badge_txt, 0xFFFFFF);

                canvas.draw_text_clipped(table_x + 90, ry + 10, 32, &entry.title, COL_TEXT);

                let size_str = if entry.size_bytes >= 1024 * 1024 {
                    format!("{:.1} MB", (entry.size_bytes as f64) / (1024.0 * 1024.0))
                } else {
                    format!("{} KB", entry.size_bytes / 1024)
                };
                canvas.draw_text(table_x + 360, ry + 10, &size_str, COL_TEXT_DIM);

                let path_max_chars = ((table_w - 560).max(16) / 8) as usize;
                canvas.draw_text_clipped(table_x + 450, ry + 10, path_max_chars, &entry.path, COL_TEXT_DIM);

                let launch_btn_x = table_x + table_w - 94;
                if self.draw_button(canvas, launch_btn_x, ry + 4, 80, 20, "Launch", true) && clicked {
                    boot_idx = Some(entry_idx);
                } else if hovered && clicked {
                    if self.selected_game_idx == entry_idx {
                        boot_idx = Some(entry_idx);
                    } else {
                        self.selected_game_idx = entry_idx;
                    }
                }
            }
        }

        if let Some(idx) = boot_idx {
            if let Some(entry) = self.game_entries.get(idx).cloned() {
                self.selected_game_idx = idx;
                self.boot_file_path(&entry.path, cpu, bus);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_toolbar(
        &mut self,
        canvas: &mut Canvas,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        clicked: bool,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        canvas.fill_rect(x, y, w, h, COL_BG_ELEVATED);
        canvas.fill_rect(x, y + h - 1, w, 1, COL_BORDER);

        let mut bx = x + 8;
        let by = y + 5;
        let bh = h - 10;

        // 1. Game List / Display Toggle
        let list_label = if self.show_game_list { "Display" } else { "Game List" };
        if self.draw_button(canvas, bx, by, 80, bh, list_label, self.show_game_list) && clicked {
            self.show_game_list = !self.show_game_list;
        }
        bx += 86;

        // 2. Open File (Native Dialog)
        if self.draw_button(canvas, bx, by, 82, bh, "Open File", false) && clicked {
            if let Some(path) = open_file_dialog(
                "Select PlayStation Disc (.bin/.iso) or PS-X EXE (.exe)",
                b"PS1 Disc / EXE (*.bin;*.iso;*.img;*.exe;*.psx)\0*.bin;*.iso;*.img;*.exe;*.psx\0All Files (*.*)\0*.*\0\0",
            ) {
                self.boot_file_path(&path, cpu, bus);
            }
        }
        bx += 88;

        // 3. Pause / Resume
        let pause_label = if self.paused { "Resume" } else { "Pause" };
        if self.draw_button(canvas, bx, by, 66, bh, pause_label, self.paused) && clicked {
            self.paused = !self.paused;
            self.push_osd(if self.paused { "Emulation Paused" } else { "Emulation Resumed" });
        }
        bx += 72;

        // 4. Reset
        if self.draw_button(canvas, bx, by, 58, bh, "Reset", false) && clicked {
            self.reset_system(cpu, bus);
        }
        bx += 64;

        // 5. Power Off
        if self.draw_button(canvas, bx, by, 54, bh, "Stop", false) && clicked {
            self.running = false;
            self.show_game_list = true;
            self.push_osd("System Powered Off (Returned to Game List)");
        }
        bx += 62;

        // Separator
        canvas.fill_rect(bx, by + 2, 1, bh - 4, COL_BORDER);
        bx += 8;

        // 6. Save State & Load State
        if self.draw_button(canvas, bx, by, 72, bh, "Save St", false) && clicked {
            self.save_state_slot(self.selected_slot, cpu, bus);
        }
        bx += 78;

        if self.draw_button(canvas, bx, by, 72, bh, "Load St", false) && clicked {
            self.load_state_slot(self.selected_slot, cpu, bus);
        }
        bx += 78;

        // 7. Fast Forward / Turbo
        if self.draw_button(canvas, bx, by, 66, bh, "Turbo", self.fast_forward) && clicked {
            self.fast_forward = !self.fast_forward;
            self.push_osd(format!(
                "Fast Forward: {}",
                if self.fast_forward { "ON" } else { "OFF" }
            ));
        }
        bx += 74;

        // Separator
        canvas.fill_rect(bx, by + 2, 1, bh - 4, COL_BORDER);
        bx += 8;

        // 8. Settings Modal & Shaders Quick-Button
        if self.draw_button(canvas, bx, by, 72, bh, "Settings", self.settings_open) && clicked {
            self.settings_open = !self.settings_open;
        }
        bx += 78;

        let shd_open = self.settings_open && self.settings_tab == SettingsTab::Shaders;
        if self.draw_button(canvas, bx, by, 68, bh, "Shaders", shd_open) && clicked {
            if shd_open {
                self.settings_open = false;
            } else {
                self.settings_open = true;
                self.settings_tab = SettingsTab::Shaders;
            }
        }
        bx += 74;

        // 9. VRAM Viewer
        let vram_open = self.tool_window == ActiveToolWindow::VramViewer;
        if self.draw_button(canvas, bx, by, 50, bh, "VRAM", vram_open) && clicked {
            self.tool_window = if vram_open {
                ActiveToolWindow::None
            } else {
                ActiveToolWindow::VramViewer
            };
        }
        bx += 54;

        // 10. CPU Debugger
        let dbg_open = self.tool_window == ActiveToolWindow::CpuDebugger;
        if self.draw_button(canvas, bx, by, 46, bh, "CPU", dbg_open) && clicked {
            self.tool_window = if dbg_open {
                ActiveToolWindow::None
            } else {
                ActiveToolWindow::CpuDebugger
            };
        }
        bx += 50;

        // 11. SPU Audio Scope
        let spu_open = self.tool_window == ActiveToolWindow::SpuInspector;
        if self.draw_button(canvas, bx, by, 46, bh, "SPU", spu_open) && clicked {
            self.tool_window = if spu_open {
                ActiveToolWindow::None
            } else {
                ActiveToolWindow::SpuInspector
            };
        }
        bx += 50;

        // 12. Memory & TTY Viewer
        let mem_open = self.tool_window == ActiveToolWindow::MemoryTtyViewer;
        if bx + 46 < x + w && self.draw_button(canvas, bx, by, 46, bh, "MEM", mem_open) && clicked {
            self.tool_window = if mem_open {
                ActiveToolWindow::None
            } else {
                ActiveToolWindow::MemoryTtyViewer
            };
        }
    }

    fn draw_status_bar(
        &self,
        canvas: &mut Canvas,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        cpu: &Cpu,
        bus: &Bus,
    ) {
        canvas.fill_rect(x, y, w, h, COL_BG_PANEL);
        canvas.fill_rect(x, y, w, 1, COL_BORDER);

        let (state_str, state_col) = if !self.running {
            ("IDLE", COL_TEXT_DIM)
        } else if self.paused {
            ("PAUSED", COL_YELLOW)
        } else if self.fast_forward {
            ("TURBO", COL_ACCENT)
        } else {
            ("RUNNING", COL_GREEN)
        };
        canvas.fill_rect(x + 8, y + 4, 64, h - 8, state_col);
        canvas.draw_text(x + 12, y + 7, state_str, 0x000000);

        let is_24bit = (bus.gpustat & (1 << 21)) != 0;
        let disp_w = if bus.display_width > 0 { bus.display_width } else { 320 };
        let disp_h = if bus.display_height > 0 { bus.display_height } else { 240 };
        let pad_mode = if bus.pad1.analog_mode {
            "DualShock (0x73)"
        } else {
            "Digital (0x41)"
        };

        let status_line = format!(
            "{} | GPU: {}x{} ({}) | Pad1: {} | CPU: {}% | Slot {} | {:.1} FPS (PC={:08X})",
            self.current_game_title,
            disp_w,
            disp_h,
            if is_24bit { "24-bit" } else { "15-bit" },
            pad_mode,
            self.cpu_overclock_pct,
            self.selected_slot + 1,
            self.current_fps,
            cpu.pc
        );
        let max_chars = ((w - 86).max(10) / 8) as usize;
        canvas.draw_text_clipped(x + 80, y + 7, max_chars, &status_line, COL_TEXT);
    }

    fn draw_osd_messages(&mut self, canvas: &mut Canvas, x: i32, y: i32) {
        // Expire messages older than 4 seconds
        while let Some((_, ts)) = self.osd_messages.front() {
            if ts.elapsed().as_secs_f32() > 4.0 {
                self.osd_messages.pop_front();
            } else {
                break;
            }
        }
        let mut cy = y;
        for (msg, _) in &self.osd_messages {
            let box_w = ((msg.len() as i32) * 8 + 16).min(canvas.width as i32 - 24);
            canvas.fill_rect_alpha(x, cy, box_w, 20, COL_BG_DARK, 210);
            canvas.draw_border(x, cy, box_w, 20, COL_ACCENT);
            canvas.draw_text_clipped(x + 8, cy + 6, ((box_w - 16) / 8) as usize, msg, COL_TEXT);
            cy += 24;
        }
    }

    fn draw_fps_overlay(&self, canvas: &mut Canvas, x: i32, y: i32, cpu: &Cpu, bus: &Bus) {
        if x < 0 {
            return;
        }
        let active_voices = bus.spu.voices.iter().filter(|v| v.active).count();
        let line1 = format!("{:5.1} FPS | Speed {:3.0}%", self.current_fps, (self.current_fps / 60.0) * 100.0);
        let line2 = format!("SPU Voices: {:2}/24 | Cyc: {}M", active_voices, cpu.cycles / 1_000_000);
        canvas.fill_rect_alpha(x, y, 218, 36, COL_BG_DARK, 200);
        canvas.draw_border(x, y, 218, 36, COL_BORDER);
        canvas.draw_text(x + 8, y + 6, &line1, COL_GREEN);
        canvas.draw_text(x + 8, y + 20, &line2, COL_TEXT_DIM);
    }

    pub fn draw_button(
        &self,
        canvas: &mut Canvas,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        label: &str,
        active_or_primary: bool,
    ) -> bool {
        let hovered = rect_contains(self.mouse_x, self.mouse_y, x, y, w, h);
        let bg = if active_or_primary {
            if hovered { COL_ACCENT_HOVER } else { COL_ACCENT }
        } else if hovered {
            COL_BG_HOVER
        } else {
            COL_BG_PANEL
        };
        let border = if hovered || active_or_primary {
            COL_ACCENT_HOVER
        } else {
            COL_BORDER
        };
        canvas.draw_panel(x, y, w, h, bg, border);
        let text_w = (label.len() as i32) * 8;
        let tx = x + ((w - text_w) / 2).max(4);
        let ty = y + (h - 8) / 2;
        canvas.draw_text(tx, ty, label, if active_or_primary { 0xFFFFFF } else { COL_TEXT });
        hovered
    }

    pub fn draw_checkbox(
        &self,
        canvas: &mut Canvas,
        x: i32,
        y: i32,
        label: &str,
        checked: bool,
    ) -> bool {
        let w = 22 + (label.len() as i32) * 8;
        let h = 18;
        let hovered = rect_contains(self.mouse_x, self.mouse_y, x, y, w, h);
        let box_bg = if checked {
            COL_ACCENT
        } else if hovered {
            COL_BG_HOVER
        } else {
            COL_BG_INPUT
        };
        canvas.draw_panel(x, y + 1, 16, 16, box_bg, if hovered { COL_ACCENT_HOVER } else { COL_BORDER });
        if checked {
            canvas.draw_char(x + 4, y + 5, 'x', 0xFFFFFF);
        }
        canvas.draw_text(x + 24, y + 5, label, if hovered { 0xFFFFFF } else { COL_TEXT });
        hovered
    }
}

impl DuckStationUi {
    fn draw_menu_bar(
        &mut self,
        canvas: &mut Canvas,
        win_w: i32,
        clicked: bool,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        let mh = UI_MENU_HEIGHT as i32;
        canvas.fill_rect(0, 0, win_w, mh, COL_BG_PANEL);
        canvas.fill_rect(0, mh - 1, win_w, 1, COL_BORDER);

        // App Brand Badge on left
        canvas.fill_rect(6, 3, 28, mh - 6, COL_ACCENT);
        canvas.draw_text(10, 8, "DS", 0xFFFFFF);

        let menus = [
            ("System", ActiveMenu::System, 42, 64),
            ("Settings", ActiveMenu::Settings, 108, 78),
            ("View", ActiveMenu::View, 188, 52),
            ("Tools", ActiveMenu::Tools, 242, 58),
        ];

        let mut menu_header_clicked = false;
        for (title, mtype, mx, mw) in menus {
            let hovered = rect_contains(self.mouse_x, self.mouse_y, mx, 0, mw, mh);
            let active = self.active_menu == mtype;
            if active || hovered {
                canvas.fill_rect(
                    mx,
                    1,
                    mw,
                    mh - 2,
                    if active { COL_ACCENT } else { COL_BG_HOVER },
                );
            }
            canvas.draw_text(mx + 10, 8, title, if active { 0xFFFFFF } else { COL_TEXT });
            if hovered && clicked {
                self.active_menu = if active { ActiveMenu::None } else { mtype };
                menu_header_clicked = true;
            } else if hovered && self.active_menu != ActiveMenu::None && self.active_menu != mtype {
                // Hover-switch between open top menus like native Windows/DuckStation menus
                self.active_menu = mtype;
            }
        }

        // Right-side quick status in menu bar
        let right_hint = "F1:Save F3:Load F5:Analog F6:Pause Tab:Turbo";
        let hint_w = (right_hint.len() as i32) * 8;
        if win_w > 320 + hint_w {
            canvas.draw_text(win_w - hint_w - 10, 8, right_hint, COL_TEXT_DIM);
        }

        if self.active_menu == ActiveMenu::None {
            return;
        }

        // Draw active dropdown menu
        let (drop_x, drop_w, items): (i32, i32, Vec<(&str, &str, bool)>) = match self.active_menu {
            ActiveMenu::System => (
                42,
                250,
                vec![
                    ("Boot BIOS Shell", "", false),
                    ("Start File (Disc / EXE)...", "Ctrl+O", false),
                    ("Change Disc Image...", "", false),
                    ("Pause / Resume", "F6", self.paused),
                    ("Frame Step (1 Frame)", "F7", false),
                    ("Reset System", "F8", false),
                    ("Power Off (Stop to List)", "", !self.running),
                    ("Fast Forward (Turbo)", "Tab", self.fast_forward),
                    ("Quick Save State (Slot 1)", "F1", self.save_slots[0].is_some()),
                    ("Quick Load State (Slot 1)", "F3", false),
                    ("Cycle Active Slot (1..5)", "F2", false),
                    ("Exit Emulator", "", false),
                ],
            ),
            ActiveMenu::Settings => (
                108,
                260,
                vec![
                    ("Console Settings...", "", self.settings_open && self.settings_tab == SettingsTab::Console),
                    ("BIOS Settings...", "", self.settings_open && self.settings_tab == SettingsTab::Bios),
                    ("Display Settings...", "", self.settings_open && self.settings_tab == SettingsTab::Display),
                    ("Enhancement Settings...", "", self.settings_open && self.settings_tab == SettingsTab::Enhancements),
                    ("Post-Processing Shaders...", "", self.settings_open && self.settings_tab == SettingsTab::Shaders),
                    ("Audio Settings...", "", self.settings_open && self.settings_tab == SettingsTab::Audio),
                    ("Controller Settings...", "", self.settings_open && self.settings_tab == SettingsTab::Controllers),
                    ("Rescan Game Library", "", false),
                ],
            ),
            ActiveMenu::View => (
                188,
                268,
                vec![
                    ("Show Toolbar", "", self.show_toolbar),
                    ("Show Status Bar", "", self.show_status_bar),
                    ("Game List Dashboard", "", self.show_game_list),
                    ("OSD Toast Notifications", "", self.show_osd),
                    ("FPS & SPU Speed Overlay", "", self.show_fps_overlay),
                    ("Cycle Aspect Ratio", "", false),
                    ("Cycle Texture Filter", "F11", self.texture_filter != TextureFilterMode::Nearest),
                    ("Cycle Post-Processing Shader", "F9", self.post_shader != PostShaderPreset::None),
                    ("CRT Scanlines Effect", "", self.scanlines || self.scanline_strength > 0),
                    ("Full 1024x512 VRAM Display", "", self.full_vram_display),
                ],
            ),
            ActiveMenu::Tools => (
                242,
                260,
                vec![
                    ("VRAM Viewer (1024x512)...", "", self.tool_window == ActiveToolWindow::VramViewer),
                    ("CPU & GTE Debugger...", "", self.tool_window == ActiveToolWindow::CpuDebugger),
                    ("SPU 24-Voice & Audio Scope...", "", self.tool_window == ActiveToolWindow::SpuInspector),
                    ("Memory Editor & TTY Log...", "", self.tool_window == ActiveToolWindow::MemoryTtyViewer),
                    ("Save VRAM Screenshot (.ppm)", "F10", false),
                ],
            ),
            ActiveMenu::None => return,
        };

        let row_h = 24;
        let drop_y = mh;
        let drop_h = (items.len() as i32) * row_h + 6;
        canvas.draw_panel(drop_x, drop_y, drop_w, drop_h, COL_BG_ELEVATED, COL_ACCENT);

        let mut clicked_idx: Option<usize> = None;
        for (i, &(label, shortcut, checked)) in items.iter().enumerate() {
            let iy = drop_y + 3 + (i as i32) * row_h;
            let hov = rect_contains(self.mouse_x, self.mouse_y, drop_x + 2, iy, drop_w - 4, row_h);
            if hov {
                canvas.fill_rect(drop_x + 2, iy, drop_w - 4, row_h, COL_ACCENT);
            }
            if checked {
                canvas.draw_char(drop_x + 8, iy + 8, '*', if hov { 0xFFFFFF } else { COL_GREEN });
            }
            canvas.draw_text(
                drop_x + 22,
                iy + 8,
                label,
                if hov { 0xFFFFFF } else { COL_TEXT },
            );
            if !shortcut.is_empty() {
                let sw = (shortcut.len() as i32) * 8;
                canvas.draw_text(
                    drop_x + drop_w - sw - 10,
                    iy + 8,
                    shortcut,
                    if hov { 0xFFFFFF } else { COL_TEXT_DIM },
                );
            }
            if hov && clicked {
                clicked_idx = Some(i);
            }
        }

        if let Some(idx) = clicked_idx {
            let menu_kind = self.active_menu;
            self.active_menu = ActiveMenu::None;
            self.execute_menu_action(menu_kind, idx, cpu, bus);
        } else if clicked && !menu_header_clicked {
            // Clicked outside dropdown -> close menu
            self.active_menu = ActiveMenu::None;
        }
    }

    fn execute_menu_action(
        &mut self,
        menu: ActiveMenu,
        idx: usize,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        match menu {
            ActiveMenu::System => match idx {
                0 => self.boot_bios_only(cpu, bus),
                1 => {
                    if let Some(path) = open_file_dialog(
                        "Select PlayStation Disc (.bin/.iso) or PS-X EXE (.exe)",
                        b"PS1 Disc / EXE (*.bin;*.iso;*.img;*.exe;*.psx)\0*.bin;*.iso;*.img;*.exe;*.psx\0All Files (*.*)\0*.*\0\0",
                    ) {
                        self.boot_file_path(&path, cpu, bus);
                    }
                }
                2 => {
                    if let Some(path) = open_file_dialog(
                        "Change PlayStation Disc Image (.bin/.iso)",
                        b"CD-ROM Images (*.bin;*.iso;*.img)\0*.bin;*.iso;*.img\0All Files (*.*)\0*.*\0\0",
                    ) {
                        match bus.load_cd_bin_file(&path) {
                            Ok(_) => {
                                let name = Path::new(&path)
                                    .file_name()
                                    .map(|s| s.to_string_lossy().to_string())
                                    .unwrap_or_else(|| path.clone());
                                self.current_disc_path = Some(path);
                                self.current_game_title = name.clone();
                                self.push_osd(format!("Swapped Disc to: {}", name));
                            }
                            Err(e) => self.push_osd(format!("Disc Swap Error: {}", e)),
                        }
                    }
                }
                3 => {
                    self.paused = !self.paused;
                    self.push_osd(if self.paused { "Emulation Paused" } else { "Emulation Resumed" });
                }
                4 => {
                    self.paused = true;
                    self.step_one_frame = true;
                    self.push_osd("Stepped 1 Video Frame");
                }
                5 => self.reset_system(cpu, bus),
                6 => {
                    self.running = false;
                    self.show_game_list = true;
                    self.push_osd("System Powered Off");
                }
                7 => {
                    self.fast_forward = !self.fast_forward;
                    self.push_osd(format!(
                        "Fast Forward: {}",
                        if self.fast_forward { "ON" } else { "OFF" }
                    ));
                }
                8 => self.save_state_slot(self.selected_slot, cpu, bus),
                9 => self.load_state_slot(self.selected_slot, cpu, bus),
                10 => {
                    self.selected_slot = (self.selected_slot + 1) % 5;
                    self.push_osd(format!("Active Save Slot: {}", self.selected_slot + 1));
                }
                11 => self.request_exit = true,
                _ => {}
            },
            ActiveMenu::Settings => match idx {
                0 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Console;
                }
                1 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Bios;
                }
                2 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Display;
                }
                3 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Enhancements;
                }
                4 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Shaders;
                }
                5 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Audio;
                }
                6 => {
                    self.settings_open = true;
                    self.settings_tab = SettingsTab::Controllers;
                }
                7 => {
                    self.scan_workspace_games();
                    self.push_osd(format!("Rescanned Workspace: {} games found", self.game_entries.len()));
                }
                _ => {}
            },
            ActiveMenu::View => match idx {
                0 => self.show_toolbar = !self.show_toolbar,
                1 => self.show_status_bar = !self.show_status_bar,
                2 => self.show_game_list = !self.show_game_list,
                3 => self.show_osd = !self.show_osd,
                4 => self.show_fps_overlay = !self.show_fps_overlay,
                5 => {
                    self.aspect_ratio = self.aspect_ratio.next();
                    self.push_osd(format!("Aspect Ratio: {}", self.aspect_ratio.label()));
                }
                6 => {
                    self.texture_filter = self.texture_filter.next();
                    self.bilinear_filtering = matches!(
                        self.texture_filter,
                        TextureFilterMode::BilinearViewport | TextureFilterMode::Bilinear3DAndViewport
                    );
                    self.push_osd(format!("Texture Filter: {}", self.texture_filter.label()));
                }
                7 => {
                    self.post_shader = self.post_shader.next();
                    self.push_osd(format!("Post-Processing Shader: {}", self.post_shader.label()));
                }
                8 => {
                    self.scanlines = !self.scanlines;
                    self.scanline_strength = if self.scanlines { 25 } else { 0 };
                    self.push_osd(format!(
                        "CRT Scanlines: {}",
                        if self.scanlines { "ON (25%)" } else { "OFF" }
                    ));
                }
                9 => {
                    self.full_vram_display = !self.full_vram_display;
                    self.push_osd(format!(
                        "Full 1024x512 VRAM View: {}",
                        if self.full_vram_display { "ON" } else { "OFF" }
                    ));
                }
                _ => {}
            },
            ActiveMenu::Tools => match idx {
                0 => self.tool_window = ActiveToolWindow::VramViewer,
                1 => self.tool_window = ActiveToolWindow::CpuDebugger,
                2 => self.tool_window = ActiveToolWindow::SpuInspector,
                3 => self.tool_window = ActiveToolWindow::MemoryTtyViewer,
                4 => {
                    let filename = "vram_screenshot.ppm";
                    match bus.dump_vram_ppm(filename) {
                        Ok(_) => self.push_osd(format!("Saved VRAM Screenshot to {}", filename)),
                        Err(e) => self.push_osd(format!("Screenshot failed: {}", e)),
                    }
                }
                _ => {}
            },
            ActiveMenu::None => {}
        }
    }

    fn draw_settings_modal(
        &mut self,
        canvas: &mut Canvas,
        win_w: i32,
        win_h: i32,
        clicked: bool,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        let mw = (win_w - 80).clamp(540, 740);
        let mh = (win_h - 90).clamp(380, 500);
        let mx = (win_w - mw) / 2;
        let my = (win_h - mh) / 2;

        // Dim backdrop
        canvas.fill_rect_alpha(0, UI_MENU_HEIGHT as i32, win_w, win_h - UI_MENU_HEIGHT as i32, 0x000000, 140);
        canvas.draw_panel(mx, my, mw, mh, COL_BG_PANEL, COL_ACCENT);

        // Title Bar
        canvas.fill_rect(mx + 1, my + 1, mw - 2, 28, COL_BG_ELEVATED);
        canvas.draw_text(mx + 12, my + 10, "DuckStation Settings", COL_TEXT_ACCENT);
        if self.draw_button(canvas, mx + mw - 68, my + 4, 62, 20, "Close", true) && clicked {
            self.settings_open = false;
            return;
        }

        // Left Sidebar Tabs
        let side_w = 148;
        canvas.fill_rect(mx + 1, my + 30, side_w, mh - 31, COL_BG_INPUT);
        canvas.fill_rect(mx + side_w, my + 30, 1, mh - 31, COL_BORDER);

        let tabs = [
            ("Console", SettingsTab::Console),
            ("BIOS", SettingsTab::Bios),
            ("Display", SettingsTab::Display),
            ("Enhancements", SettingsTab::Enhancements),
            ("Post Shaders", SettingsTab::Shaders),
            ("Audio (SPU)", SettingsTab::Audio),
            ("Controllers", SettingsTab::Controllers),
        ];

        for (i, &(tname, tab_val)) in tabs.iter().enumerate() {
            let ty = my + 38 + (i as i32) * 30;
            let active = self.settings_tab == tab_val;
            let hov = rect_contains(self.mouse_x, self.mouse_y, mx + 6, ty, side_w - 12, 24);
            if active || hov {
                canvas.fill_rect(
                    mx + 6,
                    ty,
                    side_w - 12,
                    24,
                    if active { COL_ACCENT } else { COL_BG_HOVER },
                );
            }
            canvas.draw_text(
                mx + 16,
                ty + 8,
                tname,
                if active { 0xFFFFFF } else { COL_TEXT },
            );
            if hov && clicked {
                self.settings_tab = tab_val;
            }
        }

        // Right Content Pane
        let px = mx + side_w + 18;
        let py = my + 42;
        let pw = mw - side_w - 32;

        match self.settings_tab {
            SettingsTab::Console => {
                canvas.draw_text(px, py, "Console & CPU Emulation Settings", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                canvas.draw_text(px, py + 28, "CPU Clock Speed (Overclocking):", COL_TEXT);
                let speeds = [50u32, 100, 150, 200, 300];
                for (i, &spd) in speeds.iter().enumerate() {
                    let bx = px + (i as i32) * 68;
                    let label = format!("{}%", spd);
                    if self.draw_button(canvas, bx, py + 44, 62, 22, &label, self.cpu_overclock_pct == spd)
                        && clicked
                    {
                        self.cpu_overclock_pct = spd;
                        self.push_osd(format!("CPU Clock Speed set to {}%", spd));
                    }
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 82,
                    "Fast Boot (Skip BIOS Shell intro when loading PS-EXE)",
                    self.fast_boot,
                ) && clicked
                {
                    self.fast_boot = !self.fast_boot;
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 110,
                    "Force NTSC 60 Hz Timing (33.8688 MHz / 263 Scanlines)",
                    self.force_ntsc_60,
                ) && clicked
                {
                    self.force_ntsc_60 = !self.force_ntsc_60;
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 138,
                    "Enable MIPS CPU Instruction Trace (stdout)",
                    cpu.trace_enabled,
                ) && clicked
                {
                    cpu.trace_enabled = !cpu.trace_enabled;
                }

                // Save State Manager inside Console Settings
                canvas.draw_text(px, py + 174, "Save State Slots (Instant In-Memory Snapshots):", COL_TEXT_ACCENT);
                for slot in 0..5 {
                    let sy = py + 194 + (slot as i32) * 26;
                    let slot_lbl = match self.save_slots[slot] {
                        Some(ref s) => format!("Slot {}: {}", slot + 1, s.label),
                        None => format!("Slot {}: <Empty>", slot + 1),
                    };
                    canvas.draw_text_clipped(px, sy + 5, 34, &slot_lbl, COL_TEXT);
                    if self.draw_button(canvas, px + pw - 130, sy, 58, 20, "Save", self.selected_slot == slot)
                        && clicked
                    {
                        self.save_state_slot(slot, cpu, bus);
                    }
                    if self.draw_button(canvas, px + pw - 66, sy, 58, 20, "Load", false) && clicked {
                        self.load_state_slot(slot, cpu, bus);
                    }
                }
            }

            SettingsTab::Bios => {
                canvas.draw_text(px, py, "PlayStation BIOS Configuration", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                canvas.draw_text(px, py + 28, "Current BIOS ROM Path:", COL_TEXT);
                canvas.draw_panel(px, py + 44, pw, 26, COL_BG_INPUT, COL_BORDER);
                let bpath_str = self
                    .bios_path
                    .as_deref()
                    .unwrap_or("<No BIOS Selected - Click Browse to pick SCPH1001.BIN>");
                canvas.draw_text_clipped(px + 8, py + 53, ((pw - 16) / 8) as usize, bpath_str, COL_GREEN);

                if self.draw_button(canvas, px, py + 80, 145, 24, "Browse BIOS ROM...", true) && clicked {
                    if let Some(path) = open_file_dialog(
                        "Select PlayStation BIOS ROM (512KB SCPH1001.BIN)",
                        b"BIOS ROM (*.bin;*.rom)\0*.bin;*.rom\0All Files (*.*)\0*.*\0\0",
                    ) {
                        self.bios_path = Some(path.clone());
                        let _ = bus.load_bios(&path);
                        self.push_osd(format!("Loaded BIOS ROM: {}", path));
                    }
                }
                if self.draw_button(canvas, px + 155, py + 80, 130, 24, "Boot BIOS Now", false) && clicked {
                    self.settings_open = false;
                    self.boot_bios_only(cpu, bus);
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 120,
                    "Fast Boot (Side-load PS-EXE directly into RAM)",
                    self.fast_boot,
                ) && clicked
                {
                    self.fast_boot = !self.fast_boot;
                }

                canvas.draw_text(px, py + 158, "Notes:", COL_TEXT_ACCENT);
                canvas.draw_text(px, py + 176, "- Official 512KB PS1 BIOS images (SCPH1001.BIN, SCPH5501.BIN,", COL_TEXT_DIM);
                canvas.draw_text(px, py + 192, "  SCPH7502.BIN) are auto-detected if placed in the workspace.", COL_TEXT_DIM);
                canvas.draw_text(px, py + 208, "- PS-X EXE homebrew/tests can boot with or without a BIOS.", COL_TEXT_DIM);
            }

            SettingsTab::Display => {
                canvas.draw_text(px, py, "Display & Viewport Settings", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                canvas.draw_text(px, py + 24, "Aspect Ratio:", COL_TEXT);
                let modes = [
                    AspectRatioMode::Auto43,
                    AspectRatioMode::Force169,
                    AspectRatioMode::Pixel1To1,
                    AspectRatioMode::Stretch,
                ];
                for (i, &m) in modes.iter().enumerate() {
                    let col = (i % 2) as i32;
                    let row = (i / 2) as i32;
                    let bx = px + col * 218;
                    let by = py + 40 + row * 26;
                    if self.draw_button(canvas, bx, by, 210, 22, m.label(), self.aspect_ratio == m) && clicked {
                        self.aspect_ratio = m;
                    }
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 102,
                    "Crop 8px Overscan Borders (Hide Top/Bottom/Side Garbage)",
                    self.crop_overscan,
                ) && clicked
                {
                    self.crop_overscan = !self.crop_overscan;
                }
                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 126,
                    "Integer Upscaling Only (Snap Viewport to Exact Pixel Multiples)",
                    self.integer_upscaling,
                ) && clicked
                {
                    self.integer_upscaling = !self.integer_upscaling;
                }
                if self.draw_checkbox(canvas, px, py + 150, "Show Top Toolbar", self.show_toolbar) && clicked {
                    self.show_toolbar = !self.show_toolbar;
                }
                if self.draw_checkbox(canvas, px, py + 174, "Show Bottom Status Bar", self.show_status_bar) && clicked
                {
                    self.show_status_bar = !self.show_status_bar;
                }
                if self.draw_checkbox(canvas, px, py + 198, "Show OSD Notifications", self.show_osd) && clicked {
                    self.show_osd = !self.show_osd;
                }
                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 222,
                    "Show FPS / Speed / SPU Voice Overlay",
                    self.show_fps_overlay,
                ) && clicked
                {
                    self.show_fps_overlay = !self.show_fps_overlay;
                }
                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 246,
                    "Display Full 1024x512 VRAM Instead of Cropped Viewport",
                    self.full_vram_display,
                ) && clicked
                {
                    self.full_vram_display = !self.full_vram_display;
                }
            }

            SettingsTab::Enhancements => {
                canvas.draw_text(px, py, "DuckStation Graphics & Hardware Enhancements", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                canvas.draw_text(px, py + 22, "Texture & Upscaling Filter (F11):", COL_TEXT);
                let filters = [
                    TextureFilterMode::Nearest,
                    TextureFilterMode::BilinearViewport,
                    TextureFilterMode::Bilinear3DAndViewport,
                    TextureFilterMode::XbrzSmooth,
                    TextureFilterMode::Jinc2Sharp,
                ];
                for (i, &flt) in filters.iter().enumerate() {
                    let col = (i % 2) as i32;
                    let row = (i / 2) as i32;
                    let bx = px + col * 242;
                    let by = py + 36 + row * 24;
                    if self.draw_button(canvas, bx, by, 234, 21, flt.label(), self.texture_filter == flt) && clicked {
                        self.texture_filter = flt;
                        self.bilinear_filtering = matches!(
                            flt,
                            TextureFilterMode::BilinearViewport | TextureFilterMode::Bilinear3DAndViewport
                        );
                        self.push_osd(format!("Texture Filter: {}", flt.label()));
                    }
                }

                let mut cy = py + 116;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "True Color 24-bit Output (15-bit to 24-bit Bit-Replication)",
                    self.color_dithering,
                ) && clicked
                {
                    self.color_dithering = !self.color_dithering;
                }
                cy += 23;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "PS1 Hardware 4x4 Bayer Dithering on Gouraud Polygons",
                    self.gpu_hardware_dither,
                ) && clicked
                {
                    self.gpu_hardware_dither = !self.gpu_hardware_dither;
                }
                cy += 23;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "Widescreen 16:9 GTE 3D Projection Hack (No 3D Stretching)",
                    self.widescreen_gte_hack,
                ) && clicked
                {
                    self.widescreen_gte_hack = !self.widescreen_gte_hack;
                    if self.widescreen_gte_hack {
                        self.aspect_ratio = AspectRatioMode::Force169;
                    }
                    self.push_osd(format!(
                        "Widescreen GTE 3D Hack: {}",
                        if self.widescreen_gte_hack { "ON" } else { "OFF" }
                    ));
                }
                cy += 23;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "PGXP GTE NCLIP Winding Culling Correction (Reduce Seams)",
                    self.pgxp_culling_correction,
                ) && clicked
                {
                    self.pgxp_culling_correction = !self.pgxp_culling_correction;
                }
                cy += 23;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "Fast CD-ROM Speedup (16x Sector Read / Eliminate Load Times)",
                    self.cd_fast_load,
                ) && clicked
                {
                    self.cd_fast_load = !self.cd_fast_load;
                    self.push_osd(format!(
                        "16x Fast CD-ROM Loading: {}",
                        if self.cd_fast_load { "ON" } else { "OFF" }
                    ));
                }
                cy += 23;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "480i Progressive Deinterlacing (Blend Interlaced Scanlines)",
                    self.deinterlace_480i,
                ) && clicked
                {
                    self.deinterlace_480i = !self.deinterlace_480i;
                }
                cy += 23;
                if self.draw_checkbox(
                    canvas,
                    px,
                    cy,
                    "2MB VRAM Expansion Mode (GPUSTAT Bit 14)",
                    bus.vram_2mb_mode,
                ) && clicked
                {
                    bus.vram_2mb_mode = !bus.vram_2mb_mode;
                }
            }

            SettingsTab::Shaders => {
                canvas.draw_text(px, py, "DuckStation Post-Processing Shaders & Color Grading", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                canvas.draw_text(px, py + 22, "Active Post-Processing Shader Preset (Hotkey F9):", COL_TEXT);
                let presets = [
                    PostShaderPreset::None,
                    PostShaderPreset::CrtTrinitron,
                    PostShaderPreset::CrtLottesArcade,
                    PostShaderPreset::LcdGrid,
                    PostShaderPreset::VhsComposite,
                    PostShaderPreset::CasSharpen,
                    PostShaderPreset::KawaseBloom,
                    PostShaderPreset::MonochromeNoir,
                ];
                for (i, &shd) in presets.iter().enumerate() {
                    let col = (i % 2) as i32;
                    let row = (i / 2) as i32;
                    let bx = px + col * 242;
                    let by = py + 36 + row * 25;
                    if self.draw_button(canvas, bx, by, 234, 22, shd.label(), self.post_shader == shd) && clicked {
                        self.post_shader = shd;
                        self.push_osd(format!("Shader Preset: {}", shd.label()));
                    }
                }

                let sy0 = py + 146;
                canvas.draw_text(px, sy0, "Scanline Intensity:", COL_TEXT);
                let scan_levels = [(0u8, "Off"), (25, "25%"), (50, "50%"), (75, "75%")];
                for (i, &(lvl, lbl)) in scan_levels.iter().enumerate() {
                    let bx = px + 168 + (i as i32) * 66;
                    let active = self.scanline_strength == lvl;
                    if self.draw_button(canvas, bx, sy0 - 4, 60, 20, lbl, active) && clicked {
                        self.scanline_strength = lvl;
                        self.scanlines = lvl > 0;
                    }
                }

                let sy1 = py + 174;
                canvas.draw_text(px, sy1, "Color Vibrance:", COL_TEXT);
                let vib_levels = [(100u32, "100% (Std)"), (120, "120% (Vivid)"), (145, "145% (Lush)")];
                for (i, &(vib, lbl)) in vib_levels.iter().enumerate() {
                    let bx = px + 168 + (i as i32) * 106;
                    if self.draw_button(canvas, bx, sy1 - 4, 100, 20, lbl, self.color_vibrance_pct == vib) && clicked {
                        self.color_vibrance_pct = vib;
                    }
                }

                let sy2 = py + 202;
                canvas.draw_text(px, sy2, "Gamma / Brightness:", COL_TEXT);
                let brt_levels = [(-16i32, "-16 Dark"), (0, "0 Default"), (16, "+16 Bright"), (32, "+32 Max")];
                for (i, &(brt, lbl)) in brt_levels.iter().enumerate() {
                    let bx = px + 168 + (i as i32) * 80;
                    if self.draw_button(canvas, bx, sy2 - 4, 74, 20, lbl, self.brightness_offset == brt) && clicked {
                        self.brightness_offset = brt;
                    }
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 232,
                    "CRT Curved Bezel Corner Vignette Shading",
                    self.crt_vignette,
                ) && clicked
                {
                    self.crt_vignette = !self.crt_vignette;
                }
            }

            SettingsTab::Audio => {
                canvas.draw_text(px, py, "SPU 24-Voice & CD-XA Audio Settings", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 28,
                    "Enable 44.1 kHz Stereo WaveOut Audio Output",
                    self.audio_enabled,
                ) && clicked
                {
                    self.audio_enabled = !self.audio_enabled;
                }

                canvas.draw_text(px, py + 60, "Master Volume:", COL_TEXT);
                let vols = [0i32, 25, 50, 75, 100];
                for (i, &v) in vols.iter().enumerate() {
                    let bx = px + (i as i32) * 66;
                    let lbl = format!("{}%", v);
                    if self.draw_button(canvas, bx, py + 76, 60, 22, &lbl, self.master_volume == v) && clicked {
                        self.master_volume = v;
                    }
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 114,
                    "Enable Hardware SPU Reverb Engine (39-Tap FIR + APF1/APF2)",
                    self.reverb_enabled,
                ) && clicked
                {
                    self.reverb_enabled = !self.reverb_enabled;
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 142,
                    "Enable CD-XA ADPCM & CD-DA Redbook Audio Streaming",
                    self.cd_audio_enabled,
                ) && clicked
                {
                    self.cd_audio_enabled = !self.cd_audio_enabled;
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 170,
                    "Mute Audio Output During Fast-Forward (Turbo)",
                    self.mute_in_fast_forward,
                ) && clicked
                {
                    self.mute_in_fast_forward = !self.mute_in_fast_forward;
                }

                if self.draw_button(canvas, px, py + 206, 210, 24, "Open SPU Voice Inspector...", true) && clicked {
                    self.settings_open = false;
                    self.tool_window = ActiveToolWindow::SpuInspector;
                }
            }

            SettingsTab::Controllers => {
                canvas.draw_text(px, py, "Port 1 Controller & Host Gamepad Settings", COL_TEXT_ACCENT);
                canvas.fill_rect(px, py + 14, pw, 1, COL_BORDER);

                canvas.draw_text(px, py + 26, "Controller Type (Port 1):", COL_TEXT);
                let is_analog = bus.pad1.analog_mode;
                if self.draw_button(
                    canvas,
                    px,
                    py + 42,
                    185,
                    24,
                    "DualShock Analog (0x73)",
                    is_analog,
                ) && clicked
                {
                    bus.pad1.analog_mode = true;
                }
                if self.draw_button(
                    canvas,
                    px + 195,
                    py + 42,
                    175,
                    24,
                    "Digital Pad (0x41)",
                    !is_analog,
                ) && clicked
                {
                    bus.pad1.analog_mode = false;
                }

                if self.draw_checkbox(
                    canvas,
                    px,
                    py + 76,
                    "Enable DualShock Force-Feedback Rumble Pass-Through (XInput)",
                    self.rumble_enabled,
                ) && clicked
                {
                    self.rumble_enabled = !self.rumble_enabled;
                }

                // Live Thumbstick & Button Visualizer
                canvas.draw_text(px, py + 106, "Live DualShock Thumbstick & Button Monitor:", COL_TEXT_ACCENT);
                let stick_box = 76;
                // Left stick
                canvas.draw_panel(px, py + 122, stick_box, stick_box, COL_BG_INPUT, COL_BORDER);
                canvas.draw_line(px + stick_box / 2, py + 122, px + stick_box / 2, py + 122 + stick_box, COL_BORDER);
                canvas.draw_line(px, py + 122 + stick_box / 2, px + stick_box, py + 122 + stick_box / 2, COL_BORDER);
                let lx_dot = px + ((self.last_lx as i32) * (stick_box - 8)) / 255;
                let ly_dot = py + 122 + ((self.last_ly as i32) * (stick_box - 8)) / 255;
                canvas.fill_rect(lx_dot, ly_dot, 8, 8, COL_GREEN);
                canvas.draw_text(px, py + 202, &format!("LX:{:02X} LY:{:02X}", self.last_lx, self.last_ly), COL_TEXT_DIM);

                // Right stick
                let rx_base = px + 104;
                canvas.draw_panel(rx_base, py + 122, stick_box, stick_box, COL_BG_INPUT, COL_BORDER);
                canvas.draw_line(rx_base + stick_box / 2, py + 122, rx_base + stick_box / 2, py + 122 + stick_box, COL_BORDER);
                canvas.draw_line(rx_base, py + 122 + stick_box / 2, rx_base + stick_box, py + 122 + stick_box / 2, COL_BORDER);
                let rx_dot = rx_base + ((self.last_rx as i32) * (stick_box - 8)) / 255;
                let ry_dot = py + 122 + ((self.last_ry as i32) * (stick_box - 8)) / 255;
                canvas.fill_rect(rx_dot, ry_dot, 8, 8, COL_ACCENT);
                canvas.draw_text(rx_base, py + 202, &format!("RX:{:02X} RY:{:02X}", self.last_rx, self.last_ry), COL_TEXT_DIM);

                // Keyboard & XInput Mapping Reference
                let map_x = px + 204;
                canvas.draw_text(map_x, py + 122, "Bindings (XInput + Keyboard):", COL_TEXT);
                canvas.draw_text(map_x, py + 138, "D-Pad/L-Stick: Arrows / WASD", COL_TEXT_DIM);
                canvas.draw_text(map_x, py + 152, "R-Stick:       T / F / G / H", COL_TEXT_DIM);
                canvas.draw_text(map_x, py + 166, "Cross / Cir:   Z,K / X,L", COL_TEXT_DIM);
                canvas.draw_text(map_x, py + 180, "Sq / Tri:      C,J / V,I", COL_TEXT_DIM);
                canvas.draw_text(map_x, py + 194, "L1/R1 L2/R2:   Q/E  1/2", COL_TEXT_DIM);
                canvas.draw_text(map_x, py + 208, "Start/Select:  Enter / RShift", COL_TEXT_DIM);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_tool_window(
        &mut self,
        canvas: &mut Canvas,
        win_w: i32,
        win_h: i32,
        clicked: bool,
        scroll_y: f32,
        cpu: &mut Cpu,
        bus: &mut Bus,
    ) {
        let tw = (win_w - 40).clamp(580, 860);
        let th = (win_h - 70).clamp(380, 540);
        let tx = (win_w - tw) / 2;
        let ty = (win_h - th) / 2;

        canvas.fill_rect_alpha(0, UI_MENU_HEIGHT as i32, win_w, win_h - UI_MENU_HEIGHT as i32, 0x000000, 120);
        canvas.draw_panel(tx, ty, tw, th, COL_BG_PANEL, COL_ACCENT);

        let title = match self.tool_window {
            ActiveToolWindow::VramViewer => "DuckStation Tool: Full 1024x512 VRAM Inspector",
            ActiveToolWindow::CpuDebugger => "DuckStation Tool: MIPS R3000A CPU & GTE Debugger",
            ActiveToolWindow::SpuInspector => "DuckStation Tool: 24-Voice SPU & Real-Time Audio Oscilloscope",
            ActiveToolWindow::MemoryTtyViewer => "DuckStation Tool: Memory Hex Inspector & BIOS TTY Console",
            ActiveToolWindow::None => return,
        };

        canvas.fill_rect(tx + 1, ty + 1, tw - 2, 26, COL_BG_ELEVATED);
        canvas.draw_text(tx + 12, ty + 9, title, COL_TEXT_ACCENT);
        if self.draw_button(canvas, tx + tw - 66, ty + 3, 60, 20, "Close", true) && clicked {
            self.tool_window = ActiveToolWindow::None;
            return;
        }

        let cx = tx + 14;
        let cy = ty + 36;
        let cw = tw - 28;
        let ch = th - 46;

        match self.tool_window {
            ActiveToolWindow::VramViewer => {
                // Draw 1024x512 VRAM scaled to 512x256
                let vw = 512.min(cw);
                let vh = 256.min(ch - 60);
                for py in 0..vh {
                    let sy = ((py as usize) * 512) / (vh as usize);
                    let fb_row = ((cy + py) as usize) * canvas.width;
                    let vram_row = sy * 1024;
                    for px in 0..vw {
                        let sx = ((px as usize) * 1024) / (vw as usize);
                        let p15 = bus.vram[vram_row + sx];
                        let r = ((p15 & 0x1F) << 3) as u32;
                        let g = (((p15 >> 5) & 0x1F) << 3) as u32;
                        let b = (((p15 >> 10) & 0x1F) << 3) as u32;
                        canvas.fb[fb_row + (cx + px) as usize] = (r << 16) | (g << 8) | b;
                    }
                }
                canvas.draw_border(cx, cy, vw, vh, COL_ACCENT);

                // Highlight active display area in green
                let dx = cx + (bus.display_vram_x as i32 * vw) / 1024;
                let dy = cy + (bus.display_vram_y as i32 * vh) / 512;
                let dw = ((bus.display_width.max(320) as i32) * vw) / 1024;
                let dh = ((bus.display_height.max(240) as i32) * vh) / 512;
                canvas.draw_border(dx, dy, dw.min(vw), dh.min(vh), COL_GREEN);

                // Info panel on right or bottom
                let info_y = cy + vh + 10;
                let hover_vram = if rect_contains(self.mouse_x, self.mouse_y, cx, cy, vw, vh) {
                    let hx = ((self.mouse_x - cx) * 1024) / vw;
                    let hy = ((self.mouse_y - cy) * 512) / vh;
                    let word = bus.vram[(hy as usize) * 1024 + (hx as usize)];
                    format!("Cursor VRAM: ({}, {}) = 0x{:04X}", hx, hy, word)
                } else {
                    "Hover over VRAM image to inspect pixel coordinates & 15-bit BGR word".to_string()
                };
                canvas.draw_text(cx, info_y, &hover_vram, COL_TEXT);
                let gpustat_info = format!(
                    "GPUSTAT: 0x{:08X} | Display: {}x{} @ ({},{}) | DrawArea: ({},{})..({},{})",
                    bus.gpustat,
                    bus.display_width,
                    bus.display_height,
                    bus.display_vram_x,
                    bus.display_vram_y,
                    bus.draw_area_left,
                    bus.draw_area_top,
                    bus.draw_area_right,
                    bus.draw_area_bottom
                );
                canvas.draw_text_clipped(cx, info_y + 18, (cw / 8) as usize, &gpustat_info, COL_TEXT_DIM);

                if self.draw_button(canvas, cx, info_y + 36, 175, 22, "Save VRAM to .PPM", true) && clicked {
                    let _ = bus.dump_vram_ppm("vram_screenshot.ppm");
                    self.push_osd("Saved 1024x512 VRAM to vram_screenshot.ppm");
                }
            }

            ActiveToolWindow::CpuDebugger => {
                // Top control strip: Run/Pause, Step 1 Instruction, Step Frame, Reset
                let pause_lbl = if self.paused { "Resume (F6)" } else { "Pause (F6)" };
                if self.draw_button(canvas, cx, cy, 104, 22, pause_lbl, self.paused) && clicked {
                    self.paused = !self.paused;
                }
                if self.draw_button(canvas, cx + 112, cy, 118, 22, "Step 1 Instr", false) && clicked {
                    self.paused = true;
                    self.step_one_instruction = true;
                }
                if self.draw_button(canvas, cx + 238, cy, 110, 22, "Step Frame", false) && clicked {
                    self.paused = true;
                    self.step_one_frame = true;
                }
                canvas.draw_text(
                    cx + 360,
                    cy + 7,
                    &format!("PC: 0x{:08X} | Cycles: {}", cpu.pc, cpu.cycles),
                    COL_GREEN,
                );

                // Left Column: Disassembly around PC (-6 to +10 instructions)
                let dis_y = cy + 30;
                let dis_w = 320;
                let dis_h = ch - 34;
                canvas.draw_panel(cx, dis_y, dis_w, dis_h, COL_BG_INPUT, COL_BORDER);
                canvas.draw_text(cx + 8, dis_y + 6, "MIPS R3000A Disassembly", COL_TEXT_ACCENT);

                for idx in -6i32..=12i32 {
                    let addr = cpu.pc.wrapping_add((idx * 4) as u32);
                    let row_y = dis_y + 22 + ((idx + 6) * 16);
                    if row_y + 14 > dis_y + dis_h {
                        break;
                    }
                    let word = Self::peek_u32_safe(bus, addr);
                    let asm = disassemble_mips(word, addr);
                    if idx == 0 {
                        canvas.fill_rect(cx + 2, row_y - 2, dis_w - 4, 15, COL_ACCENT_DIM);
                    }
                    let line = format!("{:08X}: {:08X}  {}", addr, word, asm);
                    canvas.draw_text_clipped(
                        cx + 6,
                        row_y,
                        ((dis_w - 12) / 8) as usize,
                        &line,
                        if idx == 0 { COL_YELLOW } else { COL_TEXT },
                    );
                }

                // Right Column: 32 GPRs + HI/LO + COP0 Status/Cause/EPC
                let reg_x = cx + dis_w + 10;
                let reg_w = cw - dis_w - 10;
                canvas.draw_panel(reg_x, dis_y, reg_w, dis_h, COL_BG_INPUT, COL_BORDER);
                canvas.draw_text(reg_x + 8, dis_y + 6, "GPR / COP0 / GTE Registers", COL_TEXT_ACCENT);

                for r in 0..16 {
                    let ry = dis_y + 24 + (r as i32) * 15;
                    if ry + 12 > dis_y + dis_h {
                        break;
                    }
                    let lhs = format!("{:<7}:{:08X}", GPR_NAMES[r], cpu.gpr[r]);
                    let rhs = format!("{:<7}:{:08X}", GPR_NAMES[r + 16], cpu.gpr[r + 16]);
                    canvas.draw_text(reg_x + 8, ry, &lhs, COL_TEXT);
                    if reg_w > 260 {
                        canvas.draw_text(reg_x + 148, ry, &rhs, COL_TEXT);
                    }
                }
                let bot_ry = dis_y + 24 + 16 * 15 + 4;
                if bot_ry + 30 < dis_y + dis_h {
                    let hilo = format!("HI:{:08X} LO:{:08X} SR:{:08X}", cpu.hi, cpu.lo, cpu.cop0[12]);
                    let cop0 = format!("CAUSE:{:08X} EPC:{:08X} I_STAT:{:04X}", cpu.cop0[13], cpu.cop0[14], bus.i_stat);
                    canvas.draw_text_clipped(reg_x + 8, bot_ry, ((reg_w - 16) / 8) as usize, &hilo, COL_TEXT_ACCENT);
                    canvas.draw_text_clipped(reg_x + 8, bot_ry + 16, ((reg_w - 16) / 8) as usize, &cop0, COL_TEXT_DIM);
                }
            }

            ActiveToolWindow::SpuInspector => {
                // Top: Real-Time Stereo Waveform Oscilloscope
                let scope_h = 86;
                canvas.draw_panel(cx, cy, cw, scope_h, COL_BG_INPUT, COL_BORDER);
                canvas.draw_text(cx + 8, cy + 6, "Real-Time 44.1 kHz Stereo Output Oscilloscope (L=Green, R=Blue)", COL_TEXT_ACCENT);
                let mid_y = cy + scope_h / 2 + 6;
                canvas.draw_line(cx + 2, mid_y, cx + cw - 4, mid_y, COL_BORDER);

                let len = self.scope_samples.len();
                if len > 1 {
                    for i in 1..len {
                        let x0 = cx + 4 + ((i as i32 - 1) * (cw - 8)) / 256;
                        let x1 = cx + 4 + ((i as i32) * (cw - 8)) / 256;
                        let (l0, r0) = self.scope_samples[i - 1];
                        let (l1, r1) = self.scope_samples[i];
                        let yl0 = mid_y - ((l0 as i32) * 32) / 32768;
                        let yl1 = mid_y - ((l1 as i32) * 32) / 32768;
                        let yr0 = mid_y - ((r0 as i32) * 32) / 32768;
                        let yr1 = mid_y - ((r1 as i32) * 32) / 32768;
                        canvas.draw_line(x0, yl0, x1, yl1, COL_GREEN);
                        canvas.draw_line(x0, yr0, x1, yr1, COL_ACCENT);
                    }
                }

                // Bottom: 24-Voice State Grid (2 columns of 12 voices)
                let grid_y = cy + scope_h + 8;
                let grid_h = ch - scope_h - 8;
                canvas.draw_panel(cx, grid_y, cw, grid_h, COL_BG_INPUT, COL_BORDER);
                canvas.draw_text(
                    cx + 8,
                    grid_y + 6,
                    &format!(
                        "SPUCNT: 0x{:04X} | SPUSTAT: 0x{:04X} | Reverb Base: 0x{:05X} | ENDX: 0x{:06X}",
                        bus.spu_cnt, bus.spu_stat, bus.spu.reverb_base_addr, bus.spu_endx
                    ),
                    COL_TEXT_ACCENT,
                );

                let col_w = (cw - 16) / 2;
                for v in 0..24 {
                    let col = v / 12;
                    let row = v % 12;
                    let vx = cx + 8 + (col as i32) * col_w;
                    let vy = grid_y + 24 + (row as i32) * 18;
                    if vy + 14 > grid_y + grid_h {
                        continue;
                    }
                    let vs = &bus.spu.voices[v];
                    let phase_str = match vs.adsr_phase {
                        AdsrPhase::Off => "OFF",
                        AdsrPhase::Attack => "ATK",
                        AdsrPhase::Decay => "DEC",
                        AdsrPhase::Sustain => "SUS",
                        AdsrPhase::Release => "REL",
                    };
                    let pitch_reg = bus.spu_voice_regs[v][2];
                    let hz = ((pitch_reg as u32) * 44100) >> 12;
                    let info = format!(
                        "V{:02} {} Env:{:04X} {:5}Hz @{:05X}",
                        v, phase_str, vs.env_level, hz, vs.current_addr
                    );
                    canvas.draw_text(
                        vx,
                        vy,
                        &info,
                        if vs.active { COL_GREEN } else { COL_TEXT_DIM },
                    );
                    // Envelope bar
                    let bar_x = vx + 248;
                    let bar_w = (col_w - 256).max(20);
                    if bar_w > 20 {
                        canvas.fill_rect(bar_x, vy + 1, bar_w, 8, COL_BG_DARK);
                        let fill_w = ((vs.env_level as i32) * bar_w) / 0x7FFF;
                        canvas.fill_rect(bar_x, vy + 1, fill_w, 8, COL_ACCENT);
                    }
                }
            }

            ActiveToolWindow::MemoryTtyViewer => {
                // Region selector buttons
                let regions = [
                    ("Main RAM (2MB)", MemRegion::MainRam),
                    ("Scratchpad (1KB)", MemRegion::Scratchpad),
                    ("BIOS ROM (512KB)", MemRegion::BiosRom),
                    ("SPU RAM (512KB)", MemRegion::SpuRam),
                ];
                for (i, &(rname, rval)) in regions.iter().enumerate() {
                    let bx = cx + (i as i32) * 142;
                    if self.draw_button(canvas, bx, cy, 136, 22, rname, self.mem_region == rval) && clicked {
                        self.mem_region = rval;
                        self.mem_view_offset = 0;
                    }
                }

                let slice: &[u8] = match self.mem_region {
                    MemRegion::MainRam => &bus.ram,
                    MemRegion::Scratchpad => &bus.scratchpad,
                    MemRegion::BiosRom => &bus.bios,
                    MemRegion::SpuRam => &bus.spu_ram,
                };

                if scroll_y < 0.0 {
                    self.mem_view_offset = (self.mem_view_offset + 64).min(slice.len().saturating_sub(256));
                } else if scroll_y > 0.0 {
                    self.mem_view_offset = self.mem_view_offset.saturating_sub(64);
                }

                // Step buttons for offset
                if self.draw_button(canvas, cx + 576, cy, 60, 22, "-0x100", false) && clicked {
                    self.mem_view_offset = self.mem_view_offset.saturating_sub(0x100);
                }
                if self.draw_button(canvas, cx + 642, cy, 60, 22, "+0x100", false) && clicked {
                    self.mem_view_offset = (self.mem_view_offset + 0x100).min(slice.len().saturating_sub(256));
                }

                let hex_y = cy + 28;
                let hex_h = ch - 120;
                canvas.draw_panel(cx, hex_y, cw, hex_h, COL_BG_INPUT, COL_BORDER);

                let rows = ((hex_h - 12) / 15).max(1) as usize;
                for r in 0..rows {
                    let off = self.mem_view_offset + r * 16;
                    if off >= slice.len() {
                        break;
                    }
                    let mut hex_part = String::with_capacity(48);
                    let mut ascii_part = String::with_capacity(16);
                    for b_idx in 0..16 {
                        if off + b_idx < slice.len() {
                            let b = slice[off + b_idx];
                            hex_part.push_str(&format!("{:02X} ", b));
                            ascii_part.push(if (0x20..=0x7E).contains(&b) { b as char } else { '.' });
                        } else {
                            hex_part.push_str("   ");
                        }
                    }
                    let line = format!("{:06X}: {} |{}|", off, hex_part, ascii_part);
                    canvas.draw_text_clipped(
                        cx + 8,
                        hex_y + 6 + (r as i32) * 15,
                        ((cw - 16) / 8) as usize,
                        &line,
                        COL_TEXT,
                    );
                }

                // Bottom: Live BIOS Kernel TTY Output Log
                let tty_y = hex_y + hex_h + 6;
                let tty_h = ch - (hex_h + 34);
                canvas.draw_panel(cx, tty_y, cw, tty_h, COL_BG_INPUT, COL_BORDER);
                canvas.draw_text(cx + 8, tty_y + 5, "BIOS Kernel TTY Console Output:", COL_TEXT_ACCENT);
                let tty_lines: Vec<&str> = bus.tty_buffer.lines().collect();
                let max_tty_rows = ((tty_h - 20) / 14).max(1) as usize;
                let start_line = tty_lines.len().saturating_sub(max_tty_rows);
                for (i, &l) in tty_lines[start_line..].iter().enumerate() {
                    canvas.draw_text_clipped(
                        cx + 8,
                        tty_y + 20 + (i as i32) * 14,
                        ((cw - 16) / 8) as usize,
                        l,
                        COL_GREEN,
                    );
                }
            }

            ActiveToolWindow::None => {}
        }
    }

    fn peek_u32_safe(bus: &Bus, vaddr: u32) -> u32 {
        let paddr = vaddr & 0x1FFF_FFFF;
        if paddr < 0x0080_0000 {
            let off = (paddr as usize) & (bus.ram.len() - 1);
            if off + 4 <= bus.ram.len() {
                return u32::from_le_bytes([
                    bus.ram[off],
                    bus.ram[off + 1],
                    bus.ram[off + 2],
                    bus.ram[off + 3],
                ]);
            }
        } else if (0x1FC0_0000..0x1FC8_0000).contains(&paddr) {
            let off = (paddr - 0x1FC0_0000) as usize;
            if off + 4 <= bus.bios.len() {
                return u32::from_le_bytes([
                    bus.bios[off],
                    bus.bios[off + 1],
                    bus.bios[off + 2],
                    bus.bios[off + 3],
                ]);
            }
        }
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_duckstation_ui_disassembler_save_states_and_rendering() {
        assert_eq!(disassemble_mips(0x0000_0000, 0x8001_0000), "nop");
        assert_eq!(
            disassemble_mips(0x3C08_1F80, 0x8001_0000),
            "lui     r8/t0, 0x1F80"
        );
        assert_eq!(
            disassemble_mips(0x2508_1810, 0x8001_0000),
            "addiu   r8/t0, r8/t0, 6160"
        );

        let mut ui = DuckStationUi::new(None, None, None);
        let mut cpu = Cpu::new();
        let mut bus = Bus::new();
        cpu.pc = 0x8001_2340;
        cpu.gpr[8] = 0xDEAD_BEEF;
        bus.ram[0x100] = 0x42;

        // Save state to Slot 1 (index 0)
        ui.save_state_slot(0, &cpu, &bus);
        cpu.pc = 0x8005_0000;
        cpu.gpr[8] = 0;
        bus.ram[0x100] = 0;

        // Restore state from Slot 1
        ui.load_state_slot(0, &mut cpu, &mut bus);
        assert_eq!(cpu.pc, 0x8001_2340);
        assert_eq!(cpu.gpr[8], 0xDEAD_BEEF);
        assert_eq!(bus.ram[0x100], 0x42);

        // Render Game Library, Settings Modal, and each Tool Window into a 960x640 buffer
        let mut fb = vec![0u32; 960 * 640];
        ui.render_and_interact(&mut fb, 960, 640, 50, 10, false, 0.0, &mut cpu, &mut bus);
        assert!(fb.iter().any(|&px| px != 0));

        ui.settings_open = true;
        for tab in [
            SettingsTab::Console,
            SettingsTab::Bios,
            SettingsTab::Display,
            SettingsTab::Enhancements,
            SettingsTab::Shaders,
            SettingsTab::Audio,
            SettingsTab::Controllers,
        ] {
            ui.settings_tab = tab;
            ui.render_and_interact(&mut fb, 960, 640, 200, 200, false, 0.0, &mut cpu, &mut bus);
        }
        ui.settings_open = false;

        // Verify all DuckStation texture filters and post-processing shader presets render cleanly
        ui.running = true;
        ui.show_game_list = false;
        bus.vram[0] = 0x7FFF;
        bus.vram[1] = 0x03E0;
        for flt in [
            TextureFilterMode::Nearest,
            TextureFilterMode::BilinearViewport,
            TextureFilterMode::Bilinear3DAndViewport,
            TextureFilterMode::XbrzSmooth,
            TextureFilterMode::Jinc2Sharp,
        ] {
            ui.texture_filter = flt;
            ui.render_and_interact(&mut fb, 960, 640, 100, 100, false, 0.0, &mut cpu, &mut bus);
        }
        for shd in [
            PostShaderPreset::CrtTrinitron,
            PostShaderPreset::CrtLottesArcade,
            PostShaderPreset::LcdGrid,
            PostShaderPreset::VhsComposite,
            PostShaderPreset::CasSharpen,
            PostShaderPreset::KawaseBloom,
            PostShaderPreset::MonochromeNoir,
            PostShaderPreset::None,
        ] {
            ui.post_shader = shd;
            ui.render_and_interact(&mut fb, 960, 640, 100, 100, false, 0.0, &mut cpu, &mut bus);
        }
        ui.texture_filter = TextureFilterMode::Nearest;
        ui.post_shader = PostShaderPreset::None;

        for tool in [
            ActiveToolWindow::VramViewer,
            ActiveToolWindow::CpuDebugger,
            ActiveToolWindow::SpuInspector,
            ActiveToolWindow::MemoryTtyViewer,
        ] {
            ui.tool_window = tool;
            ui.render_and_interact(&mut fb, 960, 640, 300, 300, false, 0.0, &mut cpu, &mut bus);
        }

        // Benchmark 60 emulated NTSC frames (33.8688M cycles + SPU + Gamepad + 960x640 UI viewport render)
        ui.tool_window = ActiveToolWindow::None;
        ui.running = true;
        ui.show_game_list = false;
        let mut poller = crate::gamepad::host_gamepad::GamepadPoller::new();
        // Write a small loop in RAM at 0x8001_0000: addiu r8, r8, 1; b -2; nop
        bus.write32(0x8001_0000, 0x2508_0001, false);
        bus.write32(0x8001_0004, 0x1000_FFFE, false);
        bus.write32(0x8001_0008, 0x0000_0000, false);
        cpu.pc = 0x8001_0000;
        cpu.next_pc = 0x8001_0004;
        bus.spu_cnt = 0xC000; // Enable SPU
        let start = Instant::now();
        let mut audio_buf = Vec::with_capacity(2048);
        for _ in 0..60 {
            let _ = poller.poll(0, 0);
            let target = cpu.cycles + (CYCLES_PER_FRAME as u64);
            while cpu.cycles < target {
                cpu.step_batch(&mut bus, 64);
            }
            bus.drain_audio_samples_into(&mut audio_buf);
            ui.render_and_interact(&mut fb, 960, 640, 100, 100, false, 0.0, &mut cpu, &mut bus);
        }
        let elapsed = start.elapsed();
        println!(
            "[Perf Benchmark] 60 full PS1 frames (34.08M CPU cycles + SPU + Gamepad + 960x640 UI) took {:.1} ms ({:.1}% real-time speed / {:.0} FPS max)",
            elapsed.as_secs_f64() * 1000.0,
            (1.0 / elapsed.as_secs_f64()) * 100.0,
            60.0 / elapsed.as_secs_f64()
        );
    }
}
