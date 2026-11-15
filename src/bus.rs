//! PS1 Physical & Virtual Memory Bus (T-02, T-05, T-06, T-07, T-08, T-10, T-11, T-12 Complete)
//! Handles KUSEG/KSEG0/KSEG1/KSEG2 segment masking, KUSEG > 512MB Bus Error detection,
//! 2MB RAM (with 4x mirroring), 1KB Scratchpad (KUSEG/KSEG0 only, forbidden in uncached KSEG1),
//! 512KB BIOS ROM, COP0 Cache Isolation guard, Interrupt Controller (I_STAT/I_MASK),
//! 7-Channel DMA Controller (SyncMode 0/1/2, DMA0/1 MDEC, DMA2 GPU, DMA3 CD-ROM, DMA4 SPU, DMA6 OTC, DPCR, DICR Master IRQ),
//! 3 Hardware Root Counters (Timers 0, 1, 2), periodic VBlank IRQ0, Expansion 2 DUART TTY,
//! Full GPU Core (GP0/GP1 State Machine, 16-Word FIFO, 1MB VRAM, and VRAM Transfers),
//! Deterministic 2D Software Rasterizer (Polygons, Lines, Rectangles, CLUT, Semi-Transparency),
//! PS-X EXE Header Parser & RAM Payload Side-Loader, SIO0 Digital Joypad & Memory Card Serial Interface,
//! CD-ROM Controller State Machine, Banked Registers, FIFOs, Sub-CPU Commands, & BIN/ISO Sector Reader,
//! 24-Channel ADPCM Sound Processing Unit (SPU, 512KB Sound RAM, SPUCNT/SPUSTAT, IRQ9, DMA4, ADPCM Decoder),
//! and Macroblock Decoder (MDEC, MDEC0/MDEC1, RLE/ZigZag/IDCT/YCbCr-to-RGB, DMA0/DMA1).

use crate::gamepad::DualShockController;
use crate::spu::{
    evaluate_volume_reg, tick_voice_adsr, AdsrPhase, SpuSynthesizer, GAUSS_TABLE, REVERB_FIR,
};
use std::collections::VecDeque;
use std::fs;
use std::path::Path;

pub const RAM_SIZE: usize = 2 * 1024 * 1024;
pub const SCRATCHPAD_SIZE: usize = 1024;
pub const BIOS_SIZE: usize = 512 * 1024;
pub const VRAM_WIDTH: usize = 1024;
pub const VRAM_HEIGHT: usize = 512;
pub const GP0_FIFO_CAPACITY: usize = 16;

pub const CPU_CLOCK_HZ: u32 = 33_868_800;
pub const SCANLINES_PER_FRAME: u32 = 263;
pub const CYCLES_PER_SCANLINE: u32 = 2160; // (33_868_800 / (59.6 * 263) ≈ 2160)
pub const CYCLES_PER_FRAME: u32 = SCANLINES_PER_FRAME * CYCLES_PER_SCANLINE; // 568,080 cycles
pub const DMA2_LINKED_LIST_SAFETY_CAP: usize = 0x10000;
pub const PS_EXE_HEADER_SIZE: usize = 0x800;
pub const SIO0_ACK_DELAY_CYCLES: u32 = 256;
pub const CD_FIFO_CAPACITY: usize = 16;
pub const CD_INT3_DELAY_CYCLES: u32 = 512;
pub const CD_INT2_DELAY_CYCLES: u32 = 2048;
pub const CD_SECTOR_SIZE_COOKED: usize = 2048;
pub const CD_SECTOR_SIZE_RAW_DATA: usize = 2340;
pub const CD_SECTOR_SIZE_RAW_FULL: usize = 2352;
pub const SPU_RAM_SIZE: usize = 512 * 1024;
pub const SPU_VOICE_COUNT: usize = 24;

pub const MDEC_ZIGZAG: [usize; 64] = [
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
];

pub const MDEC_DEFAULT_SCALE_TABLE: [i16; 64] = [
    23170, 23170, 23170, 23170, 23170, 23170, 23170, 23170,
    32138, 27245, 18204, 6392, -6393, -18205, -27246, -32139,
    30273, 12539, -12540, -30274, -30274, -12540, 12539, 30273,
    27245, -6393, -32139, -18205, 18204, 32138, 6392, -27246,
    23170, -23171, -23171, 23170, 23170, -23171, -23171, 23170,
    18204, -32139, 6392, 27245, -27246, -6393, 32138, -18205,
    12539, -30274, 30273, -12540, -12540, 30273, -30274, 12539,
    6392, -18205, 27245, -32139, 32138, -27246, 18204, -6393,
];

pub fn decode_spu_adpcm_block(block: &[u8; 16], prev1: &mut i32, prev2: &mut i32) -> [i16; 28] {
    const POS_F: [i32; 5] = [0, 60, 115, 98, 122];
    const NEG_F: [i32; 5] = [0, 0, -52, -55, -60];

    let shift_filter = block[0];
    let raw_shift = shift_filter & 0x0F;
    let shift = if raw_shift > 12 { 9 } else { raw_shift };
    let filter_idx = ((shift_filter >> 4) & 0x07).min(4) as usize;
    let f0 = POS_F[filter_idx];
    let f1 = NEG_F[filter_idx];

    let mut samples = [0i16; 28];
    for i in 0..28 {
        let byte = block[2 + (i / 2)];
        let nibble = if (i & 1) == 0 { byte & 0x0F } else { (byte >> 4) & 0x0F };
        let s16 = (((nibble as i16) << 12) >> shift) as i32;
        let predicted = s16 + ((*prev1 * f0 + *prev2 * f1 + 32) >> 6);
        let clamped = predicted.clamp(-32768, 32767) as i16;
        samples[i] = clamped;
        *prev2 = *prev1;
        *prev1 = clamped as i32;
    }
    samples
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CdPendingIrq {
    pub irq_type: u8,
    pub response: Vec<u8>,
    pub delay: u32,
    pub is_first_stage: bool,
    pub is_read_sector: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PsExe {
    pub initial_pc: u32,
    pub initial_gp: u32,
    pub load_addr: u32,
    pub load_size: u32,
    pub sp_base: u32,
    pub sp_offset: u32,
    pub initial_sp: u32,
    pub payload: Vec<u8>,
}

impl PsExe {
    pub fn from_bytes(exe_bytes: &[u8]) -> Result<Self, String> {
        if exe_bytes.len() < PS_EXE_HEADER_SIZE {
            return Err(format!(
                "Invalid PS-X EXE size: expected at least {} bytes, got {}",
                PS_EXE_HEADER_SIZE,
                exe_bytes.len()
            ));
        }
        if &exe_bytes[0x000..0x008] != b"PS-X EXE" {
            return Err("Invalid PS-X EXE magic: expected ASCII \"PS-X EXE\" at 0x000..0x008".to_string());
        }
        let initial_pc = u32::from_le_bytes([exe_bytes[0x010], exe_bytes[0x011], exe_bytes[0x012], exe_bytes[0x013]]);
        let initial_gp = u32::from_le_bytes([exe_bytes[0x014], exe_bytes[0x015], exe_bytes[0x016], exe_bytes[0x017]]);
        let load_addr = u32::from_le_bytes([exe_bytes[0x018], exe_bytes[0x019], exe_bytes[0x01A], exe_bytes[0x01B]]);
        let load_size = u32::from_le_bytes([exe_bytes[0x01C], exe_bytes[0x01D], exe_bytes[0x01E], exe_bytes[0x01F]]);
        let sp_base = u32::from_le_bytes([exe_bytes[0x030], exe_bytes[0x031], exe_bytes[0x032], exe_bytes[0x033]]);
        let sp_offset = u32::from_le_bytes([exe_bytes[0x034], exe_bytes[0x035], exe_bytes[0x036], exe_bytes[0x037]]);
        let initial_sp = sp_base.wrapping_add(sp_offset);

        let raw_payload = &exe_bytes[PS_EXE_HEADER_SIZE..];
        let copy_len = if load_size > 0 {
            (load_size as usize).min(raw_payload.len())
        } else {
            raw_payload.len()
        };
        let payload = raw_payload[..copy_len].to_vec();

        Ok(Self {
            initial_pc,
            initial_gp,
            load_addr,
            load_size,
            sp_base,
            sp_offset,
            initial_sp,
            payload,
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DmaChannel {
    pub madr: u32,
    pub bcr: u32,
    pub chcr: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timer {
    pub counter: u16,
    pub mode: u16,
    pub target: u16,
    pub irq_fired: bool,
    pub sub_cycles: u32,
}

impl Default for Timer {
    fn default() -> Self {
        Self {
            counter: 0,
            mode: 0x0400,
            target: 0,
            irq_fired: false,
            sub_cycles: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CpuToVramTransfer {
    pub dst_x: u16,
    pub dst_y: u16,
    pub width: u32,
    pub height: u32,
    pub cur_x: u32,
    pub cur_y: u32,
    pub words_remaining: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gp0Mode {
    Command,
    CpuToVram(CpuToVramTransfer),
    PolyLine { shaded: bool },
}

impl Default for Gp0Mode {
    fn default() -> Self {
        Gp0Mode::Command
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PolyVertex {
    pub x: i32,
    pub y: i32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub u: u8,
    pub v: u8,
}

#[derive(Clone)]
pub struct Bus {
    pub ram: Vec<u8>,
    pub scratchpad: Vec<u8>,
    pub bios: Vec<u8>,
    pub vram: Vec<u16>,
    pub i_stat: u32,
    pub i_mask: u32,
    pub dma: [DmaChannel; 7],
    pub dpcr: u32,
    pub dicr: u32,
    pub timers: [Timer; 3],
    pub hblank_accumulator: u32,
    pub vblank_accumulator: u32,
    pub vblank_triggered: bool,
    pub gp0_command_log: Vec<u32>,
    pub gp0_fifo: Vec<u32>,
    pub gp0_words_remaining: usize,
    pub gp0_mode: Gp0Mode,
    pub vram_read_queue: VecDeque<u32>,
    pub gpuread_latch: u32,
    pub gpustat: u32,
    pub draw_mode: u32,
    pub active_clut: u16,
    pub tex_rect_x_flip: bool,
    pub tex_rect_y_flip: bool,
    pub tex_window: u32,
    pub tex_window_mask_x: u8,
    pub tex_window_mask_y: u8,
    pub tex_window_offset_x: u8,
    pub tex_window_offset_y: u8,
    pub draw_area_top_left: u32,
    pub draw_area_bottom_right: u32,
    pub draw_area_left: u16,
    pub draw_area_top: u16,
    pub draw_area_right: u16,
    pub draw_area_bottom: u16,
    pub draw_offset_raw: u32,
    pub draw_offset_x: i16,
    pub draw_offset_y: i16,
    pub force_mask_bit: bool,
    pub check_mask_bit: bool,
    pub display_vram_x: u16,
    pub display_vram_y: u16,
    pub display_width: u16,
    pub display_height: u16,
    pub display_horiz_start: u16,
    pub display_horiz_end: u16,
    pub display_vert_start: u16,
    pub display_vert_end: u16,
    pub vram_2mb_mode: bool,
    pub gpu_dither_enabled: bool,
    pub gpu_tex_bilinear: bool,
    pub gpu_force_480i_deinterlace: bool,
    pub cd_fast_load: bool,
    pub cache_control: u32,
    pub tty_buffer: String,
    pub bus_error: bool,
    pub last_bus_error_addr: u32,
    pub pending_exe: Option<PsExe>,
    pub pad1_buttons: u16,
    pub sio_rx_fifo: VecDeque<u8>,
    pub sio_mode: u16,
    pub sio_ctrl: u16,
    pub sio_baud: u16,
    pub sio_step: u8,
    pub sio_ack_delay: u32,
    pub sio_ack_pulse_cycles: u32,
    pub sio_ack_input: bool,
    pub sio_irq_pending: bool,
    pub cd_index: u8,
    pub cd_stat: u8,
    pub cd_mode: u8,
    pub cd_irq_enable: u8,
    pub cd_irq_flags: u8,
    pub debug_pc: u32,
    pub cd_request: u8,
    pub cd_bfrd: bool,
    pub cd_busy: bool,
    pub cd_reading: bool,
    pub cd_disc_present: bool,
    pub cd_raw_2352: bool,
    pub seek_lba: u32,
    pub read_lba: u32,
    pub cd_setloc_msf: [u8; 3],
    pub cd_filter_file: u8,
    pub cd_filter_chan: u8,
    pub cd_vol_ll: u8,
    pub cd_vol_lr: u8,
    pub cd_vol_rl: u8,
    pub cd_vol_rr: u8,
    pub cd_vol_apply: u8,
    pub cd_param_fifo: VecDeque<u8>,
    pub cd_resp_fifo: VecDeque<u8>,
    pub cd_data_fifo: VecDeque<u8>,
    pub cd_sector_buffer: Vec<u8>,
    pub cd_last_header: [u8; 8],
    pub cd_position_lba: u32,
    pub cd_sector_loaded_in_fifo: bool,
    pub cd_pending_irqs: VecDeque<CdPendingIrq>,
    pub cd_image: Vec<u8>,
    pub spu_ram: Vec<u8>,
    pub spu_regs: [u16; 256],
    pub spu_voice_regs: [[u16; 8]; SPU_VOICE_COUNT],
    pub spu_key_on: u32,
    pub spu_key_off: u32,
    pub spu_pmon: u32,
    pub spu_non: u32,
    pub spu_eon: u32,
    pub spu_endx: u32,
    pub spu_ram_addr: u32,
    pub spu_irq_addr: u32,
    pub spu_cnt: u16,
    pub spu_stat: u16,
    pub spu_ram_dtc: u16,
    pub mdec_cmd: u8,
    pub mdec_words_remaining: usize,
    pub mdec_in_params: Vec<u32>,
    pub mdec_out_fifo: VecDeque<u32>,
    pub mdec_param_echo: u16,
    pub mdec_output_depth: u8,
    pub mdec_output_signed: bool,
    pub mdec_output_bit15: bool,
    pub mdec_enable_in: bool,
    pub mdec_enable_out: bool,
    pub mdec_current_block: u8,
    pub mdec_quant_y: [u8; 64],
    pub mdec_quant_uv: [u8; 64],
    pub mdec_quant_color: bool,
    pub mdec_scale_table: [i16; 64],
    pub spu: SpuSynthesizer,
    pub pad1: DualShockController,
    pub cd_muted: bool,
    pub cd_playing_cdda: bool,
}
impl Bus {
    pub fn new() -> Self {
        let mut dma = [DmaChannel::default(); 7];
        dma[6].chcr = 0x0000_0002;
        Self {
            ram: vec![0; RAM_SIZE],
            scratchpad: vec![0; SCRATCHPAD_SIZE],
            bios: vec![0; BIOS_SIZE],
            vram: vec![0; VRAM_WIDTH * VRAM_HEIGHT],
            i_stat: 0,
            i_mask: 0,
            dma,
            dpcr: 0x0FFF_FFFF,
            dicr: 0,
            timers: [Timer::default(); 3],
            hblank_accumulator: 0,
            vblank_accumulator: 0,
            vblank_triggered: false,
            gp0_command_log: Vec::new(),
            gp0_fifo: Vec::with_capacity(GP0_FIFO_CAPACITY),
            gp0_words_remaining: 0,
            gp0_mode: Gp0Mode::Command,
            vram_read_queue: VecDeque::new(),
            gpuread_latch: 0,
            gpustat: 0x1480_2000,
            draw_mode: 0,
            active_clut: 0,
            tex_rect_x_flip: false,
            tex_rect_y_flip: false,
            tex_window: 0,
            tex_window_mask_x: 0,
            tex_window_mask_y: 0,
            tex_window_offset_x: 0,
            tex_window_offset_y: 0,
            draw_area_top_left: 0,
            draw_area_bottom_right: 0,
            draw_area_left: 0,
            draw_area_top: 0,
            draw_area_right: (VRAM_WIDTH - 1) as u16,
            draw_area_bottom: (VRAM_HEIGHT - 1) as u16,
            draw_offset_raw: 0,
            draw_offset_x: 0,
            draw_offset_y: 0,
            force_mask_bit: false,
            check_mask_bit: false,
            display_vram_x: 0,
            display_vram_y: 0,
            display_width: 320,
            display_height: 240,
            display_horiz_start: 0x200,
            display_horiz_end: 0xC00,
            display_vert_start: 0x010,
            display_vert_end: 0x100,
            vram_2mb_mode: false,
            gpu_dither_enabled: false,
            gpu_tex_bilinear: false,
            gpu_force_480i_deinterlace: false,
            cd_fast_load: false,
            cache_control: 0,
            tty_buffer: String::new(),
            bus_error: false,
            last_bus_error_addr: 0,
            pending_exe: None,
            pad1_buttons: 0xFFFF,
            sio_rx_fifo: VecDeque::with_capacity(8),
            sio_mode: 0,
            sio_ctrl: 0,
            sio_baud: 0x0088,
            sio_step: 0,
            sio_ack_delay: 0,
            sio_ack_pulse_cycles: 0,
            sio_ack_input: false,
            sio_irq_pending: false,
            cd_index: 0,
            cd_stat: 0x10,
            cd_mode: 0,
            cd_irq_enable: 0,
            cd_irq_flags: 0,
            debug_pc: 0,
            cd_request: 0,
            cd_bfrd: false,
            cd_busy: false,
            cd_reading: false,
            cd_disc_present: false,
            cd_raw_2352: false,
            seek_lba: 0,
            read_lba: 0,
            cd_setloc_msf: [0, 2, 0],
            cd_filter_file: 0,
            cd_filter_chan: 0,
            cd_vol_ll: 0x80,
            cd_vol_lr: 0x00,
            cd_vol_rl: 0x00,
            cd_vol_rr: 0x80,
            cd_vol_apply: 0,
            cd_param_fifo: VecDeque::with_capacity(CD_FIFO_CAPACITY),
            cd_resp_fifo: VecDeque::with_capacity(CD_FIFO_CAPACITY),
            cd_data_fifo: VecDeque::with_capacity(CD_SECTOR_SIZE_RAW_DATA),
            cd_sector_buffer: Vec::new(),
            cd_last_header: [0; 8],
            cd_position_lba: 0,
            cd_sector_loaded_in_fifo: false,
            cd_pending_irqs: VecDeque::new(),
            cd_image: Vec::new(),
            spu_ram: vec![0; SPU_RAM_SIZE],
            spu_regs: {
                let mut regs = [0u16; 256];
                regs[(0x1F80_1DAC - 0x1F80_1C00) >> 1] = 0x0004;
                regs
            },
            spu_voice_regs: [[0u16; 8]; SPU_VOICE_COUNT],
            spu_key_on: 0,
            spu_key_off: 0,
            spu_pmon: 0,
            spu_non: 0,
            spu_eon: 0,
            spu_endx: 0x00FF_FFFF,
            spu_ram_addr: 0,
            spu_irq_addr: 0,
            spu_cnt: 0,
            spu_stat: 0,
            spu_ram_dtc: 0x0004,
            mdec_cmd: 0,
            mdec_words_remaining: 0,
            mdec_in_params: Vec::new(),
            mdec_out_fifo: VecDeque::new(),
            mdec_param_echo: 0,
            mdec_output_depth: 0,
            mdec_output_signed: false,
            mdec_output_bit15: false,
            mdec_enable_in: false,
            mdec_enable_out: false,
            mdec_current_block: 4,
            mdec_quant_y: [1u8; 64],
            mdec_quant_uv: [1u8; 64],
            mdec_quant_color: false,
            mdec_scale_table: MDEC_DEFAULT_SCALE_TABLE,
            spu: SpuSynthesizer::new(),
            pad1: DualShockController::new(),
            cd_muted: false,
            cd_playing_cdda: false,
        }
    }

    pub fn load_bios<P: AsRef<Path>>(&mut self, path: P) -> Result<(), String> {
        let data = fs::read(path).map_err(|e| format!("Failed to read BIOS: {}", e))?;
        if data.len() != BIOS_SIZE {
            return Err(format!("Invalid BIOS size: expected {} bytes, got {}", BIOS_SIZE, data.len()));
        }
        self.bios.copy_from_slice(&data);
        Ok(())
    }

    pub fn queue_exe_bytes(&mut self, exe_bytes: &[u8]) -> Result<PsExe, String> {
        let exe = PsExe::from_bytes(exe_bytes)?;
        self.pending_exe = Some(exe.clone());
        Ok(exe)
    }

    pub fn queue_exe_file<P: AsRef<Path>>(&mut self, path: P) -> Result<PsExe, String> {
        let data = fs::read(path).map_err(|e| format!("Failed to read PS-EXE: {}", e))?;
        self.queue_exe_bytes(&data)
    }

    pub fn load_exe_payload(&mut self, exe: &PsExe) {
        let base_paddr = Self::mask_address(exe.load_addr) as usize;
        for (i, &byte) in exe.payload.iter().enumerate() {
            self.ram[(base_paddr + i) & (RAM_SIZE - 1)] = byte;
        }
    }

    #[inline]
    pub fn read_sio_rx(&mut self) -> u8 {
        self.sio_rx_fifo.pop_front().unwrap_or(0xFF)
    }

    #[inline]
    pub fn read_sio_stat(&self) -> u32 {
        let mut stat = 0x0000_0005;
        if !self.sio_rx_fifo.is_empty() {
            stat |= 1 << 1;
        }
        if self.sio_ack_input {
            stat |= 1 << 7;
        }
        if self.sio_irq_pending {
            stat |= 1 << 9;
        }
        stat
    }

    pub fn write_sio_ctrl(&mut self, val: u16) {
        // Bit 6: Error / Reset
        if (val & (1 << 6)) != 0 {
            self.sio_rx_fifo.clear();
            self.sio_step = 0;
            self.sio_ack_delay = 0;
            self.sio_ack_pulse_cycles = 0;
            self.sio_ack_input = false;
            self.sio_irq_pending = false;
            self.i_stat &= !(1 << 7);
            self.sio_mode = 0;
            self.sio_baud = 0x0088;
            self.pad1.reset_transfer();
        }

        let prev_slot = self.sio_ctrl & (1 << 13);
        let new_slot = val & (1 << 13);

        // Bit 4: Acknowledge Interrupt (clears SIO IRQ and I_STAT bit 7)
        if (val & (1 << 4)) != 0 {
            self.sio_irq_pending = false;
            self.i_stat &= !(1 << 7);
        }

        self.sio_ctrl = val & !((1 << 4) | (1 << 6));

        // When /DTR (bit 1) is deasserted or slot changes, reset transfer state
        if (self.sio_ctrl & (1 << 1)) == 0 || prev_slot != new_slot {
            self.sio_step = 0;
            self.sio_ack_delay = 0;
            self.sio_ack_pulse_cycles = 0;
            self.sio_ack_input = false;
            self.pad1.reset_transfer();
        }
    }

    pub fn write_sio_tx(&mut self, val: u8) {
        self.sio_ack_input = false;
        self.sio_ack_pulse_cycles = 0;

        let dtr_active = (self.sio_ctrl & (1 << 1)) != 0;
        let port2_selected = (self.sio_ctrl & (1 << 13)) != 0;

        if !dtr_active || port2_selected {
            self.sio_rx_fifo.push_back(0xFF);
            self.sio_ack_delay = 0;
            self.sio_step = 0;
            self.pad1.reset_transfer();
            return;
        }

        // Separate Controller vs Memory Card Addressing:
        // Memory Card addressing (0x81):
        // For an unplugged / no memory card, DO NOT assert an ACK pulse,
        // return 0xFF in sio_rx_data, and reset transfer state so subsequent
        // controller polls are never corrupted.
        if val == 0x81 {
            self.sio_rx_fifo.push_back(0xFF);
            self.sio_ack_delay = 0;
            self.sio_ack_input = false;
            self.sio_ack_pulse_cycles = 0;
            self.sio_step = 0;
            self.pad1.reset_transfer();
            return;
        }

        // Controller Port addressing (0x01) at start of packet:
        // First byte received (0x01): reply with 0xFF, assert ACK after SIO0_ACK_DELAY_CYCLES,
        // and advance to step 1 (expecting poll command 0x42 or DualShock config command).
        if self.sio_step == 0 {
            if val == 0x01 {
                self.pad1.reset_transfer();
                self.sio_rx_fifo.push_back(0xFF);
                self.sio_ack_delay = SIO0_ACK_DELAY_CYCLES;
                self.sio_step = 1;
            } else {
                self.sio_rx_fifo.push_back(0xFF);
                self.sio_ack_delay = 0;
            }
            return;
        }

        // Active controller communication sequence (Digital Pad + DualShock Analog/Config protocol):
        let (resp, has_more) = self.pad1.process_sio_byte(self.sio_step, val, self.pad1_buttons);
        self.sio_rx_fifo.push_back(resp);
        if has_more {
            self.sio_ack_delay = SIO0_ACK_DELAY_CYCLES;
            self.sio_step = self.sio_step.saturating_add(1);
        } else {
            self.sio_ack_delay = 0;
            self.sio_step = 0;
        }
    }
    pub fn load_cd_bin_bytes(&mut self, data: Vec<u8>) {
        const CD_SYNC_HEADER: [u8; 12] = [
            0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
        ];
        let has_sync = data.len() >= 12 && data[0..12] == CD_SYNC_HEADER;
        self.cd_raw_2352 = has_sync
            || (!data.is_empty()
                && data.len() % CD_SECTOR_SIZE_RAW_FULL == 0
                && data.len() % CD_SECTOR_SIZE_COOKED != 0);
        self.cd_image = data;
        self.cd_disc_present = true;
        self.cd_stat = 0x02;
    }

    pub fn load_cd_bin_file<P: AsRef<Path>>(&mut self, path: P) -> Result<(), String> {
        let data = fs::read(path).map_err(|e| format!("Failed to read CD-ROM BIN image: {}", e))?;
        self.load_cd_bin_bytes(data);
        Ok(())
    }

    #[inline]
    fn bcd_to_u32(val: u8) -> u32 {
        (((val >> 4) as u32) * 10) + ((val & 0x0F) as u32)
    }

    #[inline]
    fn u32_to_bcd(val: u32) -> u8 {
        let clamped = val % 100;
        (((clamped / 10) as u8) << 4) | ((clamped % 10) as u8)
    }

    #[inline]
    pub fn read_cd_status(&self) -> u8 {
        let mut status = self.cd_index & 0x03;
        if self.cd_param_fifo.is_empty() {
            status |= 1 << 3;
        }
        if self.cd_param_fifo.len() < CD_FIFO_CAPACITY {
            status |= 1 << 4;
        }
        if !self.cd_resp_fifo.is_empty() {
            status |= 1 << 5;
        }
        if self.cd_bfrd && !self.cd_data_fifo.is_empty() {
            status |= 1 << 6;
        }
        if self.cd_busy {
            status |= 1 << 7;
        }
        status
    }

    #[inline]
    pub fn read_cd_data_byte(&mut self) -> u8 {
        if self.cd_bfrd {
            self.cd_data_fifo.pop_front().unwrap_or(0x00)
        } else {
            0x00
        }
    }

    #[inline]
    pub fn read_cd_data_word(&mut self) -> u32 {
        let b0 = self.read_cd_data_byte();
        let b1 = self.read_cd_data_byte();
        let b2 = self.read_cd_data_byte();
        let b3 = self.read_cd_data_byte();
        u32::from_le_bytes([b0, b1, b2, b3])
    }

    pub fn read_cd_reg(&mut self, offset: u8) -> u8 {
        match offset & 0x03 {
            0 => self.read_cd_status(),
            1 => self.cd_resp_fifo.pop_front().unwrap_or(0x00),
            2 => self.read_cd_data_byte(),
            3 => {
                if (self.cd_index & 1) == 0 {
                    0xE0 | (self.cd_irq_enable & 0x1F)
                } else {
                    0xE0 | (self.cd_irq_flags & 0x1F)
                }
            }
            _ => 0,
        }
    }

    pub fn write_cd_reg(&mut self, offset: u8, val: u8) {
        match offset & 0x03 {
            0 => {
                self.cd_index = val & 0x03;
            }
            1 => {
                self.write_cd_command(val);
            }
            2 => match self.cd_index & 0x03 {
                0 => {
                    if self.cd_param_fifo.len() < CD_FIFO_CAPACITY {
                        self.cd_param_fifo.push_back(val);
                    }
                }
                1 => {
                    self.cd_irq_enable = val & 0x1F;
                    if (self.cd_irq_flags & self.cd_irq_enable & 0x1F) != 0 {
                        self.i_stat |= 1 << 2;
                    }
                }
                2 => {
                    self.cd_vol_ll = val;
                }
                3 => {
                    self.cd_vol_rl = val;
                }
                _ => {}
            },
            3 => match self.cd_index & 0x03 {
                0 => {
                    self.cd_request = val;
                    if (val & 0x80) != 0 {
                        self.cd_bfrd = true;
                        if !self.cd_sector_loaded_in_fifo && !self.cd_sector_buffer.is_empty() {
                            self.cd_data_fifo.clear();
                            self.cd_data_fifo.extend(self.cd_sector_buffer.iter().copied());
                            self.cd_sector_loaded_in_fifo = true;
                        }
                    } else {
                        self.cd_bfrd = false;
                        self.cd_data_fifo.clear();
                        self.cd_sector_loaded_in_fifo = false;
                    }
                }
                1 => {
                    self.cd_irq_flags &= !(val & 0x1F);
                    if (val & 0x40) != 0 {
                        self.cd_param_fifo.clear();
                    }
                    self.try_deliver_cd_irq();
                }
                2 => {
                    self.cd_vol_lr = val;
                }
                3 => {
                    self.cd_vol_rr = val;
                    self.cd_vol_apply = val;
                }
                _ => {}
            },
            _ => {}
        }
    }

    fn queue_cd_irq(
        &mut self,
        irq_type: u8,
        response: Vec<u8>,
        delay: u32,
        is_first_stage: bool,
        is_read_sector: bool,
    ) {
        self.cd_pending_irqs.push_back(CdPendingIrq {
            irq_type,
            response,
            delay,
            is_first_stage,
            is_read_sector,
        });
    }

    pub fn write_cd_command(&mut self, cmd: u8) {
        self.cd_busy = true;
        match cmd {
            0x01 => {
                let stat = self.cd_stat;
                if self.cd_disc_present {
                    self.cd_stat = (self.cd_stat & !0x10) | 0x02;
                } else {
                    self.cd_stat = 0x10;
                }
                self.queue_cd_irq(3, vec![stat], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x02 => {
                let amm = self.cd_param_fifo.pop_front().unwrap_or(0);
                let ass = self.cd_param_fifo.pop_front().unwrap_or(0);
                let asect = self.cd_param_fifo.pop_front().unwrap_or(0);
                self.cd_setloc_msf = [amm, ass, asect];
                let mm = Self::bcd_to_u32(amm);
                let ss = Self::bcd_to_u32(ass);
                let ff = Self::bcd_to_u32(asect);
                self.seek_lba = ((mm * 60 + ss) * 75 + ff).saturating_sub(150);
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x03 => {
                // Play CD-DA audio track
                let _track = self.cd_param_fifo.pop_front();
                self.read_lba = self.seek_lba;
                self.cd_stat = (self.cd_stat & !0xD0) | 0x02 | 0x80;
                self.cd_reading = true;
                self.cd_playing_cdda = true;
                self.cd_pending_irqs.retain(|irq| !irq.is_read_sector);
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                self.queue_cd_irq(1, vec![self.cd_stat], self.cd_sector_delay(), false, true);
            }
            0x06 | 0x1B => {
                self.read_lba = self.seek_lba;
                if self.cd_disc_present {
                    self.cd_stat = (self.cd_stat & !0xD0) | 0x02 | 0x20;
                } else {
                    self.cd_stat |= 0x20;
                }
                self.cd_reading = true;
                self.cd_playing_cdda = false;
                self.cd_pending_irqs.retain(|irq| !irq.is_read_sector);
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                self.queue_cd_irq(1, vec![self.cd_stat], self.cd_sector_delay(), false, true);
            }
            0x07 | 0x08 | 0x09 => {
                self.cd_reading = false;
                self.cd_playing_cdda = false;
                self.cd_pending_irqs.retain(|irq| !irq.is_read_sector);
                self.cd_stat &= !0xE0;
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                self.queue_cd_irq(2, vec![self.cd_stat], CD_INT2_DELAY_CYCLES, false, false);
            }
            0x0A => {
                self.cd_reading = false;
                self.cd_playing_cdda = false;
                self.cd_muted = false;
                self.cd_pending_irqs.retain(|irq| !irq.is_read_sector);
                self.cd_mode = 0x20;
                self.cd_stat = 0x02;
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                self.queue_cd_irq(2, vec![0x02], CD_INT2_DELAY_CYCLES, false, false);
            }
            0x0B => {
                self.cd_muted = true;
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x0C => {
                self.cd_muted = false;
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x0D => {
                self.cd_filter_file = self.cd_param_fifo.pop_front().unwrap_or(0);
                self.cd_filter_chan = self.cd_param_fifo.pop_front().unwrap_or(0);
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x0E => {
                self.cd_mode = self.cd_param_fifo.pop_front().unwrap_or(0);
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x0F => {
                self.queue_cd_irq(
                    3,
                    vec![self.cd_stat, self.cd_mode, 0x00, self.cd_filter_file, self.cd_filter_chan],
                    CD_INT3_DELAY_CYCLES,
                    true,
                    false,
                );
            }
            0x10 => {
                self.queue_cd_irq(
                    3,
                    self.cd_last_header.to_vec(),
                    CD_INT3_DELAY_CYCLES,
                    true,
                    false,
                );
            }
            0x11 => {
                let rel_frames = self.cd_position_lba;
                let r_mm = Self::u32_to_bcd(rel_frames / (60 * 75));
                let r_ss = Self::u32_to_bcd((rel_frames / 75) % 60);
                let r_ff = Self::u32_to_bcd(rel_frames % 75);
                let abs_frames = rel_frames.saturating_add(150);
                let a_mm = Self::u32_to_bcd(abs_frames / (60 * 75));
                let a_ss = Self::u32_to_bcd((abs_frames / 75) % 60);
                let a_ff = Self::u32_to_bcd(abs_frames % 75);
                self.queue_cd_irq(
                    3,
                    vec![0x01, 0x01, r_mm, r_ss, r_ff, a_mm, a_ss, a_ff],
                    CD_INT3_DELAY_CYCLES,
                    true,
                    false,
                );
            }
            0x13 => {
                self.queue_cd_irq(3, vec![self.cd_stat, 0x01, 0x01], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x14 => {
                let _track = self.cd_param_fifo.pop_front().unwrap_or(0);
                self.queue_cd_irq(3, vec![self.cd_stat, 0x00, 0x02], CD_INT3_DELAY_CYCLES, true, false);
            }
            0x15 | 0x16 => {
                self.cd_reading = false;
                self.cd_pending_irqs.retain(|irq| !irq.is_read_sector);
                self.read_lba = self.seek_lba;
                self.cd_position_lba = self.seek_lba;
                self.cd_stat = 0x02;
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                self.queue_cd_irq(2, vec![self.cd_stat], CD_INT2_DELAY_CYCLES, false, false);
            }
            0x19 => {
                let sub = self.cd_param_fifo.pop_front().unwrap_or(0);
                let resp = match sub {
                    0x20 => vec![0x98, 0x06, 0x10, 0xC3],
                    _ => vec![self.cd_stat],
                };
                self.queue_cd_irq(3, resp, CD_INT3_DELAY_CYCLES, true, false);
            }
            0x1A => {
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                if self.cd_disc_present {
                    self.queue_cd_irq(
                        2,
                        vec![0x02, 0x00, 0x20, 0x00, b'S', b'C', b'E', b'A'],
                        CD_INT2_DELAY_CYCLES,
                        false,
                        false,
                    );
                } else {
                    self.queue_cd_irq(
                        5,
                        vec![0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                        CD_INT2_DELAY_CYCLES,
                        false,
                        false,
                    );
                }
            }
            0x1E => {
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
                self.queue_cd_irq(2, vec![self.cd_stat], CD_INT2_DELAY_CYCLES, false, false);
            }
            _ => {
                self.queue_cd_irq(3, vec![self.cd_stat], CD_INT3_DELAY_CYCLES, true, false);
            }
        }
        self.cd_param_fifo.clear();
    }

    fn extract_cd_sector(&self, lba: u32) -> Vec<u8> {
        let raw_sector_mode = (self.cd_mode & 0x20) != 0;
        let out_size = if raw_sector_mode {
            CD_SECTOR_SIZE_RAW_DATA
        } else {
            CD_SECTOR_SIZE_COOKED
        };
        if self.cd_image.is_empty() {
            return vec![0u8; out_size];
        }
        let lba_idx = lba as usize;
        if self.cd_raw_2352 {
            let sector_start = lba_idx.saturating_mul(CD_SECTOR_SIZE_RAW_FULL);
            if sector_start + CD_SECTOR_SIZE_RAW_FULL <= self.cd_image.len() {
                let sector = &self.cd_image[sector_start..sector_start + CD_SECTOR_SIZE_RAW_FULL];
                if raw_sector_mode {
                    return sector[12..CD_SECTOR_SIZE_RAW_FULL].to_vec();
                }
                let mode = sector[15];
                let payload_offset = if mode == 1 {
                    16
                } else if mode == 2 {
                    24
                } else if sector[16..24].iter().all(|&b| b == 0) && sector[24..32].iter().any(|&b| b != 0) {
                    24
                } else {
                    16
                };
                return sector[payload_offset..payload_offset + CD_SECTOR_SIZE_COOKED].to_vec();
            }
            vec![0u8; out_size]
        } else {
            let sector_start = lba_idx.saturating_mul(CD_SECTOR_SIZE_COOKED);
            let mut out = vec![0u8; out_size];
            if sector_start < self.cd_image.len() {
                let avail = (self.cd_image.len() - sector_start).min(CD_SECTOR_SIZE_COOKED);
                out[..avail].copy_from_slice(&self.cd_image[sector_start..sector_start + avail]);
            }
            out
        }
    }

    fn cd_sector_delay(&self) -> u32 {
        if self.cd_fast_load {
            return CPU_CLOCK_HZ / 1200; // 16x Fast CD-ROM Speed (~28,224 cycles)
        }
        CPU_CLOCK_HZ / if self.cd_mode & 0x80 != 0 { 150 } else { 75 }
    }

    fn try_deliver_cd_irq(&mut self) {
        if (self.cd_irq_flags & 0x07) != 0 {
            return;
        }
        let Some(front) = self.cd_pending_irqs.front() else {
            return;
        };
        if front.delay > 0 {
            return;
        }
        let irq = self.cd_pending_irqs.pop_front().unwrap();
        if irq.is_first_stage {
            self.cd_busy = false;
        }
        if irq.is_read_sector {
            if !self.cd_reading {
                return;
            }
            let raw_start = self.read_lba as usize * CD_SECTOR_SIZE_RAW_FULL;
            if self.cd_playing_cdda {
                if self.cd_raw_2352 && raw_start + CD_SECTOR_SIZE_RAW_FULL <= self.cd_image.len() {
                    let full_sector = &self.cd_image[raw_start..raw_start + CD_SECTOR_SIZE_RAW_FULL];
                    if !self.cd_muted {
                        self.spu.decode_cdda_sector(full_sector);
                    }
                }
                self.cd_position_lba = self.read_lba;
                self.read_lba = self.read_lba.wrapping_add(1);
                self.queue_cd_irq(1, vec![self.cd_stat], self.cd_sector_delay(), false, true);
                return;
            }
            if self.cd_raw_2352 && raw_start + 24 <= self.cd_image.len() {
                let sector = &self.cd_image[raw_start..raw_start + 24];
                let mode2 = sector[15] == 2;
                let audio_realtime = mode2 && sector[18] & 0x44 == 0x44;
                let filtered = audio_realtime && self.cd_mode & 0x08 != 0;
                let xa_audio = audio_realtime && self.cd_mode & 0x40 != 0
                    && (self.cd_mode & 0x08 == 0
                        || (sector[16] == self.cd_filter_file && sector[17] == self.cd_filter_chan));
                if filtered || xa_audio {
                    // XA audio is routed to the audio decoder, never to the
                    // data FIFO/INT1 handler used by STR video.
                    if xa_audio && !self.cd_muted && raw_start + CD_SECTOR_SIZE_RAW_FULL <= self.cd_image.len() {
                        let full_sector = &self.cd_image[raw_start..raw_start + CD_SECTOR_SIZE_RAW_FULL];
                        self.spu.decode_xa_sector(full_sector);
                    }
                    self.cd_position_lba = self.read_lba;
                    self.cd_last_header.copy_from_slice(&sector[12..20]);
                    self.read_lba = self.read_lba.wrapping_add(1);
                    self.queue_cd_irq(1, vec![self.cd_stat], self.cd_sector_delay(), false, true);
                    return;
                }
            }
            self.cd_sector_buffer = self.extract_cd_sector(self.read_lba);
            self.cd_position_lba = self.read_lba;
            // GetlocL reports the sector actually received, including its XA
            // subheader, rather than the next read position or filter settings.
            let start = self.read_lba as usize * CD_SECTOR_SIZE_RAW_FULL;
            if self.cd_raw_2352 && start + 20 <= self.cd_image.len() {
                self.cd_last_header.copy_from_slice(&self.cd_image[start + 12..start + 20]);
            } else {
                let frames = self.read_lba.saturating_add(150);
                self.cd_last_header = [Self::u32_to_bcd(frames / 4500), Self::u32_to_bcd(frames / 75 % 60), Self::u32_to_bcd(frames % 75), 1, 0, 0, 0, 0];
            }
            self.read_lba = self.read_lba.wrapping_add(1);
            self.cd_data_fifo.clear();
            self.cd_sector_loaded_in_fifo = false;
            self.cd_bfrd = false;
            self.queue_cd_irq(1, vec![self.cd_stat], self.cd_sector_delay(), false, true);
        }
        self.cd_resp_fifo.clear();
        for b in irq.response.into_iter().take(CD_FIFO_CAPACITY) {
            self.cd_resp_fifo.push_back(b);
        }
        self.cd_irq_flags = (self.cd_irq_flags & !0x07) | (irq.irq_type & 0x07);
        if (self.cd_irq_flags & self.cd_irq_enable & 0x1F) != 0 {
            self.i_stat |= 1 << 2;
        }
    }

    #[inline]
    fn tick_cdrom(&mut self, cycles: u32) {
        if self.cd_pending_irqs.is_empty() {
            return;
        }
        let mut rem = cycles;
        while rem > 0 {
            self.try_deliver_cd_irq();
            let Some(front) = self.cd_pending_irqs.front_mut() else {
                break;
            };
            if front.delay == 0 {
                break;
            }
            let step = rem.min(front.delay);
            front.delay -= step;
            rem -= step;
            if front.delay == 0 {
                self.try_deliver_cd_irq();
            }
        }

    }

    #[inline]
    pub fn decode_spu_adpcm_block(block: &[u8; 16], prev1: &mut i32, prev2: &mut i32) -> [i16; 28] {
        decode_spu_adpcm_block(block, prev1, prev2)
    }

    #[inline]
    fn check_spu_irq_addr(&mut self, addr: u32) {
        if (self.spu_cnt & 0x0040) != 0 && (addr & 0x7FFF8) == (self.spu_irq_addr & 0x7FFF8) {
            self.spu_stat |= 0x0040;
            self.i_stat |= 1 << 9;
        }
    }

    #[inline]
    pub fn spu_ram_write_u16(&mut self, val: u16) {
        let addr = self.spu_ram_addr & 0x7FFFF;
        self.check_spu_irq_addr(addr);
        let bytes = val.to_le_bytes();
        self.spu_ram[addr as usize] = bytes[0];
        self.spu_ram[((addr + 1) & 0x7FFFF) as usize] = bytes[1];
        self.spu_ram_addr = (addr + 2) & 0x7FFFF;
        self.check_spu_irq_addr(self.spu_ram_addr);
    }

    #[inline]
    pub fn spu_ram_read_u16(&mut self) -> u16 {
        let addr = self.spu_ram_addr & 0x7FFFF;
        self.check_spu_irq_addr(addr);
        let lo = self.spu_ram[addr as usize];
        let hi = self.spu_ram[((addr + 1) & 0x7FFFF) as usize];
        self.spu_ram_addr = (addr + 2) & 0x7FFFF;
        self.check_spu_irq_addr(self.spu_ram_addr);
        u16::from_le_bytes([lo, hi])
    }

    #[inline]
    pub fn update_spu_stat(&mut self) {
        let mode_low6 = self.spu_cnt & 0x003F;
        let irq9_flag = if (self.spu_cnt & 0x0040) != 0 {
            self.spu_stat & 0x0040
        } else {
            0
        };
        let xfer_mode = (self.spu_cnt >> 4) & 0x03;
        let dma_bits = match xfer_mode {
            2 => (1 << 7) | (1 << 8),
            3 => (1 << 7) | (1 << 9),
            _ => 0,
        };
        let capture_half = self.spu_stat & (1 << 11);
        self.spu_stat = mode_low6 | irq9_flag | dma_bits | capture_half;
        self.spu_regs[(0x1F80_1DAE - 0x1F80_1C00) >> 1] = self.spu_stat;
    }

    pub fn read_spu_reg16(&mut self, paddr: u32) -> u16 {
        let aligned = paddr & !1;
        let reg_idx = ((aligned - 0x1F80_1C00) >> 1) as usize;
        match aligned {
0x1F80_1C00..=0x1F80_1D7E => {
            let offset = (aligned - 0x1F80_1C00) as usize;
            let voice = offset >> 4;
            let reg = (offset >> 1) & 0x07;
            self.spu_voice_regs[voice][reg]
        }
            0x1F80_1D88 => (self.spu_key_on & 0xFFFF) as u16,
            0x1F80_1D8A => ((self.spu_key_on >> 16) & 0xFFFF) as u16,
            0x1F80_1D8C => (self.spu_key_off & 0xFFFF) as u16,
            0x1F80_1D8E => ((self.spu_key_off >> 16) & 0xFFFF) as u16,
            0x1F80_1D90 => (self.spu_pmon & 0xFFFF) as u16,
            0x1F80_1D92 => ((self.spu_pmon >> 16) & 0xFFFF) as u16,
            0x1F80_1D94 => (self.spu_non & 0xFFFF) as u16,
            0x1F80_1D96 => ((self.spu_non >> 16) & 0xFFFF) as u16,
            0x1F80_1D98 => (self.spu_eon & 0xFFFF) as u16,
            0x1F80_1D9A => ((self.spu_eon >> 16) & 0xFFFF) as u16,
            0x1F80_1D9C => (self.spu_endx & 0xFFFF) as u16,
            0x1F80_1D9E => ((self.spu_endx >> 16) & 0x00FF) as u16,
            0x1F80_1DA4 => (self.spu_irq_addr >> 3) as u16,
            0x1F80_1DA6 => self.spu_regs[reg_idx],
            0x1F80_1DA8 => self.spu_ram_read_u16(),
            0x1F80_1DAA => self.spu_cnt,
            0x1F80_1DAC => self.spu_ram_dtc,
            0x1F80_1DAE => {
                self.spu_stat ^= 1 << 11;
                self.update_spu_stat();
                self.spu_stat
            }
            _ => self.spu_regs[reg_idx],
        }
    }

    pub fn write_spu_reg16(&mut self, paddr: u32, val: u16) {
        let aligned = paddr & !1;
        let reg_idx = ((aligned - 0x1F80_1C00) >> 1) as usize;
        if aligned != 0x1F80_1DAE && aligned != 0x1F80_1D9C && aligned != 0x1F80_1D9E {
            self.spu_regs[reg_idx] = val;
        }
        match aligned {
            0x1F80_1C00..=0x1F80_1D7E => {
                let offset = (aligned - 0x1F80_1C00) as usize;
                let voice = offset >> 4;
                let reg = (offset >> 1) & 0x07;
                self.spu_voice_regs[voice][reg] = val;
                if reg == 3 {
                    self.spu_voice_regs[voice][7] = val;
                    self.spu_regs[(offset >> 1) + 4] = val;
                    self.spu.voices[voice].repeat_addr = ((val as u32) << 3) & 0x7FFF0;
                    self.spu.voices[voice].has_custom_repeat = false;
                } else if reg == 7 {
                    self.spu.voices[voice].repeat_addr = ((val as u32) << 3) & 0x7FFF0;
                    self.spu.voices[voice].has_custom_repeat = true;
                }
            }
            0x1F80_1D88 => {
                self.spu_key_on = (self.spu_key_on & 0xFFFF_0000) | (val as u32);
                self.spu_endx &= !(val as u32);
                for v in 0..16 {
                    if (val & (1 << v)) != 0 {
                        self.spu_voice_regs[v][6] = 0x7FFF;
                        self.spu_voice_regs[v][7] = self.spu_voice_regs[v][3];
                        self.spu.key_on_voice(v, self.spu_voice_regs[v][3], self.spu_voice_regs[v][7]);
                    }
                }
            }
            0x1F80_1D8A => {
                let hi = (val & 0x00FF) as u32;
                self.spu_key_on = (self.spu_key_on & 0x0000_FFFF) | (hi << 16);
                self.spu_endx &= !(hi << 16);
                for v in 16..24 {
                    if (hi & (1 << (v - 16))) != 0 {
                        self.spu_voice_regs[v][6] = 0x7FFF;
                        self.spu_voice_regs[v][7] = self.spu_voice_regs[v][3];
                        self.spu.key_on_voice(v, self.spu_voice_regs[v][3], self.spu_voice_regs[v][7]);
                    }
                }
            }
            0x1F80_1D8C => {
                self.spu_key_off = (self.spu_key_off & 0xFFFF_0000) | (val as u32);
                for v in 0..16 {
                    if (val & (1 << v)) != 0 {
                        self.spu_voice_regs[v][6] = 0;
                        self.spu.key_off_voice(v);
                    }
                }
            }
            0x1F80_1D8E => {
                let hi = (val & 0x00FF) as u32;
                self.spu_key_off = (self.spu_key_off & 0x0000_FFFF) | (hi << 16);
                for v in 16..24 {
                    if (hi & (1 << (v - 16))) != 0 {
                        self.spu_voice_regs[v][6] = 0;
                        self.spu.key_off_voice(v);
                    }
                }
            }
            0x1F80_1D90 => self.spu_pmon = (self.spu_pmon & 0xFFFF_0000) | ((val & 0xFFFE) as u32),
            0x1F80_1D92 => self.spu_pmon = (self.spu_pmon & 0x0000_FFFF) | (((val & 0x00FF) as u32) << 16),
            0x1F80_1D94 => self.spu_non = (self.spu_non & 0xFFFF_0000) | (val as u32),
            0x1F80_1D96 => self.spu_non = (self.spu_non & 0x0000_FFFF) | (((val & 0x00FF) as u32) << 16),
            0x1F80_1D98 => self.spu_eon = (self.spu_eon & 0xFFFF_0000) | (val as u32),
            0x1F80_1D9A => self.spu_eon = (self.spu_eon & 0x0000_FFFF) | (((val & 0x00FF) as u32) << 16),
            0x1F80_1D9C | 0x1F80_1D9E => {}
            0x1F80_1DA2 => {
                self.spu.reverb_base_addr = ((val as u32) * 8) & 0x7FFFF;
                self.spu.reverb_curr_addr = self.spu.reverb_base_addr;
            }
            0x1F80_1DA4 => {
                self.spu_irq_addr = ((val as u32) * 8) & 0x7FFFF;
                self.check_spu_irq_addr(self.spu_ram_addr);
            }
            0x1F80_1DA6 => {
                self.spu_ram_addr = ((val as u32) * 8) & 0x7FFFF;
                self.check_spu_irq_addr(self.spu_ram_addr);
            }
            0x1F80_1DA8 => {
                self.spu_ram_write_u16(val);
            }
            0x1F80_1DAA => {
                self.spu_cnt = val;
                if (val & 0x0040) == 0 {
                    self.spu_stat &= !0x0040;
                }
                self.update_spu_stat();
                self.check_spu_irq_addr(self.spu_ram_addr);
            }
            0x1F80_1DAC => {
                self.spu_ram_dtc = val;
            }
            0x1F80_1DAE => {}
            _ => {}
        }
    }

    #[inline]
    pub fn read_mdec0(&mut self) -> u32 {
        self.mdec_out_fifo.pop_front().unwrap_or(0)
    }

    #[inline]
    pub fn read_mdec1(&self) -> u32 {
        let mut stat: u32 = 0;
        if self.mdec_out_fifo.is_empty() {
            stat |= 1 << 31;
        }
        if self.mdec_words_remaining > 0 {
            stat |= 1 << 29;
        }
        if self.mdec_enable_in && self.mdec_words_remaining > 0 {
            stat |= 1 << 28;
        }
        if self.mdec_enable_out && !self.mdec_out_fifo.is_empty() {
            stat |= 1 << 27;
        }
        stat |= ((self.mdec_output_depth & 0x03) as u32) << 25;
        if self.mdec_output_signed {
            stat |= 1 << 24;
        }
        if self.mdec_output_bit15 {
            stat |= 1 << 23;
        }
        stat |= ((self.mdec_current_block & 0x07) as u32) << 16;
        let low16 = if self.mdec_words_remaining > 0 {
            (self.mdec_words_remaining - 1) as u16
        } else {
            self.mdec_param_echo
        };
        stat | (low16 as u32)
    }

    pub fn write_mdec1(&mut self, word: u32) {
        if (word & 0x8000_0000) != 0 {
            self.mdec_cmd = 0;
            self.mdec_words_remaining = 0;
            self.mdec_in_params.clear();
            self.mdec_out_fifo.clear();
            self.mdec_param_echo = 0;
            self.mdec_output_depth = 0;
            self.mdec_output_signed = false;
            self.mdec_output_bit15 = false;
            self.mdec_current_block = 4;
        }
        self.mdec_enable_in = (word & (1 << 30)) != 0;
        self.mdec_enable_out = (word & (1 << 29)) != 0;
        self.check_and_run_dma(1);
    }

    pub fn write_mdec0(&mut self, word: u32) {
        if self.mdec_words_remaining == 0 {
            let cmd = ((word >> 29) & 0x07) as u8;
            self.mdec_cmd = cmd;
            self.mdec_in_params.clear();
            self.mdec_output_depth = ((word >> 27) & 0x03) as u8;
            self.mdec_output_signed = ((word >> 26) & 1) != 0;
            self.mdec_output_bit15 = ((word >> 25) & 1) != 0;
            self.mdec_param_echo = (word & 0xFFFF) as u16;

            match cmd {
                1 => {
                    self.mdec_out_fifo.clear();
                    self.mdec_param_echo = ((word >> 16) & 0xFFFF) as u16;
                    self.mdec_words_remaining = (word & 0xFFFF) as usize;
                    if self.mdec_words_remaining == 0 {
                        self.mdec_current_block = 4;
                    }
                }
                2 => {
                    self.mdec_quant_color = (word & 1) != 0;
                    self.mdec_words_remaining = if self.mdec_quant_color { 32 } else { 16 };
                }
                3 => {
                    self.mdec_words_remaining = 32;
                }
                _ => {
                    self.mdec_words_remaining = 0;
                    self.mdec_current_block = 4;
                }
            }
            return;
        }

        self.mdec_in_params.push(word);
        self.mdec_words_remaining -= 1;
        if self.mdec_words_remaining == 0 {
            self.execute_mdec_command();
        }
    }

    fn execute_mdec_command(&mut self) {
        match self.mdec_cmd {
            1 => self.execute_mdec_decode_macroblock(),
            2 => {
                for (i, &w) in self.mdec_in_params.iter().take(16).enumerate() {
                    let bytes = w.to_le_bytes();
                    self.mdec_quant_y[i * 4..i * 4 + 4].copy_from_slice(&bytes);
                }
                if self.mdec_quant_color {
                    for (i, &w) in self.mdec_in_params.iter().skip(16).take(16).enumerate() {
                        let bytes = w.to_le_bytes();
                        self.mdec_quant_uv[i * 4..i * 4 + 4].copy_from_slice(&bytes);
                    }
                }
                self.mdec_current_block = 4;
            }
            3 => {
                for (i, &w) in self.mdec_in_params.iter().take(32).enumerate() {
                    self.mdec_scale_table[i * 2] = (w & 0xFFFF) as i16;
                    self.mdec_scale_table[i * 2 + 1] = ((w >> 16) & 0xFFFF) as i16;
                }
                self.mdec_current_block = 4;
            }
            _ => {
                self.mdec_current_block = 4;
            }
        }
        self.mdec_in_params.clear();
        self.check_and_run_dma(1);
    }

    #[inline]
    fn mdec_sign_extend_10(val: u16) -> i32 {
        (((val & 0x03FF) as i16) << 6 >> 6) as i32
    }

    fn mdec_decode_rl_block(
        halfwords: &[u16],
        cursor: &mut usize,
        quant: &[u8; 64],
        scale_table: &[i16; 64],
    ) -> Option<[i32; 64]> {
        while *cursor < halfwords.len() && halfwords[*cursor] == 0xFE00 {
            *cursor += 1;
        }
        if *cursor >= halfwords.len() {
            return None;
        }

        let mut coeff = [0i32; 64];
        let first = halfwords[*cursor];
        *cursor += 1;

        let q_scale = ((first >> 10) & 0x3F) as i32;
        let dc_val = Self::mdec_sign_extend_10(first);
        let dc_deq = if q_scale == 0 {
            dc_val * 2
        } else {
            dc_val * (quant[0] as i32)
        };
        coeff[MDEC_ZIGZAG[0]] = dc_deq.clamp(-0x400, 0x3FF);

        let mut k = 0usize;
        while *cursor < halfwords.len() {
            let hw = halfwords[*cursor];
            *cursor += 1;
            if hw == 0xFE00 {
                break;
            }
            let skip = ((hw >> 10) & 0x3F) as usize;
            k += skip + 1;
            if k >= 64 {
                break;
            }
            let ac_val = Self::mdec_sign_extend_10(hw);
            let ac_deq = if q_scale == 0 {
                ac_val * 2
            } else {
                (ac_val * (quant[k] as i32) * q_scale + 4) >> 3
            };
            let index = if q_scale == 0 { k } else { MDEC_ZIGZAG[k] };
            coeff[index] = ac_deq.clamp(-0x400, 0x3FF);
        }

        Some(Self::mdec_idct_block(&coeff, scale_table))
    }

    fn mdec_idct_block(coeff: &[i32; 64], scale_table: &[i16; 64]) -> [i32; 64] {
        let mut src = *coeff;
        for _ in 0..2 {
            let mut dst = [0i32; 64];
            for x in 0..8 {
                for y in 0..8 {
                    let mut sum = 0i64;
                    for z in 0..8 {
                        sum += src[y + z * 8] as i64 * (scale_table[x + z * 8] >> 3) as i64;
                    }
                    dst[x + y * 8] = ((sum + 0x0fff) >> 13) as i32;
                }
            }
            src = dst;
        }
        src.map(|v| v.clamp(-128, 127))
    }

    fn execute_mdec_decode_macroblock(&mut self) {
        let mut halfwords = Vec::with_capacity(self.mdec_in_params.len() * 2);
        for &w in &self.mdec_in_params {
            halfwords.push((w & 0xFFFF) as u16);
            halfwords.push(((w >> 16) & 0xFFFF) as u16);
        }

        let mut cursor = 0usize;
        let quant_y = self.mdec_quant_y;
        let quant_uv = if self.mdec_quant_color {
            self.mdec_quant_uv
        } else {
            self.mdec_quant_y
        };
        let scale = self.mdec_scale_table;

        match self.mdec_output_depth & 0x03 {
            0 => {
                while let Some(y_blk) = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_y, &scale) {
                    let mut bytes = Vec::with_capacity(32);
                    for pair in y_blk.chunks(2) {
                        let mut p0 = (pair[0].clamp(-128, 127) >> 4) as i8;
                        let mut p1 = (pair[1].clamp(-128, 127) >> 4) as i8;
                        if !self.mdec_output_signed {
                            p0 = p0.wrapping_add(8);
                            p1 = p1.wrapping_add(8);
                        }
                        let nib0 = (p0 as u8) & 0x0F;
                        let nib1 = (p1 as u8) & 0x0F;
                        bytes.push(nib0 | (nib1 << 4));
                    }
                    for chunk in bytes.chunks(4) {
                        self.mdec_out_fifo.push_back(u32::from_le_bytes([
                            chunk[0], chunk[1], chunk[2], chunk[3],
                        ]));
                    }
                }
            }
            1 => {
                while let Some(y_blk) = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_y, &scale) {
                    let mut bytes = [0u8; 64];
                    for (i, &v) in y_blk.iter().enumerate() {
                        let clamped = v.clamp(-128, 127);
                        bytes[i] = if self.mdec_output_signed {
                            clamped as i8 as u8
                        } else {
                            (clamped + 128) as u8
                        };
                    }
                    for chunk in bytes.chunks(4) {
                        self.mdec_out_fifo.push_back(u32::from_le_bytes([
                            chunk[0], chunk[1], chunk[2], chunk[3],
                        ]));
                    }
                }
            }
            2 | 3 => {
                while let Some(cr_blk) = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_uv, &scale) {
                    let cb_blk = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_uv, &scale)
                        .unwrap_or([0i32; 64]);
                    let y1_blk = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_y, &scale)
                        .unwrap_or([0i32; 64]);
                    let y2_blk = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_y, &scale)
                        .unwrap_or([0i32; 64]);
                    let y3_blk = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_y, &scale)
                        .unwrap_or([0i32; 64]);
                    let y4_blk = Self::mdec_decode_rl_block(&halfwords, &mut cursor, &quant_y, &scale)
                        .unwrap_or([0i32; 64]);

                    let mut rgb_pixels = [(0u8, 0u8, 0u8); 256];
                    for py in 0..16usize {
                        for px in 0..16usize {
                            let y_val = match (px >= 8, py >= 8) {
                                (false, false) => y1_blk[py * 8 + px],
                                (true, false) => y2_blk[py * 8 + (px - 8)],
                                (false, true) => y3_blk[(py - 8) * 8 + px],
                                (true, true) => y4_blk[(py - 8) * 8 + (px - 8)],
                            };
                            let uv_idx = (py / 2) * 8 + (px / 2);
                            let cr = cr_blk[uv_idx];
                            let cb = cb_blk[uv_idx];

                            let mut r = (y_val + ((359 * cr + 128) >> 8)).clamp(-128, 127);
                            let mut g = (y_val + ((-88 * cb - 183 * cr + 128) >> 8)).clamp(-128, 127);
                            let mut b = (y_val + ((454 * cb + 128) >> 8)).clamp(-128, 127);

                            if !self.mdec_output_signed {
                                r += 128;
                                g += 128;
                                b += 128;
                            }
                            rgb_pixels[py * 16 + px] = (r as u8, g as u8, b as u8);
                        }
                    }

                    if (self.mdec_output_depth & 0x03) == 3 {
                        let stp = if self.mdec_output_bit15 { 0x8000u16 } else { 0u16 };
                        for pair in rgb_pixels.chunks(2) {
                            let (r0, g0, b0) = pair[0];
                            let (r1, g1, b1) = pair[1];
                            let p0 = stp
                                | (((b0 >> 3) as u16) << 10)
                                | (((g0 >> 3) as u16) << 5)
                                | ((r0 >> 3) as u16);
                            let p1 = stp
                                | (((b1 >> 3) as u16) << 10)
                                | (((g1 >> 3) as u16) << 5)
                                | ((r1 >> 3) as u16);
                            self.mdec_out_fifo.push_back((p0 as u32) | ((p1 as u32) << 16));
                        }
                    } else {
                        let mut bytes = Vec::with_capacity(256 * 3);
                        for &(r, g, b) in &rgb_pixels {
                            bytes.push(r);
                            bytes.push(g);
                            bytes.push(b);
                        }
                        for chunk in bytes.chunks(4) {
                            self.mdec_out_fifo.push_back(u32::from_le_bytes([
                                chunk[0], chunk[1], chunk[2], chunk[3],
                            ]));
                        }
                    }
                }
            }
            _ => {}
        }
        self.mdec_current_block = 4;
    }
    #[inline]
    pub fn mask_address(addr: u32) -> u32 {
        match addr >> 29 {
            0b000..=0b011 => addr,
            0b100 => addr & 0x1FFF_FFFF,
            0b101 => addr & 0x1FFF_FFFF,
            _ => addr,
        }
    }
    #[inline]
    pub fn is_kuseg_bus_error(vaddr: u32) -> bool {
        (0x2000_0000..=0x7FFF_FFFF).contains(&vaddr)
    }

    #[inline]
    pub fn is_kseg1(vaddr: u32) -> bool {
        (vaddr >> 29) == 0b101
    }
    #[inline]
    pub fn is_scratchpad_accessible(vaddr: u32) -> bool {
        !Self::is_kseg1(vaddr) && !Self::is_kuseg_bus_error(vaddr)
    }
    #[inline]
    pub fn take_bus_error(&mut self) -> bool {
        let err = self.bus_error;
        self.bus_error = false;
        err
    }
    #[inline]
    pub fn irq_pending(&self) -> bool {
        (self.i_stat & self.i_mask) != 0
    }

    #[inline]
    fn read_ram_u32(&self, paddr: u32) -> u32 {
        let idx = (paddr as usize) & (RAM_SIZE - 1) & !3;
        u32::from_le_bytes([self.ram[idx], self.ram[idx + 1], self.ram[idx + 2], self.ram[idx + 3]])
    }

    #[inline]
    fn write_ram_u32(&mut self, paddr: u32, val: u32) {
        let idx = (paddr as usize) & (RAM_SIZE - 1) & !3;
        let bytes = val.to_le_bytes();
        self.ram[idx..idx + 4].copy_from_slice(&bytes);
    }

    #[inline]
    pub fn gp0_command_total_words(cmd: u8) -> usize {
        match cmd {
            0x00 | 0x01 | 0x03..=0x1F => 1,
            0x02 => 3,
            0x20..=0x23 => 4,
            0x24..=0x27 => 7,
            0x28..=0x2B => 5,
            0x2C..=0x2F => 9,
            0x30..=0x33 => 6,
            0x34..=0x37 => 9,
            0x38..=0x3B => 8,
            0x3C..=0x3F => 12,
            0x40..=0x47 => 3,
            0x48..=0x4F => 0,
            0x50..=0x57 => 4,
            0x58..=0x5F => 0,
            0x60..=0x63 => 3,
            0x64..=0x67 => 4,
            0x68..=0x6B => 2,
            0x6C..=0x6F => 3,
            0x70..=0x73 => 2,
            0x74..=0x77 => 3,
            0x78..=0x7B => 2,
            0x7C..=0x7F => 3,
            0x80..=0x9F => 4,
            0xA0..=0xBF => 3,
            0xC0..=0xDF => 3,
            0xE0..=0xFF => 1,
        }
    }

    #[inline]
    pub fn rgb24_to_bgr555(rgb24: u32) -> u16 {
        let r5 = ((rgb24 >> 3) & 0x1F) as u16;
        let g5 = (((rgb24 >> 11) & 0x1F) as u16) << 5;
        let b5 = (((rgb24 >> 19) & 0x1F) as u16) << 10;
        b5 | g5 | r5
    }

    #[inline]
    pub fn write_vram_pixel_masked(&mut self, x: usize, y: usize, pixel: u16) {
        let idx = ((y & 0x1FF) * VRAM_WIDTH) + (x & 0x3FF);
        if self.check_mask_bit && (self.vram[idx] & 0x8000) != 0 {
            return;
        }
        let out_px = if self.force_mask_bit { pixel | 0x8000 } else { pixel };
        self.vram[idx] = out_px;
    }

    #[inline]
    pub fn sign_extend_11(val: u32) -> i32 {
        (((val & 0x7FF) as i32) << 21) >> 21
    }

    #[inline]
    pub fn decode_vertex(&self, word: u32) -> (i32, i32) {
        let x = Self::sign_extend_11(word) + (self.draw_offset_x as i32);
        let y = Self::sign_extend_11(word >> 16) + (self.draw_offset_y as i32);
        (x, y)
    }

    #[inline]
    pub fn decode_rgb(word: u32) -> (u8, u8, u8) {
        ((word & 0xFF) as u8, ((word >> 8) & 0xFF) as u8, ((word >> 16) & 0xFF) as u8)
    }

    #[inline]
    pub fn decode_uv(word: u32) -> (u8, u8) {
        ((word & 0xFF) as u8, ((word >> 8) & 0xFF) as u8)
    }

    #[inline]
    pub fn rgb888_to_bgr555(r: u8, g: u8, b: u8) -> u16 {
        (((b >> 3) as u16) << 10) | (((g >> 3) as u16) << 5) | ((r >> 3) as u16)
    }

    #[inline]
    pub fn dither_rgb888_to_bgr555(x: i32, y: i32, r: u8, g: u8, b: u8) -> u16 {
        const DITHER_LUT: [[i16; 4]; 4] = [
            [-4, 0, -3, 1],
            [2, -2, 3, -1],
            [-3, 1, -4, 0],
            [3, -1, 2, -2],
        ];
        let d = DITHER_LUT[(y & 3) as usize][(x & 3) as usize];
        let rd = ((r as i16 + d).clamp(0, 255) as u16) >> 3;
        let gd = ((g as i16 + d).clamp(0, 255) as u16) >> 3;
        let bd = ((b as i16 + d).clamp(0, 255) as u16) >> 3;
        (bd << 10) | (gd << 5) | rd
    }

    #[inline]
    pub fn sample_texture_bilinear(&self, u_fp8: i32, v_fp8: i32, clut: u16) -> u16 {
        let u0 = ((u_fp8 >> 8) & 0xFF) as u8;
        let v0 = ((v_fp8 >> 8) & 0xFF) as u8;
        let u1 = u0.wrapping_add(1);
        let v1 = v0.wrapping_add(1);
        let wx = (u_fp8 & 0xFF) as u32;
        let wy = (v_fp8 & 0xFF) as u32;

        let t00 = self.sample_texture(u0, v0, clut);
        let t10 = self.sample_texture(u1, v0, clut);
        let t01 = self.sample_texture(u0, v1, clut);
        let t11 = self.sample_texture(u1, v1, clut);

        // Preserve exact PS1 0x0000 transparency on center texel
        if t00 == 0x0000 {
            return 0x0000;
        }
        let s10 = if t10 == 0 { t00 } else { t10 };
        let s01 = if t01 == 0 { t00 } else { t01 };
        let s11 = if t11 == 0 { t00 } else { t11 };

        let bilerp_ch = |shift: usize| -> u16 {
            let c00 = ((t00 >> shift) & 0x1F) as u32;
            let c10 = ((s10 >> shift) & 0x1F) as u32;
            let c01 = ((s01 >> shift) & 0x1F) as u32;
            let c11 = ((s11 >> shift) & 0x1F) as u32;
            let top = (c00 * (256 - wx) + c10 * wx) >> 8;
            let bot = (c01 * (256 - wx) + c11 * wx) >> 8;
            (((top * (256 - wy) + bot * wy) >> 8) & 0x1F) as u16
        };
        let r = bilerp_ch(0);
        let g = bilerp_ch(5);
        let b = bilerp_ch(10);
        (t00 & 0x8000) | (b << 10) | (g << 5) | r
    }

    #[inline]
    pub fn is_inside_draw_area(&self, x: i32, y: i32) -> bool {
        x >= 0
            && x < (VRAM_WIDTH as i32)
            && y >= 0
            && y < (VRAM_HEIGHT as i32)
            && x >= (self.draw_area_left as i32)
            && x <= (self.draw_area_right as i32)
            && y >= (self.draw_area_top as i32)
            && y <= (self.draw_area_bottom as i32)
    }

    #[inline]
    pub fn apply_tex_window(&self, u: u8, v: u8) -> (u8, u8) {
        let mx = self.tex_window_mask_x.wrapping_mul(8);
        let my = self.tex_window_mask_y.wrapping_mul(8);
        let ox = (self.tex_window_offset_x & self.tex_window_mask_x).wrapping_mul(8);
        let oy = (self.tex_window_offset_y & self.tex_window_mask_y).wrapping_mul(8);
        ((u & !mx) | ox, (v & !my) | oy)
    }

    #[inline]
    pub fn update_texpage_from_polygon(&mut self, texpage: u16) {
        let low9 = (texpage & 0x01FF) as u32;
        let tex_disable = (((texpage >> 11) & 1) as u32) << 15;
        self.gpustat = (self.gpustat & !(0x01FF | (1 << 15))) | low9 | tex_disable;
        self.draw_mode = (self.draw_mode & !(0x01FF | (1 << 11)))
            | low9
            | ((((texpage >> 11) & 1) as u32) << 11);
    }

    #[inline]
    pub fn sample_texture(&self, raw_u: u8, raw_v: u8, clut: u16) -> u16 {
        let (u, v) = self.apply_tex_window(raw_u, raw_v);
        let texbase_x = ((self.gpustat & 0x0F) as usize) * 64;
        let texbase_y = (((self.gpustat >> 4) & 1) as usize) * 256;
        let depth = (self.gpustat >> 7) & 0x03;
        let clut_x = ((clut & 0x3F) as usize) * 16;
        let clut_y = ((clut >> 6) & 0x1FF) as usize;

        match depth {
            0 => {
                let tx = (texbase_x + (u as usize) / 4) & 0x3FF;
                let ty = (texbase_y + (v as usize)) & 0x1FF;
                let word = self.vram[ty * VRAM_WIDTH + tx];
                let index = ((word >> ((u & 3) * 4)) & 0x0F) as usize;
                self.vram[(clut_y & 0x1FF) * VRAM_WIDTH + ((clut_x + index) & 0x3FF)]
            }
            1 => {
                let tx = (texbase_x + (u as usize) / 2) & 0x3FF;
                let ty = (texbase_y + (v as usize)) & 0x1FF;
                let word = self.vram[ty * VRAM_WIDTH + tx];
                let index = ((word >> ((u & 1) * 8)) & 0xFF) as usize;
                self.vram[(clut_y & 0x1FF) * VRAM_WIDTH + ((clut_x + index) & 0x3FF)]
            }
            _ => {
                let tx = (texbase_x + (u as usize)) & 0x3FF;
                let ty = (texbase_y + (v as usize)) & 0x1FF;
                self.vram[ty * VRAM_WIDTH + tx]
            }
        }
    }

    #[inline]
    pub fn modulate_texel(texel: u16, r8: u8, g8: u8, b8: u8) -> u16 {
        let rt = (texel & 0x1F) as u32;
        let gt = ((texel >> 5) & 0x1F) as u32;
        let bt = ((texel >> 10) & 0x1F) as u32;
        let r_out = ((rt * (r8 as u32)) >> 7).min(31) as u16;
        let g_out = ((gt * (g8 as u32)) >> 7).min(31) as u16;
        let b_out = ((bt * (b8 as u32)) >> 7).min(31) as u16;
        (texel & 0x8000) | (b_out << 10) | (g_out << 5) | r_out
    }

    #[inline]
    pub fn blend_semi_transparent(bg: u16, fg: u16, abr: u8) -> u16 {
        let blend_ch = |b: i32, f: i32| -> u16 {
            let res = match abr & 0x03 {
                0 => (b + f) >> 1,
                1 => (b + f).min(31),
                2 => (b - f).max(0),
                3 => (b + (f >> 2)).min(31),
                _ => f,
            };
            res.clamp(0, 31) as u16
        };
        let r = blend_ch((bg & 0x1F) as i32, (fg & 0x1F) as i32);
        let g = blend_ch(((bg >> 5) & 0x1F) as i32, ((fg >> 5) & 0x1F) as i32);
        let b = blend_ch(((bg >> 10) & 0x1F) as i32, ((fg >> 10) & 0x1F) as i32);
        (fg & 0x8000) | (b << 10) | (g << 5) | r
    }

    #[inline]
    fn plot_raster_pixel(&mut self, x: i32, y: i32, fg: u16, blend: bool) {
        if !self.is_inside_draw_area(x, y) {
            return;
        }
        let ux = x as usize;
        let uy = y as usize;
        let out_pixel = if blend {
            let bg = self.vram[(uy & 0x1FF) * VRAM_WIDTH + (ux & 0x3FF)];
            let abr = ((self.gpustat >> 5) & 0x03) as u8;
            Self::blend_semi_transparent(bg, fg, abr)
        } else {
            fg
        };
        self.write_vram_pixel_masked(ux, uy, out_pixel);
    }

    #[allow(clippy::too_many_arguments)]
    fn rasterize_triangle(
        &mut self,
        v0: PolyVertex,
        mut v1: PolyVertex,
        mut v2: PolyVertex,
        shaded: bool,
        textured: bool,
        semi_transparent: bool,
        raw_texture: bool,
        skip_shared_v0_v1: bool,
    ) -> bool {
        let min_x_raw = v0.x.min(v1.x).min(v2.x);
        let max_x_raw = v0.x.max(v1.x).max(v2.x);
        let min_y_raw = v0.y.min(v1.y).min(v2.y);
        let max_y_raw = v0.y.max(v1.y).max(v2.y);
        if (max_x_raw - min_x_raw) > 1023 || (max_y_raw - min_y_raw) > 511 {
            return false;
        }

        let mut area = (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
        if area == 0 {
            return false;
        }
        let mut swapped = false;
        if area < 0 {
            std::mem::swap(&mut v1, &mut v2);
            area = -area;
            swapped = true;
        }

        let clip_left = (self.draw_area_left as i32).max(0);
        let clip_top = (self.draw_area_top as i32).max(0);
        let clip_right = (self.draw_area_right as i32).min((VRAM_WIDTH as i32) - 1);
        let clip_bottom = (self.draw_area_bottom as i32).min((VRAM_HEIGHT as i32) - 1);
        let min_x = min_x_raw.max(clip_left);
        let max_x = max_x_raw.min(clip_right);
        let min_y = min_y_raw.max(clip_top);
        let max_y = max_y_raw.min(clip_bottom);
        if min_x > max_x || min_y > max_y {
            return true;
        }

        let area64 = area as i64;
        let clut = self.active_clut;

        for py in min_y..=max_y {
            for px in min_x..=max_x {
                let w0 = (v2.x - v1.x) * (py - v1.y) - (v2.y - v1.y) * (px - v1.x);
                let w1 = (v0.x - v2.x) * (py - v2.y) - (v0.y - v2.y) * (px - v2.x);
                let w2 = (v1.x - v0.x) * (py - v0.y) - (v1.y - v0.y) * (px - v0.x);
                if w0 < 0 || w1 < 0 || w2 < 0 {
                    continue;
                }
                let shared_w = if swapped { w1 } else { w2 };
                if skip_shared_v0_v1 && shared_w == 0 {
                    continue;
                }

                let interp = |a0: u8, a1: u8, a2: u8| -> u8 {
                    let num = (w0 as i64) * (a0 as i64)
                        + (w1 as i64) * (a1 as i64)
                        + (w2 as i64) * (a2 as i64)
                        + (area64 / 2);
                    (num / area64).clamp(0, 255) as u8
                };

                let (r8, g8, b8) = if shaded {
                    (interp(v0.r, v1.r, v2.r), interp(v0.g, v1.g, v2.g), interp(v0.b, v1.b, v2.b))
                } else {
                    (v0.r, v0.g, v0.b)
                };

                if textured {
                    let texel = if self.gpu_tex_bilinear {
                        let interp_fp8 = |a0: u8, a1: u8, a2: u8| -> i32 {
                            let num = ((w0 as i64) * (a0 as i64)
                                + (w1 as i64) * (a1 as i64)
                                + (w2 as i64) * (a2 as i64))
                                << 8;
                            (num / area64).clamp(0, 65535) as i32
                        };
                        let u_fp8 = interp_fp8(v0.u, v1.u, v2.u);
                        let v_fp8 = interp_fp8(v0.v, v1.v, v2.v);
                        self.sample_texture_bilinear(u_fp8, v_fp8, clut)
                    } else {
                        let u = interp(v0.u, v1.u, v2.u);
                        let v = interp(v0.v, v1.v, v2.v);
                        self.sample_texture(u, v, clut)
                    };
                    if texel == 0x0000 {
                        continue;
                    }
                    let fg = if raw_texture { texel } else { Self::modulate_texel(texel, r8, g8, b8) };
                    let do_blend = semi_transparent && ((texel & 0x8000) != 0);
                    self.plot_raster_pixel(px, py, fg, do_blend);
                } else {
                    let do_dither = self.gpu_dither_enabled && ((self.gpustat & (1 << 9)) != 0) && shaded;
                    let fg = if do_dither {
                        Self::dither_rgb888_to_bgr555(px, py, r8, g8, b8)
                    } else {
                        Self::rgb888_to_bgr555(r8, g8, b8)
                    };
                    self.plot_raster_pixel(px, py, fg, semi_transparent);
                }
            }
        }
        true
    }

    fn rasterize_line(&mut self, v0: PolyVertex, v1: PolyVertex, shaded: bool, semi_transparent: bool) {
        let dx = (v1.x - v0.x).abs();
        let dy = (v1.y - v0.y).abs();
        if dx > 1023 || dy > 511 {
            return;
        }
        let steps = dx.max(dy);
        if steps == 0 {
            let fg = Self::rgb888_to_bgr555(v0.r, v0.g, v0.b);
            self.plot_raster_pixel(v0.x, v0.y, fg, semi_transparent);
            return;
        }
        let sx = v1.x - v0.x;
        let sy = v1.y - v0.y;
        for i in 0..=steps {
            let rx = if sx >= 0 { steps / 2 } else { -(steps / 2) };
            let ry = if sy >= 0 { steps / 2 } else { -(steps / 2) };
            let px = v0.x + (sx * i + rx) / steps;
            let py = v0.y + (sy * i + ry) / steps;
            let (r8, g8, b8) = if shaded {
                let interp_c = |c0: u8, c1: u8| -> u8 {
                    let dc = (c1 as i32) - (c0 as i32);
                    let rc = if dc >= 0 { steps / 2 } else { -(steps / 2) };
                    ((c0 as i32) + (dc * i + rc) / steps).clamp(0, 255) as u8
                };
                (interp_c(v0.r, v1.r), interp_c(v0.g, v1.g), interp_c(v0.b, v1.b))
            } else {
                (v0.r, v0.g, v0.b)
            };
            let fg = Self::rgb888_to_bgr555(r8, g8, b8);
            self.plot_raster_pixel(px, py, fg, semi_transparent);
        }
    }

    fn execute_polyline(&mut self, shaded: bool) {
        if self.gp0_fifo.len() < 3 {
            return;
        }
        let w0 = self.gp0_fifo[0];
        let cmd = (w0 >> 24) as u8;
        let semi_transparent = (cmd & 0x02) != 0;
        let end_idx = self.gp0_fifo.len() - 1;

        if !shaded {
            let (r0, g0, b0) = Self::decode_rgb(w0);
            let mut vertices = Vec::with_capacity(end_idx.saturating_sub(1));
            for i in 1..end_idx {
                let (x, y) = self.decode_vertex(self.gp0_fifo[i]);
                vertices.push(PolyVertex { x, y, r: r0, g: g0, b: b0, u: 0, v: 0 });
            }
            for pair in vertices.windows(2) {
                self.rasterize_line(pair[0], pair[1], false, semi_transparent);
            }
        } else {
            let mut vertices = Vec::new();
            let mut idx = 0;
            while idx + 1 < end_idx {
                let (r, g, b) = Self::decode_rgb(self.gp0_fifo[idx]);
                let (x, y) = self.decode_vertex(self.gp0_fifo[idx + 1]);
                vertices.push(PolyVertex { x, y, r, g, b, u: 0, v: 0 });
                idx += 2;
            }
            for pair in vertices.windows(2) {
                self.rasterize_line(pair[0], pair[1], true, semi_transparent);
            }
        }
    }

    fn execute_polygon_command(&mut self, cmd: u8) {
        let shaded = (cmd & 0x10) != 0;
        let is_quad = (cmd & 0x08) != 0;
        let textured = (cmd & 0x04) != 0;
        let semi_transparent = (cmd & 0x02) != 0;
        let raw_texture = textured && ((cmd & 0x01) != 0);

        let w0 = self.gp0_fifo[0];
        let (r0, g0, b0) = Self::decode_rgb(w0);
        let mut verts = [PolyVertex::default(); 4];
        let v_count = if is_quad { 4 } else { 3 };

        match (shaded, textured) {
            (false, false) => {
                for i in 0..v_count {
                    let (x, y) = self.decode_vertex(self.gp0_fifo[1 + i]);
                    verts[i] = PolyVertex { x, y, r: r0, g: g0, b: b0, u: 0, v: 0 };
                }
            }
            (false, true) => {
                let clut_word = (self.gp0_fifo[2] >> 16) as u16;
                let texpage_word = (self.gp0_fifo[4] >> 16) as u16;
                self.active_clut = clut_word;
                self.update_texpage_from_polygon(texpage_word);
                for i in 0..v_count {
                    let (x, y) = self.decode_vertex(self.gp0_fifo[1 + i * 2]);
                    let (u, v) = Self::decode_uv(self.gp0_fifo[2 + i * 2]);
                    verts[i] = PolyVertex { x, y, r: r0, g: g0, b: b0, u, v };
                }
            }
            (true, false) => {
                for i in 0..v_count {
                    let (r, g, b) = Self::decode_rgb(self.gp0_fifo[i * 2]);
                    let (x, y) = self.decode_vertex(self.gp0_fifo[1 + i * 2]);
                    verts[i] = PolyVertex { x, y, r, g, b, u: 0, v: 0 };
                }
            }
            (true, true) => {
                let clut_word = (self.gp0_fifo[2] >> 16) as u16;
                let texpage_word = (self.gp0_fifo[5] >> 16) as u16;
                self.active_clut = clut_word;
                self.update_texpage_from_polygon(texpage_word);
                for i in 0..v_count {
                    let (r, g, b) = Self::decode_rgb(self.gp0_fifo[i * 3]);
                    let (x, y) = self.decode_vertex(self.gp0_fifo[1 + i * 3]);
                    let (u, v) = Self::decode_uv(self.gp0_fifo[2 + i * 3]);
                    verts[i] = PolyVertex { x, y, r, g, b, u, v };
                }
            }
        }

        let drew_first = self.rasterize_triangle(
            verts[0], verts[1], verts[2], shaded, textured, semi_transparent, raw_texture, false,
        );
        if is_quad {
            let _ = self.rasterize_triangle(
                verts[1], verts[2], verts[3], shaded, textured, semi_transparent, raw_texture, drew_first,
            );
        }
    }

    fn execute_line_command(&mut self, cmd: u8) {
        let shaded = (cmd & 0x10) != 0;
        let semi_transparent = (cmd & 0x02) != 0;
        let (r0, g0, b0) = Self::decode_rgb(self.gp0_fifo[0]);
        if !shaded {
            let (x0, y0) = self.decode_vertex(self.gp0_fifo[1]);
            let (x1, y1) = self.decode_vertex(self.gp0_fifo[2]);
            let v0 = PolyVertex { x: x0, y: y0, r: r0, g: g0, b: b0, u: 0, v: 0 };
            let v1 = PolyVertex { x: x1, y: y1, r: r0, g: g0, b: b0, u: 0, v: 0 };
            self.rasterize_line(v0, v1, false, semi_transparent);
        } else {
            let (x0, y0) = self.decode_vertex(self.gp0_fifo[1]);
            let (r1, g1, b1) = Self::decode_rgb(self.gp0_fifo[2]);
            let (x1, y1) = self.decode_vertex(self.gp0_fifo[3]);
            let v0 = PolyVertex { x: x0, y: y0, r: r0, g: g0, b: b0, u: 0, v: 0 };
            let v1 = PolyVertex { x: x1, y: y1, r: r1, g: g1, b: b1, u: 0, v: 0 };
            self.rasterize_line(v0, v1, true, semi_transparent);
        }
    }

    fn execute_rect_command(&mut self, cmd: u8) {
        let size_code = (cmd >> 3) & 0x03;
        let textured = (cmd & 0x04) != 0;
        let semi_transparent = (cmd & 0x02) != 0;
        let raw_texture = textured && ((cmd & 0x01) != 0);

        let (r0, g0, b0) = Self::decode_rgb(self.gp0_fifo[0]);
        let (x0, y0) = self.decode_vertex(self.gp0_fifo[1]);

        let (u0, v0, clut, size_word_idx) = if textured {
            let w2 = self.gp0_fifo[2];
            let (u, v) = Self::decode_uv(w2);
            let c = (w2 >> 16) as u16;
            self.active_clut = c;
            (u, v, c, 3usize)
        } else {
            (0u8, 0u8, self.active_clut, 2usize)
        };

        let (width, height): (i32, i32) = match size_code {
            0 => {
                let sw = self.gp0_fifo[size_word_idx];
                ((sw & 0x3FF) as i32, ((sw >> 16) & 0x1FF) as i32)
            }
            1 => (1, 1),
            2 => (8, 8),
            _ => (16, 16),
        };
        if width <= 0 || height <= 0 {
            return;
        }

        let flat_fg = Self::rgb888_to_bgr555(r0, g0, b0);
        let x_flip = self.tex_rect_x_flip;
        let y_flip = self.tex_rect_y_flip;

        for dy in 0..height {
            let py = y0 + dy;
            if py < 0
                || py >= (VRAM_HEIGHT as i32)
                || py < (self.draw_area_top as i32)
                || py > (self.draw_area_bottom as i32)
            {
                continue;
            }
            let v = if y_flip { v0.wrapping_sub(dy as u8) } else { v0.wrapping_add(dy as u8) };
            for dx in 0..width {
                let px = x0 + dx;
                if px < 0
                    || px >= (VRAM_WIDTH as i32)
                    || px < (self.draw_area_left as i32)
                    || px > (self.draw_area_right as i32)
                {
                    continue;
                }
                if textured {
                    let u = if x_flip { u0.wrapping_sub(dx as u8) } else { u0.wrapping_add(dx as u8) };
                    let texel = self.sample_texture(u, v, clut);
                    if texel == 0x0000 {
                        continue;
                    }
                    let fg = if raw_texture { texel } else { Self::modulate_texel(texel, r0, g0, b0) };
                    let do_blend = semi_transparent && ((texel & 0x8000) != 0);
                    self.plot_raster_pixel(px, py, fg, do_blend);
                } else {
                    self.plot_raster_pixel(px, py, flat_fg, semi_transparent);
                }
            }
        }
    }
    #[inline]
    pub fn update_gpustat_dma_request(&mut self) {
        let dma_dir = (self.gpustat >> 29) & 0b11;
        let req_bit = match dma_dir {
            0 => 0,
            1 => u32::from(self.gp0_fifo.len() < GP0_FIFO_CAPACITY),
            2 => (self.gpustat >> 28) & 1,
            3 => (self.gpustat >> 27) & 1,
            _ => 0,
        };
        if req_bit != 0 {
            self.gpustat |= 1 << 25;
        } else {
            self.gpustat &= !(1 << 25);
        }
    }

    #[inline]
    pub fn read_gpuread(&mut self) -> u32 {
        if let Some(word) = self.vram_read_queue.pop_front() {
            self.gpuread_latch = word;
            if self.vram_read_queue.is_empty() {
                self.gpustat &= !(1 << 27);
            }
            self.update_gpustat_dma_request();
        }
        self.gpuread_latch
    }

    pub fn send_gp0(&mut self, word: u32) {
        if self.gp0_command_log.len() >= 1024 {
            self.gp0_command_log.clear();
        }
        self.gp0_command_log.push(word);

        match self.gp0_mode {
            Gp0Mode::CpuToVram(mut xfer) => {
                let pixels = [(word & 0xFFFF) as u16, (word >> 16) as u16];
                for &px in &pixels {
                    if xfer.cur_y < xfer.height {
                        let vx = (xfer.dst_x as u32 + xfer.cur_x) as usize & 0x3FF;
                        let vy = (xfer.dst_y as u32 + xfer.cur_y) as usize & 0x1FF;
                        self.write_vram_pixel_masked(vx, vy, px);
                        xfer.cur_x += 1;
                        if xfer.cur_x >= xfer.width {
                            xfer.cur_x = 0;
                            xfer.cur_y += 1;
                        }
                    }
                }
                xfer.words_remaining = xfer.words_remaining.saturating_sub(1);
                if xfer.words_remaining == 0 {
                    self.gp0_mode = Gp0Mode::Command;
                    self.gp0_fifo.clear();
                    self.gp0_words_remaining = 0;
                } else {
                    self.gp0_mode = Gp0Mode::CpuToVram(xfer);
                }
                self.update_gpustat_dma_request();
                return;
            }
            Gp0Mode::PolyLine { shaded } => {
                self.gp0_fifo.push(word);
                let len = self.gp0_fifo.len();
                let can_terminate = if shaded {
                    len >= 4 && (len == 4 || (len % 2 == 1))
                } else {
                    len >= 3
                };
                if can_terminate && (word & 0xF000_F000) == 0x5000_5000 {
                    self.execute_polyline(shaded);
                    self.gp0_fifo.clear();
                    self.gp0_words_remaining = 0;
                    self.gp0_mode = Gp0Mode::Command;
                }
                self.update_gpustat_dma_request();
                return;
            }
            Gp0Mode::Command => {}
        }

        if self.gp0_fifo.is_empty() {
            let cmd = (word >> 24) as u8;
            match cmd {
                0x48..=0x4F => {
                    self.gp0_fifo.push(word);
                    self.gp0_mode = Gp0Mode::PolyLine { shaded: false };
                    self.update_gpustat_dma_request();
                    return;
                }
                0x58..=0x5F => {
                    self.gp0_fifo.push(word);
                    self.gp0_mode = Gp0Mode::PolyLine { shaded: true };
                    self.update_gpustat_dma_request();
                    return;
                }
                _ => {
                    let total = Self::gp0_command_total_words(cmd);
                    if total <= 1 {
                        self.gp0_fifo.push(word);
                        self.execute_gp0_command();
                        self.gp0_fifo.clear();
                        self.gp0_words_remaining = 0;
                    } else {
                        self.gp0_fifo.push(word);
                        self.gp0_words_remaining = total - 1;
                    }
                    self.update_gpustat_dma_request();
                    return;
                }
            }
        }

        self.gp0_fifo.push(word);
        if self.gp0_words_remaining > 0 {
            self.gp0_words_remaining -= 1;
        }
        if self.gp0_words_remaining == 0 {
            self.execute_gp0_command();
            self.gp0_fifo.clear();
        }
        self.update_gpustat_dma_request();
    }

    fn execute_gp0_command(&mut self) {
        if self.gp0_fifo.is_empty() {
            return;
        }
        let w0 = self.gp0_fifo[0];
        let cmd = (w0 >> 24) as u8;

        match cmd {
            0x00 | 0x03..=0x1E | 0xE0 | 0xE7..=0xFF => {}
            0x01 => {}
            0x02 => {
                let w1 = self.gp0_fifo[1];
                let w2 = self.gp0_fifo[2];
                let color15 = Self::rgb24_to_bgr555(w0 & 0x00FF_FFFF);
                let x0 = (w1 & 0x3F0) as usize;
                let y0 = ((w1 >> 16) & 0x1FF) as usize;
                let xsiz = (((w2 & 0x3FF) + 0x0F) & !0x0F) as usize;
                let ysiz = ((w2 >> 16) & 0x1FF) as usize;

                for dy in 0..ysiz {
                    let y = (y0 + dy) & 0x1FF;
                    let row_base = y * VRAM_WIDTH;
                    for dx in 0..xsiz {
                        let x = (x0 + dx) & 0x3FF;
                        self.vram[row_base + x] = color15;
                    }
                }
            }
            0x1F => {
                self.gpustat |= 1 << 24;
                self.i_stat |= 1 << 1;
            }
            0x20..=0x3F => {
                self.execute_polygon_command(cmd);
            }
            0x40..=0x47 | 0x50..=0x57 => {
                self.execute_line_command(cmd);
            }
            0x60..=0x7F => {
                self.execute_rect_command(cmd);
            }
            0x80..=0x9F => {
                let w1 = self.gp0_fifo[1];
                let w2 = self.gp0_fifo[2];
                let w3 = self.gp0_fifo[3];
                let src_x = (w1 & 0x3FF) as usize;
                let src_y = ((w1 >> 16) & 0x1FF) as usize;
                let dst_x = (w2 & 0x3FF) as usize;
                let dst_y = ((w2 >> 16) & 0x1FF) as usize;
                let raw_w = (w3 & 0x3FF) as usize;
                let raw_h = ((w3 >> 16) & 0x1FF) as usize;
                let width = if raw_w == 0 { 0x400 } else { raw_w };
                let height = if raw_h == 0 { 0x200 } else { raw_h };

                let mut temp = Vec::with_capacity(width * height);
                for dy in 0..height {
                    let sy = (src_y + dy) & 0x1FF;
                    for dx in 0..width {
                        let sx = (src_x + dx) & 0x3FF;
                        temp.push(self.vram[sy * VRAM_WIDTH + sx]);
                    }
                }
                for dy in 0..height {
                    let dy_vram = (dst_y + dy) & 0x1FF;
                    for dx in 0..width {
                        let dx_vram = (dst_x + dx) & 0x3FF;
                        let px = temp[dy * width + dx];
                        self.write_vram_pixel_masked(dx_vram, dy_vram, px);
                    }
                }
            }
            0xA0..=0xBF => {
                let w1 = self.gp0_fifo[1];
                let w2 = self.gp0_fifo[2];
                let dst_x = (w1 & 0x3FF) as u16;
                let dst_y = ((w1 >> 16) & 0x1FF) as u16;
                let raw_w = w2 & 0x3FF;
                let raw_h = (w2 >> 16) & 0x1FF;
                let width = if raw_w == 0 { 0x400 } else { raw_w };
                let height = if raw_h == 0 { 0x200 } else { raw_h };
                let words_remaining = (width * height + 1) / 2;
                self.gp0_mode = Gp0Mode::CpuToVram(CpuToVramTransfer {
                    dst_x,
                    dst_y,
                    width,
                    height,
                    cur_x: 0,
                    cur_y: 0,
                    words_remaining,
                });
            }
            0xC0..=0xDF => {
                let w1 = self.gp0_fifo[1];
                let w2 = self.gp0_fifo[2];
                let src_x = (w1 & 0x3FF) as usize;
                let src_y = ((w1 >> 16) & 0x1FF) as usize;
                let raw_w = (w2 & 0x3FF) as usize;
                let raw_h = ((w2 >> 16) & 0x1FF) as usize;
                let width = if raw_w == 0 { 0x400 } else { raw_w };
                let height = if raw_h == 0 { 0x200 } else { raw_h };

                self.vram_read_queue.clear();
                let mut pixels = Vec::with_capacity(width * height);
                for dy in 0..height {
                    let sy = (src_y + dy) & 0x1FF;
                    for dx in 0..width {
                        let sx = (src_x + dx) & 0x3FF;
                        pixels.push(self.vram[sy * VRAM_WIDTH + sx]);
                    }
                }
                for chunk in pixels.chunks(2) {
                    let lo = chunk[0] as u32;
                    let hi = if chunk.len() > 1 { chunk[1] as u32 } else { 0 };
                    self.vram_read_queue.push_back(lo | (hi << 16));
                }
                if let Some(&first) = self.vram_read_queue.front() {
                    self.gpuread_latch = first;
                    self.gpustat |= 1 << 27;
                }
                self.update_gpustat_dma_request();
            }
            0xE1 => {
                self.draw_mode = w0 & 0x00FF_FFFF;
                let low11 = w0 & 0x7FF;
                let tex_disable = ((w0 >> 11) & 1) << 15;
                self.tex_rect_x_flip = ((w0 >> 12) & 1) != 0;
                self.tex_rect_y_flip = ((w0 >> 13) & 1) != 0;
                self.gpustat = (self.gpustat & !(0x7FF | (1 << 15))) | low11 | tex_disable;
            }
            0xE2 => {
                self.tex_window = w0 & 0x000F_FFFF;
                self.tex_window_mask_x = (w0 & 0x1F) as u8;
                self.tex_window_mask_y = ((w0 >> 5) & 0x1F) as u8;
                self.tex_window_offset_x = ((w0 >> 10) & 0x1F) as u8;
                self.tex_window_offset_y = ((w0 >> 15) & 0x1F) as u8;
            }
            0xE3 => {
                self.draw_area_top_left = w0 & 0x000F_FFFF;
                self.draw_area_left = (w0 & 0x3FF) as u16;
                self.draw_area_top = ((w0 >> 10) & 0x1FF) as u16;
            }
            0xE4 => {
                self.draw_area_bottom_right = w0 & 0x000F_FFFF;
                self.draw_area_right = (w0 & 0x3FF) as u16;
                self.draw_area_bottom = ((w0 >> 10) & 0x1FF) as u16;
            }
            0xE5 => {
                self.draw_offset_raw = w0 & 0x003F_FFFF;
                let raw_x = (w0 & 0x7FF) as i16;
                let raw_y = ((w0 >> 11) & 0x7FF) as i16;
                self.draw_offset_x = (raw_x << 5) >> 5;
                self.draw_offset_y = (raw_y << 5) >> 5;
            }
            0xE6 => {
                self.force_mask_bit = (w0 & 1) != 0;
                self.check_mask_bit = (w0 & 2) != 0;
                self.gpustat = (self.gpustat & !(0b11 << 11)) | ((w0 & 0b11) << 11);
            }
            _ => {}
        }
    }

    pub fn send_gp1(&mut self, word: u32) {
        let cmd = (word >> 24) & 0x3F;
        let param = word & 0x00FF_FFFF;
        match cmd {
            0x00 => {
                self.gp0_fifo.clear();
                self.gp0_words_remaining = 0;
                self.gp0_mode = Gp0Mode::Command;
                self.vram_read_queue.clear();
                self.draw_mode = 0;
                self.active_clut = 0;
                self.tex_rect_x_flip = false;
                self.tex_rect_y_flip = false;
                self.tex_window = 0;
                self.tex_window_mask_x = 0;
                self.tex_window_mask_y = 0;
                self.tex_window_offset_x = 0;
                self.tex_window_offset_y = 0;
                self.draw_area_top_left = 0;
                self.draw_area_bottom_right = 0;
                self.draw_area_left = 0;
                self.draw_area_top = 0;
                self.draw_area_right = (VRAM_WIDTH - 1) as u16;
                self.draw_area_bottom = (VRAM_HEIGHT - 1) as u16;
                self.draw_offset_raw = 0;
                self.draw_offset_x = 0;
                self.draw_offset_y = 0;
                self.force_mask_bit = false;
                self.check_mask_bit = false;
                self.display_vram_x = 0;
                self.display_vram_y = 0;
                self.display_width = 320;
                self.display_height = 240;
                self.display_horiz_start = 0x200;
                self.display_horiz_end = 0xC00;
                self.display_vert_start = 0x010;
                self.display_vert_end = 0x100;
                self.gpustat = 0x1480_2000;
            }
            0x01 => {
                self.gp0_fifo.clear();
                self.gp0_words_remaining = 0;
                self.gp0_mode = Gp0Mode::Command;
                self.gpustat |= (1 << 26) | (1 << 28);
                self.update_gpustat_dma_request();
            }
            0x02 => {
                self.gpustat &= !(1 << 24);
            }
            0x03 => {
                if (param & 1) != 0 {
                    self.gpustat |= 1 << 23;
                } else {
                    self.gpustat &= !(1 << 23);
                }
            }
            0x04 => {
                self.gpustat = (self.gpustat & !(0b11 << 29)) | ((param & 0b11) << 29);
                self.update_gpustat_dma_request();
            }
            0x05 => {
                self.display_vram_x = (param & 0x3FF) as u16;
                self.display_vram_y = ((param >> 10) & 0x1FF) as u16;
            }
            0x06 => {
                self.display_horiz_start = (param & 0xFFF) as u16;
                self.display_horiz_end = ((param >> 12) & 0xFFF) as u16;
            }
            0x07 => {
                self.display_vert_start = (param & 0x3FF) as u16;
                self.display_vert_end = ((param >> 10) & 0x3FF) as u16;
            }
            0x08 => {
                let hres1 = (param & 0x03) << 17;
                let vres = ((param >> 2) & 1) << 19;
                let pal = ((param >> 3) & 1) << 20;
                let depth24 = ((param >> 4) & 1) << 21;
                let interlace = ((param >> 5) & 1) << 22;
                let hres2 = ((param >> 6) & 1) << 16;
                let reverse = ((param >> 7) & 1) << 14;
                let clear_mask = (0x7F << 16) | (1 << 14);
                self.gpustat = (self.gpustat & !clear_mask)
                    | hres1
                    | vres
                    | pal
                    | depth24
                    | interlace
                    | hres2
                    | reverse;
                if interlace == 0 {
                    self.gpustat |= 1 << 13;
                }

                if (param & (1 << 6)) != 0 {
                    self.display_width = 368;
                } else {
                    self.display_width = match param & 0x03 {
                        0b00 => 256,
                        0b01 => 320,
                        0b10 => 512,
                        0b11 => 640,
                        _ => unreachable!(),
                    };
                }
                self.display_height = if (param & (1 << 2)) != 0 {
                    480
                } else {
                    240
                };
            }
            0x09 => {
                self.vram_2mb_mode = (param & 1) != 0;
            }
            0x10..=0x1F => {
                match param & 0x0F {
                    0x02 => self.gpuread_latch = self.tex_window & 0x000F_FFFF,
                    0x03 => self.gpuread_latch = self.draw_area_top_left & 0x000F_FFFF,
                    0x04 => self.gpuread_latch = self.draw_area_bottom_right & 0x000F_FFFF,
                    0x05 => self.gpuread_latch = self.draw_offset_raw & 0x003F_FFFF,
                    0x07 => self.gpuread_latch = 2,
                    0x08 => self.gpuread_latch = 0,
                    other => match other & 0x07 {
                        0x02 => self.gpuread_latch = self.tex_window & 0x000F_FFFF,
                        0x03 => self.gpuread_latch = self.draw_area_top_left & 0x000F_FFFF,
                        0x04 => self.gpuread_latch = self.draw_area_bottom_right & 0x000F_FFFF,
                        0x05 => self.gpuread_latch = self.draw_offset_raw & 0x003F_FFFF,
                        0x07 => self.gpuread_latch = 2,
                        _ => {}
                    },
                }
            }
            _ => {}
        }
    }

    pub fn update_dicr_irq(&mut self, prev_master: bool) {
        let force_irq = (self.dicr & (1 << 15)) != 0;
        let master_enable = (self.dicr & (1 << 23)) != 0;
        let ch_enables = (self.dicr >> 16) & 0x7F;
        let ch_flags = (self.dicr >> 24) & 0x7F;
        let new_master = force_irq || (master_enable && ((ch_flags & ch_enables) != 0));
        if new_master {
            self.dicr |= 1 << 31;
            if !prev_master {
                self.i_stat |= 1 << 3;
            }
        } else {
            self.dicr &= !(1 << 31);
        }
    }

    fn write_dicr(&mut self, val: u32) {
        let prev_master = (self.dicr & (1 << 31)) != 0;
        let rw_mask = 0x00FF_803F;
        let ack_flags = val & 0x7F00_0000;
        let current_flags = (self.dicr & 0x7F00_0000) & !ack_flags;
        self.dicr = (val & rw_mask) | current_flags;
        self.update_dicr_irq(prev_master);
    }

    pub fn check_and_run_dma(&mut self, ch: usize) {
        if (self.dpcr & (1 << (ch * 4 + 3))) == 0 {
            return;
        }
        let chcr = self.dma[ch].chcr;
        if (chcr & (1 << 24)) == 0 {
            return;
        }
        let sync_mode = (chcr >> 9) & 0x03;
        if sync_mode == 0 && (chcr & (1 << 28)) == 0 {
            return;
        }
        self.execute_dma_channel(ch);
    }

    fn execute_dma_channel(&mut self, ch: usize) {
        if ch == 6 {
            let mut addr = self.dma[6].madr & 0x00FF_FFFC;
            let bc = self.dma[6].bcr & 0xFFFF;
            let mut words = if bc == 0 { 0x10000u32 } else { bc };

            while words > 1 {
                let prev_addr = addr.wrapping_sub(4) & 0x00FF_FFFC;
                self.write_ram_u32(addr, prev_addr);
                addr = prev_addr;
                words -= 1;
            }
            self.write_ram_u32(addr, 0x00FF_FFFF);
            self.dma[6].madr = addr;
            self.dma[6].bcr &= !0xFFFF;
            self.dma[6].chcr &= !((1 << 24) | (1 << 28));
            self.complete_dma_channel(6);
            return;
        }

        let chcr = self.dma[ch].chcr;
        let dir_from_ram = (chcr & 1) != 0;
        let step: i32 = if (chcr & 2) != 0 { -4 } else { 4 };
        let sync_mode = (chcr >> 9) & 0x03;
        let mut addr = self.dma[ch].madr & 0x00FF_FFFC;

        match sync_mode {
            0 => {
                let bc = self.dma[ch].bcr & 0xFFFF;
                let words = if bc == 0 { 0x10000u32 } else { bc };
                for _ in 0..words {
                    self.dma_transfer_word(ch, addr, dir_from_ram);
                    addr = (addr as i32).wrapping_add(step) as u32 & 0x00FF_FFFC;
                }
                self.dma[ch].madr = addr;
                self.dma[ch].bcr &= !0xFFFF;
            }
            1 => {
                let bs = self.dma[ch].bcr & 0xFFFF;
                let ba = (self.dma[ch].bcr >> 16) & 0xFFFF;
                let block_size = if bs == 0 { 0x10000u32 } else { bs };
                let block_count = if ba == 0 { 0x10000u32 } else { ba };
                for block in 0..block_count {
                    // Request-mode MDEC output must wait for complete blocks;
                    // an empty decoder is not a stream of zero-valued pixels.
                    if ch == 1 && !dir_from_ram
                        && (!self.mdec_enable_out || self.mdec_out_fifo.len() < block_size as usize) {
                        self.dma[ch].madr = addr;
                        self.dma[ch].bcr = ((block_count - block) << 16) | bs;
                        return;
                    }
                    for _ in 0..block_size {
                        self.dma_transfer_word(ch, addr, dir_from_ram);
                        addr = (addr as i32).wrapping_add(step) as u32 & 0x00FF_FFFC;
                    }
                }
                self.dma[ch].madr = addr;
                self.dma[ch].bcr &= 0xFFFF;
            }
            2 => {
                let mut nodes_visited = 0usize;
                loop {
                    let header = self.read_ram_u32(addr);
                        let word_count = header >> 24;
                    for i in 0..word_count {
                        let cmd_addr = addr.wrapping_add((i + 1) * 4) & 0x00FF_FFFC;
                        self.dma_transfer_word(ch, cmd_addr, true);
                    }
                    let next_addr = header & 0x00FF_FFFF;
                    self.dma[ch].madr = next_addr;
                    nodes_visited += 1;

                    if next_addr == 0x00FF_FFFF
                        || (next_addr & 0x0080_0000) != 0
                        || nodes_visited >= DMA2_LINKED_LIST_SAFETY_CAP
                    {
                        break;
                    }
                    addr = next_addr & 0x00FF_FFFC;
                }
            }
            _ => {}
        }

        self.dma[ch].chcr &= !((1 << 24) | (1 << 28));
        self.complete_dma_channel(ch);
    }

    #[inline]
    fn dma_transfer_word(&mut self, ch: usize, paddr: u32, dir_from_ram: bool) {
        if dir_from_ram {
            let word = self.read_ram_u32(paddr);
            match ch {
                0 => self.write_mdec0(word),
                2 => self.send_gp0(word),
                4 => {
                    self.spu_ram_write_u16((word & 0xFFFF) as u16);
                    self.spu_ram_write_u16(((word >> 16) & 0xFFFF) as u16);
                }
                _ => {}
            }
        } else {
            let word = match ch {
                1 => self.read_mdec0(),
                2 => self.read_gpuread(),
                3 => self.read_cd_data_word(),
                4 => {
                    let lo = self.spu_ram_read_u16() as u32;
                    let hi = self.spu_ram_read_u16() as u32;
                    lo | (hi << 16)
                }
                _ => 0,
            };
            self.write_ram_u32(paddr, word);
        }
    }

    fn complete_dma_channel(&mut self, ch: usize) {
        let prev_master = (self.dicr & (1 << 31)) != 0;
        if (self.dicr & (1 << (16 + ch))) != 0 {
            self.dicr |= 1 << (24 + ch);
        }
        self.update_dicr_irq(prev_master);
        if (self.dicr & (1 << 31)) != 0 {
            self.i_stat |= 1 << 3;
        }
    }

    #[inline]
    pub fn tick(&mut self, cycles: u32) {
        self.vblank_accumulator += cycles;
        self.hblank_accumulator += cycles;

        let hblanks = if self.hblank_accumulator >= CYCLES_PER_SCANLINE || cycles >= CYCLES_PER_FRAME {
            let hb = self.hblank_accumulator / CYCLES_PER_SCANLINE;
            self.hblank_accumulator %= CYCLES_PER_SCANLINE;

            let current_scanline = (self.vblank_accumulator / CYCLES_PER_SCANLINE) % SCANLINES_PER_FRAME;
            if (current_scanline == 240 || cycles >= CYCLES_PER_FRAME) && !self.vblank_triggered {
                self.i_stat |= 1 << 0; // Assert IRQ0 (VBlank)
                self.vblank_triggered = true;
            } else if current_scanline < 240 && cycles < CYCLES_PER_FRAME {
                self.vblank_triggered = false;
            }

            if current_scanline >= 240 {
                self.gpustat |= 1 << 31;
            } else {
                self.gpustat &= !(1 << 31);
            }

            while self.vblank_accumulator >= CYCLES_PER_FRAME {
                self.vblank_accumulator -= CYCLES_PER_FRAME;
            }
            hb
        } else {
            0
        };

        self.step_timer_0(cycles);

        let tmr1_src = (self.timers[1].mode >> 8) & 0x03;
        if tmr1_src == 1 || tmr1_src == 3 {
            if hblanks > 0 {
                self.step_timer(1, hblanks);
            }
        } else {
            self.step_timer_01_fast(1, cycles);
        }

        let tmr2_src = (self.timers[2].mode >> 8) & 0x03;
        if tmr2_src == 2 || tmr2_src == 3 {
            self.timers[2].sub_cycles += cycles;
            if self.timers[2].sub_cycles >= 8 {
                let ticks = self.timers[2].sub_cycles >> 3;
                self.timers[2].sub_cycles &= 7;
                self.step_timer(2, ticks);
            }
        } else {
            self.step_timer(2, cycles);
        }
        if self.sio_ack_delay > 0 {
            if cycles >= self.sio_ack_delay {
                self.sio_ack_delay = 0;
                self.sio_ack_input = true;
                self.sio_ack_pulse_cycles = 96;
                if (self.sio_ctrl & (1 << 12)) != 0 {
                    self.sio_irq_pending = true;
                    self.i_stat |= 1 << 7;
                }
            } else {
                self.sio_ack_delay -= cycles;
            }
        } else if self.sio_ack_pulse_cycles > 0 {
            if cycles >= self.sio_ack_pulse_cycles {
                self.sio_ack_pulse_cycles = 0;
                self.sio_ack_input = false;
            } else {
                self.sio_ack_pulse_cycles -= cycles;
            }
        }
        self.tick_cdrom(cycles);
        self.tick_spu(cycles);
    }

    fn tick_spu(&mut self, cycles: u32) {
        self.spu.cycle_accum += cycles;
        while self.spu.cycle_accum >= 768 {
            self.spu.cycle_accum -= 768;
            self.step_spu_sample();
        }
    }

    #[inline]
    fn read_reverb_ram_i16(&mut self, rel_words: u16, offset_samples: i32) -> i32 {
        let base = (self.spu.reverb_base_addr & 0x7FFFF) as i32;
        let work_size = (0x80000i32 - base).max(2);
        let rel_bytes = (rel_words as i32) * 8 + offset_samples * 2;
        let cur_rel = (self.spu.reverb_curr_addr as i32) - base;
        let mut wrapped = (cur_rel + rel_bytes) % work_size;
        if wrapped < 0 {
            wrapped += work_size;
        }
        let addr = ((base + wrapped) as u32) & 0x7FFFE;
        if (self.spu_cnt & 0x0040) != 0 {
            self.check_spu_irq_addr(addr);
        }
        let idx = addr as usize;
        i16::from_le_bytes([self.spu_ram[idx], self.spu_ram[idx + 1]]) as i32
    }

    #[inline]
    fn write_reverb_ram_i16(&mut self, rel_words: u16, val: i16) {
        if (self.spu_cnt & 0x0080) == 0 {
            return;
        }
        let base = self.spu.reverb_base_addr & 0x7FFFF;
        let work_size = (0x80000u32 - base).max(2);
        let rel_bytes = (rel_words as u32) * 8;
        let cur_rel = self.spu.reverb_curr_addr.wrapping_sub(base);
        let wrapped = (cur_rel + rel_bytes) % work_size;
        let addr = (base + wrapped) & 0x7FFFE;
        if (self.spu_cnt & 0x0040) != 0 {
            self.check_spu_irq_addr(addr);
        }
        let bytes = val.to_le_bytes();
        let idx = addr as usize;
        self.spu_ram[idx] = bytes[0];
        self.spu_ram[idx + 1] = bytes[1];
    }

    #[inline]
    fn write_capture_i16(&mut self, addr: u32, val: i16) {
        let masked = addr & 0x7FFFE;
        self.check_spu_irq_addr(masked);
        let bytes = val.to_le_bytes();
        self.spu_ram[masked as usize] = bytes[0];
        self.spu_ram[(masked as usize + 1) & 0x7FFFF] = bytes[1];
    }

    fn step_spu_sample(&mut self) {
        let spu_enabled = (self.spu_cnt & 0x8000) != 0;
        let unmuted = (self.spu_cnt & 0x4000) != 0;

        // 1. Step hardware LFSR Noise Generator (only when at least one voice uses noise)
        let noise_sample = if self.spu_non != 0 {
            let noise_shift = ((self.spu_cnt >> 10) & 0x0F) as u8;
            let noise_step = ((self.spu_cnt >> 8) & 0x03) as u8;
            let noise_rate = (noise_shift * 4) + noise_step;
            let (_, noise_period) = crate::spu::calc_envelope_step_and_period(noise_rate, false, false, 0);
            self.spu.noise_counter = self.spu.noise_counter.wrapping_add(1);
            if self.spu.noise_counter >= noise_period {
                self.spu.noise_counter = 0;
                let lfsr = self.spu.noise_lfsr;
                let parity = ((lfsr >> 15) ^ (lfsr >> 12) ^ (lfsr >> 11) ^ (lfsr >> 10) ^ 1) & 1;
                self.spu.noise_lfsr = (lfsr << 1) | parity;
            }
            self.spu.noise_lfsr as i16
        } else {
            0
        };

        // 2. Step 24 SPU Voices
        let mut dry_l = 0i32;
        let mut dry_r = 0i32;
        let mut rev_in_l = 0i32;
        let mut rev_in_r = 0i32;
        let mut voice1_mono = 0i16;
        let mut voice3_mono = 0i16;

        for v in 0..SPU_VOICE_COUNT {
            if !spu_enabled || !self.spu.voices[v].active {
                self.spu.voices[v].last_output = 0;
                continue;
            }

            // Fetch first ADPCM block if not yet loaded
            if !self.spu.voices[v].block_loaded {
                let addr = self.spu.voices[v].current_addr & 0x7FFF0;
                self.check_spu_irq_addr(addr);
                self.check_spu_irq_addr(addr + 8);
                let mut block = [0u8; 16];
                block.copy_from_slice(&self.spu_ram[addr as usize..addr as usize + 16]);
                let flags = block[1];
                let mut p1 = self.spu.voices[v].adpcm_prev1;
                let mut p2 = self.spu.voices[v].adpcm_prev2;
                let decoded = decode_spu_adpcm_block(&block, &mut p1, &mut p2);
                self.spu.voices[v].adpcm_prev1 = p1;
                self.spu.voices[v].adpcm_prev2 = p2;
                self.spu.voices[v].block_samples = decoded;
                self.spu.voices[v].block_flags = flags;
                self.spu.voices[v].block_loaded = true;
                self.spu.voices[v].sample_idx = 0;
                if (flags & 0x04) != 0 && !self.spu.voices[v].has_custom_repeat {
                    self.spu.voices[v].repeat_addr = addr;
                    let rep_reg = (addr >> 3) as u16;
                    self.spu_voice_regs[v][7] = rep_reg;
                    self.spu_regs[v * 8 + 7] = rep_reg;
                }
            }

            // Compute Pitch (with optional Pitch Modulation from Voice v-1)
            let mut step = self.spu_voice_regs[v][2] as u32;
            if v > 0 && (self.spu_pmon & (1 << v)) != 0 {
                let factor = (self.spu.voices[v - 1].last_output as i32) + 0x8000;
                step = (((step as i16 as i32) * factor) >> 15) as u32 & 0xFFFF;
            }
            if step > 0x3FFF {
                step = 0x4000;
            }

            // Sample generation: Noise vs 4-tap Gaussian ADPCM Interpolation
            let raw_sample: i32 = if (self.spu_non & (1 << v)) != 0 {
                noise_sample as i32
            } else {
                let gauss_idx = ((self.spu.voices[v].pitch_counter >> 4) & 0xFF) as usize;
                let h = self.spu.voices[v].hist;
                let s_cur = self.spu.voices[v].block_samples[self.spu.voices[v].sample_idx];
                let mut g = ((GAUSS_TABLE[0x0FF - gauss_idx] as i32) * (h[1] as i32)) >> 15;
                g += ((GAUSS_TABLE[0x1FF - gauss_idx] as i32) * (h[2] as i32)) >> 15;
                g += ((GAUSS_TABLE[0x100 + gauss_idx] as i32) * (h[3] as i32)) >> 15;
                g += ((GAUSS_TABLE[gauss_idx] as i32) * (s_cur as i32)) >> 15;
                g.clamp(-32768, 32767)
            };

            // Advance ADSR envelope
            let adsr_lo = self.spu_voice_regs[v][4];
            let adsr_hi = self.spu_voice_regs[v][5];
            tick_voice_adsr(&mut self.spu.voices[v], adsr_lo, adsr_hi);
            let env = self.spu.voices[v].env_level;
            self.spu_voice_regs[v][6] = env as u16;
            self.spu_regs[v * 8 + 6] = env as u16;

            let voice_out = ((raw_sample * env) >> 15).clamp(-32768, 32767) as i16;
            self.spu.voices[v].last_output = voice_out;
            if v == 1 {
                voice1_mono = voice_out;
            } else if v == 3 {
                voice3_mono = voice_out;
            }

            // Apply Voice Left / Right Volume (fixed or sweep)
            let vol_l_reg = self.spu_voice_regs[v][0];
            let vol_r_reg = self.spu_voice_regs[v][1];
            let vl = evaluate_volume_reg(
                vol_l_reg,
                &mut self.spu.voices[v].sweep_vol_l,
                &mut self.spu.voices[v].sweep_cnt_l,
            );
            let vr = evaluate_volume_reg(
                vol_r_reg,
                &mut self.spu.voices[v].sweep_vol_r,
                &mut self.spu.voices[v].sweep_cnt_r,
            );

            let out_l = ((voice_out as i32) * vl) >> 15;
            let out_r = ((voice_out as i32) * vr) >> 15;
            dry_l += out_l;
            dry_r += out_r;
            if (self.spu_eon & (1 << v)) != 0 {
                rev_in_l += out_l;
                rev_in_r += out_r;
            }

            // Advance Pitch Counter & Decode Next ADPCM Blocks as Needed
            self.spu.voices[v].pitch_counter += step;
            let mut samples_to_advance = (self.spu.voices[v].pitch_counter >> 12) as usize;
            self.spu.voices[v].pitch_counter &= 0x0FFF;

            while samples_to_advance > 0 {
                samples_to_advance -= 1;
                let cur_s = self.spu.voices[v].block_samples[self.spu.voices[v].sample_idx];
                self.spu.voices[v].hist.rotate_left(1);
                self.spu.voices[v].hist[3] = cur_s;
                self.spu.voices[v].sample_idx += 1;

                if self.spu.voices[v].sample_idx >= 28 {
                    self.spu.voices[v].sample_idx = 0;
                    let flags = self.spu.voices[v].block_flags;
                    if (flags & 0x01) != 0 {
                        // Loop End flag: set ENDX bit for this voice
                        self.spu_endx |= 1 << v;
                        self.spu.voices[v].current_addr = self.spu.voices[v].repeat_addr & 0x7FFF0;
                        if (flags & 0x02) == 0 {
                            // End + Mute (no repeat): immediately silence voice
                            self.spu.voices[v].env_level = 0;
                            self.spu.voices[v].adsr_phase = AdsrPhase::Off;
                            self.spu.voices[v].active = false;
                            self.spu_voice_regs[v][6] = 0;
                            self.spu_regs[v * 8 + 6] = 0;
                            break;
                        }
                    } else {
                        self.spu.voices[v].current_addr = (self.spu.voices[v].current_addr + 16) & 0x7FFF0;
                    }

                    // Fetch next 16-byte ADPCM block
                    let next_addr = self.spu.voices[v].current_addr & 0x7FFF0;
                    self.check_spu_irq_addr(next_addr);
                    self.check_spu_irq_addr(next_addr + 8);
                    let mut block = [0u8; 16];
                    block.copy_from_slice(&self.spu_ram[next_addr as usize..next_addr as usize + 16]);
                    let next_flags = block[1];
                    let mut p1 = self.spu.voices[v].adpcm_prev1;
                    let mut p2 = self.spu.voices[v].adpcm_prev2;
                    let decoded = decode_spu_adpcm_block(&block, &mut p1, &mut p2);
                    self.spu.voices[v].adpcm_prev1 = p1;
                    self.spu.voices[v].adpcm_prev2 = p2;
                    self.spu.voices[v].block_samples = decoded;
                    self.spu.voices[v].block_flags = next_flags;

                    if (next_flags & 0x04) != 0 && !self.spu.voices[v].has_custom_repeat {
                        self.spu.voices[v].repeat_addr = next_addr;
                        let rep_reg = (next_addr >> 3) as u16;
                        self.spu_voice_regs[v][7] = rep_reg;
                        self.spu_regs[v * 8 + 7] = rep_reg;
                    }
                }
            }
        }

        // 3. Mix CD-DA / CD-XA Audio (with CD Controller 2x2 Volume Matrix & SPU CD Volume)
        let (raw_cd_l, raw_cd_r) = self.spu.cd_audio_fifo.pop_front().unwrap_or((0, 0));
        let cd_mat_l = (((raw_cd_l as i32) * (self.cd_vol_ll as i32)
            + (raw_cd_r as i32) * (self.cd_vol_rl as i32))
            >> 7)
            .clamp(-32768, 32767) as i16;
        let cd_mat_r = (((raw_cd_l as i32) * (self.cd_vol_lr as i32)
            + (raw_cd_r as i32) * (self.cd_vol_rr as i32))
            >> 7)
            .clamp(-32768, 32767) as i16;

        // Write SPU Capture Buffers (`0x0000..0x0FFF` in SPU RAM)
        if spu_enabled {
            let cap_off = self.spu.capture_offset & 0x03FE;
            self.write_capture_i16(0x0000 + cap_off, cd_mat_l);
            self.write_capture_i16(0x0400 + cap_off, cd_mat_r);
            self.write_capture_i16(0x0800 + cap_off, voice1_mono);
            self.write_capture_i16(0x0C00 + cap_off, voice3_mono);
            self.spu.capture_offset = (cap_off + 2) & 0x03FE;
            if (self.spu.capture_offset & 0x0200) != 0 {
                self.spu_stat |= 1 << 11;
            } else {
                self.spu_stat &= !(1 << 11);
            }
            self.spu_regs[(0x1F80_1DAE - 0x1F80_1C00) >> 1] = self.spu_stat;
        }

        if (self.spu_cnt & 0x0001) != 0 {
            let spu_cd_vol_l = self.spu_regs[(0x1F80_1DB0 - 0x1F80_1C00) >> 1] as i16 as i32;
            let spu_cd_vol_r = self.spu_regs[(0x1F80_1DB2 - 0x1F80_1C00) >> 1] as i16 as i32;
            let cd_l = ((cd_mat_l as i32) * spu_cd_vol_l) >> 15;
            let cd_r = ((cd_mat_r as i32) * spu_cd_vol_r) >> 15;
            dry_l += cd_l;
            dry_r += cd_r;
            if (self.spu_cnt & 0x0004) != 0 {
                rev_in_l += cd_l;
                rev_in_r += cd_r;
            }
        }

        // 4. PS1 Hardware SPU Reverb Engine (runs at 22.05 kHz, every 2nd 44.1 kHz tick)
        let fp = self.spu.reverb_fifo_pos & 63;
        self.spu.reverb_in_fifo_l[fp] = rev_in_l.clamp(-32768, 32767) as i16;
        self.spu.reverb_in_fifo_r[fp] = rev_in_r.clamp(-32768, 32767) as i16;
        self.spu.reverb_fifo_pos = (fp + 1) & 63;

        let rev_vol_l = self.spu_regs[(0x1F80_1D84 - 0x1F80_1C00) >> 1] as i16 as i32;
        let rev_vol_r = self.spu_regs[(0x1F80_1D86 - 0x1F80_1C00) >> 1] as i16 as i32;
        let reverb_active = self.spu.reverb_enabled
            && ((self.spu_cnt & 0x0080) != 0 || rev_vol_l != 0 || rev_vol_r != 0 || (self.spu_cnt & 0x0040) != 0);

        self.spu.reverb_tick_toggle = !self.spu.reverb_tick_toggle;
        if spu_enabled && self.spu.reverb_tick_toggle && reverb_active {
            // 39-tap FIR downsample to 22.05 kHz
            let mut fir_l = 0i32;
            let mut fir_r = 0i32;
            for i in 0..39 {
                let idx = (self.spu.reverb_fifo_pos.wrapping_sub(39 - i)) & 63;
                let coeff = REVERB_FIR[i] as i32;
                fir_l += (self.spu.reverb_in_fifo_l[idx] as i32) * coeff;
                fir_r += (self.spu.reverb_in_fifo_r[idx] as i32) * coeff;
            }
            let lin_in = ((fir_l >> 15) * (self.spu_regs[0xFC] as i16 as i32)) >> 15; // vLIN (0x1DC0+0x38)
            let rin_in = ((fir_r >> 15) * (self.spu_regs[0xFD] as i16 as i32)) >> 15; // vRIN (0x1DC0+0x3A)

            // Reverb registers starting at 0x1F80_1DC0 (spu_regs[0xE0..=0xFF])
            let d_apf1 = self.spu_regs[0xE0];
            let d_apf2 = self.spu_regs[0xE1];
            let v_iir = self.spu_regs[0xE2] as i16 as i32;
            let v_comb1 = self.spu_regs[0xE3] as i16 as i32;
            let v_comb2 = self.spu_regs[0xE4] as i16 as i32;
            let v_comb3 = self.spu_regs[0xE5] as i16 as i32;
            let v_comb4 = self.spu_regs[0xE6] as i16 as i32;
            let v_wall = self.spu_regs[0xE7] as i16 as i32;
            let v_apf1 = self.spu_regs[0xE8] as i16 as i32;
            let v_apf2 = self.spu_regs[0xE9] as i16 as i32;
            let m_lsame = self.spu_regs[0xEA];
            let m_rsame = self.spu_regs[0xEB];
            let m_lcomb1 = self.spu_regs[0xEC];
            let m_rcomb1 = self.spu_regs[0xED];
            let m_lcomb2 = self.spu_regs[0xEE];
            let m_rcomb2 = self.spu_regs[0xEF];
            let d_lsame = self.spu_regs[0xF0];
            let d_rsame = self.spu_regs[0xF1];
            let m_ldiff = self.spu_regs[0xF2];
            let m_rdiff = self.spu_regs[0xF3];
            let m_lcomb3 = self.spu_regs[0xF4];
            let m_rcomb3 = self.spu_regs[0xF5];
            let m_lcomb4 = self.spu_regs[0xF6];
            let m_rcomb4 = self.spu_regs[0xF7];
            let d_ldiff = self.spu_regs[0xF8];
            let d_rdiff = self.spu_regs[0xF9];
            let m_lapf1 = self.spu_regs[0xFA];
            let m_rapf1 = self.spu_regs[0xFB];
            let m_lapf2 = self.spu_regs[0xFC];
            let m_rapf2 = self.spu_regs[0xFD];
            let v_lin = self.spu_regs[0xFE] as i16 as i32;
            let v_rin = self.spu_regs[0xFF] as i16 as i32;

            let _ = (lin_in, rin_in);
            let in_l = ((fir_l >> 15) * v_lin) >> 15;
            let in_r = ((fir_r >> 15) * v_rin) >> 15;

            // Same-side reflections
            let l_same_prev = self.read_reverb_ram_i16(m_lsame, -1);
            let l_same_src = self.read_reverb_ram_i16(d_lsame, 0);
            let l_same_val = ((((in_l + ((l_same_src * v_wall) >> 15) - l_same_prev) * v_iir) >> 15) + l_same_prev)
                .clamp(-32768, 32767) as i16;
            self.write_reverb_ram_i16(m_lsame, l_same_val);

            let r_same_prev = self.read_reverb_ram_i16(m_rsame, -1);
            let r_same_src = self.read_reverb_ram_i16(d_rsame, 0);
            let r_same_val = ((((in_r + ((r_same_src * v_wall) >> 15) - r_same_prev) * v_iir) >> 15) + r_same_prev)
                .clamp(-32768, 32767) as i16;
            self.write_reverb_ram_i16(m_rsame, r_same_val);

            // Different-side reflections
            let l_diff_prev = self.read_reverb_ram_i16(m_ldiff, -1);
            let r_diff_src = self.read_reverb_ram_i16(d_rdiff, 0);
            let l_diff_val = ((((in_l + ((r_diff_src * v_wall) >> 15) - l_diff_prev) * v_iir) >> 15) + l_diff_prev)
                .clamp(-32768, 32767) as i16;
            self.write_reverb_ram_i16(m_ldiff, l_diff_val);

            let r_diff_prev = self.read_reverb_ram_i16(m_rdiff, -1);
            let l_diff_src = self.read_reverb_ram_i16(d_ldiff, 0);
            let r_diff_val = ((((in_r + ((l_diff_src * v_wall) >> 15) - r_diff_prev) * v_iir) >> 15) + r_diff_prev)
                .clamp(-32768, 32767) as i16;
            self.write_reverb_ram_i16(m_rdiff, r_diff_val);

            // 4-tap Early Reflections Comb Filter
            let comb_l = (((v_comb1 * self.read_reverb_ram_i16(m_lcomb1, 0))
                + (v_comb2 * self.read_reverb_ram_i16(m_lcomb2, 0))
                + (v_comb3 * self.read_reverb_ram_i16(m_lcomb3, 0))
                + (v_comb4 * self.read_reverb_ram_i16(m_lcomb4, 0)))
                >> 15)
                .clamp(-32768, 32767);
            let comb_r = (((v_comb1 * self.read_reverb_ram_i16(m_rcomb1, 0))
                + (v_comb2 * self.read_reverb_ram_i16(m_rcomb2, 0))
                + (v_comb3 * self.read_reverb_ram_i16(m_rcomb3, 0))
                + (v_comb4 * self.read_reverb_ram_i16(m_rcomb4, 0)))
                >> 15)
                .clamp(-32768, 32767);

            // All-Pass Filter 1 (APF1)
            let apf1_del_l = self.read_reverb_ram_i16(m_lapf1.wrapping_sub(d_apf1), 0);
            let apf1_in_l = (comb_l - ((v_apf1 * apf1_del_l) >> 15)).clamp(-32768, 32767);
            self.write_reverb_ram_i16(m_lapf1, apf1_in_l as i16);
            let apf1_out_l = (apf1_del_l + ((apf1_in_l * v_apf1) >> 15)).clamp(-32768, 32767);

            let apf1_del_r = self.read_reverb_ram_i16(m_rapf1.wrapping_sub(d_apf1), 0);
            let apf1_in_r = (comb_r - ((v_apf1 * apf1_del_r) >> 15)).clamp(-32768, 32767);
            self.write_reverb_ram_i16(m_rapf1, apf1_in_r as i16);
            let apf1_out_r = (apf1_del_r + ((apf1_in_r * v_apf1) >> 15)).clamp(-32768, 32767);

            // All-Pass Filter 2 (APF2)
            let apf2_del_l = self.read_reverb_ram_i16(m_lapf2.wrapping_sub(d_apf2), 0);
            let apf2_in_l = (apf1_out_l - ((v_apf2 * apf2_del_l) >> 15)).clamp(-32768, 32767);
            self.write_reverb_ram_i16(m_lapf2, apf2_in_l as i16);
            let apf2_out_l = (apf2_del_l + ((apf2_in_l * v_apf2) >> 15)).clamp(-32768, 32767) as i16;

            let apf2_del_r = self.read_reverb_ram_i16(m_rapf2.wrapping_sub(d_apf2), 0);
            let apf2_in_r = (apf1_out_r - ((v_apf2 * apf2_del_r) >> 15)).clamp(-32768, 32767);
            self.write_reverb_ram_i16(m_rapf2, apf2_in_r as i16);
            let apf2_out_r = (apf2_del_r + ((apf2_in_r * v_apf2) >> 15)).clamp(-32768, 32767) as i16;

            self.spu.reverb_last_out_l = apf2_out_l;
            self.spu.reverb_last_out_r = apf2_out_r;

            // Advance reverb ring buffer address by 1 sample (2 bytes) inside `[reverb_base_addr..0x80000)`
            let base = self.spu.reverb_base_addr & 0x7FFFF;
            let next = self.spu.reverb_curr_addr + 2;
            self.spu.reverb_curr_addr = if next >= 0x80000 { base } else { next };
        }

        // Apply Reverb Output Volume (vLOUT at 0x1D84, vROUT at 0x1D86)
        let rev_vol_l = self.spu_regs[(0x1F80_1D84 - 0x1F80_1C00) >> 1] as i16 as i32;
        let rev_vol_r = self.spu_regs[(0x1F80_1D86 - 0x1F80_1C00) >> 1] as i16 as i32;
        let wet_l = ((self.spu.reverb_last_out_l as i32) * rev_vol_l) >> 15;
        let wet_r = ((self.spu.reverb_last_out_r as i32) * rev_vol_r) >> 15;

        // 5. Apply Main Volume Left / Right (`0x1F80_1D80` / `0x1F80_1D82`, fixed or sweep)
        let main_vol_l_reg = self.spu_regs[(0x1F80_1D80 - 0x1F80_1C00) >> 1];
        let main_vol_r_reg = self.spu_regs[(0x1F80_1D82 - 0x1F80_1C00) >> 1];
        let mvl = evaluate_volume_reg(
            main_vol_l_reg,
            &mut self.spu.main_sweep_l,
            &mut self.spu.main_sweep_cnt_l,
        );
        let mvr = evaluate_volume_reg(
            main_vol_r_reg,
            &mut self.spu.main_sweep_r,
            &mut self.spu.main_sweep_cnt_r,
        );
        // Update Current Main Volume registers at `0x1F80_1DB8` / `0x1F80_1DBA`
        self.spu_regs[(0x1F80_1DB8 - 0x1F80_1C00) >> 1] = mvl as u16;
        self.spu_regs[(0x1F80_1DBA - 0x1F80_1C00) >> 1] = mvr as u16;

        let (final_l, final_r) = if spu_enabled && unmuted {
            let tot_l = (((dry_l + wet_l).clamp(-32768, 32767) * mvl) >> 14).clamp(-32768, 32767) as i16;
            let tot_r = (((dry_r + wet_r).clamp(-32768, 32767) * mvr) >> 14).clamp(-32768, 32767) as i16;
            (tot_l, tot_r)
        } else {
            (0, 0)
        };

        if self.spu.output_samples.len() < 32768 {
            self.spu.output_samples.push_back((final_l, final_r));
        }
    }

    pub fn take_audio_samples(&mut self) -> Vec<(i16, i16)> {
        self.spu.output_samples.drain(..).collect()
    }

    #[inline]
    pub fn drain_audio_samples_into(&mut self, dst: &mut Vec<(i16, i16)>) {
        dst.clear();
        dst.extend(self.spu.output_samples.drain(..));
    }

    #[inline(always)]
    fn step_timer_0(&mut self, ticks: u32) {
        let cur = self.timers[0].counter as u32;
        let target = self.timers[0].target as u32;
        if cur < target && cur + ticks < target {
            self.timers[0].counter = (cur + ticks) as u16;
            return;
        }
        if cur > target && cur + ticks < 0x10000 {
            self.timers[0].counter = (cur + ticks) as u16;
            return;
        }
        self.step_timer(0, ticks);
    }

    #[inline(always)]
    fn step_timer_01_fast(&mut self, idx: usize, ticks: u32) {
        let cur = self.timers[idx].counter as u32;
        let target = self.timers[idx].target as u32;
        if cur < target && cur + ticks < target {
            self.timers[idx].counter = (cur + ticks) as u16;
            return;
        }
        if cur > target && cur + ticks < 0x10000 {
            self.timers[idx].counter = (cur + ticks) as u16;
            return;
        }
        self.step_timer(idx, ticks);
    }

    fn step_timer(&mut self, idx: usize, ticks: u32) {
        if ticks == 0 {
            return;
        }
        let mode = self.timers[idx].mode;
        if idx == 2 && (mode & 1) != 0 {
            let sync_mode = (mode >> 1) & 0x03;
            if sync_mode == 0 || sync_mode == 3 {
                return;
            }
        }

        let reset_on_target = (mode & (1 << 3)) != 0;
        let irq_on_target = (mode & (1 << 4)) != 0;
        let irq_on_overflow = (mode & (1 << 5)) != 0;
        let target = self.timers[idx].target as u32;

        let mut cur = self.timers[idx].counter as u32;
        // Fast-path: no target or overflow event crossed on this tick
        let next_threshold = if cur < target { target } else if cur == target && reset_on_target { cur } else { 0x10000 };
        if cur + ticks < next_threshold {
            self.timers[idx].counter = (cur + ticks) as u16;
            return;
        }
        let mut remaining = ticks;

        while remaining > 0 {
            if reset_on_target {
                let period = if target == 0 { 1 } else { target };
                if cur < target {
                    let dist = target - cur;
                    if remaining >= dist {
                        remaining -= dist;
                        self.timers[idx].mode |= 1 << 11;
                        if irq_on_target {
                            self.trigger_timer_irq(idx);
                        }
                        cur = 0;
                        if remaining >= period && period > 0 {
                            remaining %= period;
                        }
                    } else {
                        cur += remaining;
                        remaining = 0;
                    }
                } else if cur == target {
                    remaining -= 1;
                    self.timers[idx].mode |= 1 << 11;
                    if irq_on_target {
                        self.trigger_timer_irq(idx);
                    }
                    cur = 0;
                } else {
                        let dist_ovf = 0x10000 - cur;
                        if remaining >= dist_ovf {
                            remaining -= dist_ovf;
                            self.timers[idx].mode |= 1 << 12;
                            if irq_on_overflow {
                                self.trigger_timer_irq(idx);
                            }
                            cur = 0;
                        } else {
                            cur += remaining;
                            remaining = 0;
                        }
                    }
            } else {
                let next_event = if cur < target && target <= 0xFFFF { target - cur } else { 0x10000 - cur };
                if remaining >= next_event {
                    remaining -= next_event;
                    cur += next_event;
                    if cur == target {
                        self.timers[idx].mode |= 1 << 11;
                        if irq_on_target {
                            self.trigger_timer_irq(idx);
                        }
                    }
                    if cur >= 0x10000 {
                        cur &= 0xFFFF;
                        self.timers[idx].mode |= 1 << 12;
                        if irq_on_overflow {
                            self.trigger_timer_irq(idx);
                        }
                        if target == 0 {
                            self.timers[idx].mode |= 1 << 11;
                            if irq_on_target {
                                self.trigger_timer_irq(idx);
                            }
                        }
                    }
                } else {
                    cur += remaining;
                    remaining = 0;
                }
            }
        }

        self.timers[idx].counter = (cur & 0xFFFF) as u16;
    }

    fn trigger_timer_irq(&mut self, idx: usize) {
        let mode = self.timers[idx].mode;
        let repeat = (mode & (1 << 6)) != 0;
        let toggle = (mode & (1 << 7)) != 0;

        if !repeat && self.timers[idx].irq_fired {
            return;
        }
        self.timers[idx].irq_fired = true;

        if toggle {
            self.timers[idx].mode ^= 1 << 10;
        } else {
            self.timers[idx].mode &= !(1 << 10);
        }

        if !toggle || (self.timers[idx].mode & (1 << 10)) == 0 {
            self.i_stat |= 1 << (4 + idx);
            if !toggle {
                self.timers[idx].mode |= 1 << 10;
            }
        }
    }

    #[inline]
    fn read_timer_reg(&mut self, idx: usize, offset: u32) -> u16 {
        let val = match offset {
            0x00 => self.timers[idx].counter,
            0x04 => {
                let val = self.timers[idx].mode;
                self.timers[idx].mode &= !((1 << 11) | (1 << 12));
                val
            }
            0x08 => self.timers[idx].target,
            _ => 0,
        };
        val
    }

    #[inline]
    fn write_timer_reg(&mut self, idx: usize, offset: u32, val: u16) {
        match offset {
            0x00 => self.timers[idx].counter = val,
            0x04 => {
                self.timers[idx].mode = (val & 0x03FF) | (1 << 10);
                self.timers[idx].counter = 0;
                self.timers[idx].irq_fired = false;
                self.timers[idx].sub_cycles = 0;
            }
            0x08 => self.timers[idx].target = val,
            _ => {}
        }
    }

    #[inline(always)]
    pub fn fetch_instr32(&mut self, vaddr: u32) -> u32 {
        let paddr = vaddr & 0x1FFF_FFFF;
        if paddr < 0x0080_0000 && vaddr >= 0x8000_0000 {
            let idx = (paddr as usize) & (RAM_SIZE - 1);
            return u32::from_le_bytes([
                self.ram[idx],
                self.ram[idx + 1],
                self.ram[idx + 2],
                self.ram[idx + 3],
            ]);
        }
        if (0x1FC0_0000..=0x1FC7_FFFC).contains(&paddr) && vaddr >= 0x8000_0000 {
            let idx = (paddr - 0x1FC0_0000) as usize;
            return u32::from_le_bytes([
                self.bios[idx],
                self.bios[idx + 1],
                self.bios[idx + 2],
                self.bios[idx + 3],
            ]);
        }
        self.read32(vaddr)
    }

    #[inline]
    pub fn read8(&mut self, vaddr: u32) -> u8 {
        if Self::is_kuseg_bus_error(vaddr) {
            self.bus_error = true;
            self.last_bus_error_addr = vaddr;
            return 0;
        }
        let paddr = Self::mask_address(vaddr);
        match paddr {
            0x0000_0000..=0x007F_FFFF => self.ram[(paddr as usize) & (RAM_SIZE - 1)],
            0x1F00_0000..=0x1F7F_FFFF => 0xFF,
            0x1F80_0000..=0x1F80_03FF => {
                if Self::is_scratchpad_accessible(vaddr) {
                    self.scratchpad[(paddr - 0x1F80_0000) as usize]
                } else {
                    0
                }
            }
            0x1F80_1040 => self.read_sio_rx(),
            0x1F80_1041..=0x1F80_1043 => 0xFF,
            0x1F80_1044..=0x1F80_1047 => {
                let stat = self.read_sio_stat();
                ((stat >> ((paddr - 0x1F80_1044) * 8)) & 0xFF) as u8
            }
            0x1F80_1048 => (self.sio_mode & 0xFF) as u8,
            0x1F80_1049 => ((self.sio_mode >> 8) & 0xFF) as u8,
            0x1F80_104A => (self.sio_ctrl & 0xFF) as u8,
            0x1F80_104B => ((self.sio_ctrl >> 8) & 0xFF) as u8,
            0x1F80_104E => (self.sio_baud & 0xFF) as u8,
            0x1F80_104F => ((self.sio_baud >> 8) & 0xFF) as u8,
            0x1F80_1070 => (self.i_stat & 0xFF) as u8,
            0x1F80_1071 => ((self.i_stat >> 8) & 0xFF) as u8,
            0x1F80_1074 => (self.i_mask & 0xFF) as u8,
            0x1F80_1075 => ((self.i_mask >> 8) & 0xFF) as u8,
            0x1F80_1080..=0x1F80_10F7 => {
                let word = self.read32(paddr & !3);
                ((word >> ((paddr & 3) * 8)) & 0xFF) as u8
            }
            0x1F80_1800..=0x1F80_1803 => self.read_cd_reg((paddr - 0x1F80_1800) as u8),
            0x1F80_1820..=0x1F80_1827 => {
                let word = self.read32(paddr & !3);
                ((word >> ((paddr & 3) * 8)) & 0xFF) as u8
            }
            0x1F80_1C00..=0x1F80_1DFF => {
                let hw = self.read_spu_reg16(paddr & !1);
                if (paddr & 1) == 0 {
                    (hw & 0xFF) as u8
                } else {
                    ((hw >> 8) & 0xFF) as u8
                }
            }
            0x1FC0_0000..=0x1FC7_FFFF => self.bios[(paddr - 0x1FC0_0000) as usize],
            _ => 0,
        }
    }

    #[inline]
    pub fn read16(&mut self, vaddr: u32) -> u16 {
        if Self::is_kuseg_bus_error(vaddr) {
            self.bus_error = true;
            self.last_bus_error_addr = vaddr;
            return 0;
        }
        let paddr = Self::mask_address(vaddr);
        match paddr {
            0x0000_0000..=0x007F_FFFF => {
                let idx = (paddr as usize) & (RAM_SIZE - 1);
                u16::from_le_bytes([self.ram[idx], self.ram[(idx + 1) & (RAM_SIZE - 1)]])
            }
            0x1F80_0000..=0x1F80_03FE => {
                if Self::is_scratchpad_accessible(vaddr) {
                    let idx = (paddr - 0x1F80_0000) as usize;
                    u16::from_le_bytes([self.scratchpad[idx], self.scratchpad[idx + 1]])
                } else {
                    0
                }
            }
            0x1F80_1040 => self.read_sio_rx() as u16,
            0x1F80_1044 => (self.read_sio_stat() & 0xFFFF) as u16,
            0x1F80_1046 => ((self.read_sio_stat() >> 16) & 0xFFFF) as u16,
            0x1F80_1048 => self.sio_mode,
            0x1F80_104A => self.sio_ctrl,
            0x1F80_104E => self.sio_baud,
            0x1F80_1070 => self.i_stat as u16,
            0x1F80_1074 => self.i_mask as u16,
            0x1F80_1080..=0x1F80_10F6 => {
                let word = self.read32(paddr & !3);
                ((word >> ((paddr & 2) * 8)) & 0xFFFF) as u16
            }
            0x1F80_1100..=0x1F80_112F => {
                let idx = ((paddr - 0x1F80_1100) >> 4) as usize;
                let offset = (paddr - 0x1F80_1100) & 0x0F;
                self.read_timer_reg(idx, offset)
            }
            0x1F80_1800..=0x1F80_1803 => {
                if paddr == 0x1F80_1802 {
                    let b0 = self.read_cd_data_byte();
                    let b1 = self.read_cd_data_byte();
                    u16::from_le_bytes([b0, b1])
                } else {
                    self.read_cd_reg((paddr - 0x1F80_1800) as u8) as u16
                }
            }
            0x1F80_1820..=0x1F80_1826 => {
                let word = self.read32(paddr & !3);
                ((word >> ((paddr & 2) * 8)) & 0xFFFF) as u16
            }
            0x1F80_1C00..=0x1F80_1DFE => self.read_spu_reg16(paddr),
            0x1FC0_0000..=0x1FC7_FFFE => {
                let idx = (paddr - 0x1FC0_0000) as usize;
                u16::from_le_bytes([self.bios[idx], self.bios[idx + 1]])
            }
            _ => 0,
        }
    }

    #[inline]
    pub fn read32(&mut self, vaddr: u32) -> u32 {
        if vaddr == 0xFFFE_0130 {
            return self.cache_control;
        }
        if Self::is_kuseg_bus_error(vaddr) {
            self.bus_error = true;
            self.last_bus_error_addr = vaddr;
            return 0;
        }
        let paddr = Self::mask_address(vaddr);
        match paddr {
            0x0000_0000..=0x007F_FFFF => {
                let idx = (paddr as usize) & (RAM_SIZE - 1);
                u32::from_le_bytes([
                    self.ram[idx],
                    self.ram[(idx + 1) & (RAM_SIZE - 1)],
                    self.ram[(idx + 2) & (RAM_SIZE - 1)],
                    self.ram[(idx + 3) & (RAM_SIZE - 1)],
                ])
            }
            0x1F80_0000..=0x1F80_03FC => {
                if Self::is_scratchpad_accessible(vaddr) {
                    let idx = (paddr - 0x1F80_0000) as usize;
                    u32::from_le_bytes([
                        self.scratchpad[idx],
                        self.scratchpad[idx + 1],
                        self.scratchpad[idx + 2],
                        self.scratchpad[idx + 3],
                    ])
                } else {
                    0
                }
            }
            0x1F80_1040 => self.read_sio_rx() as u32,
            0x1F80_1044 => self.read_sio_stat(),
            0x1F80_1048 => (self.sio_mode as u32) | ((self.sio_ctrl as u32) << 16),
            0x1F80_104A => self.sio_ctrl as u32,
            0x1F80_104C => (self.sio_baud as u32) << 16,
            0x1F80_104E => self.sio_baud as u32,
            0x1F80_1070 => self.i_stat,
            0x1F80_1074 => self.i_mask,
            0x1F80_1080..=0x1F80_10EF => {
                let ch = ((paddr - 0x1F80_1080) >> 4) as usize;
                let reg = (paddr - 0x1F80_1080) & 0x0F;
                match reg {
                    0x00 => self.dma[ch].madr,
                    0x04 => self.dma[ch].bcr,
                    0x08 => self.dma[ch].chcr,
                    _ => 0,
                }
            }
            0x1F80_10F0 => self.dpcr,
            0x1F80_10F4 => self.dicr,
            0x1F80_1100..=0x1F80_112F => {
                let idx = ((paddr - 0x1F80_1100) >> 4) as usize;
                let offset = (paddr - 0x1F80_1100) & 0x0F;
                self.read_timer_reg(idx, offset) as u32
            }
            0x1F80_1800..=0x1F80_1803 => {
                if paddr == 0x1F80_1802 {
                    self.read_cd_data_word()
                } else {
                    self.read_cd_reg((paddr - 0x1F80_1800) as u8) as u32
                }
            }
            0x1F80_1810 => self.read_gpuread(),
            0x1F80_1814 => {
                self.update_gpustat_dma_request();
                self.gpustat
            }
            0x1F80_1820 => self.read_mdec0(),
            0x1F80_1824 => self.read_mdec1(),
            0x1F80_1C00..=0x1F80_1DFC => {
                let lo = self.read_spu_reg16(paddr) as u32;
                let hi = self.read_spu_reg16(paddr + 2) as u32;
                lo | (hi << 16)
            }
            0x1FC0_0000..=0x1FC7_FFFC => {
                let idx = (paddr - 0x1FC0_0000) as usize;
                u32::from_le_bytes([self.bios[idx], self.bios[idx + 1], self.bios[idx + 2], self.bios[idx + 3]])
            }
            _ => 0,
        }
    }

    #[inline]
    pub fn write8(&mut self, vaddr: u32, val: u8, cache_isolated: bool) {
        if Self::is_kuseg_bus_error(vaddr) {
            self.bus_error = true;
            self.last_bus_error_addr = vaddr;
            return;
        }
        if cache_isolated {
            return;
        }
        let paddr = Self::mask_address(vaddr);
        match paddr {
            0x0000_0000..=0x007F_FFFF => {
                let idx = (paddr as usize) & (RAM_SIZE - 1);
                self.ram[idx] = val;
            }
            0x1F80_0000..=0x1F80_03FF => {
                if Self::is_scratchpad_accessible(vaddr) {
                    let idx = (paddr - 0x1F80_0000) as usize;
                    self.scratchpad[idx] = val;
                }
            }
            0x1F80_1040 => self.write_sio_tx(val),
            0x1F80_1048 => self.sio_mode = (self.sio_mode & 0xFF00) | (val as u16),
            0x1F80_1049 => self.sio_mode = (self.sio_mode & 0x00FF) | ((val as u16) << 8),
            0x1F80_104A => {
                let new_ctrl = (self.sio_ctrl & 0xFF00) | (val as u16);
                self.write_sio_ctrl(new_ctrl);
            }
            0x1F80_104B => {
                let new_ctrl = (self.sio_ctrl & 0x00FF) | ((val as u16) << 8);
                self.write_sio_ctrl(new_ctrl);
            }
            0x1F80_104E => self.sio_baud = (self.sio_baud & 0xFF00) | (val as u16),
            0x1F80_104F => self.sio_baud = (self.sio_baud & 0x00FF) | ((val as u16) << 8),
            0x1F80_1070 => self.i_stat &= (val as u32) | 0xFFFF_FF00,
            0x1F80_1071 => self.i_stat &= ((val as u32) << 8) | 0xFFFF_00FF,
            0x1F80_1074 => self.i_mask = (self.i_mask & 0x0700) | (val as u32),
            0x1F80_1075 => self.i_mask = (self.i_mask & 0x00FF) | (((val as u32) << 8) & 0x0700),
            0x1F80_10F4..=0x1F80_10F7 => {
                let shift = (paddr & 3) * 8;
                let controls = self.dicr & 0x00FF_FFFF & !(0xFF << shift);
                self.write_dicr(controls | ((val as u32) << shift));
            },
            0x1F80_1800..=0x1F80_1803 => self.write_cd_reg((paddr - 0x1F80_1800) as u8, val),
            0x1F80_1C00..=0x1F80_1DFF => {
                let aligned = paddr & !1;
                let reg_idx = ((aligned - 0x1F80_1C00) >> 1) as usize;
                let cur = self.spu_regs[reg_idx];
                let new_val = if (paddr & 1) == 0 {
                    (cur & 0xFF00) | (val as u16)
                } else {
                    (cur & 0x00FF) | ((val as u16) << 8)
                };
                self.write_spu_reg16(aligned, new_val);
            }
            0x1F80_2023 | 0x1F80_2080 => {
                let ch = val as char;
                print!("{}", ch);
                self.tty_buffer.push(ch);
            }
            _ => {}
        }
    }

    #[inline]
    pub fn write16(&mut self, vaddr: u32, val: u16, cache_isolated: bool) {
        if Self::is_kuseg_bus_error(vaddr) {
            self.bus_error = true;
            self.last_bus_error_addr = vaddr;
            return;
        }
        if cache_isolated {
            return;
        }
        let paddr = Self::mask_address(vaddr);
        let bytes = val.to_le_bytes();
        match paddr {
            0x0000_0000..=0x007F_FFFF => {
                let idx = (paddr as usize) & (RAM_SIZE - 1);
                self.ram[idx] = bytes[0];
                self.ram[(idx + 1) & (RAM_SIZE - 1)] = bytes[1];
            }
            0x1F80_0000..=0x1F80_03FE => {
                if Self::is_scratchpad_accessible(vaddr) {
                    let idx = (paddr - 0x1F80_0000) as usize;
                    self.scratchpad[idx..idx + 2].copy_from_slice(&bytes);
                }
            }
            0x1F80_1040 => self.write_sio_tx((val & 0xFF) as u8),
            0x1F80_1048 => self.sio_mode = val,
            0x1F80_104A => self.write_sio_ctrl(val),
            0x1F80_104E => self.sio_baud = val,
            0x1F80_1070 => self.i_stat &= (val as u32) | 0xFFFF_0000,
            0x1F80_1074 => self.i_mask = (val as u32) & 0x7FF,
            0x1F80_10F4 | 0x1F80_10F6 => {
                let shift = (paddr & 2) * 8;
                let controls = self.dicr & 0x00FF_FFFF & !(0xFFFF << shift);
                self.write_dicr(controls | ((val as u32) << shift));
            },
            0x1F80_1100..=0x1F80_112F => {
                let idx = ((paddr - 0x1F80_1100) >> 4) as usize;
                let offset = (paddr - 0x1F80_1100) & 0x0F;
                self.write_timer_reg(idx, offset, val);
            }
            0x1F80_1800..=0x1F80_1803 => self.write_cd_reg((paddr - 0x1F80_1800) as u8, (val & 0xFF) as u8),
            0x1F80_1C00..=0x1F80_1DFE => self.write_spu_reg16(paddr, val),
            _ => {}
        }
    }

    #[inline]
    pub fn write32(&mut self, vaddr: u32, val: u32, cache_isolated: bool) {
        if vaddr == 0xFFFE_0130 {
            self.cache_control = val;
            return;
        }
        if Self::is_kuseg_bus_error(vaddr) {
            self.bus_error = true;
            self.last_bus_error_addr = vaddr;
            return;
        }
        if cache_isolated {
            return;
        }
        let paddr = Self::mask_address(vaddr);
        let bytes = val.to_le_bytes();
        match paddr {
            0x0000_0000..=0x007F_FFFF => {
                let idx = (paddr as usize) & (RAM_SIZE - 1);
                self.ram[idx] = bytes[0];
                self.ram[(idx + 1) & (RAM_SIZE - 1)] = bytes[1];
                self.ram[(idx + 2) & (RAM_SIZE - 1)] = bytes[2];
                self.ram[(idx + 3) & (RAM_SIZE - 1)] = bytes[3];
            }
            0x1F80_0000..=0x1F80_03FC => {
                if Self::is_scratchpad_accessible(vaddr) {
                    let idx = (paddr - 0x1F80_0000) as usize;
                    self.scratchpad[idx..idx + 4].copy_from_slice(&bytes);
                }
            }
            0x1F80_1040 => self.write_sio_tx((val & 0xFF) as u8),
            0x1F80_1048 => {
                self.sio_mode = (val & 0xFFFF) as u16;
                self.write_sio_ctrl((val >> 16) as u16);
            }
            0x1F80_104A => self.write_sio_ctrl((val & 0xFFFF) as u16),
            0x1F80_104C => self.sio_baud = (val >> 16) as u16,
            0x1F80_104E => self.sio_baud = (val & 0xFFFF) as u16,
            0x1F80_1070 => self.i_stat &= val,
            0x1F80_1074 => self.i_mask = val & 0x7FF,
            0x1F80_1080..=0x1F80_10EF => {
                let ch = ((paddr - 0x1F80_1080) >> 4) as usize;
                let reg = (paddr - 0x1F80_1080) & 0x0F;
                match reg {
                    0x00 => self.dma[ch].madr = val & 0x00FF_FFFF,
                    0x04 => self.dma[ch].bcr = val,
                    0x08 => {
                        if ch == 6 {
                            self.dma[6].chcr = (val & 0x5100_0000) | 0x0000_0002;
                        } else {
                            self.dma[ch].chcr = val;
                        }
                        self.check_and_run_dma(ch);
                    }
                    _ => {}
                }
            }
            0x1F80_10F0 => {
                self.dpcr = val;
                for ch in 0..7 {
                    self.check_and_run_dma(ch);
                }
            }
            0x1F80_10F4 => self.write_dicr(val),
            0x1F80_1100..=0x1F80_112F => {
                let idx = ((paddr - 0x1F80_1100) >> 4) as usize;
                let offset = (paddr - 0x1F80_1100) & 0x0F;
                self.write_timer_reg(idx, offset, (val & 0xFFFF) as u16);
            }
            0x1F80_1800..=0x1F80_1803 => self.write_cd_reg((paddr - 0x1F80_1800) as u8, (val & 0xFF) as u8),
            0x1F80_1810 => self.send_gp0(val),
            0x1F80_1814 => self.send_gp1(val),
            0x1F80_1820 => self.write_mdec0(val),
            0x1F80_1824 => self.write_mdec1(val),
            0x1F80_1C00..=0x1F80_1DFC => {
                self.write_spu_reg16(paddr, (val & 0xFFFF) as u16);
                self.write_spu_reg16(paddr + 2, ((val >> 16) & 0xFFFF) as u16);
            }
            _ => {}
        }
    }

    pub fn dump_vram_ppm<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        use std::io::Write;
        let mut f = fs::File::create(path)?;
        write!(f, "P6\n{} {}\n255\n", VRAM_WIDTH, VRAM_HEIGHT)?;
        let mut rgb = Vec::with_capacity(VRAM_WIDTH * VRAM_HEIGHT * 3);
        for &px in &self.vram {
            let r = ((px & 0x1F) as u8) << 3;
            let g = (((px >> 5) & 0x1F) as u8) << 3;
            let b = (((px >> 10) & 0x1F) as u8) << 3;
            rgb.push(r);
            rgb.push(g);
            rgb.push(b);
        }
        f.write_all(&rgb)?;
        Ok(())
    }

    pub fn set_pad_state(&mut self, buttons_low: u8, buttons_high: u8) {
        self.pad1_buttons = (buttons_low as u16) | ((buttons_high as u16) << 8);
        self.pad1.buttons = self.pad1_buttons;
    }

    pub fn set_pad_analog(&mut self, left_x: u8, left_y: u8, right_x: u8, right_y: u8) {
        self.pad1.left_x = left_x;
        self.pad1.left_y = left_y;
        self.pad1.right_x = right_x;
        self.pad1.right_y = right_y;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kuseg_kseg0_kseg1_masking_and_4x_ram_mirroring() {
        let mut bus = Bus::new();
        bus.write32(0x0000_1234, 0xDEAD_BEEF, false);
        assert_eq!(bus.read32(0x0000_1234), 0xDEAD_BEEF);
        assert_eq!(bus.read32(0x8000_1234), 0xDEAD_BEEF);
        assert_eq!(bus.read32(0xA000_1234), 0xDEAD_BEEF);
        assert_eq!(bus.read32(0x0020_1234), 0xDEAD_BEEF);
        assert_eq!(bus.read32(0x8040_1234), 0xDEAD_BEEF);
        assert_eq!(bus.read32(0xA060_1234), 0xDEAD_BEEF);
        assert!(!bus.take_bus_error());
    }

    #[test]
    fn test_cache_isolation_blocks_ram_and_scratchpad_writes() {
        let mut bus = Bus::new();
        bus.write32(0x8000_0000, 0xCAFE_BABE, false);
        bus.write32(0x1F80_0000, 0x1122_3344, false);
        bus.write32(0x8000_0000, 0x0000_0000, true);
        bus.write32(0x1F80_0000, 0x0000_0000, true);
        assert_eq!(bus.read32(0x8000_0000), 0xCAFE_BABE);
        assert_eq!(bus.read32(0x1F80_0000), 0x1122_3344);
    }

    #[test]
    fn test_scratchpad_kuseg_and_kseg0_only_not_kseg1() {
        let mut bus = Bus::new();
        bus.write32(0x1F80_0010, 0xABCD_EF01, false);
        assert_eq!(bus.read32(0x1F80_0010), 0xABCD_EF01);
        assert_eq!(bus.read32(0x9F80_0010), 0xABCD_EF01);

        assert_eq!(bus.read32(0xBF80_0010), 0);

        bus.write32(0xBF80_0010, 0x5555_5555, false);
        assert_eq!(bus.read32(0x1F80_0010), 0xABCD_EF01);
    }

    #[test]
    fn test_kuseg_bus_error_above_512mb() {
        let mut bus = Bus::new();
        let _ = bus.read8(0x1FFF_FFFF);
        assert!(!bus.take_bus_error());

        assert_eq!(bus.read32(0x2000_0000), 0);
        assert!(bus.take_bus_error());

        bus.write32(0x3000_0004, 0x1234_5678, false);
        assert!(bus.take_bus_error());
    }

    #[test]
    fn test_interrupt_stat_and_mask_registers() {
        let mut bus = Bus::new();
        bus.i_stat = 0b1011;
        bus.write16(0x1F80_1074, 0b0011, false);
        assert_eq!(bus.read16(0x1F80_1074), 0b0011);
        assert!(bus.irq_pending());

        bus.write16(0x1F80_1070, 0b1110, false);
        assert_eq!(bus.read16(0x1F80_1070), 0b1010);
    }
    #[test]
    fn test_dma6_otc_reverse_linked_list_table_clear() {
        let mut bus = Bus::new();
        bus.write32(0x1F80_10F0, 0x0800_0000, false);
        bus.write32(0x1F80_10F4, (1 << 23) | (1 << 22), false);

        bus.write32(0x1F80_10E0, 0x0000_1010, false);
        bus.write32(0x1F80_10E4, 4, false);
        bus.write32(0x1F80_10E8, 0x1100_0002, false);

        assert_eq!(bus.read32(0x0000_1010), 0x0000_100C);
        assert_eq!(bus.read32(0x0000_100C), 0x0000_1008);
        assert_eq!(bus.read32(0x0000_1008), 0x0000_1004);
        assert_eq!(bus.read32(0x0000_1004), 0x00FF_FFFF);

        assert_ne!(bus.read32(0x1F80_10F4) & (1 << 30), 0);
        assert_ne!(bus.read32(0x1F80_10F4) & (1 << 31), 0);
        assert_ne!(bus.i_stat & (1 << 3), 0);
    }

    #[test]
    fn test_dma2_gpu_sync_mode2_linked_list_and_dicr_irq3() {
        let mut bus = Bus::new();
        bus.write32(0x0000_2000, (2 << 24) | 0x0000_2040, false);
        bus.write32(0x0000_2004, 0xE100_0200, false);
        bus.write32(0x0000_2008, 0xE300_0000, false);
        bus.write32(0x0000_2040, (1 << 24) | 0x0000_2080, false);
        bus.write32(0x0000_2044, 0xE403_FF1F, false);
        bus.write32(0x0000_2080, 0x00FF_FFFF, false);
        bus.write32(0x1F80_10F4, (1 << 23) | (1 << 18), false);
        bus.write32(0x1F80_10A0, 0x0000_2000, false);
        bus.write32(0x1F80_10A4, 0, false);
        bus.write32(0x1F80_10A8, 0x0100_0401, false);
        assert_eq!(bus.gp0_command_log, vec![0xE100_0200, 0xE300_0000, 0xE403_FF1F]);
        assert_ne!(bus.read32(0x1F80_10F4) & (1 << 26), 0);
        assert_ne!(bus.read32(0x1F80_10F4) & (1 << 31), 0);
        assert_ne!(bus.i_stat & (1 << 3), 0);
        bus.write32(0x1F80_10F4, (1 << 23) | (1 << 18) | (1 << 26), false);
        assert_eq!(bus.read32(0x1F80_10F4) & (1 << 26), 0);
        assert_eq!(bus.read32(0x1F80_10F4) & (1 << 31), 0);
    }

    #[test]
    fn test_dicr_byte_enable_and_partial_write_acknowledgement() {
        let mut bus = Bus::new();
        bus.write32(0x1f8010f4, (1 << 23) | (1 << 17) | (1 << 19) | 0x15, false);
        bus.complete_dma_channel(1);
        assert_eq!(bus.read8(0x1f8010f6), 0x8a);
        // PSYQ disables CD completion callbacks for intermediate STR sectors
        // with LBU/SB at DICR+2. Other controls and pending flags must survive.
        let enables = bus.read8(0x1f8010f6);
        bus.write8(0x1f8010f6, enables & !8, false);
        assert_eq!(bus.read8(0x1f8010f6), 0x82);
        assert_ne!(bus.dicr & (1 << 25), 0);
        bus.write8(0x1f8010f7, 2, false);
        assert_eq!(bus.dicr & 0x7f000000, 0);
        bus.complete_dma_channel(3);
        assert_eq!(bus.dicr & (1 << 27), 0, "intermediate sector must not complete the video frame");
        // Only the last sector re-enables this channel's completion callback.
        bus.write8(0x1f8010f6, 0x8a, false);
        bus.complete_dma_channel(3);
        assert_ne!(bus.dicr & (1 << 27), 0);
        bus.write8(0x1f8010f6, 0x8a, false);
        assert_ne!(bus.dicr & (1 << 27), 0, "control write must not echo/ack flags");
        bus.write16(0x1f8010f6, 0x088a, false);
        assert_eq!(bus.dicr & 0x7f000000, 0);
        assert_eq!(bus.read16(0x1f8010f4), 0x15);
        assert_eq!(bus.read16(0x1f8010f6), 0x008a);
    }

    #[test]
    fn test_timers_0_1_2_target_reset_overflow_and_irqs() {
        let mut bus = Bus::new();

        bus.write16(0x1F80_1108, 100, false);
        bus.write16(0x1F80_1104, (1 << 3) | (1 << 4) | (1 << 6), false);
        bus.tick(99);
        assert_eq!(bus.read16(0x1F80_1100), 99);

        bus.tick(1);
        assert_eq!(bus.read16(0x1F80_1100), 0);
        assert_ne!(bus.i_stat & (1 << 4), 0);
        assert_ne!(bus.read16(0x1F80_1104) & (1 << 11), 0);
        assert_eq!(bus.read16(0x1F80_1104) & (1 << 11), 0);

        bus.write16(0x1F80_1114, (1 << 8) | (1 << 5) | (1 << 6), false);
        bus.write16(0x1F80_1110, 0xFFFE, false);
        bus.tick(CYCLES_PER_SCANLINE * 2);
        assert_eq!(bus.read16(0x1F80_1110), 0x0000);
        assert_ne!(bus.i_stat & (1 << 5), 0);
        assert_ne!(bus.read16(0x1F80_1114) & (1 << 12), 0);

        bus.write16(0x1F80_1128, 10, false);
        bus.write16(0x1F80_1124, (2 << 8) | (1 << 3) | (1 << 4), false);
        bus.tick(80);
        assert_eq!(bus.read16(0x1F80_1120), 0);
        assert_ne!(bus.i_stat & (1 << 6), 0);

        bus.i_stat = 0;
        bus.vblank_accumulator = 0;
        bus.tick(CYCLES_PER_FRAME);
        assert_ne!(bus.i_stat & (1 << 0), 0);
    }

    #[test]
    fn test_gp0_02_vram_fill_rectangle_and_rgb24_to_bgr555_alignment() {
        let mut bus = Bus::new();
        bus.write32(0x1F80_1810, 0x0240_80F8, false);
        bus.write32(0x1F80_1810, (5 << 16) | 0x0013, false);
        bus.write32(0x1F80_1810, (2 << 16) | 0x0003, false);

        let expected_px: u16 = (8 << 10) | (16 << 5) | 31;
        assert_eq!(expected_px, 0x221F);
        for dy in 0..2 {
            for dx in 0..16 {
                assert_eq!(bus.vram[(5 + dy) * VRAM_WIDTH + (16 + dx)], expected_px);
            }
        }
        assert_eq!(bus.vram[5 * VRAM_WIDTH + 15], 0);
        assert_eq!(bus.vram[5 * VRAM_WIDTH + 32], 0);
    }

    #[test]
    fn test_gp0_a0_c0_80_vram_transfers_dma2_and_e6_mask_bit_protection() {
        let mut bus = Bus::new();

        bus.write32(0x0000_3000, 0xA000_0000, false);
        bus.write32(0x0000_3004, (20 << 16) | 10, false);
        bus.write32(0x0000_3008, (2 << 16) | 2, false);
        bus.write32(0x0000_300C, 0x8222_1111, false);
        bus.write32(0x0000_3010, 0x4444_3333, false);

        bus.write32(0x1F80_1814, 0x0400_0002, false);
        bus.write32(0x1F80_10A0, 0x0000_3000, false);
        bus.write32(0x1F80_10A4, (1 << 16) | 5, false);
        bus.write32(0x1F80_10A8, 0x0100_0201, false);

        assert_eq!(bus.vram[20 * VRAM_WIDTH + 10], 0x1111);
        assert_eq!(bus.vram[20 * VRAM_WIDTH + 11], 0x8222);
        assert_eq!(bus.vram[21 * VRAM_WIDTH + 10], 0x3333);
        assert_eq!(bus.vram[21 * VRAM_WIDTH + 11], 0x4444);

        bus.write32(0x1F80_1810, 0xE600_0001, false);
        bus.write32(0x1F80_1810, 0xA000_0000, false);
        bus.write32(0x1F80_1810, (20 << 16) | 40, false);
        bus.write32(0x1F80_1810, (2 << 16) | 2, false);
        bus.write32(0x1F80_1810, 0x0666_0555, false);
        bus.write32(0x1F80_1810, 0x0888_0777, false);

        assert_eq!(bus.vram[20 * VRAM_WIDTH + 40], 0x8555);
        assert_eq!(bus.vram[20 * VRAM_WIDTH + 41], 0x8666);

        bus.write32(0x1F80_1810, 0xE600_0002, false);
        bus.write32(0x1F80_1810, 0x8000_0000, false);
        bus.write32(0x1F80_1810, (20 << 16) | 40, false);
        bus.write32(0x1F80_1810, (20 << 16) | 10, false);
        bus.write32(0x1F80_1810, (2 << 16) | 2, false);

        assert_eq!(bus.vram[20 * VRAM_WIDTH + 10], 0x8555);
        assert_eq!(bus.vram[20 * VRAM_WIDTH + 11], 0x8222);
        assert_eq!(bus.vram[21 * VRAM_WIDTH + 10], 0x8777);
        assert_eq!(bus.vram[21 * VRAM_WIDTH + 11], 0x8888);

        bus.write32(0x1F80_1810, 0xC000_0000, false);
        bus.write32(0x1F80_1810, (20 << 16) | 10, false);
        bus.write32(0x1F80_1810, (2 << 16) | 2, false);

        assert_ne!(bus.gpustat & (1 << 27), 0);
        assert_eq!(bus.read32(0x1F80_1810), 0x8222_8555);
        assert_ne!(bus.gpustat & (1 << 27), 0);
        assert_eq!(bus.read32(0x1F80_1810), 0x8888_8777);
        assert_eq!(bus.gpustat & (1 << 27), 0);
    }

    #[test]
    fn test_gp0_attributes_e1_to_e5_and_gp1_10_gpu_info_readback() {
        let mut bus = Bus::new();

        bus.write32(0x1F80_1810, 0xE100_3A5B, false);
        bus.write32(0x1F80_1810, 0xE20A_BCDE, false);
        bus.write32(0x1F80_1810, 0xE301_2345, false);
        bus.write32(0x1F80_1810, 0xE405_6789, false);
        bus.write32(0x1F80_1810, 0xE500_67FC, false);

        assert_eq!(bus.gpustat & 0x7FF, 0x25B);
        assert_ne!(bus.gpustat & (1 << 15), 0);
        assert!(bus.tex_rect_x_flip);
        assert!(bus.tex_rect_y_flip);
        assert_eq!(bus.draw_offset_x, -4);
        assert_eq!(bus.draw_offset_y, 12);

        bus.write32(0x1F80_1814, 0x1000_0002, false);
        assert_eq!(bus.read32(0x1F80_1810), 0x000A_BCDE);

        bus.write32(0x1F80_1814, 0x1000_0003, false);
        assert_eq!(bus.read32(0x1F80_1810), 0x0001_2345);

        bus.write32(0x1F80_1814, 0x1000_0004, false);
        assert_eq!(bus.read32(0x1F80_1810), 0x0005_6789);

        bus.write32(0x1F80_1814, 0x1000_0005, false);
        assert_eq!(bus.read32(0x1F80_1810), 0x0000_67FC);

        bus.write32(0x1F80_1814, 0x1000_0007, false);
        assert_eq!(bus.read32(0x1F80_1810), 2);
    }
    #[test]
    fn test_gp0_monochrome_and_shaded_polygon_rasterization_offset_and_clipping() {
        let mut bus = Bus::new();

        // Set Drawing Area to [10..=15, 10..=15] via GP0(0xE3/0xE4) and Offset (+2, +3) via GP0(0xE5)
        bus.write32(0x1F80_1810, 0xE300_0000 | (10 << 10) | 10, false);
        bus.write32(0x1F80_1810, 0xE400_0000 | (15 << 10) | 15, false);
        bus.write32(0x1F80_1810, 0xE500_0000 | (3 << 11) | 2, false);

        // 1. GP0(0x20): Monochrome 3-pt Triangle (Red 0x0000F8 -> 0x001F), shifted to (10,10)-(14,10)-(10,14)
        bus.write32(0x1F80_1810, 0x2000_00F8, false);
        bus.write32(0x1F80_1810, (7 << 16) | 8, false);
        bus.write32(0x1F80_1810, (7 << 16) | 12, false);
        bus.write32(0x1F80_1810, (11 << 16) | 8, false);

        assert_eq!(bus.vram[10 * VRAM_WIDTH + 10], 0x001F);
        assert_eq!(bus.vram[10 * VRAM_WIDTH + 14], 0x001F);
        assert_eq!(bus.vram[14 * VRAM_WIDTH + 10], 0x001F);
        assert_eq!(bus.vram[11 * VRAM_WIDTH + 11], 0x001F);
        assert_eq!(bus.vram[14 * VRAM_WIDTH + 14], 0x0000);

        // 2. GP0(0x28): Monochrome 4-pt Quad (Green 0x00F800 -> 0x03E0), shifted to (8,8)..(18,18), clipped to [10..=15]
        bus.write32(0x1F80_1810, 0x2800_F800, false);
        bus.write32(0x1F80_1810, (5 << 16) | 6, false);
        bus.write32(0x1F80_1810, (5 << 16) | 16, false);
        bus.write32(0x1F80_1810, (15 << 16) | 6, false);
        bus.write32(0x1F80_1810, (15 << 16) | 16, false);

        for y in 10..=15 {
            for x in 10..=15 {
                assert_eq!(bus.vram[y * VRAM_WIDTH + x], 0x03E0);
            }
        }
        assert_eq!(bus.vram[9 * VRAM_WIDTH + 10], 0x0000);
        assert_eq!(bus.vram[16 * VRAM_WIDTH + 10], 0x0000);
        assert_eq!(bus.vram[10 * VRAM_WIDTH + 9], 0x0000);
        assert_eq!(bus.vram[10 * VRAM_WIDTH + 16], 0x0000);

        // 3. GP0(0x30): Gouraud-Shaded 3-pt Triangle with offset (0,0) and expanded drawing area
        bus.write32(0x1F80_1810, 0xE300_0000, false);
        bus.write32(0x1F80_1810, 0xE400_0000 | (100 << 10) | 100, false);
        bus.write32(0x1F80_1810, 0xE500_0000, false);

        bus.write32(0x1F80_1810, 0x3000_00F8, false);
        bus.write32(0x1F80_1810, (20 << 16) | 20, false);
        bus.write32(0x1F80_1810, 0x0000_F800, false);
        bus.write32(0x1F80_1810, (20 << 16) | 24, false);
        bus.write32(0x1F80_1810, 0x00F8_0000, false);
        bus.write32(0x1F80_1810, (24 << 16) | 20, false);

        assert_eq!(bus.vram[20 * VRAM_WIDTH + 20], 0x001F);
        assert_eq!(bus.vram[20 * VRAM_WIDTH + 24], 0x03E0);
        assert_eq!(bus.vram[24 * VRAM_WIDTH + 20], 0x7C00);
        assert_eq!(bus.vram[20 * VRAM_WIDTH + 22], (15 << 5) | 15);
    }

    #[test]
    fn test_gp0_textured_clut_4bit_8bit_quad_and_rectangles_transparency_and_modulation() {
        let mut bus = Bus::new();

        // 1. 4-bit CLUT at VRAM (0, 240) and 4-bit texpage at (64, 0) (texpage_x = 1, depth = 0)
        let clut4_y = 240usize;
        let clut4_attr: u32 = (clut4_y as u32) << 6;
        bus.vram[clut4_y * VRAM_WIDTH + 0] = 0x0000; // Transparent key
        bus.vram[clut4_y * VRAM_WIDTH + 1] = 0x001F; // Pure Red
        bus.vram[clut4_y * VRAM_WIDTH + 2] = 0x8000; // STP Black (bit 15=1 -> drawn!)
        bus.vram[clut4_y * VRAM_WIDTH + 3] = 0x7FFF; // Pure White
        for v in 0..4 {
            bus.vram[v * VRAM_WIDTH + 64] = 0x3210;
        }
        bus.vram[10 * VRAM_WIDTH + 10] = 0x1234;

        // GP0(0x2C): Textured 4-pt Quad, Opaque, Tex-Blend with 0x808080 (1.0x modulation)
        bus.write32(0x1F80_1810, 0x2C80_8080, false);
        bus.write32(0x1F80_1810, (10 << 16) | 10, false);
        bus.write32(0x1F80_1810, (clut4_attr << 16) | 0x0000, false);
        bus.write32(0x1F80_1810, (10 << 16) | 13, false);
        bus.write32(0x1F80_1810, (1 << 16) | 0x0003, false);
        bus.write32(0x1F80_1810, (13 << 16) | 10, false);
        bus.write32(0x1F80_1810, 0x0300, false);
        bus.write32(0x1F80_1810, (13 << 16) | 13, false);
        bus.write32(0x1F80_1810, 0x0303, false);

        assert_eq!(bus.vram[10 * VRAM_WIDTH + 10], 0x1234);
        assert_eq!(bus.vram[10 * VRAM_WIDTH + 11], 0x001F);
        assert_eq!(bus.vram[10 * VRAM_WIDTH + 12], 0x8000);
        assert_eq!(bus.vram[10 * VRAM_WIDTH + 13], 0x7FFF);

        // 2. GP0(0x64): Variable-Size Textured Rectangle with 0.5x modulation (0x404040)
        bus.write32(0x1F80_1810, 0x6440_4040, false);
        bus.write32(0x1F80_1810, (20 << 16) | 30, false);
        bus.write32(0x1F80_1810, (clut4_attr << 16) | 0x0000, false);
        bus.write32(0x1F80_1810, (2 << 16) | 4, false);
        assert_eq!(bus.vram[20 * VRAM_WIDTH + 33], (15 << 10) | (15 << 5) | 15);

        // 3. 8-bit CLUT at VRAM (0, 241) and 8-bit texpage at (128, 0) (texpage_x = 2, depth = 1)
        let clut8_y = 241usize;
        let clut8_attr: u32 = (clut8_y as u32) << 6;
        bus.vram[clut8_y * VRAM_WIDTH + 0x2A] = 0x7C00;
        bus.vram[clut8_y * VRAM_WIDTH + 0x55] = 0x03E0;
        bus.vram[128] = 0x552A;

        bus.write32(0x1F80_1810, 0xE100_0000 | (1 << 7) | 2, false);
        bus.write32(0x1F80_1810, 0x7C80_8080, false);
        bus.write32(0x1F80_1810, (50 << 16) | 50, false);
        bus.write32(0x1F80_1810, (clut8_attr << 16) | 0x0000, false);

        assert_eq!(bus.vram[50 * VRAM_WIDTH + 50], 0x7C00);
        assert_eq!(bus.vram[50 * VRAM_WIDTH + 51], 0x03E0);
    }

    #[test]
    fn test_gp0_semi_transparency_abr_modes_and_line_polyline_rendering() {
        let mut bus = Bus::new();

        let bg_px: u16 = (20 << 10) | (20 << 5) | 20;
        let fg_rgb24: u32 = 0x0060_6060;
        let expected_abr = [
            (16 << 10) | (16 << 5) | 16,
            (31 << 10) | (31 << 5) | 31,
            (8 << 10) | (8 << 5) | 8,
            (23 << 10) | (23 << 5) | 23,
        ];

        for abr in 0..4u32 {
            let x = 10 + (abr as usize) * 4;
            bus.vram[10 * VRAM_WIDTH + x] = bg_px;
            bus.write32(0x1F80_1810, 0xE100_0000 | (abr << 5), false);
            bus.write32(0x1F80_1810, 0x6A00_0000 | fg_rgb24, false);
            bus.write32(0x1F80_1810, (10 << 16) | (x as u32), false);
            assert_eq!(bus.vram[10 * VRAM_WIDTH + x], expected_abr[abr as usize]);
        }

        // Monochrome single line GP0(0x40)
        bus.write32(0x1F80_1810, 0x4000_00F8, false);
        bus.write32(0x1F80_1810, (50 << 16) | 50, false);
        bus.write32(0x1F80_1810, (50 << 16) | 54, false);
        for x in 50..=54 {
            assert_eq!(bus.vram[50 * VRAM_WIDTH + x], 0x001F);
        }

        // Monochrome Poly-Line GP0(0x48) terminated by 0x5555_5555
        bus.write32(0x1F80_1810, 0x4800_F800, false);
        bus.write32(0x1F80_1810, (60 << 16) | 60, false);
        bus.write32(0x1F80_1810, (60 << 16) | 63, false);
        bus.write32(0x1F80_1810, (63 << 16) | 63, false);
        bus.write32(0x1F80_1810, 0x5555_5555, false);

        assert_eq!(bus.gp0_mode, Gp0Mode::Command);
        for x in 60..=63 {
            assert_eq!(bus.vram[60 * VRAM_WIDTH + x], 0x03E0);
        }
        for y in 60..=63 {
            assert_eq!(bus.vram[y * VRAM_WIDTH + 63], 0x03E0);
        }

        // Shaded Line GP0(0x50)
        bus.write32(0x1F80_1810, 0x5000_00F8, false);
        bus.write32(0x1F80_1810, (70 << 16) | 70, false);
        bus.write32(0x1F80_1810, 0x00F8_0000, false);
        bus.write32(0x1F80_1810, (70 << 16) | 74, false);
        assert_eq!(bus.vram[70 * VRAM_WIDTH + 70], 0x001F);
        assert_eq!(bus.vram[70 * VRAM_WIDTH + 72], (15 << 10) | 15);
        assert_eq!(bus.vram[70 * VRAM_WIDTH + 74], 0x7C00);
    }
    #[test]
    fn test_ps_exe_header_parser_and_ram_payload_loader() {
        let mut exe_bytes = vec![0u8; PS_EXE_HEADER_SIZE + 16];
        exe_bytes[0x000..0x008].copy_from_slice(b"PS-X EXE");
        exe_bytes[0x010..0x014].copy_from_slice(&0x8001_0000u32.to_le_bytes()); // initial_pc
        exe_bytes[0x014..0x018].copy_from_slice(&0x8001_8000u32.to_le_bytes()); // initial_gp
        exe_bytes[0x018..0x01C].copy_from_slice(&0x8001_0000u32.to_le_bytes()); // load_addr
        exe_bytes[0x01C..0x020].copy_from_slice(&16u32.to_le_bytes());          // load_size
        exe_bytes[0x030..0x034].copy_from_slice(&0x801F_F000u32.to_le_bytes()); // sp_base
        exe_bytes[0x034..0x038].copy_from_slice(&0x0000_0F00u32.to_le_bytes()); // sp_offset

        let payload_words: [u32; 4] = [0x3408_0042, 0x3409_1337, 0x0000_0000, 0x03E0_0008];
        for (i, &w) in payload_words.iter().enumerate() {
            let off = PS_EXE_HEADER_SIZE + i * 4;
            exe_bytes[off..off + 4].copy_from_slice(&w.to_le_bytes());
        }

        let mut bus = Bus::new();
        let parsed = bus.queue_exe_bytes(&exe_bytes).expect("Valid PS-X EXE should parse");
        assert_eq!(parsed.initial_pc, 0x8001_0000);
        assert_eq!(parsed.initial_gp, 0x8001_8000);
        assert_eq!(parsed.load_addr, 0x8001_0000);
        assert_eq!(parsed.load_size, 16);
        assert_eq!(parsed.sp_base, 0x801F_F000);
        assert_eq!(parsed.sp_offset, 0x0000_0F00);
        assert_eq!(parsed.initial_sp, 0x801F_FF00);
        assert_eq!(parsed.payload.len(), 16);
        assert!(bus.pending_exe.is_some());

        bus.load_exe_payload(&parsed);
        assert_eq!(bus.read32(0x8001_0000), 0x3408_0042);
        assert_eq!(bus.read32(0x8001_0004), 0x3409_1337);
        assert_eq!(bus.read32(0x8001_0008), 0x0000_0000);
        assert_eq!(bus.read32(0x8001_000C), 0x03E0_0008);

        // Verify invalid magic and short buffer errors
        let mut bad_magic = exe_bytes.clone();
        bad_magic[0..4].copy_from_slice(b"NOPE");
        assert!(PsExe::from_bytes(&bad_magic).is_err());
        assert!(PsExe::from_bytes(&[0u8; 128]).is_err());
    }

    #[test]
    fn test_cdrom_bank_switching_test_20_and_getid_two_stage_irq_latch() {
        let mut bus = Bus::new();

        // 1. Verify bank switching (0x1F80_1800 bits 0..1) and initial PRMEMPT (bit 3) | PRMWRDY (bit 4) = 0x18
        for bank in 0..4u8 {
            bus.write8(0x1F80_1800, bank, false);
            assert_eq!(bus.read8(0x1F80_1800), 0x18 | bank);
        }

        // Enable CD-ROM interrupts (HINTMSK = 0x1F via Index 1 at 0x1F80_1802) and CPU IRQ2 in I_MASK
        bus.write8(0x1F80_1800, 1, false);
        bus.write8(0x1F80_1802, 0x1F, false);
        bus.write16(0x1F80_1074, 1 << 2, false);

        // Verify HINTMSK readback at 0x1F80_1803 on Index 0 and Index 2 returns 0xE0 | 0x1F = 0xFF
        bus.write8(0x1F80_1800, 0, false);
        assert_eq!(bus.read8(0x1F80_1803), 0xFF);
        bus.write8(0x1F80_1800, 2, false);
        assert_eq!(bus.read8(0x1F80_1803), 0xFF);

        // 2. Execute Test(0x19, 0x20) -> INT3(0x98, 0x06, 0x10, 0xC3) + IRQ2
        bus.write8(0x1F80_1800, 0, false);
        bus.write8(0x1F80_1802, 0x20, false); // Push sub-function 0x20 to Parameter FIFO
        assert_eq!(bus.read8(0x1F80_1800) & (1 << 3), 0); // PRMEMPT cleared
        assert_ne!(bus.read8(0x1F80_1800) & (1 << 4), 0); // PRMWRDY still set (< 16 bytes)

        bus.write8(0x1F80_1801, 0x19, false); // Command 0x19 (Test)
        assert_ne!(bus.read8(0x1F80_1800) & (1 << 7), 0); // BUSYSTS = 1 while in flight
        assert_ne!(bus.read8(0x1F80_1800) & (1 << 3), 0); // Parameter FIFO consumed (PRMEMPT = 1)

        bus.tick(CD_INT3_DELAY_CYCLES - 1);
        bus.write8(0x1F80_1800, 1, false);
        assert_eq!(bus.read8(0x1F80_1803) & 0x07, 0);
        assert_eq!(bus.read32(0x1F80_1070) & (1 << 2), 0);

        bus.tick(1);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3); // INT3 active in HINTSTS
        assert_ne!(bus.read8(0x1F80_1800) & (1 << 5), 0); // RSLRRDY = 1
        assert_eq!(bus.read8(0x1F80_1800) & (1 << 7), 0); // BUSYSTS = 0
        assert_ne!(bus.read32(0x1F80_1070) & (1 << 2), 0); // I_STAT bit 2 (IRQ2)
        assert!(bus.irq_pending());

        let test_resp = [
            bus.read8(0x1F80_1801),
            bus.read8(0x1F80_1801),
            bus.read8(0x1F80_1801),
            bus.read8(0x1F80_1801),
        ];
        assert_eq!(test_resp, [0x98, 0x06, 0x10, 0xC3]);
        assert_eq!(bus.read8(0x1F80_1800) & (1 << 5), 0); // Response FIFO now empty

        // Acknowledge INT3 via HCLRCTL (Index 1 at 0x1F80_1803 = 0x07) and clear I_STAT
        bus.write8(0x1F80_1803, 0x07, false);
        bus.write16(0x1F80_1070, !(1 << 2), false);
        assert_eq!(bus.read8(0x1F80_1803), 0xE0);

        // 3. Two-Stage GetID(0x1A) with NO disc loaded -> INT3(0x10) then INT5(0x08, 0x40, ...)
        bus.write8(0x1F80_1801, 0x1A, false);
        bus.tick(CD_INT3_DELAY_CYCLES);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3); // First-stage INT3
        assert_eq!(bus.read8(0x1F80_1801), 0x10); // Shell open / No disc status

        // CRITICAL Hardware Latch Rule: ticking past second-stage delay while INT3 is unacknowledged
        // MUST NOT overwrite cd_irq_flags or deliver INT5 until HCLRCTL clears INT3!
        bus.tick(CD_INT2_DELAY_CYCLES + 500);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3);
        assert_eq!(bus.read8(0x1F80_1800) & (1 << 5), 0);

        // Now acknowledge INT3 -> since second-stage timer already elapsed, INT5 delivers immediately!
        bus.write16(0x1F80_1070, !(1 << 2), false);
        bus.write8(0x1F80_1803, 0x07, false);
        assert_eq!(bus.read8(0x1F80_1803), 0xE5); // Second-stage INT5 (Error / No Disc)
        assert_ne!(bus.read32(0x1F80_1070) & (1 << 2), 0);
        let mut nodisc_resp = [0u8; 8];
        for b in &mut nodisc_resp {
            *b = bus.read8(0x1F80_1801);
        }
        assert_eq!(nodisc_resp, [0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        bus.write8(0x1F80_1803, 0x07, false);
        bus.write16(0x1F80_1070, !(1 << 2), false);

        // 4. Load disc image and test two-stage GetID(0x1A) -> INT3(0x02) -> ack -> INT2("SCEA")
        bus.load_cd_bin_bytes(vec![0u8; 2048]);
        bus.write8(0x1F80_1801, 0x1A, false);
        bus.tick(CD_INT3_DELAY_CYCLES);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3);
        assert_eq!(bus.read8(0x1F80_1801), 0x02); // Spindle motor on

        // Acknowledge INT3 before second-stage timer elapses -> HINTSTS becomes 0xE0 until 2048 cycles elapse
        bus.write8(0x1F80_1803, 0x07, false);
        bus.write16(0x1F80_1070, !(1 << 2), false);
        assert_eq!(bus.read8(0x1F80_1803), 0xE0);

        bus.tick(CD_INT2_DELAY_CYCLES);
        assert_eq!(bus.read8(0x1F80_1803), 0xE2); // Second-stage INT2 (Complete)
        assert_ne!(bus.read32(0x1F80_1070) & (1 << 2), 0);
        let mut disc_resp = [0u8; 8];
        for b in &mut disc_resp {
            *b = bus.read8(0x1F80_1801);
        }
        assert_eq!(disc_resp, [0x02, 0x00, 0x20, 0x00, b'S', b'C', b'E', b'A']);
    }

    #[test]
    fn test_cdrom_setloc_bcd_readn_bfrd_and_dma3_sector_transfer() {
        let mut bus = Bus::new();

        // Build a raw 2352-byte/sector BIN image with 20 sectors; populate LBA 12 (BCD 00:02:12) Mode2/Form1 payload at +24
        let total_sectors = 20usize;
        let target_lba = 12usize;
        let mut bin_image = vec![0u8; total_sectors * CD_SECTOR_SIZE_RAW_FULL];
        let sync: [u8; 12] = [0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00];
        bin_image[0..12].copy_from_slice(&sync);

        let sector_base = target_lba * CD_SECTOR_SIZE_RAW_FULL;
        bin_image[sector_base..sector_base + 12].copy_from_slice(&sync);
        bin_image[sector_base + 15] = 0x02; // Mode 2 (user data at +24..+2072)

        let mut expected_sector = [0u8; CD_SECTOR_SIZE_COOKED];
        let sys_cnf = b"BOOT = cdrom:SLUS_000.01;1";
        expected_sector[..sys_cnf.len()].copy_from_slice(sys_cnf);
        for i in sys_cnf.len()..CD_SECTOR_SIZE_COOKED {
            expected_sector[i] = ((i * 7 + 3) & 0xFF) as u8;
        }
        bin_image[sector_base + 24..sector_base + 24 + CD_SECTOR_SIZE_COOKED].copy_from_slice(&expected_sector);

        bus.load_cd_bin_bytes(bin_image);
        assert!(bus.cd_disc_present);
        assert!(bus.cd_raw_2352);

        // Enable all CD-ROM IRQs (HINTMSK = 0x1F)
        bus.write8(0x1F80_1800, 1, false);
        bus.write8(0x1F80_1802, 0x1F, false);

        // 1. Setloc(0x02) with BCD (0x00, 0x02, 0x12) -> LBA = ((0*60 + 2)*75 + 12) - 150 = 12
        bus.write8(0x1F80_1800, 0, false);
        bus.write8(0x1F80_1802, 0x00, false); // MM = 00
        bus.write8(0x1F80_1802, 0x02, false); // SS = 02
        bus.write8(0x1F80_1802, 0x12, false); // FF = 12
        bus.write8(0x1F80_1801, 0x02, false); // Setloc
        assert_eq!(bus.seek_lba, 12);

        bus.tick(CD_INT3_DELAY_CYCLES);
        bus.write8(0x1F80_1800, 1, false);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3);
        assert_eq!(bus.read8(0x1F80_1801), 0x02);
        bus.write8(0x1F80_1803, 0x07, false); // Acknowledge Setloc INT3

        // 2. ReadN(0x06) -> INT3 then periodic INT1 with motor + reading status.
        bus.write8(0x1F80_1801, 0x06, false);
        assert_eq!(bus.read_lba, 12);
        assert_eq!(bus.cd_stat, 0x22);

        bus.tick(CD_INT3_DELAY_CYCLES);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3);
        assert_eq!(bus.read8(0x1F80_1801), 0x22);
        bus.write8(0x1F80_1803, 0x07, false); // Acknowledge ReadN INT3

        bus.tick(bus.cd_sector_delay());
        assert_eq!(bus.read8(0x1F80_1803), 0xE1); // INT1 (DataReady)
        assert_eq!(bus.read8(0x1F80_1801), 0x22);
        assert_eq!(bus.read_lba, 13);

        // Before setting BFRD (0x1F80_1803 Index 0 bit 7), DRQSTS (bit 6 of 0x1F80_1800) must be 0
        assert_eq!(bus.read8(0x1F80_1800) & (1 << 6), 0);

        // 3. Activate BFRD (0x80 to 0x1F80_1803 Index 0) -> loads sector buffer into Data FIFO and sets DRQSTS = 1
        bus.write8(0x1F80_1800, 0, false);
        bus.write8(0x1F80_1803, 0x80, false);
        assert_ne!(bus.read8(0x1F80_1800) & (1 << 6), 0);

        // 4. Execute DMA3 Burst Transfer (512 words = 2048 bytes) into Main RAM at 0x0001_0000
        bus.write32(0x1F80_10F4, (1 << 23) | (1 << 19), false); // Enable DMA3 IRQ in DICR
        bus.write32(0x1F80_10B0, 0x0001_0000, false); // D3_MADR
        bus.write32(0x1F80_10B4, 512, false);         // D3_BCR (512 words = 2048 bytes)
        bus.write32(0x1F80_10B8, 0x1100_0000, false); // D3_CHCR (To RAM, SyncMode 0, Start + Trigger)

        // Verify all 2048 bytes in Main RAM match the LBA 12 Mode2/Form1 payload and Data FIFO is now empty (DRQSTS = 0)
        assert_eq!(&bus.ram[0x10000..0x10000 + CD_SECTOR_SIZE_COOKED], &expected_sector[..]);
        assert_eq!(bus.read8(0x1F80_1800) & (1 << 6), 0);
        assert_ne!(bus.read32(0x1F80_10F4) & (1 << 27), 0); // DICR channel 3 IRQ flag
        assert_ne!(bus.i_stat & (1 << 3), 0);               // I_STAT bit 3 (DMA IRQ3)

        // 5. Pause(0x09) stops reading and clears the activity bits of cd_stat
        bus.write8(0x1F80_1800, 1, false);
        bus.write8(0x1F80_1803, 0x07, false); // Acknowledge INT1
        bus.write8(0x1F80_1801, 0x09, false); // Pause
        bus.tick(CD_INT3_DELAY_CYCLES);
        assert_eq!(bus.read8(0x1F80_1803), 0xE3);
        assert_eq!(bus.read8(0x1F80_1801), 0x02);
        bus.write8(0x1F80_1803, 0x07, false);
        bus.tick(CD_INT2_DELAY_CYCLES);
        assert_eq!(bus.read8(0x1F80_1803), 0xE2);
        assert_eq!(bus.read8(0x1F80_1801), 0x02);
    }

    #[test]
    fn test_cdrom_sector_cadence_and_location_metadata() {
        for (mode, interval) in [(0, CPU_CLOCK_HZ / 75), (0x80, CPU_CLOCK_HZ / 150)] {
            let mut bus = Bus::new();
            let mut image = vec![0; 2352 * 3];
            image[1..11].fill(0xff);
            let header = [0x00, 0x02, 0x01, 0x02, 0x07, 0x03, 0x48, 0x11];
            image[2352 + 12..2352 + 20].copy_from_slice(&header);
            bus.load_cd_bin_bytes(image);
            bus.cd_mode = mode;
            bus.seek_lba = 1;
            // Deliberately different filter values: GetlocL must return disc bytes.
            bus.cd_filter_file = 9;
            bus.cd_filter_chan = 8;
            bus.write_cd_command(0x06);
            bus.tick_cdrom(CD_INT3_DELAY_CYCLES);
            bus.write_cd_reg(0, 1);
            bus.write_cd_reg(3, 7);
            bus.tick_cdrom(interval - 1);
            assert_eq!(bus.cd_irq_flags, 0);
            assert_eq!(bus.read_lba, 1);
            bus.tick_cdrom(1);
            assert_eq!(bus.cd_irq_flags, 1);
            assert_eq!(bus.read_lba, 2);
            assert_eq!(bus.cd_last_header, header);
            assert_eq!(bus.cd_position_lba, 1);
            bus.write_cd_reg(3, 7);
            // Stop further sector events so the command response is isolated.
            bus.cd_reading = false;
            bus.cd_pending_irqs.clear();
            bus.write_cd_command(0x10);
            bus.tick_cdrom(CD_INT3_DELAY_CYCLES);
            assert_eq!(bus.cd_resp_fifo.iter().copied().collect::<Vec<_>>(), header);
            bus.write_cd_reg(3, 7);
            bus.write_cd_command(0x11);
            bus.tick_cdrom(CD_INT3_DELAY_CYCLES);
            assert_eq!(bus.cd_resp_fifo.iter().copied().collect::<Vec<_>>(), [1, 1, 0, 0, 1, 0, 2, 1]);
        }
    }

    #[test]
    fn test_xa_audio_sectors_do_not_interrupt_the_video_stream() {
        for xa_enabled in [false, true] {
            let mut bus = Bus::new();
            let mut image = vec![0; 2352 * 3];
            for i in 0..3 {
                let sector = &mut image[i * 2352..(i + 1) * 2352];
                sector[1..11].fill(0xff);
                sector[15] = 2;
                sector[18] = if i == 1 { 0x64 } else { 0x48 };
                sector[24] = 0x60 + i as u8;
            }
            bus.load_cd_bin_bytes(image);
            bus.cd_mode = if xa_enabled { 0xe0 } else { 0xa0 };
            bus.write_cd_command(0x1b);
            bus.tick_cdrom(CD_INT3_DELAY_CYCLES);
            bus.write_cd_reg(0, 1);
            bus.write_cd_reg(3, 7);
            bus.tick_cdrom(bus.cd_sector_delay());
            assert_eq!(bus.cd_irq_flags, 1);
            bus.write_cd_reg(3, 7);
            bus.tick_cdrom(bus.cd_sector_delay());
            assert_eq!(bus.read_lba, 2);
            if xa_enabled {
                assert_eq!(bus.cd_irq_flags, 0, "XA audio must bypass the CPU movie parser");
                bus.tick_cdrom(bus.cd_sector_delay());
                assert_eq!(bus.read_lba, 3);
                assert_eq!(bus.cd_irq_flags, 1);
                assert_eq!(bus.cd_sector_buffer[12], 0x62);
            } else {
                assert_eq!(bus.cd_irq_flags, 1, "raw audio remains readable when XA decoding is off");
                assert_eq!(bus.cd_sector_buffer[12], 0x61);
            }
        }
    }

    #[test]
    fn test_mdec_output_dma_waits_for_decoded_data() {
        let mut bus = Bus::new();
        bus.write32(0x1f8010f0, 0x08888888, false);
        bus.write_mdec1(0xe0000000);
        bus.write32(0x10000, 0xdeadbeef, false);
        bus.write32(0x1f8010f4, (1 << 23) | (1 << 17), false);
        bus.write32(0x1f801090, 0x10000, false);
        bus.write32(0x1f801094, (4 << 16) | 32, false);
        bus.write32(0x1f801098, 0x01000200, false);
        assert_eq!(bus.read32(0x10000), 0xdeadbeef);
        assert_ne!(bus.dma[1].chcr & (1 << 24), 0);
        assert_eq!(bus.dicr & (1 << 25), 0);
        bus.write_mdec0((1 << 29) | (3 << 27) | 6);
        for _ in 0..6 { bus.write_mdec0(0xfe000000); }
        assert_eq!(bus.dma[1].chcr & (1 << 24), 0);
        assert_ne!(bus.dicr & (1 << 25), 0);
        for addr in (0x10000..0x10200).step_by(4) {
            assert_eq!(bus.read32(addr), 0x42104210);
        }
    }

    #[test]
    fn test_mdec_idct_dc_level_and_ac_orientation() {
        let mut coeff = [0; 64];
        coeff[0] = 64;
        assert_eq!(Bus::mdec_idct_block(&coeff, &MDEC_DEFAULT_SCALE_TABLE), [8; 64]);
        coeff[0] = 0;
        coeff[1] = 128;
        let horizontal = Bus::mdec_idct_block(&coeff, &MDEC_DEFAULT_SCALE_TABLE);
        let expected = [22i32, 19, 13, 4, -4, -13, -19, -22];
        for row in horizontal.chunks(8) {
            for (actual, expected) in row.iter().zip(expected) { assert!((actual - expected).abs() <= 1); }
        }
        coeff[1] = 0;
        coeff[8] = 128;
        let vertical = Bus::mdec_idct_block(&coeff, &MDEC_DEFAULT_SCALE_TABLE);
        for y in 0..8 { for x in 0..8 { assert!((horizontal[y * 8 + x] - vertical[x * 8 + y]).abs() <= 1); } }
    }

    #[test]
    fn test_mdec_rle_scan_order_and_unquantized_mode() {
        let quant = [8; 64];
        let mut cursor = 0;
        // Zigzag entry 2 is the vertical frequency at matrix index 8.
        let block = Bus::mdec_decode_rl_block(&[0x0400, 0x0440, 0xfe00], &mut cursor, &quant, &MDEC_DEFAULT_SCALE_TABLE).unwrap();
        let expected = [11i32, 9, 6, 2, -2, -6, -9, -11];
        for y in 0..8 { for x in 0..8 { assert!((block[y * 8 + x] - expected[y]).abs() <= 1); } }
        // Q-scale zero bypasses zigzag and quantization: entry 2 stays at 2.
        cursor = 0;
        let block = Bus::mdec_decode_rl_block(&[0, 0x0420, 0xfe00], &mut cursor, &quant, &MDEC_DEFAULT_SCALE_TABLE).unwrap();
        let expected = [10i32, 4, -4, -10, -10, -4, 4, 10];
        for row in block.chunks(8) { for (actual, expected) in row.iter().zip(expected) { assert!((actual - expected).abs() <= 1); } }
    }

    #[test]
    fn test_sio0_digital_pad_transaction_ack_irq7_and_dtr_reset() {
        let mut bus = Bus::new();
        assert_eq!(bus.pad1_buttons, 0xFFFF);
        assert_eq!(bus.read16(0x1F80_104E), 0x0088); // Default SIO_BAUD
        assert_eq!(bus.read32(0x1F80_1044), 0x0000_0005); // TX_RDY1 | TX_RDY2, FIFO empty

        // Simulate pressing Cross (bit 14 = 0) and Up (bit 4 = 0) -> 0xFFFF & !(1<<14) & !(1<<4) = 0xBFEF
        bus.set_pad_state(0xEF, 0xBF);
        assert_eq!(bus.pad1_buttons, 0xBFEF);
        // Enable IRQ7 in I_MASK
        bus.write16(0x1F80_1074, 1 << 7, false);
        // Configure SIO_MODE (0x000D) and SIO_BAUD (0x0088)
        bus.write16(0x1F80_1048, 0x000D, false);
        bus.write16(0x1F80_104E, 0x0088, false);
        assert_eq!(bus.read16(0x1F80_1048), 0x000D);
        assert_eq!(bus.read16(0x1F80_104E), 0x0088);

        // Assert /DTR (bit 1), TX_EN (bit 0), and /ACK IRQ Enable (bit 12) on Port 1 (bit 13 = 0)
        bus.write16(0x1F80_104A, 0x1003, false);

        let tx_seq: [u8; 5] = [0x01, 0x42, 0x00, 0x00, 0x00];
        let expected_rx: [u8; 5] = [0xFF, 0x41, 0x5A, 0xEF, 0xBF];

        for step in 0..5 {
            bus.write8(0x1F80_1040, tx_seq[step], false);
            // RX_FIFONOTEMPTY (bit 1) should now be set
            assert_ne!(bus.read32(0x1F80_1044) & (1 << 1), 0);
            let rx = bus.read8(0x1F80_1040);
            assert_eq!(rx, expected_rx[step]);
            // FIFO now empty again
            assert_eq!(bus.read32(0x1F80_1044) & (1 << 1), 0);

            if step < 4 {
                // Before SIO0_ACK_DELAY_CYCLES cycles, /ACK and IRQ7 must not have fired yet
                bus.tick(SIO0_ACK_DELAY_CYCLES - 1);
                assert_eq!(bus.read32(0x1F80_1044) & ((1 << 7) | (1 << 9)), 0);
                assert_eq!(bus.read32(0x1F80_1070) & (1 << 7), 0);

                // On the SIO0_ACK_DELAY_CYCLES-th cycle, /ACK pulses (SIO_STAT.7), SIO_STAT.9 (IRQ) sets, and I_STAT.7 (IRQ7) asserts
                bus.tick(1);
                let stat = bus.read32(0x1F80_1044);
                assert_ne!(stat & (1 << 7), 0, "Expected /ACK bit 7 on step {}", step);
                assert_ne!(stat & (1 << 9), 0, "Expected SIO_STAT.9 IRQ on step {}", step);
                assert_ne!(bus.read32(0x1F80_1070) & (1 << 7), 0, "Expected I_STAT.7 IRQ7 on step {}", step);
                assert!(bus.irq_pending());

                // Acknowledge SIO IRQ via SIO_CTRL bit 4 and clear I_STAT.7
                bus.write16(0x1F80_104A, 0x1003 | (1 << 4), false);
                bus.write16(0x1F80_1070, !(1 << 7), false);
                assert_eq!(bus.read32(0x1F80_1044) & (1 << 9), 0);
                assert_eq!(bus.read32(0x1F80_1070) & (1 << 7), 0);
            } else {
                // Step 4 (final byte): NO /ACK pulse and NO IRQ7
                bus.tick(SIO0_ACK_DELAY_CYCLES);
                assert_eq!(bus.read32(0x1F80_1044) & ((1 << 7) | (1 << 9)), 0);
                assert_eq!(bus.read32(0x1F80_1070) & (1 << 7), 0);
            }
        }

        // Verify mid-transfer /DTR deassertion resets the state machine back to byte 0
        bus.write16(0x1F80_104A, 0x0000, false); // Deassert /DTR
        bus.write16(0x1F80_104A, 0x1003, false); // Reassert /DTR
        bus.write8(0x1F80_1040, 0x01, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);
        bus.write8(0x1F80_1040, 0x42, false);
        assert_eq!(bus.read8(0x1F80_1040), 0x41);
        // Abort after step 1 by deasserting /DTR
        bus.write16(0x1F80_104A, 0x0000, false);
        bus.write16(0x1F80_104A, 0x1003, false);
        // Should be back at step 0 (0x01 -> 0xFF), not step 2 (0x5A)
        bus.write8(0x1F80_1040, 0x01, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);

        // Verify Memory Card address (0x81) and Port 2 (bit 13 = 1) return 0xFF with no /ACK
        bus.write16(0x1F80_104A, 0x0000, false);
        bus.write16(0x1F80_104A, 0x1003, false);
        bus.write8(0x1F80_1040, 0x81, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);
        bus.tick(64);
        assert_eq!(bus.read32(0x1F80_1044) & ((1 << 7) | (1 << 9)), 0);

        bus.write16(0x1F80_104A, 0x3003, false); // Port 2 selected (bit 13 = 1)
        bus.write8(0x1F80_1040, 0x01, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);
        bus.tick(64);
        assert_eq!(bus.read32(0x1F80_1044) & ((1 << 7) | (1 << 9)), 0);

    }
    #[test]
    fn test_spu_cnt_stat_mirroring_manual_ram_transfer_dma4_and_adpcm_decode() {
        let mut bus = Bus::new();
        assert_eq!(bus.spu_ram.len(), 512 * 1024);
        assert_eq!(bus.read16(0x1F80_1DAC), 0x0004); // Default SPU_RAM_DTC

        // 1. Verify SPUCNT (0x1F80_1DAA) bits 0..5 mirror into SPUSTAT (0x1F80_1DAE) and DMA request bits 7..9
        bus.write16(0x1F80_1DAA, 0xC015, false); // Transfer mode = 1 (Manual Write, bits 4..5 = 01), low6 = 0x15
        assert_eq!(bus.read16(0x1F80_1DAA), 0xC015);
        let stat_manual = bus.read16(0x1F80_1DAE);
        assert_eq!(stat_manual & 0x003F, 0x0015);
        assert_eq!(stat_manual & (1 << 10), 0); // Transfer Busy bit 10 always 0 so polling loops never hang

        // DMA Write mode (bits 4..5 = 2 -> 0x0020) sets SPUSTAT bits 7 and 8 (0x0180)
        bus.write16(0x1F80_1DAA, 0xC02A, false);
        let stat_dma_write = bus.read16(0x1F80_1DAE);
        assert_eq!(stat_dma_write & 0x003F, 0x002A);
        assert_eq!(stat_dma_write & 0x0380, (1 << 7) | (1 << 8));

        // DMA Read mode (bits 4..5 = 3 -> 0x0030) sets SPUSTAT bits 7 and 9 (0x0280)
        bus.write16(0x1F80_1DAA, 0xC03F, false);
        let stat_dma_read = bus.read16(0x1F80_1DAE);
        assert_eq!(stat_dma_read & 0x003F, 0x003F);
        assert_eq!(stat_dma_read & 0x0380, (1 << 7) | (1 << 9));

        // Verify 24 voice channel registers, Key On (0x1F80_1D88), Key Off (0x1F80_1D8C), and ENDX (0x1F80_1D9C)
        bus.write16(0x1F80_1C06, 0x0200, false); // Voice 0 Start Address
        assert_eq!(bus.read16(0x1F80_1C06), 0x0200);
        assert_eq!(bus.read16(0x1F80_1C0E), 0x0200); // Repeat Address latched from Start Address
        bus.write32(0x1F80_1D88, 0x0080_0005, false); // Key On voices 0, 2, 23
        assert_eq!(bus.read32(0x1F80_1D88), 0x0080_0005);
        assert_eq!(bus.read32(0x1F80_1D9C) & 0x0080_0005, 0); // ENDX bits cleared for keyed-on voices
        assert_eq!(bus.read16(0x1F80_1C0C), 0x7FFF); // Voice 0 ENVX active
        bus.write32(0x1F80_1D8C, 0x0000_0001, false); // Key Off voice 0
        assert_eq!(bus.read16(0x1F80_1C0C), 0x0000);

        // 2. Manual Sound RAM transfer via SPU_RAM_ADDR (0x1F80_1DA6 * 8 multiplier) and SPU_RAM_DATA (0x1F80_1DA8) + IRQ9
        bus.write16(0x1F80_1074, 1 << 9, false); // Enable IRQ9 in I_MASK
        bus.write16(0x1F80_1DA4, 0x0100, false); // SPU IRQ address = 0x0100 * 8 = 0x0800
        bus.write16(0x1F80_1DAA, 0xC055, false); // SPUCNT with bit 6 (IRQ9 Enable) = 1
        bus.write16(0x1F80_1DA6, 0x0100, false); // SPU_RAM_ADDR = 0x0100 * 8 = 0x0800
        assert_eq!(bus.spu_ram_addr, 0x0800);
        assert_ne!(bus.read16(0x1F80_1DAE) & 0x0040, 0); // SPUSTAT bit 6 (IRQ9 flag) set on address match
        assert_ne!(bus.i_stat & (1 << 9), 0);            // I_STAT bit 9 asserted

        // Clearing SPUCNT bit 6 clears SPUSTAT bit 6
        bus.write16(0x1F80_1DAA, 0xC015, false);
        assert_eq!(bus.read16(0x1F80_1DAE) & 0x0040, 0);

        // Write 3 halfwords manually at 0x0800 and read them back
        bus.write16(0x1F80_1DA6, 0x0100, false);
        bus.write16(0x1F80_1DA8, 0x1122, false);
        bus.write16(0x1F80_1DA8, 0x3344, false);
        bus.write16(0x1F80_1DA8, 0x5566, false);
        assert_eq!(bus.spu_ram_addr, 0x0806);
        assert_eq!(&bus.spu_ram[0x0800..0x0806], &[0x22, 0x11, 0x44, 0x33, 0x66, 0x55]);

        bus.write16(0x1F80_1DA6, 0x0100, false);
        assert_eq!(bus.read16(0x1F80_1DA8), 0x1122);
        assert_eq!(bus.read16(0x1F80_1DA8), 0x3344);
        assert_eq!(bus.read16(0x1F80_1DA8), 0x5566);
        assert_eq!(bus.spu_ram_addr, 0x0806);

        // 3. DMA4 (SPU DMA) RAM -> SPU RAM (dir_from_ram = true) and SPU RAM -> RAM (dir_from_ram = false)
        let dma_words: [u32; 4] = [0xDEAD_BEEF, 0xCAFE_BABE, 0x1357_2468, 0x89AB_CDEF];
        for (i, &w) in dma_words.iter().enumerate() {
            bus.write32(0x0000_4000 + (i as u32) * 4, w, false);
        }
        bus.write16(0x1F80_1DA6, 0x0200, false); // spu_ram_addr = 0x1000
        bus.write16(0x1F80_1DAA, 0xC020, false); // DMA Write mode
        bus.write32(0x1F80_10C0, 0x0000_4000, false); // D4_MADR
        bus.write32(0x1F80_10C4, (1 << 16) | 4, false); // D4_BCR: BS=4, BA=1 (SyncMode 1)
        bus.write32(0x1F80_10C8, 0x0100_0201, false); // D4_CHCR: From RAM, SyncMode 1, Start
        assert_eq!(bus.spu_ram_addr, 0x1010);

        // Read back from SPU RAM (0x1000) into Main RAM (0x0000_5000) via DMA4 (dir_from_ram = false)
        bus.write16(0x1F80_1DA6, 0x0200, false); // Reset spu_ram_addr = 0x1000
        bus.write16(0x1F80_1DAA, 0xC030, false); // DMA Read mode
        bus.write32(0x1F80_10C0, 0x0000_5000, false); // D4_MADR
        bus.write32(0x1F80_10C4, (1 << 16) | 4, false); // D4_BCR
        bus.write32(0x1F80_10C8, 0x0100_0200, false); // D4_CHCR: To RAM, SyncMode 1, Start
        assert_eq!(bus.spu_ram_addr, 0x1010);
        for (i, &expected) in dma_words.iter().enumerate() {
            assert_eq!(bus.read32(0x0000_5000 + (i as u32) * 4), expected);
        }

        // 4. Verify PS1 16-byte 4-bit ADPCM block decoding (decode_spu_adpcm_block)
        // Filter 0 (f0=0, f1=0), Shift = 2 -> nibble 1 = (1 << 12) >> 2 = 1024; nibble 0xF (-1) = (-1 << 12) >> 2 = -1024
        let mut adpcm_block0 = [0u8; 16];
        adpcm_block0[0] = 0x02; // filter = 0, shift = 2
        adpcm_block0[1] = 0x00;
        for b in &mut adpcm_block0[2..16] {
            *b = 0xF1; // low nibble = 1 (+1024), high nibble = 0xF (-1024)
        }
        let mut prev1 = 0i32;
        let mut prev2 = 0i32;
        let pcm0 = decode_spu_adpcm_block(&adpcm_block0, &mut prev1, &mut prev2);
        for i in 0..14 {
            assert_eq!(pcm0[i * 2], 1024);
            assert_eq!(pcm0[i * 2 + 1], -1024);
        }
        assert_eq!(prev1, -1024);
        assert_eq!(prev2, 1024);

        // Filter 2 (pos_f = 115, neg_f = -52), Shift = 6 -> nibble 4 = +256
        let mut adpcm_block2 = [0u8; 16];
        adpcm_block2[0] = 0x26; // filter = 2, shift = 6
        adpcm_block2[1] = 0x03; // loop end + repeat flags
        for b in &mut adpcm_block2[2..16] {
            *b = 0x44; // both nibbles = 4 (+256)
        }
        let mut p1 = 0i32;
        let mut p2 = 0i32;
        let pcm2 = decode_spu_adpcm_block(&adpcm_block2, &mut p1, &mut p2);
        // Sample 0: 256 + ((0*115 + 0*-52 + 32) >> 6) = 256
        // Sample 1: 256 + ((256*115 + 0*-52 + 32) >> 6) = 256 + 460 = 716
        // Sample 2: 256 + ((716*115 + 256*-52 + 32) >> 6) = 256 + 1079 = 1335
        assert_eq!(pcm2[0], 256);
        assert_eq!(pcm2[1], 716);
        assert_eq!(pcm2[2], 1335);
        assert_eq!(p1, pcm2[27] as i32);
        assert_eq!(p2, pcm2[26] as i32);
    }

    #[test]
    fn test_mdec_reset_quant_scale_tables_and_dma0_dma1_macroblock_decode() {
        let mut bus = Bus::new();

        // 1. MDEC Reset (0x8000_0000 written to 0x1F80_1824 -> status 0x8004_0000)
        bus.write32(0x1F80_1824, 0x8000_0000, false);
        assert_eq!(bus.read32(0x1F80_1824), 0x8004_0000);

        // Enable DMA0 Data-In (bit 30) and DMA1 Data-Out (bit 29)
        bus.write32(0x1F80_1824, (1 << 30) | (1 << 29), false);
        assert_eq!(bus.read32(0x1F80_1824), 0x8004_0000);

        // 2. Command 2: Set Quant Table (64 Luma + 64 Chroma bytes = 32 words) via DMA0
        // Set all quant entries to 2 (0x0202_0202)
        bus.write32(0x1F80_1820, (2 << 29) | 1, false); // Command 2, bit 0 = 1 (Luma + Chroma -> 32 words)
        let stat_cmd2 = bus.read32(0x1F80_1824);
        assert_ne!(stat_cmd2 & (1 << 29), 0); // Command Busy = 1
        assert_ne!(stat_cmd2 & (1 << 28), 0); // DMA0 Data-In Request = 1
        assert_eq!(stat_cmd2 & 0xFFFF, 31);   // words_remaining - 1 = 31

        for i in 0..32u32 {
            bus.write32(0x0000_6000 + i * 4, 0x0202_0202, false);
        }
        bus.write32(0x1F80_1080, 0x0000_6000, false); // D0_MADR
        bus.write32(0x1F80_1084, (1 << 16) | 32, false); // D0_BCR: 32 words
        bus.write32(0x1F80_1088, 0x0100_0201, false); // D0_CHCR: From RAM, SyncMode 1, Start

        assert_eq!(bus.mdec_words_remaining, 0);
        assert!(bus.mdec_quant_y.iter().all(|&q| q == 2));
        assert!(bus.mdec_quant_uv.iter().all(|&q| q == 2));
        assert_eq!(bus.read32(0x1F80_1824) & (1 << 29), 0); // Command Busy cleared

        // 3. Command 3: Set Scale Table (64 i16 values = 32 words)
        bus.write32(0x1F80_1820, 3 << 29, false);
        assert_eq!(bus.read32(0x1F80_1824) & 0xFFFF, 31);
        for pair in MDEC_DEFAULT_SCALE_TABLE.chunks(2) {
            let w = ((pair[0] as u16) as u32) | (((pair[1] as u16) as u32) << 16);
            bus.write32(0x1F80_1820, w, false);
        }
        assert_eq!(bus.mdec_words_remaining, 0);
        assert_eq!(bus.mdec_scale_table, MDEC_DEFAULT_SCALE_TABLE);

        // 4. Command 1: Decode 15-bit RGB 16x16 Macroblock (depth=3, signed=0, bit15=1) via DMA0 and read back via DMA1
        // 6 blocks: Cr (DC=0), Cb (DC=0), Y1..Y4 (q_scale=1, DC=32 -> dequantized DC = 32*2 = 64 -> IDCT = +8 -> unsigned Y = 8 + 128 = 136)
        // Neutral chroma and Y=136 produce channel value 17, pixel 0xC631 (bit15 set).
        // Each RLE block takes 2 halfwords (1 word): [DC_hw, 0xFE00]
        let cr_cb_word: u32 = ((1u32 << 10) | 0) | (0xFE00u32 << 16);
        let y_word: u32 = ((1u32 << 10) | 32) | (0xFE00u32 << 16);
        let mb_words: [u32; 6] = [cr_cb_word, cr_cb_word, y_word, y_word, y_word, y_word];
        for (i, &w) in mb_words.iter().enumerate() {
            bus.write32(0x0000_7000 + (i as u32) * 4, w, false);
        }

        // Issue Command 1: cmd=1 (bits 31..29 = 1), depth=3 (bits 28..27 = 3), signed=0 (bit 26 = 0), bit15=1 (bit 25 = 1), words = 6
        let cmd1_word: u32 = (1 << 29) | (3 << 27) | (1 << 25) | 6;
        bus.write32(0x1F80_1820, cmd1_word, false);
        let stat_before_dma0 = bus.read32(0x1F80_1824);
        assert_ne!(stat_before_dma0 & (1 << 29), 0); // Command Busy
        assert_ne!(stat_before_dma0 & (1 << 28), 0); // DMA0 In Request
        assert_eq!((stat_before_dma0 >> 25) & 0x03, 3); // Depth = 3 (15-bit)
        assert_ne!(stat_before_dma0 & (1 << 23), 0); // Output Bit15 = 1
        assert_eq!(stat_before_dma0 & 0xFFFF, 5); // 6 - 1 = 5

        // Stream the 6 compressed words via DMA0 (MDECin)
        bus.write32(0x1F80_1080, 0x0000_7000, false); // D0_MADR
        bus.write32(0x1F80_1084, (1 << 16) | 6, false); // D0_BCR: 6 words
        bus.write32(0x1F80_1088, 0x0100_0201, false); // D0_CHCR: From RAM, SyncMode 1, Start

        // Verify Macroblock decoded 128 words (256 16-bit pixels) into mdec_out_fifo
        assert_eq!(bus.mdec_out_fifo.len(), 128);
        let stat_after_decode = bus.read32(0x1F80_1824);
        assert_eq!(stat_after_decode & (1 << 31), 0); // Data-Out FIFO Empty = 0
        assert_eq!(stat_after_decode & (1 << 29), 0); // Command Busy = 0
        assert_ne!(stat_after_decode & (1 << 27), 0); // DMA1 Data-Out Request = 1

        // Read all 128 decoded words (256 pixels) into Main RAM at 0x0000_8000 via DMA1 (MDECout)
        bus.write32(0x1F80_1090, 0x0000_8000, false); // D1_MADR
        bus.write32(0x1F80_1094, (4 << 16) | 32, false); // D1_BCR: BS=32, BA=4 -> 128 words
        bus.write32(0x1F80_1098, 0x0100_0200, false); // D1_CHCR: To RAM, SyncMode 1, Start

        assert!(bus.mdec_out_fifo.is_empty());
        assert_ne!(bus.read32(0x1F80_1824) & (1 << 31), 0); // Data-Out FIFO Empty = 1 again
        for i in 0..128u32 {
            assert_eq!(bus.read32(0x0000_8000 + i * 4), 0xC631_C631);
        }
    }

    #[test]
    fn test_gp1_display_mode_and_viewport_tracking() {
        let mut bus = Bus::new();
        assert_eq!(bus.display_width, 320);
        assert_eq!(bus.display_height, 240);
        assert_eq!(bus.display_vram_x, 0);
        assert_eq!(bus.display_vram_y, 0);

        // GP1(0x05): Display VRAM Start (x=64, y=120)
        bus.write32(0x1F80_1814, 0x0500_0000 | (120 << 10) | 64, false);
        assert_eq!(bus.display_vram_x, 64);
        assert_eq!(bus.display_vram_y, 120);

        // GP1(0x08): 256x240
        bus.write32(0x1F80_1814, 0x0800_0000, false);
        assert_eq!(bus.display_width, 256);
        assert_eq!(bus.display_height, 240);

        // GP1(0x08): 320x240
        bus.write32(0x1F80_1814, 0x0800_0001, false);
        assert_eq!(bus.display_width, 320);
        assert_eq!(bus.display_height, 240);

        // GP1(0x08): 512x240
        bus.write32(0x1F80_1814, 0x0800_0002, false);
        assert_eq!(bus.display_width, 512);
        assert_eq!(bus.display_height, 240);

        // GP1(0x08): 640x480 (hres=3, vres=1)
        bus.write32(0x1F80_1814, 0x0800_0007, false);
        assert_eq!(bus.display_width, 640);
        assert_eq!(bus.display_height, 480);

        // GP1(0x08): 368x480 (bit 6 set, bit 2 set)
        bus.write32(0x1F80_1814, 0x0800_0044, false);
        assert_eq!(bus.display_width, 368);
        assert_eq!(bus.display_height, 480);

        // GP1(0x00): Reset GPU
        bus.write32(0x1F80_1814, 0x0000_0000, false);
        assert_eq!(bus.display_width, 320);
        assert_eq!(bus.display_height, 240);
        assert_eq!(bus.display_vram_x, 0);
        assert_eq!(bus.display_vram_y, 0);
    }

    #[test]
    fn test_sio0_memory_card_probe_and_control_resets() {
        let mut bus = Bus::new();
        // Enable IRQ7 in I_MASK
        bus.write16(0x1F80_1074, 1 << 7, false);

        // 1. Assert /DTR, TX_EN, ACKIEN on Port 1
        bus.write16(0x1F80_104A, 0x1003, false);

        // Probe Memory Card (0x81)
        bus.write8(0x1F80_1040, 0x81, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);
        // Ensure no ACK pulse or IRQ even after extended cycle ticks
        bus.tick(1000);
        assert_eq!(bus.read32(0x1F80_1044) & ((1 << 7) | (1 << 9)), 0);
        assert_eq!(bus.read32(0x1F80_1070) & (1 << 7), 0);

        // Subsequent controller poll (0x01) without /DTR toggle must succeed immediately
        bus.write8(0x1F80_1040, 0x01, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);
        bus.tick(SIO0_ACK_DELAY_CYCLES);
        assert_ne!(bus.read32(0x1F80_1044) & (1 << 7), 0); // ACK bit 7 active
        assert_ne!(bus.read32(0x1F80_1070) & (1 << 7), 0); // I_STAT bit 7 set

        // 2. Acknowledge SIO IRQ via SIO_CTRL bit 4 (0x0010) must directly clear I_STAT.7
        bus.write16(0x1F80_104A, 0x1003 | (1 << 4), false);
        assert_eq!(bus.read32(0x1F80_1044) & (1 << 9), 0);
        assert_eq!(bus.read32(0x1F80_1070) & (1 << 7), 0);

        // Send 0x42 poll command -> returns 0x41 with ACK
        bus.write8(0x1F80_1040, 0x42, false);
        assert_eq!(bus.read8(0x1F80_1040), 0x41);
        bus.tick(SIO0_ACK_DELAY_CYCLES);
        assert_ne!(bus.read32(0x1F80_1070) & (1 << 7), 0);

        // 3. Error Reset via SIO_CTRL bit 6 (0x0040) must clear transfer state, I_STAT.7, and preserve control flags
        bus.write16(0x1F80_104A, 0x1003 | (1 << 6), false);
        assert_eq!(bus.read32(0x1F80_1044) & (1 << 9), 0);
        assert_eq!(bus.read32(0x1F80_1070) & (1 << 7), 0);
        assert_eq!(bus.sio_ctrl, 0x1003); // Preserves /DTR, TX_EN, ACKIEN
        assert_eq!(bus.sio_step, 0); // State reset to 0

        // Verify next controller packet starts fresh from step 0
        bus.write8(0x1F80_1040, 0x01, false);
        assert_eq!(bus.read8(0x1F80_1040), 0xFF);
        bus.tick(SIO0_ACK_DELAY_CYCLES);
        assert_ne!(bus.read32(0x1F80_1070) & (1 << 7), 0);
    }

    #[test]
    fn test_spu_voice_synthesis_adsr_gaussian_reverb_and_xa_audio_mixing() {
        let mut bus = Bus::new();

        // 1. Upload two 16-byte ADPCM blocks at SPU RAM 0x1000..0x1020
        // Block 0: Filter 0, Shift 0, Flag 0x04 (Loop Start), nibbles = 0x77 (+28672)
        // Block 1: Filter 0, Shift 0, Flag 0x03 (Loop End + Repeat), nibbles = 0x77 (+28672)
        bus.write16(0x1F80_1DA6, 0x1000 >> 3, false);
        bus.write16(0x1F80_1DA8, 0x0400, false); // shift=0, filter=0, flags=0x04 (Loop Start)
        for _ in 0..7 {
            bus.write16(0x1F80_1DA8, 0x7777, false);
        }
        bus.write16(0x1F80_1DA8, 0x0300, false); // shift=0, filter=0, flags=0x03 (Loop End + Repeat)
        for _ in 0..7 {
            bus.write16(0x1F80_1DA8, 0x7777, false);
        }

        // Configure Main Volume (0x3FFF Left/Right) and SPUCNT (Enable + Unmute + Reverb Write + CD Audio)
        bus.write16(0x1F80_1D80, 0x3FFF, false);
        bus.write16(0x1F80_1D82, 0x3FFF, false);
        bus.write16(0x1F80_1D84, 0x1FFF, false); // Reverb Vol Left
        bus.write16(0x1F80_1D86, 0x1FFF, false); // Reverb Vol Right
        bus.write16(0x1F80_1DA2, 0x4000 >> 3, false); // Reverb work area base = 0x4000
        bus.write16(0x1F80_1DAA, 0xC085, false); // SPU Enable, Unmute, Reverb Master Enable, CD Enable + CD Reverb

        // Configure Voice 0: Vol=0x3FFF, Pitch=0x1000 (44.1 kHz), Start=0x1000>>3, ADSR fast attack (AR=0, DR=0, SL=15, SR=0, RR=0)
        bus.write16(0x1F80_1C00, 0x3FFF, false); // Voice 0 Vol L
        bus.write16(0x1F80_1C02, 0x3FFF, false); // Voice 0 Vol R
        bus.write16(0x1F80_1C04, 0x1000, false); // Voice 0 Pitch (1.0x = 44.1 kHz)
        bus.write16(0x1F80_1C06, 0x1000 >> 3, false); // Voice 0 Start Addr
        bus.write16(0x1F80_1C08, 0x000F, false); // AR=0 (fastest), DR=0, SL=15 (max sustain)
        bus.write16(0x1F80_1C0A, 0x0000, false); // SR=0, RR=0
        bus.write16(0x1F80_1D98, 0x0001, false); // Enable Reverb on Voice 0

        // Key On Voice 0
        bus.write16(0x1F80_1D88, 0x0001, false);
        assert!(bus.spu.voices[0].active);
        assert_eq!(bus.spu.voices[0].adsr_phase, AdsrPhase::Attack);

        // Step 64 SPU samples (64 * 768 CPU cycles) so Voice 0 crosses both blocks and sets ENDX bit 0
        bus.tick(64 * 768);
        let samples = bus.take_audio_samples();
        assert_eq!(samples.len(), 64);
        assert!(samples.iter().any(|&(l, r)| l > 1000 && r > 1000), "Expected audible synthesized PCM output");
        assert_ne!(bus.read16(0x1F80_1D9C) & 1, 0, "Expected ENDX bit 0 set after crossing block 1 loop-end");
        assert_eq!(bus.read16(0x1F80_1C0E), 0x1000 >> 3, "Expected repeat address latched from block 0 loop-start");

        // Key Off Voice 0 and verify Release phase silences the voice
        bus.write16(0x1F80_1D8C, 0x0001, false);
        assert_eq!(bus.spu.voices[0].adsr_phase, AdsrPhase::Release);
        bus.tick(16 * 768);
        assert!(!bus.spu.voices[0].active);
        assert_eq!(bus.read16(0x1F80_1C0C), 0);

        // 2. Verify CD-XA ADPCM Sector Decoding and Upsampling to 44.1 kHz
        let mut xa_sector = vec![0u8; 2352];
        xa_sector[15] = 2; // Mode 2
        xa_sector[18] = 0x64; // Form 2 + Realtime + Audio
        xa_sector[19] = 0x01; // 4-bit Stereo 37.8 kHz
        for sg in 0..18 {
            let sg_base = 24 + sg * 128;
            for su in 0..8 {
                xa_sector[sg_base + 4 + su] = 0x02; // Filter 0, Shift 2
            }
            for d in 16..128 {
                xa_sector[sg_base + d] = 0x44; // Positive nibbles on both L and R
            }
        }
        bus.spu.decode_xa_sector(&xa_sector);
        // 18 sound groups * 4 stereo pairs * 28 words = 2016 samples @ 37.8kHz -> 2016 * 7 / 6 = 2352 stereo samples @ 44.1kHz
        assert_eq!(bus.spu.cd_audio_fifo.len(), 2352);
        assert!(bus.spu.cd_audio_fifo.iter().skip(2).all(|&(l, r)| l == 4096 && r == 4096));
    }

    #[test]
    fn test_dualshock_analog_and_config_mode_sio0_protocol() {
        let mut bus = Bus::new();
        bus.set_pad_state(0xFE, 0x7F); // Select + Square pressed
        bus.set_pad_analog(0x20, 0x40, 0xC0, 0xE0); // LX=0x20, LY=0x40, RX=0xC0, RY=0xE0

        // Helper to run a complete SIO0 controller packet and return the received bytes
        let transact = |bus: &mut Bus, tx: &[u8]| -> Vec<u8> {
            bus.write16(0x1F80_104A, 0x0000, false); // Deassert /DTR
            bus.write16(0x1F80_104A, 0x1003, false); // Assert /DTR + TX_EN + ACKIEN on Port 1
            let mut rx = Vec::with_capacity(tx.len());
            for &b in tx {
                bus.write8(0x1F80_1040, b, false);
                rx.push(bus.read8(0x1F80_1040));
                bus.tick(SIO0_ACK_DELAY_CYCLES);
                bus.write16(0x1F80_104A, 0x1003 | (1 << 4), false);
            }
            rx
        };

        // 1. Default Digital Mode (0x41): 5-byte poll returns [0xFF, 0x41, 0x5A, btn_lo, btn_hi]
        let dig_rx = transact(&mut bus, &[0x01, 0x42, 0x00, 0x00, 0x00]);
        assert_eq!(dig_rx, vec![0xFF, 0x41, 0x5A, 0xFE, 0x7F]);

        // 2. Enter Configuration Mode via 0x43 [0x01, 0x43, 0x00, 0x01, 0x00]
        let enter_cfg = transact(&mut bus, &[0x01, 0x43, 0x00, 0x01, 0x00]);
        assert_eq!(enter_cfg[1], 0x41);
        assert!(bus.pad1.config_mode);

        // 3. Enable Analog Mode & Lock LED via 0x44 [0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00]
        let set_mode = transact(&mut bus, &[0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(&set_mode[1..3], &[0xF3, 0x5A]);
        assert!(bus.pad1.analog_mode);
        assert!(bus.pad1.mode_locked);

        // 4. Query Model Status via 0x45 -> verifies LED state (byte 5 = 0x01)
        let model_rx = transact(&mut bus, &[0x01, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(model_rx, vec![0xFF, 0xF3, 0x5A, 0x01, 0x02, 0x01, 0x02, 0x01, 0x00]);

        // 5. Map Rumble Motors via 0x4D: slot 0 = small motor (0x00), slot 1 = large motor (0x01)
        let _ = transact(&mut bus, &[0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(bus.pad1.motor_map, [0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF]);

        // 6. Exit Configuration Mode via 0x43 [0x01, 0x43, 0x00, 0x00, ...]
        let _ = transact(&mut bus, &[0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert!(!bus.pad1.config_mode);

        // 7. Poll in DualShock Analog Mode (0x73) with Rumble (small=1, large=0xA5):
        // Expect [0xFF, 0x73, 0x5A, btn_lo, btn_hi, RX(0xC0), RY(0xE0), LX(0x20), LY(0x40)]
        let analog_rx = transact(&mut bus, &[0x01, 0x42, 0x00, 0x01, 0xA5, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(analog_rx, vec![0xFF, 0x73, 0x5A, 0xFE, 0x7F, 0xC0, 0xE0, 0x20, 0x40]);
        assert_eq!(bus.pad1.motor_small, 1);
        assert_eq!(bus.pad1.motor_large, 0xA5);
    }
}
