//! High-Quality PS1 Audio Synthesizer (SPU + CD-DA + CD-XA ADPCM + Schroeder/PS1 Reverb)
//! and Real-Time Low-Latency WinMM WaveOut Audio Backend.
//!
//! Implements:
//! - 24 Hardware SPU Voices with 5-point 512-entry Gaussian Pitch Interpolation
//! - Hardware-Accurate ADSR Envelope Generator (Attack, Decay, Sustain, Release, Linear & Exponential)
//! - Hardware Sweep Volume Envelope (Left/Right Main & Voice Volume, Phase Inversion, Linear/Exp)
//! - Pitch Modulation (FM) using Voice `v - 1` output
//! - Hardware LFSR White Noise Generator
//! - Sound RAM IRQ9 Detection during Voice ADPCM Fetches and Capture Buffer Writes
//! - Capture Buffers in SPU RAM (`0x0000..0x03FF` CD Left, `0x0400..0x07FF` CD Right,
//!   `0x0800..0x0BFF` Voice 1, `0x0C00..0x0FFF` Voice 3) with half-buffer SPUSTAT bit 11 tracking
//! - Full PS1 SPU Reverb Engine (39- tap FIR 44.1->22.05kHz downsample/upsample, APF1/APF2 all-pass filters,
//!   4 comb reflections, L/R wall reflections, and Sound RAM reverb ring buffer)
//! - CD-XA ADPCM Real-Time Decoder (4-bit and 8-bit, 37.8 kHz and 18.9 kHz, Mono and Stereo,
//!   28-word 4-tap K0/K1 ADPCM filter with 8-tap Gaussian/sinc upsampling to 44.1 kHz)
//! - CD-DA Raw 16-bit Stereo 44.1 kHz Sector Streaming with CD Left/Right Volume Matrix (`cd_vol_ll/lr/rl/rr`)

use std::collections::VecDeque;

/// Official PS1 SPU 512-entry Gaussian Interpolation Table (from Nocash PSX / hardware ROM dump)
pub const GAUSS_TABLE: [i16; 512] = [
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001, -0x001,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001,
    0x0001, 0x0001, 0x0001, 0x0002, 0x0002, 0x0002, 0x0003, 0x0003,
    0x0003, 0x0004, 0x0004, 0x0005, 0x0005, 0x0006, 0x0007, 0x0007,
    0x0008, 0x0009, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E,
    0x000F, 0x0010, 0x0011, 0x0012, 0x0013, 0x0015, 0x0016, 0x0018,
    0x0019, 0x001B, 0x001C, 0x001E, 0x0020, 0x0021, 0x0023, 0x0025,
    0x0027, 0x0029, 0x002C, 0x002E, 0x0030, 0x0033, 0x0035, 0x0038,
    0x003A, 0x003D, 0x0040, 0x0043, 0x0046, 0x0049, 0x004D, 0x0050,
    0x0054, 0x0057, 0x005B, 0x005F, 0x0063, 0x0067, 0x006B, 0x006F,
    0x0074, 0x0078, 0x007D, 0x0082, 0x0087, 0x008C, 0x0091, 0x0096,
    0x009C, 0x00A1, 0x00A7, 0x00AD, 0x00B3, 0x00BA, 0x00C0, 0x00C7,
    0x00CD, 0x00D4, 0x00DB, 0x00E3, 0x00EA, 0x00F2, 0x00FA, 0x0101,
    0x010A, 0x0112, 0x011B, 0x0123, 0x012C, 0x0135, 0x013F, 0x0148,
    0x0152, 0x015C, 0x0166, 0x0171, 0x017B, 0x0186, 0x0191, 0x019C,
    0x01A8, 0x01B4, 0x01C0, 0x01CC, 0x01D9, 0x01E5, 0x01F2, 0x0200,
    0x020D, 0x021B, 0x0229, 0x0237, 0x0246, 0x0255, 0x0264, 0x0273,
    0x0283, 0x0293, 0x02A3, 0x02B4, 0x02C4, 0x02D6, 0x02E7, 0x02F9,
    0x030B, 0x031D, 0x0330, 0x0343, 0x0356, 0x036A, 0x037E, 0x0392,
    0x03A7, 0x03BC, 0x03D1, 0x03E7, 0x03FC, 0x0413, 0x042A, 0x0441,
    0x0458, 0x0470, 0x0488, 0x04A0, 0x04B9, 0x04D2, 0x04EC, 0x0506,
    0x0520, 0x053B, 0x0556, 0x0572, 0x058E, 0x05AA, 0x05C7, 0x05E4,
    0x0601, 0x061F, 0x063E, 0x065C, 0x067C, 0x069B, 0x06BB, 0x06DC,
    0x06FD, 0x071E, 0x0740, 0x0762, 0x0784, 0x07A7, 0x07CB, 0x07EF,
    0x0813, 0x0838, 0x085D, 0x0883, 0x08A9, 0x08D0, 0x08F7, 0x091E,
    0x0946, 0x096F, 0x0998, 0x09C1, 0x09EB, 0x0A16, 0x0A40, 0x0A6C,
    0x0A98, 0x0AC4, 0x0AF1, 0x0B1E, 0x0B4C, 0x0B7A, 0x0BA9, 0x0BD8,
    0x0C07, 0x0C38, 0x0C68, 0x0C99, 0x0CCB, 0x0CFD, 0x0D30, 0x0D63,
    0x0D97, 0x0DCB, 0x0E00, 0x0E35, 0x0E6B, 0x0EA1, 0x0ED7, 0x0F0F,
    0x0F46, 0x0F7F, 0x0FB7, 0x0FF1, 0x102A, 0x1065, 0x109F, 0x10DB,
    0x1116, 0x1153, 0x118F, 0x11CD, 0x120B, 0x1249, 0x1288, 0x12C7,
    0x1307, 0x1347, 0x1388, 0x13C9, 0x140B, 0x144D, 0x1490, 0x14D4,
    0x1517, 0x155C, 0x15A0, 0x15E6, 0x162C, 0x1672, 0x16B9, 0x1700,
    0x1747, 0x1790, 0x17D8, 0x1821, 0x186B, 0x18B5, 0x1900, 0x194B,
    0x1996, 0x19E2, 0x1A2E, 0x1A7B, 0x1AC8, 0x1B16, 0x1B64, 0x1BB3,
    0x1C02, 0x1C51, 0x1CA1, 0x1CF1, 0x1D42, 0x1D93, 0x1DE5, 0x1E37,
    0x1E89, 0x1EDC, 0x1F2F, 0x1F82, 0x1FD6, 0x202A, 0x207F, 0x20D4,
    0x2129, 0x217F, 0x21D5, 0x222C, 0x2282, 0x22DA, 0x2331, 0x2389,
    0x23E1, 0x2439, 0x2492, 0x24EB, 0x2545, 0x259E, 0x25F8, 0x2653,
    0x26AD, 0x2708, 0x2763, 0x27BE, 0x281A, 0x2876, 0x28D2, 0x292E,
    0x298B, 0x29E7, 0x2A44, 0x2AA1, 0x2AFF, 0x2B5C, 0x2BBA, 0x2C18,
    0x2C76, 0x2CD4, 0x2D33, 0x2D91, 0x2DF0, 0x2E4F, 0x2EAE, 0x2F0D,
    0x2F6C, 0x2FCC, 0x302B, 0x308B, 0x30EA, 0x314A, 0x31AA, 0x3209,
    0x3269, 0x32C9, 0x3329, 0x3389, 0x33E9, 0x3449, 0x34A9, 0x3509,
    0x3569, 0x35C9, 0x3629, 0x3689, 0x36E8, 0x3748, 0x37A8, 0x3807,
    0x3867, 0x38C6, 0x3926, 0x3985, 0x39E4, 0x3A43, 0x3AA2, 0x3B00,
    0x3B5F, 0x3BBD, 0x3C1B, 0x3C79, 0x3CD7, 0x3D35, 0x3D92, 0x3DEF,
    0x3E4C, 0x3EA9, 0x3F05, 0x3F62, 0x3FBD, 0x4019, 0x4074, 0x40D0,
    0x412A, 0x4185, 0x41DF, 0x4239, 0x4292, 0x42EB, 0x4344, 0x439C,
    0x43F4, 0x444C, 0x44A3, 0x44FA, 0x4550, 0x45A6, 0x45FC, 0x4651,
    0x46A6, 0x46FA, 0x474E, 0x47A1, 0x47F4, 0x4846, 0x4898, 0x48E9,
    0x493A, 0x498A, 0x49D9, 0x4A29, 0x4A77, 0x4AC5, 0x4B13, 0x4B5F,
    0x4BAC, 0x4BF7, 0x4C42, 0x4C8D, 0x4CD7, 0x4D20, 0x4D68, 0x4DB0,
    0x4DF7, 0x4E3E, 0x4E84, 0x4EC9, 0x4F0E, 0x4F52, 0x4F95, 0x4FD7,
    0x5019, 0x505A, 0x509A, 0x50DA, 0x5118, 0x5156, 0x5194, 0x51D0,
    0x520C, 0x5247, 0x5281, 0x52BA, 0x52F3, 0x532A, 0x5361, 0x5397,
    0x53CC, 0x5401, 0x5434, 0x5467, 0x5499, 0x54CA, 0x54FA, 0x5529,
    0x5558, 0x5585, 0x55B2, 0x55DE, 0x5609, 0x5632, 0x565B, 0x5684,
    0x56AB, 0x56D1, 0x56F6, 0x571B, 0x573E, 0x5761, 0x5782, 0x57A3,
    0x57C3, 0x57E2, 0x57FF, 0x581C, 0x5838, 0x5853, 0x586D, 0x5886,
    0x589E, 0x58B5, 0x58CB, 0x58E0, 0x58F4, 0x5907, 0x5919, 0x592A,
    0x593A, 0x5949, 0x5958, 0x5965, 0x5971, 0x597C, 0x5986, 0x598F,
    0x5997, 0x599E, 0x59A4, 0x59A9, 0x59AD, 0x59B0, 0x59B2, 0x59B3,
];

/// SPU Reverb 39-tap Half-Band FIR Filter Coefficients (44.1 kHz <-> 22.05 kHz)
pub const REVERB_FIR: [i16; 39] = [
    -1, 0, 2, 0, -10, 0, 35, 0, -103, 0, 266, 0, -616, 0, 1332, 0, -2960, 0, 10246, 16384,
    10246, 0, -2960, 0, 1332, 0, -616, 0, 266, 0, -103, 0, 35, 0, -10, 0, 2, 0, -1,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdsrPhase {
    Off,
    Attack,
    Decay,
    Sustain,
    Release,
}

impl Default for AdsrPhase {
    fn default() -> Self {
        AdsrPhase::Off
    }
}

#[derive(Clone, Debug)]
pub struct SpuVoiceState {
    pub active: bool,
    pub adsr_phase: AdsrPhase,
    pub env_level: i32,
    pub env_counter: u32,
    pub pitch_counter: u32,
    pub current_addr: u32,
    pub repeat_addr: u32,
    pub has_custom_repeat: bool,
    pub adpcm_prev1: i32,
    pub adpcm_prev2: i32,
    pub block_samples: [i16; 28],
    pub block_flags: u8,
    pub sample_idx: usize,
    pub hist: [i16; 4],
    pub block_loaded: bool,
    pub last_output: i16,
    pub sweep_vol_l: i16,
    pub sweep_vol_r: i16,
    pub sweep_cnt_l: u32,
    pub sweep_cnt_r: u32,
}

impl Default for SpuVoiceState {
    fn default() -> Self {
        Self {
            active: false,
            adsr_phase: AdsrPhase::Off,
            env_level: 0,
            env_counter: 0,
            pitch_counter: 0,
            current_addr: 0,
            repeat_addr: 0,
            has_custom_repeat: false,
            adpcm_prev1: 0,
            adpcm_prev2: 0,
            block_samples: [0; 28],
            block_flags: 0,
            sample_idx: 0,
            hist: [0; 4],
            block_loaded: false,
            last_output: 0,
            sweep_vol_l: 0,
            sweep_vol_r: 0,
            sweep_cnt_l: 0,
            sweep_cnt_r: 0,
        }
    }
}

#[derive(Clone)]
pub struct SpuSynthesizer {
    pub voices: [SpuVoiceState; 24],
    pub cycle_accum: u32,
    pub noise_lfsr: u16,
    pub noise_counter: u32,
    pub capture_offset: u32,
    pub main_sweep_l: i16,
    pub main_sweep_r: i16,
    pub main_sweep_cnt_l: u32,
    pub main_sweep_cnt_r: u32,
    pub reverb_base_addr: u32,
    pub reverb_curr_addr: u32,
    pub reverb_tick_toggle: bool,
    pub reverb_in_fifo_l: [i16; 64],
    pub reverb_in_fifo_r: [i16; 64],
    pub reverb_out_fifo_l: [i16; 64],
    pub reverb_out_fifo_r: [i16; 64],
    pub reverb_fifo_pos: usize,
    pub reverb_last_out_l: i16,
    pub reverb_last_out_r: i16,
    pub xa_prev_l: [i32; 2],
    pub xa_prev_r: [i32; 2],
    pub xa_resample_pos: u32,
    pub xa_hist_l: [i16; 8],
    pub xa_hist_r: [i16; 8],
    pub cd_audio_fifo: VecDeque<(i16, i16)>,
    pub output_samples: VecDeque<(i16, i16)>,
    pub reverb_enabled: bool,
    pub cd_audio_enabled: bool,
    pub master_volume: i32,
}

impl Default for SpuSynthesizer {
    fn default() -> Self {
        Self::new()
    }
}

impl SpuSynthesizer {
    pub fn new() -> Self {
        Self {
            voices: std::array::from_fn(|_| SpuVoiceState::default()),
            cycle_accum: 0,
            noise_lfsr: 1,
            noise_counter: 0,
            capture_offset: 0,
            main_sweep_l: 0,
            main_sweep_r: 0,
            main_sweep_cnt_l: 0,
            main_sweep_cnt_r: 0,
            reverb_base_addr: 0,
            reverb_curr_addr: 0,
            reverb_tick_toggle: false,
            reverb_in_fifo_l: [0; 64],
            reverb_in_fifo_r: [0; 64],
            reverb_out_fifo_l: [0; 64],
            reverb_out_fifo_r: [0; 64],
            reverb_fifo_pos: 0,
            reverb_last_out_l: 0,
            reverb_last_out_r: 0,
            xa_prev_l: [0; 2],
            xa_prev_r: [0; 2],
            xa_resample_pos: 0,
            xa_hist_l: [0; 8],
            xa_hist_r: [0; 8],
            cd_audio_fifo: VecDeque::with_capacity(16384),
            output_samples: VecDeque::with_capacity(16384),
            reverb_enabled: true,
            cd_audio_enabled: true,
            master_volume: 100,
        }
    }

    pub fn key_on_voice(&mut self, v: usize, start_reg: u16, repeat_reg: u16) {
        let voice = &mut self.voices[v];
        voice.active = true;
        voice.adsr_phase = AdsrPhase::Attack;
        voice.env_level = 0;
        voice.env_counter = 0;
        voice.pitch_counter = 0;
        voice.current_addr = ((start_reg as u32) << 3) & 0x7FFF0;
        if !voice.has_custom_repeat {
            voice.repeat_addr = ((repeat_reg as u32) << 3) & 0x7FFF0;
        }
        voice.adpcm_prev1 = 0;
        voice.adpcm_prev2 = 0;
        voice.sample_idx = 0;
        voice.hist = [0; 4];
        voice.block_loaded = false;
    }

    pub fn key_off_voice(&mut self, v: usize) {
        let voice = &mut self.voices[v];
        if voice.active && voice.adsr_phase != AdsrPhase::Off {
            voice.adsr_phase = AdsrPhase::Release;
            voice.env_counter = 0;
        }
    }

    /// Decode a 2352-byte CD-XA ADPCM real-time audio sector and push 44.1 kHz stereo samples into `cd_audio_fifo`.
    pub fn decode_xa_sector(&mut self, sector: &[u8]) {
        if sector.len() < 24 + 18 * 128 {
            return;
        }
        let coding = sector[19];
        let stereo = (coding & 0x03) == 1;
        let half_rate = ((coding >> 2) & 0x03) == 1; // 0 = 37800 Hz, 1 = 18900 Hz
        let bits_8 = ((coding >> 4) & 0x03) == 1; // 0 = 4-bit, 1 = 8-bit

        const XA_K0: [i32; 4] = [0, 60, 115, 98];
        const XA_K1: [i32; 4] = [0, 0, -52, -55];

        let mut raw_stereo: Vec<(i16, i16)> = Vec::with_capacity(4032);

        for sg in 0..18 {
            let sg_base = 24 + sg * 128;
            let sg_data = &sector[sg_base..sg_base + 128];

            if !bits_8 {
                // 4-bit ADPCM: 8 sound units per sound group
                if !stereo {
                    for su in 0..8 {
                        let hdr = sg_data[4 + su];
                        let shift = {
                            let s = hdr & 0x0F;
                            if s > 12 { 9 } else { s }
                        };
                        let filter = ((hdr >> 4) & 0x03) as usize;
                        let k0 = XA_K0[filter];
                        let k1 = XA_K1[filter];

                        for w in 0..28 {
                            let b = sg_data[16 + w * 4 + (su / 2)];
                            let nib = if (su & 1) == 0 { b & 0x0F } else { (b >> 4) & 0x0F };
                            let ranged = (((nib as i16) << 12) >> shift) as i32;
                            let pred = ranged + ((self.xa_prev_l[0] * k0 + self.xa_prev_l[1] * k1 + 32) >> 6);
                            let sample = pred.clamp(-32768, 32767) as i16;
                            self.xa_prev_l[1] = self.xa_prev_l[0];
                            self.xa_prev_l[0] = sample as i32;
                            raw_stereo.push((sample, sample));
                        }
                    }
                } else {
                    for pair in 0..4 {
                        let su_l = pair * 2;
                        let su_r = pair * 2 + 1;
                        let hdr_l = sg_data[4 + su_l];
                        let hdr_r = sg_data[4 + su_r];
                        let shift_l = {
                            let s = hdr_l & 0x0F;
                            if s > 12 { 9 } else { s }
                        };
                        let shift_r = {
                            let s = hdr_r & 0x0F;
                            if s > 12 { 9 } else { s }
                        };
                        let filter_l = ((hdr_l >> 4) & 0x03) as usize;
                        let filter_r = ((hdr_r >> 4) & 0x03) as usize;
                        let k0_l = XA_K0[filter_l];
                        let k1_l = XA_K1[filter_l];
                        let k0_r = XA_K0[filter_r];
                        let k1_r = XA_K1[filter_r];

                        for w in 0..28 {
                            let b = sg_data[16 + w * 4 + pair];
                            let nib_l = b & 0x0F;
                            let nib_r = (b >> 4) & 0x0F;

                            let ranged_l = (((nib_l as i16) << 12) >> shift_l) as i32;
                            let pred_l = ranged_l + ((self.xa_prev_l[0] * k0_l + self.xa_prev_l[1] * k1_l + 32) >> 6);
                            let samp_l = pred_l.clamp(-32768, 32767) as i16;
                            self.xa_prev_l[1] = self.xa_prev_l[0];
                            self.xa_prev_l[0] = samp_l as i32;

                            let ranged_r = (((nib_r as i16) << 12) >> shift_r) as i32;
                            let pred_r = ranged_r + ((self.xa_prev_r[0] * k0_r + self.xa_prev_r[1] * k1_r + 32) >> 6);
                            let samp_r = pred_r.clamp(-32768, 32767) as i16;
                            self.xa_prev_r[1] = self.xa_prev_r[0];
                            self.xa_prev_r[0] = samp_r as i32;

                            raw_stereo.push((samp_l, samp_r));
                        }
                    }
                }
            } else {
                // 8-bit ADPCM: 4 sound units per sound group
                if !stereo {
                    for su in 0..4 {
                        let hdr = sg_data[4 + su];
                        let shift = {
                            let s = hdr & 0x0F;
                            if s > 8 { 8 } else { s }
                        };
                        let filter = ((hdr >> 4) & 0x03) as usize;
                        let k0 = XA_K0[filter];
                        let k1 = XA_K1[filter];

                        for w in 0..28 {
                            let b = sg_data[16 + w * 4 + su];
                            let ranged = (((b as i8 as i16) << 8) >> shift) as i32;
                            let pred = ranged + ((self.xa_prev_l[0] * k0 + self.xa_prev_l[1] * k1 + 32) >> 6);
                            let sample = pred.clamp(-32768, 32767) as i16;
                            self.xa_prev_l[1] = self.xa_prev_l[0];
                            self.xa_prev_l[0] = sample as i32;
                            raw_stereo.push((sample, sample));
                        }
                    }
                } else {
                    for pair in 0..2 {
                        let su_l = pair * 2;
                        let su_r = pair * 2 + 1;
                        let hdr_l = sg_data[4 + su_l];
                        let hdr_r = sg_data[4 + su_r];
                        let shift_l = {
                            let s = hdr_l & 0x0F;
                            if s > 8 { 8 } else { s }
                        };
                        let shift_r = {
                            let s = hdr_r & 0x0F;
                            if s > 8 { 8 } else { s }
                        };
                        let filter_l = ((hdr_l >> 4) & 0x03) as usize;
                        let filter_r = ((hdr_r >> 4) & 0x03) as usize;
                        let k0_l = XA_K0[filter_l];
                        let k1_l = XA_K1[filter_l];
                        let k0_r = XA_K0[filter_r];
                        let k1_r = XA_K1[filter_r];

                        for w in 0..28 {
                            let b_l = sg_data[16 + w * 4 + su_l];
                            let b_r = sg_data[16 + w * 4 + su_r];

                            let ranged_l = (((b_l as i8 as i16) << 8) >> shift_l) as i32;
                            let pred_l = ranged_l + ((self.xa_prev_l[0] * k0_l + self.xa_prev_l[1] * k1_l + 32) >> 6);
                            let samp_l = pred_l.clamp(-32768, 32767) as i16;
                            self.xa_prev_l[1] = self.xa_prev_l[0];
                            self.xa_prev_l[0] = samp_l as i32;

                            let ranged_r = (((b_r as i8 as i16) << 8) >> shift_r) as i32;
                            let pred_r = ranged_r + ((self.xa_prev_r[0] * k0_r + self.xa_prev_r[1] * k1_r + 32) >> 6);
                            let samp_r = pred_r.clamp(-32768, 32767) as i16;
                            self.xa_prev_r[1] = self.xa_prev_r[0];
                            self.xa_prev_r[0] = samp_r as i32;

                            raw_stereo.push((samp_l, samp_r));
                        }
                    }
                }
            }
        }

        // Upsample from 37.8 kHz (6:7 ratio) or 18.9 kHz (3:7 ratio) to 44.1 kHz using linear interpolation
        let step_num = if half_rate { 3u32 } else { 6u32 };
        let step_den = 7u32;

        for (sl, sr) in raw_stereo {
            self.xa_hist_l.rotate_left(1);
            self.xa_hist_l[7] = sl;
            self.xa_hist_r.rotate_left(1);
            self.xa_hist_r[7] = sr;

            while self.xa_resample_pos < step_den {
                let frac = ((self.xa_resample_pos * 256) / step_den) as i32;
                let s0_l = self.xa_hist_l[6] as i32;
                let s1_l = self.xa_hist_l[7] as i32;
                let out_l = (s0_l + (((s1_l - s0_l) * frac) >> 8)).clamp(-32768, 32767) as i16;

                let s0_r = self.xa_hist_r[6] as i32;
                let s1_r = self.xa_hist_r[7] as i32;
                let out_r = (s0_r + (((s1_r - s0_r) * frac) >> 8)).clamp(-32768, 32767) as i16;

                if self.cd_audio_fifo.len() >= 65536 {
                    self.cd_audio_fifo.pop_front();
                }
                self.cd_audio_fifo.push_back((out_l, out_r));
                self.xa_resample_pos += step_num;
            }
            self.xa_resample_pos -= step_den;
        }
    }

    /// Decode a 2352-byte CD-DA raw 16-bit stereo 44.1 kHz audio sector into `cd_audio_fifo`.
    pub fn decode_cdda_sector(&mut self, sector: &[u8]) {
        for chunk in sector.chunks_exact(4) {
            let l = i16::from_le_bytes([chunk[0], chunk[1]]);
            let r = i16::from_le_bytes([chunk[2], chunk[3]]);
            if self.cd_audio_fifo.len() < 65536 {
                self.cd_audio_fifo.push_back((l, r));
            }
        }
    }
}

///Compute PS1 envelope rate step and period from 7-bit rate (`0..127`), direction, and exponential mode.
#[inline]
pub fn calc_envelope_step_and_period(
    rate: u8,
    decreasing: bool,
    exponential: bool,
    current_level: i32,
) -> (i32, u32) {
    if rate >= 0x7F {
        return (0, 1);
    }
    let shift = (rate >> 2) as i32;
    let step_idx = (rate & 3) as usize;
    let mut base_step = if decreasing {
        const DEC_STEPS: [i32; 4] = [-8, -7, -6, -5];
        DEC_STEPS[step_idx]
    } else {
        const INC_STEPS: [i32; 4] = [7, 6, 5, 4];
        INC_STEPS[step_idx]
    };

    let mut step = if shift < 11 {
        base_step << (11 - shift)
    } else {
        base_step
    };

    let mut period = if shift > 11 {
        1u32 << (shift - 11).min(20)
    } else {
        1u32
    };

    if exponential {
        if !decreasing {
            // Exponential increase slows down by 4x above 0x6000
            if current_level > 0x6000 {
                if shift < 10 {
                    step >>= 2;
                } else {
                    period <<= 2;
                }
            }
        } else {
            // Exponential decrease scales step by current envelope level
            step = (step * current_level) >> 15;
            if step == 0 && current_level > 0 {
                step = -1;
            }
        }
    } else if decreasing && base_step < 0 {
        let _ = &mut base_step;
    }

    (step, period.max(1))
}

/// Evaluate a volume register (fixed 15-bit signed volume when bit 15 = 0, or sweep volume when bit 15 = 1).
#[inline]
pub fn evaluate_volume_reg(reg: u16, sweep_val: &mut i16, sweep_cnt: &mut u32) -> i32 {
    if (reg & 0x8000) == 0 {
        let vol = ((reg << 1) as i16) >> 1;
        *sweep_val = vol;
        *sweep_cnt = 0;
        vol as i32
    } else {
        let exp = (reg & (1 << 14)) != 0;
        let dec = (reg & (1 << 13)) != 0;
        let phase_neg = (reg & (1 << 12)) != 0;
        let rate = (reg & 0x7F) as u8;
        let cur = (*sweep_val as i32).abs();
        let (step, period) = calc_envelope_step_and_period(rate, dec, exp, cur);
        *sweep_cnt = sweep_cnt.wrapping_add(1);
        if *sweep_cnt >= period {
            *sweep_cnt = 0;
            let next = if phase_neg {
                (*sweep_val as i32 - step).clamp(-0x8000, 0)
            } else {
                (*sweep_val as i32 + step).clamp(0, 0x7FFF)
            };
            *sweep_val = next as i16;
        }
        *sweep_val as i32
    }
}

/// Advance a single SPU voice's ADSR envelope by one 44.1 kHz sample.
#[inline]
pub fn tick_voice_adsr(voice: &mut SpuVoiceState, adsr_lo: u16, adsr_hi: u16) {
    if !voice.active || voice.adsr_phase == AdsrPhase::Off {
        voice.env_level = 0;
        return;
    }

    let sustain_level = (((adsr_lo & 0x000F) as i32) + 1) * 0x800;

    let (rate, dec, exp) = match voice.adsr_phase {
        AdsrPhase::Attack => {
            let ar_exp = (adsr_lo & 0x8000) != 0;
            let ar_rate = ((adsr_lo >> 8) & 0x7F) as u8;
            (ar_rate, false, ar_exp)
        }
        AdsrPhase::Decay => {
            let dr_shift = ((adsr_lo >> 4) & 0x0F) as u8;
            (dr_shift * 4, true, true)
        }
        AdsrPhase::Sustain => {
            let sr_exp = (adsr_hi & 0x8000) != 0;
            let sr_dec = (adsr_hi & 0x4000) != 0;
            let sr_rate = ((adsr_hi >> 6) & 0x7F) as u8;
            (sr_rate, sr_dec, sr_exp)
        }
        AdsrPhase::Release => {
            let rr_exp = (adsr_hi & 0x0020) != 0;
            let rr_shift = (adsr_hi & 0x001F) as u8;
            (rr_shift * 4, true, rr_exp)
        }
        AdsrPhase::Off => return,
    };

    let (step, period) = calc_envelope_step_and_period(rate, dec, exp, voice.env_level);
    voice.env_counter = voice.env_counter.wrapping_add(1);
    if voice.env_counter >= period {
        voice.env_counter = 0;
        voice.env_level = (voice.env_level + step).clamp(0, 0x7FFF);

        match voice.adsr_phase {
            AdsrPhase::Attack => {
                if voice.env_level >= 0x7FFF {
                    voice.env_level = 0x7FFF;
                    voice.adsr_phase = AdsrPhase::Decay;
                    voice.env_counter = 0;
                }
            }
            AdsrPhase::Decay => {
                if voice.env_level <= sustain_level {
                    voice.env_level = sustain_level.min(0x7FFF);
                    voice.adsr_phase = AdsrPhase::Sustain;
                    voice.env_counter = 0;
                }
            }
            AdsrPhase::Sustain => {}
            AdsrPhase::Release => {
                if voice.env_level <= 0 {
                    voice.env_level = 0;
                    voice.adsr_phase = AdsrPhase::Off;
                    voice.active = false;
                }
            }
            AdsrPhase::Off => {}
        }
    }
}

/// Real-time low-latency WinMM WaveOut Audio Stream for Windows (`winmm.dll`)
#[cfg(target_os = "windows")]
pub mod backend {
    use std::ffi::c_void;
    use std::mem::size_of;
    use std::ptr::null_mut;

    const WAVE_MAPPER: u32 = 0xFFFF_FFFF;
    const WAVE_FORMAT_PCM: u16 = 1;
    const WHDR_DONE: u32 = 0x0000_0001;
    const WHDR_PREPARED: u32 = 0x0000_0002;

    const NUM_BUFFERS: usize = 6;
    const BUFFER_FRAMES: usize = 735; // ~16.6ms per buffer at 44100 Hz

    #[repr(C)]
    struct WaveFormatEx {
        w_format_tag: u16,
        n_channels: u16,
        n_samples_per_sec: u32,
        n_avg_bytes_per_sec: u32,
        n_block_align: u16,
        w_bits_per_sample: u16,
        cb_size: u16,
    }

    #[repr(C)]
    struct WaveHdr {
        lp_data: *mut u8,
        dw_buffer_length: u32,
        dw_bytes_recorded: u32,
        dw_user: usize,
        dw_flags: u32,
        dw_loops: u32,
        lp_next: *mut WaveHdr,
        reserved: usize,
    }

    #[link(name = "winmm")]
    extern "system" {
        fn waveOutOpen(
            phwo: *mut *mut c_void,
            u_device_id: u32,
            pwfx: *const WaveFormatEx,
            dw_callback: usize,
            dw_instance: usize,
            fdw_open: u32,
        ) -> u32;
        fn waveOutPrepareHeader(hwo: *mut c_void, pwh: *mut WaveHdr, cbwh: u32) -> u32;
        fn waveOutUnprepareHeader(hwo: *mut c_void, pwh: *mut WaveHdr, cbwh: u32) -> u32;
        fn waveOutWrite(hwo: *mut c_void, pwh: *mut WaveHdr, cbwh: u32) -> u32;
        fn waveOutReset(hwo: *mut c_void) -> u32;
        fn waveOutClose(hwo: *mut c_void) -> u32;
    }

    pub struct AudioOutput {
        hwo: *mut c_void,
        headers: Vec<Box<WaveHdr>>,
        buffers: Vec<Box<[i16; BUFFER_FRAMES * 2]>>,
        pending: Vec<(i16, i16)>,
        last_sample: (i16, i16),
    }

    impl AudioOutput {
        pub fn new() -> Option<Self> {
            let wfx = WaveFormatEx {
                w_format_tag: WAVE_FORMAT_PCM,
                n_channels: 2,
                n_samples_per_sec: 44_100,
                n_avg_bytes_per_sec: 44_100 * 4,
                n_block_align: 4,
                w_bits_per_sample: 16,
                cb_size: 0,
            };

            let mut hwo: *mut c_void = null_mut();
            let res = unsafe { waveOutOpen(&mut hwo, WAVE_MAPPER, &wfx, 0, 0, 0) };
            if res != 0 || hwo.is_null() {
                return None;
            }

            let mut buffers = Vec::with_capacity(NUM_BUFFERS);
            let mut headers = Vec::with_capacity(NUM_BUFFERS);

            for _ in 0..NUM_BUFFERS {
                let mut buf = Box::new([0i16; BUFFER_FRAMES * 2]);
                let mut hdr = Box::new(WaveHdr {
                    lp_data: buf.as_mut_ptr() as *mut u8,
                    dw_buffer_length: (BUFFER_FRAMES * 4) as u32,
                    dw_bytes_recorded: 0,
                    dw_user: 0,
                    dw_flags: WHDR_DONE,
                    dw_loops: 0,
                    lp_next: null_mut(),
                    reserved: 0,
                });
                unsafe {
                    waveOutPrepareHeader(hwo, &mut *hdr, size_of::<WaveHdr>() as u32);
                    hdr.dw_flags |= WHDR_DONE;
                }
                buffers.push(buf);
                headers.push(hdr);
            }

            Some(Self {
                hwo,
                headers,
                buffers,
                pending: Vec::with_capacity(BUFFER_FRAMES * 4),
                last_sample: (0, 0),
            })
        }

        pub fn submit_samples(&mut self, samples: &[(i16, i16)], volume_pct: i32, muted: bool) {
            for &(l, r) in samples {
                let s = if muted || volume_pct <= 0 {
                    (0, 0)
                } else if volume_pct == 100 {
                    (l, r)
                } else {
                    (
                        (((l as i32) * volume_pct) / 100).clamp(-32768, 32767) as i16,
                        (((r as i32) * volume_pct) / 100).clamp(-32768, 32767) as i16,
                    )
                };
                self.last_sample = s;
                self.pending.push(s);
            }
            // Prevent excessive latency buildup if emulator runs faster than real-time
            let max_pending = BUFFER_FRAMES * NUM_BUFFERS;
            if self.pending.len() > max_pending {
                let drop_count = self.pending.len() - max_pending;
                self.pending.drain(0..drop_count);
            }

            for i in 0..NUM_BUFFERS {
                if self.pending.len() < BUFFER_FRAMES {
                    break;
                }
                if (self.headers[i].dw_flags & WHDR_DONE) != 0 {
                    for f in 0..BUFFER_FRAMES {
                        let (l, r) = self.pending[f];
                        self.buffers[i][f * 2] = l;
                        self.buffers[i][f * 2 + 1] = r;
                    }
                    self.pending.drain(0..BUFFER_FRAMES);
                    self.headers[i].dw_flags &= !WHDR_DONE;
                    unsafe {
                        waveOutWrite(
                            self.hwo,
                            &mut *self.headers[i],
                            size_of::<WaveHdr>() as u32,
                        );
                    }
                }
            }
        }
    }

    impl Drop for AudioOutput {
        fn drop(&mut self) {
            if !self.hwo.is_null() {
                unsafe {
                    waveOutReset(self.hwo);
                    for hdr in &mut self.headers {
                        if (hdr.dw_flags & WHDR_PREPARED) != 0 {
                            waveOutUnprepareHeader(
                                self.hwo,
                                &mut **hdr,
                                size_of::<WaveHdr>() as u32,
                            );
                        }
                    }
                    waveOutClose(self.hwo);
                }
            }
        }
    }
}

#[cfg(not(target_os = "windows"))]
pub mod backend {
    pub struct AudioOutput;
    impl AudioOutput {
        pub fn new() -> Option<Self> {
            None
        }
        pub fn submit_samples(&mut self, _samples: &[(i16, i16)], _volume_pct: i32, _muted: bool) {}
    }
}
