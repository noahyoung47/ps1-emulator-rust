//! MIPS R3000A CPU Core & COP2 Geometry Transform Engine (T-03, T-04, T-06, T-09, T-10 Complete)
//! Implements the full MIPS R3000A instruction set, Load Delay Slots (with cancellation and
//! LWL/LWR in-flight pending load merging), Branch Delay Slots, hardware-accurate REGIMM (0x01)
//! bit-decoding, non-trapping DIV/DIVU edge cases, signed overflow traps (ADD/ADDI/SUB),
//! alignment exceptions (AdEL/AdES on LH/LHU/LW/SH/SW/JR/JALR/PC fetch), Bus Error exceptions
//! (IBE/DBE on KUSEG > 512MB), COP0 System Control (MFC0/MTC0/RFE/Exception stack/Interrupts),
//! full COP2 / GTE (64 Data/Control registers, UNR reciprocal division, and all 22 GTE opcodes),
//! peripheral/timer cycle stepping via Bus::tick, BIOS 0xA0/0xB0 TTY console interception,
//! and PS-X EXE Bare-Metal & BIOS Shell (0x8003_0000) Side-Loading.

pub const EX_INT: u32 = 0x00;  // Interrupt
pub const EX_ADEL: u32 = 0x04; // Address Error on Load or Instruction Fetch
pub const EX_ADES: u32 = 0x05; // Address Error on Store
pub const EX_IBE: u32 = 0x06;  // Bus Error on Instruction Fetch
pub const EX_DBE: u32 = 0x07;  // Bus Error on Data Load/Store
pub const EX_SYS: u32 = 0x08;  // Syscall
pub const EX_BP: u32 = 0x09;   // Breakpoint
pub const EX_RI: u32 = 0x0A;   // Reserved Instruction
pub const EX_CPU: u32 = 0x0B;  // Coprocessor Unusable
pub const EX_OVF: u32 = 0x0C;  // Arithmetic Overflow

pub const UNR_TABLE: [u8; 257] = [
    0xFF, 0xFD, 0xFB, 0xF9, 0xF7, 0xF5, 0xF3, 0xF1, 0xEF, 0xEE, 0xEC, 0xEA, 0xE8, 0xE6, 0xE4, 0xE3,
    0xE1, 0xDF, 0xDD, 0xDC, 0xDA, 0xD8, 0xD6, 0xD5, 0xD3, 0xD1, 0xD0, 0xCE, 0xCD, 0xCB, 0xC9, 0xC8,
    0xC6, 0xC5, 0xC3, 0xC1, 0xC0, 0xBE, 0xBD, 0xBB, 0xBA, 0xB8, 0xB7, 0xB5, 0xB4, 0xB2, 0xB1, 0xB0,
    0xAE, 0xAD, 0xAB, 0xAA, 0xA9, 0xA7, 0xA6, 0xA4, 0xA3, 0xA2, 0xA0, 0x9F, 0x9E, 0x9C, 0x9B, 0x9A,
    0x99, 0x97, 0x96, 0x95, 0x94, 0x92, 0x91, 0x90, 0x8F, 0x8D, 0x8C, 0x8B, 0x8A, 0x89, 0x87, 0x86,
    0x85, 0x84, 0x83, 0x82, 0x81, 0x7F, 0x7E, 0x7D, 0x7C, 0x7B, 0x7A, 0x79, 0x78, 0x77, 0x75, 0x74,
    0x73, 0x72, 0x71, 0x70, 0x6F, 0x6E, 0x6D, 0x6C, 0x6B, 0x6A, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64,
    0x63, 0x62, 0x61, 0x60, 0x5F, 0x5E, 0x5D, 0x5D, 0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56, 0x55,
    0x54, 0x53, 0x53, 0x52, 0x51, 0x50, 0x4F, 0x4E, 0x4D, 0x4D, 0x4C, 0x4B, 0x4A, 0x49, 0x48, 0x48,
    0x47, 0x46, 0x45, 0x44, 0x43, 0x43, 0x42, 0x41, 0x40, 0x3F, 0x3F, 0x3E, 0x3D, 0x3C, 0x3C, 0x3B,
    0x3A, 0x39, 0x39, 0x38, 0x37, 0x36, 0x36, 0x35, 0x34, 0x33, 0x33, 0x32, 0x31, 0x31, 0x30, 0x2F,
    0x2E, 0x2E, 0x2D, 0x2C, 0x2C, 0x2B, 0x2A, 0x2A, 0x29, 0x28, 0x28, 0x27, 0x26, 0x26, 0x25, 0x24,
    0x24, 0x23, 0x22, 0x22, 0x21, 0x20, 0x20, 0x1F, 0x1E, 0x1E, 0x1D, 0x1D, 0x1C, 0x1B, 0x1B, 0x1A,
    0x19, 0x19, 0x18, 0x18, 0x17, 0x16, 0x16, 0x15, 0x15, 0x14, 0x14, 0x13, 0x12, 0x12, 0x11, 0x11,
    0x10, 0x0F, 0x0F, 0x0E, 0x0E, 0x0D, 0x0D, 0x0C, 0x0C, 0x0B, 0x0A, 0x0A, 0x09, 0x09, 0x08, 0x08,
    0x07, 0x07, 0x06, 0x06, 0x05, 0x05, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00,
    0x00,
];

use crate::bus::{Bus, PsExe};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DelayedLoad {
    pub reg: usize,
    pub val: u32,
}

#[derive(Clone)]
pub struct Cpu {
    pub gpr: [u32; 32],
    pub out_gpr: [u32; 32],
    pub pc: u32,
    pub next_pc: u32,
    pub current_pc: u32,
    pub hi: u32,
    pub lo: u32,
    pub cop0: [u32; 32],
    pub cop2_data: [u32; 32],
    pub cop2_ctrl: [u32; 32],
    pub in_branch_delay: bool,
    pub next_is_branch_delay: bool,
    pub branch_taken: bool,
    pub next_branch_taken: bool,
    pub pending_load: Option<DelayedLoad>,
    pub next_load: Option<DelayedLoad>,
    pub cycles: u64,
    pub trace_enabled: bool,
    pub widescreen_hack: bool,
    pub pgxp_culling_fix: bool,
}

impl Cpu {
    pub fn new() -> Self {
        let mut cpu = Self {
            gpr: [0; 32],
            out_gpr: [0; 32],
            pc: 0xBFC0_0000, // BIOS Reset Vector
            next_pc: 0xBFC0_0004,
            current_pc: 0xBFC0_0000,
            hi: 0,
            lo: 0,
            cop0: [0; 32],
            cop2_data: [0; 32],
            cop2_ctrl: [0; 32],
            in_branch_delay: false,
            next_is_branch_delay: false,
            branch_taken: false,
            next_branch_taken: false,
            pending_load: None,
            next_load: None,
            cycles: 0,
            trace_enabled: false,
            widescreen_hack: false,
            pgxp_culling_fix: false,
        };
        cpu.cop0[12] = 0x0040_0000; // SR.BEV = 1 on reset
        cpu.cop0[15] = 0x0000_0002; // PRId = 0x00000002
        cpu
    }

    #[inline]
    pub fn cache_isolated(&self) -> bool {
        (self.cop0[12] & 0x0001_0000) != 0
    }

    #[inline]
    pub fn in_user_mode(&self) -> bool {
        (self.cop0[12] & 0x02) != 0
    }

    #[inline(always)]
    pub fn set_reg(&mut self, idx: usize, val: u32) {
        if idx != 0 {
            self.gpr[idx] = val;
            if let Some(load) = self.pending_load {
                if load.reg == idx {
                    self.pending_load = None;
                }
            }
        }
    }

    #[inline]
    pub fn queue_load(&mut self, reg: usize, val: u32) {
        if let Some(prev) = self.pending_load {
            if prev.reg == reg {
                self.pending_load = None;
            }
        }
        if reg != 0 {
            self.next_load = Some(DelayedLoad { reg, val });
        }
    }

    #[inline]
    pub fn read_reg_with_load_forwarding(&self, reg: usize) -> u32 {
        if reg == 0 {
            return 0;
        }
        if let Some(load) = self.pending_load {
            if load.reg == reg {
                return load.val;
            }
        }
        self.gpr[reg]
    }

    fn check_bios_tty_intercept(&self, bus: &mut Bus) {
        let masked_pc = self.pc & 0x1FFF_FFFF;
        let call_num = self.gpr[9];
        if (masked_pc == 0x0000_00A0 && call_num == 0x3C)
            || (masked_pc == 0x0000_00B0 && call_num == 0x3D)
        {
            let ch = (self.gpr[4] & 0xFF) as u8 as char;
            print!("{}", ch);
            bus.tty_buffer.push(ch);
        }
    }

    pub fn sideload_exe(&mut self, bus: &mut Bus, exe_bytes: &[u8]) -> Result<(), String> {
        let exe = PsExe::from_bytes(exe_bytes)?;
        bus.load_exe_payload(&exe);
        self.pc = exe.initial_pc;
        self.next_pc = exe.initial_pc.wrapping_add(4);
        self.current_pc = exe.initial_pc;
        self.in_branch_delay = false;
        self.next_is_branch_delay = false;
        self.branch_taken = false;
        self.next_branch_taken = false;
        self.pending_load = None;
        self.next_load = None;
        self.gpr[28] = exe.initial_gp;
        let sp = if exe.initial_sp != 0 {
            exe.initial_sp
        } else {
            0x801F_FFF0
        };
        self.gpr[29] = sp;
        self.gpr[30] = sp;
        self.out_gpr = self.gpr;
        Ok(())
    }

    #[inline]
    pub fn check_interrupts(&mut self, bus: &Bus) -> bool {
        if bus.irq_pending() {
            self.cop0[13] |= 1 << 10;
        } else {
            self.cop0[13] &= !(1 << 10);
        }
        let sr = self.cop0[12];
        let cause = self.cop0[13];
        (sr & 1) != 0 && ((sr & cause) & 0x0000_FF00) != 0
    }

    #[inline]
    pub fn check_irqs(&mut self, bus: &Bus) -> bool {
        self.check_interrupts(bus)
    }

    pub fn enter_exception(&mut self, ex_code: u32, bad_vaddr: Option<u32>, cop_num: u32) {
        if let Some(vaddr) = bad_vaddr {
            self.cop0[8] = vaddr; // BadVAddr (R8)
        }
        let sr = self.cop0[12];
        self.cop0[12] = (sr & !0x3F) | ((sr << 2) & 0x3C);

        let mut cause = self.cop0[13] & 0x0000_FF00;
        cause |= (ex_code & 0x1F) << 2;
        cause |= (cop_num & 0x03) << 28;
        if self.in_branch_delay {
            cause |= 1 << 31; // BD bit 31
            self.cop0[14] = self.current_pc.wrapping_sub(4);
        } else {
            self.cop0[14] = self.current_pc;
        }
        self.cop0[13] = cause;

        let vector = if (self.cop0[12] & (1 << 22)) != 0 {
            0xBFC0_0180
        } else {
            0x8000_0080
        };
        self.pc = vector;
        self.next_pc = vector.wrapping_add(4);
        self.next_is_branch_delay = false;
        self.next_branch_taken = false;
    }

    #[inline]
    pub fn step(&mut self, bus: &mut Bus) {
        self.step_no_tick(bus);
        self.cycles += 1;
        bus.tick(1);
    }

    /// Execute up to `max_steps` instructions while batching peripheral/timer/SPU ticks
    /// into larger chunks (up to 32 cycles) when no immediate hardware interrupt is pending.
    pub fn step_batch(&mut self, bus: &mut Bus, max_steps: u32) {
        let mut remaining = max_steps;
        while remaining > 0 {
            if bus.irq_pending()
                || bus.sio_ack_delay > 0
                || bus.sio_ack_pulse_cycles > 0
                || self.pc == 0x8003_0000
                || self.trace_enabled
            {
                self.step_no_tick(bus);
                self.cycles += 1;
                bus.tick(1);
                remaining -= 1;
                continue;
            }

            let burst = remaining.min(32);
            let mut executed = 0u32;
            while executed < burst {
                self.step_no_tick(bus);
                self.cycles += 1;
                executed += 1;
                if bus.irq_pending()
                    || bus.sio_ack_delay > 0
                    || self.pc == 0x8003_0000
                {
                    break;
                }
            }
            bus.tick(executed);
            remaining -= executed;
        }
    }

    #[inline(always)]
    fn step_no_tick(&mut self, bus: &mut Bus) {
        if self.pc == 0x8003_0000 {
            if let Some(exe) = bus.pending_exe.take() {
                if let Some(load) = self.pending_load.take() {
                    if load.reg != 0 {
                        self.gpr[load.reg] = load.val;
                    }
                }
                bus.load_exe_payload(&exe);
                self.pc = exe.initial_pc;
                self.next_pc = exe.initial_pc.wrapping_add(4);
                self.in_branch_delay = false;
                self.next_is_branch_delay = false;
                self.branch_taken = false;
                self.next_branch_taken = false;
                self.gpr[28] = exe.initial_gp;
                if exe.initial_sp != 0 {
                    self.gpr[29] = exe.initial_sp;
                    self.gpr[30] = exe.initial_sp;
                } else if self.gpr[29] == 0 {
                    self.gpr[29] = 0x801F_FFF0;
                    self.gpr[30] = 0x801F_FFF0;
                }
                self.out_gpr = self.gpr;
            }
        }
        self.current_pc = self.pc;
        self.in_branch_delay = self.next_is_branch_delay;
        self.branch_taken = self.next_branch_taken;

        self.next_is_branch_delay = false;
        self.next_branch_taken = false;
        self.next_load = None;

        if self.check_interrupts(bus) && !self.in_branch_delay {
            if let Some(load) = self.pending_load.take() {
                if load.reg != 0 {
                    self.gpr[load.reg] = load.val;
                }
            }
            self.enter_exception(EX_INT, None, 0);
            self.gpr[0] = 0;
            return;
        }

        if (self.pc & 3) != 0 || (self.in_user_mode() && self.pc >= 0x8000_0000) {
            if let Some(load) = self.pending_load.take() {
                if load.reg != 0 {
                    self.gpr[load.reg] = load.val;
                }
            }
            self.enter_exception(EX_ADEL, Some(self.pc), 0);
            self.gpr[0] = 0;
            return;
        }

        if (self.pc & 0x1FFF_FFEF) == 0x0000_00A0 {
            self.check_bios_tty_intercept(bus);
        }

        let instr = bus.fetch_instr32(self.pc);
        if self.trace_enabled {
            eprintln!("{:08X}: {:08X} GPR={:08X?}", self.pc, instr, self.gpr);
        }
        if bus.bus_error && bus.take_bus_error() {
            if let Some(load) = self.pending_load.take() {
                if load.reg != 0 {
                    self.gpr[load.reg] = load.val;
                }
            }
            self.enter_exception(EX_IBE, None, 0);
            self.gpr[0] = 0;
            return;
        }

        self.pc = self.next_pc;
        self.next_pc = self.next_pc.wrapping_add(4);

        // Save pending load before execute() (which may cancel it via set_reg/queue_load)
        self.execute(instr, bus);

        if let Some(load) = self.pending_load.take() {
            if load.reg != 0 {
                self.gpr[load.reg] = load.val;
            }
        }
        self.pending_load = self.next_load.take();

        self.gpr[0] = 0;
    }

    fn branch(&mut self, cond: bool, offset_imm16: u32) {
        self.next_is_branch_delay = true;
        if cond {
            let offset = ((offset_imm16 as i16) as i32 as u32) << 2;
            self.next_pc = self.pc.wrapping_add(offset);
            self.next_branch_taken = true;
        }
    }

    #[inline]
    fn execute(&mut self, instr: u32, bus: &mut Bus) {
        let op = instr >> 26;
        let rs = ((instr >> 21) & 0x1F) as usize;
        let rt = ((instr >> 16) & 0x1F) as usize;
        let rd = ((instr >> 11) & 0x1F) as usize;
        let shamt = (instr >> 6) & 0x1F;
        let funct = instr & 0x3F;
        let imm16 = instr & 0xFFFF;
        let sign_imm = (imm16 as i16) as i32 as u32;
        let target = instr & 0x03FF_FFFF;

        match op {
            // 0x00: SPECIAL
            0x00 => match funct {
                0x00 => self.set_reg(rd, self.gpr[rt] << shamt), // SLL / NOP
                0x02 => self.set_reg(rd, self.gpr[rt] >> shamt), // SRL
                0x03 => self.set_reg(rd, ((self.gpr[rt] as i32) >> shamt) as u32), // SRA
                0x04 => self.set_reg(rd, self.gpr[rt] << (self.gpr[rs] & 0x1F)), // SLLV
                0x06 => self.set_reg(rd, self.gpr[rt] >> (self.gpr[rs] & 0x1F)), // SRLV
                0x07 => self.set_reg(rd, ((self.gpr[rt] as i32) >> (self.gpr[rs] & 0x1F)) as u32), // SRAV
                0x08 => {
                    // JR
                    self.next_pc = self.gpr[rs];
                    self.next_is_branch_delay = true;
                    self.next_branch_taken = true;
                }
                0x09 => {
                    // JALR
                    let target_pc = self.gpr[rs];
                    let ret = self.next_pc;
                    self.set_reg(rd, ret);
                    self.next_pc = target_pc;
                    self.next_is_branch_delay = true;
                    self.next_branch_taken = true;
                }
                0x0C => self.enter_exception(EX_SYS, None, 0), // SYSCALL
                0x0D => self.enter_exception(EX_BP, None, 0),  // BREAK
                0x10 => self.set_reg(rd, self.hi),             // MFHI
                0x11 => self.hi = self.gpr[rs],                // MTHI
                0x12 => self.set_reg(rd, self.lo),             // MFLO
                0x13 => self.lo = self.gpr[rs],                // MTLO
                0x18 => {
                    // MULT
                    let prod = ((self.gpr[rs] as i32 as i64) * (self.gpr[rt] as i32 as i64)) as u64;
                    self.lo = prod as u32;
                    self.hi = (prod >> 32) as u32;
                }
                0x19 => {
                    // MULTU
                    let prod = (self.gpr[rs] as u64) * (self.gpr[rt] as u64);
                    self.lo = prod as u32;
                    self.hi = (prod >> 32) as u32;
                }
                0x1A => {
                    // DIV (never traps on divide-by-zero or 0x80000000 / -1)
                    let n = self.gpr[rs] as i32;
                    let d = self.gpr[rt] as i32;
                    if d == 0 {
                        self.hi = self.gpr[rs];
                        self.lo = if n >= 0 { 0xFFFF_FFFF } else { 1 };
                    } else if (n as u32) == 0x8000_0000 && d == -1 {
                        self.hi = 0;
                        self.lo = 0x8000_0000;
                    } else {
                        self.lo = (n / d) as u32;
                        self.hi = (n % d) as u32;
                    }
                }
                0x1B => {
                    // DIVU (never traps on divide-by-zero)
                    let n = self.gpr[rs];
                    let d = self.gpr[rt];
                    if d == 0 {
                        self.hi = n;
                        self.lo = 0xFFFF_FFFF;
                    } else {
                        self.lo = n / d;
                        self.hi = n % d;
                    }
                }
                0x20 => {
                    // ADD (signed overflow trap ExCode = 0x0C)
                    match (self.gpr[rs] as i32).checked_add(self.gpr[rt] as i32) {
                        Some(res) => self.set_reg(rd, res as u32),
                        None => self.enter_exception(EX_OVF, None, 0),
                    }
                }
                0x21 => self.set_reg(rd, self.gpr[rs].wrapping_add(self.gpr[rt])), // ADDU
                0x22 => {
                    // SUB (signed overflow trap ExCode = 0x0C)
                    match (self.gpr[rs] as i32).checked_sub(self.gpr[rt] as i32) {
                        Some(res) => self.set_reg(rd, res as u32),
                        None => self.enter_exception(EX_OVF, None, 0),
                    }
                }
                0x23 => self.set_reg(rd, self.gpr[rs].wrapping_sub(self.gpr[rt])), // SUBU
                0x24 => self.set_reg(rd, self.gpr[rs] & self.gpr[rt]),             // AND
                0x25 => self.set_reg(rd, self.gpr[rs] | self.gpr[rt]),             // OR
                0x26 => self.set_reg(rd, self.gpr[rs] ^ self.gpr[rt]),             // XOR
                0x27 => self.set_reg(rd, !(self.gpr[rs] | self.gpr[rt])),          // NOR
                0x2A => self.set_reg(rd, ((self.gpr[rs] as i32) < (self.gpr[rt] as i32)) as u32), // SLT
                0x2B => self.set_reg(rd, (self.gpr[rs] < self.gpr[rt]) as u32),    // SLTU
                _ => self.enter_exception(EX_RI, None, 0),
            },

            // 0x01: REGIMM / BCONDZ (hardware bit-decoding: bit 16 = BGEZ, bits 20..17 == 0b1000 = link R31)
            0x01 => {
                let is_bgez = (rt & 1) != 0;
                let link = (rt >> 1) == 0b1000;
                let rs_val = self.gpr[rs] as i32;
                let cond = if is_bgez { rs_val >= 0 } else { rs_val < 0 };
                let ret = self.next_pc;
                if link {
                    self.set_reg(31, ret);
                }
                self.branch(cond, imm16);
            }

            0x02 => {
                // J
                self.next_pc = (self.pc & 0xF000_0000) | (target << 2);
                self.next_is_branch_delay = true;
                self.next_branch_taken = true;
            }
            0x03 => {
                // JAL
                let ret = self.next_pc;
                self.next_pc = (self.pc & 0xF000_0000) | (target << 2);
                self.set_reg(31, ret);
                self.next_is_branch_delay = true;
                self.next_branch_taken = true;
            }
            0x04 => self.branch(self.gpr[rs] == self.gpr[rt], imm16),        // BEQ
            0x05 => self.branch(self.gpr[rs] != self.gpr[rt], imm16),        // BNE
            0x06 => self.branch((self.gpr[rs] as i32) <= 0, imm16),          // BLEZ
            0x07 => self.branch((self.gpr[rs] as i32) > 0, imm16),           // BGTZ
            0x08 => {
                // ADDI (signed overflow trap ExCode = 0x0C)
                match (self.gpr[rs] as i32).checked_add(sign_imm as i32) {
                    Some(res) => self.set_reg(rt, res as u32),
                    None => self.enter_exception(EX_OVF, None, 0),
                }
            }
            0x09 => self.set_reg(rt, self.gpr[rs].wrapping_add(sign_imm)),   // ADDIU
            0x0A => self.set_reg(rt, ((self.gpr[rs] as i32) < (sign_imm as i32)) as u32), // SLTI
            0x0B => self.set_reg(rt, (self.gpr[rs] < sign_imm) as u32),      // SLTIU
            0x0C => self.set_reg(rt, self.gpr[rs] & imm16),                  // ANDI
            0x0D => self.set_reg(rt, self.gpr[rs] | imm16),                  // ORI
            0x0E => self.set_reg(rt, self.gpr[rs] ^ imm16),                  // XORI
            0x0F => self.set_reg(rt, imm16 << 16),                           // LUI

            // 0x10: COP0
            0x10 => {
                if self.in_user_mode() && (self.cop0[12] & (1 << 28)) == 0 {
                    self.enter_exception(EX_CPU, None, 0);
                    return;
                }
                if (rs & 0x10) != 0 {
                    if funct == 0x10 {
                        // RFE (0x42000010)
                        let sr = self.cop0[12];
                        self.cop0[12] = (sr & !0x0F) | ((sr >> 2) & 0x0F);
                    } else {
                        self.enter_exception(EX_RI, None, 0);
                    }
                } else {
                    match rs {
                        0x00 => self.queue_load(rt, self.cop0[rd]), // MFC0
                        0x04 => {
                            // MTC0
                            let val = self.gpr[rt];
                            match rd {
                                13 => self.cop0[13] = (self.cop0[13] & !0x0300) | (val & 0x0300),
                                8 | 15 => {}
                                _ => self.cop0[rd] = val,
                            }
                        }
                        0x08 => self.branch((rt & 1) == 0, imm16),  // BC0F / BC0T
                        _ => self.enter_exception(EX_RI, None, 0),
                    }
                }
            }
            0x11 => {
                if (self.cop0[12] & (1 << 29)) == 0 {
                    self.enter_exception(EX_CPU, None, 1);
                }
            }
            0x12 => {
                if (self.cop0[12] & (1 << 30)) == 0 {
                    self.enter_exception(EX_CPU, None, 2);
                    return;
                }
                if (rs & 0x10) != 0 {
                    self.execute_gte(instr);
                } else {
                    match rs {
                        0x00 => self.queue_load(rt, self.read_cop2_data(rd)),
                        0x02 => self.queue_load(rt, self.read_cop2_ctrl(rd)),
                        0x04 => self.write_cop2_data(rd, self.gpr[rt]),
                        0x06 => self.write_cop2_ctrl(rd, self.gpr[rt]),
                        0x08 => self.branch((rt & 1) == 0, imm16),
                        _ => self.enter_exception(EX_RI, None, 0),
                    }
            }
                }
            0x13 => {
                if (self.cop0[12] & (1 << 31)) == 0 {
                    self.enter_exception(EX_CPU, None, 3);
                }
            }

            // 0x20..0x26: Loads (LB, LH, LWL, LW, LBU, LHU, LWR)
            0x20 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let val = (bus.read8(addr) as i8) as i32 as u32;
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                self.queue_load(rt, val);
            }
            0x21 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 1) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let val = (bus.read16(addr) as i16) as i32 as u32;
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                self.queue_load(rt, val);
            }
            0x22 => {
                // LWL (merges against in-flight pending_load if targeting same rt)
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let aligned_word = bus.read32(addr & !3);
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                let offset = addr & 3;
                let cur_rt = self.read_reg_with_load_forwarding(rt);
                let mask = 0x00FF_FFFFu32 >> (offset * 8);
                let new_rt = (cur_rt & mask) | (aligned_word << ((3 - offset) * 8));
                self.queue_load(rt, new_rt);
            }
            0x23 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 3) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let val = bus.read32(addr);
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                self.queue_load(rt, val);
            }
            0x24 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let val = bus.read8(addr) as u32;
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                self.queue_load(rt, val);
            }
            0x25 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 1) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let val = bus.read16(addr) as u32;
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                self.queue_load(rt, val);
            }
            0x26 => {
                // LWR (merges against in-flight pending_load if targeting same rt)
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let aligned_word = bus.read32(addr & !3);
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                let offset = addr & 3;
                let cur_rt = self.read_reg_with_load_forwarding(rt);
                let mask = if offset == 0 { 0 } else { 0xFFFF_FF00u32 << ((3 - offset) * 8) };
                let new_rt = (cur_rt & mask) | (aligned_word >> (offset * 8));
                self.queue_load(rt, new_rt);
            }

            // 0x28..0x2E: Stores (SB, SH, SWL, SW, SWR)
            0x28 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADES, Some(addr), 0);
                    return;
                }
                bus.write8(addr, self.gpr[rt] as u8, self.cache_isolated());
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                }
            }
            0x29 => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 1) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADES, Some(addr), 0);
                    return;
                }
                bus.write16(addr, self.gpr[rt] as u16, self.cache_isolated());
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                }
            }
            0x2A => {
                // SWL
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADES, Some(addr), 0);
                    return;
                }
                let aligned_addr = addr & !3;
                let aligned_word = bus.read32(aligned_addr);
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                let offset = addr & 3;
                let mem_mask = if offset == 3 { 0 } else { 0xFFFF_FF00u32 << (offset * 8) };
                let new_mem = (aligned_word & mem_mask) | (self.gpr[rt] >> ((3 - offset) * 8));
                bus.write32(aligned_addr, new_mem, self.cache_isolated());
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                }
            }
            0x2B => {
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 3) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADES, Some(addr), 0);
                    return;
                }
                bus.write32(addr, self.gpr[rt], self.cache_isolated());
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                }
            }
            0x2E => {
                // SWR
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if self.in_user_mode() && addr >= 0x8000_0000 {
                    self.enter_exception(EX_ADES, Some(addr), 0);
                    return;
                }
                let aligned_addr = addr & !3;
                let aligned_word = bus.read32(aligned_addr);
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                let offset = addr & 3;
                let mem_mask = 0x00FF_FFFFu32 >> ((3 - offset) * 8);
                let new_mem = (aligned_word & mem_mask) | (self.gpr[rt] << (offset * 8));
                bus.write32(aligned_addr, new_mem, self.cache_isolated());
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                }
            }
            0x30..=0x33 => {
                let cop_num = op - 0x30;
                let cu_mask = 1u32 << (28 + cop_num);
                if (cop_num == 0 && self.in_user_mode() && (self.cop0[12] & cu_mask) == 0)
                    || (cop_num != 0 && (self.cop0[12] & cu_mask) == 0)
                {
                    self.enter_exception(EX_CPU, None, cop_num);
                    return;
                }
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 3) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADEL, Some(addr), 0);
                    return;
                }
                let val = bus.read32(addr);
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                    return;
                }
                if cop_num == 2 {
                    self.write_cop2_data(rt, val);
                }
            }
            0x38..=0x3B => {
                let cop_num = op - 0x38;
                let cu_mask = 1u32 << (28 + cop_num);
                if (cop_num == 0 && self.in_user_mode() && (self.cop0[12] & cu_mask) == 0)
                    || (cop_num != 0 && (self.cop0[12] & cu_mask) == 0)
                {
                    self.enter_exception(EX_CPU, None, cop_num);
                    return;
                }
                let addr = self.gpr[rs].wrapping_add(sign_imm);
                if (addr & 3) != 0 || (self.in_user_mode() && addr >= 0x8000_0000) {
                    self.enter_exception(EX_ADES, Some(addr), 0);
                    return;
                }
                let val = if cop_num == 2 { self.read_cop2_data(rt) } else { 0 };
                bus.write32(addr, val, self.cache_isolated());
                if bus.take_bus_error() {
                    self.enter_exception(EX_DBE, None, 0);
                }
            }
            _ => self.enter_exception(EX_RI, None, 0),
        }
    }

    #[inline]
    pub fn finalize_gte_flag(flag: u32) -> u32 {
        let masked = flag & 0x7FFF_F000;
        let err = (((masked & 0x7F87_E000) != 0) as u32) << 31;
        masked | err
    }

    #[inline]
    fn sync_orgb(&mut self) {
        let r = (((self.cop2_data[9] as i16 as i32) / 0x80).clamp(0, 31)) as u32;
        let g = (((self.cop2_data[10] as i16 as i32) / 0x80).clamp(0, 31)) as u32;
        let b = (((self.cop2_data[11] as i16 as i32) / 0x80).clamp(0, 31)) as u32;
        let orgb = r | (g << 5) | (b << 10);
        self.cop2_data[28] = orgb;
        self.cop2_data[29] = orgb;
    }

    pub fn read_cop2_data(&self, idx: usize) -> u32 {
        match idx {
            1 | 3 | 5 | 8 | 9 | 10 | 11 => (self.cop2_data[idx] as i16 as i32) as u32,
            7 | 16..=19 => self.cop2_data[idx] & 0xFFFF,
            15 => self.cop2_data[14],
            28 | 29 => {
                let r = (((self.cop2_data[9] as i16 as i32) / 0x80).clamp(0, 31)) as u32;
                let g = (((self.cop2_data[10] as i16 as i32) / 0x80).clamp(0, 31)) as u32;
                let b = (((self.cop2_data[11] as i16 as i32) / 0x80).clamp(0, 31)) as u32;
                r | (g << 5) | (b << 10)
            }
            0..=31 => self.cop2_data[idx],
            _ => 0,
        }
    }

    pub fn write_cop2_data(&mut self, idx: usize, val: u32) {
        match idx {
            1 | 3 | 5 | 8 => self.cop2_data[idx] = (val as i16 as i32) as u32,
            9..=11 => {
                self.cop2_data[idx] = (val as i16 as i32) as u32;
                self.sync_orgb();
            }
            7 | 16..=19 => self.cop2_data[idx] = val & 0xFFFF,
            14 => {
                self.cop2_data[14] = val;
                self.cop2_data[15] = val;
            }
            15 => {
                self.cop2_data[12] = self.cop2_data[13];
                self.cop2_data[13] = self.cop2_data[14];
                self.cop2_data[14] = val;
                self.cop2_data[15] = val;
            }
            28 => {
                let ir1 = ((val & 0x1F) * 0x80) as i16 as i32 as u32;
                let ir2 = (((val >> 5) & 0x1F) * 0x80) as i16 as i32 as u32;
                let ir3 = (((val >> 10) & 0x1F) * 0x80) as i16 as i32 as u32;
                self.cop2_data[9] = ir1;
                self.cop2_data[10] = ir2;
                self.cop2_data[11] = ir3;
                self.cop2_data[28] = val & 0x7FFF;
                self.cop2_data[29] = val & 0x7FFF;
            }
            29 | 31 => {}
            30 => {
                self.cop2_data[30] = val;
                self.cop2_data[31] = if (val & 0x8000_0000) == 0 {
                    val.leading_zeros()
                } else {
                    (!val).leading_zeros()
                };
            }
            0..=31 => self.cop2_data[idx] = val,
            _ => {}
        }
    }

    pub fn read_cop2_ctrl(&self, idx: usize) -> u32 {
        match idx {
            4 | 12 | 20 | 26 | 27 | 29 | 30 => (self.cop2_ctrl[idx] as i16 as i32) as u32,
            31 => Self::finalize_gte_flag(self.cop2_ctrl[31]),
            0..=31 => self.cop2_ctrl[idx],
            _ => 0,
        }
    }

    pub fn write_cop2_ctrl(&mut self, idx: usize, val: u32) {
        match idx {
            4 | 12 | 20 | 26 | 27 | 29 | 30 => self.cop2_ctrl[idx] = (val as i16 as i32) as u32,
            31 => self.cop2_ctrl[31] = Self::finalize_gte_flag(val),
            0..=31 => self.cop2_ctrl[idx] = val,
            _ => {}
        }
    }

    #[inline]
    fn check_mac_overflow(&mut self, idx: usize, val: i64) -> i64 {
        if val > 0x7FF_FFFF_FFFF {
            self.cop2_ctrl[31] |= 1 << (31 - idx);
        } else if val < -0x800_0000_0000 {
            self.cop2_ctrl[31] |= 1 << (28 - idx);
        }
        (val << 20) >> 20
    }

    #[inline]
    fn set_mac(&mut self, idx: usize, val: i64, shift: u32) -> i32 {
        let wrapped = self.check_mac_overflow(idx, val);
        let res = (wrapped >> shift) as i32;
        self.cop2_data[24 + idx] = res as u32;
        res
    }

    #[inline]
    fn set_mac0(&mut self, val: i64) -> i32 {
        if val > 0x7FFF_FFFF {
            self.cop2_ctrl[31] |= 1 << 16;
        } else if val < -0x8000_0000 {
            self.cop2_ctrl[31] |= 1 << 15;
        }
        let res = val as i32;
        self.cop2_data[24] = res as u32;
        res
    }

    #[inline]
    fn saturate_ir(&mut self, idx: usize, val: i32, lm: bool) -> i16 {
        let min_val = if lm { 0 } else { -0x8000 };
        let clamped = if val < min_val {
            self.cop2_ctrl[31] |= 1 << (25 - idx);
            min_val
        } else if val > 0x7FFF {
            self.cop2_ctrl[31] |= 1 << (25 - idx);
            0x7FFF
        } else {
            val
        };
        clamped as i16
    }

    #[inline]
    fn set_ir(&mut self, idx: usize, val: i32, lm: bool) -> i16 {
        let ir = self.saturate_ir(idx, val, lm);
        self.cop2_data[8 + idx] = (ir as i32) as u32;
        self.sync_orgb();
        ir
    }

    #[inline]
    fn set_ir0(&mut self, val: i32) -> i16 {
        let clamped = if val < 0 {
            self.cop2_ctrl[31] |= 1 << 12;
            0
        } else if val > 0x1000 {
            self.cop2_ctrl[31] |= 1 << 12;
            0x1000
        } else {
            val
        };
        self.cop2_data[8] = (clamped as i16 as i32) as u32;
        clamped as i16
    }

    #[inline]
    fn set_otz(&mut self, val: i64) -> u16 {
        let clamped = if val < 0 {
            self.cop2_ctrl[31] |= 1 << 18;
            0
        } else if val > 0xFFFF {
            self.cop2_ctrl[31] |= 1 << 18;
            0xFFFF
        } else {
            val as u32
        };
        self.cop2_data[7] = clamped;
        clamped as u16
    }

    #[inline]
    fn push_sz(&mut self, val: i64) -> u16 {
        self.cop2_data[16] = self.cop2_data[17] & 0xFFFF;
        self.cop2_data[17] = self.cop2_data[18] & 0xFFFF;
        self.cop2_data[18] = self.cop2_data[19] & 0xFFFF;
        let clamped = if val < 0 {
            self.cop2_ctrl[31] |= 1 << 18;
            0
        } else if val > 0xFFFF {
            self.cop2_ctrl[31] |= 1 << 18;
            0xFFFF
        } else {
            val as u32
        };
        self.cop2_data[19] = clamped;
        clamped as u16
    }

    #[inline]
    fn saturate_sxy(&mut self, idx: usize, val: i32) -> i16 {
        let bit = if idx == 1 { 14 } else { 13 };
        if val < -0x400 {
            self.cop2_ctrl[31] |= 1 << bit;
            -0x400
        } else if val > 0x3FF {
            self.cop2_ctrl[31] |= 1 << bit;
            0x3FF
        } else {
            val as i16
        }
    }

    #[inline]
    fn push_color_fifo(&mut self) {
        let r_raw = (self.cop2_data[25] as i32) >> 4;
        let g_raw = (self.cop2_data[26] as i32) >> 4;
        let b_raw = (self.cop2_data[27] as i32) >> 4;
        let mut flag = self.cop2_ctrl[31];
        let mut sat_c = |val: i32, bit: u32| -> u32 {
            if val < 0 {
                flag |= 1 << bit;
                0
            } else if val > 0xFF {
                flag |= 1 << bit;
                0xFF
            } else {
                val as u32
            }
        };
        let r = sat_c(r_raw, 21);
        let g = sat_c(g_raw, 20);
        let b = sat_c(b_raw, 19);
        self.cop2_ctrl[31] = flag;
        let code = self.cop2_data[6] & 0xFF00_0000;
        self.cop2_data[20] = self.cop2_data[21];
        self.cop2_data[21] = self.cop2_data[22];
        self.cop2_data[22] = code | (b << 16) | (g << 8) | r;
    }

    fn gte_divide(&mut self, h: u16, sz3: u16) -> u32 {
        if (h as u32) < (sz3 as u32) * 2 {
            let z = sz3.leading_zeros();
            let n = (h as u64) << z;
            let d = (sz3 as u64) << z;
            let idx = ((d.wrapping_sub(0x7FC0)) >> 7) as usize;
            let u = (UNR_TABLE[idx] as u64) + 0x101;
            let d1 = (0x2000080u64.wrapping_sub(d * u)) >> 8;
            let d2 = (0x0000080u64.wrapping_add(d1 * u)) >> 8;
            let res = ((n * d2).wrapping_add(0x8000)) >> 16;
            res.min(0x1FFFF) as u32
        } else {
            self.cop2_ctrl[31] |= 1 << 17;
            0x1FFFF
        }
    }

    #[inline]
    fn gte_matrix(&self, mx: u32) -> [[i16; 3]; 3] {
        match mx & 3 {
            0 => [
                [self.cop2_ctrl[0] as i16, (self.cop2_ctrl[0] >> 16) as i16, self.cop2_ctrl[1] as i16],
                [(self.cop2_ctrl[1] >> 16) as i16, self.cop2_ctrl[2] as i16, (self.cop2_ctrl[2] >> 16) as i16],
                [self.cop2_ctrl[3] as i16, (self.cop2_ctrl[3] >> 16) as i16, self.cop2_ctrl[4] as i16],
            ],
            1 => [
                [self.cop2_ctrl[8] as i16, (self.cop2_ctrl[8] >> 16) as i16, self.cop2_ctrl[9] as i16],
                [(self.cop2_ctrl[9] >> 16) as i16, self.cop2_ctrl[10] as i16, (self.cop2_ctrl[10] >> 16) as i16],
                [self.cop2_ctrl[11] as i16, (self.cop2_ctrl[11] >> 16) as i16, self.cop2_ctrl[12] as i16],
            ],
            2 => [
                [self.cop2_ctrl[16] as i16, (self.cop2_ctrl[16] >> 16) as i16, self.cop2_ctrl[17] as i16],
                [(self.cop2_ctrl[17] >> 16) as i16, self.cop2_ctrl[18] as i16, (self.cop2_ctrl[18] >> 16) as i16],
                [self.cop2_ctrl[19] as i16, (self.cop2_ctrl[19] >> 16) as i16, self.cop2_ctrl[20] as i16],
            ],
            _ => {
                let r = ((self.cop2_data[6] & 0xFF) as i16) << 4;
                let ir0 = self.cop2_data[8] as i16;
                let rt13 = self.cop2_ctrl[1] as i16;
                let rt22 = self.cop2_ctrl[2] as i16;
                [[-r, r, ir0], [rt13, rt13, rt13], [rt22, rt22, rt22]]
            }
        }
    }

    #[inline]
    fn gte_vector(&self, v: u32) -> [i16; 3] {
        match v & 3 {
            0 => [self.cop2_data[0] as i16, (self.cop2_data[0] >> 16) as i16, self.cop2_data[1] as i16],
            1 => [self.cop2_data[2] as i16, (self.cop2_data[2] >> 16) as i16, self.cop2_data[3] as i16],
            2 => [self.cop2_data[4] as i16, (self.cop2_data[4] >> 16) as i16, self.cop2_data[5] as i16],
            _ => [self.cop2_data[9] as i16, self.cop2_data[10] as i16, self.cop2_data[11] as i16],
        }
    }

    #[inline]
    fn gte_translation(&self, cv: u32) -> [i32; 3] {
        match cv & 3 {
            0 => [self.cop2_ctrl[5] as i32, self.cop2_ctrl[6] as i32, self.cop2_ctrl[7] as i32],
            1 => [self.cop2_ctrl[13] as i32, self.cop2_ctrl[14] as i32, self.cop2_ctrl[15] as i32],
            2 => [self.cop2_ctrl[21] as i32, self.cop2_ctrl[22] as i32, self.cop2_ctrl[23] as i32],
            _ => [0, 0, 0],
        }
    }

    fn gte_multiply_matrix_vec(&mut self, mx: u32, v: u32, cv: u32, shift: u32, lm: bool) {
        let m = self.gte_matrix(mx);
        let vec = self.gte_vector(v);
        let tx = self.gte_translation(cv);
        for i in 0..3 {
            let idx = i + 1;
            let p0 = (tx[i] as i64) << 12;
            let p1 = (m[i][0] as i64) * (vec[0] as i64);
            let p2 = (m[i][1] as i64) * (vec[1] as i64);
            let p3 = (m[i][2] as i64) * (vec[2] as i64);
            if (cv & 3) == 2 {
                let s1 = self.check_mac_overflow(idx, p0 + p1);
                let _ = self.saturate_ir(idx, (s1 >> shift) as i32, false);
                let s2 = self.check_mac_overflow(idx, p2);
                let s3 = self.check_mac_overflow(idx, s2 + p3);
                let mac = self.set_mac(idx, s3, shift);
                self.set_ir(idx, mac, lm);
            } else {
                let s1 = self.check_mac_overflow(idx, p0 + p1);
                let s2 = self.check_mac_overflow(idx, s1 + p2);
                let s3 = self.check_mac_overflow(idx, s2 + p3);
                let mac = self.set_mac(idx, s3, shift);
                self.set_ir(idx, mac, lm);
            }
        }
    }

    fn gte_interpolate_far_color(&mut self, base: [i64; 3], shift: u32, lm: bool) {
        let fc = self.gte_translation(2);
        let ir0 = (self.cop2_data[8] as i16) as i64;
        for i in 0..3 {
            let idx = i + 1;
            let diff = self.check_mac_overflow(idx, ((fc[i] as i64) << 12) - base[i]);
            let tmp_ir = self.saturate_ir(idx, (diff >> shift) as i32, false) as i64;
            let acc = self.check_mac_overflow(idx, base[i] + tmp_ir * ir0);
            let mac = self.set_mac(idx, acc, shift);
            self.set_ir(idx, mac, lm);
        }
    }

    fn gte_rtps_vertex(&mut self, v: u32, shift: u32, lm: bool, compute_dq: bool) {
        let m = self.gte_matrix(0);
        let vec = self.gte_vector(v);
        let tr = self.gte_translation(0);
        let mut acc3 = 0i64;
        for i in 0..3 {
            let idx = i + 1;
            let s1 = self.check_mac_overflow(idx, ((tr[i] as i64) << 12) + (m[i][0] as i64) * (vec[0] as i64));
            let s2 = self.check_mac_overflow(idx, s1 + (m[i][1] as i64) * (vec[1] as i64));
            let s3 = self.check_mac_overflow(idx, s2 + (m[i][2] as i64) * (vec[2] as i64));
            let mac = self.set_mac(idx, s3, shift);
            if idx < 3 {
                self.set_ir(idx, mac, lm);
            } else {
                acc3 = s3;
                let ir3_chk = (s3 >> 12) as i32;
                if !(-0x8000..=0x7FFF).contains(&ir3_chk) {
                    self.cop2_ctrl[31] |= 1 << 22;
                }
                let min_ir3 = if lm { 0 } else { -0x8000 };
                let clamped_ir3 = mac.clamp(min_ir3, 0x7FFF) as i16;
                self.cop2_data[11] = (clamped_ir3 as i32) as u32;
                self.sync_orgb();
            }
        }
        let sz3 = self.push_sz(acc3 >> 12);
        let h = (self.cop2_ctrl[26] & 0xFFFF) as u16;
        let div = self.gte_divide(h, sz3) as i64;
        let ir1 = (self.cop2_data[9] as i16) as i64;
        let ir2 = (self.cop2_data[10] as i16) as i64;
        let ofx = (self.cop2_ctrl[24] as i32) as i64;
        let ofy = (self.cop2_ctrl[25] as i32) as i64;
        let proj_x = if self.widescreen_hack {
            (ir1 * div * 3) >> 2
        } else {
            ir1 * div
        };
        let mac0_x = self.set_mac0(proj_x + ofx);
        let sx2 = self.saturate_sxy(1, ((proj_x + ofx) >> 16) as i32);
        let mac0_y = self.set_mac0(ir2 * div + ofy);
        let sy2 = self.saturate_sxy(2, ((ir2 * div + ofy) >> 16) as i32);
        let _ = (mac0_x, mac0_y);
        let sxy2 = (sx2 as u16 as u32) | ((sy2 as u16 as u32) << 16);
        self.write_cop2_data(15, sxy2);
        if compute_dq {
            let dqa = (self.cop2_ctrl[27] as i16) as i64;
            let dqb = (self.cop2_ctrl[28] as i32) as i64;
            let dq = dqb + dqa * div;
            self.set_mac0(dq);
            self.set_ir0((dq >> 12) as i32);
        }
    }

    pub fn execute_gte(&mut self, instr: u32) {
        self.cop2_ctrl[31] = 0;
        let shift = if (instr & (1 << 19)) != 0 { 12 } else { 0 };
        let lm = (instr & (1 << 10)) != 0;
        let opc = instr & 0x3F;
        match opc {
            0x01 => self.gte_rtps_vertex(0, shift, lm, true),
            0x06 => {
                let sx0 = (self.cop2_data[12] as i16) as i64;
                let sy0 = ((self.cop2_data[12] >> 16) as i16) as i64;
                let sx1 = (self.cop2_data[13] as i16) as i64;
                let sy1 = ((self.cop2_data[13] >> 16) as i16) as i64;
                let sx2 = (self.cop2_data[14] as i16) as i64;
                let sy2 = ((self.cop2_data[14] >> 16) as i16) as i64;
                let mut area = sx0 * sy1 + sx1 * sy2 + sx2 * sy0 - sx0 * sy2 - sx1 * sy0 - sx2 * sy1;
                if self.pgxp_culling_fix && area == 0 && (sx0 != sx1 || sy0 != sy1 || sx1 != sx2 || sy1 != sy2) {
                    area = 1;
                }
                self.set_mac0(area);
            }
            0x0C => {
                let d1 = (self.cop2_ctrl[0] as i16) as i64;
                let d2 = (self.cop2_ctrl[2] as i16) as i64;
                let d3 = (self.cop2_ctrl[4] as i16) as i64;
                let ir1 = (self.cop2_data[9] as i16) as i64;
                let ir2 = (self.cop2_data[10] as i16) as i64;
                let ir3 = (self.cop2_data[11] as i16) as i64;
                let m1 = self.set_mac(1, ir3 * d2 - ir2 * d3, shift);
                let m2 = self.set_mac(2, ir1 * d3 - ir3 * d1, shift);
                let m3 = self.set_mac(3, ir2 * d1 - ir1 * d2, shift);
                self.set_ir(1, m1, lm);
                self.set_ir(2, m2, lm);
                self.set_ir(3, m3, lm);
            }
            0x10 => {
                let rgbc = self.cop2_data[6];
                let base = [
                    ((rgbc & 0xFF) as i64) << 16,
                    (((rgbc >> 8) & 0xFF) as i64) << 16,
                    (((rgbc >> 16) & 0xFF) as i64) << 16,
                ];
                self.gte_interpolate_far_color(base, shift, lm);
                self.push_color_fifo();
            }
            0x11 => {
                let base = [
                    ((self.cop2_data[9] as i16) as i64) << 12,
                    ((self.cop2_data[10] as i16) as i64) << 12,
                    ((self.cop2_data[11] as i16) as i64) << 12,
                ];
                self.gte_interpolate_far_color(base, shift, lm);
                self.push_color_fifo();
            }
            0x12 => {
                let mx = (instr >> 17) & 3;
                let v = (instr >> 15) & 3;
                let cv = (instr >> 13) & 3;
                self.gte_multiply_matrix_vec(mx, v, cv, shift, lm);
            }
            0x13 | 0x16 => {
                let count = if opc == 0x16 { 3 } else { 1 };
                for v in 0..count {
                    self.gte_multiply_matrix_vec(1, v, 3, shift, lm);
                    self.gte_multiply_matrix_vec(2, 3, 1, shift, lm);
                    let rgbc = self.cop2_data[6];
                    let base = [
                        (((rgbc & 0xFF) as i64) * ((self.cop2_data[9] as i16) as i64)) << 4,
                        ((((rgbc >> 8) & 0xFF) as i64) * ((self.cop2_data[10] as i16) as i64)) << 4,
                        ((((rgbc >> 16) & 0xFF) as i64) * ((self.cop2_data[11] as i16) as i64)) << 4,
                    ];
                    self.gte_interpolate_far_color(base, shift, lm);
                    self.push_color_fifo();
                }
            }
            0x14 => {
                self.gte_multiply_matrix_vec(2, 3, 1, shift, lm);
                let rgbc = self.cop2_data[6];
                let base = [
                    (((rgbc & 0xFF) as i64) * ((self.cop2_data[9] as i16) as i64)) << 4,
                    ((((rgbc >> 8) & 0xFF) as i64) * ((self.cop2_data[10] as i16) as i64)) << 4,
                    ((((rgbc >> 16) & 0xFF) as i64) * ((self.cop2_data[11] as i16) as i64)) << 4,
                ];
                self.gte_interpolate_far_color(base, shift, lm);
                self.push_color_fifo();
            }
            0x1B | 0x3F => {
                let count = if opc == 0x3F { 3 } else { 1 };
                for v in 0..count {
                    self.gte_multiply_matrix_vec(1, v, 3, shift, lm);
                    self.gte_multiply_matrix_vec(2, 3, 1, shift, lm);
                    let rgbc = self.cop2_data[6];
                    let c = [(rgbc & 0xFF) as i64, ((rgbc >> 8) & 0xFF) as i64, ((rgbc >> 16) & 0xFF) as i64];
                    for i in 0..3 {
                        let idx = i + 1;
                        let ir = (self.cop2_data[8 + idx] as i16) as i64;
                        let mac = self.set_mac(idx, (c[i] * ir) << 4, shift);
                        self.set_ir(idx, mac, lm);
                    }
                    self.push_color_fifo();
                }
            }
            0x1C => {
                self.gte_multiply_matrix_vec(2, 3, 1, shift, lm);
                let rgbc = self.cop2_data[6];
                let c = [(rgbc & 0xFF) as i64, ((rgbc >> 8) & 0xFF) as i64, ((rgbc >> 16) & 0xFF) as i64];
                for i in 0..3 {
                    let idx = i + 1;
                    let ir = (self.cop2_data[8 + idx] as i16) as i64;
                    let mac = self.set_mac(idx, (c[i] * ir) << 4, shift);
                    self.set_ir(idx, mac, lm);
                }
                self.push_color_fifo();
            }
            0x1E | 0x20 => {
                let count = if opc == 0x20 { 3 } else { 1 };
                for v in 0..count {
                    self.gte_multiply_matrix_vec(1, v, 3, shift, lm);
                    self.gte_multiply_matrix_vec(2, 3, 1, shift, lm);
                    self.push_color_fifo();
                }
            }
            0x28 => {
                for idx in 1..=3 {
                    let ir = (self.cop2_data[8 + idx] as i16) as i64;
                    let mac = self.set_mac(idx, ir * ir, shift);
                    self.set_ir(idx, mac, lm);
                }
            }
            0x29 => {
                let rgbc = self.cop2_data[6];
                let base = [
                    (((rgbc & 0xFF) as i64) * ((self.cop2_data[9] as i16) as i64)) << 4,
                    ((((rgbc >> 8) & 0xFF) as i64) * ((self.cop2_data[10] as i16) as i64)) << 4,
                    ((((rgbc >> 16) & 0xFF) as i64) * ((self.cop2_data[11] as i16) as i64)) << 4,
                ];
                self.gte_interpolate_far_color(base, shift, lm);
                self.push_color_fifo();
            }
            0x2A => {
                for _ in 0..3 {
                    let rgb0 = self.cop2_data[20];
                    let base = [
                        ((rgb0 & 0xFF) as i64) << 16,
                        (((rgb0 >> 8) & 0xFF) as i64) << 16,
                        (((rgb0 >> 16) & 0xFF) as i64) << 16,
                    ];
                    self.gte_interpolate_far_color(base, shift, lm);
                    self.push_color_fifo();
                }
            }
            0x2D => {
                let zsf3 = (self.cop2_ctrl[29] as i16) as i64;
                let sum = (self.cop2_data[17] & 0xFFFF) as i64
                    + (self.cop2_data[18] & 0xFFFF) as i64
                    + (self.cop2_data[19] & 0xFFFF) as i64;
                let prod = zsf3 * sum;
                self.set_mac0(prod);
                self.set_otz(prod >> 12);
            }
            0x2E => {
                let zsf4 = (self.cop2_ctrl[30] as i16) as i64;
                let sum = (self.cop2_data[16] & 0xFFFF) as i64
                    + (self.cop2_data[17] & 0xFFFF) as i64
                    + (self.cop2_data[18] & 0xFFFF) as i64
                    + (self.cop2_data[19] & 0xFFFF) as i64;
                let prod = zsf4 * sum;
                self.set_mac0(prod);
                self.set_otz(prod >> 12);
            }
            0x30 => {
                self.gte_rtps_vertex(0, shift, lm, false);
                self.gte_rtps_vertex(1, shift, lm, false);
                self.gte_rtps_vertex(2, shift, lm, true);
            }
            0x3D => {
                let ir0 = (self.cop2_data[8] as i16) as i64;
                for idx in 1..=3 {
                    let ir = (self.cop2_data[8 + idx] as i16) as i64;
                    let mac = self.set_mac(idx, ir0 * ir, shift);
                    self.set_ir(idx, mac, lm);
                }
                self.push_color_fifo();
            }
            0x3E => {
                let ir0 = (self.cop2_data[8] as i16) as i64;
                for idx in 1..=3 {
                    let base = ((self.cop2_data[24 + idx] as i32) as i64) << shift;
                    let ir = (self.cop2_data[8 + idx] as i16) as i64;
                    let mac = self.set_mac(idx, base + ir0 * ir, shift);
                    self.set_ir(idx, mac, lm);
                }
                self.push_color_fifo();
            }
            _ => {}
        }
        self.cop2_ctrl[31] = Self::finalize_gte_flag(self.cop2_ctrl[31]);
    }

    pub fn dump_state_json(&self, tty: &str) -> String {
        format!(
            "{{\n  \"pc\": \"0x{:08X}\",\n  \"cycles\": {},\n  \"sr\": \"0x{:08X}\",\n  \"cause\": \"0x{:08X}\",\n  \"epc\": \"0x{:08X}\",\n  \"tty_output\": {:?}\n}}",
            self.pc, self.cycles, self.cop0[12], self.cop0[13], self.cop0[14], tty
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup_cpu_with_code(code: &[u32]) -> (Cpu, Bus) {
        let mut bus = Bus::new();
        let mut cpu = Cpu::new();
        cpu.cop0[12] = 0; // Kernel mode, BEV = 0
        cpu.pc = 0x8000_0000;
        cpu.next_pc = 0x8000_0004;
        for (i, &instr) in code.iter().enumerate() {
            bus.write32(0x8000_0000 + (i as u32) * 4, instr, false);
        }
        (cpu, bus)
    }

    #[test]
    fn test_load_delay_slot_and_override_cancellation() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[0x8C01_0100, 0x3422_0000, 0x0000_0000]);
        cpu.gpr[1] = 0x1111_1111;
        bus.write32(0x0000_0100, 0xDEAD_BEEF, false);
        cpu.step(&mut bus);
        assert_eq!(cpu.gpr[1], 0x1111_1111);
        cpu.step(&mut bus);
        assert_eq!(cpu.gpr[2], 0x1111_1111);
        assert_eq!(cpu.gpr[1], 0xDEAD_BEEF);

        let (mut cpu2, mut bus2) = setup_cpu_with_code(&[0x8C01_0100, 0x3401_1234, 0x0000_0000]);
        bus2.write32(0x0000_0100, 0xDEAD_BEEF, false);
        cpu2.step(&mut bus2);
        cpu2.step(&mut bus2);
        assert_eq!(cpu2.gpr[1], 0x0000_1234);
    }

    #[test]
    fn test_lwl_lwr_swl_swr_and_inflight_load_forwarding() {
        let code = [0x8841_0004, 0x9841_0001, 0x0000_0000, 0xA861_0005, 0xB861_0002];
        let (mut cpu, mut bus) = setup_cpu_with_code(&code);
        cpu.gpr[1] = 0xAAAA_BBBB;
        cpu.gpr[2] = 0x8000_1000;
        cpu.gpr[3] = 0x8000_2000;
        bus.write32(0x8000_1000, 0x4433_2211, false);
        bus.write32(0x8000_1004, 0x8877_6655, false);

        cpu.step(&mut bus);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.gpr[1], 0x5544_3322);

        bus.write32(0x8000_2000, 0xFFFF_FFFF, false);
        bus.write32(0x8000_2004, 0xFFFF_FFFF, false);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(bus.read8(0x8000_2002), 0x22);
        assert_eq!(bus.read8(0x8000_2003), 0x33);
        assert_eq!(bus.read8(0x8000_2004), 0x44);
        assert_eq!(bus.read8(0x8000_2005), 0x55);
    }

    #[test]
    fn test_div_and_divu_edge_cases_no_trap() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[
            0x0022_001A,
            0x0062_001A,
            0x0085_001A,
            0x0022_001B,
        ]);
        cpu.gpr[1] = 42;
        cpu.gpr[2] = 0;
        cpu.gpr[3] = (-42i32) as u32;
        cpu.gpr[4] = 0x8000_0000;
        cpu.gpr[5] = 0xFFFF_FFFF;

        cpu.step(&mut bus);
        assert_eq!((cpu.hi, cpu.lo), (42, 0xFFFF_FFFF));
        cpu.step(&mut bus);
        assert_eq!((cpu.hi, cpu.lo), ((-42i32) as u32, 1));
        cpu.step(&mut bus);
        assert_eq!((cpu.hi, cpu.lo), (0, 0x8000_0000));
        cpu.step(&mut bus);
        assert_eq!((cpu.hi, cpu.lo), (42, 0xFFFF_FFFF));
    }

    #[test]
    fn test_regimm_undocumented_encodings_and_r31_link_edge_cases() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[
            0x0422_0002, // REGIMM rs=1, rt=0x02 (undocumented BLTZ), offset=+2
            0x0000_0000,
            0x0000_0000,
            0x07F1_0002, // BGEZAL $31, +2 (rs=31, rt=0x11)
            0x0000_0000,
        ]);
        cpu.gpr[1] = (-5i32) as u32;
        cpu.gpr[31] = (-1i32) as u32;

        cpu.step(&mut bus);
        assert_eq!(cpu.gpr[31], (-1i32) as u32);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x8000_000C);

        cpu.step(&mut bus);
        assert_eq!(cpu.gpr[31], 0x8000_0014);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0x8000_0014);
    }

    #[test]
    fn test_signed_overflow_and_branch_delay_epc_and_rfe() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[0x1000_0004, 0x0022_1820]);
        bus.write32(0x8000_0080, 0x4200_0010, false); // RFE
        cpu.cop0[12] = 0x0000_0005;
        cpu.gpr[1] = 0x7FFF_FFFF;
        cpu.gpr[2] = 1;
        cpu.gpr[3] = 0x9999_9999;

        cpu.step(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.gpr[3], 0x9999_9999);
        assert_eq!(cpu.cop0[14], 0x8000_0000);
        assert_eq!((cpu.cop0[13] >> 31) & 1, 1);
        assert_eq!((cpu.cop0[13] >> 2) & 0x1F, EX_OVF);
        assert_eq!(cpu.cop0[12] & 0x3F, 0b01_0100);
        assert_eq!(cpu.pc, 0x8000_0080);

        cpu.step(&mut bus);
        assert_eq!(cpu.cop0[12] & 0x0F, 0b0101);
    }

    #[test]
    fn test_alignment_exceptions_and_bios_tty_and_bus_error() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[0x8C01_0002]);
        cpu.step(&mut bus);
        assert_eq!((cpu.cop0[13] >> 2) & 0x1F, EX_ADEL);
        assert_eq!(cpu.cop0[8], 0x0000_0002);

        let (mut cpu2, mut bus2) = setup_cpu_with_code(&[0xAC01_0003]);
        cpu2.step(&mut bus2);
        assert_eq!((cpu2.cop0[13] >> 2) & 0x1F, EX_ADES);
        assert_eq!(cpu2.cop0[8], 0x0000_0003);

        let (mut cpu3, mut bus3) = setup_cpu_with_code(&[0x3C01_2000, 0x8C22_0000]);
        cpu3.step(&mut bus3);
        cpu3.step(&mut bus3);
        assert_eq!((cpu3.cop0[13] >> 2) & 0x1F, EX_DBE);
    }
    #[test]
    fn test_gte_special_registers_sxyp_lzcs_irgb_h_and_flag() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[
            0x4881_7800, // MTC2 $1, $15 (SXYP)
            0x4882_7800, // MTC2 $2, $15 (SXYP)
            0x4883_7800, // MTC2 $3, $15 (SXYP)
            0x4804_7800, // MFC2 $4, $15 (SXYP -> reads SXY2)
            0x4885_F000, // MTC2 $5, $30 (LZCS)
            0x4806_F800, // MFC2 $6, $31 (LZCR)
            0x4887_F000, // MTC2 $7, $30 (LZCS)
            0x4808_F800, // MFC2 $8, $31 (LZCR)
            0x4880_F000, // MTC2 $0, $30 (LZCS = 0)
            0x4809_F800, // MFC2 $9, $31 (LZCR = 32)
            0x488A_E000, // MTC2 $10, $28 (IRGB)
            0x480B_E800, // MFC2 $11, $29 (ORGB)
            0x48CD_D000, // CTC2 $13, $26 (H)
            0x484E_D000, // CFC2 $14, $26 (H -> sign-extended)
            0x48CF_F800, // CTC2 $15, $31 (FLAG)
            0x4850_F800, // CFC2 $16, $31 (FLAG)
            0x0000_0000, // NOP (for load delay slot)
        ]);
        cpu.cop0[12] |= 1 << 30;
        cpu.gpr[1] = 0x0010_0020;
        cpu.gpr[2] = 0x0030_0040;
        cpu.gpr[3] = 0x0050_0060;
        cpu.gpr[5] = 0x000F_FFFF;
        cpu.gpr[7] = 0xFFF0_0000;
        cpu.gpr[10] = (31 << 10) | (16 << 5) | 7;
        cpu.gpr[13] = 0x0000_8000;
        cpu.gpr[15] = 0x0004_000F;

        for _ in 0..17 {
            cpu.step(&mut bus);
        }

        assert_eq!(cpu.read_cop2_data(12), 0x0010_0020);
        assert_eq!(cpu.read_cop2_data(13), 0x0030_0040);
        assert_eq!(cpu.read_cop2_data(14), 0x0050_0060);
        assert_eq!(cpu.gpr[4], 0x0050_0060);
        assert_eq!(cpu.gpr[6], 12);
        assert_eq!(cpu.gpr[8], 12);
        assert_eq!(cpu.gpr[9], 32);
        cpu.write_cop2_data(30, 0xFFFF_FFFF);
        assert_eq!(cpu.read_cop2_data(31), 32);
        assert_eq!(cpu.read_cop2_data(9), 7 * 0x80);
        assert_eq!(cpu.read_cop2_data(10), 16 * 0x80);
        assert_eq!(cpu.read_cop2_data(11), 31 * 0x80);
        assert_eq!(cpu.gpr[11], 0x7E07);
        assert_eq!(cpu.gpr[14], 0xFFFF_8000);
        assert_eq!(cpu.gpr[16], 0x8004_0000);
        cpu.write_cop2_ctrl(31, 0x0040_000F);
        assert_eq!(cpu.read_cop2_ctrl(31), 0x0040_0000);
    }

    #[test]
    fn test_gte_nclip_avsz_mvmva_sqr_gpf_and_gpl() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[
            0x4A00_0006, // NCLIP
            0x4A00_002D, // AVSZ3
            0x4A00_002E, // AVSZ4
            0x4A08_0412, // MVMVA (sf=1, mx=0 RT, v=0 V0, cv=0 TR, lm=1)
            0x4A08_0012, // MVMVA (sf=1, mx=0 RT, v=0 V0, cv=0 TR, lm=0)
            0x4A00_0028, // SQR (sf=0, lm=0)
            0x4A08_003D, // GPF (sf=1, lm=0)
            0x4A08_003E, // GPL (sf=1, lm=0)
        ]);
        cpu.cop0[12] |= 1 << 30;

        cpu.write_cop2_data(12, (10 << 16) | 10);
        cpu.write_cop2_data(13, (10 << 16) | 30);
        cpu.write_cop2_data(14, (40 << 16) | 10);
        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(24) as i32, 600);

        cpu.write_cop2_data(16, 100);
        cpu.write_cop2_data(17, 200);
        cpu.write_cop2_data(18, 300);
        cpu.write_cop2_data(19, 400);
        cpu.write_cop2_ctrl(29, 0x1000);
        cpu.write_cop2_ctrl(30, 0x0400);
        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(7), 900);
        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(7), 250);

        cpu.write_cop2_ctrl(0, 0x0000_1000);
        cpu.write_cop2_ctrl(1, 0x0000_0000);
        cpu.write_cop2_ctrl(2, 0x0000_2000);
        cpu.write_cop2_ctrl(3, 0x0000_0000);
        cpu.write_cop2_ctrl(4, (-0x1000i16) as u16 as u32);
        cpu.write_cop2_ctrl(5, 10);
        cpu.write_cop2_ctrl(6, (-20i32) as u32);
        cpu.write_cop2_ctrl(7, 5);
        cpu.write_cop2_data(0, (50 << 16) | 100);
        cpu.write_cop2_data(1, 25);

        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(25) as i32, 110);
        assert_eq!(cpu.read_cop2_data(26) as i32, 80);
        assert_eq!(cpu.read_cop2_data(27) as i32, -20);
        assert_eq!(cpu.read_cop2_data(9) as i32, 110);
        assert_eq!(cpu.read_cop2_data(10) as i32, 80);
        assert_eq!(cpu.read_cop2_data(11) as i32, 0);
        assert_ne!(cpu.read_cop2_ctrl(31) & (1 << 22), 0);

        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(11) as i32, -20);

        cpu.write_cop2_data(9, 16);
        cpu.write_cop2_data(10, (-32i16) as u16 as u32);
        cpu.write_cop2_data(11, 64);
        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(25), 256);
        assert_eq!(cpu.read_cop2_data(26), 1024);
        assert_eq!(cpu.read_cop2_data(27), 4096);

        cpu.write_cop2_data(6, 0x2000_0000);
        cpu.write_cop2_data(8, 0x0800);
        cpu.write_cop2_data(9, 200);
        cpu.write_cop2_data(10, 400);
        cpu.write_cop2_data(11, 600);
        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(25), 100);
        assert_eq!(cpu.read_cop2_data(26), 200);
        assert_eq!(cpu.read_cop2_data(27), 300);

        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(25), 150);
        assert_eq!(cpu.read_cop2_data(26), 300);
        assert_eq!(cpu.read_cop2_data(27), 450);
        assert_eq!(cpu.read_cop2_data(22), 0x2000_0000 | ((450 >> 4) << 16) | ((300 >> 4) << 8) | (150 >> 4));
    }

    #[test]
    fn test_gte_rtps_and_rtpt_perspective_projection_and_depth_cueing() {
        let (mut cpu, mut bus) = setup_cpu_with_code(&[
            0x4A18_0001, // RTPS (sf=1, lm=0)
            0x4A28_0030, // RTPT (sf=1, lm=0)
        ]);
        cpu.cop0[12] |= 1 << 30;

        cpu.write_cop2_ctrl(0, 0x0000_1000);
        cpu.write_cop2_ctrl(1, 0);
        cpu.write_cop2_ctrl(2, 0x0000_1000);
        cpu.write_cop2_ctrl(3, 0);
        cpu.write_cop2_ctrl(4, 0x0000_1000);
        cpu.write_cop2_ctrl(5, 0);
        cpu.write_cop2_ctrl(6, 0);
        cpu.write_cop2_ctrl(7, 400);

        cpu.write_cop2_ctrl(24, 160 << 16);
        cpu.write_cop2_ctrl(25, 120 << 16);
        cpu.write_cop2_ctrl(26, 200);
        cpu.write_cop2_ctrl(27, 0x0100);
        cpu.write_cop2_ctrl(28, 0x0080_0000);

        cpu.write_cop2_data(0, (20u32 << 16) | ((-40i16 as u16) as u32));
        cpu.write_cop2_data(1, 0);
        cpu.write_cop2_data(2, (20u32 << 16) | 40);
        cpu.write_cop2_data(3, 0);
        cpu.write_cop2_data(4, ((-60i16 as u16) as u32) << 16);
        cpu.write_cop2_data(5, 0);

        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(19), 400);
        assert_eq!(cpu.read_cop2_data(14), (130 << 16) | 140);
        assert_eq!(cpu.read_cop2_data(8), 0x1000);
        assert_eq!(cpu.read_cop2_ctrl(31) & (1 << 31), 0);

        cpu.step(&mut bus);
        assert_eq!(cpu.read_cop2_data(12), (130 << 16) | 140);
        assert_eq!(cpu.read_cop2_data(13), (130 << 16) | 180);
        assert_eq!(cpu.read_cop2_data(14), (90 << 16) | 160);
        assert_eq!(cpu.read_cop2_data(17), 400);
        assert_eq!(cpu.read_cop2_data(18), 400);
        assert_eq!(cpu.read_cop2_data(19), 400);
    }

    #[test]
    fn test_ps_exe_sideload_sp_zero_preservation_and_bios_shell_hook() {
        let build_test_exe = |pc: u32, gp: u32, load_addr: u32, sp_base: u32, sp_off: u32, code: &[u32]| -> Vec<u8> {
            let payload_len = code.len() * 4;
            let mut exe = vec![0u8; 0x800 + payload_len];
            exe[0x000..0x008].copy_from_slice(b"PS-X EXE");
            exe[0x010..0x014].copy_from_slice(&pc.to_le_bytes());
            exe[0x014..0x018].copy_from_slice(&gp.to_le_bytes());
            exe[0x018..0x01C].copy_from_slice(&load_addr.to_le_bytes());
            exe[0x01C..0x020].copy_from_slice(&(payload_len as u32).to_le_bytes());
            exe[0x030..0x034].copy_from_slice(&sp_base.to_le_bytes());
            exe[0x034..0x038].copy_from_slice(&sp_off.to_le_bytes());
            for (i, &w) in code.iter().enumerate() {
                let off = 0x800 + i * 4;
                exe[off..off + 4].copy_from_slice(&w.to_le_bytes());
            }
            exe
        };

        // 1. Direct bare-metal sideload_exe with explicit non-zero SP
        let code = [
            0x3408_00AA, // ORI $t0 ($8), $zero, 0x00AA
            0x0380_4821, // ADDU $t1 ($9), $gp ($28), $zero
        ];
        let exe_with_sp = build_test_exe(0x8001_0000, 0x8001_8000, 0x8001_0000, 0x801F_F000, 0x0100, &code);
        let mut bus = Bus::new();
        let mut cpu = Cpu::new();
        cpu.sideload_exe(&mut bus, &exe_with_sp).expect("sideload_exe should succeed");
        assert_eq!(cpu.pc, 0x8001_0000);
        assert_eq!(cpu.next_pc, 0x8001_0004);
        assert_eq!(cpu.gpr[28], 0x8001_8000);
        assert_eq!(cpu.gpr[29], 0x801F_F100);
        assert_eq!(cpu.gpr[30], 0x801F_F100);
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.gpr[8], 0x00AA);
        assert_eq!(cpu.gpr[9], 0x8001_8000);

        // 2. Direct bare-metal sideload_exe with SP == 0 -> defaults to 0x801F_FFF0 (JaCzekanski ps1-tests rule)
        let exe_zero_sp = build_test_exe(0x8001_2000, 0x8002_0000, 0x8001_2000, 0, 0, &code);
        let mut bus2 = Bus::new();
        let mut cpu2 = Cpu::new();
        cpu2.sideload_exe(&mut bus2, &exe_zero_sp).expect("sideload_exe with SP=0 should succeed");
        assert_eq!(cpu2.pc, 0x8001_2000);
        assert_eq!(cpu2.gpr[28], 0x8002_0000);
        assert_eq!(cpu2.gpr[29], 0x801F_FFF0);
        assert_eq!(cpu2.gpr[30], 0x801F_FFF0);

        // 3. BIOS Shell Hook at 0x8003_0000 with SP == 0 preserving BIOS-initialized R29/R30 (0x801F_FFF0)
        let mut bus3 = Bus::new();
        let mut cpu3 = Cpu::new();
        bus3.queue_exe_bytes(&exe_zero_sp).expect("queue_exe_bytes should succeed");
        cpu3.gpr[29] = 0x801F_FFF0;
        cpu3.gpr[30] = 0x801F_FFF0;
        cpu3.pc = 0x8003_0000;
        cpu3.next_pc = 0x8003_0004;

        // Stepping at 0x8003_0000 intercepts before fetch, loads payload into RAM, jumps to 0x8001_2000, and executes ORI $t0, $0, 0xAA
        cpu3.step(&mut bus3);
        assert!(bus3.pending_exe.is_none());
        assert_eq!(cpu3.current_pc, 0x8001_2000);
        assert_eq!(cpu3.pc, 0x8001_2004);
        assert_eq!(cpu3.gpr[28], 0x8002_0000);
        assert_eq!(cpu3.gpr[29], 0x801F_FFF0);
        assert_eq!(cpu3.gpr[30], 0x801F_FFF0);
        assert_eq!(cpu3.gpr[8], 0x00AA);
    }
}
