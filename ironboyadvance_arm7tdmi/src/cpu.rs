use ironboyadvance_utils::get_set;

use crate::{
    CpuAction,
    arm::{ArmInstructionKind, Condition, lut::generate_arm_lut},
    memory::{MemoryAccess, MemoryInterface},
};

use super::{CpuMode, CpuState, arm::ArmInstruction, psr::ProgramStatusRegister};

pub const SP: usize = 13;
pub const LR: usize = 14;
pub const PC: usize = 15;

pub const SYS_BANK: usize = 0;
pub const USR_BANK: usize = SYS_BANK;
pub const FIQ_BANK: usize = 1;
pub const SVC_BANK: usize = 2;
pub const ABT_BANK: usize = 3;
pub const IRQ_BANK: usize = 4;
pub const UND_BANK: usize = 5;

pub trait Instruction {
    type Size;
    fn execute<I: MemoryInterface>(&self, cpu: &mut Arm7tdmiCpu<I>) -> CpuAction;
    fn disassamble<I: MemoryInterface>(&self, cpu: &mut Arm7tdmiCpu<I>) -> String;
    fn value(&self) -> Self::Size;
}

pub struct Arm7tdmiCpu<I: MemoryInterface> {
    general_registers: [u32; 16],
    banked_registers: [[u32; 7]; 6], //r8 to r14
    banked_spsrs: [ProgramStatusRegister; 6],
    cpsr: ProgramStatusRegister,
    spsr: ProgramStatusRegister,
    pipeline: [u32; 2],
    bus: I, // May need to make this shared
    next_memory_access: u8,
    arm_lut: [ArmInstructionKind; 4096],
}

impl<I: MemoryInterface> MemoryInterface for Arm7tdmiCpu<I> {
    fn load_8(&mut self, address: u32, access: u8) -> u32 {
        self.bus.load_8(address, access)
    }

    fn load_16(&mut self, address: u32, access: u8) -> u32 {
        self.bus.load_16(address, access)
    }

    fn load_32(&mut self, address: u32, access: u8) -> u32 {
        self.bus.load_32(address, access)
    }

    fn store_8(&mut self, address: u32, value: u8, access: u8) {
        self.bus.store_8(address, value, access);
    }

    fn store_16(&mut self, address: u32, value: u16, access: u8) {
        self.bus.store_16(address, value, access);
    }

    fn store_32(&mut self, address: u32, value: u32, access: u8) {
        self.bus.store_32(address, value, access);
    }

    fn idle_cycle(&mut self) {
        self.bus.idle_cycle();
    }
}

impl<I: MemoryInterface> Arm7tdmiCpu<I> {
    pub fn new(bus: I, skip_bios: bool) -> Self {
        let mut cpu = Arm7tdmiCpu {
            general_registers: [0; 16],
            banked_registers: [[0; 7]; 6], //r8 to r14
            banked_spsrs: [ProgramStatusRegister::from_bits(0x13); 6],
            cpsr: ProgramStatusRegister::from_bits(0x13),
            spsr: ProgramStatusRegister::from_bits(0x13),
            pipeline: [0; 2],
            bus,
            next_memory_access: MemoryAccess::Instruction | MemoryAccess::Nonsequential,
            arm_lut: [ArmInstructionKind::Undefined; 4096],
        };

        cpu.arm_lut = generate_arm_lut();

        match skip_bios {
            true => {
                cpu.general_registers[SP] = 0x03007F00;
                cpu.general_registers[LR] = 0x08000000;
                cpu.general_registers[PC] = 0x08000000;
                cpu.banked_registers[SVC_BANK][5] = 0x3007FE0;
                cpu.banked_registers[IRQ_BANK][5] = 0x3007FA0;
                cpu.cpsr.set_mode(CpuMode::System);
                cpu.cpsr.set_irq_disable(false);
            }
            false => {
                cpu.cpsr.set_mode(CpuMode::Supervisor);
                cpu.cpsr.set_irq_disable(true);
            }
        }

        //TODO: not sure if i need this forever
        //cpu.refill_pipeline();
        cpu
    }

    get_set!(general_registers, set_general_registers, [u32; 16]);
    get_set!(banked_registers, set_banked_registers, [[u32; 7]; 6]);
    get_set!(banked_spsrs, set_banked_spsrs, [ProgramStatusRegister; 6]);
    get_set!(cpsr, set_cpsr, ProgramStatusRegister);
    get_set!(spsr, set_spsr, ProgramStatusRegister);
    get_set!(pipeline, set_pipeline, [u32; 2]);

    pub fn cycle(&mut self) {
        let pc = self.general_registers[PC] & !0x1;

        match self.cpsr.state() {
            CpuState::Arm => {
                let instruction = self.pipeline[0];
                self.pipeline[0] = self.pipeline[1];
                self.pipeline[1] = self.load_32(pc, self.next_memory_access);
                let lut_index = ((instruction >> 16) & 0x0FF0) | ((instruction >> 4) & 0x000F);
                let instruction = ArmInstruction::new(self.arm_lut[lut_index as usize], instruction, pc - 8);

                println!("{}", instruction);
                println!("{}", instruction.disassamble(self));

                let condition = instruction.cond();
                if condition != Condition::AL && !self.is_condition_met(condition) {
                    self.advance_pc_arm();
                    self.next_memory_access = MemoryAccess::Instruction | MemoryAccess::Sequential;
                    return;
                }
                match instruction.execute(self) {
                    CpuAction::Advance(memory_access) => {
                        self.advance_pc_arm();
                        self.next_memory_access = memory_access;
                    }
                    CpuAction::PipelineFlush => {}
                };
            }
            CpuState::Thumb => {
                let instruction = self.pipeline[0];
                self.pipeline[0] = self.pipeline[1];
                self.pipeline[1] = self.load_32(pc, self.next_memory_access);
                self.advance_pc_thumb();

                // TODO
            }
        }
    }

    fn is_condition_met(&self, condition: Condition) -> bool {
        use Condition::*;
        match condition {
            EQ => self.cpsr.zero(),
            NE => !self.cpsr.zero(),
            CS => self.cpsr.carry(),
            CC => !self.cpsr.carry(),
            MI => self.cpsr.negative(),
            PL => !self.cpsr.negative(),
            VS => self.cpsr.overflow(),
            VC => !self.cpsr.overflow(),
            HI => self.cpsr.carry() && !self.cpsr.zero(),
            LS => !self.cpsr.carry() || self.cpsr.zero(),
            GE => self.cpsr.negative() == self.cpsr.overflow(),
            LT => self.cpsr.negative() != self.cpsr.overflow(),
            GT => !self.cpsr.zero() && (self.cpsr.negative() == self.cpsr.overflow()),
            LE => self.cpsr.zero() || (self.cpsr.negative() != self.cpsr.overflow()),
            AL => true,
        }
    }

    pub fn set_negative(&mut self, status: bool) {
        self.cpsr.set_negative(status);
    }

    pub fn set_zero(&mut self, status: bool) {
        self.cpsr.set_zero(status);
    }

    pub fn set_carry(&mut self, status: bool) {
        self.cpsr.set_carry(status);
    }

    pub fn set_overflow(&mut self, status: bool) {
        self.cpsr.set_overflow(status);
    }

    pub fn set_flags(&mut self, value: u8) {
        self.cpsr.set_flags(value);
    }

    pub fn set_state(&mut self, state: CpuState) {
        self.cpsr.set_state(state);
    }

    pub fn pc(&self) -> u32 {
        self.general_registers[PC]
    }

    pub fn set_pc(&mut self, value: u32) {
        self.general_registers[PC] = value;
    }

    pub fn advance_pc_thumb(&mut self) {
        self.general_registers[PC] = self.general_registers[PC].wrapping_add(2);
    }

    pub fn advance_pc_arm(&mut self) {
        self.general_registers[PC] = self.general_registers[PC].wrapping_add(4);
    }

    pub fn pipeline_flush(&mut self) {
        match self.cpsr.state() {
            CpuState::Arm => {
                self.pipeline[0] = self.load_32(
                    self.general_registers[PC],
                    MemoryAccess::Instruction | MemoryAccess::Nonsequential,
                );
                self.advance_pc_arm();
                self.pipeline[1] = self.load_32(
                    self.general_registers[PC],
                    MemoryAccess::Instruction | MemoryAccess::Sequential,
                );
                self.advance_pc_arm();
                self.next_memory_access = MemoryAccess::Instruction | MemoryAccess::Sequential;
            }
            CpuState::Thumb => {
                self.pipeline[0] = self.load_16(
                    self.general_registers[PC],
                    MemoryAccess::Instruction | MemoryAccess::Sequential,
                );
                self.advance_pc_thumb();
                self.pipeline[1] = self.load_16(
                    self.general_registers[PC],
                    MemoryAccess::Instruction | MemoryAccess::Sequential,
                );
                self.advance_pc_thumb();
                self.next_memory_access = MemoryAccess::Instruction | MemoryAccess::Sequential;
            }
        }
    }

    pub fn register(&self, index: usize) -> u32 {
        match index {
            0..=15 => self.general_registers[index],
            _ => panic!("Index out of range"),
        }
    }

    pub fn set_register(&mut self, index: usize, value: u32) {
        match index {
            0..=15 => self.general_registers[index] = value,
            _ => panic!("Index out of range"),
        }
    }

    pub fn banked_spsr(&self) -> ProgramStatusRegister {
        match self.cpsr.mode() {
            CpuMode::User | CpuMode::System => self.banked_spsrs[0],
            CpuMode::Fiq => self.banked_spsrs[1],
            CpuMode::Supervisor => self.banked_spsrs[2],
            CpuMode::Abort => self.banked_spsrs[3],
            CpuMode::Irq => self.banked_spsrs[4],
            CpuMode::Undefined => self.banked_spsrs[5],
            CpuMode::Invalid => panic!("invalid mode"),
        }
    }

    pub fn set_banked_spsr(&mut self, spsr: ProgramStatusRegister) {
        match self.cpsr.mode() {
            CpuMode::User | CpuMode::System => self.banked_spsrs[0] = spsr,
            CpuMode::Fiq => self.banked_spsrs[1] = spsr,
            CpuMode::Supervisor => self.banked_spsrs[2] = spsr,
            CpuMode::Abort => self.banked_spsrs[3] = spsr,
            CpuMode::Irq => self.banked_spsrs[4] = spsr,
            CpuMode::Undefined => self.banked_spsrs[5] = spsr,
            CpuMode::Invalid => panic!("invalid mode"),
        }
    }

    pub fn set_bus(&mut self, bus: I) {
        self.bus = bus
    }

    pub fn reset(&mut self, skip_bios: bool) {
        self.general_registers = [0; 16];
        self.banked_registers = [[0; 7]; 6]; //r8 to r14
        self.cpsr = ProgramStatusRegister::from_bits(0x13);
        self.spsr = ProgramStatusRegister::from_bits(0x13);
        self.pipeline = [0; 2];
        self.next_memory_access = MemoryAccess::Instruction | MemoryAccess::Nonsequential;

        match skip_bios {
            true => {
                self.general_registers[SP] = 0x03007F00;
                self.general_registers[LR] = 0x08000000;
                self.general_registers[PC] = 0x08000000;
                self.banked_registers[SVC_BANK][0] = 0x3007FE0;
                self.banked_registers[IRQ_BANK][0] = 0x3007FA0;
                self.cpsr.set_mode(CpuMode::System);
                self.cpsr.set_irq_disable(false);
            }
            false => {
                self.cpsr.set_mode(CpuMode::Supervisor);
                self.cpsr.set_irq_disable(true);
            }
        }
    }
}
