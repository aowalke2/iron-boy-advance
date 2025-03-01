use crate::{
    arm7tdmi::{Condition, CpuAction},
    memory::{MemoryAccess, MemoryInterface},
};

use super::{arm::ArmInstruction, psr::ProgramStatusRegister, CpuMode, CpuState};

const SP: usize = 13;
const LR: usize = 14;
const PC: usize = 15;

pub trait Instruction {
    type Size;
    fn decode(value: Self::Size, address: u32) -> Self;
    fn disassamble(&self) -> String;
    fn value(&self) -> Self::Size;
}

pub struct Arm7tdmiCpu<I: MemoryInterface> {
    general_registers: [u32; 16],
    general_registers_fiq: [u32; 7], //r8 to r12
    general_registers_svc: [u32; 2], //r13 to r14
    general_registers_abt: [u32; 2], //r13 to r14
    general_registers_irq: [u32; 2], //r13 to r14
    general_registers_und: [u32; 2], //r13 to r14
    cpsr: ProgramStatusRegister,
    spsrs: [ProgramStatusRegister; 6],
    instruction_pipeline: [u32; 2],
    bus: I, // May need to make this shared
    next_memory_access: MemoryAccess,
}

impl<I: MemoryInterface> MemoryInterface for Arm7tdmiCpu<I> {
    fn load_8(&mut self, address: u32, access: MemoryAccess) -> u8 {
        self.bus.load_8(address, access)
    }

    fn load_16(&mut self, address: u32, access: MemoryAccess) -> u16 {
        self.bus.load_16(address, access)
    }

    fn load_32(&mut self, address: u32, access: MemoryAccess) -> u32 {
        self.bus.load_32(address, access)
    }

    fn store_8(&mut self, address: u32, value: u8, access: MemoryAccess) {
        self.bus.store_8(address, value, access);
    }

    fn store_16(&mut self, address: u32, value: u16, access: MemoryAccess) {
        self.bus.store_16(address, value, access);
    }

    fn store_32(&mut self, address: u32, value: u32, access: MemoryAccess) {
        self.bus.store_32(address, value, access);
    }
}

impl<I: MemoryInterface> Arm7tdmiCpu<I> {
    pub fn new(bus: I, skip_bios: bool) -> Self {
        let mut cpu = Arm7tdmiCpu {
            general_registers: [0; 16],
            general_registers_fiq: [0; 7], //r8 to r12
            general_registers_svc: [0; 2], //r13 to r14
            general_registers_abt: [0; 2], //r13 to r14
            general_registers_irq: [0; 2], //r13 to r14
            general_registers_und: [0; 2], //r13 to r14
            cpsr: ProgramStatusRegister::from_bits(0x13),
            spsrs: [ProgramStatusRegister::from_bits(0x13); 6],
            instruction_pipeline: [0; 2],
            bus,
            next_memory_access: MemoryAccess::NonSequential,
        };

        match skip_bios {
            true => {
                cpu.general_registers[SP] = 0x03007F00;
                cpu.general_registers[LR] = 0x08000000;
                cpu.general_registers[PC] = 0x08000000;
                cpu.general_registers_svc[0] = 0x3007FE0;
                cpu.general_registers_irq[0] = 0x3007FA0;
                cpu.cpsr.set_cpu_mode(CpuMode::System);
                cpu.cpsr.set_irq_disable(false);
            }
            false => {
                cpu.cpsr.set_cpu_mode(CpuMode::Supervisor);
                cpu.cpsr.set_irq_disable(true);
            }
        }

        cpu.refill_pipeline();
        cpu
    }

    pub fn cycle(&mut self) {
        match self.cpsr.cpu_state() {
            CpuState::Arm => {
                let pc = self.general_registers[PC] & !0x3;
                let instruction = self.instruction_pipeline[0];
                self.instruction_pipeline[0] = self.instruction_pipeline[1];
                self.instruction_pipeline[1] = self.load_32(pc, self.next_memory_access);
                let instruction = ArmInstruction::decode(instruction, pc);
                //TODO log this
                println!("{:?}", instruction);
                println!("{}", instruction.disassamble());

                let condtion = instruction.cond();
                if condtion != Condition::AL && !self.is_condition_met(condtion) {
                    self.advance_pc_arm();
                    self.next_memory_access = MemoryAccess::NonSequential;
                    return;
                }

                match self.arm_execute(instruction) {
                    CpuAction::Advance(memory_access) => {
                        self.advance_pc_arm();
                        self.next_memory_access = memory_access;
                    }
                    CpuAction::PipelineFlush => {}
                };
            }
            CpuState::Thumb => {
                let pc = self.general_registers[PC] & !0x1;
                self.advance_pc_thumb();
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

    pub fn set_cpu_state(&mut self, state: CpuState) {
        self.cpsr.set_cpu_state(state);
    }

    pub fn set_pc(&mut self, value: u32) {
        self.general_registers[PC] = value
    }

    pub fn advance_pc_thumb(&mut self) {
        self.general_registers[PC] = self.general_registers[PC].wrapping_add(2);
    }

    pub fn advance_pc_arm(&mut self) {
        self.general_registers[PC] = self.general_registers[PC].wrapping_add(4);
    }

    pub fn refill_pipeline(&mut self) {
        match self.cpsr.cpu_state() {
            CpuState::Arm => {
                self.instruction_pipeline[0] = self.load_32(self.general_registers[PC], MemoryAccess::NonSequential);
                self.advance_pc_arm();
                self.instruction_pipeline[1] = self.load_32(self.general_registers[PC], MemoryAccess::Sequential);
                self.advance_pc_arm();
                self.next_memory_access = MemoryAccess::Sequential;
            }
            CpuState::Thumb => {
                self.instruction_pipeline[0] = self.load_16(self.general_registers[PC], MemoryAccess::NonSequential) as u32;
                self.advance_pc_thumb();
                self.instruction_pipeline[1] = self.load_16(self.general_registers[PC], MemoryAccess::Sequential) as u32;
                self.advance_pc_thumb();
                self.next_memory_access = MemoryAccess::Sequential;
            }
        }
    }

    pub fn get_general_register(&self, index: usize) -> u32 {
        self.general_registers[index]
    }
}
