use ArmInstructionKind::*;
use bitvec::{field::BitField, order::Lsb0, vec::BitVec, view::BitView};
use core::fmt;
use disassembler::*;
use execute::*;

use crate::{
    Condition, CpuAction, DataProcessingOpcode,
    barrel_shifter::{ShiftBy, ShiftType},
    cpu::Arm7tdmiCpu,
    memory::MemoryInterface,
};

use super::{Register, cpu::Instruction};

pub mod disassembler;
pub mod execute;
pub mod lut;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmInstructionKind {
    DataProcessing,
    PsrTransfer,
    Multiply,
    MultiplyLong,
    SingleDataSwap,
    BranchAndExchange,
    HalfwordAndSignedDataTransfer,
    SingleDataTransfer,
    Undefined,
    BlockDataTransfer,
    BranchAndBranchWithLink,
    SoftwareInterrupt,
    //CoprocessorDataTransfer,
    //CoprocessorDataOperation,
    //CoprocessorRegisterTransfer,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArmInstruction {
    kind: ArmInstructionKind,
    bits: BitVec<u32>,
    executed_pc: u32,
}

impl fmt::Display for ArmInstruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ArmInstruction: kind: {:?}, bits: {} -> (0x{:08X}), executed_pc:{} -> (0x{:08X})",
            self.kind,
            self.bits.load::<u32>(),
            self.bits.load::<u32>(),
            self.executed_pc,
            self.executed_pc
        )
    }
}

impl Instruction for ArmInstruction {
    type Size = u32;

    fn disassemble<I: MemoryInterface>(&self, cpu: &mut Arm7tdmiCpu<I>) -> String {
        match self.kind {
            BranchAndExchange => disassemble_branch_exchange(self),
            BranchAndBranchWithLink => disassemble_branch_and_branch_link(self),
            DataProcessing => disassemble_data_processing(self),
            PsrTransfer => disassemble_psr_transfer(cpu, self),
            Multiply => disassemble_multiply(self),
            MultiplyLong => disassemble_multiply_long(self),
            SingleDataTransfer => disassemble_single_data_transfer(self),
            HalfwordAndSignedDataTransfer => disassemble_halfword_and_signed_data_transfer(self),
            BlockDataTransfer => disassemble_block_data_transfer(self),
            SingleDataSwap => disassemble_single_data_swap(self),
            SoftwareInterrupt => disassemble_software_interrupt(self),
            Undefined => disassemble_undefined(self),
        }
    }

    fn execute<I: MemoryInterface>(&self, cpu: &mut Arm7tdmiCpu<I>) -> CpuAction {
        match self.kind {
            BranchAndExchange => execute_branch_exchange(cpu, self),
            BranchAndBranchWithLink => execute_branch_and_branch_link(cpu, self),
            DataProcessing => execute_data_processing(cpu, self),
            PsrTransfer => execute_psr_transfer(cpu, self),
            Multiply => execute_multiply(cpu, self),
            MultiplyLong => execute_multiply_long(cpu, self),
            SingleDataTransfer => execute_single_data_transfer(cpu, self),
            HalfwordAndSignedDataTransfer => execute_halfword_and_signed_data_transfer(cpu, self),
            BlockDataTransfer => execute_block_data_transfer(cpu, self),
            SingleDataSwap => execute_single_data_swap(cpu, self),
            SoftwareInterrupt => execute_software_interrupt(cpu, self),
            Undefined => execute_undefined(cpu, self),
        }
    }

    fn value(&self) -> u32 {
        self.bits.load::<u32>()
    }
}

impl ArmInstruction {
    pub fn new(kind: ArmInstructionKind, instruction: u32, executed_pc: u32) -> ArmInstruction {
        ArmInstruction {
            kind,
            bits: instruction.view_bits::<Lsb0>().to_bitvec(),
            executed_pc,
        }
    }

    pub fn cond(&self) -> Condition {
        self.bits[28..=31].load::<u32>().into()
    }

    pub fn rn(&self) -> Register {
        match self.kind {
            BranchAndExchange => self.bits[0..=3].load::<u32>().into(),
            DataProcessing | SingleDataTransfer | HalfwordAndSignedDataTransfer | BlockDataTransfer | SingleDataSwap => {
                self.bits[16..=19].load::<u32>().into()
            }
            Multiply => self.bits[12..=15].load::<u32>().into(),
            _ => unimplemented!(),
        }
    }

    pub fn rd(&self) -> Register {
        match self.kind {
            PsrTransfer | DataProcessing | SingleDataTransfer | HalfwordAndSignedDataTransfer | SingleDataSwap => {
                self.bits[12..=15].load::<u32>().into()
            }
            Multiply => self.bits[16..=19].load::<u32>().into(),
            _ => unimplemented!(),
        }
    }

    pub fn rd_hi(&self) -> Register {
        self.bits[16..=19].load::<u32>().into()
    }

    pub fn rd_lo(&self) -> Register {
        self.bits[12..=15].load::<u32>().into()
    }

    pub fn rm(&self) -> Register {
        match self.kind {
            PsrTransfer
            | DataProcessing
            | Multiply
            | MultiplyLong
            | SingleDataTransfer
            | HalfwordAndSignedDataTransfer
            | SingleDataSwap => self.bits[0..=3].load::<u32>().into(),
            _ => unimplemented!(),
        }
    }

    pub fn rs(&self) -> Register {
        match self.kind {
            DataProcessing | Multiply | MultiplyLong | SingleDataTransfer => self.bits[8..=11].load::<u32>().into(),
            _ => unimplemented!(),
        }
    }

    pub fn link(&self) -> bool {
        self.bits[24]
    }

    pub fn offset(&self) -> u32 {
        match self.kind {
            BranchAndBranchWithLink => self.bits[0..=23].load(),
            _ => unimplemented!(),
        }
    }

    pub fn is_immediate(&self) -> bool {
        match self.kind {
            PsrTransfer | DataProcessing => self.bits[25],
            SingleDataTransfer => !self.bits[25],
            HalfwordAndSignedDataTransfer => self.bits[22],
            _ => unimplemented!(),
        }
    }

    pub fn opcode(&self) -> DataProcessingOpcode {
        self.bits[21..=24].load::<u32>().into()
    }

    pub fn sets_flags(&self) -> bool {
        match self.kind {
            DataProcessing | Multiply | MultiplyLong => self.bits[20],
            _ => unimplemented!(),
        }
    }

    pub fn shift_by(&self) -> ShiftBy {
        match self.bits[4] {
            true => ShiftBy::Register,
            false => ShiftBy::Immediate,
        }
    }

    pub fn shift_amount(&self) -> u32 {
        self.bits[7..=11].load()
    }

    pub fn shift_type(&self) -> ShiftType {
        self.bits[5..=6].load::<u32>().into()
    }

    pub fn rotate(&self) -> u32 {
        match self.kind {
            PsrTransfer | DataProcessing => self.bits[8..=11].load(),
            _ => unimplemented!(),
        }
    }

    pub fn immediate(&self) -> u32 {
        match self.kind {
            PsrTransfer | DataProcessing => self.bits[0..=7].load(),
            SingleDataTransfer => self.bits[0..=11].load(),
            _ => unimplemented!(),
        }
    }

    pub fn immediate_hi(&self) -> u32 {
        self.bits[8..=11].load()
    }

    pub fn immediate_lo(&self) -> u32 {
        self.bits[0..=3].load()
    }

    pub fn is_spsr(&self) -> bool {
        self.bits[22]
    }

    pub fn accumulate(&self) -> bool {
        match self.kind {
            Multiply | MultiplyLong => self.bits[21],
            _ => unimplemented!(),
        }
    }

    pub fn unsigned(&self) -> bool {
        self.bits[22]
    }

    pub fn pre_index(&self) -> bool {
        match self.kind {
            SingleDataTransfer | HalfwordAndSignedDataTransfer | BlockDataTransfer => self.bits[24],
            _ => unimplemented!(),
        }
    }

    pub fn add(&self) -> bool {
        match self.kind {
            SingleDataTransfer | HalfwordAndSignedDataTransfer | BlockDataTransfer => self.bits[23],
            _ => unimplemented!(),
        }
    }

    pub fn byte(&self) -> bool {
        match self.kind {
            SingleDataTransfer | SingleDataSwap => self.bits[22],
            _ => unimplemented!(),
        }
    }

    pub fn write_back(&self) -> bool {
        match self.kind {
            SingleDataTransfer | HalfwordAndSignedDataTransfer | BlockDataTransfer => self.bits[21],
            _ => unimplemented!(),
        }
    }

    pub fn load(&self) -> bool {
        match self.kind {
            SingleDataTransfer | HalfwordAndSignedDataTransfer | BlockDataTransfer => self.bits[20],
            _ => unimplemented!(),
        }
    }

    pub fn signed(&self) -> bool {
        self.bits[6]
    }

    pub fn halfword(&self) -> bool {
        self.bits[5]
    }

    pub fn load_psr_force_user(&self) -> bool {
        self.bits[22]
    }

    pub fn register_list(&self) -> Vec<usize> {
        self.bits[0..=15]
            .iter()
            .enumerate()
            .filter_map(|(i, b)| match b.as_ref() {
                true => Some(i),
                false => None,
            })
            .collect()
    }

    pub fn comment(&self) -> u32 {
        self.bits[0..=23].load()
    }
}
