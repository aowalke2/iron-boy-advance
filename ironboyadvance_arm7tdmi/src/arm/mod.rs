use bitvec::{field::BitField, order::Lsb0, vec::BitVec, view::BitView};
use disassembler::branch::{disassemble_branch_and_branch_with_link, disassemble_branch_and_exchange};
use execute::branch::{execute_branch_and_branch_with_link, execute_branch_and_exchange};

use crate::{CpuAction, DataProcessingInstructionKind, cpu::Arm7tdmiCpu, memory::MemoryInterface};

use super::{Condition, Register, cpu::Instruction};

pub mod disassembler;
pub mod execute;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmInstructionKind {
    BranchAndExchange,
    BlockDataTransfer,
    BranchAndBranchWithLink,
    SoftwareInterrupt,
    Undefined,
    SingleDataTransfer,
    SingleDataSwap,
    Multiply,
    MultiplyLong,
    HalfwordDataTransferRegister,
    HalfwordDataTransferImmediate,
    TransferPsrToRegister,
    TransferRegisterToPsr,
    DataProcessing,
}

impl From<u32> for ArmInstructionKind {
    fn from(instruction: u32) -> ArmInstructionKind {
        use ArmInstructionKind::*;
        // Decoding order matters
        if instruction & 0x0FFFFFF0 == 0x012FFF10 {
            BranchAndExchange
        } else if instruction & 0x0E000000 == 0x0800_0000 {
            BlockDataTransfer
        } else if instruction & 0x0F000000 == 0x0A000000 || instruction & 0x0F000000 == 0x0B000000 {
            BranchAndBranchWithLink
        } else if instruction & 0x0F000000 == 0x0F000000 {
            SoftwareInterrupt
        } else if instruction & 0x0E000010 == 0x06000010 {
            Undefined
        } else if instruction & 0x0C000000 == 0x04000000 {
            SingleDataTransfer
        } else if instruction & 0x0FB00FF0 == 0x01000090 {
            SingleDataSwap
        } else if instruction & 0x0F8000F0 == 0x00000090 {
            Multiply
        } else if instruction & 0x0F8000F0 == 0x00800090 {
            MultiplyLong
        } else if instruction & 0x0E400F90 == 0x00000090 {
            HalfwordDataTransferRegister
        } else if instruction & 0x0E400090 == 0x00400090 {
            HalfwordDataTransferImmediate
        } else if instruction & 0x0FBF0000 == 0x010F0000 {
            TransferPsrToRegister
        } else if instruction & 0x0DB0F000 == 0x0120F000 {
            TransferRegisterToPsr
        } else if instruction & 0x0C000000 == 0x0000_0000 {
            DataProcessing
        } else {
            unimplemented!("Instruction undefined or unimplemented")
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArmInstruction {
    kind: ArmInstructionKind,
    bits: BitVec<u32>,
    executed_pc: u32,
}

impl Instruction for ArmInstruction {
    type Size = u32;

    fn decode(instruction: u32, executed_pc: u32) -> ArmInstruction {
        ArmInstruction {
            kind: instruction.into(),
            bits: instruction.view_bits::<Lsb0>().to_bitvec(),
            executed_pc,
        }
    }

    fn disassamble(&self) -> String {
        use ArmInstructionKind::*;
        match self.kind {
            BranchAndExchange => disassemble_branch_and_exchange(self),
            BranchAndBranchWithLink => disassemble_branch_and_branch_with_link(self),
            _ => todo!(),
        }
    }

    fn execute<I: MemoryInterface>(&self, cpu: &mut Arm7tdmiCpu<I>) -> CpuAction {
        use ArmInstructionKind::*;
        match self.kind {
            BranchAndExchange => execute_branch_and_exchange(cpu, self),
            BranchAndBranchWithLink => execute_branch_and_branch_with_link(cpu, self),
            _ => todo!(),
        }
    }

    fn value(&self) -> u32 {
        self.bits.load::<u32>()
    }
}

impl ArmInstruction {
    pub fn cond(&self) -> Condition {
        self.bits[28..=31].load::<u32>().into()
    }

    pub fn rn(&self) -> Register {
        use ArmInstructionKind::*;
        match self.kind {
            BranchAndExchange => self.bits[0..=3].load::<u32>().into(),
            DataProcessing => self.bits[16..=19].load::<u32>().into(),
            _ => todo!(),
        }
    }

    pub fn rd(&self) -> Register {
        use ArmInstructionKind::*;
        match self.kind {
            DataProcessing => self.bits[12..=15].load::<u32>().into(),
            _ => todo!(),
        }
    }

    pub fn rm(&self) -> Register {
        use ArmInstructionKind::*;
        match self.kind {
            DataProcessing => self.bits[0..=3].load::<u32>().into(),
            _ => todo!(),
        }
    }

    pub fn link(&self) -> bool {
        self.bits[24]
    }

    pub fn offset(&self) -> i32 {
        use ArmInstructionKind::*;
        match self.kind {
            BranchAndBranchWithLink => ((self.bits[0..=23].load::<u32>() << 8) as i32) >> 6,
            _ => todo!(),
        }
    }

    pub fn is_immediate_operand(&self) -> bool {
        self.bits[25]
    }

    pub fn opcode(&self) -> DataProcessingInstructionKind {
        self.bits[21..=24].load::<u32>().into()
    }

    pub fn sets_condition(&self) -> bool {
        self.bits[20]
    }

    pub fn shift(&self) -> u32 {
        self.bits[4..=11].load::<u32>()
    }

    pub fn rotate(&self) -> u32 {
        self.bits[8..=11].load::<u32>()
    }

    pub fn immediate(&self) -> u32 {
        self.bits[0..=7].load::<u32>()
    }
}
