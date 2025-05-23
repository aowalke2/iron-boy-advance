use bitfields::bitfield;

use super::{CpuMode, CpuState};

#[bitfield(u32)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ProgramStatusRegister {
    #[bits(5)]
    mode: CpuMode,
    #[bits(1)]
    state: CpuState,
    fiq_disable: bool,
    irq_disable: bool,
    #[bits(20)]
    _reserved: u32,
    overflow: bool,
    carry: bool,
    zero: bool,
    negative: bool,
}

impl ProgramStatusRegister {
    pub fn flags(&self) -> u8 {
        (self.negative() as u8) << 3 | (self.zero() as u8) << 2 | (self.carry() as u8) << 1 | (self.overflow() as u8)
    }

    pub fn set_flags(&mut self, value: u8) {
        self.set_negative(value & 0b1000 != 0);
        self.set_zero(value & 0b0100 != 0);
        self.set_carry((value & 0b0010) != 0);
        self.set_overflow(value & 0b0001 != 0);
    }
}

#[cfg(test)]
mod tests {

    use crate::{CpuMode, CpuState};

    use super::ProgramStatusRegister;

    #[test]
    fn from_bits_psr() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.into_bits(), 0xF00000FF)
    }

    #[test]
    fn set_psr_flags() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFF11);
        psr.set_flags(0xE);
        assert_eq!(psr.into_bits(), 0xE0000011);
        assert_eq!(psr.flags(), 0xE);

        psr.set_flags(0x0);
        assert_eq!(psr.into_bits(), 0x00000011);
        assert_eq!(psr.flags(), 0x0);
    }

    #[test]
    fn get_psr_negative() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.negative(), true)
    }

    #[test]
    fn set_psr_negative() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_negative(false);
        assert_eq!(psr.negative(), false)
    }

    #[test]
    fn get_psr_zero() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.zero(), true)
    }

    #[test]
    fn set_psr_zero() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_zero(false);
        assert_eq!(psr.zero(), false)
    }

    #[test]
    fn get_psr_carry() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.carry(), true)
    }

    #[test]
    fn set_psr_carry() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_carry(false);
        assert_eq!(psr.carry(), false)
    }

    #[test]
    fn get_psr_overflow() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.overflow(), true)
    }

    #[test]
    fn set_psr_overflow() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_overflow(false);
        assert_eq!(psr.overflow(), false)
    }

    #[test]
    fn get_psr_irq_disable() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.irq_disable(), true)
    }

    #[test]
    fn set_psr_irq_disable() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_irq_disable(false);
        assert_eq!(psr.irq_disable(), false)
    }

    #[test]
    fn get_psr_fiq_disable() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.fiq_disable(), true)
    }

    #[test]
    fn set_psr_fiq_disable() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_fiq_disable(false);
        assert_eq!(psr.fiq_disable(), false)
    }

    #[test]
    fn get_psr_state() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.state(), CpuState::Thumb)
    }

    #[test]
    fn set_psr_state() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_state(CpuState::Arm);
        assert_eq!(psr.state(), CpuState::Arm)
    }

    #[test]
    fn get_psr_mode() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        assert_eq!(psr.mode(), CpuMode::System)
    }

    #[test]
    fn set_psr_mode() {
        let mut psr = ProgramStatusRegister::from_bits(0xFFFFFFFF);
        psr.set_mode(CpuMode::User);
        assert_eq!(psr.mode(), CpuMode::User);

        psr.set_mode(CpuMode::Fiq);
        assert_eq!(psr.mode(), CpuMode::Fiq);

        psr.set_mode(CpuMode::Irq);
        assert_eq!(psr.mode(), CpuMode::Irq);

        psr.set_mode(CpuMode::Supervisor);
        assert_eq!(psr.mode(), CpuMode::Supervisor);

        psr.set_mode(CpuMode::Abort);
        assert_eq!(psr.mode(), CpuMode::Abort);

        psr.set_mode(CpuMode::Undefined);
        assert_eq!(psr.mode(), CpuMode::Undefined);
    }

    #[test]
    #[should_panic]
    fn get_psr_mode_panics() {
        let psr = ProgramStatusRegister::from_bits(0xFFFFFF15);
        psr.mode();
    }
}
