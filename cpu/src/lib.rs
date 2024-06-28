use core::bus::{Bus, MemoryAccess};

use mode_and_state::{CpuMode, CpuState};
use psr::ProgramStatusRegister;

mod arm;
mod mode_and_state;
mod psr;
mod thumb;

const SP: usize = 13;
const LR: usize = 14;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Register {
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15,
}

pub struct Cpu {
    general_registers: Vec<Vec<u32>>,
    pc: u32,
    cpsr: ProgramStatusRegister,
    spsrs: Vec<ProgramStatusRegister>,
    bus: Bus,
}

impl MemoryAccess for Cpu {
    fn read_byte(&self, address: u32) -> u8 {
        self.bus.read_byte(address)
    }

    fn write_byte(&mut self, address: u32, value: u8) {
        self.bus.write_byte(address, value)
    }
}

impl Cpu {
    pub fn new(bus: Bus) -> Self {
        Cpu {
            general_registers: build_general_registers(),
            pc: 0,
            cpsr: ProgramStatusRegister::new(0),
            spsrs: vec![ProgramStatusRegister::new(0); 5],
            bus,
        }
    }

    fn increment_program_counter(&mut self) {
        match self.cpsr.state() {
            CpuState::ARM => self.pc.wrapping_add(4),
            CpuState::Thumb => self.pc.wrapping_add(2),
        };
    }

    pub fn cycle(&mut self) {
        self.fetch();
        self.decode();
        self.execute();
    }

    pub fn fetch(&mut self) {}
    pub fn decode(&mut self) {}
    pub fn execute(&mut self) {}
}

fn build_general_registers() -> Vec<Vec<u32>> {
    let mut general_registers = Vec::new();
    for i in 0..16 {
        match i {
            0..=7 => general_registers.push(vec![0; 1]),
            8..=12 => general_registers.push(vec![0; 2]),
            13 | 14 => general_registers.push(vec![0; 6]),
            _ => general_registers.push(vec![0; 1]),
        }
    }
    general_registers
}
