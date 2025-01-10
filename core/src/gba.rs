use std::{cell::RefCell, rc::Rc};

use crate::{
    arm7tdmi::cpu::Arm7tdmiCpu, bios::Bios, cartridge::Cartridge, memory::system_bus::SystemBus,
    scheduler::Scheduler, sharp_sm83::cpu::SharpSm83Cpu, GbaError,
};

pub struct GameBoyAdvance {
    arm7tdmi: Arm7tdmiCpu<SystemBus>,
    // may end making a common cpu trait
    // sharp_sm83: SharpSm83Cpu<SystemBus>,
    scheduler: Rc<RefCell<Scheduler>>,
}

impl GameBoyAdvance {
    pub fn new(
        rom_buffer: Vec<u8>,
        bios_buffer: Vec<u8>,
        show_logs: bool,
    ) -> Result<GameBoyAdvance, GbaError> {
        let scheduler = Rc::new(RefCell::new(Scheduler::new()));
        let cartridge = Cartridge::load(rom_buffer)?;
        let gba = GameBoyAdvance {
            arm7tdmi: Arm7tdmiCpu::new(SystemBus::new(
                cartridge,
                Bios::load(bios_buffer),
                scheduler.clone(),
            )),
            scheduler,
        };
        Ok(gba)
    }
}
