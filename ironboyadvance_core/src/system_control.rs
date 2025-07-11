use bitfields::bitfield;

#[bitfield(u16)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct WaitStateControl {
    #[bits(2)]
    sram_wait_control: u8,
    #[bits(2)]
    ws0_first_access: u8,
    #[bits(1)]
    ws0_second_access: u8,
    #[bits(2)]
    ws1_first_access: u8,
    #[bits(1)]
    ws1_second_access: u8,
    #[bits(2)]
    ws2_first_access: u8,
    #[bits(1)]
    ws2_second_access: u8,
    #[bits(2)]
    phi_terminal_output: u8,
    _reserved: bool,
    game_pak_prefetch_buffer_enable: bool,
    game_pak_type_flag: bool,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum HaltMode {
    Running,
    Halted,
    Stopped,
}

pub struct SystemControl {
    waitstate_control: WaitStateControl,
    halt_mode: HaltMode,
}

impl SystemControl {
    pub fn new() -> Self {
        SystemControl {
            waitstate_control: WaitStateControl::from_bits(0),
            halt_mode: HaltMode::Running,
        }
    }

    pub fn set_waitstate_control(&mut self, value: u16) {
        self.waitstate_control.set_bits(value)
    }

    pub fn waitstate_control(&self) -> WaitStateControl {
        self.waitstate_control
    }

    pub fn set_halt_mode(&mut self, halt_mode: HaltMode) {
        self.halt_mode = halt_mode
    }

    pub fn halt_mode(&self) -> HaltMode {
        self.halt_mode
    }
}
