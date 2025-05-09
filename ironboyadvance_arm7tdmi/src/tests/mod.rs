//TODO: implement step tests
#[cfg(test)]
use std::fs;

use serde::Deserialize;
use serde_repr::Deserialize_repr;

use crate::cpu::{ABT_BANK, FIQ_BANK, IRQ_BANK, SVC_BANK, UND_BANK, USR_BANK};
#[allow(unused)]
use crate::{CpuMode, cpu::Arm7tdmiCpu, psr::ProgramStatusRegister};

mod test_bus;

#[derive(Debug, Deserialize_repr, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum TransactionKind {
    InstructionRead = 0,
    GeneralRead,
    Write,
}

#[derive(Debug, Deserialize_repr, Clone, Copy)]
#[repr(u8)]
enum Size {
    Byte = 1,
    HalfWord = 2,
    Word = 4,
}

#[derive(Debug)]
#[allow(unused)]
enum Access {
    Nonsequential = 0b0,
    Sequential = 0b1,
    Code = 0b10,
    Dma = 0b100,
    Lock = 0b1000,
}

#[derive(Debug, Deserialize, Clone, Copy)]
#[allow(unused)]
pub struct Transaction {
    kind: TransactionKind,
    size: Size,
    pub addr: u32,
    pub data: u32,
    pub cycle: u8,
    pub access: u8,
}

#[derive(Debug, Deserialize)]
#[allow(non_snake_case, unused)]
struct State {
    #[serde(rename = "R")]
    r: [u32; 16],
    #[serde(rename = "R_fiq")]
    r_fiq: [u32; 7],
    #[serde(rename = "R_svc")]
    r_svc: [u32; 2],
    #[serde(rename = "R_abt")]
    r_abt: [u32; 2],
    #[serde(rename = "R_irq")]
    r_irq: [u32; 2],
    #[serde(rename = "R_und")]
    r_und: [u32; 2],
    #[serde(rename = "CPSR")]
    cpsr: u32,
    #[serde(rename = "SPSR")]
    spsr: [u32; 5],
    pipeline: [u32; 2],
}

#[derive(Debug, Deserialize)]
#[allow(unused)]
struct Test {
    #[serde(rename = "initial")]
    initial_state: State,
    #[serde(rename = "final")]
    final_state: State,
    transactions: Vec<Transaction>,
    opcode: u32,
    base_addr: [u32; 1],
}

#[test]
fn single_step_tests() {
    // Will keep a list of the files I want to run until i complete all the instructions
    // completed
    let files = [
        "arm_b_bl.json", //Done
                         // "arm_bx.json",   //Done
                         // "arm_cdp.json",                       //Skip
                         // "arm_data_proc_immediate.json", //Done
                         // "arm_data_proc_immediate_shift.json", //Done
                         // "arm_data_proc_register_shift.json",  //Done
                         // "arm_ldm_stm.json",
                         // "arm_ldr_str_immediate_offset.json", //Done
                         // "arm_ldr_str_register_offset.json", //Done
                         // "arm_ldrh_strh.json", //Done
                         // "arm_ldrsb_ldrsh.json", //Done
                         // "arm_mcr_mrc.json", //Skip
                         // "arm_mrs.json", //Done
                         // "arm_msr_imm.json", //Done
                         // "arm_msr_reg.json", //Done
                         // "arm_mul_mla.json", //Done
                         // "arm_mull_mlal.json", //Done
                         // "arm_stc_ldc.json", //Skip
                         // "arm_swi.json",
                         // "arm_swp.json",
                         // "thumb_add_cmp_mov_hi.json",
                         // "thumb_add_sp_or_pc.json",
                         // "thumb_add_sub.json",
                         // "thumb_add_sub_sp.json",
                         // "thumb_b.json",
                         // "thumb_bcc.json",
                         // "thumb_bl_blx_prefix.json",
                         // "thumb_bl_suffix.json",
                         // "thumb_bx.json",
                         // "thumb_data_proc.json",
                         // "thumb_ldm_stm.json",
                         // "thumb_ldr_pc_rel.json",
                         // "thumb_ldr_str_imm_offset.json",
                         // "thumb_ldr_str_reg_offset.json",
                         // "thumb_ldr_str_sp_rel.json",
                         // "thumb_ldrb_strb_imm_offset.json",
                         // "thumb_ldrh_strh_imm_offset.json",
                         // "thumb_ldrh_strh_reg_offset.json",
                         // "thumb_ldrsb_strb_reg_offset.json",
                         // "thumb_ldrsh_ldrsb_reg_offset.json",
                         // "thumb_lsl_lsr_asr.json",
                         // "thumb_mov_cmp_add_sub.json",
                         // "thumb_push_pop.json",
                         // "thumb_swi.json",
                         // "thumb_undefined_bcc.json",
    ];

    // let directory = fs::read_dir("../external/arm7tdmi/v1").expect("Unable to access directory");
    // for file in directory {
    //     let file = file.unwrap().path();
    // }

    let skip = ["arm_cdp.json", "arm_mcr_mrc.json", "arm_stc_ldc.json"];
    let multiplication = ["arm_mul_mla.json", "arm_mull_mlal.json"];

    let mut cpu = Arm7tdmiCpu::new(test_bus::TestBus::default(), true);
    for file in files {
        if skip.contains(&file) {
            continue;
        }

        let test_json = fs::read_to_string(format!("../external/arm7tdmi/v1/{file}")).expect("unable to read file");
        let tests: Vec<Test> = serde_json::from_str(&test_json).unwrap();
        for test in tests {
            let test_bus = test_bus::TestBus::new(test.base_addr[0], test.opcode, test.transactions.clone());
            cpu.set_bus(test_bus);

            let intial_state = test.initial_state;
            let final_state = test.final_state;

            cpu.set_general_registers(intial_state.r);

            let mut banked_registers = [[0; 7]; 6];
            for i in 0..7 {
                banked_registers[USR_BANK][i] = intial_state.r[i + 8];
                banked_registers[FIQ_BANK][i] = intial_state.r_fiq[i];
            }

            for i in 0..2 {
                banked_registers[SVC_BANK][i] = intial_state.r_svc[i];
                banked_registers[ABT_BANK][i] = intial_state.r_abt[i];
                banked_registers[IRQ_BANK][i] = intial_state.r_irq[i];
                banked_registers[UND_BANK][i] = intial_state.r_und[i];
            }

            cpu.set_banked_registers(banked_registers);
            cpu.set_cpsr(ProgramStatusRegister::from_bits(intial_state.cpsr));

            let mut banked_spsrs = [intial_state.cpsr; 6];
            for i in 1..6 {
                banked_spsrs[i] = intial_state.spsr[i - 1];
            }

            cpu.set_banked_spsrs(banked_spsrs.map(|x| ProgramStatusRegister::from_bits(x)));
            cpu.set_pipeline(intial_state.pipeline);

            cpu.cycle();

            assert_eq!(cpu.general_registers(), final_state.r);
            assert_eq!(cpu.banked_registers()[FIQ_BANK], final_state.r_fiq);
            assert_eq!(cpu.banked_registers()[SVC_BANK][5..], final_state.r_svc);
            assert_eq!(cpu.banked_registers()[ABT_BANK][5..], final_state.r_abt);
            assert_eq!(cpu.banked_registers()[IRQ_BANK][5..], final_state.r_irq);
            assert_eq!(cpu.banked_registers()[UND_BANK][5..], final_state.r_und);

            assert_eq!(
                cpu.banked_spsrs()[1..].iter().map(|x| x.into_bits()).collect::<Vec<u32>>(),
                final_state.spsr.map(|x| ProgramStatusRegister::from_bits(x).into_bits())
            );

            let expected = ProgramStatusRegister::from_bits(final_state.cpsr);
            let mut actual = cpu.cpsr();

            // the booth multiplication sets the carry. data sheet says its set to a meaningless value. Will ignore result for now
            if multiplication.contains(&file) {
                actual.set_carry(expected.carry());
            }
            assert_eq!(expected.into_bits(), actual.into_bits());

            assert_eq!(cpu.pipeline(), final_state.pipeline);

            cpu.reset(true)
        }
    }
}
