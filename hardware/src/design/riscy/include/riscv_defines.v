
//package riscv_defines,

////////////////////////////////////////////////
//    ___         ____          _             //
//   / _ \ _ __  / ___|___   __| | ___  ___   //
//  | | | | '_ \| |   / _ \ / _` |/ _ \/ __|  //
//  | |_| | |_) | |__| (_) | (_| |  __/\__ \  //
//   \___/| .__/ \____\___/ \__,_|\___||___/  //
//        |_|                                 //
////////////////////////////////////////////////

parameter OPCODE_SYSTEM    = 7'h73,
parameter OPCODE_FENCE     = 7'h0f,
parameter OPCODE_OP        = 7'h33,
parameter OPCODE_OPIMM     = 7'h13,
parameter OPCODE_STORE     = 7'h23,
parameter OPCODE_LOAD      = 7'h03,
parameter OPCODE_BRANCH    = 7'h63,
parameter OPCODE_JALR      = 7'h67,
parameter OPCODE_JAL       = 7'h6f,
parameter OPCODE_AUIPC     = 7'h17,
parameter OPCODE_LUI       = 7'h37,
parameter OPCODE_OP_FP     = 7'h53,
parameter OPCODE_OP_FMADD  = 7'h43,
parameter OPCODE_OP_FNMADD = 7'h4f,
parameter OPCODE_OP_FMSUB  = 7'h47,
parameter OPCODE_OP_FNMSUB = 7'h4b,
parameter OPCODE_STORE_FP  = 7'h27,
parameter OPCODE_LOAD_FP   = 7'h07,

// those opcodes are now used for PULP custom instructions
// parameter OPCODE_CUST0     = 7'h0b
// parameter OPCODE_CUST1     = 7'h2b

// PULP custom
parameter OPCODE_LOAD_POST  = 7'h0b,
parameter OPCODE_STORE_POST = 7'h2b,
parameter OPCODE_PULP_OP    = 7'h5b,
parameter OPCODE_VECOP      = 7'h57,
parameter OPCODE_HWLOOP     = 7'h7b,

parameter REGC_S1   = 2'b10,
parameter REGC_S4   = 2'b00,
parameter REGC_RD   = 2'b01,
parameter REGC_ZERO = 2'b11,


//////////////////////////////////////////////////////////////////////////////
//      _    _    _   _    ___                       _   _                  //
//     / \  | |  | | | |  / _ \ _ __   ___ _ __ __ _| |_(_) ___  _ __  ___  //
//    / _ \ | |  | | | | | | | | '_ \ / _ \ '__/ _` | __| |/ _ \| '_ \/ __| //
//   / ___ \| |__| |_| | | |_| | |_) |  __/ | | (_| | |_| | (_) | | | \__ \ //
//  /_/   \_\_____\___/   \___/| .__/ \___|_|  \__,_|\__|_|\___/|_| |_|___/ //
//                             |_|                                          //
//////////////////////////////////////////////////////////////////////////////

parameter ALU_OP_WIDTH = 7,

parameter ALU_ADD   = 7'b0011000,
parameter ALU_SUB   = 7'b0011001,
parameter ALU_ADDU  = 7'b0011010,
parameter ALU_SUBU  = 7'b0011011,
parameter ALU_ADDR  = 7'b0011100,
parameter ALU_SUBR  = 7'b0011101,
parameter ALU_ADDUR = 7'b0011110,
parameter ALU_SUBUR = 7'b0011111,

parameter ALU_XOR   = 7'b0101111,
parameter ALU_OR    = 7'b0101110,
parameter ALU_AND   = 7'b0010101,

// Shifts
parameter ALU_SRA   = 7'b0100100,
parameter ALU_SRL   = 7'b0100101,
parameter ALU_ROR   = 7'b0100110,
parameter ALU_SLL   = 7'b0100111,

// bit manipulation
parameter ALU_BEXT  = 7'b0101000,
parameter ALU_BEXTU = 7'b0101001,
parameter ALU_BINS  = 7'b0101010,
parameter ALU_BCLR  = 7'b0101011,
parameter ALU_BSET  = 7'b0101100,

// Bit counting
parameter ALU_FF1   = 7'b0110110,
parameter ALU_FL1   = 7'b0110111,
parameter ALU_CNT   = 7'b0110100,
parameter ALU_CLB   = 7'b0110101,

// Sign-/zero-extensions
parameter ALU_EXTS  = 7'b0111110,
parameter ALU_EXT   = 7'b0111111,

// Comparisons
parameter ALU_LTS   = 7'b0000000,
parameter ALU_LTU   = 7'b0000001,
parameter ALU_LES   = 7'b0000100,
parameter ALU_LEU   = 7'b0000101,
parameter ALU_GTS   = 7'b0001000,
parameter ALU_GTU   = 7'b0001001,
parameter ALU_GES   = 7'b0001010,
parameter ALU_GEU   = 7'b0001011,
parameter ALU_EQ    = 7'b0001100,
parameter ALU_NE    = 7'b0001101,

// Set Lower Than operations
parameter ALU_SLTS  = 7'b0000010,
parameter ALU_SLTU  = 7'b0000011,
parameter ALU_SLETS = 7'b0000110,
parameter ALU_SLETU = 7'b0000111,

// Absolute value
parameter ALU_ABS   = 7'b0010100,
parameter ALU_CLIP  = 7'b0010110,
parameter ALU_CLIPU = 7'b0010111,

// Insert/extract
parameter ALU_INS   = 7'b0101101,

// min/max
parameter ALU_MIN   = 7'b0010000,
parameter ALU_MINU  = 7'b0010001,
parameter ALU_MAX   = 7'b0010010,
parameter ALU_MAXU  = 7'b0010011,

// div/rem
parameter ALU_DIVU  = 7'b0110000, // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_DIV   = 7'b0110001, // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_REMU  = 7'b0110010, // bit 0 is used for signed mode, bit 1 is used for remdiv
parameter ALU_REM   = 7'b0110011, // bit 0 is used for signed mode, bit 1 is used for remdiv

parameter ALU_SHUF  = 7'b0111010,
parameter ALU_SHUF2 = 7'b0111011,
parameter ALU_PCKLO = 7'b0111000,
parameter ALU_PCKHI = 7'b0111001,

// fpu
parameter ALU_FKEEP   = 7'b1111111,   // hack, to support fcvt.s.d
parameter ALU_FSGNJ   = 7'b1000000,
parameter ALU_FSGNJN  = 7'b1000001,
parameter ALU_FSGNJX  = 7'b1000010,
parameter ALU_FEQ     = 7'b1000011,
parameter ALU_FLT     = 7'b1000100,
parameter ALU_FLE     = 7'b1000101,
parameter ALU_FMAX    = 7'b1000110,
parameter ALU_FMIN    = 7'b1000111,
parameter ALU_FCLASS  = 7'b1001000,

parameter MUL_MAC32 = 3'b000,
parameter MUL_MSU32 = 3'b001,
parameter MUL_I     = 3'b010,
parameter MUL_IR    = 3'b011,
parameter MUL_DOT8  = 3'b100,
parameter MUL_DOT16 = 3'b101,
parameter MUL_H     = 3'b110,

// vector modes
parameter VEC_MODE32 = 2'b00,
parameter VEC_MODE16 = 2'b10,
parameter VEC_MODE8  = 2'b11,

/////////////////////////////////////////////////////////
//    ____ ____    ____            _     _             //
//   / ___/ ___|  |  _ \ ___  __ _(_)___| |_ ___ _ __  //
//  | |   \___ \  | |_) / _ \/ _` | / __| __/ _ \ '__| //
//  | |___ ___) | |  _ <  __/ (_| | \__ \ ||  __/ |    //
//   \____|____/  |_| \_\___|\__, |_|___/\__\___|_|    //
//                           |___/                     //
/////////////////////////////////////////////////////////

// CSR operations
parameter CSR_OP_NONE  = 2'b00,
parameter CSR_OP_WRITE = 2'b01,
parameter CSR_OP_SET   = 2'b10,
parameter CSR_OP_CLEAR = 2'b11,


// SPR for debugger, not accessible by CPU
parameter SP_DVR0       = 16'h3000,
parameter SP_DCR0       = 16'h3008,
parameter SP_DMR1       = 16'h3010,
parameter SP_DMR2       = 16'h3011,

parameter SP_DVR_MSB = 8'h00,
parameter SP_DCR_MSB = 8'h01,
parameter SP_DMR_MSB = 8'h02,
parameter SP_DSR_MSB = 8'h04,

// Privileged mode
//typedef enum logic[1:0] {
//  PRIV_LVL_M = 2'b11,
//  PRIV_LVL_H = 2'b10,
//  PRIV_LVL_S = 2'b01,
//  PRIV_LVL_U = 2'b00
//} PrivLvl_t,
parameter PRIVLVL_T_SIZE = 2,
parameter PRIV_LVL_M = 2'b11,PRIV_LVL_H = 2'b10,PRIV_LVL_S = 2'b01,PRIV_LVL_U = 2'b00, 
///////////////////////////////////////////////
//   ___ ____    ____  _                     //
//  |_ _|  _ \  / ___|| |_ __ _  __ _  ___   //
//   | || | | | \___ \| __/ _` |/ _` |/ _ \  //
//   | || |_| |  ___) | || (_| | (_| |  __/  //
//  |___|____/  |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// forwarding operand mux
parameter SEL_REGFILE      = 2'b00,
parameter SEL_FW_EX        = 2'b01,
parameter SEL_FW_WB        = 2'b10,

// operand a selection
parameter OP_A_REGA_OR_FWD = 3'b000,
parameter OP_A_CURRPC      = 3'b001,
parameter OP_A_IMM         = 3'b010,
parameter OP_A_REGB_OR_FWD = 3'b011,
parameter OP_A_REGC_OR_FWD = 3'b100,

// immediate a selection
parameter IMMA_Z      = 1'b0,
parameter IMMA_ZERO   = 1'b1,

// operand b selection
parameter OP_B_REGB_OR_FWD = 3'b000,
parameter OP_B_REGC_OR_FWD = 3'b001,
parameter OP_B_IMM         = 3'b010,
parameter OP_B_REGA_OR_FWD = 3'b011,
parameter OP_B_BMASK       = 3'b100,

// immediate b selection
parameter IMMB_I      = 4'b0000,
parameter IMMB_S      = 4'b0001,
parameter IMMB_U      = 4'b0010,
parameter IMMB_PCINCR = 4'b0011,
parameter IMMB_S2     = 4'b0100,
parameter IMMB_S3     = 4'b0101,
parameter IMMB_VS     = 4'b0110,
parameter IMMB_VU     = 4'b0111,
parameter IMMB_SHUF   = 4'b1000,
parameter IMMB_CLIP   = 4'b1001,
parameter IMMB_BI     = 4'b1011,

// bit mask selection
parameter BMASK_A_ZERO = 1'b0,
parameter BMASK_A_S3   = 1'b1,

parameter BMASK_B_S2   = 2'b00,
parameter BMASK_B_S3   = 2'b01,
parameter BMASK_B_ZERO = 2'b10,
parameter BMASK_B_ONE  = 2'b11,

parameter BMASK_A_REG  = 1'b0,
parameter BMASK_A_IMM  = 1'b1,
parameter BMASK_B_REG  = 1'b0,
parameter BMASK_B_IMM  = 1'b1,


// multiplication immediates
parameter MIMM_ZERO    = 1'b0,
parameter MIMM_S3      = 1'b1,

// operand c selection
parameter OP_C_REGC_OR_FWD = 2'b00,
parameter OP_C_REGB_OR_FWD = 2'b01,
parameter OP_C_JT          = 2'b10,

// branch types
parameter BRANCH_NONE = 2'b00,
parameter BRANCH_JAL  = 2'b01,
parameter BRANCH_JALR = 2'b10,
parameter BRANCH_COND = 2'b11, // conditional branches

// jump target mux
parameter JT_JAL  = 2'b01,
parameter JT_JALR = 2'b10,
parameter JT_COND = 2'b11,


///////////////////////////////////////////////
//   ___ _____   ____  _                     //
//  |_ _|  ___| / ___|| |_ __ _  __ _  ___   //
//   | || |_    \___ \| __/ _` |/ _` |/ _ \  //
//   | ||  _|    ___) | || (_| | (_| |  __/  //
//  |___|_|     |____/ \__\__,_|\__, |\___|  //
//                              |___/        //
///////////////////////////////////////////////

// PC mux selector defines
parameter PC_BOOT          = 3'b000,
parameter PC_JUMP          = 3'b010,
parameter PC_BRANCH        = 3'b011,
parameter PC_EXCEPTION     = 3'b100,
parameter PC_ERET          = 3'b101,
parameter PC_DBG_NPC       = 3'b111,

// Exception PC mux selector defines
parameter EXC_PC_ILLINSN   = 2'b00,
parameter EXC_PC_ECALL     = 2'b01,
parameter EXC_PC_LOAD      = 2'b10,
parameter EXC_PC_STORE     = 2'b10,
parameter EXC_PC_IRQ       = 2'b11,

// Exception Cause
parameter EXC_CAUSE_ILLEGAL_INSN = 6'h02,
parameter EXC_CAUSE_BREAKPOINT   = 6'h03,
parameter EXC_CAUSE_ECALL_UMODE  = 6'h08,
parameter EXC_CAUSE_ECALL_MMODE  = 6'h0B,

// Trap mux selector
parameter TRAP_MACHINE      = 1'b0,
parameter TRAP_USER         = 1'b1,

// Exceptions offsets
// target address = {boot_addr[31:8], EXC_OFF} (boot_addr must be 32 BYTE aligned!)
// offset 00 to 7e is used for external interrupts
parameter EXC_OFF_RST      = 8'h80,
parameter EXC_OFF_ILLINSN  = 8'h84,
parameter EXC_OFF_ECALL    = 8'h88,
parameter EXC_OFF_LSUERR   = 8'h8c,


// Debug module
parameter DBG_SETS_W = 6,

parameter DBG_SETS_IRQ    = 5,
parameter DBG_SETS_ECALL  = 4,
parameter DBG_SETS_EILL   = 3,
parameter DBG_SETS_ELSU   = 2,
parameter DBG_SETS_EBRK   = 1,
parameter DBG_SETS_SSTE   = 0,

parameter DBG_CAUSE_HALT   = 6'h1F,

// private FPU
parameter C_CMD               = 4,
parameter C_FPU_ADD_CMD       = 4'h0,
parameter C_FPU_SUB_CMD       = 4'h1,
parameter C_FPU_MUL_CMD       = 4'h2,
parameter C_FPU_DIV_CMD       = 4'h3,
parameter C_FPU_I2F_CMD       = 4'h4,
parameter C_FPU_F2I_CMD       = 4'h5,
parameter C_FPU_SQRT_CMD      = 4'h6,
parameter C_FPU_NOP_CMD       = 4'h7,
parameter C_FPU_FMADD_CMD     = 4'h8,
parameter C_FPU_FMSUB_CMD     = 4'h9,
parameter C_FPU_FNMADD_CMD    = 4'hA,
parameter C_FPU_FNMSUB_CMD    = 4'hB,
   
parameter C_FFLAG             = 5,
parameter C_RM                = 3,
parameter C_RM_NEAREST        = 3'h0,
parameter C_RM_TRUNC          = 3'h1,
parameter C_RM_PLUSINF        = 3'h3,
parameter C_RM_MINUSINF       = 3'h2,
parameter C_PC                = 5,

//endpackage
