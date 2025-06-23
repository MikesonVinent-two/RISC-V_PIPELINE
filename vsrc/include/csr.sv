`ifndef CSR_SV
`define CSR_SV

`ifdef VERILATOR
`include "include/common.sv"
`endif

package csr_pkg;
  import common::*;

  parameter u12 CSR_MHARTID = 12'hf14;
  parameter u12 CSR_MIE = 12'h304;
  parameter u12 CSR_MIP = 12'h344;
  parameter u12 CSR_MTVEC = 12'h305;
  parameter u12 CSR_MSTATUS = 12'h300;
  parameter u12 CSR_MSCRATCH = 12'h340;
  parameter u12 CSR_MEPC = 12'h341;
  parameter u12 CSR_SATP = 12'h180;
  parameter u12 CSR_MCAUSE = 12'h342;
  parameter u12 CSR_MCYCLE = 12'hb00;
  parameter u12 CSR_MTVAL = 12'h343;
  parameter u12 CSR_PMPADDR0 = 12'h3b0;
  parameter u12 CSR_PMPCFG0 = 12'h3a0;
  parameter u12 CSR_MEDELEG = 12'h302;
  parameter u12 CSR_MIDELEG = 12'h303;
  parameter u12 CSR_STVEC = 12'h105;
  parameter u12 CSR_SSTATUS = 12'h100;
  parameter u12 CSR_SSCRATCH = 12'h140;
  parameter u12 CSR_SEPC = 12'h141;
  parameter u12 CSR_SCAUSE = 12'h142;
  parameter u12 CSR_STVAL = 12'h143;
  parameter u12 CSR_SIE = 12'h104;
  parameter u12 CSR_SIP = 12'h144;

  parameter u64 MSTATUS_MASK = 64'h7e79bb;
  parameter u64 SSTATUS_MASK = 64'h800000030001e000;
  parameter u64 MIP_MASK = 64'h333;
  parameter u64 MTVEC_MASK = ~(64'h2);
  parameter u64 MEDELEG_MASK = 64'h0;
  parameter u64 MIDELEG_MASK = 64'h0;

  // mstatus 位域掩码定义
  parameter u64 MSTATUS_MPP_MASK  = 64'h0000_0000_0000_1800;  // MPP [12:11]
  parameter u64 MSTATUS_MPIE_MASK = 64'h0000_0000_0000_0080;  // MPIE [7]
  parameter u64 MSTATUS_MIE_MASK  = 64'h0000_0000_0000_0008;  // MIE [3]
  parameter u64 MSTATUS_FS_MASK   = 64'h0000_0000_0000_6000;  // FS [14:13]
  parameter u64 MSTATUS_XS_MASK   = 64'h0000_0000_0001_8000;  // XS [16:15]
  parameter u64 MSTATUS_MPRV_MASK = 64'h0000_0000_0002_0000;  // MPRV [17]
  parameter u64 MSTATUS_SUM_MASK  = 64'h0000_0000_0004_0000;  // SUM [18]
  parameter u64 MSTATUS_MXR_MASK  = 64'h0000_0000_0008_0000;  // MXR [19]
  parameter u64 MSTATUS_TVM_MASK  = 64'h0000_0000_0010_0000;  // TVM [20]
  parameter u64 MSTATUS_TW_MASK   = 64'h0000_0000_0020_0000;  // TW [21]
  parameter u64 MSTATUS_TSR_MASK  = 64'h0000_0000_0040_0000;  // TSR [22]

  // 组合掩码
  parameter u64 MSTATUS_WRITABLE_MASK = 
    MSTATUS_MPP_MASK  | MSTATUS_MPIE_MASK | MSTATUS_MIE_MASK  |
    MSTATUS_FS_MASK   | MSTATUS_XS_MASK   | MSTATUS_MPRV_MASK |
    MSTATUS_SUM_MASK  | MSTATUS_MXR_MASK  | MSTATUS_TVM_MASK  |
    MSTATUS_TW_MASK   | MSTATUS_TSR_MASK;

  parameter u2 PRIVILEGE_MODE = 2'b11;
  parameter u2 USER_MODE = 2'b00;

  // PMP 相关常量定义
  parameter u64 PMPADDR_MASK = 64'h3FFFFFFFFFFF;  // PMP地址掩码 (54位物理地址)
  parameter u8 PMPCFG_MASK = 8'h9F;               // PMP配置掩码 (R=1, W=2, X=4, A=3<<3, L=7)
  
  // PMP配置位定义
  parameter u8 PMP_R = 8'h01;     // 读权限
  parameter u8 PMP_W = 8'h02;     // 写权限  
  parameter u8 PMP_X = 8'h04;     // 执行权限
  parameter u8 PMP_A_OFF = 8'h00; // 地址匹配关闭
  parameter u8 PMP_A_TOR = 8'h08; // 顶部范围匹配
  parameter u8 PMP_A_NA4 = 8'h10; // 自然对齐4字节
  parameter u8 PMP_A_NAPOT = 8'h18; // 自然对齐2的幂次
  parameter u8 PMP_L = 8'h80;     // 锁定位

  // PMP访问类型枚举
  typedef enum logic [1:0] {
    PMP_ACCESS_READ  = 2'b00,
    PMP_ACCESS_WRITE = 2'b01,
    PMP_ACCESS_EXEC  = 2'b10
  } pmp_access_type_t;

  // PMP检查结果结构
  typedef struct packed {
    logic allowed;      // 是否允许访问
    logic fault;        // 是否产生访问错误
  } pmp_check_result_t;

  typedef struct packed {
    u1 sd;
    logic [MXLEN-2-36:0] wpri1;
    u2 sxl;
    u2 uxl;
    u9 wpri2;
    u1 tsr;
    u1 tw;
    u1 tvm;
    u1 mxr;
    u1 sum;
    u1 mprv;
    u2 xs;
    u2 fs;
    u2 mpp;
    u2 wpri3;
    u1 spp;
    u1 mpie;
    u1 wpri4;
    u1 spie;
    u1 upie;
    u1 mie;
    u1 wpri5;
    u1 sie;
    u1 uie;
  } mstatus_t;

  typedef struct packed {
    u4  mode;
    u16 asid;
    u44 ppn;
  } satp_t;

  typedef enum logic [1:0] {
    INSTR_MISALIGN,
    LOAD_MISALIGN,
    STORE_MISALIGN,
    DEFAULT
  } Misalign_type;

  typedef struct packed {
    u1 valid;
    Misalign_type misalign_type;
    u64 addr;
    u64 pc;
  } Misalign;

    typedef enum logic [3:0] {
    EXC_NONE          = 4'd0,
    EXC_INST_ADDR_MISALIGNED = 4'd1,
    EXC_INST_ACCESS_FAULT = 4'd2,
    EXC_ILLEGAL_INST  = 4'd3,
    EXC_BREAKPOINT    = 4'd4,
    EXC_LOAD_ADDR_MISALIGNED = 4'd5,
    EXC_LOAD_ACCESS_FAULT = 4'd6,
    EXC_STORE_ADDR_MISALIGNED = 4'd7,
    EXC_STORE_ACCESS_FAULT = 4'd8,
    EXC_ECALL_U       = 4'd9,
    EXC_ECALL_M       = 4'd11,
    EXC_INST_PAGE_FAULT = 4'd12,
    EXC_LOAD_PAGE_FAULT = 4'd13,
    EXC_STORE_PAGE_FAULT = 4'd14
  } exception_code_t;

  typedef enum logic [3:0] {
    SWINTPROCESSING,
    TRINTPROCESSING,
    EXINTPROCESSING,
    TRAPDEFAULT
  } interrupt_trap_cause_t;

  typedef struct packed {
    u1 valid;
    interrupt_trap_cause_t trap_cause;
    u64 pc;
  } interrupt_trap_t;

  
  
endpackage

`endif
