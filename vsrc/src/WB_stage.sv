`ifndef __WB_STAGE_SV
`define __WB_STAGE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr.sv"
`else

`endif

// 写回阶段模块
module WB_stage
  import common::*;
  import csr_pkg::*;
(
    input  logic         clk,                        // 时钟信号
    input  logic         reset,                      // 复位信号
    input  logic  [63:0] pc_in,                      // 输入的程序计数器值
    input  logic  [63:0] result_in,                  // ALU计算结果输入
    input  logic  [63:0] mem_data_in,                // 内存数据输入
    input  logic  [ 4:0] rd,                         // 目标寄存器地址
    input  logic         reg_write,                  // 寄存器写使能
    input  logic         mem_to_reg,                 // 内存到寄存器的控制信号
    input  logic         mem_exception,              // 内存异常标志
    input  logic         block,                      // 阻塞信号
    input  logic         result_valid,               // 结果有效标志
    input  logic         trint,                      // 定时器中断
    input  logic         swint,                      // 软件中断
    input  logic         exint,                      // 外部中断
    // 原子指令状态输入
    input  logic         atomic_in_progress,           // 原子操作进行中标志
    output logic  [63:0] pc_out,                     // 程序计数器输出
    output logic  [63:0] result_out,                 // 结果输出
    output word_t        regfile          [  31:0],  // 寄存器文件
    input  logic         mem_finish,
    // 拆分后的CSR寄存器输出
    output word_t        csr_mstatus,               // mstatus寄存器
    output word_t        csr_mtvec,                 // mtvec寄存器
    output word_t        csr_mepc,                  // mepc寄存器
    output word_t        csr_mcause,                // mcause寄存器
    output word_t        csr_mtval,                 // mtval寄存器
    output word_t        csr_mscratch,              // mscratch寄存器
    output word_t        csr_mhartid,               // mhartid寄存器
    output word_t        csr_medeleg,               // medeleg寄存器
    output word_t        csr_mideleg,               // mideleg寄存器
    output word_t        csr_mie,                   // mie寄存器
    output word_t        csr_mip,                   // mip寄存器
    output word_t        csr_sstatus,               // sstatus寄存器
    output word_t        csr_stvec,                 // stvec寄存器
    output word_t        csr_sepc,                  // sepc寄存器
    output word_t        csr_scause,                // scause寄存器
    output word_t        csr_stval,                 // stval寄存器
    output word_t        csr_sscratch,              // sscratch寄存器
    output word_t        csr_sie,                   // sie寄存器
    output word_t        csr_sip,                   // sip寄存器
    output word_t        csr_satp,                  // satp寄存器
    output word_t        csr_misa,                  // misa寄存器
    output word_t        csr_mvendorid,             // mvendorid寄存器
    output word_t        csr_marchid,               // marchid寄存器
    output word_t        csr_mimpid,                // mimpid寄存器
    output word_t        csr_custom_csrs[20:0],     // 自定义CSR寄存器
    output word_t        csr_pmpaddr0,              // pmpaddr0寄存器
    output word_t        csr_pmpcfg0,               // pmpcfg0寄存器
    
    output logic         finish_w,                   // 写回完成标志
    output logic  [63:0] csr_read_data,              // CSR读取数据
    output logic  [63:0] mcycle,                     // 时钟周期计数器
    output logic  [63:0] minstret,                   // 指令计数器
    output logic  [ 1:0] privilege_mode,             // 当前特权模式
    output logic         exception,                  // 异常信号
    input  logic  [31:0] ex_instr_in,  
    input  logic  [31:0] instr_in,
    output logic  [31:0] instr_out,
    output logic         reg_write_out,
    output logic         valid,
    output logic         mem_read_out,
    output logic         mem_write_out,
    input  logic         mem_read_in,
    input  logic         mem_write_in,
    output logic         mem_exception_out,
    output logic  [63:0] mem_addr_out,
    input  logic  [63:0] mem_addr_in,
    output logic  [ 4:0] wb_rd,
    output logic         skip,
    output Misalign      misalign,
    output logic         ecall_need,
    output  logic         mret_to_ecall_dll_valid,
    output logic         MMU_exception
);

  logic [63:0] current_pc = 64'b0;  // 当前PC值，初始化为0

  interrupt_trap_t interrupt_trap;

  // CSR相关信号
  logic [11:0] csr_addr;  // CSR地址
  logic        csr_write_en;  // CSR写使能
  logic [63:0] csr_write_data;  // CSR写数据
  logic [ 2:0] csr_op;  // CSR操作类型
  logic        csr_access_allowed;  // 特权级访问权限
  logic        csr_write_allowed;  // 写入权限
  logic [4:0]  csr_index;  // CSR索引（用于自定义CSR）

  // CSR读取逻辑
  logic        exception_tmp;  // 临时信号用于存储在组合逻辑中生成的异常信号
  logic [63:0] exception_cause; // 异常原因编码
  logic [63:0] exception_val;   // 异常附加信息
  logic        is_interrupt;    // 标识是否为中断
  logic        valid_next;


  // 添加判断是否有待处理中断的逻辑    
  logic has_pending_interrupt;
  logic [63:0] interrupt_cause;

  // 添加异常处理相关的额外信号

  logic temp_valid;// 我真服了，一个个的
  logic [63:0] temp_pc;

  logic mstatus_has_just_been_directly_set;

  exception_code_t exception_code;
  
  // 添加异常/中断跳转目标地址
  logic [63:0] trap_vector;
  logic        trap_vector_mode;

  // 添加trap处理相关的统一信号
  logic        take_trap;            // 是否需要陷入处理
  logic [63:0] trap_cause;           // 陷入原因（异常或中断）
  logic [63:0] trap_value;           // 陷入相关值
  logic        is_interrupt_trap;    // 标识陷入是否为中断
    // 初始化寄存器文件
    initial begin
        for (int i = 0; i < 32; i = i + 1) begin
            regfile[i] = '0;  // 将所有寄存器初始化为0
        end
    end

  // PMP 检查函数
  function automatic pmp_check_result_t pmp_check(
    input logic [63:0] addr,
    input pmp_access_type_t access_type,
    input logic [1:0] priv_mode
  );
    pmp_check_result_t result;
    logic [7:0] cfg;
    logic [63:0] pmp_addr;
    logic [63:0] pmp_base, pmp_top;
    logic match;
    logic [1:0] addr_mode;
    logic perm_check;
    
    // 默认值
    result.allowed = 1'b0;
    result.fault = 1'b0;
    
    // 获取PMP配置
    cfg = csr_pmpcfg0[7:0];  // 只实现PMP0
    pmp_addr = csr_pmpaddr0;
    
    // 如果PMP关闭，机器模式默认允许所有访问
    if ((cfg & 8'h18) == PMP_A_OFF) begin
      result.allowed = (priv_mode == 2'b11);
      return result;
    end
    
    // 地址匹配模式
    addr_mode = cfg[4:3];
    match = 1'b0;
    
    case (addr_mode)
      2'b00: begin // OFF - 已处理
        match = 1'b0;
      end
             2'b01: begin // TOR (Top of Range)
         pmp_base = 64'h0;  // 简化实现，基地址为0
         pmp_top = {8'b0, pmp_addr[53:0], 2'b00};  // 地址左移2位，补齐64位
         match = (addr >= pmp_base) && (addr < pmp_top);
       end
       2'b10: begin // NA4 (Natural Aligned 4-byte)
         pmp_base = {8'b0, pmp_addr[53:0], 2'b00};  // 补齐64位
         match = (addr >= pmp_base) && (addr < (pmp_base + 4));
       end
       2'b11: begin // NAPOT (Natural Aligned Power of Two)
         // 简化实现：假设为4KB页面
         pmp_base = {10'b0, pmp_addr[53:12], 12'b0};  // 补齐64位
         match = (addr >= pmp_base) && (addr < (pmp_base + 4096));
       end
    endcase
    
    if (match) begin
      // 检查权限
      case (access_type)
        PMP_ACCESS_READ:  perm_check = cfg[0];  // R位
        PMP_ACCESS_WRITE: perm_check = cfg[1];  // W位
        PMP_ACCESS_EXEC:  perm_check = cfg[2];  // X位
        default:          perm_check = 1'b0;
      endcase
      
      result.allowed = perm_check;
      result.fault = !perm_check;
    end else begin
      // 不匹配的情况下，机器模式默认允许
      result.allowed = (priv_mode == 2'b11);
      result.fault = 1'b0;
    end
    
    return result;
  endfunction

  // 用于获取CSR寄存器值的函数
  function automatic word_t get_csr_value(input logic [11:0] addr);
    // 使用优化的逻辑以减少MUX层数
    word_t result = 64'h0; // 默认返回值
    
    // 并行比较CSR地址并设置结果
    if (addr == CSR_MSTATUS)     result = csr_mstatus;
    if (addr == CSR_MTVEC)       result = csr_mtvec;
    if (addr == CSR_MEPC)        result = csr_mepc;
    if (addr == CSR_MCAUSE)      result = csr_mcause;
    if (addr == CSR_MTVAL)       result = csr_mtval;
    if (addr == CSR_MSCRATCH)    result = csr_mscratch;
    if (addr == CSR_MHARTID)     result = csr_mhartid;
    if (addr == CSR_MEDELEG)     result = csr_medeleg;
    if (addr == CSR_MIDELEG)     result = csr_mideleg;
    if (addr == CSR_MIE)         result = csr_mie;
    if (addr == CSR_MIP)         result = csr_mip;
    if (addr == CSR_SSTATUS)     result = csr_sstatus;
    if (addr == CSR_STVEC)       result = csr_stvec;
    if (addr == CSR_SEPC)        result = csr_sepc;
    if (addr == CSR_SCAUSE)      result = csr_scause;
    if (addr == CSR_STVAL)       result = csr_stval;
    if (addr == CSR_SSCRATCH)    result = csr_sscratch;
    if (addr == CSR_SIE)         result = csr_sie;
    if (addr == CSR_SIP)         result = csr_sip;
    if (addr == CSR_SATP)        result = csr_satp;
    if (addr == 12'h301)         result = csr_misa;        // MISA
    if (addr == 12'hF11)         result = csr_mvendorid;   // MVENDORID
    if (addr == 12'hF12)         result = csr_marchid;     // MARCHID
    if (addr == 12'hF13)         result = csr_mimpid;      // MIMPID
    if (addr == 12'hB02)         result = minstret;        // MINSTRET
    if (addr == CSR_PMPADDR0)    result = csr_pmpaddr0;    // PMPADDR0
    if (addr == CSR_PMPCFG0)     result = csr_pmpcfg0;     // PMPCFG0
    
    // 检查自定义CSR
    if (addr >= 12'h800 && addr <= 12'h814) begin
      result = csr_custom_csrs[addr[4:0]];
    end
    
    return result;
  endfunction
  

  always_comb begin
    // 从EX阶段指令获取CSR地址
    csr_addr = ex_instr_in[31:20];
    csr_read_data = 64'b0;
    csr_access_allowed = (csr_addr[9:8] <= privilege_mode);
    exception_tmp = 1'b0;  // 默认无异常
    exception_code = EXC_NONE;
    exception_val = 64'h0;
    is_interrupt = 1'b0;
    trap_vector = 64'h0;
    trap_vector_mode = 1'b0;

    // 首先检查是否存在地址不对齐异常
    if (misalign.valid) begin
      exception_tmp = 1'b1;
      exception_val = misalign.addr;
      case (misalign.misalign_type)
        INSTR_MISALIGN: exception_code = EXC_INST_ADDR_MISALIGNED;
        LOAD_MISALIGN:  exception_code = EXC_LOAD_ADDR_MISALIGNED;
        STORE_MISALIGN: exception_code = EXC_STORE_ADDR_MISALIGNED;
        default: exception_code = EXC_INST_ADDR_MISALIGNED;  // 默认情况
      endcase
    // 检查PMP访问异常
    end else if (mem_exception) begin
      exception_tmp = 1'b1;
      exception_val = mem_addr_in;  // 使用导致异常的内存地址
      // 根据内存操作类型确定异常代码
      if (mem_write_in) begin
        exception_code = EXC_STORE_ACCESS_FAULT;
      end else if (mem_read_in) begin
        exception_code = EXC_LOAD_ACCESS_FAULT;
      end else begin
        exception_code = EXC_INST_ACCESS_FAULT;  // 指令获取异常
      end
    end
    // 如果没有地址不对齐异常，再检查其他异常
    else begin
      // 检查非法指令
      case (instr_in[6:0])
        7'b0110111,  // LUI
        7'b0010111,  // AUIPC
        7'b1101111,  // JAL
        7'b1100111,  // JALR
        7'b1100011,  // 分支指令(BEQ,BNE,BLT,BGE,BLTU,BGEU)
        7'b0000011,  // 加载指令(LB,LH,LW,LBU,LHU)
        7'b0100011,  // 存储指令(SB,SH,SW)
        7'b0010011,  // 立即数运算指令(ADDI,SLTI,SLTIU,XORI,ORI,ANDI,SLLI,SRLI,SRAI)
        7'b0110011,  // 寄存器运算指令(ADD,SUB,SLL,SLT,SLTU,XOR,SRL,SRA,OR,AND)
        7'b1110011,  // 系统指令(ECALL,EBREAK,CSRRW,CSRRS,CSRRC,CSRRWI,CSRRSI,CSRRCI)
        7'b0001111,  // FENCE指令
        7'b0101111,  // 原子操作指令(AMO*)
        7'b0011011,  // RV64I 立即数运算指令(ADDIW,SLLIW,SRLIW,SRAIW)
        7'b0111011:  // RV64I 寄存器运算指令(ADDW,SUBW,SLLW,SRLW,SRAW)
          ; // 合法指令，不做处理
        default: begin
          // 未定义的指令，触发非法指令异常
          exception_tmp = 1'b1;
          exception_code = EXC_ILLEGAL_INST;
          exception_val = {32'b0, instr_in};
        end
      endcase

      // MRET: 30200073
      if (instr_in == 32'h30200073) begin
        if (privilege_mode < 2'b11) begin
          // 如果当前特权模式不足以执行该指令，则触发非法指令异常
          exception_tmp = 1'b1;
          exception_code = EXC_ILLEGAL_INST;
          exception_val = {32'b0, instr_in};
        end
      end

      // 检查是否有非法CSR访问
      if (instr_in[6:0] == 7'b1110011 && instr_in[14:12] != 3'b000) begin
        if (!csr_access_allowed) begin
          csr_read_data = 64'b0;
          exception_tmp = 1'b1;
          exception_code = EXC_ILLEGAL_INST;
          exception_val = {32'b0, instr_in};  // 将32位instr_in扩展为64位
        end else begin
          // 特殊处理性能计数器
          case (csr_addr)
            CSR_MCYCLE: csr_read_data = mcycle;
            default: csr_read_data = get_csr_value(csr_addr);
          endcase
        end
      end
      
      // 检查指令是否为ECALL
      if (instr_in == 32'h00000073) begin  // ECALL
        exception_tmp = 1'b1;
        case (privilege_mode)
          2'b00: exception_code = EXC_ECALL_U;
          2'b11: exception_code = EXC_ECALL_M;
          default: exception_code = EXC_ECALL_U;
        endcase
        exception_val = 64'h0;  // ECALL通常不设置mtval
      end
      
      // 检查是否有待处理中断
      has_pending_interrupt = 1'b0;
      interrupt_cause = 64'h0;
      
      // 只有当MIE=1(全局中断使能)且没有原子操作在进行时才检查中断
      if ((privilege_mode < 2'b11 || (privilege_mode == 2'b11 && csr_mstatus[3])) && 
          (exint|trint|swint |MMU_exception) && !atomic_in_progress) begin
        
        // 按优先级检查中断
        if (exint) begin  // 外部中断
          has_pending_interrupt = 1'b1;
          interrupt_cause = 64'h8000000000000B;  // 外部中断编码: 11
        end else if (trint) begin  // 定时器中断
          has_pending_interrupt = 1'b1;
          interrupt_cause = 64'h8000000000000007;  // 定时器中断编码: 7
        end else if (swint) begin  // 软件中断
          has_pending_interrupt = 1'b1;
          interrupt_cause = 64'h8000000000000003;  // 软件中断编码: 3
        end else begin
          has_pending_interrupt = 1'b0;
          interrupt_cause = 64'h0;
        end
      end
      
      // 计算异常向量地址
      // 只使用M模式的向量表
      trap_vector_mode = csr_mtvec[0];  // 模式位
      if (has_pending_interrupt && trap_vector_mode) begin
        // 向量模式中断处理
        trap_vector = {csr_mtvec[63:2], 2'b00} + (64'(interrupt_cause[3:0]) << 2);
      end else begin
        // 直接模式或异常处理
        trap_vector = {csr_mtvec[63:2], 2'b00};
      end
    end
  end

  // CSR写入控制信号生成
  always_comb begin
    logic [1:0] csr_priv;  // 存放CSR地址的特权级字段
    logic [11:0] csr_addr_temp;  // 临时变量存储CSR地址
    
    csr_write_en = 1'b0;
    csr_write_data = 64'b0;
    csr_op = 3'b000;
    csr_write_allowed = 1'b0;  // 默认不允许写入
    csr_index = 5'b0;  // 默认CSR索引
    csr_priv = 2'b0;   // 初始化特权级变量
    csr_addr_temp = 12'b0;  // 初始化CSR地址临时变量

    if (ex_instr_in[6:0] == 7'b1110011) begin  // CSR指令
      csr_op = ex_instr_in[14:12];

      // 特权级检查 - 修复位宽不匹配问题
      // 使用两步操作提取特权级位域
      csr_addr_temp = ex_instr_in[31:20];
      csr_priv = 2'(csr_addr_temp >> 8);  // 显式类型转换为2位
      csr_write_allowed = csr_priv <= privilege_mode;

      if (csr_write_allowed) begin
        case (csr_op)
          3'b001: begin  // CSRRW
            csr_write_en   = reg_write;
            csr_write_data = regfile[instr_in[19:15]];
          end
          3'b010: begin  // CSRRS
            csr_write_en   = reg_write && (instr_in[19:15] != 5'b00000);
            csr_write_data = get_csr_value(instr_in[31:20]) | regfile[instr_in[19:15]];
          end
          3'b011: begin  // CSRRC
            csr_write_en   = reg_write && (instr_in[19:15] != 5'b00000);
            csr_write_data = get_csr_value(instr_in[31:20]) & ~regfile[instr_in[19:15]];
          end
          3'b101: begin  // CSRRWI
            csr_write_en   = reg_write;
            csr_write_data = {59'b0, instr_in[19:15]};
          end
          3'b110: begin  // CSRRSI
            csr_write_en   = reg_write && (instr_in[19:15] != 5'b00000);
            csr_write_data = get_csr_value(instr_in[31:20]) | {59'b0, instr_in[19:15]};
          end
          3'b111: begin  // CSRRCI
            csr_write_en   = reg_write && (instr_in[19:15] != 5'b00000);
            csr_write_data = get_csr_value(instr_in[31:20]) & ~{59'b0, instr_in[19:15]};
          end
          default: begin
            csr_write_en   = 1'b0;
            csr_write_data = 64'b0;
          end
        endcase
      end
    end
  end

  // 修改trap_cause的生成逻辑
  always_comb begin
    // 默认值
    trap_cause = 64'b0;
    
    if (has_pending_interrupt) begin
      // 中断的mcause[63]必须为1
      trap_cause = interrupt_cause | 64'h8000000000000000;
    end else if (exception_tmp) begin
      // 异常的mcause[63]必须为0，异常码在低位
      case (exception_code)
        EXC_INST_ADDR_MISALIGNED: trap_cause = 64'd0;
        EXC_INST_ACCESS_FAULT:    trap_cause = 64'd1;
        EXC_ILLEGAL_INST:         trap_cause = 64'd2;
        EXC_BREAKPOINT:           trap_cause = 64'd3;
        EXC_LOAD_ADDR_MISALIGNED: trap_cause = 64'd4;
        EXC_LOAD_ACCESS_FAULT:    trap_cause = 64'd5;
        EXC_STORE_ADDR_MISALIGNED: trap_cause = 64'd6;
        EXC_STORE_ACCESS_FAULT:   trap_cause = 64'd7;
        EXC_ECALL_U:              trap_cause = 64'd8;
        EXC_ECALL_M:              trap_cause = 64'd11;
        EXC_INST_PAGE_FAULT:      trap_cause = 64'd12;
        EXC_LOAD_PAGE_FAULT:      trap_cause = 64'd13;
        EXC_STORE_PAGE_FAULT:     trap_cause = 64'd15;
        default:                  trap_cause = 64'd2; // 默认为非法指令
      endcase
    end
  end



  // 写回阶段主要逻辑
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      pc_out <= 64'b0;
      result_out <= 64'b0;
      exception <= 1'b0;  // 重置exception信号
      current_pc <= 64'b0; // 重置current_pc

      // 重置CSR寄存器 - 使用更精确的初始化值
      // 初始化为M模式(MPP=11)，关闭中断(MIE=0)，其他位清零
      csr_mstatus   <= 64'h0000_0000_0000_0080;
      
      // 重置CSR寄存器
      csr_mtvec     <= 64'h0;     // 默认中断向量表基地址
      csr_mip       <= 64'h0;     // 初始无待处理中断
      csr_mie       <= 64'h0;     // 初始禁用所有中断
      csr_mscratch  <= 64'h0;     // 临时寄存器初始化为0
      csr_mcause    <= 64'h0;     // 初始无异常原因
      csr_mtval     <= 64'h0;     // 初始无异常值
      csr_mepc      <= 64'h0;     // 初始无异常PC
      csr_mhartid   <= 64'h0;     // Hart ID初始化为0

      csr_satp      <= 64'h0;     // 初始禁用地址转换
      temp_valid    <= 1'b0;

      // 初始化PMP寄存器
      csr_pmpaddr0  <= 64'h0;     // PMP地址寄存器0初始化
      csr_pmpcfg0   <= 64'h0;     // PMP配置寄存器0初始化

      // 初始化自定义CSR寄存器
      for (int i = 0; i < 21; i++) begin
        csr_custom_csrs[i] <= 64'b0;
      end

      mcycle                    <= 64'h0;
      minstret                  <= 64'h0;
      privilege_mode             <= 2'b11;  // 明确设置当前模式为M模式
      instr_out                 <= 32'b0;
      reg_write_out             <= 1'b0;
      valid_next                <= 1'b0;
      mem_read_out              <= 1'b0;
      mem_write_out             <= 1'b0;
      mem_exception_out         <= 1'b0;
      mem_addr_out              <= 64'b0;
      wb_rd                     <= 5'b0;
      interrupt_trap.valid      <= 1'b0;
      interrupt_trap.trap_cause <= TRAPDEFAULT;
      interrupt_trap.pc         <= 64'b0;
      ecall_need                <= 1'b0;
      mstatus_has_just_been_directly_set <= 1'b0;
    end else begin
      pc_out <= pc_in;

      if(mret_to_ecall_dll_valid) begin
        mret_to_ecall_dll_valid <= 1'b0;
        misalign.valid <= 1'b0;
        misalign.addr <= 64'b0;
        misalign.misalign_type <= DEFAULT;

        if(temp_valid) begin
          csr_mepc <= temp_pc + 4;
          temp_valid <= 1'b0;
        end else begin
          csr_mepc <= pc_in;
        end
        // 2. 设置mcause和mtval
        csr_mcause <= trap_cause;
        csr_mtval <= trap_value;
          
        // 3. 更新mstatus - 更精确的位操作
        csr_mstatus <= (csr_mstatus & ~(MSTATUS_MPP_MASK | MSTATUS_MPIE_MASK | MSTATUS_MIE_MASK)) |
                        (({62'b0, privilege_mode} << 11) & MSTATUS_MPP_MASK) |
                        (((csr_mstatus & MSTATUS_MIE_MASK) << 4) & MSTATUS_MPIE_MASK);
        // 4. 切换到M模式
        privilege_mode <= 2'b11;    
        // 5. 设置PC为异常向量地址
        pc_out <= pc_in;  
      end


      // 检测中断触发条件
      if((privilege_mode < 2'b11 || (privilege_mode == 2'b11 && csr_mstatus[3])) && (trint|swint|MMU_exception|exint) && !interrupt_trap.valid) begin
        // 第一次检测到中断时设置valid和ecall_need
        if(trint) begin
          interrupt_trap.valid <= 1'b1;
          interrupt_trap.trap_cause <= TRINTPROCESSING;
          interrupt_trap.pc <= pc_in;
          ecall_need <= 1'b1;
        end else if(swint) begin 
          interrupt_trap.valid <= 1'b1;
          interrupt_trap.trap_cause <= SWINTPROCESSING;
          interrupt_trap.pc <= pc_in;
          ecall_need <= 1'b1;
        end else if(exint) begin
          interrupt_trap.valid <= 1'b1;
          interrupt_trap.trap_cause <= EXINTPROCESSING;
          interrupt_trap.pc <= pc_in;
          ecall_need <= 1'b1;
        end
      end 
      
      if(interrupt_trap.valid) begin
        // 当中断有效时,检查对应的中断信号是否已经拉低

        case(interrupt_trap.trap_cause)
          TRINTPROCESSING: if(!trint) begin
            interrupt_trap.valid <= 1'b0;
            interrupt_trap.trap_cause <= TRAPDEFAULT;
          end
          SWINTPROCESSING: if(!swint) begin
            interrupt_trap.valid <= 1'b0;
            interrupt_trap.trap_cause <= TRAPDEFAULT;
          end
          EXINTPROCESSING: if(!exint) begin
            interrupt_trap.valid <= 1'b0;
            interrupt_trap.trap_cause <= TRAPDEFAULT;
          end
          default: ;
        endcase
      end 

      // 计算是否需要陷入处理以及相关信息
      take_trap = has_pending_interrupt || exception_tmp;
      is_interrupt_trap = has_pending_interrupt;
      trap_value = has_pending_interrupt ? 64'h0 : exception_val;

      // 更新exception信号
      exception <= take_trap;

      // 性能计数器更新
      mcycle <= mcycle + 1;
      if (valid) minstret <= minstret + 1;

      // 更新中断pending状态
      csr_mip <= (csr_mip & ~64'h888) |               // 清除原有的中断pending位
                 ({63'b0, trint} << 7) |              // 设置定时器中断pending位
                 ({63'b0, swint} << 3) |              // 设置软件中断pending位
                 ({63'b0, exint} << 11);              // 设置外部中断pending位

      // PC更新和寄存器写入
      if (mem_finish) begin
        finish_w <= 1;
        skip <= (mem_read_in || mem_write_in) && !mem_addr_in[31];
        // 结果选择逻辑
        if (reg_write && rd != 5'b00000) begin
          if (mem_exception) begin
            result_out <= 64'hDEADBEEF;  // 发生异常时返回特定值
          end else if (mem_to_reg) begin
            result_out  <= mem_data_in;  // 选择内存数据
            regfile[rd] <= word_t'(mem_data_in);
          end else if (instr_in[6:0] == 7'b1110011 && 
                      (instr_in[14:12] == 3'b001 || instr_in[14:12] == 3'b010 || 
                       instr_in[14:12] == 3'b011 || instr_in[14:12] == 3'b101 || 
                       instr_in[14:12] == 3'b110 || instr_in[14:12] == 3'b111)) begin
            // CSR指令，将CSR寄存器的值写入目标寄存器
            result_out  <= csr_read_data;  // 选择CSR读取的数据
            regfile[rd] <= word_t'(csr_read_data);
          end else begin
            result_out <= result_in;  // 选择ALU结果
            if (result_valid) begin
              regfile[rd] <= word_t'(result_in);
            end
          end
        end

        // CSR写入
        if (csr_write_en) begin
          logic [4:0] temp_index;  // 预先声明索引变量
          
          case (instr_in[31:20])
            CSR_MCYCLE: mcycle <= csr_write_data;
            12'hB02: minstret <= csr_write_data;  // MINSTRET

            // 使用掩码确保只有允许的位能被修改
            CSR_MSTATUS: begin
                // 只允许修改可写位，保留其他位不变
                csr_mstatus <= (csr_mstatus & ~MSTATUS_WRITABLE_MASK) | 
                             (csr_write_data & MSTATUS_WRITABLE_MASK);
            end
            
            CSR_SSTATUS:
                csr_sstatus <= (csr_sstatus & ~SSTATUS_MASK) | 
                             (csr_write_data & SSTATUS_MASK);
            CSR_MIP:
                csr_mip <= (csr_mip & ~csr_pkg::MIP_MASK) | 
                         (csr_write_data & csr_pkg::MIP_MASK);
            CSR_SATP:
                csr_satp <= (csr_satp) | 
                           (csr_write_data);
            
            CSR_MTVEC:
                csr_mtvec <= (csr_mtvec & ~MTVEC_MASK) | 
                           (csr_write_data & MTVEC_MASK);
            CSR_MEDELEG:
                csr_medeleg <= (csr_medeleg & ~MEDELEG_MASK) | 
                             (csr_write_data & MEDELEG_MASK);
            CSR_MIDELEG:
                csr_mideleg <= (csr_mideleg & ~MIDELEG_MASK) | 
                             (csr_write_data & MIDELEG_MASK);
            CSR_MEPC:      csr_mepc <= csr_write_data;
            CSR_MCAUSE:    csr_mcause <= csr_write_data;
            CSR_MTVAL:     csr_mtval <= csr_write_data;
            CSR_MSCRATCH:  csr_mscratch <= csr_write_data;
            CSR_MHARTID:   csr_mhartid <= csr_write_data;
            CSR_MIE:       csr_mie <= csr_write_data;
            CSR_PMPADDR0:  csr_pmpaddr0 <= csr_write_data & PMPADDR_MASK;
            CSR_PMPCFG0:   csr_pmpcfg0 <= csr_write_data & {56'h0, PMPCFG_MASK};
            default: begin
              // 检查是否为自定义CSR
              if (instr_in[31:20] >= 12'h800 && instr_in[31:20] <= 12'h814) begin
                // 使用前面声明的临时变量
                temp_index = instr_in[24:20];  // 这里直接使用[4:0]取低5位
                csr_custom_csrs[temp_index] <= csr_write_data;
              end
            end
          endcase
        end

        if(csr_write_en && instr_in[31:20] == CSR_MSTATUS) begin
          if((privilege_mode < 2'b11 || (privilege_mode == 2'b11 && csr_mstatus[3])) == 0) begin
            if ((privilege_mode < 2'b11 || (privilege_mode == 2'b11 && csr_write_data[3])) == 1) begin
                mstatus_has_just_been_directly_set <= 1'b1;
                temp_valid <= 1'b1;
                temp_pc <= pc_in;
              
            end
          end
        end else begin
          mstatus_has_just_been_directly_set <= 1'b0;
        end

        // MRET指令处理
        if (instr_in == 32'h30200073) begin  // MRET
          // 1. 从MPP获取返回的特权模式
          privilege_mode <= csr_mstatus[12:11];
          
          // 2. 更新mstatus
          csr_mstatus <= (csr_mstatus & ~(MSTATUS_MPP_MASK | MSTATUS_MPIE_MASK | MSTATUS_MIE_MASK)) |
                        MSTATUS_MPIE_MASK |
                        (((csr_mstatus & MSTATUS_MPIE_MASK) >> 4) & MSTATUS_MIE_MASK);

          // 3. 设置返回地址为mepc的值
          pc_out <= pc_in; 

        end         // 统一的异常和中断处理逻辑
        if (take_trap) begin

          misalign.valid <= 1'b0;
          misalign.addr <= 64'b0;
          misalign.misalign_type <= DEFAULT;
          csr_mepc <= pc_in;
          
          // 2. 设置mcause和mtval
          csr_mcause <= trap_cause;
          csr_mtval <= trap_value;
          
          // 3. 更新mstatus - 更精确的位操作
          csr_mstatus <= (csr_mstatus & ~(MSTATUS_MPP_MASK | MSTATUS_MPIE_MASK | MSTATUS_MIE_MASK)) |
                        (({62'b0, privilege_mode} << 11) & MSTATUS_MPP_MASK) |
                        (((csr_mstatus & MSTATUS_MIE_MASK) << 4) & MSTATUS_MPIE_MASK);
          
          // 4. 切换到M模式
          privilege_mode <= 2'b11;
          
          
          // 5. 设置PC为异常向量地址
          pc_out <= pc_in;  
        end

        

        current_pc <= pc_in;
        instr_out <= instr_in;
        reg_write_out <= reg_write;
        valid_next <= 1;  // 使用组合逻辑生成的信号
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        mem_exception_out <= mem_exception;
        mem_addr_out <= mem_addr_in;
        wb_rd <= rd;
      end else begin
        finish_w <= 0;
        valid_next <= 0;
        mem_read_out <= 0;
        mem_write_out <= 0;
        mem_exception_out <= 0;
        mem_addr_out <= 0;
        wb_rd <= 0;
      end
    end
  end

  assign valid = valid_next;

endmodule

`endif
