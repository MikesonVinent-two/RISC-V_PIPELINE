// 五级流水线RISC-V处理器核心模块
`ifndef __CORE_SV  // 如果没有定义 __CORE_SV
`define __CORE_SV // 定义 __CORE_SV

`ifdef VERILATOR  // 如果定义了 VERILATOR
`include "include/common.sv"  // 包含 common.sv 文件
`include "include/csr.sv"     // 包含 csr.sv 文件
`include "src/branch_predictor.sv"  // 包含分支预测器
`include "src/IF_stage.sv"  // 包含 IF_stage.sv 文件
`include "src/ID_stage.sv"  // 包含 ID_stage.sv 文件
`include "src/EX_stage.sv"  // 包含 EX_stage.sv 文件
`include "src/MEM_stage.sv"  // 包含 MEM_stage.sv 文件
`include "src/WB_stage.sv"  // 包含 WB_stage.sv 文件
`endif

module core
  import common::*;
  import csr_pkg::*;
(
    // 基本控制信号
    input  logic       clk,    reset,     // 时钟和复位信号
    // 指令总线接口
    input  ibus_resp_t iresp,             // 指令总线响应
    // 数据总线接口
    input  dbus_resp_t dresp,             // 数据总线响应
    // 中断信号
    input  logic       trint, swint, exint, // 定时器中断、软件中断、外部中断
    // 指令总线接口
    output ibus_req_t  ireq,              // 指令总线请求
    // 数据总线接口
    output dbus_req_t  dreq,               // 数据总线请求

    output word_t satp,
    output logic [1:0] priviledge_mode,
    output logic skip,
    output logic MMU_exception
);

  // 寄存器堆 - 使用wire类型，由WB_stage驱动
  (* ram_style = "distributed" *) word_t regfile [31:0];  // 当前寄存器值，使用分布式RAM实现

  // CSR寄存器模块信号（拆分成单独的寄存器）
  word_t csr_mstatus, csr_mtvec, csr_mepc, csr_mcause, csr_mtval;
  word_t csr_mscratch, csr_mhartid, csr_medeleg, csr_mideleg, csr_mie, csr_mip;
  word_t csr_sstatus, csr_stvec, csr_sepc, csr_scause, csr_stval, csr_sscratch;
  word_t csr_sie, csr_sip, csr_satp, csr_misa, csr_mvendorid, csr_marchid;
  word_t csr_mimpid, csr_custom_csrs[20:0]; // 为其他可能需要的自定义CSR寄存器预留空间
  
  // PMP 寄存器
  word_t csr_pmpaddr0, csr_pmpcfg0;
  
  Misalign misalign;
  
  logic [63:0]  csr_read_data;            // CSR读取数据
  logic [63:0]  mcycle;                   // 时钟周期计数器
  logic [63:0]  minstret;                 // 指令计数器
  
  // 分支预测性能统计
  real total_branch, total_j, succ_branch, succ_j, wpc_j, wpc_ret;
  logic[30:0] print_cnt;

  // 流水线控制信号
  wire          stall;                     // 流水线暂停信号
  wire          block;                     // 流水线阻塞信号
  logic         exception;                 // 异常信号
  wire          valid;                     // 指令有效信号 - 由WB_stage驱动
  logic         oop;                       // 乱序执行标志
  logic         result_valid;              // 结果有效标志
  wire          finish_r;                  // 读操作完成标志 - 由MEM_stage驱动
  wire          if_finish;                 // PC更新完成标志 - 由WB_stage驱动

  // 分支控制信号 - 使用wire类型，由assign驱动
  wire          branch_taken;              // 分支是否taken
  wire   [63:0] branch_target;             // 分支目标地址
  wire          local_skip;                // 跳转信号                // 用于Difftest的skip信号
  
  // 流水线完成信号
  wire finish_w_if, finish_w_id, finish_w_ex, finish_w_mem, finish_w_wb;  // 各级流水线写完成信号，修改为wire类型                    // 整体完成信号
  
  // IF阶段信号 - 修改为wire类型
  wire   [63:0] if_pc;                     // IF阶段PC值
  wire   [31:0] if_instr;                  // IF阶段指令，修改为wire类型
  
  // ID阶段相关信号 - 明确wire类型，由ID_stage驱动
  wire id_branch;                          // ID阶段分支信号
  
  // EX阶段信号 - 修改为wire类型
  wire ex_branch;                          // 分支信号
  wire [63:0] ex_branch_target;            // 分支目标
  wire id_finish;                          // ID阶段完成信号

  logic ecall_need;

  logic mret_to_ecall_dll_valid;
  
  // 分支预测相关信号
  logic        if_pred_taken;              // IF阶段预测taken
  logic [63:0] if_pred_pc;                 // IF阶段预测PC
  logic        if_hit_ras;                 // IF阶段RAS命中
  logic        bp_update_en;               // 分支预测器更新使能
  logic [63:0] bp_update_pc;               // 更新PC
  logic [31:0] bp_update_instr;            // 更新指令
  logic        bp_actual_taken;            // 实际分支结果
  logic [63:0] bp_actual_target;           // 实际目标地址
  
  // 新的分支预测错误检测机制
  // 存储每条指令的预测信息，用于后续比较
  typedef struct packed {
    logic        valid;
    logic [63:0] pc;
    logic        pred_taken;
    logic [63:0] pred_target;
  } prediction_entry_t;
  
  // 使用小的预测信息缓存（考虑到流水线深度）
  parameter PRED_CACHE_SIZE = 8;
  prediction_entry_t pred_cache [PRED_CACHE_SIZE-1:0];
  logic [$clog2(PRED_CACHE_SIZE)-1:0] pred_wr_ptr, pred_rd_ptr;
  
  // 分支预测失败检测和处理
  logic        branch_mispred;             // 分支预测失败
  logic        direction_mispred;          // 方向预测错误
  logic        target_mispred;             // 目标地址预测错误
  logic [63:0] correct_pc;                 // 正确的PC值
  logic        pipeline_flush;             // 流水线冲刷信号
  
  // 分支预测控制
  logic        bp_enable;                  // 分支预测使能信号
  
  // 原子指令支持
  // Reservation Set管理 - 简单实现，最多支持2个地址
  parameter RESERVATION_SET_SIZE = 2;
  reservation_t reservation_set [RESERVATION_SET_SIZE-1:0];
  logic         atomic_in_progress;        // 原子操作进行中标志
  logic [63:0]  atomic_pc;                 // 当前原子操作的PC
  logic [31:0]  atomic_instr;              // 当前原子操作的指令
  
  // 检测原子操作是否在进行中
  // 当EX阶段识别到原子指令时设置，当MEM阶段完成原子操作时清除
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      atomic_in_progress <= 1'b0;
      atomic_pc <= 64'b0;
      atomic_instr <= 32'b0;
    end else begin
      // 原子指令开始执行
      if (mem_is_atomic && ex_finish) begin
        atomic_in_progress <= 1'b1;
        atomic_pc <= ex_pc;
        atomic_instr <= ex_instr;
      end
      // 原子指令执行完成
      else if (atomic_in_progress && mem_finish) begin
        atomic_in_progress <= 1'b0;
        atomic_pc <= 64'b0;
        atomic_instr <= 32'b0;
      end
    end
  end
  
  // 分支预测使能控制（可以通过参数或条件控制）
  assign bp_enable = 1'b1;                 // 默认启用分支预测
  
  // 预测信息缓存管理
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      pred_wr_ptr <= 0;
      pred_rd_ptr <= 0;
      for (int i = 0; i < PRED_CACHE_SIZE; i++) begin
        pred_cache[i].valid <= 1'b0;
        pred_cache[i].pc <= 64'b0;
        pred_cache[i].pred_taken <= 1'b0;
        pred_cache[i].pred_target <= 64'b0;
      end
      // 初始化Reservation Set
      atomic_in_progress <= 1'b0;
      atomic_pc <= 64'b0;
      atomic_instr <= 32'b0;
      for (int i = 0; i < RESERVATION_SET_SIZE; i++) begin
        reservation_set[i].valid <= 1'b0;
        reservation_set[i].addr <= 64'b0;
      end
    end else begin
      // 存储预测信息：当IF阶段完成且进行了预测时
      // 现在if_pred_taken和if_pred_pc在if_finish时是有效的（已缓存）
      if (if_finish && (is_branch_instr(if_instr) || is_jal_instr(if_instr) || is_jalr_instr(if_instr))) begin
        pred_cache[pred_wr_ptr].valid <= 1'b1;
        pred_cache[pred_wr_ptr].pc <= if_pc;
        pred_cache[pred_wr_ptr].pred_taken <= if_pred_taken;
        pred_cache[pred_wr_ptr].pred_target <= if_pred_pc;
        pred_wr_ptr <= (pred_wr_ptr + 1) % $clog2(PRED_CACHE_SIZE)'(PRED_CACHE_SIZE);
      end
      
      // 清除已处理的预测信息
      if (bp_update_en && pred_cache[pred_rd_ptr].valid && (pred_cache[pred_rd_ptr].pc == bp_update_pc)) begin
        pred_cache[pred_rd_ptr].valid <= 1'b0;
        pred_rd_ptr <= (pred_rd_ptr + 1) % $clog2(PRED_CACHE_SIZE)'(PRED_CACHE_SIZE);
      end
      
      // Reservation Set管理逻辑
      // 当有原子操作完成时，清除相应的保留标记
      // 这里需要与MEM阶段协调，后续在MEM阶段中实现具体的管理逻辑
      
      // LR指令成功时，添加保留地址
      if (mem_update_reservation && mem_lr_success) begin
        // 查找空闲的保留集条目
        if (!reservation_set[0].valid) begin
          reservation_set[0].valid <= 1'b1;
          reservation_set[0].addr <= mem_reservation_addr;
        end else if (!reservation_set[1].valid) begin
          reservation_set[1].valid <= 1'b1;
          reservation_set[1].addr <= mem_reservation_addr;
        end else begin
          // 如果两个条目都被占用，替换第一个
          reservation_set[0].valid <= 1'b1;
          reservation_set[0].addr <= mem_reservation_addr;
        end
      end
      
      // SC指令完成或其他需要清除保留的情况
      if (mem_clear_reservation) begin
        // 清除所有保留
        for (int i = 0; i < RESERVATION_SET_SIZE; i++) begin
          reservation_set[i].valid <= 1'b0;
        end
      end
      
      // 中断或异常发生时，清除所有保留（根据RISC-V规范）
      if (exception) begin
        for (int i = 0; i < RESERVATION_SET_SIZE; i++) begin
          reservation_set[i].valid <= 1'b0;
        end
      end
    end
  end
  
  // 添加指令分类函数
  function automatic logic is_branch_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b1100011);  // BEQ, BNE, BLT, etc.
  endfunction
  
  function automatic logic is_jal_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b1101111);  // JAL
  endfunction
  
  function automatic logic is_jalr_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b1100111);  // JALR
  endfunction
  
  // 原子指令识别函数
  function automatic logic is_atomic_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b0101111);  // 原子指令opcode
  endfunction
  
  function automatic logic is_amo_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b0101111) && (inst[31:27] != 5'b00010) && (inst[31:27] != 5'b00011);
  endfunction
  
  function automatic logic is_lr_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b0101111) && (inst[31:27] == 5'b00010);  // LR
  endfunction
  
  function automatic logic is_sc_instr(input logic [31:0] inst);
    return (inst[6:0] == 7'b0101111) && (inst[31:27] == 5'b00011);  // SC
  endfunction
  
  // 分支预测错误检测逻辑
  logic stored_pred_taken;
  logic [63:0] stored_pred_target;
  
  always_comb begin
    // 默认值
    stored_pred_taken = 1'b0;
    stored_pred_target = 64'b0;
    direction_mispred = 1'b0;
    target_mispred = 1'b0;
    branch_mispred = 1'b0;
    correct_pc = 64'b0;
    pipeline_flush = 1'b0;
    
    // 只在EX阶段有分支预测更新时进行错误检测
    if (bp_update_en && pred_cache[pred_rd_ptr].valid && (pred_cache[pred_rd_ptr].pc == bp_update_pc)) begin
      // 找到对应的预测信息，进行比较
      stored_pred_taken = pred_cache[pred_rd_ptr].pred_taken;
      stored_pred_target = pred_cache[pred_rd_ptr].pred_target;
      
      // 检测方向预测错误 - 使用bp_actual_taken确保时序一致
      direction_mispred = (stored_pred_taken != bp_actual_taken);
      
      // 检测目标地址预测错误
      target_mispred = stored_pred_taken && bp_actual_taken && (stored_pred_target != bp_actual_target);
      
      // 总的预测错误
      branch_mispred = direction_mispred || target_mispred;
      
      // 计算正确的PC值
      if (bp_actual_taken) begin
        correct_pc = bp_actual_target;  // 实际taken，使用实际目标
      end else begin
        correct_pc = bp_update_pc + 4;  // 实际not taken，顺序执行
      end
      
      // 流水线冲刷信号
      pipeline_flush = branch_mispred;
    end
  end

  // 连续赋值
  assign branch_taken = ex_branch;         // 分支是否被采取
  assign branch_target = ex_branch_target; // 分支目标地址
  assign stall = (mem_exception || 
                 ((id_rs1 != 0) && (id_rs1 == mem_rd)) || 
                 ((id_rs2 != 0) && (id_rs2 == mem_rd)));  // 移除乘除法忙碌信号，改用IF阶段状态机控制  
  assign block = stall;                    // 阻塞情况


  IF_stage if_stage (
      .clk(clk),  // 使用16ps周期的时钟信号
      .reset(reset),  // 复位信号
      .pc(if_pc),  // 修改为wire，由IF_stage驱动
      .instr(if_instr),  // 保持 32 位
      .ibus_req(ireq),  // 指令总线请求
      .ibus_resp(iresp),  // 指令总线响应
      .ecall_need(ecall_need),
      .branch_taken(branch_taken),  // 分支是否被采取
      .branch_target(branch_target),  // 分支目标地址
      .stall(stall),
      .misalign(misalign),
      .finish_mem(finish_r),
      .finish_wb(finish_w_wb),
      .wb_pc(wb_pc),
      .if_finish(if_finish),
      .branch_wait(id_branch),  // 使用ID stage中的分支信号
      .csr_mtvec(csr_mtvec),
      .csr_mepc(csr_mepc),
      .csr_mstatus(csr_mstatus),
      .csr_mscratch(csr_mscratch),
      .csr_mhartid(csr_mhartid),
      .csr_medeleg(csr_medeleg),
      .priviledge_mode(priviledge_mode),
      .csr_satp(csr_satp),   // 添加SATP寄存器连接
      .mret_to_ecall_dll_valid(mret_to_ecall_dll_valid),
      .muldiv_done(muldiv_done),     // 添加乘除法完成信号
      
      // 分支预测连接
      .pred_taken(if_pred_taken),
      .pred_pc(if_pred_pc),
      .hit_ras(if_hit_ras),
      .bp_update_en(bp_update_en),
      .bp_update_pc(bp_update_pc),
      .bp_update_instr(bp_update_instr),
      .bp_actual_taken(bp_actual_taken),
      .bp_actual_target(bp_actual_target),
      
      // 流水线冲刷连接
      .pipeline_flush(pipeline_flush),
      .correct_pc(correct_pc),
      
      // 分支预测控制连接
      .bp_enable(bp_enable),
      
      // PMP 相关连接
      .csr_pmpaddr0(csr_pmpaddr0),
      .csr_pmpcfg0(csr_pmpcfg0)
  );

  // ID stage
  logic [31:0] id_instr;  // ID 阶段的 PC 和指令
  logic [63:0] id_pc;
  logic [4:0] id_rs1, id_rs2, id_rd;  // ID 阶段的寄存器地址
  logic [63:0] id_imm;  // ID 阶段的立即数
  logic [ 1:0] id_mem_size;  // ID 阶段的内存大小    
  logic
      id_reg_write,
      id_mem_read,
      id_mem_write,
      id_alu_src,
      id_mem_to_reg;  // ID 阶段的控制信号
      
  // 分支预测性能统计
  always_ff @(posedge clk)begin
      if(reset) begin
          total_branch <= 0;
          total_j <= 0;
          succ_branch <= 0;
          succ_j <= 0;
          wpc_j <= 0;
          wpc_ret <= 0;
          print_cnt <= 0;
      end else begin
          if(print_cnt[24] == 1)begin // 每隔固定的时间输出结果
              $display("=== Branch Prediction Statistics ===");
              $display("total_branch:%.2f ", total_branch);
              $display("branch success:%.2f%%", total_branch > 0 ? (succ_branch/total_branch)*100 : 0);
              $display("total_j:%.2f ", total_j);
              $display("jump success:%.2f%%", total_j > 0 ? (succ_j/total_j)*100 : 0);
              $display("jump wpc:%.2f%%", total_j > 0 ? (wpc_j/total_j)*100 : 0);
              $display("ret wpc:%.2f%%", total_j > 0 ? (wpc_ret/total_j)*100 : 0);
              $display("=====================================");
              print_cnt <= '0;	
          end else begin
              print_cnt <= print_cnt + 1;
          end
          
          // 统一的分支预测统计逻辑 - 在缓存清除之前进行统计
          if(bp_update_en && pred_cache[pred_rd_ptr].valid && (pred_cache[pred_rd_ptr].pc == bp_update_pc)) begin
              // 条件分支统计
              if(bp_update_instr[6:0] == 7'b1100011) begin  // 条件分支
                  total_branch <= total_branch + 1;
                  if(pred_cache[pred_rd_ptr].pred_taken == bp_actual_taken) begin
                      succ_branch <= succ_branch + 1;
                  end
              end
              
              // 跳转指令统计 (JAL + JALR)
              if(bp_update_instr[6:0] == 7'b1101111 || bp_update_instr[6:0] == 7'b1100111) begin
                  total_j <= total_j + 1;
                  
                  // JAL指令处理
                  if(bp_update_instr[6:0] == 7'b1101111) begin  // JAL
                      // JAL总是taken，主要看目标地址预测
                      if(pred_cache[pred_rd_ptr].pred_taken && (pred_cache[pred_rd_ptr].pred_target == bp_actual_target)) begin
                          succ_j <= succ_j + 1;
                      end
                  end else begin  // JALR
                      // JALR需要同时预测taken和目标地址
                      if((pred_cache[pred_rd_ptr].pred_taken == bp_actual_taken) && 
                         (!bp_actual_taken || (pred_cache[pred_rd_ptr].pred_target == bp_actual_target))) begin
                          succ_j <= succ_j + 1;
                      end
                  end
                  
                  // 目标地址预测错误统计
                  if(bp_actual_taken && (pred_cache[pred_rd_ptr].pred_target != bp_actual_target)) begin
                      wpc_j <= wpc_j + 1;
                      // 返回指令统计 (JALR x0, x1/x5)
                      if((bp_update_instr[6:0] == 7'b1100111) && (bp_update_instr[11:7] == 5'd0) && 
                         ((bp_update_instr[19:15] == 5'd1) || (bp_update_instr[19:15] == 5'd5))) begin
                          wpc_ret <= wpc_ret + 1;
                      end
                  end
              end
          end
      end
  end
  
  ID_stage id_stage (
      .clk(clk),  // 使用16ps周期的时钟信号
      .reset(reset),  // 复位信号
      .block(block),  // 是否阻塞
      .pipeline_flush(pipeline_flush),  // 流水线冲刷信号
      .pc_in(if_pc),  // 
      .instr_in(if_instr),  // 保持 32 位
      .pc_out(id_pc),  // ID 阶段的 PC 输出
      .instr_out(id_instr),  // ID 阶段的指令输出
      .rs1(id_rs1),  // ID 阶段的 rs1 地址
      .rs2(id_rs2),  // ID 阶段的 rs2 地址
      .rd(id_rd),  // ID 阶段的 rd 地址
      .imm(id_imm),  // ID 阶段的立即数
      .mem_size(id_mem_size),  // ID 阶段的内存大小
      .reg_write(id_reg_write),  // ID 阶段的寄存器写使能
      .mem_read(id_mem_read),  // ID 阶段的内存读使能
      .mem_write(id_mem_write),  // ID 阶段的内存写使能
      .branch(id_branch),  // ID 阶段的分支信号
      .alu_src(id_alu_src),  // ID 阶段的 ALU 源选择
      .mem_to_reg(id_mem_to_reg),  // ID 阶段的内存到寄存器信号
      .id_finish(id_finish),  // ID 阶段完成信号
      .if_finish(if_finish)  // ID 阶段完成信号
  );

  // EX stage
  logic [63:0] ex_pc;
  logic [31:0] ex_instr;
  logic [63:0] ex_result;  // EX 阶段的 PC 和结果
  logic [4:0] ex_rd;
  logic ex_mem_read;
  logic ex_mem_write;
  logic ex_mem_to_reg;
  logic ex_reg_write;
  logic [1:0] ex_mem_size;
  logic [4:0] ex_rs2;
  logic ex_finish;
  logic muldiv_busy;  // 乘除法忙碌信号
  logic muldiv_done;  // 乘除法完成信号
  EX_stage ex_stage (
      .clk(clk),  // 使用16ps周期的时钟信号
      .reset(reset),  // 复位信号
      .pc_in(id_pc),  // EX 阶段的 PC 输入
      .instr_in(id_instr),  // EX 阶段的指令输入
      .imm(id_imm),
      .rs1(id_rs1),  // 传递 id_rs1_data
      .rs2(id_rs2),  // 传递 id_rs2_data
      .rs2_out(ex_rs2),
      .pipeline_flush(pipeline_flush),  // 流水线冲刷信号
      .rd_in(id_rd),
      .pc_out(ex_pc),  // EX 阶段的 PC 输出
      .result(ex_result),  // EX 阶段的结果输出
      .block(block),  // 是否阻塞
      .id_finish(id_finish),  // ID 阶段完成信号
      .ex_finish(ex_finish),  // EX 阶段完成信号
      .regfile(regfile),  // 传递寄存器文件
      .rd_out(ex_rd),
      .valid(result_valid),
      .misalign(misalign),
      .instr_out(ex_instr),
      .mem_size_in(id_mem_size),
      .mem_size_out(ex_mem_size),
      .mem_read_in(id_mem_read),
      .mem_write_in(id_mem_write),
      .mem_to_reg_in(id_mem_to_reg),
      .reg_write_in(id_reg_write),
      .mem_read_out(ex_mem_read),
      .mem_write_out(ex_mem_write),
      .mem_to_reg_out(ex_mem_to_reg),
      .reg_write_out(ex_reg_write),
      .branch_out(ex_branch),           // 连接wire信号
      .branch_target_out(ex_branch_target), // 连接wire信号
      .csr_read_data(csr_read_data),  // 添加CSR读取数据连接
      .muldiv_busy(muldiv_busy),      // 添加乘除法忙碌信号输出
      .muldiv_done(muldiv_done),      // 添加乘除法完成信号输出
      
      // 分支预测相关信号
      .if_pred_taken(if_pred_taken),
      .if_pred_pc(if_pred_pc),
      .if_hit_ras(if_hit_ras),
      .bp_update_en(bp_update_en),
      .bp_update_pc(bp_update_pc),
      .bp_update_instr(bp_update_instr),
      .bp_actual_taken(bp_actual_taken),
      .bp_actual_target(bp_actual_target),
      
      // 原子指令相关信号
          .atomic_in_progress(atomic_in_progress),
    .is_atomic_out(mem_is_atomic),
    .atomic_op_out(mem_atomic_op)
  );

  // MEM stage
  logic [63:0] mem_result,mem_ex_out;  // MEM 阶段的 PC 和结果
  logic [63:0] mem_pc;
  logic [31:0] mem_instr;
  logic        mem_exception;  // MEM 阶段的异常信号
  logic [63:0] op;
  logic [4:0] mem_rd;
  logic mem_reg_write;
  logic mem_mem_to_reg;
  logic mem_ex_result_valid;
  logic mem_mem_read;
  logic mem_mem_write;
  logic [63:0] mem_addr_out;
  logic mem_finish;
  
  // 原子指令相关信号
  logic mem_is_atomic;
  atomic_op_t mem_atomic_op;
  logic mem_lr_success;
  logic mem_sc_success;
  logic mem_update_reservation;
  addr_t mem_reservation_addr;
  logic mem_clear_reservation;

  MEM_stage mem_stage (
      .clk(clk),  // 使用16ps周期的时钟信号
      .reset(reset),  // 复位信号
      .pc_in(ex_pc),  // MEM 阶段的 PC 输入
      .result_in(ex_result),  // MEM 阶段的结果输入
      .ex_result_out(mem_ex_out),
      .mem_read(ex_mem_read),  // MEM 阶段的内存读使能
      .mem_write(ex_mem_write),  // MEM 阶段的内存写使能
      .write_data(regfile[ex_rs2]),  // 扩展为 64 位
      .mem_size(ex_mem_size),  // 假设立即数的低两位表示内存访问大小
      .pc_out(mem_pc),  // MEM 阶段的 PC 输出，截断为 32 位
      .result_out(mem_result),  // MEM 阶段的结果输出
      .instr_in(ex_instr),  // MEM 阶段的指令输入
      .dbus_req(dreq),  // 数据总线请求
      .dbus_resp(dresp),  // 数据总线响应
      .mem_exception(mem_exception),  // MEM 阶段的异常信号
      .block(block),  // 是否阻塞
      .finish_w(finish_w_mem),  // MEM 阶段完成信号
      .finish_r(finish_r),  // MEM 阶段读完成信号 - 作为输出
      .rd_in(ex_rd),
      .rd_out(mem_rd),
      .reg_write_in(ex_reg_write),
      .reg_write_out(mem_reg_write),
      .mem_to_reg_in(ex_mem_to_reg),
      .mem_to_reg_out(mem_mem_to_reg),
      .ex_result_valid(result_valid),
      .mem_result_valid(mem_ex_result_valid),
      .instr_out(mem_instr),
      .mem_read_out(mem_mem_read),
      .mem_write_out(mem_mem_write),
      .mem_addr_out(mem_addr_out),
      .ex_finish(ex_finish),
      .mem_finish(mem_finish),
      .misalign(misalign),
      .is_atomic(mem_is_atomic),
      .atomic_op(mem_atomic_op),
      .reservation_set(reservation_set),
      .lr_success(mem_lr_success),
      .sc_success(mem_sc_success),
      .update_reservation(mem_update_reservation),
      .reservation_addr(mem_reservation_addr),
      .clear_reservation(mem_clear_reservation),
      .csr_pmpaddr0(csr_pmpaddr0),
      .csr_pmpcfg0(csr_pmpcfg0),
      .privilege_mode(priviledge_mode)
  );

  // WB stage
  logic [63:0] wb_pc, wb_result;  // WB 阶段的 PC 和结果
  logic [31:0] wb_instr;
  logic wb_reg_write;
  logic wb_mem_read;
  logic wb_mem_write;
  logic wb_mem_exception;
  logic [63:0] wb_addr_out;
  logic [4:0] wb_rd;
  WB_stage wb_stage (
      .clk(clk),  // 使用16ps周期的时钟信号
      .instr_in(mem_instr),  // WB 阶段的指令输入
      .reset(reset),  // 复位信号
      .pc_in(mem_pc),  // WB 阶段的 PC 输入，截断为 32 位
      .result_in(mem_ex_out),  // WB 阶段的结果输入
      .mem_data_in(mem_result),  // 数据总线响应数据
      .rd(mem_rd),  // WB 阶段的 rd 地址
      .reg_write(mem_reg_write),  // WB 阶段的寄存器写使能
      .mem_to_reg(mem_mem_to_reg),  // WB 阶段的内存到寄存器信号
      .mem_exception(mem_exception),  // WB 阶段的异常信号
      .trint(trint),  // 添加定时器中断
      .swint(swint),  // 添加软件中断
      .exint(exint),  // 添加外部中断
      .atomic_in_progress(atomic_in_progress), // 添加原子操作进行中标志
      .pc_out(wb_pc),  // WB 阶段的 PC 输出
      .result_out(wb_result),  // WB 阶段的结果输出
      .regfile(regfile),  // 寄存器文件
      .csr_mstatus(csr_mstatus),
      .csr_mtvec(csr_mtvec),
      .csr_mepc(csr_mepc),
      .csr_mcause(csr_mcause),
      .csr_mtval(csr_mtval),
      .csr_mscratch(csr_mscratch),
      .csr_mhartid(csr_mhartid),
      .csr_medeleg(csr_medeleg),
      .csr_mideleg(csr_mideleg),
      .csr_mie(csr_mie),
      .csr_mip(csr_mip),
      .csr_sstatus(csr_sstatus),
      .csr_stvec(csr_stvec),
      .csr_sepc(csr_sepc),
      .csr_scause(csr_scause),
      .csr_stval(csr_stval),
      .csr_sscratch(csr_sscratch),
      .csr_sie(csr_sie),
      .csr_sip(csr_sip),
      .csr_satp(satp),
      .csr_misa(csr_misa),
      .csr_mvendorid(csr_mvendorid),
      .csr_marchid(csr_marchid),
      .csr_mimpid(csr_mimpid),
      .csr_custom_csrs(csr_custom_csrs),
      .csr_pmpaddr0(csr_pmpaddr0),
      .csr_pmpcfg0(csr_pmpcfg0),
      .block(block),  // 是否阻塞
      .finish_w(finish_w_wb),  // WB 阶段完成信号
      .csr_read_data(csr_read_data),  // 添加CSR读取数据
      .ex_instr_in(ex_instr),
      .mcycle(mcycle),  // 添加时钟周期计数器
      .minstret(minstret),  // 添加指令计数器
      .privilege_mode(priviledge_mode),  // 添加当前特权模式
      .exception(exception),  // 添加异常信号
      .result_valid(mem_ex_result_valid),
      .instr_out(wb_instr),
      .reg_write_out(wb_reg_write),
      .valid(valid),
      .mem_read_out(wb_mem_read),
      .mem_write_out(wb_mem_write),
      .mem_read_in(mem_mem_read),
      .mem_write_in(mem_mem_write),
      .mem_exception_out(wb_mem_exception),
      .mem_addr_out(wb_addr_out),
      .mem_addr_in(mem_addr_out),
      .wb_rd(wb_rd),
      .skip(skip),
      .mem_finish(mem_finish),
      .misalign(misalign),
      .ecall_need(ecall_need),
      .mret_to_ecall_dll_valid(mret_to_ecall_dll_valid),
      .MMU_exception(MMU_exception)
  );

//   // 添加标志位以跟踪是否已经执行过1dfc地址
//   logic passed_1dfc;

//   // 初始化标志位
//   initial begin
//     passed_1dfc = 1'b0;
//   end

//   always_ff @(posedge clk) begin
//     if (valid) begin
//       if (wb_pc == 64'h0000000080001dfc) begin
//         passed_1dfc <= 1'b1;
//       end
      
//       if (passed_1dfc || wb_pc == 64'h0000000080001dfc) begin
//         $display("--------------------------------");
//         $display("pc: %h", wb_pc);
//         $display("instr: %h", wb_instr);
//         $display("result: %h", wb_result);
//         $display("rd: %d", wb_rd);
//         $display("priviledge_mode: %d", priviledge_mode);
//         $display("mstatus: %h", csr_mstatus);
//         $display("mepc: %h", csr_mepc);
//         $display("mtvec: %h", csr_mtvec);
//         $display("mip: %h", csr_mip);
//         $display("mie: %h", csr_mie);
//         $display("satp: %h", csr_satp);
//         $display("mscratch: %h", csr_mscratch);
//         $display("--------------------------------");      
//       end
//     end
//   end

`ifdef VERILATOR
  
  DifftestInstrCommit DifftestInstrCommit (
      .clock (clk),   
      .coreid(csr_mhartid[7:0]),      // 核心 ID
      .index (0),      // 索引
      .valid (valid),  // 指令提交有效信号
      .pc    (wb_pc),  // 保持 64 位
      .instr   (wb_instr),       // 保持 32 位
      .skip    (skip),          // 跳过信号
      .isRVC   (0),              // 是否为 RVC 指令
      .scFailed(0),              // SC 失败信号
      .wen     (wb_reg_write),   // 写使能信号
      .wdest   ({3'b0, wb_rd}),  // 写目标地址
      .wdata   (wb_result)       // 写数据
  );

  DifftestArchIntRegState DifftestArchIntRegState (
      .clock (clk),          // 使用16ps周期的时钟信号
      .coreid(csr_mhartid[7:0]),            // 核心 ID
      .gpr_0 (regfile[0]),   // 通用寄存器 0
      .gpr_1 (regfile[1]),   // 通用寄存器 1
      .gpr_2 (regfile[2]),   // 通用寄存器 2
      .gpr_3 (regfile[3]),   // 通用寄存器 3
      .gpr_4 (regfile[4]),   // 通用寄存器 4
      .gpr_5 (regfile[5]),   // 通用寄存器 5
      .gpr_6 (regfile[6]),   // 通用寄存器 6
      .gpr_7 (regfile[7]),   // 通用寄存器 7
      .gpr_8 (regfile[8]),   // 通用寄存器 8
      .gpr_9 (regfile[9]),   // 通用寄存器 9
      .gpr_10(regfile[10]),  // 通用寄存器 10
      .gpr_11(regfile[11]),  // 通用寄存器 11
      .gpr_12(regfile[12]),  // 通用寄存器 12
      .gpr_13(regfile[13]),  // 通用寄存器 13
      .gpr_14(regfile[14]),  // 通用寄存器 14
      .gpr_15(regfile[15]),  // 通用寄存器 15
      .gpr_16(regfile[16]),  // 通用寄存器 16
      .gpr_17(regfile[17]),  // 通用寄存器 17
      .gpr_18(regfile[18]),  // 通用寄存器 18
      .gpr_19(regfile[19]),  // 通用寄存器 19
      .gpr_20(regfile[20]),  // 通用寄存器 20
      .gpr_21(regfile[21]),  // 通用寄存器 21
      .gpr_22(regfile[22]),  // 通用寄存器 22
      .gpr_23(regfile[23]),  // 通用寄存器 23
      .gpr_24(regfile[24]),  // 通用寄存器 24
      .gpr_25(regfile[25]),  // 通用寄存器 25
      .gpr_26(regfile[26]),  // 通用寄存器 26
      .gpr_27(regfile[27]),  // 通用寄存器 27
      .gpr_28(regfile[28]),  // 通用寄存器 28
      .gpr_29(regfile[29]),  // 通用寄存器 29
      .gpr_30(regfile[30]),  // 通用寄存器 30
      .gpr_31(regfile[31])   // 通用寄存器 31
  );

  DifftestTrapEvent DifftestTrapEvent (
      .clock   (clk),                     // 使用16ps周期的时钟信号
      .coreid  (csr_mhartid[7:0]),                       // 核心 ID
      .valid   (),               // 有效信号
      .code    (),  // 代码
      .pc      (),                  // 程序计数器
      .cycleCnt(0),                       // 周期计数
      .instrCnt(0)                        // 指令计数
  );

  DifftestCSRState DifftestCSRState (
      .clock         (clk),                                      // 时钟信号
      .coreid        (csr_mhartid[7:0]),                                        // 核心 ID
      .priviledgeMode(priviledge_mode),                                        // 特权模式
      .mstatus       (csr_mstatus),                              // mstatus 寄存器
      .sstatus       (),       // sstatus 寄存器
      .mepc          (csr_mepc),                                 // mepc 寄存器
      .sepc          (csr_sepc),                                 // sepc 寄存器
      .mtval         (csr_mtval),                                // mtval 寄存器
      .stval         (csr_stval),                                // stval 寄存器
      .mtvec         (csr_mtvec),                                // mtvec 寄存器
      .mip           (csr_mip),                                  // mip 寄存器
      .mie           (csr_mie),                                  // mie 寄存器
      .mscratch      (csr_mscratch),                             // mscratch 寄存器
      .sscratch      (),                             // sscratch 寄存器
      .mideleg       (),                              // mideleg 寄存器
      .medeleg       (),                              // medeleg 寄存器
      .stvec         (),                                // stvec 寄存器
      .mcause        (csr_mcause),                               // mcause 寄存器
      .scause        (),                               // scause 寄存器
      .satp          (satp)                                  // satp 寄存器
  );
  
`endif
endmodule  // 结束 core 模块
`endif  // 结束 __CORE_SV 宏定义