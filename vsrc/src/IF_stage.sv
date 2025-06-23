`ifndef __IF_STAGE_SV
`define __IF_STAGE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr.sv"
`include "src/branch_predictor.sv"
`else

`endif

// 取指阶段模块
// 该模块负责从内存中获取指令并更新程序计数器(PC)
// 支持流水线暂停、分支跳转和基本内存操作
module IF_stage
  import common::*;
  import csr_pkg::*;
(
    input  logic              clk,
    input  logic              reset,
    output logic       [63:0] pc,             // 程序计数器
    output logic       [31:0] instr,          // 当前指令
    output ibus_req_t         ibus_req,       // 指令总线请求
    input  ibus_resp_t        ibus_resp,      // 指令总线响应
    input  logic              branch_taken,   // 分支跳转信号
    input  logic       [63:0] branch_target,  // 分支跳转目标地址
    input  logic              stall,          // 流水线暂停信号
    input logic              finish_mem,     // 完成信号
    input logic              finish_wb,      // 完成信号
    input logic [63:0]       wb_pc,
    output logic              if_finish,      // PC更新完成信号
    output Misalign          misalign,       // 异常信号
    input logic              branch_wait,     // 分支等待信号
    input logic [63:0] csr_mtvec,
    input logic [63:0] csr_mepc,
    input logic [63:0] csr_mstatus,
    input logic [63:0] csr_mscratch,
    input logic [63:0] csr_mhartid,
    input logic [63:0] csr_medeleg,
    input logic [1:0] priviledge_mode,
    input logic [63:0] csr_satp,
    output logic ecall_need,
    output logic mret_to_ecall_dll_valid,
    input logic muldiv_done,            // 乘除法完成信号
    
    // 分支预测相关接口
    output logic        pred_taken,     // 预测分支是否taken
    output logic [63:0] pred_pc,       // 预测目标地址  
    output logic        hit_ras,       // 是否命中RAS
    input  logic        bp_update_en,  // 分支预测器更新使能
    input  logic [63:0] bp_update_pc,  // 更新PC
    input  logic [31:0] bp_update_instr, // 更新指令
    input  logic        bp_actual_taken, // 实际分支结果
    input  logic [63:0] bp_actual_target, // 实际目标地址
    
    // 流水线冲刷接口
    input  logic        pipeline_flush,    // 流水线冲刷信号
    input  logic [63:0] correct_pc,        // 正确的PC值
    
    // 分支预测控制
    input  logic        bp_enable,         // 分支预测使能信号
    
    // PMP 相关输入
    input  word_t       csr_pmpaddr0,
    input  word_t       csr_pmpcfg0
);

  logic [31:0] instruction = 32'b0;         // 初始化instruction
  logic [63:0] current_pc, next_pc;         // 移除初始值赋值，避免多重驱动
  logic valid_counter;                       // 移除初始值赋值，避免多重驱动
  logic [3:0] temp_counter;                  // 移除初始值赋值，避免多重驱动
  logic branch_taken_temp;                   // 移除初始值赋值，避免多重驱动
  
  // 分支预测器信号
  logic        bp_pred_taken;     // 分支预测器预测taken
  logic [63:0] bp_pred_target;    // 分支预测器预测目标
  logic        bp_hit_ras;        // RAS命中
  logic        use_prediction;    // 是否使用预测
  
  // 预测信息缓存 - 解决时序问题
  logic        cached_pred_taken;   // 缓存的预测taken信号
  logic [63:0] cached_pred_target;  // 缓存的预测目标地址
  logic        cached_hit_ras;      // 缓存的RAS命中信号
  logic        pred_info_valid;     // 预测信息有效标志
  logic        if_finish_prev;      // 上一周期的if_finish，用于清除预测信息
  
  // 流水线冲刷状态锁存
  logic        flush_pending;     // 冲刷待处理标志
  logic [63:0] flush_target_pc;   // 冲刷目标PC

  // 乘除法指令检测函数
  function automatic logic is_muldiv_instr(input logic [31:0] instr);
    // 检测是否为M扩展指令（R-type且funct7=0000001）
    return (instr[6:0] == 7'b0110011 || instr[6:0] == 7'b0111011) && 
           (instr[31:25] == 7'b0000001);
  endfunction

  // PMP 执行权限检查函数
  function automatic pmp_check_result_t pmp_check_exec(
    input logic [63:0] addr
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
      result.allowed = (priviledge_mode == 2'b11);
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
      // 检查执行权限
      perm_check = cfg[2];  // X位
      result.allowed = perm_check;
      result.fault = !perm_check;
    end else begin
      // 不匹配的情况下，机器模式默认允许
      result.allowed = (priviledge_mode == 2'b11);
      result.fault = 1'b0;
    end
    
    return result;
  endfunction

  // MMU地址转换信号
  logic [63:0] vaddr;                        // 虚拟地址
  logic [63:0] paddr;                        // 物理地址
  logic [63:0] pte_addr;                     // 页表项地址
  logic [63:0] pte_data;                     // 页表项数据
  logic [63:0] page_table_base;              // 页表基址

  // 取指状态机定义
  enum logic [3:0] {
    IDLE,               // 空闲状态，准备取指
    COMMON_REQUEST_SENT,       // 已发送取指请求，等待响应
    JUMP_WAIT,               // PC更新临时状态
    MEM_WAIT,
    MMU_WAIT,
    MRET_TRAP,
    ECALL_TRAP,
    TRAP,
    MERT_TO_ECALL_DLL,
    MULDIV_WAIT,        // 乘除法等待状态
    FLUSH_WAIT ,         // 流水线冲刷等待状态
    MSTATUS_WAIT
  } state;  // 移除初始值赋值，避免多重驱动

  // 分支预测器实例
  branch_predictor bp_inst (
    .clk(clk),
    .reset(reset),
    .pc(current_pc),              // 使用当前PC进行预测
    .instr(ibus_resp.data),       // 当前指令内容
    .instr_valid(ibus_resp.data_ok), // 指令有效信号
    .pred_taken(bp_pred_taken),
    .pred_target(bp_pred_target),
    .hit_ras(bp_hit_ras),
    .update_en(bp_update_en),
    .update_pc(bp_update_pc),
    .update_instr(bp_update_instr),
    .actual_taken(bp_actual_taken),
    .actual_target(bp_actual_target)
  );
  
  // 输出分支预测结果 - 使用缓存的预测信息
  assign pred_taken = pred_info_valid ? cached_pred_taken : 1'b0;
  assign pred_pc = pred_info_valid ? cached_pred_target : 64'b0;
  assign hit_ras = pred_info_valid ? cached_hit_ras : 1'b0;

  // 下一个PC计算逻辑
  logic [63:0] next_pc_calc;
  always_comb begin
    // 默认顺序执行
    next_pc_calc = current_pc + 4;
  end

  // 内部寄存器初始化仅在always_ff块内进行
  // 移除initial块避免多重驱动

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
              // 删除调试信息
      instruction <= 32'b0;
      current_pc  <= PCINIT;
      next_pc <= PCINIT + 4;  // 使用非阻塞赋值
      ibus_req.valid <= 1'b1;
      ibus_req.addr  <= PCINIT;
      state          <= COMMON_REQUEST_SENT;
      valid_counter  <= 1'b0;  // 修改为显式指定位宽
      temp_counter   <= 4'b0;  // 修改为显式指定位宽
      branch_taken_temp <= 1'b0;  // 添加初始化
      // 初始化输出信号
      pc <= 64'b0;
      instr <= 32'b0;
      if_finish <= 1'b0;
      misalign.valid <= 1'b0;
      misalign.addr <= 64'b0;
      misalign.misalign_type <= DEFAULT;
      misalign.pc <= 64'b0;
      ecall_need <= 1'b0;
      mret_to_ecall_dll_valid <= 1'b0;
      // 初始化冲刷相关信号
      flush_pending <= 1'b0;
      flush_target_pc <= 64'b0;
      // 初始化预测信息缓存
      cached_pred_taken <= 1'b0;
      cached_pred_target <= 64'b0;
      cached_hit_ras <= 1'b0;
      pred_info_valid <= 1'b0;
      if_finish_prev <= 1'b0;
          end else begin
        // 更新if_finish_prev并清除过期的预测信息
        if_finish_prev <= if_finish;
        if (if_finish_prev && !stall) begin
          // 上一周期if_finish为高且当前周期不是stall，清除预测信息
          pred_info_valid <= 1'b0;
        end
        
        // 优先处理流水线冲刷
        if (pipeline_flush && !flush_pending) begin
        // 新的冲刷请求：锁存冲刷信息
        // 删除调试信息
        flush_pending <= 1'b1;
        flush_target_pc <= correct_pc;
        
        // 立即清空当前指令输出，防止错误指令被后续阶段使用
        instruction <= 32'b0;
        pc <= 64'b0;  // 清空PC输出
        instr <= 32'b0;
        if_finish <= 1'b0;  // 冲刷后需要重新取指
        pred_info_valid <= 1'b0;  // 清除预测信息
        
        // 根据当前状态决定如何处理冲刷
        if (state == COMMON_REQUEST_SENT && ibus_req.valid) begin
          // 如果正在等待ibus响应，不能立即修改请求
          // 进入FLUSH_WAIT状态，等待当前响应完成
          state <= FLUSH_WAIT;
        end else begin
          // 如果没有待处理的ibus请求，直接发送新请求
          current_pc <= correct_pc;
          ibus_req.valid <= 1'b1;
          ibus_req.addr <= correct_pc;
          state <= COMMON_REQUEST_SENT;
          flush_pending <= 1'b0;  // 冲刷完成
        end
      end else if (stall) begin
        // 在stall时完全暂停取指，保持所有状态不变
        // 不更新PC，不发送新的指令请求，不改变状态机状态
        // 保持if_finish为低，确保后续阶段也暂停
        if_finish <= 1'b0;
      end else begin
        // 在任何状态下都要检查是否有待处理的冲刷
        if (flush_pending && state != FLUSH_WAIT) begin
          // 有待处理的冲刷，但不在FLUSH_WAIT状态
          // 立即处理冲刷（适用于非REQUEST_SENT状态）
          instruction <= 32'b0;
          pc <= flush_target_pc;
          instr <= 32'b0;
          current_pc <= flush_target_pc;
          ibus_req.valid <= 1'b1;
          ibus_req.addr <= flush_target_pc;
          state <= COMMON_REQUEST_SENT;
          flush_pending <= 1'b0;
          if_finish <= 1'b0;
          pred_info_valid <= 1'b0;  // 清除预测信息
        end else begin
          unique case (state)
          COMMON_REQUEST_SENT: begin
            // 如果没有待处理的指令请求，先发送取指请求
            if (!ibus_req.valid) begin
              ibus_req.valid <= 1'b1;
              ibus_req.addr <= current_pc;
            end
            
            if (branch_wait && !bp_enable) begin
              state <= JUMP_WAIT;
              if_finish <= 1'b0;
            end else begin
              if (ibus_resp.data_ok) begin
                // if(ecall_need) begin
                //     state <= ECALL_TRAP;
                //     current_pc <= csr_mtvec;
                //     ecall_need <= 1'b0;
                //     //mret_to_ecall_dll_valid <= 1'b1;
                // end
                
                // 缓存分支预测信息 - 在data_ok时锁存预测信息
                cached_pred_taken <= bp_pred_taken;
                cached_pred_target <= bp_pred_target;
                cached_hit_ras <= bp_hit_ras;
                pred_info_valid <= 1'b1;
                
                // 检查是否有待处理的冲刷请求
                if (flush_pending) begin
                  // 有冲刷请求：丢弃当前响应，跳转到冲刷目标
                  ibus_req.valid <= 1'b1;
                  ibus_req.addr <= flush_target_pc;
                  current_pc <= flush_target_pc;
                  flush_pending <= 1'b0;
                  
                  // 清空输出和预测信息
                  instruction <= 32'b0;
                  pc <= 64'b0;
                  instr <= 32'b0;
                  if_finish <= 1'b0;
                  pred_info_valid <= 1'b0;  // 冲刷时清除预测信息
                  // 保持在COMMON_REQUEST_SENT状态
                end 
                
                else  begin              
                instruction <= ibus_resp.data;
                pc <= current_pc;
                instr <= ibus_resp.data;
                
                // 调试信息：显示取到的指令
                // 删除调试信息
                
                // 检测mret和ecall指令
                if (ibus_resp.data == 32'h30200073) begin  // MRET指令
                  // 跳转到mepc
                  ibus_req.valid <= 1'b0;
                  current_pc <= csr_mepc;
                  state <= MRET_TRAP;
                  if_finish <= 1'b1;
                end
                else if(ibus_resp.data == 32'h00000073) begin
                  ibus_req.valid <= 1'b0;
                  current_pc <= csr_mtvec;
                  state <= ECALL_TRAP;
                  if_finish <= 1'b1;
                end  else if (is_muldiv_instr(ibus_resp.data)) begin
                  // 检测到乘除法指令，进入等待状态
                  // 删除调试信息
                  current_pc <= next_pc_calc;
                  ibus_req.valid <= 1'b0;
                  state <= MULDIV_WAIT;
                  if_finish <= 1'b1;
                end else if ((ibus_resp.data[6:0] == 7'b1110011) && 
                           (ibus_resp.data[31:20] == 12'h300)) begin  // CSR指令且操作mstatus寄存器
                  ibus_req.valid <= 1'b0;
                  current_pc <= next_pc_calc;
                  state <= MSTATUS_WAIT;
                  if_finish <= 1'b1;
                end
                else if (ibus_resp.data[6:0] == 7'b0100011 || ibus_resp.data[6:0] == 7'b0000011 || ibus_resp.data[6:0] == 7'b0101111) begin
                  // Load指令(0000011)、Store指令(0100011)和原子指令(0101111)都需要进入MEM_WAIT状态
                  current_pc <= next_pc_calc;
                  ibus_req.valid <= 1'b0;
                  ibus_req.addr <= 64'b0;
                  state <= MEM_WAIT;
                  if_finish <= 1'b1;
                end else begin
                  // 正常指令处理：检查是否为分支指令并进行预测
                  if (bp_enable && bp_pred_taken) begin
                    // 分支预测taken：跳转到预测目标
                    // 删除调试信息
                    current_pc <= bp_pred_target;
                    ibus_req.valid <= 1'b0;  // 停止当前请求
                    if_finish <= 1'b1;
                    state <= COMMON_REQUEST_SENT;
                    // 预测信息在if_finish后保持有效，供core使用
                  end else begin
                    // 分支预测not taken或分支预测禁用：顺序执行
                    // 删除调试信息
                    current_pc <= next_pc_calc;
                    ibus_req.valid <= 1'b0;  // 停止当前请求
                    if_finish <= 1'b1;
                    state <= COMMON_REQUEST_SENT;
                    // 预测信息在if_finish后保持有效，供core使用
                  end
                end
              end
              end

              else begin
                if_finish <= 1'b0;
              end
            end
          end
          

          MEM_WAIT: begin
            if(misalign.valid) begin
              state <= ECALL_TRAP;
              current_pc <= csr_mtvec;
            end else begin

              if_finish <= 1'b0;


              if(ecall_need) begin
                state <= ECALL_TRAP;
                current_pc <= csr_mtvec;
                ecall_need <= 1'b0;
              end else begin
              if (finish_mem) begin
                ibus_req.valid <= 1'b1; 
                ibus_req.addr  <= current_pc;  // 使用当前PC
                state <= COMMON_REQUEST_SENT;
              end else begin
                state <= MEM_WAIT;
              end
            end
          end
          end

          JUMP_WAIT: begin
            if (branch_taken) begin
              // 删除调试信息
              branch_taken_temp <= 1'b1;  // 使用非阻塞赋值
            end else if (branch_taken_temp) begin
              // 实际分支taken
              next_pc <= branch_target;   // 使用非阻塞赋值
              if (ibus_resp.data_ok) begin
                // 检查跳转地址是否2字节对齐
                if (branch_target[1:0] != 2'b00) begin
                  // 设置misalign异常
                  misalign.valid <= 1'b1;
                  misalign.addr <= branch_target;
                  misalign.misalign_type <= INSTR_MISALIGN;
                  state <= ECALL_TRAP;
                  current_pc <= csr_mtvec;
                  branch_taken_temp <= 1'b0;
                  misalign.pc <= branch_target;
                
                  pc <= branch_target;
                  instr <= 32'h00000000;
                  if_finish <= 1'b1;
                  ibus_req.valid <= 1'b0;
                end else begin
                  // 分支taken，跳转到目标地址
                  // 删除调试信息
                  ibus_req.valid <= 1'b1;
                  ibus_req.addr <= branch_target;
                  current_pc <= branch_target;
                  branch_taken_temp <= 1'b0;
                  state <= COMMON_REQUEST_SENT;
                end
              end
            end else begin
              // 实际分支not taken，恢复到pc+4
              ibus_req.valid <= 1'b1;
              ibus_req.addr <= pc + 4;
              current_pc <= pc + 4;
              state <= COMMON_REQUEST_SENT;
            end
          end

          MRET_TRAP:begin
            if_finish <= 1'b0;
            if (finish_wb && wb_pc == pc) begin
              state <= MERT_TO_ECALL_DLL;
            end
          end

          MSTATUS_WAIT:begin
            
            if_finish <= 1'b0;
            if (finish_wb && !ecall_need) begin
              state <= COMMON_REQUEST_SENT;
            end else if(finish_wb && ecall_need) begin
              current_pc <= csr_mtvec;
              ecall_need <= 1'b0;
              state <= COMMON_REQUEST_SENT;
            end
          end

          MERT_TO_ECALL_DLL:begin
            if(ecall_need) begin
              mret_to_ecall_dll_valid <= 1'b1;
              state <= COMMON_REQUEST_SENT;
              ibus_req.valid <= 1'b1;
              ibus_req.addr <= csr_mtvec;
              current_pc <= csr_mtvec;
              ecall_need <= 1'b0;
            end else begin       
              state <= COMMON_REQUEST_SENT;
              ibus_req.valid <= 1'b1;
              ibus_req.addr <= current_pc;  // 使用当前PC           
            end
          end

          ECALL_TRAP:begin
            if_finish <= 1'b0;
            if (wb_pc == pc) begin
              state <= COMMON_REQUEST_SENT;
              ibus_req.valid <= 1'b1;
              ibus_req.addr <= current_pc;  // 使用当前PC
            end
          end

          MULDIV_WAIT: begin
            // 在乘除法等待状态，暂停取指，等待EX阶段完成信号
            if_finish <= 1'b0;
            if (muldiv_done) begin
              // 乘除法完成，恢复取指
              ibus_req.valid <= 1'b1;
              ibus_req.addr <= current_pc;  // 使用当前PC
              state <= COMMON_REQUEST_SENT;
            end
          end

          FLUSH_WAIT: begin
            // 流水线冲刷等待状态：等待当前ibus响应完成，然后发送新请求
            if_finish <= 1'b0;
            
            if (ibus_resp.data_ok) begin
              // 收到响应，丢弃响应数据，检查目标地址对齐
              if (flush_target_pc[1:0] != 2'b00) begin
                // 设置misalign异常
                misalign.valid <= 1'b1;
                misalign.addr <= flush_target_pc;
                misalign.misalign_type <= INSTR_MISALIGN;
                misalign.pc <= flush_target_pc;
                state <= ECALL_TRAP;
                current_pc <= csr_mtvec;
                flush_pending <= 1'b0;  // 清除冲刷待处理标志
                
                pc <= flush_target_pc;
                instr <= 32'h00000000;
                if_finish <= 1'b1;
                ibus_req.valid <= 1'b0;
              end else begin
                // 地址对齐正确，发送新的正确请求
                ibus_req.valid <= 1'b1;
                ibus_req.addr <= flush_target_pc;  // 使用锁存的目标PC
                current_pc <= flush_target_pc;     // 更新当前PC
                state <= COMMON_REQUEST_SENT;
                flush_pending <= 1'b0;             // 清除冲刷待处理标志
                
                // 确保输出清空
                instruction <= 32'b0;
                pc <= 64'b0;
                instr <= 32'b0;
              end
            end
            // 在等待响应期间，保持ibus_req.valid不变，遵守总线协议
          end
      
          default: begin
            instruction <= 32'h00000000;
            ibus_req.valid <= 1'b0;
            state <= COMMON_REQUEST_SENT;
          end
        endcase
        end  // end of flush_pending check
      end
    end
  end

endmodule

`endif
