`ifndef __BRANCH_PREDICTOR_SV
`define __BRANCH_PREDICTOR_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif

// 分支预测器模块
// 实现2位饱和计数器 + 分支目标缓冲区(BTB) + 返回地址栈(RAS)
module branch_predictor
  import common::*;
#(
    parameter int BHT_SIZE = 1024,  // 分支历史表大小
    parameter int RAS_SIZE = 16,    // 返回地址栈大小
    parameter int BTB_SIZE = 256    // 分支目标缓冲区大小
)(
    input  logic clk,
    input  logic reset,
    
    // 预测接口 - 需要PC和指令内容
    input  logic [63:0] pc,           // 当前PC
    input  logic [31:0] instr,        // 当前指令内容
    input  logic        instr_valid,  // 指令有效信号
    output logic        pred_taken,   // 预测分支是否taken
    output logic [63:0] pred_target,  // 预测目标地址
    output logic        hit_ras,      // 是否命中RAS
    
    // 更新接口 - 基于实际执行结果进行更新
    input  logic        update_en,    // 更新使能
    input  logic [63:0] update_pc,    // 更新的PC
    input  logic [31:0] update_instr, // 更新的指令
    input  logic        actual_taken, // 实际分支结果
    input  logic [63:0] actual_target // 实际目标地址
);

    // 分支历史表 - 2位饱和计数器
    logic [1:0] bht [BHT_SIZE-1:0];
    
    // 分支目标缓冲区 (BTB)
    typedef struct packed {
        logic        valid;
        logic [63:0] pc;
        logic [63:0] target;
        logic [2:0]  instr_type;  // 0=BRANCH, 1=JAL, 2=JALR, 3=CALL, 4=RET
    } btb_entry_t;
    
    btb_entry_t btb [BTB_SIZE-1:0];
    
    // 返回地址栈
    logic [63:0] ras [RAS_SIZE-1:0];
    logic [$clog2(RAS_SIZE)-1:0] ras_top;
    
    // 内部信号
    logic [9:0] bht_index;
    logic [7:0] btb_index;
    logic [9:0] update_bht_index;
    logic [7:0] update_btb_index;
    logic btb_hit;
    
    // 指令分类信号
    logic is_branch, is_jal, is_jalr, is_call, is_ret;
    logic [2:0] instr_type;
    
    // 当前指令分类函数
    function automatic logic is_branch_instr(input logic [31:0] inst);
        return (inst[6:0] == 7'b1100011);  // BEQ, BNE, BLT, etc.
    endfunction
    
    function automatic logic is_jal_instr(input logic [31:0] inst);
        return (inst[6:0] == 7'b1101111);  // JAL
    endfunction
    
    function automatic logic is_jalr_instr(input logic [31:0] inst);
        return (inst[6:0] == 7'b1100111);  // JALR
    endfunction
    
    function automatic logic is_call_instr(input logic [31:0] inst);
        logic is_jal_call = is_jal_instr(inst) && (inst[11:7] != 5'd0);
        logic is_jalr_call = is_jalr_instr(inst) && ((inst[11:7] == 5'd1) || (inst[11:7] == 5'd5));
        return is_jal_call || is_jalr_call;
    endfunction
    
    function automatic logic is_ret_instr(input logic [31:0] inst);
        return is_jalr_instr(inst) && (inst[11:7] == 5'd0) && 
               ((inst[19:15] == 5'd1) || (inst[19:15] == 5'd5));
    endfunction
    
    // 预测逻辑 - 必须基于有效的指令内容进行预测
    always_comb begin
        // 计算索引
        bht_index = pc[11:2];
        btb_index = pc[9:2];
        
        // 检查BTB命中
        btb_hit = btb[btb_index].valid && (btb[btb_index].pc == pc);
        
        // 默认预测：不跳转
        pred_taken = 1'b0;
        pred_target = pc + 4;
        hit_ras = 1'b0;
        
        // 只有在指令有效且确实是分支/跳转指令时才进行预测
        if (instr_valid && (is_branch_instr(instr) || is_jal_instr(instr) || is_jalr_instr(instr))) begin
            if (btb_hit) begin
                // BTB命中，根据指令类型进行预测
                case (btb[btb_index].instr_type)
                    3'd0: begin  // BRANCH - 使用BHT预测
                        pred_taken = bht[bht_index][1];  // 2位计数器的高位
                        pred_target = pred_taken ? btb[btb_index].target : (pc + 4);
                    end
                    3'd1, 3'd3: begin  // JAL, CALL - 总是taken
                        pred_taken = 1'b1;
                        pred_target = btb[btb_index].target;
                    end
                    3'd2: begin  // JALR - 使用BHT预测
                        pred_taken = bht[bht_index][1];
                        pred_target = pred_taken ? btb[btb_index].target : (pc + 4);
                    end
                    3'd4: begin  // RET - 从RAS预测
                        if (ras_top > 0) begin
                            pred_taken = 1'b1;
                            pred_target = ras[ras_top - 1];
                            hit_ras = 1'b1;
                        end else begin
                            pred_taken = 1'b0;
                            pred_target = pc + 4;
                        end
                    end
                    default: begin
                        pred_taken = 1'b0;
                        pred_target = pc + 4;
                    end
                endcase
            end else begin
                                 // BTB未命中，但指令确实是分支/跳转指令
                 // 进行保守预测
                 if (is_jal_instr(instr)) begin
                     // JAL总是跳转，可以直接计算目标地址
                     pred_taken = 1'b1;
                     pred_target = pc + {{44{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
                 end else if (is_branch_instr(instr)) begin
                     // 条件分支：保守预测不跳转
                     pred_taken = 1'b0;
                     pred_target = pc + 4;
                 end else if (is_jalr_instr(instr)) begin
                     // JALR很难预测，保守预测不跳转
                     pred_taken = 1'b0;
                     pred_target = pc + 4;
                 end
            end
        end
    end
    
    // 更新逻辑 - 基于实际执行的指令
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // 初始化BHT
            for (int i = 0; i < BHT_SIZE; i++) begin
                bht[i] <= 2'b01;  // 初始化为弱不跳转
            end
            
            // 初始化BTB
            for (int i = 0; i < BTB_SIZE; i++) begin
                btb[i].valid <= 1'b0;
                btb[i].pc <= 64'b0;
                btb[i].target <= 64'b0;
                btb[i].instr_type <= 3'b0;
            end
            
            // 初始化RAS
            for (int i = 0; i < RAS_SIZE; i++) begin
                ras[i] <= 64'b0;
            end
            ras_top <= 0;
        end else if (update_en) begin
            // 计算更新索引
            update_bht_index = update_pc[11:2];
            update_btb_index = update_pc[9:2];
            
            // 基于指令内容进行精确分类
            is_branch = (update_instr[6:0] == 7'b1100011);  // BEQ, BNE, BLT, etc.
            is_jal    = (update_instr[6:0] == 7'b1101111);  // JAL
            is_jalr   = (update_instr[6:0] == 7'b1100111);  // JALR
            is_call   = (is_jal && (update_instr[11:7] != 5'd0)) ||  // JAL rd, offset (rd != x0)
                        (is_jalr && ((update_instr[11:7] == 5'd1) || (update_instr[11:7] == 5'd5))); // JALR x1/x5
            is_ret    = (is_jalr && (update_instr[11:7] == 5'd0) && 
                        ((update_instr[19:15] == 5'd1) || (update_instr[19:15] == 5'd5))); // JALR x0, x1/x5
            
            // 确定指令类型
            if (is_ret) instr_type = 3'd4;
            else if (is_call) instr_type = 3'd3;
            else if (is_jalr) instr_type = 3'd2;
            else if (is_jal) instr_type = 3'd1;
            else if (is_branch) instr_type = 3'd0;
            else instr_type = 3'd0;  // 默认为分支类型
            
            // 只为分支和跳转指令建立BTB条目
            if (is_branch || is_jal || is_jalr) begin
                // 更新BTB
                btb[update_btb_index].valid <= 1'b1;
                btb[update_btb_index].pc <= update_pc;
                btb[update_btb_index].target <= actual_taken ? actual_target : (update_pc + 4);
                btb[update_btb_index].instr_type <= instr_type;
                
                // 更新BHT（对于条件分支和JALR）
                if (is_branch || is_jalr) begin
                    if (actual_taken) begin
                        // 分支taken，增加计数器
                        if (bht[update_bht_index] != 2'b11) begin
                            bht[update_bht_index] <= bht[update_bht_index] + 1;
                        end
                    end else begin
                        // 分支not taken，减少计数器
                        if (bht[update_bht_index] != 2'b00) begin
                            bht[update_bht_index] <= bht[update_bht_index] - 1;
                        end
                    end
                end
                
                // 更新RAS
                if (is_call && actual_taken) begin
                    // 函数调用：压栈返回地址
                    /* verilator lint_off WIDTHEXPAND */
                    if (ras_top < RAS_SIZE) begin
                    /* verilator lint_on WIDTHEXPAND */
                        ras[ras_top] <= update_pc + 4;
                        ras_top <= ras_top + 1;
                    end else begin
                        // RAS满了，循环覆盖
                        for (int i = 0; i < RAS_SIZE-1; i++) begin
                            ras[i] <= ras[i+1];
                        end
                        ras[RAS_SIZE-1] <= update_pc + 4;
                    end
                end else if (is_ret && actual_taken) begin
                    // 函数返回：出栈
                    if (ras_top > 0) begin
                        ras_top <= ras_top - 1;
                    end
                end
            end
        end
    end

endmodule

`endif 