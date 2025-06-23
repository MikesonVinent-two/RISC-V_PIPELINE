`ifndef __EX_STAGE_SV
`define __EX_STAGE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr.sv"
`include "src/multiplier.sv"
`include "src/divider.sv"
`else

`endif

// EX_stage: 执行阶段模块
// 功能：执行算术逻辑运算、分支跳转判断和地址计算
module EX_stage
  import common::*;
  import csr_pkg::*;
(
    // 时钟和复位信号
    input  logic         clk,
                        reset,
    // 输入信号                    
    input  logic  [63:0] pc_in,          // 程序计数器输入
    input  logic  [31:0] instr_in,       // 指令输入
    input  logic  [63:0] imm,            // 立即数
    input  logic  [ 4:0] rs1,            // 源寄存器1
                        rs2,             // 源寄存器2
    input  logic         block,          // 阻塞信号
    input  logic         pipeline_flush, // 流水线冲刷信号
    input  logic         reg_write_in,      // 寄存器写使能
    input  logic  [ 4:0] rd_in,             // 目标寄存器
    input  logic         mem_read_in,       // 内存读使能输入
    input  logic         mem_write_in,      // 内存写使能输入
    input  logic         mem_to_reg_in,     // 内存到寄存器控制输入
    input  logic   [1:0]      mem_size_in,
    // CSR相关输入 - 获取CSR寄存器值
    output Misalign          misalign,       // 异常信号
    input  logic  [63:0] csr_read_data,      // CSR读取数据
    // 输出信号
    output logic  [ 4:0] rd_out,             // 目标寄存器
    output logic  [63:0] pc_out,         // 程序计数器输出
    output logic  [63:0] result,         // ALU结果
    input logic         id_finish,        // ID 阶段完成信号
    output logic         ex_finish,        // 写回完成信号
    input word_t        regfile  [31:0], // 寄存器文件
    output logic         valid,          // 有效信号
    output logic  [31:0] instr_out,      // 指令输出
    output logic         mem_read_out,    // 内存读使能输出
    output logic         mem_write_out,   // 内存写使能输出
    output logic         mem_to_reg_out,  // 内存到寄存器控制输出
    output logic         reg_write_out,    // 寄存器写使能输出
    output logic   [1:0]      mem_size_out,
    output logic   [4:0] rs2_out,
    output logic         branch_out,
    output logic  [63:0] branch_target_out,
    output logic         muldiv_busy,      // 乘除法忙碌信号输出
    output logic         muldiv_done,      // 乘除法完成信号输出
    
    // 分支预测相关接口
    input  logic         if_pred_taken,    // IF阶段预测taken
    input  logic [63:0]  if_pred_pc,       // IF阶段预测PC
    input  logic         if_hit_ras,       // IF阶段RAS命中
    output logic         bp_update_en,     // 分支预测器更新使能
    output logic [63:0]  bp_update_pc,     // 更新PC
    output logic [31:0]  bp_update_instr,  // 更新指令
    output logic         bp_actual_taken,  // 实际分支结果
    output logic [63:0]  bp_actual_target, // 实际目标地址
    
    // 原子指令相关接口
    input  logic         atomic_in_progress, // 原子操作进行中标志
    output logic         is_atomic_out,    // 是否为原子指令
    output atomic_op_t   atomic_op_out     // 原子操作类型
);

    // 内部信号定义
    logic [63:0] alu_result = '0;     // ALU计算结果
    logic [63:0] branch_target = '0;  // 分支目标地址
    logic        branch_taken = '0;    // 分支是否taken
    logic [63:0] rs1_data;            // 源寄存器1数据
    logic [63:0] rs2_data;            // 源寄存器2数据
    logic [63:0] current_pc = '0;     // 当前PC值
    logic        op = '0;             // 操作类型
    logic        valid_out = '0;
    logic        first_op = '1;       // 首次操作标志
    logic [31:0] add_result = '0;
    logic [31:0] xor_result = '0;
    logic [63:0] op1 = '0;            // 操作数1
    logic [63:0] op2 = '0;            // 操作数2
    
    // 乘除法器相关信号
    logic        mul_start, div_start;       // 乘除法器启动信号
    logic [63:0] mul_result, div_result;     // 乘除法器结果
    logic        mul_done, div_done;         // 乘除法器完成信号
    logic        mul_busy, div_busy;         // 乘除法器忙碌信号
    logic        div_by_zero;                // 除零标志
    logic        muldiv_in_progress;         // 乘除法运算中标志
    logic [2:0]  mul_type, div_type;         // 乘除法类型
    logic        muldiv_started;             // 乘除法是否已启动标志
    
    // 新增中间变量，减少条件判断层数
    logic is_rtype, is_itype, is_load, is_store, is_branch, is_jal, is_jalr, is_lui, is_auipc, is_csr;
    logic is_mext_instr;
    logic is_atomic_local;  // 原子指令标志
    logic [2:0] funct3;
    logic is_w_instr;
    logic do_signed_op;

    logic        muldiv_done_reg;        // 乘除法完成寄存器
    
    // 添加乘除法指令状态保存寄存器
    logic [63:0] muldiv_pc_saved;        // 保存乘除法指令的PC
    logic [31:0] muldiv_instr_saved;     // 保存乘除法指令
    logic [4:0]  muldiv_rd_saved;        // 保存乘除法指令的目标寄存器
    logic        muldiv_reg_write_saved; // 保存寄存器写使能
    logic        muldiv_mem_read_saved;  // 保存内存读使能
    logic        muldiv_mem_write_saved; // 保存内存写使能
    logic        muldiv_mem_to_reg_saved;// 保存内存到寄存器控制
    logic [1:0]  muldiv_mem_size_saved;  // 保存内存访问大小
    logic [4:0]  muldiv_rs2_saved;       // 保存rs2地址
    
    // 添加乘除法操作数保存寄存器
    logic [63:0] muldiv_rs1_data_saved;  // 保存乘除法指令的rs1操作数
    logic [63:0] muldiv_rs2_data_saved;  // 保存乘除法指令的rs2操作数
    
    // 添加乘除法完成状态控制
    logic        muldiv_just_finished;   // 乘除法刚完成标志
    
    // 添加指令状态保存，避免依赖ID阶段被冲刷的信号
    logic [63:0] saved_pc;              // 保存的PC
    logic [31:0] saved_instr;           // 保存的指令
    logic [63:0] saved_imm;             // 保存的立即数
    logic [4:0]  saved_rs1, saved_rs2;  // 保存的寄存器地址
    logic        instr_saved;           // 指令是否已保存
    
    // 添加指令处理状态跟踪，避免重复更新分支预测器
    logic [63:0] last_processed_pc;     // 上次处理的指令PC
    logic [31:0] last_processed_instr;  // 上次处理的指令
    logic        bp_already_updated;    // 当前指令是否已经更新过分支预测器
    
    // 分支处理局部变量
    logic [63:0] branch_target_local;   // 分支目标地址（局部）
    logic [63:0] rs1_data_local;        // 分支处理用的rs1数据
    logic [63:0] rs2_data_local;        // 分支处理用的rs2数据

    // 实例化乘法器
    multiplier mul_unit (
        .clk(clk),
        .reset(reset),
        .start(mul_start),
        .multiplicand(muldiv_rs1_data_saved),
        .multiplier(muldiv_rs2_data_saved),
        .mul_type(mul_type),
        .result(mul_result),
        .done(mul_done),
        .busy(mul_busy)
    );

    // 实例化除法器
    divider div_unit (
        .clk(clk),
        .reset(reset),
        .start(div_start),
        .dividend(muldiv_rs1_data_saved),
        .divisor(muldiv_rs2_data_saved),
        .div_type(div_type),
        .result(div_result),
        .done(div_done),
        .busy(div_busy),
        .div_by_zero(div_by_zero)
    );

    // 组合逻辑：寄存器读取和立即数扩展
    always_comb begin
        rs1_data = regfile[rs1];
        rs2_data = regfile[rs2];
        
        // 解码指令类型，提前确定各种条件，减少嵌套if-else
        is_rtype = (instr_in[6:0] == 7'b0110011) || (instr_in[6:0] == 7'b0111011);
        is_itype = (instr_in[6:0] == 7'b0010011) || (instr_in[6:0] == 7'b0011011);
        is_load = (instr_in[6:0] == 7'b0000011);
        is_store = (instr_in[6:0] == 7'b0100011);
        is_branch = (instr_in[6:0] == 7'b1100011);
        is_jal = (instr_in[6:0] == 7'b1101111);
        is_jalr = (instr_in[6:0] == 7'b1100111);
        is_lui = (instr_in[6:0] == 7'b0110111);
        is_auipc = (instr_in[6:0] == 7'b0010111);
        is_csr = (instr_in[6:0] == 7'b1110011);
        
        // M扩展指令判断
        is_mext_instr = is_rtype && (instr_in[31:25] == 7'b0000001);
        
        // 原子指令判断
        is_atomic_local = (instr_in[6:0] == 7'b0101111);
        
        // 是否为W指令（RV64特有）
        is_w_instr = (instr_in[6:0] == 7'b0111011) || (instr_in[6:0] == 7'b0011011);
        
        // 函数类型
        funct3 = instr_in[14:12];
        
        // 是否为有符号操作
        do_signed_op = !funct3[0];
        
        // 检查是否有乘除法运算在进行
        muldiv_in_progress = mul_busy || div_busy;
        
        // 原子指令输出
        is_atomic_out = is_atomic_local;
        if (is_atomic_local) begin
                    // 删除调试信息
            // 安全地转换funct5到原子操作类型
            case (instr_in[31:27])
                5'b00000: atomic_op_out = AMO_ADD;
                5'b00001: atomic_op_out = AMO_SWAP;
                5'b00010: atomic_op_out = AMO_LR;
                5'b00011: atomic_op_out = AMO_SC;
                5'b00100: atomic_op_out = AMO_XOR;
                5'b01000: atomic_op_out = AMO_OR;
                5'b01100: atomic_op_out = AMO_AND;
                5'b10000: atomic_op_out = AMO_MIN;
                5'b10100: atomic_op_out = AMO_MAX;
                5'b11000: atomic_op_out = AMO_MINU;
                5'b11100: atomic_op_out = AMO_MAXU;
                default:  atomic_op_out = AMO_ADD;  // 默认值
            endcase
        end else begin
            atomic_op_out = AMO_ADD;  // 默认值，当不是原子指令时此值无意义
        end
    end

    // 输出乘除法忙碌信号 - 简化版本，主要用于内部状态跟踪
    // IF阶段会直接检测乘除法指令并控制流水线，这里主要用于EX内部状态管理
    assign muldiv_busy = muldiv_in_progress;
    
    // 输出乘除法完成信号 - 修改为寄存存储，提供稳定的完成指示
    assign muldiv_done = muldiv_done_reg;

    // 时序逻辑：状态更新
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            first_op <= 1'b1;
            current_pc <= 64'h00000000_00000000;
            muldiv_started <= 1'b0;
            muldiv_done_reg <= 1'b0;           // 初始化乘除法完成寄存器
            // 初始化保存寄存器
            muldiv_pc_saved <= 64'h0;
            muldiv_instr_saved <= 32'h0;
            muldiv_rd_saved <= 5'h0;
            muldiv_reg_write_saved <= 1'b0;
            muldiv_mem_read_saved <= 1'b0;
            muldiv_mem_write_saved <= 1'b0;
            muldiv_mem_to_reg_saved <= 1'b0;
            muldiv_mem_size_saved <= 2'h0;
            muldiv_rs2_saved <= 5'h0;
            muldiv_rs1_data_saved <= 64'h0;    // 初始化乘除法操作数保存寄存器
            muldiv_rs2_data_saved <= 64'h0;    // 初始化乘除法操作数保存寄存器
            muldiv_just_finished <= 1'b0;
            // 初始化指令状态保存
            saved_pc <= 64'h0;
            saved_instr <= 32'h0;
            saved_imm <= 64'h0;
            saved_rs1 <= 5'h0;
            saved_rs2 <= 5'h0;
            instr_saved <= 1'b0;
            // 初始化指令处理状态跟踪
            last_processed_pc <= 64'h0;
            last_processed_instr <= 32'b0;
            bp_already_updated <= 1'b0;
        end else if (!block) begin
            first_op <= 1'b0;
            current_pc <= pc_in;
            
            // 检查当前指令是否已经处理过
            if (instr_saved) begin
                bp_already_updated <= (saved_pc == last_processed_pc) && (saved_instr == last_processed_instr);
            end else begin
                bp_already_updated <= (pc_in == last_processed_pc) && (instr_in == last_processed_instr);
            end
            
            // 在每个新指令到达时保存状态，避免依赖可能被冲刷的ID阶段信号
            if (id_finish && !instr_saved) begin
                saved_pc <= pc_in;
                saved_instr <= instr_in;
                saved_imm <= imm;
                saved_rs1 <= rs1;
                saved_rs2 <= rs2;
                instr_saved <= 1'b1;
            end else if (!id_finish) begin
                instr_saved <= 1'b0;  // 重置保存标志，准备下一条指令
            end
            
            // 管理乘除法启动状态和保存状态
            if (is_mext_instr && id_finish && !muldiv_in_progress) begin
                muldiv_started <= 1'b1;
                muldiv_done_reg <= 1'b0;       // 启动时清零完成信号
                muldiv_just_finished <= 1'b0;  // 清零刚完成标志
                // 保存乘除法指令的状态
                muldiv_pc_saved <= pc_in;
                muldiv_instr_saved <= instr_in;
                muldiv_rd_saved <= rd_in;
                muldiv_reg_write_saved <= reg_write_in;
                muldiv_mem_read_saved <= mem_read_in;
                muldiv_mem_write_saved <= mem_write_in;
                muldiv_mem_to_reg_saved <= mem_to_reg_in;
                muldiv_mem_size_saved <= mem_size_in;
                muldiv_rs2_saved <= rs2;
                muldiv_rs1_data_saved <= rs1_data;
                muldiv_rs2_data_saved <= rs2_data;
            end else if (mul_done || div_done) begin
                muldiv_started <= 1'b0;
                muldiv_done_reg <= 1'b1;       // 运算完成时设置完成信号
                muldiv_just_finished <= 1'b1;  // 设置刚完成标志，下个周期会触发ex_finish
            end else if (muldiv_just_finished) begin
                // 如果上个周期是刚完成状态，这个周期清零所有标志
                muldiv_done_reg <= 1'b0;       // 清零完成信号
                muldiv_just_finished <= 1'b0;  // 清零刚完成标志
                muldiv_started <= 1'b0;
                muldiv_done_reg <= 1'b0; 
            end else begin
                muldiv_just_finished <= 1'b0;  // 默认清零刚完成标志
                muldiv_started <= 1'b0;
                muldiv_done_reg <= 1'b0; 
            end
        end
    end

    // ALU运算和控制逻辑
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // 所有输出信号初始化
            rd_out <= '0;
            pc_out <= 64'b0;
            result <= 64'b0;
            valid <= 1'b0;
            instr_out <= '0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_size_out <= '0;
            rs2_out <= '0;
            branch_out <= 1'b0;
            branch_target_out <= '0;
            // 初始化启动信号
            mul_start <= 1'b0;
            div_start <= 1'b0;
            // 初始化分支预测器更新信号
            bp_update_en <= 1'b0;
            bp_update_pc <= 64'b0;
            bp_update_instr <= 32'b0;
            bp_actual_taken <= 1'b0;
            bp_actual_target <= 64'b0;
        end else if (!block) begin
            // 使用提前解码的指令类型变量进行处理，减少嵌套条件判断

            // 计算ALU结果
            alu_result = 64'b0; // 默认为0
            branch_taken = 1'b0; // 默认不跳转
            
            // 默认清零启动信号
            mul_start <= 1'b0;
            div_start <= 1'b0;
            
            // 默认分支预测器更新信号
            bp_update_en <= 1'b0;
            bp_update_pc <= 64'b0;
            bp_update_instr <= 32'b0;
            bp_actual_taken <= 1'b0;
            bp_actual_target <= 64'b0;
            
            // 1. ALU操作 - 使用更扁平的结构
            
            // R-type运算指令
            if (is_rtype) begin
                if (is_mext_instr) begin
                    // M扩展指令处理 - 乘除法运算
                    // 只有在新指令到达且未启动时才启动乘除法运算
                    if (id_finish && !muldiv_in_progress && !muldiv_started) begin
                        // 启动乘除法运算
                        case (funct3)
                            3'b000, 3'b001, 3'b010, 3'b011: begin // MUL, MULH, MULHSU, MULHU
                                mul_start <= 1'b1;
                                div_start <= 1'b0;
                                mul_type <= funct3;
                                if (is_w_instr) mul_type <= 3'b100; // MULW
                            end
                            3'b100, 3'b101, 3'b110, 3'b111: begin // DIV, DIVU, REM, REMU
                                mul_start <= 1'b0;
                                div_start <= 1'b1;
                                div_type <= funct3;
                                if (is_w_instr) begin
                                    // 正确映射32位除法指令到除法器类型
                                    case (funct3)
                                        3'b100: div_type <= 3'b000; // DIVW
                                        3'b101: div_type <= 3'b001; // DIVUW  
                                        3'b110: div_type <= 3'b010; // REMW
                                        3'b111: div_type <= 3'b011; // REMUW
                                        default: div_type <= funct3;
                                    endcase
                                end
                            end
                            default: begin
                                mul_start <= 1'b0;
                                div_start <= 1'b0;
                            end
                        endcase
                    end else begin
                        mul_start <= 1'b0;
                        div_start <= 1'b0;
                    end
                    
                    // 选择乘除法结果
                    if (mul_done) begin
                        alu_result = mul_result;
                    end else if (div_done) begin
                        alu_result = div_result;
                    end else begin
                        alu_result = 64'b0; // 运算进行中，结果暂为0
                    end
                end else begin
                    // 标准R-type指令
                    if (funct3 == 3'b000) begin
                        // ADD/SUB/ADDW/SUBW
                        if (is_w_instr) begin
                            logic [31:0] add_sub_result;
                            add_sub_result = instr_in[30] ? (rs1_data[31:0] - rs2_data[31:0]) : (rs1_data[31:0] + rs2_data[31:0]);
                            alu_result = {{32{add_sub_result[31]}}, add_sub_result};
                        end else begin
                            alu_result = instr_in[30] ? (rs1_data - rs2_data) : (rs1_data + rs2_data);
                        end
                    end else if (funct3 == 3'b001) begin
                        // SLL/SLLW
                        if (is_w_instr) begin
                            logic [31:0] sll_result;
                            sll_result = rs1_data[31:0] << rs2_data[4:0];
                            alu_result = {{32{sll_result[31]}}, sll_result};
                        end else begin
                            alu_result = rs1_data << rs2_data[5:0];
                        end
                    end else if (funct3 == 3'b010) begin
                        // SLT
                        alu_result = $signed(rs1_data) < $signed(rs2_data) ? 1 : 0;
                    end else if (funct3 == 3'b011) begin
                        // SLTU
                        alu_result = rs1_data < rs2_data ? 1 : 0;
                    end else if (funct3 == 3'b100) begin
                        // XOR/XORW
                        if (is_w_instr) begin
                            logic [31:0] xor_result;
                            xor_result = rs1_data[31:0] ^ rs2_data[31:0];
                            alu_result = {{32{xor_result[31]}}, xor_result};
                        end else begin
                            alu_result = rs1_data ^ rs2_data;
                        end
                    end else if (funct3 == 3'b101) begin
                        // SRL/SRA/SRLW/SRAW
                        if (is_w_instr) begin
                            logic [31:0] shift_result;
                            if (instr_in[30])
                                shift_result = $signed(rs1_data[31:0]) >>> rs2_data[4:0];
                            else
                                shift_result = rs1_data[31:0] >> rs2_data[4:0];
                            alu_result = {{32{shift_result[31]}}, shift_result};
                        end else begin
                            if (instr_in[30])
                                alu_result = $signed(rs1_data) >>> rs2_data[5:0];
                            else
                                alu_result = rs1_data >> rs2_data[5:0];
                        end
                    end else if (funct3 == 3'b110) begin
                        // OR/ORW
                        if (is_w_instr) begin
                            logic [31:0] or_result;
                            or_result = rs1_data[31:0] | rs2_data[31:0];
                            alu_result = {{32{or_result[31]}}, or_result};
                        end else begin
                            alu_result = rs1_data | rs2_data;
                        end
                    end else if (funct3 == 3'b111) begin
                        // AND/ANDW
                        if (is_w_instr) begin
                            logic [31:0] and_result;
                            and_result = rs1_data[31:0] & rs2_data[31:0];
                            alu_result = {{32{and_result[31]}}, and_result};
                        end else begin
                            alu_result = rs1_data & rs2_data;
                        end
                    end
                end
            end

            // I-type运算指令
            if (is_itype) begin
                if (funct3 == 3'b000) begin
                    // ADDI/ADDIW
                    if (is_w_instr) begin
                        add_result = rs1_data[31:0] + imm[31:0];
                        alu_result = {{32{add_result[31]}}, add_result};
                    end else begin
                        alu_result = rs1_data + imm;
                    end
                end else if (funct3 == 3'b001) begin
                    // SLLI/SLLIW
                    if (is_w_instr) begin
                        logic [31:0] sll_result;
                        sll_result = rs1_data[31:0] << instr_in[24:20];
                        alu_result = {{32{sll_result[31]}}, sll_result};
                    end else begin
                        alu_result = rs1_data << imm;
                    end
                end else if (funct3 == 3'b010) begin
                    // SLTI
                    alu_result = $signed(rs1_data) < $signed(imm) ? 1 : 0;
                end else if (funct3 == 3'b011) begin
                    // SLTIU
                    alu_result = rs1_data < imm ? 1 : 0;
                end else if (funct3 == 3'b100) begin
                    // XORI
                    alu_result = rs1_data ^ imm;
                end else if (funct3 == 3'b101) begin
                    // SRLI/SRAI/SRLIW/SRAIW
                    if (is_w_instr) begin
                        logic [31:0] shift_result;
                        if (instr_in[30])
                            shift_result = $signed(rs1_data[31:0]) >>> instr_in[24:20];
                        else
                            shift_result = rs1_data[31:0] >> instr_in[24:20];
                        alu_result = {{32{shift_result[31]}}, shift_result};
                    end else begin
                        if (instr_in[30])
                            alu_result = $signed(rs1_data) >>> instr_in[25:20];
                        else
                            alu_result = rs1_data >> instr_in[25:20];
                    end
                end else if (funct3 == 3'b110) begin
                    // ORI
                    alu_result = rs1_data | imm;
                end else if (funct3 == 3'b111) begin
                    // ANDI
                    alu_result = rs1_data & imm;
                end
            end

            

            // 分支指令处理
            if (is_branch) begin
                if (instr_saved) begin
                    // 使用保存的状态（流水线冲刷后重新执行）
                    branch_target_local = saved_pc + saved_imm;
                    rs1_data_local = regfile[saved_rs1];
                    rs2_data_local = regfile[saved_rs2];
                    
                    // 计算分支条件
                    if (funct3 == 3'b000) begin
                        branch_taken = (rs1_data_local == rs2_data_local);    // BEQ
                    end else if (funct3 == 3'b001) begin
                        branch_taken = (rs1_data_local != rs2_data_local);    // BNE
                    end else if (funct3 == 3'b100) begin
                        branch_taken = $signed(rs1_data_local) < $signed(rs2_data_local);    // BLT
                    end else if (funct3 == 3'b101) begin
                        branch_taken = $signed(rs1_data_local) >= $signed(rs2_data_local);   // BGE
                    end else if (funct3 == 3'b110) begin
                        branch_taken = rs1_data_local < rs2_data_local;       // BLTU
                    end else if (funct3 == 3'b111) begin
                        branch_taken = rs1_data_local >= rs2_data_local;      // BGEU
                    end
                    branch_target = branch_target_local;
                    
                    // 只有在指令没有被处理过时才更新分支预测器
                    if (!bp_already_updated) begin
                        bp_update_en <= 1'b1;
                        bp_update_pc <= saved_pc;
                        bp_update_instr <= saved_instr;
                        bp_actual_taken <= branch_taken;
                        bp_actual_target <= branch_target_local;
                        // 记录已处理的指令
                        last_processed_pc <= saved_pc;
                        last_processed_instr <= saved_instr;
                    end
                end else begin
                    // 使用当前状态（正常执行）
                    branch_target_local = pc_in + imm;
                    rs1_data_local = rs1_data;
                    rs2_data_local = rs2_data;
                    
                    // 计算分支条件
                    if (funct3 == 3'b000) begin
                        branch_taken = (rs1_data_local == rs2_data_local);    // BEQ
                    end else if (funct3 == 3'b001) begin
                        branch_taken = (rs1_data_local != rs2_data_local);    // BNE
                    end else if (funct3 == 3'b100) begin
                        branch_taken = $signed(rs1_data_local) < $signed(rs2_data_local);    // BLT
                    end else if (funct3 == 3'b101) begin
                        branch_taken = $signed(rs1_data_local) >= $signed(rs2_data_local);   // BGE
                    end else if (funct3 == 3'b110) begin
                        branch_taken = rs1_data_local < rs2_data_local;       // BLTU
                    end else if (funct3 == 3'b111) begin
                        branch_taken = rs1_data_local >= rs2_data_local;      // BGEU
                    end
                    branch_target = branch_target_local;
                    
                    // 更新分支预测器 - 使用当前状态
                    if (!bp_already_updated) begin
                        bp_update_en <= 1'b1;
                        bp_update_pc <= pc_in;
                        bp_update_instr <= instr_in;
                        bp_actual_taken <= branch_taken;
                        bp_actual_target <= branch_target_local;
                        // 记录已处理的指令
                        last_processed_pc <= pc_in;
                        last_processed_instr <= instr_in;
                    end
                end
            end

            // LUI指令
            if (is_lui) begin
                alu_result = imm;
            end

            // AUIPC指令
            if (is_auipc) begin
                alu_result = pc_in + imm;
            end

            // JAL指令 - JAL不读取寄存器，可以直接使用当前信号
            if (is_jal) begin
                logic [63:0] jal_target;
                if (instr_saved) begin
                    // 使用保存的状态（流水线冲刷后重新执行）
                    jal_target = saved_pc + saved_imm;
                    alu_result = saved_pc + 4;

                    
                    // 只有在指令没有被处理过时才更新分支预测器
                    if (!bp_already_updated) begin
                        bp_update_en <= 1'b1;
                        bp_update_pc <= saved_pc;
                        bp_update_instr <= saved_instr;
                        bp_actual_taken <= 1'b1;
                        bp_actual_target <= jal_target;
                        // 记录已处理的指令
                        last_processed_pc <= saved_pc;
                        last_processed_instr <= saved_instr;
                    end
                end else begin
                    // 使用当前状态（正常执行）
                    jal_target = pc_in + imm;
                    alu_result = pc_in + 4;

                    
                    // 更新分支预测器 - 使用当前状态
                    if (!bp_already_updated) begin
                        bp_update_en <= 1'b1;
                        bp_update_pc <= pc_in;
                        bp_update_instr <= instr_in;
                        bp_actual_taken <= 1'b1;
                        bp_actual_target <= jal_target;
                        // 记录已处理的指令
                        last_processed_pc <= pc_in;
                        last_processed_instr <= instr_in;
                    end
                end
                
                branch_taken = 1'b1;
                branch_target = jal_target;
            end

            // JALR指令
            if (is_jalr) begin
                logic [63:0] jalr_target;
                if (instr_saved) begin
                    // 使用保存的状态（流水线冲刷后重新执行）
                    logic [63:0] rs1_saved_data = regfile[saved_rs1];
                    jalr_target = (rs1_saved_data + saved_imm) & ~1;
                    alu_result = saved_pc + 4;
                    
                    // 只有在指令没有被处理过时才更新分支预测器
                    if (!bp_already_updated) begin
                        bp_update_en <= 1'b1;
                        bp_update_pc <= saved_pc;
                        bp_update_instr <= saved_instr;
                        bp_actual_taken <= 1'b1;
                        bp_actual_target <= jalr_target;
                        // 记录已处理的指令
                        last_processed_pc <= saved_pc;
                        last_processed_instr <= saved_instr;
                    end
                end else begin
                    // 使用当前状态（正常执行）
                    jalr_target = (rs1_data + imm) & ~1;
                    alu_result = pc_in + 4;
                    
                    // 更新分支预测器 - 使用当前状态
                    if (!bp_already_updated) begin
                        bp_update_en <= 1'b1;
                        bp_update_pc <= pc_in;
                        bp_update_instr <= instr_in;
                        bp_actual_taken <= 1'b1;
                        bp_actual_target <= jalr_target;
                        // 记录已处理的指令
                        last_processed_pc <= pc_in;
                        last_processed_instr <= instr_in;
                    end
                end
                
                branch_taken = 1'b1;
                branch_target = jalr_target;
            end

            // CSR指令
            if (is_csr) begin
                alu_result = csr_read_data;
            end

            // 内存访问指令
            if (is_load || is_store) begin
                alu_result = rs1_data + imm;
                        // 删除调试信息
            end
            
            // 原子指令地址计算
            if (is_atomic_local) begin
                alu_result = rs1_data + imm;
                        // 删除调试信息
            end

            // 状态更新和输出控制 
            valid <= 0;
            // 优先处理乘除法完成信号，然后处理其他指令
            if (muldiv_just_finished) begin
                // 乘除法刚完成的下一个周期 - 输出完成信号和结果
                pc_out <= muldiv_pc_saved;      // 使用保存的PC
                ex_finish <= 1'b1;              // 拉高完成信号一个周期
                instr_out <= muldiv_instr_saved; // 使用保存的指令
                
                if (muldiv_reg_write_saved && muldiv_rd_saved != 5'b00000) begin
                    result <= alu_result;        // alu_result已包含乘除法结果
                    valid <= 1;
                end
                
                rd_out <= muldiv_rd_saved;       // 使用保存的目标寄存器
                mem_read_out <= muldiv_mem_read_saved;   // 使用保存的控制信号
                mem_write_out <= muldiv_mem_write_saved;
                mem_to_reg_out <= muldiv_mem_to_reg_saved;
                reg_write_out <= muldiv_reg_write_saved;
                mem_size_out <= muldiv_mem_size_saved;
                rs2_out <= muldiv_rs2_saved;     // 使用保存的rs2
                branch_out <= 1'b0;              // 乘除法指令不涉及分支
                muldiv_just_finished <= 1'b0;
            end
            // 处理其他指令：ID完成且无乘除法运算，且不是乘除法刚完成的周期
            // 对于原子指令：只在没有原子操作进行中时处理
            // 对于非原子指令：在没有原子操作进行中时处理
            else if ((id_finish || first_op) && !muldiv_in_progress && !muldiv_just_finished && 
                    (!atomic_in_progress || is_atomic_local)) begin
                // 地址对齐检查 - 只有在没有已存在的misalign异常时才进行检查
                if (!misalign.valid) begin
                    // 内存访问指令的地址对齐检查
                    if (mem_read_in || mem_write_in) begin
                        case (mem_size_in)
                            2'b00: begin  // 字节访问不需要对齐检查
                                misalign.valid <= 1'b0;
                            end
                            2'b01: begin  // 半字访问需要2字节对齐
                                if (alu_result[0] != 0) begin
                                    misalign.valid <= 1'b1;
                                    misalign.addr <= alu_result;
                                    misalign.misalign_type <= mem_read_in ? LOAD_MISALIGN : STORE_MISALIGN;
                                    misalign.pc <= pc_in;
                                    alu_result = 64'h0;
                                end else begin
                                    misalign.valid <= 1'b0;
                                end
                            end
                            2'b10: begin  // 字访问需要4字节对齐
                                if (alu_result[1:0] != 0) begin
                                    misalign.valid <= 1'b1;
                                    misalign.addr <= alu_result;
                                    misalign.misalign_type <= mem_read_in ? LOAD_MISALIGN : STORE_MISALIGN;
                                    misalign.pc <= pc_in;
                                    alu_result = 64'h0;
                                end else begin
                                    misalign.valid <= 1'b0;
                                end
                            end
                            2'b11: begin  // 双字访问需要8字节对齐
                                if (alu_result[2:0] != 0) begin
                                    misalign.valid <= 1'b1;
                                    misalign.addr <= alu_result;
                                    misalign.misalign_type <= mem_read_in ? LOAD_MISALIGN : STORE_MISALIGN;
                                    misalign.pc <= pc_in;
                                    alu_result = 64'h0;
                                end else begin
                                    misalign.valid <= 1'b0;
                                end
                            end
                        endcase
                    end
                    // 原子指令的地址对齐检查
                    else if (is_atomic_local) begin
                        // 原子指令必须4字节对齐
                        if (alu_result[1:0] != 0) begin
                            misalign.valid <= 1'b1;
                            misalign.addr <= alu_result;
                            misalign.misalign_type <= STORE_MISALIGN;  // 原子指令类似于存储操作
                            misalign.pc <= pc_in;
                            alu_result = 64'h0;
                        end else begin
                            misalign.valid <= 1'b0;
                        end
                    end
                    else begin
                        misalign.valid <= 1'b0;
                    end
                end


                pc_out <= pc_in;
                instr_out <= instr_in;
                
                // 对于乘除法指令，需要等待运算完成
                if (is_mext_instr) begin
                    ex_finish <= 1'b0;  // 先设为未完成，等乘除法完成
                end else begin
                    ex_finish <= 1'b1;
                end
                
                if (reg_write_in && rd_in != 5'b00000) begin
                    result <= alu_result;
                    // 删除调试信息
                    if (!is_mext_instr) valid <= 1;  // 非乘除法指令立即有效

                end
                if (mem_read_in || mem_write_in) begin
                    result <= alu_result;
                    // 删除调试信息
                end
                if (branch_taken) begin
                    branch_target_out <= branch_target;
                end
                rd_out <= rd_in;
                mem_read_out <= mem_read_in;
                mem_write_out <= mem_write_in;
                mem_to_reg_out <= mem_to_reg_in;
                reg_write_out <= reg_write_in;
                mem_size_out <= mem_size_in;
                rs2_out <= rs2;
                branch_out <= branch_taken;
            end
            else begin
                branch_out <= 1'b0;
                ex_finish <= 1'b0;
                branch_taken = 1'b0;    
            end
        end
    end

endmodule

`endif
