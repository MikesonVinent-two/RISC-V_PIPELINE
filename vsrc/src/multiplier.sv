`ifndef __MULTIPLIER_SV
`define __MULTIPLIER_SV

`ifdef VERILATOR
`include "include/common.sv"
`endif

// 多周期乘法器模块
module multiplier
    import common::*;
(
    input  logic        clk,
    input  logic        reset,
    input  logic        start,          // 开始乘法运算
    input  logic [63:0] multiplicand,   // 被乘数
    input  logic [63:0] multiplier,     // 乘数
    input  logic [2:0]  mul_type,       // 乘法类型 (MUL, MULH, MULHSU, MULHU, MULW)
    output logic [63:0] result,         // 结果
    output logic        done,           // 完成信号
    output logic        busy            // 忙碌信号
);

    // 状态定义
    typedef enum logic [2:0] {
        IDLE,       // 空闲状态
        COMPUTE,    // 计算状态
        FINISH,     // 完成状态
        FLUSH       // 清理状态
    } state_t;

    state_t state;
    logic [127:0] product;          // 128位乘积
    logic [63:0]  temp_multiplier;  // 临时乘数
    logic [63:0]  temp_multiplicand; // 临时被乘数
    logic [6:0]   counter;          // 计数器
    logic         is_signed_a, is_signed_b;  // 操作数符号标志
    logic [63:0]  abs_a, abs_b;     // 绝对值
    logic         result_negative;   // 结果符号

    // 乘法类型解码
    always_comb begin
        case (mul_type)
            3'b000: begin // MUL - 有符号×有符号，取低64位
                is_signed_a = 1'b1;
                is_signed_b = 1'b1;
            end
            3'b001: begin // MULH - 有符号×有符号，取高64位
                is_signed_a = 1'b1;
                is_signed_b = 1'b1;
            end
            3'b010: begin // MULHSU - 有符号×无符号，取高64位
                is_signed_a = 1'b1;
                is_signed_b = 1'b0;
            end
            3'b011: begin // MULHU - 无符号×无符号，取高64位
                is_signed_a = 1'b0;
                is_signed_b = 1'b0;
            end
            3'b100: begin // MULW - 有符号×有符号，32位结果符号扩展
                is_signed_a = 1'b1;
                is_signed_b = 1'b1;
            end
            default: begin
                is_signed_a = 1'b0;
                is_signed_b = 1'b0;
            end
        endcase
    end

    // 计算绝对值和结果符号
    always_comb begin
        abs_a = (is_signed_a && multiplicand[63]) ? (~multiplicand + 1) : multiplicand;
        abs_b = (is_signed_b && multiplier[63]) ? (~multiplier + 1) : multiplier;
        result_negative = (is_signed_a && multiplicand[63]) ^ (is_signed_b && multiplier[63]);
    end

    // 主状态机
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            product <= 128'b0;
            temp_multiplier <= 64'b0;
            temp_multiplicand <= 64'b0;
            counter <= 7'b0;
            result <= 64'b0;
            done <= 1'b0;
            busy <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    // IDLE状态：清零done信号，等待新的启动
                    done <= 1'b0;
                    busy <= 1'b0;
                    
                    if (start) begin
                        state <= COMPUTE;
                        product <= {64'b0, 64'b0};  // 初始化为0
                        temp_multiplier <= abs_b;
                        temp_multiplicand <= abs_a;
                        counter <= 7'd64;
                        busy <= 1'b1;
                    end
                end

                COMPUTE: begin
                    if (counter > 0) begin
                        // 修正的移位加法算法
                        // 先进行条件性加法，再进行移位操作
                        if (temp_multiplier[0]) begin
                            product <= (product + {temp_multiplicand, 64'b0}) >> 1;
                        end else begin
                            product <= product >> 1;
                        end
                        temp_multiplier <= temp_multiplier >> 1;
                        counter <= counter - 1;
                    end else begin
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    // 根据乘法类型生成最终结果
                    case (mul_type)
                        3'b000: begin // MUL
                            result <= result_negative ? (~product[63:0] + 1) : product[63:0];
                        end
                        3'b001: begin // MULH
                            result <= result_negative ? (~product[127:64] + 1) : product[127:64];
                        end
                        3'b010: begin // MULHSU
                            result <= result_negative ? (~product[127:64] + 1) : product[127:64];
                        end
                        3'b011: begin // MULHU
                            result <= product[127:64];
                        end
                        3'b100: begin // MULW
                            logic [31:0] temp_result;
                            temp_result = result_negative ? (~product[31:0] + 1) : product[31:0];
                            result <= {{32{temp_result[31]}}, temp_result};
                        end
                        default: result <= product[63:0];
                    endcase
                    
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= FLUSH;  // 转到清理状态
                end

                FLUSH: begin
                    // 清理阶段：清零所有临时状态，保持done信号一个周期
                    product <= 128'b0;
                    temp_multiplier <= 64'b0;
                    temp_multiplicand <= 64'b0;
                    counter <= 7'b0;
                    // done信号保持高电平，在下个周期转到IDLE时清零
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

`endif 