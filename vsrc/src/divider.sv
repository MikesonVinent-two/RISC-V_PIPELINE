`ifndef __DIVIDER_SV
`define __DIVIDER_SV

`ifdef VERILATOR
`include "include/common.sv"
`endif

// 多周期除法器模块
module divider
    import common::*;
(
    input  logic        clk,
    input  logic        reset,
    input  logic        start,          // 开始除法运算
    input  logic [63:0] dividend,       // 被除数
    input  logic [63:0] divisor,        // 除数
    input  logic [2:0]  div_type,       // 除法类型 (DIV, DIVU, REM, REMU, DIVW, DIVUW, REMW, REMUW)
    output logic [63:0] result,         // 结果
    output logic        done,           // 完成信号
    output logic        busy,           // 忙碌信号
    output logic        div_by_zero     // 除零标志
);

    // 状态定义
    typedef enum logic [2:0] {
        IDLE,       // 空闲状态
        COMPUTE,    // 计算状态
        FINISH,     // 完成状态
        FLUSH       // 清理状态
    } state_t;

    state_t state;
    logic [63:0]  quotient;         // 商
    logic [63:0]  remainder;        // 余数
    logic [63:0]  temp_dividend;    // 临时被除数
    logic [63:0]  temp_divisor;     // 临时除数
    logic [6:0]   counter;          // 计数器
    logic         is_signed_div;    // 是否有符号除法
    logic         is_remainder;     // 是否求余数
    logic         is_word_op;       // 是否为32位操作
    logic [63:0]  abs_dividend, abs_divisor;  // 绝对值
    logic         result_negative;   // 结果符号
    logic         dividend_negative, divisor_negative;  // 操作数符号

    // 除法类型解码
    always_comb begin
        case (div_type)
            3'b100: begin // DIV - 有符号除法
                is_signed_div = 1'b1;
                is_remainder = 1'b0;
                is_word_op = 1'b0;
            end
            3'b101: begin // DIVU - 无符号除法
                is_signed_div = 1'b0;
                is_remainder = 1'b0;
                is_word_op = 1'b0;
            end
            3'b110: begin // REM - 有符号求余
                is_signed_div = 1'b1;
                is_remainder = 1'b1;
                is_word_op = 1'b0;
            end
            3'b111: begin // REMU - 无符号求余
                is_signed_div = 1'b0;
                is_remainder = 1'b1;
                is_word_op = 1'b0;
            end
            3'b000: begin // DIVW - 32位有符号除法
                is_signed_div = 1'b1;
                is_remainder = 1'b0;
                is_word_op = 1'b1;
            end
            3'b001: begin // DIVUW - 32位无符号除法
                is_signed_div = 1'b0;
                is_remainder = 1'b0;
                is_word_op = 1'b1;
            end
            3'b010: begin // REMW - 32位有符号求余
                is_signed_div = 1'b1;
                is_remainder = 1'b1;
                is_word_op = 1'b1;
            end
            3'b011: begin // REMUW - 32位无符号求余
                is_signed_div = 1'b0;
                is_remainder = 1'b1;
                is_word_op = 1'b1;
            end
            default: begin
                is_signed_div = 1'b0;
                is_remainder = 1'b0;
                is_word_op = 1'b0;
            end
        endcase
    end

    // 符号处理
    always_comb begin
        if (is_word_op) begin
            // 32位操作：先取符号，再计算绝对值，最后零扩展
            dividend_negative = is_signed_div && dividend[31];
            divisor_negative = is_signed_div && divisor[31];
            
            // 修正：32位操作数的绝对值计算和扩展
            if (dividend_negative) begin
                abs_dividend = {32'b0, (~dividend[31:0] + 1)};
            end else begin
                abs_dividend = {32'b0, dividend[31:0]};
            end
            
            if (divisor_negative) begin
                abs_divisor = {32'b0, (~divisor[31:0] + 1)};
            end else begin
                abs_divisor = {32'b0, divisor[31:0]};
            end
        end else begin
            // 64位操作
            dividend_negative = is_signed_div && dividend[63];
            divisor_negative = is_signed_div && divisor[63];
            abs_dividend = dividend_negative ? (~dividend + 1) : dividend;
            abs_divisor = divisor_negative ? (~divisor + 1) : divisor;
        end
        
        // 结果符号：除法和求余的符号规则不同
        if (is_remainder) begin
            result_negative = dividend_negative;  // 余数符号同被除数
        end else begin
            result_negative = dividend_negative ^ divisor_negative;  // 商的符号
        end
    end

    // 主状态机
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            quotient <= 64'b0;
            remainder <= 64'b0;
            temp_dividend <= 64'b0;
            temp_divisor <= 64'b0;
            counter <= 7'b0;
            result <= 64'b0;
            done <= 1'b0;
            busy <= 1'b0;
            div_by_zero <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    // IDLE状态：清零done信号，等待新的启动
                    done <= 1'b0;
                    busy <= 1'b0;
                    div_by_zero <= 1'b0;
                    
                    if (start) begin
                        // 检查除零
                        if (divisor == 64'b0 || (is_word_op && divisor[31:0] == 32'b0)) begin
                            div_by_zero <= 1'b1;
                            // 按RISC-V标准处理除零
                            if (is_remainder) begin
                                // 求余除零：返回被除数
                                if (is_word_op) begin
                                    // 32位操作需要符号扩展
                                    result <= {{32{dividend[31]}}, dividend[31:0]};
                                end else begin
                                    result <= dividend;
                                end
                            end else begin
                                // 除法除零：返回-1
                                result <= {64{1'b1}};
                            end
                            done <= 1'b1;
                            state <= FLUSH;
                        end
                        // 检查溢出：INT_MIN / -1
                        else if ((is_word_op && dividend[31:0] == 32'h80000000 && divisor[31:0] == 32'hffffffff && is_signed_div) ||
                                (!is_word_op && dividend == 64'h8000000000000000 && divisor == 64'hffffffffffffffff && is_signed_div)) begin
                            // 溢出情况
                            if (is_remainder) begin
                                // 求余溢出：返回0
                                result <= 64'b0;
                            end else begin
                                // 除法溢出：返回INT_MIN
                                if (is_word_op) begin
                                    result <= {{32{1'b1}}, 32'h80000000};  // 符号扩展的INT32_MIN
                                end else begin
                                    result <= 64'h8000000000000000;  // INT64_MIN
                                end
                            end
                            done <= 1'b1;
                            state <= FLUSH;
                        end
                        else begin
                            state <= COMPUTE;
                            quotient <= 64'b0;
                            remainder <= 64'b0;
                            // 修正：对于32位操作，需要将被除数左对齐
                            if (is_word_op) begin
                                temp_dividend <= abs_dividend << 32;  // 32位操作左移32位对齐
                            end else begin
                                temp_dividend <= abs_dividend;        // 64位操作不需要对齐
                            end
                            temp_divisor <= abs_divisor;
                            counter <= is_word_op ? 7'd32 : 7'd64;
                            busy <= 1'b1;
                        end
                    end
                end

                COMPUTE: begin
                    if (counter > 0) begin
                        // 修正的长除法算法
                        logic [63:0] shifted_remainder;
                        shifted_remainder = {remainder[62:0], temp_dividend[63]};
                        
                        if (shifted_remainder >= temp_divisor) begin
                            remainder <= shifted_remainder - temp_divisor;
                            quotient <= {quotient[62:0], 1'b1};
                        end else begin
                            remainder <= shifted_remainder;
                            quotient <= {quotient[62:0], 1'b0};
                        end
                        
                        temp_dividend <= temp_dividend << 1;
                        counter <= counter - 1;
                    end else begin
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    // 根据操作类型和符号生成最终结果
                    if (is_remainder) begin
                        // 求余数
                        if (is_word_op) begin
                            logic [31:0] temp_rem;
                            temp_rem = result_negative ? (~remainder[31:0] + 1) : remainder[31:0];
                            result <= {{32{temp_rem[31]}}, temp_rem};
                        end else begin
                            result <= result_negative ? (~remainder + 1) : remainder;
                        end
                    end else begin
                        // 求商
                        if (is_word_op) begin
                            logic [31:0] temp_quot;
                            temp_quot = result_negative ? (~quotient[31:0] + 1) : quotient[31:0];
                            result <= {{32{temp_quot[31]}}, temp_quot};
                        end else begin
                            result <= result_negative ? (~quotient + 1) : quotient;
                        end
                    end
                    
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= FLUSH;  // 转到清理状态
                end

                FLUSH: begin
                    // 清理阶段：清零所有临时状态，保持done信号一个周期
                    quotient <= 64'b0;
                    remainder <= 64'b0;
                    temp_dividend <= 64'b0;
                    temp_divisor <= 64'b0;
                    counter <= 7'b0;
                    div_by_zero <= 1'b0;
                    // done信号保持高电平，在下个周期转到IDLE时清零
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

`endif 