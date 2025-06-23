`ifndef __ID_STAGE_SV
`define __ID_STAGE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr.sv"
`else

`endif

// 指令解码阶段：负责解析指令并生成控制信号
module ID_stage import common::*;(
	// 时钟和控制信号
	input  logic        clk, reset,
	input  logic        block,
	
	// 流水线冲刷信号
	input  logic        pipeline_flush,
	
	// 指令和PC输入
	input  logic [63:0] pc_in,
	input  logic [31:0] instr_in,
	
	// 指令和PC输出
	output logic [63:0] pc_out,
	output logic [31:0] instr_out,
	
	// 寄存器地址
	output logic [4:0]  rs1,
	output logic [4:0]  rs2,
	output logic [4:0]  rd,
	
	// 立即数和内存访问控制
	output logic [63:0] imm,
	output logic [1:0]  mem_size,

	// 控制信号
	output logic        reg_write,   // 寄存器写使能
	output logic        mem_read,    // 内存读使能
	output logic        mem_write,   // 内存写使能
	output logic        branch,      // 分支指令标志
	output logic        alu_src,     // ALU输入选择
	output logic        mem_to_reg,  // 写回数据选择
	output logic        id_finish,     // 阶段完成标志
	input logic        if_finish     // 阶段完成标志
);

	// 存储上一个PC值
	logic [63:0] prev_pc;
	
	// 指令解码和立即数生成
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			prev_pc <= 64'b0;
			pc_out <= 64'b0;
			instr_out <= 32'b0;
			rs1 <= 5'b0;
			rs2 <= 5'b0;
			rd <= 5'b0;
			imm <= 64'b0;
			mem_size <= 2'b00;
			reg_write <= 1'b0;
			mem_read <= 1'b0;
			mem_write <= 1'b0;
			branch <= 1'b0;
			alu_src <= 1'b0;
			mem_to_reg <= 1'b0;
		end else if (pipeline_flush) begin
			// 流水线冲刷：清空所有输出信号
			pc_out <= 64'b0;
			instr_out <= 32'b0;
			rs1 <= 5'b0;
			rs2 <= 5'b0;
			rd <= 5'b0;
			imm <= 64'b0;
			mem_size <= 2'b00;
			reg_write <= 1'b0;
			mem_read <= 1'b0;
			mem_write <= 1'b0;
			branch <= 1'b0;
			alu_src <= 1'b0;
			mem_to_reg <= 1'b0;
			id_finish <= 1'b0;
		end else if (!block && if_finish) begin
			prev_pc <= pc_in;
			pc_out <= pc_in;
			instr_out <= instr_in;
			id_finish <= 1'b1;
			// 解码寄存器地址
			rs1 <= instr_in[19:15];
			rs2 <= instr_in[24:20];
			rd <= instr_in[11:7];
			
			// 根据指令类型生成立即数
			case (instr_in[6:0])
				7'b0010011, 7'b0011011: imm <= {{52{instr_in[31]}}, instr_in[31:20]};                                    // I-type
				7'b0000011:             imm <= {{52{instr_in[31]}}, instr_in[31:20]};                                    // Load
				7'b0100011:             imm <= {{52{instr_in[31]}}, instr_in[31:25], instr_in[11:7]};                   // Store
				7'b1100011:             imm <= {{52{instr_in[31]}}, instr_in[7], instr_in[30:25], instr_in[11:8],1'b0}; // Branch
				7'b0110111: imm <= {{32{instr_in[31]}},instr_in[31:12], 12'b0}; // LUI
				7'b0010111: imm <= {{32{instr_in[31]}},instr_in[31:12], 12'b0}; // AUIPC
				7'b1101111:             imm <= {{44{instr_in[31]}}, instr_in[19:12], instr_in[20], instr_in[30:21], 1'b0}; // JAL
				7'b1100111:             imm <= {{52{instr_in[31]}}, instr_in[31:20]};                                    // JALR
				7'b1110011:             begin
					// CSR指令处理
					case (instr_in[14:12])
						3'b001, 3'b010, 3'b011: // CSRRW, CSRRS, CSRRC - 使用寄存器值
							imm <= {52'b0, instr_in[31:20]};                              // CSR地址
						3'b101, 3'b110, 3'b111: // CSRRWI, CSRRSI, CSRRCI - 使用立即数
							imm <= {59'b0, instr_in[19:15]};                              // zimm值（零扩展的5位立即数）
						default: 
							imm <= 64'b0;
					endcase
				end
				default:                imm <= 64'b0;
			endcase
			
			// 控制信号生成
			case (instr_in[6:0])
				7'b0110011, 7'b0111011: begin // R-type算术指令
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100000;
					mem_size <= 2'b00;
				end
				
				7'b0010011, 7'b0011011: begin // I-type算术指令
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
					mem_size <= 2'b00;
				end
				
				7'b0000011: begin // 加载指令
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b110011;
					case (instr_in[14:12])
						3'b000:  mem_size <= 2'b00; // 字节
						3'b001:  mem_size <= 2'b01; // 半字
						3'b010:  mem_size <= 2'b10; // 字
						3'b011:  mem_size <= 2'b11; // 双字
						default: mem_size <= 2'b00;
					endcase
				end
				
				7'b0100011: begin // 存储指令
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b001010;
					case (instr_in[14:12])
						3'b000:  mem_size <= 2'b00; // 字节
						3'b001:  mem_size <= 2'b01; // 半字
						3'b010:  mem_size <= 2'b10; // 字
						3'b011:  mem_size <= 2'b11; // 双字
						default: mem_size <= 2'b00;
					endcase
				end
				
				7'b1100011: begin // 分支指令 (beq, bne, blt, bge, bltu, bgeu)
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b000100;
					mem_size <= 2'b00;
				end
				
				7'b0110111, 7'b0010111: begin // LUI, AUIPC
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
					mem_size <= 2'b00;
				end
				
				7'b1101111: begin // JAL
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100110;
					mem_size <= 2'b00;
				end
				
				7'b1100111: begin // JALR
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100110;
					mem_size <= 2'b00;
				end
				
				7'b1110011: begin // CSR instructions (CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI)
					// 根据不同CSR指令类型设置不同的控制信号
					case (instr_in[14:12])
						3'b001: begin // CSRRW
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
							mem_size <= 2'b00;
						end
						3'b010: begin // CSRRS
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
							mem_size <= 2'b00;
						end
						3'b011: begin // CSRRC
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
							mem_size <= 2'b00;
						end
						3'b101: begin // CSRRWI
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
							mem_size <= 2'b00;
						end
						3'b110: begin // CSRRSI
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
							mem_size <= 2'b00;
						end
						3'b111: begin // CSRRCI
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b100010;
							mem_size <= 2'b00;
						end
						default: begin
							{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b000000;
							mem_size <= 2'b00;
						end
					endcase
				end
				
				7'b0101111: begin // 原子指令 (AMO, LR, SC)
					// 原子指令需要读写内存，并写回寄存器
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b111010;
					mem_size <= 2'b10; // 32位原子操作
					imm <= 64'b0; // 原子指令使用寄存器+0的地址模式
				end
				
				default: begin
					{reg_write, mem_read, mem_write, branch, alu_src, mem_to_reg} <= 6'b000000;
					mem_size <= 2'b00;
				end
			endcase
		end else if (!block && (prev_pc == pc_in)) begin
			id_finish <= 1'b0;
			pc_out <= pc_out;
			instr_out <= instr_out;
			rs1 <= 0;
			rs2 <= 0;
			rd <= 0;
			imm <= 0;
			mem_size <= 0;
			reg_write <= 0;
			mem_read <= 0;
			mem_write <= 0;
			branch <= 0;
			alu_src <= 0;
			mem_to_reg <= 0;
		end else if (block) begin
			// 阻塞时保持所有输出状态不变，但确保id_finish为低
			// 这样可以防止EX阶段继续处理
			id_finish <= 1'b0;
		end
	end 

endmodule

`endif
