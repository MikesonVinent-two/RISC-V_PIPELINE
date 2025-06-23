`ifndef __MMU_SV
`define __MMU_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr.sv"
`endif

// 内存管理单元(MMU)模块 - 负责虚拟地址到物理地址的转换
// 实现了RISC-V标准的三级页表转换机制
module MMU_module
    import common::*;
    import csr_pkg::*;
(
    input logic clk,          // 时钟信号
    input logic rst,          // 复位信号

    input cbus_req_t req_in,  // 输入请求总线
    output cbus_req_t req_out, // 输出请求总线
    input cbus_resp_t resp_in, // 输入响应总线
    output cbus_resp_t resp_out, // 输出响应总线

    output logic skip,        // 跳过标志，用于某些特殊情况
    input word_t satp,        // 地址转换和保护寄存器，包含页表基地址
    input logic [1:0] priviledge_mode, // 当前CPU特权模式
    output logic MMU_exception  // MMU异常标志
);
    logic [63:0] data_temp;

    // 页表项检查函数
    function automatic logic check_pte_valid(logic [63:0] pte, logic is_leaf);
        // V位必须为1
        if (!pte[0]) return 0;  // V位为0，无效

        // 叶子节点：至少有一个权限位为1
        // 非叶子节点：所有权限位必须为0
        return is_leaf ? (|pte[3:1]) : (~|pte[3:1]);
    endfunction

    // MMU状态机定义
    typedef enum {
    FLUSH,  // 刷新状态 - 清除前一次转换的结果
    IDLE,   // 空闲状态 - 等待新的地址转换请求
    PTE0,   // 页表条目0 - 获取三级页表项
    PTE1,   // 页表条目1 - 获取二级页表项
    PTE2,   // 页表条目2 - 获取一级页表项
    PHY     // 物理地址 - 执行实际的物理内存访问
    } mmu_state;

    mmu_state state;  // 当前MMU状态

    // 状态机实现 - 处理虚拟地址到物理地址的转换过程
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // 复位时，将状态机设置为空闲状态
            state <= IDLE;
            data_temp <= 0;
            MMU_exception <= 0;  // 复位时清除异常标志
        end else begin
            case (state)
                FLUSH: begin
                    // 刷新状态：清除上一次MMU转换的响应结果
                    // 此状态确保各个信号正确复位，准备下一次地址转换
                    state <= IDLE;
                    resp_out.last <= 0;
                    resp_out.ready <= 0;
                    resp_out.data <= 0;
                    skip <= 0;
                    data_temp <= 0;
                    MMU_exception <= 0;  // 清除异常标志
                end

                IDLE: if (req_in.valid) begin  // 当收到有效请求时
                    if (priviledge_mode == PRIVILEGE_MODE) begin
                        // 在机器模式下，虚拟地址直接映射为物理地址，无需页表转换
                        // 直接将输入请求传递给输出
                        state <= PHY;
                        req_out.valid <= req_in.valid;
                        req_out.is_write <= req_in.is_write;
                        req_out.size <= req_in.size;
                        req_out.addr <= req_in.addr;
                        req_out.strobe <= req_in.strobe;
                        req_out.data <= req_in.data;
                        req_out.len <= req_in.len;
                        req_out.burst <= req_in.burst;
                    end else if(satp[63:60] == 4'b1000) begin
                        // 在非机器模式下，需要通过三级页表进行地址转换
                        // 首先，访问一级页表
                        state <= PTE2;
                        req_out.valid <= req_in.valid;
                        req_out.is_write <= 0;  // 页表访问始终是读操作
                        req_out.size <= MSIZE8; // 8字节访问
                        // 构造一级页表地址：页表基址 + 虚拟地址高9位作为索引 + 末尾3位为0
                        req_out.addr <= {8'b0, satp[43:0]/* PPN */, req_in.addr[38:30], 3'b0};
                        req_out.strobe <= 0;    // 读取操作
                        req_out.data <= req_in.data;
                        req_out.len <= req_in.len;
                        req_out.burst <= req_in.burst;
                    end else begin
                        state <= PHY;
                        req_out.valid <= req_in.valid;
                        req_out.is_write <= req_in.is_write;
                        req_out.size <= req_in.size;
                        req_out.addr <= req_in.addr;
                        req_out.strobe <= req_in.strobe;
                        req_out.data <= req_in.data;
                        req_out.len <= req_in.len;
                        req_out.burst <= req_in.burst;
                    end
                end

                PTE2: if (resp_in.last && resp_in.ready) begin
                    if (resp_in.data != data_temp) begin
                        // PTE2是非叶子节点
                        if (!check_pte_valid(resp_in.data, 1'b0)) begin
                            state <= FLUSH;
                            MMU_exception <= 1;
                        end else begin
                            state <= PTE1;
                            req_out.valid <= req_in.valid;
                            req_out.is_write <= 0;
                            req_out.size <= MSIZE8;
                            req_out.addr <= {8'b0, resp_in.data[53:10], req_in.addr[29:21], 3'b0};
                            req_out.strobe <= 0;
                            req_out.data <= req_in.data;
                            req_out.len <= req_in.len;
                            req_out.burst <= req_in.burst;
                            data_temp <= resp_in.data;
                        end
                    end
                end

                PTE1: if (resp_in.last && resp_in.ready) begin
                    if (resp_in.data != data_temp) begin
                        // PTE1是非叶子节点
                        if (!check_pte_valid(resp_in.data, 1'b0)) begin
                            state <= FLUSH;
                            MMU_exception <= 1;
                        end else begin
                            state <= PTE0;
                            req_out.valid <= req_in.valid;
                            req_out.is_write <= 0;
                            req_out.size <= MSIZE8;
                            req_out.addr <= {8'b0, resp_in.data[53:10], req_in.addr[20:12], 3'b0};
                            req_out.strobe <= 0;
                            req_out.data <= req_in.data;
                            req_out.len <= req_in.len;
                            req_out.burst <= req_in.burst;
                            data_temp <= resp_in.data;
                        end
                    end
                end

                PTE0: if (resp_in.last && resp_in.ready) begin
                    if (resp_in.data != data_temp) begin
                        // PTE0是叶子节点
                        if (!check_pte_valid(resp_in.data, 1'b1)) begin
                            state <= FLUSH;
                            MMU_exception <= 1;
                        end else begin
                            state <= PHY;
                            req_out.valid <= req_in.valid;
                            req_out.is_write <= req_in.is_write;
                            req_out.size <= req_in.size;
                            req_out.addr <= {8'b0, resp_in.data[53:10], req_in.addr[11:0]};
                            req_out.strobe <= req_in.strobe;
                            req_out.data <= req_in.data;
                            req_out.len <= req_in.len;
                            req_out.burst <= req_in.burst;
                            data_temp <= resp_in.data;
                            skip <= resp_in.data[29] == 0;
                        end
                    end
                end

                PHY: if (resp_in.last && resp_in.ready) begin
                    // 物理地址访问完成，内存操作已经执行
                    // 转入刷新状态，准备下一次地址转换
                    if(priviledge_mode == USER_MODE&&satp[63:60] == 4'b1000) begin
                        if(resp_in.data != data_temp) begin
                            state <= FLUSH;

                            // 清除输出请求，表示当前请求已完成
                            req_out.valid <= 0;

                            // 将内存访问结果复制到输出响应总线
                            resp_out.last <= resp_in.last;
                            resp_out.ready <= resp_in.ready;
                            resp_out.data <= resp_in.data;
                        end
                    end else begin
                        state <= FLUSH;
                            // 清除输出请求，表示当前请求已完成
                        req_out.valid <= 0;
                            // 将内存访问结果复制到输出响应总线
                        resp_out.last <= resp_in.last;
                        resp_out.ready <= resp_in.ready;
                        resp_out.data <= resp_in.data;
                        
                    end
                end
            endcase
        end
    end

endmodule

`endif