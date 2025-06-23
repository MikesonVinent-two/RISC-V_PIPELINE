`ifndef __CBUSARBITER_SV
`define __CBUSARBITER_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "util/MMU_module.sv"
`else

`endif
/**
 * this implementation is not efficient, since
 * it adds one cycle lantency to all requests.
 * 
 * 这个实现不够高效，因为它会给所有请求增加一个周期的延迟。
 */

module CBusArbiter
    import common::*;
#(
    parameter int NUM_INPUTS = 2,  // NOTE: NUM_INPUTS >= 1  // 输入端口数量，必须大于等于1

    localparam int MAX_INDEX = NUM_INPUTS - 1  // 最大索引值
) (
    input logic clk, reset,  // 时钟和复位信号
    input word_t satp,  // 页表基址寄存器
    input logic [1:0] priviledge_mode,  // 特权模式

    input  cbus_req_t  [MAX_INDEX:0] ireqs,  // 输入请求数组
    output cbus_resp_t [MAX_INDEX:0] iresps,  // 输出响应数组
    output cbus_req_t  oreq,  // 输出请求
    input  cbus_resp_t oresp,  // 输入响应

    output logic skip,  // 跳过标志
    output logic MMU_exception  // MMU异常标志
);

    cbus_req_t req_virt;  // 虚拟请求
    cbus_resp_t resp_virt;  // 虚拟响应

     // MMU实例，用于地址转换
    MMU_module mmu_module (
        .clk(clk),
        .rst(reset),
        .req_in(req_virt),
        .req_out(oreq),
        .resp_in(oresp),
        .resp_out(resp_virt),
        .skip(skip),
        .MMU_exception(MMU_exception),
        .satp(satp),
        .priviledge_mode(priviledge_mode)
    );

    logic busy;  // 忙状态标志
    int index, select;  // 当前处理的索引和选择的索引
    cbus_req_t saved_req, selected_req;  // 保存的请求和选择的请求

    // 将虚拟请求连接到选中的输入请求
    // assign req_virt = ireqs[index];
    assign req_virt = busy ? ireqs[index] : '0;  // 防止提前发出请求
    assign selected_req = ireqs[select];  // 选择当前处理的请求

    // 选择一个优先的请求
    // 简单的优先级选择：选择第一个有效的请求
    always_comb begin
        select = 0;

        for (int i = 0; i < NUM_INPUTS; i++) begin
            if (ireqs[i].valid) begin
                select = i;
                break;
            end
        end
    end

    // 将响应反馈给选中的请求
    always_comb begin
        iresps = '0;  // 初始化所有响应为0

        if (busy) begin  // 如果处于忙状态
            for (int i = 0; i < NUM_INPUTS; i++) begin
                if (index == i)  // 只有当前处理的索引才获得响应
                    iresps[i] = resp_virt;
            end
        end
    end

    // 状态控制
    always_ff @(posedge clk)
    if (~reset) begin  // 非复位状态
        if (busy) begin  // 如果处于忙状态
            if (resp_virt.last)  // 如果是最后一个响应
                {busy, saved_req} <= '0;  // 清除忙状态和保存的请求
        end else begin  // 如果不处于忙状态
            // 如果有有效请求，则设置忙状态
            busy <= selected_req.valid;
            index <= select;  // 更新当前处理的索引
            saved_req <= selected_req;  // 保存当前处理的请求
        end
    end else begin  // 复位状态
        {busy, index, saved_req} <= '0;  // 清除所有状态
    end

    `UNUSED_OK({saved_req});  // 标记saved_req可能未使用
endmodule



`endif