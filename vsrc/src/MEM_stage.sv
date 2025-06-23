`ifndef __MEM_STAGE_SV
`define __MEM_STAGE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr.sv"
`else

`endif

module MEM_stage
  import common::*;
  import csr_pkg::*;
(
    input  logic              clk,
    reset,
    input  logic       [63:0] pc_in,
    input  logic       [63:0] result_in,
    input  logic              mem_read,
    input  logic              mem_write,
    input  word_t             write_data,
    input  logic       [ 1:0] mem_size,          // 00: byte, 01: halfword, 10: word
    input  logic              block,
    input  logic       [31:0] instr_in,
    // 原子指令相关输入
    input  logic              is_atomic,         // 是否为原子指令
    input  atomic_op_t        atomic_op,         // 原子操作类型
    input  reservation_t      reservation_set [1:0], // Reservation Set（从core传入）
    output logic              lr_success,        // LR指令成功标志
    output logic              sc_success,        // SC指令成功标志
    output logic              update_reservation, // 更新保留集标志
    output addr_t             reservation_addr,  // 保留地址
    output logic              clear_reservation, // 清除保留集标志
    // 现有输出信号
    output logic       [63:0] pc_out,
    output logic       [63:0] result_out,
    output dbus_req_t         dbus_req,
    input  dbus_resp_t        dbus_resp,
    output logic              mem_exception,
    output logic              finish_w,
    output logic              finish_r,
    input  logic       [ 4:0] rd_in,
    output logic       [ 4:0] rd_out,
    output logic       [63:0] ex_result_out,
    input  logic              reg_write_in,
    output logic              reg_write_out,
    input  logic              mem_to_reg_in,
    output logic              mem_to_reg_out,
    output logic              mem_result_valid,
    input  logic              ex_result_valid,
    output logic       [31:0] instr_out,
    output logic              mem_read_out,
    output logic              mem_write_out,
    output logic       [63:0] mem_addr_out,
    output logic              mem_finish,
    input  logic              ex_finish,
    input  Misalign          misalign,
    // PMP 相关输入
    input  word_t             csr_pmpaddr0,
    input  word_t             csr_pmpcfg0,
    input  logic       [ 1:0] privilege_mode
);
  logic [31:0] mem_data = 32'b0;   // 初始化mem_data
  assign finish_w = !block;
  // 移除对finish_r的阻塞赋值,只在always_ff块中使用非阻塞赋值
  logic pc_valid = 1'b0;            // 初始化pc_valid
  logic current_mem_read = 1'b0;    // 初始化current_mem_read
  logic current_mem_write = 1'b0;   // 初始化current_mem_write
  logic [63:0] last_pc = 64'b0;     // 初始化last_pc
  logic [63:0] op = 64'b0;          // 初始化op
  logic [5:0] offset = 6'b0;        // 初始化offset
  logic [2:0] count = 3'b0;         // 初始化count

  // 状态定义
  enum logic [2:0] {
    IDLE,              // 空闲状态
    REQUEST_SENT,      // 请求已发送
    RESPONSE_RECEIVED, // 响应已接收
    ATOMIC_READ,       // 原子操作读阶段
    ATOMIC_WRITE,      // 原子操作写阶段
    ATOMIC_COMPLETE    // 原子操作完成
  } state = IDLE;
  
  // 原子指令相关内部信号
  logic [63:0] atomic_read_data;    // 原子操作读取的数据
  logic [63:0] atomic_write_data;   // 原子操作要写入的数据
  logic [63:0] atomic_result;       // 原子操作结果（返回给rd的值）
  logic        reservation_match;   // 保留地址匹配标志
  logic        atomic_read_done;    // 原子读操作完成
  logic        atomic_write_done;   // 原子写操作完成
  // 保存原子指令的上下文信息
  logic [63:0] saved_atomic_addr;   // 保存的原子指令地址
  logic [63:0] saved_atomic_pc;     // 保存的原子指令PC
  logic [4:0]  saved_atomic_rd;     // 保存的原子指令目标寄存器
  logic [63:0] saved_atomic_write_data; // 保存的原子指令写入数据
  atomic_op_t  saved_atomic_op;     // 保存的原子操作类型

  // 原子操作计算函数
  function automatic logic [63:0] compute_atomic_result(
    input atomic_op_t op,
    input logic [63:0] mem_data,
    input logic [63:0] reg_data
  );
    case (op)
      AMO_SWAP: return reg_data;                           // AMOSWAP
      AMO_ADD:  return mem_data + reg_data;                // AMOADD
      AMO_XOR:  return mem_data ^ reg_data;                // AMOXOR
      AMO_AND:  return mem_data & reg_data;                // AMOAND
      AMO_OR:   return mem_data | reg_data;                // AMOOR
      AMO_MIN:  return ($signed(mem_data) < $signed(reg_data)) ? mem_data : reg_data;  // AMOMIN
      AMO_MAX:  return ($signed(mem_data) > $signed(reg_data)) ? mem_data : reg_data;  // AMOMAX
      AMO_MINU: return (mem_data < reg_data) ? mem_data : reg_data;  // AMOMINU
      AMO_MAXU: return (mem_data > reg_data) ? mem_data : reg_data;  // AMOMAXU
      default:  return mem_data;
    endcase
  endfunction
  
  // 保留地址检查函数
  function automatic logic check_reservation(
    input addr_t addr,
    input reservation_t reservations [1:0]
  );
    return (reservations[0].valid && (reservations[0].addr == addr)) ||
           (reservations[1].valid && (reservations[1].addr == addr));
  endfunction

  // PMP 检查函数
  function automatic pmp_check_result_t pmp_check_mem(
    input logic [63:0] addr,
    input pmp_access_type_t access_type
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
      result.allowed = (privilege_mode == 2'b11);
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
      result.allowed = (privilege_mode == 2'b11);
      result.fault = 1'b0;
    end
    
    return result;
  endfunction

  // 主状态机逻辑
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      pc_out <= 64'b0;
      result_out <= 64'b0;
      mem_exception <= 1'b0;
      state <= IDLE;
      current_mem_read <= 1'b0;
      current_mem_write <= 1'b0;
      dbus_req.valid <= 1'b0;
      dbus_req.addr <= 64'b0;
      dbus_req.strobe <= 8'b0;
      dbus_req.data <= 64'b0;
      dbus_req.size <= MSIZE1;
      dbus_req.is_atomic <= 1'b0;
      dbus_req.atomic_op <= AMO_ADD;
      last_pc <= 64'b0;
      finish_r <= 1'b0;
      offset <= 6'b0;
      count <= 3'b0;
      // 原子指令相关信号初始化
      atomic_read_data <= 64'b0;
      atomic_write_data <= 64'b0;
      atomic_result <= 64'b0;
      reservation_match <= 1'b0;
      atomic_read_done <= 1'b0;
      atomic_write_done <= 1'b0;
      lr_success <= 1'b0;
      sc_success <= 1'b0;
      update_reservation <= 1'b0;
      reservation_addr <= 64'b0;
      clear_reservation <= 1'b0;
      // 初始化保存的原子指令信息
      saved_atomic_addr <= 64'b0;
      saved_atomic_pc <= 64'b0;
      saved_atomic_rd <= 5'b0;
      saved_atomic_write_data <= 64'b0;
      saved_atomic_op <= AMO_ADD;
    end else  begin
      case (state)
        IDLE: begin
          finish_r <= 1'b0;
          offset   <= 6'b0;
          mem_read_out <= 1'b0;
          mem_write_out <= 1'b0;
          lr_success <= 1'b0;
          sc_success <= 1'b0;
          update_reservation <= 1'b0;
          clear_reservation <= 1'b0;
          
          if (misalign.valid && ex_finish) begin
              mem_finish <= 1'b1;
              state <= IDLE;
              current_mem_read <= 1'b0;
              current_mem_write <= 1'b0;
              pc_out <= pc_in;
              instr_out <= instr_in;
              result_out <= result_in;
              rd_out <= rd_in;
              ex_result_out <= result_in;
              reg_write_out <= reg_write_in;
              mem_to_reg_out <= mem_to_reg_in;
              mem_result_valid <= ex_result_valid;
          end
          // 处理原子指令
          else if (is_atomic && ex_finish && !misalign.valid) begin
            // 删除调试信息
            // 保存原子指令的上下文信息
            saved_atomic_addr <= result_in;
            saved_atomic_pc <= pc_in;
            saved_atomic_rd <= rd_in;
            saved_atomic_write_data <= write_data;
            saved_atomic_op <= atomic_op;
            
            dbus_req.addr <= result_in;
            dbus_req.valid <= 1'b1;
            dbus_req.is_atomic <= 1'b1;
            dbus_req.atomic_op <= atomic_op;
            dbus_req.size <= MSIZE4;  // 原子指令只支持32位
            dbus_req.strobe <= 8'b0;  // 读操作
            dbus_req.data <= 64'b0;
            offset[5:3] <= result_in[2:0];
            
            // 根据原子操作类型进入不同处理流程
            case (atomic_op)
              AMO_LR: begin  // LR
                state <= ATOMIC_READ;
                atomic_read_done <= 1'b0;
              end
              AMO_SC: begin  // SC
                // 检查保留地址
                reservation_match <= check_reservation(saved_atomic_addr, reservation_set);
                if (check_reservation(saved_atomic_addr, reservation_set)) begin
                  // 有匹配的保留，执行写操作
                  dbus_req.strobe <= 8'b00001111 << saved_atomic_addr[2:0];
                  dbus_req.data <= {2{write_data[31:0]}};
                  state <= ATOMIC_WRITE;
                  atomic_write_done <= 1'b0;
                end else begin
                  // 没有匹配的保留，SC失败
                  sc_success <= 1'b0;
                  result_out <= 64'b1;  // SC失败返回1
                  state <= ATOMIC_COMPLETE;
                end
              end
              default: begin  // AMO指令
                state <= ATOMIC_READ;
                atomic_read_done <= 1'b0;
              end
            endcase
          end
          else if ((mem_read | mem_write) && ex_finish && !misalign.valid) begin  // 只在PC改变时执行
            // PMP 检查
            pmp_check_result_t pmp_result;
            pmp_access_type_t access_type;
            
            // 确定访问类型
            access_type = mem_write ? PMP_ACCESS_WRITE : PMP_ACCESS_READ;
            
            // 执行PMP检查
            pmp_result = pmp_check_mem(result_in, access_type);
            
            if (!pmp_result.allowed) begin
              // PMP检查失败，产生访问错误异常
              mem_exception <= 1'b1;
              result_out <= result_in;
              rd_out <= rd_in;
              ex_result_out <= result_in;
              reg_write_out <= reg_write_in;
              mem_to_reg_out <= mem_to_reg_in;
              mem_result_valid <= ex_result_valid;
              instr_out <= instr_in;
              pc_out <= pc_in;
              mem_read_out <= mem_read;
              mem_write_out <= mem_write;
              mem_addr_out <= result_in;
              mem_finish <= 1'b1;
              state <= IDLE;
            end else begin
              // PMP检查通过，继续正常的内存访问
              current_mem_read <= mem_read;
              current_mem_write <= mem_write;
              dbus_req.addr <= result_in;
              dbus_req.valid <= 1'b1;
              offset[5 : 3] <= result_in[2 : 0];

            // 处理写内存操作
            if (mem_write) begin
              case (mem_size)
                2'b00: begin  // sb
                  dbus_req.strobe <= 8'b00000001 << result_in[2:0];
                  dbus_req.data   <= {8{write_data[7:0]}};
                  dbus_req.size   <= MSIZE1;
                end
                2'b01: begin  // sh
                  dbus_req.strobe <= 8'b00000011 << result_in[2:0];
                  dbus_req.data   <= {4{write_data[15:0]}};
                  dbus_req.size   <= MSIZE2;
                end
                2'b10: begin  // sw
                  dbus_req.strobe <= 8'b00001111 << result_in[2:0];
                  dbus_req.data   <= {2{write_data[31:0]}};
                  dbus_req.size   <= MSIZE4;
                end
                2'b11: begin  // sd
                  dbus_req.strobe <= 8'b11111111;
                  dbus_req.data   <= write_data;
                  dbus_req.size   <= MSIZE8;
                end
                default: begin
                  dbus_req.strobe <= 8'b0;
                  dbus_req.data   <= 64'b0;
                  dbus_req.size   <= MSIZE1;
                end
              endcase
            end else begin
              dbus_req.strobe <= 8'b0;
              dbus_req.data <= 64'b0;
              dbus_req.size <= mem_read ? (mem_size == 2'b00 ? MSIZE1 :  // lb/lbu
              mem_size == 2'b01 ? MSIZE2 :  // lh/lhu
              mem_size == 2'b10 ? MSIZE4 :  // lw/lwu
              MSIZE8  // ld
              ) : MSIZE1;
            end
              state <= REQUEST_SENT;
            end
          end else begin
            dbus_req.valid <= 1'b0;
            if (ex_finish) begin
              pc_out <= pc_in;
              last_pc <= pc_in;
              result_out <= result_in;
              rd_out <= rd_in;
              ex_result_out <= result_in;
              reg_write_out <= reg_write_in;
              mem_to_reg_out <= mem_to_reg_in;
              mem_result_valid <= ex_result_valid;
              instr_out <= instr_in;
              mem_read_out <= mem_read;
              mem_write_out <= mem_write;
              mem_addr_out <= 64'b0;
              mem_finish <= 1'b1;
            end else begin
              rd_out <= 0;
              mem_finish <= 1'b0;
            end
          end
        end

        REQUEST_SENT: begin  //请求已经发送了，在等待指令
          if (dbus_resp.data_ok) begin  //指令送来了
            if (current_mem_read) begin
              op = dbus_resp.data >> offset;
              case (instr_in[14:12])
                3'b000:  result_out <= {{56{op[7]}}, op[7:0]};  // lb
                3'b001:  result_out <= {{48{op[15]}}, op[15:0]};  // lh
                3'b010:  result_out <= {{32{op[31]}}, op[31:0]};  // lw
                3'b100:  result_out <= {56'b0, op[7:0]};  // lbu
                3'b101:  result_out <= {48'b0, op[15:0]};  // lhu
                3'b110:  result_out <= {32'b0, op[31:0]};  // lwu
                3'b011:  result_out <= op;  // ld
                default: result_out <= 64'b0;
              endcase
              dbus_req.valid <= 1'b0;
              mem_exception  <= 1'b0;
            end else if (mem_write) begin
              result_out <= result_in;

              mem_exception <= 1'b0;
            end else begin
              result_out <= result_in;
              mem_exception <= 1'b0;
            end
            dbus_req.valid <= 1'b0;
            dbus_req.strobe <= 8'b0;
            dbus_req.data <= 64'b0;
            dbus_req.addr <= 64'b0;
            dbus_req.size <= MSIZE1;
            // if(pc_in == 64'h00000000_80015324) begin
            //     if(count == 3'b0) begin
            //         result_out <= 64'h000000000000000a;
            //         count <= count + 1;
            //     end else  begin
            //         result_out <= 64'h0000000000000008;
            //     end 
            // end
            state <= RESPONSE_RECEIVED;  // 进入 RESPONSE_RECEIVED 状态
          end else begin
            state <= REQUEST_SENT;  // 保持在 REQUEST_SENT 状态
          end
        end

        RESPONSE_RECEIVED: begin
          finish_r <= 1'b1;
          dbus_req.valid <= 1'b0;
          last_pc <= pc_in;  // 更新last_pc
          pc_out <= pc_in;
          mem_finish <= 1'b1;
          ex_result_out <= result_in;
          reg_write_out <= reg_write_in;
          mem_to_reg_out <= mem_to_reg_in;
          mem_result_valid <= ex_result_valid;
          instr_out <= instr_in;
          rd_out <= rd_in;
          current_mem_read <= 1'b0;
          current_mem_write <= 1'b0;
          mem_read_out <= mem_read;
          mem_write_out <= mem_write;
          mem_addr_out <= result_in;
          state <= IDLE;
        end

        ATOMIC_READ: begin
          if (dbus_resp.data_ok) begin
            // 删除调试信息
            atomic_read_data <= dbus_resp.data >> offset;
            dbus_req.valid <= 1'b0;
            
            case (saved_atomic_op)
              AMO_LR: begin  // LR
                // LR指令：读取数据并设置保留
                automatic logic [63:0] read_data = dbus_resp.data >> offset;
                atomic_result <= {{32{read_data[31]}}, read_data[31:0]};  // 符号扩展
                update_reservation <= 1'b1;
                reservation_addr <= saved_atomic_addr;
                lr_success <= 1'b1;
                state <= ATOMIC_COMPLETE;
              end
              default: begin  // AMO指令
                // AMO指令：计算新值并准备写回
                automatic logic [63:0] read_data = dbus_resp.data >> offset;
                atomic_write_data <= compute_atomic_result(saved_atomic_op, read_data, saved_atomic_write_data);
                atomic_result <= {{32{read_data[31]}}, read_data[31:0]};  // 返回原始值
                
                // 准备写操作
                // 删除调试信息
                dbus_req.valid <= 1'b1;
                dbus_req.strobe <= 8'b00001111 << saved_atomic_addr[2:0];
                dbus_req.data <= {2{compute_atomic_result(saved_atomic_op, read_data, saved_atomic_write_data)[31:0]}};
                state <= ATOMIC_WRITE;
              end
            endcase
          end
        end

        ATOMIC_WRITE: begin
          if (dbus_resp.data_ok) begin
            // 删除调试信息
            dbus_req.valid <= 1'b0;
            dbus_req.strobe <= 8'b0;
            dbus_req.data <= 64'b0;
            
            case (saved_atomic_op)
              AMO_SC: begin  // SC
                sc_success <= 1'b1;
                result_out <= 64'b0;  // SC成功返回0
                clear_reservation <= 1'b1;  // SC完成后清除所有保留
              end
              default: begin  // AMO指令写完成
                // AMO指令已经在ATOMIC_READ阶段设置了result_out
              end
            endcase
            
            state <= ATOMIC_COMPLETE;
          end
        end

        ATOMIC_COMPLETE: begin
          // 删除调试信息
          finish_r <= 1'b1;
          pc_out <= saved_atomic_pc;
          last_pc <= saved_atomic_pc;
          mem_finish <= 1'b1;
          ex_result_out <= saved_atomic_addr;  // 使用保存的地址作为EX结果
          reg_write_out <= 1'b1;  // 原子指令需要写回寄存器
          mem_to_reg_out <= 1'b1;  // 使用内存结果
          mem_result_valid <= 1'b1;
          instr_out <= instr_in;  // 清空指令输出
          rd_out <= saved_atomic_rd;
          mem_read_out <= 1'b0;
          mem_write_out <= 1'b0;
          mem_addr_out <= saved_atomic_addr;
          
          // 设置最终结果
          if (saved_atomic_op == AMO_LR || saved_atomic_op != AMO_SC) begin  // LR或AMO指令
            result_out <= atomic_result;
          end
          // SC指令的结果已在ATOMIC_WRITE中设置
          
          state <= IDLE;
        end

        default: begin
          state <= IDLE;
          dbus_req.valid <= 1'b0;
          current_mem_read <= 1'b0;
          current_mem_write <= 1'b0;
        end

      endcase
    end
  end

endmodule

`endif
