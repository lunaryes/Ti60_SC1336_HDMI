`timescale 1ns / 1ps
//=====================================================================
// gray_hist_uart —— 片内 256 bin 灰度直方图 + 串口回传（M6）
//---------------------------------------------------------------------
// 为什么做在片上：现场没有 HDMI 采集卡，抓不到无损帧。这个模块直接在
//   image_pipeline_top 内部 rgb_to_gray 的输出上统计 Y 的分布（也就是 Canny
//   实际看到的灰度），测完通过现有 UART 回传 256 个 bin。主机侧
//   sim/read_hist.py 解析并打印与 gray_hist.py 同口径的统计量。
//
// 统计口径：
//   · 只统计 de_i=1 的像素，bin = y_i[7:0]；
//   · 一次测量 = 两个 vsync 上升沿之间的**完整一帧**；
//   · 每帧最多 1280×720 = 921600 个像素，计数器 20 bit（1,048,575）不饱和；
//   · 帧尾把实际像素数 frame_total 一并回传，主机用它校验 sum(bins)==total。
//
// 一次测量的时序（收到 hist_req_i 之后）：
//   H_IDLE --req--> H_WAIT_VS --vs↑--> H_ACC --vs↑--> H_DRAIN --> H_DUMP --> H_CLEAR
//     --(还有请求)--> H_WAIT_VS / --(没有)--> H_IDLE
//   所以从发命令到收完数据最坏约 2 帧 + 778 字节 ≈ 50 ms；host 超时给 1 s 足够。
//   注意：测量的是**请求之后**的完整帧，不是请求时刻正在显示的那一帧。
//
// 串口协议（460800 8N1，二进制，共 778 字节）：
//   [0]     0xA5          magic
//   [1]     0x5A
//   [2]     0x48          'H'
//   [3..4]  frame_id      16 bit little-endian（每次测量 +1）
//   [5..7]  frame_total   24 bit little-endian（本帧实际 de 像素数）
//   [8..775] 256 bins × 3 字节，little-endian 20 bit（bin0 在前）
//   [776..777] checksum   16 bit little-endian = bytes[0..775] 之和 mod 65536
//
// 与 uart_cmd 的配合（顶层 example_top 负责）：
//   · uart_cmd 收到 'H' 出 hist_req_o 单拍；
//   · 本模块 busy_o=1 期间，顶层把 uart_cmd 的 rx_i 拉高（屏蔽新命令），
//     避免流回传期间又插进来一条 'R' 把 TX 抢走；
//   · 本模块 tx_busy_o=1 时顶层把 uart_tx_o 切到本模块的 tx_o；
//   · 真正开始发流之前还会等 cmd_tx_busy_i=0，保证 uart_cmd 的在途回包先发完。
//   主机协议：发一个字节 'H'（0x48），然后读满 778 字节，期间不要发别的。
//
// 资源：256×20 同步读 RAM（实测 1 块 EFX_RAM10），无 DSP、无乘法。
//   累加器用"同步读 + 上一拍写回放"处理同 bin 连续像素（读和写在同一个
//   沿时读数据是旧值），写地址/数据延迟 2 拍，回放寄存器 1 组。
//=====================================================================
module gray_hist_uart #(
    parameter integer CLK_FREQ = 74250000,   // clk_pixel
    parameter integer BAUD     = 460800
)(
    input  wire       clk,
    input  wire       rst_n,

    // 待统计的 Y 像素流（与 image_pipeline_top 内部 rgb_to_gray 输出同拍）
    input  wire [7:0] y_i,
    input  wire       de_i,
    input  wire       vsync_i,      // 帧有效电平

    input  wire       hist_req_i,   // 来自 uart_cmd 的 'H'，单拍
    input  wire       cmd_tx_busy_i,// uart_cmd 的 TX 忙（含待发请求）

    output wire       busy_o,       // 测量/回传期间为 1：顶层用它屏蔽 uart_cmd 的 RX
    output wire       tx_busy_o,    // 正在占用 TX 引脚：顶层用它做 TX 二选一
    output wire       tx_o
);

    //-----------------------------------------------------------------
    // 帧状态机
    //-----------------------------------------------------------------
    localparam [2:0] H_CLEAR     = 3'd0,
                     H_IDLE      = 3'd1,
                     H_WAIT_VS   = 3'd2,
                     H_ACC       = 3'd3,
                     H_DRAIN     = 3'd4,
                     H_DUMP_INIT = 3'd5,
                     H_DUMP      = 3'd6;

    reg [2:0]  state;
    reg [7:0]  clear_addr;
    reg [1:0]  drain_cnt;
    reg        req_pending;
    reg [15:0] frame_id;
    reg [23:0] total_cnt;
    reg [23:0] frame_total;

    reg        vs_d;
    wire       vs_rise = vsync_i & ~vs_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vs_d <= 1'b0;
        else        vs_d <= vsync_i;
    end

    //-----------------------------------------------------------------
    // 直方图 RAM：256 × 20 bit，同步读 1 拍
    //   读和写在同一沿命中同一地址时是 read-before-write（旧值），
    //   累加器的回放寄存器负责补上这一拍。
    //-----------------------------------------------------------------
    reg [19:0] hist_mem [0:255];
    reg [19:0] mem_rd;
    reg        mem_we;
    reg [7:0]  mem_wa;
    reg [19:0] mem_wd;
    wire [7:0] mem_ra;

    always @(posedge clk) begin
        if (mem_we) hist_mem[mem_wa] <= mem_wd;
        mem_rd <= hist_mem[mem_ra];
    end

    //-----------------------------------------------------------------
    // 累加流水：第 0 级发读地址，第 1 级拿读数据 + 回放，第 2 级写回
    //   回放条件：上一拍写回的地址 == 当前读数据的地址。
    //   这一拍正好覆盖"读沿和写沿重合"的连续同 bin 像素（read-before-write）。
    //-----------------------------------------------------------------
    reg [7:0]  acc_ra;
    reg        acc_rv;
    reg [7:0]  acc_ra1;
    reg        acc_rv1;
    reg        acc_fwd_v;
    reg [7:0]  acc_fwd_a;
    reg [19:0] acc_fwd_d;

    wire [19:0] acc_base  = (acc_fwd_v && (acc_fwd_a == acc_ra1)) ? acc_fwd_d : mem_rd;
    wire [19:0] acc_new   = (acc_base == 20'hFFFFF) ? acc_base : (acc_base + 20'd1);

    wire        acc_we = ((state == H_ACC) || (state == H_DRAIN)) && acc_rv1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_ra    <= 8'd0;
            acc_rv    <= 1'b0;
            acc_ra1   <= 8'd0;
            acc_rv1   <= 1'b0;
            acc_fwd_v <= 1'b0;
            acc_fwd_a <= 8'd0;
            acc_fwd_d <= 20'd0;
        end else begin
            // 第 0 级：只在 H_ACC 收像素
            acc_ra <= y_i;
            acc_rv <= (state == H_ACC) && de_i;

            // 第 1 级：地址和数据对齐（RAM 同步读 1 拍）
            acc_ra1 <= acc_ra;
            acc_rv1 <= acc_rv;

            // 第 2 级：写回；同时把这一拍的写回值留给下一个像素做回放
            acc_fwd_a <= acc_ra1;
            acc_fwd_d <= acc_new;
            acc_fwd_v <= acc_rv1;
        end
    end

    // 写端口：H_CLEAR 时清零，其余时间给累加器
    always @(*) begin
        if (state == H_CLEAR) begin
            mem_we = 1'b1;
            mem_wa = clear_addr;
            mem_wd = 20'd0;
        end else begin
            mem_we = acc_we;
            mem_wa = acc_ra1;
            mem_wd = acc_new;
        end
    end

    //-----------------------------------------------------------------
    // 发送子状态机
    //-----------------------------------------------------------------
    localparam [1:0] D_IDLE = 2'd0, D_RD1 = 2'd1, D_RD2 = 2'd2, D_WAIT = 2'd3;

    localparam [9:0] NBYTES   = 10'd778;   // 8 头 + 768 bin + 2 校验
    localparam [9:0] HDR_END  = 10'd8;
    localparam [9:0] BIN_END  = 10'd776;

    reg [1:0]  dstate;
    reg [9:0]  byte_idx;
    reg [15:0] chk;
    reg [7:0]  bin_idx;
    reg [1:0]  byte_sel;
    reg        bin_valid;
    reg [7:0]  dump_ra;
    reg [19:0] bin_data;
    reg        tx_start;
    reg [7:0]  tx_data;

    wire       tx_busy;
    wire       tx_done;

    // 读地址：只有 D_RD1 这一拍读 bin；其余时间读什么无所谓（结果被忽略）
    assign mem_ra = ((state == H_DUMP) && (dstate == D_RD1)) ? dump_ra : acc_ra;

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart_tx (
        .clk     (clk),
        .rst_n   (rst_n),
        .start_i (tx_start),
        .data_i  (tx_data),
        .busy_o  (tx_busy),
        .tx_o    (tx_o),
        .done_o  (tx_done)
    );

    // 头部字节（byte_idx = 0..7）
    function [7:0] hdr_byte;
        input [2:0]  idx;
        input [15:0] fid;
        input [23:0] tot;
        begin
            case (idx)
                3'd0:    hdr_byte = 8'hA5;
                3'd1:    hdr_byte = 8'h5A;
                3'd2:    hdr_byte = 8'h48;      // 'H'
                3'd3:    hdr_byte = fid[7:0];
                3'd4:    hdr_byte = fid[15:8];
                3'd5:    hdr_byte = tot[7:0];
                3'd6:    hdr_byte = tot[15:8];
                default: hdr_byte = tot[23:16];
            endcase
        end
    endfunction

    reg [7:0] bsel;   // 本拍要发的组合字节

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= H_CLEAR;
            clear_addr  <= 8'd0;
            drain_cnt   <= 2'd0;
            req_pending <= 1'b0;
            frame_id    <= 16'd0;
            total_cnt   <= 24'd0;
            frame_total <= 24'd0;

            dstate      <= D_IDLE;
            byte_idx    <= 10'd0;
            chk         <= 16'd0;
            bin_idx     <= 8'd0;
            byte_sel    <= 2'd0;
            bin_valid   <= 1'b0;
            dump_ra     <= 8'd0;
            bin_data    <= 20'd0;
            tx_start    <= 1'b0;
            tx_data     <= 8'd0;
        end else begin
            tx_start <= 1'b0;

            // 任何状态下收到 'H' 都先记下来，等当前这轮结束再服务
            if (hist_req_i) req_pending <= 1'b1;

            case (state)
                //-----------------------------------------------------
                // 清零：256 拍。上电和每次回传结束后都走一遍。
                //-----------------------------------------------------
                H_CLEAR: begin
                    if (clear_addr == 8'd255) begin
                        clear_addr <= 8'd0;
                        // 清零期间又来的请求，在这里接着测
                        if (req_pending) state <= H_WAIT_VS;
                        else             state <= H_IDLE;
                    end else begin
                        clear_addr <= clear_addr + 1'b1;
                    end
                end

                //-----------------------------------------------------
                // 空闲：等 'H'
                //-----------------------------------------------------
                H_IDLE: begin
                    if (hist_req_i || req_pending) begin
                        req_pending <= 1'b1;
                        state       <= H_WAIT_VS;
                    end
                end

                //-----------------------------------------------------
                // 等下一帧的 vsync 上升沿，保证从帧首开始统计
                //-----------------------------------------------------
                H_WAIT_VS: begin
                    if (vs_rise) begin
                        frame_id    <= frame_id + 1'b1;
                        total_cnt   <= 24'd0;
                        req_pending <= 1'b0;    // 本轮请求已消费
                        state       <= H_ACC;
                    end
                end

                //-----------------------------------------------------
                // 累加一整帧
                //-----------------------------------------------------
                H_ACC: begin
                    if (de_i) total_cnt <= total_cnt + 1'b1;
                    if (vs_rise) begin
                        // 若 vsync 与最后一个 de 同拍（真实时序里不会，消隐期
                        // 才抬 vsync），这一拍像素仍会进累加流水，总数要带上它，
                        // 否则主机的 sum(bins)==total 校验会差 1。
                        frame_total <= total_cnt + (de_i ? 24'd1 : 24'd0);
                        drain_cnt   <= 2'd0;
                        state       <= H_DRAIN;
                    end
                end

                //-----------------------------------------------------
                // 排空累加流水（最多 2 拍写回），然后准备发流
                //-----------------------------------------------------
                H_DRAIN: begin
                    if (drain_cnt == 2'd3) state <= H_DUMP_INIT;
                    else                   drain_cnt <= drain_cnt + 1'b1;
                end

                //-----------------------------------------------------
                // 等 uart_cmd 的在途回包发完，再接管 TX；同时初始化计数
                //-----------------------------------------------------
                H_DUMP_INIT: begin
                    if (!cmd_tx_busy_i) begin
                        byte_idx  <= 10'd0;
                        chk       <= 16'd0;
                        bin_idx   <= 8'd0;
                        byte_sel  <= 2'd0;
                        bin_valid <= 1'b0;
                        dstate    <= D_IDLE;
                        state     <= H_DUMP;
                    end
                end

                //-----------------------------------------------------
                // 发 778 字节：头 → 768 bin → 校验
                //-----------------------------------------------------
                H_DUMP: begin
                    case (dstate)
                        D_IDLE: begin
                            if (!tx_busy) begin
                                if (byte_idx < HDR_END) begin
                                    bsel     = hdr_byte(byte_idx[2:0], frame_id, frame_total);
                                    tx_data  <= bsel;
                                    chk      <= chk + {8'd0, bsel};
                                    tx_start <= 1'b1;
                                    byte_idx <= byte_idx + 1'b1;
                                    dstate   <= D_WAIT;
                                end else if (byte_idx < BIN_END) begin
                                    if (!bin_valid) begin
                                        // 新的一组 3 字节：先读该 bin
                                        dump_ra <= bin_idx;
                                        dstate  <= D_RD1;
                                    end else begin
                                        // bin_data 在 D_RD2 已锁存，这里只选字节
                                        case (byte_sel)
                                            2'd0:    bsel = bin_data[7:0];
                                            2'd1:    bsel = bin_data[15:8];
                                            default: bsel = bin_data[19:16];
                                        endcase
                                        tx_data  <= bsel;
                                        chk      <= chk + {8'd0, bsel};
                                        tx_start <= 1'b1;
                                        byte_idx <= byte_idx + 1'b1;
                                        if (byte_sel == 2'd2) begin
                                            byte_sel  <= 2'd0;
                                            bin_valid <= 1'b0;
                                            bin_idx   <= bin_idx + 1'b1;
                                        end else begin
                                            byte_sel <= byte_sel + 1'b1;
                                        end
                                        dstate <= D_WAIT;
                                    end
                                end else begin
                                    // 777/778：校验和（不参与累加）
                                    bsel     = (byte_idx == 10'd776) ? chk[7:0] : chk[15:8];
                                    tx_data  <= bsel;
                                    tx_start <= 1'b1;
                                    byte_idx <= byte_idx + 1'b1;
                                    dstate   <= D_WAIT;
                                end
                            end
                        end

                        // 读 bin：dump_ra 在 D_RD1 稳定，RAM 在沿上采它
                        D_RD1: dstate <= D_RD2;

                        // mem_rd 此时就是 bin 数据
                        D_RD2: begin
                            bin_data  <= mem_rd;
                            bin_valid <= 1'b1;
                            dstate    <= D_IDLE;
                        end

                        // 等一个字节发完
                        D_WAIT: begin
                            if (tx_done) begin
                                if (byte_idx == NBYTES) begin
                                    clear_addr <= 8'd0;
                                    dstate     <= D_IDLE;
                                    state      <= H_CLEAR;
                                end else begin
                                    dstate <= D_IDLE;
                                end
                            end
                        end

                        default: dstate <= D_IDLE;
                    endcase
                end

                default: state <= H_IDLE;
            endcase
        end
    end

    // busy_o：从请求被接受到清零结束，顶层用它屏蔽 uart_cmd 的 RX。
    // 组合上带上 hist_req_i，保证收到 'H' 的当拍就开始屏蔽。
    assign busy_o = (state != H_IDLE) || hist_req_i;

    // 只有真正发流时占用 TX 引脚；H_DUMP_INIT 可能还在等 uart_cmd 发完，
    // 那段时间不能切走。
    assign tx_busy_o = (state == H_DUMP);

endmodule
