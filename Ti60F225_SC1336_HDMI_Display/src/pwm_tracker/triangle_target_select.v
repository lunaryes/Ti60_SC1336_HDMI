// Pixel bbox edges: 8*xmin .. 8*xmax+7. Reference for cx2=xmin+xmax+1 is GRID_W.
module triangle_target_select #(
    parameter integer MAX_OBJECTS = 8,
    parameter integer GRID_W      = 160,
    parameter integer GRID_H      = 90,
    parameter [3:0]   TRI_CLASS   = 4'd1,      // shape_geometry_engine.v:226 -> 1 = 三角形
    parameter integer MIN_SIZE    = 2,         // 最小边（格），滤掉噪点
    parameter integer MAX_SIZE    = 128,        // 最大边（格），滤掉整屏误判
    parameter integer LOST_FRAMES = 3,
    parameter integer MATCH_RADIUS = 40, // doubled grid units: 160 pixels
    parameter integer H_WEIGHT    = 4            // 水平误差权重（垂直不做动作，只降权）
)(
    input  wire        clk,          // clk_pixel 74.25 MHz
    input  wire        rst_n,
    input  wire        clear_i,      // invalidate association on pause/rearm
    input  wire        vsync_i,      // 与 objects_i 同域

    input  wire [MAX_OBJECTS*36-1:0] objects_i,
    input  wire [MAX_OBJECTS-1:0]    valid_i,

    output reg  signed [9:0] ex_o,   // 水平误差，半格单位，右正左负
    output reg  signed [9:0] ey_o,   // 垂直误差，半格单位（本设计不使用，留作诊断）
    output reg         found_o,      // 本帧是否找到三角形
    output reg  [5:0]  size_w_o,
    output reg  [5:0]  size_h_o,
    output reg         frame_o       // 每帧一个脉冲，用作下游 CDC 的发送请求
);

localparam integer IW = (MAX_OBJECTS < 2) ? 1 : $clog2(MAX_OBJECTS);

localparam [9:0] CX2_REF = GRID_W;   // 159
localparam [9:0] CY2_REF = GRID_H;   // 89

//---------------------------------------------------------------- 槽选择
// 用 for + 相等比较展开成常量偏移的片选，避免非常量位宽的部分选择
// （36 bit 非 2 的幂，Efinity 对变量索引会生成很差的桶形移位器）。
reg [IW-1:0] idx;
reg [35:0]   obj;
integer k;

always @(*) begin
    obj = 36'd0;
    for (k = 0; k < MAX_OBJECTS; k = k + 1)
        if (idx == k) obj = objects_i[k*36 +: 36];
end

wire [3:0] cls  = obj[35:32];
wire [7:0] xmin = obj[31:24];
wire [7:0] xmax = obj[23:16];
wire [7:0] ymin = obj[15:8];
wire [7:0] ymax = obj[7:0];

// 宽高用 9 bit：若上游给来 xmax < xmin 的坏数据，减法下溢成 511，
// 会被 MAX_SIZE 条件自然滤掉，不需要额外判断。
wire [8:0] bw = {1'b0, xmax} - {1'b0, xmin} + 9'd1;
wire [8:0] bh = {1'b0, ymax} - {1'b0, ymin} + 9'd1;

wire [9:0] cx2 = {2'b0, xmin} + {2'b0, xmax} + 10'd1;
wire [9:0] cy2 = {2'b0, ymin} + {2'b0, ymax} + 10'd1;

wire [9:0] dx = (cx2 >= CX2_REF) ? (cx2 - CX2_REF) : (CX2_REF - cx2);
wire [9:0] dy = (cy2 >= CY2_REF) ? (cy2 - CY2_REF) : (CY2_REF - cy2);

reg locked;
reg [9:0] last_cx,last_cy;
reg [8:0] last_w,last_h;
reg [7:0] misses;
wire [9:0] mx=cx2>=last_cx?cx2-last_cx:last_cx-cx2;
wire [9:0] my=cy2>=last_cy?cy2-last_cy:last_cy-cy2;
wire [8:0] mw=bw>=last_w?bw-last_w:last_w-bw;
wire [8:0] mh=bh>=last_h?bh-last_h:last_h-bh;
wire match_ok=mx<=MATCH_RADIUS && my<=MATCH_RADIUS &&
 ({1'b0,bw}<<1)>={1'b0,last_w} && {1'b0,bw}<=({1'b0,last_w}<<1) &&
 ({1'b0,bh}<<1)>={1'b0,last_h} && {1'b0,bh}<=({1'b0,last_h}<<1);
wire [13:0] cost_w=locked? ((mx<<2)+my+mw+mh):((dx<<2)+dy);

wire cand = valid_i[idx]
         && (cls == TRI_CLASS)
 && xmin<=xmax && ymin<=ymax && xmax<GRID_W && ymax<GRID_H
         && (bw >= MIN_SIZE) && (bw <= MAX_SIZE)
         && (bh >= MIN_SIZE) && (bh <= MAX_SIZE);

//---------------------------------------------------------------- 逐帧扫描
localparam [1:0] IDLE = 2'd0, SCAN = 2'd1, DONE = 2'd2;

reg [1:0]  st;
reg        vs_d;
reg [13:0] best_cost;
reg [9:0]  best_cx, best_cy;
reg [8:0]  best_w,  best_h;
reg        best_v;

// DONE 拍组合出差值（best_* 此时已经稳定）
wire signed [11:0] ex_c = $signed({2'b0, best_cx}) - $signed({2'b0, CX2_REF});
wire signed [11:0] ey_c = $signed({2'b0, best_cy}) - $signed({2'b0, CY2_REF});

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        locked<=0;last_cx<=0;last_cy<=0;last_w<=0;last_h<=0;misses<=0;
        st        <= IDLE;
        vs_d      <= 1'b0;
        idx       <= {IW{1'b0}};
        best_cost <= 14'h3FFF;
        best_cx   <= 10'd0;
        best_cy   <= 10'd0;
        best_w    <= 9'd0;
        best_h    <= 9'd0;
        best_v    <= 1'b0;
        ex_o      <= 10'sd0;
        ey_o      <= 10'sd0;
        found_o   <= 1'b0;
        size_w_o  <= 6'd0;
        size_h_o  <= 6'd0;
        frame_o   <= 1'b0;
    end else begin
        vs_d    <= vsync_i;
        frame_o <= 1'b0;

        case (st)
        IDLE: begin
            // objects/valid 在 vsync 上升沿整体更新，等一拍再开始扫，
            // 保证扫到的是本帧的新内容。
            if (vsync_i && !vs_d) begin
                idx       <= {IW{1'b0}};
                best_cost <= 14'h3FFF;
                best_v    <= 1'b0;
                st        <= SCAN;
            end
        end

        SCAN: begin
            if (cand && (!locked || match_ok) && (cost_w < best_cost)) begin
                best_cost <= cost_w;
                best_v    <= 1'b1;
                best_cx   <= cx2;
                best_cy   <= cy2;
                best_w    <= bw;
                best_h    <= bh;
            end
            if (idx == MAX_OBJECTS - 1) st <= DONE;
            else                        idx <= idx + 1'b1;
        end

        DONE: begin
            if(best_v) begin
                locked<=1;misses<=0;last_cx<=best_cx;last_cy<=best_cy;
                last_w<=best_w;last_h<=best_h;
            end else if(locked) begin
                if(misses>=LOST_FRAMES-1) begin locked<=0;misses<=0;end
                else misses<=misses+1'b1;
            end
            found_o  <= best_v;
            ex_o     <= ex_c[9:0];
            ey_o     <= ey_c[9:0];
            size_w_o <= best_w[5:0];
            size_h_o <= best_h[5:0];
            frame_o  <= 1'b1;          // 每帧必有且只有一个脉冲（含"没找到"的帧）
            st       <= IDLE;
        end

        default: st <= IDLE;
        endcase
        if(clear_i) begin locked<=0;misses<=0;st<=IDLE;found_o<=0;frame_o<=0;end
    end
end

endmodule
