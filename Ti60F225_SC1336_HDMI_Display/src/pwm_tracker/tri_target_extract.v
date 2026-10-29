`timescale 1ns / 1ps
//=====================================================================
// tri_target_extract —— 从 8 个几何描述符里挑出"三角形"并算出画面中心误差
//---------------------------------------------------------------------
// 上游接口（src/recognition/shape_geometry_top.v）：
//   objects_o : MAX_OBJECTS x 36 bit，每个描述符的字段布局见
//               shape_geometry_engine.v:52
//               { class[35:32], xmin[31:24], xmax[23:16], ymin[15:8], ymax[7:0] }
//   valid_o   : 每个槽是否有效
//   两者都在 vsync 上升沿整体更新，并稳定保持一整帧
//               （recognition_object_tracker.v:44-45）
//
// 坐标系：
//   几何网格是 160 x 90（8x8 像素池化成一格，见 shape_geometry_engine.v:45），
//   所以 xmin/xmax 是"格坐标"，不是像素坐标。
//   画面中心在 x = 79.5 格、y = 44.5 格。
//   为了全程整数运算，误差统一用"半格"表示：
//       cx2 = xmin + xmax + 1      （= 2 x 对象中心，半格单位）
//       ex  = cx2 - (GRID_W-1)     （= cx2 - 159，中心处为 0）
//   ex 的单位是半格 = 4 个原始像素（格宽 8 像素）。
//
// 选目标规则：最近中心，不是最大面积。
//   最近中心天然形成"锁定"正反馈：正在跟的那个目标离中心最近，
//   下一帧仍然会选中它。若改用最大面积，两个大小接近的三角形会
//   来回切换，导致角度指令跳变。
//
// 一个必须知道的副作用：
//   shape_geometry_engine.v:334-336 会剔除贴边目标
//   （xmin<=1 || xmax>=W-2 || ymin<=1 || ymax>=H-2 直接丢弃）。
//   所以"目标跑到画面边上"在本系统里表现为"目标消失"，
//   控制器收到的永远是"found=0"，永远收不到一个很大的误差。
//   下游 follow_ctrl 的丢失策略就是按这个事实设计的。
//=====================================================================
module tri_target_extract #(
    parameter integer MAX_OBJECTS = 8,
    parameter integer GRID_W      = 160,
    parameter integer GRID_H      = 90,
    parameter [3:0]   TRI_CLASS   = 4'd1,      // shape_geometry_engine.v:226 -> 1 = 三角形
    parameter integer MIN_SIZE    = 2,         // 最小边（格），滤掉噪点
    parameter integer MAX_SIZE    = 48,        // 最大边（格），滤掉整屏误判
    parameter integer MIN_AREA    = 9,         // 最小面积（格^2，bw*bh），滤掉细长噪块/孤立点块
                                                 // 阶段0（对照 Basys3 工程 sum_cnt>50 门限）：真目标
                                                 // 在 160x90 网格里至少 3x3 格；噪块通常 1~4 格
    parameter integer H_WEIGHT    = 4            // 水平误差权重（垂直不做动作，只降权）
)(
    input  wire        clk,          // clk_pixel 74.25 MHz
    input  wire        rst_n,
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

localparam [9:0] CX2_REF = GRID_W - 1;   // 159
localparam [9:0] CY2_REF = GRID_H - 1;   // 89

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

wire [13:0] cost_w = ({dx, 2'b00}) + {4'd0, dy};

wire [17:0] area = bw * bh;   // 9x9 bit 乘积，最大 262144，18 bit 足够

wire cand = valid_i[idx]
         && (cls == TRI_CLASS)
         && (bw >= MIN_SIZE) && (bw <= MAX_SIZE)
         && (bh >= MIN_SIZE) && (bh <= MAX_SIZE)
         && (area >= MIN_AREA);

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
            if (cand && (cost_w < best_cost)) begin
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
    end
end

endmodule
