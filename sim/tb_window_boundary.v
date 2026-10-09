//=====================================================================
// tb_window_boundary.v —— pix_window_3x3 边界行为观测
//
// 目的：记录 pix_window_3x3 的**已知边界偏差**，作为回归基线。
//       契约已于 2026-10-09 修订为"本模块不做边界处理，由 FrameBoundCrop
//       裁剪切边兜底"（原"复制边缘像素、不补零"条款作废），见 docs/ARCH.md §11.4。
//       本脚本打印"旧契约期望 vs 实测"，6 个观测点**预期全部不符**——
//       这是记录在案的既定行为，不是回归失败。
//       将来若真做了边界处理，本脚本的输出应全部变为相符。
//
// 图案：pix(row,col) = row*16 + col —— 每个像素值唯一，
//       因此任何行列错位/取错邻列都会立刻暴露成具体数值。
//
// 观测点（输出 wrow/wcol -> 窗口中心 = 图像 (wrow-1, wcol)）：
//   首行   wrow=0       : 检查"上一行"是否为本帧第 0 行（而非上一帧残留）
//   左边界 wcol=0       : 检查"左邻列"是否复制第 0 列
//   左边界 wcol=1       : 检查是否按 c==1 规则复制中心
//   末列   wcol=IMG_W-1 : 检查"右邻列"是否复制最后一列
//   末行   wrow=IMG_H-1 : 检查"下一行"是否复制最后一行
//=====================================================================
`timescale 1ns/1ps

module tb_window_boundary;

    localparam IMG_W   = 16;
    localparam IMG_H   = 8;
    localparam CLK_PER = 10;
    localparam H_BLANK = 4;

    reg clk = 1'b0;
    always #(CLK_PER/2) clk = ~clk;

    reg        rst_n   = 1'b0;
    reg        vsync_i = 1'b0;
    reg        de_i    = 1'b0;
    reg [23:0] rgb_i   = 24'd0;

    wire       vsync_o, de_o;
    wire [7:0] p11, p12, p13;
    wire [7:0] p21, p22, p23;
    wire [7:0] p31, p32, p33;

    pix_window_3x3 #(.IMG_W(IMG_W), .IMG_H(IMG_H)) dut (
        .clk(clk), .rst_n(rst_n), .vsync_i(vsync_i), .de_i(de_i), .rgb_i(rgb_i),
        .vsync_o(vsync_o), .de_o(de_o),
        .p11(p11), .p12(p12), .p13(p13),
        .p21(p21), .p22(p22), .p23(p23),
        .p31(p31), .p32(p32), .p33(p33)
    );

    //-----------------------------------------------------------------
    // 源图案：每个像素唯一
    //-----------------------------------------------------------------
    function integer src_pix;
        input integer x, y;
        begin
            src_pix = y*16 + x;
        end
    endfunction

    //-----------------------------------------------------------------
    // 旧契约期望（2026-10-09 已作废）：复制边缘像素（clamp），不补零
    // 保留它只是为了把"旧契约 vs 实测"的差的打印出来，作为既定行为基线
    //-----------------------------------------------------------------
    function integer exp_clamp;
        input integer x, y;
        integer cx, cy;
        begin
            cx = (x < 0)       ? 0     : (x >= IMG_W) ? IMG_W-1 : x;
            cy = (y < 0)       ? 0     : (y >= IMG_H) ? IMG_H-1 : y;
            exp_clamp = src_pix(cx, cy);
        end
    endfunction

    // 期望窗口：输出 (wrow,wcol) -> 中心 = 图像 (wrow-1, wcol)
    task show_expect;
        input integer wrow, wcol;
        begin
            $write("   [模块内 col=%0d bank=%0d center_bad=%0d left_bad=%0d]  ",
                   dut.col, dut.row_bank, dut.center_bad, dut.left_bad);
            $write("旧契约 ");
            $write("%3d %3d %3d / %3d %3d %3d / %3d %3d %3d",
                   exp_clamp(wcol-1, wrow-2), exp_clamp(wcol, wrow-2), exp_clamp(wcol+1, wrow-2),
                   exp_clamp(wcol-1, wrow-1), exp_clamp(wcol,   wrow-1), exp_clamp(wcol+1, wrow-1),
                   exp_clamp(wcol-1, wrow  ), exp_clamp(wcol,   wrow  ), exp_clamp(wcol+1, wrow  ));
            $write("   实测 ");
            $display("%3d %3d %3d / %3d %3d %3d / %3d %3d %3d",
                     p11,p12,p13, p21,p22,p23, p31,p32,p33);
        end
    endtask

    //-----------------------------------------------------------------
    // 输出侧行列计数（de_o 有效期内）
    //-----------------------------------------------------------------
    integer wrow, wcol;
    reg vsync_o_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wrow <= 0; wcol <= 0; vsync_o_d <= 1'b0;
        end else begin
            vsync_o_d <= vsync_o;
            if (vsync_o & ~vsync_o_d) begin
                wrow <= 0; wcol <= 0;
            end else if (de_o) begin
                if (wcol == IMG_W-1) begin wcol <= 0; wrow <= wrow + 1; end
                else                      wcol <= wcol + 1;
            end
        end
    end

    integer frame_no = 0;

    //-----------------------------------------------------------------
    // 观测：第 2 帧，只看边界位置
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && de_o && (frame_no == 2)) begin
            if ((wrow == 0) && (wcol <= 2)) begin
                $write("[帧2 首行   ] wrow=%0d wcol=%0d  ", wrow, wcol);
                show_expect(wrow, wcol);
            end
            if ((wrow == 4) && (wcol <= 1)) begin
                $write("[帧2 中行   ] wrow=%0d wcol=%0d  ", wrow, wcol);
                show_expect(wrow, wcol);
            end
            if ((wrow == 4) && (wcol == IMG_W-1)) begin
                $write("[帧2 末列   ] wrow=%0d wcol=%0d  ", wrow, wcol);
                show_expect(wrow, wcol);
            end
            if ((wrow == IMG_H-1) && (wcol == 0)) begin
                $write("[帧2 末行   ] wrow=%0d wcol=%0d  ", wrow, wcol);
                show_expect(wrow, wcol);
            end
        end
    end

    //-----------------------------------------------------------------
    // 驱动
    //-----------------------------------------------------------------
    integer x, y, f, pv;

    initial begin
        $display("================ pix_window_3x3 边界行为实测 ================");
        $display("图案 pix(row,col) = row*16 + col（每个像素唯一）");
        $display("窗口布局 p11 p12 p13 / p21 p22 p23 / p31 p32 p33");
        $display("");

        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;

        for (f = 1; f <= 2; f = f + 1) begin
            frame_no = f;

            @(negedge clk); vsync_i = 1'b1; de_i = 1'b0; rgb_i = 24'd0;
            repeat (3) @(negedge clk);
            @(negedge clk); vsync_i = 1'b0;
            repeat (2) @(negedge clk);

            for (y = 0; y < IMG_H; y = y + 1) begin
                @(negedge clk); de_i = 1'b0; rgb_i = 24'd0;
                repeat (H_BLANK-1) @(negedge clk);

                for (x = 0; x < IMG_W; x = x + 1) begin
                    @(negedge clk);
                    pv    = src_pix(x,y);
                    de_i  = 1'b1;
                    rgb_i = {pv[7:0], pv[7:0], pv[7:0]};
                end
                @(negedge clk); de_i = 1'b0; rgb_i = 24'd0;
            end
            repeat (6) @(negedge clk);
        end

        repeat (10) @(negedge clk);
        $display("");
        $display("================ 观测结束 ================");
        $finish;
    end

endmodule
