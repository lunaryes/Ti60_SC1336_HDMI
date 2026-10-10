`timescale 1ns / 1ps

module pix_window_3x3 #(
    parameter integer IMG_W = 1280
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vsync_i,
    input  wire       de_i,
    input  wire [7:0] gray_i,

    output reg        vsync_o,
    output reg        de_o,
    output reg  [7:0] p11,
    output reg  [7:0] p12,
    output reg  [7:0] p13,
    output reg  [7:0] p21,
    output reg  [7:0] p22,
    output reg  [7:0] p23,
    output reg  [7:0] p31,
    output reg  [7:0] p32,
    output reg  [7:0] p33
);

    localparam integer COL_W = (IMG_W <= 2) ? 1 : $clog2(IMG_W);

    // line_buf_1 stores the preceding image row; line_buf_2 stores
    // the row before that. The arrays are intentionally not reset so
    // synthesis can infer FPGA block RAM.
    reg [7:0] line_buf_1 [0:IMG_W-1];
    reg [7:0] line_buf_2 [0:IMG_W-1];

    reg [COL_W-1:0] col_cnt;
    reg             de_d;

    reg [7:0] row1_d1;
    reg [7:0] row1_d2;
    reg [7:0] row2_d1;
    reg [7:0] row2_d2;
    reg [7:0] row3_d1;
    reg [7:0] row3_d2;

    wire line_start = de_i && !de_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= {COL_W{1'b0}};
            de_d    <= 1'b0;

            vsync_o <= 1'b0;
            de_o    <= 1'b0;

            row1_d1 <= 8'd0;
            row1_d2 <= 8'd0;
            row2_d1 <= 8'd0;
            row2_d2 <= 8'd0;
            row3_d1 <= 8'd0;
            row3_d2 <= 8'd0;

            p11 <= 8'd0;
            p12 <= 8'd0;
            p13 <= 8'd0;
            p21 <= 8'd0;
            p22 <= 8'd0;
            p23 <= 8'd0;
            p31 <= 8'd0;
            p32 <= 8'd0;
            p33 <= 8'd0;
        end else begin
            de_d    <= de_i;
            vsync_o <= vsync_i;
            de_o    <= de_i;

            if (vsync_i) begin
                col_cnt <= {COL_W{1'b0}};

                row1_d1 <= 8'd0;
                row1_d2 <= 8'd0;
                row2_d1 <= 8'd0;
                row2_d2 <= 8'd0;
                row3_d1 <= 8'd0;
                row3_d2 <= 8'd0;
            end else if (de_i) begin
                // Nonblocking assignments make the RAM reads below refer
                // to the two preceding rows before the buffers are updated.
                line_buf_2[col_cnt] <= line_buf_1[col_cnt];
                line_buf_1[col_cnt] <= gray_i;

                if (line_start) begin
                    // Replicate the first column into the missing left
                    // neighbour positions.
                    row1_d1 <= line_buf_2[0];
                    row1_d2 <= line_buf_2[0];
                    row2_d1 <= line_buf_1[0];
                    row2_d2 <= line_buf_1[0];
                    row3_d1 <= gray_i;
                    row3_d2 <= gray_i;

                    p11 <= line_buf_2[0];
                    p12 <= line_buf_2[0];
                    p13 <= line_buf_2[0];
                    p21 <= line_buf_1[0];
                    p22 <= line_buf_1[0];
                    p23 <= line_buf_1[0];
                    p31 <= gray_i;
                    p32 <= gray_i;
                    p33 <= gray_i;

                    col_cnt <= {{(COL_W-1){1'b0}}, 1'b1};
                end else begin
                    p11 <= row1_d2;
                    p12 <= row1_d1;
                    p13 <= line_buf_2[col_cnt];
                    p21 <= row2_d2;
                    p22 <= row2_d1;
                    p23 <= line_buf_1[col_cnt];
                    p31 <= row3_d2;
                    p32 <= row3_d1;
                    p33 <= gray_i;

                    row1_d2 <= row1_d1;
                    row1_d1 <= line_buf_2[col_cnt];
                    row2_d2 <= row2_d1;
                    row2_d1 <= line_buf_1[col_cnt];
                    row3_d2 <= row3_d1;
                    row3_d1 <= gray_i;

                    if (col_cnt == IMG_W - 1)
                        col_cnt <= {COL_W{1'b0}};
                    else
                        col_cnt <= col_cnt + {{(COL_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

endmodule
