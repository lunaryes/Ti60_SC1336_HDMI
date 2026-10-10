`timescale 1ns / 1ps

// Frame-based grayscale histogram equalization.
//
// During one frame, pixels are counted in one histogram bank.  At the next
// vsync rising edge the banks are exchanged: the completed bank is converted
// into a 256-entry lookup table while the other bank begins counting the new
// frame.  The lookup table is therefore applied to the following frame.
//
// A normal 1280 x 720 frame has 921600 active pixels.  The implementation
// deliberately uses only de_i-qualified pixels; horizontal/vertical blanking
// never changes the histogram.
module histogram_equalization #(
    parameter integer FRAME_PIXELS = 1280 * 720,
    parameter integer BIN_W        = 20
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       vsync_i,
    input  wire       de_i,
    input  wire [7:0] gray_i,

    output reg        vsync_o,
    output reg        de_o,
    output reg  [7:0] gray_o
);

    // Two banks let one frame be counted while the preceding frame is read.
    reg [BIN_W-1:0] hist0 [0:255];
    reg [BIN_W-1:0] hist1 [0:255];
    reg [7:0]       equal_lut [0:255];

    reg             active_bank;
    reg             build_bank;
    reg             build_busy;
    reg             lut_valid;
    reg             vsync_d;
    reg [7:0]       build_addr;
    reg [BIN_W:0]   cdf_sum;

    wire            vsync_rise = vsync_i & ~vsync_d;
    wire [BIN_W-1:0] build_count = build_bank ? hist1[build_addr]
                                             : hist0[build_addr];
    wire [BIN_W:0]   cdf_next = cdf_sum + build_count;

    // cdf_next is at most FRAME_PIXELS.  The division is by a compile-time
    // parameter, so synthesis can implement it as constant arithmetic.
    wire [BIN_W+8:0] lut_numerator = cdf_next * 8'd255;
    wire [7:0]       lut_value = (FRAME_PIXELS != 0)
                              ? (lut_numerator / FRAME_PIXELS)
                              : 8'd0;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_bank <= 1'b0;
            build_bank  <= 1'b0;
            build_busy  <= 1'b0;
            lut_valid   <= 1'b0;
            vsync_d     <= 1'b0;
            build_addr  <= 8'd0;
            cdf_sum     <= {(BIN_W+1){1'b0}};

            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            gray_o  <= 8'd0;

            for (i = 0; i < 256; i = i + 1) begin
                hist0[i]     <= {BIN_W{1'b0}};
                hist1[i]     <= {BIN_W{1'b0}};
                equal_lut[i] <= 8'd0;
            end
        end else begin
            vsync_d <= vsync_i;

            // The output timing follows the input timing by one clk.
            vsync_o <= vsync_i;
            de_o    <= de_i;
            // Before the first complete histogram exists, preserve the
            // grayscale image instead of producing a black first frame.
            gray_o  <= de_i ? (lut_valid ? equal_lut[gray_i] : gray_i) : 8'd0;

            // Count active grayscale pixels in the bank assigned to this
            // frame.  build_bank always denotes the inactive bank.
            if (de_i) begin
                if (active_bank)
                    hist1[gray_i] <= hist1[gray_i] + {{(BIN_W-1){1'b0}}, 1'b1};
                else
                    hist0[gray_i] <= hist0[gray_i] + {{(BIN_W-1){1'b0}}, 1'b1};
            end

            // A rising vsync marks the boundary between two frames.  The
            // former active bank holds the just-finished frame histogram.
            if (vsync_rise) begin
                build_bank  <= active_bank;
                active_bank <= ~active_bank;
                build_busy  <= 1'b1;
                build_addr  <= 8'd0;
                cdf_sum     <= {(BIN_W+1){1'b0}};
            end else if (build_busy) begin
                // One histogram bin is converted and cleared per clock.
                // This takes 256 clocks, normally well inside vertical blank.
                equal_lut[build_addr] <= lut_value;
                cdf_sum               <= cdf_next;

                if (build_bank)
                    hist1[build_addr] <= {BIN_W{1'b0}};
                else
                    hist0[build_addr] <= {BIN_W{1'b0}};

                if (build_addr == 8'hff) begin
                    build_busy <= 1'b0;
                    lut_valid  <= 1'b1;
                end else begin
                    build_addr <= build_addr + 8'd1;
                end
            end
        end
    end

endmodule
