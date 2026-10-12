`timescale 1ns / 1ps

// Frame-based grayscale histogram equalization.
//
// One histogram bank collects the current frame while the other bank is
// converted to a LUT and cleared. The LUT is applied to the next frame.
// The mapping uses the measured active-pixel count and subtracts CDFmin so
// that the occupied gray range is stretched to 0..255.
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

    reg [BIN_W-1:0] hist0 [0:255];
    reg [BIN_W-1:0] hist1 [0:255];
    reg [7:0]       equal_lut [0:255];

    reg             active_bank;
    reg             build_bank;
    reg             build_busy;
    reg             lut_valid;
    reg             vsync_d;
    reg [7:0]       build_addr;
    reg [BIN_W:0]   active_pixels;
    reg [BIN_W:0]   build_pixels;
    reg [BIN_W:0]   cdf_sum;
    reg [BIN_W:0]   cdf_min;
    reg             cdf_min_valid;

    wire vsync_rise = vsync_i & ~vsync_d;
    wire [BIN_W-1:0] build_count = build_bank ? hist1[build_addr]
                                             : hist0[build_addr];
    wire [BIN_W:0] cdf_next = cdf_sum + build_count;
    wire first_nonzero = !cdf_min_valid && (build_count != {BIN_W{1'b0}});
    wire [BIN_W:0] cdf_min_next = first_nonzero ? cdf_next : cdf_min;
    wire [BIN_W:0] normalized_cdf = (cdf_next > cdf_min_next)
                                      ? (cdf_next - cdf_min_next)
                                      : {(BIN_W+1){1'b0}};
    wire [BIN_W:0] normalized_pixels = (build_pixels > cdf_min_next)
                                         ? (build_pixels - cdf_min_next)
                                         : {(BIN_W+1){1'b0}};
    wire [BIN_W+8:0] lut_numerator = normalized_cdf * 8'd255;

    // A uniform frame has no range to stretch; retain its original level.
    wire [7:0] lut_value = (normalized_pixels != {(BIN_W+1){1'b0}})
                          ? (lut_numerator / normalized_pixels)
                          : build_addr;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_bank  <= 1'b0;
            build_bank   <= 1'b0;
            build_busy   <= 1'b0;
            lut_valid    <= 1'b0;
            vsync_d      <= 1'b0;
            build_addr   <= 8'd0;
            active_pixels <= {(BIN_W+1){1'b0}};
            build_pixels <= {(BIN_W+1){1'b0}};
            cdf_sum      <= {(BIN_W+1){1'b0}};
            cdf_min      <= {(BIN_W+1){1'b0}};
            cdf_min_valid <= 1'b0;

            vsync_o <= 1'b0;
            de_o    <= 1'b0;
            gray_o  <= 8'd0;

            for (i = 0; i < 256; i = i + 1) begin
                hist0[i]     <= {BIN_W{1'b0}};
                hist1[i]     <= {BIN_W{1'b0}};
                equal_lut[i] <= i[7:0];
            end
        end else begin
            vsync_d <= vsync_i;

            vsync_o <= vsync_i;
            de_o    <= de_i;
            gray_o  <= de_i ? (lut_valid ? equal_lut[gray_i] : gray_i) : 8'd0;

            if (de_i) begin
                active_pixels <= active_pixels + {{BIN_W{1'b0}}, 1'b1};
                if (active_bank)
                    hist1[gray_i] <= hist1[gray_i] + {{(BIN_W-1){1'b0}}, 1'b1};
                else
                    hist0[gray_i] <= hist0[gray_i] + {{(BIN_W-1){1'b0}}, 1'b1};
            end

            // Ignore the initial vsync before any active pixel has arrived.
            // Normally the 256-cycle build completes within vertical blanking.
            if (vsync_rise && !build_busy &&
                (active_pixels != {(BIN_W+1){1'b0}})) begin
                build_bank    <= active_bank;
                active_bank   <= ~active_bank;
                build_pixels  <= active_pixels;
                active_pixels <= {(BIN_W+1){1'b0}};
                build_busy    <= 1'b1;
                build_addr    <= 8'd0;
                cdf_sum       <= {(BIN_W+1){1'b0}};
                cdf_min       <= {(BIN_W+1){1'b0}};
                cdf_min_valid <= 1'b0;
            end else if (build_busy) begin
                equal_lut[build_addr] <= lut_value;
                cdf_sum               <= cdf_next;

                if (first_nonzero) begin
                    cdf_min       <= cdf_next;
                    cdf_min_valid <= 1'b1;
                end

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

    // FRAME_PIXELS remains a public compatibility parameter; the measured
    // de_i pixel count is deliberately used for the actual normalization.

endmodule
