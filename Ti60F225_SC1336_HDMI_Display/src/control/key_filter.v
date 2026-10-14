`timescale 1ns / 1ps

// Active-low pushbutton synchronizer and debounce filter.
//
// This project uses clk_pixel = 74.25 MHz for the image-processing pipeline.
// With the default 20 ms debounce interval, the input must remain unchanged
// for 1,485,000 clocks before it is accepted. One key_flag pulse is emitted
// only when a stable press is detected; holding or releasing the key does not
// create additional pulses.
module key_filter #(
    parameter integer CLK_FREQ_HZ = 74_250_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer CNT_MAX = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS - 1
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,

    output reg  key_flag
);

    localparam integer CNT_W = (CNT_MAX < 1) ? 1 : $clog2(CNT_MAX + 1);
    localparam [CNT_W-1:0] CNT_LIMIT = CNT_MAX;

    // The Ti60 F225 user pushbuttons are asynchronous, active-low inputs.
    // Two flip-flops reduce the probability of metastability entering the
    // debounce logic. Reset to 1 because 1 means "released".
    (* async_reg = "true" *) reg key_meta;
    (* async_reg = "true" *) reg key_sync;

    reg             key_stable;
    reg [CNT_W-1:0] stable_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
        end else begin
            key_meta <= key_in;
            key_sync <= key_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_stable <= 1'b1;
            stable_cnt <= {CNT_W{1'b0}};
            key_flag   <= 1'b0;
        end else begin
            // key_flag is deliberately a one-clock event, not a level.
            key_flag <= 1'b0;

            if (key_sync == key_stable) begin
                // No pending state change, or the contact bounced back.
                stable_cnt <= {CNT_W{1'b0}};
            end else if (stable_cnt == CNT_LIMIT) begin
                // A new level has remained stable for the full interval.
                stable_cnt <= {CNT_W{1'b0}};
                key_stable <= key_sync;

                // Emit a pulse only for the high-to-low press transition.
                if (!key_sync)
                    key_flag <= 1'b1;
            end else begin
                stable_cnt <= stable_cnt + 1'b1;
            end
        end
    end

endmodule
