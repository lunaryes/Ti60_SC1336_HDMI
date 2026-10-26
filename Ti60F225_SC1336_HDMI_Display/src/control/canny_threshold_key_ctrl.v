`timescale 1ns / 1ps

// Runtime key controller for the Canny pipeline on VF-Ti60F225.
//
// This module directly accepts the board's two raw, active-low pushbuttons
// and contains the complete synchronizer, debounce, press-event generation,
// and threshold/smoothing update logic.  No external key_filter instance is
// required.
//
// VF-Ti60F225 button assignment (1.8 V LVCMOS):
//   key_data[0] = K1 = CSI = GPIOL_P_02 -> threshold +4
//   key_data[1] = K2 = CSO = GPIOL_N_02 -> threshold -4
// Both nets have 10 kOhm pull-ups and are connected to GND when pressed.
// K3 is CRESET_N and must not be used for threshold adjustment.
//
// K1 raises threshold by 4; K2 lowers it by 4. Bounds saturate.
// Both keys pressed together do not change the threshold. UART has priority.
// Smoothing is controlled only by UART, default standard Gaussian.
//
// The effective threshold is used as image_pipeline_top.thr_i.  That module
// maps it to Canny's double threshold as follows:
//
//     thr_high = threshold_o
//     thr_low  = threshold_o >> 1
//
// Values are chosen for the mag8 = (|Gx| + |Gy|) >> 3 domain used by the
// feature_canny branch.  The defaults provide a practical initial tuning
// range; they can be overridden after checking the live camera image.
module canny_threshold_key_ctrl #(
    parameter integer CLK_FREQ_HZ = 74_250_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer CNT_MAX =
        (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS - 1,

    parameter [7:0] THR_DEFAULT = 8'd12,
    parameter [7:0] THR_MIN     = 8'd4,
    parameter [7:0] THR_MAX     = 8'd32,
    parameter [7:0] THR_STEP    = 8'd4
)(
    input  wire       clk,
    input  wire       rst_n,

    // Raw VF-Ti60F225 pushbuttons. Released = 2'b11, pressed = 1'b0.
    input  wire [1:0] key_data,

    // Optional software/UART write channel.  uart_thr_valid_i must pulse for
    // one clk when uart_thr_i is valid.  UART has priority over the buttons.
    input  wire       uart_thr_valid_i,
    input  wire [7:0] uart_thr_i,

    // Smoothing-mode write channel from UART 'S'.  Same one-clock pulse
    // contract; UART has priority over K1.
    input  wire       uart_smooth_valid_i,
    input  wire [1:0] uart_smooth_i,

    output reg  [7:0] threshold_o,
    output wire [7:0] threshold_low_o,
    output reg  [1:0] smooth_mode_o,

    // Optional observation port: one-clock pulses, bit order matches key_data.
    output wire [1:0] key_press_o
);

    localparam integer CNT_W =
        (CNT_MAX < 1) ? 1 : $clog2(CNT_MAX + 1);
    localparam [CNT_W-1:0] CNT_LIMIT = CNT_MAX;

    //-----------------------------------------------------------------
    // 1) Synchronize both asynchronous button inputs into clk_pixel.
    //-----------------------------------------------------------------
    (* async_reg = "true" *) reg [1:0] key_meta;
    (* async_reg = "true" *) reg [1:0] key_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 2'b11;
            key_sync <= 2'b11;
        end else begin
            key_meta <= key_data;
            key_sync <= key_meta;
        end
    end

    //-----------------------------------------------------------------
    // 2) Debounce K1 and K2 independently.
    //
    // Each new level must remain unchanged for DEBOUNCE_MS before it is
    // accepted.  A one-clock pulse is generated only for a stable 1 -> 0
    // transition, so holding or releasing a key cannot repeatedly adjust
    // the threshold.
    //-----------------------------------------------------------------
    reg [1:0] key_stable;
    reg [1:0] key_press;
    reg [CNT_W-1:0] key0_cnt;
    reg [CNT_W-1:0] key1_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_stable <= 2'b11;
            key_press  <= 2'b00;
            key0_cnt   <= {CNT_W{1'b0}};
            key1_cnt   <= {CNT_W{1'b0}};
        end else begin
            // Default low makes each accepted press exactly one clock long.
            key_press <= 2'b00;

            // K1 / key_data[0]: threshold increase.
            if (key_sync[0] == key_stable[0]) begin
                key0_cnt <= {CNT_W{1'b0}};
            end else if (key0_cnt == CNT_LIMIT) begin
                key0_cnt       <= {CNT_W{1'b0}};
                key_stable[0]  <= key_sync[0];
                if (!key_sync[0])
                    key_press[0] <= 1'b1;
            end else begin
                key0_cnt <= key0_cnt + 1'b1;
            end

            // K2 / key_data[1]: threshold decrease.
            if (key_sync[1] == key_stable[1]) begin
                key1_cnt <= {CNT_W{1'b0}};
            end else if (key1_cnt == CNT_LIMIT) begin
                key1_cnt       <= {CNT_W{1'b0}};
                key_stable[1]  <= key_sync[1];
                if (!key_sync[1])
                    key_press[1] <= 1'b1;
            end else begin
                key1_cnt <= key1_cnt + 1'b1;
            end
        end
    end

    assign key_press_o = key_press;
    assign threshold_low_o = {1'b0, threshold_o[7:1]};

    //-----------------------------------------------------------------
    // 3) Saturating threshold adjustment. Holding a key produces one event.
    // UART values are clamped to the same range as physical buttons.
    //-----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            threshold_o <= (THR_DEFAULT < THR_MIN) ? THR_MIN :
                           (THR_DEFAULT > THR_MAX) ? THR_MAX : THR_DEFAULT;
        else if (uart_thr_valid_i)
            threshold_o <= (uart_thr_i < THR_MIN) ? THR_MIN :
                           (uart_thr_i > THR_MAX) ? THR_MAX : uart_thr_i;
        else if (key_stable != 2'b00) begin
            case (key_press)
                2'b01: threshold_o <= ({1'b0,threshold_o}+{1'b0,THR_STEP} >= {1'b0,THR_MAX})
                                      ? THR_MAX : threshold_o+THR_STEP;
                2'b10: threshold_o <= ({1'b0,threshold_o} <= {1'b0,THR_MIN}+{1'b0,THR_STEP})
                                      ? THR_MIN : threshold_o-THR_STEP;
                default: ;
            endcase
        end
    end

    // 4) Standard Gaussian on reset. Keys never change smoothing.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) smooth_mode_o <= 2'd0;
        else if (uart_smooth_valid_i)
            smooth_mode_o <= (uart_smooth_i > 2'd2) ? 2'd2 : uart_smooth_i;
    end

endmodule

