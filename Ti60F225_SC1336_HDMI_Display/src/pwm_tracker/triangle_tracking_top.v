module triangle_tracking_top #(
 parameter integer CLK_HZ=96000000,SPEED_MODE=0,DIR_INVERT=0
)(
 input wire clk_pixel,rst_pixel_n,vsync_i,
 input wire [287:0] objects_i,input wire [7:0] valid_i,
 input wire clk_sys,rst_sys_n,enable_i,
 output wire pwm_o,output wire [3:0] status_o,
 output wire signed [9:0] error_o,output wire [20:0] pulse_o,
 output wire [5:0] debug_o
);
 wire signed [9:0] ex,ey;
 wire found,frame;
 triangle_target_select select_target(
 .clk(clk_pixel),.rst_n(rst_pixel_n),.vsync_i(vsync_i),.clear_i(!en_pix[1] || epoch_pix[1]!=epoch_seen),
 .objects_i(objects_i),.valid_i(valid_i),.ex_o(ex),.ey_o(ey),
 .found_o(found),.frame_o(frame),.size_w_o(),.size_h_o());
 // The system domain owns a generation bit. A resume invalidates every
 // previous CDC payload before requesting samples from a new pixel frame.
 reg [1:0] enable_sync;
 reg enable_prev, epoch;
 always @(posedge clk_sys or negedge rst_sys_n)
 if(!rst_sys_n) begin enable_sync<=0;enable_prev<=0;epoch<=0;end
 else begin
  enable_sync<={enable_sync[0],enable_i};enable_prev<=enable_sync[1];
  if(enable_sync[1] && !enable_prev) epoch<=!epoch;
 end
 reg [1:0] epoch_pix,en_pix;
 reg epoch_seen,armed,vs_prev;
 always @(posedge clk_pixel or negedge rst_pixel_n)
 if(!rst_pixel_n) begin
  epoch_pix<=0;en_pix<=0;epoch_seen<=0;armed<=0;vs_prev<=0;
 end else begin
  epoch_pix<={epoch_pix[0],epoch};en_pix<={en_pix[0],enable_i};
  vs_prev<=vsync_i;
  if(!en_pix[1] || epoch_pix[1]!=epoch_seen) begin
   armed<=0;epoch_seen<=epoch_pix[1];
  end else if(vsync_i && !vs_prev) armed<=1;
 end
 wire [11:0] sample;
 wire sample_valid;
 axis_cdc #(.W(12)) transfer(
 .clk_s(clk_pixel),.rst_s_n(rst_pixel_n),.data_s({epoch_seen,found,ex}),.valid_s(frame && armed && en_pix[1] && epoch_pix[1]==epoch_seen),
 .clk_d(clk_sys),.rst_d_n(rst_sys_n),.data_d(sample),.valid_d(sample_valid));
 assign error_o=sample[9:0];
 wire tick;
 triangle_follow_ctrl #(.CLK_HZ(CLK_HZ),.SPEED_MODE(SPEED_MODE),
 .DIR_INVERT(DIR_INVERT)) controller(
 .clk(clk_sys),.rst_n(rst_sys_n),.enable_i(enable_i),.tick_i(tick),
 .sample_i(sample_valid && sample[11]==epoch && enable_sync[1] && enable_prev),.found_i(sample[10]),.ex_i(sample[9:0]),
 .pulse_o(pulse_o),.status_o(status_o));
 // Diagnostic signals; no influence on control or PWM command.
 reg [24:0] sample_age,change_age;
 reg [20:0] previous_pulse;
 localparam integer SHOW_CYCLES=CLK_HZ/4;
 always @(posedge clk_sys or negedge rst_sys_n)
 if(!rst_sys_n) begin
  sample_age<=SHOW_CYCLES;change_age<=SHOW_CYCLES;
  previous_pulse<=SPEED_MODE?144000:140160;
 end else begin
  if(sample_valid) sample_age<=0;
  else if(sample_age<SHOW_CYCLES) sample_age<=sample_age+1'b1;
  previous_pulse<=pulse_o;
  if(pulse_o!=previous_pulse) change_age<=0;
  else if(change_age<SHOW_CYCLES) change_age<=change_age+1'b1;
 end
 // [0] raw selected target; [1] tracking usable; [2] paused;
 // [3] recent vision sample; [4] recent pulse change; [5] limit.
 assign debug_o={status_o[3],change_age<SHOW_CYCLES,
                 sample_age<SHOW_CYCLES,!enable_sync[1],status_o[0],sample[10]};
 // Width changes only at period boundaries.
 localparam integer PERIOD=CLK_HZ/50;
 reg [20:0] count,width;
 assign tick=count==PERIOD-1;
 assign pwm_o=rst_sys_n && count<width;
 always @(posedge clk_sys or negedge rst_sys_n)
 if(!rst_sys_n) begin count<=0;width<=SPEED_MODE?144000:140160;end
 else if(tick) begin count<=0;width<=pulse_o;end
 else count<=count+1'b1;
endmodule
