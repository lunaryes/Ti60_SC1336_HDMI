`timescale 1ns/1ps
// Two masters, independent read/write round-robin arbitration. Grants are
// retained through B / RLAST; address and payload cannot switch under stall.
module temporal_axi_mux(
 input wire clk,rst,
 input wire [31:0] a_awaddr,b_awaddr,input wire [7:0] a_awlen,b_awlen,
 input wire a_awvalid,b_awvalid,output wire a_awready,b_awready,
 input wire [127:0] a_wdata,b_wdata,input wire a_wlast,b_wlast,a_wvalid,b_wvalid,
 output wire a_wready,b_wready,a_bvalid,b_bvalid,
 input wire [31:0] a_araddr,b_araddr,input wire [7:0] a_arlen,b_arlen,
 input wire a_arvalid,b_arvalid,output wire a_arready,b_arready,
 output wire a_rvalid,b_rvalid,
 output wire [31:0] awaddr,araddr,output wire [7:0] awlen,arlen,
 output wire awvalid,arvalid,input wire awready,arready,
 output wire [127:0] wdata,output wire wlast,wvalid,input wire wready,bvalid,rlast,rvalid
);
 reg [1:0] ws,rs;reg wo,ro,wprevious,rprevious;
 always @(posedge clk)begin
  if(rst)begin ws<=0;rs<=0;wo<=0;ro<=0;wprevious<=1;rprevious<=1;end
  else begin
   if(ws==0 && (a_awvalid||b_awvalid))begin
    wo<=b_awvalid && (!a_awvalid || !wprevious);ws<=1;
   end
   if(ws==1 && awready)ws<=2;
   if(ws==2 && bvalid)begin ws<=0;wprevious<=wo;end
   if(rs==0 && (a_arvalid||b_arvalid))begin
    ro<=b_arvalid && (!a_arvalid || !rprevious);rs<=1;
   end
   if(rs==1 && arready)rs<=2;
   if(rs==2 && rvalid && rlast)begin rs<=0;rprevious<=ro;end
  end
 end
 assign awaddr=wo?b_awaddr:a_awaddr;assign awlen=wo?b_awlen:a_awlen;
 assign awvalid=ws==1 && (wo?b_awvalid:a_awvalid);
 assign a_awready=ws==1&&!wo&&awready;assign b_awready=ws==1&&wo&&awready;
 assign wdata=wo?b_wdata:a_wdata;assign wlast=wo?b_wlast:a_wlast;
 assign wvalid=ws==2 && (wo?b_wvalid:a_wvalid);
 assign a_wready=ws==2&&!wo&&wready;assign b_wready=ws==2&&wo&&wready;
 assign a_bvalid=ws==2&&!wo&&bvalid;assign b_bvalid=ws==2&&wo&&bvalid;
 assign araddr=ro?b_araddr:a_araddr;assign arlen=ro?b_arlen:a_arlen;
 assign arvalid=rs==1 && (ro?b_arvalid:a_arvalid);
 assign a_arready=rs==1&&!ro&&arready;assign b_arready=rs==1&&ro&&arready;
 assign a_rvalid=rs==2&&!ro&&rvalid;assign b_rvalid=rs==2&&ro&&rvalid;
endmodule
