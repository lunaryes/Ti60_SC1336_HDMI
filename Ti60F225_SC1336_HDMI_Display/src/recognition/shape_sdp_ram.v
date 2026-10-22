`timescale 1ns/1ps
// Inferred EFX_RAM10-compatible simple dual-port RAM. Contents are initialized
// by the controller's capture/clear states, never by a massive reset loop.
module shape_sdp_ram #(
    parameter integer WIDTH=1,DEPTH=14400,AW=14
)(input wire clk,we,input wire [AW-1:0] wa,ra,
  input wire [WIDTH-1:0] wd,output reg [WIDTH-1:0] rd);
    reg [WIDTH-1:0] mem[0:DEPTH-1];
    always @(posedge clk) begin
        if(we) mem[wa]<=wd;
        rd<=mem[ra];
    end
endmodule
