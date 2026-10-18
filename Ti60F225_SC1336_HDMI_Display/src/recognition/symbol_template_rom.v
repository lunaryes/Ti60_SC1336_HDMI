`timescale 1ns / 1ps

// 8x8 binary templates for controlled-scene recognition.
// Bit 63 is row 0 / column 0, bit 0 is row 7 / column 7.
// Addresses 0..49 map to class IDs 1..50 in symbol_template_matcher.
module symbol_template_rom (
    input  wire [5:0]  addr_i,
    output reg  [63:0] bitmap_o,
    output reg  [7:0]  ascii_o
);
    always @(*) begin
        bitmap_o = 64'h0000000000000000;
        ascii_o  = 8'h00;
        case (addr_i)
            // Digits 0..9, class IDs 1..10.
            6'd0:  begin bitmap_o = 64'h3C666E7666663C00; ascii_o = "0"; end
            6'd1:  begin bitmap_o = 64'h1838181818187E00; ascii_o = "1"; end
            6'd2:  begin bitmap_o = 64'h3C66060C30607E00; ascii_o = "2"; end
            6'd3:  begin bitmap_o = 64'h3C66061C06663C00; ascii_o = "3"; end
            6'd4:  begin bitmap_o = 64'h0C1C3C6C7E0C0C00; ascii_o = "4"; end
            6'd5:  begin bitmap_o = 64'h7E607C0606663C00; ascii_o = "5"; end
            6'd6:  begin bitmap_o = 64'h1C30607C66663C00; ascii_o = "6"; end
            6'd7:  begin bitmap_o = 64'h7E660C1818181800; ascii_o = "7"; end
            6'd8:  begin bitmap_o = 64'h3C66663C66663C00; ascii_o = "8"; end
            6'd9:  begin bitmap_o = 64'h3C66663E060C3800; ascii_o = "9"; end

            // Upper-case A..Z, class IDs 11..36.
            6'd10: begin bitmap_o = 64'h183C66667E666600; ascii_o = "A"; end
            6'd11: begin bitmap_o = 64'h7C66667C66667C00; ascii_o = "B"; end
            6'd12: begin bitmap_o = 64'h3C66606060663C00; ascii_o = "C"; end
            6'd13: begin bitmap_o = 64'h786C6666666C7800; ascii_o = "D"; end
            6'd14: begin bitmap_o = 64'h7E60607C60607E00; ascii_o = "E"; end
            6'd15: begin bitmap_o = 64'h7E60607C60606000; ascii_o = "F"; end
            6'd16: begin bitmap_o = 64'h3C66606E66663C00; ascii_o = "G"; end
            6'd17: begin bitmap_o = 64'h6666667E66666600; ascii_o = "H"; end
            6'd18: begin bitmap_o = 64'h3C18181818183C00; ascii_o = "I"; end
            6'd19: begin bitmap_o = 64'h1E0C0C0C0C6C3800; ascii_o = "J"; end
            6'd20: begin bitmap_o = 64'h666C7870786C6600; ascii_o = "K"; end
            6'd21: begin bitmap_o = 64'h6060606060607E00; ascii_o = "L"; end
            6'd22: begin bitmap_o = 64'h63777F6B63636300; ascii_o = "M"; end
            6'd23: begin bitmap_o = 64'h66767E7E6E666600; ascii_o = "N"; end
            6'd24: begin bitmap_o = 64'h3C66666666663C00; ascii_o = "O"; end
            6'd25: begin bitmap_o = 64'h7C66667C60606000; ascii_o = "P"; end
            6'd26: begin bitmap_o = 64'h3C6666666E3C0E00; ascii_o = "Q"; end
            6'd27: begin bitmap_o = 64'h7C66667C786C6600; ascii_o = "R"; end
            6'd28: begin bitmap_o = 64'h3C66603C06663C00; ascii_o = "S"; end
            6'd29: begin bitmap_o = 64'h7E5A181818183C00; ascii_o = "T"; end
            6'd30: begin bitmap_o = 64'h6666666666663C00; ascii_o = "U"; end
            6'd31: begin bitmap_o = 64'h66666666663C1800; ascii_o = "V"; end
            6'd32: begin bitmap_o = 64'h6363636B7F776300; ascii_o = "W"; end
            6'd33: begin bitmap_o = 64'h66663C183C666600; ascii_o = "X"; end
            6'd34: begin bitmap_o = 64'h6666663C18181800; ascii_o = "Y"; end
            6'd35: begin bitmap_o = 64'h7E060C1830607E00; ascii_o = "Z"; end

            // Geometric symbols, class IDs 37..50. ascii_o is a short tag
            // used by the overlay; class_id_o remains the unambiguous result.
            6'd36: begin bitmap_o = 64'h3C4281818181423C; ascii_o = "O"; end // circle
            6'd37: begin bitmap_o = 64'h003C428181423C00; ascii_o = "E"; end // ellipse
            6'd38: begin bitmap_o = 64'h182442428181FF00; ascii_o = "T"; end // triangle
            6'd39: begin bitmap_o = 64'hFF818181818181FF; ascii_o = "S"; end // square
            6'd40: begin bitmap_o = 64'h00FF81818181FF00; ascii_o = "R"; end // rectangle
            6'd41: begin bitmap_o = 64'h1824428142241800; ascii_o = "D"; end // diamond
            6'd42: begin bitmap_o = 64'h000000FF00000000; ascii_o = "-"; end // horizontal line
            6'd43: begin bitmap_o = 64'h1818181818181818; ascii_o = "|"; end // vertical line
            6'd44: begin bitmap_o = 64'h181818FF18181800; ascii_o = "+"; end // plus
            6'd45: begin bitmap_o = 64'h8142241818244281; ascii_o = "X"; end // cross
            6'd46: begin bitmap_o = 64'h183C7E1818181800; ascii_o = "U"; end // arrow up
            6'd47: begin bitmap_o = 64'h103070FF70301000; ascii_o = ">"; end // arrow right
            6'd48: begin bitmap_o = 64'h181818187E3C1800; ascii_o = "D"; end // arrow down
            6'd49: begin bitmap_o = 64'h080C0EFF0E0C0800; ascii_o = "<"; end // arrow left
            default: begin bitmap_o = 64'h0; ascii_o = 8'h00; end
        endcase
    end
endmodule
