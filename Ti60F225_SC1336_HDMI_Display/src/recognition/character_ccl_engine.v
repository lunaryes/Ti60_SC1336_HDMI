`timescale 1ns/1ps
// 无卡片字符分割：八连通两遍 CCL，等价标签表 + 片内标签图。
// 不使用外框推断字符位置。直接裁紧每个前景连通域并统计 16x16 占比。
// 原有几何 CCL/Moore 和高分辨率单字符 OCR 不作修改。
module character_ccl_engine #(
 parameter integer W=320,H=180,AW=$clog2(W*H),MAX_CANDIDATES=32,
 parameter integer MIN_HEIGHT=14,MAX_HEIGHT=128
)(
 input wire clk,rst_n,start_i,output reg busy_o,done_o,
 output reg [AW-1:0] raw_addr_o,input wire [1:0] raw_data_i,
 // 每项 {ASCII[7:0],xmin[8:0],xmax[8:0],ymin[7:0],ymax[7:0]}。
 output reg [167:0] objects_o,output reg [3:0] valid_o,
 output reg overflow_o,output reg [23:0] cycles_o
);
 localparam integer N=W*H;
 localparam IDLE=0,RAW_ISS=1,RAW_WAIT=2,RAW_EVAL=3,NB_ISS=4,NB_WAIT=5,NB_EVAL=6,
   ROOT_ISS=7,ROOT_WAIT=8,ROOT_EVAL=9,ASSIGN=10,ADV=11,
   CLEAR_STATS=12,P2_ISS=13,P2_WAIT=14,P2_EVAL=15,STATS_ISS=16,STATS_WAIT=17,
   STATS_WRITE=18,P2_ADV=19,CAND_ISS=20,CAND_WAIT=21,CAND_EVAL=22,
   BIN_SETUP=23,BIN_ISS=24,BIN_WAIT=25,BIN_EVAL=26,BIN_STORE=27,
   MATCH_START=28,MATCH_WAIT=29,CAND_ADV=30,NEXT_POLARITY=31,PUBLISH=32;
 reg [5:0] state;reg polarity,second_pass;
 reg label_we,parent_we,stats_we;
 reg [AW-1:0] label_wa,label_ra;reg [7:0] label_wd;wire [7:0] label_rd;
 reg [7:0] parent_wa,parent_ra,parent_wd;wire [7:0] parent_rd;
 reg [7:0] stats_wa,stats_ra;reg [49:0] stats_wd;wire [49:0] stats_rd;
 shape_sdp_ram #(.WIDTH(8),.DEPTH(N),.AW(AW)) u_labels(clk,label_we,label_wa,label_ra,label_wd,label_rd);
 shape_sdp_ram #(.WIDTH(8),.DEPTH(256),.AW(8)) u_parent(clk,parent_we,parent_wa,parent_ra,parent_wd,parent_rd);
 shape_sdp_ram #(.WIDTH(50),.DEPTH(256),.AW(8)) u_stats(clk,stats_we,stats_wa,stats_ra,stats_wd,stats_rd);
 reg [AW-1:0] index;reg [8:0] x;reg [7:0] y,left_label,root_min,root;
 reg [8:0] next_label;reg [1:0] nb;
 reg [7:0] clear_label,candidate_label;reg [5:0] candidates;
 reg [1:0] object_count;reg [167:0] pending_objects;reg [3:0] pending_valid;
 reg [23:0] work_cycles;
 reg [8:0] xmin,xmax;reg [7:0] ymin,ymax;
 wire [9:0] box_w={1'b0,stats_rd[17:9]}-{1'b0,stats_rd[8:0]}+1'b1;
 wire [8:0] box_h={1'b0,stats_rd[33:26]}-{1'b0,stats_rd[25:18]}+1'b1;
 wire [18:0] box_area=box_w*box_h;
 wire [15:0] area=stats_rd[49:34];
 wire candidate_ok=area>=16 && box_h>=MIN_HEIGHT && box_h<=MAX_HEIGHT && box_w>=2&&box_w<=128&&
   stats_rd[8:0]>1&&stats_rd[17:9]<W-2&&stats_rd[25:18]>1&&stats_rd[33:26]<H-2&&
   box_w*8>=box_h && box_w*2<=box_h*3 && area*12>=box_area &&
   (area*4<box_area*3 || box_w*4<box_h);
 reg [3:0] bin_x,bin_y;reg [8:0] bx0,bx1,px;reg [7:0] by0,by1,py;
 reg [9:0] width;reg [8:0] height;
 wire [13:0] xb0=bin_x*width+15;
 wire [13:0] xb1=({1'b0,bin_x}+1'b1)*width+15;
 wire [12:0] yb0=bin_y*height+15;
 wire [12:0] yb1=({1'b0,bin_y}+1'b1)*height+15;
 // 目标小于16格时用最近邻扩展，不能留下未采样的空列（尤其是1/I）。
 wire [8:0] sample_x0=width<16?(bin_x*width)>>4:xb0>>4;
 wire [8:0] sample_x1=width<16?(bin_x*width)>>4:(xb1>>4)-1'b1;
 wire [7:0] sample_y0=height<16?(bin_y*height)>>4:yb0>>4;
 wire [7:0] sample_y1=height<16?(bin_y*height)>>4:(yb1>>4)-1'b1;
 reg [7:0] hits,total;reg [255:0] glyph;reg match_start;
 wire match_done,recognized;wire [5:0] matched_class;wire [7:0] matched_ascii;
 wire [8:0] best,second;wire [255:0] matched_template;
 character_matcher16 u_matcher(.clk(clk),.rst_n(rst_n),.start_i(match_start),.candidate_valid_i(1'b1),
   .glyph_i(glyph),.width_i({1'b0,width}),.height_i({1'b0,height}),.result_valid_o(match_done),
   .recognized_o(recognized),.class_id_o(matched_class),.ascii_o(matched_ascii),
   .best_distance_o(best),.second_distance_o(second),.template_o(matched_template));
 reg [49:0] updated_stats;
 always @(*) begin
   updated_stats=stats_rd;
   if(area==0) updated_stats={16'd1,y,y,x,x};
   else begin
     updated_stats[49:34]=area==65535?area:area+1'b1;
     if(x<stats_rd[8:0]) updated_stats[8:0]=x;
     if(x>stats_rd[17:9]) updated_stats[17:9]=x;
     if(y<stats_rd[25:18]) updated_stats[25:18]=y;
     if(y>stats_rd[33:26]) updated_stats[33:26]=y;
   end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;busy_o<=0;done_o<=0;raw_addr_o<=0;objects_o<=0;valid_o<=0;overflow_o<=0;cycles_o<=0;
   label_we<=0;parent_we<=0;stats_we<=0;label_wa<=0;label_ra<=0;label_wd<=0;
   parent_wa<=0;parent_ra<=0;parent_wd<=0;stats_wa<=0;stats_ra<=0;stats_wd<=0;
   index<=0;x<=0;y<=0;left_label<=0;root_min<=0;root<=0;next_label<=1;nb<=0;
   polarity<=0;second_pass<=0;clear_label<=0;candidate_label<=0;candidates<=0;
   object_count<=0;pending_objects<=0;pending_valid<=0;work_cycles<=0;
   xmin<=0;xmax<=0;ymin<=0;ymax<=0;width<=0;height<=0;
   bin_x<=0;bin_y<=0;bx0<=0;bx1<=0;by0<=0;by1<=0;px<=0;py<=0;hits<=0;total<=0;glyph<=0;match_start<=0;
  end else begin
   done_o<=0;label_we<=0;parent_we<=0;stats_we<=0;match_start<=0;
   if(busy_o) work_cycles<=work_cycles+1'b1;
   if(busy_o&&work_cycles==24'd6000000) begin
     objects_o<=0;valid_o<=0;overflow_o<=1;cycles_o<=work_cycles;done_o<=1;busy_o<=0;state<=IDLE;
   end else case(state)
    IDLE:if(start_i) begin
      busy_o<=1;work_cycles<=0;overflow_o<=0;polarity<=0;second_pass<=0;
      pending_objects<=0;pending_valid<=0;object_count<=0;candidates<=0;
      next_label<=1;index<=0;x<=0;y<=0;left_label<=0;state<=RAW_ISS;
    end
    RAW_ISS:begin raw_addr_o<=index;state<=RAW_WAIT;end
    RAW_WAIT:state<=RAW_EVAL;
    RAW_EVAL:begin
      if(!raw_data_i[polarity]) begin label_we<=1;label_wa<=index;label_wd<=0;left_label<=0;state<=ADV;end
      else begin nb<=0;root_min<=0;state<=NB_ISS;end
    end
    NB_ISS:begin
      if(nb==0) begin
        if(left_label!=0) begin root<=left_label;state<=ROOT_ISS;end
        else begin nb<=1;state<=NB_ISS;end
      end else if(y==0||(nb==1&&x==0)||(nb==3&&x==W-1)) begin
        if(nb==3) state<=ASSIGN;else nb<=nb+1'b1;
      end else begin
        case(nb) 1:label_ra<=index-W-1;2:label_ra<=index-W;default:label_ra<=index-W+1;endcase
        state<=NB_WAIT;
      end
    end
    NB_WAIT:state<=NB_EVAL;
    NB_EVAL:begin
      if(label_rd!=0) begin root<=label_rd;state<=ROOT_ISS;end
      else if(nb==3) state<=ASSIGN;
      else begin nb<=nb+1'b1;state<=NB_ISS;end
    end
    ROOT_ISS:begin parent_ra<=root;state<=ROOT_WAIT;end
    ROOT_WAIT:state<=ROOT_EVAL;
    ROOT_EVAL:begin
      if(parent_rd!=root) begin root<=parent_rd;state<=ROOT_ISS;end
      else if(second_pass) state<=STATS_ISS;
      else begin
        if(root_min==0) root_min<=root;
        else if(root!=root_min) begin
          parent_we<=1;parent_wa<=root>root_min?root:root_min;parent_wd<=root<root_min?root:root_min;
          if(root<root_min) root_min<=root;
        end
        if(nb==3) state<=ASSIGN;else begin nb<=nb+1'b1;state<=NB_ISS;end
      end
    end
    ASSIGN:begin
      if(root_min==0&&next_label==256) begin overflow_o<=1;pending_valid<=0;state<=PUBLISH;end
      else begin
        label_we<=1;label_wa<=index;label_wd<=root_min==0?next_label[7:0]:root_min;
        left_label<=root_min==0?next_label[7:0]:root_min;
        if(root_min==0) begin
          parent_we<=1;parent_wa<=next_label[7:0];parent_wd<=next_label[7:0];next_label<=next_label+1'b1;
        end
        state<=ADV;
      end
    end
    ADV:begin
      if(index==N-1) begin clear_label<=0;state<=CLEAR_STATS;end
      else begin
        index<=index+1'b1;
        if(x==W-1) begin x<=0;y<=y+1'b1;left_label<=0;end else x<=x+1'b1;
        state<=RAW_ISS;
      end
    end
    CLEAR_STATS:begin
      stats_we<=1;stats_wa<=clear_label;stats_wd<=0;
      if(clear_label==255) begin index<=0;x<=0;y<=0;second_pass<=1;state<=P2_ISS;end
      else clear_label<=clear_label+1'b1;
    end
    P2_ISS:begin label_ra<=index;state<=P2_WAIT;end
    P2_WAIT:state<=P2_EVAL;
    P2_EVAL:begin
      if(label_rd==0) state<=P2_ADV;
      else begin root<=label_rd;state<=ROOT_ISS;end
    end
    STATS_ISS:begin stats_ra<=root;state<=STATS_WAIT;end
    STATS_WAIT:state<=STATS_WRITE;
    STATS_WRITE:begin
      stats_we<=1;stats_wa<=root;stats_wd<=updated_stats;
      label_we<=1;label_wa<=index;label_wd<=root;state<=P2_ADV;
    end
    P2_ADV:begin
      if(index==N-1) begin candidate_label<=1;state<=CAND_ISS;end
      else begin
        index<=index+1'b1;if(x==W-1) begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
        state<=P2_ISS;
      end
    end
    CAND_ISS:begin stats_ra<=candidate_label;state<=CAND_WAIT;end
    CAND_WAIT:state<=CAND_EVAL;
    CAND_EVAL:begin
      if(candidate_ok&&candidates<MAX_CANDIDATES&&pending_valid!=4'b1111) begin
        xmin<=stats_rd[8:0];xmax<=stats_rd[17:9];ymin<=stats_rd[25:18];ymax<=stats_rd[33:26];
        width<=box_w;height<=box_h;bin_x<=0;bin_y<=0;glyph<=0;candidates<=candidates+1'b1;state<=BIN_SETUP;
      end else state<=CAND_ADV;
    end
    BIN_SETUP:begin
      bx0<=sample_x0;bx1<=sample_x1;by0<=sample_y0;by1<=sample_y1;
      px<=sample_x0;py<=sample_y0;hits<=0;total<=0;state<=BIN_ISS;
    end
    BIN_ISS:begin label_ra<=(ymin+py)*W+xmin+px;state<=BIN_WAIT;end
    BIN_WAIT:state<=BIN_EVAL;
    BIN_EVAL:begin
      total<=total+1'b1;if(label_rd==candidate_label) hits<=hits+1'b1;
      if(px==bx1) begin
        px<=bx0;if(py==by1) state<=BIN_STORE;else begin py<=py+1'b1;state<=BIN_ISS;end
      end else begin px<=px+1'b1;state<=BIN_ISS;end
    end
    BIN_STORE:begin
      glyph[8'd255-{bin_y,bin_x}]<=total!=0&&({2'b0,hits}<<2)>=total;
      if(bin_x==15) begin
        bin_x<=0;if(bin_y==15) state<=MATCH_START;else begin bin_y<=bin_y+1'b1;state<=BIN_SETUP;end
      end else begin bin_x<=bin_x+1'b1;state<=BIN_SETUP;end
    end
    MATCH_START:begin match_start<=1;state<=MATCH_WAIT;end
    MATCH_WAIT:if(match_done) begin
      if(recognized) begin
        pending_objects[object_count*42+:42]<={matched_ascii,xmin,xmax,ymin,ymax};
        pending_valid[object_count]<=1;object_count<=object_count+1'b1;
      end
      state<=CAND_ADV;
    end
    CAND_ADV:begin
      if(candidate_label==255||candidates>=MAX_CANDIDATES||pending_valid==4'b1111) state<=NEXT_POLARITY;
      else begin candidate_label<=candidate_label+1'b1;state<=CAND_ISS;end
    end
    NEXT_POLARITY:begin
      if(polarity||pending_valid==4'b1111) state<=PUBLISH;
      else begin polarity<=1;second_pass<=0;candidates<=0;next_label<=1;index<=0;x<=0;y<=0;left_label<=0;state<=RAW_ISS;end
    end
    PUBLISH:begin objects_o<=pending_objects;valid_o<=pending_valid;done_o<=1;busy_o<=0;cycles_o<=work_cycles;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
