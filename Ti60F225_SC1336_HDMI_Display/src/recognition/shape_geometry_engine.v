`timescale 1ns/1ps
// ---------------------------------------------------------------------------
// Ti60 shape recognition engine (rewrite, shape-only profile).
//
// Why it was rewritten
//   The previous classifier measured a Moore contour with Ramer-Douglas-Peucker
//   vertices and turn signs. On real 8x pooled Canny edges the contour is a
//   staircase, so a square grew extra vertices, a circle looked like a polygon
//   and background clutter could still satisfy the rules. Vertex counting is
//   not noise tolerant at 160x90, which is why labels were mostly wrong.
//
// What is used now - only area invariants, insensitive to 1..2 pixel steps
//   1. 8-connected flood fill gives the component bbox and pixel count.
//   2. Per row span fill: a row of a closed outline crosses the component at
//      run 1 (left edge) and run 2 (right edge). Everything between them is
//      the shape slice, so the slice is taken from run 1 to run 2. Rows with a
//      single run (tangent rows of arcs, straight lines) are dropped, which
//      removes the classic "open contour fills the whole bbox" failure.
//   3. Features of the filled region:
//        extent        area / filled-bbox area
//        quadrants     4 * area_quadrant / filled-bbox area (TL TR BL BR)
//        row widths    at 1/8, 2/8, 4/8, 6/8, 7/8 of the filled height
//        full rows     rows wider than 95% of the filled width
//   4. Decision rules, first match wins:
//        triangle  extent ~0.5, one corner pair occupied, opposite pair empty
//        square    extent ~1.0, square aspect, all corners occupied
//        rectangle extent ~1.0, non-square aspect, all corners occupied
//        pentagon  extent ~0.7, tapered top rows and wide bottom rows
//        circle    extent ~pi/4, square aspect, all corners ~0.78
//        ellipse   extent ~pi/4, non-square aspect
//        plus      extent ~0.55, corners ~0.55, a third of the rows full width
//        quad      extent ~0.5, all corners ~0.5, almost no full row (diamond)
//   Dense solid regions also accept one-run slices. Quadrants accumulate
//   four times their area so they share the whole-bbox normalization.
//   Components with two or more endpoints are rejected; one endpoint is
//   permitted for acute rasterized triangle tips. This is an open-U filter,
//   not a general proof that a contour is topologically closed.
//   Components that are too small, touch the frame border, are slivers, are
//   clipped by the pixel budget, or match no rule are rejected. Queue or
//   component-budget exhaustion aborts the frame with an empty result.
//   Only shape classes exist here: digit and letter recognition was removed
//   from this profile.
// ---------------------------------------------------------------------------
module shape_geometry_engine #(
    parameter integer W=160,H=90,AW=$clog2(W*H),MAX_OBJECTS=8,
    parameter integer MIN_PIXELS=12,MIN_BOX=6,MAX_PIXELS=3000,
    parameter integer QDEPTH=512
)(
    input wire clk,rst_n,start_i,
    output reg busy_o,result_valid_o,
    output reg [AW-1:0] raw_addr_o,input wire raw_data_i,
    // Each descriptor: {class[3:0],xmin[7:0],xmax[7:0],ymin[7:0],ymax[7:0]}.
    output reg [MAX_OBJECTS*36-1:0] objects_o,
    output reg [MAX_OBJECTS-1:0] valid_o,
    output reg [15:0] rejected_o,
    output reg [23:0] cycles_o
);
    localparam integer N=W*H;
    localparam integer MAXBW=(W*3)>>2;
    localparam integer MAXBH=(H*3)>>2;
    localparam integer NR=H;
    localparam integer QAWL=$clog2(QDEPTH);

    localparam [5:0] IDLE=0,CLR=1,SCAN_ISS=2,SCAN_WAIT=3,SCAN_EVAL=4,SCAN_ADV=5,
        POP_ISS=6,POP_WAIT=7,POP_EVAL=8,NB_ISS=9,NB_WAIT=10,NB_EVAL=11,
        DRAIN_ISS=12,DRAIN_WAIT=13,DRAIN_EVAL=14,FILTER=15,
        F_ISS=16,F_WAIT=17,F_EVAL=18,
        PP1_INIT=19,PP1_STEP=20,PP2_INIT=21,PP2_STEP=22,CLASS=23,STORE=24,PUBLISH=25;
    reg [5:0] state;

    reg visited_we,queue_we;
    reg [15:0] visited_wd,component_id;
    reg [3:0] degree;
    reg [15:0] endpoint_count;
    reg [AW-1:0] visited_wa,visited_ra;
    reg [QAWL-1:0] queue_wa,queue_ra;
    reg [15:0] queue_wd;
    wire [15:0] visited_rd;
    wire [15:0] queue_rd;
    shape_sdp_ram #(.WIDTH(16),.DEPTH(N),.AW(AW)) u_visited(
        .clk(clk),.we(visited_we),.wa(visited_wa),.wd(visited_wd),.ra(visited_ra),.rd(visited_rd));
    shape_sdp_ram #(.WIDTH(16),.DEPTH(QDEPTH),.AW(QAWL)) u_queue(
        .clk(clk),.we(queue_we),.wa(queue_wa),.wd(queue_wd),.ra(queue_ra),.rd(queue_rd));

    reg [AW-1:0] index,scan_index;
    reg [QAWL-1:0] head,tail;
    reg [7:0] sx,sy,cx,cy,seed_x,seed_y;
    reg [7:0] xmin,xmax,ymin,ymax;
    reg [2:0] neighbor;
    reg [15:0] pix_count;
    reg [23:0] work_cycles;
    reg [15:0] reject_count;
    reg [MAX_OBJECTS*36-1:0] pending_objects;
    reg [MAX_OBJECTS-1:0] pending_valid;
    reg [3:0] object_count;
    reg [3:0] shape_class;

    // ---- span fill pass ----------------------------------------------------
    reg [7:0] fx,fy,row_l_r,row_r_r,row_idx,last_ink_x;
    reg prev_ink,row_any,span_done;
    reg [2:0] run_idx;
    reg [7:0] rl[0:NR-1],rr[0:NR-1];
    reg [2:0] rc[0:NR-1];

    // ---- post passes -------------------------------------------------------
    reg [7:0] pi,fminx,fmaxx,fminy,fmaxy,fw,fh,fxmid,fymid;
    reg [7:0] s1,s2,s3,s4,s5,wq1,wq2,wq3,wq4,wq5,full;
    reg [23:0] area,q0,q1,q2,q3;

    // ---- neighbour coordinates --------------------------------------------
    function signed [9:0] dx8;
      input [2:0] d;
      begin case(d) 0,1,7:dx8=1;3,4,5:dx8=-1;default:dx8=0;endcase end
    endfunction
    function signed [9:0] dy8;
      input [2:0] d;
      begin case(d) 1,2,3:dy8=1;5,6,7:dy8=-1;default:dy8=0;endcase end
    endfunction
    wire signed [9:0] nbx=$signed({2'b0,cx})+dx8(neighbor);
    wire signed [9:0] nby=$signed({2'b0,cy})+dy8(neighbor);
    wire nb_inside=(nbx>=0)&&(nbx<W)&&(nby>=0)&&(nby<H);
    wire [9:0] nbx_u=nbx;
    wire [9:0] nby_u=nby;
    wire [31:0] nb_addr_w=nby_u*W+nbx_u;
    wire [AW-1:0] nb_addr=nb_addr_w[AW-1:0];

    wire [8:0] box_w={1'b0,xmax}-{1'b0,xmin}+9'd1;
    wire [8:0] box_h={1'b0,ymax}-{1'b0,ymin}+9'd1;

    // Circular frontier queue: the frontier of a 160x90 contour is small, but
    // the total push count is not, so the index must wrap instead of saturating.
    wire q_full=((tail+{{(QAWL-1){1'b0}},1'b1})==head);

    wire [31:0] fill_addr_w=fy*W+fx;
    wire [AW-1:0] fill_addr=fill_addr_w[AW-1:0];

    // ---- span fill decision -----------------------------------------------
    // A per-pixel component tag keeps earlier components out of this fill; an
    // isolated speckle inside the bbox cannot open or close the slice.
    wire ink_c=raw_data_i&&(visited_rd==component_id);
    wire run_start=(state==F_EVAL)&&!prev_ink&&ink_c;
    wire run_end=(state==F_EVAL)&&prev_ink&&!ink_c;
    wire [2:0] ridx=run_idx+((run_start&&run_idx!=7)?3'd1:3'd0);
    // Slice = run 1 start .. run 2 end. A third run or a finished run-2 closes
    // the slice so the fill never runs into the background on the right.
    wire span_stop=span_done||(ridx>=3'd3)||(run_end&&run_idx>=3'd2);
    wire fill_now=(state==F_EVAL)&&(ridx!=3'd0)&&!span_stop;
    wire row_first=fill_now&&!row_any;
    wire [7:0] eff_l=row_first?fx:row_l_r;
    wire [7:0] eff_r=fill_now?fx:row_r_r;
    wire eff_any=row_any|fill_now;

    // ---- row extent of the current post-pass row --------------------------
    reg [7:0] rl_q,rr_q;
    reg [2:0] rc_q;
    always @(*) begin
      rl_q=8'hff;rr_q=8'hff;rc_q=3'd0;
      if(pi<NR) begin rl_q=rl[pi];rr_q=rr[pi];rc_q=rc[pi];end
    end
    wire solid_component=pix_count*3>=box_w*box_h;
    wire span_ok=(rl_q!=8'hff)&&((rc_q==3'd2)||(rc_q==3'd1&&solid_component));
    wire [7:0] pp_y=ymin+pi;
    wire [7:0] pp_w=(rr_q-rl_q)+1'b1;
    wire [8:0] pp_left=(rr_q<=fxmid)?({1'b0,rr_q}-{1'b0,rl_q}+9'd1):
        ((rl_q<=fxmid)?({1'b0,fxmid}-{1'b0,rl_q}+9'd1):9'd0);
    wire [8:0] pp_right=(rl_q>fxmid)?({1'b0,rr_q}-{1'b0,rl_q}+9'd1):
        ((rr_q>fxmid)?({1'b0,rr_q}-{1'b0,fxmid}):9'd0);

    // ---- classification thresholds ----------------------------------------
    wire [23:0] ba={16'd0,fw}*{16'd0,fh};
    wire e38=area*50>=ba*19;      // extent >= 0.38
    wire e42=area*50>=ba*21;      // >= 0.42
    wire e62=area*50<=ba*31;      // <= 0.62
    wire e63=area*100>=ba*63;     // >= 0.63
    wire e66=area*50>=ba*33;      // >= 0.66
    wire e76=area*25<=ba*19;      // <= 0.76
    wire e84=area*25>=ba*21;      // >= 0.84
    wire e86=area*50<=ba*43;      // <= 0.86
    wire q_hi0=q0*10>=ba*7,q_hi1=q1*10>=ba*7,q_hi2=q2*10>=ba*7,q_hi3=q3*10>=ba*7;
    wire q_any_hi=q_hi0|q_hi1|q_hi2|q_hi3;
    wire q_lo0=q0*20<=ba*9,q_lo1=q1*20<=ba*9,q_lo2=q2*20<=ba*9,q_lo3=q3*20<=ba*9;
    wire q_any_lo=q_lo0|q_lo1|q_lo2|q_lo3;
    // A box fills its corner quadrants almost completely (~100%), a circle only
    // ~78%. The stricter 85% test keeps a pixelated circle from being read as a
    // square when its extent drifts up towards 0.85.
    wire q_v85_0=q0*20>=ba*17,q_v85_1=q1*20>=ba*17,q_v85_2=q2*20>=ba*17,q_v85_3=q3*20>=ba*17;
    wire q_all_vhi=q_v85_0&q_v85_1&q_v85_2&q_v85_3;
    wire q_m66_0=q0*50>=ba*33,q_m66_1=q1*50>=ba*33,q_m66_2=q2*50>=ba*33,q_m66_3=q3*50>=ba*33;
    wire q_m92_0=q0*25<=ba*23,q_m92_1=q1*25<=ba*23,q_m92_2=q2*25<=ba*23,q_m92_3=q3*25<=ba*23;
    wire q_all_mid=q_m66_0&q_m92_0&q_m66_1&q_m92_1&q_m66_2&q_m92_2&q_m66_3&q_m92_3;
    wire q_s62_0=q0*50<=ba*31,q_s62_1=q1*50<=ba*31,q_s62_2=q2*50<=ba*31,q_s62_3=q3*50<=ba*31;
    wire q_s68_0=q0*50>=ba*34,q_s68_1=q1*50>=ba*34,q_s68_2=q2*50>=ba*34,q_s68_3=q3*50>=ba*34;
    wire q_p45_0=q0*20>=ba*9,q_p45_1=q1*20>=ba*9,q_p45_2=q2*20>=ba*9,q_p45_3=q3*20>=ba*9;
    wire q_p70_0=q0*10<=ba*7,q_p70_1=q1*10<=ba*7,q_p70_2=q2*10<=ba*7,q_p70_3=q3*10<=ba*7;
    wire q_d40_0=q0*5>=ba*2,q_d40_1=q1*5>=ba*2,q_d40_2=q2*5>=ba*2,q_d40_3=q3*5>=ba*2;
    wire q_plus=q_p45_0&q_p45_1&q_p45_2&q_p45_3&q_p70_0&q_p70_1&q_p70_2&q_p70_3;
    wire q_dia=q_d40_0&q_d40_1&q_d40_2&q_d40_3&q_s62_0&q_s62_1&q_s62_2&q_s62_3;
    wire sq_aspect=(fw*50>=fh*43)&&(fw*25<=fh*29);   // 0.86 .. 1.16
    wire w1_le50=wq1*2<=fw;
    wire w1_ge60=wq1*5>=fw*3;
    wire w1_le82=wq1*50<=fw*41;
    wire w2_le95=wq2*20<=fw*19;
    wire w5_ge60=wq5*5>=fw*3;
    wire w5_le50=wq5*2<=fw;
    wire full_ge20=full*5>=fh;
    wire full_le10=full*10<=fh;
    // A triangle occupies one half of its bbox: the two quadrants on the apex
    // side are almost empty and the opposite pair is almost full. Single-corner
    // occupancy is kept for right triangles. A trapezoid (all quadrants >=50)
    // matches neither, which is what keeps it out of the triangle class.
    wire tri_a=q_lo0&q_lo1&q_hi2&q_hi3;
    wire tri_b=q_hi0&q_hi1&q_lo2&q_lo3;
    wire tri_c=q_lo0&q_lo2&q_hi1&q_hi3;
    wire tri_d=q_hi0&q_hi2&q_lo1&q_lo3;
    wire tri_e=q_hi0&q_lo3;
    wire tri_f=q_lo0&q_hi3;
    wire tri_g=q_hi1&q_lo2;
    wire tri_h=q_lo1&q_hi2;
    wire tri_pat=tri_a|tri_b|tri_c|tri_d|tri_e|tri_f|tri_g|tri_h;
    wire pent_up=e63&e76&sq_aspect&q_p70_0&q_p70_1&q_s68_2&q_s68_3&w1_le50&w5_ge60;
    wire pent_dn=e63&e76&sq_aspect&q_s68_0&q_s68_1&q_p70_2&q_p70_3&w1_ge60&w5_le50;

    reg [3:0] cls_c;
    always @(*) begin
      cls_c=4'd0;
      if(e38&e62&tri_pat) cls_c=4'd1;                                  // triangle
      else if(e84&q_all_vhi) cls_c=sq_aspect?4'd2:4'd3;                 // square / rectangle
      else if(pent_up|pent_dn) cls_c=4'd6;                              // pentagon
      else if(e66&e86&q_all_mid&w1_le82&w2_le95) cls_c=sq_aspect?4'd4:4'd5; // circle / ellipse
      else if(e42&e62&sq_aspect&q_plus&full_ge20) cls_c=4'd8;           // plus
      else if(e42&e62&sq_aspect&q_dia&full_le10) cls_c=4'd9;            // quad (diamond)
    end

    always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
        state<=IDLE;busy_o<=0;result_valid_o<=0;objects_o<=0;valid_o<=0;rejected_o<=0;cycles_o<=0;
        raw_addr_o<=0;visited_we<=0;visited_wa<=0;visited_ra<=0;visited_wd<=0;
        queue_we<=0;queue_wa<=0;queue_ra<=0;queue_wd<=0;
        index<=0;scan_index<=0;head<=0;tail<=0;
        sx<=0;sy<=0;cx<=0;cy<=0;seed_x<=0;seed_y<=0;xmin<=0;xmax<=0;ymin<=0;ymax<=0;
        neighbor<=0;pix_count<=0;component_id<=0;degree<=0;endpoint_count<=0;work_cycles<=0;reject_count<=0;
        pending_objects<=0;pending_valid<=0;object_count<=0;shape_class<=0;
        fx<=0;fy<=0;last_ink_x<=0;row_l_r<=0;row_r_r<=0;row_idx<=0;prev_ink<=0;row_any<=0;run_idx<=0;span_done<=0;
        pi<=0;fminx<=0;fmaxx<=0;fminy<=0;fmaxy<=0;fw<=0;fh<=0;fxmid<=0;fymid<=0;
        s1<=0;s2<=0;s3<=0;s4<=0;s5<=0;wq1<=0;wq2<=0;wq3<=0;wq4<=0;wq5<=0;full<=0;
        area<=0;q0<=0;q1<=0;q2<=0;q3<=0;
      end else begin
        result_valid_o<=0;visited_we<=0;queue_we<=0;
        if(busy_o) work_cycles<=work_cycles+1'b1;
        if(busy_o&&work_cycles==24'd4000000) begin
          // Bounded fail-closed behaviour keeps video running no matter what.
          objects_o<=0;valid_o<=0;rejected_o<=16'hffff;cycles_o<=work_cycles;
          result_valid_o<=1;busy_o<=0;state<=IDLE;
        end else case(state)
          IDLE: if(start_i) begin
            busy_o<=1;pending_objects<=0;pending_valid<=0;object_count<=0;reject_count<=0;
            work_cycles<=0;index<=0;component_id<=0;state<=CLR;
          end
          CLR: begin
            visited_we<=1;visited_wa<=index;visited_wd<=0;
            if(index==N-1) begin
              index<=0;scan_index<=0;sx<=0;sy<=0;state<=SCAN_ISS;
            end else index<=index+1'b1;
          end
          SCAN_ISS: begin
            raw_addr_o<=scan_index;visited_ra<=scan_index;state<=SCAN_WAIT;
          end
          SCAN_WAIT: state<=SCAN_EVAL;
          SCAN_EVAL: begin
            if(raw_data_i&&!visited_rd) begin
              seed_x<=sx;seed_y<=sy;
              xmin<=sx;xmax<=sx;ymin<=sy;ymax<=sy;
              pix_count<=0;head<=0;tail<=1;component_id<=component_id+1'b1;endpoint_count<=0;
              queue_we<=1;queue_wa<=0;queue_wd<={sy,sx};
              visited_we<=1;visited_wa<=scan_index;visited_wd<=component_id+1'b1;
              state<=POP_ISS;
            end else if(scan_index==N-1) state<=PUBLISH;
            else begin
              scan_index<=scan_index+1'b1;
              if(sx==W-1) begin sx<=0;sy<=sy+1'b1;end else sx<=sx+1'b1;
              state<=SCAN_ISS;
            end
          end
          SCAN_ADV: begin
            if(scan_index==N-1) state<=PUBLISH;
            else begin
              scan_index<=scan_index+1'b1;
              if(sx==W-1) begin sx<=0;sy<=sy+1'b1;end else sx<=sx+1'b1;
              state<=SCAN_ISS;
            end
          end
          POP_ISS: if(head==tail) state<=FILTER;
                   else begin queue_ra<=head;state<=POP_WAIT;end
          POP_WAIT: state<=POP_EVAL;
          POP_EVAL: begin
            cx<=queue_rd[7:0];cy<=queue_rd[15:8];
            head<=head+1'b1;pix_count<=pix_count+1'b1;
            if(queue_rd[7:0]<xmin) xmin<=queue_rd[7:0];
            if(queue_rd[7:0]>xmax) xmax<=queue_rd[7:0];
            if(queue_rd[15:8]<ymin) ymin<=queue_rd[15:8];
            if(queue_rd[15:8]>ymax) ymax<=queue_rd[15:8];
            neighbor<=0;degree<=0;
            if(pix_count>=MAX_PIXELS) state<=DRAIN_ISS;
            else state<=NB_ISS;
          end
          NB_ISS: begin
            if(!nb_inside) begin
              if(neighbor==7) begin if(degree<2) endpoint_count<=endpoint_count+1'b1;state<=POP_ISS;end else neighbor<=neighbor+1'b1;
            end else begin
              raw_addr_o<=nb_addr;visited_ra<=nb_addr;state<=NB_WAIT;
            end
          end
          NB_WAIT: state<=NB_EVAL;
          NB_EVAL: begin
            if(raw_data_i) degree<=degree+1'b1;
            if(raw_data_i&&!visited_rd&&!q_full) begin
              queue_we<=1;queue_wa<=tail;queue_wd<={nby_u[7:0],nbx_u[7:0]};
              tail<=tail+1'b1;
              visited_we<=1;visited_wa<=nb_addr;visited_wd<=component_id;
            end
            if(raw_data_i&&!visited_rd&&q_full) begin
              pending_objects<=0;pending_valid<=0;reject_count<=16'hffff;state<=PUBLISH;
            end else if(neighbor==7) begin
              if(degree+(raw_data_i?1:0)<2) endpoint_count<=endpoint_count+1'b1;
              state<=POP_ISS;
            end else begin neighbor<=neighbor+1'b1;state<=NB_ISS;end
          end
          // Abort this analysis on oversized components; scanning their unvisited
          // remainder could otherwise classify fragments as separate shapes.
          DRAIN_ISS: begin pending_objects<=0;pending_valid<=0;reject_count<=16'hffff;state<=PUBLISH;end
          DRAIN_WAIT: state<=FILTER;
          DRAIN_EVAL: state<=FILTER;
          FILTER: begin
            if(endpoint_count>=2||pix_count<MIN_PIXELS||box_w<MIN_BOX||box_h<MIN_BOX||
               xmin<=1||ymin<=1||xmax>=W-2||ymax>=H-2||
               box_w>MAXBW||box_h>MAXBH||pix_count>=MAX_PIXELS||
               box_w>5*box_h||box_h>5*box_w) begin
              reject_count<=reject_count+1'b1;state<=SCAN_ADV;
            end else begin
              fx<=xmin;fy<=ymin;row_idx<=0;row_any<=0;prev_ink<=0;run_idx<=0;
              span_done<=0;row_l_r<=0;row_r_r<=0;state<=F_ISS;
            end
          end
          F_ISS: begin
            raw_addr_o<=fill_addr;visited_ra<=fill_addr;state<=F_WAIT;
          end
          F_WAIT: state<=F_EVAL;
          F_EVAL: begin
            prev_ink<=ink_c;
            if(ink_c) last_ink_x<=fx;
            if(run_start) run_idx<=ridx;
            if(run_end&&run_idx>=3'd2) span_done<=1'b1;
            // The slice is measured only; interior pixels are deliberately not
            // marked visited, otherwise a shape nested inside a larger contour
            // would be swallowed before it could be detected on its own.
            if(fill_now) begin
              row_l_r<=eff_l;row_r_r<=eff_r;row_any<=1'b1;
            end
            if(fx==xmax) begin
              rl[row_idx]<=(ridx!=0&&eff_any)?eff_l:8'hff;
              rr[row_idx]<=(ridx==1)?(ink_c?fx:last_ink_x):eff_r;
              rc[row_idx]<=ridx;
              row_idx<=row_idx+1'b1;
              row_any<=1'b0;prev_ink<=1'b0;run_idx<=0;span_done<=1'b0;
              if(fy==ymax) state<=PP1_INIT;
              else begin fy<=fy+1'b1;fx<=xmin;state<=F_ISS;end
            end else begin fx<=fx+1'b1;state<=F_ISS;end
          end
          PP1_INIT: begin
            pi<=0;fminx<=8'hff;fmaxx<=0;fminy<=8'hff;fmaxy<=0;area<=0;state<=PP1_STEP;
          end
          PP1_STEP: begin
            if(pi>=box_h) state<=PP2_INIT;
            else begin
              if(span_ok) begin
                if(rl_q<fminx) fminx<=rl_q;
                if(rr_q>fmaxx) fmaxx<=rr_q;
                if(pp_y<fminy) fminy<=pp_y;
                if(pp_y>fmaxy) fmaxy<=pp_y;
                area<=area+{16'd0,pp_w};
              end
              pi<=pi+1'b1;
            end
          end
          PP2_INIT: begin
            fw<=fmaxx-fminx+1'b1;
            fh<=fmaxy-fminy+1'b1;
            fxmid<=({1'b0,fminx}+{1'b0,fmaxx})>>1;
            fymid<=({1'b0,fminy}+{1'b0,fmaxy})>>1;
            s1<=fminy+((fmaxy-fminy+1'b1)>>3);
            s2<=fminy+((fmaxy-fminy+1'b1)>>2);
            s3<=fminy+((fmaxy-fminy+1'b1)>>1);
            s4<=fminy+(((fmaxy-fminy+1'b1)*3)>>2);
            s5<=fminy+(((fmaxy-fminy+1'b1)*7)>>3);
            pi<=0;q0<=0;q1<=0;q2<=0;q3<=0;full<=0;
            wq1<=0;wq2<=0;wq3<=0;wq4<=0;wq5<=0;state<=PP2_STEP;
          end
          PP2_STEP: begin
            if(pi>=box_h) state<=CLASS;
            else begin
              if(span_ok) begin
                if(pp_y<=fymid) begin q0<=q0+(pp_left<<2);q1<=q1+(pp_right<<2);end
                else begin q2<=q2+(pp_left<<2);q3<=q3+(pp_right<<2);end
                if(pp_y==s1) wq1<=pp_w;
                if(pp_y==s2) wq2<=pp_w;
                if(pp_y==s3) wq3<=pp_w;
                if(pp_y==s4) wq4<=pp_w;
                if(pp_y==s5) wq5<=pp_w;
                if(pp_w*20>=fw*19) full<=full+1'b1;
              end
              pi<=pi+1'b1;
            end
          end
          CLASS: begin
            // A filled region below 8x8 has no stable extent at this scale.
            shape_class<=(fw<8||fh<8||area<32)?4'd0:cls_c;
            state<=STORE;
          end
          STORE: begin
            if(shape_class!=0&&object_count<MAX_OBJECTS) begin
              pending_objects[object_count*36 +: 36]<={shape_class,xmin,xmax,ymin,ymax};
              pending_valid[object_count]<=1'b1;object_count<=object_count+1'b1;
            end else reject_count<=reject_count+1'b1;
            state<=SCAN_ADV;
          end
          PUBLISH: begin
            objects_o<=pending_objects;valid_o<=pending_valid;
            rejected_o<=reject_count;cycles_o<=work_cycles;
            result_valid_o<=1;busy_o<=0;state<=IDLE;
          end
          default: state<=IDLE;
        endcase
      end
    end
endmodule
