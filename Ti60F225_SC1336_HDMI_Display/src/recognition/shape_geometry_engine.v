`timescale 1ns/1ps
// Ti60 geometry engine inspired by the ObNoDog processing stages. This is an
// original implementation, not a copy of its Xilinx RTL. One shared RAM port
// replaces eight replicated contour buffers. Flood-fill labeling separates
// components; Moore tracing measures the OUTER enclosed area (hollow-safe).
// Ramer-Douglas-Peucker vertices supplement circularity to distinguish polygons.
module shape_geometry_engine #(
    parameter integer W=160,H=90,AW=$clog2(W*H),MAX_OBJECTS=8,
    parameter integer MAX_TRACE=4096,PAW=$clog2(MAX_TRACE),MIN_AREA=36
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
    localparam [5:0] IDLE=0,M_ISS=1,M_WAIT=2,M_EVAL=3,V_CLEAR=4,
      SCAN_ISS=5,SCAN_WAIT=6,SCAN_EVAL=7,SCAN_ADV=8,
      POP_ISS=9,POP_WAIT=10,POP_EVAL=11,NB_ISS=12,NB_WAIT=13,NB_EVAL=14,FILTER=15,
      TRACE_INIT=16,TR_ISS=17,TR_WAIT=18,TR_EVAL=19,RDP_INIT=20,
      SEG_POP=21,SEG_A_ISS=22,SEG_A_WAIT=23,SEG_A_LOAD=24,
      SEG_B_ISS=25,SEG_B_WAIT=26,SEG_B_LOAD=27,
      SEG_P_ISS=28,SEG_P_WAIT=29,SEG_P_EVAL=30,SEG_MATH=31,SEG_DECIDE=32,
      CLASS_INIT=33,CLASS_EDGE=34,CLASS_EDGE_CHECK=35,CLASS_DECIDE=36,
      STORE=37,PUBLISH=38,PRUNE_START=39,PRUNE_MATH=40,PRUNE_CHECK=41,PRUNE_SHIFT=42;
    reg [5:0] state;
    reg temp_we,work_we,visited_we,queue_we,path_we;
    reg [AW-1:0] temp_wa,temp_ra,work_wa,work_ra,visited_wa,visited_ra,queue_wa,queue_ra;
    reg temp_wd,work_wd,visited_wd;
    wire temp_rd,work_rd,visited_rd;
    reg [15:0] queue_wd;wire [15:0] queue_rd;
    reg [PAW-1:0] path_wa,path_ra;reg [15:0] path_wd;wire [15:0] path_rd;
    shape_sdp_ram #(.WIDTH(1),.DEPTH(N),.AW(AW)) u_temp(.clk(clk),.we(temp_we),.wa(temp_wa),.wd(temp_wd),.ra(temp_ra),.rd(temp_rd));
    shape_sdp_ram #(.WIDTH(1),.DEPTH(N),.AW(AW)) u_work(.clk(clk),.we(work_we),.wa(work_wa),.wd(work_wd),.ra(work_ra),.rd(work_rd));
    shape_sdp_ram #(.WIDTH(1),.DEPTH(N),.AW(AW)) u_visited(.clk(clk),.we(visited_we),.wa(visited_wa),.wd(visited_wd),.ra(visited_ra),.rd(visited_rd));
    shape_sdp_ram #(.WIDTH(16),.DEPTH(N),.AW(AW)) u_queue(.clk(clk),.we(queue_we),.wa(queue_wa),.wd(queue_wd),.ra(queue_ra),.rd(queue_rd));
    shape_sdp_ram #(.WIDTH(16),.DEPTH(MAX_TRACE),.AW(PAW)) u_path(.clk(clk),.we(path_we),.wa(path_wa),.wd(path_wd),.ra(path_ra),.rd(path_rd));

    reg [AW-1:0] index,scan_index,head,tail;
    reg [7:0] mx,my,sx,sy,cx,cy,seed_x,seed_y,xmin,xmax,ymin,ymax;
    reg [3:0] neighbor;
    reg morph_pass,morph_acc;
    reg [15:0] component_pixels;
    // A watchdog bounds pathological components and frame processing.
    reg [23:0] work_cycles;
    reg [15:0] reject_count;
    reg [MAX_OBJECTS*36-1:0] pending_objects;
    reg [MAX_OBJECTS-1:0] pending_valid;
    reg [3:0] object_count;

    function signed [9:0] dx8;
      input [2:0] d;
      begin case(d) 0,1,7:dx8=1;3,4,5:dx8=-1;default:dx8=0;endcase end
    endfunction
    function signed [9:0] dy8;
      input [2:0] d;
      begin case(d) 1,2,3:dy8=1;5,6,7:dy8=-1;default:dy8=0;endcase end
    endfunction
    wire signed [9:0] nb_x=$signed({2'b0,cx})+dx8(neighbor[2:0]);
    wire signed [9:0] nb_y=$signed({2'b0,cy})+dy8(neighbor[2:0]);
    wire nb_inside=nb_x>=0&&nb_x<W&&nb_y>=0&&nb_y<H;
    wire [AW-1:0] nb_addr=nb_y*W+nb_x;
    reg signed [9:0] morph_dx,morph_dy;
    always @(*) begin
      case(neighbor) 0,3,6:morph_dx=-1;2,5,8:morph_dx=1;default:morph_dx=0;endcase
      case(neighbor) 0,1,2:morph_dy=-1;6,7,8:morph_dy=1;default:morph_dy=0;endcase
    end
    wire signed [9:0] morph_x=$signed({2'b0,mx})+morph_dx;
    wire signed [9:0] morph_y=$signed({2'b0,my})+morph_dy;
    wire morph_inside=morph_x>=0&&morph_x<W&&morph_y>=0&&morph_y<H;
    wire [AW-1:0] morph_addr=morph_y*W+morph_x;
    wire morph_bit=morph_inside ? (morph_pass?temp_rd:raw_data_i):1'b0;
    wire morph_next=morph_pass?(morph_acc&morph_bit):(morph_acc|morph_bit);
    wire [8:0] box_w={1'b0,xmax}-{1'b0,xmin}+1'b1;
    wire [8:0] box_h={1'b0,ymax}-{1'b0,ymin}+1'b1;

    reg [2:0] search_dir;
    reg [7:0] first_x,first_y;
    reg [3:0] attempts;
    wire signed [9:0] tr_x=$signed({2'b0,cx})+dx8(search_dir);
    wire signed [9:0] tr_y=$signed({2'b0,cy})+dy8(search_dir);
    wire tr_inside=tr_x>=0&&tr_x<W&&tr_y>=0&&tr_y<H;
    wire [AW-1:0] tr_addr=tr_y*W+tr_x;
    wire [2:0] next_back={search_dir[2:1],1'b0}+3'd6;
    reg [PAW-1:0] trace_count,trace_end,far_index;
    reg [19:0] perimeter10;
    reg signed [31:0] area_acc;
    reg [31:0] area2;
    reg [19:0] far_distance;
    wire signed [19:0] step_cross=$signed({1'b0,cx})*$signed(tr_y)-$signed({1'b0,cy})*$signed(tr_x);
    wire signed [31:0] next_area=area_acc+step_cross;
    wire signed [9:0] seed_dx=tr_x-$signed({2'b0,seed_x});
    wire signed [9:0] seed_dy=tr_y-$signed({2'b0,seed_y});
    wire [19:0] seed_distance=seed_dx*seed_dx+seed_dy*seed_dy;

    reg [PAW-1:0] stack_a[0:31],stack_b[0:31];
    reg [5:0] sp;
    reg [PAW-1:0] sa,sb,pi,max_index;
    reg [7:0] ax,ay,bx,by;
    reg [4:0] vertices,curved_leaves;
    reg [7:0] vx[0:15],vy[0:15];
    reg [7:0] epsilon2;
    reg [19:0] chord2;
    reg [19:0] max_cross;
    reg [39:0] cross_square,rdp_limit;
    wire signed [9:0] pax=$signed({2'b0,path_rd[7:0]})-$signed({2'b0,ax});
    wire signed [9:0] pay=$signed({2'b0,path_rd[15:8]})-$signed({2'b0,ay});
    wire signed [9:0] abx=$signed({2'b0,bx})-$signed({2'b0,ax});
    wire signed [9:0] aby=$signed({2'b0,by})-$signed({2'b0,ay});
    wire signed [20:0] rdp_cross=pax*aby-pay*abx;
    wire [19:0] abs_cross=rdp_cross<0?-rdp_cross:rdp_cross;

    reg [47:0] circularity_num,perimeter_square;
    reg [17:0] bbox_area;
    reg [4:0] vi;
    reg [4:0] shift_index,remove_index;
    wire [4:0] vj=vi+1>=vertices?vi+1-vertices:vi+1;
    wire [4:0] vk=vi+2>=vertices?vi+2-vertices:vi+2;
    wire signed [9:0] ex1=$signed({2'b0,vx[vj]})-$signed({2'b0,vx[vi]});
    wire signed [9:0] ey1=$signed({2'b0,vy[vj]})-$signed({2'b0,vy[vi]});
    wire signed [9:0] ex2=$signed({2'b0,vx[vk]})-$signed({2'b0,vx[vj]});
    wire signed [9:0] ey2=$signed({2'b0,vy[vk]})-$signed({2'b0,vy[vj]});
    wire signed [20:0] edge_cross=ex1*ey2-ey1*ex2;
    wire signed [20:0] edge_dot=ex1*ex2+ey1*ey2;
    wire signed [10:0] acx=ex1+ex2,acy=ey1+ey2;
    wire [21:0] acxx=acx*acx,acyy=acy*acy;
    wire [22:0] ac_length2={1'b0,acxx}+{1'b0,acyy};
    wire [19:0] side1=ex1*ex1+ey1*ey1;
    wire [19:0] side2=ex2*ex2+ey2*ey2;
    wire [9:0] abs_ex1=ex1<0?-ex1:ex1,abs_ey1=ey1<0?-ey1:ey1;
    reg convex,turn_set,turn_sign,orthogonal;
    reg [4:0] axis_edges,left_vertices,right_vertices,top_vertices,bottom_vertices;
    reg [19:0] min_side,max_side;
    reg [43:0] dot_square,side_product;
    reg [3:0] shape_class;
    integer i;

    always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
        state<=IDLE;busy_o<=0;result_valid_o<=0;objects_o<=0;valid_o<=0;rejected_o<=0;cycles_o<=0;
        raw_addr_o<=0;temp_we<=0;work_we<=0;visited_we<=0;queue_we<=0;path_we<=0;
        temp_wa<=0;temp_ra<=0;temp_wd<=0;work_wa<=0;work_ra<=0;work_wd<=0;
        visited_wa<=0;visited_ra<=0;visited_wd<=0;queue_wa<=0;queue_ra<=0;queue_wd<=0;
        path_wa<=0;path_ra<=0;path_wd<=0;index<=0;scan_index<=0;head<=0;tail<=0;
        mx<=0;my<=0;sx<=0;sy<=0;cx<=0;cy<=0;seed_x<=0;seed_y<=0;
        xmin<=0;xmax<=0;ymin<=0;ymax<=0;neighbor<=0;morph_pass<=0;morph_acc<=0;
        component_pixels<=0;work_cycles<=0;reject_count<=0;pending_objects<=0;pending_valid<=0;object_count<=0;
        search_dir<=4;first_x<=0;first_y<=0;attempts<=0;trace_count<=0;trace_end<=0;far_index<=0;perimeter10<=0;area_acc<=0;area2<=0;far_distance<=0;
        sp<=0;sa<=0;sb<=0;pi<=0;max_index<=0;ax<=0;ay<=0;bx<=0;by<=0;vertices<=0;curved_leaves<=0;epsilon2<=0;chord2<=0;
        max_cross<=0;cross_square<=0;rdp_limit<=0;circularity_num<=0;perimeter_square<=0;bbox_area<=0;vi<=0;shift_index<=0;remove_index<=0;
        convex<=1;turn_set<=0;turn_sign<=0;orthogonal<=1;axis_edges<=0;min_side<=20'hfffff;max_side<=0;
        left_vertices<=0;right_vertices<=0;top_vertices<=0;bottom_vertices<=0;dot_square<=0;side_product<=0;shape_class<=0;
        for(i=0;i<16;i=i+1) begin vx[i]<=0;vy[i]<=0;end
        for(i=0;i<32;i=i+1) begin stack_a[i]<=0;stack_b[i]<=0;end
      end else begin
        result_valid_o<=0;temp_we<=0;work_we<=0;visited_we<=0;queue_we<=0;path_we<=0;
        if(busy_o) work_cycles<=work_cycles+1'b1;
        if(busy_o&&work_cycles==24'd4000000) begin
          // Bounded fail-closed behavior; video still runs uninterrupted.
          objects_o<=0;valid_o<=0;rejected_o<=16'hffff;cycles_o<=work_cycles;
          result_valid_o<=1;busy_o<=0;state<=IDLE;
        end else case(state)
          IDLE: if(start_i) begin
            busy_o<=1;pending_objects<=0;pending_valid<=0;object_count<=0;reject_count<=0;work_cycles<=0;
            mx<=0;my<=0;index<=0;neighbor<=0;morph_pass<=0;morph_acc<=0;state<=M_ISS;
          end
          M_ISS: begin
            if(morph_pass) temp_ra<=morph_addr;else raw_addr_o<=morph_addr;
            state<=M_WAIT;
          end
          M_WAIT: state<=M_EVAL;
          M_EVAL: begin
            if(neighbor==8) begin
              if(morph_pass) begin work_we<=1;work_wa<=index;work_wd<=morph_next;end
              else begin temp_we<=1;temp_wa<=index;temp_wd<=morph_next;end
              neighbor<=0;morph_acc<=morph_pass;
              if(index==N-1) begin
                index<=0;mx<=0;my<=0;
                if(!morph_pass) begin morph_pass<=1;morph_acc<=1;state<=M_ISS;end
                else state<=V_CLEAR;
              end else begin
                index<=index+1'b1;
                if(mx==W-1) begin mx<=0;my<=my+1'b1;end else mx<=mx+1'b1;
                state<=M_ISS;
              end
            end else begin neighbor<=neighbor+1'b1;morph_acc<=morph_next;state<=M_ISS;end
          end
          V_CLEAR: begin
            visited_we<=1;visited_wa<=index;visited_wd<=0;
            if(index==N-1) begin scan_index<=0;sx<=0;sy<=0;state<=SCAN_ISS;end
            else index<=index+1'b1;
          end
          SCAN_ISS: begin work_ra<=scan_index;visited_ra<=scan_index;state<=SCAN_WAIT;end
          SCAN_WAIT: state<=SCAN_EVAL;
          SCAN_EVAL: begin
            if(work_rd&&!visited_rd) begin
              seed_x<=sx;seed_y<=sy;xmin<=sx;xmax<=sx;ymin<=sy;ymax<=sy;
              component_pixels<=0;head<=0;tail<=1;
              queue_we<=1;queue_wa<=0;queue_wd<={sy,sx};
              visited_we<=1;visited_wa<=scan_index;visited_wd<=1;state<=POP_ISS;
            end else state<=SCAN_ADV;
          end
          SCAN_ADV: begin
            if(scan_index==N-1) state<=PUBLISH;
            else begin scan_index<=scan_index+1'b1;if(sx==W-1) begin sx<=0;sy<=sy+1'b1;end else sx<=sx+1'b1;state<=SCAN_ISS;end
          end
          POP_ISS: begin
            if(head==tail) state<=FILTER;
            else begin queue_ra<=head;state<=POP_WAIT;end
          end
          POP_WAIT: state<=POP_EVAL;
          POP_EVAL: begin
            cx<=queue_rd[7:0];cy<=queue_rd[15:8];head<=head+1'b1;component_pixels<=component_pixels+1'b1;
            if(queue_rd[7:0]<xmin) xmin<=queue_rd[7:0];if(queue_rd[7:0]>xmax) xmax<=queue_rd[7:0];
            if(queue_rd[15:8]<ymin) ymin<=queue_rd[15:8];if(queue_rd[15:8]>ymax) ymax<=queue_rd[15:8];
            neighbor<=0;state<=NB_ISS;
          end
          NB_ISS: begin
            if(!nb_inside) begin if(neighbor==7) state<=POP_ISS;else neighbor<=neighbor+1'b1;end
            else begin work_ra<=nb_addr;visited_ra<=nb_addr;state<=NB_WAIT;end
          end
          NB_WAIT: state<=NB_EVAL;
          NB_EVAL: begin
            if(work_rd&&!visited_rd) begin
              if(tail<N) begin
                queue_we<=1;queue_wa<=tail;queue_wd<={nb_y[7:0],nb_x[7:0]};tail<=tail+1'b1;
                visited_we<=1;visited_wa<=nb_addr;visited_wd<=1;
              end
            end
            if(neighbor==7) state<=POP_ISS;else begin neighbor<=neighbor+1'b1;state<=NB_ISS;end
          end
          FILTER: begin
            if(component_pixels<6||box_w<8||box_h<8||xmin<=1||ymin<=1||xmax>=W-2||ymax>=H-2) begin
              reject_count<=reject_count+1'b1;state<=SCAN_ADV;
            end else state<=TRACE_INIT;
          end
          TRACE_INIT: begin
            cx<=seed_x;cy<=seed_y;search_dir<=4;attempts<=0;trace_count<=1;perimeter10<=0;area_acc<=0;
            far_distance<=0;far_index<=0;path_we<=1;path_wa<=0;path_wd<={seed_y,seed_x};state<=TR_ISS;
          end
          TR_ISS: begin
            if(!tr_inside) begin
              if(attempts==7) begin reject_count<=reject_count+1'b1;state<=SCAN_ADV;end
              else begin search_dir<=search_dir+1'b1;attempts<=attempts+1'b1;end
            end else begin work_ra<=tr_addr;state<=TR_WAIT;end
          end
          TR_WAIT: state<=TR_EVAL;
          TR_EVAL: begin
            if(work_rd) begin
              // Jacob's directed-edge stopping criterion: re-enter the seed
              // and depart along the same first edge. Requiring a particular
              // backtrack direction fails for inverted/rasterized triangles.
              if(cx==seed_x&&cy==seed_y&&tr_x==first_x&&tr_y==first_y&&trace_count>2) begin
                trace_end<=trace_count-1'b1;area2<=area_acc<0?-area_acc:area_acc;state<=RDP_INIT;
              end else if(trace_count==MAX_TRACE-1) begin reject_count<=reject_count+1'b1;state<=SCAN_ADV;end
              else begin
                path_we<=1;path_wa<=trace_count;path_wd<={tr_y[7:0],tr_x[7:0]};
                perimeter10<=perimeter10+(search_dir[0]?20'd14:20'd10);area_acc<=next_area;
                if(trace_count==1) begin first_x<=tr_x[7:0];first_y<=tr_y[7:0];end
                if(seed_distance>far_distance) begin far_distance<=seed_distance;far_index<=trace_count;end
                trace_count<=trace_count+1'b1;cx<=tr_x[7:0];cy<=tr_y[7:0];search_dir<=next_back;attempts<=0;state<=TR_ISS;
              end
            end else if(attempts==7) begin reject_count<=reject_count+1'b1;state<=SCAN_ADV;end
            else begin search_dir<=search_dir+1'b1;attempts<=attempts+1'b1;state<=TR_ISS;end
          end
          RDP_INIT: begin
            if(area2<MIN_AREA*2||far_index==0) begin reject_count<=reject_count+1'b1;state<=SCAN_ADV;end
            else begin
              stack_a[0]<=far_index;stack_b[0]<=trace_end;stack_a[1]<=0;stack_b[1]<=far_index;
              sp<=2;vertices<=0;curved_leaves<=0;
              epsilon2<=((box_w<box_h?box_w:box_h)>>4)<2 ? 4 :
                         (((box_w<box_h?box_w:box_h)>>4)*((box_w<box_h?box_w:box_h)>>4));
              state<=SEG_POP;
            end
          end
          SEG_POP: begin
            if(sp==0) state<=PRUNE_START;
            else begin sa<=stack_a[sp-1];sb<=stack_b[sp-1];sp<=sp-1'b1;state<=SEG_A_ISS;end
          end
          SEG_A_ISS: begin path_ra<=sa;state<=SEG_A_WAIT;end
          SEG_A_WAIT: state<=SEG_A_LOAD;
          SEG_A_LOAD: begin ax<=path_rd[7:0];ay<=path_rd[15:8];state<=SEG_B_ISS;end
          SEG_B_ISS: begin path_ra<=sb;state<=SEG_B_WAIT;end
          SEG_B_WAIT: state<=SEG_B_LOAD;
          SEG_B_LOAD: begin
            bx<=path_rd[7:0];by<=path_rd[15:8];pi<=sa+1'b1;max_cross<=0;max_index<=sa;
            // Use newly fetched endpoint here, not the previous segment's bx.
            chord2<=($signed({2'b0,path_rd[7:0]})-$signed({2'b0,ax}))*($signed({2'b0,path_rd[7:0]})-$signed({2'b0,ax}))+
                    ($signed({2'b0,path_rd[15:8]})-$signed({2'b0,ay}))*($signed({2'b0,path_rd[15:8]})-$signed({2'b0,ay}));
            if(sb<=sa+1) state<=SEG_MATH;else state<=SEG_P_ISS;
          end
          SEG_P_ISS: begin path_ra<=pi;state<=SEG_P_WAIT;end
          SEG_P_WAIT: state<=SEG_P_EVAL;
          SEG_P_EVAL: begin
            if(abs_cross>max_cross) begin max_cross<=abs_cross;max_index<=pi;end
            if(pi==sb-1) state<=SEG_MATH;else begin pi<=pi+1'b1;state<=SEG_P_ISS;end
          end
          SEG_MATH: begin cross_square<=max_cross*max_cross;rdp_limit<=chord2*epsilon2;state<=SEG_DECIDE;end
          SEG_DECIDE: begin
            if(cross_square>rdp_limit&&max_index>sa&&max_index<sb) begin
              if(sp>29||vertices+sp>=15) begin reject_count<=reject_count+1'b1;state<=SCAN_ADV;end
              else begin
                stack_a[sp]<=max_index;stack_b[sp]<=sb;stack_a[sp+1]<=sa;stack_b[sp+1]<=max_index;sp<=sp+2;state<=SEG_POP;
              end
            end else if(vertices==16) begin reject_count<=reject_count+1'b1;state<=SCAN_ADV;end
            else begin
              vx[vertices]<=ax;vy[vertices]<=ay;vertices<=vertices+1'b1;
              // Leaf deviation above one pixel is evidence for an arc rather
              // than a straight polygon side (ignore tiny chords).
              if(chord2>16&&cross_square>chord2&&curved_leaves<31) curved_leaves<=curved_leaves+1'b1;
              state<=SEG_POP;
            end
          end
          // A closed contour's seed/farthest split can lie in the middle of
          // a straight side. Remove those artificial vertices, including
          // across the wrap-around, before counting polygon corners.
          PRUNE_START: begin vi<=0;state<=PRUNE_MATH;end
          PRUNE_MATH: begin
            cross_square<=edge_cross*edge_cross;
            rdp_limit<=epsilon2*ac_length2;
            state<=PRUNE_CHECK;
          end
          PRUNE_CHECK: begin
`ifdef SHAPE_TRACE_DEBUG
            $display("prune n=%0d vi=%0d cross=%0d square=%0d limit=%0d dot=%0d eps=%0d e1=%0d,%0d e2=%0d,%0d",vertices,vi,edge_cross,cross_square,rdp_limit,edge_dot,epsilon2,ex1,ey1,ex2,ey2);
`endif
            if(vertices>3&&edge_dot>=0&&cross_square<=rdp_limit) begin
              remove_index<=vj;shift_index<=vj;state<=PRUNE_SHIFT;
            end else if(vi==vertices-1) state<=CLASS_INIT;
            else begin vi<=vi+1'b1;state<=PRUNE_MATH;end
          end
          PRUNE_SHIFT: begin
            if(shift_index==vertices-1) begin vertices<=vertices-1'b1;state<=PRUNE_START;end
            else begin vx[shift_index]<=vx[shift_index+1];vy[shift_index]<=vy[shift_index+1];shift_index<=shift_index+1'b1;end
          end
          CLASS_INIT: begin
            circularity_num<=area2*48'd62832;perimeter_square<=perimeter10*perimeter10;bbox_area<=box_w*box_h;
            vi<=0;convex<=1;turn_set<=0;orthogonal<=1;axis_edges<=0;
            left_vertices<=0;right_vertices<=0;top_vertices<=0;bottom_vertices<=0;
            min_side<=20'hfffff;max_side<=0;
            if(vertices<3) state<=SCAN_ADV;else state<=CLASS_EDGE;
          end
          CLASS_EDGE: begin
            if(edge_cross!=0) begin
              if(!turn_set) begin turn_set<=1;turn_sign<=edge_cross[20];end
              else if(turn_sign!=edge_cross[20]) convex<=0;
            end
            if(abs_ex1<=2||abs_ey1<=2) axis_edges<=axis_edges+1'b1;
            if(vx[vi]<=xmin+2) left_vertices<=left_vertices+1'b1;
            if(vx[vi]+2>=xmax) right_vertices<=right_vertices+1'b1;
            if(vy[vi]<=ymin+2) top_vertices<=top_vertices+1'b1;
            if(vy[vi]+2>=ymax) bottom_vertices<=bottom_vertices+1'b1;
            if(side1<min_side) min_side<=side1;if(side1>max_side) max_side<=side1;
            dot_square<=edge_dot*edge_dot;side_product<=side1*side2;state<=CLASS_EDGE_CHECK;
          end
          CLASS_EDGE_CHECK: begin
            if((dot_square<<4)>side_product) orthogonal<=0;
            if(vi==vertices-1) state<=CLASS_DECIDE;else begin vi<=vi+1'b1;state<=CLASS_EDGE;end
          end
          CLASS_DECIDE: begin
            shape_class<=0;
            if(vertices==3&&convex&&circularity_num>=perimeter_square*25) shape_class<=1; // TRIANGLE
            else if(vertices==4&&convex) begin
              if(orthogonal) shape_class<=(max_side*4<=min_side*5)?2:3; // SQUARE / RECTANGLE
              else shape_class<=9; // non-rectangular convex QUAD
            end else if(vertices==5&&convex) shape_class<=6; // PENTAGON
            else if((vertices>=7||(vertices==6&&curved_leaves>=3))&&convex&&circularity_num>=perimeter_square*60&&
                    area2*100>=bbox_area*125&&area2*100<=bbox_area*195) begin
              shape_class<=((box_w*4<=box_h*5)&&(box_h*4<=box_w*5)&&circularity_num>=perimeter_square*85)?4:5;
            end else if(vertices==12&&!convex&&axis_edges>=10&&left_vertices==2&&right_vertices==2&&top_vertices==2&&bottom_vertices==2)
              shape_class<=8; // PLUS
            state<=STORE;
          end
          STORE: begin
            if(shape_class!=0&&object_count<MAX_OBJECTS) begin
              pending_objects[object_count*36 +: 36]<={shape_class,xmin,xmax,ymin,ymax};
              pending_valid[object_count]<=1;object_count<=object_count+1'b1;
            end else reject_count<=reject_count+1'b1;
            state<=SCAN_ADV;
          end
          PUBLISH: begin
            objects_o<=pending_objects;valid_o<=pending_valid;rejected_o<=reject_count;cycles_o<=work_cycles;
            result_valid_o<=1;busy_o<=0;state<=IDLE;
          end
          default: state<=IDLE;
        endcase
      end
    end
endmodule
