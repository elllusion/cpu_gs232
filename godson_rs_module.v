/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

`include "global.h"
`include "bus.h"        
`include "reg.h"

module godson_alurs1_module(
    clock,
    reset,
    commitbus_ex,
    rissuebus,
    alursfull,
    allow,
    alurs_to_alu,
    brbus_to_rs
);
    
input clock;     
input reset;                   
input commitbus_ex;
input [`Lrissuebus_to_fix-1:0] rissuebus;     
input allow; //it is always 1'b1
input [6:0] brbus_to_rs;
output alursfull;
output [84:0] alurs_to_alu;
wire[2:0]   issue_brqid = rissuebus[81:79];     
wire[1:0]   issue_ac    = rissuebus[78:77];     
wire        issue_valid = rissuebus[76];     
wire [3:0]  issue_qid   = rissuebus[75:72];     
wire [7:0]  issue_op    = rissuebus[71:64];     
wire [31:0] issue_Vj    = rissuebus[63:32];     
wire [31:0] issue_Vk    = rissuebus[31:0];     

//HAVE_DSP_UNIT
wire op_of_qb = ((issue_op==`OP_SUBQ) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_CMP_LT)| (issue_op==`OP_CMP_LE) |
                 (issue_op==`OP_ADDQ) | (issue_op==`OP_ADDQ_S) | (issue_op==`OP_RADDU) | (issue_op==`OP_PICK)) & 
                 (issue_ac==2'b00);
                
wire sub_op = (issue_op==`OP_SUBQ) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_CMP_LT) | (issue_op==`OP_CMP_LE);

wire op_of_ph = ((issue_op==`OP_SUBQ) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_CMP_LT) | (issue_op==`OP_CMP_LE) |
                 (issue_op==`OP_ADDQ) | (issue_op==`OP_ADDQ_S)) & (issue_ac==2'b01);

wire op_of_w = ((issue_op==`OP_ADDQ_S) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_ADDWC) | (issue_op==`OP_ADDSC) | 
                (issue_op==`OP_MODSUB)) & (issue_ac==2'b10);
wire brbus_brerr;
assign brbus_brerr  = brbus_to_rs[0];
wire [5:0] brbus_brmask;
assign brbus_brmask = brbus_to_rs[6:1]; 
//-----------------RS registers defination---------------------       
reg       alurs_busy;  
reg [7:0] alurs_op;   
reg [3:0] alurs_qid; 
reg [2:0] alurs_brqid;
wire myout;
wire issue_to_alu;
wire issue_can_write;               
wire B_enable;
wire set_B;                   
//-----------------local wires defination----------------------
wire Vk_enable;
wire [31:0] wire_Vk_in;

wire Vj_enable;
wire [31:0] wire_Vj_in;

reg [31:0] alurs_vk;
reg [31:0] alurs_vj;

assign Vk_enable         = issue_can_write;
assign wire_Vk_in        = alurs_vk;

assign Vj_enable         = issue_can_write;
assign wire_Vj_in        = alurs_vj;

wire brcancel_issue;
assign brcancel_issue = (brbus_brmask[issue_brqid]);
wire brcancel_out;
assign brcancel_out = (brbus_brmask[alurs_brqid]);
always @(posedge clock)
begin

  if(Vk_enable)
   begin
      alurs_vk[31:0] <= issue_Vk;
   end
   if(Vj_enable)
   begin
      alurs_vj[31:0] <= issue_Vj; 
   end

end

assign myout        = alurs_busy; 
assign issue_to_alu = allow & myout; 
assign issue_can_write = issue_valid & !brcancel_issue; 
assign B_enable        = reset  | commitbus_ex  | issue_to_alu | issue_can_write | brcancel_out;
assign set_B           = ~reset & ~commitbus_ex & issue_can_write; 

reg [2:0]alurs_op_ac;

always @(posedge clock)
    begin
      if(B_enable)
         alurs_busy<=set_B;
    end    
always @(posedge clock)
    begin
      if(issue_can_write)
         begin
           alurs_op_ac<={sub_op, (op_of_qb ? 2'b01 : op_of_ph ? 2'b10 : op_of_w ? 2'b11 :2'b00)};
           alurs_op<=issue_op;
           alurs_qid<=issue_qid;
           alurs_brqid<=issue_brqid;
         end
    end

//HAVE_DSP_UNIT
reg [1:0] alurs_ac;
always @(posedge clock)
    begin
      if (issue_can_write)
          alurs_ac<=issue_ac;
    end

// HAVE_DSP_UNIT
assign alurs_to_alu = {alurs_op_ac, alurs_brqid, myout, alurs_op, alurs_ac, alurs_qid, wire_Vj_in, wire_Vk_in};

assign alursfull = 1'b0;     //error:  lumin
endmodule    

module godson_alurs2_module(clock,
                           reset,
                           commitbus_ex,
                           rissuebus,
                           allow,
                           alursfull,
                           alurs_to_alu,
                           brbus_to_rs);
    
input clock;     
input reset;                   
input commitbus_ex;
input [`Lrissuebus_to_fix-1:0] rissuebus;     
input allow; //from alu
input [6:0] brbus_to_rs;
output alursfull; //to queue   error:  lumin
output [87:0] alurs_to_alu;
assign alursfull = 1'b0;     //error:  lumin
wire[2:0]   issue_brqid = rissuebus[81:79];     
wire[1:0]   issue_ac    = rissuebus[78:77];     
wire        issue_valid = rissuebus[76];     
wire [3:0]  issue_qid   = rissuebus[75:72];     
wire [7:0]  issue_op    = rissuebus[71:64];     
wire [31:0] issue_Vj    = rissuebus[63:32];     
wire [31:0] issue_Vk    = rissuebus[31:0];     

//HAVE_DSP_UNIT
wire mul_reg_1 = issue_op==`OP_MULEU_S_PH_QBL;
wire mul_reg_2 = issue_op==`OP_MULEU_S_PH_QBR;
wire mul_reg_3 = issue_op==`OP_DPAQ_S_W_PH  | issue_op==`OP_DPSQ_S_W_PH  | issue_op==`OP_MAQ_S_W_PHR |
                 issue_op==`OP_MAQ_SA_W_PHR | issue_op==`OP_MULSAQ_S_W_PH| issue_op==`OP_MULQ_RS_PH |
                 issue_op==`OP_MULEQ_S_W_PHR;

wire mul_reg_4 = issue_op==`OP_MAQ_S_W_PHL | issue_op==`OP_MAQ_SA_W_PHL |issue_op==`OP_MULEQ_S_W_PHL;
wire mul_reg_5 = issue_op==`OP_DPAU_H_QBL | issue_op==`OP_DPSU_H_QBL; 
wire mul_reg_6 = issue_op==`OP_DPAU_H_QBR | issue_op==`OP_DPSU_H_QBR;    

reg[2:0] mul_reg;
always @(posedge clock) begin
 if(reset)
    mul_reg <= 3'b000;
 if (issue_valid)
     mul_reg <= mul_reg_1 ? 3'b001 : mul_reg_2 ? 3'b010 : mul_reg_3 ? 3'b011 : mul_reg_4 ? 3'b100 :
                mul_reg_5 ? 3'b101 : mul_reg_6 ? 3'b110 : 3'b000; 
                
end

// HAVE_DSP_UNIT
wire op_of_qb = ((issue_op==`OP_SUBQ) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_CMP_LT)| (issue_op==`OP_CMP_LE) |
                 (issue_op==`OP_ADDQ) | (issue_op==`OP_ADDQ_S) | (issue_op==`OP_RADDU) | (issue_op==`OP_PICK)) & 
                (issue_ac==2'b00);
                
wire sub_op = (issue_op==`OP_SUBQ) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_CMP_LT) | (issue_op==`OP_CMP_LE);

wire op_of_ph = ((issue_op==`OP_SUBQ) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_CMP_LT) | (issue_op==`OP_CMP_LE) |
                 (issue_op==`OP_ADDQ) | (issue_op==`OP_ADDQ_S)) & (issue_ac==2'b01);

wire op_of_w = ((issue_op==`OP_ADDQ_S) | (issue_op==`OP_SUBQ_S) | (issue_op==`OP_ADDWC) | (issue_op==`OP_ADDSC) | 
                (issue_op==`OP_MODSUB)) & (issue_ac==2'b10);
wire brbus_brerr;
assign brbus_brerr  = brbus_to_rs[0];
wire [5:0] brbus_brmask;
assign brbus_brmask = brbus_to_rs[6:1]; 
//-----------------RS registers defination---------------------       
reg       alurs_busy;  
reg [7:0] alurs_op;   
reg [3:0] alurs_qid;
reg [2:0] alurs_brqid;
wire myout;
wire issue_to_alu;
wire issue_can_write;               
wire B_enable;
wire set_B;                   
//-----------------local wires defination----------------------
wire Vk_enable;
wire [31:0] wire_Vk_in;

wire Vj_enable;
wire [31:0] wire_Vj_in;

reg [31:0] alurs_vk;
reg [31:0] alurs_vj;

assign Vk_enable         = issue_can_write;
assign wire_Vk_in        = alurs_vk;

assign Vj_enable         = issue_can_write;
assign wire_Vj_in        = alurs_vj;

wire brcancel_issue;
assign brcancel_issue = (brbus_brmask[issue_brqid]);
wire brcancel_out;
assign brcancel_out = (brbus_brmask[alurs_brqid]);
always @(posedge clock)
begin

  if(Vk_enable)
   begin
      alurs_vk[31:0] <= issue_Vk;
   end
   if(Vj_enable)
   begin
      alurs_vj[31:0] <= issue_Vj; 
   end

end

assign myout        = alurs_busy; 
assign issue_to_alu = allow & myout; 
assign issue_can_write = issue_valid & !brcancel_issue; 
assign B_enable        = reset  | commitbus_ex  | issue_to_alu | issue_can_write | brcancel_out;
assign set_B           = ~reset & ~commitbus_ex & issue_can_write; 

reg [2:0] alurs_op_ac;
always @(posedge clock)
    begin
      if(B_enable)
         alurs_busy<=set_B;
    end    
always @(posedge clock)
    begin
      if(issue_can_write)
         begin
           alurs_op_ac<={sub_op, (op_of_qb ? 2'b01 : op_of_ph ? 2'b10 : op_of_w ? 2'b11 :2'b00)};
           alurs_op<=issue_op;
           alurs_qid<=issue_qid;
           alurs_brqid<=issue_brqid;
         end
    end

//HAVE_DSP_UNIT
reg [1:0] alurs_ac;
always @(posedge clock)
    begin
      if (issue_can_write)
          alurs_ac<=issue_ac;
    end

// HAVE_DSP_UNIT
assign alurs_to_alu = {alurs_op_ac, mul_reg, alurs_brqid, myout, alurs_op, alurs_ac, alurs_qid, wire_Vj_in, wire_Vk_in};
wire alursbusy = alurs_busy & !allow;  

endmodule    
/*****************************************************************************************
**************************godson addr reserve station module******************************
*****************************************************************************************/
module godson_mmrs_module (clock,
                           reset,
                           commitbus_ex,
                           rissuebus,
                           allow,

                           mmrs_to_addr,
                           mmrsfull,
                           brbus_to_rs
                           ,qissuebus0_src1_rdy,
                            mmres,
                            from_queue_qj
                           );

 input clock;
 input reset;
 input commitbus_ex;
 input [`Lrissuebus_to_mm-1:0] rissuebus;
 input qissuebus0_src1_rdy;
 input [4:0] from_queue_qj;
 input [36:0] mmres;
 input allow;
 input [6:0] brbus_to_rs;
 output [`Lmmrs-1:0] mmrs_to_addr;
 output mmrsfull;

wire [4:0] issue_Qj = from_queue_qj; 
wire        valid_mmres = mmres[0];
wire [3:0]  qid_mmres   = mmres[4:1];
wire [31:0] value_mmres = mmres[36:5];
 //outputs to cp0
 wire valid_out;
 wire [7:0] op_out;    
 wire [3:0] qid_out;
 wire [2:0] brqid_out;
 wire [31:0] Vj_out;
 wire [31:0] Vk_h_out;
 wire [31:0] Vk_out;
 wire [31:0] Vk_addr_out;
 wire [31:0] Vl_out;
 wire [2:0]  fpqid_out;
 wire brbus_brerr;
 assign brbus_brerr  = brbus_to_rs[0];
 wire [5:0] brbus_brmask;
 assign brbus_brmask = brbus_to_rs[6:1];
 //----------RS registers------------
 reg mmrs_busy [`MMrsize-1:0];    
 reg [3:0]  mmrs_qid [`MMrsize-1:0];
 reg [2:0]  mmrs_brqid [`MMrsize-1:0];
 reg [7:0]  mmrs_op [`MMrsize-1:0]; 
 reg [31:0] mmrs_vj [`MMrsize-1:0];
 reg [31:0] mmrs_vk [`MMrsize-1:0];
 reg [`Lword-1:0] mmrs_vl [`MMrsize-1:0]; 
 reg [`MMrsize-1:0] mmrs_vj_rdy;
 reg [3:0]        mmrs_qj  [`MMrsize-1:0];

//----------head register----------
reg  head;

wire brcancel0, brcancel1; 
assign brcancel0 = brbus_brmask[mmrs_brqid[0]];  
assign brcancel1 = brbus_brmask[mmrs_brqid[1]];  

wire        issue_write_fpq= rissuebus[136];     
wire        issue_wb2_h    = rissuebus[135];     
wire        issue_wb2      = rissuebus[134];     
wire [31:0] issue_imm      = rissuebus[133:102];     
wire [2:0]  issue_fpqid    = rissuebus[101:99];     
wire [3:0]  issue_fpqid2   = rissuebus[98:95];     
wire [3:0]  issue_fpqid2_h = rissuebus[94:91];     
wire        issue_rdy2     = rissuebus[90];     
wire        issue_rdy2_h   = rissuebus[89];     
wire        issue_rdy      = rissuebus[88];    
wire [2:0]  issue_brqid    = rissuebus[87:85];     
wire        issue_valid    = rissuebus[84];
wire [3:0]  issue_qid      = rissuebus[83:80];     
wire [7:0]  issue_op       = rissuebus[79:72];     
wire [31:0] issue_Vj       = rissuebus[71:40];     
wire [31:0] issue_Vk       = rissuebus[39:8];     
wire [7:0]  issue_src2     = rissuebus[7:0];

wire brcancel_issue;
assign brcancel_issue = (brbus_brmask[issue_brqid]);
 
//------------------------------local variation defination--------------------------------    
wire       tail;        
wire       write_head;   

assign tail = (head==1'b0 & !mmrs_busy[0] & !mmrs_busy[1]) ? 1'b0 :
              (head==1'b0 &  mmrs_busy[0] & !mmrs_busy[1]) ? 1'b1 :
              (head==1'b1 & !mmrs_busy[0] & !mmrs_busy[1]) ? 1'b1 :
              (head==1'b1 & !mmrs_busy[0] &  mmrs_busy[1]) ? 1'b0 : 1'b0;

assign write_head = ~head; 
always @(posedge clock)
begin      
    if(reset)
    begin
        head <=1'b0;
    end
    else if (allow & valid_out)
    begin
        head <=write_head;
    end
end

//-----------------------------------------RS[0]-----------------------------------

wire myout_0;
wire issue_to_mm_0;
wire issue_can_write_0;               
wire B_enable_0;
wire set_B_0;
wire hit_on_res_0;

assign hit_on_res_0  = valid_mmres & mmrs_qj[0]==qid_mmres;

assign myout_0 = mmrs_busy[0] & (mmrs_vj_rdy[0] | hit_on_res_0);

assign issue_to_mm_0 = allow & myout_0 & (head==1'b0);
assign issue_can_write_0 = (tail==1'b0) & issue_valid & !brcancel_issue;
wire   issue_can_write_fpq_0 = (tail==1'b0) & issue_valid & !brcancel_issue & issue_write_fpq;
assign B_enable_0        = reset | commitbus_ex | issue_to_mm_0 | issue_can_write_0 | brcancel0;
assign set_B_0           = ~reset & ~commitbus_ex & issue_can_write_0;

//--------------------------------------------RS[1]---------------------------------------
wire myout_1;
wire issue_to_mm_1;
wire issue_can_write_1;               
wire B_enable_1;
wire set_B_1;
wire hit_on_res_1;

assign hit_on_res_1  = valid_mmres & mmrs_qj[1]==qid_mmres;

assign myout_1 = mmrs_busy[1] & (mmrs_vj_rdy[1] | hit_on_res_1);

assign issue_to_mm_1 = allow & myout_1 & (head==1'b1);
assign issue_can_write_1 = (tail==1'b1) & issue_valid & !brcancel_issue;
wire   issue_can_write_fpq_1 = (tail==1'b1) & issue_valid & !brcancel_issue & issue_write_fpq;
assign B_enable_1        = reset | commitbus_ex | issue_to_mm_1 | issue_can_write_1 | brcancel1;
assign set_B_1           = ~reset & ~commitbus_ex & issue_can_write_1;

//RS registers
always     @(posedge clock)
begin
    if (B_enable_0)             
        mmrs_busy[0]<=set_B_0;
    if (B_enable_1)
        mmrs_busy[1]<=set_B_1;
end                      
   
always @(posedge clock)
begin
    if (reset | commitbus_ex | mmrs_busy[0]&brcancel0 | issue_to_mm_0)
        mmrs_vj_rdy[0] <= 1'b0;
    else if (issue_can_write_0)
        mmrs_vj_rdy[0] <= issue_Qj[4] | qissuebus0_src1_rdy;
    else if (mmrs_busy[0] & ~mmrs_vj_rdy[0] & hit_on_res_0)
        mmrs_vj_rdy[0] <= 1'b1;

    if (reset | commitbus_ex | mmrs_busy[1]&brcancel1 | issue_to_mm_1)
        mmrs_vj_rdy[1] <= 1'b0;
    else if (issue_can_write_1)
        mmrs_vj_rdy[1] <= issue_Qj[4] | qissuebus0_src1_rdy;
    else if (mmrs_busy[1] & ~mmrs_vj_rdy[1] & hit_on_res_1)
        mmrs_vj_rdy[1] <= 1'b1;
end

wire iterm0_hit = mmrs_busy[0] & ~mmrs_vj_rdy[0] & hit_on_res_0 & ~issue_to_mm_0;
wire iterm1_hit = mmrs_busy[1] & ~mmrs_vj_rdy[1] & hit_on_res_1 & ~issue_to_mm_1;

always @(posedge clock)
begin
    if (issue_can_write_0 | iterm0_hit)
        mmrs_vj[0] <= (iterm0_hit) ? value_mmres : issue_Vj;

    if (issue_can_write_1 | iterm1_hit)
        mmrs_vj[1] <= (iterm1_hit) ? value_mmres : issue_Vj;
end

always     @(posedge clock)
begin
    if (issue_can_write_0)
    begin
        mmrs_op[0]       <= issue_op;
        mmrs_qid[0]      <= issue_qid;
        mmrs_brqid[0]    <= issue_brqid;
        mmrs_qj[0]       <= issue_Qj[3:0];
        mmrs_vl[0]       <= issue_imm;
        mmrs_vk[0]       <= issue_Vk;
    end

    if (issue_can_write_1)
    begin
        mmrs_op[1]       <= issue_op;
        mmrs_qid[1]      <= issue_qid;
        mmrs_brqid[1]    <= issue_brqid;
        mmrs_qj[1]       <= issue_Qj[3:0];
        mmrs_vl[1]       <= issue_imm;
        mmrs_vk[1]       <= issue_Vk;
    end   
end

//----------------output to queue--------------------------------------
assign mmrsfull = mmrs_busy[0] & mmrs_busy[1];
//----------------outputs to CP0 funtional unit------------------------
assign valid_out  = head==1'b0 ? myout_0      : myout_1;
assign op_out     = head==1'b0 ? mmrs_op[0]   : mmrs_op[1];
assign qid_out    = head==1'b0 ? mmrs_qid[0]  : mmrs_qid[1];
assign brqid_out  = head==1'b0 ? mmrs_brqid[0]: mmrs_brqid[1];

assign Vl_out     = head==1'b0 ? mmrs_vl[0]   : mmrs_vl[1];
assign Vj_out     = ((head==1'b0 & mmrs_busy[0] & ~mmrs_vj_rdy[0] & hit_on_res_0)|
                     (head==1'b1 & mmrs_busy[1] & ~mmrs_vj_rdy[1] & hit_on_res_1)) ? value_mmres : 
                                                                      (head==1'b0) ? mmrs_vj[0]  : mmrs_vj[1];
assign Vk_addr_out     = head==1'b0 ? mmrs_vk[0]   : mmrs_vk[1];

assign fpqid_out =3'b0;
assign Vk_out = (head==1'b0) ? mmrs_vk[0] : mmrs_vk[1];
assign Vk_h_out   = 32'h0;

assign mmrs_to_addr = {Vk_addr_out, fpqid_out, brqid_out, valid_out,op_out,qid_out,Vj_out,Vk_out,Vk_h_out,Vl_out};
endmodule

