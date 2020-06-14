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

module godson_alu_module_nomul(
    clock,reset,commitbus_ex,
    alurs_to_alu,
    // HAVE_DSP_UNIT
    DSPCtl_value,
    src3_to_alu,
    alures,
    alures_jalr_target,
    brbus_to_fu
);
    
input clock;
input reset;
input commitbus_ex;
input [84:0] alurs_to_alu;
input [10:0] src3_to_alu;
input [6:0] brbus_to_fu;
//HAVE_DSP_UNIT
input [31:0] DSPCtl_value;

output [121:0] alures;
output [31:0] alures_jalr_target;

//inputs from alurs
wire [2:0]  op_ac =alurs_to_alu[84:82];
wire [2:0]  brqid =alurs_to_alu[81:79];
wire        valid =alurs_to_alu[78];
wire [7:0]  op    =alurs_to_alu[77:70];
wire [1:0]  ac    =alurs_to_alu[69:68];
wire [3:0]  qid   =alurs_to_alu[67:64];
wire [31:0] vj    =alurs_to_alu[63:32];
wire [31:0] vk    =alurs_to_alu[31:0];

//HAVE_DSP_UNIT
wire write_dspctl = op==`OP_EXTPDP || op==`OP_MTHLIP || op==`OP_ADDSC || op==`OP_EXTP   ||  
                    op==`OP_ADDQ   || op==`OP_ADDQ_S || op==`OP_SUBQ  || op==`OP_SUBQ_S || op==`OP_ABSQ_S ||
                    op==`OP_PRECRQU_S_QB_PH || op==`OP_ADDWC  || op==`OP_PRECRQ_RS_PH_W || op==`OP_SHLLV  ||
                    op==`OP_SHLLVS          || op==`OP_EXTR_W || op==`OP_EXTR_R_W       || op==`OP_EXTR_RS_W ||
                    op==`OP_EXTR_S_H        || 
                    op==`OP_DPAU_H_QBL     || op==`OP_DPAQ_S_W_PH    || op==`OP_DPAU_H_QBR  || op==`OP_DPAQ_SA_L_W || 
                    op==`OP_DPSQ_S_W_PH    || op==`OP_MAQ_S_W_PHL    || op==`OP_DPSQ_SA_L_W || op==`OP_MAQ_S_W_PHR || 
                    op==`OP_MAQ_SA_W_PHL   || op==`OP_MULEU_S_PH_QBL || op==`OP_MAQ_SA_W_PHR || 
                    op==`OP_MULEU_S_PH_QBR || op==`OP_MULEQ_S_W_PHL  ||op==`OP_MULSAQ_S_W_PH || 
                    op==`OP_MULEQ_S_W_PHR  || op==`OP_MULQ_RS_PH ||
                    op==`OP_CMP_EQ || op==`OP_CMP_LT || op==`OP_CMP_LE || op==`OP_WRDSP;

wire brbus_brerr;
wire [5:0] brbus_brmask;

assign brbus_brerr  = brbus_to_fu[0];
assign brbus_brmask = brbus_to_fu[6:1];
//internal wires
wire alures_valid;    
wire [3:0] alures_qid;
wire [31:0] alures_value;
wire alures_ov;        
wire alures_trap;    
wire alures_bnt;
wire alures_con_true;

wire alu_valid;
assign alu_valid = valid ;

wire [31:0] alu_res;

reg [9:0] src3;
always @(posedge clock)
if (reset)
    src3 <= 10'b0;
else if (src3_to_alu[10])
    src3 <= src3_to_alu[9:0];

godson_alu alu_0(
    .a(vj[31:0]), .b(vk), .c(src3),.op(op[7:0]), 
    .out(alu_res),.jalr_target(alures_jalr_target),
    .ov(alures_ov), .trap(alures_trap), .bnt(alures_bnt), .condition_true(alures_con_true)) ;    

/////dsp///////////////
wire [31:0] dsp_result;       
wire [5:0] pos_out_dsp;                 
wire [5:0] scount_out;
wire carry_out;
wire efi_out_dsp;
wire [7:0] outflag_out_dsp;
wire [3:0] ccond_out;                 
wire dsp_bnt;    
wire[31:0] dspres_DSPCtl;


wire [7:0] outflag_out;
wire [5:0] pos_out;
wire       efi_out;
//HAVE_DSP_UNIT
godson_dsp dsp_alu_0 (.a(vj), .b(vk), .ac(ac),.op(op), .op_ac(op_ac),
                      .pos_in(DSPCtl_value[5:0]),.scount_in(DSPCtl_value[12:7]),.carry_in(DSPCtl_value[13]),
                      .efi_in(DSPCtl_value[14]),
                      .outflag_in(DSPCtl_value[23:16]),.ccond_in(DSPCtl_value[27:24]),
                      .result(dsp_result), .pos_out(pos_out_dsp),.scount_out(scount_out),
                      .carry_out(carry_out),.efi_out(efi_out_dsp), .outflag_out(outflag_out_dsp),
                      .ccond_out(ccond_out),.dsp_bnt(dsp_bnt));

assign outflag_out   = {8{alu_valid}} & outflag_out_dsp;
assign dspres_DSPCtl = {4'b0,ccond_out,outflag_out,1'b0,efi_out_dsp,carry_out,scount_out,1'b0,pos_out_dsp};
assign alures_value = alu_valid &~(op[7:6]==2'b11)? alu_res : dsp_result;
assign alures_valid = alu_valid; 
assign alures_qid   = qid;

wire block_end;
BLOCK_END block_end_0 (.op(op), .end_op(block_end));

assign alures[121] = write_dspctl;
assign alures[120] = block_end;
assign alures[119:88] = dspres_DSPCtl;
//HAVE_DSP_UNIT
assign alures[86]    = alu_valid && (alures_bnt || dsp_bnt);
assign alures[87]    = alures_con_true &alu_valid;
assign alures[85]    = alures_valid;
assign alures[84:81] = alures_qid;
assign alures[80:17] = {32'b0, alures_value};
assign alures[16]    = alu_valid && alures_ov;
assign alures[15]    = alu_valid && alures_trap;
assign alures[14]    = 1'b0;
assign alures[13]    = 1'b0;
assign alures[12]    = 1'b0;
assign alures[11]    = 1'b0;
assign alures[10]    = 1'b0;
assign alures[9]     = 1'b0;
assign alures[8]     = 1'b0;
assign alures[7]     = 1'b0;
assign alures[6]     = 1'b0;
assign alures[5:0]   = 6'b0;
                                                                                                                     
endmodule

module godson_alu_module(clock,reset,commitbus_ex,
    alurs_to_alu,
    //HAVE_DSP_UNIT
    DSPCtl_value,
    src3_to_alu,
    alures,
    allow,
    brbus_to_fu,acc_write_ok,acc_qid
    
    ,alures_jalr_target, alu_res

    ,alu2rsfull_tmp
    );
    
input clock;
input reset;
input commitbus_ex;
input [87:0] alurs_to_alu;
input [10:0] src3_to_alu;
input [6:0] brbus_to_fu;
input  acc_write_ok;
//HAVE_DSP_UNIT
input [31:0] DSPCtl_value;

output [122:0] alures;
    
output [31:0] alu_res;
output [31:0] alures_jalr_target;

output allow;
output [3:0] acc_qid;
output alu2rsfull_tmp;
//inputs from alurs
wire [2:0] op_ac =alurs_to_alu[87:85];
wire [2:0] opreg =alurs_to_alu[84:82];
wire [2:0] brqid =alurs_to_alu[81:79];
wire        valid =alurs_to_alu[78];
wire [7:0]  op    =alurs_to_alu[77:70];
wire [1:0]  ac    =alurs_to_alu[69:68];
wire [3:0]  qid   =alurs_to_alu[67:64];
wire [31:0] vj    =alurs_to_alu[63:32];
wire [31:0] vk    =alurs_to_alu[31:0];

wire brbus_brerr;
wire [5:0] brbus_brmask;

assign brbus_brerr  = brbus_to_fu[0];
assign brbus_brmask = brbus_to_fu[6:1];
//internal wires
wire alures_valid;    
wire [3:0] alures_qid;
wire [31:0] alures_value;
wire alures_ov;        
wire alures_trap;    
wire alures_bnt;
wire alures_con_true;

wire alu_valid,mul_valid,div_valid ;
assign mul_valid = valid & ((op==`OP_MULT)||(op==`OP_MULTU)||(op==`OP_MUL)||
                   (op==`OP_MADD)||(op==`OP_MADDU)||(op==`OP_MSUB)||(op==`OP_MSUBU) ||
                   //HAVE_DSP_UNIT
                   (op==`OP_MULEU_S_PH_QBL) || (op == `OP_MULEU_S_PH_QBR)|| (op == `OP_MULQ_RS_PH)||
                   (op==`OP_MULEQ_S_W_PHL) || (op == `OP_MULEQ_S_W_PHR)|| (op == `OP_MULSAQ_S_W_PH)||
                   (op==`OP_MAQ_S_W_PHL) || (op == `OP_MAQ_S_W_PHR)|| (op == `OP_MAQ_SA_W_PHL)||
                   (op==`OP_MAQ_SA_W_PHR) || (op == `OP_DPAU_H_QBL)|| (op == `OP_DPAU_H_QBR)||
                   (op==`OP_DPAQ_S_W_PH) || (op == `OP_DPAQ_SA_L_W)|| (op == `OP_DPSU_H_QBL)||
                   (op==`OP_DPSU_H_QBR) || (op == `OP_DPSQ_S_W_PH)|| (op == `OP_DPSQ_SA_L_W) ||
                   (op==`OP_SHILO) || (op==`OP_MTHLIP)|| 
                   (op==`OP_EXTP) || (op==`OP_EXTPDP) || (op==`OP_EXTR_W) || (op==`OP_EXTR_R_W) ||
                   (op==`OP_EXTR_RS_W) || (op==`OP_EXTR_S_H)||
                   (op==`OP_MFHI) || (op==`OP_MFLO) || (op==`OP_MTHI) || (op==`OP_MTLO)); 

assign div_valid = valid & ((op==`OP_DIV)||(op==`OP_DIVU));
assign alu_valid = valid & !mul_valid & !div_valid;

wire [3:0]acc_qid_mul, acc_qid_div; 
wire mul_op_valid;
assign acc_qid = mul_op_valid ? acc_qid_mul : acc_qid_div;

reg [9:0] src3;
always @(posedge clock)
if (reset)
    src3 <= 10'b0;
else if (src3_to_alu[10])
    src3 <= src3_to_alu[9:0];

wire [31:0]alu_res_tmp;
godson_alu alu_1(
    .a(vj[31:0]), .b(vk), .c(src3),.op(op[7:0]), 
    .out(alu_res_tmp), .jalr_target(alures_jalr_target), 
    .ov(alures_ov), .trap(alures_trap), .bnt(alures_bnt), .condition_true(alures_con_true)) ;    
/////dsp///////////////
wire [31:0] dsp_result;       
wire [5:0] pos_out_dsp;                 
wire [5:0] pos_out;                 
wire [5:0] scount_out;
wire carry_out;
wire efi_out_dsp;
wire efi_out;
wire [7:0] outflag_out_dsp;
wire [3:0] ccond_out;                 
wire dsp_bnt;    
wire[31:0] dspres_DSPCtl;


wire [7:0] outflag_out;
//HAVE_DSP_UNIT
godson_dsp dsp_alu_1 (.a(vj), .b(vk), .ac(ac),.op(op), .op_ac(op_ac),
                      .pos_in(DSPCtl_value[5:0]),.scount_in(DSPCtl_value[12:7]),.carry_in(DSPCtl_value[13]),
                      .efi_in(DSPCtl_value[14]),
                      .outflag_in(DSPCtl_value[23:16]),.ccond_in(DSPCtl_value[27:24]),
                      .result(dsp_result), .pos_out(pos_out_dsp),.scount_out(scount_out),
                      .carry_out(carry_out),.efi_out(efi_out_dsp), .outflag_out(outflag_out_dsp),
                      .ccond_out(ccond_out),.dsp_bnt(dsp_bnt));

/***************mul*************/
wire [63:0] mul_res;
wire [7:0]  mulres_op;

wire [7:0] outflag_out_mul;
wire [5:0] pos_out_mul;
wire       efi_out_mul;

wire div_allow;
wire [66:0] div_res;
wire [55:0] div_res_h;
//HAVE_DSP_UNIT
wire op_in = ((op==`OP_DPAQ_SA_L_W) | (op==`OP_DPSQ_SA_L_W)) ? 1'b0 : op[0]; 

wire valid_p1, validreg0, not_write_acc_op;

godson_mul mul_0( .clock(clock), .reset(reset), .commitbus_ex(commitbus_ex),.opreg(opreg),
        .vj(vj[31:0]), .vk(vk), .ac(ac), .qid(qid), .op(op_in), .mulin_op(op), .valid(mul_valid),
        .mulres(mul_res), .mulres_op(mulres_op),
        .mulbusrep_h(!alu_valid), .mulbusrep(!alu_valid), 
        .brbus_brerr(brbus_brerr), .brbus_brmask(brbus_brmask), .acc_write_ok(acc_write_ok), .acc_qid(acc_qid_mul),
        .divres(div_res), .divres_h(div_res_h),
        //HAVE_DSP_UNIT
        .pos_in(DSPCtl_value[5:0]), .efi_in(DSPCtl_value[14]),
        .pos_out(pos_out_mul), .efi_out(efi_out_mul), .outflag_in(DSPCtl_value[23:16]), .outflag_out(outflag_out_mul),
        .mul_op_valid(mul_op_valid),.brqid(brqid),
        .valid_p1_tmp(valid_p1), .validreg0_tmp(validreg0), .not_write_acc_op(not_write_acc_op));
                                                                                                                  
godson_div div_1(.clock(clock), .reset(reset), .commitbus_ex(commitbus_ex), .vj(vj[31:0]),
        .vk(vk), .qid(qid), .op(op), .valid(div_valid),
        .divres_h(div_res_h), .divres(div_res), .divbusrep_h(!(alu_valid|mul_res[54])), .divbusrep(!(alu_valid|mul_res[54])),
        .allow(div_allow), .brbus_brerr(brbus_brerr), .brbus_brmask(brbus_brmask),.brqid(brqid),.acc_write_ok(acc_write_ok), .acc_qid(acc_qid_div));
//HAVE_DSP_UNIT
assign outflag_out   = alu_valid ? outflag_out_dsp : outflag_out_mul;
assign pos_out   = alu_valid ? pos_out_dsp : pos_out_mul;
assign efi_out   = alu_valid ? efi_out_dsp : efi_out_mul;
assign dspres_DSPCtl = {4'b0,ccond_out,outflag_out,1'b0,efi_out,carry_out,scount_out,1'b0,pos_out};
assign alures_value = alu_valid &~(op[7:6]==2'b11)? alu_res_tmp: 
                      alu_valid                   ? dsp_result: mul_res[48:17];
assign alu_res = (op[7:6]==2'b00) ? alu_res_tmp : dsp_result;

assign alures_valid = alu_valid | mul_res[54] & not_write_acc_op | (mul_res[54] | div_res[54]) & acc_write_ok; 
assign alures_qid   = alu_valid? qid : mul_res[55]? mul_res[53:50] :  div_res[53:50];

wire [7:0] alures_op = alu_valid? op : mul_res[55]? mul_res[63:56] :  div_res[63:56];
// HAVE_DSP_UNIT
wire dsp_acc_op = (alures_op==`OP_MAQ_SA_W_PHL) ||
                  (alures_op==`OP_MAQ_SA_W_PHR) ||(alures_op==`OP_MAQ_S_W_PHL)||(alures_op==`OP_MAQ_S_W_PHR)|| 
                  (alures_op==`OP_MULSAQ_S_W_PH)||(alures_op==`OP_DPAU_H_QBL) ||(alures_op==`OP_DPAU_H_QBR) ||
                  (alures_op==`OP_DPAQ_S_W_PH)  ||(alures_op==`OP_DPAQ_SA_L_W)||(alures_op==`OP_DPSU_H_QBL) || 
                  (alures_op==`OP_DPSU_H_QBR)   ||(alures_op==`OP_DPSQ_S_W_PH)||(alures_op==`OP_DPSQ_SA_L_W)||
                  (alures_op==`OP_SHILO)        ||(alures_op==`OP_MTHLIP);

wire fix_acc_op =(alures_op==`OP_MULT)||(alures_op==`OP_MULTU)||(alures_op==`OP_MADD)||(alures_op==`OP_MADDU)||
                 (alures_op==`OP_MSUB)||(alures_op==`OP_MSUBU)||(alures_op==`OP_DIV) ||(alures_op==`OP_DIVU) || 
                 (alures_op==`OP_MTHI)||(alures_op==`OP_MTLO); 

wire alures_acc_op = dsp_acc_op || fix_acc_op;
wire block_end;
BLOCK_END block_end_0 (.op(alures_op), .end_op(block_end));

// HAVE_DSP_UNIT
wire write_dspctl = alures_op==`OP_EXTPDP || alures_op==`OP_MTHLIP || alures_op==`OP_ADDSC  || alures_op==`OP_EXTP ||
                    alures_op==`OP_EXTPDP || alures_op==`OP_ADDQ   || alures_op==`OP_ADDQ_S || alures_op==`OP_SUBQ ||
                    alures_op==`OP_SUBQ_S || alures_op==`OP_ABSQ_S || alures_op==`OP_PRECRQU_S_QB_PH               || 
                    alures_op==`OP_ADDWC         || alures_op==`OP_PRECRQ_RS_PH_W || alures_op==`OP_SHLLV          ||
                    alures_op==`OP_SHLLVS        || alures_op==`OP_EXTR_W         || alures_op==`OP_EXTR_R_W       ||
                    alures_op==`OP_EXTR_RS_W     || alures_op==`OP_EXTR_S_H       || alures_op==`OP_DPAU_H_QBL     || 
                    alures_op==`OP_DPAQ_S_W_PH   || alures_op==`OP_DPAU_H_QBR     || alures_op==`OP_DPAQ_SA_L_W    || 
                    alures_op==`OP_DPSQ_S_W_PH   || alures_op==`OP_MAQ_S_W_PHL    || alures_op==`OP_DPSQ_SA_L_W    || 
                    alures_op==`OP_MAQ_S_W_PHR   || alures_op==`OP_MAQ_SA_W_PHL   || alures_op==`OP_MULEU_S_PH_QBL || 
                    alures_op==`OP_MAQ_SA_W_PHR  || alures_op==`OP_MULEU_S_PH_QBR || alures_op==`OP_MULEQ_S_W_PHL  ||
                    alures_op==`OP_MULSAQ_S_W_PH || alures_op==`OP_MULEQ_S_W_PHR  || alures_op==`OP_WRDSP          ||
                    alures_op==`OP_MULQ_RS_PH    || alures_op==`OP_CMP_EQ         || 
                    alures_op==`OP_CMP_LT        || alures_op==`OP_CMP_LE          ;

assign alures[122] = write_dspctl;
assign alures[121] = block_end;
assign alures[120] = alures_acc_op;
assign alures[119:88] = dspres_DSPCtl;

assign alures[16]    = alu_valid && alures_ov;
assign alures[15]    = alu_valid && alures_trap;
assign alures[87]    = alures_con_true &alu_valid;


//HAVE_DSP_UNIT
assign alures[86]    = alu_valid && (alures_bnt ||dsp_bnt);

assign alures[85]    = alures_valid;
assign alures[84:81] = alures_qid;
assign alures[80:17] = {32'b0,alures_value};
assign alures[14]    = 1'b0;
assign alures[13]    = 1'b0;
assign alures[12]    = 1'b0;
assign alures[11]    = 1'b0;
assign alures[10]    = 1'b0;
assign alures[9]     = 1'b0;
assign alures[8]     = 1'b0;
assign alures[7]     = 1'b0;
assign alures[6]     = 1'b0;
assign alures[5:0]   = 6'b0;

assign allow = ~mul_valid | ~valid_p1 | ~validreg0 | not_write_acc_op | acc_write_ok;
assign alu2rsfull_tmp = mul_valid && valid_p1 & validreg0 & ~not_write_acc_op /*& ~acc_write_ok*/;//~allow  

endmodule

//HAVE_DSP_UNIT

module godson_dsp(a, b, ac, op, op_ac, pos_in, scount_in, carry_in, 
                  efi_in, outflag_in, ccond_in,
                  
                  result, pos_out, scount_out, carry_out, efi_out, outflag_out, ccond_out, dsp_bnt);
input [31:0] a;
input [31:0] b;
input [1:0] ac;
input [7:0] op;                 
input [2:0] op_ac;                 
input [5:0] pos_in;                 
input [5:0] scount_in;                 
input carry_in;                 
input efi_in;                 
input [7:0]outflag_in;                 
input [3:0]ccond_in;                 

//output [64:0] result;       
output [31:0] result;       
output [5:0] pos_out;                 
output [5:0] scount_out;
output carry_out;
output efi_out;
output [7:0] outflag_out;
output [3:0] ccond_out;                 
output dsp_bnt;                 

/*************first kind: sub class*************************/
assign dsp_bnt = (op==`OP_BPOSGE32) & (~pos_in[5]);
//Byte operate
wire sub_op;
wire op_of_qb = (op_ac[1:0]==2'b01);                 

assign sub_op = (op_ac[2]==1'b1); 

wire [8:0]bv1_0, bv1_1, bv1_2, bv1_3;
wire [8:0]bv2_0, bv2_1, bv2_2, bv2_3;
assign bv1_0 = op_of_qb ? {1'b0, a[7:0]} : 9'b0;
assign bv1_1 = op_of_qb ? {1'b0, a[15:8]} : 9'b0;
assign bv1_2 = op_of_qb ? {1'b0, a[23:16]} : 9'b0;
assign bv1_3 = op_of_qb ? {1'b0, a[31:24]} : 9'b0;
assign bv2_0 = op_of_qb ? (sub_op ? ~{1'b0, b[7:0]} : {1'b0, b[7:0]}) : 9'b0;
assign bv2_1 = op_of_qb ? (sub_op ? ~{1'b0, b[15:8]} : {1'b0, b[15:8]}) : 9'b0;
assign bv2_2 = op_of_qb ? (sub_op ? ~{1'b0, b[23:16]} : {1'b0, b[23:16]}) : 9'b0;
assign bv2_3 = op_of_qb ? (sub_op ? ~{1'b0, b[31:24]} : {1'b0, b[31:24]}) : 9'b0;
wire bcin = sub_op;

//SUBU.QB, SUBU_S.QB, ADDU.QB, ADDU_S.QB 
wire [8:0] bsum_0, bsum_1, bsum_2, bsum_3;
assign bsum_0 = bv1_0 + bv2_0 + bcin;
assign bsum_1 = bv1_1 + bv2_1 + bcin;
assign bsum_2 = bv1_2 + bv2_2 + bcin;
assign bsum_3 = bv1_3 + bv2_3 + bcin;

//RADDU.W.QB
//wire [9:0]braddu_temp; //nomatch
wire [10:0]braddu_temp; 
assign braddu_temp = {2'b0, bv1_0} + {2'b0, bv1_1} + {2'b0, bv1_2} + {2'b0, bv1_3}; 

wire [3:0] eq_4;
assign eq_4[0] = (a[ 7: 0]==b[ 7: 0]);
assign eq_4[1] = (a[15: 8]==b[15: 8]);
assign eq_4[2] = (a[23:16]==b[23:16]);
assign eq_4[3] = (a[31:24]==b[31:24]);

wire [3:0] blt;
assign blt[0] =  bsum_0[8];
assign blt[1] =  bsum_1[8];
assign blt[2] =  bsum_2[8]; 
assign blt[3] =  bsum_3[8];

//PICK.QB
wire [7:0]pick_0, pick_1, pick_2, pick_3;
assign pick_3 = ccond_in[3] ? bv1_3[7:0] : bv2_3[7:0];
assign pick_2 = ccond_in[2] ? bv1_2[7:0] : bv2_2[7:0];
assign pick_1 = ccond_in[1] ? bv1_1[7:0] : bv2_1[7:0];
assign pick_0 = ccond_in[0] ? bv1_0[7:0] : bv2_0[7:0];

wire [7:0]boutflag;
assign boutflag = (op==`OP_ADDQ)&(ac==2'b00) ? 
                  {3'b0,((bsum_3[8]|bsum_2[8]|bsum_1[8]|bsum_0[8])?1'b1:1'b0),4'b0} : 
                  (op==`OP_ADDQ_S)&(ac==2'b00) ? 
                  {3'b0,((bsum_3[8]|bsum_2[8]|bsum_1[8]|bsum_0[8])?1'b1:1'b0),4'b0} :
                  (op==`OP_SUBQ)&(ac==2'b00) ? 
                  {3'b0, ((bsum_3[8]|bsum_2[8]|bsum_1[8]|bsum_0[8])?1'b1:1'b0),4'b0} :
                  (op==`OP_SUBQ_S)&(ac==2'b00) ? 
                  {3'b0,((bsum_3[8]|bsum_2[8]|bsum_1[8]|bsum_0[8])?1'b1:1'b0),4'b0}:8'b0;
            
reg [31:0]bresult;
always @(sub_op or bsum_0 or bsum_1 or bsum_2 or bsum_3 or a or b or blt or eq_4 or op 
         or braddu_temp or pick_0 or pick_1 or pick_2 or pick_3) begin
     case(op) // synopsys full_case parallel_case
      
      /*ADDU.QB*/ 
      `OP_ADDQ :   
           bresult = {bsum_3[7:0],bsum_2[7:0],bsum_1[7:0],bsum_0[7:0]};
       
       /*ADDU_S.QB*/
       `OP_ADDQ_S :  
           bresult = {bsum_3[8] ? 8'hff : bsum_3[7:0], bsum_2[8] ? 8'hff : bsum_2[7:0],
                      bsum_1[8] ? 8'hff : bsum_1[7:0], bsum_0[8] ? 8'hff : bsum_0[7:0]};
       
       `OP_SUBQ : 
       /*SUBU.QB*/
           bresult = {bsum_3[7:0],bsum_2[7:0],bsum_1[7:0],bsum_0[7:0]};
       
       `OP_SUBQ_S :
       /*SUBU_S.QB*/
           bresult = {bsum_3[8] ? 8'h00 : bsum_3[7:0],
                      bsum_2[8] ? 8'h00 : bsum_2[7:0],
                      bsum_1[8] ? 8'h00 : bsum_1[7:0],
                      bsum_0[8] ? 8'h00 : bsum_0[7:0]};
       
       `OP_RADDU :   
           //bresult = {22'b0, braddu_temp}; nomatch 
           bresult = {21'b0, braddu_temp};  
       
       `OP_CMP_EQ :
       /*CMPGU.EQ.QB , CMPU.EQ.QB*/
           bresult = {28'h0, eq_4[3], eq_4[2],eq_4[1],eq_4[0]};      
       
       `OP_CMP_LT :
       /*CMPGU.LT.QB , CMPU.LT.QB*/
           bresult = {28'h0, blt[3], blt[2], blt[1], blt[0]};      
      
      //8'hde :
       `OP_CMP_LE :
       /*CMPGU.LE.QB , CMPU.LE.QB*/
           bresult = {28'h0, (blt[3]|eq_4[3]), (blt[2]|eq_4[2]), (blt[1]|eq_4[1]), (blt[0]|eq_4[0])};      
        
        `OP_PICK : 
            bresult = {pick_3, pick_2, pick_1, pick_0};
        
        `OP_PRECEQU_PH_QBL :
        /*PRECEQU_PH_QBL*/
           bresult = {{1'b0, a[31:24], 7'b0}, {1'b0, a[23:16], 7'b0}};
        
        `OP_PRECEQU_PH_QBR : 
         /*PRECEQU_PH_QBR*/
           bresult = {{1'b0, a[15:8], 7'b0}, {1'b0, a[7:0], 7'b0}};
        
        `OP_PRECEQU_PH_QBLA : 
         /*PRECEQU_PH_QBLA*/ 
           bresult = {{1'b0, a[31:24], 7'b0}, {1'b0, a[15:8], 7'b0}};
        
        `OP_PRECEQU_PH_QBRA : 
         /*PRECEQU_PH_QBRA*/ 
           bresult = {{1'b0, a[23:16], 7'b0}, {1'b0, a[7:0], 7'b0}};
        
        `OP_PRECEU_PH_QBL : 
         /*PRECEU_PH_QBL*/ 
           bresult = {{8'b0, a[31:24]}, {8'b0, a[23:16]}};
        
        `OP_PRECEU_PH_QBR : 
         /*PRECEU_PH_QBR*/ 
           bresult = {{8'b0, a[15:8]}, {8'b0, a[7:0]}};
        
        `OP_PRECEU_PH_QBLA : 
         /*PRECEU_PH_QBLA*/ 
           bresult = {{8'b0, a[31:24]}, {8'b0, a[15:8]}};
        
        `OP_PRECEU_PH_QBRA : 
         /*PRECEU_PH_QBRA*/ 
           bresult = {{8'b0, a[23:16]}, {8'b0, a[7:0]}};
        
       // `OP_REPL :  
         /*REPL(V).QB*/ 
       default :
           bresult = {a[7:0], a[7:0], a[7:0], a[7:0]};
   endcase
end //end always

/*********************halfword operate*****************************/
wire op_of_ph = (op_ac[1:0]==2'b10);
wire [16:0] hv1_0, hv1_1, hv2_0, hv2_1;
assign hv1_0 = op_of_ph ? {a[15], a[15:0]} : 16'b0;
assign hv1_1 = op_of_ph ? {a[31], a[31:16]} : 16'b0;
assign hv2_0 = op_of_ph ? (sub_op ? ~{b[15], b[15:0]} : {b[15], b[15:0]}) : 16'b0;
assign hv2_1 = op_of_ph ? (sub_op ? ~{b[31], b[31:16]} : {b[31], b[31:16]}) : 16'b0;

wire hcin;
wire [16:0] hsum_0, hsum_1;
assign hcin = sub_op;
assign hsum_0 = hv1_0 + hv2_0 + hcin;
assign hsum_1 = hv1_1 + hv2_1 + hcin;

wire [1:0]hlt;
assign hlt[0] = hsum_0[16];
assign hlt[1] = hsum_1[16];

/********for PRECRQU_S.QB.PH*************/
wire [16:0]hsum0, hsum1, hsum2, hsum3;
assign hsum0 = (op==`OP_PRECRQU_S_QB_PH) ? ({1'b0, a[31:16]} + (~{1'b0, 16'h7f80}) + 1'b1) : 17'b0;
assign hsum1 = (op==`OP_PRECRQU_S_QB_PH) ? ({1'b0, a[15:0]}  + (~{1'b0, 16'h7f80}) + 1'b1) : 17'b0;
assign hsum2 = (op==`OP_PRECRQU_S_QB_PH) ? ({1'b0, b[31:16]} + (~{1'b0, 16'h7f80}) + 1'b1) : 17'b0;
assign hsum3 = (op==`OP_PRECRQU_S_QB_PH) ? ({1'b0, b[15:0]}  + (~{1'b0, 16'h7f80}) + 1'b1) : 17'b0;
wire [3:0]heq_4;
assign heq_4[3] = (a[31:16]==16'h7f80);
assign heq_4[2] = (a[15:0] ==16'h7f80);
assign heq_4[1] = (b[31:16]==16'h7f80);
assign heq_4[0] = (b[15:0] ==16'h7f80);
wire hcond1, hcond2, hcond3, hcond4, hcond5, hcond6, hcond7, hcond8;
assign hcond1 = a[31];
assign hcond2 = a[15];
assign hcond3 = b[31];
assign hcond4 = b[15];
assign hcond5 = !a[31] & !hsum0[16] & !heq_4[3];
assign hcond6 = !a[15] & !hsum1[16] & !heq_4[2];
assign hcond7 = !b[31] & !hsum2[16] & !heq_4[1];
assign hcond8 = !b[15] & !hsum3[16] & !heq_4[0];

wire [7:0]houtflag;
assign houtflag = (op==`OP_ADDQ)&(ac==2'b01) ? 
                  {3'b0,(((hsum_0[16]!=hsum_0[15])|(hsum_1[16]!=hsum_1[15]))?1'b1:1'b0),4'b0} :
                  (op==`OP_ADDQ_S)&(ac==2'b01) ? 
                  {3'b0,(((hsum_0[16]!=hsum_0[15])|(hsum_1[16]!=hsum_1[15]))?1'b1:1'b0),4'b0} :
                  (op==`OP_SUBQ)&(ac==2'b01) ? 
                  {3'b0,(((hsum_0[16]!=hsum_0[15])|(hsum_1[16]!=hsum_1[15]))?1'b1:1'b0),4'b0} : 
                  (op==`OP_ABSQ_S)&(ac==2'b01) ? 
                  {3'b0,(((a[31:16]==16'h8000)|(a[15:0]==16'h8000))?1'b1:1'b0),4'b0} :
                  (op==`OP_SUBQ_S)&(ac==2'b01) ? 
                  {3'b0,(((hsum_0[16]!=hsum_0[15])|(hsum_1[16]!=hsum_1[15]))?1'b1:1'b0),4'b0} :
                  (op==`OP_PRECRQU_S_QB_PH)&(ac==2'b01) ? 
                  {1'b0,((hcond1|hcond2|hcond3|hcond4|hcond5|hcond6|hcond7|hcond8)? 
                   1'b1:1'b0),6'b0}:8'b0;

reg [31:0]hresult;
always @(sub_op or hsum_0 or hsum_1 or a or b or hlt or eq_4 or op or ccond_in or hcond1 
         or hcond2 or hcond3 or hcond4 or hcond5 or hcond6 or hcond7 or hcond8) begin
     case(op) // synopsys full_case parallel_case
       `OP_ADDQ :
       /*ADDQ.PH*/ 
           hresult = {hsum_1[15:0],hsum_0[15:0]}; 
       
       `OP_ADDQ_S :
       /*ADDQ_S.PH*/ 
           hresult = {((hsum_1[16]!=hsum_1[15]) & (!hsum_1[16])) ? 16'h7fff : 
                      ((hsum_1[16]!=hsum_1[15]) & hsum_1[16])    ? 16'h8000 : hsum_1[15:0], 
                      ((hsum_0[16]!=hsum_0[15]) & (!hsum_0[16])) ? 16'h7fff : 
                      ((hsum_0[16]!=hsum_0[15]) & hsum_0[16])    ? 16'h8000 : hsum_0[15:0]}; 
       
       `OP_SUBQ :
       /*SUBQ.PH*/ 
           hresult = {hsum_1[15:0],hsum_0[15:0]}; 
       
       `OP_SUBQ_S :
       /*SUBQ_S.PH*/ 
           hresult = {((hsum_1[16]!=hsum_1[15]) & (!hsum_1[16])) ? 16'h7fff : 
                      ((hsum_1[16]!=hsum_1[15]) & hsum_1[16])    ? 16'h8000 : hsum_1[15:0], 
                      ((hsum_0[16]!=hsum_0[15]) & (!hsum_0[16])) ? 16'h7fff : 
                      ((hsum_0[16]!=hsum_0[15]) & hsum_0[16])    ? 16'h8000 : hsum_0[15:0]}; 
     
       /*CMP.EQ.PH*/
       `OP_CMP_EQ :
           hresult = {30'b0, (eq_4[3]&eq_4[2]), (eq_4[1]&eq_4[0])}; 
          
       /*CMP.LT.PH*/
       `OP_CMP_LT :
           hresult = {30'b0, hlt[1], hlt[0]}; 
       
       /*CMP.LE.PH*/
       `OP_CMP_LE :
           hresult = {30'b0, (hlt[1]|eq_4[3]&eq_4[2]), (hlt[0]|eq_4[1]&eq_4[0])}; 
       
       /*PICK.PH*/
       `OP_PICK :
           hresult = {ccond_in[1] ? a[31:16] : b[31:16], ccond_in[0] ? a[15:0] : b[15:0]}; 
      
      /*PACKRL.PH*/ 
       `OP_PACKRL_PH :
           hresult = {a[15:0], b[31:16]};
      
      /*ABSQ_S.PH*/
       `OP_ABSQ_S :
           hresult = {(a[31:16]==16'h8000) ? 16'h7fff : a[31] ? (~a[31:16] + 1'b1) : a[31:16],
                      (a[15:0] ==16'h8000) ? 16'h7fff : a[15] ? (~a[15:0]  + 1'b1) : a[15:0] }; 
     
     /*PRECRQ.QB.PH*/
       `OP_PRECRQ_QB_PH :
           hresult = {a[31:24], a[15:8], b[31:24], b[15:8]};
      
      /*PRECRQU_S.QB.PH*/
       `OP_PRECRQU_S_QB_PH :
           hresult = {hcond1 ? 8'h0 : hcond5 ? 8'hff : a[30:23],
                      hcond2 ? 8'h0 : hcond6 ? 8'hff : a[14:7],
                      hcond3 ? 8'h0 : hcond7 ? 8'hff : b[30:23],
                      hcond4 ? 8'h0 : hcond8 ? 8'hff : b[14:7]};
  
      /*PRECEQ.W.PHL*/
      `OP_PRECEQ_W_PHL : 
           hresult = {a[31:16], 16'h0};
      
      /*PRECEQ.W.PHR*/
       `OP_PRECEQ_W_PHR :
           hresult = {a[15:0], 16'h0};
      
      /*REPL(V).PH*/
      //`OP_REPL :
        default :   hresult = {a[15:0], a[15:0]};
   endcase
end  //end always

/****************************word operate*****************************************/
wire op_of_w = (op_ac[1:0] == 2'b11);
wire [32:0] wv1, wv2;
assign wv1 = ((op==`OP_ADDQ_S)|(op==`OP_SUBQ_S)|(op==`OP_ADDWC)) ? {a[31], a[31:0]} : {1'b0, a[31:0]};

assign wv2 = sub_op ? ~{b[31], b} : ((op==`OP_ADDQ_S)|(op==`OP_ADDWC)) ? {b[31], b} : 
                        (op==`OP_MODSUB) ? (~{25'b0, b[7:0]}) : {1'b0, b};
wire wcin;
assign wcin = sub_op | ((op==`OP_ADDWC) & carry_in) | (op==`OP_MODSUB);

wire [32:0] wsum;
assign wsum = op_of_w ? (wv1 +wv2 + wcin) : 33'b0;

/*****************for PRECRQ_RS.PH.W*********************/
wire [32:0] wsum1, wsum2;
assign wsum1 = (op==`OP_PRECRQ_RS_PH_W) ? ({a[31], a[31:0]} + 16'h8000) : 33'b0;
assign wsum2 = (op==`OP_PRECRQ_RS_PH_W) ? ({b[31], b[31:0]} + 16'h8000) : 33'b0;
/*****************for INSV********************************/
wire [31:0] temp = (op==`OP_INSV) ? (({32{1'b1}} << scount_in) ^ {32{1'b1}}) : 32'b0;
wire [31:0] a_0 = a & temp;
wire [31:0] temp_1 = temp << pos_in;
wire [31:0] a_1 = a_0 << pos_in; 
wire [31:0] b_1 = b & ~temp_1;
wire [31:0] res1 = (scount_in == 6'b0) ? 32'b0 : (b_1 | a_1);

wire [7:0]woutflag;
assign woutflag = (op==`OP_ADDQ_S) ? {3'b0,((wsum[32]!=wsum[31])?1'b1:1'b0),4'b0} :
                  (op==`OP_SUBQ_S) ? {3'b0,((wsum[32]!=wsum[31])?1'b1:1'b0),4'b0} : 
                  (op==`OP_ADDWC)  ? {3'b0,((wsum[32]!=wsum[31])?1'b1:1'b0),4'b0} : 
                  (op==`OP_ABSQ_S) ? {3'b0,((a[31:0]==32'h80000000)?1'b1:1'b0),4'b0} : 
                  (op==`OP_PRECRQ_RS_PH_W) ? 
                  {1'b0,(((wsum1[32]!=wsum1[31])|(wsum2[32]!=wsum2[31]))?1'b1:1'b0),6'b0}:8'b0; 
                  
wire wcarry;
assign wcarry = (op==`OP_ADDSC)  & wsum[32]; 

reg [31:0]wresult;
always @ (sub_op or op or a or b or wsum or wsum1 or wsum2 or res1) begin
  case(op)
    `OP_ADDQ_S : //ADDQ_S.W 
       wresult = ((wsum[32]!=wsum[31])&!wsum[32]) ? 32'h7fffffff : 
                 ((wsum[32]!=wsum[31])&wsum[32])  ? 32'h80000000 : wsum[31:0];
    `OP_SUBQ_S : //SUBQ_S.W 
       wresult = ((wsum[32]!=wsum[31])&!wsum[32]) ? 32'h7fffffff : 
                 ((wsum[32]!=wsum[31])&wsum[32])  ? 32'h80000000 : wsum[31:0];
    `OP_ADDSC : //ADDSC  
       wresult = wsum[31:0];
    `OP_ADDWC : //ADDWC
       wresult = wsum[31:0];
    `OP_MODSUB : //MODSUB
       //wresult = (a[31:0]==32'b0) ? b[23:8] : wsum[31:0];
       wresult = (a[31:0]==32'b0) ? {16'b0,b[23:8]} : wsum[31:0];
    `OP_ABSQ_S :
       wresult = (a[31:0] == 32'h80000000) ? 32'h7fffffff : a[31] ? (~a[31:0] + 1'b1) : a[31:0];
    `OP_PRECRQ_PH_W : //PRECRQ.PH.W
       wresult = {a[31:16], b[31:16]};
    `OP_PRECRQ_RS_PH_W : //PRECRQ_RS.PH.W
       wresult = {(wsum1[32]!=wsum1[31]) ? 16'h7fff : wsum1[31:16],
                  (wsum2[32]!=wsum2[31]) ? 16'h7fff : wsum2[31:16]};
    `OP_BITREV :
       wresult = {16'h0, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8],
                         a[9],a[10], a[11], a[12], a[13], a[14],a[15]};
   // `OP_INSV :
     default :  wresult = res1;        
  endcase
end

wire [31:0]sub_result;
assign sub_result = (ac==2'b00) ? bresult : (ac==2'b01) ? hresult : wresult; 
wire [7:0] sub_outflag; 
assign sub_outflag = (ac==2'b00) ? boutflag : (ac==2'b01) ? houtflag : woutflag;
/**********************the second kind: shift*******************/
wire [31:0] shift_result;
//SHLL.QB or SHLLV.QB, SHRL.QB or SHRLV.QB
wire shift_qb = ((op==`OP_SHRAV) | (op==`OP_SHLLV)) & (ac==2'b00); 
wire [2:0] bsa;
assign bsa = shift_qb ? b[2:0] : 3'b0;
wire[7:0]ba1,ba2,ba3,ba4;
assign ba1 = a[7:0]; 
assign ba2 = a[15:8]; 
assign ba3 = a[23:16]; 
assign ba4 = a[31:24];

wire[15:0]bsa1_ll, bsa2_ll, bsa3_ll, bsa4_ll;
assign bsa1_ll = {8'b0, ba1} << bsa;
assign bsa2_ll = {8'b0, ba2} << bsa;
assign bsa3_ll = {8'b0, ba3} << bsa;
assign bsa4_ll = {8'b0, ba4} << bsa;

wire[7:0]bsa1_rl, bsa2_rl, bsa3_rl, bsa4_rl;
assign bsa1_rl = ba1 >> bsa;
assign bsa2_rl = ba2 >> bsa;
assign bsa3_rl = ba3 >> bsa;
assign bsa4_rl = ba4 >> bsa;

wire bsetcond1, bsetcond2, bsetcond3, bsetcond4;
assign bsetcond1 = |(bsa1_ll[15:8]);
assign bsetcond2 = |(bsa2_ll[15:8]);
assign bsetcond3 = |(bsa3_ll[15:8]);
assign bsetcond4 = |(bsa4_ll[15:8]);

//SHLL.PH or SHLLV.PH
wire shift_ph = ((op==`OP_SHLLV) | (op==`OP_SHLLVS) | (op==`OP_SHRAV) | (op==`OP_SHRAVR)) & (ac==2'b01);
wire [3:0] hsa;
assign hsa = shift_ph ? b[3:0] : 4'b0;
wire[15:0]ha1,ha2;
assign ha1 = a[31:16]; 
assign ha2 = a[15:0]; 
//wire[15:0]hsa1_ll, hsa2_ll;
wire[31:0]hsa1_la, hsa2_la;

assign hsa1_la = {{16{a[31]}}, ha1} << hsa;
assign hsa2_la = {{16{a[15]}}, ha2} << hsa;

wire hsetcond1, hsetcond2, hsetcond3, hsetcond4;
assign hsetcond1 = (hsa!=4'b0) & (~a[31]) & (|hsa1_la[31:15]);
assign hsetcond2 = (hsa!=4'b0) & a[31]    & ~(&hsa1_la[31:15]); 
assign hsetcond3 = (hsa!=4'b0) & (~a[15]) & (|hsa2_la[31:15]); 
assign hsetcond4 = (hsa!=4'b0) & a[15]    & ~(&hsa2_la[31:15]); 

wire [15:0]shllvs_high = (hsa==4'b0) ? a[31:16] : (~a[31] & (|hsa1_la[30:15])) ? 16'h7fff :
                         (a[31] & ~(&hsa1_la[30:15])) ? 16'h8000 : hsa1_la[15:0];   

wire [15:0]shllvs_low = (hsa==4'b0) ? a[15:0] : (~a[15] & (|hsa2_la[30:15])) ? 16'h7fff :
                        (a[15] & ~(&hsa2_la[30:15])) ? 16'h8000 : hsa2_la[15:0];   

//SHRA.PH or SHRAV.PH or SHRA_R.PH or SHRAV_R.PH
wire[15:0]hsa1_ra_tmp, hsa2_ra_tmp;
wire [3:0] hsa_r;
assign hsa_r = hsa + 4'b1111; //hsa_r=hsa-1
wire [3:0]hsa_tmp = (op==`OP_SHRAV) ? hsa : hsa_r;

sra16 sra16_0(.sign(a[31]), .in(ha1), .shift(hsa_tmp), .out(hsa1_ra_tmp));
sra16 sra16_1(.sign(a[15]), .in(ha2), .shift(hsa_tmp), .out(hsa2_ra_tmp));

wire[16:0]hsa1_ra_r, hsa2_ra_r;
assign hsa1_ra_r = (hsa==4'b0) ? {a[31:16], 1'b0} : ({a[31], hsa1_ra_tmp} + 1'b1);
assign hsa2_ra_r = (hsa==4'b0) ? {a[15:0], 1'b0} : ({a[15], hsa2_ra_tmp} + 1'b1);

/*******************SHLL_S.W or SHLLV_S.W******************/
wire shift_w = ((op==`OP_SHLLVS) | (op==`OP_SHRAVR)) & (ac==2'b10);
wire [4:0] wsa; 
assign wsa = shift_w ? b[4:0] : 5'b0;
wire [63:0]wsa_la;
assign wsa_la = {{32{a[31]}}, a[31:0]} << wsa;

wire wsetcond1, wsetcond2;
assign wsetcond1 = (wsa!=5'b0) & (~a[31]) &  (|wsa_la[63:31]); 
assign wsetcond2 = (wsa!=5'b0) & a[31]    & ~(&wsa_la[63:31]);

wire [31:0]shllvs_w = (wsa==5'b0) ? a : (~a[31] & (|wsa_la[62:31])) ? 32'h7fffffff : 
                      (a[31] & ~(&wsa_la[62:31])) ? 32'h80000000 : wsa_la[31:0]; 
/********************SHRA_R.W or SHRAV_R.W******************************/
wire [4:0] wsa_r;
assign wsa_r = wsa + 5'b11111;
wire [31:0] wsa_ra_r0;
wire [32:0] wsa_ra_r;
assign wsa_ra_r =  (wsa ==5'b0) ? {a, 1'b0} : {a[31], wsa_ra_r0} + 1'b1;

sra32 sra32_2(.sign(a[31]), .in(a), .shift(wsa_r), .out(wsa_ra_r0));


assign shift_result =(op==`OP_SHRAV) ?
                     ((ac==2'b00)?{bsa4_rl, bsa3_rl, bsa2_rl, bsa1_rl} : {hsa1_ra_tmp, hsa2_ra_tmp}) :
                     (op==`OP_SHRAVR) ? 
                     ((ac==2'b01) ? {hsa1_ra_r[16:1], hsa2_ra_r[16:1]} : wsa_ra_r[32:1]) :
                     (op==`OP_SHLLVS) ? 
                     ((ac==2'b01)?{shllvs_high, shllvs_low} : shllvs_w) :
                     /**********the last is SHLLV***************************/
       (ac==2'b00) ? {bsa4_ll[7:0],bsa3_ll[7:0],bsa2_ll[7:0],bsa1_ll[7:0]} : {hsa1_la[15:0], hsa2_la[15:0]};

wire [7:0]shift_outflag;
assign shift_outflag = (op==`OP_SHLLV)&(ac==2'b00)?
                       {1'b0,((bsetcond1|bsetcond2|bsetcond3|bsetcond4)?1'b1:1'b0),6'b0} :
                       ((op==`OP_SHLLV)|(op==`OP_SHLLVS))&(ac==2'b01) ? 
                       {1'b0,((hsetcond1|hsetcond2|hsetcond3|hsetcond4)?1'b1:1'b0),6'b0} : 
                       (op==`OP_SHLLVS)&(ac==2'b10) ? 
                       {1'b0,((wsetcond1|wsetcond2)?1'b1:1'b0),6'b0}: 8'b0;  

/***********************ac and DSPcontrol sub-class*******************************/
// WRDSP, RDDSP
wire [5:0]ac_sa_0, mask;
assign ac_sa_0 = b[5:0];
assign mask = b[5:0];
wire [5:0]ac_sa_1;
assign ac_sa_1 = (~ac_sa_0) + 1'b1;

assign pos_out = ((op==`OP_WRDSP) & mask[0]) ? a[5:0] : pos_in; 

assign scount_out = ((op==`OP_WRDSP) & mask[1]) ? a[12:7] : scount_in;

wire ac_carry;
assign ac_carry = ((op==`OP_WRDSP) & mask[2]) ? a[13] : carry_in;

wire [7:0]ac_outflag; 
assign ac_outflag = (op==`OP_WRDSP) & mask[3] ? a[23:16] : outflag_in;

assign ccond_out = ((op==`OP_WRDSP) & mask[4]) ? a[27:24] : 
                   (((op==`OP_CMP_EQ) | (op==`OP_CMP_LT)| (op==`OP_CMP_LE)) & (ac==2'b00))  ? bresult[3:0] :
                   (((op==`OP_CMP_EQ) | (op==`OP_CMP_LT)| (op==`OP_CMP_LE)) & (ac==2'b01))  ? 
                      {ccond_in[3:2],hresult[1:0]} : ccond_in;

assign efi_out = ((op==`OP_WRDSP) & mask[5]) ? a[14] : efi_in; 

wire [31:0] ac_result;

wire [3:0] ccond_tmp_r = mask[4] ? ccond_in : 4'b0;
wire [3:0] ccond_tmp_w = mask[4] ? a[27:24] : ccond_in;

wire [7:0] outflag_tmp_r = mask[3] ? outflag_in : 8'b0;
wire [7:0] outflag_tmp_w = mask[3] ? a[23:16] : outflag_in;

wire efi_tmp_r = mask[5] ? efi_in : 1'b0;
wire efi_tmp_w = mask[5] ? a[14] : efi_in;

wire carry_tmp_r = mask[2] ? carry_in : 1'b0;
wire carry_tmp_w = mask[2] ? a[13] : carry_in;

wire [5:0] scount_tmp_r = mask[1] ? scount_in : 6'b0;
wire [5:0] scount_tmp_w = mask[1] ? a[12:7] : scount_in;

wire [5:0] pos_tmp_r = mask[0] ? pos_in : 6'b0;
wire [5:0] pos_tmp_w = mask[0] ? a[5:0] : pos_in;

assign ac_result =  (op==`OP_RDDSP) ? 
                    {4'b0, ccond_tmp_r, outflag_tmp_r, 1'b0, efi_tmp_r,
                     carry_tmp_r, scount_tmp_r, 1'b0, pos_tmp_r} :
                    {4'b0, ccond_tmp_w, outflag_tmp_w, 1'b0, efi_tmp_w,  
                    carry_tmp_w, scount_tmp_w, 1'b0, pos_tmp_w};//OP_RDDSP & OP_WRDSP
/********************************************************************/

wire sub_class;
assign sub_class = (op[7:4]==4'hc)|((op[7:4]==4'hd)&(op!=`OP_SHLLV)&(op!=`OP_SHLLVS)&(op!=`OP_SHRAV)&(op!=`OP_SHRAVR))|(op==`OP_BITREV)|(op==`OP_INSV)|(op==`OP_REPL); 
wire shift_class;
assign shift_class = (op==`OP_SHLLV) | (op==`OP_SHLLVS) | (op==`OP_SHRAV) | (op==`OP_SHRAVR);
wire ac_class;
assign ac_class = (op==`OP_RDDSP) |(op==`OP_WRDSP);   

assign result = sub_class ?  sub_result : shift_class ? shift_result: ac_result;//last is ac_class  

assign outflag_out = sub_class ? sub_outflag : shift_class ? shift_outflag : ac_outflag;
 
assign carry_out = (op==`OP_ADDSC) ? wcarry : ac_carry; 

endmodule

module godson_alu(a, b,c, op, out, ov, trap, bnt, condition_true,jalr_target) ;
input [31:0] a, b;
input [9 :0] c;
input [7:0]  op;                 
output [31:0] out,jalr_target;        
output trap,ov,bnt;
output condition_true;

wire sel_slt,sel_shift,sel_cloz,sel_gate,sel_movc,sel_r2,sel_default;
wire [31:0] slt_out,shift_out,cloz_out,gate_out,movc_out,r2_out,out1;    

//-----ADD,ADDU,SUB,SUBU,LUI,TEQ,TNE,TLT,TLTU,TGE,TGEU,SLT,SLTU-----//
wire [31:0] a_tmp; 
wire [31:0] b_tmp; 
wire cin,cout;
wire alu_op = (op==`OP_TEQ)||(op==`OP_TNE)||(op==`OP_TLT)||(op==`OP_TLTU)
              ||(op==`OP_TGE)||(op==`OP_TGEU)||(op==`OP_SLT)||(op==`OP_SLTU)
              ||(op==`OP_SUB)||(op==`OP_SUBU)||(op==`OP_ADD)||(op==`OP_ADDU);    
        
assign cin =  (op==`OP_TEQ)||(op==`OP_TNE)||(op==`OP_TLT)||(op==`OP_TLTU)
              ||(op==`OP_TGE)||(op==`OP_TGEU)||(op==`OP_SLT)||(op==`OP_SLTU)
              ||(op==`OP_SUB)||(op==`OP_SUBU);

assign a_tmp = alu_op ? a : 32'b0; 

assign b_tmp = cin ? ~b : alu_op ? b : 32'b0;

assign {cout,out1} = {1'b0, a_tmp} + {1'b0, b_tmp} + cin; 
 
wire equ = (a==b);  
wire lt  = (a[31]&out1[31])|(a[31]&(~b[31]))|(out1[31]&(~b[31])); 

assign trap = (equ&(op==`OP_TEQ))   | ((!equ)&(op==`OP_TNE))   |
              (lt&(op==`OP_TLT))    | ((!cout)&(op==`OP_TLTU)) |
              ((!lt)&(op==`OP_TGE)) | (cout&(op==`OP_TGEU));
    
assign ov   = (((a[31]& b[31]&(~out1[31]))   | ((~a[31])&(~b[31])&out1[31]))&(op==`OP_ADD)) |
              (((a[31]&(~b[31])&(~out1[31])) | ((~a[31])&b[31]&out1[31]))&(op==`OP_SUB));

assign sel_slt = (op==`OP_SLT)||(op==`OP_SLTU);
assign slt_out = (op==`OP_SLTU)? {31'b0, ~cout} : {31'b0, lt};    

//--------------------SLL,SRL,SRA--------------------------//
wire [31:0] sll_out,sr_out,shift_temp;

assign sel_shift = (op==`OP_SLL)||(op==`OP_SRL)||(op==`OP_SRA);
assign sll_out   = a << b[4:0];
wire [4:0] shift_offset = sel_shift ? b[4:0] : 5'b0;

sra32 sra32_0(.sign(op[0]), .in(a), .shift(shift_offset), .out(sr_out));

assign shift_out = (sr_out & {32{op[1]}}) | (sll_out & {32{~op[1]}});

//--------------------CLO,CLZ--------------------------//
wire [31:0] cloz_in1,cloz_in2;
wire [4:0] cloz_out_t;
wire nonzero;

assign  sel_cloz  = (op==`OP_CLO) || (op==`OP_CLZ);

change_order_32 change_order(.in(a),.out(cloz_in1));

assign cloz_in2 = (op==`OP_CLO)? ~cloz_in1 : cloz_in1;

first_one_32_5 first_one(.in(cloz_in2),.out(cloz_out_t),.nonzero(nonzero));

assign cloz_out = nonzero? {27'b0,cloz_out_t}: {26'b0,6'b100000};
 
//--------------------AND,OR,NOR,XOR--------------------------//
wire [31:0] and_out,or_out,xor_out;
assign sel_gate   = (op==`OP_AND)||(op==`OP_OR)||(op==`OP_NOR)||(op==`OP_XOR);
wire [31:0] a_gate = sel_gate ? a : 32'b0;
wire [31:0] b_gate = sel_gate ? b : 32'b0;
assign  and_out   = a_gate & b_gate;
assign  or_out    = a_gate | b_gate;
assign  xor_out   = a_gate ^ b_gate;
                                                                                                                  
assign  gate_out  = (op==`OP_AND)? and_out: (op==`OP_OR)? or_out:
                    (op==`OP_NOR)? ~or_out: xor_out;

//--------------------MOVN,MOVZ--------------------------//
assign  sel_movc       = (op==`OP_MOVN)||(op==`OP_MOVZ);
wire    eqz            = (a==32'b0);
wire    condition_true = ((op==`OP_MOVN)&!eqz)||((op==`OP_MOVZ)&eqz);
assign  movc_out       = b; 

//---------------------jump and branch----------------------//
wire tag = a[31] | eqz;
wire sel_j,sel_jr,sel_beq,sel_bne,sel_blez,sel_bgtz,sel_bltz,sel_bgez;
wire sel_jreg;
wire sel_link;

assign sel_link  = (op == `OP_JAL) || (op==`OP_BLTZAL) || (op==`OP_BLTZALL) ||
                   (op==`OP_BGEZAL) || (op==`OP_BGEZALL) ||(op ==`OP_JALR);

assign sel_j    = (op==`OP_J) | (op==`OP_JAL);
assign sel_jr   = (op==`OP_JR) | (op==`OP_JALR);
assign sel_jreg   = (op==`OP_JR);
assign sel_beq  = (op==`OP_BEQ) | (op==`OP_BEQL);
assign sel_bne  = (op==`OP_BNE) | (op==`OP_BNEL);
assign sel_blez = (op==`OP_BLEZ) | (op==`OP_BLEZL);
assign sel_bgtz = (op==`OP_BGTZ) | (op==`OP_BGTZL);
assign sel_bltz = (op==`OP_BLTZ) | (op==`OP_BLTZAL) | (op==`OP_BLTZL) | (op==`OP_BLTZALL);
assign sel_bgez = (op==`OP_BGEZ) | (op==`OP_BGEZAL) | (op==`OP_BGEZL) | (op==`OP_BGEZALL);

assign bnt = ((sel_beq&~equ)|(sel_bne&equ)|(sel_blez&~tag)|(sel_bgtz&tag)
             |(sel_bltz&~a[31])|(sel_bgez&a[31]))|(~equ&sel_jr);

//--------------------Release2: EXT,INS,WSBH,ROTR,ROTRV,SEB,SEH---------------//
wire [31:0] ext_out,ins_out,wsbh_out,rotr_out,seb_out,seh_out,ext_temp,rotr_temp1, rotr_temp2;
wire ext_sel,ins_sel, wsbh_sel, rotr_sel,seb_sel, seh_sel;

assign sel_r2   = ext_sel || ins_sel || wsbh_sel || rotr_sel|| seb_sel|| seh_sel; 
assign ext_sel  = (op==`OP_EXT);
assign ins_sel  = (op==`OP_INS) ;
assign wsbh_sel = (op == `OP_WSBH);
assign rotr_sel = (op == `OP_ROTR);
assign seb_sel  = (op == `OP_SEB);
assign seh_sel  = (op == `OP_SEH);
wire op_release2 = (op==`OP_EXT) || (op==`OP_INS) || (op == `OP_WSBH) || (op == `OP_ROTR) ||
                   (op == `OP_SEB) || (op == `OP_SEH);
wire [31:0]a_release2 = op_release2 ? a : 32'b0;
wire [31:0]b_release2 = op_release2 ? b : 32'b0;
wire [9:0] c_release2 = op_release2 ? c : 10'b0;
//for EXT operation
wire[4:0]  lsb  = b_release2[4:0];
wire[4:0]  msbd = b_release2[9:5];
wire[5:0]  msb =  msbd + 1'b1;
wire [31:0] temp_0 = ({32{1'b1}} << msb) ^ {32{1'b1}};
assign ext_temp = a_release2 >>lsb;
assign ext_out = ext_temp & temp_0;  

//for INS operation
wire[4:0] lsb_ins = c_release2[4:0];
wire[4:0] msb_ins = c_release2[9:5];
wire[4:0] size_temp  = msb_ins - lsb_ins;
wire[5:0] size       = size_temp + 1'b1 ;

wire [31:0] temp = ({32{1'b1}} << size) ^ {32{1'b1}};
wire [31:0] a_0 = a_release2 & temp;
wire [31:0] temp_1 = temp << lsb_ins;
wire [31:0] a_1 = a_0 << lsb_ins; 
wire [31:0] b_1 = b_release2 & ~temp_1;
assign ins_out = b_1 | a_1;

assign  wsbh_out = {a[23:16],a[31:24],a[7:0],a[15:8]};

assign  {rotr_temp1,rotr_temp2} = {a_release2,32'b0} >> lsb;
assign  rotr_out                = rotr_temp1 | rotr_temp2;

assign  seb_out = {{24{a[7]}},a[7:0]};
assign  seh_out = {{16{a[15]}},a[15:0]};

assign  r2_out = ({32{ext_sel}} & ext_out) | 
                 ({32{ins_sel}} & ins_out) |
                 ({32{wsbh_sel}} & wsbh_out) |
                 ({32{rotr_sel}} & rotr_out) |
                 ({32{seb_sel}} & seb_out) |
                 ({32{seh_sel}} & seh_out); 
//----------------------value output--------------------------//
assign sel_default = !(sel_shift | sel_slt | sel_gate | sel_j   | sel_jr  | sel_cloz | sel_movc |
                      sel_beq  | sel_bne  | sel_blez | sel_bgtz | sel_bltz | sel_bgez | sel_r2);

assign out = ({32{sel_shift}} & shift_out) | 
             ({32{sel_cloz}}  & cloz_out) | 
             ({32{sel_slt}}   & slt_out) | 
             ({32{sel_gate}}  & gate_out) | 
             ({32{sel_movc}}  & movc_out) | 
             ({32{sel_jreg}}  & a) | 
             ({32{sel_r2}}    & r2_out) | 
             ({32{sel_link}}  & b) | 
             ({32{sel_default}} & out1);
assign jalr_target  = a;
                  
endmodule


// FULLSIZE_MUL
module godson_mul(clock, reset, commitbus_ex, vj, vk, ac, qid, op, mulin_op, valid, mulres, mulres_op, 
                  mulbusrep_h, mulbusrep, brbus_brerr, brbus_brmask, acc_write_ok, 
                  divres, divres_h, acc_qid,
                  //HAVE_DSP_UNIT
                  pos_in, efi_in, outflag_in, pos_out, efi_out, outflag_out, 
                  mul_op_valid, brqid, valid_p1_tmp, validreg0_tmp, not_write_acc_op,opreg);
input clock;
input reset;                   
input commitbus_ex;
input [31:0] vj, vk;
input [4:1] qid;
input op;
input [7:0] mulin_op;
input valid;
input mulbusrep, mulbusrep_h;
input [1:0] ac;
input brbus_brerr;
input [5:0]brbus_brmask;
input acc_write_ok;
input [66:0] divres;
input [55:0] divres_h;
input [2:0]brqid;
input [2:0]opreg;
//HAVE_DSP_UNIT
input [7:0] outflag_in;
input [5:0] pos_in;
input  efi_in;
output [7:0] outflag_out;
output [5:0] pos_out;
output  efi_out;
output [63:0] mulres;
output [7:0]  mulres_op;
output [3:0]acc_qid;
output mul_op_valid;
output valid_p1_tmp;
output validreg0_tmp;
output not_write_acc_op;

wire divres_valid = divres[54] & acc_write_ok;
wire [31:0] divres_value = divres[48:17];
wire [3:0] divres_qid = divres[53:50];
wire [2:0] divres_brqid = divres[66:64];
wire divres_h_valid = divres_h[54] & acc_write_ok;
wire [31:0] divres_h_value = divres_h[48:17];
wire out;
wire [63:0] mul_product;
wire [63:0] carry_for32, sum_for32;

reg ValidRegHi0;
reg ValidReg0;        
reg vj_xor_vk_32_reg;
reg vj_xor_vk_16_reg_0;
reg vj_xor_vk_16_reg_1;
reg [31:0]vjReg0;
reg [31:0]vkReg0;
reg [4:1]QidReg0; 
reg [2:0]brQidReg0;
reg [2:0]brQidReg1;     
reg [7:0]opReg0, outflagReg0;
reg [5:0]posReg0;
reg efiReg0;
reg [1:0]acReg0; 
reg [7:0]opreg0; 
reg [31:0]vjReg1;
reg [31:0]vkReg1;
reg [4:1]QidReg1;     
reg [7:0]opReg1, outflagReg1;
reg [1:0]acReg1; 
reg [5:0]posReg1;
reg efiReg1;
reg valid_p1;

assign valid_p1_tmp = valid_p1;
assign validreg0_tmp = ValidReg0;

assign acc_qid = QidReg1;
assign mul_op_valid = ValidReg0;

//HAVE_DSP_UNIT
reg [31:0] reg_hi0, reg_hi1, reg_hi2, reg_hi3;
reg [31:0] reg_low0, reg_low1, reg_low2, reg_low3;

/*****************src is HILO***********************************************************************/
wire [63:0]vlReg0;     
assign vlReg0 = //HAVE_DSP_UNIT
                (acReg1==2'b00) ? {reg_hi0, reg_low0} :
                (acReg1==2'b01) ? {reg_hi1, reg_low1} : 
                (acReg1==2'b10) ? {reg_hi2, reg_low2} : {reg_hi3, reg_low3}
                ;

//HAVE_DSP_UNIT
//EXTR(V).W, EXTR(V)_R.W, EXTR(V)_RS.W, EXTR(V)_S.H
wire [31:0]hi_value, lw_value;
assign hi_value = vlReg0[63:32]; 
assign lw_value = vlReg0[31:0]; 

wire [4:0]wsa_ac, wsa_r_ac;
//assign wsa_ac = vk[4:0];
assign wsa_ac = vkReg1[4:0];
assign wsa_r_ac = wsa_ac +5'b11111;//wsa_ac-1

wire [5:0]ac_sa_0, mask;
wire sign = ((opReg1==`OP_EXTR_W)|(opReg1==`OP_EXTR_R_W)|(opReg1==`OP_EXTR_RS_W)|(opReg1==`OP_EXTR_S_H)) &
              hi_value[31] ? 1'b1 : 1'b0;

wire [4:0]right_shift;
assign right_shift = ((opReg1==`OP_EXTR_W)|(opReg1==`OP_EXTR_R_W)|(opReg1==`OP_EXTR_RS_W)|(opReg1==`OP_EXTR_S_H)) ?
                      wsa_r_ac : ac_sa_0[4:0]; 

wire [31:0]extp_res;
wire [6:0]sum0;
wire [6:0]sum3;
                       
wire [63:0]res_tmp_1;
wire [63:0]res_tmp;
wire [31:0]res_r_tmp;
wire [31:0]tmp1;

wire [63:0] right_tmp;
sra64 sra64_0 (.sign(sign), .in({hi_value, lw_value}), .shift(right_shift), .out(right_tmp));
wire [32:0] right_tmp_1 = right_tmp[32:0] + 1'b1;
assign res_tmp = (wsa_ac==5'b0) ? {hi_value, lw_value} : {right_tmp[63], right_tmp[63:1]};
assign res_r_tmp = (wsa_ac==5'b0) ?lw_value : right_tmp_1[32:1];

wire ex_setcond1, ex_setcond2;
assign ex_setcond1 = ((~hi_value[31]) & (|res_tmp[63:31])) | ((~hi_value[31]) & (&right_tmp[31:0]) & (wsa_ac!=5'b0));
assign ex_setcond2 = hi_value[31] & ~(&res_tmp[63:31]);
//EXTP(V), EXTPDP(V)
assign sum0 = {1'b0, posReg1} + (~{2'b0, wsa_ac}) + 1'b1;//pos-size
assign sum3 = {1'b0, sum0[5:0]} + 7'b1111111;//pos-(size+1)
wire [5:0] s_add_1 = wsa_ac + 1'b1;
wire [31:0] temp_extp = ({32{1'b1}} << s_add_1) ^ {32{1'b1}};
wire [63:0] temp_extp_1 = {hi_value, lw_value} >> sum0[5:0];
wire [31:0] temp_extp_2 = temp_extp_1[31:0] & temp_extp;

//SHILO(V), MTHLIP
wire [5:0]ac_sa_1;
assign ac_sa_0 = vkReg1[5:0];
assign ac_sa_1 = (~ac_sa_0) + 1'b1;
wire [63:0] left_tmp;
assign left_tmp = {hi_value, lw_value} << ac_sa_1; 
assign mask = vkReg1[5:0];
wire [63:0]tmp;
assign tmp = (~ac_sa_0[5]) ? right_tmp : left_tmp;

/************************write DSPcontrol**************************/
assign pos_out = (opReg1==`OP_EXTPDP) ? (~sum0[6] & ~sum3[6] ? sum3[5:0] : (&sum3) ? 6'h3f : posReg1) :
                 (opReg1==`OP_MTHLIP) ? ((~posReg1[5]) ? (posReg1 + 6'b100000) : 6'b011111) : posReg1; 
//////////////////////////////////////////////////////////////
assign efi_out = ((opReg1==`OP_EXTP) | (opReg1==`OP_EXTPDP)) ? (~sum0[6] ? 1'b0 : 1'b1) : efiReg1; 

wire [31:0] ac_result;

wire [31:0] result_tmp1 = ex_setcond1 ? 32'h7fffffff : ex_setcond2 ? 32'h80000000 : res_r_tmp; 

wire [31:0] result_tmp2 = ~hi_value[31] & (|res_tmp[63:15]) ? 32'h00007fff :
                          hi_value[31] & ~(&res_tmp[63:15]) ? 32'hffff8000 : res_tmp[31:0];

assign ac_result = ((opReg1==`OP_EXTP)|(opReg1==`OP_EXTPDP)) ? (~sum0[6] ? temp_extp_2 : 32'b0) :
                   (opReg1==`OP_EXTR_W) ? res_tmp[31:0] :
                   (opReg1==`OP_EXTR_R_W) ?res_r_tmp :
                   (opReg1==`OP_EXTR_S_H)  ? result_tmp2 : /*(opReg1==`OP_EXTR_RS_W) ?*/ result_tmp1;
                  
wire [63:0] ac_dresult;
assign ac_dresult = (opReg1==`OP_SHILO) ? tmp[63:0] : {lw_value, vkReg1};//MTHLIP

wire ac_class;
assign ac_class = (opReg1==`OP_EXTP) | (opReg1==`OP_EXTPDP) | (opReg1==`OP_EXTR_W) | 
                  (opReg1==`OP_EXTR_R_W) |
                  (opReg1==`OP_EXTR_RS_W) | (opReg1==`OP_EXTR_S_H);   
wire ac_d_class;
assign ac_d_class = (opReg1==`OP_SHILO) | (opReg1==`OP_MTHLIP);


//added by lumin: for DSP
wire s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25;
wire o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,o15,o16,o17,o18,o19,o20,o21,o22,o23,o24,o25;
wire v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15,v16,v17,v18,v19,v20,v21,v22,v23,v24,v25;
wire [15:0] vj16_0, vk16_0;
wire [15:0] vj16_1, vk16_1;
wire [31:0] mul_for16_0;
wire [31:0] mul_for16_1;
wire [63:0] mul_for32;
wire overflow1, overflow2, overflow3, overflow4, overflow5, overflow6, overflow7, overflow;

// appply result bus
//wire mulres_valid_h = ValidReg0 & (not_write_acc_op | acc_write_ok);
//wire mulres_valid   = ValidReg0 & (not_write_acc_op | acc_write_ok); 
wire mulres_valid_h = ValidReg0;
wire mulres_valid   = ValidReg0; 

assign  not_write_acc_op = (opReg1==`OP_MUL) | (opReg1==`OP_MFLO) | (opReg1==`OP_MFHI) 
                     //HAVE_DSP_UNIT
                        |(opReg1==`OP_MULEU_S_PH_QBL) | (opReg1==`OP_MULEU_S_PH_QBR) | (opReg1==`OP_MULQ_RS_PH)
                        |(opReg1==`OP_MULEQ_S_W_PHL) | (opReg1==`OP_MULEQ_S_W_PHR) 
                        |(opReg1==`OP_EXTP) | (opReg1==`OP_EXTPDP) | (opReg1==`OP_EXTR_W) | (opReg1==`OP_EXTR_R_W) 
                        |(opReg1==`OP_EXTR_RS_W) | (opReg1==`OP_EXTR_S_H)   
                        ;

wire write_acc_h_op = ~not_write_acc_op & (opReg1!=`OP_MTLO) & (opReg1!=`OP_MTHI);

wire allow_tmp_0 = mulbusrep & acc_write_ok;        
wire allow_tmp_1 = ~valid_p1 | ~ValidReg0 | mulbusrep & not_write_acc_op;        
wire allow_tmp_2 = ~ValidReg0 | mulbusrep & not_write_acc_op;        
wire allow =  allow_tmp_1 | acc_write_ok;        
//wire allow_in_p2 = ~ValidReg0 | mulbusrep & (not_write_acc_op | acc_write_ok);  
wire allow_in_p2 =  allow_tmp_2 | allow_tmp_0;  
wire init  = reset | commitbus_ex;
wire brcancel_in;
assign brcancel_in = brbus_brerr & (brbus_brmask[brqid]);
wire brcancel_p1;
assign brcancel_p1 = brbus_brerr & (brbus_brmask[brQidReg0]);
wire brcancel_out;
assign brcancel_out = brbus_brerr & (brbus_brmask[brQidReg1]);

wire vj_xor_vk_16_0;
wire vj_xor_vk_16_1;
wire [31:0]mul_for16_tmp_0;
wire [31:0]mul_for16_tmp_1;
wire [31:0] temp_for16_0;
wire [31:0] temp_for16_1;

reg [63:0] sumReg_for32, carryReg_for32;
always @(posedge clock) begin                      
  if (valid&!brcancel_in&allow) begin
    QidReg0  <= qid[4:1];
    brQidReg0  <= brqid;
    opReg0   <= mulin_op[7:0];
    //HAVE_DSP_UNIT
    sumReg_for32   <= (s1|s2|s3|s4|s5|s6|s7|s11|s15) ? sum_for32 : {mul_for16_tmp_1, mul_for16_tmp_0};
    carryReg_for32 <= (s1|s2|s3|s4|s5|s6|s7|s11|s15) ? carry_for32 : {temp_for16_1, temp_for16_0};
    vj_xor_vk_16_reg_0 <= vj_xor_vk_16_0;
    vj_xor_vk_16_reg_1 <= vj_xor_vk_16_1;
    outflagReg0    <= outflag_in;
    posReg0    <= pos_in;
    efiReg0    <= efi_in;
    vkReg0 <= vk;
    acReg0         <= ac;
    vjReg0 <= vj;
   end
end
always @(posedge clock) begin                      
  if(init | (!allow | ~valid) & brcancel_p1)
        valid_p1 <= 1'b0;
  else if(allow & valid)
        valid_p1 <= !brcancel_in;
  else if(allow_in_p2)
       valid_p1 <= 1'b0;
end

//added by lumin
//for op_in
assign s1 = mulin_op==`OP_MUL;
assign s2 = mulin_op==`OP_MULT;
assign s3 = mulin_op==`OP_MULTU;
assign s4 = mulin_op==`OP_MADD;
assign s5 = mulin_op==`OP_MADDU;
assign s6 = mulin_op==`OP_MSUB;
assign s7 = mulin_op==`OP_MSUBU;
//HAVE_DSP_UNIT
assign s8 = mulin_op==`OP_DPAU_H_QBL;
assign s9 = mulin_op==`OP_DPAU_H_QBR;
assign s10 = mulin_op==`OP_DPAQ_S_W_PH;
assign s11 = mulin_op==`OP_DPAQ_SA_L_W;
assign s12 = mulin_op==`OP_DPSU_H_QBL;
assign s13 = mulin_op==`OP_DPSU_H_QBR;
assign s14 = mulin_op==`OP_DPSQ_S_W_PH;
assign s15 = mulin_op==`OP_DPSQ_SA_L_W;
assign s16 = mulin_op==`OP_MAQ_S_W_PHL;
assign s17 = mulin_op==`OP_MAQ_S_W_PHR;
assign s18 = mulin_op==`OP_MAQ_SA_W_PHL;
assign s19 = mulin_op==`OP_MAQ_SA_W_PHR;
assign s20 = mulin_op==`OP_MULSAQ_S_W_PH;
assign s21 = mulin_op==`OP_MULEU_S_PH_QBL;
assign s22 = mulin_op==`OP_MULEU_S_PH_QBR;
assign s23 = mulin_op==`OP_MULQ_RS_PH;
assign s24 = mulin_op==`OP_MULEQ_S_W_PHL;
assign s25 = mulin_op==`OP_MULEQ_S_W_PHR;
//for op_out
assign o1 = opReg0==`OP_MUL;
assign o2 = opReg0==`OP_MULT;
assign o3 = opReg0==`OP_MULTU;
assign o4 = opReg0==`OP_MADD;
assign o5 = opReg0==`OP_MADDU;
assign o6 = opReg0==`OP_MSUB;
assign o7 = opReg0==`OP_MSUBU;
//HAVE_DSP_UNIT
assign o8 = opReg0==`OP_DPAU_H_QBL;
assign o9 = opReg0==`OP_DPAU_H_QBR;
assign o10 = opReg0==`OP_DPAQ_S_W_PH;
assign o11 = opReg0==`OP_DPAQ_SA_L_W;
assign o12 = opReg0==`OP_DPSU_H_QBL;
assign o13 = opReg0==`OP_DPSU_H_QBR;
assign o14 = opReg0==`OP_DPSQ_S_W_PH;
assign o15 = opReg0==`OP_DPSQ_SA_L_W;
assign o16 = opReg0==`OP_MAQ_S_W_PHL;
assign o17 = opReg0==`OP_MAQ_S_W_PHR;
assign o18 = opReg0==`OP_MAQ_SA_W_PHL;
assign o19 = opReg0==`OP_MAQ_SA_W_PHR;
assign o20 = opReg0==`OP_MULSAQ_S_W_PH;
assign o21 = opReg0==`OP_MULEU_S_PH_QBL;
assign o22 = opReg0==`OP_MULEU_S_PH_QBR;
assign o23 = opReg0==`OP_MULQ_RS_PH;
assign o24 = opReg0==`OP_MULEQ_S_W_PHL;
assign o25 = opReg0==`OP_MULEQ_S_W_PHR;
assign v1 = opReg1==`OP_MUL;
assign v2 = opReg1==`OP_MULT;
assign v3 = opReg1==`OP_MULTU;
assign v4 = opReg1==`OP_MADD;
assign v5 = opReg1==`OP_MADDU;
assign v6 = opReg1==`OP_MSUB;
assign v7 = opReg1==`OP_MSUBU;
//HAVE_DSP_UNIT
assign v8 = opReg1==`OP_DPAU_H_QBL;
assign v9 = opReg1==`OP_DPAU_H_QBR;
assign v10 = opReg1==`OP_DPAQ_S_W_PH;
assign v11 = opReg1==`OP_DPAQ_SA_L_W;
assign v12 = opReg1==`OP_DPSU_H_QBL;
assign v13 = opReg1==`OP_DPSU_H_QBR;
assign v14 = opReg1==`OP_DPSQ_S_W_PH;
assign v15 = opReg1==`OP_DPSQ_SA_L_W;
assign v16 = opReg1==`OP_MAQ_S_W_PHL;
assign v17 = opReg1==`OP_MAQ_S_W_PHR;
assign v18 = opReg1==`OP_MAQ_SA_W_PHL;
assign v19 = opReg1==`OP_MAQ_SA_W_PHR;
assign v20 = opReg1==`OP_MULSAQ_S_W_PH;
assign v21 = opReg1==`OP_MULEU_S_PH_QBL;
assign v22 = opReg1==`OP_MULEU_S_PH_QBR;
assign v23 = opReg1==`OP_MULQ_RS_PH;
assign v24 = opReg1==`OP_MULEQ_S_W_PHL;
assign v25 = opReg1==`OP_MULEQ_S_W_PHR;
//for in
// HAVE_DSP_UNIT
/*assign vj16_1 = (s8|s12|s21|s9|s13|s22) ? {8'b0, (s8|s12|s21)?vj[31:24]:vj[15:8]} :
                (s10|s14|s20|s23) ? (vj[31] ? ~vj[31:16] : vj[31:16]) : 16'b0;

assign vk16_1 = (s8|s12|s9|s13) ? {8'b0, (s8|s12)?vk[31:24]:vk[15:8]} :
                (s10|s14|s20|s23|s21|s22) ? ((s10|s14|s20|s23)&vk[31] ? ~vk[31:16] : vk[31:16]) : 16'b0;

assign vj16_0 = (s8|s12|s21|s9|s13|s22) ? {8'b0, (s8|s12|s21)?vj[23:16]:vj[7:0]} :
                (s10|s14|s17|s19|s20|s23|s25) ? (vj[15] ? ~vj[15:0] : vj[15:0]) :
                (s16|s18|s24) ? (vj[31] ? ~vj[31:16] : vj[31:16]) : 16'b0;

assign vk16_0 = (s8|s12|s9|s13|s21|s22) ? {(s8|s12|s9|s13)?8'b0:vk[15:8], (s8|s12)?vk[23:16]:vk[7:0]} :
                (s10|s14|s17|s19|s20|s23|s25) ? (vk[15] ? ~vk[15:0] :vk[15:0]) :
                (s16|s18|s24) ? (vk[31] ? ~vk[31:16] : vk[31:16]) : 16'b0;
*/

assign vj16_1[15:8] = (opreg==3'h3) ? (vj[31] ? ~vj[31:24] : vj[31:24]) : 8'b0;

assign vj16_1[7:0] = (opreg==3'h1|opreg==3'h5) ? vj[31:24] : (opreg==3'h2|opreg==3'h6) ? vj[15:8] :
                     (opreg==3'h3) ? (vj[31] ? ~vj[23:16] : vj[23:16]) : 8'b0;

assign vk16_1[15:8] = (opreg==3'h3) ? (vk[31] ? ~vk[31:24] : vk[31:24]) : 
                      (opreg==3'h1|opreg==3'h2) ? vk[31:24] : 8'b0;
                     
assign vk16_1[7:0] = (opreg==3'h5) ? vk[31:24] : (opreg==3'h6) ? vk[15:8] :
                     (opreg==3'h3 | opreg==3'h1|opreg==3'h2) ? ((opreg==3'b11 & vk[31]) ? ~vk[23:16] : vk[23:16]) :
                      8'b0; 

assign vj16_0[15:8] = (opreg==3'h3) ? (vj[15] ? ~vj[15:8] : vj[15:8]) : 
                      (opreg==3'h4) ? (vj[31] ? ~vj[31:24] : vj[31:24]) : 8'b0;

assign vj16_0[7:0] = (opreg==3'h3|opreg==3'h2|opreg==3'h6) ? ((opreg==3'h3 & vj[15]) ? ~vj[7:0] : vj[7:0]) :
                     (opreg==3'h4|opreg==3'h1|opreg==3'h5) ? ((opreg==3'h4 & vj[31]) ? ~vj[23:16] : vj[23:16]) : 
                      8'b0;

assign vk16_0[15:8] = (opreg==3'h3|opreg==3'h1|opreg==3'h2) ? ((opreg==3'h3 & vk[15]) ? ~vk[15:8] : vk[15:8]) :
                      (opreg==3'h4) ? (vk[31] ? ~vk[31:24] : vk[31:24]) : 8'b0;

assign vk16_0[7:0] = (opreg==3'h4|opreg==3'h5) ? ((opreg==3'h4 & vk[31]) ? ~vk[23:16] : vk[23:16]) : 
                     (opreg==3'h3|opreg==3'h1|opreg==3'h2|opreg==3'h6) ? ((opreg==3'h3&vk[15]) ? ~vk[7:0]:vk[7:0]):
                      8'b0; 

wire dsp_negtive_vj_1_0= (s10|s14|s17|s19|s20|s23|s25) & vj[15];
wire dsp_negtive_vj_2_0= (s16|s18|s24) & vj[31];
wire dsp_negtive_vk_1_0= (s10|s14|s17|s19|s20|s23|s25) & vk[15];
wire dsp_negtive_vk_2_0= (s16|s18|s24) & vk[31];

wire dsp_negtive_vj_1_1 = (s10|s14|s20|s23) & vj[31];
wire dsp_negtive_vk_1_1 = (s10|s14|s20|s23) & vk[31];

//wire [15:0] vj16_tmp_0 = dsp_negtive_vj_1_0 ? ~vj[15:0] : dsp_negtive_vj_2_0 ? ~vj[31:16] : vj16_0; 
//wire [15:0] vk16_tmp_0 = dsp_negtive_vk_1_0 ? ~vk[15:0] : dsp_negtive_vk_2_0 ? ~vk[31:16] : vk16_0; 

//wire [15:0] vj16_tmp_1 = dsp_negtive_vj_1_1 ? ~vj[31:16] : vj16_1; 
//wire [15:0] vk16_tmp_1 = dsp_negtive_vk_1_1 ? ~vk[31:16] : vk16_1; 

assign temp_for16_0 = (dsp_negtive_vj_1_0& dsp_negtive_vk_1_0) ? ({16'b0,~vj[15:0]}+{16'b0,~vk[15:0]}+1'b1) :
                      (dsp_negtive_vj_2_0& dsp_negtive_vk_2_0) ? ({16'b0,~vj[31:16]}+{16'b0,~vk[31:16]}+1'b1) :
                      (dsp_negtive_vj_1_0| dsp_negtive_vj_2_0) ? vk16_0 + 32'hffffffff:
                      (dsp_negtive_vk_1_0| dsp_negtive_vk_2_0) ? vj16_0 + 32'hffffffff : 32'b0;

assign temp_for16_1 = (dsp_negtive_vj_1_1& dsp_negtive_vk_1_1) ? ({16'b0,~vj[31:16]}+{16'b0,~vk[31:16]}+1'b1) :
                       dsp_negtive_vj_1_1 ? vk16_1 + 32'hffffffff:
                       dsp_negtive_vk_1_1 ? vj16_1 + 32'hffffffff: 32'b0;


//assign mul_for16_tmp_0 = vj16_tmp_0 * vk16_tmp_0;
//assign mul_for16_tmp_1 = vj16_tmp_1 * vk16_tmp_1;
assign mul_for16_tmp_0 = vj16_0 * vk16_0;
assign mul_for16_tmp_1 = vj16_1 * vk16_1;

assign vj_xor_vk_16_0 = (dsp_negtive_vj_1_0 ^ dsp_negtive_vk_1_0) | (dsp_negtive_vj_2_0 ^ dsp_negtive_vk_2_0);
assign vj_xor_vk_16_1 = dsp_negtive_vj_1_1 ^ dsp_negtive_vk_1_1;

wire [31:0] mul_for16_tmp_0_1 = sumReg_for32[31:0] + carryReg_for32[31:0];
wire [31:0] mul_for16_tmp_1_1 = sumReg_for32[63:32] + carryReg_for32[63:32];

assign mul_for16_0 = vj_xor_vk_16_reg_0 ? (~mul_for16_tmp_0_1) : mul_for16_tmp_0_1;
assign mul_for16_1 = vj_xor_vk_16_reg_1 ? (~mul_for16_tmp_1_1) : mul_for16_tmp_1_1;
BoothTDM boothMUL( .Unsign(op), .Multiplicand(vj), .Multiplier(vk), .sum(sum_for32), .carry(carry_for32));

assign mul_for32 = {carryReg_for32[62:0],1'b0} + sumReg_for32;

wire [15:0] temp1_16, temp2_16;
wire [31:0] temp1_32, temp2_32, temp3_32;
wire [63:0] temp1_64;
//HAVE_DSP_UNIT
assign overflow1 = (vkReg0[31:16] == 16'h8000)    & (vjReg0[31:16] == 16'h8000);
assign overflow2 = (vkReg0[15:0] == 16'h8000)     & (vjReg0[15:0] == 16'h8000);
assign overflow3 = (vkReg0[31:0] == 32'h80000000) & (vjReg0[31:0] == 32'h80000000);
assign overflow4 = (|(sumReg_for32[55:48])) | (|(sumReg_for32[23:16]));

assign temp1_16 = (o21|o22) & (|sumReg_for32[55:48]) ? 16'hffff : sumReg_for32[47:32];
               
assign temp2_16 = (o21|o22) & (|sumReg_for32[23:16]) ? 16'hffff : sumReg_for32[15:0];

assign temp1_64 = overflow3 ? 64'h7fffffffffffffff : {mul_for32[62:0],1'b0};

assign temp1_32 = overflow1 ? 32'h7fffffff : {mul_for16_1[30:14] + 1'b1, mul_for16_1[13:0], 1'b0};//for o23 
                                     
assign temp2_32 = overflow2 ? 32'h7fffffff : {mul_for16_0[30:14] + 1'b1, mul_for16_0[13:0], 1'b0};//for o23

assign temp3_32 = (o16|o18|o24) ? (overflow1 ? 32'h7fffffff : {mul_for16_0[30:0],1'b0}) : 
                  (o17|o19|o25) ? (overflow2 ? 32'h7fffffff : {mul_for16_0[30:0],1'b0}) :32'b0; 
//result_1 for o8,o9,o12,o13,o10,o14,o20
//for o10,o14,o20
wire [63:0] temp_1_32 = ((o10|o14|o20)&overflow1) ? 64'h7fffffff : {{32{mul_for16_1[30]}},mul_for16_1[30:0], 1'b0};
                  
wire [63:0] temp_2_32 = (o10|o14) ? (overflow2 ? 64'h7fffffff : {{32{mul_for16_0[30]}}, mul_for16_0[30:0], 1'b0}) : 
                     (o20&overflow2)?64'hffffffff80000000: ~{{32{mul_for16_0[30]}}, mul_for16_0[30:0],1'b0}; 

wire [63:0]result_1 = (o8|o9|o12|o13) ? (sumReg_for32[47:32] + sumReg_for32[15:0]) : (temp_1_32 + temp_2_32); 
wire [63:0] mul_tmp = o1 ? {32'b0,mul_for32[31:0]} :(o2|o3|o4|o5) ? mul_for32 : ~mul_for32;//last are o6,o7

wire [63:0] result_tmp;
wire [64:0] result_tmp_1;
// HAVE_DSP_UNIT
assign result_tmp_1 = o11 ? {temp1_64[63], temp1_64} : ~{temp1_64[63], temp1_64}; 
assign result_tmp = (o1|o2|o3|o4|o5|o6|o7) ? mul_tmp :
                    (o21|o22) ? {32'b0, temp1_16, temp2_16} :
                    (o16|o17|o18|o19) ? {{32{temp3_32[31]}}, temp3_32} :
                    (o24|o25) ? {32'b0,temp3_32} :
                     o23 ? {32'b0, temp1_32[31:16], temp2_32[31:16]} : 
                    (o8|o9) ? {32'b0, result_1[31:0]} :
                    (o12|o13) ? ~{32'b0, result_1[31:0]} :
                     o14 ? ~result_1 :
                     result_1;//o10|o20
 
reg [64:0] result_reg;
reg [3:0] overflow_reg;
always @(posedge clock) begin 
  if(reset)begin
      opReg1   <= `OP_SLL;
      acReg1   <= 2'b0;
      // HAVE_DSP_UNIT
      outflagReg1 <= outflag_in;
    end
  if (valid_p1&!brcancel_p1&allow_in_p2) begin
    QidReg1  <= QidReg0;
    brQidReg1  <= brQidReg0;
    opReg1   <= opReg0;
    acReg1   <= acReg0;
    //HAVE_DSP_UNIT
    outflagReg1 <= outflagReg0;
    posReg1 <= posReg0;
    efiReg1 <= efiReg0;
    vkReg1 <= vkReg0;
    result_reg <= (o11|o15) ? result_tmp_1 : {1'b0, result_tmp};
    overflow_reg <= {overflow4, overflow3, overflow2, overflow1};
    vjReg1 <= vjReg0;
  end
end
always @(posedge clock) begin                      
  if(init | !allow_in_p2 & brcancel_out)
        ValidReg0 <= 1'b0;
  else if(allow_in_p2)
        ValidReg0 <= valid_p1 & !brcancel_p1;
  
end

wire [64:0]result;
//HAVE_DSP_UNIT
assign result = ((v4|v5|v6|v7|v8|v9|v10|v11|v12|v13|v14|v15|v16|v17|v18|v19|v20) ? {vlReg0[63],vlReg0}:65'b0) +
                  result_reg + (v6|v7|v12|v13|v14|v15|v20);


assign overflow5 = result[64] ^ result[63];
assign overflow6 = (~result[63]) & (|(result[62:31]));
assign overflow7 = result[63] & (~(&(result[62:31])));   

assign overflow = v18|v19 ? (v18 ? overflow_reg[0]: overflow_reg[1]) | overflow6 | overflow7 :
                  v11|v15 ? overflow_reg[2] | overflow5 :
                  v21|v22 ? overflow_reg[3] :
                  v10|v14|v23|v20 ? overflow_reg[0] | overflow_reg[1] :
                  v16|v24 ? overflow_reg[0] :
                  v17|v25 ? overflow_reg[1] : 1'b0;

wire [7:0]ac_outflag; 
assign ac_outflag = {((opReg1==`OP_EXTR_W) | (opReg1==`OP_EXTR_R_W) | 
                      (opReg1==`OP_EXTR_RS_W)) & (ex_setcond1 | ex_setcond2), 7'b0} |
                    {((opReg1==`OP_EXTR_S_H) & (~res_tmp[63]) & (|res_tmp[63:15])) |
                     ((opReg1==`OP_EXTR_S_H) & res_tmp[63] & ~(&res_tmp[63:15])), 7'b0};

wire [7:0]outflag_out_tmp;                  
assign outflag_out_tmp[4]   = outflagReg1[4];
assign outflag_out_tmp[7:6] = outflagReg1[7:6];
assign outflag_out_tmp[5]   = outflagReg1[5] | (v21|v22|v23|v24|v25)&overflow;
assign outflag_out_tmp[3:0] =((acReg1==0) ? {3'b0, (v10|v11|v14|v15|v16|v17|v18|v19|v20)&overflow} :
                              (acReg1==1) ? {2'b0, (v10|v11|v14|v15|v16|v17|v18|v19|v20)&overflow, 1'b0} :
                              (acReg1==2) ? {1'b0, (v10|v11|v14|v15|v16|v17|v18|v19|v20)&overflow, 2'b0} :
                              (acReg1==3) ? {(v10|v11|v14|v15|v16|v17|v18|v19|v20)&overflow, 3'b0} : 4'b0)
                              | outflagReg1[3:0];

assign outflag_out = ac_class ? ac_outflag : outflag_out_tmp;  
wire [63:0] mul_product_tmp;
// HAVE_DSP_UNIT
assign mul_product_tmp = (v18|v19) & overflow6 ? 64'h7fffffff : 
                         (v18|v19) & overflow7 ? 64'hffffffff80000000 :
                         (v11|v15) & overflow5 ? (~result[64] ? 64'h7fffffffffffffff : 64'h8000000000000000) :
                         result[63:0];

assign mul_product = ac_d_class ? ac_dresult : (opReg1==`OP_MFHI | opReg1==`OP_MFLO) ? vlReg0 : mul_product_tmp; 
wire src_to_hi0 = (opReg1==`OP_MTHI) && (acReg1==2'b00); 
wire src_to_hi1 = (opReg1==`OP_MTHI) && (acReg1==2'b01); 
wire src_to_hi2 = (opReg1==`OP_MTHI) && (acReg1==2'b10); 
wire src_to_hi3 = (opReg1==`OP_MTHI) && (acReg1==2'b11); 

wire src_to_lo0 = (opReg1==`OP_MTLO) && (acReg1==2'b00); 
wire src_to_lo1 = (opReg1==`OP_MTLO) && (acReg1==2'b01); 
wire src_to_lo2 = (opReg1==`OP_MTLO) && (acReg1==2'b10); 
wire src_to_lo3 = (opReg1==`OP_MTLO) && (acReg1==2'b11); 

wire value_h_to_hi0 = write_acc_h_op && (acReg1==2'b00); 
wire value_h_to_hi1 = write_acc_h_op && (acReg1==2'b01); 
wire value_h_to_hi2 = write_acc_h_op && (acReg1==2'b10); 
wire value_h_to_hi3 = write_acc_h_op && (acReg1==2'b11); 

wire value_to_lo0 = write_acc_h_op && (acReg1==2'b00); 
wire value_to_lo1 = write_acc_h_op && (acReg1==2'b01); 
wire value_to_lo2 = write_acc_h_op && (acReg1==2'b10); 
wire value_to_lo3 = write_acc_h_op && (acReg1==2'b11); 


wire brcancel_div;
assign brcancel_div = brbus_brerr & (brbus_brmask[divres_brqid]);

always @(posedge clock) begin                      
  
  if (divres_h_valid & !brcancel_div)
      reg_hi0 <= divres_h_value;
      
  if (divres_valid & !brcancel_div)
      reg_low0 <= divres_value;
  
 //HAVE_DSP_UNIT 
  if (ValidReg0 & acc_write_ok & mulbusrep  & src_to_hi0) 
      reg_hi0  <= vjReg1;
 
  if (ValidReg0 & acc_write_ok & mulbusrep & src_to_hi1) 
      reg_hi1  <= vjReg1;
  
  if (ValidReg0 & acc_write_ok & mulbusrep & src_to_hi2) 
      reg_hi2  <= vjReg1;
  
  if (ValidReg0 & acc_write_ok & mulbusrep & src_to_hi3) 
      reg_hi3  <= vjReg1;

// HAVE_DSP_UNIT
 if (ValidReg0 & acc_write_ok & mulbusrep & src_to_lo0) 
      reg_low0  <= vjReg1;
 
 if (ValidReg0 & acc_write_ok & mulbusrep & src_to_lo1) 
      reg_low1  <= vjReg1;
  
 if (ValidReg0 & acc_write_ok & mulbusrep & src_to_lo2) 
      reg_low2  <= vjReg1;
  
 if (ValidReg0 & acc_write_ok & mulbusrep & src_to_lo3) 
      reg_low3  <= vjReg1;
 
// HAVE_DSP_UNIT 
  if (ValidReg0 & acc_write_ok & mulbusrep_h & value_h_to_hi0) 
      reg_hi0  <= mul_product[63:32];
 
  if (ValidReg0 & acc_write_ok & mulbusrep_h & value_h_to_hi1) 
      reg_hi1  <= mul_product[63:32];
  
  if (ValidReg0 & acc_write_ok & mulbusrep_h & value_h_to_hi2) 
      reg_hi2  <= mul_product[63:32];
  
  if (ValidReg0 & acc_write_ok & mulbusrep_h & value_h_to_hi3) 
      reg_hi3  <= mul_product[63:32];
// HAVE_DSP_UNIT
  if (ValidReg0 & acc_write_ok  & mulbusrep & value_to_lo0) 
      reg_low0  <= mul_product[31:0];
 
  if (ValidReg0 & acc_write_ok & mulbusrep & value_to_lo1) 
      reg_low1  <= mul_product[31:0];
  
  if (ValidReg0 & acc_write_ok & mulbusrep & value_to_lo2) 
      reg_low2  <= mul_product[31:0];
  
  if (ValidReg0 & acc_write_ok & mulbusrep & value_to_lo3) 
      reg_low3  <= mul_product[31:0];
end

// output to result bus
assign mulres_op = opReg1;

assign mulres[63:56] = mulres_op;    //bnt=0    
assign mulres[55]    = ValidReg0;    //bnt=0    
assign mulres[54]    = mulres_valid;
assign mulres[53:49] = {QidReg1[4:1], 1'b0};
//HAVE_DSP_UNIT
assign mulres[48:17] = (opReg1==`OP_MFHI) ? vlReg0[63:32] : (opReg1==`OP_MFLO) ? vlReg0[31:0] :
                       ac_class ? ac_result : result_reg[31:0];
assign mulres[16]    = 1'b0;
assign mulres[15]    = 1'b0;
assign mulres[14]    = 1'b0;
assign mulres[13]    = 1'b0;
assign mulres[12]    = 1'b0;
assign mulres[11]    = 1'b0;
assign mulres[10]    = 1'b0;
assign mulres[9]     = 1'b0;
assign mulres[8]     = 1'b0;
assign mulres[7]     = 1'b0;
assign mulres[6]     = 1'b0;
assign mulres[5:0]   = 6'b0;

endmodule

module BoothEncoder( MultiplierGroup, A, X2, S, Bs, SE, Unsign ) ;
input [2:0] MultiplierGroup;
input Bs, Unsign;
output A, X2, S, SE;

assign S  = (MultiplierGroup[0]&MultiplierGroup[1])|!MultiplierGroup[2];
assign A  = MultiplierGroup[2]|!(MultiplierGroup[0]|MultiplierGroup[1]);
assign X2 = (MultiplierGroup[0]==MultiplierGroup[1]);
                                                                                                                                                             
assign SE = (~|MultiplierGroup[2:0]) | 
            (&MultiplierGroup[2:0])  | 
            (~(Unsign | Bs | MultiplierGroup[2])) | 
            (Bs & (~Unsign) & MultiplierGroup[2]) | 
            (Unsign & (~MultiplierGroup[2]));
            
endmodule 

module BSel( M0, M1, A, X2, S, PP ) ;
input M0, M1, X2, A, S;
output PP; 

BMX bmx( .M0(M0), .M1(M1), .A(A), .X2(X2), .S(S), .PP(PP) );

endmodule

module BoothSelector( Multiplicand, A, X2, S, PartialProduct ) ;
input [32:0] Multiplicand;
input A, X2, S;
output [32:0] PartialProduct;

assign PartialProduct[0]  = (~X2 & Multiplicand[0]) ^ ~S;

BSel BS1( .M0(Multiplicand[0]), .M1(Multiplicand[1]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[1]) );
BSel BS2( .M0(Multiplicand[1]), .M1(Multiplicand[2]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[2]) );
BSel BS3( .M0(Multiplicand[2]), .M1(Multiplicand[3]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[3]) );
BSel BS4( .M0(Multiplicand[3]), .M1(Multiplicand[4]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[4]) );

BSel BS5( .M0(Multiplicand[4]), .M1(Multiplicand[5]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[5]) );
BSel BS6( .M0(Multiplicand[5]), .M1(Multiplicand[6]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[6]) );
BSel BS7( .M0(Multiplicand[6]), .M1(Multiplicand[7]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[7]) );
BSel BS8( .M0(Multiplicand[7]), .M1(Multiplicand[8]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[8]) );

BSel BS9( .M0(Multiplicand[8]), .M1(Multiplicand[9]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[9]) );
BSel BS10( .M0(Multiplicand[9]), .M1(Multiplicand[10]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[10]) );
BSel BS11( .M0(Multiplicand[10]), .M1(Multiplicand[11]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[11]) );
BSel BS12( .M0(Multiplicand[11]), .M1(Multiplicand[12]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[12]) );

BSel BS13( .M0(Multiplicand[12]), .M1(Multiplicand[13]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[13]) );
BSel BS14( .M0(Multiplicand[13]), .M1(Multiplicand[14]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[14]) );
BSel BS15( .M0(Multiplicand[14]), .M1(Multiplicand[15]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[15]) );
BSel BS16( .M0(Multiplicand[15]), .M1(Multiplicand[16]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[16]) );

BSel BS17( .M0(Multiplicand[16]), .M1(Multiplicand[17]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[17]) );
BSel BS18( .M0(Multiplicand[17]), .M1(Multiplicand[18]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[18]) );
BSel BS19( .M0(Multiplicand[18]), .M1(Multiplicand[19]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[19]) );
BSel BS20( .M0(Multiplicand[19]), .M1(Multiplicand[20]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[20]) );

BSel BS21( .M0(Multiplicand[20]), .M1(Multiplicand[21]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[21]) );
BSel BS22( .M0(Multiplicand[21]), .M1(Multiplicand[22]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[22]) );
BSel BS23( .M0(Multiplicand[22]), .M1(Multiplicand[23]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[23]) );
BSel BS24( .M0(Multiplicand[23]), .M1(Multiplicand[24]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[24]) );

BSel BS25( .M0(Multiplicand[24]), .M1(Multiplicand[25]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[25]) );
BSel BS26( .M0(Multiplicand[25]), .M1(Multiplicand[26]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[26]) );
BSel BS27( .M0(Multiplicand[26]), .M1(Multiplicand[27]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[27]) );
BSel BS28( .M0(Multiplicand[27]), .M1(Multiplicand[28]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[28]) );

BSel BS29( .M0(Multiplicand[28]), .M1(Multiplicand[29]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[29]) );
BSel BS30( .M0(Multiplicand[29]), .M1(Multiplicand[30]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[30]) );
BSel BS31( .M0(Multiplicand[30]), .M1(Multiplicand[31]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[31]) );
BSel BS32( .M0(Multiplicand[31]), .M1(Multiplicand[32]), .A(A), .X2(X2), .S(S), .PP(PartialProduct[32]) );

endmodule 

module BoothTDM(Unsign, Multiplicand, Multiplier, sum, carry);
input Unsign;
input [31:0]Multiplicand, Multiplier;
output [63:0] sum,carry;       

wire [32:0]MultiplicandEx;
wire [15:0]SE, A, S, X2;
wire [32:0]PartialProduct0,  PartialProduct1,  PartialProduct2,  PartialProduct3,
           PartialProduct4,  PartialProduct5,  PartialProduct6,  PartialProduct7,
           PartialProduct8,  PartialProduct9,  PartialProduct10, PartialProduct11,
           PartialProduct12, PartialProduct13, PartialProduct14, PartialProduct15;
wire [31:0]PartialProduct16;           

wire [13:0] cin0,  cin1,  cin2,  cin3,  cin4,  cin5,  cin6,  cin7, 
            cin8,  cin9,  cin10, cin11, cin12, cin13, cin14, cin15, 
            cin16, cin17, cin18, cin19, cin20, cin21, cin22, cin23, 
            cin24, cin25, cin26, cin27, cin28, cin29, cin30, cin31,
            cin32, cin33, cin34, cin35, cin36, cin37, cin38, cin39, 
            cin40, cin41, cin42, cin43, cin44, cin45, cin46, cin47, 
            cin48, cin49, cin50, cin51, cin52, cin53, cin54, cin55, 
            cin56, cin57, cin58, cin59, cin60, cin61, cin62, cin63;

// reg [63:0] sumReg, carryReg; // changed by zhoux
wire [63:0] carry, sum;
wire [15:0] init0 = 16'b0;       
wire init1 = 1'b1;       

BoothEncoder BoothEncoder0( 
    .MultiplierGroup({Multiplier[1], Multiplier[0], 1'b0}), 
    .A(A[0]), 
    .S(S[0]), 
    .X2(X2[0]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[0]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder1( 
    .MultiplierGroup({Multiplier[3], Multiplier[2], Multiplier[1]}), 
    .A(A[1]), 
    .S(S[1]), 
    .X2(X2[1]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[1]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder2( 
    .MultiplierGroup({Multiplier[5], Multiplier[4], Multiplier[3]}), 
    .A(A[2]), 
    .S(S[2]), 
    .X2(X2[2]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[2]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder3( 
    .MultiplierGroup({Multiplier[7], Multiplier[6], Multiplier[5]}), 
    .A(A[3]), 
    .S(S[3]), 
    .X2(X2[3]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[3]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder4( 
    .MultiplierGroup({Multiplier[9], Multiplier[8], Multiplier[7]}), 
    .A(A[4]), 
    .S(S[4]), 
    .X2(X2[4]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[4]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder5( 
    .MultiplierGroup({Multiplier[11], Multiplier[10], Multiplier[9]}), 
    .A(A[5]), 
    .S(S[5]), 
    .X2(X2[5]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[5]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder6( 
    .MultiplierGroup({Multiplier[13], Multiplier[12], Multiplier[11]}), 
    .A(A[6]), 
    .S(S[6]), 
    .X2(X2[6]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[6]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder7( 
    .MultiplierGroup({Multiplier[15], Multiplier[14], Multiplier[13]}), 
    .A(A[7]), 
    .S(S[7]), 
    .X2(X2[7]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[7]), 
    .Unsign(Unsign) ) ;
    
BoothEncoder BoothEncoder8( 
    .MultiplierGroup({Multiplier[17], Multiplier[16], Multiplier[15]}), 
    .A(A[8]), 
    .S(S[8]), 
    .X2(X2[8]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[8]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder9( 
    .MultiplierGroup({Multiplier[19], Multiplier[18], Multiplier[17]}), 
    .A(A[9]), 
    .S(S[9]), 
    .X2(X2[9]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[9]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder10( 
    .MultiplierGroup({Multiplier[21], Multiplier[20], Multiplier[19]}), 
    .A(A[10]), 
    .S(S[10]), 
    .X2(X2[10]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[10]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder11( 
    .MultiplierGroup({Multiplier[23], Multiplier[22], Multiplier[21]}), 
    .A(A[11]), 
    .S(S[11]), 
    .X2(X2[11]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[11]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder12( 
    .MultiplierGroup({Multiplier[25], Multiplier[24], Multiplier[23]}), 
    .A(A[12]), 
    .S(S[12]), 
    .X2(X2[12]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[12]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder13( 
    .MultiplierGroup({Multiplier[27], Multiplier[26], Multiplier[25]}), 
    .A(A[13]), 
    .S(S[13]), 
    .X2(X2[13]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[13]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder14( 
    .MultiplierGroup({Multiplier[29], Multiplier[28], Multiplier[27]}), 
    .A(A[14]), 
    .S(S[14]), 
    .X2(X2[14]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[14]), 
    .Unsign(Unsign) ) ;

BoothEncoder BoothEncoder15( 
    .MultiplierGroup({Multiplier[31], Multiplier[30], Multiplier[29]}), 
    .A(A[15]), 
    .S(S[15]), 
    .X2(X2[15]), 
    .Bs(Multiplicand[31]), 
    .SE(SE[15]), 
    .Unsign(Unsign) ) ;
    
assign MultiplicandEx = {{(~Unsign) & Multiplicand[31]}, Multiplicand };

BoothSelector BoothSelector0( 
    .Multiplicand(MultiplicandEx), 
    .A(A[0]), 
    .S(S[0]), 
    .X2(X2[0]), 
    .PartialProduct(PartialProduct0) ) ;

BoothSelector BoothSelector1( 
    .Multiplicand(MultiplicandEx), 
    .A(A[1]), 
    .S(S[1]), 
    .X2(X2[1]), 
    .PartialProduct(PartialProduct1) ) ;

BoothSelector BoothSelector2( 
    .Multiplicand(MultiplicandEx), 
    .A(A[2]), 
    .S(S[2]), 
    .X2(X2[2]), 
    .PartialProduct(PartialProduct2) ) ;

BoothSelector BoothSelector3( 
    .Multiplicand(MultiplicandEx), 
    .A(A[3]), 
    .S(S[3]), 
    .X2(X2[3]), 
    .PartialProduct(PartialProduct3) ) ;

BoothSelector BoothSelector4( 
    .Multiplicand(MultiplicandEx), 
    .A(A[4]), 
    .S(S[4]), 
    .X2(X2[4]), 
    .PartialProduct(PartialProduct4) ) ;

BoothSelector BoothSelector5( 
    .Multiplicand(MultiplicandEx), 
    .A(A[5]), 
    .S(S[5]), 
    .X2(X2[5]), 
    .PartialProduct(PartialProduct5) ) ;

BoothSelector BoothSelector6( 
    .Multiplicand(MultiplicandEx), 
    .A(A[6]), 
    .S(S[6]), 
    .X2(X2[6]), 
    .PartialProduct(PartialProduct6) ) ;

BoothSelector BoothSelector7( 
    .Multiplicand(MultiplicandEx), 
    .A(A[7]), 
    .S(S[7]), 
    .X2(X2[7]), 
    .PartialProduct(PartialProduct7) ) ;
    
BoothSelector BoothSelector8( 
    .Multiplicand(MultiplicandEx), 
    .A(A[8]), 
    .S(S[8]), 
    .X2(X2[8]), 
    .PartialProduct(PartialProduct8) ) ;

BoothSelector BoothSelector9( 
    .Multiplicand(MultiplicandEx), 
    .A(A[9]), 
    .S(S[9]), 
    .X2(X2[9]), 
    .PartialProduct(PartialProduct9) ) ;

BoothSelector BoothSelector10( 
    .Multiplicand(MultiplicandEx), 
    .A(A[10]), 
    .S(S[10]), 
    .X2(X2[10]), 
    .PartialProduct(PartialProduct10) ) ;

BoothSelector BoothSelector11( 
    .Multiplicand(MultiplicandEx), 
    .A(A[11]), 
    .S(S[11]), 
    .X2(X2[11]), 
    .PartialProduct(PartialProduct11) ) ;
    
BoothSelector BoothSelector12( 
    .Multiplicand(MultiplicandEx), 
    .A(A[12]), 
    .S(S[12]), 
    .X2(X2[12]), 
    .PartialProduct(PartialProduct12) ) ;

BoothSelector BoothSelector13( 
    .Multiplicand(MultiplicandEx), 
    .A(A[13]), 
    .S(S[13]), 
    .X2(X2[13]), 
    .PartialProduct(PartialProduct13) ) ;

BoothSelector BoothSelector14( 
    .Multiplicand(MultiplicandEx), 
    .A(A[14]), 
    .S(S[14]), 
    .X2(X2[14]), 
    .PartialProduct(PartialProduct14) ) ;

BoothSelector BoothSelector15( 
    .Multiplicand(MultiplicandEx), 
    .A(A[15]), 
    .S(S[15]), 
    .X2(X2[15]), 
    .PartialProduct(PartialProduct15) ) ;

assign PartialProduct16 = Multiplicand & {32{Unsign & Multiplier[31]}};

compressor17_2 w0(
        .in({PartialProduct0[0], ~S[0], init0[14:0]/*15'b0*/}), 
        .cin(init0[13:0]), 
        .cout(cin0), 
        .s(sum[0]), 
        .c(carry[0]));


compressor17_2 w1(
        .in({PartialProduct0[1], init0[15:0]/*16'b0*/}), 
        .cin(cin0), 
        .cout(cin1), 
        .s(sum[1]), 
        .c(carry[1]));
        

compressor17_2 w2(
        .in({PartialProduct0[2], PartialProduct1[0], ~S[1], init0[13:0]/*14'b0*/}), 
        .cin(cin1), 
        .cout(cin2), 
        .s(sum[2]), 
        .c(carry[2]));
        

compressor17_2 w3(
        .in({PartialProduct0[3], PartialProduct1[1], init0[14:0]/*15'b0*/}), 
        .cin(cin2), 
        .cout(cin3), 
        .s(sum[3]), 
        .c(carry[3]));
        

compressor17_2 w4(
        .in({PartialProduct0[4], PartialProduct1[2], PartialProduct2[0], ~S[2], init0[12:0]/*13'b0*/}), 
        .cin(cin3), 
        .cout(cin4), 
        .s(sum[4]), 
        .c(carry[4]));
        

compressor17_2 w5(
        .in({PartialProduct0[5], PartialProduct1[3], PartialProduct2[1], init0[13:0]/*14'b0*/}), 
        .cin(cin4), 
        .cout(cin5), 
        .s(sum[5]), 
        .c(carry[5]));
        

compressor17_2 w6(
        .in({PartialProduct0[6], PartialProduct1[4], PartialProduct2[2], PartialProduct3[0], ~S[3], init0[11:0]/*12'b0*/}), 
        .cin(cin5), 
        .cout(cin6), 
        .s(sum[6]), 
        .c(carry[6]));
        

compressor17_2 w7(
        .in({PartialProduct0[7], PartialProduct1[5], PartialProduct2[3], PartialProduct3[1], init0[12:0]/*13'b0*/}), 
        .cin(cin6), 
        .cout(cin7), 
        .s(sum[7]), 
        .c(carry[7]));
        

compressor17_2 w8(
        .in({PartialProduct0[8], PartialProduct1[6], PartialProduct2[4], PartialProduct3[2], PartialProduct4[0], ~S[4], init0[10:0]/*11'b0*/}), 
        .cin(cin7), 
        .cout(cin8), 
        .s(sum[8]), 
        .c(carry[8]));
        

compressor17_2 w9(
        .in({PartialProduct0[9], PartialProduct1[7], PartialProduct2[5], PartialProduct3[3], PartialProduct4[1], init0[11:0]/*12'b0*/}), 
        .cin(cin8), 
        .cout(cin9), 
        .s(sum[9]), 
        .c(carry[9]));
        

compressor17_2 w10(
        .in({PartialProduct0[10], PartialProduct1[8], PartialProduct2[6], PartialProduct3[4], PartialProduct4[2], PartialProduct5[0], ~S[5], init0[9:0]/*10'b0*/}), 
        .cin(cin9), 
        .cout(cin10), 
        .s(sum[10]), 
        .c(carry[10]));
        

compressor17_2 w11(
        .in({PartialProduct0[11], PartialProduct1[9], PartialProduct2[7], PartialProduct3[5], PartialProduct4[3], PartialProduct5[1], init0[10:0]/*11'b0*/}), 
        .cin(cin10), 
        .cout(cin11), 
        .s(sum[11]), 
        .c(carry[11]));
        

compressor17_2 w12(
        .in({PartialProduct0[12], PartialProduct1[10], PartialProduct2[8], PartialProduct3[6], PartialProduct4[4], PartialProduct5[2], PartialProduct6[0], ~S[6], init0[8:0]/*9'b0*/}), 
        .cin(cin11), 
        .cout(cin12), 
        .s(sum[12]), 
        .c(carry[12]));
        

compressor17_2 w13(
        .in({PartialProduct0[13], PartialProduct1[11], PartialProduct2[9], PartialProduct3[7], PartialProduct4[5], PartialProduct5[3], PartialProduct6[1], init0[9:0]/*10'b0*/}), 
        .cin(cin12), 
        .cout(cin13), 
        .s(sum[13]), 
        .c(carry[13]));
        

compressor17_2 w14(
        .in({PartialProduct0[14], PartialProduct1[12], PartialProduct2[10], PartialProduct3[8], PartialProduct4[6], PartialProduct5[4], PartialProduct6[2], PartialProduct7[0], ~S[7], init0[7:0]/*8'b0*/}), 
        .cin(cin13), 
        .cout(cin14), 
        .s(sum[14]), 
        .c(carry[14]));
        

compressor17_2 w15(
        .in({PartialProduct0[15], PartialProduct1[13], PartialProduct2[11], PartialProduct3[9], PartialProduct4[7], PartialProduct5[5], PartialProduct6[3], PartialProduct7[1], init0[8:0]/*9'b0*/}), 
        .cin(cin14), 
        .cout(cin15), 
        .s(sum[15]), 
        .c(carry[15]));
        

compressor17_2 w16(
        .in({PartialProduct0[16], PartialProduct1[14], PartialProduct2[12], PartialProduct3[10], PartialProduct4[8], PartialProduct5[6], PartialProduct6[4], PartialProduct7[2], PartialProduct8[0], ~S[8], init0[6:0]/*7'b0*/}), 
        .cin(cin15), 
        .cout(cin16), 
        .s(sum[16]), 
        .c(carry[16]));
        

compressor17_2 w17(
        .in({PartialProduct0[17], PartialProduct1[15], PartialProduct2[13], PartialProduct3[11], PartialProduct4[9], PartialProduct5[7], PartialProduct6[5], PartialProduct7[3], PartialProduct8[1], init0[7:0]/*8'b0*/}), 
        .cin(cin16), 
        .cout(cin17), 
        .s(sum[17]), 
        .c(carry[17]));
        

compressor17_2 w18(
        .in({PartialProduct0[18], PartialProduct1[16], PartialProduct2[14], PartialProduct3[12], PartialProduct4[10], PartialProduct5[8], PartialProduct6[6], PartialProduct7[4], PartialProduct8[2], PartialProduct9[0], ~S[9], init0[5:0]/*6'b0*/}), 
        .cin(cin17), 
        .cout(cin18), 
        .s(sum[18]), 
        .c(carry[18]));
        

compressor17_2 w19(
        .in({PartialProduct0[19], PartialProduct1[17], PartialProduct2[15], PartialProduct3[13], PartialProduct4[11], PartialProduct5[9], PartialProduct6[7], PartialProduct7[5], PartialProduct8[3], PartialProduct9[1], init0[6:0]/*7'b0*/}), 
        .cin(cin18), 
        .cout(cin19), 
        .s(sum[19]), 
        .c(carry[19]));
        
        
compressor17_2 w20(
        .in({PartialProduct0[20], PartialProduct1[18], PartialProduct2[16], PartialProduct3[14], PartialProduct4[12], PartialProduct5[10], PartialProduct6[8], PartialProduct7[6], PartialProduct8[4], PartialProduct9[2], PartialProduct10[0], ~S[10], init0[4:0]/*5'b0*/}),         .cin(cin19), 
        .cout(cin20), 
        .s(sum[20]), 
        .c(carry[20]));
        

compressor17_2 w21(
        .in({PartialProduct0[21], PartialProduct1[19], PartialProduct2[17], PartialProduct3[15], PartialProduct4[13], PartialProduct5[11], PartialProduct6[9], PartialProduct7[7], PartialProduct8[5], PartialProduct9[3], PartialProduct10[1], init0[5:0]/*6'b0*/}), 
        .cin(cin20),                             
        .cout(cin21), 
        .s(sum[21]), 
        .c(carry[21]));
        

compressor17_2 w22(
        .in({PartialProduct0[22], PartialProduct1[20], PartialProduct2[18], PartialProduct3[16], PartialProduct4[14], PartialProduct5[12], PartialProduct6[10], PartialProduct7[8], PartialProduct8[6], PartialProduct9[4], PartialProduct10[2], PartialProduct11[0], ~S[11], init0[3:0]/*4'b0*/}), 
        .cin(cin21), 
        .cout(cin22), 
        .s(sum[22]), 
        .c(carry[22]));
        

compressor17_2 w23(
        .in({PartialProduct0[23], PartialProduct1[21], PartialProduct2[19], PartialProduct3[17], PartialProduct4[15], PartialProduct5[13], PartialProduct6[11], PartialProduct7[9], PartialProduct8[7], PartialProduct9[5], PartialProduct10[3], PartialProduct11[1], init0[4:0]/*5'b0*/}), 
        .cin(cin22), 
        .cout(cin23), 
        .s(sum[23]), 
        .c(carry[23]));
        

compressor17_2 w24(
        .in({PartialProduct0[24], PartialProduct1[22], PartialProduct2[20], PartialProduct3[18], PartialProduct4[16], PartialProduct5[14], PartialProduct6[12], PartialProduct7[10], PartialProduct8[8], PartialProduct9[6], PartialProduct10[4], PartialProduct11[2], PartialProduct12[0], ~S[12], init0[2:0]/*3'b0*/}), 
        .cin(cin23), 
        .cout(cin24), 
        .s(sum[24]), 
        .c(carry[24]));
        

compressor17_2 w25(
        .in({PartialProduct0[25], PartialProduct1[23], PartialProduct2[21], PartialProduct3[19], PartialProduct4[17], PartialProduct5[15], PartialProduct6[13], PartialProduct7[11], PartialProduct8[9], PartialProduct9[7], PartialProduct10[5], PartialProduct11[3], PartialProduct12[1], init0[3:0]/*4'b0*/}), 
        .cin(cin24), 
        .cout(cin25), 
        .s(sum[25]), 
        .c(carry[25]));
        

compressor17_2 w26(
        .in({PartialProduct0[26], PartialProduct1[24], PartialProduct2[22], PartialProduct3[20], PartialProduct4[18], PartialProduct5[16], PartialProduct6[14], PartialProduct7[12], PartialProduct8[10], PartialProduct9[8], PartialProduct10[6], PartialProduct11[4], PartialProduct12[2], PartialProduct13[0], ~S[13], init0[1:0]/*2'b0*/}), 
        .cin(cin25), 
        .cout(cin26), 
        .s(sum[26]), 
        .c(carry[26]));
        

compressor17_2 w27(
        .in({PartialProduct0[27], PartialProduct1[25], PartialProduct2[23], PartialProduct3[21], PartialProduct4[19], PartialProduct5[17], PartialProduct6[15], PartialProduct7[13], PartialProduct8[11], PartialProduct9[9], PartialProduct10[7], PartialProduct11[5], PartialProduct12[3], PartialProduct13[1], init0[2:0]/*3'b0*/}), 
        .cin(cin26), 
        .cout(cin27), 
        .s(sum[27]), 
        .c(carry[27]));
        

compressor17_2 w28(
        .in({PartialProduct0[28], PartialProduct1[26], PartialProduct2[24], PartialProduct3[22], PartialProduct4[20], PartialProduct5[18], PartialProduct6[16], PartialProduct7[14], PartialProduct8[12], PartialProduct9[10], PartialProduct10[8], PartialProduct11[6], PartialProduct12[4], PartialProduct13[2], PartialProduct14[0], ~S[14], init0[0]/*1'b0*/}), 
        .cin(cin27), 
        .cout(cin28), 
        .s(sum[28]), 
        .c(carry[28]));
        

compressor17_2 w29(
        .in({PartialProduct0[29], PartialProduct1[27], PartialProduct2[25], PartialProduct3[23], PartialProduct4[21], PartialProduct5[19], PartialProduct6[17], PartialProduct7[15], PartialProduct8[13], PartialProduct9[11], PartialProduct10[9], PartialProduct11[7], PartialProduct12[5], PartialProduct13[3], PartialProduct14[1], init0[1:0]/*2'b0*/}), 
        .cin(cin28), 
        .cout(cin29), 
        .s(sum[29]), 
        .c(carry[29]));
        

compressor17_2 w30(
        .in({PartialProduct0[30], PartialProduct1[28], PartialProduct2[26], PartialProduct3[24], PartialProduct4[22], PartialProduct5[20], PartialProduct6[18], PartialProduct7[16], PartialProduct8[14], PartialProduct9[12], PartialProduct10[10], PartialProduct11[8], PartialProduct12[6], PartialProduct13[4], PartialProduct14[2], PartialProduct15[0], ~S[15]}), 
        .cin(cin29), 
        .cout(cin30), 
        .s(sum[30]), 
        .c(carry[30]));
        

compressor17_2 w31(
        .in({PartialProduct0[31], PartialProduct1[29], PartialProduct2[27], PartialProduct3[25], PartialProduct4[23], PartialProduct5[21], PartialProduct6[19], PartialProduct7[17], PartialProduct8[15], PartialProduct9[13], PartialProduct10[11], PartialProduct11[9], PartialProduct12[7], PartialProduct13[5], PartialProduct14[3], PartialProduct15[1], init0[0]}), 
        .cin(cin30), 
        .cout(cin31), 
        .s(sum[31]), 
        .c(carry[31]));
        

compressor17_2 w32(
        .in({PartialProduct0[32], PartialProduct1[30], PartialProduct2[28], PartialProduct3[26], PartialProduct4[24], PartialProduct5[22], PartialProduct6[20], PartialProduct7[18], PartialProduct8[16], PartialProduct9[14], PartialProduct10[12], PartialProduct11[10], PartialProduct12[8], PartialProduct13[6], PartialProduct14[4], PartialProduct15[2], PartialProduct16[0]}), 
        .cin(cin31), 
        .cout(cin32), 
        .s(sum[32]), 
        .c(carry[32]));
        

compressor17_2 w33(
        .in({~SE[0], PartialProduct1[31], PartialProduct2[29], PartialProduct3[27], PartialProduct4[25], PartialProduct5[23], PartialProduct6[21], PartialProduct7[19], PartialProduct8[17], PartialProduct9[15], PartialProduct10[13], PartialProduct11[11], PartialProduct12[9], PartialProduct13[7], PartialProduct14[5], PartialProduct15[3], PartialProduct16[1]}), 
        .cin(cin32), 
        .cout(cin33), 
        .s(sum[33]), 
        .c(carry[33]));
        

compressor17_2 w34(
        .in({~SE[0], PartialProduct1[32], PartialProduct2[30], PartialProduct3[28], PartialProduct4[26], PartialProduct5[24], PartialProduct6[22], PartialProduct7[20], PartialProduct8[18], PartialProduct9[16], PartialProduct10[14], PartialProduct11[12], PartialProduct12[10], PartialProduct13[8], PartialProduct14[6], PartialProduct15[4], PartialProduct16[2]}), 
        .cin(cin33), 
        .cout(cin34), 
        .s(sum[34]), 
        .c(carry[34]));
        

compressor17_2 w35(
        .in({SE[0], SE[1], PartialProduct2[31], PartialProduct3[29], PartialProduct4[27], PartialProduct5[25], PartialProduct6[23], PartialProduct7[21], PartialProduct8[19], PartialProduct9[17], PartialProduct10[15], PartialProduct11[13], PartialProduct12[11], PartialProduct13[9], PartialProduct14[7], PartialProduct15[5], PartialProduct16[3]}), 
        .cin(cin34), 
        .cout(cin35), 
        .s(sum[35]), 
        .c(carry[35]));
        

compressor17_2 w36(
        .in({init0[0]/*1'b0*/, init1, PartialProduct2[32], PartialProduct3[30], PartialProduct4[28], PartialProduct5[26], PartialProduct6[24], PartialProduct7[22], PartialProduct8[20], PartialProduct9[18], PartialProduct10[16], PartialProduct11[14], PartialProduct12[12], PartialProduct13[10], PartialProduct14[8], PartialProduct15[6], PartialProduct16[4]}), 
        .cin(cin35), 
        .cout(cin36), 
        .s(sum[36]), 
        .c(carry[36]));
        

compressor17_2 w37(
        .in({init0[1:0]/*2'b0*/, SE[2], PartialProduct3[31], PartialProduct4[29], PartialProduct5[27], PartialProduct6[25], PartialProduct7[23], PartialProduct8[21], PartialProduct9[19], PartialProduct10[17], PartialProduct11[15], PartialProduct12[13], PartialProduct13[11], PartialProduct14[9], PartialProduct15[7], PartialProduct16[5]}), 
        .cin(cin36), 
        .cout(cin37), 
        .s(sum[37]), 
        .c(carry[37]));
        

compressor17_2 w38(
        .in({init0[1:0]/*2'b0*/, init1, PartialProduct3[32], PartialProduct4[30], PartialProduct5[28], PartialProduct6[26], PartialProduct7[24], PartialProduct8[22], PartialProduct9[20], PartialProduct10[18], PartialProduct11[16], PartialProduct12[14], PartialProduct13[12], PartialProduct14[10], PartialProduct15[8], PartialProduct16[6]}), 
        .cin(cin37), 
        .cout(cin38), 
        .s(sum[38]), 
        .c(carry[38]));
        

compressor17_2 w39(
        .in({init0[2:0]/*3'b0*/, SE[3], PartialProduct4[31], PartialProduct5[29], PartialProduct6[27], PartialProduct7[25], PartialProduct8[23], PartialProduct9[21], PartialProduct10[19], PartialProduct11[17], PartialProduct12[15], PartialProduct13[13], PartialProduct14[11], PartialProduct15[9], PartialProduct16[7]}), 
        .cin(cin38), 
        .cout(cin39), 
        .s(sum[39]), 
        .c(carry[39]));
        
        
compressor17_2 w40(
        .in({init0[2:0]/*3'b0*/, init1, PartialProduct4[32], PartialProduct5[30], PartialProduct6[28], PartialProduct7[26], PartialProduct8[24], PartialProduct9[22], PartialProduct10[20], PartialProduct11[18], PartialProduct12[16], PartialProduct13[14], PartialProduct14[12], PartialProduct15[10], PartialProduct16[8]}), 
        .cin(cin39), 
        .cout(cin40), 
        .s(sum[40]), 
        .c(carry[40]));
        

compressor17_2 w41(
        .in({init0[3:0]/*4'b0*/, SE[4], PartialProduct5[31], PartialProduct6[29], PartialProduct7[27], PartialProduct8[25], PartialProduct9[23], PartialProduct10[21], PartialProduct11[19], PartialProduct12[17], PartialProduct13[15], PartialProduct14[13], PartialProduct15[11], PartialProduct16[9]}), 
        .cin(cin40), 
        .cout(cin41), 
        .s(sum[41]), 
        .c(carry[41]));
        

compressor17_2 w42(
        .in({init0[3:0]/*4'b0*/, init1, PartialProduct5[32], PartialProduct6[30], PartialProduct7[28], PartialProduct8[26], PartialProduct9[24], PartialProduct10[22], PartialProduct11[20], PartialProduct12[18], PartialProduct13[16], PartialProduct14[14], PartialProduct15[12], PartialProduct16[10]}), 
        .cin(cin41), 
        .cout(cin42), 
        .s(sum[42]), 
        .c(carry[42]));
        

compressor17_2 w43(
        .in({init0[4:0]/*5'b0*/, SE[5], PartialProduct6[31], PartialProduct7[29], PartialProduct8[27], PartialProduct9[25], PartialProduct10[23], PartialProduct11[21], PartialProduct12[19], PartialProduct13[17], PartialProduct14[15], PartialProduct15[13], PartialProduct16[11]}), 
        .cin(cin42), 
        .cout(cin43), 
        .s(sum[43]), 
        .c(carry[43]));
        

compressor17_2 w44(
        .in({init0[4:0]/*5'b0*/, init1, PartialProduct6[32], PartialProduct7[30], PartialProduct8[28], PartialProduct9[26], PartialProduct10[24], PartialProduct11[22], PartialProduct12[20], PartialProduct13[18], PartialProduct14[16], PartialProduct15[14], PartialProduct16[12]}), 
        .cin(cin43), 
        .cout(cin44), 
        .s(sum[44]), 
        .c(carry[44]));
        

compressor17_2 w45(
        .in({init0[5:0]/*6'b0*/, SE[6], PartialProduct7[31], PartialProduct8[29], PartialProduct9[27], PartialProduct10[25], PartialProduct11[23], PartialProduct12[21], PartialProduct13[19], PartialProduct14[17], PartialProduct15[15], PartialProduct16[13]}), 
        .cin(cin44), 
        .cout(cin45), 
        .s(sum[45]), 
        .c(carry[45]));
        

compressor17_2 w46(
        .in({init0[5:0]/*6'b0*/, init1, PartialProduct7[32], PartialProduct8[30], PartialProduct9[28], PartialProduct10[26], PartialProduct11[24], PartialProduct12[22], PartialProduct13[20], PartialProduct14[18], PartialProduct15[16], PartialProduct16[14]}), 
        .cin(cin45), 
        .cout(cin46), 
        .s(sum[46]), 
        .c(carry[46]));
        

compressor17_2 w47(
        .in({init0[6:0]/*7'b0*/, SE[7], PartialProduct8[31], PartialProduct9[29], PartialProduct10[27], PartialProduct11[25], PartialProduct12[23], PartialProduct13[21], PartialProduct14[19], PartialProduct15[17], PartialProduct16[15]}), 
        .cin(cin46), 
        .cout(cin47), 
        .s(sum[47]), 
        .c(carry[47]));
        

compressor17_2 w48(
        .in({init0[6:0]/*7'b0*/, init1, PartialProduct8[32], PartialProduct9[30], PartialProduct10[28], PartialProduct11[26], PartialProduct12[24], PartialProduct13[22], PartialProduct14[20], PartialProduct15[18], PartialProduct16[16]}), 
        .cin(cin47), 
        .cout(cin48), 
        .s(sum[48]), 
        .c(carry[48]));
        

compressor17_2 w49(
        .in({init0[7:0]/*8'b0*/, SE[8], PartialProduct9[31], PartialProduct10[29], PartialProduct11[27], PartialProduct12[25], PartialProduct13[23], PartialProduct14[21], PartialProduct15[19], PartialProduct16[17]}), 
        .cin(cin48), 
        .cout(cin49), 
        .s(sum[49]), 
        .c(carry[49]));
        
        
compressor17_2 w50(
        .in({init0[7:0]/*8'b0*/, init1, PartialProduct9[32], PartialProduct10[30], PartialProduct11[28], PartialProduct12[26], PartialProduct13[24], PartialProduct14[22], PartialProduct15[20], PartialProduct16[18]}), 
        .cin(cin49), 
        .cout(cin50), 
        .s(sum[50]), 
        .c(carry[50]));
        

compressor17_2 w51(
        .in({init0[8:0]/*9'b0*/, SE[9], PartialProduct10[31], PartialProduct11[29], PartialProduct12[27], PartialProduct13[25], PartialProduct14[23], PartialProduct15[21], PartialProduct16[19]}), 
        .cin(cin50), 
        .cout(cin51), 
        .s(sum[51]), 
        .c(carry[51]));
        

compressor17_2 w52(
        .in({init0[8:0]/*9'b0*/, init1, PartialProduct10[32], PartialProduct11[30], PartialProduct12[28], PartialProduct13[26], PartialProduct14[24], PartialProduct15[22], PartialProduct16[20]}), 
        .cin(cin51), 
        .cout(cin52), 
        .s(sum[52]), 
        .c(carry[52]));
        

compressor17_2 w53(
        .in({init0[9:0]/*10'b0*/, SE[10], PartialProduct11[31], PartialProduct12[29], PartialProduct13[27], PartialProduct14[25], PartialProduct15[23], PartialProduct16[21]}), 
        .cin(cin52), 
        .cout(cin53), 
        .s(sum[53]), 
        .c(carry[53]));
        

compressor17_2 w54(
        .in({init0[9:0]/*10'b0*/, init1, PartialProduct11[32], PartialProduct12[30], PartialProduct13[28], PartialProduct14[26], PartialProduct15[24], PartialProduct16[22]}), 
        .cin(cin53), 
        .cout(cin54), 
        .s(sum[54]), 
        .c(carry[54]));
        

compressor17_2 w55(
        .in({init0[10:0]/*11'b0*/, SE[11], PartialProduct12[31], PartialProduct13[29], PartialProduct14[27], PartialProduct15[25], PartialProduct16[23]}), 
        .cin(cin54), 
        .cout(cin55), 
        .s(sum[55]), 
        .c(carry[55]));
        

compressor17_2 w56(
        .in({init0[10:0]/*11'b0*/, init1, PartialProduct12[32], PartialProduct13[30], PartialProduct14[28], PartialProduct15[26], PartialProduct16[24]}), 
        .cin(cin55), 
        .cout(cin56), 
        .s(sum[56]), 
        .c(carry[56]));
        

compressor17_2 w57(
        .in({init0[11:0]/*12'b0*/, SE[12], PartialProduct13[31], PartialProduct14[29], PartialProduct15[27], PartialProduct16[25]}), 
        .cin(cin56), 
        .cout(cin57), 
        .s(sum[57]), 
        .c(carry[57]));
        

compressor17_2 w58(
        .in({init0[11:0]/*12'b0*/, init1, PartialProduct13[32], PartialProduct14[30], PartialProduct15[28], PartialProduct16[26]}), 
        .cin(cin57), 
        .cout(cin58), 
        .s(sum[58]), 
        .c(carry[58]));
        

compressor17_2 w59(
        .in({init0[12:0]/*13'b0*/, SE[13], PartialProduct14[31], PartialProduct15[29], PartialProduct16[27]}), 
        .cin(cin58), 
        .cout(cin59), 
        .s(sum[59]), 
        .c(carry[59]));
        
        
compressor17_2 w60(
        .in({init0[12:0]/*13'b0*/, init1, PartialProduct14[32], PartialProduct15[30], PartialProduct16[28]}), 
        .cin(cin59), 
        .cout(cin60), 
        .s(sum[60]), 
        .c(carry[60]));
        

compressor17_2 w61(
        .in({init0[13:0]/*14'b0*/, SE[14], PartialProduct15[31], PartialProduct16[29]}), 
        .cin(cin60), 
        .cout(cin61), 
        .s(sum[61]), 
        .c(carry[61]));
        

compressor17_2 w62(
        .in({init0[13:0]/*14'b0*/, init1, PartialProduct15[32], PartialProduct16[30]}), 
        .cin(cin61), 
        .cout(cin62), 
        .s(sum[62]), 
        .c(carry[62]));
        

compressor17_2 w63(
        .in({init0[14:0]/*15'b0*/, SE[15], PartialProduct16[31]}), 
        .cin(cin62), 
        .cout(cin63), 
        .s(sum[63]), 
        .c(carry[63]));

endmodule


module compressor17_2 (in, cin, cout, s, c);
input [16:0] in;
input [13:0] cin;
output [13:0] cout;
output s;
output c;      

wire counter0_s, counter1_s, counter2_s,
     counter3_s, counter4_s, counter5_s,
     counter6_s, counter7_s, counter8_s,
     counter9_s, counter10_s, counter11_s,
     counter12_s, counter13_s;

// Level 1
CSA3_2  counter0( .in(in[2:0]), .s(counter0_s), .c(cout[0]));
CSA3_2  counter1( .in(in[5:3]), .s(counter1_s), .c(cout[1]));
CSA3_2  counter2( .in(in[8:6]), .s(counter2_s), .c(cout[2]));
CSA3_2  counter3( .in(in[11:9]), .s(counter3_s), .c(cout[3]));
CSA3_2  counter4( .in(in[14:12]), .s(counter4_s), .c(cout[4]));
        
// Level 2
CSA3_2  counter5( .in({counter0_s, counter1_s, counter2_s}), .s(counter5_s), .c(cout[5]));
CSA3_2  counter6( .in({counter3_s, counter4_s, cin[0]} ), .s(counter6_s), .c(cout[6]));
CSA3_2  counter7( .in({cin[1], cin[2], cin[3]} ), .s(counter7_s), .c(cout[7]));
CSA3_2  counter8( .in({cin[4], in[15], in[16]} ), .s(counter8_s), .c(cout[8]));

// Level 3
CSA3_2  counter9( .in({counter5_s, counter6_s, counter7_s}), .s(counter9_s), .c(cout[9]));
CSA3_2  counter10( .in({counter8_s, cin[5], cin[6]}), .s(counter10_s), .c(cout[10]));

// Level 4
CSA3_2  counter11( .in({counter9_s, counter10_s, cin[7]}), .s(counter11_s), .c(cout[11]));
CSA3_2  counter12( .in({cin[8], cin[9], cin[10]}), .s(counter12_s), .c(cout[12]));

// Level 5
CSA3_2  counter13( .in({counter11_s, counter12_s, cin[11]}), .s(counter13_s), .c(cout[13]));

// Level 6
CSA3_2  counter14( .in({counter13_s, cin[13:12]}), .s(s), .c(c) );

endmodule 

module godson_div( clock,
                   reset,
                   commitbus_ex,
                   vj,
                   vk,
                   qid,
                   op,
                   valid,
                   divres_h,
                   divres,
                   divbusrep_h,
                   divbusrep,
                   allow,
                   brbus_brerr,
                   brbus_brmask,
                   acc_write_ok,
                   acc_qid,
                   brqid);

input         clock;
input         reset;
input         commitbus_ex;
input  [31:0] vj;
input  [31:0] vk;
input  [ 4:1] qid;
input  [ 7:0] op;
input         valid;
input         divbusrep;
input         divbusrep_h;

input         brbus_brerr;
input  [5:0]  brbus_brmask;
input         acc_write_ok;
input  [ 2:0] brqid;
output [ 3:0] acc_qid;

output [66:0] divres;
output [55:0] divres_h;
output        allow;

wire          clear;

assign        clear =reset | commitbus_ex;

reg  [ 5:0] counter;

wire        counter_63;
wire        counter_0;
wire        counter_1;
wire        counter_33;
wire        counter_32_2;
wire        counter_1_0;
wire        counter_63_33;

assign      counter_63   = counter == 6'b111111;
assign      counter_0    = counter == 6'b0;
assign      counter_1    = counter == 6'b000001;
assign      counter_33   = counter == 6'b100001;
assign      counter_1_0  = counter_0 | counter_1;
assign      counter_63_33= counter_33| counter_63;
assign      counter_32_2 = !(counter_1_0 | counter_63_33);

wire [31:0] dividend, divisor;
wire [31:0] vj_comp,vk_comp;
wire        divisor_gt_dividend;

assign      vj_comp  = ~vj + 1'b1;
assign      vk_comp  = ~vk + 1'b1;

assign      dividend = ((~op[0])&vj[31]) ? vj_comp : vj;
assign      divisor  = ((~op[0])&vk[31]) ? vk_comp : vk;

assign      divisor_gt_dividend = (divisor > dividend);

reg         rem_rdy;
reg         quot_rdy;
wire        rem_rdy_sign;
wire        quot_rdy_sign;

wire        stage2_rdy;

reg  [32:0] P;
reg  [31:0] a;
reg  [ 4:1] qidreg;
reg  [ 3:1] brqidreg;
reg  [ 7:0] op_0;
reg         divid_sign_reg;
reg         divis_sign_reg;

assign acc_qid = qidreg;

wire  brcancel_in;
wire  brcancel_1;

assign brcancel_in = brbus_brerr & brbus_brmask[brqid];
assign brcancel_1  = brbus_brerr & brbus_brmask[brqidreg];

wire [31:0] P_recover;
wire [31:0] cloz_order_changed_0;
wire [ 5:0] cloz_out_0_6;
wire [ 4:0] cloz_out_0;
wire        nonzero_0;

reg  [31:0] dividend_reserved;

wire [65:0] shift_temp;
wire [65:0] shift_temp1;
wire [65:0] shifted;
wire [32:0] P_temp;
wire [31:0] a_mvd;
wire [31:0] P_mvd;
wire [31:0] b_in;
wire [ 5:0] skip_step;
wire [ 5:0] shift_step;
wire [ 5:0] shift_switch;
wire [ 5:0] shift_switch1;
wire [ 5:0] counter_remain;
wire [ 5:0] divisor_in_effect;
wire [ 5:0] divisor_ls_step;
wire [ 5:0] counter_incr;
wire [ 3:0] q_in;
wire [ 2:0] brq_in;
wire [ 7:0] op_0_in;
wire        divid_sign_in;
wire        divis_sign_in;
wire        counter_en;
wire        divop_is_necessary;
wire        divop_isnot_necessary;
wire        cloz_0_sign_0;

reg  [31:0] P_mvd_reserved;
wire [31:0] dividend_temp;
reg  [31:0] b_mvd;
wire [31:0] b_mvd_temp;
reg  [5:0]  divisor_ls_reg;

wire        divisor_0_sign;

assign      divisor_0_sign    = (counter_0 & !nonzero_0) ? 1'b1 : 1'b0;

assign      cloz_0_sign_0     = (cloz_out_0_6 == 6'b0);
assign      skip_step         = (( counter_remain < cloz_out_0_6 ) | !nonzero_0 ) ? counter_remain : cloz_out_0_6;
assign      shift_step        = (P[32] | (nonzero_0 & cloz_0_sign_0 & counter_32_2 )) ? 6'b1 : skip_step;

assign      counter_remain    = 6'b100001 - counter;
assign      divisor_ls_step   = counter_0 ? (6'b100001 - cloz_out_0_6) : divisor_ls_reg;
assign      divisor_in_effect = divisor_ls_step + cloz_out_0_6;
assign      P_recover         = P[32] ? P_mvd_reserved : P[31:0];

assign      shift_temp1       =  counter_0    ? { P,33'b0}                             : {1'b0,dividend_reserved,33'b0};
assign      shift_temp        =  counter_32_2 ? { 1'b0,P_recover[31:0],a[31:0],~P[32]} : {34'b0,P_recover};
assign      shifted           = (counter_1_0 ? shift_temp1 : shift_temp ) << ( counter_33 ? divisor_ls_step : shift_step ); 

assign      a_mvd             = shifted[32: 1];
assign      P_mvd             = shifted[64:33];
assign      P_temp            = shifted[65:33] - {1'b0,b_mvd};

assign      counter_incr      = counter + shift_step;

wire        divop_will_begin;
assign      counter_en        = clear
                              | divop_is_necessary
                              | divop_isnot_necessary  //////////////////////////////////////////////////////////////////////////////////////////??????????????????????????
                              | counter_1_0 
                              | counter_32_2 
                              | counter_33;

assign      divop_is_necessary    =  divop_will_begin & !divisor_gt_dividend;
assign      divop_isnot_necessary =  divop_will_begin &  divisor_gt_dividend;// divop_will_begin &  divisor_gt_dividend;

assign      b_mvd_temp        =                            b_mvd;
assign      dividend_temp     =                dividend_reserved;
assign      b_in              = divisor;                                          
assign      q_in              = allow ? qid     :         qidreg;
assign      brq_in            = allow ? brqid   :       brqidreg;
assign      op_0_in           = allow ? op      :           op_0;
assign      divid_sign_in     = allow ? vj[31]  : divid_sign_reg;
assign      divis_sign_in     = allow ? vk[31]  : divis_sign_reg;

wire        rem_rdy_en;
wire        quot_rdy_en;

wire        quot_can_write;
wire        rem_can_write;
//wire        brcancel_during_op;
//wire        brcancel_in_valid;   ////////////////////////
wire        brcancel_1_valid;
//wire        brcancel_on_divbus;

//assign      brcancel_on_divbus=  counter_63 & brcancel_1 & quot_rdy&!quot_can_write;
assign      brcancel_1_valid  = brcancel_1 & ( !counter_63  | counter_63 & quot_rdy );
//assign      brcancel_during_op= !counter_63 & brcancel_1;

assign      quot_can_write    = divbusrep   & acc_write_ok;
assign      rem_can_write     = divbusrep_h & acc_write_ok;

wire        result_can_send;
assign      result_can_send   = (counter_33 & !brcancel_1);// | divop_isnot_necessary;

assign      divop_will_begin  = allow & valid & !brcancel_in;///////////////
wire        stop_immediately;
assign      stop_immediately  = divop_isnot_necessary | divisor_0_sign | brcancel_1_valid;//brcancel_during_op | brcancel_on_divbus;
//assign      brcancel_in_valid = brcancel_in & allow & valid;// brcancel_in & counter_63 & allow & valid;//quot_can_write;///////////////////////////
    

assign      allow             = counter_63 & (quot_can_write | stage2_rdy);// | divisor_0_sign | brcancel_1_valid;//stop_immediately;// | brcancel_in_valid;
assign      stage2_rdy        = ~(quot_rdy | rem_rdy);
/*
assign      quot_rdy_sign     = !clear & (result_can_send | (!quot_can_write & quot_rdy)) & !brcancel_in_valid;
assign      rem_rdy_sign      = !clear & (result_can_send | (!rem_can_write  & rem_rdy )) & !brcancel_in_valid;
assign      rem_rdy_en        =  clear |  result_can_send |  rem_rdy  & rem_can_write  | brcancel_in_valid;
assign      quot_rdy_en       =  clear |  result_can_send |  quot_rdy & quot_can_write| brcancel_in_valid;
*/
assign      quot_rdy_sign     = !clear 
                               & (divop_isnot_necessary | 
                                 (divisor_0_sign | result_can_send | (!quot_can_write & quot_rdy)) & !(brcancel_1_valid));
assign      rem_rdy_sign      = !clear 
                               & (divop_isnot_necessary | 
                                 (divisor_0_sign | result_can_send | (!rem_can_write & rem_rdy)) & !(brcancel_1_valid));
assign      rem_rdy_en        =  clear |  result_can_send |  rem_rdy  & rem_can_write     |  stop_immediately;
assign      quot_rdy_en       =  clear |  result_can_send |  quot_rdy & quot_can_write    |  stop_immediately;

wire [31:0] rem_unsign;
wire [31:0] quot_unsign;
wire [31:0] quot_last;
wire        different_sign;
wire        op_sign;

wire [32:0] last_P;
wire [31:0] last_a;

assign      op_sign           = divid_sign_reg & (~op_0[0]);
assign      quot_last         = {a[30:0],~P[32]};
assign      different_sign    = (divid_sign_reg ^ divis_sign_reg) & (~op_0[0]);

assign      rem_unsign        = ~shifted[64:33] + 1'b1;
assign      quot_unsign       = ~quot_last + 1'b1;

assign      last_P            = op_sign        ? rem_unsign  : shifted[64:33];
assign      last_a            = different_sign ? quot_unsign :      quot_last;

always @(posedge clock) begin
    if(counter_en)   counter     <= clear | stop_immediately               ? 6'b111111         :
                                    divop_is_necessary                     ? 6'b0              :
                                    counter_0                              ? 6'b1              :
                                    counter_1                              ? divisor_in_effect :
                                    counter_32_2                           ? counter_incr      : 6'b111111;
    
    if(divop_is_necessary) begin
        P                        <=                 {1'b0,b_in};
        dividend_reserved        <=                    dividend;
                           end
    else if(divop_isnot_necessary) begin
        P                        <=                          vj;
        a                        <=                       32'b0;
                                   end
    else if(counter_0) begin
        P                        <=        {1'b0,dividend_temp};
        b_mvd                    <=                       P_mvd;
        dividend_reserved        <=               dividend_temp;
                       end
    else if(counter_1 | counter_32_2) begin
        P                        <=                      P_temp;
        a                        <=                       a_mvd;
        b_mvd                    <=                  b_mvd_temp;
        dividend_reserved        <=               dividend_temp;
                                 end
    else if(counter_33) begin
        P                        <=                      last_P;
        a                        <=                      last_a;
         end
    
    if(quot_rdy_en) quot_rdy     <=               quot_rdy_sign;
    if(rem_rdy_en ) rem_rdy      <=                rem_rdy_sign;
        
        divisor_ls_reg           <=             divisor_ls_step;
        P_mvd_reserved           <=                       P_mvd;
        qidreg                   <=                        q_in;
        brqidreg                 <=                      brq_in;
        op_0                     <=                     op_0_in;
        divid_sign_reg           <=               divid_sign_in;
        divis_sign_reg           <=               divis_sign_in;
end

assign cloz_order_changed_0 = { P[ 0],P[ 1],P[ 2],P[ 3],P[ 4],P[ 5],P[ 6],P[ 7]
                               ,P[ 8],P[ 9],P[10],P[11],P[12],P[13],P[14],P[15]
                               ,P[16],P[17],P[18],P[19],P[20],P[21],P[22],P[23]
                               ,P[24],P[25],P[26],P[27],P[28],P[29],P[30],P[31]};

wire [2:0] P_a3,P_a2,P_a1,P_a0;
wire [3:0] P_nz;
wire [7:0] P_in3,P_in2,P_in1,P_in0;

assign P_in3 = cloz_order_changed_0[31:24];
assign P_in2 = cloz_order_changed_0[23:16];
assign P_in1 = cloz_order_changed_0[15:8];
assign P_in0 = cloz_order_changed_0[7:0];

assign P_a3[2]=(~(|P_in3[3:0]))&(|P_in3[7:4]);
assign P_a3[1]=((~P_in3[0]) & (~P_in3[1]) & (~P_in3[4]) & (~P_in3[5]) & (P_in3[6] | P_in3[7])) |
              ((~P_in3[0]) & (~P_in3[1]) & (P_in3[2] | P_in3[3]));
assign P_a3[0]=((~P_in3[0]) & (~P_in3[2]) & (~P_in3[4]) & (~P_in3[6]) & P_in3[7])|
              ((~P_in3[0]) & (~P_in3[2]) & (~P_in3[4]) & P_in3[5])|
              ((~P_in3[0]) & (~P_in3[2]) & P_in3[3])|
              ((~P_in3[0]) & P_in3[1]);
assign P_nz[3]=|P_in3[7:0];

assign P_a2[2]=(~(|P_in2[3:0]))&(|P_in2[7:4]);
assign P_a2[1]=((~P_in2[0]) & (~P_in2[1]) & (~P_in2[4]) & (~P_in2[5]) & (P_in2[6] | P_in2[7])) |
              ((~P_in2[0]) & (~P_in2[1]) & (P_in2[2] | P_in2[3]));
assign P_a2[0]=((~P_in2[0]) & (~P_in2[2]) & (~P_in2[4]) & (~P_in2[6]) & P_in2[7])|
              ((~P_in2[0]) & (~P_in2[2]) & (~P_in2[4]) & P_in2[5])|
              ((~P_in2[0]) & (~P_in2[2]) & P_in2[3])|
              ((~P_in2[0]) & P_in2[1]);
assign P_nz[2]=|P_in2[7:0];

assign P_a1[2]=(~(|P_in1[3:0]))&(|P_in1[7:4]);
assign P_a1[1]=((~P_in1[0]) & (~P_in1[1]) & (~P_in1[4]) & (~P_in1[5]) & (P_in1[6] | P_in1[7])) |
              ((~P_in1[0]) & (~P_in1[1]) & (P_in1[2] | P_in1[3]));
assign P_a1[0]=((~P_in1[0]) & (~P_in1[2]) & (~P_in1[4]) & (~P_in1[6]) & P_in1[7])|
              ((~P_in1[0]) & (~P_in1[2]) & (~P_in1[4]) & P_in1[5])|
              ((~P_in1[0]) & (~P_in1[2]) & P_in1[3])|
              ((~P_in1[0]) & P_in1[1]);
assign P_nz[1]=|P_in1[7:0];

assign P_a0[2]=(~(|P_in0[3:0]))&(|P_in0[7:4]);
assign P_a0[1]=((~P_in0[0]) & (~P_in0[1]) & (~P_in0[4]) & (~P_in0[5]) & (P_in0[6] | P_in0[7])) |
              ((~P_in0[0]) & (~P_in0[1]) & (P_in0[2] | P_in0[3]));
assign P_a0[0]=((~P_in0[0]) & (~P_in0[2]) & (~P_in0[4]) & (~P_in0[6]) & P_in0[7])|
              ((~P_in0[0]) & (~P_in0[2]) & (~P_in0[4]) & P_in0[5])|
              ((~P_in0[0]) & (~P_in0[2]) & P_in0[3])|
              ((~P_in0[0]) & P_in0[1]);
assign P_nz[0]=|P_in0[7:0];

assign cloz_out_0[4] = (~P_nz[0])&(~P_nz[1]) & (P_nz[2] | P_nz[3]);
assign cloz_out_0[3] = ((~P_nz[0])&( P_nz[1])) | ((~P_nz[0])&(~P_nz[2])&( P_nz[3]));
assign nonzero_0   = |P_nz[3:0];

assign cloz_out_0[2:0]= P_a0 |
                (P_a1 & {3{~P_nz[0]}})    |
                (P_a2 & {3{~|P_nz[1:0]}}) |
                (P_a3 & {3{~|P_nz[2:0]}});
 
assign      cloz_out_0_6    = {1'b0,cloz_out_0};
assign      divres  [66:64] =             brqidreg;
assign      divres  [63:56] =                 op_0;
assign      divres  [   55] =                 1'b0;
assign      divres  [   54] =   quot_rdy&divbusrep;
assign      divres  [53:49] =       {qidreg, 1'b0};
assign      divres  [48:17] =                    a;
assign      divres  [   16] =                 1'b0;
assign      divres  [   15] =                 1'b0;
assign      divres  [   14] =                 1'b0;
assign      divres  [   13] =                 1'b0;
assign      divres  [   12] =                 1'b0;
assign      divres  [   11] =                 1'b0;
assign      divres  [   10] =                 1'b0;
assign      divres  [    9] =                 1'b0;
assign      divres  [    8] =                 1'b0;
assign      divres  [    7] =                 1'b0;
assign      divres  [    6] =                 1'b0;
assign      divres  [ 5 :0] =                 6'b0;
                           
assign      divres_h[   55] =                 1'b0;
assign      divres_h[   54] =  rem_rdy&divbusrep_h;
assign      divres_h[53:49] =       {qidreg, 1'b1};
assign      divres_h[48:17] =              P[31:0];
assign      divres_h[   16] =                 1'b0;
assign      divres_h[   15] =                 1'b0;
assign      divres_h[   14] =                 1'b0;
assign      divres_h[   13] =                 1'b0;
assign      divres_h[   12] =                 1'b0;
assign      divres_h[   11] =                 1'b0;
assign      divres_h[   10] =                 1'b0;
assign      divres_h[    9] =                 1'b0;
assign      divres_h[    8] =                 1'b0;
assign      divres_h[    7] =                 1'b0;
assign      divres_h[    6] =                 1'b0;
assign      divres_h[ 5 :0] =                 6'b0;

endmodule

module	sra64(sign,in,shift,out);
input		sign;
input  [63:0]	in;
input  [4:0]	shift;
output [63:0]	out;

reg    [63:0]	result_temp1,result_temp2,result_temp3;

always @(sign or in or shift[1:0])
  case(shift[1:0])
    2'b00:  result_temp1 = in;
    2'b01:  result_temp1 = { {1{sign}},in[63:1] };
    2'b10:  result_temp1 = { {2{sign}},in[63:2] };
    2'b11:  result_temp1 = { {3{sign}},in[63:3] };
  endcase
  
always @(sign or result_temp1 or shift[3:2])
  case(shift[3:2])
    2'b00:  result_temp2 = result_temp1;
    2'b01:  result_temp2 = { {4{sign}}, result_temp1[63:4]  };
    2'b10:  result_temp2 = { {8{sign}}, result_temp1[63:8]  };
    2'b11:  result_temp2 = { {12{sign}},result_temp1[63:12] };
  endcase
  
always @(sign or result_temp2 or shift[4])
  case(shift[4])
    1'b0:  result_temp3 = result_temp2;
    1'b1:  result_temp3 = { {16{sign}},result_temp2[63:16] };
  endcase
  
assign	out=result_temp3;

endmodule 

module	sra32(sign,in,shift,out);
input		sign;
input  [31:0]	in;
input  [4:0]	shift;
output [31:0]	out;

reg    [31:0]	result_temp1,result_temp2,result_temp3;
wire sign_tmp = sign & in[31];
always @(sign_tmp or in or shift[1:0])
  case(shift[1:0])
    2'b00:  result_temp1 = in;
    2'b01:  result_temp1 = { {1{sign_tmp}},in[31:1] };
    2'b10:  result_temp1 = { {2{sign_tmp}},in[31:2] };
    2'b11:  result_temp1 = { {3{sign_tmp}},in[31:3] };
  endcase
  
always @(sign_tmp or result_temp1 or shift[3:2])
  case(shift[3:2])
    2'b00:  result_temp2 = result_temp1;
    2'b01:  result_temp2 = { {4{sign_tmp}}, result_temp1[31:4]  };
    2'b10:  result_temp2 = { {8{sign_tmp}}, result_temp1[31:8]  };
    2'b11:  result_temp2 = { {12{sign_tmp}},result_temp1[31:12] };
  endcase
  
always @(sign_tmp or result_temp2 or shift[4])
  case(shift[4])
    1'b0:  result_temp3 = result_temp2;
    1'b1:  result_temp3 = { {16{sign_tmp}},result_temp2[31:16] };
  endcase
  
assign	out=result_temp3;

endmodule 

module	sra16(sign,in,shift,out);
input		sign;
input  [15:0]	in;
input  [3:0]	shift;
output [15:0]	out;

reg    [15:0]	result_temp1,result_temp2;
wire sign_tmp = sign & in[15];
always @(in or shift[1:0] or sign_tmp)
  case(shift[1:0])
    2'b00:  result_temp1 = in;
    2'b01:  result_temp1 = { {1{sign_tmp}},in[15:1] };
    2'b10:  result_temp1 = { {2{sign_tmp}},in[15:2] };
    2'b11:  result_temp1 = { {3{sign_tmp}},in[15:3] };
  endcase
  
always @(sign_tmp or result_temp1 or shift[3:2])
  case(shift[3:2])
    2'b00:  result_temp2 = result_temp1;
    2'b01:  result_temp2 = { {4{sign_tmp}}, result_temp1[15:4]  };
    2'b10:  result_temp2 = { {8{sign_tmp}}, result_temp1[15:8]  };
    2'b11:  result_temp2 = { {12{sign_tmp}},result_temp1[15:12] };
  endcase
  
assign	out=result_temp2;

endmodule 
