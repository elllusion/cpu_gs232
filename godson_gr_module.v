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
`include "reg.h"
`include "bus.h"


module godson_gr_module(
    clock,
    reset,
    qissuebus0,
    rissuebus0,
    
    rissuebus1,
    qissuebus1,
    
    commitbus1,
    commitbus0,
    
    //HAVE_DSP_UNIT
    DSPCtl_value
);
input clock;
input reset;

input [`Lqissuebus0_to_gr-1:0] qissuebus0;
output [122:0]                 rissuebus0;

input [`Lqissuebus1_to_gr-1:0] qissuebus1;
output [99:0]                  rissuebus1;

input [`Lcommitbus_to_gr-1:0] commitbus0;

input [`Lcommitbus_to_gr-1:0] commitbus1;

//HAVE_DSP_UNIT
output [31:0] DSPCtl_value;

wire [1:0] qissuebus0_ac;
wire [1:0] qissuebus0_rs;
wire       qissuebus0_valid;
wire [3:0] qissuebus0_qid;
wire [2:0] qissuebus0_brqid;
wire [7:0] qissuebus0_op;
wire [7:0] qissuebus0_src1;
wire [7:0] qissuebus0_src2;
wire [`Lword-1:0] qissuebus0_res1;
wire [`Lword-1:0] qissuebus0_res2;
wire qissuebus0_rdy1;
wire qissuebus0_rdy2;
wire fpq_rdy2;
wire fpq_rdy2_h;
wire fpq_wb2;
wire fpq_wb2_h;
wire [3:0] fpq_qid2;
wire [3:0] fpq_qid2_h;
wire [2:0] fpq_tail;
wire [7:0] qissuebus0_dest;

wire [1:0] qissuebus1_ac;
wire [1:0] qissuebus1_rs;
wire       qissuebus1_valid;
wire [3:0] qissuebus1_qid;
wire [2:0] qissuebus1_brqid;
wire [7:0] qissuebus1_op;
wire [7:0] qissuebus1_src1;
wire [7:0] qissuebus1_src2;
wire [`Lword-1:0] qissuebus1_res1;
wire [`Lword-1:0] qissuebus1_res2;
wire qissuebus1_rdy1;
wire qissuebus1_rdy2;

wire [31:0] commitbus0_dspctl;
wire [1:0]  commitbus0_ac;
wire        commitbus0_con_true;
wire        commitbus0_valid;
wire [7:0]  commitbus0_op;
wire [7:0]  commitbus0_dest;
wire        commitbus0_ex; 
wire [`Lword-1:0] commitbus0_value;

wire [31:0] commitbus1_dspctl;
wire [1:0]  commitbus1_ac;
wire        commitbus1_con_true;
wire        commitbus1_valid;
wire [3:0]  commitbus1_qid;
wire [7:0]  commitbus1_op;
wire [7:0]  commitbus1_dest;
wire        commitbus1_ex; 
wire [`Lword-1:0] commitbus1_value;

wire [2:0] rissuebus0_fpqid;
wire [3:0] rissuebus0_fpqid2;
wire [3:0] rissuebus0_fpqid2_h;
wire rissuebus0_rdy2;
wire rissuebus0_rdy2_h;
wire rissuebus0_wb2;
wire rissuebus0_wb2_h;
wire [7:0]rissuebus0_dest;

wire [1:0] rissuebus0_ac;
wire [1:0] rissuebus0_rs;
wire rissuebus0_valid;
wire [3:0] rissuebus0_qid;
wire [2:0] rissuebus0_brqid;
wire [7:0] rissuebus0_op;
wire [`Lword-1:0] rissuebus0_vj;
wire [`Lword-1:0] rissuebus0_vk;
wire [7:0] rissuebus0_src1;
wire [7:0] rissuebus0_src2;

wire [1:0] rissuebus1_ac;
wire [1:0] rissuebus1_rs;
wire rissuebus1_valid;
wire [3:0] rissuebus1_qid;
wire [2:0] rissuebus1_brqid;
wire [7:0] rissuebus1_op;
wire [`Lword-1:0] rissuebus1_vj;
wire [5:0] rissuebus1_qj;
wire [`Lword-1:0] rissuebus1_vk;
wire [5:0] rissuebus1_qk;
wire [7:0] rissuebus1_src1;
wire [7:0] rissuebus1_src2;


wire [148:0] commitbus0_tmp, commitbus1_tmp;
assign commitbus0_tmp[84:0] = commitbus0;
assign commitbus1_tmp[84:0] = commitbus1;
assign commitbus0_ac       = commitbus0_tmp[84:83];
assign commitbus0_dspctl   = commitbus0_tmp[82:51];
assign commitbus0_con_true = commitbus0_tmp[50];
assign commitbus0_valid    = commitbus0_tmp[49];
assign commitbus0_op       = commitbus0_tmp[48:41];
assign commitbus0_dest     = commitbus0_tmp[40:33];
assign commitbus0_value    = commitbus0_tmp[32:1];
assign commitbus0_ex       = commitbus0_tmp[0];

assign commitbus1_ac       = commitbus1_tmp[84:83];
assign commitbus1_dspctl   = commitbus1_tmp[82:51];
assign commitbus1_con_true = commitbus1_tmp[50];
assign commitbus1_valid    = commitbus1_tmp[49];
assign commitbus1_op       = commitbus1_tmp[48:41];
assign commitbus1_dest     = commitbus1_tmp[40:33];
assign commitbus1_value    = commitbus1_tmp[32:1];
assign commitbus1_ex       = commitbus1_tmp[0];

assign qissuebus0_dest     = qissuebus0[124:117];
assign fpq_wb2_h           = qissuebus0[116];
assign fpq_wb2             = qissuebus0[115];
assign fpq_tail            = qissuebus0[114:112];
assign fpq_qid2            = qissuebus0[111:108];
assign fpq_qid2_h          = qissuebus0[107:104];
assign fpq_rdy2            = qissuebus0[103];
assign fpq_rdy2_h          = qissuebus0[102];
assign qissuebus0_rdy2     = qissuebus0[101];
assign qissuebus0_rdy1     = qissuebus0[100];
assign qissuebus0_brqid    = qissuebus0[99:97];
assign qissuebus0_ac       = qissuebus0[96:95];
assign qissuebus0_rs       = qissuebus0[94:93];
assign qissuebus0_valid    = qissuebus0[92];
assign qissuebus0_qid      = qissuebus0[91:88];
assign qissuebus0_op       = qissuebus0[87:80];
assign qissuebus0_src1     = qissuebus0[79:72];
assign qissuebus0_src2     = qissuebus0[71:64];
assign qissuebus0_res1     = qissuebus0[63:32];
assign qissuebus0_res2     = qissuebus0[31:0];

assign qissuebus1_rdy2  = qissuebus1[101];
assign qissuebus1_rdy1  = qissuebus1[100];
assign qissuebus1_brqid = qissuebus1[99:97];
assign qissuebus1_ac    = qissuebus1[96:95];
assign qissuebus1_rs    = qissuebus1[94:93];
assign qissuebus1_valid = qissuebus1[92];
assign qissuebus1_qid   = qissuebus1[91:88];
assign qissuebus1_op    = qissuebus1[87:80];
assign qissuebus1_src1  = qissuebus1[79:72];
assign qissuebus1_src2  = qissuebus1[71:64];
assign qissuebus1_res1  = qissuebus1[63:32];
assign qissuebus1_res2  = qissuebus1[31:0];

assign rissuebus0_ac    = qissuebus0_ac;
assign rissuebus0_rs    = qissuebus0_rs;
assign rissuebus0_valid = qissuebus0_valid;
assign rissuebus0_qid   = qissuebus0_qid;
assign rissuebus0_brqid = qissuebus0_brqid;
assign rissuebus0_op    = qissuebus0_op;
assign rissuebus0_src1  = qissuebus0_src1;
assign rissuebus0_src2  = qissuebus0_src2;
assign rissuebus0_dest  = qissuebus0_dest ;

assign rissuebus0_fpqid    = fpq_tail;
assign rissuebus0_fpqid2   = fpq_qid2; 
assign rissuebus0_fpqid2_h = fpq_qid2_h; 
assign rissuebus0_rdy2     = (qissuebus0_src2[7:6]==2'b01) ? fpq_rdy2 : 1'b1; 
assign rissuebus0_rdy2_h   = (qissuebus0_src2[7:6]==2'b01) ? fpq_rdy2_h : 1'b1; 
assign rissuebus0_wb2      = fpq_wb2;
assign rissuebus0_wb2_h    = fpq_wb2_h ;


assign rissuebus0[122:115] = rissuebus0_dest;
assign rissuebus0[114]     = rissuebus0_wb2_h;
assign rissuebus0[113]     = rissuebus0_wb2;
assign rissuebus0[112:110] = rissuebus0_fpqid;
assign rissuebus0[109:106] = rissuebus0_fpqid2;
assign rissuebus0[105:102] = rissuebus0_fpqid2_h;
assign rissuebus0[101]     = rissuebus0_rdy2;
assign rissuebus0[100]     = rissuebus0_rdy2_h;
assign rissuebus0[99:97]   = rissuebus0_brqid;
assign rissuebus0[96:95]   = rissuebus0_ac;
assign rissuebus0[94:93]   = rissuebus0_rs;
assign rissuebus0[92]      = rissuebus0_valid;
assign rissuebus0[91:88]   = rissuebus0_qid;
assign rissuebus0[87:80]   = rissuebus0_op;
assign rissuebus0[79:48]   = rissuebus0_vj;
assign rissuebus0[47:16]   = rissuebus0_vk;
assign rissuebus0[15:8]    = rissuebus0_src1;
assign rissuebus0[7:0]     = rissuebus0_src2;

assign rissuebus1_ac    = qissuebus1_ac;
assign rissuebus1_rs    = qissuebus1_rs;
assign rissuebus1_valid = qissuebus1_valid;
assign rissuebus1_qid   = qissuebus1_qid;
assign rissuebus1_brqid = qissuebus1_brqid;
assign rissuebus1_op    = qissuebus1_op;
assign rissuebus1_src1  = qissuebus1_src1;
assign rissuebus1_src2  = qissuebus1_src2;

assign rissuebus1[99:97] = rissuebus1_brqid;
assign rissuebus1[96:95] = rissuebus1_ac;
assign rissuebus1[94:93] = rissuebus1_rs;
assign rissuebus1[92]    = rissuebus1_valid;
assign rissuebus1[91:88] = rissuebus1_qid;
assign rissuebus1[87:80] = rissuebus1_op;
assign rissuebus1[79:48] = rissuebus1_vj;
assign rissuebus1[47:16] = rissuebus1_vk;
assign rissuebus1[15:8]  = rissuebus1_src1;
assign rissuebus1[7:0]   = rissuebus1_src2;

//HAVE_DSP_UNIT
reg [`Lword-1:0] reg_DSPCtl;
assign DSPCtl_value = reg_DSPCtl;


wire src1_zero_0,src1_imm_0,src1_gr_0,src1_res_0;
wire src1_DSPCtl_0;
wire [4:0] gr_read_addr1_0;
wire [`Lword-1:0] vj_0,vj_other_0,gr_read_value1_0;

wire src2_zero_0,src2_imm_0,src2_gr_0,src2_res_0;
wire [4:0] gr_read_addr2_0;
wire [`Lword-1:0] vk_0, gr_read_value2_0;

wire exception_0;
wire commit_write_enable_0,gr_write_enable_0;
wire [4:0] gr_write_addr_0;
wire [`Lword-1:0] gr_write_value_0;


wire src1_zero_1,src1_imm_1,src1_gr_1,src1_res_1;
wire src1_DSPCtl_1;
wire [4:0] gr_read_addr1_1;
wire [`Lword-1:0] vj_1,vj_other_1,gr_read_value1_1;

wire src2_zero_1,src2_imm_1,src2_gr_1,src2_res_1;
wire [4:0] gr_read_addr2_1;
wire [`Lword-1:0] vk_1,gr_read_value2_1;

wire exception_1;
wire commit_write_enable_1,gr_write_enable_1;
wire [4:0] gr_write_addr_1;
wire [`Lword-1:0] gr_write_value_1;
/*******************First read port*********************/
assign gr_read_addr1_0 = qissuebus0_src1[4:0];

assign src1_zero_0 = (qissuebus0_src1==8'b00000000);
assign src1_imm_0  = (qissuebus0_src1==8'b00111111);
assign src1_gr_0   = (~src1_zero_0)&&(qissuebus0_src1[7:5]==3'b000)&&qissuebus0_rdy1;
assign src1_res_0  = (~src1_zero_0)&&(qissuebus0_src1[7:6]==2'b00 )&&(~qissuebus0_rdy1) | src1_imm_0;

//HAVE_DSP_UNIT
assign src1_DSPCtl_0  = (qissuebus0_src1==8'b00101000)&&qissuebus0_rdy1; 
assign vj_other_0  = ({32{src1_DSPCtl_0 }} & reg_DSPCtl);
assign vj_0   = vj_other_0 | ({32{src1_gr_0 }} & gr_read_value1_0) | ({32{src1_res_0}} & qissuebus0_res1);

/*******************Second read port*********************/
assign gr_read_addr2_0 = qissuebus0_src2[4:0];

assign src2_zero_0 = (qissuebus0_src2==8'b00000000);
assign src2_imm_0  = (qissuebus0_src2==8'b00111111);
assign src2_gr_0   = (~src2_zero_0)&&(qissuebus0_src2[7:5]==3'b000)&&qissuebus0_rdy2;
assign src2_res_0  = (~src2_zero_0)&&(qissuebus0_src2[7:6]==2'b00 )&&(~qissuebus0_rdy2) | src2_imm_0;

assign vk_0   = ({32{src2_gr_0 }} & gr_read_value2_0) | ({32{src2_res_0}} & qissuebus0_res2);

/***********************third read port*****************************************/

assign gr_read_addr1_1= qissuebus1_src1[4:0];

assign src1_zero_1 = (qissuebus1_src1==8'b00000000);
assign src1_imm_1  = (qissuebus1_src1==8'b00111111);
assign src1_gr_1   = (~src1_zero_1)&&(qissuebus1_src1[7:5]==3'b000)&&qissuebus1_rdy1;
assign src1_res_1  = (~src1_zero_1)&&(qissuebus1_src1[7:6]==2'b00 )&&(~qissuebus1_rdy1) | src1_imm_1;

//HAVE_DSP_UNIT
assign src1_DSPCtl_1   = (qissuebus1_src1==8'b00101000)&&qissuebus1_rdy1;
assign vj_other_1  = {32{src1_DSPCtl_1 }} & reg_DSPCtl;

assign vj_1   = vj_other_1 | ({32{src1_gr_1 }} & gr_read_value1_1) | ({32{src1_res_1}} & qissuebus1_res1);

/*******************forth read port*********************/
assign gr_read_addr2_1 = qissuebus1_src2[4:0];

assign src2_zero_1 = (qissuebus1_src2==8'b00000000);
assign src2_imm_1  = (qissuebus1_src2==8'b00111111);
assign src2_gr_1   = (~src2_zero_1)&&(qissuebus1_src2[7:5]==3'b000)&&qissuebus1_rdy2;
assign src2_res_1  = (~src2_zero_1)&&(qissuebus1_src2[7:6]==2'b00 )&&(~qissuebus1_rdy2)|src2_imm_1 ;

assign vk_1   = ({32{src2_gr_1 }} & gr_read_value2_1) | ({32{src2_res_1}} & qissuebus1_res2);
/*******************first Write port*********************/

wire commitbus0_op_mv = (commitbus0_op==`OP_MOVN)||(commitbus0_op==`OP_MOVZ)||
                        (commitbus0_op==`OP_MOVT)||(commitbus0_op==`OP_MOVF);
assign exception_0    = commitbus0_ex;
assign commit_write_enable_0 = commitbus0_valid & (commitbus0_dest!=8'b0) & (~exception_0)& 
                               (commitbus0_con_true | ~commitbus0_op_mv);

assign gr_write_enable_0 = commit_write_enable_0 & (commitbus0_dest[7:5]==3'b000);

assign gr_write_addr_0   = commitbus0_dest[4:0];

assign gr_write_value_0  = commitbus0_value;

/*******************second Write port*********************/
wire commitbus1_op_mv = (commitbus1_op==`OP_MOVN)||(commitbus1_op==`OP_MOVZ)||
                        (commitbus1_op==`OP_MOVT)||(commitbus1_op==`OP_MOVF);
assign exception_1    = commitbus1_ex ;
assign commit_write_enable_1 = commitbus1_valid & (commitbus1_dest!=8'b0) & (~exception_1)& 
                               (commitbus1_con_true | ~commitbus1_op_mv);

assign gr_write_enable_1     = commit_write_enable_1 & (commitbus1_dest[7:5]==3'b000);

assign gr_write_addr_1       = commitbus1_dest[4:0];

assign gr_write_value_1  = commitbus1_value;
/********************write hi/low reg*************************************/
wire write_hi_0,write_hi_1;
wire write_low_0,write_low_1;
WRITE_HI_OP write_hi0(.op(commitbus0_op), .write_hi_op(write_hi_0));
WRITE_LOW_OP write_low0(.op(commitbus0_op), .write_low_op(write_low_0));

WRITE_HI_OP write_hi1(.op(commitbus1_op), .write_hi_op(write_hi_1));
WRITE_LOW_OP write_low1(.op(commitbus1_op), .write_low_op(write_low_1));

wire commit_write_hi_enable_0, commit_write_low_enable_0;
wire commit_write_hi_enable_1, commit_write_low_enable_1;

assign commit_write_hi_enable_0 = commitbus0_valid & write_hi_0 & (~exception_0)& 
                                  (commitbus0_con_true | ~commitbus0_op_mv);

assign commit_write_low_enable_0 = commitbus0_valid & write_low_0 & (~exception_0)& 
                                   (commitbus0_con_true | ~commitbus0_op_mv);

assign commit_write_hi_enable_1 = commitbus1_valid & write_hi_1 & (~exception_1)& 
                                  (commitbus1_con_true | ~commitbus1_op_mv);

assign commit_write_low_enable_1 = commitbus1_valid & write_low_1 & (~exception_1)& 
                                   (commitbus1_con_true | ~commitbus1_op_mv);
/********************* write dspctl reg ********************************/
//HAVE_DSP_UNIT
wire dspctl_pos_wren_0, dspctl_pos_wren_1;
wire dspctl_scount_wren_0, dspctl_scount_wren_1;
wire dspctl_carry_wren_0, dspctl_carry_wren_1;
wire dspctl_efi_wren_0, dspctl_efi_wren_1;
wire dspctl_outflag_wren_0, dspctl_outflag_wren_1;
wire dspctl_ccond_wren_0, dspctl_ccond_wren_1;

wire commitbus0_wrdsp_pos,  commitbus1_wrdsp_pos;
wire commitbus0_wrdsp_carry,commitbus1_wrdsp_carry;
wire commitbus0_wrdsp_efi,  commitbus1_wrdsp_efi;
wire commitbus0_wrdsp_outflag,  commitbus1_wrdsp_outflag;
wire commitbus0_ccond,  commitbus1_ccond;

assign  commitbus0_wrdsp_pos =(commitbus0_op==`OP_EXTPDP)|| (commitbus0_op==`OP_MTHLIP) ;
assign  commitbus0_wrdsp_carry =(commitbus0_op==`OP_ADDSC); 
assign  commitbus0_wrdsp_efi =(commitbus0_op==`OP_EXTP) || (commitbus0_op==`OP_EXTPDP); 
assign  commitbus0_wrdsp_outflag = (commitbus0_op==`OP_ADDQ) ||    (commitbus0_op==`OP_ADDQ_S)||
                                   (commitbus0_op==`OP_SUBQ) || (commitbus0_op==`OP_SUBQ_S)|| 
                                   (commitbus0_op==`OP_ABSQ_S) ||  (commitbus0_op==`OP_PRECRQU_S_QB_PH)|| 
                                   (commitbus0_op==`OP_ADDWC) ||   (commitbus0_op==`OP_PRECRQ_RS_PH_W)|| 
                                   (commitbus0_op==`OP_SHLLV) ||   (commitbus0_op==`OP_SHLLVS)|| 
                                   (commitbus0_op==`OP_EXTR_W) ||  (commitbus0_op==`OP_EXTR_R_W)|| 
                                   (commitbus0_op==`OP_EXTR_RS_W)||(commitbus0_op==`OP_EXTR_S_H)|| 
                                   (commitbus0_op==`OP_DPAU_H_QBL)   || (commitbus0_op==`OP_DPAQ_S_W_PH)   || 
                                   (commitbus0_op==`OP_DPAU_H_QBR)   || (commitbus0_op==`OP_DPAQ_SA_L_W)   || 
                                   (commitbus0_op==`OP_DPSQ_S_W_PH)  || (commitbus0_op==`OP_MAQ_S_W_PHL)   || 
                                   (commitbus0_op==`OP_DPSQ_SA_L_W)  || (commitbus0_op==`OP_MAQ_S_W_PHR)   || 
                                   (commitbus0_op==`OP_MAQ_SA_W_PHL) || (commitbus0_op==`OP_MULEU_S_PH_QBL)|| 
                                   (commitbus0_op==`OP_MAQ_SA_W_PHR) || (commitbus0_op==`OP_MULEU_S_PH_QBR)|| 
                                   (commitbus0_op==`OP_MULEQ_S_W_PHL)|| (commitbus0_op==`OP_MULSAQ_S_W_PH) || 
                                   (commitbus0_op==`OP_MULEQ_S_W_PHR)|| (commitbus0_op==`OP_MULQ_RS_PH); 
assign commitbus0_ccond  = (commitbus0_dest == 8'h29) & 
                 ((commitbus0_op == `OP_CMP_EQ) | (commitbus0_op == `OP_CMP_LT) | (commitbus0_op == `OP_CMP_LE));

assign  commitbus1_wrdsp_carry =(commitbus1_op==`OP_ADDSC); 
assign  commitbus1_wrdsp_pos =(commitbus1_op==`OP_EXTPDP) || (commitbus1_op==`OP_MTHLIP) ;
assign  commitbus1_wrdsp_efi =(commitbus1_op==`OP_EXTP) || (commitbus1_op==`OP_EXTPDP); 
assign  commitbus1_wrdsp_outflag = (commitbus1_op==`OP_ADDQ) ||    (commitbus1_op==`OP_ADDQ_S)||
                                   (commitbus1_op==`OP_SUBQ) || (commitbus1_op==`OP_SUBQ_S)|| 
                                   (commitbus1_op==`OP_ABSQ_S) ||  (commitbus1_op==`OP_PRECRQU_S_QB_PH)|| 
                                   (commitbus1_op==`OP_ADDWC) ||   (commitbus1_op==`OP_PRECRQ_RS_PH_W)|| 
                                   (commitbus1_op==`OP_SHLLV) ||   (commitbus1_op==`OP_SHLLVS)|| 
                                   (commitbus1_op==`OP_EXTR_W) ||  (commitbus1_op==`OP_EXTR_R_W)|| 
                                   (commitbus1_op==`OP_EXTR_RS_W)||(commitbus1_op==`OP_EXTR_S_H)|| 
                                   (commitbus1_op==`OP_DPAU_H_QBL)   || (commitbus1_op==`OP_DPAQ_S_W_PH)   || 
                                   (commitbus1_op==`OP_DPAU_H_QBR)   || (commitbus1_op==`OP_DPAQ_SA_L_W)   || 
                                   (commitbus1_op==`OP_DPSQ_S_W_PH)  || (commitbus1_op==`OP_MAQ_S_W_PHL)   || 
                                   (commitbus1_op==`OP_DPSQ_SA_L_W)  || (commitbus1_op==`OP_MAQ_S_W_PHR)   || 
                                   (commitbus1_op==`OP_MAQ_SA_W_PHL) || (commitbus1_op==`OP_MULEU_S_PH_QBL)|| 
                                   (commitbus1_op==`OP_MAQ_SA_W_PHR) || (commitbus1_op==`OP_MULEU_S_PH_QBR)|| 
                                   (commitbus1_op==`OP_MULEQ_S_W_PHL)|| (commitbus1_op==`OP_MULSAQ_S_W_PH) || 
                                   (commitbus1_op==`OP_MULEQ_S_W_PHR)|| (commitbus1_op==`OP_MULQ_RS_PH); 

assign commitbus1_ccond  = (commitbus1_dest == 8'h29) &
                 ((commitbus1_op == `OP_CMP_EQ) | (commitbus1_op == `OP_CMP_LT) | (commitbus1_op == `OP_CMP_LE));

assign dspctl_pos_wren_0 = commitbus0_valid & ~commitbus0_ex & ((commitbus0_op==`OP_WRDSP) || commitbus0_wrdsp_pos);
assign dspctl_scount_wren_0 = commitbus0_valid & ~commitbus0_ex & (commitbus0_op==`OP_WRDSP) ;
assign dspctl_carry_wren_0 = commitbus0_valid & ~commitbus0_ex&((commitbus0_op==`OP_WRDSP)||commitbus0_wrdsp_carry);
assign dspctl_efi_wren_0 = commitbus0_valid & ~commitbus0_ex&((commitbus0_op==`OP_WRDSP)||commitbus0_wrdsp_efi);
assign dspctl_outflag_wren_0=commitbus0_valid&~commitbus0_ex&commitbus0_wrdsp_outflag&(|commitbus0_dspctl[23:16]);
wire dspctl_outflag_wren_0_by_wrdsp = commitbus0_valid & ~commitbus0_ex & (commitbus0_op==`OP_WRDSP);
assign dspctl_ccond_wren_0 = commitbus0_valid & ~commitbus0_ex & ((commitbus0_op==`OP_WRDSP) || commitbus0_ccond);
wire[3:0] dspctl_ccond_value_0  = commitbus0_dspctl[27:24];

assign dspctl_scount_wren_1 = commitbus1_valid & ~commitbus1_ex & (commitbus1_op==`OP_WRDSP) ;
assign dspctl_pos_wren_1 = commitbus1_valid & ~commitbus1_ex & ((commitbus1_op==`OP_WRDSP) || commitbus1_wrdsp_pos);
assign dspctl_carry_wren_1 = commitbus1_valid & ~commitbus1_ex&((commitbus1_op==`OP_WRDSP)||commitbus1_wrdsp_carry);
assign dspctl_efi_wren_1 = commitbus1_valid & ~commitbus1_ex&((commitbus1_op==`OP_WRDSP)||commitbus1_wrdsp_efi);
assign dspctl_outflag_wren_1=commitbus1_valid&~commitbus1_ex&commitbus1_wrdsp_outflag&(|commitbus1_dspctl[23:16]);
wire dspctl_outflag_wren_1_by_wrdsp = commitbus1_valid & ~commitbus1_ex & (commitbus1_op==`OP_WRDSP);
assign dspctl_ccond_wren_1 = commitbus1_valid & ~commitbus1_ex & ((commitbus1_op==`OP_WRDSP) || commitbus1_ccond);
wire[3:0] dspctl_ccond_value_1  = commitbus1_dspctl[27:24];

/*******************Common Part *********************/
reg_heap_32_32 gr_heap(.clock(clock),
                       .wen_0(gr_write_enable_0),
                       .waddr_0(gr_write_addr_0),
                       .wvalue_0(gr_write_value_0),
                       .raddr1_0(gr_read_addr1_0),
                       .raddr2_0(gr_read_addr2_0),
                       .rvalue1_0(gr_read_value1_0),
                       .rvalue2_0(gr_read_value2_0)
                      ,.wen_1(gr_write_enable_1),
                       .waddr_1(gr_write_addr_1),
                       .wvalue_1(gr_write_value_1),
                       .raddr1_1(gr_read_addr1_1),
                       .raddr2_1(gr_read_addr2_1),
                       .rvalue1_1(gr_read_value1_1),
                       .rvalue2_1(gr_read_value2_1)
                       );



//HAVE_DSP_UNIT 
always @ (posedge clock) begin
if (reset)
  reg_DSPCtl <= 32'b0;
else begin 

if(dspctl_pos_wren_1 || dspctl_pos_wren_0)
      reg_DSPCtl[5:0] <= dspctl_pos_wren_1 ? commitbus1_dspctl[5:0] : commitbus0_dspctl[5:0];
if(dspctl_scount_wren_1 || dspctl_scount_wren_0)
      reg_DSPCtl[12:7] <= dspctl_scount_wren_1 ? commitbus1_dspctl[12:7] : commitbus0_dspctl[12:7];
if(dspctl_carry_wren_1 || dspctl_carry_wren_0)
      reg_DSPCtl[13] <= dspctl_carry_wren_1 ? commitbus1_dspctl[13] : commitbus0_dspctl[13];
if(dspctl_efi_wren_1 || dspctl_efi_wren_0)
      reg_DSPCtl[14] <= dspctl_efi_wren_1 ? commitbus1_dspctl[14] : commitbus0_dspctl[14];

if(dspctl_outflag_wren_1|dspctl_outflag_wren_0|dspctl_outflag_wren_0_by_wrdsp|dspctl_outflag_wren_1_by_wrdsp)
      reg_DSPCtl[23:16] <= dspctl_outflag_wren_1_by_wrdsp ? commitbus1_dspctl[23:16] :
                           dspctl_outflag_wren_0_by_wrdsp&dspctl_outflag_wren_1 ? 
                           (commitbus0_dspctl[23:16]|commitbus1_dspctl[23:16]) :
                           dspctl_outflag_wren_0_by_wrdsp ? commitbus0_dspctl[23:16] :
                           dspctl_outflag_wren_1&dspctl_outflag_wren_0 ?
                           (commitbus1_dspctl[23:16]|commitbus0_dspctl[23:16]|DSPCtl_value[23:16]) :
                           dspctl_outflag_wren_1 ? (commitbus1_dspctl[23:16]|DSPCtl_value[23:16]) :
                           (commitbus0_dspctl[23:16]|DSPCtl_value[23:16]);

if(dspctl_ccond_wren_1 || dspctl_ccond_wren_0)
      reg_DSPCtl[27:24] <= dspctl_ccond_wren_1 ? dspctl_ccond_value_1  : dspctl_ccond_value_0;

     end
end // always

assign rissuebus0_vj = {vj_0};
assign rissuebus0_vk = {vk_0};
assign rissuebus1_vj = {vj_1};
assign rissuebus1_vk = {vk_1};
endmodule 

module reg_heap_32_32(clock,wen_0,waddr_0,raddr1_0,raddr2_0,wvalue_0,rvalue1_0,rvalue2_0
                            ,wen_1,waddr_1,raddr1_1,raddr2_1,wvalue_1,rvalue1_1,rvalue2_1
                      );
input clock;
input wen_0;
input [4:0] waddr_0;
input [4:0] raddr1_0;
input [4:0] raddr2_0;
input [`Lword-1:0] wvalue_0;

input wen_1;
input [4:0] waddr_1;
input [4:0] raddr1_1;
input [4:0] raddr2_1;
input [`Lword-1:0] wvalue_1;

output [`Lword-1:0] rvalue1_0;
output [`Lword-1:0] rvalue2_0;

output [`Lword-1:0] rvalue1_1;
output [`Lword-1:0] rvalue2_1;

reg [`Lword-1:0] gr[31:0];

assign rvalue1_0 = gr[raddr1_0];
assign rvalue2_0 = gr[raddr2_0];
assign rvalue1_1 = gr[raddr1_1];
assign rvalue2_1 = gr[raddr2_1];

wire [31:0] write_enable_0;
wire [31:0] write_enable_1;

wire [31:0] write_enable_0_t;
wire [31:0] write_enable_1_t;

decoder_5_32 gr_decoder_32_0(.in(waddr_0),.out(write_enable_0_t));

decoder_5_32 gr_decoder_32_1(.in(waddr_1),.out(write_enable_1_t));

assign write_enable_0 = write_enable_0_t & {32{wen_0}};

assign write_enable_1 = write_enable_1_t & {32{wen_1}};

wire gr_gating_clock;
assign gr_gating_clock = clock;

integer i;
always 	@( posedge gr_gating_clock) begin
  for (i=0;i<32;i=i+1) begin
    if(write_enable_1[i] || write_enable_0[i])  
    gr[i]<=write_enable_1[i] ? wvalue_1 : wvalue_0 ;
  end		
end

endmodule
