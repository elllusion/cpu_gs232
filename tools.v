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

module decoder_3_8(in,out);

input [2:0] in;     
output [7:0] out; 

wire [2:0]in;
wire [7:0] out;

assign out[7]=( in[2])&( in[1])&( in[0]);
assign out[6]=( in[2])&( in[1])&(~in[0]);
assign out[5]=( in[2])&(~in[1])&( in[0]);
assign out[4]=( in[2])&(~in[1])&(~in[0]);
assign out[3]=(~in[2])&( in[1])&( in[0]);
assign out[2]=(~in[2])&( in[1])&(~in[0]);
assign out[1]=(~in[2])&(~in[1])&( in[0]);
assign out[0]=(~in[2])&(~in[1])&(~in[0]);
endmodule

module decoder_4_16(enable,in,out);
input enable;                   
input  [3:0] in;
output [15:0] out; 

wire enable;
wire [3:0] in;
wire [15:0] out;
wire [3:0] high_d;
wire [3:0] low_d;

assign high_d[3]=( in[3])&( in[2])&enable;
assign high_d[2]=( in[3])&(~in[2])&enable;
assign high_d[1]=(~in[3])&( in[2])&enable;
assign high_d[0]=(~in[3])&(~in[2])&enable;

assign low_d[3]=( in[1])&( in[0]);    
assign low_d[2]=( in[1])&(~in[0]);
assign low_d[1]=(~in[1])&( in[0]);
assign low_d[0]=(~in[1])&(~in[0]);

assign out[15]=high_d[3]&low_d[3];
assign out[14]=high_d[3]&low_d[2];
assign out[13]=high_d[3]&low_d[1];
assign out[12]=high_d[3]&low_d[0];
assign out[11]=high_d[2]&low_d[3];
assign out[10]=high_d[2]&low_d[2];
assign out[ 9]=high_d[2]&low_d[1];
assign out[ 8]=high_d[2]&low_d[0];    
assign out[ 7]=high_d[1]&low_d[3];
assign out[ 6]=high_d[1]&low_d[2];
assign out[ 5]=high_d[1]&low_d[1];
assign out[ 4]=high_d[1]&low_d[0];
assign out[ 3]=high_d[0]&low_d[3];
assign out[ 2]=high_d[0]&low_d[2];
assign out[ 1]=high_d[0]&low_d[1];
assign out[ 0]=high_d[0]&low_d[0];    
endmodule

module decoder_5_32(in,out);
    
input [4:0] in;
output [31:0] out; 

wire [4:0]in;
wire [31:0]out;

wire [3:0] high_d;
wire [7:0] low_d;

assign high_d[3]=( in[4])&( in[3]);
assign high_d[2]=( in[4])&(~in[3]);
assign high_d[1]=(~in[4])&( in[3]);
assign high_d[0]=(~in[4])&(~in[3]);

assign low_d[7]=( in[2])&( in[1])&( in[0]);
assign low_d[6]=( in[2])&( in[1])&(~in[0]);
assign low_d[5]=( in[2])&(~in[1])&( in[0]);
assign low_d[4]=( in[2])&(~in[1])&(~in[0]);
assign low_d[3]=(~in[2])&( in[1])&( in[0]);
assign low_d[2]=(~in[2])&( in[1])&(~in[0]);
assign low_d[1]=(~in[2])&(~in[1])&( in[0]);
assign low_d[0]=(~in[2])&(~in[1])&(~in[0]);

assign out[31]=high_d[3]&low_d[7];
assign out[30]=high_d[3]&low_d[6];
assign out[29]=high_d[3]&low_d[5];
assign out[28]=high_d[3]&low_d[4];
assign out[27]=high_d[3]&low_d[3];
assign out[26]=high_d[3]&low_d[2];
assign out[25]=high_d[3]&low_d[1];
assign out[24]=high_d[3]&low_d[0];    
assign out[23]=high_d[2]&low_d[7];
assign out[22]=high_d[2]&low_d[6];
assign out[21]=high_d[2]&low_d[5];
assign out[20]=high_d[2]&low_d[4];
assign out[19]=high_d[2]&low_d[3];
assign out[18]=high_d[2]&low_d[2];
assign out[17]=high_d[2]&low_d[1];
assign out[16]=high_d[2]&low_d[0];    
assign out[15]=high_d[1]&low_d[7];
assign out[14]=high_d[1]&low_d[6];
assign out[13]=high_d[1]&low_d[5];
assign out[12]=high_d[1]&low_d[4];
assign out[11]=high_d[1]&low_d[3];
assign out[10]=high_d[1]&low_d[2];
assign out[ 9]=high_d[1]&low_d[1];
assign out[ 8]=high_d[1]&low_d[0];    
assign out[ 7]=high_d[0]&low_d[7];
assign out[ 6]=high_d[0]&low_d[6];
assign out[ 5]=high_d[0]&low_d[5];
assign out[ 4]=high_d[0]&low_d[4];
assign out[ 3]=high_d[0]&low_d[3];
assign out[ 2]=high_d[0]&low_d[2];
assign out[ 1]=high_d[0]&low_d[1];
assign out[ 0]=high_d[0]&low_d[0];    

endmodule

module decoder_6_64(in,out);
input [5:0] in;
output [63:0] out; 

wire [5:0]in;
wire [63:0] out;
wire [7:0] high_d;
wire [7:0] low_d;

assign high_d[7]=( in[5])&( in[4])&( in[3]);
assign high_d[6]=( in[5])&( in[4])&(~in[3]);
assign high_d[5]=( in[5])&(~in[4])&( in[3]);
assign high_d[4]=( in[5])&(~in[4])&(~in[3]);
assign high_d[3]=(~in[5])&( in[4])&( in[3]);
assign high_d[2]=(~in[5])&( in[4])&(~in[3]);
assign high_d[1]=(~in[5])&(~in[4])&( in[3]);
assign high_d[0]=(~in[5])&(~in[4])&(~in[3]);

assign low_d[7]=( in[2])&( in[1])&( in[0]);
assign low_d[6]=( in[2])&( in[1])&(~in[0]);
assign low_d[5]=( in[2])&(~in[1])&( in[0]);
assign low_d[4]=( in[2])&(~in[1])&(~in[0]);
assign low_d[3]=(~in[2])&( in[1])&( in[0]);
assign low_d[2]=(~in[2])&( in[1])&(~in[0]);
assign low_d[1]=(~in[2])&(~in[1])&( in[0]);
assign low_d[0]=(~in[2])&(~in[1])&(~in[0]);

assign out[63]=high_d[7]&low_d[7];
assign out[62]=high_d[7]&low_d[6];
assign out[61]=high_d[7]&low_d[5];
assign out[60]=high_d[7]&low_d[4];
assign out[59]=high_d[7]&low_d[3];
assign out[58]=high_d[7]&low_d[2];
assign out[57]=high_d[7]&low_d[1];
assign out[56]=high_d[7]&low_d[0];    
assign out[55]=high_d[6]&low_d[7];
assign out[54]=high_d[6]&low_d[6];
assign out[53]=high_d[6]&low_d[5];
assign out[52]=high_d[6]&low_d[4];
assign out[51]=high_d[6]&low_d[3];
assign out[50]=high_d[6]&low_d[2];
assign out[49]=high_d[6]&low_d[1];
assign out[48]=high_d[6]&low_d[0];    
assign out[47]=high_d[5]&low_d[7];
assign out[46]=high_d[5]&low_d[6];
assign out[45]=high_d[5]&low_d[5];
assign out[44]=high_d[5]&low_d[4];
assign out[43]=high_d[5]&low_d[3];
assign out[42]=high_d[5]&low_d[2];
assign out[41]=high_d[5]&low_d[1];
assign out[40]=high_d[5]&low_d[0];    
assign out[39]=high_d[4]&low_d[7];
assign out[38]=high_d[4]&low_d[6];
assign out[37]=high_d[4]&low_d[5];
assign out[36]=high_d[4]&low_d[4];
assign out[35]=high_d[4]&low_d[3];
assign out[34]=high_d[4]&low_d[2];
assign out[33]=high_d[4]&low_d[1];
assign out[32]=high_d[4]&low_d[0];    
assign out[31]=high_d[3]&low_d[7];
assign out[30]=high_d[3]&low_d[6];
assign out[29]=high_d[3]&low_d[5];
assign out[28]=high_d[3]&low_d[4];
assign out[27]=high_d[3]&low_d[3];
assign out[26]=high_d[3]&low_d[2];
assign out[25]=high_d[3]&low_d[1];
assign out[24]=high_d[3]&low_d[0];    
assign out[23]=high_d[2]&low_d[7];
assign out[22]=high_d[2]&low_d[6];
assign out[21]=high_d[2]&low_d[5];
assign out[20]=high_d[2]&low_d[4];
assign out[19]=high_d[2]&low_d[3];
assign out[18]=high_d[2]&low_d[2];
assign out[17]=high_d[2]&low_d[1];
assign out[16]=high_d[2]&low_d[0];    
assign out[15]=high_d[1]&low_d[7];
assign out[14]=high_d[1]&low_d[6];
assign out[13]=high_d[1]&low_d[5];
assign out[12]=high_d[1]&low_d[4];
assign out[11]=high_d[1]&low_d[3];
assign out[10]=high_d[1]&low_d[2];
assign out[ 9]=high_d[1]&low_d[1];
assign out[ 8]=high_d[1]&low_d[0];    
assign out[ 7]=high_d[0]&low_d[7];
assign out[ 6]=high_d[0]&low_d[6];
assign out[ 5]=high_d[0]&low_d[5];
assign out[ 4]=high_d[0]&low_d[4];
assign out[ 3]=high_d[0]&low_d[3];
assign out[ 2]=high_d[0]&low_d[2];
assign out[ 1]=high_d[0]&low_d[1];
assign out[ 0]=high_d[0]&low_d[0];    

endmodule

module MM_OP(op,mm_op);
input [7:0] op;
output mm_op;

assign mm_op=(op[7:6]==2'b10)||(op==`OP_MOVT)||(op==`OP_MOVF)||(op==`OP_FMOVN)||(op==`OP_FMOVZ);
endmodule           
module LD_ST_OP(op,ld_st_op);
input [7:0] op;
output ld_st_op;

wire [7:0]op;
wire ld_st_op;
assign ld_st_op = (op[7:4]==4'h9)| (op==`OP_PREFX) | (op==`OP_LWC1) | (op==`OP_SWC1) |
                  (op==`OP_LWL)  | (op==`OP_LWR)   | (op==`OP_SWL)  | (op==`OP_SWR);

//assign ld_st_op = (op==`OP_LB)   | (op==`OP_LH)   | (op==`OP_LW)   | (op==`OP_LDC1) | 
//		  (op==`OP_LBU)  | (op==`OP_LHU)  | (op==`OP_LL)   | (op==`OP_SB)   |
//		  (op==`OP_SH)   | (op==`OP_SW)   | (op==`OP_SDC1) | (op==`OP_SC)   |
//                  (op==`OP_LWL)  | (op==`OP_LWR)  | (op==`OP_SWL)  | (op==`OP_SWR)  |
//                  (op[7:4]==4'ha)| (op[7:4]==4'hb);
//assign ld_st_op = (op[7:6]==2'b10); 
endmodule                   

module ALU_MUL_OP(op,mul_op);
input [7:0] op;
output mul_op;

wire dsp_mul;
//HAVE_DSP_UNIT 
assign dsp_mul = (op==`OP_MULEU_S_PH_QBL) || (op==`OP_MULEU_S_PH_QBR) || (op==`OP_MULQ_RS_PH) ||
                 (op==`OP_MULEQ_S_W_PHL)  || (op==`OP_MULEQ_S_W_PHR)  || (op==`OP_MAQ_SA_W_PHL) ||
                 (op==`OP_MAQ_SA_W_PHR)   || (op==`OP_MAQ_S_W_PHL)    || (op==`OP_MAQ_S_W_PHR) || 
                 (op==`OP_MULSAQ_S_W_PH)  || (op==`OP_DPAU_H_QBL)     || (op==`OP_DPAU_H_QBR) ||
                 (op==`OP_DPAQ_S_W_PH)    || (op==`OP_DPAQ_SA_L_W)    || (op==`OP_DPSU_H_QBL) || 
                 (op==`OP_DPSU_H_QBR)     || (op==`OP_DPSQ_S_W_PH)    || (op==`OP_DPSQ_SA_L_W) |
                 (op==`OP_SHILO) || (op==`OP_MTHLIP) ||
                 (op==`OP_EXTP) || (op==`OP_EXTPDP) || (op==`OP_EXTR_W) || (op==`OP_EXTR_R_W) ||
                 (op==`OP_EXTR_RS_W) || (op==`OP_EXTR_S_H);  
 
assign mul_op= (op==`OP_MULT) ||(op==`OP_MULTU) ||(op==`OP_MUL)||
               (op==`OP_MADD)||(op==`OP_MADDU)||
               (op==`OP_MSUB)||(op==`OP_MSUBU)||
               (op==`OP_DIV) ||(op==`OP_DIVU) ||
               (op==`OP_MFHI) || (op==`OP_MFLO) || (op==`OP_MTHI) || (op==`OP_MTLO) || dsp_mul ; 
endmodule

module WRITE_HI_OP(op,write_hi_op);
input [7:0] op;
output write_hi_op;

//HAVE_DSP_UNIT 
wire dsp_hi_op = (op==`OP_MAQ_SA_W_PHL) ||
                   (op==`OP_MAQ_SA_W_PHR)   || (op==`OP_MAQ_S_W_PHL)    || (op==`OP_MAQ_S_W_PHR) || 
                   (op==`OP_MULSAQ_S_W_PH)  || (op==`OP_DPAU_H_QBL)     || (op==`OP_DPAU_H_QBR) ||
                   (op==`OP_DPAQ_S_W_PH)    || (op==`OP_DPAQ_SA_L_W)    || (op==`OP_DPSU_H_QBL) || 
                   (op==`OP_DPSU_H_QBR)     || (op==`OP_DPSQ_S_W_PH)    || (op==`OP_DPSQ_SA_L_W) ||
                   (op==`OP_SHILO) || (op==`OP_MTHLIP);   


wire fix_hi_op= (op==`OP_MULT) || (op==`OP_MULTU) || (op==`OP_MADD) || (op==`OP_MADDU) ||
                  (op==`OP_MSUB) || (op==`OP_MSUBU) || (op==`OP_DIV) || (op==`OP_DIVU) || 
                  (op==`OP_MTHI); 

assign write_hi_op = dsp_hi_op || fix_hi_op; 

endmodule

module WRITE_LOW_OP(op,write_low_op);
input [7:0] op;
output write_low_op;

//HAVE_DSP_UNIT 
wire dsp_low_op = (op==`OP_MAQ_SA_W_PHL) ||
                  (op==`OP_MAQ_SA_W_PHR)   || (op==`OP_MAQ_S_W_PHL)    || (op==`OP_MAQ_S_W_PHR) || 
                  (op==`OP_MULSAQ_S_W_PH)  || (op==`OP_DPAU_H_QBL)     || (op==`OP_DPAU_H_QBR) ||
                  (op==`OP_DPAQ_S_W_PH)    || (op==`OP_DPAQ_SA_L_W)    || (op==`OP_DPSU_H_QBL) || 
                  (op==`OP_DPSU_H_QBR)     || (op==`OP_DPSQ_S_W_PH)    || (op==`OP_DPSQ_SA_L_W) ||
                  (op==`OP_SHILO) || (op==`OP_MTHLIP);   

wire fix_low_op= (op==`OP_MULT) || (op==`OP_MULTU) || (op==`OP_MADD) || (op==`OP_MADDU) ||
                   (op==`OP_MSUB) || (op==`OP_MSUBU) || (op==`OP_DIV) || (op==`OP_DIVU) || 
                   (op==`OP_MTLO); 

assign write_low_op = dsp_low_op || fix_low_op; 

endmodule

module FALU_OP(op,falu_op);    
    
input [7:0] op;
output falu_op;
assign falu_op = (op[7:6]==2'b01) | (op==`OP_LWC1) | (op==`OP_LDC1) | (op==`OP_SWC1) | (op==`OP_SDC1) |
                 (op==`OP_MFC1) | (op==`OP_MTC1) | (op==`OP_CFC1) | (op==`OP_CTC1);

endmodule                   

module FALU_ON_ADDR(op,falu_addr);    
    
input [7:0] op;
output falu_addr;

assign falu_addr = (op==`OP_LWC1) | (op==`OP_LDC1) | (op==`OP_SWC1) | (op==`OP_SDC1) |
                   (op==`OP_MFC1) | (op==`OP_MTC1) | (op==`OP_CFC1) | (op==`OP_CTC1) |
                   (op==`OP_MOVF) | (op==`OP_MOVT) | (op==`OP_FMOVN) | (op==`OP_FMOVZ);
endmodule                   

module BLOCK_END(op,end_op);
input [7:0] op;
output end_op;

assign end_op=(op==`OP_J)      || (op==`OP_JAL)    ||
                 (op==`OP_JR)     || (op==`OP_JALR)   ||
                 (op==`OP_BEQ)    || (op==`OP_BNE)    ||
                 (op==`OP_BLEZ)   || (op==`OP_BGTZ)   ||
                 (op==`OP_BLTZ)   || (op==`OP_BGEZ)   ||
                 (op==`OP_BLTZAL) || (op==`OP_BGEZAL) ||
                 (op==`OP_BEQL)   || (op==`OP_BNEL)   ||
                 (op==`OP_BLEZL)  || (op==`OP_BGTZL)  ||
                 (op==`OP_BLTZL)  || (op==`OP_BGEZL)  ||
                 (op==`OP_BLTZALL)|| (op==`OP_BGEZALL)||
                 (op==`OP_BC1F)   || (op==`OP_BC1T)   ||
                 (op==`OP_BC1FL)  || (op==`OP_BC1TL)  ||
                 (op==`OP_ERET)  || (op==`OP_DERET)  
                 //HAVE_DSP_UNIT
                 ||(op==`OP_BPOSGE32)
                 ;

endmodule                   

module BR_OP(op,br_op);
input [7:0] op;
output br_op;

wire [7:0]op;
wire br_op;

assign br_op=(op==`OP_BEQ)    || (op==`OP_BNE)    ||
             (op==`OP_BLEZ)   || (op==`OP_BGTZ)   ||
             (op==`OP_BLTZ)   || (op==`OP_BGEZ)   ||
             (op==`OP_BLTZAL) || (op==`OP_BGEZAL) ||
             (op==`OP_BEQL)   || (op==`OP_BNEL)   ||
             (op==`OP_BLEZL)  || (op==`OP_BGTZL)  ||
             (op==`OP_BLTZL)  || (op==`OP_BGEZL)  ||
             (op==`OP_BLTZALL)|| (op==`OP_BGEZALL)||
             (op==`OP_BC1F)   || (op==`OP_BC1T)   ||
             (op==`OP_BC1FL)  || (op==`OP_BC1TL)  
             //HAVE_DSP_UNIT
             ||(op==`OP_BPOSGE32)
             ;
endmodule     
              
module BR_CONFIG_OP(op,br_op);
input [7:0] op;
output br_op;

wire [7:0]op;
wire br_op;

assign br_op=(op==`OP_BEQ)    || (op==`OP_BNE)    ||
             (op==`OP_BLEZ)   || (op==`OP_BGTZ)   ||
             (op==`OP_BLTZ)   || (op==`OP_BGEZ)   ||
             (op==`OP_BC1F)   || (op==`OP_BC1T)  
             //HAVE_DSP_UNIT
             || (op==`OP_BPOSGE32)
             ;


endmodule                   

module BLIKELY_OP(op,blikely_op);
input [7:0] op;
output blikely_op;

wire [7:0]op;
wire blikely_op;
assign blikely_op=(op==`OP_BEQL)   || (op==`OP_BNEL)   ||
                  (op==`OP_BLEZL)  || (op==`OP_BGTZL)  ||
                  (op==`OP_BLTZL)  || (op==`OP_BGEZL)  ||
                  (op==`OP_BLTZALL)|| (op==`OP_BGEZALL)||
                  (op==`OP_BC1FL)  || (op==`OP_BC1TL) ;


endmodule                   

module STATIC_PREDICT_OP(op,static_pred_op);
input [7:0] op;
output static_pred_op;

wire [7:0]op;
wire static_pred_op;
assign static_pred_op=(op==`OP_BEQL)   || (op==`OP_BNEL)   ||
                  (op==`OP_BLEZL)  || (op==`OP_BGTZL)  ||
                  (op==`OP_BLTZL)  || (op==`OP_BGEZL)  ||
                  (op==`OP_BLTZALL)|| (op==`OP_BGEZALL)||
                  (op==`OP_BC1FL)  || (op==`OP_BC1TL) || 
                  (op==`OP_BLTZAL)|| (op==`OP_BGEZAL);


endmodule                   

module BMX (PP, X2, A, S, M1, M0);
output PP;
input X2, A, S, M1, M0;
wire PP, X2, A, S, M1, M0;
wire abar, sbar, m1bar, m0bar;
wire _and0, _and1, _and2, _or3, _or3bar, _and4, _and5, _and6, _or7, _or7bar;
wire _and8, _mux9, _mux17, _and11, _and12, _or13, _and14, _and15, _or16;
not   I0  (abar,A);
not   I1  (sbar,S);
not   I2  (m1bar,M1);
not   I3  (m0bar,M0);
and   I4  (_and0,1'bX,_mux9);
and   I5  (_and1,S,A);
and   I6  (_and2,S,abar,m1bar);
or    I7  (_or3,_and1,_and2,_and4);
not   I8  (_or3bar,_or3);
and   I9  (_and4,A,sbar,M1);
and   I10 (_and5,S,A);
and   I11 (_and6,S,abar,m0bar);
or    I12 (_or7,_and5,_and6,_and8);
not   I13 (_or7bar,_or7);
and   I14 (_and8,A,sbar,M0);
BMX_mux I15 (.Y(_mux9), .A(_or3bar), .B(_or7bar), .S0(X2));
or    I16 (PP,_and0,_mux17);
and   I17 (_and11,A,sbar,m1bar);
and   I18 (_and12,S,abar,M1);
or    I19 (_or13,_and11,_and12);
and   I20 (_and14,A,sbar,m0bar);
and   I21 (_and15,S,abar,M0);
or    I22 (_or16,_and14,_and15);
BMX_mux I23 (.Y(_mux17), .A(_or13), .B(_or16), .S0(X2));
endmodule // BMXX1

module BMX_mux (Y, A, B, S0);
output Y;
input A, B, S0;
wire Y, A, B, S0;

assign Y=((S0 & B) | ((!S0) & A) | (A & B));
endmodule // MX2X1

module change_order_32(in,out);
input  [31:0] in;
output [31:0] out;
                                                                                                                              
assign out = {in[ 0],in[ 1],in[ 2],in[ 3],in[ 4],in[ 5],in[ 6],in[ 7],in[ 8],in[ 9],
              in[10],in[11],in[12],in[13],in[14],in[15],in[16],in[17],in[18],in[19],
              in[20],in[21],in[22],in[23],in[24],in[25],in[26],in[27],in[28],in[29],
              in[30],in[31]};
endmodule

module first_one_4_2(in,out,nonzero);
input  [3:0] in;
output [1:0] out;
output nonzero;
 
assign out[1] = (~in[0])&(~in[1]) & (in[2] | in[3]);
assign out[0] = ((~in[0])&( in[1])) | ((~in[0])&(~in[2])&( in[3]));
assign nonzero   = |in[3:0];
endmodule
 
module first_one_8_3(in,out,nonzero);
input  [7:0] in;
output [2:0] out;
output nonzero;
 
assign out[2]=(~(|in[3:0]))&(|in[7:4]);
assign out[1]=((~in[0]) & (~in[1]) & (~in[4]) & (~in[5]) & (in[6] | in[7])) |
              ((~in[0]) & (~in[1]) & (in[2] | in[3]));
assign out[0]=((~in[0]) & (~in[2]) & (~in[4]) & (~in[6]) & in[7])|
              ((~in[0]) & (~in[2]) & (~in[4]) & in[5])|
              ((~in[0]) & (~in[2]) & in[3])|
              ((~in[0]) & in[1]);
assign nonzero=|in[7:0];
endmodule



module first_one_16_4(in,out,nonzero);
input  [15:0] in;
output [3:0] out;
output nonzero;
 
wire [1:0] a3,a2,a1,a0;
wire [3:0] nz;
wire zero;
 
first_one_4_2 first_one_4_2_3(.in(in[15:12]),.out(a3),.nonzero(nz[3]));
first_one_4_2 first_one_4_2_2(.in(in[11: 8]),.out(a2),.nonzero(nz[2]));
first_one_4_2 first_one_4_2_1(.in(in[ 7: 4]),.out(a1),.nonzero(nz[1]));
first_one_4_2 first_one_4_2_0(.in(in[ 3: 0]),.out(a0),.nonzero(nz[0]));

first_one_4_2 first_one4(.in(nz),.out(out[3:2]),.nonzero(nonzero));

assign out[1:0]= a0 |
                (a1 & {2{~nz[0]}}) |
                (a2 & {2{~|nz[1:0]}}) |
                (a3 & {2{~|nz[2:0]}});
endmodule
 

module first_one_32_5(in,out,nonzero);
input  [31:0] in;
output [4:0] out;
output nonzero;

wire [2:0] a3,a2,a1,a0;
wire [3:0] nz;
wire zero;

first_one_8_3 first_one3(.in(in[31:24]),.out(a3),.nonzero(nz[3]));
first_one_8_3 first_one2(.in(in[23:16]),.out(a2),.nonzero(nz[2]));
first_one_8_3 first_one1(.in(in[15: 8]),.out(a1),.nonzero(nz[1]));
first_one_8_3 first_one0(.in(in[ 7: 0]),.out(a0),.nonzero(nz[0]));

first_one_4_2 first_one4(.in(nz),.out(out[4:3]),.nonzero(nonzero));

assign out[2:0]= a0 |
                (a1 & {3{~nz[0]}})    |
                (a2 & {3{~|nz[1:0]}}) |
                (a3 & {3{~|nz[2:0]}});
endmodule

module CSA3_2(in,s,c);
input[2:0]      in;
output          s,c;
                                                                                                                              
assign  s = in[0]^in[1]^in[2];
assign  c = (in[0]&in[1]) | (in[0]&in[2]) | (in[1]&in[2]);
endmodule

module brq_tail_vector(brqid, vec);
input  [2:0]  brqid;
output [5:0] vec;
reg    [5:0] vec;
always@(brqid)
case(brqid)
 3'h0 : vec = 6'h01;
 3'h1 : vec = 6'h03;
 3'h2 : vec = 6'h07;
 3'h3 : vec = 6'h0f;
 3'h4 : vec = 6'h1f;
 default:
        vec = 6'h3f;
endcase
endmodule

module brq_head_vector(brqid, vec);
input  [2:0]  brqid;
output [5:0] vec;
reg    [5:0] vec;
always@(brqid)
case(brqid)
 3'h0 : vec = 6'h00;
 3'h1 : vec = 6'h01;
 3'h2 : vec = 6'h03;
 3'h3 : vec = 6'h07;
 3'h4 : vec = 6'h0f;
 default:
        vec = 6'h1f;
endcase
endmodule

module wait_operate(op, vector);
input [7:0] op;
output vector;
assign vector =  (op ==`OP_SYNC) ||
                 (op ==`OP_CTC1) || (op ==`OP_CFC1) ||
                 (op ==`OP_INS)
                 //HAVE_DSP_UNIT 
                  ||(op==`OP_PICK)||(op ==`OP_RDDSP)
                  ||(op==`OP_ADDWC)||(op==`OP_CMP_LT)||(op==`OP_CMP_EQ)||(op==`OP_CMP_LE)
                  ||(op==`OP_WRDSP)||(op==`OP_EXTPDP)||(op==`OP_MTHLIP)||(op==`OP_EXTP)||(op==`OP_RDDSP) 
                  ||(op ==`OP_INSV)||(op ==`OP_BPOSGE32)
                 ; 

endmodule

module ctc1_operate(op, ctc1);
input [7:0] op;
output ctc1;

assign ctc1 = (op==`OP_CTC1)   || (op == `OP_MOVN)  || (op == `OP_MOVZ)|| (op == `OP_MOVT) || (op == `OP_MOVF)||
              (op == `OP_FMOVN)|| (op == `OP_FMOVZ) || (op == `OP_FMOVT)||(op == `OP_FMOVF)
              //HAVE_DSP_UNIT
              ||(op == `OP_WRDSP)|| (op == `OP_MTHLIP)|| (op == `OP_ADDSC)
              ;

endmodule

module nonzero_low_16(in,out);
input  [15:0] in;
output [15:0] out;

assign out[0] = in[0]; 
assign out[1] = |in[1:0]; 
assign out[2] = |in[2:0]; 
assign out[3] = |in[3:0]; 
assign out[4] = |in[4:0]; 
assign out[5] = |in[5:0]; 
assign out[6] = |in[6:0]; 
assign out[7] = |in[7:0]; 
assign out[8] = |in[8:0]; 
assign out[9] = |in[9:0]; 
assign out[10] = |in[10:0]; 
assign out[11] = |in[11:0]; 
assign out[12] = |in[12:0]; 
assign out[13] = |in[13:0]; 
assign out[14] = |in[14:0]; 
assign out[15] = |in[15:0]; 
endmodule

module first_one_16_16(in,out,has);
input  [15:0] in;
output [15:0] out;
output has;

assign out[ 0] = in[ 0];
assign out[ 1] = in[ 1] & (~in[0]);
assign out[ 2] = in[ 2] & (~|in[ 1:0]);
assign out[ 3] = in[ 3] & (~|in[ 2:0]);
assign out[ 4] = in[ 4] & (~|in[ 3:0]);
assign out[ 5] = in[ 5] & (~|in[ 4:0]);
assign out[ 6] = in[ 6] & (~|in[ 5:0]);
assign out[ 7] = in[ 7] & (~|in[ 6:0]);
assign out[ 8] = in[ 8] & (~|in[ 7:0]);
assign out[ 9] = in[ 9] & (~|in[ 8:0]);
assign out[10] = in[10] & (~|in[ 9:0]);
assign out[11] = in[11] & (~|in[10:0]);
assign out[12] = in[12] & (~|in[11:0]);
assign out[13] = in[13] & (~|in[12:0]);
assign out[14] = in[14] & (~|in[13:0]);
assign out[15] = in[15] & (~|in[14:0]);

assign has = |in[15:0];

endmodule

module first_one_6_6(in,out,has);
input  [5:0] in;
output [5:0] out;
output has;

assign out[ 0] = in[ 0];
assign out[ 1] = in[ 1] & (~in[0]);
assign out[ 2] = in[ 2] & (~|in[ 1:0]);
assign out[ 3] = in[ 3] & (~|in[ 2:0]);
assign out[ 4] = in[ 4] & (~|in[ 3:0]);
assign out[ 5] = in[ 5] & (~|in[ 4:0]);

assign has = |in[5:0];
endmodule

module brq_start_vector(start_brqid, vec);
input  [2:0]  start_brqid;
output [5:0] vec;
reg    [5:0] vec;
always@(start_brqid)
case(start_brqid)
  3'h0 : vec =  16'h3e ; 
  3'h1 : vec =  16'h3c ; 
  3'h2 : vec =  16'h38 ; 
  3'h3 : vec =  16'h30 ; 
  3'h4 : vec =  16'h20 ; 
  default :
         vec =  16'h00 ; 
endcase
endmodule

module ALU_ONE(op,alu_one);
input  [7:0] op;
output alu_one;

wire mul_op= (op==`OP_MULT)||(op==`OP_MULTU)||(op==`OP_MUL)||
             (op==`OP_MADD)||(op==`OP_MADDU)||
             (op==`OP_MSUB)||(op==`OP_MSUBU)||
             (op==`OP_DIV) ||(op==`OP_DIVU) || 
             (op==`OP_MFHI)||(op==`OP_MFLO) ||(op==`OP_MTHI)||(op==`OP_MTLO)
             ;
//HAVE_DSP_UNIT
wire dsp_mul = (op==`OP_MULEU_S_PH_QBL) || (op==`OP_MULEU_S_PH_QBR) || (op==`OP_MULQ_RS_PH) ||
               (op==`OP_MULEQ_S_W_PHL)  || (op==`OP_MULEQ_S_W_PHR)  || (op==`OP_MAQ_SA_W_PHL) ||
               (op==`OP_MAQ_SA_W_PHR)   || (op==`OP_MAQ_S_W_PHL)    || (op==`OP_MAQ_S_W_PHR) || 
               (op==`OP_MULSAQ_S_W_PH)  || (op==`OP_DPAU_H_QBL)     || (op==`OP_DPAU_H_QBR) ||
               (op==`OP_DPAQ_S_W_PH)    || (op==`OP_DPAQ_SA_L_W)    || (op==`OP_DPSU_H_QBL) || 
               (op==`OP_DPSU_H_QBR)     || (op==`OP_DPSQ_S_W_PH)    || (op==`OP_DPSQ_SA_L_W) ||
               (op==`OP_SHILO) || (op==`OP_MTHLIP) ||
               (op==`OP_EXTP) || (op==`OP_EXTPDP) || (op==`OP_EXTR_W) || (op==`OP_EXTR_R_W) ||
               (op==`OP_EXTR_RS_W) || (op==`OP_EXTR_S_H);   

assign alu_one = (op[7:6]==2'b00)&&!mul_op
                 //HAVE_DSP_UNIT
                  || (op[7:6]==2'b11)&&!dsp_mul
                  ;


endmodule
