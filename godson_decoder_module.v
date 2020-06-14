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
module godson_decoder_module(
    reset,
    commitbus_ex,
    exl,
    erl,
    ksu,
    cu,
    entryhi_vpn2,
    HWREna,
    irbus,
    DEBUG_MODE,
    decbus
);

input  reset;
input  exl;
input  erl;
input  [1:0] ksu;
input  [3:0] cu;
input  [18:0] entryhi_vpn2;
input  [3:0] HWREna;
input  commitbus_ex;
input  [`Lirbus_2issue-1:0] irbus;
input  DEBUG_MODE;
output [`Ldecbus_2issue-1:0] decbus;

wire [`Lirbus-1:0] irbus_0; 
wire [`Lirbus-1:0] irbus_1; 

wire [`Ldecbus-1:0] decbus_0; 
wire [`Ldecbus-1:0] decbus_1; 

assign irbus_0 = irbus[118:0];
assign irbus_1 = irbus[237:119];


assign decbus[`Ldecbus-1:0] = decbus_0;
assign decbus[`Ldecbus_2issue-1:`Ldecbus] = decbus_1;

inst_decoder decoder_inst_0(.reset(reset),.commitbus_ex(commitbus_ex),
                            .exl(exl),.erl(erl),.ksu(ksu),.cu(cu),.HWREna(HWREna),
                            .entryhi_vpn2(entryhi_vpn2),
                            .irbus(irbus_0),.DEBUG_MODE(DEBUG_MODE),.decbus(decbus_0)
                            );

inst_decoder decoder_inst_1(.reset(reset),.commitbus_ex(commitbus_ex),
                            .exl(exl),.erl(erl),.ksu(ksu),.cu(cu),.HWREna(HWREna),
                            .entryhi_vpn2(entryhi_vpn2),
                            .irbus(irbus_1),.DEBUG_MODE(DEBUG_MODE),.decbus(decbus_1)
                            );

endmodule

module inst_decoder(reset,commitbus_ex, exl,erl,ksu,cu,HWREna,entryhi_vpn2,irbus,DEBUG_MODE,decbus);

input  reset; 
input  exl;
input  erl;
input  [1:0] ksu;
input  [3:0] cu;
input  [3:0] HWREna;
input  [18:0] entryhi_vpn2;
input  commitbus_ex;
input  [`Lirbus-1:0] irbus;
input  DEBUG_MODE;
output [`Ldecbus-1:0] decbus;

wire irbus_valid;
wire [`Lword-1:0] irbus_pc;    
wire [`Lword-1:0] irbus_inst;
wire irbus_bd;
wire irbus_adei;
wire irbus_tlbii;
wire irbus_tlbir;
wire irbus_ibe;        
wire irbus_dib;    
wire [`Lword-1:0] irbus_taken_addr; //taken target of the common branch instruction 
wire [1:0] irbus_pred_status;      //the prediction status of the common branch instruction 
wire [7:0] irbus_gshare;
wire [1:0] irbus_rashead;
wire [1:0] irbus_other_link;
wire       irbus_block_begin;
wire       irbus_int;

wire decbus_valid;
wire [7:0] decbus_op;
wire [4:0] decbus_fmt;
wire [7:0] decbus_src1;
wire [7:0] decbus_src2;
wire [7:0] decbus_dest;
wire [`Lword-1:0] decbus_imm;
wire [`Lword-1:0] decbus_pc;
wire [1:0] decbus_ce;
wire decbus_bd;       
wire decbus_adei;
wire decbus_tlbii;
wire decbus_tlbir;
wire decbus_ibe;
wire decbus_ri;
wire decbus_cpui;
wire decbus_sys;
wire decbus_bp;                        
wire decbus_dib;                        
wire decbus_sdbbp;                        
wire [1:0] decbus_ac;
wire       decbus_nop;
wire decbus_double_src1;
wire decbus_double_src2;
wire decbus_double_dest;

wire kernel;
wire superuser;
wire user;

assign irbus_dib         = irbus[70];
assign irbus_valid       = irbus[69];
assign irbus_pc[31:0]    = irbus[68:37];
assign irbus_inst[31:0]  = irbus[36:5];
assign irbus_bd          = irbus[4];  
assign irbus_adei        = irbus[3];
assign irbus_tlbii       = irbus[2];
assign irbus_tlbir       = irbus[1];
assign irbus_ibe         = irbus[0];
assign irbus_taken_addr  = irbus[102:71];
assign irbus_pred_status = irbus[104:103];
assign irbus_gshare      = irbus[112:105];
assign irbus_rashead     = irbus[114:113];
assign irbus_other_link  = irbus[116:115];
assign irbus_block_begin = irbus[117];
assign irbus_int         = irbus[118];
        
wire [5:0]op;   /*bits 31-26 of inst*/
wire [4:0]rs;   /*bits 25-21 of inst*/
wire [4:0]rt;   /*bits 20-16 of inst*/
wire [4:0]rd;   /*bits 16-11 of inst*/
wire [4:0]sa;   /*bits 10- 6 of inst*/
wire [5:0]func; /*bits  5- 0 of inst*/
wire [15:0]imm;  /*bits 15- 0 of inst*/
wire [25:0]target;  /*bits 25-0 of inst*/
wire [2:0] cp0_sel;  /*bits 2 -0 of inst*/
wire [2:0] cop1x_fmt;  /*bits 2 -0 of inst*/
wire [2:0] cop1x_func;  /*bits 5 -3 of inst*/
wire [3:0] sepcial_rotate; /*bits 25-22 of inst*/
wire [3:0] sepcial_rotatev; /*bits 10-7 of inst*/


wire [7:0]dec_opbit;
wire [63:0]op_d;
wire [31:0]rs_d;
wire [31:0]rt_d;
wire [31:0]rd_d;
wire [31:0]sa_d;
wire [63:0]func_d;
wire [7:0]cop1x_fmt_d;
wire [7:0]cop1x_func_d;
    
wire decsrc1_rs;
wire decsrc1_rt;
wire decsrc1_cmp;
wire decsrc1_fs; 
//wire decsrc1_ac;
wire decsrc1_imm;
wire decsrc1_DSPCtl;

wire decsrc2_rs;
wire decsrc2_rt;
wire decsrc2_imm;
wire decsrc2_ft;   
wire decsrc2_cmp;

wire decdest_rt;
wire decdest_rd;
wire decdest_gr31;
wire decdest_fd;
wire decdest_fs;
wire decdest_fcr;
wire decdest_fcr31; 
//wire decdest_ac;
wire decdest_DSPcc;

wire decimm_immrl;
wire decimm_immra;
wire decimm_immll;
wire decimm_offset;      
wire decimm_sa;     
wire decimm_rd;     
wire decimm_pc;
wire decimm_entryhi;
wire decimm_rd_sa;
wire decimm_status;
//for DSP
wire decimm_rs;
wire decimm_rt_rd;
wire decimm_rs_rt1;

wire [31:0] temp=32'b0;

assign  op[5:0]=irbus_inst[31:26];   
assign  rs[4:0]=irbus_inst[25:21];   
assign  rt[4:0]=irbus_inst[20:16];   
assign  rd[4:0]=irbus_inst[15:11];   
assign  sa[4:0]=irbus_inst[10:6];   
assign  func[5:0]=irbus_inst[5:0]; 
assign  imm[15:0]=irbus_inst[15:0];  
assign  target[25:0]=irbus_inst[25:0];  
assign  cp0_sel[2:0]=irbus_inst[2:0];   
assign  cop1x_fmt[2:0]=irbus_inst[2:0]; 
assign  cop1x_func[2:0]=irbus_inst[5:3];

decoder_6_64
    dec6to641(.in(op[5:0]),.out(op_d[63:0]));
decoder_6_64
    dec6to642(.in(func[5:0]),.out(func_d[63:0]));
decoder_5_32
    dec21(.in(rs[4:0]),.out(rs_d[31:0]));
decoder_5_32
    dec5to322(.in(rt[4:0]),.out(rt_d[31:0]));
decoder_5_32
    dec5to323(.in(rd[4:0]),.out(rd_d[31:0]));
decoder_5_32
    dec5to324(.in(sa[4:0]),.out(sa_d[31:0]));
decoder_3_8
        dec3to81(.in(cop1x_func[2:0]),.out(cop1x_func_d[7:0]));
decoder_3_8
        dec3to82(.in(cop1x_fmt[2:0]),.out(cop1x_fmt_d[7:0]));

/*************************************fix OP **************************/
wire inst_NOP    =op_d[6'h0]&func_d[6'h0]&rs_d[5'h0]&rt_d[5'h0]&sa_d[5'h0];

wire inst_ADD    =op_d[6'h0]&sa_d[5'h0]&func_d[6'h20];

wire inst_ADDI   =op_d[6'h08];

wire inst_ADDIU  =op_d[6'h09];

wire inst_ADDU   =op_d[6'h0]&sa_d[5'h0]&func_d[6'h21];        

wire inst_AND    =op_d[6'h0]&sa_d[5'h0]&func_d[6'h24];

wire inst_ANDI   =op_d[6'h0c];        

wire inst_BEQ    =op_d[6'h04];

wire inst_BEQL   =op_d[6'h14];    

wire inst_BGEZ   =op_d[6'h01]&rt_d[5'h01];        

wire inst_BGEZL  =op_d[6'h01]&rt_d[5'h03];    

wire inst_BGEZAL =op_d[6'h01]&rt_d[5'h11];    

wire inst_BGEZALL=op_d[6'h01]&rt_d[5'h13];        

wire inst_BGTZ   =op_d[6'h07]&rt_d[5'h0];        

wire inst_BGTZL  =op_d[6'h17]&rt_d[5'h0];

wire inst_BLEZ   =op_d[6'h06]&rt_d[5'h0];

wire inst_BLEZL  =op_d[6'h16]&rt_d[5'h0];

wire inst_BLTZ   =op_d[6'h01]&rt_d[5'h0];

wire inst_BLTZAL =op_d[6'h01]&rt_d[5'h10];

wire inst_BLTZALL=op_d[6'h01]&rt_d[5'h12];

wire inst_BLTZL  =op_d[6'h01]&rt_d[5'h02];

wire inst_BNE    =op_d[6'h05];
    
wire inst_BNEL   =op_d[6'h15];

wire  inst_BREAK =op_d[6'h0]&func_d[6'h0d];
    
wire  inst_SDBBP =op_d[6'h1c]&func_d[6'h3f];

wire  inst_J=op_d[6'h02];
    
wire  inst_JAL=op_d[6'h03];

wire  inst_JALR=op_d[6'h0]&rt_d[5'h0]&sa_d[5'h0]&func_d[6'h09];

wire  inst_JR=op_d[6'h0]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h08];
    
wire  inst_LUI=op_d[6'h0f]&rs_d[5'h0];

//HAVE_DSP_UNIT
 wire inst_MFLO_DSP        = op_d[6'h00] & sa_d[5'h00] & func_d[6'h12] & (rs[4:2]==3'b00) & rt_d[5'h00];
 wire inst_MFHI_DSP        = op_d[6'h00] & sa_d[5'h00] & func_d[6'h10] & (rs[4:2]==3'b00) & rt_d[5'h00];
 wire inst_MTLO_DSP        = op_d[6'h00] & sa_d[5'h00] & func_d[6'h13] & (rd[4:2]==3'b00) & rt_d[5'h00];
 wire inst_MTHI_DSP        = op_d[6'h00] & sa_d[5'h00] & func_d[6'h11] & (rd[4:2]==3'b00) & rt_d[5'h00];
 wire inst_MFLO            = 1'b0;
 wire inst_MFHI            = 1'b0;
 wire inst_MTLO            = 1'b0;
 wire inst_MTHI            = 1'b0;
wire  inst_MUL=op_d[6'h1c]&sa_d[5'h00]&func_d[6'h02];//add 
    
wire  inst_MULT=op_d[6'h00]&rd_d[5'h0]&sa_d[5'h00]&func_d[6'h18];

wire  inst_MULTU=op_d[6'h00]&rd_d[5'h0]&sa_d[5'h00]&func_d[6'h19];

wire  inst_MADD=op_d[6'h1c]&rd_d[5'h0]&sa_d[5'h00]&func_d[6'h00];//add

wire  inst_MADDU=op_d[6'h1c]&rd_d[5'h0]&sa_d[5'h00]&func_d[6'h01];//add

wire  inst_MSUB=op_d[6'h1c]&rd_d[5'h0]&sa_d[5'h00]&func_d[6'h04];//add

wire  inst_MSUBU=op_d[6'h1c]&rd_d[5'h0]&sa_d[5'h00]&func_d[6'h05];//add

wire  inst_CLO=op_d[6'h1c]&sa_d[5'h00]&func_d[6'h21];//add 
    
wire  inst_CLZ=op_d[6'h1c]&sa_d[5'h0]&func_d[6'h20]; //add

wire  inst_MOVZ=op_d[6'h00]&sa_d[5'h00]&func_d[6'h0a];//add

wire  inst_MOVN=op_d[6'h00]&sa_d[5'h00]&func_d[6'h0b];//add

wire  inst_NOR=op_d[6'h00]&sa_d[5'h00]&func_d[6'h27];

wire  inst_OR=op_d[6'h00]&sa_d[5'h00]&func_d[6'h25];

wire  inst_ORI=op_d[6'h0d];

wire  inst_SLL=op_d[6'h00]&rs_d[5'h0]&func_d[6'h0];

wire  inst_SLLV=op_d[6'h0]&sa_d[5'h0]&func_d[6'h04];

wire  inst_SLT=op_d[6'h0]&sa_d[5'h0]&func_d[6'h2a];

wire  inst_SLTI=op_d[6'h0a];

wire  inst_SLTIU=op_d[6'h0b];

wire  inst_SLTU=op_d[6'h0]&sa_d[5'h0]&func_d[6'h2b];

wire  inst_SRA=op_d[6'h0]&rs_d[5'h0]&func_d[6'h03];

wire  inst_SRAV=op_d[6'h0]&sa_d[5'h0]&func_d[6'h07];

wire  inst_SRL=op_d[6'h0]&rs_d[5'h0]&func_d[6'h02];

wire  inst_SRLV=op_d[6'h0]&sa_d[5'h0]&func_d[6'h06];

wire  inst_SUB=op_d[6'h0]&sa_d[5'h0]&func_d[6'h22];

wire  inst_SUBU=op_d[6'h0]&sa_d[5'h0]&func_d[6'h23];

wire  inst_SYSCALL=op_d[6'h0]&func_d[6'h0c];

wire  inst_TEQ=op_d[6'h0]&func_d[6'h34];

wire  inst_TEQI=op_d[6'h01]&rt_d[5'h0c];

wire  inst_TGE=op_d[6'h0]&func_d[6'h30];

wire  inst_TGEI=op_d[6'h01]&rt_d[5'h08];

wire  inst_TGEIU=op_d[6'h01]&rt_d[5'h09];

wire  inst_TGEU=op_d[6'h0]&func_d[6'h31];

wire  inst_TLT=op_d[6'h0]&func_d[6'h32];

wire  inst_TLTI=op_d[6'h01]&rt_d[5'h0a];

wire  inst_TLTIU=op_d[6'h01]&rt_d[5'h0b];

wire  inst_TLTU=op_d[6'h0]&func_d[6'h33];

wire  inst_TNE=op_d[6'h0]&func_d[6'h36];

wire  inst_TNEI=op_d[6'h01]&rt_d[5'h0e];

wire  inst_XOR=op_d[6'h0]&sa_d[5'h0]&func_d[6'h26];

wire  inst_XORI=op_d[6'h0e];

wire  inst_DIV=op_d[6'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h1a];
        
wire  inst_DIVU=op_d[6'h00]&rd_d[5'h00]&sa_d[5'h00]&func_d[6'h1b];
    
/*cpu instructions of release2 */
wire  inst_SEB = op_d[6'h1f] & rs_d[5'h0] & sa_d[5'h10] & func_d[6'h20];

wire  inst_SEH = op_d[6'h1f] & rs_d[5'h0] & sa_d[5'h18] & func_d[6'h20];

wire  inst_EXT = op_d[6'h1f]  & func_d[6'h0];

wire  inst_INS = op_d[6'h1f]  & func_d[6'h4];

wire  inst_WSBH = op_d[6'h1f] & rs_d[5'h0] & sa_d[5'h2] & func_d[6'h20];

wire  inst_RDHWR = op_d[6'h1f] & sa_d[5'h0] & func_d[6'h3b] &rs_d[5'h0] ;

wire  inst_ROTR = op_d[6'h0] &  rs_d[5'h1]  & func_d[6'h2];

wire  inst_ROTRV = op_d[6'h0] &  sa_d[5'h1] & func_d[6'h6] ;

wire  inst_JALR_HB= op_d[6'h0] & rt_d[5'h0] & sa_d[5'h10] & func_d[6'h09];

wire  inst_JR_HB =  op_d[6'h0] & rt_d[5'h0] & rd_d[5'h0] &sa_d[5'h10] & func_d[6'h08]; 

/*CP0 instructions*/
wire  inst_SYNCI = op_d[6'h01] & rt_d[5'h1f];  //release2

wire  inst_PREF=op_d[6'h33];

wire  inst_PREFX=op_d[6'h13] & sa_d[5'h00] & func_d[6'h0f];

wire  inst_MFC1  =op_d[6'h11]&&rs_d[5'h00]&&sa_d[5'h00]&&func_d[6'h00];

wire  inst_MTC1  =op_d[6'h11]&rs_d[5'h04]&sa_d[5'h0]&func_d[6'h0];

wire  inst_CFC1  =op_d[6'h11]&&rs_d[5'h02]&&sa_d[5'h00]&&func_d[6'h0];

wire  inst_CTC1  =op_d[6'h11]&rs_d[5'h06]&sa_d[5'h00]&func_d[6'h00];

wire  inst_CACHE0=op_d[6'h2f]&rt_d[5'h00];

wire  inst_CACHE1=op_d[6'h2f]&rt_d[5'h01];

wire  inst_CACHE2=op_d[6'h2f]&rt_d[5'h02];

wire  inst_CACHE3=op_d[6'h2f]&rt_d[5'h03];

wire  inst_CACHE4=op_d[6'h2f]&rt_d[5'h04];

wire  inst_CACHE5=op_d[6'h2f]&rt_d[5'h05];

wire  inst_CACHE6=op_d[6'h2f]&rt_d[5'h06];

wire  inst_CACHE7=op_d[6'h2f]&rt_d[5'h07];

wire  inst_CACHE8=op_d[6'h2f]&rt_d[5'h08];

wire  inst_CACHE9=op_d[6'h2f]&rt_d[5'h09];

wire  inst_CACHE10=op_d[6'h2f]&rt_d[5'h0a];

wire  inst_CACHE11=op_d[6'h2f]&rt_d[5'h0b];

wire  inst_CACHE12=op_d[6'h2f]&rt_d[5'h0c];

wire  inst_CACHE13=op_d[6'h2f]&rt_d[5'h0d];

wire  inst_CACHE14=op_d[6'h2f]&rt_d[5'h0e];

wire  inst_CACHE15=op_d[6'h2f]&rt_d[5'h0f];
    
wire  inst_CACHE16=op_d[6'h2f]&rt_d[5'h10];
    
wire  inst_CACHE17=op_d[6'h2f]&rt_d[5'h11];

wire  inst_CACHE18=op_d[6'h2f]&rt_d[5'h12];

wire  inst_CACHE19=op_d[6'h2f]&rt_d[5'h13];

wire  inst_CACHE20=op_d[6'h2f]&rt_d[5'h14];

wire  inst_CACHE21=op_d[6'h2f]&rt_d[5'h15];

wire  inst_CACHE22=op_d[6'h2f]&rt_d[5'h16];

wire  inst_CACHE23=op_d[6'h2f]&rt_d[5'h17];

wire  inst_CACHE24=op_d[6'h2f]&rt_d[5'h18];

wire  inst_CACHE25=op_d[6'h2f]&rt_d[5'h19];

wire  inst_CACHE26=op_d[6'h2f]&rt_d[5'h1a];

wire  inst_CACHE27=op_d[6'h2f]&rt_d[5'h1b];

wire  inst_CACHE28=op_d[6'h2f]&rt_d[5'h1c];

wire  inst_CACHE29=op_d[6'h2f]&rt_d[5'h1d];

wire  inst_CACHE30=op_d[6'h2f]&rt_d[5'h1e];

wire  inst_CACHE31=op_d[6'h2f]&rt_d[5'h1f];

wire  inst_ERET=op_d[6'h10]&rs_d[5'h10]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h18];

wire  inst_DERET=op_d[6'h10]&rs_d[5'h10]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h1f];

wire  inst_LB=op_d[6'h20];
    
wire  inst_LBU=op_d[6'h24];

wire  inst_LDC1=op_d[6'h35];

wire  inst_LH=op_d[6'h21];
    
wire  inst_LHU=op_d[6'h25];
    
wire  inst_LL=op_d[6'h30];

wire  inst_LW=op_d[6'h23];
    
wire  inst_LWC1=op_d[6'h31];

wire  inst_LWL=op_d[6'h22];
    
wire  inst_LWR=op_d[6'h26];
    
    
wire  inst_MFC0 =op_d[6'h10]&&rs_d[5'h00]&&sa_d[5'h0]&&(func[5:3]==3'b0);

wire  inst_MTC0 =op_d[6'h10]&&rs_d[5'h04]&&sa_d[5'h0]&&(func[5:3]==3'b0);

wire  inst_SB=op_d[6'h28];
    
wire  inst_SC=op_d[6'h38];

wire  inst_SDC1=op_d[6'h3d];    

wire  inst_SH=op_d[6'h29];

wire  inst_SW=op_d[6'h2b];

wire  inst_SWC1=op_d[6'h39];
    
wire  inst_SWL=op_d[6'h2a];

wire  inst_SWR=op_d[6'h2e];

wire  inst_SYNC=op_d[6'h0]&rs_d[5'h0]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h0f];

wire  inst_TLBP=op_d[6'h10]&rs_d[5'h10]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h08];
    
wire  inst_TLBR=op_d[6'h10]&rs_d[5'h10]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h01];

wire  inst_TLBWI=op_d[6'h10]&rs_d[5'h10]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h02];

wire  inst_TLBWR=op_d[6'h10]&rs_d[5'h10]&rt_d[5'h0]&rd_d[5'h0]&sa_d[5'h0]&func_d[6'h06];

wire  inst_WAIT =  op_d[6'h10] &func_d[6'h20] & rs[4];

/*release2 's cp0 instruction*/
wire  inst_DI = op_d[6'h10] & rs_d[5'h0b] & rd_d[5'h0c] & sa_d[5'h0] & func_d[6'h0];

wire  inst_EI = op_d[6'h10] & rs_d[5'h0b] & rd_d[5'h0c] & sa_d[5'h0] & func_d[6'h20];

wire  inst_RDPGPR = op_d[6'h10] & rs_d[5'ha] & sa_d[5'h0] & func_d[6'h0];

wire  inst_WRPGPR = op_d[6'h10] & rs_d[5'he] & sa_d[5'h0] & func_d[6'h0];

//DSP instrution
wire  inst_LBUX = op_d[6'h1f] & sa_d[5'h06] & func_d[6'h0a];  
wire  inst_LHX  = op_d[6'h1f] & sa_d[5'h04] & func_d[6'h0a];  
wire  inst_LWX  = op_d[6'h1f] & sa_d[5'h00] & func_d[6'h0a];  

/*FP insts*/
wire sa_c= (sa[1:0]==2'b00);

wire  inst_MOVF=op_d[6'h00]&&sa_d[5'h00]&&func_d[6'h01]&&(rt[1:0]==2'b00);

wire  inst_MOVT=op_d[6'h00]&&sa_d[5'h00]&&func_d[6'h01]&&(rt[1:0]==2'b01);
 
wire inst_BC1F   =op_d[6'h11]&&rs_d[5'h08]&&(!rt[1]&&!rt[0]);

wire inst_BC1FL  =op_d[6'h11]&&rs_d[5'h08]&&( rt[1]&&!rt[0]);

wire inst_BC1T   =op_d[6'h11]&&rs_d[5'h08]&&(!rt[1]&& rt[0]);

wire inst_BC1TL  =op_d[6'h11]&&rs_d[5'h08]&&( rt[1]&& rt[0]);

wire inst_FMOVT   =op_d[6'h11]&&(rt[1:0]==2'b01)&&func_d[6'h11]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);

wire inst_FMOVF   =op_d[6'h11]&&(rt[1:0]==2'b00)&&func_d[6'h11]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);

wire inst_FMOVN   =op_d[6'h11]&&func_d[6'h13]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);

wire inst_FMOVZ   =op_d[6'h11]&&func_d[6'h12]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);

wire inst_ABS_F   =op_d[6'h11]&&rt_d[5'h0]&&func_d[6'h05]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_ADD_F   =op_d[6'h11]&&func_d[6'h00]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);

wire inst_C_F     =op_d[6'h11]&&sa_c&&func_d[6'h30]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_SF    =op_d[6'h11]&&sa_c&&func_d[6'h38]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_UN    =op_d[6'h11]&&sa_c&&func_d[6'h31]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_NGLE  =op_d[6'h11]&&sa_c&&func_d[6'h39]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_EQ    =op_d[6'h11]&&sa_c&&func_d[6'h32]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_SEQ   =op_d[6'h11]&&sa_c&&func_d[6'h3a]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_UEQ   =op_d[6'h11]&&sa_c&&func_d[6'h33]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_NGL   =op_d[6'h11]&&sa_c&&func_d[6'h3b]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_OLT   =op_d[6'h11]&&sa_c&&func_d[6'h34]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_LT    =op_d[6'h11]&&sa_c&&func_d[6'h3c]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_ULT   =op_d[6'h11]&&sa_c&&func_d[6'h35]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_NGE   =op_d[6'h11]&&sa_c&&func_d[6'h3d]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_OLE   =op_d[6'h11]&&sa_c&&func_d[6'h36]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_LE    =op_d[6'h11]&&sa_c&&func_d[6'h3e]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_ULE   =op_d[6'h11]&&sa_c&&func_d[6'h37]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_C_NGT   =op_d[6'h11]&&sa_c&&func_d[6'h3f]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);

wire inst_CEIL_L  =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h0a]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_CEIL_W  =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h0e]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_CVT_D   =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h21]&&(rs_d[5'h10]||rs_d[5'h14]||rs_d[5'h15]);
wire inst_CVT_L   =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h25]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_CVT_S   =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h20]&&(rs_d[5'h11]||rs_d[5'h14]||rs_d[5'h15]);
wire inst_CVT_W   =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h24]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_DIV_F   =op_d[6'h11]&&func_d[6'h03]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_FLOOR_L =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h0b]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_FLOOR_W =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h0f]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_MOVE    =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h06]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_MUL_F   =op_d[6'h11]&&func_d[6'h02]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_NEG_F   =op_d[6'h11]&&func_d[6'h07]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_ROUND_L =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h08]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_ROUND_W =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h0c]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_SQRT_F  =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h04]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_SUB_F   =op_d[6'h11]&&func_d[6'h01]&&(rs_d[5'h10]||rs_d[5'h11]||rs_d[5'h16]);
wire inst_TRUNC_L =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h09]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_TRUNC_W =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h0d]&&(rs_d[5'h10]||rs_d[5'h11]);


wire inst_RECIP   =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h15]&&(rs_d[5'h10]||rs_d[5'h11]);
wire inst_RSQRT   =op_d[6'h11]&&rt_d[5'h00]&&func_d[6'h16]&&(rs_d[5'h10]||rs_d[5'h11]);

//HAVE_DSP_UNIT
 wire inst_ADDQ_PH        = op_d[6'h1f] & sa_d[5'h0a] & func_d[6'h10];
 wire inst_ADDQ_S_PH      = op_d[6'h1f] & sa_d[5'h0e] & func_d[6'h10]; 
 wire inst_ADDQ_S_W       = op_d[6'h1f] & sa_d[5'h16] & func_d[6'h10];
 wire inst_ADDU_QB        = op_d[6'h1f] & sa_d[5'h00] & func_d[6'h10];
 wire inst_ADDU_S_QB      = op_d[6'h1f] & sa_d[5'h04] & func_d[6'h10];
 wire inst_SUBQ_PH        = op_d[6'h1f] & sa_d[5'h0b] & func_d[6'h10]; 
 wire inst_SUBQ_S_PH      = op_d[6'h1f] & sa_d[5'h0f] & func_d[6'h10]; 
 wire inst_SUBQ_S_W       = op_d[6'h1f] & sa_d[5'h17] & func_d[6'h10]; 
 wire inst_SUBU_QB        = op_d[6'h1f] & sa_d[5'h01] & func_d[6'h10]; 
 wire inst_SUBU_S_QB      = op_d[6'h1f] & sa_d[5'h05] & func_d[6'h10]; 
 wire inst_ADDSC          = op_d[6'h1f] & sa_d[5'h10] & func_d[6'h10];
 wire inst_ADDWC          = op_d[6'h1f] & sa_d[5'h11] & func_d[6'h10];
 wire inst_MODSUB         = op_d[6'h1f] & sa_d[5'h12] & func_d[6'h10]; 
 wire inst_RADDU          = op_d[6'h1f] & rt_d[5'h0]  & sa_d[5'h14] & func_d[6'h10];
 wire inst_ABSQ_S_PH      = op_d[6'h1f] & rs_d[5'h0]  & sa_d[5'h09] & func_d[6'h12]; 
 wire inst_ABSQ_S_W       = op_d[6'h1f] & rs_d[5'h0]  & sa_d[5'h11] & func_d[6'h12]; 
 wire inst_PRECRQ_QB_PH   = op_d[6'h1f] & sa_d[5'h0c] & func_d[6'h11]; 
 wire inst_PRECRQ_PH_W    = op_d[6'h1f] & sa_d[5'h14] & func_d[6'h11];
 wire inst_PRECRQ_RS_PH_W = op_d[6'h1f] & sa_d[5'h15] & func_d[6'h11];
 wire inst_PRECRQU_S_QB_PH = op_d[6'h1f] & sa_d[5'h0f] & func_d[6'h11];
 wire inst_PRECEQ_W_PHL   = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h0c] & func_d[6'h12];
 wire inst_PRECEQ_W_PHR   = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h0d] & func_d[6'h12];
 wire inst_PRECEQU_PH_QBL = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h04] & func_d[6'h12]; 
 wire inst_PRECEQU_PH_QBR = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h05] & func_d[6'h12];
 wire inst_PRECEQU_PH_QBLA= op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h06] & func_d[6'h12];
 wire inst_PRECEQU_PH_QBRA= op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h07] & func_d[6'h12]; 
 wire inst_PRECEU_PH_QBL  = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h1c] & func_d[6'h12]; 
 wire inst_PRECEU_PH_QBR  = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h1d] & func_d[6'h12];
 wire inst_PRECEU_PH_QBLA = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h1e] & func_d[6'h12]; 
 wire inst_PRECEU_PH_QBRA = op_d[6'h1f] & rs_d[5'h0] &sa_d[5'h1f] & func_d[6'h12];
 wire inst_SHLL_QB         = op_d[6'h1f] & sa_d[5'h00] & func_d[6'h13] & (rs[4:3] == 2'b00) ;       
 wire inst_SHLLV_QB        = op_d[6'h1f] & sa_d[5'h02] & func_d[6'h13] ;       
 wire inst_SHLL_PH         = op_d[6'h1f] & sa_d[5'h08] & func_d[6'h13] & (rs[4] == 1'b0) ;       
 wire inst_SHLLV_PH        = op_d[6'h1f] & sa_d[5'h0a] & func_d[6'h13] ;       
 wire inst_SHLL_S_PH       = op_d[6'h1f] & sa_d[5'h0c] & func_d[6'h13] & (rs[4] == 1'b0) ;
 wire inst_SHLLV_S_PH      = op_d[6'h1f] & sa_d[5'h0e] & func_d[6'h13] ;
 wire inst_SHLL_S_W        = op_d[6'h1f] & sa_d[5'h14] & func_d[6'h13] ;
 wire inst_SHLLV_S_W       = op_d[6'h1f] & sa_d[5'h16] & func_d[6'h13] ;
 wire inst_SHRL_QB         = op_d[6'h1f] & sa_d[5'h01] & func_d[6'h13] & (rs[4:3] == 2'b00) ;       
 wire inst_SHRLV_QB        = op_d[6'h1f] & sa_d[5'h03] & func_d[6'h13] ;       
 wire inst_SHRA_PH         = op_d[6'h1f] & sa_d[5'h09] & func_d[6'h13] & (rs[4] == 1'b0) ;       
 wire inst_SHRAV_PH        = op_d[6'h1f] & sa_d[5'h0b] & func_d[6'h13] ;       
 wire inst_SHRA_R_PH       = op_d[6'h1f] & sa_d[5'h0d] & func_d[6'h13] & (rs[4] == 1'b0) ;
 wire inst_SHRAV_R_PH      = op_d[6'h1f] & sa_d[5'h0f] & func_d[6'h13] ;
 wire inst_SHRA_R_W        = op_d[6'h1f] & sa_d[5'h15] & func_d[6'h13] ;
 wire inst_SHRAV_R_W       = op_d[6'h1f] & sa_d[5'h17] & func_d[6'h13] ;
 wire inst_CMPU_EQ_QB      = op_d[6'h1f] & sa_d[5'h00] & func_d[6'h11] & rd_d[5'h00];
 wire inst_CMPU_LT_QB      = op_d[6'h1f] & sa_d[5'h01] & func_d[6'h11] & rd_d[5'h00];
 wire inst_CMPU_LE_QB      = op_d[6'h1f] & sa_d[5'h02] & func_d[6'h11] & rd_d[5'h00];
 wire inst_CMPGU_EQ_QB     = op_d[6'h1f] & sa_d[5'h04] & func_d[6'h11] ;
 wire inst_CMPGU_LT_QB     = op_d[6'h1f] & sa_d[5'h05] & func_d[6'h11] ;
 wire inst_CMPGU_LE_QB     = op_d[6'h1f] & sa_d[5'h06] & func_d[6'h11] ;
 wire inst_CMP_EQ_PH       = op_d[6'h1f] & sa_d[5'h08] & func_d[6'h11] & rd_d[5'h00];
 wire inst_CMP_LT_PH       = op_d[6'h1f] & sa_d[5'h09] & func_d[6'h11] & rd_d[5'h00];
 wire inst_CMP_LE_PH       = op_d[6'h1f] & sa_d[5'h0a] & func_d[6'h11] & rd_d[5'h00];
 wire inst_PICK_QB         = op_d[6'h1f] & sa_d[5'h03] & func_d[6'h11] ; 
 wire inst_PICK_PH         = op_d[6'h1f] & sa_d[5'h0b] & func_d[6'h11] ; 
 wire inst_PACKRL_PH       = op_d[6'h1f] & sa_d[5'h0e] & func_d[6'h11];
 wire inst_MULEU_S_PH_QBL  = op_d[6'h1f] & sa_d[5'h06] & func_d[6'h10];
 wire inst_MULEU_S_PH_QBR  = op_d[6'h1f] & sa_d[5'h07] & func_d[6'h10]; 
 wire inst_MULQ_RS_PH      = op_d[6'h1f] & sa_d[5'h1f] & func_d[6'h10];
 wire inst_MULEQ_S_W_PHL   = op_d[6'h1f] & sa_d[5'h1c] & func_d[6'h10];
 wire inst_MULEQ_S_W_PHR   = op_d[6'h1f] & sa_d[5'h1d] & func_d[6'h10];
 wire inst_MAQ_S_W_PHL     = op_d[6'h1f] & sa_d[5'h14] & func_d[6'h30] & (rd[4:2]== 3'b000) ;
 wire inst_MAQ_S_W_PHR     = op_d[6'h1f] & sa_d[5'h16] & func_d[6'h30] & (rd[4:2]== 3'b000); 
 wire inst_MAQ_SA_W_PHL    = op_d[6'h1f] & sa_d[5'h10] & func_d[6'h30] & (rd[4:2]== 3'b000); 
 wire inst_MAQ_SA_W_PHR    = op_d[6'h1f] & sa_d[5'h12] & func_d[6'h30] & (rd[4:2]== 3'b000); 
 wire inst_DPAU_H_QBL      = op_d[6'h1f] & sa_d[5'h03] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_DPAU_H_QBR      = op_d[6'h1f] & sa_d[5'h07] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_DPSU_H_QBL      = op_d[6'h1f] & sa_d[5'h0b] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_DPSU_H_QBR      = op_d[6'h1f] & sa_d[5'h0f] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_DPAQ_S_W_PH     = op_d[6'h1f] & sa_d[5'h04] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_DPAQ_SA_L_W     = op_d[6'h1f] & sa_d[5'h0c] & func_d[6'h30] & (rd[4:2]== 3'b000); 
 wire inst_DPSQ_S_W_PH     = op_d[6'h1f] & sa_d[5'h05] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_DPSQ_SA_L_W     = op_d[6'h1f] & sa_d[5'h0d] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_MULSAQ_S_W_PH   = op_d[6'h1f] & sa_d[5'h06] & func_d[6'h30] & (rd[4:2]== 3'b000);
 wire inst_BITREV          = op_d[6'h1f] & sa_d[5'h1b] & func_d[6'h12] & rs_d[5'h00]; 
 wire inst_INSV            = op_d[6'h1f] & sa_d[5'h0]  & func_d[6'h0c] & rd_d[5'h00]; 
 wire inst_REPL_QB         = op_d[6'h1f] & sa_d[5'h02] & func_d[6'h12] & (rs[4:3]== 2'b00); 
 wire inst_REPLV_QB        = op_d[6'h1f] & sa_d[5'h03] & func_d[6'h12] & rs_d[5'h00]; 
 wire inst_REPL_PH         = op_d[6'h1f] & sa_d[5'h0a] & func_d[6'h12] ; 
 wire inst_REPLV_PH        = op_d[6'h1f] & sa_d[5'h0b] & func_d[6'h12] & rs_d[5'h00]; 
 wire inst_EXTR_W          = op_d[6'h1f] & sa_d[5'h00] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTR_R_W        = op_d[6'h1f] & sa_d[5'h04] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTR_RS_W       = op_d[6'h1f] & sa_d[5'h06] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTR_S_H        = op_d[6'h1f] & sa_d[5'h0e] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTRV_S_H       = op_d[6'h1f] & sa_d[5'h0f] & func_d[6'h38] & (rd[4:2] ==3'b000); 
 wire inst_EXTRV_W         = op_d[6'h1f] & sa_d[5'h01] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTRV_R_W       = op_d[6'h1f] & sa_d[5'h05] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTRV_RS_W      = op_d[6'h1f] & sa_d[5'h07] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTP            = op_d[6'h1f] & sa_d[5'h02] & func_d[6'h38] & (rd[4:2] ==3'b000); 
 wire inst_EXTPV           = op_d[6'h1f] & sa_d[5'h03] & func_d[6'h38] & (rd[4:2] ==3'b000); 
 wire inst_EXTPDP          = op_d[6'h1f] & sa_d[5'h0a] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_EXTPDPV         = op_d[6'h1f] & sa_d[5'h0b] & func_d[6'h38] & (rd[4:2] ==3'b000);
 wire inst_SHILO           = op_d[6'h1f] & sa_d[5'h1a] & func_d[6'h38] & (rd[4:2] ==3'b000) & (rt[3:0]==4'b0000); 
 wire inst_SHILOV          = op_d[6'h1f] & sa_d[5'h1b] & func_d[6'h38] & (rd[4:2] ==3'b000) & rt_d[5'h00]; 
 wire inst_MTHLIP          = op_d[6'h1f] & sa_d[5'h1f] & func_d[6'h38] ;
 wire inst_BPOSEGE32       = op_d[6'h01] & rs_d[5'h00] & rt_d[5'h1c] ;
 wire inst_RDDSP           = op_d[6'h1f] & sa_d[5'h12] & func_d[6'h38] ;
 wire inst_WRDSP           = op_d[6'h1f] & sa_d[5'h13] & func_d[6'h38] ;

assign    decbus_valid=irbus_valid;
//assign    decbus_valid=irbus_valid&(!reset)&(!commitbus_ex);

assign    decbus_pc[31:0]=irbus_pc[31:0];

assign    decbus_adei=irbus_adei;

assign    decbus_tlbii=irbus_tlbii;

assign    decbus_tlbir=irbus_tlbir;

assign    decbus_ibe=irbus_ibe;    

assign    decbus_dib=irbus_dib;    

wire rdhwr_en =( rd_d[5'h0]&HWREna[0] | rd_d[5'h1]&HWREna[1] |rd_d[5'h2]&HWREna[2] |rd_d[5'h3]&HWREna[3])
               | kernel | cu[0];
wire decfmt_11;
wire inst_cop1;
assign    decbus_ri= ~(decbus_cpui
                      |inst_SYNC|inst_ERET|inst_BREAK|inst_SYSCALL|inst_SDBBP |inst_DERET
                      |inst_MFC0|inst_MTC0
                      |inst_LB|inst_LH|inst_LW|inst_LWC1|inst_LDC1|inst_LBU|inst_LHU|inst_LL
                      |inst_SB|inst_SH|inst_SW|inst_SWC1|inst_SDC1|inst_SC
                      |inst_TLBP|inst_TLBR|inst_TLBWI|inst_TLBWR
                      |inst_CACHE0 /*|inst_CACHE4*/ |inst_CACHE8 |inst_SYNCI
                      |inst_CACHE16 /*|inst_CACHE20 |inst_CACHE24*/
                      |inst_CACHE1 |inst_CACHE5 |inst_CACHE9 /*|inst_CACHE13*/
                      |inst_CACHE17 |inst_CACHE21 /*|inst_CACHE25*/    
                      |inst_CACHE28 |inst_CACHE29
                      |inst_DIV_F
                      |inst_DIV|inst_DIVU
                      |inst_ADD_F|inst_ROUND_L|inst_MFC1|inst_CFC1|inst_CVT_S|inst_C_F|inst_C_SF
                      |inst_SUB_F|inst_TRUNC_L|inst_CVT_D|inst_C_UN|inst_C_NGLE
                      |inst_MUL_F|inst_CEIL_L|inst_C_EQ|inst_C_SEQ
                      |inst_FLOOR_L|inst_C_UEQ|inst_C_NGL
                      |inst_ROUND_W|inst_CVT_W|inst_C_OLT|inst_C_LT
                      |inst_ABS_F|inst_TRUNC_W|inst_CVT_L|inst_C_ULT|inst_C_NGE
                      |inst_MOVE|inst_CEIL_W|inst_C_OLE|inst_C_LE
                      |inst_NEG_F|inst_FLOOR_W|inst_C_ULE|inst_C_NGT
                      |inst_TEQ|inst_TEQI|inst_TNE|inst_TNEI|inst_TLT|inst_TLTI|inst_TLTU|inst_TLTIU
                      |inst_TGE|inst_TGEI|inst_TGEU|inst_TGEIU|inst_SLT|inst_SLTI|inst_SLTU|inst_SLTIU
                      |inst_BC1F|inst_BC1T|inst_BC1FL|inst_BC1TL|inst_J|inst_JR|inst_JR_HB|inst_JAL|inst_JALR|inst_JALR_HB
                      |inst_BEQ|inst_BNE|inst_BLEZ|inst_BGTZ|inst_BLTZ|inst_BGEZ|inst_BLTZAL|inst_BGEZAL
                      |inst_BEQL|inst_BNEL|inst_BLEZL|inst_BGTZL|inst_BLTZL|inst_BGEZL|inst_BLTZALL|inst_BGEZALL
                      |inst_SLL|inst_SLLV|inst_SRL|inst_SRLV|inst_SRA|inst_SRAV|inst_MULT|inst_MULTU
                      |inst_ADD|inst_ADDI|inst_ADDU|inst_ADDIU|inst_LUI | inst_WAIT
                      |inst_LBUX | inst_LWX | inst_LHX
                     //HAVE_DSP_UNIT
                      |inst_MFHI_DSP|inst_MFLO_DSP|inst_MTHI_DSP|inst_MTLO_DSP
                      |inst_MTC1|inst_CTC1
                      |inst_SUB|inst_SUBU|inst_AND|inst_ANDI|inst_OR|inst_ORI|inst_XOR|inst_XORI|inst_NOR
                      |inst_CLO|inst_CLZ|inst_MOVN|inst_MOVZ|inst_MOVF|inst_MOVT
                      |inst_MADD|inst_MADDU|inst_MSUB|inst_MSUBU|inst_MUL
                      |inst_FMOVT|inst_FMOVF|inst_FMOVN|inst_FMOVZ
                      |inst_SEB|inst_SEH|inst_EXT|inst_INS|inst_WSBH|inst_ROTR|inst_ROTRV
                      |inst_EI|inst_DI |inst_RDPGPR| inst_WRPGPR
                      |inst_RDHWR&rdhwr_en&~(|rd[4:2])//wait head
                      |inst_RECIP|inst_RSQRT|inst_SQRT_F
                      |inst_PREF | inst_PREFX                
                      |inst_LWL|inst_LWR|inst_SWL|inst_SWR
                      //HAVE_DSP_UNIT
                      |inst_ADDQ_PH| inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB
                      |inst_ADDU_S_QB |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W
                      |inst_SUBU_QB | inst_SUBU_S_QB | inst_ADDSC | inst_ADDWC 
                      |inst_MODSUB | inst_RADDU | inst_ABSQ_S_PH | inst_ABSQ_S_W 
                      |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                      |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR | inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR
                      |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA | inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR
                      |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA | inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH
                      |inst_SHLLV_PH | inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                      |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH | inst_SHRA_R_PH | inst_SHRAV_R_PH
                      |inst_SHRA_R_W | inst_SHRAV_R_W | inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB | inst_CMP_EQ_PH | inst_CMP_LT_PH 
                      |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH  |inst_PACKRL_PH | inst_MULEU_S_PH_QBL 
                      |inst_MULEU_S_PH_QBR | inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL | inst_MULEQ_S_W_PHR 
                      |inst_MAQ_S_W_PHL | inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                      |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                      |inst_DPSQ_S_W_PH | inst_DPSQ_SA_L_W | inst_MULSAQ_S_W_PH | inst_BITREV | inst_INSV | inst_REPL_QB
                      |inst_REPLV_QB | inst_REPL_PH | inst_REPLV_PH  | inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W
                      |inst_EXTR_S_H | inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W |inst_EXTP
                      |inst_EXTPV | inst_EXTPDP | inst_EXTPDPV | inst_SHILO | inst_SHILOV |inst_MTHLIP | inst_BPOSEGE32
                      |inst_RDDSP | inst_WRDSP
                      );

wire decfmt_sub6  = op_d[6'h12]&&(func[5:2]==4'b0010)&&(rs[4]&&rs[3]&&(!rs[2]||rs[2]&&!rs[1])) ; 
wire decfmt_sub4  = op_d[6'h12]&&(func[5:2]==4'b0011)&&(rs[4]&&rs[3]&&rs[2]&&!rs[1]) ; 
wire decfmt_sub12 = op_d[6'h12]&&(func[5:2]==4'b0011)&&(rs[4]&&rs[3]&&!rs[2]) ; 

wire decfmt_cc_rt = 1'b0;
wire decfmt_cc_sa = 1'b0;

wire decfmt_cop1x = op_d[6'h13]&&(func[5]==1'b1)&&(cop1x_fmt[2:1]==2'b00); //fmadd,fsub,fmaddu,fsubu fmt=s,d
wire decfmt_mhilo = op_d[6'h00]&&(func[5:2]==4'b0100) || //mfhi mflo mthi mtlo
                     op_d[6'h11]&&rs_d[5'h16]&&(func[5:2]==4'b1011) || //pll,plu,pul,puu
                     op_d[6'h1c]&&rs_d[5'h11]&&rt_d[5'h00]&&(func[5:1]==5'b11001);//cvt.ud.d cvt.ld.d

assign decfmt_11    = op_d[6'h11]&&rs_d[5'h16] || //float ps, 22->11
                     op_d[6'h13]&&(func[5]==1'b1)&&(cop1x_fmt[2:0]==3'b110); //fmadd,fsub,fmaddu,fsubu fmt=ps
wire decfmt_20    = inst_ADD|inst_ADDI|inst_ADDIU|inst_ADDU|inst_DIV|inst_DIVU|inst_MULT|inst_MULTU|inst_MUL
                    |inst_SLL|inst_SLLV|inst_SRA|inst_SRAV|inst_SRL|inst_SRLV|inst_SUB|inst_SUBU;

assign decbus_fmt[4:0] = decfmt_sub6 ?(rs[4:0]-5'h6):
                         decfmt_sub4 ?(rs[4:0]-5'h4):
                         decfmt_sub12?(rs[4:0]-5'hc):
                         decfmt_cc_rt?{rs[1:0],rt[4:2]}:
                         decfmt_cc_sa?{rs[1:0],sa[4:2]}:
                         decfmt_mhilo?{3'b0,func[1:0]}:
                         decfmt_11   ?5'd11:
                         decfmt_20   ?5'd20:
                         decfmt_cop1x?{2'b10,cop1x_fmt[2:0]}:
                         rs[4:0];
//ac field in decbus
wire decac_qb         = inst_ADDU_QB | inst_ADDU_S_QB |inst_SUBU_QB | inst_SUBU_S_QB 
                        | inst_SHLL_QB | inst_SHLLV_QB|inst_SHRL_QB | inst_SHRLV_QB
                        | inst_CMPU_EQ_QB| inst_CMPU_LT_QB| inst_CMPU_LE_QB 
                        | inst_CMPGU_EQ_QB| inst_CMPGU_LT_QB| inst_CMPGU_LE_QB 
                        |inst_REPLV_QB | inst_REPL_QB | inst_PICK_QB;
wire decac_ph         = inst_ADDQ_PH| inst_ADDQ_S_PH|inst_SUBQ_PH | inst_SUBQ_S_PH |inst_ABSQ_S_PH
                        | inst_SHLL_PH | inst_SHLLV_PH|inst_SHRA_PH | inst_SHRAV_PH
                        | inst_SHLL_S_PH | inst_SHLLV_S_PH| inst_SHRA_R_PH | inst_SHRAV_R_PH
                        | inst_CMP_EQ_PH| inst_CMP_LT_PH| inst_CMP_LE_PH |inst_REPLV_PH |inst_REPL_PH
                        | inst_PRECRQ_QB_PH |inst_PRECRQU_S_QB_PH 
                        | inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR | inst_PACKRL_PH | inst_PICK_PH;
wire decac_w         =  inst_ADDQ_S_W| inst_SUBQ_S_W |inst_ABSQ_S_W
                        | inst_SHLL_S_W | inst_SHLLV_S_W| inst_SHRA_R_W | inst_SHRAV_R_W
                        | inst_ADDSC | inst_ADDWC | inst_MODSUB 
                        |inst_PRECRQ_RS_PH_W | inst_PRECRQ_PH_W
                        |inst_BITREV |inst_INSV;
wire decac_ac        =  inst_SHILO|inst_SHILOV|inst_MTHLIP
                        |inst_EXTR_RS_W | inst_EXTR_W | inst_EXTR_R_W
                        |inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W
                        |inst_EXTR_S_H | inst_EXTRV_S_H 
                        |inst_EXTP | inst_EXTPV |inst_EXTPDP | inst_EXTPDPV  
                        |inst_MTHI_DSP  | inst_MTLO_DSP
                        |inst_MAQ_S_W_PHL| inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                        |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                        |inst_DPSQ_S_W_PH|inst_DPSQ_SA_L_W|inst_MULSAQ_S_W_PH;
wire decac_rs       = inst_MFHI_DSP | inst_MFLO_DSP;
assign decbus_ac    = decac_qb ? 2'b00 :
                      decac_ph ? 2'b01:
                      decac_rs  ? rs[1:0] :
                      decac_w  ? 2'b10:
                      decac_ac ? rd[1:0]: 2'b00;

assign decbus_nop    = inst_NOP | inst_WAIT;

assign dec_opbit[7] = inst_SYNC|inst_ERET
                      |inst_TLBP|inst_TLBR|inst_TLBWI|inst_TLBWR|inst_MFC0|inst_RDHWR|inst_MTC0
                      |inst_LB|inst_LH|inst_LW|inst_LWC1|inst_LDC1|inst_LBU|inst_LHU|inst_LL
                      |inst_SB|inst_SH|inst_SW|inst_SWC1|inst_SDC1|inst_SC
                      |inst_CACHE0|inst_CACHE1|inst_CACHE4|inst_CACHE5|inst_CACHE6
                      |inst_CACHE7
                      |inst_CACHE8|inst_CACHE9|inst_CACHE10|inst_CACHE11|inst_CACHE12|inst_CACHE13|inst_CACHE14|inst_CACHE15
                      |inst_CACHE16|inst_CACHE17|inst_CACHE18|inst_CACHE19|inst_CACHE20|inst_CACHE21|inst_CACHE22|inst_CACHE23
                      |inst_CACHE24|inst_CACHE25|inst_CACHE26|inst_CACHE27|inst_CACHE28|inst_CACHE29|inst_CACHE30|inst_CACHE31
                      |inst_LWL|inst_LWR|inst_SWL|inst_SWR|inst_DERET|inst_CFC1|inst_CTC1|inst_MFC1|inst_MTC1
                      |inst_DI |inst_EI|inst_SYNCI | inst_PREF | inst_PREFX
                      //for DSP inst
                      |inst_ADDQ_PH| inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB
                      |inst_ADDU_S_QB |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W
                      |inst_SUBU_QB | inst_SUBU_S_QB | inst_ADDSC | inst_ADDWC 
                      |inst_MODSUB | inst_RADDU | inst_ABSQ_S_PH | inst_ABSQ_S_W 
                      |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                      |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR | inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR
                      |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA | inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR
                      |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA | inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH
                      |inst_SHLLV_PH | inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                      |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH | inst_SHRA_R_PH | inst_SHRAV_R_PH
                      |inst_SHRA_R_W | inst_SHRAV_R_W | inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB | inst_CMP_EQ_PH | inst_CMP_LT_PH 
                      |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH  |inst_PACKRL_PH | inst_MULEU_S_PH_QBL 
                      |inst_MULEU_S_PH_QBR | inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL | inst_MULEQ_S_W_PHR 
                      |inst_MAQ_S_W_PHL | inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                      |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                      |inst_DPSQ_S_W_PH | inst_DPSQ_SA_L_W | inst_MULSAQ_S_W_PH | inst_BITREV | inst_INSV | inst_REPL_QB
                      |inst_REPLV_QB | inst_REPL_PH | inst_REPLV_PH  | inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W
                      |inst_EXTR_S_H | inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W |inst_EXTP
                      |inst_EXTPV | inst_EXTPDP | inst_EXTPDPV | inst_SHILO | inst_SHILOV |inst_MTHLIP | inst_BPOSEGE32
                      |inst_RDDSP | inst_WRDSP | inst_LBUX | inst_LHX | inst_LWX
                      ;
            
assign dec_opbit[6] = inst_ADD_F|inst_ROUND_L|inst_CVT_S|inst_C_F|inst_C_SF
                      |inst_SUB_F|inst_TRUNC_L|inst_CVT_D|inst_C_UN|inst_C_NGLE
                      |inst_MUL_F|inst_CEIL_L|inst_C_EQ|inst_C_SEQ
                      |inst_DIV_F|inst_FLOOR_L|inst_C_UEQ|inst_C_NGL
                      |inst_SQRT_F|inst_ROUND_W|inst_CVT_W|inst_C_OLT|inst_C_LT
                      |inst_ABS_F|inst_TRUNC_W|inst_CVT_L|inst_C_ULT|inst_C_NGE
                      |inst_MOVE|inst_CEIL_W|inst_C_OLE|inst_C_LE
                      |inst_NEG_F|inst_FLOOR_W|inst_C_ULE|inst_C_NGT
                      |inst_BC1F|inst_BC1T|inst_BC1FL|inst_BC1TL
                      |inst_FMOVF|inst_FMOVT|inst_FMOVZ|inst_FMOVN
                      |inst_MOVF|inst_MOVT
                      |inst_RECIP|inst_RECIP
                      |inst_RSQRT
                      //for DSP inst
                      |inst_ADDQ_PH| inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB
                      |inst_ADDU_S_QB |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W
                      |inst_SUBU_QB | inst_SUBU_S_QB | inst_ADDSC | inst_ADDWC 
                      |inst_MODSUB | inst_RADDU | inst_ABSQ_S_PH | inst_ABSQ_S_W 
                      |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                      |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR | inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR
                      |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA | inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR
                      |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA | inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH
                      |inst_SHLLV_PH | inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                      |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH | inst_SHRA_R_PH | inst_SHRAV_R_PH
                      |inst_SHRA_R_W | inst_SHRAV_R_W | inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB | inst_CMP_EQ_PH | inst_CMP_LT_PH 
                      |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH  |inst_PACKRL_PH | inst_MULEU_S_PH_QBL 
                      |inst_MULEU_S_PH_QBR | inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL | inst_MULEQ_S_W_PHR 
                      |inst_MAQ_S_W_PHL | inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                      |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                      |inst_DPSQ_S_W_PH | inst_DPSQ_SA_L_W | inst_MULSAQ_S_W_PH | inst_BITREV | inst_INSV | inst_REPL_QB
                      |inst_REPLV_QB | inst_REPL_PH | inst_REPLV_PH  | inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W
                      |inst_EXTR_S_H | inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W |inst_EXTP
                      |inst_EXTPV | inst_EXTPDP | inst_EXTPDPV | inst_SHILO | inst_SHILOV |inst_MTHLIP | inst_BPOSEGE32
                      |inst_RDDSP | inst_WRDSP 
                      ;    
                
assign dec_opbit[5] = inst_TEQ|inst_TEQI|inst_TNE|inst_TNEI|inst_TLT|inst_TLTI|inst_TLTU|inst_TLTIU
                      |inst_TGE|inst_TGEI|inst_TGEU|inst_TGEIU|inst_SLT|inst_SLTI|inst_SLTU|inst_SLTIU
                      |inst_BC1F|inst_BC1T|inst_BC1FL|inst_BC1TL|inst_J|inst_JR|inst_JR_HB|inst_JAL|inst_JALR|inst_JALR_HB
                      |inst_BEQ|inst_BNE|inst_BLEZ|inst_BGTZ|inst_BLTZ|inst_BGEZ|inst_BLTZAL|inst_BGEZAL
                      |inst_BEQL|inst_BNEL|inst_BLEZL|inst_BGTZL|inst_BLTZL|inst_BGEZL|inst_BLTZALL|inst_BGEZALL
                      |inst_CACHE0|inst_CACHE1|inst_LWC1|inst_SWC1|inst_CACHE4|inst_CACHE5|inst_CACHE6
                      |inst_CACHE7
                      |inst_CACHE8|inst_CACHE9|inst_CACHE10|inst_CACHE11|inst_CACHE12|inst_CACHE13|inst_CACHE14|inst_CACHE15
                      |inst_CACHE16|inst_CACHE17|inst_CACHE18|inst_CACHE19|inst_CACHE20|inst_CACHE21|inst_CACHE22|inst_CACHE23
                      |inst_CACHE24|inst_CACHE25|inst_CACHE26|inst_CACHE27|inst_CACHE28|inst_CACHE29|inst_CACHE30|inst_CACHE31
                      |inst_CVT_S|inst_CVT_D|inst_CVT_W|inst_CVT_L
                      |inst_C_F|inst_C_UN|inst_C_EQ|inst_C_UEQ|inst_C_OLT|inst_C_ULT|inst_C_OLE|inst_C_ULE
                      |inst_C_SF|inst_C_NGLE|inst_C_SEQ|inst_C_NGL|inst_C_LT|inst_C_NGE|inst_C_LE|inst_C_NGT
                      |inst_LWL|inst_LWR|inst_SWL|inst_SWR
                      |inst_MADD|inst_MADDU|inst_MSUB|inst_MSUBU
                      |inst_FMOVF|inst_FMOVT|inst_FMOVN|inst_FMOVZ 
                      |inst_MOVF|inst_MOVT | inst_PREFX
                      |inst_MULEU_S_PH_QBL |inst_MULEU_S_PH_QBR | inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL
                      |inst_MULEQ_S_W_PHR |inst_MAQ_S_W_PHL | inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL
                      |inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL |inst_DPAU_H_QBR | inst_DPSU_H_QBL 
                      |inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W |inst_DPSQ_S_W_PH 
                      |inst_DPSQ_SA_L_W | inst_MULSAQ_S_W_PH | inst_BITREV | inst_INSV | inst_REPL_QB
                      |inst_REPLV_QB | inst_REPL_PH | inst_REPLV_PH  | inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W
                      |inst_EXTR_S_H | inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W |inst_EXTP
                      |inst_EXTPV | inst_EXTPDP | inst_EXTPDPV | inst_SHILO | inst_SHILOV 
                      |inst_MTHLIP | inst_BPOSEGE32 |inst_RDDSP | inst_WRDSP
                      ;    

assign dec_opbit[4] = inst_SLL|inst_SLLV|inst_PREF|inst_SRL|inst_SRLV|inst_SRA|inst_SRAV|inst_MULT|inst_MULTU
                      |inst_DIV|inst_DIVU
                      |inst_ADD|inst_ADDI|inst_ADDU|inst_ADDIU|inst_LUI
                      |inst_SUB|inst_SUBU|inst_AND|inst_ANDI|inst_OR|inst_ORI|inst_XOR|inst_XORI|inst_NOR
                      |inst_BEQ|inst_BNE|inst_BLEZ|inst_BGTZ|inst_BLTZ|inst_BGEZ|inst_BLTZAL|inst_BGEZAL
                      |inst_BEQL|inst_BNEL|inst_BLEZL|inst_BGTZL|inst_BLTZL|inst_BGEZL|inst_BLTZALL|inst_BGEZALL
                      |inst_LB|inst_LH|inst_LW|inst_LDC1|inst_LBU|inst_LHU|inst_LL
                      |inst_SB|inst_SH|inst_SW|inst_SDC1|inst_SC
                      |inst_CACHE16|inst_CACHE17|inst_CACHE18|inst_CACHE19|inst_CACHE20|inst_CACHE21|inst_CACHE22|inst_CACHE23
                      |inst_CACHE24|inst_CACHE25|inst_CACHE26|inst_CACHE27|inst_CACHE28|inst_CACHE29|inst_CACHE30|inst_CACHE31
                      |inst_C_F|inst_C_UN|inst_C_EQ|inst_C_UEQ|inst_C_OLT|inst_C_ULT|inst_C_OLE|inst_C_ULE
                      |inst_C_SF|inst_C_NGLE|inst_C_SEQ|inst_C_NGL|inst_C_LT|inst_C_NGE|inst_C_LE|inst_C_NGT
                      |inst_LWL|inst_LWR|inst_SWL|inst_SWR
                      |inst_MUL
                      |inst_RECIP|inst_RSQRT 
                      |inst_RDPGPR |inst_WRPGPR 
                      |inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA 
                      |inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA
                      |inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH |inst_SHLLV_PH 
                      |inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                      |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH 
                      |inst_SHRA_R_PH | inst_SHRAV_R_PH |inst_SHRA_R_W | inst_SHRAV_R_W
                      |inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB 
                      |inst_CMP_EQ_PH | inst_CMP_LT_PH |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH   
                      |inst_DPAU_H_QBL|inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR 
                      |inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W |inst_DPSQ_S_W_PH | inst_DPSQ_SA_L_W 
                      |inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W |inst_EXTR_S_H 
                      |inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W |inst_EXTRV_S_H
                      |inst_MTHLIP | inst_BPOSEGE32 | inst_RDDSP | inst_WRDSP 
                      | inst_LBUX | inst_LHX | inst_LWX
                      ; //add


assign dec_opbit[3] = inst_ADD|inst_ADDI|inst_ADDU|inst_ADDIU|inst_LUI|inst_MFHI|inst_MFLO|inst_MTHI|inst_MTLO
                      |inst_SUB|inst_SUBU|inst_AND|inst_ANDI|inst_OR|inst_ORI|inst_XOR|inst_XORI|inst_NOR
                      |inst_BC1F|inst_BC1T|inst_BC1FL|inst_BC1TL|inst_J|inst_JR|inst_JR_HB|inst_JAL|inst_JALR|inst_JALR_HB
                      |inst_BEQL|inst_BNEL|inst_BLEZL|inst_BGTZL|inst_BLTZL|inst_BGEZL|inst_BLTZALL|inst_BGEZALL
                      |inst_TLBP|inst_TLBR|inst_TLBWI|inst_TLBWR|inst_MFC0|inst_RDHWR|inst_MTC0
                      |inst_SB|inst_SH|inst_SW|inst_SDC1|inst_SC
                      |inst_CACHE8|inst_CACHE9|inst_CACHE10|inst_CACHE11|inst_CACHE12|inst_CACHE13|inst_CACHE14|inst_CACHE15
                      |inst_CACHE24|inst_CACHE25|inst_CACHE26|inst_CACHE27|inst_CACHE28|inst_CACHE29|inst_CACHE30|inst_CACHE31
                      |inst_ROUND_L|inst_TRUNC_L|inst_CEIL_L|inst_FLOOR_L|inst_ROUND_W|inst_TRUNC_W|inst_CEIL_W|inst_FLOOR_W
                      |inst_C_SF|inst_C_NGLE|inst_C_SEQ|inst_C_NGL|inst_C_LT|inst_C_NGE|inst_C_LE|inst_C_NGT
                      |inst_LWL|inst_LWR|inst_SWL|inst_SWR|inst_DERET 
                      |inst_MOVN|inst_MOVZ
                      |inst_MADD|inst_MADDU|inst_MSUB|inst_MSUBU
                      |inst_FMOVF|inst_FMOVT|inst_FMOVN|inst_FMOVZ
                      |inst_RECIP|inst_RSQRT | inst_MFLO_DSP |inst_MFHI_DSP|inst_MTHI_DSP|inst_MTLO_DSP
                      |inst_SEB | inst_SEH|inst_SYNCI
                      |inst_RDPGPR |inst_WRPGPR
                      |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                      |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR |inst_PACKRL_PH | inst_MODSUB
                      |inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH |inst_SHLLV_PH
                      |inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                      |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH
                      |inst_SHRA_R_PH | inst_SHRAV_R_PH |inst_SHRA_R_W | inst_SHRAV_R_W 
                      |inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB 
                      |inst_CMP_EQ_PH | inst_CMP_LT_PH |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH  
                      |inst_MAQ_S_W_PHL | inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL |inst_MAQ_SA_W_PHR 
                      |inst_MULSAQ_S_W_PH |inst_EXTP|inst_EXTPV | inst_EXTPDP | inst_EXTPDPV
                      |inst_SHILO | inst_SHILOV
                      |inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W |inst_EXTR_S_H 
                      |inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W 
                      |inst_MTHLIP | inst_BPOSEGE32 |inst_RDDSP | inst_WRDSP
                      | inst_LBUX | inst_LHX | inst_LWX
                      ;//add
   
assign dec_opbit[2] = inst_MULT|inst_AND|inst_ANDI|inst_TGE|inst_TGEI|inst_J|inst_BLTZ|inst_BLTZL
                      |inst_MULTU|inst_OR|inst_ORI|inst_TGEU|inst_TGEIU|inst_JR|inst_JR_HB|inst_BGEZ|inst_BGEZL
                      |inst_DIV|inst_XOR|inst_XORI|inst_SLT|inst_SLTI|inst_JAL|inst_BLTZAL|inst_BLTZALL
                      |inst_DIVU|inst_NOR|inst_SLTU|inst_SLTIU|inst_JALR|inst_JALR_HB|inst_BGEZAL|inst_BGEZALL
                      |inst_MFC0|inst_RDHWR|inst_LBU|inst_CACHE4|inst_CACHE12|inst_CACHE20|inst_CACHE28
                      |inst_MTC0|inst_LHU|inst_CACHE5|inst_CACHE13|inst_CACHE21|inst_CACHE29
                      |inst_SYNC|inst_LL|inst_SC|inst_CACHE6|inst_CACHE14|inst_CACHE22|inst_CACHE30
                      |inst_ERET|inst_CACHE7|inst_CACHE15|inst_CACHE23|inst_CACHE31
                      |inst_SQRT_F|inst_ROUND_W|inst_CVT_W|inst_C_OLT|inst_C_LT
                      |inst_ABS_F|inst_TRUNC_W|inst_CVT_L|inst_C_ULT|inst_C_NGE
                      |inst_MOVE|inst_CEIL_W|inst_C_OLE|inst_C_LE
                      |inst_NEG_F|inst_FLOOR_W|inst_C_ULE|inst_C_NGT
                      |inst_MFLO_DSP |inst_MFHI_DSP|inst_MTLO_DSP|inst_MTHI_DSP
                      |inst_MFHI|inst_MFLO |inst_MTHI|inst_MTLO
                      |inst_DERET  | inst_PREF | inst_PREFX
                      |inst_MOVF|inst_MOVT 
                      |inst_FMOVF|inst_FMOVT|inst_FMOVN|inst_FMOVZ
                      |inst_RECIP|inst_RSQRT
                      |inst_WSBH|inst_ROTR|inst_ROTRV
                      |inst_DI | inst_EI | inst_SYNCI  
                      |inst_ADDSC | inst_ADDWC  |inst_ABSQ_S_PH | inst_ABSQ_S_W |inst_RADDU
                      |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR |inst_PACKRL_PH | inst_MODSUB
                      |inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA
                      |inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB 
                      |inst_CMP_EQ_PH | inst_CMP_LT_PH |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH  
                      |inst_BITREV | inst_INSV | inst_REPL_QB |inst_REPLV_QB | inst_REPL_PH |inst_REPLV_PH
                      |inst_EXTP|inst_EXTPV | inst_EXTPDP | inst_EXTPDPV |inst_SHILO | inst_SHILOV
                      |inst_DPSU_H_QBL | inst_DPSU_H_QBR |inst_DPSQ_S_W_PH | inst_DPSQ_SA_L_W 
                      |inst_MTHLIP | inst_BPOSEGE32 |inst_RDDSP | inst_WRDSP
                      |inst_MULSAQ_S_W_PH |inst_MULEQ_S_W_PHR 
                      | inst_LBUX | inst_LHX | inst_LWX;//add


assign dec_opbit[1] = inst_SRL|inst_SRLV|inst_SUB|inst_TLT|inst_TLTI|inst_BC1FL|inst_BLEZ|inst_BLEZL
                      |inst_SRA|inst_SRAV|inst_SUBU|inst_TLTU|inst_TLTIU|inst_BC1TL|inst_BGTZ|inst_BGTZL
                      |inst_DIV|inst_XOR|inst_XORI|inst_SLT|inst_SLTI|inst_JAL|inst_BLTZAL|inst_BLTZALL
                      |inst_DIVU|inst_NOR|inst_SLTU|inst_SLTIU|inst_JALR|inst_JALR_HB|inst_BGEZAL|inst_BGEZALL
                      |inst_TLBWI|inst_LW|inst_LWC1|inst_SW|inst_CACHE10|inst_CACHE18|inst_CACHE26
                      |inst_TLBWR|inst_LDC1|inst_SDC1|inst_SWC1|inst_CACHE11|inst_CACHE19|inst_CACHE27
                      |inst_SYNC|inst_LL|inst_SC|inst_CACHE6|inst_CACHE14|inst_CACHE22|inst_CACHE30
                      |inst_ERET|inst_CACHE7|inst_CACHE15|inst_CACHE23|inst_CACHE31
                      |inst_MUL_F|inst_CEIL_L|inst_C_EQ|inst_C_SEQ
                      |inst_DIV_F|inst_FLOOR_L|inst_C_UEQ|inst_C_NGL
                      |inst_MOVE|inst_CEIL_W|inst_C_OLE|inst_C_LE
                      |inst_NEG_F|inst_FLOOR_W|inst_C_ULE|inst_C_NGT|inst_DERET
                      |inst_MOVN|inst_MOVZ|inst_MSUB|inst_MSUBU 
                      |inst_MTLO_DSP|inst_MTHI_DSP|inst_MTLO|inst_MTHI
                      |inst_FMOVN|inst_FMOVZ | inst_PREF  
                      |inst_CTC1|inst_MTC1 
                      |inst_MOVF|inst_MOVT | inst_PREFX
                      |inst_EXT |inst_INS |inst_ROTR|inst_ROTRV 
                      |inst_SWL |inst_SWR |inst_SYNCI
                      |inst_ADDSC | inst_ADDWC |inst_PACKRL_PH | inst_MODSUB
                      |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA
                      |inst_CMPU_LE_QB | inst_CMPGU_LE_QB |inst_CMP_LE_PH | inst_PICK_QB | inst_PICK_PH  
                      |inst_INSV | inst_REPL_QB |inst_REPLV_QB | inst_REPL_PH |inst_REPLV_PH
                      |inst_EXTPDP | inst_EXTPDPV |inst_SHILO | inst_SHILOV
                      |inst_DPSQ_S_W_PH | inst_DPSQ_SA_L_W |inst_RDDSP | inst_WRDSP
                      |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W |inst_SUBU_QB | inst_SUBU_S_QB 
                      |inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA 
                      |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH 
                      |inst_SHRA_R_PH | inst_SHRAV_R_PH |inst_SHRA_R_W | inst_SHRAV_R_W 
                      |inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL | inst_MAQ_SA_W_PHL| inst_MAQ_SA_W_PHR
                      |inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                      |inst_EXTR_RS_W |inst_EXTR_S_H| inst_EXTRV_RS_W |inst_EXTRV_S_H |inst_LWX; 


assign dec_opbit[0] = inst_ADDU|inst_ADDIU|inst_LUI|inst_MFLO|inst_MTLO|inst_CTC1|inst_TNE|inst_TNEI
                      |inst_BC1T|inst_BNE|inst_BNEL|inst_SRA|inst_SRAV|inst_SUBU|inst_TLTU|inst_TLTIU|inst_BC1TL|inst_BGTZ
                      |inst_BGTZL|inst_OR|inst_ORI|inst_TGEU|inst_TGEIU|inst_JR|inst_JR_HB|inst_BGEZ|inst_BGEZL
                      |inst_DIVU|inst_NOR|inst_SLTU|inst_SLTIU|inst_JALR|inst_JALR_HB|inst_BGEZAL|inst_BGEZALL
                      |inst_TLBR|inst_LH|inst_SH|inst_CACHE1|inst_CACHE9|inst_CACHE17|inst_CACHE25
                      |inst_TLBWR|inst_LDC1|inst_SDC1|inst_SWC1|inst_CACHE11|inst_CACHE19|inst_CACHE27
                      |inst_MTC0|inst_LHU|inst_CACHE5|inst_CACHE13|inst_CACHE21|inst_CACHE29
                      |inst_ERET|inst_CACHE7|inst_CACHE15|inst_CACHE23|inst_CACHE31        
                      |inst_SUB_F|inst_TRUNC_L|inst_CVT_D|inst_C_UN|inst_C_NGLE
                      |inst_DIV_F|inst_FLOOR_L|inst_C_UEQ|inst_C_NGL
                      |inst_ABS_F|inst_TRUNC_W|inst_CVT_L|inst_C_ULT|inst_C_NGE
                      |inst_NEG_F|inst_FLOOR_W|inst_C_ULE|inst_C_NGT
                      |inst_LWR|inst_SWR|inst_DERET
                      |inst_CLZ|inst_MOVZ|inst_MOVT|inst_MADDU|inst_MSUBU
                      |inst_PREF|inst_SLL|inst_SLLV|inst_MULTU|inst_FMOVT|inst_FMOVN
                      |inst_RSQRT
                      |inst_CFC1  
                      |inst_INS|inst_SEH |inst_EI 
                      |inst_RDPGPR | inst_WRPGPR 
                      |inst_MFLO_DSP|inst_MTLO_DSP
                      |inst_ADDQ_S_PH | inst_ADDQ_S_W |inst_ADDU_S_QB |inst_PRECRQ_PH_W | inst_PRECEQU_PH_QBR
                      |inst_SUBQ_S_PH | inst_SUBQ_S_W | inst_SUBU_S_QB | inst_PRECRQU_S_QB_PH |inst_PRECEQU_PH_QBRA
                      |inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                      |inst_SHRA_R_PH | inst_SHRAV_R_PH |inst_SHRA_R_W | inst_SHRAV_R_W
                      |inst_MULEU_S_PH_QBR |inst_MAQ_S_W_PHR |inst_DPAU_H_QBR | inst_EXTR_R_W| inst_EXTRV_R_W
                      |inst_MULEQ_S_W_PHL| inst_MAQ_SA_W_PHR| inst_DPAQ_SA_L_W|inst_EXTR_S_H | inst_EXTRV_S_H
                      |inst_RADDU| inst_PRECEQ_W_PHR |inst_PRECEU_PH_QBR
                      |inst_CMPU_LT_QB| inst_CMPGU_LT_QB| inst_CMP_LT_PH 
                      |inst_BITREV|inst_EXTP |inst_EXTPV| inst_DPSU_H_QBR | inst_BPOSEGE32 
                      |inst_ADDWC|inst_MODSUB| inst_PRECEU_PH_QBRA| inst_PICK_QB | inst_PICK_PH 
                      |inst_REPLV_QB | inst_REPL_PH | inst_REPLV_PH | inst_REPL_QB
                      |inst_SHILO | inst_SHILOV| inst_DPSQ_SA_L_W| inst_WRDSP | inst_LHX | inst_LWX;

assign decbus_op[7:0] = dec_opbit[7:0];
assign decbus_sys     = inst_SYSCALL;    
assign decbus_bp      = inst_BREAK;
assign decbus_sdbbp   = inst_SDBBP;
assign decbus_bd      = irbus_bd;

assign kernel         = erl|exl|(~ksu[1]&~ksu[0])|DEBUG_MODE;
assign superuser      = (~ksu[1]&ksu[0])&~erl&~exl&(~DEBUG_MODE);
assign user           = (ksu[1]&~ksu[0])&~erl&~exl&(~DEBUG_MODE);

//generate cpui
wire inst_cop2_gs     = 1'b0;

wire user_mfc0op      = inst_MFC0 && (rd_d[5'h18]||rd_d[5'h19]);
wire inst_cop0        = op_d[6'h10] && !user_mfc0op || op_d[6'h2f];
assign inst_cop1        = (op_d[6'h11] | inst_MOVT | inst_MOVF|inst_LDC1|inst_SWC1|inst_SDC1|inst_LWC1) | inst_cop2_gs;
wire inst_cop2        = (op_d[6'h12] || op_d[6'h32] || op_d[6'h3a] || op_d[6'h36] || op_d[6'h3e]) & ~inst_cop2_gs;
wire inst_cop1x       =  op_d[6'h13] & !inst_PREFX;
assign decbus_cpui    = (~cu[2] & (inst_cop2))               |
                        (~cu[1] & (inst_cop1 | inst_cop1x )) |
                        (~cu[0] & (inst_cop0 & ~kernel));

assign decbus_ce[1:0] = (~cu[2] & (inst_cop2))                ?  2'b10:
                        (~cu[1] & (inst_cop1 | inst_cop1x  )) ?  2'b01: 2'b00;
                    
assign decsrc1_rs     = inst_ADD|inst_ADDI|inst_ADDIU|inst_ADDU|inst_AND|inst_ANDI
                        |inst_BEQ|inst_BEQL|inst_BGEZ|inst_BGEZAL|inst_BGEZALL|inst_BGEZL|inst_BGTZ|inst_BGTZL|inst_BLEZ
                        |inst_BLEZL|inst_BLTZ|inst_BLTZAL|inst_BLTZALL|inst_BLTZL|inst_BNE|inst_BNEL
                        |inst_DIV|inst_DIVU
                        |inst_JALR|inst_JALR_HB|inst_JR|inst_JR_HB|inst_MTHI|inst_MTLO|inst_MULT
                        |inst_MULTU|inst_NOR|inst_OR|inst_ORI|inst_SLTI|inst_SLTIU|inst_SUB|inst_SUBU
                        |inst_TEQ|inst_TEQI|inst_TGE|inst_TGEI|inst_TGEIU|inst_TGEU
                        |inst_TLT|inst_TLTI|inst_TLTIU|inst_TLTU|inst_TNE|inst_TNEI
                        |inst_XOR|inst_XORI
                        |inst_CACHE0|inst_CACHE1|inst_CACHE3|inst_CACHE4|inst_CACHE5|inst_CACHE6
                        |inst_CACHE7|inst_SYNCI
                        |inst_CACHE8|inst_CACHE9|inst_CACHE10|inst_CACHE11|inst_CACHE12|inst_CACHE13|inst_CACHE14|inst_CACHE15
                        |inst_CACHE16|inst_CACHE17|inst_CACHE18|inst_CACHE19|inst_CACHE20|inst_CACHE21|inst_CACHE22
                        |inst_CACHE23|inst_CACHE24|inst_CACHE25|inst_CACHE26|inst_CACHE27|inst_CACHE28|inst_CACHE29
                        |inst_CACHE30|inst_CACHE31
                        |inst_LB|inst_LBU|inst_LDC1|inst_LH|inst_LHU|inst_LL|inst_LW|inst_LWC1
                        |inst_SB|inst_SC|inst_SDC1|inst_SH|inst_SW|inst_SWC1
                        |inst_SLT|inst_SLTU
                        |inst_LWL|inst_LWR|inst_SWL|inst_SWR
                        |inst_MUL|inst_CLZ|inst_CLO|inst_MADD|inst_MADDU|inst_MSUB|inst_MSUBU
                        |inst_EXT|inst_INS
                        |inst_INSV |inst_MTLO_DSP |inst_MTHI_DSP
                        |inst_ADDQ_PH| inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB
                        |inst_ADDU_S_QB |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W
                        |inst_SUBU_QB | inst_SUBU_S_QB | inst_ADDSC | inst_ADDWC 
                        |inst_MODSUB | inst_RADDU 
                        |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                        |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB | inst_CMP_EQ_PH | inst_CMP_LT_PH 
                        |inst_CMP_LE_PH| inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                        |inst_PICK_QB | inst_PICK_PH  |inst_PACKRL_PH | inst_MULEU_S_PH_QBL 
                        |inst_MULEU_S_PH_QBR | inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL | inst_MULEQ_S_W_PHR 
                        |inst_MAQ_S_W_PHL| inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                        |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                        |inst_DPSQ_S_W_PH|inst_DPSQ_SA_L_W|inst_MULSAQ_S_W_PH 
                        |inst_WRDSP | inst_LBUX | inst_LHX | inst_LWX
                        |inst_MOVF | inst_MOVT | inst_PREF | inst_PREFX
                        ;
    
assign decsrc1_cmp    = 1'b0;

assign decsrc2_cmp    = 1'b0;

assign decsrc1_rt     =  inst_CTC1
                        |inst_MTC1|inst_SLL|inst_SLLV|inst_SRA
                        |inst_SRLV|inst_SRL|inst_SRAV
                        |inst_MOVN|inst_MOVZ|inst_FMOVN|inst_FMOVZ
                        |inst_SEB|inst_SEH|inst_WSBH|inst_ROTR|inst_ROTRV
                        |inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH |inst_SHLLV_PH 
                        |inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                        |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH
                        |inst_SHRA_R_PH | inst_SHRAV_R_PH |inst_SHRA_R_W | inst_SHRAV_R_W 
                        |inst_BITREV | inst_REPLV_QB | inst_REPLV_PH | inst_ABSQ_S_PH | inst_ABSQ_S_W
                        |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR | inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR
                        |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA | inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR
                        |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA 
                        ;

assign decsrc1_DSPCtl = inst_RDDSP | inst_BPOSEGE32;   
assign decsrc1_imm    = inst_REPL_QB | inst_REPL_PH;  


assign decsrc1_fs     = 1'b0;

wire   decsrc2_fcr;
assign decsrc2_fcr  = 1'b0;

assign decsrc2_rs   = inst_SLLV
                      |inst_SRAV|inst_SRLV|inst_ROTRV
                      |inst_MOVN|inst_MOVZ
                      |inst_SHLLV_PH | inst_SHLLV_S_PH  | inst_SHLLV_S_W | inst_SHLLV_QB
                      |inst_SHRLV_QB | inst_SHRAV_PH | inst_SHRAV_R_PH | inst_SHRAV_R_W 
                      |inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W
                      |inst_EXTPV | inst_EXTPDPV | inst_SHILOV |inst_MTHLIP
                      ;

assign decsrc2_rt   = inst_ADD|inst_ADDU|inst_AND|inst_BEQ|inst_BEQL
                      |inst_BNE|inst_BNEL/*|inst_CTC1*/
                      |inst_DIV|inst_DIVU
                      |inst_MULT
                      |inst_MULTU|inst_NOR|inst_OR|inst_SUB|inst_SUBU
                      |inst_TEQ|inst_TGE|inst_TGEU|inst_TLT|inst_TLTU|inst_TNE
                      |inst_XOR|inst_MTC0
                      |inst_SB|inst_SC|inst_SH|inst_SW
                      |inst_SLT|inst_SLTU| inst_PREFX
                      |inst_LWL|inst_LWR|inst_SWL|inst_SWR
                      |inst_MUL|inst_MADD|inst_MSUB|inst_MADDU|inst_MSUBU|inst_INS
                      |inst_RDPGPR |inst_WRPGPR 
                      |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB | inst_CMP_EQ_PH | inst_CMP_LT_PH 
                      |inst_CMP_LE_PH| inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                      |inst_PICK_QB | inst_PICK_PH  |inst_PACKRL_PH
                      |inst_MULEU_S_PH_QBL |inst_MULEU_S_PH_QBR | inst_MULQ_RS_PH 
                      |inst_MULEQ_S_W_PHL | inst_MULEQ_S_W_PHR 
                      |inst_MAQ_S_W_PHL| inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                      |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                      |inst_DPSQ_S_W_PH|inst_DPSQ_SA_L_W|inst_MULSAQ_S_W_PH 
                      |inst_LBUX | inst_LHX | inst_LWX 
                      |inst_ADDQ_PH| inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB
                      |inst_ADDU_S_QB |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W
                      |inst_SUBU_QB | inst_SUBU_S_QB | inst_ADDSC | inst_ADDWC |inst_MODSUB 
                      |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                      |inst_INSV ; 
    
assign decsrc2_imm  = inst_ADDI|inst_ADDIU|inst_ANDI
                      |inst_LUI|inst_ORI|inst_SLL|inst_SLTI|inst_SLTIU
                      |inst_SRA|inst_SRL|inst_TEQI|inst_TGEI|inst_TGEIU
                      |inst_TLTI|inst_TLTIU|inst_TNEI|inst_XORI|inst_TLBP
                      |inst_EXT|inst_ROTR
                      |inst_SHLL_QB | inst_SHLL_PH |inst_SHLL_S_PH |  inst_SHLL_S_W 
                      |inst_SHRL_QB | inst_SHRA_PH  |inst_SHRA_R_PH  |inst_SHRA_R_W 
                      |inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W |inst_EXTR_S_H 
                      |inst_EXTP|inst_EXTPDP
                      |inst_SHILO | inst_RDDSP |inst_WRDSP
                      |inst_JAL|inst_BGEZAL|inst_BGEZALL|inst_BLTZAL|inst_BLTZALL
                      |inst_JR | inst_JR_HB | inst_JALR | inst_JALR_HB;

assign decsrc2_ft   = 1'b0;

wire  decsrc2_fs    = 1'b0;

assign decdest_fcr  = 1'b0;
    
assign decdest_fcr31    = 1'b0;
    
assign decdest_fd       = 1'b0;
    
assign decdest_gr31     =inst_JAL|inst_BGEZAL|inst_BGEZALL|inst_BLTZAL|inst_BLTZALL;
    
assign decdest_rd       =inst_ADD|inst_ADDU|inst_AND
                         |inst_JALR|inst_JALR_HB|inst_MFHI|inst_MFLO|inst_NOR|inst_OR
                         |inst_SLL|inst_SLLV|inst_SLT|inst_SLTU|inst_SRA|inst_SRAV
                         |inst_SRL|inst_SRLV|inst_SUB|inst_SUBU|inst_XOR
                         |inst_MUL|inst_CLO|inst_CLZ|inst_MOVF|inst_MOVT|inst_MOVN|inst_MOVZ
                         |inst_SEB|inst_SEH|inst_WSBH|inst_ROTR|inst_ROTRV
                         |inst_RDPGPR| inst_WRPGPR 
                         |inst_SHLL_QB | inst_SHLLV_QB | inst_SHLL_PH |inst_SHLLV_PH 
                         |inst_SHLL_S_PH | inst_SHLLV_S_PH | inst_SHLL_S_W | inst_SHLLV_S_W
                         |inst_SHRL_QB | inst_SHRLV_QB | inst_SHRA_PH | inst_SHRAV_PH
                         |inst_SHRA_R_PH | inst_SHRAV_R_PH |inst_SHRA_R_W | inst_SHRAV_R_W 
                         |inst_BITREV | inst_REPLV_QB | inst_REPLV_PH | inst_REPL_QB | inst_REPL_PH 
                         |inst_CMPGU_EQ_QB | inst_CMPGU_LT_QB  | inst_CMPGU_LE_QB//add 
                         |inst_PICK_QB | inst_PICK_PH  |inst_PACKRL_PH |inst_MFHI_DSP|inst_MFLO_DSP
                         |inst_RDDSP |inst_MULEU_S_PH_QBL |inst_MULEU_S_PH_QBR 
                         |inst_MULQ_RS_PH | inst_MULEQ_S_W_PHL | inst_MULEQ_S_W_PHR 
                         |inst_ADDQ_PH| inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB
                         |inst_ADDU_S_QB |inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W
                         |inst_SUBU_QB | inst_SUBU_S_QB | inst_ADDSC | inst_ADDWC |inst_MODSUB | inst_RADDU 
                         |inst_PRECRQ_QB_PH | inst_PRECRQ_PH_W | inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH
                         |inst_PRECEQ_W_PHL | inst_PRECEQ_W_PHR | inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR
                         |inst_PRECEQU_PH_QBLA | inst_PRECEQU_PH_QBRA | inst_PRECEU_PH_QBL | inst_PRECEU_PH_QBR
                         |inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA|inst_LBUX|inst_LHX|inst_LWX 
                         |inst_ABSQ_S_PH | inst_ABSQ_S_W;
    
assign decdest_rt       =inst_ADDI|inst_ADDIU|inst_ANDI
                         |inst_LUI|inst_ORI|inst_SLTI|inst_SLTIU|inst_XORI
                         |inst_LB|inst_LBU|inst_LH
                         |inst_LHU|inst_LL|inst_LW
                         |inst_MFC0|inst_SC|inst_CFC1|inst_MFC1
                         |inst_LWL|inst_LWR
                         |inst_EXT|inst_INS|inst_RDHWR|inst_DI |inst_EI
                         |inst_INSV
                         |inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W |inst_EXTR_S_H 
                         |inst_EXTRV_S_H | inst_EXTRV_W | inst_EXTRV_R_W | inst_EXTRV_RS_W
                         |inst_EXTP |inst_EXTPV | inst_EXTPDP | inst_EXTPDPV 
                         ;

assign decdest_fs       = 1'b0;

wire decdest_ft;
assign decdest_ft       = 1'b0;

/*
assign decdest_ac       =inst_SHILO|inst_SHILOV|inst_MTHLIP
                        |inst_MAQ_S_W_PHL| inst_MAQ_S_W_PHR | inst_MAQ_SA_W_PHL | inst_MAQ_SA_W_PHR | inst_DPAU_H_QBL
                        |inst_DPAU_H_QBR | inst_DPSU_H_QBL | inst_DPSU_H_QBR | inst_DPAQ_S_W_PH | inst_DPAQ_SA_L_W
                        |inst_DPSQ_S_W_PH|inst_DPSQ_SA_L_W|inst_MULSAQ_S_W_PH ;
wire[7:0] decdest_regac = (rs[1:0]== 2'b00) ? 8'h20:
                          (rs[1:0]== 2'b01) ? 8'h22:
                          (rs[1:0]== 2'b10) ? 8'h24: 8'h26;
*/

assign decdest_DSPcc  = inst_CMPU_EQ_QB |inst_CMPU_LT_QB | inst_CMPU_LE_QB|
                         inst_CMP_LT_PH | inst_CMP_EQ_PH |inst_CMP_LE_PH  ;

assign decimm_pc        =irbus_adei|irbus_tlbii|irbus_tlbir|irbus_ibe|irbus_dib;
assign decimm_immll     =inst_LUI;

assign decimm_immra     =inst_ADDI|inst_ADDIU
                         |inst_SLTI|inst_SLTIU|inst_TEQI|inst_TGEI
                         |inst_TGEIU|inst_TLTI|inst_TLTIU|inst_TNEI
                         |inst_LB|inst_LH|inst_LW|inst_LWC1|inst_LDC1|inst_LBU|inst_LHU|inst_LL
                         |inst_SB|inst_SH|inst_SW|inst_SWC1|inst_SDC1|inst_SC | inst_PREF
                         |inst_CACHE0|inst_CACHE1|inst_CACHE3|inst_CACHE4|inst_CACHE5|inst_CACHE6
                         |inst_CACHE7|inst_SYNCI
                         |inst_CACHE8|inst_CACHE9|inst_CACHE10|inst_CACHE11|inst_CACHE12|inst_CACHE13|inst_CACHE14
                         |inst_CACHE15
                         |inst_CACHE16|inst_CACHE17|inst_CACHE18|inst_CACHE19|inst_CACHE20|inst_CACHE21|inst_CACHE22
                         |inst_CACHE23
                         |inst_CACHE24|inst_CACHE25|inst_CACHE26|inst_CACHE27|inst_CACHE28|inst_CACHE29|inst_CACHE30
                         |inst_CACHE31
                         |inst_LWL|inst_LWR|inst_SWL|inst_SWR;

assign decimm_immrl     =inst_ANDI|inst_ORI|inst_XORI;

assign decimm_offset    =inst_BC1F|inst_BC1T|inst_BC1FL|inst_BC1TL
                         |inst_BEQ|inst_BNE|inst_BLEZ|inst_BGTZ
                         |inst_BLTZ|inst_BGEZ
                         |inst_BEQL|inst_BNEL|inst_BLEZL|inst_BGTZL
                         |inst_BLTZL|inst_BGEZL
                         |inst_BPOSEGE32;


assign decimm_sa        =inst_SLL|inst_SRA|inst_SRL|inst_ROTR;
    
assign decimm_rd        =inst_MFC0|inst_MTC0;
assign decimm_entryhi   =inst_TLBP;
assign decimm_rd_sa   =inst_EXT | inst_INS;
assign decimm_status    =inst_DI  | inst_EI;
assign decimm_rs        = inst_SHLL_QB | inst_SHLL_PH |inst_SHLL_S_PH |  inst_SHLL_S_W 
                         |inst_SHRL_QB | inst_SHRA_PH  |inst_SHRA_R_PH  |inst_SHRA_R_W 
                         |inst_EXTR_W | inst_EXTR_R_W | inst_EXTR_RS_W |inst_EXTR_S_H
                         |inst_EXTP|inst_EXTPDP;
assign decimm_rt_rd     =inst_WRDSP;
assign decimm_rs_rt1    =inst_SHILO;

wire [8:0] decimm_RDHWR = rd_d[5'h0] ? 9'b101111001 :
                          rd_d[5'h1] ? 9'b100000000 :
                          rd_d[5'h2] ? 9'b101001000 : 9'b100000001;

wire decinst_jr;
wire decinst_link;
wire decbus_ex;
wire decbus_imm_hi_en;
assign decbus_imm_hi_en = decimm_pc | decimm_immll | decimm_immra | decimm_entryhi |
                          decimm_immrl | inst_RDHWR | decimm_status | inst_RDHWR |
                          decimm_offset | decinst_jr | decinst_link;

wire [15:0] decbus_imm_hi;
assign decbus_imm_hi = ((decimm_offset | decinst_jr | decinst_link) & ~decbus_ex) ? irbus_taken_addr[31:16] :
                       (decimm_pc) ? irbus_pc[31:16] :
                       (decimm_immll) ? imm[15:0] :
                       (decimm_immra) ? {16{imm[15]}} :
                       (decimm_entryhi) ? entryhi_vpn2[18:3] : 16'b0;
                       //last is decimm_immrl | inst_RDHWR | decimm_status | inst_RDHWR

wire decbus_imm_low_en;
assign decbus_imm_low_en = decimm_pc | decimm_immll | decimm_immra | decimm_entryhi |
                           decimm_immrl | decimm_sa | decimm_rd | inst_RDHWR | decimm_status | decimm_rd_sa |
                           decimm_rs | inst_RDDSP | inst_REPL_QB | inst_REPL_PH | decimm_rt_rd | decimm_rs_rt1 |
                           decimm_offset | decinst_jr | decinst_link;

wire [15:0] decbus_imm_low;
assign decbus_imm_low = ((decimm_offset | decinst_jr | decinst_link) & ~decbus_ex) ? irbus_taken_addr[15:0] :
                        (decimm_pc) ? irbus_pc[15:0] :
                        (decimm_immra) ? imm[15:0] :
                        (decimm_entryhi) ? {entryhi_vpn2[2:0], 13'b0} :
                        (decimm_immrl) ? imm[15:0] : 
                        (decimm_sa) ? {11'b0, sa[4:0]} :
                        (decimm_rd) ? {8'b0, rd[4:0], cp0_sel[2:0]} :  
                        (inst_RDHWR) ? {7'b0, decimm_RDHWR} :
                        (decimm_status) ? {8'b0, 8'b01100000} :
                        (decimm_rd_sa) ? {6'b0, rd[4:0], sa[4:0]} :
                        (decimm_rs) ? {11'b0, rs[4:0]} :
                        (inst_RDDSP) ? {6'b0, rs[4:0], rt[4:0]} :
                        (inst_REPL_QB) ? {8'b0, rs[2:0], rt[4:0]} :
                        (inst_REPL_PH) ? {{6{rs[4]}}, rs[4:0], rt[4:0]} :
                        (decimm_rt_rd) ? {6'b0, rt[4:0], rd[4:0]} :
                        (decimm_rs_rt1) ? {10'b0, rs[4:0], rt[4]} : 16'b0;//last is decimm_immll

assign decbus_imm = {decbus_imm_hi, decbus_imm_low};

wire decinst_blikely;// the decoded instruction is branch likely
assign decinst_blikely = inst_BC1FL|inst_BC1TL
                         |inst_BEQL|inst_BNEL|inst_BLEZL|inst_BGTZL
                         |inst_BLTZL|inst_BGEZL|inst_BLTZALL|inst_BGEZALL;
assign decinst_jr  = (inst_JR | inst_JR_HB) | inst_JALR | inst_JALR_HB; 

assign decinst_link  =inst_JAL|inst_BGEZAL|inst_BGEZALL|inst_BLTZAL|inst_BLTZALL;

wire decinst_br = decimm_offset | decinst_jr | decinst_link | inst_J | inst_ERET | inst_DERET 
                  //HAVE_DSP_UNIT
                   | inst_BPOSEGE32
                   ; 

wire[3:0] fmt_plus;
plus1  fmt_plus1(.in(decbus_fmt[2:0]),.out(fmt_plus));
wire[7:0] fcc_rnum ={temp[7], 2'b11, temp[4], fmt_plus[3:0]}; 
                    //{temp[7:4],fmt_plus[3:0]} | 8'h60; //rename the fcr31's cc bits to fcr number 33-40.  
                                                       //32 for fcr0 
assign decbus_src1[7:0]=({8{decsrc1_rs }} & {temp[7:5],rs[4:0]})|
                        ({8{decsrc1_rt }} & {temp[7:5],rt[4:0]})|
                        ({8{decsrc1_cmp}} & fcc_rnum)|
                        ({8{decsrc1_fs }} & {temp[7], 1'b1, temp[5], rd[4:0]})| //({temp[7:5],rd[4:0]}|8'h40))|
                        ({8{decsrc1_DSPCtl}} & 8'h28)|
                //      ({8{decsrc1_ac}}  & decsrc1_regac)|
                        ({8{decsrc1_imm}} & 8'h3f);
                        
assign decbus_src2[7:0]=({8{decsrc2_rs }} & {temp[7:5],rs[4:0]})|
                        ({8{decsrc2_rt }} & {temp[7:5],rt[4:0]})|
                        ({8{decsrc2_imm}} & 8'h3f)|
                        ({8{decsrc2_ft }} & {temp[7], 1'b1, temp[5], rt[4:0]})| //({temp[7:5],rt[4:0]}|8'h40))|
                        ({8{decsrc2_cmp}} & fcc_rnum)|
                        ({8{decsrc2_fcr}} & {temp[7], 2'b11, rd[4:0]})| //({temp[7:5],rd[4:0]}|8'h60))|
                        ({8{decsrc2_fs }} & {temp[7], 1'b1, temp[5], rd[4:0]}); //({temp[7:5],rd[4:0]}|8'h40))
                        

assign decbus_dest[7:0]= (decdest_fcr)   ? {temp[7], 2'b11, rd[4:0]}//{temp[7:5],rd[4:0]}|8'h60
                       : (decdest_fcr31) ? fcc_rnum
                       : (decdest_fd)    ? {temp[7], 1'b1, temp[5], sa[4:0]}//{temp[7:5],sa[4:0]}|8'h40
                       : (decdest_gr31)  ? 8'h1f
                       : (decdest_rd)    ? {temp[7:5],rd[4:0]}
                       : (decdest_rt)    ? {temp[7:5],rt[4:0]}
                       : (decdest_fs)    ? {temp[7], 1'b1, temp[5], rd[4:0]}//{temp[7:5],rd[4:0]}|8'h40 
                       : (decdest_ft)    ? {temp[7], 1'b1, temp[5], rt[4:0]}//{temp[7:5],rt[4:0]}|8'h40
                 //      : (decdest_ac)    ? decdest_regac
                       : (decdest_DSPcc) ? 8'h29 
                       : 8'b0;      

wire decdest_double_fmt = 1'b0;
                         
wire decdest_double    =  1'b0;

wire decsrc2_double_fmt   = 1'b0;
                         
assign decbus_double_src1 = decsrc1_fs &(rs_d['h11] | rs_d['h15]);
                          
assign decbus_double_src2 =  decsrc2_double_fmt&(rs_d['h11] | rs_d['h15]) | inst_SDC1;
assign decbus_double_dest =  decdest_double_fmt&(rs_d['h11] | rs_d['h15]) | decdest_double; 
                          
//assign decbus_ex = decbus_adei | decbus_tlbii | decbus_tlbir | decbus_ibe | decbus_ri | decbus_cpui | 
//                   decbus_dib | decbus_sdbbp | decbus_sys |decbus_bp ;  
assign decbus_ex = decbus_adei | decbus_tlbii | decbus_tlbir | decbus_ibe | decbus_ri | decbus_cpui | 
                   decbus_dib;  

wire decbus_mm_op =  inst_DI | inst_EI | inst_SYNC | inst_ERET | inst_TLBP |
                     inst_TLBR | inst_TLBWI | inst_TLBWR | inst_MFC0 | inst_MTC0 |
                     inst_SYNCI | inst_DERET | 
                     inst_LBUX | inst_LHX | inst_LWX | 
                     inst_CACHE0 | inst_CACHE8 | inst_CACHE16 | inst_CACHE28 |
                     inst_CACHE1 | inst_CACHE5 | inst_CACHE9  | inst_CACHE17 | inst_CACHE21 | inst_CACHE29 |
                     inst_LB | inst_LH | inst_LW | inst_LWL | inst_LWR | inst_SWL | inst_SWR |
                     inst_LBU | inst_LHU | inst_LL | inst_SC | inst_PREF |inst_PREFX | inst_SB | inst_SH | inst_SW ;
                   
wire decbus_alu_one = inst_CLO | inst_CLZ | inst_EXT | inst_INS | inst_WSBH | inst_ROTR | inst_SEB | inst_SEH  |
                      inst_MOVN | inst_MOVZ | inst_SLL | inst_SRL | inst_SRA | inst_ADD |inst_ADDU | inst_SUB |
                      inst_SUBU | inst_AND | inst_OR | inst_XOR | inst_NOR | inst_TEQ | inst_TNE |inst_TLT | 
                      inst_TLTU | inst_TGE | inst_TGEU | inst_SLT | inst_SLTU | inst_J | inst_JR | inst_JAL |
                      inst_JALR | inst_BEQ |
                      inst_BNE| inst_BLEZ| inst_BGTZ| inst_BLTZ| inst_BGEZ| inst_BLTZAL| inst_BGEZAL| inst_BEQL| 
                      inst_BNEL | inst_JR_HB | inst_JALR_HB|
                      inst_BLEZL | inst_BGTZL | inst_BLTZL | inst_BGEZL | inst_BLTZALL | 
                      inst_BGEZALL |inst_LUI|inst_ORI|inst_ADDI|inst_ADDIU|inst_ANDI|inst_SLTI |
                      inst_SLTIU|inst_TEQI | inst_TNEI |inst_TLTI |inst_TLTIU | inst_TGEI | inst_TGEIU | 
                      inst_SLTI|
                      inst_RDPGPR | inst_WRPGPR |
                      inst_SLLV | inst_SRLV | inst_SRAV | inst_ROTRV|inst_XORI
                      //HAVE_DSP_UNIT
                      |inst_ADDQ_PH | inst_ADDQ_S_PH | inst_ADDQ_S_W | inst_ADDU_QB| inst_ADDU_S_QB |
                      inst_SUBQ_PH | inst_SUBQ_S_PH | inst_SUBQ_S_W | inst_SUBU_QB| inst_SUBU_S_QB |
                      inst_ADDSC| inst_ADDWC| inst_MODSUB| inst_RADDU | inst_ABSQ_S_PH | inst_ABSQ_S_W | 
                      inst_PRECRQ_QB_PH  | inst_PRECRQ_PH_W |
                      inst_PRECRQ_RS_PH_W | inst_PRECRQU_S_QB_PH| inst_PRECEQ_W_PHL   | inst_PRECEQ_W_PHR   |
                      inst_PRECEQU_PH_QBL | inst_PRECEQU_PH_QBR | inst_PRECEQU_PH_QBLA| inst_PRECEQU_PH_QBRA|
                      inst_PRECEU_PH_QBL  | inst_PRECEU_PH_QBR  | inst_PRECEU_PH_QBLA | inst_PRECEU_PH_QBRA |
                      inst_SHLL_QB        | inst_SHLLV_QB   |inst_SHLL_PH|inst_SHLLV_PH| inst_SHLL_S_PH  |
                      inst_SHLLV_S_PH | inst_SHLL_S_W  | inst_SHLLV_S_W | inst_SHRL_QB| inst_SHRLV_QB |inst_SHRA_PH |       
                      inst_SHRAV_PH   | inst_SHRA_R_PH| inst_SHRAV_R_PH | inst_SHRA_R_W | inst_SHRAV_R_W |
                      inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB | inst_CMPGU_EQ_QB| inst_CMPGU_LT_QB|
                      inst_CMPGU_LE_QB | inst_CMP_EQ_PH  | inst_CMP_LT_PH  | inst_CMP_LE_PH  | inst_PICK_QB  | 
                      inst_PICK_PH | inst_PACKRL_PH | inst_BITREV | inst_INSV | inst_REPL_QB | inst_REPLV_QB | 
                      inst_REPL_PH | inst_REPLV_PH | inst_BPOSEGE32 | inst_RDDSP | inst_WRDSP  
                      ;
wire decbus_falu =    inst_ADD_F|inst_ROUND_L|inst_CVT_S|inst_C_F|inst_C_SF
                      |inst_SUB_F|inst_TRUNC_L|inst_CVT_D|inst_C_UN|inst_C_NGLE
                      |inst_MUL_F|inst_CEIL_L|inst_C_EQ|inst_C_SEQ
                      |inst_DIV_F|inst_FLOOR_L|inst_C_UEQ|inst_C_NGL
                      |inst_SQRT_F|inst_ROUND_W|inst_CVT_W|inst_C_OLT|inst_C_LT
                      |inst_ABS_F|inst_TRUNC_W|inst_CVT_L|inst_C_ULT|inst_C_NGE
                      |inst_MOVE|inst_CEIL_W|inst_C_OLE|inst_C_LE
                      |inst_NEG_F|inst_FLOOR_W|inst_C_ULE|inst_C_NGT
                      |inst_BC1F|inst_BC1T|inst_BC1FL|inst_BC1TL
                      |inst_FMOVF|inst_FMOVT
                      |inst_RECIP|inst_RECIP
                      |inst_RSQRT
                      |inst_MFC1|inst_MTC1|inst_CFC1|inst_CTC1
                      |inst_LWC1|inst_LDC1|inst_SWC1|inst_SDC1
                      |inst_MOVF|inst_MOVT|inst_FMOVN|inst_FMOVZ
                      ;
wire decbus_block_end = (decinst_br) &~decbus_bd;
wire decbus_acc_op  = inst_MULT| inst_MULTU |inst_MADD| inst_MADDU| inst_MSUB| inst_MSUBU|inst_DIV |
                      inst_DIVU | inst_MFHI | inst_MFLO | inst_MTHI | inst_MTLO | inst_MUL
                      //HAVE_DSP_UNIT
                      |inst_MULEU_S_PH_QBL  | inst_MULEU_S_PH_QBR  | inst_MULQ_RS_PH      | inst_MULEQ_S_W_PHL   |
                      inst_MULEQ_S_W_PHR   | inst_MAQ_S_W_PHL     | inst_MAQ_S_W_PHR     | inst_MAQ_SA_W_PHL    |
                      inst_MAQ_SA_W_PHR    | inst_DPAU_H_QBL      | inst_DPAU_H_QBR      | inst_DPSU_H_QBL      |
                      inst_DPSU_H_QBR      | inst_DPAQ_S_W_PH     | inst_DPAQ_SA_L_W     | inst_DPSQ_S_W_PH     |
                      inst_DPSQ_SA_L_W     | inst_MULSAQ_S_W_PH   | inst_EXTR_W     | inst_EXTR_R_W   |
                      inst_EXTR_RS_W  | inst_EXTR_S_H   | inst_EXTRV_S_H  | inst_EXTRV_W    | inst_EXTRV_R_W  |
                      inst_EXTRV_RS_W | inst_EXTP       | inst_EXTPV      | inst_EXTPDP     | inst_EXTPDPV    |
                      inst_SHILO      | inst_SHILOV| inst_MTHLIP |
                      inst_MFLO_DSP | inst_MFHI_DSP | inst_MTLO_DSP | inst_MTHI_DSP
                      ;
wire  decbus_stall = inst_CTC1 | inst_MOVN | inst_MOVZ | inst_MOVT | inst_MOVF | inst_FMOVN | inst_FMOVZ 
                     |inst_FMOVT | inst_FMOVF | inst_WRDSP | inst_MTHLIP | inst_ADDSC;

wire decbus_wait = inst_SYNC | inst_CTC1 | inst_CFC1 | inst_INS 
                   //HAVE_DSP_UNIT
                   |inst_PICK_QB | inst_PICK_PH | inst_RDDSP | inst_ADDWC
                   |inst_CMPU_EQ_QB | inst_CMPU_LT_QB | inst_CMPU_LE_QB 
                   |inst_CMP_EQ_PH | inst_CMP_LT_PH | inst_CMP_LE_PH                         
                   |inst_WRDSP | inst_EXTPDP | inst_EXTPDPV | inst_MTHLIP | inst_EXTP | inst_EXTPV |inst_RDDSP 
                   |inst_BPOSEGE32 | inst_INSV 
                   ;
wire decbus_blikely     = decinst_blikely;

assign decbus[144]        = irbus_int;
assign decbus[143]        = decbus_imm_hi_en;
assign decbus[142]        = decbus_imm_low_en;
assign decbus[141]        = decbus_block_end;
assign decbus[140]        = decbus_blikely;
assign decbus[139]        = decbus_wait;
assign decbus[138]        = decbus_stall;
assign decbus[137]        = decbus_acc_op;
assign decbus[136]        = decbus_falu;
assign decbus[135]        = decbus_alu_one;
assign decbus[134]        = decbus_mm_op;
assign decbus[133]        = decbus_double_dest;
assign decbus[132]        = decbus_double_src2;
assign decbus[131]        = decbus_double_src1;
assign decbus[130]        = irbus_block_begin ;
assign decbus[129]        = decbus_nop ;
assign decbus[128:127]    = decbus_ac;
assign decbus[126:125]=irbus_other_link;
assign decbus[124:123]=irbus_rashead;
assign decbus[122:115]=irbus_gshare;
assign decbus[114]    =decbus_valid;
assign decbus[113]    =decbus_sdbbp;
assign decbus[112]    =decbus_dib;
assign decbus[111:104]= decbus_bd &decinst_br ? 8'h11: decbus_op[7:0];
assign decbus[103:99]=decbus_fmt[4:0];
assign decbus[98:91] =decbus_src1[7:0];
assign decbus[90:83]  =decbus_src2[7:0];
assign decbus[82:75]  =decbus_dest[7:0];
assign decbus[74:43]  = decbus_imm[31:0]; 
assign decbus[42:11]  = decbus_pc[31:0];
assign decbus[10:9]   = (decimm_offset & ~decbus_ex)? irbus_pred_status[1:0] : decbus_ce[1:0];
assign decbus[8]      =decbus_bd;
assign decbus[7]      =decbus_adei;
assign decbus[6]      =decbus_tlbii;
assign decbus[5]      =decbus_tlbir;
assign decbus[4]      =decbus_ibe;
assign decbus[3]      =decbus_ri;
assign decbus[2]      =decbus_cpui;
assign decbus[1]      =decbus_sys;
assign decbus[0]      =decbus_bp;    


endmodule 

module plus1(in, out);
input [2:0] in;
output[3:0] out;

reg   [3:0] out;
always @ (in) 
case(in)
    3'b000: out = 4'b0001; 
    3'b001: out = 4'b0010; 
    3'b010: out = 4'b0011; 
    3'b011: out = 4'b0100; 
    3'b100: out = 4'b0101; 
    3'b101: out = 4'b0110; 
    3'b110: out = 4'b0111; 
    3'b111: out = 4'b1000; 
endcase
endmodule
