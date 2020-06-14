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
module godson_hb_module(
    CLOCK,
    RESET,
    DEBUG_MODE,
    HB_REQBUS,
    HB_ICOMPBUS,
    HB_DCOMPBUS,
    LOADDATA,
    LOADDATA_QID,
    LOADDATA_VALID,
    LOADDATA_H_VALID,
    LOADDATA_H,
    BYTELANE,
    COMMITBUS,
    BRBUS_TO_HB,
    ICACHERDY_AFTER_DERET,
    FIRST_MEM_PASS_AFTER_DERET,
    HB_DIT,
    HB_DIB,
    HB_DDT,
    HB_DDBL,
    HB_DDBS,
    HB_DDBLIMPR,
    HB_DDBSIMPR,
    DRESBUS_FROM_HB,
    HB_LOAD_ADDR_MATCH,
    hb_dbs_bs_o
);
input CLOCK;
input RESET;
input DEBUG_MODE;
input[78:0] HB_REQBUS;
input[40:0] HB_ICOMPBUS;
input[`Lhb_dcompbus-1:0] HB_DCOMPBUS;
input[50:0] COMMITBUS;
input[47:0] BRBUS_TO_HB;
//input DSS_ENABLE;//DSS_ENABLE;
input[1:0] ICACHERDY_AFTER_DERET;
input FIRST_MEM_PASS_AFTER_DERET;
output[1:0] HB_DIT;
output[1:0] HB_DIB;
output HB_DDT;
output HB_DDBL;
output HB_DDBS;
output HB_DDBLIMPR;
output HB_DDBSIMPR;
output[64:0] DRESBUS_FROM_HB;          
 
input[31:0] LOADDATA;
input[ 3:0] LOADDATA_QID;
input LOADDATA_VALID;
input LOADDATA_H_VALID;
input[31:0] LOADDATA_H;
input [3:0] BYTELANE;
output [1:0] HB_LOAD_ADDR_MATCH;
output [`DBKP_NUM-1:0] hb_dbs_bs_o;

wire[31:0] RREG_FROM_IB;
wire[31:0] RREG_FROM_DB;
wire[31:0] rreg_from_hb;
wire[7:0] COMMITBUS_OP;
wire[31:0] COMMITBUS_PC;
wire[5:0] COMMITBUS_EXCODE;
wire COMMITBUS_EX;
wire[3:0] COMMITBUS_QID;
//added by xucp

assign COMMITBUS_QID=COMMITBUS[50:47];
assign COMMITBUS_OP=COMMITBUS[46:39];
assign COMMITBUS_PC=COMMITBUS[38:7];
assign COMMITBUS_EXCODE=COMMITBUS[6:1];
assign COMMITBUS_EX=COMMITBUS[0];

wire brbus_valid   ;
wire brbus_err     ;
wire[7:0] brbus_op ;
wire[5:0] brbus_to_db;
assign brbus_valid = BRBUS_TO_HB[0];
assign brbus_err   = BRBUS_TO_HB[1];
assign brbus_op    = BRBUS_TO_HB[9:2];
assign brbus_to_db = BRBUS_TO_HB[47:42];
          
ib ib_ins(.CLOCK(CLOCK),
          .RESET(RESET),
          .DEBUG_MODE(DEBUG_MODE),
          .HB_REQBUS(HB_REQBUS),
          .HB_ICOMPBUS(HB_ICOMPBUS),
          .COMMITBUS_EXCODE(COMMITBUS_EXCODE),
          .COMMITBUS_EX(COMMITBUS_EX),
          .COMMITBUS_PC(COMMITBUS_PC),
          .brbus_valid(brbus_valid),
          .brbus_err(brbus_err),
          .brbus_op(brbus_op),
          //DSS_ENABLE,//DSS_ENABLE,
          .ICACHERDY_AFTER_DERET(ICACHERDY_AFTER_DERET),
          .RREG_FROM_IB(RREG_FROM_IB),
          .HB_DIT(HB_DIT),
          .HB_DIB(HB_DIB));
db db_ins(.CLOCK(CLOCK),
          .RESET(RESET),
          .DEBUG_MODE(DEBUG_MODE),
          .HB_REQBUS(HB_REQBUS),
          .HB_DCOMPBUS(HB_DCOMPBUS),
          .LOADDATA(LOADDATA),
          .LOADDATA_QID(LOADDATA_QID),
          .LOADDATA_VALID(LOADDATA_VALID),
          .LOADDATA_H_VALID(LOADDATA_H_VALID),
          .LOADDATA_H(LOADDATA_H),
          .BYTELANE(BYTELANE),
          .brbus_to_db_i(brbus_to_db),
          .COMMITBUS_QID(COMMITBUS_QID),
          .COMMITBUS_EX(COMMITBUS_EX),
          .COMMITBUS_EXCODE(COMMITBUS_EXCODE),
          //DSS_ENABLE,//DSS_ENABLE,
          .FIRST_MEM_PASS_AFTER_DERET(FIRST_MEM_PASS_AFTER_DERET),
          .RREG_FROM_DB(RREG_FROM_DB),
          .HB_DDT(HB_DDT),
          .HB_DDBL(HB_DDBL),
          .HB_DDBS(HB_DDBS),
          .HB_DDBLIMPR(HB_DDBLIMPR),
          .HB_DDBSIMPR(HB_DDBSIMPR),
          .HB_LOAD_ADDR_MATCH(HB_LOAD_ADDR_MATCH),
          .hb_dbs_bs_o(hb_dbs_bs_o));

assign rreg_from_hb=(HB_REQBUS[17])?RREG_FROM_IB:RREG_FROM_DB;          

assign DRESBUS_FROM_HB[64:33]=(HB_REQBUS[7])?rreg_from_hb:32'b0; 
assign DRESBUS_FROM_HB[32:1]= (HB_REQBUS[7])?32'b0:rreg_from_hb;
assign DRESBUS_FROM_HB[0]=HB_REQBUS[0]&!HB_REQBUS[78]&!HB_REQBUS[77];            
endmodule
          
//*******************************************************************
//copyrignt                         
//file name                                 ib.v
//module name                               ib
//author                                    cao fei
//version                                   0.1
//function description                      instruction breakpoint 
//modification comment/date/author          
//testbench file
//test result file
//********************************************************************
// `define EX_DIB          6'D51
// `define EX_DDBL         6'D52
// `define EX_DDBS         6'D53
// `define EX_DDBLIMPR     6'D54
// `define EX_DDBSIMPR     6'D56
// `define OP_LW       8'H92
// `define OP_SW       8'H9a
module ib(CLOCK,
          RESET,
          DEBUG_MODE,
          HB_REQBUS,
          HB_ICOMPBUS,
          COMMITBUS_EXCODE,
          COMMITBUS_EX,
          COMMITBUS_PC,
          brbus_valid,
          brbus_err,
          brbus_op,
          //DSS_ENABLE,
          ICACHERDY_AFTER_DERET,
          RREG_FROM_IB,
          HB_DIT,
          HB_DIB);
parameter ibpnum=`IBKP_NUM;//instruction break point number
input CLOCK;
input RESET;
input DEBUG_MODE;
input[78:0] HB_REQBUS;
input[40:0] HB_ICOMPBUS;
input[5:0] COMMITBUS_EXCODE;
input COMMITBUS_EX;
input[31:0]  COMMITBUS_PC;
input  brbus_valid;
input  brbus_err  ;
input[7:0]   brbus_op   ;
//input DSS_ENABLE;
input[1:0] ICACHERDY_AFTER_DERET;
output[31:0] RREG_FROM_IB;
output[1:0] HB_DIT;
output[1:0] HB_DIB;

//ib_reg
reg[31:0] ibs;
reg[32*ibpnum-1:0] iba;
reg[32*ibpnum-1:0] ibm;
reg[32*ibpnum-1:0] ibasid;
reg[32*ibpnum-1:0] ibc;
integer i,j,k,l,m,n,t,p;

wire hb_reqbus_ades;
wire hb_reqbus_adel;
wire[31:0] hb_reqbus_value;
wire[7:0] hb_reqbus_op;//store 8'h9a OP_SW     load 8'h92 OP_LW
wire[31:0] hb_reqbus_addr;
wire[3:0] hb_reqbus_qid;//no use
wire hb_reqbus_valid;

wire[31:0] pc;
wire[7:0] asid;
wire hb_ivalid;
wire [31:0] pc_next; //for two-issuing. when pc is cachelen boundary, then only fetch one instruction

wire [31:0] compare_commit_pc = {32{COMMITBUS_EX}}& COMMITBUS_PC;

wire[3:0] ibs_reset = ibpnum;

reg[31:0] ib_out;
reg[ibpnum*4:0]  reg_rw_select;
reg[1:0] ihardbreak_skip;
wire[ibpnum-1:0] ib_match;

reg[ibpnum-1:0] ib_match_be_0;
reg[ibpnum-1:0] ib_match_te_0;

reg[ibpnum-1:0] ib_match_be_1;
reg[ibpnum-1:0] ib_match_te_1;

reg[ibpnum-1:0] ib_match_0;
reg[ibpnum-1:0] ib_match_1;

reg[ibpnum-1:0] ib_match_commit;

assign hb_reqbus_ades=HB_REQBUS[78];
assign hb_reqbus_adel=HB_REQBUS[77];
assign hb_reqbus_value=HB_REQBUS[76:45];
assign hb_reqbus_op=HB_REQBUS[44:37];
assign hb_reqbus_addr=HB_REQBUS[36:5];
assign hb_reqbus_qid=HB_REQBUS[4:1];//no use
assign hb_reqbus_valid=HB_REQBUS[0];

assign pc=  HB_ICOMPBUS[40:9];
assign asid=HB_ICOMPBUS[8:1];
assign hb_ivalid=  HB_ICOMPBUS[0];
assign pc_next = HB_ICOMPBUS[40:9] + 3'b100;


// reg addr deCODE
always@(hb_reqbus_addr or hb_reqbus_valid)
    if(hb_reqbus_valid&&hb_reqbus_addr[12])
        begin
            if({hb_reqbus_addr[11:8],hb_reqbus_addr[4:3]}==0)
                reg_rw_select[0]=1'b1;
            else 
                reg_rw_select[0]=1'b0;    
            for(k=1;k<(ibpnum+1);k=k+1)
                begin 
                for(j=0;j<4;j=j+1)
                    begin
                    if((k==hb_reqbus_addr[11:8])&&(j==hb_reqbus_addr[4:3]))
                        reg_rw_select[k+j*ibpnum]=1'b1;
                    else 
                        reg_rw_select[k+j*ibpnum]=1'b0;
                    end
                end      
        end                    
    else
        reg_rw_select=0;

wire hbi_gating_clock;
assign hbi_gating_clock = CLOCK;
      
always@(posedge hbi_gating_clock)
    if(RESET)
        begin
            iba <= 0;
            ibm <= 0;
            ibasid <= 0;
            ibc <= 0;
        end
    else if(DEBUG_MODE&&!hb_reqbus_ades&&(hb_reqbus_op==`OP_SW))//STORE 8'h9a 
        begin
            for(t=0;t<ibpnum;t=t+1)
              begin
                if(reg_rw_select[t+1])
                    {iba[32*t+31],iba[32*t+30],iba[32*t+29],iba[32*t+28],
                     iba[32*t+27],iba[32*t+26],iba[32*t+25],iba[32*t+24],
                     iba[32*t+23],iba[32*t+22],iba[32*t+21],iba[32*t+20],
                     iba[32*t+19],iba[32*t+18],iba[32*t+17],iba[32*t+16],
                     iba[32*t+15],iba[32*t+14],iba[32*t+13],iba[32*t+12],
                     iba[32*t+11],iba[32*t+10],iba[32*t+9],iba[32*t+8],
                     iba[32*t+7],iba[32*t+6],iba[32*t+5],iba[32*t+4],
                     iba[32*t+3],iba[32*t+2],iba[32*t+1],iba[32*t]} <= hb_reqbus_value;

                if(reg_rw_select[t+ibpnum+1])
                     {ibm[32*t+31],ibm[32*t+30],ibm[32*t+29],ibm[32*t+28],
                     ibm[32*t+27],ibm[32*t+26],ibm[32*t+25],ibm[32*t+24],
                     ibm[32*t+23],ibm[32*t+22],ibm[32*t+21],ibm[32*t+20],
                     ibm[32*t+19],ibm[32*t+18],ibm[32*t+17],ibm[32*t+16],
                     ibm[32*t+15],ibm[32*t+14],ibm[32*t+13],ibm[32*t+12],
                     ibm[32*t+11],ibm[32*t+10],ibm[32*t+9],ibm[32*t+8],
                     ibm[32*t+7],ibm[32*t+6],ibm[32*t+5],ibm[32*t+4],
                     ibm[32*t+3],ibm[32*t+2],ibm[32*t+1],ibm[32*t]} <= hb_reqbus_value;
           
                if(reg_rw_select[t+2*ibpnum+1])  
                    {ibasid[32*t+7],ibasid[32*t+6],ibasid[32*t+5],ibasid[32*t+4],
                     ibasid[32*t+3],ibasid[32*t+2],ibasid[32*t+1],ibasid[32*t]} <= hb_reqbus_value[7:0];
           
                if(reg_rw_select[t+3*ibpnum+1])
                    {ibc[32*t+23],ibc[32*t+2],ibc[32*t]}<={hb_reqbus_value[23],hb_reqbus_value[2],hb_reqbus_value[0]};  
        end
   end
        
//r
always@(reg_rw_select or DEBUG_MODE or hb_reqbus_adel or 
        hb_reqbus_op or iba or ibs or ibasid or ibm or ibc)
if(DEBUG_MODE&&!hb_reqbus_adel&&(hb_reqbus_op==`OP_LW))
begin
    if(reg_rw_select[0])
        ib_out=ibs;    
    else
       begin
           ib_out=32'b0;
           for(i=0;i<ibpnum;i=i+1)
            if(reg_rw_select[i+1])
              ib_out={iba[32*i+31],iba[32*i+30],iba[32*i+29],iba[32*i+28],
                     iba[32*i+27],iba[32*i+26],iba[32*i+25],iba[32*i+24],
                     iba[32*i+23],iba[32*i+22],iba[32*i+21],iba[32*i+20],
                     iba[32*i+19],iba[32*i+18],iba[32*i+17],iba[32*i+16],
                     iba[32*i+15],iba[32*i+14],iba[32*i+13],iba[32*i+12],
                     iba[32*i+11],iba[32*i+10],iba[32*i+9],iba[32*i+8],
                     iba[32*i+7],iba[32*i+6],iba[32*i+5],iba[32*i+4],
                     iba[32*i+3],iba[32*i+2],iba[32*i+1],iba[32*i]} ;
           else if(reg_rw_select[i+ibpnum+1])        
              ib_out= {ibm[32*i+31],ibm[32*i+30],ibm[32*i+29],ibm[32*i+28],
                     ibm[32*i+27],ibm[32*i+26],ibm[32*i+25],ibm[32*i+24],
                     ibm[32*i+23],ibm[32*i+22],ibm[32*i+21],ibm[32*i+20],
                     ibm[32*i+19],ibm[32*i+18],ibm[32*i+17],ibm[32*i+16],
                     ibm[32*i+15],ibm[32*i+14],ibm[32*i+13],ibm[32*i+12],
                     ibm[32*i+11],ibm[32*i+10],ibm[32*i+9],ibm[32*i+8],
                     ibm[32*i+7],ibm[32*i+6],ibm[32*i+5],ibm[32*i+4],
                     ibm[32*i+3],ibm[32*i+2],ibm[32*i+1],ibm[32*i]};   
           else if(reg_rw_select[i+2*ibpnum+1])
            ib_out= {ibasid[32*i+31],ibasid[32*i+30],ibasid[32*i+29],ibasid[32*i+28],
                     ibasid[32*i+27],ibasid[32*i+26],ibasid[32*i+25],ibasid[32*i+24],
                     ibasid[32*i+23],ibasid[32*i+22],ibasid[32*i+21],ibasid[32*i+20],
                     ibasid[32*i+19],ibasid[32*i+18],ibasid[32*i+17],ibasid[32*i+16], 
                     ibasid[32*i+15],ibasid[32*i+14],ibasid[32*i+13],ibasid[32*i+12],
                     ibasid[32*i+11],ibasid[32*i+10],ibasid[32*i+9],ibasid[32*i+8], 
                     ibasid[32*i+7],ibasid[32*i+6],ibasid[32*i+5],ibasid[32*i+4],
                     ibasid[32*i+3],ibasid[32*i+2],ibasid[32*i+1],ibasid[32*i]};
           else if(reg_rw_select[i+3*ibpnum+1])
            ib_out={ibc[32*i+31],ibc[32*i+30],ibc[32*i+29],ibc[32*i+28],
                    ibc[32*i+27],ibc[32*i+26],ibc[32*i+25],ibc[32*i+24],
                    ibc[32*i+23],ibc[32*i+22],ibc[32*i+21],ibc[32*i+20],
                    ibc[32*i+19],ibc[32*i+18],ibc[32*i+17],ibc[32*i+16],
                    ibc[32*i+15],ibc[32*i+14],ibc[32*i+13],ibc[32*i+12],
                    ibc[32*i+11],ibc[32*i+10],ibc[32*i+9],ibc[32*i+8],
                    ibc[32*i+7],ibc[32*i+6],ibc[32*i+5],ibc[32*i+4],
                    ibc[32*i+3],ibc[32*i+2],ibc[32*i+1],ibc[32*i]};
        end
end 
else
    ib_out=32'b0;      

assign RREG_FROM_IB=ib_out;                                                 

//compare
always@(asid or pc or hb_ivalid or ibasid or ibm or iba or ibc)
    for(l=0;l<ibpnum;l=l+1)
        begin
            ib_match_0[l]=(!ibc[32*l+23]|(asid=={ibasid[32*l+7],ibasid[32*l+6],ibasid[32*l+5],ibasid[32*l+4],
                                               ibasid[32*l+3],ibasid[32*l+2],ibasid[32*l+1],ibasid[32*l]}))&
                        (({ibm[32*l+31],ibm[32*l+30],ibm[32*l+29],ibm[32*l+28],
                           ibm[32*l+27],ibm[32*l+26],ibm[32*l+25],ibm[32*l+24],
                           ibm[32*l+23],ibm[32*l+22],ibm[32*l+21],ibm[32*l+20],
                           ibm[32*l+19],ibm[32*l+18],ibm[32*l+17],ibm[32*l+16],
                           ibm[32*l+15],ibm[32*l+14],ibm[32*l+13],ibm[32*l+12],
                           ibm[32*l+11],ibm[32*l+10],ibm[32*l+9],ibm[32*l+8],
                           ibm[32*l+7],ibm[32*l+6],ibm[32*l+5],ibm[32*l+4],
                           ibm[32*l+3],ibm[32*l+2],ibm[32*l+1],ibm[32*l]}|
                         ~(pc^{iba[32*l+31],iba[32*l+30],iba[32*l+29],iba[32*l+28],
                               iba[32*l+27],iba[32*l+26],iba[32*l+25],iba[32*l+24],
                               iba[32*l+23],iba[32*l+22],iba[32*l+21],iba[32*l+20],
                               iba[32*l+19],iba[32*l+18],iba[32*l+17],iba[32*l+16],
                               iba[32*l+15],iba[32*l+14],iba[32*l+13],iba[32*l+12],
                               iba[32*l+11],iba[32*l+10],iba[32*l+9],iba[32*l+8],
                               iba[32*l+7],iba[32*l+6],iba[32*l+5],iba[32*l+4],
                               iba[32*l+3],iba[32*l+2],iba[32*l+1],iba[32*l]}))==32'hffffffff)&hb_ivalid;
        end 

//compare pc+4
always@(asid or pc_next or hb_ivalid or ibasid or ibm or iba or ibc)
    for(l=0;l<ibpnum;l=l+1)
        begin
            ib_match_1[l]=(!ibc[32*l+23]|(asid=={ibasid[32*l+7],ibasid[32*l+6],ibasid[32*l+5],ibasid[32*l+4],
                                               ibasid[32*l+3],ibasid[32*l+2],ibasid[32*l+1],ibasid[32*l]}))&
                        (({ibm[32*l+31],ibm[32*l+30],ibm[32*l+29],ibm[32*l+28],
                           ibm[32*l+27],ibm[32*l+26],ibm[32*l+25],ibm[32*l+24],
                           ibm[32*l+23],ibm[32*l+22],ibm[32*l+21],ibm[32*l+20],
                           ibm[32*l+19],ibm[32*l+18],ibm[32*l+17],ibm[32*l+16],
                           ibm[32*l+15],ibm[32*l+14],ibm[32*l+13],ibm[32*l+12],
                           ibm[32*l+11],ibm[32*l+10],ibm[32*l+9],ibm[32*l+8],
                           ibm[32*l+7],ibm[32*l+6],ibm[32*l+5],ibm[32*l+4],
                           ibm[32*l+3],ibm[32*l+2],ibm[32*l+1],ibm[32*l]}|
                         ~(pc_next^{iba[32*l+31],iba[32*l+30],iba[32*l+29],iba[32*l+28],
                               iba[32*l+27],iba[32*l+26],iba[32*l+25],iba[32*l+24],
                               iba[32*l+23],iba[32*l+22],iba[32*l+21],iba[32*l+20],
                               iba[32*l+19],iba[32*l+18],iba[32*l+17],iba[32*l+16],
                               iba[32*l+15],iba[32*l+14],iba[32*l+13],iba[32*l+12],
                               iba[32*l+11],iba[32*l+10],iba[32*l+9],iba[32*l+8],
                               iba[32*l+7],iba[32*l+6],iba[32*l+5],iba[32*l+4],
                               iba[32*l+3],iba[32*l+2],iba[32*l+1],iba[32*l]}))==32'hffffffff)&hb_ivalid;
        end     

//compare commit_pc 
always@(asid or compare_commit_pc or ibasid or ibm or iba or ibc)
    for(l=0;l<ibpnum;l=l+1)
        begin
            ib_match_commit[l]=(!ibc[32*l+23]|(asid=={ibasid[32*l+7],ibasid[32*l+6],ibasid[32*l+5],ibasid[32*l+4],
                                               ibasid[32*l+3],ibasid[32*l+2],ibasid[32*l+1],ibasid[32*l]}))&
                        (({ibm[32*l+31],ibm[32*l+30],ibm[32*l+29],ibm[32*l+28],
                           ibm[32*l+27],ibm[32*l+26],ibm[32*l+25],ibm[32*l+24],
                           ibm[32*l+23],ibm[32*l+22],ibm[32*l+21],ibm[32*l+20],
                           ibm[32*l+19],ibm[32*l+18],ibm[32*l+17],ibm[32*l+16],
                           ibm[32*l+15],ibm[32*l+14],ibm[32*l+13],ibm[32*l+12],
                           ibm[32*l+11],ibm[32*l+10],ibm[32*l+9],ibm[32*l+8],
                           ibm[32*l+7],ibm[32*l+6],ibm[32*l+5],ibm[32*l+4],
                           ibm[32*l+3],ibm[32*l+2],ibm[32*l+1],ibm[32*l]}|
                         ~(compare_commit_pc^{iba[32*l+31],iba[32*l+30],iba[32*l+29],iba[32*l+28],
                               iba[32*l+27],iba[32*l+26],iba[32*l+25],iba[32*l+24],
                               iba[32*l+23],iba[32*l+22],iba[32*l+21],iba[32*l+20],
                               iba[32*l+19],iba[32*l+18],iba[32*l+17],iba[32*l+16],
                               iba[32*l+15],iba[32*l+14],iba[32*l+13],iba[32*l+12],
                               iba[32*l+11],iba[32*l+10],iba[32*l+9],iba[32*l+8],
                               iba[32*l+7],iba[32*l+6],iba[32*l+5],iba[32*l+4],
                               iba[32*l+3],iba[32*l+2],iba[32*l+1],iba[32*l]}))==32'hffffffff);
        end     

assign ib_match = {ibpnum{COMMITBUS_EX&(COMMITBUS_EXCODE==`EX_DIB)}}&ib_match_commit; 


always@(posedge CLOCK)
if(RESET)
    ihardbreak_skip<=2'b00;
else if(COMMITBUS_EX&&(COMMITBUS_EXCODE==`EX_DIB))//EX_DIB
    ihardbreak_skip<=2'b11;
else 
begin
if(ICACHERDY_AFTER_DERET[0])
    ihardbreak_skip[0]<=0;
if(ICACHERDY_AFTER_DERET[1])
    ihardbreak_skip[1]<=0;
end
//dib_0        

always@(ib_match_0 or ibc)
    for(m=0;m<ibpnum;m=m+1)
        begin
            ib_match_te_0[m]=ib_match_0[m]&ibc[32*m+2];
            ib_match_be_0[m]=ib_match_0[m]&ibc[32*m];
        end        

wire deret_resolved = brbus_err&brbus_valid &(brbus_op==`OP_DERET); 

wire ex_ejtagboot=(COMMITBUS_EXCODE[5:0]==`EX_EJTAGBOOT);
wire ex_ibe=(COMMITBUS_EXCODE[5:0]==`EX_IBE);
/*
reg deret_noresolved;
always @(posedge CLOCK)
begin
  if (RESET)
    deret_noresolved <= 1'b0;
  else if (!DEBUG_MODE&COMMITBUS_EX&((COMMITBUS_EXCODE[5:3]==3'b110)|| ex_ejtagboot|| ex_ibe)  ) 
    deret_noresolved <= 1'b1;
  else if (deret_resolved)
    deret_noresolved <= 1'b0;
end
*/
assign HB_DIT[0]=(|ib_match_te_0);
assign HB_DIB[0]=(|ib_match_be_0)&!ihardbreak_skip[0] ;
//assign HB_DIT[0]=(|ib_match_te_0)&!deret_noresolved;
//assign HB_DIB[0]=(|ib_match_be_0)&!deret_noresolved&(!ihardbreak_skip |ICACHERDY_AFTER_DERET[1]);
//dib_1

always@(ib_match_1 or ibc)
    for(m=0;m<ibpnum;m=m+1)
        begin
            ib_match_te_1[m]=ib_match_1[m]&ibc[32*m+2];
            ib_match_be_1[m]=ib_match_1[m]&ibc[32*m];
        end    
        
assign HB_DIT[1]=(|ib_match_te_1);
assign HB_DIB[1]=(|ib_match_be_1)&~ihardbreak_skip[1];

always@(posedge CLOCK)
    if(RESET)
        ibs<={4'b0_1_00,ibs_reset,24'b000000000_000000000000000};
    else if(DEBUG_MODE&&!hb_reqbus_ades&&(hb_reqbus_op==`OP_SW)&&reg_rw_select[0])
        ibs[ibpnum-1:0]<=ibs[ibpnum-1:0] & hb_reqbus_value[ibpnum-1:0];   
    else if (!DEBUG_MODE&COMMITBUS_EX&(COMMITBUS_EXCODE==`EX_DIB)&!(|ibs[ibpnum-1:0]))
         for(p=0;p<ibpnum;p=p+1)
           ibs[p]<=ib_match[p] | ibc[32*p+2];
    else if (!DEBUG_MODE&COMMITBUS_EX&!(COMMITBUS_EXCODE==`EX_DIB))
          ibs[ibpnum-1:0] <=0; 

                     
endmodule

//*******************************************************************
//copyrignt                         
//file name                                 db.v
//module name                               db
//author                                    cao fei
//version                                   0.1
//function description                      data breakpoint 
//modification comment/date/author          
//testbench file
//test result file
//********************************************************************
module db(CLOCK,
          RESET,
          DEBUG_MODE,
          HB_REQBUS,
          HB_DCOMPBUS,
          LOADDATA,
          LOADDATA_QID,
          LOADDATA_VALID,
          LOADDATA_H_VALID,
          LOADDATA_H,
          BYTELANE,
          brbus_to_db_i,
          COMMITBUS_QID,
          COMMITBUS_EX,
          COMMITBUS_EXCODE,
          //DSS_ENABLE,
          FIRST_MEM_PASS_AFTER_DERET,
          RREG_FROM_DB,
          HB_DDT,
          HB_DDBL,
          HB_DDBS,
          HB_DDBLIMPR,
          HB_DDBSIMPR,
          HB_LOAD_ADDR_MATCH,
          hb_dbs_bs_o);
parameter dbpnum=`DBKP_NUM;//instruction break point number
input CLOCK;
input RESET;
input DEBUG_MODE;
input[78:0] HB_REQBUS;
input[`Lhb_dcompbus-1:0] HB_DCOMPBUS;
input[3:0] COMMITBUS_QID;
input[5:0] brbus_to_db_i;
input COMMITBUS_EX;
input[5:0] COMMITBUS_EXCODE;
//input DSS_ENABLE;
input FIRST_MEM_PASS_AFTER_DERET;
output[31:0] RREG_FROM_DB;
output HB_DDT;
output HB_DDBL;
output HB_DDBS;
output HB_DDBLIMPR;
output HB_DDBSIMPR;

input[31:0] LOADDATA;
input[ 3:0] LOADDATA_QID;
input LOADDATA_VALID;  
input LOADDATA_H_VALID;
input[31:0] LOADDATA_H;
input [3:0] BYTELANE;
output [1:0] HB_LOAD_ADDR_MATCH;
output [`DBKP_NUM-1:0] hb_dbs_bs_o;
wire [5:0] brbus_brmask   = brbus_to_db_i;

//db_reg
reg[31:0] dbs;
reg[32*dbpnum-1:0] dba;
reg[32*dbpnum-1:0] dbm;
reg[32*dbpnum-1:0] dbasid;
reg[32*dbpnum-1:0] dbc;
reg[32*dbpnum-1:0] dbv;
reg[dbpnum-1:0] dbs_addr_match_h; //high 32bit address of ldc1 match
integer i,j,k,l,m,n,p,q;

wire hb_reqbus_ades;
wire hb_reqbus_adel;
wire[31:0] hb_reqbus_value;
wire[7:0] hb_reqbus_op;//store 8'h9a   load 8'h92
wire[31:0] hb_reqbus_addr;
wire[3:0] hb_reqbus_qid;//no use
wire hb_reqbus_valid;

wire[31:0] storedata;
wire ades,adel;
wire[7:0] asid;
wire hb_dvalid;
wire[3:0] bytelane;
wire hb_type;//0:load   1:store
wire[31:0] addr;
wire[31:0] addr_h = {addr[31:3], 1'b1, addr[1:0]};
wire[3:0] hb_dcompbus_qid;
wire[2:0] hb_dcompbus_brqid;
wire      hb_dcompbus_load;
wire[3:0] dbs_reset;
wire      hb_is_dw;
wire[31:0]storedata_h;

reg[31:0] db_out;
reg[dbpnum*5:0]  reg_rw_select;
reg dhardbreak_skip;
reg[dbpnum-1:0] db_match_no_ddblimpr;
wire[dbpnum-1:0] db_match;
reg[dbpnum-1:0] db_addr_match;
reg[dbpnum-1:0] db_addr_match_h;
reg[dbpnum-1:0] db_load_value_match;//split load and store data
reg[dbpnum-1:0] db_load_value_match_h;//split load and store data
reg[dbpnum-1:0] db_store_value_match;
reg[dbpnum-1:0] db_store_value_match_h;
reg[dbpnum-1:0] db_no_value_compare;
reg[dbpnum-1:0] db_match_be_s;
reg[dbpnum-1:0] db_match_be_l;
reg[dbpnum-1:0] db_match_be_simpr;
reg[dbpnum-1:0] db_match_be_limpr;
reg[dbpnum-1:0] db_match_te;
reg[2:0] brqid;
assign hb_reqbus_ades=HB_REQBUS[78];
assign hb_reqbus_adel=HB_REQBUS[77];
assign hb_reqbus_value=HB_REQBUS[76:45];
assign hb_reqbus_op=HB_REQBUS[44:37];
assign hb_reqbus_addr=HB_REQBUS[36:5];
assign hb_reqbus_qid=HB_REQBUS[4:1];
assign hb_reqbus_valid=HB_REQBUS[0];

assign storedata_h = HB_DCOMPBUS[120:89];
assign hb_is_dw = HB_DCOMPBUS[88];
assign hb_dcompbus_brqid = HB_DCOMPBUS[87:85];
assign hb_dcompbus_load = HB_DCOMPBUS[84];
assign hb_dcompbus_qid=HB_DCOMPBUS[83:80];
assign ades=HB_DCOMPBUS[79];
assign adel=HB_DCOMPBUS[78];
//assign LOADDATA_VALID=HB_DCOMPBUS[110];
//assign LOADDATA=HB_DCOMPBUS[109:78];
assign storedata=HB_DCOMPBUS[77:46];
assign bytelane=HB_DCOMPBUS[45:42];
assign hb_type=HB_DCOMPBUS[41];
assign addr=HB_DCOMPBUS[40:9];
assign asid=HB_DCOMPBUS[8:1];
assign hb_dvalid=HB_DCOMPBUS[0];

// reg addr deCODE
always@(hb_reqbus_addr or hb_reqbus_valid)
    if(hb_reqbus_valid&&!hb_reqbus_addr[12])
        begin
            if({hb_reqbus_addr[11:8],hb_reqbus_addr[5:3]}==0)
                reg_rw_select[0]=1'b1;
            else 
                reg_rw_select[0]=1'b0;    
            for(k=1;k<(dbpnum+1);k=k+1)
                for(j=0;j<5;j=j+1)
                    if((k==hb_reqbus_addr[11:8])&&(j==hb_reqbus_addr[5:3]))
                        reg_rw_select[k+j*dbpnum]=1'b1;
                    else 
                        reg_rw_select[k+j*dbpnum]=1'b0;    
        end                    
    else
        reg_rw_select=0;

wire hbd_gating_clock;
assign hbd_gating_clock = CLOCK;

always@(posedge hbd_gating_clock)
    if(RESET)
        begin   
            dba<=0;
            dbm<=0;
            dbasid<=0;
            dbc<=0;
            dbv<=0;
        end
    else if(DEBUG_MODE&&!hb_reqbus_ades&&(hb_reqbus_op==`OP_SW))//STORE 8'h9a
        begin
            for(l=0;l<dbpnum;l=l+1)
              begin
                if(reg_rw_select[l+1])
                    {dba[32*l+31],dba[32*l+30],dba[32*l+29],dba[32*l+28],
                     dba[32*l+27],dba[32*l+26],dba[32*l+25],dba[32*l+24],
                     dba[32*l+23],dba[32*l+22],dba[32*l+21],dba[32*l+20],
                     dba[32*l+19],dba[32*l+18],dba[32*l+17],dba[32*l+16],
                     dba[32*l+15],dba[32*l+14],dba[32*l+13],dba[32*l+12],
                     dba[32*l+11],dba[32*l+10],dba[32*l+9],dba[32*l+8],
                     dba[32*l+7],dba[32*l+6],dba[32*l+5],dba[32*l+4],
                     dba[32*l+3],dba[32*l+2],dba[32*l+1],dba[32*l]} <= hb_reqbus_value;
           
                if(reg_rw_select[l+dbpnum+1])
                     {dbm[32*l+31],dbm[32*l+30],dbm[32*l+29],dbm[32*l+28],
                     dbm[32*l+27],dbm[32*l+26],dbm[32*l+25],dbm[32*l+24],
                     dbm[32*l+23],dbm[32*l+22],dbm[32*l+21],dbm[32*l+20],
                     dbm[32*l+19],dbm[32*l+18],dbm[32*l+17],dbm[32*l+16],
                     dbm[32*l+15],dbm[32*l+14],dbm[32*l+13],dbm[32*l+12],
                     dbm[32*l+11],dbm[32*l+10],dbm[32*l+9],dbm[32*l+8],
                     dbm[32*l+7],dbm[32*l+6],dbm[32*l+5],dbm[32*l+4],
                     dbm[32*l+3],dbm[32*l+2],dbm[32*l+1],dbm[32*l]} <= hb_reqbus_value;
           
                if(reg_rw_select[l+2*dbpnum+1])  
                     {dbasid[32*l+7],dbasid[32*l+6],dbasid[32*l+5],dbasid[32*l+4],
                      dbasid[32*l+3],dbasid[32*l+2],dbasid[32*l+1],dbasid[32*l]} <= hb_reqbus_value[7:0];
           
                if(reg_rw_select[l+3*dbpnum+1])  
                    {dbc[32*l+23],dbc[32*l+17],dbc[32*l+16],dbc[32*l+15],
                     dbc[32*l+14],dbc[32*l+13],dbc[32*l+12],dbc[32*l+7],
                     dbc[32*l+6],dbc[32*l+5],dbc[32*l+4],
                     dbc[32*l+2],dbc[32*l]}<={hb_reqbus_value[23],hb_reqbus_value[17:12],
                                              hb_reqbus_value[7:4],hb_reqbus_value[2],hb_reqbus_value[0]};  
           
                if(reg_rw_select[l+4*dbpnum+1])
                     {dbv[32*l+31],dbv[32*l+30],dbv[32*l+29],dbv[32*l+28],
                     dbv[32*l+27],dbv[32*l+26],dbv[32*l+25],dbv[32*l+24],
                     dbv[32*l+23],dbv[32*l+22],dbv[32*l+21],dbv[32*l+20],
                     dbv[32*l+19],dbv[32*l+18],dbv[32*l+17],dbv[32*l+16],
                     dbv[32*l+15],dbv[32*l+14],dbv[32*l+13],dbv[32*l+12],
                     dbv[32*l+11],dbv[32*l+10],dbv[32*l+9],dbv[32*l+8],
                     dbv[32*l+7],dbv[32*l+6],dbv[32*l+5],dbv[32*l+4],
                     dbv[32*l+3],dbv[32*l+2],dbv[32*l+1],dbv[32*l]} <=hb_reqbus_value;                   
        end
 end       
//r
always@(reg_rw_select or DEBUG_MODE or hb_reqbus_adel or hb_reqbus_op 
              or dbs or dba or dbm or dbasid or dbc or dbv)
if(DEBUG_MODE&&!hb_reqbus_adel&&(hb_reqbus_op==`OP_LW))
begin
   if(reg_rw_select[0])
        db_out=dbs; 
   else begin
    db_out=32'b0;
    for(i=0;i<dbpnum;i=i+1) begin
        if(reg_rw_select[i+1])
            db_out= {dba[32*i+31],dba[32*i+30],dba[32*i+29],dba[32*i+28],
                     dba[32*i+27],dba[32*i+26],dba[32*i+25],dba[32*i+24],
                     dba[32*i+23],dba[32*i+22],dba[32*i+21],dba[32*i+20],
                     dba[32*i+19],dba[32*i+18],dba[32*i+17],dba[32*i+16],
                     dba[32*i+15],dba[32*i+14],dba[32*i+13],dba[32*i+12],
                     dba[32*i+11],dba[32*i+10],dba[32*i+9],dba[32*i+8],
                     dba[32*i+7],dba[32*i+6],dba[32*i+5],dba[32*i+4],
                     dba[32*i+3],dba[32*i+2],dba[32*i+1],dba[32*i]};
       else  if(reg_rw_select[i+dbpnum+1])        
            db_out= {dbm[32*i+31],dbm[32*i+30],dbm[32*i+29],dbm[32*i+28],
                     dbm[32*i+27],dbm[32*i+26],dbm[32*i+25],dbm[32*i+24],
                     dbm[32*i+23],dbm[32*i+22],dbm[32*i+21],dbm[32*i+20],
                     dbm[32*i+19],dbm[32*i+18],dbm[32*i+17],dbm[32*i+16],
                     dbm[32*i+15],dbm[32*i+14],dbm[32*i+13],dbm[32*i+12],
                     dbm[32*i+11],dbm[32*i+10],dbm[32*i+9],dbm[32*i+8],
                     dbm[32*i+7],dbm[32*i+6],dbm[32*i+5],dbm[32*i+4],
                     dbm[32*i+3],dbm[32*i+2],dbm[32*i+1],dbm[32*i]};   
       else if(reg_rw_select[i+2*dbpnum+1])
            db_out= {dbasid[32*i+31],dbasid[32*i+30],dbasid[32*i+29],dbasid[32*i+28],
                     dbasid[32*i+27],dbasid[32*i+26],dbasid[32*i+25],dbasid[32*i+24],
                     dbasid[32*i+23],dbasid[32*i+22],dbasid[32*i+21],dbasid[32*i+20],
                     dbasid[32*i+19],dbasid[32*i+18],dbasid[32*i+17],dbasid[32*i+16],
                     dbasid[32*i+15],dbasid[32*i+14],dbasid[32*i+13],dbasid[32*i+12],
                     dbasid[32*i+11],dbasid[32*i+10],dbasid[32*i+9],dbasid[32*i+8],
                     dbasid[32*i+7],dbasid[32*i+6],dbasid[32*i+5],dbasid[32*i+4],
                     dbasid[32*i+3],dbasid[32*i+2],dbasid[32*i+1],dbasid[32*i]};
       else if(reg_rw_select[i+3*dbpnum+1])
            db_out= {dbc[32*i+31],dbc[32*i+30],dbc[32*i+29],dbc[32*i+28],
                     dbc[32*i+27],dbc[32*i+26],dbc[32*i+25],dbc[32*i+24],
                     dbc[32*i+23],dbc[32*i+22],dbc[32*i+21],dbc[32*i+20],
                     dbc[32*i+19],dbc[32*i+18],dbc[32*i+17],dbc[32*i+16],
                     dbc[32*i+15],dbc[32*i+14],dbc[32*i+13],dbc[32*i+12],
                     dbc[32*i+11],dbc[32*i+10],dbc[32*i+9],dbc[32*i+8],
                     dbc[32*i+7],dbc[32*i+6],dbc[32*i+5],dbc[32*i+4],
                     dbc[32*i+3],dbc[32*i+2],dbc[32*i+1],dbc[32*i]};
       else if(reg_rw_select[i+4*dbpnum+1])
            db_out= {dbv[32*i+31],dbv[32*i+30],dbv[32*i+29],dbv[32*i+28],
                     dbv[32*i+27],dbv[32*i+26],dbv[32*i+25],dbv[32*i+24],
                     dbv[32*i+23],dbv[32*i+22],dbv[32*i+21],dbv[32*i+20],
                     dbv[32*i+19],dbv[32*i+18],dbv[32*i+17],dbv[32*i+16],
                     dbv[32*i+15],dbv[32*i+14],dbv[32*i+13],dbv[32*i+12],
                     dbv[32*i+11],dbv[32*i+10],dbv[32*i+9],dbv[32*i+8],
                     dbv[32*i+7],dbv[32*i+6],dbv[32*i+5],dbv[32*i+4],
                     dbv[32*i+3],dbv[32*i+2],dbv[32*i+1],dbv[32*i]};            
   end
   end
   end
else
    db_out=32'b0;     

assign RREG_FROM_DB=db_out;                                                 

//compare        
always@(asid or hb_dvalid or bytelane or addr or hb_type or  
         dbasid or dbc or dbm or dba or dbv or  
        storedata or ades or adel or hb_dcompbus_load or
        BYTELANE or LOADDATA or LOADDATA_VALID or
        addr_h or storedata_h or LOADDATA_H or LOADDATA_H_VALID or hb_is_dw)
    for(m=0;m<dbpnum;m=m+1)
        begin
            db_addr_match[m]=(!dbc[32*m+23]|(asid=={dbasid[32*m+7],dbasid[32*m+6],dbasid[32*m+5],dbasid[32*m+4],
                                                    dbasid[32*m+3],dbasid[32*m+2],dbasid[32*m+1],dbasid[32*m]}))&
                             (({dbm[32*m+31],dbm[32*m+30],dbm[32*m+29],dbm[32*m+28],
                                dbm[32*m+27],dbm[32*m+26],dbm[32*m+25],dbm[32*m+24],
                                dbm[32*m+23],dbm[32*m+22],dbm[32*m+21],dbm[32*m+20],
                                dbm[32*m+19],dbm[32*m+18],dbm[32*m+17],dbm[32*m+16],
                                dbm[32*m+15],dbm[32*m+14],dbm[32*m+13],dbm[32*m+12],
                                dbm[32*m+11],dbm[32*m+10],dbm[32*m+9],dbm[32*m+8],
                                dbm[32*m+7],dbm[32*m+6],dbm[32*m+5],dbm[32*m+4],
                                dbm[32*m+3],dbm[32*m+2],dbm[32*m+1],dbm[32*m]}|
                              (~(addr^{dba[32*m+31],dba[32*m+30],dba[32*m+29],dba[32*m+28],
                                       dba[32*m+27],dba[32*m+26],dba[32*m+25],dba[32*m+24],
                                       dba[32*m+23],dba[32*m+22],dba[32*m+21],dba[32*m+20],
                                       dba[32*m+19],dba[32*m+18],dba[32*m+17],dba[32*m+16],
                                       dba[32*m+15],dba[32*m+14],dba[32*m+13],dba[32*m+12],
                                       dba[32*m+11],dba[32*m+10],dba[32*m+9],dba[32*m+8],
                                       dba[32*m+7],dba[32*m+6],dba[32*m+5],dba[32*m+4],
                                       dba[32*m+3],dba[32*m+2],dba[32*m+1],dba[32*m]})))==32'hffffffff)&
                              ((~{dbc[32*m+17],dbc[32*m+16],dbc[32*m+15],dbc[32*m+14]}&bytelane)!=4'b0)
                              &hb_dvalid;

            db_addr_match_h[m]=(!dbc[32*m+23]|(asid=={dbasid[32*m+7],dbasid[32*m+6],dbasid[32*m+5],dbasid[32*m+4],
                                                    dbasid[32*m+3],dbasid[32*m+2],dbasid[32*m+1],dbasid[32*m]}))&
                             (({dbm[32*m+31],dbm[32*m+30],dbm[32*m+29],dbm[32*m+28],
                                dbm[32*m+27],dbm[32*m+26],dbm[32*m+25],dbm[32*m+24],
                                dbm[32*m+23],dbm[32*m+22],dbm[32*m+21],dbm[32*m+20],
                                dbm[32*m+19],dbm[32*m+18],dbm[32*m+17],dbm[32*m+16],
                                dbm[32*m+15],dbm[32*m+14],dbm[32*m+13],dbm[32*m+12],
                                dbm[32*m+11],dbm[32*m+10],dbm[32*m+9],dbm[32*m+8],
                                dbm[32*m+7],dbm[32*m+6],dbm[32*m+5],dbm[32*m+4],
                                dbm[32*m+3],dbm[32*m+2],dbm[32*m+1],dbm[32*m]}|
                              (~(addr_h^{dba[32*m+31],dba[32*m+30],dba[32*m+29],dba[32*m+28],
                                         dba[32*m+27],dba[32*m+26],dba[32*m+25],dba[32*m+24],
                                         dba[32*m+23],dba[32*m+22],dba[32*m+21],dba[32*m+20],
                                         dba[32*m+19],dba[32*m+18],dba[32*m+17],dba[32*m+16],
                                         dba[32*m+15],dba[32*m+14],dba[32*m+13],dba[32*m+12],
                                         dba[32*m+11],dba[32*m+10],dba[32*m+9],dba[32*m+8],
                                         dba[32*m+7],dba[32*m+6],dba[32*m+5],dba[32*m+4],
                                         dba[32*m+3],dba[32*m+2],dba[32*m+1],dba[32*m]})))==32'hffffffff)&
                              (({dbc[32*m+17],dbc[32*m+16],dbc[32*m+15],dbc[32*m+14]})==4'h0)&hb_is_dw
                              &hb_dvalid;

            db_no_value_compare[m]=(({dbc[32*m+7],dbc[32*m+6],dbc[32*m+5],dbc[32*m+4]}|
                                     {dbc[32*m+17],dbc[32*m+16],dbc[32*m+15],dbc[32*m+14]}|
                                      ~bytelane)==4'b1111);
            db_load_value_match[m]=((LOADDATA[7:0]=={dbv[32*m+7],dbv[32*m+6],dbv[32*m+5],dbv[32*m+4],
                                            dbv[32*m+3],dbv[32*m+2],dbv[32*m+1],dbv[32*m]})|
                              !BYTELANE[0]|dbc[32*m+14]|dbc[32*m+4])&
                               ((LOADDATA[15:8]=={dbv[32*m+15],dbv[32*m+14],dbv[32*m+13],dbv[32*m+12],
                                              dbv[32*m+11],dbv[32*m+10],dbv[32*m+9],dbv[32*m+8]})|
                              !BYTELANE[1]|dbc[32*m+15]|dbc[32*m+5])&
                               ((LOADDATA[23:16]=={dbv[32*m+23],dbv[32*m+22],dbv[32*m+21],dbv[32*m+20],
                                               dbv[32*m+19],dbv[32*m+18],dbv[32*m+17],dbv[32*m+16]})|
                              !BYTELANE[2]|dbc[32*m+16]|dbc[32*m+6])&
                               ((LOADDATA[31:24]=={dbv[32*m+31],dbv[32*m+30],dbv[32*m+29],dbv[32*m+28],
                                               dbv[32*m+27],dbv[32*m+26],dbv[32*m+25],dbv[32*m+24]})|
                              !BYTELANE[3]|dbc[32*m+17]|dbc[32*m+7])&
                               (LOADDATA_VALID);                                        
        
            db_load_value_match_h[m]=((LOADDATA_H[7:0]=={dbv[32*m+7],dbv[32*m+6],dbv[32*m+5],dbv[32*m+4],
                                            dbv[32*m+3],dbv[32*m+2],dbv[32*m+1],dbv[32*m]})|
                              dbc[32*m+14]|dbc[32*m+4])&
                               ((LOADDATA_H[15:8]=={dbv[32*m+15],dbv[32*m+14],dbv[32*m+13],dbv[32*m+12],
                                              dbv[32*m+11],dbv[32*m+10],dbv[32*m+9],dbv[32*m+8]})|
                              dbc[32*m+15]|dbc[32*m+5])&
                               ((LOADDATA_H[23:16]=={dbv[32*m+23],dbv[32*m+22],dbv[32*m+21],dbv[32*m+20],
                                               dbv[32*m+19],dbv[32*m+18],dbv[32*m+17],dbv[32*m+16]})|
                              dbc[32*m+16]|dbc[32*m+6])&
                               ((LOADDATA_H[31:24]=={dbv[32*m+31],dbv[32*m+30],dbv[32*m+29],dbv[32*m+28],
                                               dbv[32*m+27],dbv[32*m+26],dbv[32*m+25],dbv[32*m+24]})|
                              dbc[32*m+17]|dbc[32*m+7])&
                               (LOADDATA_H_VALID);                                        
        
            db_store_value_match[m]=((storedata[7:0]=={dbv[32*m+7],dbv[32*m+6],dbv[32*m+5],dbv[32*m+4],
                                            dbv[32*m+3],dbv[32*m+2],dbv[32*m+1],dbv[32*m]})|
                              !bytelane[0]|dbc[32*m+14]|dbc[32*m+4])&
                               ((storedata[15:8]=={dbv[32*m+15],dbv[32*m+14],dbv[32*m+13],dbv[32*m+12],
                                              dbv[32*m+11],dbv[32*m+10],dbv[32*m+9],dbv[32*m+8]})|
                              !bytelane[1]|dbc[32*m+15]|dbc[32*m+5])&
                               ((storedata[23:16]=={dbv[32*m+23],dbv[32*m+22],dbv[32*m+21],dbv[32*m+20],
                                               dbv[32*m+19],dbv[32*m+18],dbv[32*m+17],dbv[32*m+16]})|
                              !bytelane[2]|dbc[32*m+16]|dbc[32*m+6])&
                               ((storedata[31:24]=={dbv[32*m+31],dbv[32*m+30],dbv[32*m+29],dbv[32*m+28],
                                               dbv[32*m+27],dbv[32*m+26],dbv[32*m+25],dbv[32*m+24]})|
                              !bytelane[3]|dbc[32*m+17]|dbc[32*m+7])&
                              !hb_dcompbus_load & 
                               (hb_type&hb_dvalid&!ades);                                                  

            db_store_value_match_h[m]=((storedata_h[7:0]=={dbv[32*m+7],dbv[32*m+6],dbv[32*m+5],dbv[32*m+4],
                                            dbv[32*m+3],dbv[32*m+2],dbv[32*m+1],dbv[32*m]})|
                              dbc[32*m+14]|dbc[32*m+4])&
                               ((storedata_h[15:8]=={dbv[32*m+15],dbv[32*m+14],dbv[32*m+13],dbv[32*m+12],
                                              dbv[32*m+11],dbv[32*m+10],dbv[32*m+9],dbv[32*m+8]})|
                              dbc[32*m+15]|dbc[32*m+5])&
                               ((storedata_h[23:16]=={dbv[32*m+23],dbv[32*m+22],dbv[32*m+21],dbv[32*m+20],
                                               dbv[32*m+19],dbv[32*m+18],dbv[32*m+17],dbv[32*m+16]})|
                              dbc[32*m+16]|dbc[32*m+6])&
                               ((storedata_h[31:24]=={dbv[32*m+31],dbv[32*m+30],dbv[32*m+29],dbv[32*m+28],
                                               dbv[32*m+27],dbv[32*m+26],dbv[32*m+25],dbv[32*m+24]})|
                              dbc[32*m+17]|dbc[32*m+7])& hb_is_dw &
                              !hb_dcompbus_load & 
                               (hb_type&hb_dvalid&!ades);                                                  
end          

always@(hb_type or db_addr_match or db_no_value_compare or 
        db_store_value_match or dbc or
        db_addr_match_h or db_store_value_match_h)
    for(n=0;n<dbpnum;n=n+1)
        begin
            db_match_no_ddblimpr[n]=((!hb_type&!dbc[32*n+12])|
                                     ( hb_type&!dbc[32*n+13])) &
                                    ((db_addr_match[n]   & (db_no_value_compare[n]|db_store_value_match[n]  )) | 
                                     (db_addr_match_h[n] & (db_no_value_compare[n]|db_store_value_match_h[n])));
        end             

assign db_match = db_match_no_ddblimpr | db_load_value_match | db_load_value_match_h;

always@(posedge CLOCK)
if(RESET)
    dhardbreak_skip<=0;
else if(COMMITBUS_EX&&((COMMITBUS_EXCODE==`EX_DDBL)||(COMMITBUS_EXCODE==`EX_DDBS)
                      ||(COMMITBUS_EXCODE==`EX_DDBLIMPR)||(COMMITBUS_EXCODE==`EX_DDBSIMPR)))
    dhardbreak_skip<=1;
else if(FIRST_MEM_PASS_AFTER_DERET)
    dhardbreak_skip<=0;
        
always@(db_match or db_load_value_match or db_load_value_match_h or db_store_value_match or 
          db_no_value_compare or db_addr_match or hb_type or dbc or
          dbs or db_addr_match_h or db_load_value_match_h or db_store_value_match_h or 
          dbs_addr_match_h or LOADDATA_VALID or LOADDATA_H_VALID)
    for(p=0;p<dbpnum;p=p+1)
    begin
        db_match_te[p]=db_match[p]&dbc[32*p+2];
        db_match_be_simpr[p]= ((db_store_value_match[p]&db_addr_match[p] | db_store_value_match_h[p]&db_addr_match_h[p])
                               &!db_no_value_compare[p]
                               & ( hb_type & !dbc[32*p+13]) & dbc[32*p]);
        db_match_be_s[p]    = (db_no_value_compare[p]& (db_addr_match[p]|db_addr_match_h[p])
                               & ( hb_type & !dbc[32*p+13]) & dbc[32*p]);
        db_match_be_l[p]    = (db_no_value_compare[p]& (db_addr_match[p]|db_addr_match_h[p])
                               & (!hb_type & !dbc[32*p+12]) & dbc[32*p]);                                                             
        db_match_be_limpr[p]= (db_load_value_match[p]   & dbs[p] & ~dbs_addr_match_h[p] & LOADDATA_VALID  | 
                               db_load_value_match_h[p] & dbs[p] &  dbs_addr_match_h[p] & LOADDATA_H_VALID) & 
                              dbc[32*p];
    end        


assign HB_DDT=(|db_match_te)&!DEBUG_MODE;
assign HB_DDBSIMPR=(|db_match_be_simpr)&!DEBUG_MODE&!dhardbreak_skip;
assign HB_DDBLIMPR=(|(db_match_be_limpr)) & !DEBUG_MODE & !dhardbreak_skip;
assign HB_DDBS=(|db_match_be_s)&!DEBUG_MODE&!dhardbreak_skip;
assign HB_DDBL=(|db_match_be_l)&!DEBUG_MODE&!dhardbreak_skip;

wire [dbpnum-1:0] load_addr_match_tmp = db_addr_match & ~db_no_value_compare & ~({dbc[32+12], dbc[12]}) & ~({2{hb_type}}); 
wire [dbpnum-1:0] load_addr_match = load_addr_match_tmp & {2{hb_dcompbus_load}};
assign HB_LOAD_ADDR_MATCH[0] = |load_addr_match_tmp & ~DEBUG_MODE & ~dhardbreak_skip ;

wire [dbpnum-1:0] load_addr_match_tmp_h = db_addr_match_h & ~db_no_value_compare & ~({dbc[32+12], dbc[12]}) & ~({2{hb_type}}); 
wire [dbpnum-1:0] load_addr_match_h = load_addr_match_tmp_h & {2{hb_dcompbus_load}};
assign HB_LOAD_ADDR_MATCH[1] = |load_addr_match_tmp_h & ~DEBUG_MODE & ~dhardbreak_skip ;

wire   brbus_cancel = brbus_brmask[hb_dcompbus_brqid];

//w dbs_bs
assign  dbs_reset=dbpnum;
always@(posedge CLOCK)
    if(RESET)
    begin    
        dbs<={4'b0_1_00,dbs_reset,24'b000000000_000000000000000};
        brqid<=3'b0;
        dbs_addr_match_h[dbpnum-1:0]<={dbpnum{1'b0}};
    end
    else if((!DEBUG_MODE && COMMITBUS_EX &&
             (COMMITBUS_EXCODE!=`EX_DDBL)    &&(COMMITBUS_EXCODE!=`EX_DDBS) &&
             (COMMITBUS_EXCODE!=`EX_DDBLIMPR)&&(COMMITBUS_EXCODE!=`EX_DDBSIMPR)) ||
            (|dbs[dbpnum-1:0] && brbus_brmask[brqid] && !DEBUG_MODE))
    begin
        dbs[dbpnum-1:0]<={dbpnum{1'b0}};
        dbs_addr_match_h[dbpnum-1:0]<={dbpnum{1'b0}};
    end
    else if (DEBUG_MODE&&!hb_reqbus_ades&&(hb_reqbus_op==`OP_SW)&&reg_rw_select[0])
    begin
        dbs[dbpnum-1:0]<=dbs[dbpnum-1:0] & hb_reqbus_value[dbpnum-1:0];   
        dbs_addr_match_h[dbpnum-1:0]<=dbs_addr_match_h[dbpnum-1:0] & hb_reqbus_value[dbpnum-1:0];
    end
    else if (!DEBUG_MODE & !COMMITBUS_EX)
    begin
        for (q=0;q<dbpnum;q=q+1)
            if ((LOADDATA_VALID   & dbs[q] & !dbs_addr_match_h[q] & ~db_load_value_match[q]  ) |
                (LOADDATA_H_VALID & dbs[q] &  dbs_addr_match_h[q] & ~db_load_value_match_h[q]))
            begin
                dbs[q] <= 1'b0;
                if (LOADDATA_H_VALID & dbs[q] &  dbs_addr_match_h[q] & ~db_load_value_match_h[q])
                    dbs_addr_match_h[q] <= 1'b0;
            end
            else if ((db_match_no_ddblimpr[q]|load_addr_match[q]|load_addr_match_h[q]) & ((dbc[32*q]&!dhardbreak_skip)|dbc[32*q+2]) & ~brbus_cancel & (~|dbs[dbpnum-1:0]))
            begin 
                dbs[q]<= 1'b1;// add for DSS_ENABLE control
                brqid <= hb_dcompbus_brqid;
                if (load_addr_match_h[q])
                    dbs_addr_match_h[q] <= 1'b1;
            end
    end
                               
assign hb_dbs_bs_o = dbs[dbpnum-1:0];

endmodule



