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

`include "bus.h" 
`include "reg.h" 
`include "global.h"

module godson_queue_module(
    clock,reset,
    decbus,
    resbus0,
    resbus1,
    alures_jalr_target1,
    insts_to_alu1_o,
    resbus2,
    resbus3,
    
    alures_jalr_target2,
    alu_res,
    qissuebus1,
    
    mmfull,alu2full,falufull,
    DSS_ENABLE,EJTAGBRK,DEBUG_MODE,INTE,
    fcr31,softreset,nmi,interrupt,
    qissuebus0,
    qissuebus0_src1_rdy,
    
    commitbus0,
    
    commitbus1,
    
    cr_cfg6_rti_i,
    qfull,
    qstalli,
    store_ok,
    cp0_mod_ok,
    EJTAGBOOT,
    brq_full,
    brq_tail_next_valid,
    src3_to_alu, //for INS's third src
    store_qid, 
    cp0_qid,
    //for performance counter
    cp0_forward_bus_i, 
    cp0_cancel_bus_i,

    brbus,
    offset_to_mm,
    brbus_deret_err,
    acc_write_ok,
    acc_qid,
    
    mm_ok,
    falu_ok,
    int_trigger_o,
    flush_pipeline_cycle_o,
    inst_to_queue_cycle_o,
    insts_to_alu2_o,
    insts_to_addr_o,
    insts_to_falu_o,
    insts_to_queue_o,
    issue_to_mmrs_qj
    //to fpq
);                                                   
                                 
input                       clock;
input                       reset;
input [`Lresbus0-1:0]       resbus0;
input [`Lresbus1-1:0]       resbus1;
input [31:0]                alures_jalr_target1;
output                      insts_to_alu1_o;
input [`Lresbus2-1:0]       resbus2;
input [`Lresbus3-1:0]       resbus3;
input [7:0]                 store_qid;
input [3:0]                 cp0_qid;
input                       mmfull,alu2full,falufull;
input [31:0]                fcr31;
input                       softreset,nmi,interrupt;
input                       cr_cfg6_rti_i;
input                       DSS_ENABLE,EJTAGBRK,DEBUG_MODE,INTE;
input [8:0]                 cp0_forward_bus_i;
input [4:0]                 cp0_cancel_bus_i;// not used

input [31:0]                alu_res;
input [31:0]                alures_jalr_target2;
    
input [3:0]                 acc_qid;
input [`Ldecbus_2issue-1:0] decbus;

output [4:0]                issue_to_mmrs_qj;
output [`Lqissuebus-1:0]    qissuebus0;
output [`Lqissuebus-1:0]    qissuebus1;
output                      qissuebus0_src1_rdy;
output [`Lcommitbus-1:0]        commitbus0;

output [`Lcommitbus-1:0]        commitbus1;

output [1:0]                    qfull;
output                          brq_full;
output                          brq_tail_next_valid;
output                          qstalli;
output [1:0]                    store_ok;
output                          cp0_mod_ok;
output                          EJTAGBOOT;
output[10:0]                    src3_to_alu;
output[97:0]                    brbus;
output[15:0]                    offset_to_mm;
output                          acc_write_ok;
output                          mm_ok;
output                          falu_ok;
output                          brbus_deret_err;
output                          int_trigger_o;

output                          flush_pipeline_cycle_o;
output                          inst_to_queue_cycle_o;
output                          insts_to_alu2_o;
output                          insts_to_addr_o;
output                          insts_to_falu_o;
output[1:0]                     insts_to_queue_o;


//    decbus
wire [`Ldecbus-1:0] decbus0,decbus1;
assign decbus0 = decbus[`Ldecbus-1:0];
assign decbus1 = decbus[`Ldecbus_2issue-1:`Ldecbus];

wire       decbus0_valid;
wire [7:0] decbus0_op;

wire [4:0] decbus0_fmt;
wire [4:0] decbus1_fmt;

wire [7:0] decbus0_src1;
wire [7:0] decbus0_src2;
wire [7:0] decbus0_dest;
wire [`Lword-1:0] decbus0_imm;
wire [`Lword-1:0] decbus0_pc;
wire [1:0] decbus0_ce;
wire       decbus0_bd;
wire       decbus0_adei;
wire       decbus0_tlbii;
wire       decbus0_tlbir;
wire       decbus0_ibe;
wire       decbus0_ri;
wire       decbus0_cpu;
wire       decbus0_sys;
wire       decbus0_bp;
wire       decbus0_sdbbp;
wire       decbus0_dib;
wire[7:0]  decbus0_gshare;
wire[1:0]  decbus0_rashead;
wire[1:0]  decbus0_other_link;
wire[1:0]  decbus0_ac;
wire       decbus0_nop;
wire       decbus0_block_begin;
wire       decbus0_int;

//decbus*_rdy*: not from decbus
wire       decbus0_rdy1;
wire       decbus0_rdy2;
wire       decbus0_rdy1_h;
wire       decbus0_rdy2_h;

wire [`Qpower-1:0] decbus0_qid1_queue;
wire [`Qpower-1:0] decbus0_qid2_queue;
wire [`Qpower-1:0] decbus0_qid1;
wire [`Qpower-1:0] decbus0_qid2;

wire       decbus1_valid;
wire [7:0] decbus1_op;
wire [7:0] decbus1_src1;
wire [7:0] decbus1_src2;
wire [7:0] decbus1_dest;
wire [`Lword-1:0] decbus1_imm;
wire [`Lword-1:0] decbus1_pc;
wire [1:0] decbus1_ce;
wire       decbus1_bd;
wire       decbus1_adei;
wire       decbus1_tlbii;
wire       decbus1_tlbir;
wire       decbus1_ibe;
wire       decbus1_ri;
wire       decbus1_cpu;
wire       decbus1_sys;
wire       decbus1_bp;
wire       decbus1_sdbbp;
wire       decbus1_dib;
wire[7:0]  decbus1_gshare;
wire[1:0]  decbus1_rashead;
wire[1:0]  decbus1_other_link;
wire[1:0]  decbus1_ac;
wire       decbus1_nop;
wire       decbus1_block_begin;
wire       decbus1_int;

wire       decbus1_rdy1;
wire       decbus1_rdy2;
wire       decbus1_rdy1_h;
wire       decbus1_rdy2_h;
wire [`Qpower-1:0] decbus1_qid1_queue;
wire [`Qpower-1:0] decbus1_qid2_queue;
wire [`Qpower-1:0] decbus1_qid1;
wire [`Qpower-1:0] decbus1_qid2;
wire brbus_brerror;
wire[`Qsize-1:0] brbus_vector;

wire decbus0_double_dest = decbus0[133];
wire decbus0_double_src2 = decbus0[132]; 
wire decbus0_double_src1 = decbus0[131]; 

wire dec0_mm_op, dec1_mm_op;
wire dec0_alu_one, dec1_alu_one;
wire dec0_falu, dec1_falu;
wire dec0_acc_op, dec1_acc_op;
wire dec0_wait, dec1_wait;
wire dec0_stall,dec1_stall;
wire decbus0_blikely,decbus1_blikely;
wire decbus0_block_end, decbus1_block_end;
wire commitbus_ex;
wire decbus0_imm_hi_en, decbus1_imm_hi_en;
wire decbus0_imm_low_en, decbus1_imm_low_en;

assign decbus0_int         = decbus0[144];
assign decbus0_imm_hi_en   = decbus0[143] & decbus0_valid;
assign decbus0_imm_low_en  = decbus0[142] & decbus0_valid;
assign decbus0_block_end   = decbus0[141];
assign decbus0_blikely     = decbus0[140] & decbus0_valid;
assign dec0_wait           = decbus0[139] & decbus0_valid;
assign dec0_stall          = decbus0[138] & decbus0_valid;
assign dec0_acc_op         = decbus0[137] & decbus0_valid;
assign dec0_falu           = decbus0[136] & decbus0_valid;
assign dec0_alu_one        = decbus0[135] & decbus0_valid;
assign dec0_mm_op          = decbus0[134] & decbus0_valid;
assign decbus0_block_begin = decbus0[130];
assign decbus0_nop         = decbus0[129];
assign decbus0_ac          = decbus0[128:127];
assign decbus0_other_link  = decbus0[126:125];
assign decbus0_rashead     = decbus0[124:123];
assign decbus0_gshare      = decbus0[122:115];
assign decbus0_valid       = decbus0[114]&!brbus_brerror & !commitbus_ex;
assign decbus0_sdbbp       = decbus0[113];
assign decbus0_dib         = decbus0[112];
assign decbus0_op          = decbus0[111:104];
assign decbus0_fmt         = decbus0[103:99];
assign decbus0_src1        = decbus0[98:91];
assign decbus0_src2        = decbus0[90:83];
assign decbus0_dest        = decbus0[82:75];
assign decbus0_imm         = decbus0[74:43];
assign decbus0_pc          = decbus0[42:11];
assign decbus0_ce          = decbus0[10:9];
assign decbus0_bd          = decbus0[8];
assign decbus0_adei        = decbus0[7];
assign decbus0_tlbii       = decbus0[6];
assign decbus0_tlbir       = decbus0[5];
assign decbus0_ibe         = decbus0[4];
assign decbus0_ri          = decbus0[3];
assign decbus0_cpu         = decbus0[2];
assign decbus0_sys         = decbus0[1];
assign decbus0_bp          = decbus0[0];

wire decbus1_double_dest = decbus1[133];
wire decbus1_double_src2 = decbus1[132]; 
wire decbus1_double_src1 = decbus1[131]; 

assign decbus1_int         = decbus1[144];
assign decbus1_imm_hi_en   = decbus1[143] & decbus1_valid;
assign decbus1_imm_low_en  = decbus1[142] & decbus1_valid;
assign decbus1_block_end   = decbus1[141];
assign decbus1_blikely     = decbus1[140] & decbus1_valid;
assign dec1_wait           = decbus1[139] & decbus1_valid;
assign dec1_stall          = decbus1[138] & decbus1_valid;
assign dec1_acc_op         = decbus1[137] & decbus1_valid;
assign dec1_falu           = decbus1[136] & decbus1_valid;
assign dec1_alu_one        = decbus1[135] & decbus1_valid;
assign dec1_mm_op          = decbus1[134] & decbus1_valid;
assign decbus1_block_begin = decbus1[130];
assign decbus1_nop         = decbus1[129];
assign decbus1_ac          = decbus1[128:127];
assign decbus1_other_link  = decbus1[126:125];
assign decbus1_rashead     = decbus1[124:123];
assign decbus1_gshare      = decbus1[122:115];
assign decbus1_valid       = decbus1[114]&!brbus_brerror&!commitbus_ex;
assign decbus1_sdbbp       = decbus1[113];
assign decbus1_dib         = decbus1[112];
assign decbus1_op          = decbus1[111:104];
assign decbus1_fmt         = decbus1[103:99];
assign decbus1_src1        = decbus1[98:91];
assign decbus1_src2        = decbus1[90:83];
assign decbus1_dest        = decbus1[82:75];
assign decbus1_imm         = decbus1[74:43];
assign decbus1_pc          = decbus1[42:11];
assign decbus1_ce          = decbus1[10:9];
assign decbus1_bd          = decbus1[8];
assign decbus1_adei        = decbus1[7];
assign decbus1_tlbii       = decbus1[6];
assign decbus1_tlbir       = decbus1[5];
assign decbus1_ibe         = decbus1[4];
assign decbus1_ri          = decbus1[3];
assign decbus1_cpu         = decbus1[2];
assign decbus1_sys         = decbus1[1];
assign decbus1_bp          = decbus1[0];
//    resbus
wire resbus0_op_ctc1;
wire resbus0_eret_deret;
wire resbus0_con_true;
wire resbus0_valid;  
wire [`Qpower-1:0] resbus0_qid;
wire [`Ldword-1:0] resbus0_value;
wire resbus0_ov;        
wire resbus0_trap;    
wire resbus0_mod;    
wire resbus0_tlbli;   
wire resbus0_tlblr;
wire resbus0_tlbsi;
wire resbus0_tlbsr; 
wire resbus0_adel; 
wire resbus0_ades; 
wire resbus0_dbe;  
wire resbus0_watch;  
wire resbus0_bnt;  
wire [5:0] resbus0_evzoui;
wire resbus0_ddbl;
wire resbus0_ddbs;
wire resbus0_ddblimpr;
wire resbus0_ddbsimpr;
wire resbus0_ri;

wire[31:0] resbus1_dspctl;
wire resbus1_con_true;
wire resbus1_valid;  
wire [`Qpower-1:0] resbus1_qid;
wire [`Ldword-1:0] resbus1_value;
wire resbus1_ov;        
wire resbus1_trap;    
wire resbus1_mod;    
wire resbus1_tlbli;   
wire resbus1_tlblr;
wire resbus1_tlbsi;
wire resbus1_tlbsr; 
wire resbus1_adel; 
wire resbus1_ades; 
wire resbus1_dbe;  
wire resbus1_watch;  
wire resbus1_bnt;  
wire [5:0] resbus1_evzoui;

wire [2:0] resbus2_fpqid;
wire resbus2_con_true;
wire resbus2_valid; 
wire [`Qpower-1:0] resbus2_qid;
wire [`Ldword-1:0] resbus2_value;
wire resbus2_ov;        
wire resbus2_trap;
wire resbus2_mod;    
wire resbus2_tlbli;   
wire resbus2_tlblr;
wire resbus2_tlbsi;
wire resbus2_tlbsr; 
wire resbus2_adel; 
wire resbus2_ades; 
wire resbus2_dbe;  
wire resbus2_watch;  
wire resbus2_bnt;  
wire [5:0] resbus2_evzoui; 

wire[31:0] resbus3_dspctl;
wire resbus3_con_true;
wire resbus3_valid;  
wire [`Qpower-1:0] resbus3_qid;
wire [`Ldword-1:0] resbus3_value;
wire resbus3_ov;        
wire resbus3_trap;    
wire resbus3_mod;    
wire resbus3_tlbli;   
wire resbus3_tlblr;
wire resbus3_tlbsi;
wire resbus3_tlbsr; 
wire resbus3_adel; 
wire resbus3_ades; 
wire resbus3_dbe;  
wire resbus3_watch;  
wire resbus3_bnt;  
wire [5:0] resbus3_evzoui;
wire resbus3_acc_op;  

wire resbus1_block_end;
wire resbus2_block_end, resbus3_block_end;

wire resbus0_write_fpq_tmp;
wire resbus0_write_fpq;
wire [2:0] resbus0_fpqid;
wire resbus0_wb_fpq;

assign resbus0_wb_fpq         = resbus0[98];
assign resbus0_write_fpq_tmp  = resbus0[97];
assign resbus0_fpqid      = resbus0[96:94];
assign resbus0_op_ctc1    = resbus0[93];
assign resbus0_eret_deret = resbus0[92];
assign resbus0_con_true   = resbus0[91];
assign resbus0_ddbl       = resbus0[90];
assign resbus0_ddbs       = resbus0[89];
assign resbus0_ddblimpr   = resbus0[88];
assign resbus0_ddbsimpr   = resbus0[87];
assign resbus0_bnt        = resbus0[86];
assign resbus0_valid      = resbus0[85]&&(!commitbus_ex);
assign resbus0_qid        = resbus0[84:81];
assign resbus0_value      = resbus0[80:17];  //change to 64bits
assign resbus0_ov         = 1'b0;
assign resbus0_ri         = resbus0[15];
assign resbus0_mod        = resbus0[14];
assign resbus0_tlbli      = resbus0[13];
assign resbus0_tlblr      = resbus0[12];
assign resbus0_tlbsi      = resbus0[11];
assign resbus0_tlbsr      = resbus0[10];
assign resbus0_adel       = resbus0[9];
assign resbus0_ades       = resbus0[8];
assign resbus0_dbe        = resbus0[7];
assign resbus0_watch      = resbus0[6];
assign resbus0_evzoui     = 6'h0;
//HAVE_DSP_UNIT
wire   resbus1_write_dspctl= resbus1[121];
assign resbus1_dspctl   = resbus1[119:88];
assign resbus1_block_end   = resbus1[120];
assign resbus1_con_true = resbus1[87];
assign resbus1_bnt      = resbus1[86];
assign resbus1_valid    = resbus1[85]&&(!commitbus_ex);
assign resbus1_qid      = resbus1[84:81];
assign resbus1_value    = resbus1[80:17];  //change to 64bits
assign resbus1_ov       = resbus1[16];
assign resbus1_trap     = resbus1[15];
assign resbus1_mod      = resbus1[14];
assign resbus1_tlbli    = resbus1[13];
assign resbus1_tlblr    = resbus1[12];
assign resbus1_tlbsi    = resbus1[11];
assign resbus1_tlbsr    = resbus1[10];
assign resbus1_adel     = resbus1[9];
assign resbus1_ades     = resbus1[8];
assign resbus1_dbe      = resbus1[7];
assign resbus1_watch    = resbus1[6];
assign resbus1_evzoui   = resbus1[5:0];

assign resbus2_fpqid    = 3'b0;
assign resbus2_block_end =1'b0;  
assign resbus2_con_true = 1'b0;  
assign resbus2_bnt      = 1'b0; 
assign resbus2_valid    = 1'b0; 
assign resbus2_qid      = 4'b0;  
assign resbus2_value    = 64'b0; 
assign resbus2_ov       = 1'b0;  
assign resbus2_trap     = 1'b0;  
assign resbus2_mod      = 1'b0;  
assign resbus2_tlbli    = 1'b0;  
assign resbus2_tlblr    = 1'b0;  
assign resbus2_tlbsi    = 1'b0;  
assign resbus2_tlbsr    = 1'b0;  
assign resbus2_adel     = 1'b0;  
assign resbus2_ades     = 1'b0;  
assign resbus2_dbe      = 1'b0;  
assign resbus2_watch    = 1'b0;  
assign resbus2_evzoui   = 6'b0;  

//HAVE_DSP_UNIT
wire   resbus3_write_dspctl= resbus3[122];
assign resbus3_dspctl   = resbus3[119:88];
assign resbus3_block_end= resbus3[121];
assign resbus3_acc_op   = resbus3[120];
assign resbus3_bnt      = resbus3[86];
assign resbus3_valid    = resbus3[85]&&(!commitbus_ex);
assign resbus3_qid      = resbus3[84:81];
assign resbus3_value    = resbus3[80:17];  //change to 64bits

assign resbus3_con_true = resbus3[87];
assign resbus3_ov       = resbus3[16];
assign resbus3_trap     = resbus3[15];

assign resbus3_mod      = resbus3[14];
assign resbus3_tlbli    = resbus3[13];
assign resbus3_tlblr    = resbus3[12];
assign resbus3_tlbsi    = resbus3[11];
assign resbus3_tlbsr    = resbus3[10];
assign resbus3_adel     = resbus3[9];
assign resbus3_ades     = resbus3[8];
assign resbus3_dbe      = resbus3[7];
assign resbus3_watch    = resbus3[6];
assign resbus3_evzoui   = resbus3[5:0];

wire [7:0] dest0,dest1,dest2,dest3,dest4,dest5,dest6,dest7,dest8,dest9,dest10,dest11,dest12,dest13,dest14,dest15;

//------------------------entries(regs) in operation queue-----------------------
reg [1:0]  queue_state[`Qsize-1:0]; //`EMPTY; `UNISSUED; `ISSUED
reg [7:0]  queue_op[`Qsize-1:0];
reg [7:0]  queue_src1[`Qsize-1:0];       //the highest 2 bits: 2'b00: fix; 2'b01: float; 2'b10: cp0
reg [7:0]  queue_src2[`Qsize-1:0];
reg [7:0]  queue_dest[`Qsize-1:0];
reg [15:0] queue_imm_hi[`Qsize-1:0]; //also the lower  32bits of result
reg [15:0] queue_imm_low[`Qsize-1:0]; //also the lower  32bits of result
reg [5:0]  queue_excode[`Qsize-1:0];
reg [1:0]  queue_ce[`Qsize-1:0];
reg [`BRQpower-1:0]  queue_brqid[`Qsize-1:0];
reg [1:0]  queue_other_link[`Qsize-1:0];   //to save way prediction 
reg [1:0]  queue_ac[`Qsize-1:0];   //to save ac indicating the num of ac 
//HAVE_DSP_UNIT
reg [31:0] queue_dspctl[`Qsize-1:0];   //the value writen to DSPCtrl register ; 
reg [`Qpower-1:0]  queue_qid1[`Qsize-1:0];   
reg [`Qpower-1:0]  queue_qid2[`Qsize-1:0];
reg [`Qsize-1:0] queue_wb;
reg [`Qsize-1:0] queue_rdy1;   //it's only assigned from decbus0_rdy1 when the inst enter queue
reg [`Qsize-1:0] queue_rdy2;
reg [`Qsize-1:0] queue_ex;
reg [`Qsize-1:0] queue_bd;
reg [`Qsize-1:0] queue_con_true; // for move conditon instruction;
reg [`Qsize-1:0] queue_block_begin; // for move conditon instruction;

/*******************************************************************/
wire [1:0] queue_state_0,queue_state_1,queue_state_2,queue_state_3;
wire [1:0] queue_state_4,queue_state_5,queue_state_6,queue_state_7;
wire [1:0] queue_state_8,queue_state_9,queue_state_10,queue_state_11;
wire [1:0] queue_state_12,queue_state_13,queue_state_14,queue_state_15;

wire [`BRQpower-1:0] queue_brqid_0, queue_brqid_1, queue_brqid_2, queue_brqid_3;
wire [`BRQpower-1:0] queue_brqid_4, queue_brqid_5, queue_brqid_6, queue_brqid_7;
wire [`BRQpower-1:0] queue_brqid_8, queue_brqid_9, queue_brqid_10,queue_brqid_11;
wire [`BRQpower-1:0] queue_brqid_12,queue_brqid_13,queue_brqid_14,queue_brqid_15;

wire [7:0] queue_op_0,queue_op_1,queue_op_2,queue_op_3;
wire [7:0] queue_op_4,queue_op_5,queue_op_6,queue_op_7;
wire [7:0] queue_op_8,queue_op_9,queue_op_10,queue_op_11;
wire [7:0] queue_op_12,queue_op_13,queue_op_14,queue_op_15;

wire [7:0] queue_src1_0,queue_src1_1,queue_src1_2,queue_src1_3;
wire [7:0] queue_src1_4,queue_src1_5,queue_src1_6,queue_src1_7;
wire [7:0] queue_src1_8,queue_src1_9,queue_src1_10,queue_src1_11;
wire [7:0] queue_src1_12,queue_src1_13,queue_src1_14,queue_src1_15;

wire [7:0] queue_src2_0,queue_src2_1,queue_src2_2,queue_src2_3;
wire [7:0] queue_src2_4,queue_src2_5,queue_src2_6,queue_src2_7;
wire [7:0] queue_src2_8,queue_src2_9,queue_src2_10,queue_src2_11;
wire [7:0] queue_src2_12,queue_src2_13,queue_src2_14,queue_src2_15;

wire [7:0] queue_dest_0,queue_dest_1,queue_dest_2,queue_dest_3;
wire [7:0] queue_dest_4,queue_dest_5,queue_dest_6,queue_dest_7;
wire [7:0] queue_dest_8,queue_dest_9,queue_dest_10,queue_dest_11;
wire [7:0] queue_dest_12,queue_dest_13,queue_dest_14,queue_dest_15;

wire [15:0] queue_imm_low_0, queue_imm_low_1, queue_imm_low_2, queue_imm_low_3;
wire [15:0] queue_imm_low_4, queue_imm_low_5, queue_imm_low_6, queue_imm_low_7;
wire [15:0] queue_imm_low_8, queue_imm_low_9, queue_imm_low_10,queue_imm_low_11;
wire [15:0] queue_imm_low_12,queue_imm_low_13,queue_imm_low_14,queue_imm_low_15;

wire [15:0] queue_imm_hi_0, queue_imm_hi_1, queue_imm_hi_2, queue_imm_hi_3;
wire [15:0] queue_imm_hi_4, queue_imm_hi_5, queue_imm_hi_6, queue_imm_hi_7;
wire [15:0] queue_imm_hi_8, queue_imm_hi_9, queue_imm_hi_10,queue_imm_hi_11;
wire [15:0] queue_imm_hi_12,queue_imm_hi_13,queue_imm_hi_14,queue_imm_hi_15;

wire       queue_wb_0,queue_wb_1,queue_wb_2,queue_wb_3;
wire       queue_wb_4,queue_wb_5,queue_wb_6,queue_wb_7;
wire       queue_wb_8,queue_wb_9,queue_wb_10,queue_wb_11;
wire       queue_wb_12,queue_wb_13,queue_wb_14,queue_wb_15;

wire       queue_ex_0,queue_ex_1,queue_ex_2,queue_ex_3;
wire       queue_ex_4,queue_ex_5,queue_ex_6,queue_ex_7;
wire       queue_ex_8,queue_ex_9,queue_ex_10,queue_ex_11;
wire       queue_ex_12,queue_ex_13,queue_ex_14,queue_ex_15;

wire [5:0] queue_excode_0,queue_excode_1,queue_excode_2,queue_excode_3;
wire [5:0] queue_excode_4,queue_excode_5,queue_excode_6,queue_excode_7;
wire [5:0] queue_excode_8,queue_excode_9,queue_excode_10,queue_excode_11;
wire [5:0] queue_excode_12,queue_excode_13,queue_excode_14,queue_excode_15;

wire [1:0] queue_ce_0,queue_ce_1,queue_ce_2,queue_ce_3;
wire [1:0] queue_ce_4,queue_ce_5,queue_ce_6,queue_ce_7;
wire [1:0] queue_ce_8,queue_ce_9,queue_ce_10,queue_ce_11;
wire [1:0] queue_ce_12,queue_ce_13,queue_ce_14,queue_ce_15;

wire       queue_bd_0,queue_bd_1,queue_bd_2,queue_bd_3;
wire       queue_bd_4,queue_bd_5,queue_bd_6,queue_bd_7;
wire       queue_bd_8,queue_bd_9,queue_bd_10,queue_bd_11;
wire       queue_bd_12,queue_bd_13,queue_bd_14,queue_bd_15;

wire       queue_rdy1_0,queue_rdy1_1,queue_rdy1_2,queue_rdy1_3;
wire       queue_rdy1_4,queue_rdy1_5,queue_rdy1_6,queue_rdy1_7;
wire       queue_rdy1_8,queue_rdy1_9,queue_rdy1_10,queue_rdy1_11;
wire       queue_rdy1_12,queue_rdy1_13,queue_rdy1_14,queue_rdy1_15;

wire       queue_rdy2_0,queue_rdy2_1,queue_rdy2_2,queue_rdy2_3;
wire       queue_rdy2_4,queue_rdy2_5,queue_rdy2_6,queue_rdy2_7;
wire       queue_rdy2_8,queue_rdy2_9,queue_rdy2_10,queue_rdy2_11;
wire       queue_rdy2_12,queue_rdy2_13,queue_rdy2_14,queue_rdy2_15;

wire [`Lword-1:0] queue_dspctl_0, queue_dspctl_1, queue_dspctl_2, queue_dspctl_3;
wire [`Lword-1:0] queue_dspctl_4, queue_dspctl_5, queue_dspctl_6, queue_dspctl_7;
wire [`Lword-1:0] queue_dspctl_8, queue_dspctl_9, queue_dspctl_10,queue_dspctl_11;
wire [`Lword-1:0] queue_dspctl_12,queue_dspctl_13,queue_dspctl_14,queue_dspctl_15;

wire [1:0] queue_ac_0, queue_ac_1, queue_ac_2, queue_ac_3;
wire [1:0] queue_ac_4, queue_ac_5, queue_ac_6, queue_ac_7;
wire [1:0] queue_ac_8, queue_ac_9, queue_ac_10,queue_ac_11;
wire [1:0] queue_ac_12,queue_ac_13,queue_ac_14,queue_ac_15;

wire [`Qpower-1:0] queue_qid1_0, queue_qid1_1, queue_qid1_2, queue_qid1_3;
wire [`Qpower-1:0] queue_qid1_4, queue_qid1_5, queue_qid1_6, queue_qid1_7;
wire [`Qpower-1:0] queue_qid1_8, queue_qid1_9, queue_qid1_10,queue_qid1_11;
wire [`Qpower-1:0] queue_qid1_12,queue_qid1_13,queue_qid1_14,queue_qid1_15;

wire [`Qpower-1:0] queue_qid2_0,queue_qid2_1,queue_qid2_2,queue_qid2_3;
wire [`Qpower-1:0] queue_qid2_4,queue_qid2_5,queue_qid2_6,queue_qid2_7;
wire [`Qpower-1:0] queue_qid2_8,queue_qid2_9,queue_qid2_10,queue_qid2_11;
wire [`Qpower-1:0] queue_qid2_12,queue_qid2_13,queue_qid2_14,queue_qid2_15;

wire [`Qpower-1:0] queue_qid1_tmp[`Qsize-1:0]; 
wire [`Qpower-1:0] queue_qid2_tmp[`Qsize-1:0];

assign queue_qid1_tmp[0]=queue_qid1_0;
assign queue_qid1_tmp[1]=queue_qid1_1;
assign queue_qid1_tmp[2]=queue_qid1_2;
assign queue_qid1_tmp[3]=queue_qid1_3;
assign queue_qid1_tmp[4]=queue_qid1_4;
assign queue_qid1_tmp[5]=queue_qid1_5;
assign queue_qid1_tmp[6]=queue_qid1_6;
assign queue_qid1_tmp[7]=queue_qid1_7;
assign queue_qid1_tmp[8]=queue_qid1_8;
assign queue_qid1_tmp[9]=queue_qid1_9;
assign queue_qid1_tmp[10]=queue_qid1_10;
assign queue_qid1_tmp[11]=queue_qid1_11;
assign queue_qid1_tmp[12]=queue_qid1_12;
assign queue_qid1_tmp[13]=queue_qid1_13;
assign queue_qid1_tmp[14]=queue_qid1_14;
assign queue_qid1_tmp[15]=queue_qid1_15;
assign queue_qid2_tmp[0]=queue_qid2_0;
assign queue_qid2_tmp[1]=queue_qid2_1;
assign queue_qid2_tmp[2]=queue_qid2_2;
assign queue_qid2_tmp[3]=queue_qid2_3;
assign queue_qid2_tmp[4]=queue_qid2_4;
assign queue_qid2_tmp[5]=queue_qid2_5;
assign queue_qid2_tmp[6]=queue_qid2_6;
assign queue_qid2_tmp[7]=queue_qid2_7;
assign queue_qid2_tmp[8]=queue_qid2_8;
assign queue_qid2_tmp[9]=queue_qid2_9;
assign queue_qid2_tmp[10]=queue_qid2_10;
assign queue_qid2_tmp[11]=queue_qid2_11;
assign queue_qid2_tmp[12]=queue_qid2_12;
assign queue_qid2_tmp[13]=queue_qid2_13;
assign queue_qid2_tmp[14]=queue_qid2_14;
assign queue_qid2_tmp[15]=queue_qid2_15;
assign queue_state_0=queue_state[0];       assign queue_state_1=queue_state[1]; 
assign queue_state_2=queue_state[2];       assign queue_state_3=queue_state[3]; 
assign queue_state_4=queue_state[4];       assign queue_state_5=queue_state[5]; 
assign queue_state_6=queue_state[6];       assign queue_state_7=queue_state[7]; 

assign queue_state_8=queue_state[8];       assign queue_state_9=queue_state[9]; 
assign queue_state_10=queue_state[10];       assign queue_state_11=queue_state[11]; 
assign queue_state_12=queue_state[12];       assign queue_state_13=queue_state[13]; 
assign queue_state_14=queue_state[14];       assign queue_state_15=queue_state[15]; 

assign queue_brqid_0 =queue_brqid[0];       assign queue_brqid_1 =queue_brqid[1]; 
assign queue_brqid_2 =queue_brqid[2];       assign queue_brqid_3 =queue_brqid[3]; 
assign queue_brqid_4 =queue_brqid[4];       assign queue_brqid_5 =queue_brqid[5]; 
assign queue_brqid_6 =queue_brqid[6];       assign queue_brqid_7 =queue_brqid[7]; 

assign queue_brqid_8 =queue_brqid[8];       assign queue_brqid_9 =queue_brqid[9]; 
assign queue_brqid_10=queue_brqid[10];      assign queue_brqid_11=queue_brqid[11]; 
assign queue_brqid_12=queue_brqid[12];      assign queue_brqid_13=queue_brqid[13]; 
assign queue_brqid_14=queue_brqid[14];      assign queue_brqid_15=queue_brqid[15]; 

assign queue_op_0=queue_op[0];             assign queue_op_1=queue_op[1]; 
assign queue_op_2=queue_op[2];             assign queue_op_3=queue_op[3]; 
assign queue_op_4=queue_op[4];             assign queue_op_5=queue_op[5]; 
assign queue_op_6=queue_op[6];             assign queue_op_7=queue_op[7]; 

assign queue_op_8=queue_op[8];       assign queue_op_9=queue_op[9]; 
assign queue_op_10=queue_op[10];       assign queue_op_11=queue_op[11]; 
assign queue_op_12=queue_op[12];       assign queue_op_13=queue_op[13]; 
assign queue_op_14=queue_op[14];       assign queue_op_15=queue_op[15]; 

assign queue_src1_0=queue_src1[0];         assign queue_src1_1=queue_src1[1]; 
assign queue_src1_2=queue_src1[2];         assign queue_src1_3=queue_src1[3]; 
assign queue_src1_4=queue_src1[4];         assign queue_src1_5=queue_src1[5]; 
assign queue_src1_6=queue_src1[6];         assign queue_src1_7=queue_src1[7]; 

assign queue_src1_8=queue_src1[8];       assign queue_src1_9=queue_src1[9]; 
assign queue_src1_10=queue_src1[10];       assign queue_src1_11=queue_src1[11]; 
assign queue_src1_12=queue_src1[12];       assign queue_src1_13=queue_src1[13]; 
assign queue_src1_14=queue_src1[14];       assign queue_src1_15=queue_src1[15]; 

assign queue_src2_0=queue_src2[0];         assign queue_src2_1=queue_src2[1];
assign queue_src2_2=queue_src2[2];         assign queue_src2_3=queue_src2[3];
assign queue_src2_4=queue_src2[4];         assign queue_src2_5=queue_src2[5];
assign queue_src2_6=queue_src2[6];         assign queue_src2_7=queue_src2[7];

assign queue_src2_8=queue_src2[8];       assign queue_src2_9=queue_src2[9]; 
assign queue_src2_10=queue_src2[10];       assign queue_src2_11=queue_src2[11]; 
assign queue_src2_12=queue_src2[12];       assign queue_src2_13=queue_src2[13]; 
assign queue_src2_14=queue_src2[14];       assign queue_src2_15=queue_src2[15]; 

assign queue_dest_0=queue_dest[0];         assign queue_dest_1=queue_dest[1]; 
assign queue_dest_2=queue_dest[2];         assign queue_dest_3=queue_dest[3]; 
assign queue_dest_4=queue_dest[4];         assign queue_dest_5=queue_dest[5]; 
assign queue_dest_6=queue_dest[6];         assign queue_dest_7=queue_dest[7]; 

assign queue_dest_8=queue_dest[8];       assign queue_dest_9=queue_dest[9]; 
assign queue_dest_10=queue_dest[10];       assign queue_dest_11=queue_dest[11]; 
assign queue_dest_12=queue_dest[12];       assign queue_dest_13=queue_dest[13]; 
assign queue_dest_14=queue_dest[14];       assign queue_dest_15=queue_dest[15]; 

assign queue_imm_low_0= queue_imm_low[0];        assign queue_imm_low_1= queue_imm_low[1]; 
assign queue_imm_low_2= queue_imm_low[2];        assign queue_imm_low_3= queue_imm_low[3]; 
assign queue_imm_low_4= queue_imm_low[4];        assign queue_imm_low_5= queue_imm_low[5]; 
assign queue_imm_low_6= queue_imm_low[6];        assign queue_imm_low_7= queue_imm_low[7]; 

assign queue_imm_low_8= queue_imm_low[8];        assign queue_imm_low_9= queue_imm_low[9]; 
assign queue_imm_low_10=queue_imm_low[10];       assign queue_imm_low_11=queue_imm_low[11]; 
assign queue_imm_low_12=queue_imm_low[12];       assign queue_imm_low_13=queue_imm_low[13]; 
assign queue_imm_low_14=queue_imm_low[14];       assign queue_imm_low_15=queue_imm_low[15]; 

assign queue_imm_hi_0= queue_imm_hi[0];        assign queue_imm_hi_1= queue_imm_hi[1]; 
assign queue_imm_hi_2= queue_imm_hi[2];        assign queue_imm_hi_3= queue_imm_hi[3]; 
assign queue_imm_hi_4= queue_imm_hi[4];        assign queue_imm_hi_5= queue_imm_hi[5]; 
assign queue_imm_hi_6= queue_imm_hi[6];        assign queue_imm_hi_7= queue_imm_hi[7]; 
assign queue_imm_hi_8= queue_imm_hi[8];        assign queue_imm_hi_9= queue_imm_hi[9]; 
assign queue_imm_hi_10=queue_imm_hi[10];       assign queue_imm_hi_11=queue_imm_hi[11]; 
assign queue_imm_hi_12=queue_imm_hi[12];       assign queue_imm_hi_13=queue_imm_hi[13]; 
assign queue_imm_hi_14=queue_imm_hi[14];       assign queue_imm_hi_15=queue_imm_hi[15]; 

assign queue_wb_0=queue_wb[0];             assign queue_wb_1=queue_wb[1]; 
assign queue_wb_2=queue_wb[2];             assign queue_wb_3=queue_wb[3]; 
assign queue_wb_4=queue_wb[4];             assign queue_wb_5=queue_wb[5]; 
assign queue_wb_6=queue_wb[6];             assign queue_wb_7=queue_wb[7]; 
assign queue_wb_8=queue_wb[8];       assign queue_wb_9=queue_wb[9]; 
assign queue_wb_10=queue_wb[10];       assign queue_wb_11=queue_wb[11]; 
assign queue_wb_12=queue_wb[12];       assign queue_wb_13=queue_wb[13]; 
assign queue_wb_14=queue_wb[14];       assign queue_wb_15=queue_wb[15]; 

assign queue_ex_0=queue_ex[0];             assign queue_ex_1=queue_ex[1]; 
assign queue_ex_2=queue_ex[2];             assign queue_ex_3=queue_ex[3]; 
assign queue_ex_4=queue_ex[4];             assign queue_ex_5=queue_ex[5]; 
assign queue_ex_6=queue_ex[6];             assign queue_ex_7=queue_ex[7]; 

assign queue_ex_8=queue_ex[8];       assign queue_ex_9=queue_ex[9]; 
assign queue_ex_10=queue_ex[10];       assign queue_ex_11=queue_ex[11]; 
assign queue_ex_12=queue_ex[12];       assign queue_ex_13=queue_ex[13]; 
assign queue_ex_14=queue_ex[14];       assign queue_ex_15=queue_ex[15]; 

assign queue_excode_0=queue_excode[0];     assign queue_excode_1=queue_excode[1]; 
assign queue_excode_2=queue_excode[2];     assign queue_excode_3=queue_excode[3]; 
assign queue_excode_4=queue_excode[4];     assign queue_excode_5=queue_excode[5]; 
assign queue_excode_6=queue_excode[6];     assign queue_excode_7=queue_excode[7]; 

assign queue_excode_8=queue_excode[8];       assign queue_excode_9=queue_excode[9]; 
assign queue_excode_10=queue_excode[10];       assign queue_excode_11=queue_excode[11]; 
assign queue_excode_12=queue_excode[12];       assign queue_excode_13=queue_excode[13]; 
assign queue_excode_14=queue_excode[14];       assign queue_excode_15=queue_excode[15]; 

assign queue_ce_0=queue_ce[0];             assign queue_ce_1=queue_ce[1]; 
assign queue_ce_2=queue_ce[2];             assign queue_ce_3=queue_ce[3]; 
assign queue_ce_4=queue_ce[4];             assign queue_ce_5=queue_ce[5]; 
assign queue_ce_6=queue_ce[6];             assign queue_ce_7=queue_ce[7]; 

assign queue_ce_8=queue_ce[8];       assign queue_ce_9=queue_ce[9]; 
assign queue_ce_10=queue_ce[10];       assign queue_ce_11=queue_ce[11]; 
assign queue_ce_12=queue_ce[12];       assign queue_ce_13=queue_ce[13]; 
assign queue_ce_14=queue_ce[14];       assign queue_ce_15=queue_ce[15]; 

assign queue_bd_0=queue_bd[0];             assign queue_bd_1=queue_bd[1]; 
assign queue_bd_2=queue_bd[2];             assign queue_bd_3=queue_bd[3]; 
assign queue_bd_4=queue_bd[4];             assign queue_bd_5=queue_bd[5]; 
assign queue_bd_6=queue_bd[6];             assign queue_bd_7=queue_bd[7]; 

assign queue_bd_8=queue_bd[8];       assign queue_bd_9=queue_bd[9]; 
assign queue_bd_10=queue_bd[10];       assign queue_bd_11=queue_bd[11]; 
assign queue_bd_12=queue_bd[12];       assign queue_bd_13=queue_bd[13]; 
assign queue_bd_14=queue_bd[14];       assign queue_bd_15=queue_bd[15]; 

assign queue_rdy1_0=queue_rdy1[0];         assign queue_rdy1_1=queue_rdy1[1]; 
assign queue_rdy1_2=queue_rdy1[2];         assign queue_rdy1_3=queue_rdy1[3]; 
assign queue_rdy1_4=queue_rdy1[4];         assign queue_rdy1_5=queue_rdy1[5]; 
assign queue_rdy1_6=queue_rdy1[6];         assign queue_rdy1_7=queue_rdy1[7]; 

assign queue_rdy1_8=queue_rdy1[8];       assign queue_rdy1_9=queue_rdy1[9]; 
assign queue_rdy1_10=queue_rdy1[10];       assign queue_rdy1_11=queue_rdy1[11]; 
assign queue_rdy1_12=queue_rdy1[12];       assign queue_rdy1_13=queue_rdy1[13]; 
assign queue_rdy1_14=queue_rdy1[14];       assign queue_rdy1_15=queue_rdy1[15]; 

assign queue_rdy2_0=queue_rdy2[0];         assign queue_rdy2_1=queue_rdy2[1]; 
assign queue_rdy2_2=queue_rdy2[2];         assign queue_rdy2_3=queue_rdy2[3]; 
assign queue_rdy2_4=queue_rdy2[4];         assign queue_rdy2_5=queue_rdy2[5]; 
assign queue_rdy2_6=queue_rdy2[6];         assign queue_rdy2_7=queue_rdy2[7]; 

assign queue_rdy2_8=queue_rdy2[8];       assign queue_rdy2_9=queue_rdy2[9]; 
assign queue_rdy2_10=queue_rdy2[10];       assign queue_rdy2_11=queue_rdy2[11]; 
assign queue_rdy2_12=queue_rdy2[12];       assign queue_rdy2_13=queue_rdy2[13]; 
assign queue_rdy2_14=queue_rdy2[14];       assign queue_rdy2_15=queue_rdy2[15]; 

assign queue_qid1_0= queue_qid1[0];         assign queue_qid1_1=queue_qid1[1]; 
assign queue_qid1_2= queue_qid1[2];         assign queue_qid1_3=queue_qid1[3]; 
assign queue_qid1_4= queue_qid1[4];         assign queue_qid1_5=queue_qid1[5]; 
assign queue_qid1_6= queue_qid1[6];         assign queue_qid1_7=queue_qid1[7]; 

assign queue_qid1_8= queue_qid1[8];         assign queue_qid1_9=queue_qid1[9]; 
assign queue_qid1_10=queue_qid1[10];       assign queue_qid1_11=queue_qid1[11]; 
assign queue_qid1_12=queue_qid1[12];       assign queue_qid1_13=queue_qid1[13]; 
assign queue_qid1_14=queue_qid1[14];       assign queue_qid1_15=queue_qid1[15]; 

assign queue_qid2_0= queue_qid2[0];         assign queue_qid2_1=queue_qid2[1]; 
assign queue_qid2_2= queue_qid2[2];         assign queue_qid2_3=queue_qid2[3]; 
assign queue_qid2_4= queue_qid2[4];         assign queue_qid2_5=queue_qid2[5]; 
assign queue_qid2_6= queue_qid2[6];         assign queue_qid2_7=queue_qid2[7]; 

assign queue_qid2_8= queue_qid2[8];         assign queue_qid2_9=queue_qid2[9]; 
assign queue_qid2_10=queue_qid2[10];       assign queue_qid2_11=queue_qid2[11]; 
assign queue_qid2_12=queue_qid2[12];       assign queue_qid2_13=queue_qid2[13]; 
assign queue_qid2_14=queue_qid2[14];       assign queue_qid2_15=queue_qid2[15]; 

//HAVE_DSP_UNIT
assign queue_dspctl_0 = queue_dspctl[0];        assign queue_dspctl_1 = queue_dspctl[1]; 
assign queue_dspctl_2 = queue_dspctl[2];        assign queue_dspctl_3 = queue_dspctl[3]; 
assign queue_dspctl_4 = queue_dspctl[4];        assign queue_dspctl_5 = queue_dspctl[5]; 
assign queue_dspctl_6 = queue_dspctl[6];        assign queue_dspctl_7 = queue_dspctl[7]; 

assign queue_dspctl_8 = queue_dspctl[8];        assign queue_dspctl_9 = queue_dspctl[9]; 
assign queue_dspctl_10= queue_dspctl[10];       assign queue_dspctl_11= queue_dspctl[11]; 
assign queue_dspctl_12= queue_dspctl[12];       assign queue_dspctl_13= queue_dspctl[13]; 
assign queue_dspctl_14= queue_dspctl[14];       assign queue_dspctl_15= queue_dspctl[15]; 

assign queue_ac_0 = queue_ac[0];        assign queue_ac_1 = queue_ac[1]; 
assign queue_ac_2 = queue_ac[2];        assign queue_ac_3 = queue_ac[3]; 
assign queue_ac_4 = queue_ac[4];        assign queue_ac_5 = queue_ac[5]; 
assign queue_ac_6 = queue_ac[6];        assign queue_ac_7 = queue_ac[7]; 

assign queue_ac_8 = queue_ac[8];        assign queue_ac_9 = queue_ac[9]; 
assign queue_ac_10= queue_ac[10];       assign queue_ac_11= queue_ac[11]; 
assign queue_ac_12= queue_ac[12];       assign queue_ac_13= queue_ac[13]; 
assign queue_ac_14= queue_ac[14];       assign queue_ac_15= queue_ac[15]; 
//-------------entries in branch queue----------------------------------
reg[`BRQsize-1 : 0]     brq_valid       ;
reg[`BRQsize-1 : 0]     brq_wb          ;
reg[`BRQsize-1 : 0]     brq_bdrdy       ;
reg[`BRQsize-1 : 0]     brq_brerror     ;
reg[`BRQsize-1 : 0]     brq_resolved  ;
reg[`BRQsize-1 : 0]     brq_brtaken     ;
reg[31:0]  brq_pc       [`BRQsize-1 : 0];
reg[1:0]   brq_rashead  [`BRQsize-1 : 0];
reg[7:0]   brq_gshare   [`BRQsize-1 : 0];
reg[1:0]   brq_status   [`BRQsize-1 : 0];
reg[7:0]   brq_op       [`BRQsize-1 : 0];
reg[`Qpower-1:0]   brq_offset   [`BRQsize-1 : 0];
reg[`Qpower-1:0]   brq_qid      [`BRQsize-1 : 0];

reg[31:0]  commit_pc;

reg [`BRQpower-1:0]  brq_head, brq_tail;
wire[`BRQpower-1:0]  brq_head_next, brq_tail_next,brq_tail_minus1;
wire[`BRQsize-1:0]  brq_head_d, brq_tail_d, brq_tail_minus1_d;

wire[31:0] brq_pc_0 = brq_pc[0];                 wire[31:0] brq_pc_1 = brq_pc[1];
wire[31:0] brq_pc_2 = brq_pc[2];                 wire[31:0] brq_pc_3 = brq_pc[3];
wire[31:0] brq_pc_4 = brq_pc[4];                 wire[31:0] brq_pc_5 = brq_pc[5];
wire[7:0] brq_op_0 = brq_op[0];                 wire[7:0] brq_op_1 = brq_op[1];
wire[7:0] brq_op_2 = brq_op[2];                 wire[7:0] brq_op_3 = brq_op[3];
wire[7:0] brq_op_4 = brq_op[4];                 wire[7:0] brq_op_5 = brq_op[5];
wire[1:0] brq_rashead_0 = brq_rashead[0];        wire[1:0] brq_rashead_1 = brq_rashead[1];
wire[1:0] brq_rashead_2 = brq_rashead[2];        wire[1:0] brq_rashead_3 = brq_rashead[3];
wire[1:0] brq_rashead_4 = brq_rashead[4];        wire[1:0] brq_rashead_5 = brq_rashead[5];
wire[7:0] brq_gshare_0 = brq_gshare[0];          wire[7:0] brq_gshare_1 = brq_gshare[1];
wire[7:0] brq_gshare_2 = brq_gshare[2];          wire[7:0] brq_gshare_3 = brq_gshare[3];
wire[7:0] brq_gshare_4 = brq_gshare[4];          wire[7:0] brq_gshare_5 = brq_gshare[5];
wire[`Qpower-1:0] brq_offset_0 = brq_offset[0];          wire[`Qpower-1:0] brq_offset_1 = brq_offset[1];
wire[`Qpower-1:0] brq_offset_2 = brq_offset[2];          wire[`Qpower-1:0] brq_offset_3 = brq_offset[3];
wire[`Qpower-1:0] brq_offset_4 = brq_offset[4];          wire[`Qpower-1:0] brq_offset_5 = brq_offset[5];
wire[1:0] brq_status_0 = brq_status[0];          wire[1:0] brq_status_1 = brq_status[1];
wire[1:0] brq_status_2 = brq_status[2];          wire[1:0] brq_status_3 = brq_status[3];
wire[1:0] brq_status_4 = brq_status[4];          wire[1:0] brq_status_5 = brq_status[5];
wire[`Qpower-1:0] brq_qid_0 = brq_qid[0];             wire[`Qpower-1:0] brq_qid_1 = brq_qid[1];
wire[`Qpower-1:0] brq_qid_2 = brq_qid[2];             wire[`Qpower-1:0] brq_qid_3 = brq_qid[3];
wire[`Qpower-1:0] brq_qid_4 = brq_qid[4];             wire[`Qpower-1:0] brq_qid_5 = brq_qid[5];

integer i;
wire [`Qsize-1:0] wb0_d,wb1_d,wb3_d,wb2_d;
wire [`Qsize-1:0] s_empty,s_unissued,s_issued,s_wb;
wire [`Qsize-1:0] s_empty_tmp;
wire [`Qsize-1:0] full,stalli,store_wait;
wire [`Qsize-1:0] q_op_ctc1,q_op_eret,q_op_jalr;
wire [`Qsize-1:0] q_op_branch,q_op_blikely,q_op_br,q_op_static_br;
wire [`Qsize-1:0] q_exestep;

wire [`Qsize-1:0] in_en_rdy1;
wire [`Qsize-1:0] in_en_rdy2;
wire [`Qsize-1:0] in_en_rdy1_h;
wire [`Qsize-1:0] in_en_rdy2_h;

wire [`Qsize-1:0] in_en_wait;

wire [`Qsize-1:0] in_en_state,in_en_op, in_en_imm_hi, in_en_imm_low,
                  in_en_wb,in_en_ex, in_en_excode, in_en_con, in_en_other_link;

wire [`Qsize-1:0] in_en_ac, in_en_dspctl;
wire in_en_head;

wire        dint;     //debug interrupt
wire        dss;      //debug single-step
wire        f_int;      //final interrupt
wire        nmi_now;
wire [`Qsize-1:0] rt_int_v;

assign int_trigger_o = dint | f_int | nmi_now;

//--------wires in qissuebus---------------------------------
wire [1:0]           qissuebus0_rs;
wire                 qissuebus0_valid;
wire [`Qpower-1:0]   qissuebus0_qid;
wire [`BRQpower-1:0] qissuebus0_brqid;
wire [7:0]           qissuebus0_op;
wire [7:0]           qissuebus0_src1;
wire [7:0]           qissuebus0_src2;
wire [7:0]           qissuebus0_dest;
wire [`Lword-1:0]    qissuebus0_res1;
wire [`Lword-1:0]    qissuebus0_res2;
wire [15:0]          qissuebus0_offset;
wire [1:0]           qissuebus0_ac;
wire                 qissuebus0_rdy1;
wire                 qissuebus0_rdy2;
wire [`Qpower-1:0]   qissuebus0_qid1;
wire [`Qpower-1:0]   qissuebus0_qid2;

assign issue_to_mmrs_qj = {qissuebus0_rdy1, qissuebus0_qid1};

wire [1:0]           qissuebus1_rs;
wire                 qissuebus1_valid;
wire [`Qpower-1:0]   qissuebus1_qid;
wire [`BRQpower-1:0] qissuebus1_brqid;
wire [7:0] qissuebus1_op;
wire [7:0] qissuebus1_src1;
wire [7:0] qissuebus1_src2;
wire [7:0] qissuebus1_dest;
wire [`Lword-1:0] qissuebus1_res1;
wire [`Lword-1:0] qissuebus1_res2;
wire [1:0]  qissuebus1_ac;
wire       qissuebus1_rdy1;
wire       qissuebus1_rdy2;
wire [`Qpower-1:0] qissuebus1_qid1;
wire [`Qpower-1:0] qissuebus1_qid2;
//--------wires in commitbus---------------------------------
wire[31:0]        commitbus_low_0          , commitbus_low_1;
wire[31:0]        commitbus_hi_0           , commitbus_hi_1;
wire[1:0]         commitbus_ac_0           , commitbus_ac_1;
wire[31:0]        commitbus_dspctl_0       , commitbus_dspctl_1;
wire[7:0]         commitbus_gshare_0       , commitbus_gshare_1;
wire[1:0]         commitbus_rashead_0      , commitbus_rashead_1;
wire[1:0]         commitbus_other_link_0   , commitbus_other_link_1;
wire              commitbus_con_true_0     , commitbus_con_true_1;
wire [`Lword-1:0] commitbus_taken_target_0 , commitbus_taken_target_1;
wire [1:0]        commitbus_old_status_0   , commitbus_old_status_1;
wire              commitbus_valid_0        , commitbus_valid_1;
wire [`Qpower-1:0]        commitbus_qid_0          , commitbus_qid_1;
wire [7:0]        commitbus_op_0           , commitbus_op_1;
wire [7:0]        commitbus_dest_0         , commitbus_dest_1;
wire              commitbus_fp_0           , commitbus_fp_1;
wire [`Lword-1:0] commitbus_pc_0           , commitbus_pc_1;
wire [`Lword-1:0] commitbus_value_0        , commitbus_value_1;
wire [5:0]        commitbus_excode_0       , commitbus_excode_1;
wire              commitbus_bd_0           , commitbus_bd_1;
wire [1:0]        commitbus_ce_0           , commitbus_ce_1;
wire              commitbus_ex_0           , commitbus_ex_1;
wire              commit_wb_0              , commit_wb_1;
wire [1:0]        commit_state_0           , commit_state_1;
wire [5:0]        commit_excode_0          , commit_excode_1;
wire              commit_ex_0              , commit_ex_1;

assign commitbus_ex = commitbus_ex_0 | commitbus_ex_1;

assign offset_to_mm = qissuebus0_offset;//mm operation nust be on qissuebus0.
/****************** wires for issue *******************/
wire [`Qpower-1:0]        resqid1_0, resqid2_0; 
wire [`Qpower-1:0]        resqid1_1, resqid2_1;

/*---------------------------------First, basic part----------------------------------------------*/
decoder_4_16 decode4_16_0(.enable(1'b1),.in(resbus0_qid),.out(wb0_d));
decoder_4_16 decode4_16_1(.enable(1'b1),.in(resbus1_qid),.out(wb1_d));
decoder_4_16 decode4_16_2(.enable(1'b1),.in(resbus2_qid),.out(wb2_d));
decoder_4_16 decode4_16_3(.enable(1'b1),.in(resbus3_qid),.out(wb3_d));

wire [`Qsize-1:0] wait_vector;
wire [`Qsize-1:0] mm_op_vector;
wire [`Qsize-1:0] falu_op_vector;
wire [`Qsize-1:0] acc_op_vector;
wire [`Qsize-1:0] div_op_vector;
wire [`Qsize-1:0] acc_wait;
wire [`Qsize-1:0] mm_is_falu_vector;
wire [`Qsize-1:0] lw_op_v_tmp;
wire [`Qsize-1:0] lw_op_vector;

queue_decode q0(.q_state(queue_state_0),.q_op(queue_op_0),.q_wb(queue_wb_0),.q_ex(queue_ex_0),
                .s_empty(s_empty[0]),.s_unissued(s_unissued[0]),.s_issued(s_issued[0]),.s_wb(s_wb[0]),
                .full(full[0]),.stalli(stalli[0]),.store_wait(store_wait[0]),.jalr(q_op_jalr[0]),
                .ctc1(q_op_ctc1[0]),.eret(q_op_eret[0]),.branch(q_op_branch[0]),
                .blikely(q_op_blikely[0]),.br(q_op_br[0]),
                .static_br(q_op_static_br[0]), 
                .q_exestep(q_exestep[0]),
                .wait_operation(wait_vector[0]),
                .mm_operation(mm_op_vector[0]), 
                .falu_operation(falu_op_vector[0]), 
                .acc_op(acc_op_vector[0]), .div_op(div_op_vector[0]), .acc_wait(acc_wait[0]), .mm_is_falu(mm_is_falu_vector[0]));

queue_decode q1(.q_state(queue_state_1),.q_op(queue_op_1),.q_wb(queue_wb_1),.q_ex(queue_ex_1),
                .s_empty(s_empty[1]),.s_unissued(s_unissued[1]),.s_issued(s_issued[1]),.s_wb(s_wb[1]),
                .full(full[1]),.stalli(stalli[1]),.store_wait(store_wait[1]),.jalr(q_op_jalr[1]),
                .ctc1(q_op_ctc1[1]),.eret(q_op_eret[1]),.branch(q_op_branch[1]),
                .blikely(q_op_blikely[1]),.br(q_op_br[1]),
                .static_br(q_op_static_br[1]), 
                .q_exestep(q_exestep[1]),
                .wait_operation(wait_vector[1]),
                .mm_operation(mm_op_vector[1]),
                .falu_operation(falu_op_vector[1]),
                .acc_op(acc_op_vector[1]), .div_op(div_op_vector[1]), .acc_wait(acc_wait[1]), .mm_is_falu(mm_is_falu_vector[1]));
                  
queue_decode q2(.q_state(queue_state_2),.q_op(queue_op_2),.q_wb(queue_wb_2),.q_ex(queue_ex_2),
                .s_empty(s_empty[2]),.s_unissued(s_unissued[2]),.s_issued(s_issued[2]),.s_wb(s_wb[2]),
                .full(full[2]),.stalli(stalli[2]),.store_wait(store_wait[2]),.jalr(q_op_jalr[2]),
                .ctc1(q_op_ctc1[2]),.eret(q_op_eret[2]),.branch(q_op_branch[2]),
                .blikely(q_op_blikely[2]),.br(q_op_br[2]),
                .static_br(q_op_static_br[2]), 
                .q_exestep(q_exestep[2]),
                .wait_operation(wait_vector[2]),
                .mm_operation(mm_op_vector[2]),
                .falu_operation(falu_op_vector[2]),
                .acc_op(acc_op_vector[2]), .div_op(div_op_vector[2]), .acc_wait(acc_wait[2]), .mm_is_falu(mm_is_falu_vector[2]));
                      
queue_decode q3(.q_state(queue_state_3),.q_op(queue_op_3),.q_wb(queue_wb_3),.q_ex(queue_ex_3),
                .s_empty(s_empty[3]),.s_unissued(s_unissued[3]),.s_issued(s_issued[3]),.s_wb(s_wb[3]),
                .full(full[3]),.stalli(stalli[3]),.store_wait(store_wait[3]),.jalr(q_op_jalr[3]),
                .ctc1(q_op_ctc1[3]),.eret(q_op_eret[3]),.branch(q_op_branch[3]),
                .blikely(q_op_blikely[3]),.br(q_op_br[3]),
                .static_br(q_op_static_br[3]), 
                .q_exestep(q_exestep[3]),
                .wait_operation(wait_vector[3]),
                .mm_operation(mm_op_vector[3]),
                .falu_operation(falu_op_vector[3]),
                .acc_op(acc_op_vector[3]), .div_op(div_op_vector[3]), .acc_wait(acc_wait[3]), .mm_is_falu(mm_is_falu_vector[3]));
   
queue_decode q4(.q_state(queue_state_4),.q_op(queue_op_4),.q_wb(queue_wb_4),.q_ex(queue_ex_4),
                .s_empty(s_empty[4]),.s_unissued(s_unissued[4]),.s_issued(s_issued[4]),.s_wb(s_wb[4]),
                .full(full[4]),.stalli(stalli[4]),.store_wait(store_wait[4]),.jalr(q_op_jalr[4]),
                .ctc1(q_op_ctc1[4]),.eret(q_op_eret[4]),.branch(q_op_branch[4]),
                .blikely(q_op_blikely[4]),.br(q_op_br[4]),
                .static_br(q_op_static_br[4]), 
                .q_exestep(q_exestep[4]),
                .wait_operation(wait_vector[4]),
                .mm_operation(mm_op_vector[4]),
                .falu_operation(falu_op_vector[4]),
                .acc_op(acc_op_vector[4]), .div_op(div_op_vector[4]), .acc_wait(acc_wait[4]), .mm_is_falu(mm_is_falu_vector[4]));
   
queue_decode q5(.q_state(queue_state_5),.q_op(queue_op_5),.q_wb(queue_wb_5),.q_ex(queue_ex_5),
                .s_empty(s_empty[5]),.s_unissued(s_unissued[5]),.s_issued(s_issued[5]),.s_wb(s_wb[5]),
                .full(full[5]),.stalli(stalli[5]),.store_wait(store_wait[5]),.jalr(q_op_jalr[5]),
                .ctc1(q_op_ctc1[5]),.eret(q_op_eret[5]),.branch(q_op_branch[5]),
                .blikely(q_op_blikely[5]),.br(q_op_br[5]),
                .static_br(q_op_static_br[5]), 
                .q_exestep(q_exestep[5]),
                .wait_operation(wait_vector[5]),
                .mm_operation(mm_op_vector[5]),
                .falu_operation(falu_op_vector[5]),
                .acc_op(acc_op_vector[5]), .div_op(div_op_vector[5]), .acc_wait(acc_wait[5]), .mm_is_falu(mm_is_falu_vector[5]));
                   
queue_decode q6(.q_state(queue_state_6),.q_op(queue_op_6),.q_wb(queue_wb_6),.q_ex(queue_ex_6),
                .s_empty(s_empty[6]),.s_unissued(s_unissued[6]),.s_issued(s_issued[6]),.s_wb(s_wb[6]),
                .full(full[6]),.stalli(stalli[6]),.store_wait(store_wait[6]),.jalr(q_op_jalr[6]),
                .ctc1(q_op_ctc1[6]),.eret(q_op_eret[6]),.branch(q_op_branch[6]),
                .blikely(q_op_blikely[6]),.br(q_op_br[6]),
                .static_br(q_op_static_br[6]), 
                .q_exestep(q_exestep[6]),
                .wait_operation(wait_vector[6]),
                .mm_operation(mm_op_vector[6]),
                .falu_operation(falu_op_vector[6]),
                .acc_op(acc_op_vector[6]), .div_op(div_op_vector[6]), .acc_wait(acc_wait[6]), .mm_is_falu(mm_is_falu_vector[6]));
                   
queue_decode q7(.q_state(queue_state_7),.q_op(queue_op_7),.q_wb(queue_wb_7),.q_ex(queue_ex_7),
                .s_empty(s_empty[7]),.s_unissued(s_unissued[7]),.s_issued(s_issued[7]),.s_wb(s_wb[7]),
                .full(full[7]),.stalli(stalli[7]),.store_wait(store_wait[7]),.jalr(q_op_jalr[7]),
                .ctc1(q_op_ctc1[7]),.eret(q_op_eret[7]),.branch(q_op_branch[7]),
                .blikely(q_op_blikely[7]),.br(q_op_br[7]),
                .static_br(q_op_static_br[7]), 
                .q_exestep(q_exestep[7]),
                .wait_operation(wait_vector[7]),
                .mm_operation(mm_op_vector[7]),
                .falu_operation(falu_op_vector[7]),
                .acc_op(acc_op_vector[7]), .div_op(div_op_vector[7]), .acc_wait(acc_wait[7]), .mm_is_falu(mm_is_falu_vector[7]));
queue_decode q8(.q_state(queue_state_8),.q_op(queue_op_8),.q_wb(queue_wb_8),.q_ex(queue_ex_8),
                .s_empty(s_empty[8]),.s_unissued(s_unissued[8]),.s_issued(s_issued[8]),.s_wb(s_wb[8]),
                .full(full[8]),.stalli(stalli[8]),.store_wait(store_wait[8]),.jalr(q_op_jalr[8]),
                .ctc1(q_op_ctc1[8]),.eret(q_op_eret[8]),.branch(q_op_branch[8]),
                .blikely(q_op_blikely[8]),.br(q_op_br[8]),
                .static_br(q_op_static_br[8]), 
                .q_exestep(q_exestep[8]),
                .wait_operation(wait_vector[8]),
                .mm_operation(mm_op_vector[8]),
                .falu_operation(falu_op_vector[8]),
                .acc_op(acc_op_vector[8]), .div_op(div_op_vector[8]), .acc_wait(acc_wait[8]), .mm_is_falu(mm_is_falu_vector[8]));
                   
queue_decode q9(.q_state(queue_state_9),.q_op(queue_op_9),.q_wb(queue_wb_9),.q_ex(queue_ex_9),
                .s_empty(s_empty[9]),.s_unissued(s_unissued[9]),.s_issued(s_issued[9]),.s_wb(s_wb[9]),
                .full(full[9]),.stalli(stalli[9]),.store_wait(store_wait[9]), .jalr(q_op_jalr[9]),
                .ctc1(q_op_ctc1[9]),.eret(q_op_eret[9]),.branch(q_op_branch[9]),
                .blikely(q_op_blikely[9]),.br(q_op_br[9]),
                .static_br(q_op_static_br[9]), 
                .q_exestep(q_exestep[9]),
                .wait_operation(wait_vector[9]),
                .mm_operation(mm_op_vector[9]),
                .falu_operation(falu_op_vector[9]),
                .acc_op(acc_op_vector[9]), .div_op(div_op_vector[9]), .acc_wait(acc_wait[9]), .mm_is_falu(mm_is_falu_vector[9]));
                   
queue_decode q10(.q_state(queue_state_10),.q_op(queue_op_10),.q_wb(queue_wb_10),.q_ex(queue_ex_10),
                .s_empty(s_empty[10]),.s_unissued(s_unissued[10]),.s_issued(s_issued[10]),.s_wb(s_wb[10]),
                .full(full[10]),.stalli(stalli[10]),.store_wait(store_wait[10]),.jalr(q_op_jalr[10]),
                .ctc1(q_op_ctc1[10]),.eret(q_op_eret[10]),.branch(q_op_branch[10]),
                .blikely(q_op_blikely[10]),.br(q_op_br[10]),
                .static_br(q_op_static_br[10]), 
                .q_exestep(q_exestep[10]),
                .wait_operation(wait_vector[10]),
                .mm_operation(mm_op_vector[10]),
                .falu_operation(falu_op_vector[10]),
                .acc_op(acc_op_vector[10]), .div_op(div_op_vector[10]), .acc_wait(acc_wait[10]), .mm_is_falu(mm_is_falu_vector[10]));

queue_decode q11(.q_state(queue_state_11),.q_op(queue_op_11),.q_wb(queue_wb_11),.q_ex(queue_ex_11),
                .s_empty(s_empty[11]),.s_unissued(s_unissued[11]),.s_issued(s_issued[11]),.s_wb(s_wb[11]),
                .full(full[11]),.stalli(stalli[11]),.store_wait(store_wait[11]),.jalr(q_op_jalr[11]),
                .ctc1(q_op_ctc1[11]),.eret(q_op_eret[11]),.branch(q_op_branch[11]),
                .blikely(q_op_blikely[11]),.br(q_op_br[11]),
                .static_br(q_op_static_br[11]), 
                .q_exestep(q_exestep[11]),
                .wait_operation(wait_vector[11]),
                .mm_operation(mm_op_vector[11]), 
                .falu_operation(falu_op_vector[11]), 
                .acc_op(acc_op_vector[11]), .div_op(div_op_vector[11]), .acc_wait(acc_wait[11]), .mm_is_falu(mm_is_falu_vector[11]));
                   
queue_decode q12(.q_state(queue_state_12),.q_op(queue_op_12),.q_wb(queue_wb_12),.q_ex(queue_ex_12),
                .s_empty(s_empty[12]),.s_unissued(s_unissued[12]),.s_issued(s_issued[12]),.s_wb(s_wb[12]),
                .full(full[12]),.stalli(stalli[12]),.store_wait(store_wait[12]),.jalr(q_op_jalr[12]),
                .ctc1(q_op_ctc1[12]),.eret(q_op_eret[12]),.branch(q_op_branch[12]),
                .blikely(q_op_blikely[12]),.br(q_op_br[12]),
                .static_br(q_op_static_br[12]), 
                .q_exestep(q_exestep[12]),
                .wait_operation(wait_vector[12]),
                .mm_operation(mm_op_vector[12]),
                .falu_operation(falu_op_vector[12]),
                .acc_op(acc_op_vector[12]), .div_op(div_op_vector[12]), .acc_wait(acc_wait[12]), .mm_is_falu(mm_is_falu_vector[12]));
                   
queue_decode q13(.q_state(queue_state_13),.q_op(queue_op_13),.q_wb(queue_wb_13),.q_ex(queue_ex_13),
                .s_empty(s_empty[13]),.s_unissued(s_unissued[13]),.s_issued(s_issued[13]),.s_wb(s_wb[13]),
                .full(full[13]),.stalli(stalli[13]),.store_wait(store_wait[13]),.jalr(q_op_jalr[13]),
                .ctc1(q_op_ctc1[13]),.eret(q_op_eret[13]),.branch(q_op_branch[13]),
                .blikely(q_op_blikely[13]),.br(q_op_br[13]),
                .static_br(q_op_static_br[13]), 
                .q_exestep(q_exestep[13]),
                .wait_operation(wait_vector[13]),
                .mm_operation(mm_op_vector[13]),
                .falu_operation(falu_op_vector[13]),
                .acc_op(acc_op_vector[13]), .div_op(div_op_vector[13]), .acc_wait(acc_wait[13]), .mm_is_falu(mm_is_falu_vector[13]));
                   
queue_decode q14(.q_state(queue_state_14),.q_op(queue_op_14),.q_wb(queue_wb_14),.q_ex(queue_ex_14),
                .s_empty(s_empty[14]),.s_unissued(s_unissued[14]),.s_issued(s_issued[14]),.s_wb(s_wb[14]),
                .full(full[14]),.stalli(stalli[14]),.store_wait(store_wait[14]),.jalr(q_op_jalr[14]),
                .ctc1(q_op_ctc1[14]),.eret(q_op_eret[14]),.branch(q_op_branch[14]),
                .blikely(q_op_blikely[14]),.br(q_op_br[14]),
                .static_br(q_op_static_br[14]), 
                .q_exestep(q_exestep[14]),
                .wait_operation(wait_vector[14]),
                .mm_operation(mm_op_vector[14]),
                .falu_operation(falu_op_vector[14]),
                .acc_op(acc_op_vector[14]), .div_op(div_op_vector[14]), .acc_wait(acc_wait[14]), .mm_is_falu(mm_is_falu_vector[14]));

queue_decode q15(.q_state(queue_state_15),.q_op(queue_op_15),.q_wb(queue_wb_15),.q_ex(queue_ex_15),
                .s_empty(s_empty[15]),.s_unissued(s_unissued[15]),.s_issued(s_issued[15]),.s_wb(s_wb[15]),
                .full(full[15]),.stalli(stalli[15]),.store_wait(store_wait[15]),.jalr(q_op_jalr[15]),
                .ctc1(q_op_ctc1[15]),.eret(q_op_eret[15]),.branch(q_op_branch[15]),
                .blikely(q_op_blikely[15]),.br(q_op_br[15]),
                .static_br(q_op_static_br[15]), 
                .q_exestep(q_exestep[15]),
                .wait_operation(wait_vector[15]),
                .mm_operation(mm_op_vector[15]),
                .falu_operation(falu_op_vector[15]),
                .acc_op(acc_op_vector[15]), .div_op(div_op_vector[15]), .acc_wait(acc_wait[15]), .mm_is_falu(mm_is_falu_vector[15]));
assign lw_op_v_tmp[0] = (queue_op_0  == `OP_LW) || (queue_op_0  == `OP_LWX);
assign lw_op_v_tmp[1] = (queue_op_1  == `OP_LW) || (queue_op_1  == `OP_LWX);
assign lw_op_v_tmp[2] = (queue_op_2  == `OP_LW) || (queue_op_2  == `OP_LWX);
assign lw_op_v_tmp[3] = (queue_op_3  == `OP_LW) || (queue_op_3  == `OP_LWX);
assign lw_op_v_tmp[4] = (queue_op_4  == `OP_LW) || (queue_op_4  == `OP_LWX);
assign lw_op_v_tmp[5] = (queue_op_5  == `OP_LW) || (queue_op_5  == `OP_LWX);
assign lw_op_v_tmp[6] = (queue_op_6  == `OP_LW) || (queue_op_6  == `OP_LWX);
assign lw_op_v_tmp[7] = (queue_op_7  == `OP_LW) || (queue_op_7  == `OP_LWX);
assign lw_op_v_tmp[8] = (queue_op_8  == `OP_LW) || (queue_op_8  == `OP_LWX);
assign lw_op_v_tmp[9] = (queue_op_9  == `OP_LW) || (queue_op_9  == `OP_LWX);
assign lw_op_v_tmp[10]= (queue_op_10 == `OP_LW) || (queue_op_10 == `OP_LWX);
assign lw_op_v_tmp[11]= (queue_op_11 == `OP_LW) || (queue_op_11 == `OP_LWX);
assign lw_op_v_tmp[12]= (queue_op_12 == `OP_LW) || (queue_op_12 == `OP_LWX);
assign lw_op_v_tmp[13]= (queue_op_13 == `OP_LW) || (queue_op_13 == `OP_LWX);
assign lw_op_v_tmp[14]= (queue_op_14 == `OP_LW) || (queue_op_14 == `OP_LWX);
assign lw_op_v_tmp[15]= (queue_op_15 == `OP_LW) || (queue_op_15 == `OP_LWX);
assign lw_op_vector   = ~s_empty & lw_op_v_tmp;
/*********************generate issue_rdy signals******************************/
/**************************choose the first mm, falu, alu1, alu2 operations*******************************/
reg  [`Qpower-1:0] head;
reg  [`Qsize-1:0] ge_head_vector;
reg  [`Qsize-1:0] queue_issue_rdy;

wire [`Qpower-1:0] head_next;
wire [`Qpower-1:0] head_next_next;
queue_next head_next_m(.in(head), .next(head_next));
queue_next head_next_next_m(.in(head_next), .next(head_next_next));

wire [`Qsize-1:0] head_d;
wire [`Qsize-1:0] head_next_d;
wire [`Qsize-1:0] head_next_next_d;
queue_decode_eq_4_16 decode_head(.in(head),.out(head_d));
assign head_next_d      = {head_d[`Qsize-2:0],head_d[`Qsize-1]};
assign head_next_next_d = {head_d[`Qsize-3:0],head_d[`Qsize-1], head_d[`Qsize-2]};

assign in_en_head  = commitbus_valid_0||commitbus_valid_1||reset||commitbus_ex;

wire [`Qpower-1:0]  head_tmp_1;

assign head_tmp_1 = (reset | commitbus_ex) ? `Qpower'b0 :
                    (commitbus_valid_0 & commitbus_valid_1) ? head_next_next : 
                     commitbus_valid_0 ? head_next : head; 

wire [`Qsize-1:0] ge_head_vector_tmp;
queue_decode_ge_4_16 ge_head_0 (.in(head_tmp_1), .out(ge_head_vector_tmp));

always @ (posedge clock)
begin
   if (in_en_head)
     begin
        head <= head_tmp_1;
        ge_head_vector <= ge_head_vector_tmp;
     end
end

wire [`Qpower-1:0] tail;
wire [`Qpower-1:0] tail_next;
wire [`Qsize-1:0] tail_d;
wire [`Qsize-1:0] tail_next_d;
wire [`Qsize-1:0] tail_tmp_d;

assign tail_tmp_d = (~{s_empty[`Qsize-2:0],s_empty[`Qsize-1]}) & s_empty;

assign tail_d = (&s_empty) ? head_d : tail_tmp_d; //tail_d points to the first EMPTY entry in queue.
                                                 //when queue full, tail is 0(not equal head)
assign tail_next_d = {tail_d[`Qsize-2:0], tail_d[`Qsize-1]};

queue_encode_eq_16_4 encode_tail(.in(tail_d),.out(tail));
queue_encode_eq_16_4 encode_tail_next(.in(tail_next_d),.out(tail_next));

wire [`Qsize-1:0] issue_0_d;
wire [`Qsize-1:0] issue_1_d;

wire [`Qsize-1:0] issued_tmp = {16{qissuebus0_valid}} & issue_0_d | {16{qissuebus1_valid}} & issue_1_d; 

//find the first mm operation that can be issued in the next cycle
wire dec0_ex, dec1_ex;
wire [`Qsize-1:0] mm_op_vector_1;
wire [`Qsize-1:0] mm_op_vector_2;
wire [`Qsize-1:0] mm_op_vector_1_tmp;
wire [`Qsize-1:0] mm_op_vector_2_tmp;
wire have_mm_full;
wire have_mm_ge_head;
wire [`Qsize-1:0]mm_op_vector_5;
wire dec0_mm_is_falu, dec1_mm_is_falu;
wire [`Qsize-1:0] mm_is_falu_vector_tmp;
wire [`Qsize-1:0] mm_op_vector_tmp;
wire mm_is_falu_tmp;

FALU_ON_ADDR falu_on_addr_0(.op(decbus0_op), .falu_addr(dec0_mm_is_falu));
FALU_ON_ADDR falu_on_addr_1(.op(decbus1_op), .falu_addr(dec1_mm_is_falu));

assign mm_is_falu_vector_tmp = mm_is_falu_vector | 
                              {16{dec0_mm_is_falu & decbus0_valid}} & tail_d | 
                              {16{dec1_mm_is_falu & decbus1_valid}} & tail_next_d; 
assign mm_op_vector_tmp = {16{dec0_mm_op}} & tail_d |
                          {16{dec1_mm_op}} & tail_next_d;   
assign mm_op_vector_1 = mm_op_vector & s_unissued & ~issued_tmp | mm_op_vector_tmp;
assign mm_op_vector_2 = mm_op_vector_1 & ge_head_vector_tmp;

first_one_16_16 first_one_16_16_0  (.in(mm_op_vector_1),  .out(mm_op_vector_1_tmp),  .has(have_mm_full));
first_one_16_16 first_one_16_16_1  (.in(mm_op_vector_2),  .out(mm_op_vector_2_tmp),  .has(have_mm_ge_head));

wire [`Qpower-1:0] mm_0, mm_1, mm_qid;
queue_encode_eq_16_4 encode_mm0(.in(mm_op_vector_1_tmp), .out(mm_0));
queue_encode_eq_16_4 encode_mm1(.in(mm_op_vector_2_tmp), .out(mm_1));

wire mm_is_falu_tmp_1 = |(mm_op_vector_1_tmp & mm_is_falu_vector_tmp); 
wire mm_is_falu_tmp_2 = |(mm_op_vector_2_tmp & mm_is_falu_vector_tmp); 

assign mm_qid = have_mm_ge_head ? mm_1 : mm_0;
assign mm_is_falu_tmp = have_mm_ge_head ? mm_is_falu_tmp_2 : mm_is_falu_tmp_1;

reg [`Qpower-1:0] mm_qid_reg;
reg mm_valid_reg;
reg mm_is_falu_reg;

always @(posedge clock)
begin
   mm_qid_reg <= mm_qid;
   mm_valid_reg <= have_mm_full;
   mm_is_falu_reg <= mm_is_falu_tmp;
end
queue_decode_eq_4_16 decode_mm(.in(mm_qid_reg),.out(mm_op_vector_5));

wire mm_is_falu = mm_is_falu_reg;

//find the first falu operation that can be issued next cycle
wire falu_is_mm;
wire [`Qsize-1:0] falu_op_vector_tmp;
wire [`Qsize-1:0] stall_op;
wire [`Qsize-1:0] wait_op;
wire [`Qsize-1:0] falu_op_vector_1;
wire [`Qsize-1:0] falu_op_vector_2;
wire [`Qsize-1:0] falu_op_vector_1_tmp;
wire [`Qsize-1:0] falu_op_vector_2_tmp;
wire [`Qsize-1:0]falu_op_vector_3;
wire have_falu_full_tmp;
wire have_falu_ge_head;
wire have_falu_full;
wire [`Qsize-1:0] head_d_tmp;
wire [`Qsize-1:0] queue_issue_to_fpq_rdy_tmp;

assign head_d_tmp = (reset | commitbus_ex) ? 16'h01 :
                    (commitbus_valid_0 & commitbus_valid_1) ? head_next_next_d : 
                     commitbus_valid_0 ? head_next_d : head_d;

assign falu_op_vector_3 = 16'b0;
assign have_falu_full   = 1'b0;
assign falu_is_mm       = 1'b0;

//find the first alu_one operation that can be issued next cycle
wire commit0_alu_one, commit1_alu_one;

ALU_ONE alu_one_commit0(.op(commitbus_op_0), .alu_one(commit0_alu_one));

ALU_ONE alu_one_commit1(.op(commitbus_op_1), .alu_one(commit1_alu_one));

wire [`Qsize-1:0] commitbus0_qid_d, commitbus1_qid_d;
queue_decode_eq_4_16 decode_commit0(.in(commitbus_qid_0),.out(commitbus0_qid_d));
queue_decode_eq_4_16 decode_commit1(.in(commitbus_qid_1),.out(commitbus1_qid_d));

//in alu_one_vector, the entry which has been committed should be 0
wire [`Qsize-1:0]commit_alu_one;

assign commit_alu_one = {16{commitbus_valid_0 & commit0_alu_one}} & commitbus0_qid_d |
                        {16{commitbus_valid_1 & commit1_alu_one}} & commitbus1_qid_d;

wire dec0_alu1 = dec0_alu_one & ~(decbus0_nop|dec0_ex);

wire dec1_alu1 = dec1_alu_one & ~(decbus1_nop|dec1_ex);

wire [`Qsize-1:0] in_en_alu_one = tail_d & {16{dec0_alu1}} | tail_next_d & {16{dec1_alu1}} |
                                  commit_alu_one | brbus_vector | {16{commitbus_ex_0 | commitbus_ex_1}};
//the last is the situation that queue-entry's state is changed to EMPTY by brbus-error or exception or commit

reg [`Qsize-1:0] alu_one_vector;
always @(posedge clock)
begin
  if(reset)
    alu_one_vector <= 1'b0;
  else begin  
  for (i=0;i<`Qsize;i=i+1) 
    begin 
      if (in_en_alu_one[i])
          alu_one_vector[i]<= dec0_alu1 & tail_d[i] | dec1_alu1 & tail_next_d[i];
    end
  end
end

wire [`Qsize-1:0]queue_issue_rdy_tmp;
wire[`Qsize-1:0] alu_one_vector_tmp;
assign alu_one_vector_tmp = alu_one_vector & ~commit_alu_one |
                            {16{dec0_alu1}} & tail_d | {16{dec1_alu1}} & tail_next_d;  
wire [`Qsize-1:0]alu_one_vector_tmp_1;
assign alu_one_vector_tmp_1 = alu_one_vector_tmp & ge_head_vector_tmp; 
wire [`Qsize-1:0]alu_one_vector_tmp_2;
assign alu_one_vector_tmp_2 = alu_one_vector_tmp_1 & queue_issue_rdy_tmp; 
wire [`Qsize-1:0]alu_one_vector_tmp_3;
assign alu_one_vector_tmp_3 = alu_one_vector_tmp & queue_issue_rdy_tmp; 

reg [`Qsize-1:0] alu_one_vector_2_reg;
reg [`Qsize-1:0] alu_one_vector_4_reg;
always @(posedge clock)
begin
  if(reset) begin
    alu_one_vector_2_reg <= 1'b0;
    alu_one_vector_4_reg <= 1'b0;
  end
  else begin   
    alu_one_vector_2_reg <= alu_one_vector_tmp_3;// ge
    alu_one_vector_4_reg <= alu_one_vector_tmp_2;// full
  end
end

wire [`Qsize-1:0] alu_one_vector_2_tmp;
wire [`Qsize-1:0] alu_one_vector_4_tmp;
wire have_alu_one_full;
wire have_alu_one_ge_head;
wire [`Qsize-1:0]alu_one_vector_5;
first_one_16_16 first_one_16_16_6(.in(alu_one_vector_2_reg),.out(alu_one_vector_2_tmp),.has(have_alu_one_full));
first_one_16_16 first_one_16_16_7(.in(alu_one_vector_4_reg),.out(alu_one_vector_4_tmp),.has(have_alu_one_ge_head));

assign alu_one_vector_5 = have_alu_one_ge_head ? alu_one_vector_4_tmp : alu_one_vector_2_tmp; // alu1 operation found

wire [`Qsize-1:0] acc_op_vector_1;
wire [`Qsize-1:0] acc_op_vector_2;
wire [`Qsize-1:0] acc_op_vector_4;
wire [`Qsize-1:0] acc_op_vector_1_tmp;
wire [`Qsize-1:0] acc_op_vector_2_tmp;
wire have_acc_full;
wire have_acc_ge_head;
                         
wire [`Qsize-1:0] acc_op_vector_tmp;
assign acc_op_vector_tmp = {16{dec0_acc_op}} & tail_d |
                           {16{dec1_acc_op}} & tail_next_d;   
assign acc_op_vector_1 = acc_op_vector & s_unissued & ~issued_tmp | acc_op_vector_tmp;
assign acc_op_vector_2 = acc_op_vector_1 & ge_head_vector_tmp;

first_one_16_16 first_one_16_16_acc_0  (.in(acc_op_vector_1),  .out(acc_op_vector_1_tmp),  .has(have_acc_full));
first_one_16_16 first_one_16_16_acc_1  (.in(acc_op_vector_2),  .out(acc_op_vector_2_tmp),  .has(have_acc_ge_head));

wire [`Qsize-1:0]acc_qid_tmp;
assign acc_qid_tmp = have_acc_ge_head ? acc_op_vector_2_tmp : acc_op_vector_1_tmp;

reg [`Qsize-1:0]acc_qid_vector_reg;
always @(posedge clock)
begin
  if(reset)
      acc_qid_vector_reg <= 1'b0;
  else 
      acc_qid_vector_reg <= acc_qid_tmp;
end

wire [`Qsize-1:0]next_empty;

assign next_empty = {16{commitbus_valid_0}} & commitbus0_qid_d | {16{commitbus_valid_1}} & commitbus1_qid_d; 
wire next_div_issued = qissuebus0_valid & div_op_vector[qissuebus0_qid] |
                       qissuebus1_valid & div_op_vector[qissuebus1_qid];

reg  div_issued_reg;
wire div_issued;
assign div_issued = (|(div_op_vector & s_issued & ~next_empty)) | next_div_issued ;
always @(posedge clock)
begin
  if(reset)
     div_issued_reg <= 1'b0;
  else 
     div_issued_reg <= div_issued; 
end
assign acc_op_vector_4 = {16{~div_issued_reg}} & acc_qid_vector_reg; 
wire [`Qsize-1:0] alu_second_1;
wire [`Qsize-1:0] alu_second_1_tmp;
wire [`Qsize-1:0] alu_second_2_full;
wire [`Qsize-1:0] alu_second_2_ge;
wire [`Qsize-1:0] alu_second_3;
wire [`Qsize-1:0] alu_second_4;
wire [`Qsize-1:0] ge_head_and_rdy = ge_head_vector & queue_issue_rdy;

assign alu_second_1_tmp  = alu_one_vector   | acc_op_vector_4;
assign alu_second_1      = alu_second_1_tmp & queue_issue_rdy;
assign alu_second_3      = alu_second_1_tmp & ge_head_and_rdy;
assign alu_second_2_full = alu_second_1 & ~alu_one_vector_2_tmp;  
assign alu_second_2_ge   = alu_second_1 & ~alu_one_vector_4_tmp;  
assign alu_second_4      = alu_second_3 & ~alu_one_vector_4_tmp; 

wire have_alu_second_full_0;
wire have_alu_second_full_1;
wire have_alu_second_ge_head;
wire [`Qsize-1:0] alu_second_2_tmp_full;
wire [`Qsize-1:0] alu_second_2_tmp_ge;
wire [`Qsize-1:0] alu_second_2_tmp;
wire [`Qsize-1:0] alu_second_4_tmp;

first_one_16_16 first_one_16_16_8(.in(alu_second_2_full),.out(alu_second_2_tmp_full),.has(have_alu_second_full_0));
first_one_16_16 first_one_16_16_9(.in(alu_second_2_ge),.out(alu_second_2_tmp_ge),.has(have_alu_second_full_1));
first_one_16_16 first_one_16_16_10(.in(alu_second_4),.out(alu_second_4_tmp),.has(have_alu_second_ge_head));
assign alu_second_2_tmp = have_alu_one_ge_head    ? alu_second_2_tmp_ge : alu_second_2_tmp_full;

/**************alu1 has higher priority than alu2********************/
wire have_mm;
wire have_falu;
wire have_alu1;
wire have_alu2;
assign have_mm = mm_valid_reg & queue_issue_rdy[mm_qid_reg];


assign have_alu1 = have_alu_one_full;
assign have_alu2 = have_alu_one_ge_head ? have_alu_second_full_1 : have_alu_second_full_0;

wire acc_write_ok;
wire alu2full_tmp;
wire alu1_ok;
wire alu2_ok;

assign falu_ok = 1'b0;

assign alu1_ok = have_alu1;
assign alu2full_tmp = alu2full & ~acc_write_ok;
assign alu2_ok      = !alu2full_tmp & have_alu2;

assign mm_ok  = !mmfull & have_mm;

wire issue0_ok = mm_ok | alu2_ok;
wire issue1_ok = alu1_ok | (mm_ok & alu2_ok);  

assign issue_0_d = have_alu_second_ge_head ? (mm_ok ? mm_op_vector_5 : alu_second_4_tmp) : 
                                             (mm_ok ? mm_op_vector_5 : alu_second_2_tmp);
assign issue_1_d = have_alu_second_ge_head ? (alu1_ok ? alu_one_vector_5 : alu_second_4_tmp) : 
                                             (alu1_ok ? alu_one_vector_5 : alu_second_2_tmp);


wire [`Qpower-1:0]issue_0, issue_1;

queue_encode_eq_16_4 encode_issue0(.in(issue_0_d),.out(issue_0));
queue_encode_eq_16_4 encode_issue1(.in(issue_1_d),.out(issue_1));

wire   dec0_jalr;
assign dec0_jalr = decbus0_op == `OP_JALR;

wire [`Qsize-1:0] head_tmp; 

assign head_tmp = (commitbus_valid_0 & commitbus_valid_1) ? (~head_d & ~head_next_d) : 
                   commitbus_valid_0 ? ~head_d : 16'hffff;

wire brbus_valid;
wire [`BRQsize-1:0] brbus_brqid_d;
wire [`Qsize-1:0] brbus_qid_d;
wire [`Qsize-1:0] brbus_qid_tmp;                     

assign brbus_qid_tmp = brbus_valid ? ~brbus_qid_d : 16'hffff;

wire [`Qsize-1:0] q_op_ctc1_tmp;
wire [`Qsize-1:0] q_op_jalr_tmp;
assign q_op_ctc1_tmp = q_op_ctc1 & head_tmp & ~s_empty 
                     | tail_d & {16{dec0_stall}};
assign q_op_jalr_tmp = q_op_jalr & ~s_wb & head_tmp & brbus_qid_tmp & ~s_empty 
                     | tail_d & {16{decbus0_valid & dec0_jalr}};
wire [`Qsize-1:0] q_op_stall;
wire [`Qsize-1:0] q_op_stall_2;
assign q_op_stall   = {q_op_ctc1_tmp[`Qsize-2:0], q_op_ctc1_tmp[`Qsize-1]} | {q_op_jalr_tmp[`Qsize-3:0], q_op_jalr_tmp[`Qsize-1:`Qsize-2]};
assign q_op_stall_2 = q_op_stall & ge_head_vector_tmp; 

wire [`Qsize-1:0] q_op_stall_3;
wire [`Qsize-1:0] q_op_stall_4;
nonzero_low_16 nonzero_stall_0 (.in(q_op_stall_2), .out(q_op_stall_3));
nonzero_low_16 nonzero_stall_1 (.in(q_op_stall), .out(q_op_stall_4));

assign stall_op = ((|q_op_stall_2) ? (q_op_stall_3 | ~ge_head_vector_tmp) : 
                  (q_op_stall_4 & ~ge_head_vector_tmp));  
assign wait_op  = wait_vector & head_tmp & s_unissued |
                  tail_d & {16{dec0_wait}} |
                  tail_next_d & {16{dec1_wait}};
wire [`Qsize-1:0] commit0_eq_qid1;
wire [`Qsize-1:0] commit0_eq_qid2;
wire [`Qsize-1:0] commit0_eq_qid1_h;
wire [`Qsize-1:0] commit0_eq_qid2_h;
wire [`Qsize-1:0] commit1_eq_qid1;
wire [`Qsize-1:0] commit1_eq_qid2;
wire [`Qsize-1:0] commit1_eq_qid1_h;
wire [`Qsize-1:0] commit1_eq_qid2_h;

wire [`Qsize-1:0] wb_tmp;
wire [`Qsize-1:0] forward_tmp;
wire [`Qsize-1:0] write_tmp;

wire[`Qsize-1:0]dec0_canbe_issued;
wire[`Qsize-1:0]dec1_canbe_issued;

assign dec0_canbe_issued = {16{decbus0_valid & ~(decbus0_nop | dec0_ex) & 
                            decbus0_rdy1 & decbus0_rdy2  
                           }} & tail_d;
assign dec1_canbe_issued = {16{decbus1_valid & ~(decbus1_nop | dec1_ex) & 
                            decbus1_rdy1 & decbus1_rdy2
                           }} & tail_next_d;
wire [`Qsize-1:0] next_unissued = ({16{~qissuebus0_valid}}|~issue_0_d) & ({16{~qissuebus1_valid}}|~issue_1_d) &  
                             ~rt_int_v & ~brbus_vector &
                             ~{16{commitbus_ex_0 | commitbus_ex_1}} &
                             (s_unissued | dec0_canbe_issued | dec1_canbe_issued);

assign wb_tmp = queue_wb 
               |{16{resbus3_valid}} & wb3_d 
               |{16{resbus0_valid}} & wb0_d 
               |{16{resbus1_valid}} & wb1_d;  

wire [`Qsize-1:0]forwardbus_alu1_valid;
wire [`Qsize-1:0]forwardbus_alu2_valid;
wire             forwardbus_cp0_valid;
wire [`Qpower-1:0] forwardbus_cp0_qid;

wire [`Qsize-1:0]alu_one_op_fwd0;
wire [`Qsize-1:0]alu_one_op_fwd1;

assign alu_one_op_fwd0 = alu_one_vector & issue_0_d;

assign forwardbus_alu1_valid = {16{qissuebus0_valid}} & alu_one_op_fwd0;
assign alu_one_op_fwd1 = alu_one_vector & issue_1_d;
assign forwardbus_alu2_valid = {16{qissuebus1_valid}} & alu_one_op_fwd1;
assign forward_tmp = forwardbus_alu1_valid | forwardbus_alu2_valid;  

assign forwardbus_cp0_valid = cp0_forward_bus_i[4];
assign forwardbus_cp0_qid   = cp0_forward_bus_i[`Qpower-1:0];

wire [`Qsize-1:0] forwardbus_cp0_qid_d;
queue_decode_eq_4_16 decode_fwd3(.in(forwardbus_cp0_qid),.out(forwardbus_cp0_qid_d));

assign write_tmp = wb_tmp | forward_tmp;

wire [`Qsize-1:0] queue_rdy1_tmp;
wire [`Qsize-1:0] queue_rdy2_tmp;
assign queue_rdy1_tmp = queue_rdy1;  
assign queue_rdy2_tmp = queue_rdy2;  

assign queue_issue_rdy_tmp[0]=(((queue_rdy1_tmp[0] | write_tmp[queue_qid1_tmp[0]] |
                                 mm_op_vector[0] & lw_op_vector[queue_qid1_tmp[0]] |   
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[0]]) &
                                (queue_rdy2_tmp[0] | write_tmp[queue_qid2_tmp[0]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[0]])  
                                |tail_d[0]|tail_next_d[0]) & ~(stall_op[0]|wait_op[0]) |
                                 head_d_tmp[0]) & next_unissued[0];
 
assign queue_issue_rdy_tmp[1]=(((queue_rdy1_tmp[1] | write_tmp[queue_qid1_tmp[1]] |
                                 mm_op_vector[1] & lw_op_vector[queue_qid1_tmp[1]] |
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[1]]) &
                                (queue_rdy2_tmp[1] | write_tmp[queue_qid2_tmp[1]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[1]])  
                                |tail_d[1]|tail_next_d[1]) & ~(stall_op[1]|wait_op[1]) |
                                 head_d_tmp[1]) & next_unissued[1];
 
assign queue_issue_rdy_tmp[2]=(((queue_rdy1_tmp[2] | write_tmp[queue_qid1_tmp[2]] |
                                 mm_op_vector[2] & lw_op_vector[queue_qid1_tmp[2]] | 
                                 forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid1_tmp[2]]) &
                                (queue_rdy2_tmp[2] | write_tmp[queue_qid2_tmp[2]] |
                                 forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid2_tmp[2]])  
                                |tail_d[2] | tail_next_d[2]) & ~(stall_op[2]|wait_op[2]) |
                                 head_d_tmp[2]) & next_unissued[2];
            
assign queue_issue_rdy_tmp[3]=(((queue_rdy1_tmp[3] | write_tmp[queue_qid1_tmp[3]] |
                                 mm_op_vector[3] & lw_op_vector[queue_qid1_tmp[3]] |
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[3]]) &
                                (queue_rdy2_tmp[3] | write_tmp[queue_qid2_tmp[3]] |
                                forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[3]])  
                                |tail_d[3] | tail_next_d[3]) & ~(stall_op[3]|wait_op[3]) |
                                 head_d_tmp[3]) & next_unissued[3];
 
assign queue_issue_rdy_tmp[4]=(((queue_rdy1_tmp[4] | write_tmp[queue_qid1_tmp[4]] | 
                                 mm_op_vector[4] & lw_op_vector[queue_qid1_tmp[4]] |
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[4]]) &
                                (queue_rdy2_tmp[4] | write_tmp[queue_qid2_tmp[4]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[4]])  
                                |tail_d[4] | tail_next_d[4]) & ~(stall_op[4]|wait_op[4]) |
                                 head_d_tmp[4]) & next_unissued[4];
 
assign queue_issue_rdy_tmp[5]=(((queue_rdy1_tmp[5] | write_tmp[queue_qid1_tmp[5]] | 
                                 mm_op_vector[5] & lw_op_vector[queue_qid1_tmp[5]] |  
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[5]]) &
                                (queue_rdy2_tmp[5] | write_tmp[queue_qid2_tmp[5]] |
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[5]])  
                               |tail_d[5]|tail_next_d[5]) & ~(stall_op[5]|wait_op[5]) |
                                head_d_tmp[5]) & next_unissued[5];
            
assign queue_issue_rdy_tmp[6]=(((queue_rdy1_tmp[6] | write_tmp[queue_qid1_tmp[6]] | 
                                 mm_op_vector[6] & lw_op_vector[queue_qid1_tmp[6]] |  
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[6]]) &
                                (queue_rdy2_tmp[6] | write_tmp[queue_qid2_tmp[6]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[6]])  
                                |tail_d[6]|tail_next_d[6]) & ~(stall_op[6]|wait_op[6]) |
                                 head_d_tmp[6]) & next_unissued[6];
 
assign queue_issue_rdy_tmp[7]=(((queue_rdy1_tmp[7] | write_tmp[queue_qid1_tmp[7]] |
                                 mm_op_vector[7] & lw_op_vector[queue_qid1_tmp[7]] |  
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[7]]) &
                                (queue_rdy2_tmp[7] | write_tmp[queue_qid2_tmp[7]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[7]])  
                                |tail_d[7]|tail_next_d[7]) & ~(stall_op[7]|wait_op[7]) |
                                 head_d_tmp[7]) & next_unissued[7];
 
assign queue_issue_rdy_tmp[8]=(((queue_rdy1_tmp[8] | write_tmp[queue_qid1_tmp[8]] | 
                                 mm_op_vector[8] & lw_op_vector[queue_qid1_tmp[8]] |  
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[8]]) &
                                (queue_rdy2_tmp[8] | write_tmp[queue_qid2_tmp[8]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[8]])  
                                |tail_d[8]|tail_next_d[8]) & ~(stall_op[8]|wait_op[8]) |
                                 head_d_tmp[8]) & next_unissued[8];
            
assign queue_issue_rdy_tmp[9]=(((queue_rdy1_tmp[9] | write_tmp[queue_qid1_tmp[9]] | 
                                 mm_op_vector[9] & lw_op_vector[queue_qid1_tmp[9]] |  
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[9]]) &
                                (queue_rdy2_tmp[9] | write_tmp[queue_qid2_tmp[9]] | 
                                 forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[9]])  
                                |tail_d[9]|tail_next_d[9]) & ~(stall_op[9]|wait_op[9]) |
                                 head_d_tmp[9]) & next_unissued[9];
 
assign queue_issue_rdy_tmp[10]=(((queue_rdy1_tmp[10] | write_tmp[queue_qid1_tmp[10]] |
                                  mm_op_vector[10] & lw_op_vector[queue_qid1_tmp[10]] |  
                                  forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[10]]) &
                                 (queue_rdy2_tmp[10] | write_tmp[queue_qid2_tmp[10]] | 
                                  forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[10]])  
                                |tail_d[10]|tail_next_d[10])&~(stall_op[10]|wait_op[10]) |
                                 head_d_tmp[10]) & next_unissued[10];
 
assign queue_issue_rdy_tmp[11]=(((queue_rdy1_tmp[11] | write_tmp[queue_qid1_tmp[11]] | 
                                  mm_op_vector[11] & lw_op_vector[queue_qid1_tmp[11]] | 
                                  forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[11]]) &
                                 (queue_rdy2_tmp[11] | write_tmp[queue_qid2_tmp[11]] | 
                                  forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[11]])  
                                |tail_d[11]|tail_next_d[11])&~(stall_op[11]|wait_op[11]) |
                                 head_d_tmp[11]) & next_unissued[11];
            
assign queue_issue_rdy_tmp[12]=(((queue_rdy1_tmp[12] | write_tmp[queue_qid1_tmp[12]] | 
                                  mm_op_vector[12] & lw_op_vector[queue_qid1_tmp[12]] |  
                                  forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid1_tmp[12]]) &
                                 (queue_rdy2_tmp[12] | write_tmp[queue_qid2_tmp[12]] | 
                                  forwardbus_cp0_valid & forwardbus_cp0_qid_d[queue_qid2_tmp[12]])  
                                |tail_d[12]|tail_next_d[12])&~(stall_op[12]|wait_op[12]) |
                                 head_d_tmp[12]) & next_unissued[12];
 
assign queue_issue_rdy_tmp[13]=(((queue_rdy1_tmp[13] | write_tmp[queue_qid1_tmp[13]] | 
                                  mm_op_vector[13] & lw_op_vector[queue_qid1_tmp[13]] |  
                                  forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid1_tmp[13]]) &
                                 (queue_rdy2_tmp[13] | write_tmp[queue_qid2_tmp[13]] | 
                                  forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid2_tmp[13]])  
                                |tail_d[13]|tail_next_d[13])&~(stall_op[13]|wait_op[13]) |
                                 head_d_tmp[13]) & next_unissued[13];
 
assign queue_issue_rdy_tmp[14]=(((queue_rdy1_tmp[14] | write_tmp[queue_qid1_tmp[14]] |
                                  mm_op_vector[14] & lw_op_vector[queue_qid1_tmp[14]] |  
                                  forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid1_tmp[14]]) &
                                 (queue_rdy2_tmp[14] | write_tmp[queue_qid2_tmp[14]] |
                                  forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid2_tmp[14]])  
                                |tail_d[14]|tail_next_d[14])&~(stall_op[14]|wait_op[14]) |
                                 head_d_tmp[14]) & next_unissued[14];
            
assign queue_issue_rdy_tmp[15]=(((queue_rdy1_tmp[15] | write_tmp[queue_qid1_tmp[15]] |
                                  mm_op_vector[15] & lw_op_vector[queue_qid1_tmp[15]] |  
                                  forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid1_tmp[15]]) &
                                 (queue_rdy2_tmp[15] | write_tmp[queue_qid2_tmp[15]] |
                                  forwardbus_cp0_valid&forwardbus_cp0_qid_d[queue_qid2_tmp[15]])  
                                |tail_d[15]|tail_next_d[15])&~(stall_op[15]|wait_op[15])|
                                 head_d_tmp[15]) & next_unissued[15];

always @(posedge  clock) begin
  if(reset)
     queue_issue_rdy <= 1'b0;
  else 
     queue_issue_rdy <= queue_issue_rdy_tmp;
end
/*--------------------------------Second, issue part----------------------------------------------*/

/*qissuebus0 part*/
assign qissuebus0_rs = mm_ok   ? 2'b00 : 
                       falu_ok ? 2'b11 : 2'b10;      
assign qissuebus0_qid    = issue_0;

assign qissuebus0_brqid = {`BRQpower{issue_0_d[0] }}& queue_brqid_0  | {`BRQpower{issue_0_d[1] }} & queue_brqid_1 |
                          {`BRQpower{issue_0_d[2] }}& queue_brqid_2  | {`BRQpower{issue_0_d[3] }} & queue_brqid_3 |
                          {`BRQpower{issue_0_d[4] }}& queue_brqid_4  | {`BRQpower{issue_0_d[5] }} & queue_brqid_5 |
                          {`BRQpower{issue_0_d[6] }}& queue_brqid_6  | {`BRQpower{issue_0_d[7] }} & queue_brqid_7 
                         |{`BRQpower{issue_0_d[8] }}& queue_brqid_8  | {`BRQpower{issue_0_d[9] }} & queue_brqid_9 |
                          {`BRQpower{issue_0_d[10]}}& queue_brqid_10 | {`BRQpower{issue_0_d[11]}} & queue_brqid_11|
                          {`BRQpower{issue_0_d[12]}}& queue_brqid_12 | {`BRQpower{issue_0_d[13]}} & queue_brqid_13| 
                          {`BRQpower{issue_0_d[14]}}& queue_brqid_14 | {`BRQpower{issue_0_d[15]}} & queue_brqid_15
                          ;

assign qissuebus0_op = {8{issue_0_d[0] }}& queue_op_0  | {8{issue_0_d[1] }} & queue_op_1 |
                       {8{issue_0_d[2] }}& queue_op_2  | {8{issue_0_d[3] }} & queue_op_3 |
                       {8{issue_0_d[4] }}& queue_op_4  | {8{issue_0_d[5] }} & queue_op_5 |
                       {8{issue_0_d[6] }}& queue_op_6  | {8{issue_0_d[7] }} & queue_op_7 
                      |{8{issue_0_d[8] }}& queue_op_8  | {8{issue_0_d[9] }} & queue_op_9 |
                       {8{issue_0_d[10]}}& queue_op_10 | {8{issue_0_d[11]}} & queue_op_11|
                       {8{issue_0_d[12]}}& queue_op_12 | {8{issue_0_d[13]}} & queue_op_13| 
                       {8{issue_0_d[14]}}& queue_op_14 | {8{issue_0_d[15]}} & queue_op_15
                       ;

assign qissuebus0_src1 = {8{issue_0_d[0] }}& queue_src1_0  | {8{issue_0_d[1] }}& queue_src1_1 | 
                         {8{issue_0_d[2] }}& queue_src1_2  | {8{issue_0_d[3] }}& queue_src1_3 | 
                         {8{issue_0_d[4] }}& queue_src1_4  | {8{issue_0_d[5] }}& queue_src1_5 |
                         {8{issue_0_d[6] }}& queue_src1_6  | {8{issue_0_d[7] }}& queue_src1_7 
                        |{8{issue_0_d[8] }}& queue_src1_8  | {8{issue_0_d[9] }}& queue_src1_9 |
                         {8{issue_0_d[10]}}& queue_src1_10 | {8{issue_0_d[11]}}& queue_src1_11|
                         {8{issue_0_d[12]}}& queue_src1_12 | {8{issue_0_d[13]}}& queue_src1_13|
                         {8{issue_0_d[14]}}& queue_src1_14 | {8{issue_0_d[15]}}& queue_src1_15
                         ;

assign qissuebus0_src2 = {8{issue_0_d[0] }} & queue_src2_0  | {8{issue_0_d[1] }} & queue_src2_1  | 
                         {8{issue_0_d[2] }} & queue_src2_2  | {8{issue_0_d[3] }} & queue_src2_3  | 
                         {8{issue_0_d[4] }} & queue_src2_4  | {8{issue_0_d[5] }} & queue_src2_5  |
                         {8{issue_0_d[6] }} & queue_src2_6  | {8{issue_0_d[7] }} & queue_src2_7  
                        |{8{issue_0_d[8] }} & queue_src2_8  | {8{issue_0_d[9] }} & queue_src2_9  | 
                         {8{issue_0_d[10]}} & queue_src2_10 | {8{issue_0_d[11]}} & queue_src2_11 |
                         {8{issue_0_d[12]}} & queue_src2_12 | {8{issue_0_d[13]}} & queue_src2_13 |
                         {8{issue_0_d[14]}} & queue_src2_14 | {8{issue_0_d[15]}} & queue_src2_15
                         ;

assign qissuebus0_dest = {8{issue_0_d[0] }} & queue_dest_0  | {8{issue_0_d[1] }} & queue_dest_1  | 
                         {8{issue_0_d[2] }} & queue_dest_2  | {8{issue_0_d[3] }} & queue_dest_3  | 
                         {8{issue_0_d[4] }} & queue_dest_4  | {8{issue_0_d[5] }} & queue_dest_5  |
                         {8{issue_0_d[6] }} & queue_dest_6  | {8{issue_0_d[7] }} & queue_dest_7  
                        |{8{issue_0_d[8] }} & queue_dest_8  | {8{issue_0_d[9] }} & queue_dest_9  | 
                         {8{issue_0_d[10]}} & queue_dest_10 | {8{issue_0_d[11]}} & queue_dest_11 |
                         {8{issue_0_d[12]}} & queue_dest_12 | {8{issue_0_d[13]}} & queue_dest_13 |
                         {8{issue_0_d[14]}} & queue_dest_14 | {8{issue_0_d[15]}} & queue_dest_15
                         ;

assign qissuebus0_offset = {16{issue_0_d[0] }} & queue_imm_low_0 |
                           {16{issue_0_d[1] }} & queue_imm_low_1 | 
                           {16{issue_0_d[2] }} & queue_imm_low_2 |
                           {16{issue_0_d[3] }} & queue_imm_low_3 | 
                           {16{issue_0_d[4] }} & queue_imm_low_4 |
                           {16{issue_0_d[5] }} & queue_imm_low_5 |
                           {16{issue_0_d[6] }} & queue_imm_low_6 | 
                           {16{issue_0_d[7] }} & queue_imm_low_7 |
                           {16{issue_0_d[8] }} & queue_imm_low_8 |
                           {16{issue_0_d[9] }} & queue_imm_low_9 | 
                           {16{issue_0_d[10]}} & queue_imm_low_10 |
                           {16{issue_0_d[11]}} & queue_imm_low_11 |
                           {16{issue_0_d[12]}} & queue_imm_low_12 |
                           {16{issue_0_d[13]}} & queue_imm_low_13 |
                           {16{issue_0_d[14]}} & queue_imm_low_14 |
                           {16{issue_0_d[15]}} & queue_imm_low_15 
                           ;

assign qissuebus0_ac = queue_ac[issue_0]; 

assign qissuebus0_valid = issue0_ok;

assign qissuebus0_qid1 = {4{issue_0_d[0] }} & queue_qid1_0  | {4{issue_0_d[1] }} & queue_qid1_1  | 
                         {4{issue_0_d[2] }} & queue_qid1_2  | {4{issue_0_d[3] }} & queue_qid1_3  | 
                         {4{issue_0_d[4] }} & queue_qid1_4  | {4{issue_0_d[5] }} & queue_qid1_5  |
                         {4{issue_0_d[6] }} & queue_qid1_6  | {4{issue_0_d[7] }} & queue_qid1_7  |
                         {4{issue_0_d[8] }} & queue_qid1_8  | {4{issue_0_d[9] }} & queue_qid1_9  | 
                         {4{issue_0_d[10]}} & queue_qid1_10 | {4{issue_0_d[11]}} & queue_qid1_11 |
                         {4{issue_0_d[12]}} & queue_qid1_12 | {4{issue_0_d[13]}} & queue_qid1_13 |
                         {4{issue_0_d[14]}} & queue_qid1_14 | {4{issue_0_d[15]}} & queue_qid1_15 ;

assign qissuebus0_qid2 = {4{issue_0_d[0]}}  & queue_qid2_0  | {4{issue_0_d[1] }} & queue_qid2_1  | 
                         {4{issue_0_d[2] }} & queue_qid2_2  | {4{issue_0_d[3] }} & queue_qid2_3  | 
                         {4{issue_0_d[4] }} & queue_qid2_4  | {4{issue_0_d[5] }} & queue_qid2_5  |
                         {4{issue_0_d[6] }} & queue_qid2_6  | {4{issue_0_d[7] }} & queue_qid2_7  |
                         {4{issue_0_d[8] }} & queue_qid2_8  | {4{issue_0_d[9] }} & queue_qid2_9  | 
                         {4{issue_0_d[10]}} & queue_qid2_10 | {4{issue_0_d[11]}} & queue_qid2_11 |
                         {4{issue_0_d[12]}} & queue_qid2_12 | {4{issue_0_d[13]}} & queue_qid2_13 |
                         {4{issue_0_d[14]}} & queue_qid2_14 | {4{issue_0_d[15]}} & queue_qid2_15 ;

assign resqid1_0   = qissuebus0_qid1; //src1
assign resqid2_0   = qissuebus0_qid2; //src2

wire [31:0] qissuebus0_res1_tmp;

assign qissuebus0_res1_tmp = {32{resbus1_valid & (resbus1_qid==resqid1_0)}} &  resbus1_value[31:0] |

                             {32{resbus3_valid & (resbus3_qid==resqid1_0)}} &  alu_res |
                             {32{resbus0_valid & (resbus0_qid==resqid1_0)}} &  resbus0_value[31:0];

wire qissuebus0_res1_valid;

assign qissuebus0_res1_valid = resbus1_valid & (resbus1_qid==resqid1_0) | 
                               resbus3_valid & (resbus3_qid==resqid1_0) |
                               resbus0_valid & (resbus0_qid==resqid1_0);
assign qissuebus0_res1 = qissuebus0_res1_valid ? qissuebus0_res1_tmp : 
                         {queue_imm_hi[resqid1_0], queue_imm_low[resqid1_0]};

wire [31:0] qissuebus0_res2_tmp;

assign qissuebus0_res2_tmp = {32{resbus1_valid & (resbus1_qid==resqid2_0)}} &  resbus1_value[31:0] |
                             {32{resbus3_valid & (resbus3_qid==resqid2_0)}} &  alu_res |
                             {32{resbus0_valid & (resbus0_qid==resqid2_0)}} &  resbus0_value[31:0];

wire qissuebus0_res2_valid;

assign qissuebus0_res2_valid = resbus1_valid & (resbus1_qid==resqid2_0) | 
                               resbus3_valid & (resbus3_qid==resqid2_0) |
                               resbus0_valid & (resbus0_qid==resqid2_0);

assign qissuebus0_res2 = qissuebus0_res2_valid ? qissuebus0_res2_tmp : 
                         {queue_imm_hi[resqid2_0], queue_imm_low[resqid2_0]};

assign qissuebus0_src1_rdy = ~lw_op_vector[qissuebus0_qid1] | queue_wb[qissuebus0_qid1] | qissuebus0_res1_valid;

assign qissuebus0_rdy1 = issue_0_d[0]  & queue_rdy1[0 ] | issue_0_d[1]  & queue_rdy1[1 ] |
                         issue_0_d[2]  & queue_rdy1[2 ] | issue_0_d[3]  & queue_rdy1[3 ] |
                         issue_0_d[4]  & queue_rdy1[4 ] | issue_0_d[5]  & queue_rdy1[5 ] |
                         issue_0_d[6]  & queue_rdy1[6 ] | issue_0_d[7]  & queue_rdy1[7 ] 
                        |issue_0_d[8]  & queue_rdy1[8 ] | issue_0_d[9]  & queue_rdy1[9 ] |
                         issue_0_d[10] & queue_rdy1[10] | issue_0_d[11] & queue_rdy1[11] |
                         issue_0_d[12] & queue_rdy1[12] | issue_0_d[13] & queue_rdy1[13] |
                         issue_0_d[14] & queue_rdy1[14] | issue_0_d[15] & queue_rdy1[15] 
                        |(head==issue_0);

assign qissuebus0_rdy2 = issue_0_d[0]  & queue_rdy2[0 ] | issue_0_d[1]  & queue_rdy2[1 ] |
                         issue_0_d[2]  & queue_rdy2[2 ] | issue_0_d[3]  & queue_rdy2[3 ] |
                         issue_0_d[4]  & queue_rdy2[4 ] | issue_0_d[5]  & queue_rdy2[5 ] |
                         issue_0_d[6]  & queue_rdy2[6 ] | issue_0_d[7]  & queue_rdy2[7 ] 
                        |issue_0_d[8]  & queue_rdy2[8 ] | issue_0_d[9]  & queue_rdy2[9 ] |
                         issue_0_d[10] & queue_rdy2[10] | issue_0_d[11] & queue_rdy2[11] |
                         issue_0_d[12] & queue_rdy2[12] | issue_0_d[13] & queue_rdy2[13] |
                         issue_0_d[14] & queue_rdy2[14] | issue_0_d[15] & queue_rdy2[15] 
                        |(head==issue_0);


/*qissuebus1 part*/
assign qissuebus1_rs = alu1_ok ? 2'b01 : (mm_ok & falu_ok) ? 2'b11 : 2'b10;
assign qissuebus1_qid    = issue_1;

assign qissuebus1_brqid = {`BRQpower{issue_1_d[0] }}& queue_brqid_0  | {`BRQpower{issue_1_d[1] }} & queue_brqid_1 |
                          {`BRQpower{issue_1_d[2] }}& queue_brqid_2  | {`BRQpower{issue_1_d[3] }} & queue_brqid_3 |
                          {`BRQpower{issue_1_d[4] }}& queue_brqid_4  | {`BRQpower{issue_1_d[5] }} & queue_brqid_5 |
                          {`BRQpower{issue_1_d[6] }}& queue_brqid_6  | {`BRQpower{issue_1_d[7] }} & queue_brqid_7 |
                          {`BRQpower{issue_1_d[8] }}& queue_brqid_8  | {`BRQpower{issue_1_d[9] }} & queue_brqid_9 |
                          {`BRQpower{issue_1_d[10]}}& queue_brqid_10 | {`BRQpower{issue_1_d[11]}} & queue_brqid_11|
                          {`BRQpower{issue_1_d[12]}}& queue_brqid_12 | {`BRQpower{issue_1_d[13]}} & queue_brqid_13| 
                          {`BRQpower{issue_1_d[14]}}& queue_brqid_14 | {`BRQpower{issue_1_d[15]}} & queue_brqid_15;

assign qissuebus1_op = {8{issue_1_d[0] }}& queue_op_0  | {8{issue_1_d[1] }} & queue_op_1 |
                       {8{issue_1_d[2] }}& queue_op_2  | {8{issue_1_d[3] }} & queue_op_3 |
                       {8{issue_1_d[4] }}& queue_op_4  | {8{issue_1_d[5] }} & queue_op_5 |
                       {8{issue_1_d[6] }}& queue_op_6  | {8{issue_1_d[7] }} & queue_op_7 |
                       {8{issue_1_d[8] }}& queue_op_8  | {8{issue_1_d[9] }} & queue_op_9 |
                       {8{issue_1_d[10]}}& queue_op_10 | {8{issue_1_d[11]}} & queue_op_11|
                       {8{issue_1_d[12]}}& queue_op_12 | {8{issue_1_d[13]}} & queue_op_13| 
                       {8{issue_1_d[14]}}& queue_op_14 | {8{issue_1_d[15]}} & queue_op_15;

assign qissuebus1_src1 = {8{issue_1_d[0] }}& queue_src1_0  | {8{issue_1_d[1] }}& queue_src1_1 | 
                         {8{issue_1_d[2] }}& queue_src1_2  | {8{issue_1_d[3] }}& queue_src1_3 | 
                         {8{issue_1_d[4] }}& queue_src1_4  | {8{issue_1_d[5] }}& queue_src1_5 |
                         {8{issue_1_d[6] }}& queue_src1_6  | {8{issue_1_d[7] }}& queue_src1_7 |
                         {8{issue_1_d[8] }}& queue_src1_8  | {8{issue_1_d[9] }}& queue_src1_9 |
                         {8{issue_1_d[10]}}& queue_src1_10 | {8{issue_1_d[11]}}& queue_src1_11|
                         {8{issue_1_d[12]}}& queue_src1_12 | {8{issue_1_d[13]}}& queue_src1_13|
                         {8{issue_1_d[14]}}& queue_src1_14 | {8{issue_1_d[15]}}& queue_src1_15;

assign qissuebus1_src2 = {8{issue_1_d[0] }} & queue_src2_0  | {8{issue_1_d[1] }} & queue_src2_1  | 
                         {8{issue_1_d[2] }} & queue_src2_2  | {8{issue_1_d[3] }} & queue_src2_3  | 
                         {8{issue_1_d[4] }} & queue_src2_4  | {8{issue_1_d[5] }} & queue_src2_5  |
                         {8{issue_1_d[6] }} & queue_src2_6  | {8{issue_1_d[7] }} & queue_src2_7  |
                         {8{issue_1_d[8] }} & queue_src2_8  | {8{issue_1_d[9] }} & queue_src2_9  | 
                         {8{issue_1_d[10]}} & queue_src2_10 | {8{issue_1_d[11]}} & queue_src2_11 |
                         {8{issue_1_d[12]}} & queue_src2_12 | {8{issue_1_d[13]}} & queue_src2_13 |
                         {8{issue_1_d[14]}} & queue_src2_14 | {8{issue_1_d[15]}} & queue_src2_15;

assign qissuebus1_dest = {8{issue_1_d[0] }} & queue_dest_0  | {8{issue_1_d[1] }} & queue_dest_1  | 
                         {8{issue_1_d[2] }} & queue_dest_2  | {8{issue_1_d[3] }} & queue_dest_3  | 
                         {8{issue_1_d[4] }} & queue_dest_4  | {8{issue_1_d[5] }} & queue_dest_5  |
                         {8{issue_1_d[6] }} & queue_dest_6  | {8{issue_1_d[7] }} & queue_dest_7  |
                         {8{issue_1_d[8] }} & queue_dest_8  | {8{issue_1_d[9] }} & queue_dest_9  | 
                         {8{issue_1_d[10]}} & queue_dest_10 | {8{issue_1_d[11]}} & queue_dest_11 |
                         {8{issue_1_d[12]}} & queue_dest_12 | {8{issue_1_d[13]}} & queue_dest_13 |
                         {8{issue_1_d[14]}} & queue_dest_14 | {8{issue_1_d[15]}} & queue_dest_15;

assign qissuebus1_ac = queue_ac[issue_1]; 

assign qissuebus1_valid = issue1_ok;        

assign qissuebus1_qid1 = {4{issue_1_d[0]}}  & queue_qid1_0  | {4{issue_1_d[1]}}  & queue_qid1_1  | 
                         {4{issue_1_d[2] }} & queue_qid1_2  | {4{issue_1_d[3] }} & queue_qid1_3  | 
                         {4{issue_1_d[4] }} & queue_qid1_4  | {4{issue_1_d[5] }} & queue_qid1_5  |
                         {4{issue_1_d[6] }} & queue_qid1_6  | {4{issue_1_d[7] }} & queue_qid1_7  |
                         {4{issue_1_d[8] }} & queue_qid1_8  | {4{issue_1_d[9] }} & queue_qid1_9  | 
                         {4{issue_1_d[10]}} & queue_qid1_10 | {4{issue_1_d[11]}} & queue_qid1_11 |
                         {4{issue_1_d[12]}} & queue_qid1_12 | {4{issue_1_d[13]}} & queue_qid1_13 |
                         {4{issue_1_d[14]}} & queue_qid1_14 | {4{issue_1_d[15]}} & queue_qid1_15;

assign qissuebus1_qid2 = {4{issue_1_d[0]}} & queue_qid2_0  | {4{issue_1_d[1]}}  & queue_qid2_1  | 
                         {4{issue_1_d[2] }} & queue_qid2_2  | {4{issue_1_d[3] }} & queue_qid2_3  | 
                         {4{issue_1_d[4] }} & queue_qid2_4  | {4{issue_1_d[5] }} & queue_qid2_5  |
                         {4{issue_1_d[6] }} & queue_qid2_6  | {4{issue_1_d[7] }} & queue_qid2_7  |
                         {4{issue_1_d[8] }} & queue_qid2_8  | {4{issue_1_d[9] }} & queue_qid2_9  | 
                         {4{issue_1_d[10]}} & queue_qid2_10 | {4{issue_1_d[11]}} & queue_qid2_11 |
                         {4{issue_1_d[12]}} & queue_qid2_12 | {4{issue_1_d[13]}} & queue_qid2_13 |
                         {4{issue_1_d[14]}} & queue_qid2_14 | {4{issue_1_d[15]}} & queue_qid2_15;

assign resqid1_1 = qissuebus1_qid1; //src1
assign resqid2_1 = qissuebus1_qid2; //src2

wire [31:0] qissuebus1_res1_tmp;

assign qissuebus1_res1_tmp = {32{resbus1_valid & (resbus1_qid==resqid1_1)}} &  resbus1_value[31:0] |
                             {32{resbus3_valid & (resbus3_qid==resqid1_1)}} &  alu_res |
                             {32{resbus0_valid & (resbus0_qid==resqid1_1)}} &  resbus0_value[31:0];

wire qissuebus1_res1_valid;
assign qissuebus1_res1_valid = resbus1_valid & (resbus1_qid==resqid1_1) | 
                               resbus3_valid & (resbus3_qid==resqid1_1) |
                               resbus0_valid & (resbus0_qid==resqid1_1);
assign qissuebus1_res1 = qissuebus1_res1_valid ? qissuebus1_res1_tmp : 
                         {queue_imm_hi[resqid1_1], queue_imm_low[resqid1_1]};


wire [31:0] qissuebus1_res2_tmp;
assign qissuebus1_res2_tmp = {32{resbus1_valid & (resbus1_qid==resqid2_1)}} &  resbus1_value[31:0] |
                             {32{resbus3_valid & (resbus3_qid==resqid2_1)}} &  alu_res |
                             {32{resbus0_valid & (resbus0_qid==resqid2_1)}} &  resbus0_value[31:0];
wire qissuebus1_res2_valid;

assign qissuebus1_res2_valid = resbus1_valid & (resbus1_qid==resqid2_1) | 
                               resbus3_valid & (resbus3_qid==resqid2_1) |
                               resbus0_valid & (resbus0_qid==resqid2_1);

assign qissuebus1_res2 = qissuebus1_res2_valid ? qissuebus1_res2_tmp : 
                         {queue_imm_hi[resqid2_1], queue_imm_low[resqid2_1]};

assign qissuebus1_rdy1 = issue_1_d[0]  & queue_rdy1[0 ] | issue_1_d[1]  & queue_rdy1[1 ] |
                         issue_1_d[2]  & queue_rdy1[2 ] | issue_1_d[3]  & queue_rdy1[3 ] |
                         issue_1_d[4]  & queue_rdy1[4 ] | issue_1_d[5]  & queue_rdy1[5 ] |
                         issue_1_d[6]  & queue_rdy1[6 ] | issue_1_d[7]  & queue_rdy1[7 ] |
                         issue_1_d[8]  & queue_rdy1[8 ] | issue_1_d[9]  & queue_rdy1[9 ] |
                         issue_1_d[10] & queue_rdy1[10] | issue_1_d[11] & queue_rdy1[11] |
                         issue_1_d[12] & queue_rdy1[12] | issue_1_d[13] & queue_rdy1[13] |
                         issue_1_d[14] & queue_rdy1[14] | issue_1_d[15] & queue_rdy1[15] | (head==issue_1);

assign qissuebus1_rdy2 = issue_1_d[0]  & queue_rdy2[0 ] | issue_1_d[1]  & queue_rdy2[1 ] |
                         issue_1_d[2]  & queue_rdy2[2 ] | issue_1_d[3]  & queue_rdy2[3 ] |
                         issue_1_d[4]  & queue_rdy2[4 ] | issue_1_d[5]  & queue_rdy2[5 ] |
                         issue_1_d[6]  & queue_rdy2[6 ] | issue_1_d[7]  & queue_rdy2[7 ] |
                         issue_1_d[8]  & queue_rdy2[8 ] | issue_1_d[9]  & queue_rdy2[9 ] |
                         issue_1_d[10] & queue_rdy2[10] | issue_1_d[11] & queue_rdy2[11] |
                         issue_1_d[12] & queue_rdy2[12] | issue_1_d[13] & queue_rdy2[13] |
                         issue_1_d[14] & queue_rdy2[14] | issue_1_d[15] & queue_rdy2[15] | (head==issue_1);

/*************************************************************************************
                                    about fpq
*************************************************************************************/
assign qissuebus0[117]     = 1'b0;
assign qissuebus0[116]     = 1'b0;
assign qissuebus0[115]     = 1'b0;
assign qissuebus0[114:110] = 5'b0;
assign qissuebus0[106:105] = qissuebus0_ac;
assign qissuebus0[104:103] = qissuebus0_rs;
assign qissuebus0[102]     = qissuebus0_rdy1;
assign qissuebus0[101]     = qissuebus0_rdy2;
assign qissuebus0[100]     = qissuebus0_valid;
assign qissuebus0[109:107] = qissuebus0_brqid;
assign qissuebus0[99:96]   = qissuebus0_qid;
assign qissuebus0[95:88]   = qissuebus0_op;
assign qissuebus0[87:80]   = qissuebus0_src1; //reg number
assign qissuebus0[79:72]   = qissuebus0_src2; //reg number
assign qissuebus0[71:64]   = qissuebus0_dest; //reg number
assign qissuebus0[63:32]   = qissuebus0_res1; //source operand
assign qissuebus0[31:0]    = qissuebus0_res2; //source operand

assign qissuebus1[117]     = 1'b0;
assign qissuebus1[116]     = 1'b0;
assign qissuebus1[115]     = 1'b0;
assign qissuebus1[114:110] = 5'b0;
assign qissuebus1[109:107] = qissuebus1_brqid;
assign qissuebus1[106:105] = qissuebus1_ac;
assign qissuebus1[104:103] = qissuebus1_rs;
assign qissuebus1[102]     = qissuebus1_rdy1; 
assign qissuebus1[101]     = qissuebus1_rdy2;
assign qissuebus1[100]     = qissuebus1_valid;
assign qissuebus1[99:96]   = qissuebus1_qid;
assign qissuebus1[95:88]   = qissuebus1_op;
assign qissuebus1[87:80]   = qissuebus1_src1; //reg number
assign qissuebus1[79:72]   = qissuebus1_src2; //reg number
assign qissuebus1[71:64]   = qissuebus1_dest; //reg number
assign qissuebus1[63:32]   = qissuebus1_res1; //source operand
assign qissuebus1[31:0]    = qissuebus1_res2;  //source operand
/***********************************brbus part**************************************************/
wire [`BRQsize-1 : 0] in_en_brq_rashead;
wire [`BRQsize-1 : 0] in_en_brq_valid;
wire [`BRQsize-1 : 0] in_en_brq_status;
wire [`BRQsize-1 : 0] in_en_brq_bdrdy;
wire [`BRQsize-1 : 0] in_en_brq_wb;
wire [`BRQsize-1 : 0] in_en_brq_resolved;
wire commitbus_block_begin_0, commitbus_block_begin_1;
wire[`BRQpower-1:0] commitbus_brqid_0, commitbus_brqid_1;
wire[`BRQsize-1:0] resbus0_brqid_d  , resbus1_brqid_d , resbus2_brqid_d,  resbus3_brqid_d;

wire decbus0_eret = (decbus0_op == `OP_DERET || decbus0_op == `OP_ERET);
wire decbus1_eret = (decbus1_op == `OP_DERET || decbus1_op == `OP_ERET);

wire commit_blikely_0, commit_blikely_1;
wire commit_eret_0, commit_eret_1;

wire[`BRQsize-1:0] commitbus_brqid_0_d; // not used
wire[`BRQsize-1:0] commitbus_brqid_1_d; // not used
brq_decode_eq_3_6 commit0_decode (.in(commitbus_brqid_0), .out(commitbus_brqid_0_d));  // not used
brq_decode_eq_3_6 commit1_decode (.in(commitbus_brqid_1), .out(commitbus_brqid_1_d));  // not used

wire commit0_last_inst = commitbus_valid_0 & (commitbus_bd_0 
                                            | commit_blikely_0 & ~brq_brtaken[commitbus_brqid_0] 
                                            | commit_eret_0);
wire commit1_one_block = commitbus_valid_1 & (commit_blikely_1 & ~brq_brtaken[commitbus_brqid_1] 
                                            | commit_eret_1);
wire [`BRQsize-1:0] cancel_brq_vector; 
assign in_en_brq_valid   = {`BRQsize{commitbus_ex | reset}} |
                           {`BRQsize{decbus0_valid&decbus0_block_begin | decbus1_valid&decbus1_block_begin}} & brq_tail_d |
                           {`BRQsize{brbus_brerror}} & cancel_brq_vector |
                           {`BRQsize{commit0_last_inst}}                     & brq_head_d |
                           {`BRQsize{commitbus_valid_1 &  commitbus_bd_1}}   & brq_head_d |
                           {`BRQsize{commit1_one_block & ~commit0_last_inst}}& brq_head_d |   // think about it
                           {`BRQsize{commit1_one_block &  commit0_last_inst}}& commitbus_brqid_1_d
                           ;

assign in_en_brq_wb      = {`BRQsize{decbus0_valid&decbus0_block_begin}}&brq_tail_d |
                           {`BRQsize{decbus1_valid&decbus1_block_begin}}&brq_tail_d | 
                           {`BRQsize{resbus0_eret_deret}}             &resbus0_brqid_d | 
                           {`BRQsize{resbus1_valid&resbus1_block_end}}&resbus1_brqid_d |
                           {`BRQsize{resbus2_valid&resbus2_block_end}}&resbus2_brqid_d |
                           {`BRQsize{resbus3_valid&resbus3_block_end}}&resbus3_brqid_d;

assign in_en_brq_resolved = {`BRQsize{decbus0_valid&decbus0_block_begin}}&brq_tail_d | 
                            {`BRQsize{decbus1_valid&decbus1_block_begin}}&brq_tail_d |
                            {`BRQsize{brbus_valid}}&brbus_brqid_d;

assign in_en_brq_bdrdy   = {`BRQsize{decbus0_valid&decbus0_block_begin|decbus1_valid&decbus1_block_begin}}&brq_tail_d |
                           {`BRQsize{decbus0_valid&(decbus0_bd|decbus0_eret)&~decbus0_block_begin |
                              decbus1_valid&(decbus1_bd|decbus1_eret)&~decbus0_block_begin & ~decbus1_block_begin}}
                              & brq_tail_minus1_d | 
                           {`BRQsize{decbus0_valid&decbus0_eret&decbus0_block_begin |   ////////  not exclusive
                              decbus1_valid&(decbus1_bd|decbus1_eret) & (decbus0_block_begin|decbus1_block_begin)}} // 3 cases actually
                              & brq_tail_d;
wire [`BRQsize-1:0]brq_valid_tmp;
assign brq_valid_tmp =(commitbus_ex|reset)?`BRQsize'b0 : 
                      (~({`BRQsize{brbus_brerror}}&cancel_brq_vector) &
                       ~({`BRQsize{commit0_last_inst|commit1_one_block|commitbus_valid_1&commitbus_bd_1}}&brq_head_d) &
                       ~({`BRQsize{commit1_one_block&commit0_last_inst}}&commitbus_brqid_1_d)) & brq_valid |
                      {`BRQsize{(decbus0_valid&decbus0_block_begin|decbus1_valid&decbus1_block_begin)}} & brq_tail_d;
wire [`BRQsize-1:0]brq_wb_tmp;
wire [`BRQsize-1:0]brq_resolved_tmp;
wire [`BRQsize-1:0]brq_bdrdy_tmp;
wire [`BRQsize-1:0]brq_bdrdy_tmp_1;
wire [`BRQsize-1:0]tail_minus1_rdy;
wire [`BRQsize-1:0]tail_rdy;
assign brq_wb_tmp = brq_wb &
                   ~({`BRQsize{decbus0_valid&decbus0_block_begin|decbus1_valid&decbus1_block_begin}}&brq_tail_d) |
                   {`BRQsize{resbus0_eret_deret}}&resbus0_brqid_d | 
                   {`BRQsize{resbus1_valid&resbus1_block_end}}&resbus1_brqid_d |
                   {`BRQsize{resbus2_valid&resbus2_block_end}}&resbus2_brqid_d | 
                   {`BRQsize{resbus3_valid&resbus3_block_end}}&resbus3_brqid_d;
                  
assign brq_resolved_tmp = brq_resolved&
                          ~({`BRQsize{decbus0_valid&decbus0_block_begin|decbus1_valid&decbus1_block_begin}}&brq_tail_d)|
                          {`BRQsize{brbus_valid}}&brbus_brqid_d;
assign tail_minus1_rdy = {`BRQsize{decbus0_valid&(decbus0_bd|decbus0_eret)&~decbus0_block_begin | 
                            decbus1_valid&(decbus1_bd|decbus1_eret)&~decbus0_block_begin&~decbus1_block_begin}}
                          & brq_tail_minus1_d;

assign tail_rdy = {`BRQsize{decbus0_valid&decbus0_eret&decbus0_block_begin |
                     decbus1_valid&(decbus1_bd|decbus1_eret)&(decbus0_block_begin|decbus1_block_begin)}} 
                     &brq_tail_d;

assign brq_bdrdy_tmp_1 = brq_bdrdy & 
                        ~({`BRQsize{decbus0_valid&decbus0_block_begin|decbus1_valid&decbus1_block_begin}}&brq_tail_d);
assign brq_bdrdy_tmp =  brq_bdrdy_tmp_1 | tail_minus1_rdy | tail_rdy; 

wire [`BRQsize-1:0] br_vector_tmp;
assign br_vector_tmp[ 0] = brq_valid_tmp[0] &&brq_wb_tmp[0]&&(~brq_resolved_tmp[0])&&brq_bdrdy_tmp[0];
assign br_vector_tmp[ 1] = brq_valid_tmp[1] &&brq_wb_tmp[1]&&(~brq_resolved_tmp[1])&&brq_bdrdy_tmp[1];
assign br_vector_tmp[ 2] = brq_valid_tmp[2] &&brq_wb_tmp[2]&&(~brq_resolved_tmp[2])&&brq_bdrdy_tmp[2];
assign br_vector_tmp[ 3] = brq_valid_tmp[3] &&brq_wb_tmp[3]&&(~brq_resolved_tmp[3])&&brq_bdrdy_tmp[3];
assign br_vector_tmp[ 4] = brq_valid_tmp[4] &&brq_wb_tmp[4]&&(~brq_resolved_tmp[4])&&brq_bdrdy_tmp[4];
assign br_vector_tmp[ 5] = brq_valid_tmp[5] &&brq_wb_tmp[5]&&(~brq_resolved_tmp[5])&&brq_bdrdy_tmp[5];

wire brbus_valid_tmp;
wire brbus_valid_tmp_1;
wire[`BRQsize-1:0] ge_brq_head_vector;

wire [`BRQpower-1:0] brq_head_next_tmp =  (commitbus_valid_0 & commitbus_bd_0 |
                                 commitbus_valid_1 & commitbus_bd_1 |
                                 commitbus_valid_0 & commit_blikely_0 & ~brq_brtaken[commitbus_brqid_0] |
                                 commitbus_valid_1 & commit_blikely_1 & ~brq_brtaken[commitbus_brqid_1] |
                                 commitbus_valid_0 & commit_eret_0 |
                                 commitbus_valid_1 & commit_eret_1) ? brq_head_next : brq_head;

generate_ge_brq_head_vector ge_brq_head_0 (.in(brq_head_next_tmp), .out(ge_brq_head_vector));// in 1 out 111110

wire [`BRQsize-1:0] br_vector_tmp_1;
wire [`BRQsize-1:0] br_vector_tmp_2;
wire [`BRQsize-1:0] br_vector_tmp_3;
wire [`BRQsize-1:0] br_vector_tmp_4;
assign br_vector_tmp_1 = br_vector_tmp & ge_brq_head_vector;

first_one_6_6 brbus_0 (.in(br_vector_tmp_1), .out(br_vector_tmp_2),  .has(brbus_valid_tmp_1));
first_one_6_6 brbus_1 (.in(br_vector_tmp),   .out(br_vector_tmp_3),  .has(brbus_valid_tmp));

assign br_vector_tmp_4 = (|br_vector_tmp_1) ? br_vector_tmp_2 : br_vector_tmp_3;
wire [`Qpower-1:0] brbus_qid;
wire [`BRQpower-1:0] brbus_brqid; 
reg[`BRQsize-1:0] brbus_brqid_d_reg;
reg brbus_valid_reg;
always @ (posedge clock) begin
   if(reset) begin
       brbus_brqid_d_reg<= `BRQsize'b0;
       brbus_valid_reg <= 1'b0;
   end
   else begin
       brbus_brqid_d_reg <= br_vector_tmp_4;
       brbus_valid_reg   <= brbus_valid_tmp;
   end 
end

assign brbus_brqid_d = brbus_brqid_d_reg;

queue_encode_eq_6_3 encode_brqid(.in(brbus_brqid_d),.out(brbus_brqid));

assign brbus_qid = {4{brbus_brqid_d[0]}} & brq_qid_0 | {4{brbus_brqid_d[1]}} & brq_qid_1 |
                   {4{brbus_brqid_d[2]}} & brq_qid_2 | {4{brbus_brqid_d[3]}} & brq_qid_3 |
                   {4{brbus_brqid_d[4]}} & brq_qid_4 | {4{brbus_brqid_d[5]}} & brq_qid_5;
assign brbus_valid = brbus_valid_reg;
assign brbus_brerror = brbus_valid & (brbus_brqid_d[0] & brq_brerror[0] | brbus_brqid_d[1] & brq_brerror[1] |
                                      brbus_brqid_d[2] & brq_brerror[2] | brbus_brqid_d[3] & brq_brerror[3] |
                                      brbus_brqid_d[4] & brq_brerror[4] | brbus_brqid_d[5] & brq_brerror[5]);

reg [`Qsize-1:0] clear_br_ex;
always@(brbus_qid or brbus_brerror)
 begin
    for(i=0;i<`Qsize;i=i+1)
        if(i==brbus_qid)
            clear_br_ex[i] = brbus_brerror;//1'b1;
        else clear_br_ex[i] = 1'b0;
 end
//-------------generate brbus_vector-----------------------------
wire [`BRQsize-1:0] brq_brbus_vector_tmp; 
assign brq_brbus_vector_tmp[ 0] = (cancel_brq_vector[0]) &brbus_brerror ;
assign brq_brbus_vector_tmp[ 1] = (cancel_brq_vector[1]) &brbus_brerror ;
assign brq_brbus_vector_tmp[ 2] = (cancel_brq_vector[2]) &brbus_brerror ;
assign brq_brbus_vector_tmp[ 3] = (cancel_brq_vector[3]) &brbus_brerror ;
assign brq_brbus_vector_tmp[ 4] = (cancel_brq_vector[4]) &brbus_brerror ;
assign brq_brbus_vector_tmp[ 5] = (cancel_brq_vector[5]) &brbus_brerror ;
// the source of brqid is head_next
assign brbus_vector[ 0] = (cancel_brq_vector[queue_brqid_0]) &brbus_brerror ;
assign brbus_vector[ 1] = (cancel_brq_vector[queue_brqid_1]) &brbus_brerror ;
assign brbus_vector[ 2] = (cancel_brq_vector[queue_brqid_2]) &brbus_brerror ;
assign brbus_vector[ 3] = (cancel_brq_vector[queue_brqid_3]) &brbus_brerror ;
assign brbus_vector[ 4] = (cancel_brq_vector[queue_brqid_4]) &brbus_brerror ;
assign brbus_vector[ 5] = (cancel_brq_vector[queue_brqid_5]) &brbus_brerror ;
assign brbus_vector[ 6] = (cancel_brq_vector[queue_brqid_6]) &brbus_brerror ;
assign brbus_vector[ 7] = (cancel_brq_vector[queue_brqid_7]) &brbus_brerror ;
assign brbus_vector[ 8] = (cancel_brq_vector[queue_brqid_8]) &brbus_brerror ;
assign brbus_vector[ 9] = (cancel_brq_vector[queue_brqid_9]) &brbus_brerror ;
assign brbus_vector[10] = (cancel_brq_vector[queue_brqid_10])&brbus_brerror ;
assign brbus_vector[11] = (cancel_brq_vector[queue_brqid_11])&brbus_brerror ;
assign brbus_vector[12] = (cancel_brq_vector[queue_brqid_12])&brbus_brerror ;
assign brbus_vector[13] = (cancel_brq_vector[queue_brqid_13])&brbus_brerror ;
assign brbus_vector[14] = (cancel_brq_vector[queue_brqid_14])&brbus_brerror ;
assign brbus_vector[15] = (cancel_brq_vector[queue_brqid_15])&brbus_brerror ;
reg[31:0] jalr_target_reg;

wire [7:0]  brbus_op           = {8{brbus_brqid_d[0]}} & brq_op_0 | {8{brbus_brqid_d[1]}} & brq_op_1 |
                                 {8{brbus_brqid_d[2]}} & brq_op_2 | {8{brbus_brqid_d[3]}} & brq_op_3 
                                 |
                                 {8{brbus_brqid_d[4]}} & brq_op_4 | {8{brbus_brqid_d[5]}} & brq_op_5
                                 ;

wire [31:0] brbus_value        = q_op_jalr[brbus_qid] ? jalr_target_reg :  
                                 {queue_imm_hi[brbus_qid], queue_imm_low[brbus_qid]};

wire [1:0]  brbus_other_link   = queue_other_link[brbus_qid];

wire [1:0]  brbus_old_status   = {2{brbus_brqid_d[0]}} & brq_status_0 | {2{brbus_brqid_d[1]}} & brq_status_1 |
                                 {2{brbus_brqid_d[2]}} & brq_status_2 | {2{brbus_brqid_d[3]}} & brq_status_3 
                                 |
                                 {2{brbus_brqid_d[4]}} & brq_status_4 | {2{brbus_brqid_d[5]}} & brq_status_5
                                 ;

wire [7:0]  brbus_gshare       = {8{brbus_brqid_d[0]}} & brq_gshare_0 | {8{brbus_brqid_d[1]}} & brq_gshare_1 |
                                 {8{brbus_brqid_d[2]}} & brq_gshare_2 | {8{brbus_brqid_d[3]}} & brq_gshare_3 
                                 |
                                 {8{brbus_brqid_d[4]}} & brq_gshare_4 | {8{brbus_brqid_d[5]}} & brq_gshare_5
                                 ;

wire [1:0]  brbus_rashead_tmp  = {2{brbus_brqid_d[0]}} & brq_rashead_0 | {2{brbus_brqid_d[1]}} & brq_rashead_1 |
                                 {2{brbus_brqid_d[2]}} & brq_rashead_2 | {2{brbus_brqid_d[3]}} & brq_rashead_3 
                                 |
                                 {2{brbus_brqid_d[4]}} & brq_rashead_4 | {2{brbus_brqid_d[5]}} & brq_rashead_5
                                 ;

wire [1:0]  brbus_rashead      = (brbus_op == `OP_JALR ) & (queue_dest[brbus_qid] == 8'h1f) ?
                                  (brbus_rashead_tmp+1'b1 ): brbus_rashead_tmp;
wire        brbus_brtaken      = brbus_brqid_d[0] & brq_brtaken[0]| brbus_brqid_d[1] & brq_brtaken[1]|
                                 brbus_brqid_d[2] & brq_brtaken[2]| brbus_brqid_d[3] & brq_brtaken[3]
                                 |
                                 brbus_brqid_d[4] & brq_brtaken[4]| brbus_brqid_d[5] & brq_brtaken[5]
                                 ;
wire[4:0] qid_tmp  = brbus_qid >= head ? {1'b0,brbus_qid} : (brbus_qid + 5'b10000);

//wire[3:0] brbus_qid_offset = qid_tmp - head;  //nomatch 
wire[`Qpower:0] brbus_qid_offset = qid_tmp - {1'b0,head};  //nomatch 

wire[`Qpower+1:0] brbus_qid_offset_tmp = (brbus_qid_offset << 2'b10); 

wire[`Qpower-1:0] brq_offset_tmp0 = {4{brbus_brqid_d[0]}} & brq_offset_0 | {4{brbus_brqid_d[1]}} & brq_offset_1 |
                                    {4{brbus_brqid_d[2]}} & brq_offset_2 | {4{brbus_brqid_d[3]}} & brq_offset_3 |
                                    {4{brbus_brqid_d[4]}} & brq_offset_4 | {4{brbus_brqid_d[5]}} & brq_offset_5;

wire[31:0] brq_pc_tmp = {32{brbus_brqid_d[0]}} & brq_pc_0 | {32{brbus_brqid_d[1]}} & brq_pc_1 |
                        {32{brbus_brqid_d[2]}} & brq_pc_2 | {32{brbus_brqid_d[3]}} & brq_pc_3 
                        |
                        {32{brbus_brqid_d[4]}} & brq_pc_4 | {32{brbus_brqid_d[5]}} & brq_pc_5
                        ;

wire[`Qpower+1:0]   brq_offset_tmp = (brq_offset_tmp0 << 2'b10);//multiply 4 to get pc offset 

wire[31:0]  brbus_value_h_t =  commitbus_valid_0&commitbus_block_begin_0 ?
                              (brq_pc[commitbus_brqid_0] + 6'b111100) : commit_pc + brbus_qid_offset_tmp;
wire [31:0] brbus_value_h   = (brq_offset_tmp0 == 4'b1111) ? (brbus_value_h_t) : (brq_pc_tmp+brq_offset_tmp);
wire brbus_irstalli_bd = |(stalli & {brbus_qid_d[`Qsize-2:0],brbus_qid_d[`Qsize-1]});

wire brbus_jr31;
wire brbus_config;

assign brbus[0]     = brbus_valid;
assign brbus[1]     = brbus_brerror;
assign brbus[7:2]   = brq_brbus_vector_tmp;
assign brbus[15:8]  = brbus_op;
assign brbus[47:16] = brbus_value;
assign brbus[79:48] = brbus_value_h;
assign brbus[81:80] = brbus_old_status;
assign brbus[89:82] = brbus_gshare;
assign brbus[91:90] = brbus_rashead;
assign brbus[93:92] = brbus_other_link;
assign brbus[94]    = brbus_irstalli_bd;
assign brbus[95]    = brbus_brtaken;
assign brbus[96]    = brbus_jr31;
assign brbus[97]    = brbus_config;

assign brbus_deret_err = brbus_valid &brbus_brerror&(brbus_op == `OP_DERET);
/*----------------------------Third, commit part------------------------------------------*/
assign commitbus_qid_0 = head;
assign commitbus_qid_1 = head_next;

//first commit instruction
wire [`Qpower-1:0]head_even;
wire [`Qpower-1:0]head_next_odd;
assign head_even = {head[0] ? head[`Qpower-1:1]+1'b1 : head[`Qpower-1:1], 1'b0};
assign head_next_odd = {head[`Qpower-1:1],1'b1};

assign commitbus_fp_0 = 1'b0;

assign commitbus_fp_1 = 1'b0;


wire [7:0]queue_op_even;
wire [7:0]queue_op_odd;
assign queue_op_even = queue_op[head_even];
assign queue_op_odd = queue_op[head_next_odd];
assign commitbus_op_0 = head[0] ? queue_op_odd : queue_op_even;

assign commitbus_op_1 = head[0] ? queue_op_even: queue_op_odd;

wire[`BRQpower-1:0] queue_brqid_even;
wire[`BRQpower-1:0] queue_brqid_odd;
assign queue_brqid_even = queue_brqid[head_even];
assign queue_brqid_odd  = queue_brqid[head_next_odd];
assign commitbus_brqid_0 = head[0] ? queue_brqid_odd : queue_brqid_even;
assign commitbus_brqid_1 = head[0] ? queue_brqid_even: queue_brqid_odd;
wire queue_wb_even;
wire queue_wb_odd;
assign queue_wb_even = queue_wb[head_even];
assign queue_wb_odd = queue_wb[head_next_odd];
assign commit_wb_0 = head[0] ? queue_wb_odd : queue_wb_even;
assign commit_wb_1 = head[0] ? queue_wb_even: queue_wb_odd;
wire [1:0]queue_state_even;
wire [1:0]queue_state_odd;
assign queue_state_even = queue_state[head_even];
assign queue_state_odd = queue_state[head_next_odd];
assign commit_state_0 = head[0] ? queue_state_odd : queue_state_even;
assign commit_state_1 = head[0] ? queue_state_even: queue_state_odd;

wire queue_ex_even;
wire queue_ex_odd;
assign queue_ex_even = queue_ex[head_even];
assign queue_ex_odd  = queue_ex[head_next_odd];
assign commit_ex_0 = head[0] ? queue_ex_odd : queue_ex_even;
assign commit_ex_1 = head[0] ? queue_ex_even: queue_ex_odd;

wire [5:0] queue_excode_even;
wire [5:0] queue_excode_odd;
assign queue_excode_even = queue_excode[head_even];
assign queue_excode_odd  = queue_excode[head_next_odd];
assign commit_excode_0 = head[0] ? queue_excode_odd : queue_excode_even;
assign commit_excode_1 = head[0] ? queue_excode_even: queue_excode_odd;

wire  queue_block_begin_even;
wire  queue_block_begin_odd;
assign queue_block_begin_even = queue_block_begin[head_even];
assign queue_block_begin_odd  = queue_block_begin[head_next_odd];
assign commitbus_block_begin_0 = head[0] ? queue_block_begin_odd : queue_block_begin_even;
assign commitbus_block_begin_1 = head[0] ? queue_block_begin_even: queue_block_begin_odd;

assign commitbus_pc_0 = commitbus_block_begin_0 ? brq_pc[commitbus_brqid_0] : commit_pc;
assign commitbus_pc_1 = commitbus_block_begin_1 ? brq_pc[commitbus_brqid_1] : 
                        commitbus_block_begin_0 ? (brq_pc[commitbus_brqid_0] + 3'b100) : (commit_pc + 3'b100);

wire queue_con_true_even;
wire queue_con_true_odd;
assign queue_con_true_even = queue_con_true[head_even];
assign queue_con_true_odd  = queue_con_true[head_next_odd];
assign commitbus_con_true_0 = head[0] ? queue_con_true_odd : queue_con_true_even;
assign commitbus_con_true_1 = head[0] ? queue_con_true_even: queue_con_true_odd;

wire [31:0] queue_imm_even;
wire [31:0] queue_imm_odd;

assign queue_imm_even = {queue_imm_hi[head_even], queue_imm_low[head_even]};
assign queue_imm_odd  = {queue_imm_hi[head_next_odd], queue_imm_low[head_next_odd]};
assign commitbus_taken_target_0 = head[0] ? queue_imm_odd : queue_imm_even;
assign commitbus_taken_target_1 = head[0] ? queue_imm_even: queue_imm_odd;

assign commitbus_value_0 = head[0] ? queue_imm_odd : queue_imm_even;
assign commitbus_value_1 = head[0] ? queue_imm_even: queue_imm_odd;

assign commitbus_old_status_0 = brq_status[brq_head];
assign commitbus_old_status_1 = brq_status[brq_head];

wire [1:0] queue_ce_even;
wire [1:0] queue_ce_odd;
assign queue_ce_even = queue_ce[head_even];
assign queue_ce_odd  = queue_ce[head_next_odd];
assign commitbus_ce_0 = head[0] ? queue_ce_odd : queue_ce_even;
assign commitbus_ce_1 = head[0] ? queue_ce_even: queue_ce_odd;

wire [7:0] queue_dest_even;
wire [7:0] queue_dest_odd;
assign queue_dest_even = queue_dest[head_even];
assign queue_dest_odd  = queue_dest[head_next_odd];
assign commitbus_dest_0 = head[0] ? queue_dest_odd : queue_dest_even;
assign commitbus_dest_1 = head[0] ? queue_dest_even: queue_dest_odd;

wire [1:0] queue_ac_even;
wire [1:0] queue_ac_odd;
assign queue_ac_even = queue_ac[head_even];
assign queue_ac_odd  = queue_ac[head_next_odd];
assign commitbus_ac_0 = head[0] ? queue_ac_odd : queue_ac_even;
assign commitbus_ac_1 = head[0] ? queue_ac_even: queue_ac_odd;

wire queue_bd_even;
wire queue_bd_odd;
assign queue_bd_even = queue_bd[head_even];
assign queue_bd_odd  = queue_bd[head_next_odd];
assign commitbus_bd_0 = head[0] ? queue_bd_odd : queue_bd_even;
assign commitbus_bd_1 = head[0] ? queue_bd_even: queue_bd_odd;

assign commitbus_gshare_0 = brq_gshare[brq_head];
assign commitbus_gshare_1 = brq_gshare[brq_head];

assign commitbus_rashead_0 = brq_rashead[brq_head];
assign commitbus_rashead_1 = brq_rashead[brq_head];

wire [1:0] queue_other_link_even;
wire [1:0] queue_other_link_odd;
assign queue_other_link_even = queue_other_link[head_even];
assign queue_other_link_odd  = queue_other_link[head_next_odd];
assign commitbus_other_link_0 = head[0] ? queue_other_link_odd : queue_other_link_even;
assign commitbus_other_link_1 = head[0] ? queue_other_link_even: queue_other_link_odd;

wire [31:0] queue_dspctl_even;
wire [31:0] queue_dspctl_odd;
//HAVE_DSP_UNIT
assign queue_dspctl_even = queue_dspctl[head_even];
assign queue_dspctl_odd  = queue_dspctl[head_next_odd];
assign commitbus_dspctl_0 = head[0] ? queue_dspctl_odd : queue_dspctl_even;
assign commitbus_dspctl_1 = head[0] ? queue_dspctl_even: queue_dspctl_odd;

wire   commit_next_empty_0,commit_next_empty_1; //indicate whether the next entry after the current instruction to be commited is empty

assign commit_next_empty_0 = s_empty[head_next];
assign commit_next_empty_1 = s_empty[head_next_next];

wire commit_branch_0, commit_branch_1;
wire commit_bnt_0, commit_bnt_1;
wire commit_bdrdy_0, commit_bdrdy_1;

reg  dss_flag;
wire execution_step;
wire ex_debug_0;
wire ex_debug_1;
reg  reset_delay1;

always @(posedge clock)
  reset_delay1<=reset;

assign commit_eret_0    =  q_op_eret[head];
assign commit_eret_1    =  q_op_eret[head_next];

assign commit_branch_0  =  q_op_branch[head];
assign commit_branch_1  =  q_op_branch[head_next];

assign commit_blikely_0 = q_op_blikely[head];
assign commit_blikely_1 = q_op_blikely[head_next];
assign commit_bdrdy_0   = (!commit_branch_0)||(!commit_next_empty_0); //the instruction in delay slot need to be in opqueue!
assign commit_bdrdy_1   = (!commit_branch_1)||(!commit_next_empty_1);

//HAVE_DSP_UNIT
wire [7:0] outflag_res_0 = commitbus_dspctl_0[23:16];

wire [7:0] outflag_res_1 = commitbus_dspctl_1[23:16];

//wire just_one_commited_ok = ~(^(outflag_res_0 | outflag_res_1)) & (|outflag_res_0);//two instructions write different bits of ouflag

// HAVE_DSP_UNIT
wire commitbus_wrdsp_0 = (commitbus_op_0 == `OP_WRDSP);

wire commitbus_wrdsp_1 = (commitbus_op_1 == `OP_WRDSP);

//wire second_commit_ok =  ~commitbus_wrdsp_0 & ~commitbus_wrdsp_1 & ~just_one_commited_ok; 

assign commitbus_valid_0 = (commit_state_0==`ISSUED)&&(commit_ex_0||commit_wb_0); 
assign commitbus_valid_1 = DSS_ENABLE ? 1'b0 : // when dss enble, then only commit one instruction once
                           commitbus_valid_0&&commit_ex_0 ? 1'b0:
                           commitbus_valid_0&&(commit_state_1==`ISSUED)&&(commit_ex_1||commit_wb_1)/*&
                           second_commit_ok*/;
                           
assign commitbus_excode_0= EJTAGBOOT    ? `EX_EJTAGBOOT :
                           softreset    ? `EX_SOFTRESET : 
                           (commit_ex_0 & nmi_now & cr_cfg6_rti_i) ? `EX_NMI :
                           (commit_ex_0 & f_int & cr_cfg6_rti_i)   ? `EX_INTERRUPT :
                           commit_ex_0 ? commit_excode_0 : 6'h0;
assign commitbus_excode_1= EJTAGBOOT    ? `EX_EJTAGBOOT :
                           softreset    ? `EX_SOFTRESET : 
                           (commit_ex_1 & nmi_now & cr_cfg6_rti_i) ? `EX_NMI :
                           (commit_ex_1 & f_int & cr_cfg6_rti_i)   ? `EX_INTERRUPT :
                           commit_ex_1 ? commit_excode_1 : 6'h0;

assign commitbus_ex_0=EJTAGBOOT||softreset||(commitbus_valid_0&&commit_ex_0);
assign commitbus_ex_1=EJTAGBOOT||softreset||(commitbus_valid_1&&commit_ex_1);
assign commit_bnt_0=commitbus_ex_0&&(commitbus_excode_0==`EX_BNT);
assign commit_bnt_1=commitbus_ex_1&&(commitbus_excode_1==`EX_BNT);

// generates dss_flag (commeted out temporally --qffan)
//modified by xucp for two-issuing
//when dss enable, only commit one instruction
assign ex_debug_0=commitbus_ex_0&& // Attention:exclude dss,sdbbp and ejtagboot here.
                ((commitbus_excode_0==`EX_DIB)||(commitbus_excode_0==`EX_DDBL)||
                 (commitbus_excode_0==`EX_DDBS)||(commitbus_excode_0==`EX_DDBLIMPR)||
                 (commitbus_excode_0==`EX_DDBSIMPR)||(commitbus_excode_0==`EX_DINT));

assign execution_step=(commit_bnt_0&&(commit_eret_0||commit_blikely_0))||
                      (commitbus_valid_0&&(~commit_branch_0)&&(~ex_debug_0))||
                      (commitbus_ex_0&&(commitbus_excode_0==`EX_DSS));

always @(posedge clock)
if (reset)
   dss_flag<=1'b0;
else
   if (execution_step&&(~DEBUG_MODE)&&DSS_ENABLE)
      dss_flag<=~dss_flag;

/***************************************************************************************************/
/*--------------------------------Fourth, Register rename in decode stage---------------------------*/
/***************************************************************************************************/
wire [5:0] dec0_excode  , dec1_excode  ;
wire       res0_ex, res1_ex, res2_ex, res3_ex;
wire [5:0] res0_excode, res1_excode, res2_excode, res3_excode;

wire [7:0] decsrc1_0    , decsrc2_0     ;
wire [7:0] decsrc1_1    , decsrc2_1     ;

wire [`Qsize-1:0] equal_dest_src1_0;
wire [`Qsize-1:0] equal_dest_src2_0;
wire [`Qsize-1:0] match_dest_src1_0;
wire [`Qsize-1:0] match_dest_src2_0;
wire [`Qsize-1:0] equal_dest_src1_1;
wire [`Qsize-1:0] equal_dest_src2_1;
wire [`Qsize-1:0] match_dest_src1_1;
wire [`Qsize-1:0] match_dest_src2_1;

assign decsrc1_0 = decbus0_src1;
assign decsrc2_0 = decbus0_src2;
assign decsrc1_1 = decbus1_src1;
assign decsrc2_1 = decbus1_src2;
assign dest0  = queue_dest[0];  
assign dest1  = queue_dest[1];
assign dest2  = queue_dest[2];
assign dest3  = queue_dest[3];
assign dest4  = queue_dest[4];
assign dest5  = queue_dest[5];
assign dest6  = queue_dest[6];
assign dest7  = queue_dest[7];
assign dest8  = queue_dest[8];  
assign dest9  = queue_dest[9];
assign dest10 = queue_dest[10];
assign dest11 = queue_dest[11];
assign dest12 = queue_dest[12];
assign dest13 = queue_dest[13];
assign dest14 = queue_dest[14];
assign dest15 = queue_dest[15];

wire [`Qsize-1:0] tail_lt_d;
assign tail_lt_d[15] = 1'b0;
assign tail_lt_d[14] =  tail_d[15];
assign tail_lt_d[13] = |tail_d[15:14];
assign tail_lt_d[12] = |tail_d[15:13];
assign tail_lt_d[11] = |tail_d[15:12];
assign tail_lt_d[10] = |tail_d[15:11];
assign tail_lt_d[9] =  |tail_d[15:10];
assign tail_lt_d[8] =  |tail_d[15:9];
assign tail_lt_d[7] =  |tail_d[15:8];
assign tail_lt_d[6] =  |tail_d[15:7];
assign tail_lt_d[5] =  |tail_d[15:6];
assign tail_lt_d[4] =  |tail_d[15:5];
assign tail_lt_d[3] =  |tail_d[15:4];
assign tail_lt_d[2] =  |tail_d[15:3];
assign tail_lt_d[1] =  |tail_d[15:2];
assign tail_lt_d[0] =  |tail_d[15:1];

/*decbus0 part*/
wire [`Qsize-1:0]commit_tmp = {16{commitbus_valid_0}} & head_d | {16{commitbus_valid_1}} & head_next_d;

assign equal_dest_src1_0[0] =  (dest0==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[1] =  (dest1==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[2] =  (dest2==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[3] =  (dest3==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[4] =  (dest4==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[5] =  (dest5==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[6] =  (dest6==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[7] =  (dest7==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[8] =  (dest8==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[9] =  (dest9==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[10] = (dest10==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[11] = (dest11==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[12] = (dest12==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[13] = (dest13==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[14] = (dest14==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src1_0[15] = (dest15==decsrc1_0)&(decsrc1_0[7:6]==2'b00);
assign equal_dest_src2_0[0] =  (dest0==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[1] =  (dest1==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[2] =  (dest2==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[3] =  (dest3==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[4] =  (dest4==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[5] =  (dest5==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[6] =  (dest6==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[7] =  (dest7==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[8] =  (dest8==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[9] =  (dest9==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[10] = (dest10==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[11] = (dest11==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[12] = (dest12==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[13] = (dest13==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[14] = (dest14==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign equal_dest_src2_0[15] = (dest15==decsrc2_0)&(decsrc2_0[7:6]==2'b00);
assign match_dest_src1_0     = ~s_empty & ~commit_tmp & equal_dest_src1_0;
assign match_dest_src2_0     = ~s_empty & ~commit_tmp & equal_dest_src2_0; 
assign decbus0_rdy1 = (decbus0_src1==8'h0)|(decbus0_src1==8'h3f)|~(|match_dest_src1_0); 
assign decbus0_rdy2 = (decbus0_src2==8'h0)|(decbus0_src2==8'h3f)|~(|match_dest_src2_0);

queue_rename_encode_no_falu qrename1_0  (.match_d(match_dest_src1_0), 
                                 .mask_lt(tail_lt_d),.match_qid(decbus0_qid1_queue));
queue_rename_encode_no_falu qrename2_0  (.match_d(match_dest_src2_0), 
                                 .mask_lt(tail_lt_d),.match_qid(decbus0_qid2_queue));
assign decbus0_qid1 = (decbus0_src1 == 8'h3f|decbus0_op==`OP_INS) ?
                        tail : decbus0_qid1_queue;
assign decbus0_qid2 = (decbus0_src2 == 8'h3f) ? tail : decbus0_qid2_queue;

/*decbus1 part*/

assign equal_dest_src1_1[0] = (dest0==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[1] = (dest1==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[2] = (dest2==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[3] = (dest3==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[4] = (dest4==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[5] = (dest5==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[6] = (dest6==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[7] = (dest7==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[8] = (dest8==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[9] = (dest9==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[10] = (dest10==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[11] = (dest11==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[12] = (dest12==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[13] = (dest13==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[14] = (dest14==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
assign equal_dest_src1_1[15] = (dest15==decsrc1_1)&(decbus1_src1[7:6]==2'b00);

assign equal_dest_src2_1[0] = (dest0==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[1] = (dest1==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[2] = (dest2==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[3] = (dest3==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[4] = (dest4==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[5] = (dest5==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[6] = (dest6==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[7] = (dest7==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[8] = (dest8==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[9] = (dest9==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[10] = (dest10==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[11] = (dest11==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[12] = (dest12==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[13] = (dest13==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[14] = (dest14==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign equal_dest_src2_1[15] = (dest15==decsrc2_1)&(decbus1_src2[7:6]==2'b00);
assign match_dest_src1_1    = ~s_empty & ~commit_tmp & equal_dest_src1_1; 

assign match_dest_src2_1    = ~s_empty & ~commit_tmp & equal_dest_src2_1; 

queue_rename_encode_no_falu qrename1_1  (.match_d(match_dest_src1_1), 
                                 .mask_lt(tail_lt_d),.match_qid(decbus1_qid1_queue));
queue_rename_encode_no_falu qrename2_1  (.match_d(match_dest_src2_1), 
                                 .mask_lt(tail_lt_d),.match_qid(decbus1_qid2_queue));

//----------------------------resolve dependences between decbus0 and decbus1-----------------------------

wire dec_match_dest_src1 = (decbus0_dest==decsrc1_1)&(decbus1_src1[7:6]==2'b00);
wire dec_match_dest_src2 = (decbus0_dest==decsrc2_1)&(decbus1_src2[7:6]==2'b00);

assign decbus1_rdy1   = (decbus1_src1==8'h0)|(decbus1_src1==8'h3f)|
                        ~(|{match_dest_src1_1, dec_match_dest_src1});
assign decbus1_rdy2   = (decbus1_src2==8'h0)|(decbus1_src2==8'h3f)|
                        ~(|{match_dest_src2_1, dec_match_dest_src2});
assign decbus1_qid1   = ((decbus1_src1==8'h3f)|(decbus1_op==`OP_INS)) ? tail_next:
                        dec_match_dest_src1 ? tail : decbus1_qid1_queue;
assign decbus1_qid2   = (decbus1_src2==8'h3f) ? tail_next:
                        dec_match_dest_src2 ? tail : decbus1_qid2_queue;

/*-------------------Fifth, modify the queue for all cases------------------------------------------*/ 
wire   wb0_br;
wire   wb1_br;
wire   wb3_br;
wire   wb2_br;

assign wb0_br = q_op_br[resbus0_qid];
assign wb1_br = q_op_br[resbus1_qid];
assign wb3_br = q_op_br[resbus3_qid];
assign wb2_br = q_op_br[resbus2_qid];

/**********ctc1 caused evzoui exception***/
wire fpe0 = 1'b0;

assign dint=EJTAGBRK&&(~DEBUG_MODE);
assign dss=dss_flag&&(~DEBUG_MODE)&&DSS_ENABLE;
assign f_int=interrupt&&INTE&&(~DEBUG_MODE);
assign nmi_now=nmi&&(~DEBUG_MODE);

assign dec0_ex = decbus0_adei||decbus0_tlbii||decbus0_tlbir||decbus0_ibe||
                 decbus0_ri  ||decbus0_cpu  ||decbus0_sys  ||decbus0_bp ||
                 nmi_now     ||f_int          ||decbus0_sdbbp||decbus0_dib||
                 dint        ||dss || decbus0_int;
assign dec1_ex = decbus1_adei||decbus1_tlbii||decbus1_tlbir||decbus1_ibe||
                 decbus1_ri  ||decbus1_cpu  ||decbus1_sys  ||decbus1_bp ||
                 nmi_now     ||f_int          ||decbus1_sdbbp||decbus1_dib||
                 dint        ||dss   || decbus1_int;
assign res0_ex = resbus0_mod  || resbus0_ri  ||
                 resbus0_tlbli||resbus0_tlblr||resbus0_tlbsi||resbus0_tlbsr||
                 resbus0_adel ||resbus0_ades ||resbus0_dbe  ||resbus0_watch||
                 resbus0_ddbl ||resbus0_ddbs ||resbus0_ddblimpr||
                 resbus0_ddbsimpr;
assign res1_ex = resbus1_ov   ||resbus1_trap ||resbus1_mod  ||
                 resbus1_tlbli||resbus1_tlblr||resbus1_tlbsi||resbus1_tlbsr||
                 resbus1_adel ||resbus1_ades ||resbus1_dbe  ||resbus1_watch;
assign res3_ex = resbus3_ov   ||
                 resbus3_trap ||
                 resbus3_mod  ||
                 resbus3_tlbli||resbus3_tlblr||resbus3_tlbsi||resbus3_tlbsr||
                 resbus3_adel ||resbus3_ades ||resbus3_dbe  ||resbus3_watch ;                


assign res2_ex = resbus2_ov   ||resbus2_mod  ||
                 resbus2_tlbli||resbus2_tlblr||resbus2_tlbsi||resbus2_tlbsr||
                 resbus2_adel ||resbus2_ades ||resbus2_dbe  ||resbus2_watch ;


assign dec0_excode = dss           ? `EX_DSS :
                     dint          ? `EX_DINT:
                     nmi_now       ? `EX_NMI :
                     (f_int | decbus0_int)         ? `EX_INTERRUPT :
                     decbus0_dib   ? `EX_DIB :
                     decbus0_adei  ? `EX_ADEL :
                     decbus0_tlbir ? `EX_TLBLR:
                     decbus0_tlbii ? `EX_TLBLI:
                     decbus0_ibe   ? `EX_IBE:
                     decbus0_sys   ? `EX_SYS:
                     decbus0_bp    ? `EX_BP:
                     decbus0_sdbbp ? `EX_SDBBP:
                     decbus0_ri    ? `EX_RI: `EX_CPU; //decbus0_cpu

assign dec1_excode = dss           ? `EX_DSS :
                     dint          ? `EX_DINT:
                     nmi_now       ? `EX_NMI :
                     (f_int  | decbus1_int)        ? `EX_INTERRUPT :
                     decbus1_dib   ? `EX_DIB :
                     decbus1_adei  ? `EX_ADEL :
                     decbus1_tlbir ? `EX_TLBLR:
                     decbus1_tlbii ? `EX_TLBLI:
                     decbus1_ibe   ? `EX_IBE:
                     decbus1_sys   ? `EX_SYS:
                     decbus1_bp    ? `EX_BP:
                     decbus1_sdbbp ? `EX_SDBBP:
                     decbus1_ri    ? `EX_RI: `EX_CPU; //decbus1_cpu
assign res0_excode = resbus0_ri       ? `EX_RI:
                     resbus0_ddbl     ? `EX_DDBL:
                     resbus0_ddbs     ? `EX_DDBS:
                     resbus0_ddbsimpr ? `EX_DDBSIMPR:
                     resbus0_adel     ? `EX_ADEL:
                     resbus0_ades     ? `EX_ADES:
                     resbus0_tlblr    ? `EX_TLBLR:
                     resbus0_tlbsr    ? `EX_TLBSR:
                     resbus0_tlbli    ? `EX_TLBLI:
                     resbus0_tlbsi    ? `EX_TLBSI:
                     resbus0_mod      ? `EX_MOD:
                     resbus0_watch    ? `EX_WATCH:
                     resbus0_dbe      ? `EX_DBE: 
                     resbus0_ddblimpr ? `EX_DDBLIMPR : `EX_FPE; //fpe0

assign res1_excode = resbus1_ov       ? `EX_OV: `EX_TRAP;
assign res3_excode = resbus3_ov       ? `EX_OV: `EX_TRAP; 
assign res2_excode = resbus2_ov       ? `EX_OV: `EX_FPE; 

/**********************form resbus0_to_fpq, resbus2_to_fpq*************************/
wire [`BRQpower-1:0] resbus0_brqid;
wire [`BRQpower-1:0] resbus1_brqid;
wire [`BRQpower-1:0] resbus2_brqid;
wire [`BRQpower-1:0] resbus3_brqid;
/**********************************************************************************/
wire [7:0] tail_even_odd_d_t,tail_even_odd_d;

decoder_3_8 tail_even_odd(tail[3:1],tail_even_odd_d_t);

assign tail_even_odd_d = tail_even_odd_d_t & {8{decbus0_valid}};

wire [7:0] write_even;
wire [7:0] write_odd;

assign write_even = reset ? 8'hff : tail[0] ? ({tail_even_odd_d[6:0], tail_even_odd_d[7]} & {8{decbus1_valid}}) : tail_even_odd_d;
assign write_odd  = reset ? 8'hff : tail_even_odd_d & {8{(tail[0] | decbus1_valid)}};

wire [3:0] queue_qid1_even_w = tail[0] ? decbus1_qid1 : decbus0_qid1;
wire [3:0] queue_qid1_odd_w  = tail[0] ? decbus0_qid1 : decbus1_qid1;

wire [3:0] queue_qid2_even_w = tail[0] ? decbus1_qid2 : decbus0_qid2;
wire [3:0] queue_qid2_odd_w  = tail[0] ? decbus0_qid2 : decbus1_qid2;


wire qissuebus0_fp = (qissuebus0_src1[7:6]==2'b01) | (qissuebus0_src2[7:6]==2'b01) | (qissuebus0_dest[7:6]==2'b01);
wire qissuebus1_fp = (qissuebus1_src1[7:6]==2'b01) | (qissuebus1_src2[7:6]==2'b01) | (qissuebus1_dest[7:6]==2'b01);

wire [7:0] queue_op_even_w = reset ? `OP_SLL : tail[0] ? decbus1_op : decbus0_op;
wire [7:0] queue_op_odd_w  = reset ? `OP_SLL : tail[0] ? decbus0_op : decbus1_op;

wire queue_bd_even_w = tail[0] ? decbus1_bd : decbus0_bd;
wire queue_bd_odd_w  = tail[0] ? decbus0_bd : decbus1_bd;

wire [1:0] queue_ce_even_w = tail[0] ? decbus1_ce : decbus0_ce;
wire [1:0] queue_ce_odd_w  = tail[0] ? decbus0_ce : decbus1_ce;

wire [7:0] queue_src1_even_w = tail[0] ? decbus1_src1 : decbus0_src1;
wire [7:0] queue_src1_odd_w  = tail[0] ? decbus0_src1 : decbus1_src1;

wire [7:0] queue_src2_even_w = tail[0] ? decbus1_src2 : decbus0_src2;
wire [7:0] queue_src2_odd_w  = tail[0] ? decbus0_src2 : decbus1_src2;

wire [7:0] queue_dest_even_w = tail[0] ? decbus1_dest : decbus0_dest;
wire [7:0] queue_dest_odd_w  = tail[0] ? decbus0_dest : decbus1_dest;

wire [`Qsize-1:0] in_en_fp;
assign in_en_fp = {16{reset | commitbus_ex}} |
                  {16{commitbus_valid_0}} & head_d      |
                  {16{commitbus_valid_1}} & head_next_d |
                  {16{brbus_valid}}&brbus_vector         |
                  {16{qissuebus0_valid & qissuebus0_fp}} & issue_0_d | 
                  {16{qissuebus1_valid & qissuebus1_fp}} & issue_1_d;
 
//in_en_op
assign in_en_op = {16{reset}};

always @(posedge clock)
begin
  for(i=0;i<8;i=i+1) begin
    if(write_even[i])
      begin
       queue_op[2*i] <= queue_op_even_w;
       queue_src1[2*i] <= queue_src1_even_w; 
       queue_src2[2*i] <= queue_src2_even_w; 
       queue_dest[2*i] <= queue_dest_even_w; 
       queue_bd[2*i] <= queue_bd_even_w; 
       queue_ce[2*i] <= queue_ce_even_w; 
       queue_qid1[2*i] <= queue_qid1_even_w; 
       queue_qid2[2*i] <= queue_qid2_even_w; 
       
      end
    if(write_odd[i])
      begin
       queue_op[2*i+1]<=queue_op_odd_w;
       queue_src1[2*i+1]<=queue_src1_odd_w;
       queue_src2[2*i+1]<=queue_src2_odd_w;
       queue_dest[2*i+1]<=queue_dest_odd_w;
       queue_bd[2*i+1]<=queue_bd_odd_w;
       queue_ce[2*i+1]<=queue_ce_odd_w;
       queue_qid1[2*i+1] <= queue_qid1_odd_w; 
       queue_qid2[2*i+1] <= queue_qid2_odd_w; 
      
      end
 end
end
assign commit0_eq_qid1[0] =  (commitbus_qid_0 == queue_qid1_tmp[0]) ; 
assign commit0_eq_qid1[1] =  (commitbus_qid_0 == queue_qid1_tmp[1]) ; 
assign commit0_eq_qid1[2] =  (commitbus_qid_0 == queue_qid1_tmp[2]) ; 
assign commit0_eq_qid1[3] =  (commitbus_qid_0 == queue_qid1_tmp[3]) ; 
assign commit0_eq_qid1[4] =  (commitbus_qid_0 == queue_qid1_tmp[4]) ; 
assign commit0_eq_qid1[5] =  (commitbus_qid_0 == queue_qid1_tmp[5]) ; 
assign commit0_eq_qid1[6] =  (commitbus_qid_0 == queue_qid1_tmp[6]) ; 
assign commit0_eq_qid1[7] =  (commitbus_qid_0 == queue_qid1_tmp[7]) ; 
assign commit0_eq_qid1[8] =  (commitbus_qid_0 == queue_qid1_tmp[8]) ; 
assign commit0_eq_qid1[9] =  (commitbus_qid_0 == queue_qid1_tmp[9]) ; 
assign commit0_eq_qid1[10] = (commitbus_qid_0 == queue_qid1_tmp[10]) ; 
assign commit0_eq_qid1[11] = (commitbus_qid_0 == queue_qid1_tmp[11]) ; 
assign commit0_eq_qid1[12] = (commitbus_qid_0 == queue_qid1_tmp[12]) ; 
assign commit0_eq_qid1[13] = (commitbus_qid_0 == queue_qid1_tmp[13]) ; 
assign commit0_eq_qid1[14] = (commitbus_qid_0 == queue_qid1_tmp[14]) ; 
assign commit0_eq_qid1[15] = (commitbus_qid_0 == queue_qid1_tmp[15]) ; 

assign commit1_eq_qid1[0] =  (commitbus_qid_1 == queue_qid1_tmp[0]) ;
assign commit1_eq_qid1[1] =  (commitbus_qid_1 == queue_qid1_tmp[1]) ;
assign commit1_eq_qid1[2] =  (commitbus_qid_1 == queue_qid1_tmp[2]) ;
assign commit1_eq_qid1[3] =  (commitbus_qid_1 == queue_qid1_tmp[3]) ;
assign commit1_eq_qid1[4] =  (commitbus_qid_1 == queue_qid1_tmp[4]) ;
assign commit1_eq_qid1[5] =  (commitbus_qid_1 == queue_qid1_tmp[5]) ;
assign commit1_eq_qid1[6] =  (commitbus_qid_1 == queue_qid1_tmp[6]) ;
assign commit1_eq_qid1[7] =  (commitbus_qid_1 == queue_qid1_tmp[7]) ;
assign commit1_eq_qid1[8] =  (commitbus_qid_1 == queue_qid1_tmp[8]) ;
assign commit1_eq_qid1[9] =  (commitbus_qid_1 == queue_qid1_tmp[9]) ;
assign commit1_eq_qid1[10] = (commitbus_qid_1 == queue_qid1_tmp[10]) ;
assign commit1_eq_qid1[11] = (commitbus_qid_1 == queue_qid1_tmp[11]) ;
assign commit1_eq_qid1[12] = (commitbus_qid_1 == queue_qid1_tmp[12]) ;
assign commit1_eq_qid1[13] = (commitbus_qid_1 == queue_qid1_tmp[13]) ;
assign commit1_eq_qid1[14] = (commitbus_qid_1 == queue_qid1_tmp[14]) ;
assign commit1_eq_qid1[15] = (commitbus_qid_1 == queue_qid1_tmp[15]) ;

assign commit0_eq_qid2[0] =  (commitbus_qid_0 == queue_qid2_tmp[0]) ;
assign commit0_eq_qid2[1] =  (commitbus_qid_0 == queue_qid2_tmp[1]) ;
assign commit0_eq_qid2[2] =  (commitbus_qid_0 == queue_qid2_tmp[2]) ;
assign commit0_eq_qid2[3] =  (commitbus_qid_0 == queue_qid2_tmp[3]) ;
assign commit0_eq_qid2[4] =  (commitbus_qid_0 == queue_qid2_tmp[4]) ;
assign commit0_eq_qid2[5] =  (commitbus_qid_0 == queue_qid2_tmp[5]) ;
assign commit0_eq_qid2[6] =  (commitbus_qid_0 == queue_qid2_tmp[6]) ;
assign commit0_eq_qid2[7] =  (commitbus_qid_0 == queue_qid2_tmp[7]) ;
assign commit0_eq_qid2[8] =  (commitbus_qid_0 == queue_qid2_tmp[8]) ;
assign commit0_eq_qid2[9] =  (commitbus_qid_0 == queue_qid2_tmp[9]) ;
assign commit0_eq_qid2[10] = (commitbus_qid_0 == queue_qid2_tmp[10]) ;
assign commit0_eq_qid2[11] = (commitbus_qid_0 == queue_qid2_tmp[11]) ;
assign commit0_eq_qid2[12] = (commitbus_qid_0 == queue_qid2_tmp[12]) ;
assign commit0_eq_qid2[13] = (commitbus_qid_0 == queue_qid2_tmp[13]) ;
assign commit0_eq_qid2[14] = (commitbus_qid_0 == queue_qid2_tmp[14]) ;
assign commit0_eq_qid2[15] = (commitbus_qid_0 == queue_qid2_tmp[15]) ;

assign commit1_eq_qid2[0] =  (commitbus_qid_1 == queue_qid2_tmp[0]) ;
assign commit1_eq_qid2[1] =  (commitbus_qid_1 == queue_qid2_tmp[1]) ;
assign commit1_eq_qid2[2] =  (commitbus_qid_1 == queue_qid2_tmp[2]) ;
assign commit1_eq_qid2[3] =  (commitbus_qid_1 == queue_qid2_tmp[3]) ;
assign commit1_eq_qid2[4] =  (commitbus_qid_1 == queue_qid2_tmp[4]) ;
assign commit1_eq_qid2[5] =  (commitbus_qid_1 == queue_qid2_tmp[5]) ;
assign commit1_eq_qid2[6] =  (commitbus_qid_1 == queue_qid2_tmp[6]) ;
assign commit1_eq_qid2[7] =  (commitbus_qid_1 == queue_qid2_tmp[7]) ;
assign commit1_eq_qid2[8] =  (commitbus_qid_1 == queue_qid2_tmp[8]) ;
assign commit1_eq_qid2[9] =  (commitbus_qid_1 == queue_qid2_tmp[9]) ;
assign commit1_eq_qid2[10] = (commitbus_qid_1 == queue_qid2_tmp[10]) ;
assign commit1_eq_qid2[11] = (commitbus_qid_1 == queue_qid2_tmp[11]) ;
assign commit1_eq_qid2[12] = (commitbus_qid_1 == queue_qid2_tmp[12]) ;
assign commit1_eq_qid2[13] = (commitbus_qid_1 == queue_qid2_tmp[13]) ;
assign commit1_eq_qid2[14] = (commitbus_qid_1 == queue_qid2_tmp[14]) ;
assign commit1_eq_qid2[15] = (commitbus_qid_1 == queue_qid2_tmp[15]) ;

assign in_en_rdy1 = {16{decbus0_valid}} & tail_d    |
                    {16{decbus1_valid}} & tail_next_d|
                    {16{commitbus_valid_0}} & commit0_eq_qid1 
                   |{16{commitbus_valid_1}} & commit1_eq_qid1
                   ;

assign in_en_rdy2 = {16{decbus0_valid}} & tail_d    |
                    {16{decbus1_valid}} & tail_next_d|
                    {16{commitbus_valid_0}} & commit0_eq_qid2 
                   |{16{commitbus_valid_1}} & commit1_eq_qid2
                    ;

assign in_en_state = ({16{reset||commitbus_ex}})              |
                     ({16{commitbus_valid_0}} & head_d )      |
                     ({16{commitbus_valid_1}} & head_next_d ) |
                     ({16{decbus0_valid}} & tail_d )          |
                     ({16{decbus1_valid}} & tail_next_d )     |
                     ({16{qissuebus0_valid}} & issue_0_d )    |
                     ({16{qissuebus1_valid}} & issue_1_d )    |
                     ({16{brbus_valid}}&brbus_vector)         |  
                     rt_int_v;
assign in_en_imm_hi = ({16{decbus0_imm_hi_en}} & tail_d) |
                      ({16{decbus1_imm_hi_en}} & tail_next_d) |
                      ({16{resbus0_valid&&(!wb0_br)}} & wb0_d)  | 
                      ({16{resbus1_valid&&(!wb1_br)}} & wb1_d)  |
                      ({16{resbus3_valid&&(!wb3_br)}} & wb3_d & {16{~resbus3_acc_op}});

assign in_en_imm_low = ({16{decbus0_imm_low_en}} & tail_d) |
                       ({16{decbus1_imm_low_en}} & tail_next_d) |
                       ({16{resbus0_valid&&(!wb0_br)}} & wb0_d)  | 
                       ({16{resbus1_valid&&(!wb1_br)}} & wb1_d)  |
                       ({16{resbus3_valid&&(!wb3_br)}} & wb3_d & {16{~resbus3_acc_op}});

assign in_en_wb = ({16{decbus0_valid}} & tail_d) |
                  ({16{decbus1_valid}} & tail_next_d) |
                  ({16{resbus0_valid&~resbus0_bnt}} & wb0_d)  |
                  ({16{resbus1_valid&~resbus1_block_end}} & wb1_d)  |
                  ({16{resbus2_valid&~resbus2_block_end}} & wb2_d)  |
                  ({16{resbus3_valid&~resbus3_block_end}} & wb3_d)  |
                  ({16{brbus_valid}} & brbus_qid_d)|
                  rt_int_v; 

assign resbus0_write_fpq = resbus0_write_fpq_tmp & (~res0_ex | fpe0);

//in_en_other_link
wire [1:0] queue_other_link_even_w = tail[0] ? decbus1_other_link : decbus0_other_link;
wire [1:0] queue_other_link_odd_w  = tail[0] ? decbus0_other_link : decbus1_other_link;
always @(posedge clock)
begin
  for(i=0;i<8;i=i+1) begin
    if(write_even[i])
      begin
       queue_other_link[2*i] <= queue_other_link_even_w; 
      end
    if(write_odd[i])
      begin
       queue_other_link[2*i+1] <= queue_other_link_odd_w; 
      end
 end
end

//in_en_block_begin
wire  queue_block_begin_even_w = tail[0] ? decbus1_block_begin : decbus0_block_begin;
wire  queue_block_begin_odd_w  = tail[0] ? decbus0_block_begin : decbus1_block_begin;
always @(posedge clock)
begin
  for(i=0;i<8;i=i+1) begin
    if(write_even[i])
      begin
       queue_block_begin[2*i] <= queue_block_begin_even_w; 
      end
    if(write_odd[i])
      begin
       queue_block_begin[2*i+1] <= queue_block_begin_odd_w; 
      end
 end
end

//in_en_brqid
wire [`Qpower-1:0] tail_minus1 = tail - 1'b1; //for queue's tail

wire[`BRQpower-1:0] decbus0_brqid  = (decbus0_bd&q_op_blikely[tail_minus1]|decbus0_block_begin)?brq_tail:brq_tail_minus1; 

wire[`BRQpower-1:0] decbus1_brqid  = (~decbus0_block_begin&decbus1_bd&decbus0_blikely|decbus1_block_begin|
                                       decbus0_block_begin&~(decbus1_bd&decbus0_blikely)) ? brq_tail :
                                      (decbus0_block_begin&decbus1_bd&decbus0_blikely) ? brq_tail_next : brq_tail_minus1; 

wire[`BRQpower-1:0] queue_brqid_even_w = tail[0] ? decbus1_brqid : decbus0_brqid;
wire[`BRQpower-1:0] queue_brqid_odd_w  = tail[0] ? decbus0_brqid : decbus1_brqid;

always @(posedge clock)
begin
  for(i=0;i<8;i=i+1) begin
    if(write_even[i])
      begin
       queue_brqid[2*i] <= queue_brqid_even_w; 
      end
    if(write_odd[i])
      begin
       queue_brqid[2*i+1] <= queue_brqid_odd_w; 
      end
 end
end
//in_en_ac
wire [1:0] queue_ac_even_w = tail[0] ? decbus1_ac: decbus0_ac;
wire [1:0] queue_ac_odd_w  = tail[0] ? decbus0_ac: decbus1_ac;
always @(posedge clock)
begin
  for(i=0;i<8;i=i+1) begin
    if(write_even[i])
      begin
       queue_ac[2*i] <= queue_ac_even_w; 
      end
    if(write_odd[i])
      begin
       queue_ac[2*i+1] <= queue_ac_odd_w; 
      end
 end
end

reg    nmi_r, nmi_1r, nmi_2r;
reg    int_r, int_1r, int_2r;


assign in_en_con  = ({16{resbus0_valid}} & wb0_d)  |
                    ({16{resbus2_valid}} & wb2_d) |
                    ({16{resbus1_valid}} & wb1_d) |
                    ({16{resbus3_valid}} & wb3_d);

always @(posedge clock)
begin
    if (reset)
    begin
        nmi_r  <= 1'b0; 
        nmi_1r <= 1'b0; 
        nmi_2r <= 1'b0; 
        int_r  <= 1'b0; 
        int_1r <= 1'b0; 
        int_2r <= 1'b0; 
    end 
    else
    begin
        nmi_r  <= nmi_now;
        nmi_1r <= nmi_r;
        nmi_2r <= nmi_1r;
        int_r  <= f_int;
        int_1r <= int_r;
        int_2r <= int_1r;
    end
end
assign rt_int_v = {16{(nmi_2r|int_2r)&cr_cfg6_rti_i}} & (s_unissued|s_issued);

assign in_en_ex = ({16{decbus0_valid}} & tail_d ) |
                  ({16{decbus1_valid}} & tail_next_d ) |
                  ({16{resbus0_valid & res0_ex}} & wb0_d) |
                  ({16{resbus1_valid & res1_ex}} & wb1_d) |
                  ({16{resbus3_valid & res3_ex}} & wb3_d) |
                  ({16{resbus2_valid & res2_ex}} & wb2_d) |
                  rt_int_v &  ~clear_br_ex;

assign in_en_excode = ({16{decbus0_valid & dec0_ex}} & tail_d ) |
                      ({16{decbus1_valid & dec1_ex}} & tail_next_d ) |
                      ({16{resbus0_valid & res0_ex}} & wb0_d)  |
                      ({16{resbus1_valid & res1_ex}} & wb1_d ) |
                      ({16{resbus3_valid & res3_ex}} & wb3_d ) |
                      ({16{resbus2_valid & res2_ex}} & wb2_d ) |
                      rt_int_v & ~clear_br_ex;                    

//HAVE_DSP_UNIT
assign in_en_dspctl = ({16{resbus1_valid & resbus1_write_dspctl}} & wb1_d ) |
                      ({16{resbus3_valid & resbus3_write_dspctl}} & wb3_d ); 

queue_decode_eq_4_16 decode_brbus_qid(.in(brbus_qid),.out(brbus_qid_d));

wire queue_imm_hi_gating_clock;
assign queue_imm_hi_gating_clock = clock;
wire queue_imm_low_gating_clock;
assign queue_imm_low_gating_clock = clock;

always @ (posedge clock)
begin 	   
  for (i=0;i<`Qsize;i=i+1) 
    begin
       if (in_en_rdy1[i]) //enter from decbus
          begin
          queue_rdy1[i]<= ((commitbus_valid_0 && commit0_eq_qid1[i]&s_unissued[i]) 
                            ||(commitbus_valid_1 && commit1_eq_qid1[i]&s_unissued[i])
                            ) ? 1'b1 :
                           (decbus0_valid && tail_d[i]) ? decbus0_rdy1 : decbus1_rdy1;

          end
       if (in_en_rdy2[i]) //enter from decbus
          begin
          queue_rdy2[i]<= ((commitbus_valid_0 && commit0_eq_qid2[i]&s_unissued[i]) 
                         ||(commitbus_valid_1 && commit1_eq_qid2[i]&s_unissued[i])
                            ) ? 1'b1 :
                           (decbus0_valid && tail_d[i]) ? decbus0_rdy2 : decbus1_rdy2;
          end
    end
end   

always @ (posedge queue_imm_hi_gating_clock)
begin 	   
  for (i=0;i<`Qsize;i=i+1) 
    begin 
      if (in_en_imm_hi[i])
        begin
          queue_imm_hi[i]<= ({16{(decbus0_imm_hi_en && tail_d[i])}} & decbus0_imm[31:16])  |
                            ({16{(decbus1_imm_hi_en && tail_next_d[i])}} & decbus1_imm[31:16])  |
                            ({16{(resbus0_valid && wb0_d[i] )}} & resbus0_value[31:16]) |
                            ({16{(resbus1_valid && wb1_d[i] )}} & resbus1_value[31:16]) |
                            ({16{(resbus3_valid && wb3_d[i] )}} & resbus3_value[31:16] &{16{~resbus3_acc_op}});
        end
    end
end

always @ (posedge queue_imm_low_gating_clock)
begin 	   
  for (i=0;i<`Qsize;i=i+1) 
    begin 
      if (in_en_imm_low[i])
        begin
          queue_imm_low[i]<=({16{(decbus0_imm_low_en && tail_d[i])}} & decbus0_imm[15:0])  |
                            ({16{(decbus1_imm_low_en && tail_next_d[i])}} & decbus1_imm[15:0])  |
                            ({16{(resbus0_valid && wb0_d[i] )}} & resbus0_value[15:0]) |
                            ({16{(resbus1_valid && wb1_d[i] )}} & resbus1_value[15:0]) |
                            ({16{(resbus3_valid && wb3_d[i] )}} & resbus3_value[15:0] &{16{~resbus3_acc_op}});
        end
    end
end

always @ (posedge clock)
begin 	   
  for (i=0;i<`Qsize;i=i+1) 
    begin 
      if (in_en_wb[i])
        begin 
           queue_wb[i]<=~((decbus0_valid && tail_d[i] && ~decbus0_nop && ~dec0_ex) 
                           |
                          (decbus1_valid && tail_next_d[i] && ~decbus1_nop && ~dec1_ex)
                           )
                           ;
        end
      
      if (in_en_con[i])
        begin
          queue_con_true[i]<=((resbus0_valid && wb0_d[i]) & resbus0_con_true) |
                             ((resbus2_valid && wb2_d[i]) & resbus2_con_true) |
                             ((resbus1_valid && wb1_d[i]) & resbus1_con_true)
                             |((resbus3_valid && wb3_d[i]) & resbus3_con_true)
                             ;
        end 

//HAVE_DSP_UNIT
      if (in_en_dspctl[i])
        begin
          queue_dspctl[i]<=({32{(resbus1_valid && wb1_d[i] && resbus1_write_dspctl)}} & resbus1_dspctl) |
                           ({32{(resbus3_valid && wb3_d[i] && resbus3_write_dspctl)}} & resbus3_dspctl); 
        end 
      if (in_en_ex[i])
        begin
          queue_ex[i]     <= (decbus0_valid && tail_d[i] && dec0_ex) ||
                             (decbus1_valid && tail_next_d[i] && dec1_ex) ||
                             (resbus0_valid && wb0_d[i]) ||
                             (resbus1_valid && wb1_d[i]) ||
                             (resbus3_valid && wb3_d[i]) ||
                             (resbus2_valid && wb2_d[i]) ||
                             (rt_int_v[i] && ~clear_br_ex[i]);
        end
     if (in_en_excode[i])
        begin
         queue_excode[i] <= (cr_cfg6_rti_i & nmi_2r & ~clear_br_ex[i] & (s_unissued[i]|s_issued[i]))?`EX_NMI :
                             (cr_cfg6_rti_i & int_2r & ~clear_br_ex[i] & (s_unissued[i]|s_issued[i]))?`EX_INTERRUPT :
                             (({6{(decbus0_valid && tail_d[i])}} & dec0_excode) |
                             ({6{(decbus1_valid && tail_next_d[i])}} & dec1_excode) |
                             ({6{(resbus0_valid && wb0_d[i])}} & res0_excode) |
                             ({6{(resbus1_valid && wb1_d[i])}} & res1_excode) |
                             ({6{(resbus3_valid && wb3_d[i])}} & res3_excode) |
                             ({6{(resbus2_valid && wb2_d[i])}} & res2_excode));
        end
      if (in_en_state[i]) 
        begin
         queue_state[i] <= reset ? `EMPTY :
                           (commitbus_ex_0 | commitbus_ex_1 |
                            (commitbus_valid_0 & head_d[i]) | (commitbus_valid_1 & head_next_d[i]) |
                            (brbus_valid & brbus_brerror & brbus_vector[i])) ? `EMPTY : 
                           ((decbus0_valid & tail_d[i] & (decbus0_nop | dec0_ex)) | 
                            (decbus1_valid & tail_next_d[i] & (decbus1_nop | dec1_ex)) |
                            (qissuebus0_valid & issue_0_d[i]) | (qissuebus1_valid & issue_1_d[i]) | rt_int_v[i] 
                           ) ? `ISSUED : 
                           ((decbus0_valid & tail_d[i]) | (decbus1_valid & tail_next_d[i])
                           ) ? `UNISSUED : queue_state[i];
        end
    end /*for*/
 

end /*always*/
/*reg for jalr target*/
/* only one jalr can issued to alu. jalr stall issue
wire qid1_lt_qid3 = (resbus3_qid > resbus1_qid) & (head >= resbus3_qid | head <= resbus1_qid) | 
                    (resbus3_qid < resbus1_qid) &  head > resbus3_qid  & head <= resbus1_qid;
*/
/*
always @(posedge clock)
begin
   if(reset) 
      jalr_target_reg <= 32'b0;
   else if   (resbus1_valid & q_op_jalr[resbus1_qid]  & ~(resbus3_valid & q_op_jalr[resbus3_qid]))
      jalr_target_reg <= alures_jalr_target1;
   else if (~(resbus1_valid & q_op_jalr[resbus1_qid]) &   resbus3_valid & q_op_jalr[resbus3_qid])
      jalr_target_reg <= alures_jalr_target2;
   else if (resbus1_valid & q_op_jalr[resbus1_qid] & resbus3_valid & q_op_jalr[resbus3_qid] & qid1_lt_qid3)
      jalr_target_reg <= alures_jalr_target1;
   else if (resbus1_valid & q_op_jalr[resbus1_qid] & resbus3_valid & q_op_jalr[resbus3_qid] & ~qid1_lt_qid3)
      jalr_target_reg <= alures_jalr_target2;
end
*/
always @(posedge clock)
begin
   if(reset) 
      jalr_target_reg <= 32'b0;
   else if (resbus1_valid & q_op_jalr[resbus1_qid])
      jalr_target_reg <= alures_jalr_target1;
   else if (resbus3_valid & q_op_jalr[resbus3_qid])
      jalr_target_reg <= alures_jalr_target2;
end
//------------------------------outputs-----------------------------------------------------------------------------
assign EJTAGBOOT=EJTAGBRK&&reset_delay1;

assign qfull[1]   = &full; 
assign qfull[0]   = (tail_next==head)?1'b1:1'b0; //1'b0: 2 empty entries
wire   qstall_cp0 = (|stalli);
wire   qstall_dss = (|q_exestep)&&(~DEBUG_MODE)&&DSS_ENABLE;
assign qstalli    = qstall_cp0||qstall_dss;

wire [`Qsize-1:0] acc_wait_tmp;
assign      acc_wait_tmp = s_unissued | s_issued | queue_ex;
wire [`Qsize-1:0] may_roll_back_mem = (cr_cfg6_rti_i) ? acc_wait_tmp : store_wait;
wire [`Qsize-1:0] may_roll_back_acc = (cr_cfg6_rti_i) ? acc_wait_tmp : acc_wait;

/**************** store_ok generation ****************************/
wire [`Qsize-1:0] head_ge_d,store_qid_ge_d_0,store_qid_ge_d_1,acc_qid_ge_d;
wire [`Qsize-1:0] store_between_ge_0,store_between_lt_0;
wire [`Qsize-1:0] store_between_ge_1,store_between_lt_1;
wire [`Qsize-1:0] store_between_0;
wire [`Qsize-1:0] store_between_1;
wire [`Qsize-1:0] acc_between_ge,acc_between_lt;
wire [`Qsize-1:0] acc_between;

queue_decode_ge_4_16 ge416_head(.in(head),.out(head_ge_d));
queue_decode_ge_4_16 ge416_stqid_0(.in(store_qid[3:0]),.out(store_qid_ge_d_0));
queue_decode_ge_4_16 ge416_stqid_1(.in(store_qid[7:4]),.out(store_qid_ge_d_1));
queue_decode_ge_4_16 ge416_accqid(.in(acc_qid[`Qpower-1:0]),.out(acc_qid_ge_d));

assign store_between_ge_0 = head_ge_d&(~store_qid_ge_d_0);          
assign store_between_lt_0 = head_ge_d|(~store_qid_ge_d_0);     
assign store_between_0    = (store_qid[3:0]>=head)? store_between_ge_0 : store_between_lt_0; 

wire f_int_stall = f_int   | int_r | int_1r | int_2r;
wire nmi_stall   = nmi_now | nmi_r | nmi_1r | nmi_2r;
assign store_ok[0]   = ~(|(store_between_0&may_roll_back_mem)) & ~((nmi_stall | f_int_stall) & cr_cfg6_rti_i);

assign store_between_ge_1 = head_ge_d&(~store_qid_ge_d_1);          
assign store_between_lt_1 = head_ge_d|(~store_qid_ge_d_1);     
assign store_between_1 = (store_qid[7:4]>=head)? store_between_ge_1 : store_between_lt_1; 

assign store_ok[1]   = ~(|(store_between_1&may_roll_back_mem)) & ~((nmi_stall | f_int_stall) & cr_cfg6_rti_i);

assign acc_between_ge = head_ge_d&(~acc_qid_ge_d);      
assign acc_between_lt = head_ge_d|(~acc_qid_ge_d);   
assign acc_between = (acc_qid[`Qpower-1:0]>=head)? acc_between_ge : acc_between_lt; 

//assign acc_write_ok   = ~(|(acc_between&may_roll_back_acc)) & ~((nmi_now |f_int) & cr_cfg6_rti_i);
wire acc_write_ok_tmp = ~(|(acc_between&acc_wait_tmp));
assign acc_write_ok =cr_cfg6_rti_i ? acc_write_ok_tmp & ~(nmi_stall | f_int_stall) : ~(|(acc_between & acc_wait));

/**************** cp0_mod_ok generation ****************************/
wire [`Qsize-1:0] cp0_mod_qid_ge_d;
wire [`Qsize-1:0] cp0_mod_between_ge, cp0_mod_between_lt;
wire [`Qsize-1:0] cp0_mod_between;

queue_decode_ge_4_16 u1_ge416_cp0qid(.in(cp0_qid[`Qpower-1:0]), .out(cp0_mod_qid_ge_d));

assign cp0_mod_between_ge = head_ge_d & (~cp0_mod_qid_ge_d);
assign cp0_mod_between_lt = head_ge_d | (~cp0_mod_qid_ge_d);
assign cp0_mod_between    = (cp0_qid[`Qpower-1:0]>=head) ? cp0_mod_between_ge :
                                              cp0_mod_between_lt;
assign cp0_mod_ok = ~(|(cp0_mod_between&may_roll_back_mem)) & ~((nmi_stall | f_int_stall) & cr_cfg6_rti_i);
/**************** commitbus ************/
assign commitbus0[175:174] = commitbus_ac_0; 
//HAVE_DSP_UNIT
assign commitbus0[173:142] = commitbus_dspctl_0; 
assign commitbus0[141:140] = commitbus_other_link_0; 
assign commitbus0[139:138] = commitbus_rashead_0; 
assign commitbus0[137:130] = commitbus_gshare_0;
assign commitbus0[129]     = commitbus_con_true_0;
assign commitbus0[128:127] = commitbus_old_status_0;// prediction status for the commit common branch instruction
assign commitbus0[126:95]  = commitbus_taken_target_0; //taken target addr for the commit common branch instruction
assign commitbus0[94]      = commitbus_valid_0;
assign commitbus0[93:90]   = commitbus_qid_0;
assign commitbus0[89:82]   = commitbus_op_0;
assign commitbus0[81:74]   = commitbus_dest_0;
assign commitbus0[73:42]   = commitbus_value_0;   //jr target address
assign commitbus0[41:10]   = commitbus_pc_0; //pc
assign commitbus0[9:4]     = commitbus_excode_0;
assign commitbus0[3]       = commitbus_bd_0;
assign commitbus0[2:1]     = commitbus_ce_0;
assign commitbus0[0]       = commitbus_ex_0;

assign commitbus1[175:174] = commitbus_ac_1; 
// HAVE_DSP_UNIT
assign commitbus1[173:142] = commitbus_dspctl_1; 
assign commitbus1[141:140] = commitbus_other_link_1; 
assign commitbus1[139:138] = commitbus_rashead_1; 
assign commitbus1[137:130] = commitbus_gshare_1;
assign commitbus1[129]     = commitbus_con_true_1;
assign commitbus1[128:127] = commitbus_old_status_1;// prediction status for the commit common branch instruction
assign commitbus1[126:95]  = commitbus_taken_target_1; //taken target addr for the commit common branch instruction
assign commitbus1[94]      = commitbus_valid_1;
assign commitbus1[93:90]   = commitbus_qid_1;
assign commitbus1[89:82]   = commitbus_op_1;
assign commitbus1[81:74]   = commitbus_dest_1;
assign commitbus1[73:42]   = commitbus_value_1;   //jr target address
assign commitbus1[41:10]   = commitbus_pc_1; //pc
assign commitbus1[9:4]     = commitbus_excode_1;
assign commitbus1[3]       = commitbus_bd_1;
assign commitbus1[2:1]     = commitbus_ce_1;
assign commitbus1[0]       = commitbus_ex_1;

wire brbus_config_br;

assign brbus_jr31      = brbus_valid & (brbus_op == `OP_JR) & (queue_src1[brbus_qid]== 8'b00011111) ; 
assign brbus_config = brbus_valid & brbus_config_br; 

BR_CONFIG_OP op_test0(.op(brbus_op), .br_op(brbus_config_br) );

/***************************bits are changed*****************************/

assign src3_to_alu[9:0]   = (qissuebus0_op == `OP_INS) ? qissuebus0_res1[9:0] : qissuebus1_res1[9:0];
assign src3_to_alu[10]    = qissuebus0_valid&(qissuebus0_op==`OP_INS) | qissuebus1_valid&(qissuebus1_op==`OP_INS); 

//--------------------branch_queue operation--------------------------------------------//
wire res0_pred_err,  resbus0_brtaken;
wire res1_pred_err,  resbus1_brtaken;
wire res2_pred_err,  resbus2_brtaken;
wire res3_pred_err,  resbus3_brtaken;

wire[`BRQpower-1:0] brbus_brqid_next;

assign brq_full = &brq_valid;
assign brq_tail_next_valid = brq_valid[brq_tail_next];

assign resbus0_brqid = queue_brqid[resbus0_qid];
assign resbus1_brqid = queue_brqid[resbus1_qid];
assign resbus2_brqid = queue_brqid[resbus2_qid];
assign resbus3_brqid = queue_brqid[resbus3_qid];

brq_index_minus1 tail_minus1_m(.in(brq_tail),.out(brq_tail_minus1));
brq_decode_eq_3_6 brq_tail_minus1_decode (.in(brq_tail_minus1), .out(brq_tail_minus1_d));
brq_decode_eq_3_6 brq_tail_decode (.in(brq_tail), .out(brq_tail_d));
brq_decode_eq_3_6 brq_head_decode (.in(brq_head), .out(brq_head_d));
brq_next brq_tail_next_m(.in(brq_tail),.next(brq_tail_next));
queue_encode_eq_6_3 encode_brqid_1(.in({brbus_brqid_d[4:0], brbus_brqid_d[5]}), .out(brbus_brqid_next));

brq_decode_eq_3_6 resbus0_brqid_decode (.in(resbus0_brqid), .out(resbus0_brqid_d));
brq_decode_eq_3_6 resbus1_brqid_decode (.in(resbus1_brqid), .out(resbus1_brqid_d));
brq_decode_eq_3_6 resbus2_brqid_decode (.in(resbus2_brqid), .out(resbus2_brqid_d));
brq_decode_eq_3_6 resbus3_brqid_decode (.in(resbus3_brqid), .out(resbus3_brqid_d));
//if resbus1_bnt is 1 and the instruction is branch for both common branch and blikely), the branch is  not taken;
// but if resbus1_bnt is 1 the instruction is JR, eret or deret, the instruction is taken
//so the res1_bnt express static branch prediction is wrong in the old version 
//for JR31, BNT not express prediction error, the prediction of JR31 is taken
wire[1:0] pred_status1  = brq_status[resbus1_brqid];
wire[1:0] pred_status2  = brq_status[resbus2_brqid];
wire[1:0] pred_status3  = brq_status[resbus3_brqid];

wire res1_static_br = q_op_static_br[resbus1_qid]; //branch likely instruction
wire res3_static_br = q_op_static_br[resbus3_qid];
wire res2_static_br = q_op_static_br[resbus2_qid];

assign res0_pred_err = resbus0_bnt;
assign resbus0_brtaken = resbus0_bnt; //for eret and deret  // not used
assign res1_pred_err = (wb1_br & ~res1_static_br) ? 
                       (resbus1_bnt& pred_status1[1] | ~resbus1_bnt & ~pred_status1[1]) : resbus1_bnt;
assign resbus1_brtaken = wb1_br ? ~resbus1_bnt : 1'b1;
assign res2_pred_err = (wb2_br & ~res2_static_br) ?
                       (resbus2_bnt& pred_status2[1] | ~resbus2_bnt & ~pred_status2[1]) : resbus2_bnt;
assign resbus2_brtaken = wb2_br ? ~resbus2_bnt : 1'b1;
assign res3_pred_err = (wb3_br & ~res3_static_br) ? 
                       (resbus3_bnt& pred_status3[1] | ~resbus3_bnt & ~pred_status3[1]) : resbus3_bnt;
assign resbus3_brtaken = wb3_br ? ~resbus3_bnt : 1'b1;


assign in_en_brq_rashead = {6{decbus0_valid&decbus0_block_begin | decbus1_valid&decbus1_block_begin}}&brq_tail_d;

assign in_en_brq_status  = {6{decbus0_valid&decbus0_block_end&~decbus0_block_begin|
                              decbus1_valid&decbus1_block_end&~decbus0_block_begin&~decbus1_block_begin
                              }}&brq_tail_minus1_d |
                           {6{decbus0_valid&decbus0_block_end&decbus0_block_begin|
                              decbus1_valid&decbus1_block_end&decbus1_block_begin|
                              decbus1_valid&decbus1_block_end&decbus0_block_begin}}&brq_tail_d;
wire[`BRQpower-1:0] start_brqid = brbus_brqid;// valid wb ~resolved bdrdy
wire[`BRQsize-1:0] start_vec;
wire[`BRQsize-1:0] tail_vec;
wire[`BRQsize-1:0] head_vec;

brq_start_vector brq_start_vec_0(.start_brqid(start_brqid),.vec(start_vec));
brq_tail_vector  brq_tail_vec_0(.brqid(brq_tail),.vec(tail_vec));
brq_head_vector  brq_head_vec_0(.brqid(brq_head),.vec(head_vec));

assign cancel_brq_vector = (~brq_full) ? 
                           ((start_brqid <brq_tail)  ? (start_vec & tail_vec):
                            (start_brqid >brq_tail)  ? (start_vec | tail_vec): 1'b0) :
                           ((start_brqid >=brq_head) ? (start_vec | head_vec):(start_vec & head_vec));

reg [3:0] offset;
wire[3:0] offset_plus1, offset_plus2;
brq_offset_next offset1 (.in(offset), .next(offset_plus1));
brq_offset_next offset2 (.in(offset_plus1), .next(offset_plus2));

always @(posedge clock)
begin
if(reset)begin
 brq_wb <= 1'b0;
 brq_valid <= 1'b0;
 brq_resolved <= 6'b111111;
 brq_brerror <= 1'b0;
 brq_bdrdy <= 1'b0;
end
else begin
for (i=0;i<`BRQsize;i=i+1) 
begin

if (in_en_brq_rashead[i])
   begin
     brq_rashead[i] <= decbus0_valid&decbus0_block_begin ? decbus0_rashead: decbus1_rashead;
     brq_gshare[i]  <= decbus0_valid&decbus0_block_begin ? decbus0_gshare : decbus1_gshare;
     brq_pc[i]      <= decbus0_valid&decbus0_block_begin ? decbus0_pc     : decbus1_pc;
   end
 
 /*brq_qid is the queue qid of the branch instruction in the block;
   brq_offset is the offset between the block-begin instruction and the branch instruction in the block */ 
  if(in_en_brq_status[i])
   begin
     brq_status [i]  <= decbus0_valid&decbus0_block_end   ? decbus0_ce     : decbus1_ce; 
     brq_op [i]      <= decbus0_valid&decbus0_block_end   ? decbus0_op     : decbus1_op; 
     brq_qid    [i]  <= decbus0_valid&decbus0_block_end   ? tail           : tail_next; 
     brq_offset [i]  <= decbus0_valid&decbus0_block_end   ?(decbus0_block_begin ? 4'b0: offset_plus1) :
                       (decbus0_block_begin ? 4'b1: decbus1_block_begin ? 4'b0: offset_plus2); 
   end
  
  if(in_en_brq_bdrdy[i])
     brq_bdrdy[i]  <=  decbus0_valid&(decbus0_bd|decbus0_eret)&~decbus0_block_begin&brq_tail_minus1_d[i] | 
                       decbus1_valid&(decbus1_bd|decbus1_eret)&~decbus0_block_begin&~decbus1_block_begin&
                       brq_tail_minus1_d[i] |
                       decbus0_valid&decbus0_eret&decbus0_block_begin&brq_tail_d[i] |
                       decbus1_valid&(decbus1_bd|decbus1_eret)&(decbus0_block_begin|decbus1_block_begin)&
                       brq_tail_d[i];
 
 if(in_en_brq_wb[i])
  begin
    brq_wb[i]     <= ~(decbus0_valid&decbus0_block_begin&brq_tail_d[i] |
                        decbus1_valid&decbus1_block_begin&brq_tail_d[i])  ;
    brq_brerror[i]  <= resbus0_brqid_d[i]&resbus0_eret_deret                 ? 1'b1: 
                        resbus1_brqid_d[i]&resbus1_valid&resbus1_block_end    ? res1_pred_err:
                        resbus3_brqid_d[i]&resbus3_valid&resbus3_block_end    ? res3_pred_err:
                        resbus2_brqid_d[i]&resbus2_valid&resbus2_block_end    ? res2_pred_err: 1'b0;
     brq_brtaken[i]  <= resbus0_brqid_d[i]&resbus0_eret_deret                 ? 1'b1:
                        resbus1_brqid_d[i]&resbus1_valid&resbus1_block_end    ? resbus1_brtaken:
                        resbus3_brqid_d[i]&resbus3_valid&resbus3_block_end    ? resbus3_brtaken:
                        resbus2_brqid_d[i]&resbus2_valid&resbus2_block_end    ? resbus2_brtaken: 1'b0;
  end

 if(in_en_brq_resolved[i])
   begin
     brq_resolved[i] <= ~(decbus0_valid&decbus0_block_begin&brq_tail_d[i] |
                          decbus1_valid&decbus1_block_begin&brq_tail_d[i]);
   end

  if(in_en_brq_valid[i])
     brq_valid[i]<=(commitbus_ex|reset) ? 1'b0:
                   (brbus_brerror&cancel_brq_vector[i]) ? 1'b0:
                   (brq_tail_d[i]&(decbus0_valid&decbus0_block_begin|decbus1_valid&decbus1_block_begin))?1'b1:1'b0; 
end
end
end

always @(posedge clock)
begin
if(reset | commitbus_ex)
  begin
   brq_tail <= 1'b0;
  end
else if (decbus0_valid&decbus0_block_begin | decbus1_valid&decbus1_block_begin)
   brq_tail <= brq_tail_next;
else if (brbus_valid & brbus_brerror)
   brq_tail <= brbus_brqid_next ;
end

wire[`BRQpower-1:0] brq_head_next_t;
assign brq_head_next_t = commit1_one_block&commit0_last_inst ?  commitbus_brqid_1 : brq_head;

brq_next brq_head_next_m(.in(brq_head_next_t),.next(brq_head_next));

always @(posedge clock)
begin
if(reset | commitbus_ex)
  begin
   brq_head <= 1'b0;
  end
else if (commitbus_valid_0 & commitbus_bd_0 |
         commitbus_valid_0 & commit_blikely_0 & ~brq_brtaken[commitbus_brqid_0] |
         commitbus_valid_0 & commit_eret_0 
        |commitbus_valid_1 & commitbus_bd_1 |
         commitbus_valid_1 & commit_blikely_1 & ~brq_brtaken[commitbus_brqid_1] |
         commitbus_valid_1 & commit_eret_1
         )
   brq_head <= brq_head_next;
end

always @ (posedge clock)
begin
if (reset)
  offset <= 4'b0;
else if(decbus1_valid & decbus1_block_begin)
  offset <= 4'b0;
else if(decbus0_valid&decbus0_block_begin& ~decbus1_valid)
  offset <= 4'b0;
else if(decbus0_valid&decbus0_block_begin& decbus1_valid)
  offset <= 4'b1;
else if(decbus0_valid & decbus1_valid)
  offset <= offset_plus2;
else if(decbus0_valid &~decbus1_valid)
  offset <= offset_plus1;
end

wire [31:0] commit_pc_t;
wire [3:0]  commit_pc_offset;
assign commit_pc_t      = (commitbus_valid_1&commitbus_block_begin_1) ? brq_pc[commitbus_brqid_1]:
                          (commitbus_valid_0&commitbus_block_begin_0) ? brq_pc[commitbus_brqid_0]: commit_pc;
assign commit_pc_offset = (commitbus_valid_1&commitbus_block_begin_1) ? 4'b0100:  //pc+4
                           commitbus_valid_1                          ? 4'b1000:  //pc+8
                           4'b0100;

always @(posedge clock) //commit_pc only record basic block's sequential pc, according to commtibus_value_h
begin
  if (reset)
    commit_pc <= 32'b0;
  else if (decbus0_valid & (&s_empty)  )
    commit_pc <= decbus0_pc;
  else if (commitbus_valid_0 
         | commitbus_valid_1
         ) 
    commit_pc <=commit_pc_t + commit_pc_offset;
end
//----------------------------------branch_queue end ---------------------//
//the test of HI_LOW register

assign flush_pipeline_cycle_o = commitbus_ex | brbus_brerror;
assign inst_to_queue_cycle_o = decbus0_valid | decbus1_valid;

assign insts_to_alu1_o = qissuebus0_valid & (qissuebus0_rs==2'b01) | qissuebus1_valid & (qissuebus1_rs==2'b01) ;

assign insts_to_alu2_o = qissuebus0_valid & (qissuebus0_rs==2'b10) | qissuebus1_valid & (qissuebus1_rs==2'b10) ;
assign insts_to_addr_o = qissuebus0_valid & (qissuebus0_rs==2'b00) | qissuebus1_valid & (qissuebus1_rs==2'b00) ;
assign insts_to_falu_o = qissuebus0_valid & (qissuebus0_rs==2'b11) | qissuebus1_valid & (qissuebus1_rs==2'b11) ;

assign insts_to_queue_o = decbus0_valid + decbus1_valid;

endmodule  //godson_queue_module

module queue_decode(q_state,q_op,q_wb,q_ex,s_empty,
                    s_unissued,s_issued,s_wb,
                    full,stalli,store_wait,
                    ctc1,eret,branch,jalr,
                    blikely,br,q_exestep,static_br,
                    wait_operation,mm_operation,falu_operation,
                    div_op,acc_op,acc_wait, mm_is_falu);
input  [1:0] q_state;
input  [7:0] q_op;
input        q_wb;
input        q_ex;
output       s_empty,s_unissued,s_issued,s_wb;
output       full;
output       stalli;
output       store_wait;
output       ctc1;
output       jalr;
output       eret;
output       branch;
output       blikely;
output       static_br;
output       br;
output       q_exestep;
output       wait_operation;
output       mm_operation;
output       falu_operation;
output       div_op;
output       acc_op;
output       acc_wait;
output       mm_is_falu;

FALU_ON_ADDR falu_on_addr__tmp(.op(q_op), .falu_addr(mm_is_falu));

wire         stall_cp0;
wire         store_wait_ex;

assign s_empty    = (!q_state[1])&&(!q_state[0]);
assign s_unissued = (!q_state[1])&&( q_state[0]);
assign s_issued   = ( q_state[1])&&( q_state[0])&&(!q_wb);
assign s_wb       = ( q_state[1])&&( q_state[0])&&( q_wb);

STALL_FETCH_OP stall_fetch_op_u1(.op(q_op),.out(stall_cp0));

assign full   = (!s_empty);
assign stalli = ((s_unissued||s_issued)&&(stall_cp0));

assign eret = (q_op==`OP_ERET)||(q_op==`OP_DERET);

//stall issue
ctc1_operate  ctc1_operate_0(.op(q_op), .ctc1(ctc1));

assign jalr = (q_op == `OP_JALR);

assign branch = (q_op[7:2] == 6'h0b) | (q_op[7:4] == 4'h3) | (q_op[7:2]== 6'h1a)  
                //HAVE_DSP_UNIT
                |(q_op==`OP_BPOSGE32)
                ;

BLIKELY_OP         qdecode_blikely_op(.op(q_op),.blikely_op(blikely));//likely branches
STATIC_PREDICT_OP  qdecode_static_br (.op(q_op),.static_pred_op(static_br));//likely and link branches
BR_OP              qdecode_br_op     (.op(q_op),.br_op(br));//all branches except jump

assign store_wait_ex = (s_issued || s_unissued) && 
                       ((q_op==`OP_ADD) || (q_op==`OP_SUB) ||
                        (q_op==`OP_TEQ) || (q_op==`OP_TNE) ||
                        (q_op==`OP_TLT) || (q_op==`OP_TLTU)||
                        (q_op==`OP_TGE) || (q_op==`OP_TGEU)||
                        (q_op[7:6]==2'b01) || branch);

assign store_wait  = store_wait_ex||q_ex;

wire acc_wait_ex;
/*
assign acc_wait_ex = (s_issued || s_unissued) && 
                     ((q_op==`OP_ADD) || (q_op==`OP_SUB) || (q_op==`OP_TEQ) || (q_op==`OP_TNE) ||
                     (q_op==`OP_TLT) || (q_op==`OP_TLTU)|| (q_op==`OP_TGE) || (q_op==`OP_TGEU)|| (q_op[7:6]==2'b01)
                   ||(q_op[7:6]==2'b10) & (q_op!=`OP_MFC1) & (q_op!=`OP_CFC1) & (q_op!=`OP_MTC1) & (q_op!=`OP_CTC1) &
                     (q_op!=`OP_DI) & (q_op!=`OP_EI) & (q_op!=`OP_TLBP) & (q_op!=`OP_TLBR) & (q_op!=`OP_TLBWI) & 
                     (q_op!=`OP_TLBWR) & (q_op!=`OP_MFC0) & (q_op!=`OP_MTC0) & (q_op!=`OP_PREF) || branch);
*/
assign acc_wait_ex = (s_issued | s_unissued) & 
                     (q_op[7:2]==6'h6 & ~q_op[0] | q_op[7:3]==5'h4 & ~(q_op[2] & q_op[1]) | (q_op[7:6]==2'b01) |
                     (q_op[7:6]==2'b10) & (q_op[7:4]!=4'h8 | ~q_op[2] | ~q_op[1]) & (q_op!=`OP_PREF) | branch);

assign acc_wait = acc_wait_ex || q_ex;

assign q_exestep   = full&&(~branch);

wait_operate wait_operate_0(.op(q_op), .vector(wait_operation));
MM_OP mm_op_0(.op(q_op), .mm_op(mm_operation));
FALU_OP falu_op_3(.op(q_op), .falu_op(falu_operation));
assign div_op = (q_op==`OP_DIV) | (q_op==`OP_DIVU);
ALU_MUL_OP acc_op_0 (.op(q_op), .mul_op(acc_op));

endmodule

module queue_decode_ge_4_16(in,out);
input  [3:0] in;
output [15:0] out; 

assign out[15] = 1'b1;
assign out[14] = (!in[3]) | (!in[2]) | (!in[1]) | (!in[0]);
assign out[13] = (!in[3]) | (!in[2]) | (!in[1]);
assign out[12] = (!in[3]) | (!in[2]) | (!in[1]) & (!in[0]);
assign out[11] = (!in[3]) | (!in[2]);
assign out[10] = (!in[3]) | (!in[2]) & ((!in[1]) | (!in[0])); 
assign out[9] = (!in[3]) | (!in[2]) & (!in[1]);
assign out[8] = (!in[3]) | (!in[2]) & (!in[1]) & (!in[0]);
assign out[7] = (!in[3]);
assign out[6] = (!in[3]) & ((!in[2]) | (!in[1]) | (!in[0]));
assign out[5] = (!in[3]) & ((!in[2]) | (!in[1]));
assign out[4] = (!in[3]) & ((!in[2]) | (!in[1]) & (!in[0]));
assign out[3] = (!in[3]) & (!in[2]);
assign out[2] = (!in[3]) & (!in[2]) & ((!in[1]) | (!in[0]));
assign out[1] = (!in[3]) & (!in[2]) & (!in[1]);
assign out[0] = (!in[3]) & (!in[2]) & (!in[1]) & (!in[0]);
endmodule

module queue_decode_eq_4_16(in,out);
input  [3:0] in;
output [15:0] out;

assign out[0] = !in[3] & !in[2] & !in[1] & !in[0];
assign out[1] = !in[3] & !in[2] & !in[1] &  in[0];
assign out[2] = !in[3] & !in[2] &  in[1] & !in[0];
assign out[3] = !in[3] & !in[2] &  in[1] &  in[0];
assign out[4] = !in[3] &  in[2] & !in[1] & !in[0];
assign out[5] = !in[3] &  in[2] & !in[1] &  in[0];
assign out[6] = !in[3] &  in[2] &  in[1] & !in[0];
assign out[7] = !in[3] &  in[2] &  in[1] &  in[0];
assign out[8] = in[3] & !in[2] & !in[1] & !in[0];
assign out[9] = in[3] & !in[2] & !in[1] &  in[0];
assign out[10] = in[3] & !in[2] &  in[1] & !in[0];
assign out[11] = in[3] & !in[2] &  in[1] &  in[0];
assign out[12] = in[3] &  in[2] & !in[1] & !in[0];
assign out[13] = in[3] &  in[2] & !in[1] &  in[0];
assign out[14] = in[3] &  in[2] &  in[1] & !in[0];
assign out[15] = in[3] &  in[2] &  in[1] &  in[0];

endmodule

module queue_encode_eq_16_4(in,out);
input  [15:0] in;
output [3:0] out;

assign out[0] = in[1] | in[3] | in[5]  | in[7]  | in[9]  | in[11] | in[13] | in[15]; 
assign out[1] = in[2] | in[3] | in[6]  | in[7]  | in[10] | in[11] | in[14] | in[15]; 
assign out[2] = in[4] | in[5] | in[6]  | in[7]  | in[12] | in[13] | in[14] | in[15]; 
assign out[3] = in[8] | in[9] | in[10] | in[11] | in[12] | in[13] | in[14] | in[15]; 

endmodule

module queue_encode_eq_6_3(in,out);
input  [5:0] in;
output [2:0] out;

assign out[0] = in[1] | in[3] | in[5] ; 
assign out[1] = in[2] | in[3]  ; 
assign out[2] = in[4] | in[5]  ; 
endmodule

module queue_rename_encode_no_falu(match_d,mask_lt,match_qid);
input  [`Qsize-1:0] match_d,mask_lt;
output [`Qpower-1:0]        match_qid;

wire   [`Qpower-1:0]        qid_gt,qid_lt;
wire                        less;

wire [`Qsize-1:0] match_d_m;
assign match_d_m   = match_d & mask_lt;

wire [`Qsize-1:0] match16_d = {match_d[0],     match_d[1],    
                         match_d[2],     match_d[3],    
                         match_d[4],     match_d[5],    
                         match_d[6],     match_d[7]
                         ,    
                         match_d[8],     match_d[9],    
                         match_d[10],    match_d[11],   
                         match_d[12],    match_d[13],   
                         match_d[14],    match_d[15]
                         };
wire [`Qsize-1:0] match16_d_m = {match_d_m[0],   match_d_m[1],  
                           match_d_m[2],   match_d_m[3],  
                           match_d_m[4],   match_d_m[5],  
                           match_d_m[6],   match_d_m[7]
                           ,  
                           match_d_m[8],   match_d_m[9],  
                           match_d_m[10],  match_d_m[11], 
                           match_d_m[12],  match_d_m[13], 
                           match_d_m[14],  match_d_m[15]
                           };
first_one_16_4 qencoder_gt_16_4_0(.in(match16_d),  .out(qid_gt), .nonzero());
first_one_16_4 qencoder_gt_16_4_1(.in(match16_d_m),.out(qid_lt), .nonzero());

assign less      = |match_d_m; //less==1'b1:an entry match between tail and queue[0] in op queue
assign match_qid = less ? ~qid_lt : ~qid_gt;  //need a '~', because the bit order in match16_d and match16_d_m

endmodule

module queue_next(in, next);
input[3:0]  in;
output[3:0] next;

reg[3:0] next;

always@(in)
case(in) // synopsys full_case parallel_case
    4'b0000:
        next = 4'b0001;
    4'b0001:
        next = 4'b0010;
    4'b0010:
        next = 4'b0011;
    4'b0011:
        next = 4'b0100;
    4'b0100:
        next = 4'b0101;
    4'b0101:
        next = 4'b0110;
    4'b0110:
        next = 4'b0111;
    4'b0111:
        next = 4'b1000;
    4'b1000:
        next = 4'b1001;
    4'b1001:
        next = 4'b1010;
    4'b1010:
        next = 4'b1011;
    4'b1011:
        next = 4'b1100;
    4'b1100:
        next = 4'b1101;
    4'b1101:
        next = 4'b1110;
    4'b1110:
        next = 4'b1111;
    4'b1111:
        next = 4'b0000;
endcase

endmodule

module STALL_FETCH_OP(op,out);
input [7:0] op;
output out;

 assign out = (op==`OP_CACHE0) || (op==`OP_CACHE8) || (op==`OP_CACHE16) ||
              (op==`OP_CACHE28)||
              (op==`OP_TLBR)   || (op==`OP_ERET)   || (op==`OP_DERET) ||
              (op==`OP_TLBWI)  || (op==`OP_TLBWR)  ||
              (op==`OP_EI)     || (op==`OP_DI)     ||
              (op==`OP_MTC0)   || (op == `OP_LL)   || (op==`OP_SYNCI);

endmodule

module generate_ge_brq_head_vector(in, out);
input  [2:0] in;
output [5:0] out;
assign out[5] = (!in[2]|!in[1]);
assign out[4] = (!in[2]|!in[1]&!in[0]);
assign out[3] = !in[2];
assign out[2] = !in[2]&(!in[1]|!in[0]);
assign out[1] = !in[2]&!in[1];
assign out[0] = !in[2]&!in[1]&!in[0];
// in 0 out 111111
// in 1 out 111110
// in 2 out 111100
endmodule

module brq_decode_eq_3_6(in,out);
input  [2:0] in;
output [5:0] out;

assign out[0] = !in[2] & !in[1] & !in[0];
assign out[1] = !in[2] & !in[1] &  in[0];
assign out[2] = !in[2] &  in[1] & !in[0];
assign out[3] = !in[2] &  in[1] &  in[0];
assign out[4] =  in[2] & !in[1] & !in[0];
assign out[5] =  in[2] & !in[1] &  in[0];

endmodule

module brq_index_minus1(in,out);
input  [2:0] in;
output [2:0] out;

reg[2:0] out;
always @ (in)
begin
case (in)// synopsys full_case parallel_case
  3'b000: 
    out = 3'b101;
  3'b001: 
    out = 3'b000;
  3'b010: 
    out = 3'b001;
  3'b011: 
    out = 3'b010;
  3'b100: 
    out = 3'b011;
  3'b101: 
    out = 3'b100;
  default:
    out = 3'b000;
 endcase
end

endmodule

module brq_next(in, next);
input[2:0]  in;
output[2:0] next;

reg[2:0] next;

always@(in)
case(in) // synopsys full_case parallel_case
    3'b000:
       next = 3'b001;
    3'b001:
       next = 3'b010;
    3'b010:
       next = 3'b011;
    3'b011:
       next = 3'b100;
    3'b100:
       next = 3'b101;
    3'b101:
       next = 3'b000;
    default:
       next = 3'b000;
endcase

endmodule

module brq_offset_next(in, next);
input[3:0]  in;
output[3:0] next;

reg[3:0] next;

always@(in)
case(in) // synopsys full_case parallel_case
    4'b0000:
        next = 4'b0001;
    4'b0001:
        next = 4'b0010;
    4'b0010:
        next = 4'b0011;
    4'b0011:
        next = 4'b0100;
    4'b0100:
        next = 4'b0101;
    4'b0101:
        next = 4'b0110;
    4'b0110:
        next = 4'b0111;
    4'b0111:
        next = 4'b1000;
    4'b1000:
        next = 4'b1001;
    4'b1001:
        next = 4'b1010;
    4'b1010:
        next = 4'b1011;
    4'b1011:
        next = 4'b1100;
    4'b1100:
        next = 4'b1101;
    4'b1101:
        next = 4'b1110;
    4'b1110:
        next = 4'b1111;
    4'b1111:
        next = 4'b1111;
endcase

endmodule
