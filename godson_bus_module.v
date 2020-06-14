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

module godson_rissuebus_module(rissuebus0,
                               rissuebus1,
                               offset_to_mm,
                               rissuebus_to_fix1,
                               rissuebus_to_fix2,
                               rissuebus_to_mm,
                               mm_ok,
                               falu_ok
                              );
 input  [`Lrissuebus0-1:0] rissuebus0;
 input  [`Lrissuebus1-1:0] rissuebus1;
 input  [15:0]  offset_to_mm;
 input mm_ok, falu_ok;
 output [`Lrissuebus_to_fix-1:0] rissuebus_to_fix1;
output [`Lrissuebus_to_fix-1:0] rissuebus_to_fix2;
 output [`Lrissuebus_to_mm-1:0] rissuebus_to_mm;

 wire [1:0]  rissuebus0_ac;   
 wire [1:0]  rissuebus0_rs;   
 wire rissuebus0_valid;       
 wire [3:0]  rissuebus0_qid;
 wire [2:0]  rissuebus0_brqid; 
 wire [7:0]  rissuebus0_op;   
 wire [31:0] rissuebus0_vj;   
 wire [31:0] rissuebus0_vk;   
 wire [7:0]  rissuebus0_src1; 
 wire [7:0]  rissuebus0_src2; 
 wire [7:0]  rissuebus0_dest; 
 wire [31:0] rissuebus0_imm; 

wire rissuebus0_wb2;
wire rissuebus0_wb2_h;
wire rissuebus0_rdy2;
wire rissuebus0_rdy2_h;
wire [3:0] rissuebus0_fpqid2;
wire [3:0] rissuebus0_fpqid2_h;
wire [2:0] rissuebus0_fpqid;

 wire [1:0]  rissuebus1_ac;   
 wire [1:0]  rissuebus1_rs;   
 wire rissuebus1_valid;       
 wire [3:0]  rissuebus1_qid;
 wire [2:0]  rissuebus1_brqid;
 wire [7:0]  rissuebus1_op;   
 wire [31:0] rissuebus1_vj;   
 wire [31:0] rissuebus1_vk;   
 wire [7:0]  rissuebus1_src1; 
 wire [7:0]  rissuebus1_src2; 
  
 assign rissuebus0_dest     = rissuebus0[122:115]; 
 assign rissuebus0_wb2_h    = rissuebus0[114]; 
 assign rissuebus0_wb2      = rissuebus0[113]; 
 assign rissuebus0_fpqid    = rissuebus0[112:110]; 
 assign rissuebus0_fpqid2   = rissuebus0[109:106]; 
 assign rissuebus0_fpqid2_h = rissuebus0[105:102]; 
 assign rissuebus0_rdy2     = rissuebus0[101]; 
 assign rissuebus0_rdy2_h   = rissuebus0[100]; 
 assign rissuebus0_brqid    = rissuebus0[99:97];    
 assign rissuebus0_ac       = rissuebus0[96:95];    
 assign rissuebus0_rs       = rissuebus0[94:93];    
 assign rissuebus0_valid    = rissuebus0[92];     
 assign rissuebus0_qid      = rissuebus0[91:88];    
 assign rissuebus0_op       = rissuebus0[87:80];    
 assign rissuebus0_vj       = rissuebus0[79:48];  
 assign rissuebus0_vk       = rissuebus0[47:16];  
 assign rissuebus0_src1     = rissuebus0[15:8];
 assign rissuebus0_src2     = rissuebus0[7:0];
 assign rissuebus0_imm      = {{16{offset_to_mm[15]}},offset_to_mm};    
  
 assign rissuebus1_brqid = rissuebus1[99:97];    
 assign rissuebus1_ac    = rissuebus1[96:95];    
 assign rissuebus1_rs    = rissuebus1[94:93];    
 assign rissuebus1_valid = rissuebus1[92];     
 assign rissuebus1_qid   = rissuebus1[91:88];    
 assign rissuebus1_op    = rissuebus1[87:80];    
 assign rissuebus1_vj    = rissuebus1[79:48];  
 assign rissuebus1_vk    = rissuebus1[47:16];  
 assign rissuebus1_src1  = rissuebus1[15:8];
 assign rissuebus1_src2  = rissuebus1[7:0];
 
 assign rissuebus_to_fix1[81:79] = rissuebus1_brqid;
 assign rissuebus_to_fix1[78:77] = rissuebus1_ac;
 assign rissuebus_to_fix1[76]    = (rissuebus1_valid & rissuebus1_rs==2'b01);
 assign rissuebus_to_fix1[75:72] = rissuebus1_qid;
 assign rissuebus_to_fix1[71:64] = rissuebus1_op;
 assign rissuebus_to_fix1[63:32] = rissuebus1_vj;
 assign rissuebus_to_fix1[31:0]  = rissuebus1_vk;

assign rissuebus_to_fix2[81:79] = (mm_ok | falu_ok) ? rissuebus1_brqid : rissuebus0_brqid;
assign rissuebus_to_fix2[78:77] = (mm_ok | falu_ok) ? rissuebus1_ac  : rissuebus0_ac;
assign rissuebus_to_fix2[76]    = (rissuebus0_valid & rissuebus0_rs==2'b10) | (rissuebus1_valid & rissuebus1_rs==2'b10);
assign rissuebus_to_fix2[75:72] = (mm_ok | falu_ok) ? rissuebus1_qid : rissuebus0_qid;
assign rissuebus_to_fix2[71:64] = (mm_ok | falu_ok) ? rissuebus1_op  : rissuebus0_op;
assign rissuebus_to_fix2[63:32] = (mm_ok | falu_ok) ? rissuebus1_vj  : rissuebus0_vj;
assign rissuebus_to_fix2[31:0]  = (mm_ok | falu_ok) ? rissuebus1_vk  : rissuebus0_vk;
 
 assign rissuebus_to_mm[136]     = (rissuebus0_dest[7:6]==2'b01) | (rissuebus0_src2[7:6]==2'b01);
 assign rissuebus_to_mm[135]     = rissuebus0_wb2_h;
 assign rissuebus_to_mm[134]     = rissuebus0_wb2;
 assign rissuebus_to_mm[133:102] = rissuebus0_imm;
 assign rissuebus_to_mm[101:99] = rissuebus0_fpqid;
 assign rissuebus_to_mm[98:95]  = rissuebus0_fpqid2;
 assign rissuebus_to_mm[94:91]  = rissuebus0_fpqid2_h;
 assign rissuebus_to_mm[90]     = rissuebus0_rdy2;
 assign rissuebus_to_mm[89]     = rissuebus0_rdy2_h;
 assign rissuebus_to_mm[88]     = (rissuebus0_src2[7:6]==2'b00) ? 1'b1 : 1'b0;
 assign rissuebus_to_mm[87:85]  = rissuebus0_brqid;
 assign rissuebus_to_mm[84]     = (rissuebus0_valid & rissuebus0_rs==2'b00);
 assign rissuebus_to_mm[83:80]  = rissuebus0_qid;
 assign rissuebus_to_mm[79:72]  = rissuebus0_op;
 assign rissuebus_to_mm[71:40]  = rissuebus0_vj; 
 assign rissuebus_to_mm[39:8]   = rissuebus0_vk;
 assign rissuebus_to_mm[7:0]    = rissuebus0_src2;
endmodule 

module godson_qissuebus_module(qissuebus0,
                               qissuebus0_to_gr
			                   ,qissuebus1
                               ,qissuebus1_to_gr
                               );
                              
 input [`Lqissuebus-1:0] qissuebus0;
 output [`Lqissuebus0_to_gr-1:0] qissuebus0_to_gr;

 input  [`Lqissuebus-1:0] qissuebus1;
 output [`Lqissuebus1_to_gr-1:0] qissuebus1_to_gr;
 
 wire [2:0] fpq_tail;
 wire [2:0] qissuebus0_brqid;
 wire qissuebus0_double_src1;
 wire qissuebus0_double_src2;
 wire qissuebus0_double_dest;
 wire [4:0] qissuebus0_fmt;
 wire [1:0] qissuebus0_ac;
 wire [1:0] qissuebus0_rs;
 wire qissuebus0_rdy1;
 wire qissuebus0_rdy2;
 wire qissuebus0_valid;
 wire [3:0] qissuebus0_qid;
 wire [7:0] qissuebus0_op;      
 wire [7:0] qissuebus0_src1;    
 wire [7:0] qissuebus0_src2;    
 wire [7:0] qissuebus0_dest;    
 wire [31:0] qissuebus0_res1;   
 wire [31:0] qissuebus0_res2;   
 wire [3:0] fpq_qid2;   
 wire [3:0] fpq_qid2_h;   
 wire fpq_rdy2;   
 wire fpq_rdy2_h;   
 wire fpq_wb2;   
 wire fpq_wb2_h;   
 
 wire [1:0] qissuebus1_rs;     
 wire qissuebus1_valid;        
 wire [3:0] qissuebus1_qid; 
 wire [2:0] qissuebus1_brqid; 
 wire [7:0] qissuebus1_op;     
 wire qissuebus1_rdy1;   
 wire qissuebus1_rdy2;   
 wire [7:0] qissuebus1_src1;   
 wire [7:0] qissuebus1_src2;   
 wire [7:0] qissuebus1_dest;   
 wire [31:0] qissuebus1_res1;  
 wire [31:0] qissuebus1_res2;  
 wire [4:0] qissuebus1_fmt;    
 wire [1:0] qissuebus1_ac;     
 wire qissuebus1_double_src1;     
 wire qissuebus1_double_src2;     
 wire qissuebus1_double_dest;     

assign qissuebus0_double_src1 = qissuebus0[117];
assign qissuebus0_double_src2 = qissuebus0[116];
assign qissuebus0_double_dest = qissuebus0[115];
assign qissuebus0_fmt         = qissuebus0[114:110];
assign qissuebus0_brqid       = qissuebus0[109:107];
assign qissuebus0_ac          = qissuebus0[106:105];
assign qissuebus0_rs          = qissuebus0[104:103];
assign qissuebus0_rdy1        = qissuebus0[102];
assign qissuebus0_rdy2        = qissuebus0[101];
assign qissuebus0_valid       = qissuebus0[100];
assign qissuebus0_qid         = qissuebus0[99:96];
assign qissuebus0_op          = qissuebus0[95:88];
assign qissuebus0_src1        = qissuebus0[87:80];
assign qissuebus0_src2        = qissuebus0[79:72];
assign qissuebus0_dest        = qissuebus0[71:64];
assign qissuebus0_res1        = qissuebus0[63:32];
assign qissuebus0_res2        = qissuebus0[31:0];
assign fpq_tail               = 3'b0;  
assign fpq_wb2                = 1'b0;  
assign fpq_wb2_h              = 1'b0;  
assign fpq_qid2               = 4'h0;  
assign fpq_qid2_h             = 4'h0;  
assign fpq_rdy2               = 1'b0;  
assign fpq_rdy2_h             = 1'b0;  

 assign qissuebus1_double_src1 = qissuebus1[117];
 assign qissuebus1_double_src2 = qissuebus1[116];
 assign qissuebus1_double_dest = qissuebus1[115];
 assign qissuebus1_fmt    = qissuebus1[114:110];
 assign qissuebus1_brqid  = qissuebus1[109:107];
 assign qissuebus1_ac     = qissuebus1[106:105]; //2'b00:mm, 2'b01:alu, 2'b10:falu
 assign qissuebus1_rs     = qissuebus1[104:103]; //2'b00:mm, 2'b01:alu, 2'b10:falu
 assign qissuebus1_rdy1   = qissuebus1[102];
 assign qissuebus1_rdy2   = qissuebus1[101];
 assign qissuebus1_valid  = qissuebus1[100];
 assign qissuebus1_qid    = qissuebus1[99:96];
 assign qissuebus1_op     = qissuebus1[95:88];
 assign qissuebus1_src1   = qissuebus1[87:80];
 assign qissuebus1_src2   = qissuebus1[79:72];
 assign qissuebus1_dest   = qissuebus1[71:64];
 assign qissuebus1_res1   = qissuebus1[63:32];
 assign qissuebus1_res2   = qissuebus1[31:0];

 assign qissuebus0_to_gr[124:117] = qissuebus0_dest;
 assign qissuebus0_to_gr[116]     = fpq_wb2_h;
 assign qissuebus0_to_gr[115]     = fpq_wb2;
 assign qissuebus0_to_gr[114:112] = fpq_tail;
 assign qissuebus0_to_gr[111:108] = fpq_qid2;
 assign qissuebus0_to_gr[107:104] = fpq_qid2_h;
 assign qissuebus0_to_gr[103]     = fpq_rdy2;
 assign qissuebus0_to_gr[102]     = fpq_rdy2_h;
 assign qissuebus0_to_gr[101]     = qissuebus0_rdy2;
 assign qissuebus0_to_gr[100]     = qissuebus0_rdy1;
 assign qissuebus0_to_gr[99:97]   = qissuebus0_brqid;
 assign qissuebus0_to_gr[96:95]   = qissuebus0_ac;
 assign qissuebus0_to_gr[94:93]   = qissuebus0_rs;
 assign qissuebus0_to_gr[92]      = qissuebus0_valid;
 assign qissuebus0_to_gr[91:88]   = qissuebus0_qid;
 assign qissuebus0_to_gr[87:80]   = qissuebus0_op;
 assign qissuebus0_to_gr[79:72]   = qissuebus0_src1;
 assign qissuebus0_to_gr[71:64]   = qissuebus0_src2;
 assign qissuebus0_to_gr[63:32]   = qissuebus0_res1;
 assign qissuebus0_to_gr[31:0]    = qissuebus0_res2;

 assign qissuebus1_to_gr[101]   = qissuebus1_rdy2;
 assign qissuebus1_to_gr[100]   = qissuebus1_rdy1;
 assign qissuebus1_to_gr[99:97] = qissuebus1_brqid;
 assign qissuebus1_to_gr[96:95] = qissuebus1_ac;
 assign qissuebus1_to_gr[94:93] = qissuebus1_rs;
 assign qissuebus1_to_gr[92]    = qissuebus1_valid;
 assign qissuebus1_to_gr[91:88] = qissuebus1_qid;
 assign qissuebus1_to_gr[87:80] = qissuebus1_op;
 assign qissuebus1_to_gr[79:72] = qissuebus1_src1;
 assign qissuebus1_to_gr[71:64] = qissuebus1_src2;
 assign qissuebus1_to_gr[63:32] = qissuebus1_res1;
 assign qissuebus1_to_gr[31:0]  = qissuebus1_res2;

endmodule 

/*
////////////////////////////////////////////////////////////////////////
// In this version commitbus_op is added into the commitbus_to_tlb to 
// comply with the ejtag feature realization in the tlb.
// Modifying Time:2005/01/19
////////////////////////////////////////////////////////////////////////
*/
module godson_commitbus_module(
                               commitbus0,
                               commitbus0_to_gr,
                               commitbus1,
                               commitbus1_to_gr,
                               
                               commitbus_to_fetch,
                               commitbus0_to_tlb,
                               
                               commitbus1_to_tlb,
                               
                               commitbus_to_hb,
                               commitbus_ex_out,
                               commitbus_to_itlb) ;
 input  [`Lcommitbus-1:0] commitbus0;
 output [`Lcommitbus_to_gr-1:0] commitbus0_to_gr;

 input  [`Lcommitbus-1:0] commitbus1;
 output [`Lcommitbus_to_gr-1:0] commitbus1_to_gr;
 
 output [`Lcommitbus_to_fetch-1:0] commitbus_to_fetch;
 
 output [`Lcommitbus_to_tlb-1:0]   commitbus0_to_tlb;

 output [`Lcommitbus_to_tlb-1:0] commitbus1_to_tlb;

 output [`Lcommitbus_to_hb-1:0] commitbus_to_hb;
 output [`Lcommitbus_to_itlb-1:0] commitbus_to_itlb;
 output commitbus_ex_out;

 wire        commitbus0_fp,         commitbus1_fp;
 wire [31:0] commitbus0_dspctl,     commitbus1_dspctl;
 wire [31:0] commitbus0_low,        commitbus1_low;
 wire [31:0] commitbus0_hi,         commitbus1_hi;
 wire [1:0]  commitbus0_ac,         commitbus1_ac;
 wire        commitbus0_con_true,   commitbus1_con_true;
 wire [7:0]  commitbus0_gshare,     commitbus1_gshare;
 wire [1:0]  commitbus0_rashead,    commitbus1_rashead;
 wire [1:0]  commitbus0_other_link, commitbus1_other_link;
 wire [1:0]  commitbus0_old_status;
 wire        commitbus0_valid;          
 wire [3:0]  commitbus0_qid;      
 wire [7:0]  commitbus0_op;       
 wire [7:0]  commitbus0_dest;     
 wire [31:0] commitbus0_pc;   
 wire [31:0] commitbus0_value;   
 wire [31:0] commitbus0_fpq_value;   
 wire [31:0] commitbus0_value_h; 
 wire [4:0]  commitbus0_fmt;      
 wire [5:0]  commitbus0_evzoui;   
 wire [5:0]  commitbus0_excode;   
 wire        commitbus0_bd;     
 wire [1:0]  commitbus0_ce;     
 wire        commitbus0_ex;     
 wire        commitbus0_double_dest;        
 wire [`Lword-1:0] commitbus0_taken_target;

 wire [1:0]  commitbus1_old_status;
 wire        commitbus1_valid;        
 wire [3:0]  commitbus1_qid;   
 wire [7:0]  commitbus1_op;    
 wire [7:0]  commitbus1_dest;  
 wire [31:0] commitbus1_pc;   
 wire [31:0] commitbus1_value;   
 wire [31:0] commitbus1_fpq_value;   
 wire [31:0] commitbus1_value_h; 
 wire [4:0]  commitbus1_fmt;     
 wire [5:0]  commitbus1_evzoui;  
 wire [5:0]  commitbus1_excode;  
 wire        commitbus1_bd;     
 wire [1:0]  commitbus1_ce;     
 wire        commitbus1_ex;     
 wire        commitbus1_double_dest;  
 wire [`Lword-1:0] commitbus1_taken_target;

 assign commitbus0_ac           = commitbus0[175:174]; 
 assign commitbus0_dspctl       = commitbus0[173:142]; 
 assign commitbus0_other_link   = commitbus0[141:140]; 
 assign commitbus0_rashead      = commitbus0[139:138]; 
 assign commitbus0_gshare       = commitbus0[137:130]; 
 assign commitbus0_con_true     = commitbus0[129]; 
 assign commitbus0_old_status   = commitbus0[128:127];      
 assign commitbus0_taken_target = commitbus0[126:95];     
 assign commitbus0_valid        = commitbus0[94];      
 assign commitbus0_qid          = commitbus0[93:90];     
 assign commitbus0_op           = commitbus0[89:82];     
 assign commitbus0_dest         = commitbus0[81:74];     
 assign commitbus0_value        = commitbus0[73:42];     
 assign commitbus0_pc           = commitbus0[41:10];     
 assign commitbus0_excode       = commitbus0[9:4];     
 assign commitbus0_bd           = commitbus0[3];         
 assign commitbus0_ce           = commitbus0[2:1];         
 assign commitbus0_ex           = commitbus0[0];         

 assign commitbus1_ac           = commitbus1[175:174]; 
 assign commitbus1_dspctl       = commitbus1[173:142]; 
 assign commitbus1_other_link   = commitbus1[141:140]; 
 assign commitbus1_rashead      = commitbus1[139:138]; 
 assign commitbus1_gshare       = commitbus1[137:130]; 
 assign commitbus1_con_true     = commitbus1[129]; 
 assign commitbus1_old_status   = commitbus1[128:127];      
 assign commitbus1_taken_target = commitbus1[126:95];     
 assign commitbus1_valid        = commitbus1[94];      
 assign commitbus1_qid          = commitbus1[93:90];     
 assign commitbus1_op           = commitbus1[89:82];     
 assign commitbus1_dest         = commitbus1[81:74];     
 assign commitbus1_value        = commitbus1[73:42];     
 assign commitbus1_pc           = commitbus1[41:10];     
 assign commitbus1_excode       = commitbus1[9:4];     
 assign commitbus1_bd           = commitbus1[3];         
 assign commitbus1_ce           = commitbus1[2:1];         
 assign commitbus1_ex           = commitbus1[0];         



//-----------------commitbus_to_fetch---------------------------------------------------
 assign commitbus_to_fetch[35:34]  = commitbus1_rashead;
 assign commitbus_to_fetch[33:26]  = commitbus1_gshare;
 assign commitbus_to_fetch[25:20]  = commitbus1_excode;
 assign commitbus_to_fetch[19]     = commitbus1_ex;
 assign commitbus_to_fetch[18]     = commitbus1_valid;
 
 assign commitbus_to_fetch[17:16]   = commitbus0_rashead;
 assign commitbus_to_fetch[15:8]    = commitbus0_gshare;
 assign commitbus_to_fetch[ 7:2]    = commitbus0_excode;
 assign commitbus_to_fetch[ 1]      = commitbus0_ex;
 assign commitbus_to_fetch[ 0]      = commitbus0_valid;

//----------------------commitbus_to_gr------------------
 assign commitbus0_to_gr[84:83] = commitbus0_ac;
 assign commitbus0_to_gr[82:51] = commitbus0_dspctl;
 assign commitbus0_to_gr[50]    = commitbus0_con_true;
 assign commitbus0_to_gr[49]    = commitbus0_valid;
 assign commitbus0_to_gr[48:41] = commitbus0_op;
 assign commitbus0_to_gr[40:33] = commitbus0_dest;
 assign commitbus0_to_gr[32:1]  = commitbus0_value;
 assign commitbus0_to_gr[0]     = commitbus0_ex;

 assign commitbus1_to_gr[84:83] = commitbus1_ac;
 assign commitbus1_to_gr[82:51] = commitbus1_dspctl;
 assign commitbus1_to_gr[50]    = commitbus1_con_true;
 assign commitbus1_to_gr[49]    = commitbus1_valid;
 assign commitbus1_to_gr[48:41] = commitbus1_op;
 assign commitbus1_to_gr[40:33] = commitbus1_dest;
 assign commitbus1_to_gr[32:1]  = commitbus1_value;
 assign commitbus1_to_gr[0]     = commitbus1_ex;

 assign commitbus0_to_tlb[82:75]    = commitbus0_op;
 assign commitbus0_to_tlb[74:73]    = commitbus0_ce; 
 assign commitbus0_to_tlb[72]       = commitbus0_bd;
 assign commitbus0_to_tlb[71]       = commitbus0_valid;
 assign commitbus0_to_tlb[70:39]    = commitbus0_pc; //pc
 assign commitbus0_to_tlb[38:7]     = commitbus0_value;//
 assign commitbus0_to_tlb[6:1]      = commitbus0_excode;
 assign commitbus0_to_tlb[0]        = commitbus0_ex;
 
 assign commitbus1_to_tlb[82:75]    = commitbus1_op;
 assign commitbus1_to_tlb[74:73]    = commitbus1_ce; 
 assign commitbus1_to_tlb[72]       = commitbus1_bd;
 assign commitbus1_to_tlb[71]       = commitbus1_valid;
 assign commitbus1_to_tlb[70:39]    = commitbus1_pc; //pc
 assign commitbus1_to_tlb[38:7]     = commitbus1_value;//
 assign commitbus1_to_tlb[6:1]      = commitbus1_excode;
 assign commitbus1_to_tlb[0]        = commitbus1_ex;

 assign commitbus_to_hb[50:47]      = commitbus0_ex ? commitbus0_qid    : commitbus1_qid ;
 assign commitbus_to_hb[46:39]      = commitbus0_ex ? commitbus0_op     : commitbus1_op;
 assign commitbus_to_hb[38: 7]      = commitbus0_ex ? commitbus0_pc   : commitbus1_pc;
 assign commitbus_to_hb[6:1]        = commitbus0_ex ? commitbus0_excode : commitbus1_excode;
 assign commitbus_to_hb[0]          = commitbus0_ex ? commitbus0_ex     : commitbus1_ex;
 assign commitbus_ex_out            = commitbus0_ex ? commitbus0_ex     : commitbus1_ex;
 assign commitbus_to_itlb[8]      = commitbus0_ex    ? commitbus0_bd : commitbus1_bd;
 assign commitbus_to_itlb[7]      = commitbus0_valid ? (commitbus0_op == `OP_DERET)&commitbus0_valid :
                                                       (commitbus1_op == `OP_DERET)&commitbus1_valid ;
 assign commitbus_to_itlb[6:1]    = commitbus0_ex ? commitbus0_excode : commitbus1_excode;
 assign commitbus_to_itlb[0]      = commitbus0_ex ? commitbus0_ex     : commitbus1_ex;

endmodule


module godson_brbus_module(brbus, brbus_to_rs, brbus_to_fu, brbus_to_icache, brbus_to_cp0, brbus_to_hb);
input [95:0]brbus;
output [6:0]brbus_to_rs;
output [6:0]brbus_to_fu;
output [1:0] brbus_to_icache;
output [18:0] brbus_to_cp0;
output [47:0]brbus_to_hb;

wire        brbus_valid;
wire        brbus_brerr;
wire [5:0]  brbus_brmask;
wire [7:0]  brbus_op;
wire [31:0] brbus_value;
wire [31:0] brbus_value_h;
wire [1:0]  brbus_old_status;
wire [7:0]  brbus_gshare;
wire [1:0]  brbus_rashead;
wire [1:0]  brbus_other_link;
wire        brbus_jr31;
wire        brbus_bht;

wire        brbus_static_br;

STATIC_PREDICT_OP  brbus_static_predic(.op(brbus_op), .static_pred_op(brbus_static_br));

assign brbus_valid        = brbus[0];
assign brbus_brerr        = brbus[1];
assign brbus_brmask       = brbus[7:2];
assign brbus_op           = brbus[15:8];
assign brbus_value        = brbus[47:16];
assign brbus_value_h      = brbus[79:48];
assign brbus_old_status   = brbus[81:80];
assign brbus_gshare       = brbus[89:82];
assign brbus_rashead      = brbus[91:90];
assign brbus_other_link   = brbus[93:92];
assign brbus_jr31         = brbus[94];
assign brbus_bht          = brbus[95];

assign brbus_to_rs[0]    = brbus_brerr;
assign brbus_to_rs[6:1] = brbus_brmask;
assign brbus_to_fu[0] = 1'b1; //brbus_brerr; brbus_brerr has implied in brbus_brmask
assign brbus_to_fu[6:1] = brbus_brmask;
assign brbus_to_icache[0] = brbus_valid;
assign brbus_to_icache[1] = brbus_brerr;

assign brbus_to_cp0[0] = brbus_brerr;
assign brbus_to_cp0[6:1] = brbus_brmask;
assign brbus_to_cp0[7] = brbus_valid;
assign brbus_to_cp0[15:8] = brbus_op;
assign brbus_to_cp0[16] = brbus_jr31;
assign brbus_to_cp0[17] = brbus_bht;
assign brbus_to_cp0[18] = brbus_static_br;

assign brbus_to_hb[0]     = brbus_valid;
assign brbus_to_hb[1]     = brbus_brerr;
assign brbus_to_hb[9:2]   = brbus_op;
assign brbus_to_hb[41:10] = brbus_value_h;
assign brbus_to_hb[47:42] = brbus_brmask;
endmodule
