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

module godson_addr_module(
    clock,
    reset,

    mmrs_to_addr_i,
    addr_to_tlb_o,
    addr_to_dcache_o,

    tlb_allowin_i,
    tlb_stall_i,
    tlb_has_ll_i,
    DEBUG_MODE,
    cr_cfg6_rti_i,
    no_conflict_to_addr_i,
    memqueue_stall_i,
    memqueue_has_ll_i,
    inst_valid_at_addr_o,
    inst_cache_at_addr_o,
    inst_sync_at_addr_o,

    addr_allowin_o,
    
    cp0_mod_ok_i,
    cp0_qid_o

);
// --- whole system signals
input   clock;
input   reset;

// --- connect with mmrs module
input   [`Lmmrs-1:0] mmrs_to_addr_i;
output               addr_allowin_o;

// --- connect with tlb module
input   tlb_allowin_i;
input   tlb_stall_i;
input   tlb_has_ll_i;
input   DEBUG_MODE;
input   cr_cfg6_rti_i;
output  [`Laddr_to_tlb-1:0] addr_to_tlb_o;

// --- connect with dcache module
input           no_conflict_to_addr_i;
output  [`Laddr_to_dcache-1:0] addr_to_dcache_o;

// --- connect with memqueue module
input   memqueue_stall_i;
input   memqueue_has_ll_i;
output  inst_valid_at_addr_o;
output  inst_cache_at_addr_o;
output  inst_sync_at_addr_o;

// --- connect with queue module
input   cp0_mod_ok_i;
output  [  3:0] cp0_qid_o;


// all fields of mmrs_to_addr_i
wire [31:0] mmrs_addr_vk  = mmrs_to_addr_i[178:147];
wire [ 2:0] mmrs_fpqid    = mmrs_to_addr_i[146:144];
wire [ 2:0] mmrs_brqid    = mmrs_to_addr_i[143:141];
wire        mmrs_valid    = mmrs_to_addr_i[140    ];
wire [ 7:0] mmrs_op       = mmrs_to_addr_i[139:132];
wire [ 3:0] mmrs_qid      = mmrs_to_addr_i[131:128];
wire [31:0] mmrs_vj       = mmrs_to_addr_i[127: 96];
wire [31:0] mmrs_vk       = mmrs_to_addr_i[ 95: 64]; 
wire [31:0] mmrs_vk_h     = mmrs_to_addr_i[ 63: 32]; 
wire [31:0] mmrs_vl       = mmrs_to_addr_i[ 31:  0];

// all fields of addr_to_tlb_o
wire [ 2:0] brqid_to_tlb;
wire        valid_to_tlb;
wire        rd_tag_to_tlb;
wire        rd_data_to_tlb;
wire [ 2:0] fpqid_to_tlb;
wire [ 3:0] qid_to_tlb;
wire [ 7:0] op_to_tlb;
wire [31:0] vaddr_to_tlb;
wire [31:0] value_to_tlb;
wire [31:0] value_h_to_tlb;
wire        cond_true_to_tlb;

assign addr_to_tlb_o[117:115] = fpqid_to_tlb;
assign addr_to_tlb_o[114:112] = brqid_to_tlb;
assign addr_to_tlb_o[111    ] = cond_true_to_tlb;
assign addr_to_tlb_o[110    ] = valid_to_tlb;
assign addr_to_tlb_o[109    ] = rd_tag_to_tlb;
assign addr_to_tlb_o[108    ] = rd_data_to_tlb;
assign addr_to_tlb_o[107:104] = qid_to_tlb;
assign addr_to_tlb_o[103: 96] = op_to_tlb;
assign addr_to_tlb_o[ 95: 64] = vaddr_to_tlb;
assign addr_to_tlb_o[ 63: 32] = value_to_tlb;
assign addr_to_tlb_o[ 31:  0] = value_h_to_tlb;

// all fields of addr_to_dcache_o
wire        valid_to_dcache;
wire        read_tag_to_dcache;
wire        read_data_to_dcache;
wire [ 1:0] set_to_dcache;
wire [ 7:0] op_to_dcache;
wire [11:0] laddr_to_dcache;

assign addr_to_dcache_o[24   ] = valid_to_dcache;
assign addr_to_dcache_o[23   ] = read_tag_to_dcache;
assign addr_to_dcache_o[22   ] = read_data_to_dcache;
assign addr_to_dcache_o[21:20] = set_to_dcache;
assign addr_to_dcache_o[19:12] = op_to_dcache;
assign addr_to_dcache_o[11: 0] = laddr_to_dcache;


wire [31:0] value_of_store;

// LL 
wire        op_ll;
// indicate this inst. belongs to CP0 class
wire        op_cp0;

wire        op_cache5;
// DSP load instruction
wire        op_index_load;
// load instruction 
wire        op_load;
// store instruction
wire        op_store;
// d-cache instruction
wire        op_dcache;
// i-cache instruction
wire        op_icache;
// these inst. should access dcache tag ram
wire        op_cache_tag;
// these inst. should access dcache data ram
wire        op_cache_data;

// ejtag load store
wire        op_ejtag_ld_st;
// the cp0 inst. should be stalled until cp0_mod_ok_i=1
wire        cp0_stall;
// when in real time int mode, ll should block sc and eret follow it
wire        ll_stall;

// base address 
wire [31:0] base_addr;
// offset value
wire [31:0] offset_value;
// vaddr of memory access
wire [31:0] mem_vaddr;

// classify operations to different types
wire   op_sc  = (mmrs_op==`OP_SC);
wire   op_eret= (mmrs_op==`OP_ERET);
assign op_ll  = (mmrs_op==`OP_LL);
assign op_cp0 = op_eret              || (mmrs_op==`OP_DERET) ||
                (mmrs_op==`OP_TLBP)  || (mmrs_op==`OP_TLBR)  ||
                (mmrs_op==`OP_TLBWI) || (mmrs_op==`OP_TLBWR) ||
                (mmrs_op==`OP_MFC0)  || (mmrs_op==`OP_MTC0)  ||
                (mmrs_op==`OP_DI)    || (mmrs_op==`OP_EI)    ;
assign op_cache5 = (mmrs_op==`OP_CACHE5);

assign op_load =(mmrs_op==`OP_LB)      || (mmrs_op==`OP_LH)      ||
                (mmrs_op==`OP_LW)      || (mmrs_op==`OP_LBU)     ||
                (mmrs_op==`OP_LHU)     || (mmrs_op==`OP_LL)      ||
                (mmrs_op==`OP_LBUX)    || (mmrs_op==`OP_LHX)     ||
                (mmrs_op==`OP_LWX)     || 
                (mmrs_op==`OP_LWL)     || (mmrs_op==`OP_LWR)     ;

assign op_store = (mmrs_op==`OP_SB)      || (mmrs_op==`OP_SH)      ||
                  (mmrs_op==`OP_SW)      || op_sc                  ||
                  (mmrs_op==`OP_SWL)     || (mmrs_op==`OP_SWR)     ;

assign op_cache_tag = op_load                || op_store               ||
                      op_cache5              || (mmrs_op==`OP_CACHE17) ||
                      (mmrs_op==`OP_CACHE21) || (mmrs_op==`OP_CACHE29) ||
                      (mmrs_op==`OP_PREFX)   || (mmrs_op==`OP_PREF)    || 
                      (mmrs_op==`OP_SYNCI)   ;

assign op_cache_data = op_load;


// generate the pipeline allowin signal to mmrs module
assign cp0_qid_o = mmrs_qid;
assign op_ejtag_ld_st = (op_load|op_store) & DEBUG_MODE & (base_addr[31:23]==9'b1111_1111_0) &
                        ((base_addr[22:21]==2'b01) || 
                         (base_addr[22:20]==3'b001 && ~offset_value[31]) ||
                         (base_addr[22:20]==3'b100 &&  offset_value[31])   );
assign cp0_stall = (op_cp0 | op_ll | op_cache5 | op_icache | op_ejtag_ld_st) & ~cp0_mod_ok_i; 
assign ll_stall  = (op_sc | op_eret) & (tlb_has_ll_i | memqueue_has_ll_i) & cr_cfg6_rti_i;
assign addr_allowin_o = tlb_allowin_i & no_conflict_to_addr_i & ~memqueue_stall_i & ~tlb_stall_i & ~cp0_stall & ~ll_stall;

// caculate vaddr of mem access operations
assign op_index_load = (mmrs_op==`OP_LBUX) || (mmrs_op==`OP_LHX) || (mmrs_op==`OP_LWX) || (mmrs_op==`OP_PREFX);
//assign offset_value = op_index_load ? {((mmrs_op==`OP_PREFX) ? mmrs_addr_vk[31:30] : 2'b0), mmrs_addr_vk[29:0]} : mmrs_vl;
assign offset_value = op_index_load ? mmrs_addr_vk[31:0] : mmrs_vl;
assign base_addr = mmrs_vj;
assign mem_vaddr = base_addr + offset_value;


// dcache access bus
assign valid_to_dcache = mmrs_valid & tlb_allowin_i & ~memqueue_stall_i & ~tlb_stall_i;
assign read_tag_to_dcache  = op_cache_tag;
assign read_data_to_dcache = op_cache_data;
assign set_to_dcache = mem_vaddr[13:12];
assign op_to_dcache = mmrs_op;
assign laddr_to_dcache = mem_vaddr[11:0];

// bus from addr  to tlb
assign valid_to_tlb   = mmrs_valid & ~memqueue_stall_i & ~tlb_stall_i & ~cp0_stall & ~ll_stall & no_conflict_to_addr_i;
assign rd_tag_to_tlb  = op_cache_tag;
assign rd_data_to_tlb = op_cache_data;
assign qid_to_tlb     = mmrs_qid;
assign fpqid_to_tlb   = mmrs_fpqid;
assign op_to_tlb      = mmrs_op;
assign vaddr_to_tlb   = mem_vaddr;
assign brqid_to_tlb   = mmrs_brqid;
assign value_to_tlb   = (op_store) ? value_of_store : mmrs_vk;
assign value_h_to_tlb = mmrs_vk_h;
assign cond_true_to_tlb = 1'b0;


// output to memqueue

assign op_dcache = (mmrs_op==`OP_CACHE1)  || op_cache5              ||
                   (mmrs_op==`OP_CACHE9)  || (mmrs_op==`OP_CACHE17) ||
                   (mmrs_op==`OP_CACHE21) || (mmrs_op==`OP_CACHE29) ||
                   (mmrs_op==`OP_SYNCI)   ;

assign op_icache = (mmrs_op==`OP_CACHE0)  || (mmrs_op==`OP_CACHE8)  ||
                   (mmrs_op==`OP_CACHE16) || (mmrs_op==`OP_CACHE28) ||
                   (mmrs_op==`OP_SYNCI)   ;

assign inst_valid_at_addr_o = mmrs_valid;

assign inst_cache_at_addr_o = op_dcache | op_icache;

assign inst_sync_at_addr_o = (mmrs_op==`OP_SYNC);

form_store_value u_form_st_value(.valuein(mmrs_vk), 
                                 .valuein_h(mmrs_vk_h), 
                                 .offset(mem_vaddr[1:0]), 
                                 .op(mmrs_op), 
                                 .valueout(value_of_store));

endmodule //godson_addr_module


module form_store_value(valuein, valuein_h, offset, op, valueout);
input  [31:0] valuein;
input  [31:0] valuein_h;
input  [ 1:0] offset;
input  [ 7:0] op;
output [31:0] valueout;

wire op_sb  = (op == `OP_SB );
wire op_sh  = (op == `OP_SH );
wire op_sw  = (op == `OP_SW );
wire op_sc  = (op == `OP_SC );
wire op_sdc1= 1'b0;
wire op_swc1= 1'b0;
wire op_swl = (op == `OP_SWL);
wire op_swr = (op == `OP_SWR);
wire [31:0] sb_value;
wire [31:0] sh_value;
wire [31:0] sw_value;
wire [31:0] swl_value;
wire [31:0] swr_value;
assign sb_value = {valuein[7:0], valuein[7:0], valuein[7:0], valuein[7:0]};
assign sh_value = {valuein[15:0], valuein[15:0]};
assign sw_value = valuein;
shift_swl u_shift_swl(.offset(offset), .value_in(valuein), .value_out(swl_value));
shift_swr u_shift_swr(.offset(offset), .value_in(valuein), .value_out(swr_value));

assign valueout = {32{op_sb}}&sb_value      |
                  {32{op_sh}}&sh_value      |
                  {32{op_sw|op_sc|op_sdc1|op_swc1}}&sw_value    |
                  {32{op_swl}}&swl_value    |
                  {32{op_swr}}&swr_value    ;

endmodule


module shift_swl(offset,value_in,value_out);
input  [1:0] offset;
input  [31:0] value_in;
output [31:0] value_out;
reg    [31:0] value_out;

always @(offset or value_in)
   begin
     case(offset) // synopsys full_case parallel_case
        2'b00:value_out={value_in[31: 8],value_in[31:24]};
        2'b01:value_out={value_in[31:16],value_in[31:16]};
        2'b10:value_out={value_in[31:24],value_in[31: 8]};
        2'b11:value_out=value_in;
     endcase
   end
endmodule //shift_swl

module shift_swr(offset,value_in,value_out);
input  [1:0] offset;
input  [31:0] value_in;
output [31:0] value_out;
reg    [31:0] value_out;

always @(offset or value_in)
   begin
     case(offset) // synopsys full_case parallel_case
        2'b00:value_out=value_in;
        2'b01:value_out={value_in[23: 0],value_in[ 7: 0]};
        2'b10:value_out={value_in[15: 0],value_in[15: 0]}; 
                    //when read from st queue or write into miss queue we actually use the upper part,
                    //but when we directly write it to mem, we use the lower part.
        2'b11:value_out={value_in[ 7: 0],value_in[23: 0]};
                    //when read from st queue or write into miss queue we actually use the uppest 8 bit,
                    //but when we directly write it to mem, we use the lowest 8 bit
     endcase
   end
endmodule //shift_swr



