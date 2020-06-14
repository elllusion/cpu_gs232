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

module godson_dcache_module(
    clock,
    reset,
    
    addr_to_dcache_i,
    no_conflict_to_addr_o,
    
    tlb_read_again_i,
    dcachepaddr_i,
    conflict_to_tlb_o,
    dcache_result_o,
    tag_to_tlb_o,
    st_ld_conflict_o,
    
    store_req_i,
    cache_req_i,
    replace_req_i,
    refill_req_i,
    memq_to_dcache_i,
    replace_dump_o,
    dcache_tag_o,
    
    ram_to_dcache_i,
    dcache_to_ram_o

);
parameter   ALL_SET_EN = 4'b1111;

input           clock;
input           reset;

input   [`Laddr_to_dcache-1:0] addr_to_dcache_i;
output          no_conflict_to_addr_o;

input   [`Ltlb_read_again-1:0] tlb_read_again_i;
input   [ 31:0] dcachepaddr_i;
output          conflict_to_tlb_o;
output  [`Ldcache_result-1:0] dcache_result_o;
output  [ 31:0] tag_to_tlb_o;
output          st_ld_conflict_o;

input           store_req_i;
input           cache_req_i;
input           replace_req_i;
input   [  1:0] refill_req_i;
input   [`Lmemq_to_dcache-1:0] memq_to_dcache_i;
output  [`Lreplace_dump-1:0] replace_dump_o;
output  [ 31:0] dcache_tag_o;

input   [`Lram_to_dcache-1:0] ram_to_dcache_i;
output  [`Ldcache_to_ram-1:0] dcache_to_ram_o;


// all fields of addr_to_dcache_i
wire        valid_from_addr     = addr_to_dcache_i[24   ];
wire        read_tag_from_addr  = addr_to_dcache_i[23   ];
wire        read_data_from_addr = addr_to_dcache_i[22   ];
wire [ 1:0] set_from_addr       = addr_to_dcache_i[21:20];
wire [ 7:0] op_from_addr        = addr_to_dcache_i[19:12];
wire [11:0] laddr_from_addr     = addr_to_dcache_i[11: 0];

// all fields of tlb_read_again_i
wire        valid_tlb_rag	= tlb_read_again_i[22   ];
wire        rd_tag_tlb_rag	= tlb_read_again_i[21   ];
wire        rd_data_tlb_rag	= tlb_read_again_i[20   ];
wire [ 7:0] op_tlb_rag	    = tlb_read_again_i[19:12];
wire [11:0] laddr_tlb_rag	= tlb_read_again_i[11: 0];

// all fields of dcache_result_o
wire        hit_dcache;
wire [ 1:0] hit_set_dcache;
wire [ 3:0] lock_dcache;
wire [31:0] value_h_dcache;
wire [31:0] value_dcache;
assign dcache_result_o[70   ] = hit_dcache; 
assign dcache_result_o[69:68] = hit_set_dcache;
assign dcache_result_o[67:64] = lock_dcache;
assign dcache_result_o[63:32] = value_h_dcache; 
assign dcache_result_o[31: 0] = value_dcache;

// all fields of store_req_i
wire        valid_st        = store_req_i;

// all fields of cache_req_i
wire        valid_cache     = cache_req_i;

// all fields of replace_req_i
wire        valid_replace   = replace_req_i;

// all fields of refill_req_i
wire        valid_refill = refill_req_i[1];    
wire        dirty_refill = refill_req_i[0];

// all fields of memq_to_dcache_i
wire [  1:0] st_set_memq    = memq_to_dcache_i[338:337];
wire [ 11:0] st_laddr_memq  = memq_to_dcache_i[336:325];
wire [  1:0] set_memq       = memq_to_dcache_i[324:323];
wire [ 11:0] laddr_memq     = memq_to_dcache_i[322:311];
wire         tag_wen_memq   = memq_to_dcache_i[310    ]; 
wire [ 21:0] tagin_memq     = memq_to_dcache_i[309:288];
wire [ 31:0] data_wen_memq  = memq_to_dcache_i[287:256];
wire [255:0] datain_memq    = memq_to_dcache_i[255:  0];

wire [ 1:0] set_replace     = set_memq;
wire [11:0] laddr_replace   = laddr_memq;

wire [  1:0] set_refill      = set_memq;    
wire [ 11:0] laddr_refill    = laddr_memq;    
wire [ 21:0] tagin_refill    = tagin_memq;

wire [ 1:0] set_st     = st_set_memq;     
wire [11:0] laddr_st   = st_laddr_memq; 

wire [ 1:0] set_cache  = set_memq;
wire [11:0] laddr_cache= laddr_memq; 

// all fields of replace_dump_o
wire         valid_dump;
wire         dirty_dump;
wire [ 21:0] tagout_dump;
wire [255:0] dataout_dump;

assign replace_dump_o[279    ] = valid_dump;
assign replace_dump_o[278    ] = dirty_dump;
assign replace_dump_o[277:256] = tagout_dump;
assign replace_dump_o[255:  0] = dataout_dump;

// all fields of ram_to_dcache_i
wire [277:0] set3_ram2dcache = ram_to_dcache_i[1111: 834];
wire [277:0] set2_ram2dcache = ram_to_dcache_i[833 : 556];
wire [277:0] set1_ram2dcache = ram_to_dcache_i[555 : 278];
wire [277:0] set0_ram2dcache = ram_to_dcache_i[277 :   0];

wire [ 63:0] set3_datain_bank3 = set3_ram2dcache[277:214];
wire [ 63:0] set3_datain_bank2 = set3_ram2dcache[213:150];
wire [ 63:0] set3_datain_bank1 = set3_ram2dcache[149: 86];
wire [ 63:0] set3_datain_bank0 = set3_ram2dcache[ 85: 22];
wire [ 21:0] set3_tagin        = set3_ram2dcache[ 21:  0];

wire [ 63:0] set2_datain_bank3 = set2_ram2dcache[277:214];
wire [ 63:0] set2_datain_bank2 = set2_ram2dcache[213:150];
wire [ 63:0] set2_datain_bank1 = set2_ram2dcache[149: 86];
wire [ 63:0] set2_datain_bank0 = set2_ram2dcache[ 85: 22];
wire [ 21:0] set2_tagin        = set2_ram2dcache[ 21:  0];

wire [ 63:0] set1_datain_bank3 = set1_ram2dcache[277:214];
wire [ 63:0] set1_datain_bank2 = set1_ram2dcache[213:150];
wire [ 63:0] set1_datain_bank1 = set1_ram2dcache[149: 86];
wire [ 63:0] set1_datain_bank0 = set1_ram2dcache[ 85: 22];
wire [ 21:0] set1_tagin        = set1_ram2dcache[ 21:  0];

wire [ 63:0] set0_datain_bank3 = set0_ram2dcache[277:214];
wire [ 63:0] set0_datain_bank2 = set0_ram2dcache[213:150];
wire [ 63:0] set0_datain_bank1 = set0_ram2dcache[149: 86];
wire [ 63:0] set0_datain_bank0 = set0_ram2dcache[ 85: 22];
wire [ 21:0] set0_tagin        = set0_ram2dcache[ 21:  0];


// all fields of dcache_to_ram_o
wire [350:0] set3_dcache2ram;
assign dcache_to_ram_o[1403:1053] = set3_dcache2ram;
wire [350:0] set2_dcache2ram;
assign dcache_to_ram_o[1052: 702] = set2_dcache2ram;
wire [350:0] set1_dcache2ram;
assign dcache_to_ram_o[ 701: 351] = set1_dcache2ram;
wire [350:0] set0_dcache2ram;
assign dcache_to_ram_o[ 350:   0] = set0_dcache2ram;

//------------------- set 3 -------------------
wire [79:0] set3_data_bank3;
wire [63:0] set3_data_bank3_D;
wire [ 7:0] set3_data_bank3_wen;
wire [ 6:0] set3_data_bank3_A;
wire        set3_data_bank3_cen;
assign set3_data_bank3[79:16] = set3_data_bank3_D;
assign set3_data_bank3[15: 8] = ~set3_data_bank3_wen;
assign set3_data_bank3[ 7: 1] = set3_data_bank3_A;
assign set3_data_bank3[    0] = ~set3_data_bank3_cen;

wire [79:0] set3_data_bank2;
wire [63:0] set3_data_bank2_D;
wire [ 7:0] set3_data_bank2_wen;
wire [ 6:0] set3_data_bank2_A;
wire        set3_data_bank2_cen;
assign set3_data_bank2[79:16] = set3_data_bank2_D;
assign set3_data_bank2[15: 8] = ~set3_data_bank2_wen;
assign set3_data_bank2[ 7: 1] = set3_data_bank2_A;
assign set3_data_bank2[    0] = ~set3_data_bank2_cen;

wire [79:0] set3_data_bank1;
wire [63:0] set3_data_bank1_D;
wire [ 7:0] set3_data_bank1_wen;
wire [ 6:0] set3_data_bank1_A;
wire        set3_data_bank1_cen;
assign set3_data_bank1[79:16] = set3_data_bank1_D;
assign set3_data_bank1[15: 8] = ~set3_data_bank1_wen;
assign set3_data_bank1[ 7: 1] = set3_data_bank1_A;
assign set3_data_bank1[    0] = ~set3_data_bank1_cen;

wire [79:0] set3_data_bank0;
wire [63:0] set3_data_bank0_D;
wire [ 7:0] set3_data_bank0_wen;
wire [ 6:0] set3_data_bank0_A;
wire        set3_data_bank0_cen;
assign set3_data_bank0[79:16] = set3_data_bank0_D;
assign set3_data_bank0[15: 8] = ~set3_data_bank0_wen;
assign set3_data_bank0[ 7: 1] = set3_data_bank0_A;
assign set3_data_bank0[    0] = ~set3_data_bank0_cen;

wire [30:0] set3_tag;
wire [21:0] set3_tag_D;
wire        set3_tag_wen;
wire [ 6:0] set3_tag_A;
wire        set3_tag_cen;
assign set3_tag[30:9] = set3_tag_D;
assign set3_tag[   8] = ~set3_tag_wen;
assign set3_tag[ 7:1] = set3_tag_A;
assign set3_tag[   0] = ~set3_tag_cen;

assign set3_dcache2ram[350:271] = set3_data_bank3;
assign set3_dcache2ram[270:191] = set3_data_bank2;
assign set3_dcache2ram[190:111] = set3_data_bank1;
assign set3_dcache2ram[110: 31] = set3_data_bank0;
assign set3_dcache2ram[ 30:  0] = set3_tag;


//------------------- set 2 -------------------
wire [79:0] set2_data_bank3;
wire [63:0] set2_data_bank3_D;
wire [ 7:0] set2_data_bank3_wen;
wire [ 6:0] set2_data_bank3_A;
wire        set2_data_bank3_cen;
assign set2_data_bank3[79:16] = set2_data_bank3_D;
assign set2_data_bank3[15: 8] = ~set2_data_bank3_wen;
assign set2_data_bank3[ 7: 1] = set2_data_bank3_A;
assign set2_data_bank3[    0] = ~set2_data_bank3_cen;

wire [79:0] set2_data_bank2;
wire [63:0] set2_data_bank2_D;
wire [ 7:0] set2_data_bank2_wen;
wire [ 6:0] set2_data_bank2_A;
wire        set2_data_bank2_cen;
assign set2_data_bank2[79:16] = set2_data_bank2_D;
assign set2_data_bank2[15: 8] = ~set2_data_bank2_wen;
assign set2_data_bank2[ 7: 1] = set2_data_bank2_A;
assign set2_data_bank2[    0] = ~set2_data_bank2_cen;

wire [79:0] set2_data_bank1;
wire [63:0] set2_data_bank1_D;
wire [ 7:0] set2_data_bank1_wen;
wire [ 6:0] set2_data_bank1_A;
wire        set2_data_bank1_cen;
assign set2_data_bank1[79:16] = set2_data_bank1_D;
assign set2_data_bank1[15: 8] = ~set2_data_bank1_wen;
assign set2_data_bank1[ 7: 1] = set2_data_bank1_A;
assign set2_data_bank1[    0] = ~set2_data_bank1_cen;

wire [79:0] set2_data_bank0;
wire [63:0] set2_data_bank0_D;
wire [ 7:0] set2_data_bank0_wen;
wire [ 6:0] set2_data_bank0_A;
wire        set2_data_bank0_cen;
assign set2_data_bank0[79:16] = set2_data_bank0_D;
assign set2_data_bank0[15: 8] = ~set2_data_bank0_wen;
assign set2_data_bank0[ 7: 1] = set2_data_bank0_A;
assign set2_data_bank0[    0] = ~set2_data_bank0_cen;

wire [30:0] set2_tag;
wire [21:0] set2_tag_D;
wire        set2_tag_wen;
wire [ 6:0] set2_tag_A;
wire        set2_tag_cen;
assign set2_tag[30:9] = set2_tag_D;
assign set2_tag[   8] = ~set2_tag_wen;
assign set2_tag[ 7:1] = set2_tag_A;
assign set2_tag[   0] = ~set2_tag_cen;

assign set2_dcache2ram[350:271] = set2_data_bank3;
assign set2_dcache2ram[270:191] = set2_data_bank2;
assign set2_dcache2ram[190:111] = set2_data_bank1;
assign set2_dcache2ram[110: 31] = set2_data_bank0;
assign set2_dcache2ram[ 30:  0] = set2_tag;

//------------------- set 1 -------------------
wire [79:0] set1_data_bank3;
wire [63:0] set1_data_bank3_D;
wire [ 7:0] set1_data_bank3_wen;
wire [ 6:0] set1_data_bank3_A;
wire        set1_data_bank3_cen;
assign set1_data_bank3[79:16] = set1_data_bank3_D;
assign set1_data_bank3[15: 8] = ~set1_data_bank3_wen;
assign set1_data_bank3[ 7: 1] = set1_data_bank3_A;
assign set1_data_bank3[    0] = ~set1_data_bank3_cen;

wire [79:0] set1_data_bank2;
wire [63:0] set1_data_bank2_D;
wire [ 7:0] set1_data_bank2_wen;
wire [ 6:0] set1_data_bank2_A;
wire        set1_data_bank2_cen;
assign set1_data_bank2[79:16] = set1_data_bank2_D;
assign set1_data_bank2[15: 8] = ~set1_data_bank2_wen;
assign set1_data_bank2[ 7: 1] = set1_data_bank2_A;
assign set1_data_bank2[    0] = ~set1_data_bank2_cen;

wire [79:0] set1_data_bank1;
wire [63:0] set1_data_bank1_D;
wire [ 7:0] set1_data_bank1_wen;
wire [ 6:0] set1_data_bank1_A;
wire        set1_data_bank1_cen;
assign set1_data_bank1[79:16] = set1_data_bank1_D;
assign set1_data_bank1[15: 8] = ~set1_data_bank1_wen;
assign set1_data_bank1[ 7: 1] = set1_data_bank1_A;
assign set1_data_bank1[    0] = ~set1_data_bank1_cen;

wire [79:0] set1_data_bank0;
wire [63:0] set1_data_bank0_D;
wire [ 7:0] set1_data_bank0_wen;
wire [ 6:0] set1_data_bank0_A;
wire        set1_data_bank0_cen;
assign set1_data_bank0[79:16] = set1_data_bank0_D;
assign set1_data_bank0[15: 8] = ~set1_data_bank0_wen;
assign set1_data_bank0[ 7: 1] = set1_data_bank0_A;
assign set1_data_bank0[    0] = ~set1_data_bank0_cen;

wire [30:0] set1_tag;
wire [21:0] set1_tag_D;
wire        set1_tag_wen;
wire [ 6:0] set1_tag_A;
wire        set1_tag_cen;
assign set1_tag[30:9] = set1_tag_D;
assign set1_tag[   8] = ~set1_tag_wen;
assign set1_tag[ 7:1] = set1_tag_A;
assign set1_tag[   0] = ~set1_tag_cen;

assign set1_dcache2ram[350:271] = set1_data_bank3;
assign set1_dcache2ram[270:191] = set1_data_bank2;
assign set1_dcache2ram[190:111] = set1_data_bank1;
assign set1_dcache2ram[110: 31] = set1_data_bank0;
assign set1_dcache2ram[ 30:  0] = set1_tag;

//------------------- set 0 -------------------
wire [79:0] set0_data_bank3;
wire [63:0] set0_data_bank3_D;
wire [ 7:0] set0_data_bank3_wen;
wire [ 6:0] set0_data_bank3_A;
wire        set0_data_bank3_cen;
assign set0_data_bank3[79:16] = set0_data_bank3_D;
assign set0_data_bank3[15: 8] = ~set0_data_bank3_wen;
assign set0_data_bank3[ 7: 1] = set0_data_bank3_A;
assign set0_data_bank3[    0] = ~set0_data_bank3_cen;

wire [79:0] set0_data_bank2;
wire [63:0] set0_data_bank2_D;
wire [ 7:0] set0_data_bank2_wen;
wire [ 6:0] set0_data_bank2_A;
wire        set0_data_bank2_cen;
assign set0_data_bank2[79:16] = set0_data_bank2_D;
assign set0_data_bank2[15: 8] = ~set0_data_bank2_wen;
assign set0_data_bank2[ 7: 1] = set0_data_bank2_A;
assign set0_data_bank2[    0] = ~set0_data_bank2_cen;

wire [79:0] set0_data_bank1;
wire [63:0] set0_data_bank1_D;
wire [ 7:0] set0_data_bank1_wen;
wire [ 6:0] set0_data_bank1_A;
wire        set0_data_bank1_cen;
assign set0_data_bank1[79:16] = set0_data_bank1_D;
assign set0_data_bank1[15: 8] = ~set0_data_bank1_wen;
assign set0_data_bank1[ 7: 1] = set0_data_bank1_A;
assign set0_data_bank1[    0] = ~set0_data_bank1_cen;

wire [79:0] set0_data_bank0;
wire [63:0] set0_data_bank0_D;
wire [ 7:0] set0_data_bank0_wen;
wire [ 6:0] set0_data_bank0_A;
wire        set0_data_bank0_cen;
assign set0_data_bank0[79:16] = set0_data_bank0_D;
assign set0_data_bank0[15: 8] = ~set0_data_bank0_wen;
assign set0_data_bank0[ 7: 1] = set0_data_bank0_A;
assign set0_data_bank0[    0] = ~set0_data_bank0_cen;

wire [30:0] set0_tag;
wire [21:0] set0_tag_D;
wire        set0_tag_wen;
wire [ 6:0] set0_tag_A;
wire        set0_tag_cen;
assign set0_tag[30:9] = set0_tag_D;
assign set0_tag[   8] = ~set0_tag_wen;
assign set0_tag[ 7:1] = set0_tag_A;
assign set0_tag[   0] = ~set0_tag_cen;

assign set0_dcache2ram[350:271] = set0_data_bank3;
assign set0_dcache2ram[270:191] = set0_data_bank2;
assign set0_dcache2ram[190:111] = set0_data_bank1;
assign set0_dcache2ram[110: 31] = set0_data_bank0;
assign set0_dcache2ram[ 30:  0] = set0_tag;

reg  [127:0] set0_dirty_r;
reg  [127:0] set1_dirty_r;
reg  [127:0] set2_dirty_r;
reg  [127:0] set3_dirty_r;

wire        addr_read_tag;
wire        tlb_read_tag;
wire        mq_access_tag;
wire        addr_read_data;
wire        tlb_read_data;
wire        mq_access_data;

wire [ 3:0] addr_ld_bank_en;
wire [ 3:0] tlb_ld_bank_en;
wire [ 3:0] mq_st_bank_v;
wire [ 3:0] mq_st_bank_en;
wire [ 3:0] addr_rd_data_en;
wire [ 3:0] tlb_rd_data_en;

wire [ 3:0] set_tag_cen;
wire [ 3:0] set_tag_wen;

wire [ 3:0] set_data_en;
wire [ 3:0] set_data_wen;

wire [ 3:0] bank_en;
wire [ 3:0] bank_ren;
wire [ 3:0] bank_wen;

wire [ 6:0] tag_index;
wire [ 6:0] bank3_index;
wire [ 6:0] bank2_index;
wire [ 6:0] bank1_index;
wire [ 6:0] bank0_index;

//CEN for each set
wire [3:0] set0_bank_cen;
wire [3:0] set1_bank_cen;
wire [3:0] set2_bank_cen;
wire [3:0] set3_bank_cen;

//WEN for each set
wire [3:0] set0_bank_wen;
wire [3:0] set1_bank_wen;
wire [3:0] set2_bank_wen;
wire [3:0] set3_bank_wen;

wire        op_cache5;
wire        op_replace;
reg         op_replace_r;
reg         dump_dirty_r;
reg  [1:0]  set_nu_r; // save set number for cache5 tag select or replace data select
wire [ 3:0] set_nu_r_bitmap;

wire [ 3:0] set_hit;


// cache ram request arbitration
assign addr_read_tag = valid_from_addr & read_tag_from_addr;
assign tlb_read_tag  = valid_tlb_rag & rd_tag_tlb_rag;
assign mq_access_tag  = valid_replace | valid_refill | valid_cache;
assign addr_read_data = valid_from_addr & read_data_from_addr;
assign tlb_read_data  = valid_tlb_rag & rd_data_tlb_rag;


dcache_decode_2_4 u0_dec_2_4(.in(laddr_from_addr[4:3]), .out(addr_ld_bank_en));
dcache_decode_2_4 u1_dec_2_4(.in(laddr_tlb_rag[4:3]), .out(tlb_ld_bank_en));
dcache_decode_2_4 u2_dec_2_4(.in(laddr_st[4:3]), .out(mq_st_bank_v));
assign addr_rd_data_en = (addr_read_data) ? addr_ld_bank_en:4'b0000;
assign tlb_rd_data_en = (tlb_read_data) ? tlb_ld_bank_en:4'b0000;
assign mq_st_bank_en  = (valid_st) ? mq_st_bank_v : 4'b0000;

assign conflict_to_tlb_o = (rd_tag_tlb_rag&mq_access_tag)   | 
                           (|(mq_st_bank_en&tlb_ld_bank_en)&rd_data_tlb_rag);


wire   conflict_to_addr = (read_tag_from_addr & (tlb_read_tag | mq_access_tag))   |
                          (|(addr_ld_bank_en&mq_st_bank_en)&read_data_from_addr);

assign no_conflict_to_addr_o = ~conflict_to_addr;

assign st_ld_conflict_o = (|(mq_st_bank_en&addr_rd_data_en)) | (|(mq_st_bank_en&tlb_rd_data_en));

//------------ set enable control ---------
wire [ 3:0] replace_set_en;
wire [ 3:0] refill_set_en;
wire [ 3:0] st_set_en;
wire [ 3:0] cache_set_en;
wire [ 3:0] replace_set_v;
wire [ 3:0] refill_set_v;
wire [ 3:0] st_set_v;
wire [ 3:0] cache_set_v;
dcache_decode_2_4 u3_dec_2_4(.in(set_replace[1:0]), .out(replace_set_v));
//dcache_decode_2_4 u4_dec_2_4(.in(set_refill[1:0] ), .out(refill_set_v ));
assign refill_set_v = replace_set_v; 
dcache_decode_2_4 u5_dec_2_4(.in(set_st[1:0]),      .out(st_set_v)     );
//dcache_decode_2_4 u6_dec_2_4(.in(set_cache[1:0]),   .out(cache_set_v)  );
assign cache_set_v  = replace_set_v;

assign replace_set_en = {4{valid_replace}} & replace_set_v;
assign refill_set_en  = {4{valid_refill}}  & refill_set_v;
assign st_set_en      = {4{valid_st}}      & st_set_v;
assign cache_set_en   = {4{valid_cache}}   & cache_set_v;

wire [63:0] select_dw_set0;
wire [63:0] select_dw_set1;
wire [63:0] select_dw_set2;
wire [63:0] select_dw_set3;
wire [31:0] select_sw_set0;
wire [31:0] select_sw_set1;
wire [31:0] select_sw_set2;
wire [31:0] select_sw_set3;

assign set_tag_wen = refill_set_en | cache_set_en;

assign set_tag_cen  = {4{(valid_replace|valid_refill|valid_cache)}} & (replace_set_v) |
                      {4{~(valid_replace|valid_refill|valid_cache)&(tlb_read_tag|addr_read_tag)}} & ALL_SET_EN;


//WEN of refill for each set
wire [3:0] set0_refill_wen;
wire [3:0] set1_refill_wen;
wire [3:0] set2_refill_wen;
wire [3:0] set3_refill_wen;

//WEN of store for each set
wire [3:0] set0_st_wen;
wire [3:0] set1_st_wen;
wire [3:0] set2_st_wen;
wire [3:0] set3_st_wen;

//REN of replace for each set
wire [3:0] set0_replace_ren;
wire [3:0] set1_replace_ren;
wire [3:0] set2_replace_ren;
wire [3:0] set3_replace_ren;

//REN of tlb read again for each set
wire [3:0] set0_tlb_rag_ren;
wire [3:0] set1_tlb_rag_ren;
wire [3:0] set2_tlb_rag_ren;
wire [3:0] set3_tlb_rag_ren;

//REN of addr read for each set
wire [3:0] set0_addr_ren;
wire [3:0] set1_addr_ren;
wire [3:0] set2_addr_ren;
wire [3:0] set3_addr_ren;

assign set0_refill_wen = refill_set_en[0] ? 4'b1111 : 4'b0000;
assign set1_refill_wen = refill_set_en[1] ? 4'b1111 : 4'b0000;
assign set2_refill_wen = refill_set_en[2] ? 4'b1111 : 4'b0000;
assign set3_refill_wen = refill_set_en[3] ? 4'b1111 : 4'b0000;

assign set0_st_wen = st_set_en[0] ? mq_st_bank_v : 4'b0000;
assign set1_st_wen = st_set_en[1] ? mq_st_bank_v : 4'b0000;
assign set2_st_wen = st_set_en[2] ? mq_st_bank_v : 4'b0000;
assign set3_st_wen = st_set_en[3] ? mq_st_bank_v : 4'b0000;

assign set0_replace_ren = replace_set_en[0] ? 4'b1111 : 4'b0000;
assign set1_replace_ren = replace_set_en[1] ? 4'b1111 : 4'b0000;
assign set2_replace_ren = replace_set_en[2] ? 4'b1111 : 4'b0000;
assign set3_replace_ren = replace_set_en[3] ? 4'b1111 : 4'b0000;

assign set0_tlb_rag_ren = tlb_rd_data_en;
assign set1_tlb_rag_ren = tlb_rd_data_en;
assign set2_tlb_rag_ren = tlb_rd_data_en;
assign set3_tlb_rag_ren = tlb_rd_data_en;

assign set0_addr_ren = addr_rd_data_en;
assign set1_addr_ren = addr_rd_data_en;
assign set2_addr_ren = addr_rd_data_en;
assign set3_addr_ren = addr_rd_data_en;

assign set0_bank_wen = set0_refill_wen | set0_st_wen;
assign set1_bank_wen = set1_refill_wen | set1_st_wen;
assign set2_bank_wen = set2_refill_wen | set2_st_wen;
assign set3_bank_wen = set3_refill_wen | set3_st_wen;

assign set0_bank_cen[0] = (set0_replace_ren[0]|set0_refill_wen[0]|set0_st_wen[0]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])& set0_tlb_rag_ren[0] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])&~tlb_read_data & set0_addr_ren[0];
assign set0_bank_cen[1] = (set0_replace_ren[1]|set0_refill_wen[1]|set0_st_wen[1]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])& set0_tlb_rag_ren[1] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])&~tlb_read_data & set0_addr_ren[1];
assign set0_bank_cen[2] = (set0_replace_ren[2]|set0_refill_wen[2]|set0_st_wen[2]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])& set0_tlb_rag_ren[2] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])&~tlb_read_data & set0_addr_ren[2];
assign set0_bank_cen[3] = (set0_replace_ren[3]|set0_refill_wen[3]|set0_st_wen[3]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])& set0_tlb_rag_ren[3] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])&~tlb_read_data & set0_addr_ren[3];

assign set1_bank_cen[0] = (set1_replace_ren[0]|set1_refill_wen[0]|set1_st_wen[0]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])& set1_tlb_rag_ren[0] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])&~tlb_read_data & set1_addr_ren[0];
assign set1_bank_cen[1] = (set1_replace_ren[1]|set1_refill_wen[1]|set1_st_wen[1]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])& set1_tlb_rag_ren[1] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])&~tlb_read_data & set1_addr_ren[1];
assign set1_bank_cen[2] = (set1_replace_ren[2]|set1_refill_wen[2]|set1_st_wen[2]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])& set1_tlb_rag_ren[2] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])&~tlb_read_data & set1_addr_ren[2];
assign set1_bank_cen[3] = (set1_replace_ren[3]|set1_refill_wen[3]|set1_st_wen[3]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])& set1_tlb_rag_ren[3] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])&~tlb_read_data & set1_addr_ren[3];

assign set2_bank_cen[0] = (set2_replace_ren[0]|set2_refill_wen[0]|set2_st_wen[0]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])& set2_tlb_rag_ren[0] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])&~tlb_read_data & set2_addr_ren[0];
assign set2_bank_cen[1] = (set2_replace_ren[1]|set2_refill_wen[1]|set2_st_wen[1]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])& set2_tlb_rag_ren[1] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])&~tlb_read_data & set2_addr_ren[1];
assign set2_bank_cen[2] = (set2_replace_ren[2]|set2_refill_wen[2]|set2_st_wen[2]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])& set2_tlb_rag_ren[2] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])&~tlb_read_data & set2_addr_ren[2];
assign set2_bank_cen[3] = (set2_replace_ren[3]|set2_refill_wen[3]|set2_st_wen[3]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])& set2_tlb_rag_ren[3] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])&~tlb_read_data & set2_addr_ren[3];

assign set3_bank_cen[0] = (set3_replace_ren[0]|set3_refill_wen[0]|set3_st_wen[0]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])& set3_tlb_rag_ren[0] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[0])&~tlb_read_data & set3_addr_ren[0];
assign set3_bank_cen[1] = (set3_replace_ren[1]|set3_refill_wen[1]|set3_st_wen[1]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])& set3_tlb_rag_ren[1] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[1])&~tlb_read_data & set3_addr_ren[1];
assign set3_bank_cen[2] = (set3_replace_ren[2]|set3_refill_wen[2]|set3_st_wen[2]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])& set3_tlb_rag_ren[2] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[2])&~tlb_read_data & set3_addr_ren[2];
assign set3_bank_cen[3] = (set3_replace_ren[3]|set3_refill_wen[3]|set3_st_wen[3]) | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])& set3_tlb_rag_ren[3] | 
                         ~(valid_replace|valid_refill|mq_st_bank_en[3])&~tlb_read_data & set3_addr_ren[3];

//------------- tag index and bank index -----------------
/*  note: two parts of code are same because laddr_replace, laddr_refill, laddr_cache are same
assign tag_index   =(valid_replace) ? laddr_replace[11:5] :
                     (valid_refill) ? laddr_refill[11:5]  :
                      (valid_cache) ? laddr_cache[11:5]   :
                     (tlb_read_tag) ? laddr_tlb_rag[11:5] : laddr_from_addr[11:5]; //addr_read_tag 
*/
assign tag_index   =(valid_replace|valid_refill|valid_cache) ? laddr_memq[11:5] :
                                              (tlb_read_tag) ? laddr_tlb_rag[11:5] : laddr_from_addr[11:5]; //addr_read_tag 

/*  note: two parts of code are same because laddr_replace, laddr_refill, laddr_st are same
assign bank0_index = (valid_replace) ? laddr_replace[11:5] :
                      (valid_refill) ? laddr_refill[11:5] :
                (valid_st&laddr_st[4:3]==2'b00) ? laddr_st[11:5] :
    (tlb_read_data&(laddr_tlb_rag[4:3]==2'b00)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))
*/
assign bank0_index = (valid_replace|valid_refill|(valid_st&laddr_st[4:3]==2'b00)) ? laddr_memq[11:5] :
                                      (tlb_read_data&(laddr_tlb_rag[4:3]==2'b00)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))

/*  note: two parts of code are same because laddr_replace, laddr_refill, laddr_st are same
assign bank1_index = (valid_replace) ? laddr_replace[11:5] :
                      (valid_refill) ? laddr_refill[11:5] :
                (valid_st&laddr_st[4:3]==2'b01) ? laddr_st[11:5] :
    (tlb_read_data&(laddr_tlb_rag[4:3]==2'b01)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))
*/
assign bank1_index = (valid_replace|valid_refill|(valid_st&laddr_st[4:3]==2'b01)) ? laddr_memq[11:5] :
                                      (tlb_read_data&(laddr_tlb_rag[4:3]==2'b01)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))

/*  note: two parts of code are same because laddr_replace, laddr_refill, laddr_st are same
assign bank2_index = (valid_replace) ? laddr_replace[11:5] :
                      (valid_refill) ? laddr_refill[11:5] :
                (valid_st&laddr_st[4:3]==2'b10) ? laddr_st[11:5] :
    (tlb_read_data&(laddr_tlb_rag[4:3]==2'b10)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))
*/
assign bank2_index = (valid_replace|valid_refill|(valid_st&laddr_st[4:3]==2'b10)) ? laddr_memq[11:5] :
                                      (tlb_read_data&(laddr_tlb_rag[4:3]==2'b10)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))

/*  note: two parts of code are same because laddr_replace, laddr_refill, laddr_st are same
assign bank3_index = (valid_replace) ? laddr_replace[11:5] :
                      (valid_refill) ? laddr_refill[11:5] :
                (valid_st&laddr_st[4:3]==2'b11) ? laddr_st[11:5] :
    (tlb_read_data&(laddr_tlb_rag[4:3]==2'b11)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))
*/
assign bank3_index = (valid_replace|valid_refill|(valid_st&laddr_st[4:3]==2'b11)) ? laddr_memq[11:5] :
                                      (tlb_read_data&(laddr_tlb_rag[4:3]==2'b11)) ? laddr_tlb_rag[11:5] :
                                                 laddr_from_addr[11:5]; //(addr_read_data&(laddr_from_addr[4:3]==2'b00))

assign op_cache5 = (addr_read_tag&(op_from_addr==`OP_CACHE5));
assign op_replace = valid_replace;

always @(posedge clock)
begin
    if (reset)
    begin
        op_replace_r <= 1'b0;
        set_nu_r <= 2'b00;
        dump_dirty_r <= 1'b0;
    end
    else
    begin

        op_replace_r <= op_replace;

        if (op_replace)
            dump_dirty_r <= ((set_replace[1:0]==2'b00) & set0_dirty_r[laddr_replace[11:5]]) |
                            ((set_replace[1:0]==2'b01) & set1_dirty_r[laddr_replace[11:5]]) |
                            ((set_replace[1:0]==2'b10) & set2_dirty_r[laddr_replace[11:5]]) |
                            ((set_replace[1:0]==2'b11) & set3_dirty_r[laddr_replace[11:5]]) ;

        if (op_replace)
            set_nu_r <= set_replace[1:0];
        else if (op_cache5)
            set_nu_r <= set_from_addr[1:0];
    end
end

dcache_decode_2_4 u7_dec_2_4(.in(set_nu_r), .out(set_nu_r_bitmap));

assign set_hit[3] = (dcachepaddr_i[31:12] == set3_tagin[20:1])&set3_tagin[0];
assign set_hit[2] = (dcachepaddr_i[31:12] == set2_tagin[20:1])&set2_tagin[0];
assign set_hit[1] = (dcachepaddr_i[31:12] == set1_tagin[20:1])&set1_tagin[0];
assign set_hit[0] = (dcachepaddr_i[31:12] == set0_tagin[20:1])&set0_tagin[0];

////// tag_to_tlb_o bus
wire [21:0] tag_to_tlb_tmp = tagout_dump;
assign tag_to_tlb_o = {8'h00, tag_to_tlb_tmp[21:0], tag_to_tlb_tmp[0], 1'b0};

wire [21:0] tag_out_tmp = {22{set_hit[0]}}&{set0_tagin[21:0]} |
                          {22{set_hit[1]}}&{set1_tagin[21:0]} |
                          {22{set_hit[2]}}&{set2_tagin[21:0]} |
                          {22{set_hit[3]}}&{set3_tagin[21:0]} ;

assign dcache_tag_o = {8'h00, tag_out_tmp[21:0], tag_out_tmp[0], 1'b0};

////// dcache_result_o bus
assign hit_dcache     = |set_hit;
assign hit_set_dcache = set_hit[0] ? 2'b00 :
                        set_hit[1] ? 2'b01 :
                        set_hit[2] ? 2'b10 :
                        set_hit[3] ? 2'b11 : 2'b00;

assign lock_dcache    = {set3_tagin[21]&set3_tagin[0], 
                         set2_tagin[21]&set2_tagin[0], 
                         set1_tagin[21]&set1_tagin[0], 
                         set0_tagin[21]&set0_tagin[0]};

load_data_sel u_ld_d_l_sel_0(.datain(set0_ram2dcache[277:22]), 
                             .offset(dcachepaddr_i[4:3]), 
                             .dataout(select_dw_set0));
assign select_sw_set0 = (dcachepaddr_i[2]) ? select_dw_set0[63:32] : select_dw_set0[31:0];

load_data_sel u_ld_d_l_sel_1(.datain(set1_ram2dcache[277:22]), 
                             .offset(dcachepaddr_i[4:3]), 
                             .dataout(select_dw_set1));
assign select_sw_set1 = (dcachepaddr_i[2]) ? select_dw_set1[63:32] : select_dw_set1[31:0];

load_data_sel u_ld_d_l_sel_2(.datain(set2_ram2dcache[277:22]), 
                             .offset(dcachepaddr_i[4:3]), 
                             .dataout(select_dw_set2));
assign select_sw_set2 = (dcachepaddr_i[2]) ? select_dw_set2[63:32] : select_dw_set2[31:0];

load_data_sel u_ld_d_l_sel_3(.datain(set3_ram2dcache[277:22]), 
                             .offset(dcachepaddr_i[4:3]), 
                             .dataout(select_dw_set3));
assign select_sw_set3 = (dcachepaddr_i[2]) ? select_dw_set3[63:32] : select_dw_set3[31:0];


    assign value_h_dcache = 32'h0;
    assign value_dcache   = {32{set_hit[0]}}&select_sw_set0 |
                            {32{set_hit[1]}}&select_sw_set1 |
                            {32{set_hit[2]}}&select_sw_set2 |
                            {32{set_hit[3]}}&select_sw_set3 ;


////// replace_dump_o bus
assign valid_dump = op_replace_r;
assign dirty_dump = dump_dirty_r;

assign tagout_dump  = {22{set_nu_r_bitmap[0]}}&{set0_ram2dcache[21:0]} |
                      {22{set_nu_r_bitmap[1]}}&{set1_ram2dcache[21:0]} |
                      {22{set_nu_r_bitmap[2]}}&{set2_ram2dcache[21:0]} |
                      {22{set_nu_r_bitmap[3]}}&{set3_ram2dcache[21:0]} ;
assign dataout_dump = {256{set_nu_r_bitmap[0]}}&{set0_ram2dcache[277:22]} |
                      {256{set_nu_r_bitmap[1]}}&{set1_ram2dcache[277:22]} |
                      {256{set_nu_r_bitmap[2]}}&{set2_ram2dcache[277:22]} |
                      {256{set_nu_r_bitmap[3]}}&{set3_ram2dcache[277:22]} ;

//ecc_replace_dump_o bus
////// -------------------------------
////// dcache_to_ram_o bus
wire [31:0] set0_dcache_byte_wen = (refill_set_en[0]|st_set_en[0]) ? data_wen_memq : 32'b0;
wire [31:0] set1_dcache_byte_wen = (refill_set_en[1]|st_set_en[1]) ? data_wen_memq : 32'b0;
wire [31:0] set2_dcache_byte_wen = (refill_set_en[2]|st_set_en[2]) ? data_wen_memq : 32'b0;
wire [31:0] set3_dcache_byte_wen = (refill_set_en[3]|st_set_en[3]) ? data_wen_memq : 32'b0;


wire [255:0] dcache_dataout = datain_memq;


//---------- set0 -----------
assign set0_data_bank3_D   = dcache_dataout[255:192];
assign set0_data_bank3_wen = set0_dcache_byte_wen[31:24];
assign set0_data_bank3_A   = bank3_index;
assign set0_data_bank3_cen = set0_bank_cen[3];

assign set0_data_bank2_D   = dcache_dataout[191:128];
assign set0_data_bank2_wen = set0_dcache_byte_wen[23:16];
assign set0_data_bank2_A   = bank2_index;
assign set0_data_bank2_cen = set0_bank_cen[2];

assign set0_data_bank1_D   = dcache_dataout[127: 64];
assign set0_data_bank1_wen = set0_dcache_byte_wen[15: 8];
assign set0_data_bank1_A   = bank1_index;
assign set0_data_bank1_cen = set0_bank_cen[1];

assign set0_data_bank0_D   = dcache_dataout[ 63: 0];
assign set0_data_bank0_wen = set0_dcache_byte_wen[ 7: 0];
assign set0_data_bank0_A   = bank0_index;
assign set0_data_bank0_cen = set0_bank_cen[0];

assign set0_tag_D   = tagin_memq;
assign set0_tag_wen = set_tag_wen[0];
assign set0_tag_A   = tag_index;
assign set0_tag_cen = set_tag_cen[0];

//---------- set1 -----------
assign set1_data_bank3_D   = dcache_dataout[255:192];
assign set1_data_bank3_wen = set1_dcache_byte_wen[31:24];
assign set1_data_bank3_A   = bank3_index;
assign set1_data_bank3_cen = set1_bank_cen[3];

assign set1_data_bank2_D   = dcache_dataout[191:128];
assign set1_data_bank2_wen = set1_dcache_byte_wen[23:16];
assign set1_data_bank2_A   = bank2_index;
assign set1_data_bank2_cen = set1_bank_cen[2];

assign set1_data_bank1_D   = dcache_dataout[127: 64];
assign set1_data_bank1_wen = set1_dcache_byte_wen[15: 8];
assign set1_data_bank1_A   = bank1_index;
assign set1_data_bank1_cen = set1_bank_cen[1];

assign set1_data_bank0_D   = dcache_dataout[ 63: 0];
assign set1_data_bank0_wen = set1_dcache_byte_wen[ 7: 0];
assign set1_data_bank0_A   = bank0_index;
assign set1_data_bank0_cen = set1_bank_cen[0];

assign set1_tag_D   = tagin_memq;
assign set1_tag_wen = set_tag_wen[1];
assign set1_tag_A   = tag_index;
assign set1_tag_cen = set_tag_cen[1];

//---------- set2 -----------
assign set2_data_bank3_D   = dcache_dataout[255:192];
assign set2_data_bank3_wen = set2_dcache_byte_wen[31:24];
assign set2_data_bank3_A   = bank3_index;
assign set2_data_bank3_cen = set2_bank_cen[3];

assign set2_data_bank2_D   = dcache_dataout[191:128];
assign set2_data_bank2_wen = set2_dcache_byte_wen[23:16];
assign set2_data_bank2_A   = bank2_index;
assign set2_data_bank2_cen = set2_bank_cen[2];

assign set2_data_bank1_D   = dcache_dataout[127: 64];
assign set2_data_bank1_wen = set2_dcache_byte_wen[15: 8];
assign set2_data_bank1_A   = bank1_index;
assign set2_data_bank1_cen = set2_bank_cen[1];

assign set2_data_bank0_D   = dcache_dataout[ 63: 0];
assign set2_data_bank0_wen = set2_dcache_byte_wen[ 7: 0];
assign set2_data_bank0_A   = bank0_index;
assign set2_data_bank0_cen = set2_bank_cen[0];

assign set2_tag_D   = tagin_memq;
assign set2_tag_wen = set_tag_wen[2];
assign set2_tag_A   = tag_index;
assign set2_tag_cen = set_tag_cen[2];

//---------- set3 -----------
assign set3_data_bank3_D   = dcache_dataout[255:192];
assign set3_data_bank3_wen = set3_dcache_byte_wen[31:24];
assign set3_data_bank3_A   = bank3_index;
assign set3_data_bank3_cen = set3_bank_cen[3];

assign set3_data_bank2_D   = dcache_dataout[191:128];
assign set3_data_bank2_wen = set3_dcache_byte_wen[23:16];
assign set3_data_bank2_A   = bank2_index;
assign set3_data_bank2_cen = set3_bank_cen[2];

assign set3_data_bank1_D   = dcache_dataout[127: 64];
assign set3_data_bank1_wen = set3_dcache_byte_wen[15: 8];
assign set3_data_bank1_A   = bank1_index;
assign set3_data_bank1_cen = set3_bank_cen[1];

assign set3_data_bank0_D   = dcache_dataout[ 63: 0];
assign set3_data_bank0_wen = set3_dcache_byte_wen[ 7: 0];
assign set3_data_bank0_A   = bank0_index;
assign set3_data_bank0_cen = set3_bank_cen[0];

assign set3_tag_D   = tagin_memq;
assign set3_tag_wen = set_tag_wen[3];
assign set3_tag_A   = tag_index;
assign set3_tag_cen = set_tag_cen[3];

//---------------- dirty bit ---------------
wire [ 3:0] set_dirty_wen;
assign set_dirty_wen = refill_set_en | ({4{valid_cache&~tagin_memq[0]}} & cache_set_v) | st_set_en;

wire [ 6:0] dirty_set_index;
assign dirty_set_index = (valid_st) ? laddr_st[11:5] : laddr_memq[11:5];
wire        dirty_set_value;
assign dirty_set_value = valid_st | valid_refill&dirty_refill;

always @(posedge clock)
begin
    if (set_dirty_wen[0])
        set0_dirty_r[dirty_set_index] <= dirty_set_value;

    if (set_dirty_wen[1])
        set1_dirty_r[dirty_set_index] <= dirty_set_value;

    if (set_dirty_wen[2])
        set2_dirty_r[dirty_set_index] <= dirty_set_value;

    if (set_dirty_wen[3])
        set3_dirty_r[dirty_set_index] <= dirty_set_value;
end



endmodule //godson_dcache_module



module dcache_decode_2_4(in, out);
input  [1:0] in;
output [3:0] out;

assign out[0] = in==2'b00;
assign out[1] = in==2'b01;
assign out[2] = in==2'b10;
assign out[3] = in==2'b11;

endmodule


module load_data_sel(datain, offset, dataout);
input  [255:0] datain;
input  [  1:0] offset; //word offset
output [ 63:0] dataout;

assign dataout = ({64{(offset==2'b00)}} & datain[ 63:  0]) |
                 ({64{(offset==2'b01)}} & datain[127: 64]) |
                 ({64{(offset==2'b10)}} & datain[191:128]) |
                 ({64{(offset==2'b11)}} & datain[255:192]) ;

endmodule
