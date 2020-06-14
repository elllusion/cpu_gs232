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

module godson_memqueue_module(
    clock,
    reset,
    commitbus_ex_i,
    
    cp0res_o,
    mmres_to_mmrs_o,
    
    store_ok_i,
    brbus_to_cp0_i,
    store_qid_o,
    cp0_forward_bus_o,
    cp0_cancel_bus_o,
    
    inst_valid_at_addr_i,
    inst_cache_at_addr_i,
    inst_sync_at_addr_i,
    memqueue_stall_o,
    memqueue_has_ll_o,
    
    dcache_value_i,
    dcache_hit_i,
    dcache_tag_i,

    tlb_to_memqueue_i,
    tlb_forward_bus_i, 
    cr_llbit_value_i,
    cr_cfg7_dcache_i,
    load_queue_full_o,
    store_queue_full_o,
    miss_req_queue_full_o,
    ex_memqueue_o,
    rand_num_o,
    ll_set_llbit_o,
    
    replace_dump_i,
    store_req_o,
    cache_req_o,
    replace_req_o,
    refill_req_o,
    memq_to_dcache_o,
    
    cp0_memres_i,
    cp0_mem_req_o,
    cp0_memraddr_o,
    cp0_memwaddr_o,
    
    inst_cache_block_o,
    inst_uncache_block_o,

    HB_DDBLIMPR,
    LOADDATA,
    LOADDATA_QID,
    LOADDATA_VALID,
    LOADDATA_H_VALID,
    LOADDATA_H,
    BYTELANE,

    duncache_valid_o,
    data_inter_conflict_o,
    missq_full_o,
    not_store_ok_o
   

);
// whole system signals
input   clock;
input   reset;
input   commitbus_ex_i;

// mmres bus
output  [ 98:0] cp0res_o;

output  [ 36:0] mmres_to_mmrs_o;

////// connect with queue
input   [  1:0] store_ok_i;
input   [  6:0] brbus_to_cp0_i;
output  [  7:0] store_qid_o;
output  [  8:0] cp0_forward_bus_o;
output  [  4:0] cp0_cancel_bus_o;

////// connect with addr module
input           inst_valid_at_addr_i;
input           inst_cache_at_addr_i;
input           inst_sync_at_addr_i;

///// stall signal output to addr module
output          memqueue_stall_o;
output          memqueue_has_ll_o;

///// connect with dcache module
input   [ 63:0] dcache_value_i;
input           dcache_hit_i;
input   [ 31:0] dcache_tag_i;

////// connect with tlb module
input   [`Ltlb_to_memq-1:0] tlb_to_memqueue_i;
input   [`Ltlb_forward-1:0] tlb_forward_bus_i; 
input           cr_llbit_value_i;
input   [  1:0] cr_cfg7_dcache_i;
output          load_queue_full_o;
output          store_queue_full_o;
output          miss_req_queue_full_o;
output          ex_memqueue_o;
output  [  2:0] rand_num_o;
output          ll_set_llbit_o;

////// connect with dcache module
input   [`Lreplace_dump-1:0] replace_dump_i;
output          store_req_o;
output          cache_req_o;
output          replace_req_o;
output  [  1:0] refill_req_o;
output  [`Lmemq_to_dcache-1:0] memq_to_dcache_o;

////// connect with buffer module
input   [`Lmemres-1:0] cp0_memres_i;
output  [`Lmemraddr-1:0] cp0_memraddr_o;
output  [`Lmemwaddr-1:0] cp0_memwaddr_o;

output          cp0_mem_req_o;

////// connect with icache module
output          inst_cache_block_o;
output          inst_uncache_block_o;

///// connect with hb module for EJTAG addr&data match
input         HB_DDBLIMPR;
output [31:0] LOADDATA;
output [ 3:0] LOADDATA_QID;
output        LOADDATA_VALID;
output        LOADDATA_H_VALID;
output [31:0] LOADDATA_H;
output [ 3:0] BYTELANE;

///// performance counter
output      duncache_valid_o;
output      data_inter_conflict_o;
output      missq_full_o;
output      not_store_ok_o;


// all fields of cp0res_o
wire        op_ctc1_wb;
wire        eret_deret_wb;
wire        condition_true;
wire        ddbl_mmres;
wire        ddbs_mmres;
wire        ddblimpr_mmres;
wire        ddbsimpr_mmres;
wire        bnt_mmres;
wire        valid_mmres;
wire [ 3:0] qid_mmres;
wire [ 7:0] op_mmres;
wire [ 2:0] fpqid_mmres;
wire [31:0] value_h_mmres;
wire [31:0] value_mmres;
wire        mod_mmres;
wire        tlbli_mmres;
wire        tlblr_mmres;
wire        tlbsi_mmres;
wire        tlbsr_mmres;
wire        adel_mmres;
wire        ades_mmres;
wire        dbe_mmres;
wire        watch_mmres;
wire        ri_mmres;

wire mmres_write_fpq;
wire mmres_wb_fpq;
assign mmres_write_fpq = 1'b0;
assign mmres_wb_fpq =1'b0;
assign cp0res_o[98]    = mmres_wb_fpq;
assign cp0res_o[97]    = mmres_write_fpq;
assign cp0res_o[96:94] = fpqid_mmres;
assign cp0res_o[93   ] = op_ctc1_wb;
assign cp0res_o[92   ] = eret_deret_wb;
assign cp0res_o[91   ] = condition_true;
assign cp0res_o[90   ] = ddbl_mmres;
assign cp0res_o[89   ] = ddbs_mmres;
assign cp0res_o[88   ] = ddblimpr_mmres;
assign cp0res_o[87   ] = ddbsimpr_mmres;
assign cp0res_o[86   ] = bnt_mmres;
assign cp0res_o[85   ] = valid_mmres;
assign cp0res_o[84:81] = qid_mmres;
assign cp0res_o[80:17] = {value_h_mmres, value_mmres};
assign cp0res_o[16   ] = 1'b0;
assign cp0res_o[15   ] = ri_mmres;
assign cp0res_o[14   ] = mod_mmres;
assign cp0res_o[13   ] = tlbli_mmres;
assign cp0res_o[12   ] = tlblr_mmres;
assign cp0res_o[11   ] = tlbsi_mmres;
assign cp0res_o[10   ] = tlbsr_mmres;
assign cp0res_o[ 9   ] = adel_mmres;
assign cp0res_o[ 8   ] = ades_mmres;
assign cp0res_o[ 7   ] = dbe_mmres;
assign cp0res_o[ 6   ] = watch_mmres;
assign cp0res_o[ 5: 0] = 6'h00;

////// all fields of mmres_to_mmrs_o
wire        valid_res_to_mmrs;
wire [3:0]  qid_res_to_mmrs;
wire [31:0] value_res_to_mmrs;

assign mmres_to_mmrs_o[0]    = valid_res_to_mmrs;
assign mmres_to_mmrs_o[4:1]  = qid_res_to_mmrs;
assign mmres_to_mmrs_o[36:5] = value_res_to_mmrs;

////// all fields of brbus_to_cp0_i
wire       brbus_brerr    = brbus_to_cp0_i[  0];
wire [5:0] brbus_brmask   = brbus_to_cp0_i[6:1];
////// all fields of cp0_forward_bus_o
wire       forward_valid;
wire [3:0] forward_qid;
wire [2:0] forward_fpqid;
wire       forward_fpq;

assign cp0_forward_bus_o[8]   = forward_fpq;
assign cp0_forward_bus_o[7:5] = forward_fpqid;
assign cp0_forward_bus_o[4  ] = forward_valid;
assign cp0_forward_bus_o[3:0] = forward_qid;

////// all fields of cp0_cancel_bus_o
wire        cancel_valid;
wire [3:0]  cancel_qid;
assign cp0_cancel_bus_o[4  ] = cancel_valid;
assign cp0_cancel_bus_o[3:0] = cancel_qid;


////// all fields of dcache_value_i
wire [31:0] value_h_dcache  = dcache_value_i[63:32];
wire [31:0] value_dcache    = dcache_value_i[31: 0];

////// all fields of tlb_to_memqueue_i
wire        tlb_ex_ddblimpr_h = tlb_to_memqueue_i[148    ];
wire [2:0]  tlb_fpqid         = tlb_to_memqueue_i[147:145];
wire        tlb_ex_ri         = tlb_to_memqueue_i[144    ];
wire [ 2:0] tlb_brqid         = tlb_to_memqueue_i[143:141];
wire [ 3:0] tlb_bytelane      = tlb_to_memqueue_i[140:137];
wire        tlb_cond_true     = tlb_to_memqueue_i[136    ];
wire        tlb_ejtag_dseg_en = tlb_to_memqueue_i[135    ];
wire        tlb_valid_in      = tlb_to_memqueue_i[134    ];
wire        tlb_ex            = tlb_to_memqueue_i[133    ];
wire        tlb_op_load       = tlb_to_memqueue_i[132    ];
wire        tlb_op_store      = tlb_to_memqueue_i[131    ];
wire        tlb_cached        = tlb_to_memqueue_i[130    ];
wire        tlb_uncache_acc   = tlb_to_memqueue_i[129    ];
wire        tlb_hit           = tlb_to_memqueue_i[128    ];
wire [ 1:0] tlb_hit_set       = tlb_to_memqueue_i[127:126];
wire [ 3:0] tlb_qid           = tlb_to_memqueue_i[125:122];
wire [ 7:0] tlb_op            = tlb_to_memqueue_i[121:114];
wire [31:0] tlb_paddr         = tlb_to_memqueue_i[113:82 ];
wire [31:0] tlb_value_h       = tlb_to_memqueue_i[81 :50 ];
wire [31:0] tlb_value         = tlb_to_memqueue_i[49 :18 ];
wire [ 3:0] tlb_lock          = tlb_to_memqueue_i[17 :14 ];
wire        tlb_ex_ddbl       = tlb_to_memqueue_i[13     ];
wire        tlb_ex_ddbs       = tlb_to_memqueue_i[12     ];
wire        tlb_ex_ddblimpr   = tlb_to_memqueue_i[11     ];
wire        tlb_ex_ddbsimpr   = tlb_to_memqueue_i[10     ];
wire        tlb_ex_cacheerr   = tlb_to_memqueue_i[9      ];
wire        tlb_ex_mcheck     = tlb_to_memqueue_i[8      ];
wire        tlb_ex_watch      = tlb_to_memqueue_i[7      ];
wire        tlb_ex_ades       = tlb_to_memqueue_i[6      ];
wire        tlb_ex_adel       = tlb_to_memqueue_i[5      ];
wire        tlb_ex_tlbsr      = tlb_to_memqueue_i[4      ];
wire        tlb_ex_tlbsi      = tlb_to_memqueue_i[3      ];
wire        tlb_ex_tlblr      = tlb_to_memqueue_i[2      ];
wire        tlb_ex_tlbli      = tlb_to_memqueue_i[1      ];
wire        tlb_ex_mod        = tlb_to_memqueue_i[0      ];

wire        tlb_brbus_cancel= brbus_brmask[tlb_brqid];
wire        tlb_valid       = tlb_valid_in & ~tlb_brbus_cancel;

////// all fields of tlb_forward_bus_i bus
wire        tlb_forward_valid;
wire [ 3:0] tlb_forward_qid   = tlb_forward_bus_i[3:0];
wire [ 2:0] tlb_forward_fpqid = tlb_forward_bus_i[7:5];
wire        fpq_forward       = tlb_forward_bus_i[8];

////// all fields of cp0_memraddr_o bus
wire        memraddr_valid;
wire [ 3:0] memraddr_width;
wire [31:0] memraddr_addr;
wire [ 2:0] memraddr_id;

assign cp0_memraddr_o[39:37] = memraddr_id;
assign cp0_memraddr_o[36   ] = memraddr_valid;
assign cp0_memraddr_o[35:32] = memraddr_width;
assign cp0_memraddr_o[31:0 ] = memraddr_addr;

//// memraddr of cp0
wire        cp0_memrd_valid;
wire [ 3:0] cp0_memrd_width;
wire [31:0] cp0_memrd_addr;


////// all fields of cp0_memwaddr_o bus
wire [255:0] memwaddr_data;
wire         memwaddr_valid;
wire [3:0]   memwaddr_width;
wire [31:0]  memwaddr_addr;
wire [3:0]   memwaddr_ben;
wire [2:0]   memwaddr_id;
wire [3:0]   memwaddr_acc_wen;

assign cp0_memwaddr_o[`Lmemwaddr-1:300] = memwaddr_acc_wen;
assign cp0_memwaddr_o[299:44] = memwaddr_data;
assign cp0_memwaddr_o[ 43:40] = memwaddr_ben;
assign cp0_memwaddr_o[ 39:37] = memwaddr_id;
assign cp0_memwaddr_o[ 36   ] = memwaddr_valid;
assign cp0_memwaddr_o[ 35:32] = memwaddr_width;
assign cp0_memwaddr_o[ 31:0 ] = memwaddr_addr;

//// memwaddr of cp0
wire [255:0] cp0_memwr_data;
wire         cp0_memwr_valid;
wire [3:0]   cp0_memwr_width;
wire [31:0]  cp0_memwr_addr;


////// all fields of cp0_memres_i bus
wire        memres_valid       = cp0_memres_i[0];
wire        memres_unc_acc_rdy = cp0_memres_i[1];
wire [ 2:0] memres_id          = cp0_memres_i[4:2];
wire [ 2:0] memres_count       = cp0_memres_i[7:5];
wire        memres_rd_rdy      = cp0_memres_i[8];
wire        memres_wr_rdy      = cp0_memres_i[9];
wire        memres_uncache_rdy = cp0_memres_i[10];

wire        memres_is_dw    = cp0_memres_i[11];
wire [31:0] memres_data     = cp0_memres_i[43:12];
wire [ 1:0] memres_wtq_valid= cp0_memres_i[45:44];


wire memres_is_block = (memres_id==3'b000) || (memres_id==3'b001) || (memres_id==3'b010);
wire [31:0] memres_addr_mid;

// to record which word of LDC1 has come back
reg         is_dw_r;

always @(posedge clock)
begin
    if (reset)
        is_dw_r <= 1'b0;
    else if (memres_valid & memres_is_dw)
        is_dw_r <= ~is_dw_r;
end

////// all fields of replace_dump_i
wire         valid_dump   = replace_dump_i[279    ];
wire         dirty_dump   = replace_dump_i[278    ];
wire [ 21:0] tagout_dump  = replace_dump_i[277:256];
wire [255:0] dataout_dump = replace_dump_i[255:  0];


////// all fields of store_req_o
wire         valid_st_req;         

assign store_req_o = valid_st_req;

wire [  7:0] op_st_req;     
wire [  1:0] set_st_req;     
wire [ 11:0] laddr_st_req; 
wire [ 31:0] wen_st_req;
wire [255:0] value_st_req;     

////// all fields of cache_req_o
wire        valid_cache_req;

assign      cache_req_o = valid_cache_req;

//1 indicate the replace of cache1 and cache21 has done
reg         cache1_21_r;

wire [  1:0] set_cache_req;
wire [ 11:0] laddr_cache_req;
wire [ 21:0] tagout_cache_req;

////// all fields of replace_req_o
wire        valid_replace;

assign replace_req_o = valid_replace ;

wire [ 1:0] set_replace;
wire [11:0] laddr_replace;

wire        valid_replace_missq;

////// all fields of refill_req_o
wire         valid_refill;
wire         dirty_refill;
wire [  1:0] set_refill;
wire [ 11:0] laddr_refill;
wire [ 21:0] tagout_refill;
wire [255:0] dataout_refill;
assign refill_req_o[1] = valid_refill;    
assign refill_req_o[0] = dirty_refill;    

// all fields of memq_to_dcache_o
wire [  1:0] st_set_memq;
wire [ 11:0] st_laddr_memq;
wire [  1:0] set_memq;
wire [ 11:0] laddr_memq;
wire         tag_wen_memq;
wire [ 21:0] tagout_memq;
wire [ 31:0] data_wen_memq;
wire [255:0] dataout_memq;

assign memq_to_dcache_o[338:337]  = st_set_memq;
assign memq_to_dcache_o[336:325]  = st_laddr_memq;
assign memq_to_dcache_o[324:323]  = set_memq;
assign memq_to_dcache_o[322:311]  = laddr_memq;
assign memq_to_dcache_o[310    ]  = tag_wen_memq;
assign memq_to_dcache_o[309:288]  = tagout_memq;
assign memq_to_dcache_o[287:256]  = data_wen_memq;
assign memq_to_dcache_o[255:  0]  = dataout_memq;


////// load queue
parameter   BLANK_LDQ       = 2'b00;
parameter   READY_LDQ       = 2'b01;
parameter   WAIT_BUF_LDQ    = 2'b10;
parameter   WAIT_RESULT_LDQ = 2'b11;

//the state of each load queue iterm
reg  [ 1:0] state_ld_q[3:0];
//this operation has trigered an exception
reg  [ 3:0] ex_ld_q;
reg  [ 7:0] op_ld_q[3:0];
reg  [ 3:0] qid_ld_q[3:0];
reg  [ 2:0] brqid_ld_q[3:0];
reg  [ 3:0] lock_ld_q[3:0];
reg  [31:0] addr_ld_q[3:0];
//indicate which byte is ready of low 32 bit of load. For CP0 and
//operations which has exception, it's 4'hf. But for prefetch inst
//we don't care about the return value, we only care about whether 
//it hit or not
reg  [ 3:0] brdy_ld_q[3:0];
//to save the low 32 bit of load return value
reg  [31:0] value_ld_q[3:0];
//same as brdy_ld_q, but for high 32 bit 
reg  [ 3:0] brdy_h_ld_q[3:0];
//to save the high 32 bit of load return value
reg  [31:0] value_h_ld_q[3:0];
reg  [ 3:0] ddblimpr_ld_q;
reg  [ 3:0] ddblimpr_h_ld_q;
wire [ 3:0] ldq_high_addr_match;


//from which iterm we find an blank iterm to record a new operation
reg  [ 1:0] wr_p_ldq; 
//from which iterm we find an ready operation to write back
reg  [ 1:0] wb_p_ldq; 

wire        ldq_all_blank;


////// store queue
parameter   BLANK_STQ       = 3'b000;
parameter   HIT_STQ         = 3'b001;
parameter   MISS_STQ        = 3'b010;
parameter   WAIT_MEM_STQ    = 3'b011;
parameter   WAIT_RESULT_STQ = 3'b100;
parameter   READY_STQ       = 3'b101;

//the state of each store queue iterm
reg  [ 2:0] state_st_q[1:0];
reg  [ 7:0] op_st_q[1:0];
reg  [ 3:0] qid_st_q[1:0];
reg  [ 2:0] brqid_st_q[1:0];
//the address of this operation is in uncache acceleration region
reg  [ 1:0] uncache_acc_st_q;
// which set a store/cache hit in 
reg  [ 1:0] hit_set_st_q[1:0];
reg  [ 3:0] lock_st_q[1:0];
reg  [31:0] addr_st_q[1:0];
//byte enable of low 32 bit of store. It's 4'h0 for uncache load.
reg  [ 3:0] ben_st_q[1:0]; 
//low 32 bit value of store, also save the low 32 bit result of load
reg  [31:0] value_st_q[1:0];
//high 32 bit value of store, also save the high 32 bit result of load
reg  [31:0] value_h_st_q[1:0];
reg  [ 1:0] st_ok_st_q;
reg  [ 1:0] ddblimpr_st_q;
reg  [ 1:0] ddblimpr_h_st_q;

reg         head_st;
wire        tail_st;


wire        head_op_dcache_stq;
wire        head_op_load;

// state vector of store queue
wire [ 1:0] s_blank_st_q;
wire [ 1:0] s_hit_st_q;
wire [ 1:0] s_miss_st_q;
wire [ 1:0] s_in_queue_st_q;
wire [ 1:0] s_wait_mem_st_q;
wire [ 1:0] s_wait_res_st_q;
wire [ 1:0] s_ready_st_q;   

wire        stq_all_blank;

wire [ 2:0] head_state_stq;
wire [ 7:0] head_op_stq;
wire [ 1:0] head_set_stq;
wire [ 2:0] head_fpqid_stq;
wire [ 3:0] head_qid_stq;
wire [ 3:0] head_ben_stq;
wire [ 3:0] head_ben_h_stq;
wire [31:0] head_addr_stq;
wire [31:0] head_value_stq;
wire [31:0] head_value_h_stq;
wire        head_st_ok_stq;
wire        head_ddblimpr_stq;
wire        head_ddblimpr_h_stq;
wire        head_acc_stq;

wire        st_wb_valid;

////// miss requst queue
parameter   INVALID_MRSQ    = 3'b111;

reg  [ 2:0] mrsq[5:0];

reg  [ 2:0] tail_mrsq;
reg  [ 2:0] head_mrsq;

wire        full_mrsq;

////// exception recorder buffer
reg         ddbl_r;
reg         ddbs_r;
reg         ddbsimpr_r;
reg         cacheerr_r;
reg         mcheck_r;
//reg         watch_r;
reg         ades_r;
reg         adel_r;
reg         tlbsr_r;
reg         tlbsi_r;
reg         tlblr_r;
reg         tlbli_r;
reg         mod_r;
reg         ri_r;
reg  [3:0]  bytelane_r;

always @(posedge clock)
begin
    if (reset|commitbus_ex_i)
    begin
        ddbl_r <= 1'b0;
        ddbs_r <= 1'b0;
        ddbsimpr_r <= 1'b0;
        cacheerr_r <= 1'b0;
        mcheck_r <= 1'b0;
        //watch_r <= 1'b0;
        ades_r <= 1'b0;
        adel_r <= 1'b0;
        tlbsr_r <= 1'b0;
        tlbsi_r <= 1'b0;
        tlblr_r <= 1'b0;
        tlbli_r <= 1'b0;
        mod_r <= 1'b0;
        ri_r <= 1'b0;
    end
    else if (tlb_valid & tlb_ex)
    begin
        ddbl_r <= tlb_ex_ddbl;
        ddbs_r <= tlb_ex_ddbs;
        ddbsimpr_r <= tlb_ex_ddbsimpr;
        cacheerr_r <= tlb_ex_cacheerr;
        mcheck_r <= tlb_ex_mcheck;
        //watch_r <= tlb_ex_watch;
        ades_r <= tlb_ex_ades;
        adel_r <= tlb_ex_adel;
        tlbsr_r <= tlb_ex_tlbsr;
        tlbsi_r <= tlb_ex_tlbsi;
        tlblr_r <= tlb_ex_tlblr;
        tlbli_r <= tlb_ex_tlbli;
        mod_r <= tlb_ex_mod;
        ri_r <= tlb_ex_ri;
    end
end

always @(posedge clock)
begin
    if (reset|commitbus_ex_i)
        bytelane_r <= 4'b0;
    else if (tlb_valid & tlb_ex_ddblimpr)
        bytelane_r <= tlb_bytelane;
end

////// miss queue
parameter   BLANK_MISSQ     = 3'b000;
parameter   VALID_MISSQ     = 3'b001;
parameter   MEM_READ_MISSQ  = 3'b010;
parameter   ALL_BACK_MISSQ  = 3'b011;
parameter   REPLACE_MISSQ   = 3'b100;
parameter   REFILL_MISSQ    = 3'b101;

reg [  2:0] state_miss_q[2:0];
//address of the first operation that apply this iterm
reg [ 31:0] addr_miss_q[2:0]; 
//lock information read from original set
reg [  3:0] lock_miss_q[2:0];
//which byte of this iterm has been ready
reg [ 31:0] ben_miss_q[2:0];
reg [255:0] data_miss_q[2:0];
//when this cacheline write into cache, it should be locked
reg [  2:0] set_lock_miss_q;
//this line has been written by store
reg [  2:0] dirty_miss_q;
//indicate a load miss fall in this line
reg [  2:0] ld_in_miss_q;
//record how many words come back from memory
reg [  3:0] back_count_miss_q[2:0];

//from which iterm we find an 
reg [1:0]   wr_in_p_r;

wire        missq_all_blank;

reg [1:0]   st_fill_index_r;

////  msq to missq request bus
wire        to_missq_req;
wire [2:0]  to_missq_queue_nu;
wire [7:0]  to_missq_op;
wire [3:0]  to_missq_lock;
wire [31:0] to_missq_paddr;
wire        to_missq_set_lock;
wire [3:0]  to_missq_ben;
wire [31:0] to_missq_value;
wire [3:0]  to_missq_ben_h;
wire [31:0] to_missq_value_h;

wire        to_missq_ready;

wire        replace_cache;
wire [1:0]  set_replace_cache;
wire [11:0] laddr_replace_cache;
wire        refill_cache;
wire [21:0] tag_refill_cache;

wire [31:0] store_wen_line_l;
wire [31:0] store_wen_line_h;
wire [31:0] store_wen_line;

wire [255:0] st_sw_datain_missq;
wire [255:0] st_dw_datain_missq;
wire [255:0] store_datain_missq;

wire        mrsq_cancel_valid;
wire [2:0]  mrsq_cancel_queue_nu;

wire        head_to_missq;
wire        first_load_to_missq;


////// miss queue
wire        missq_allowin;
wire        missq_read_mem;
wire        missq_allowin_st;

////// forward bus
reg         forward_valid_r;
reg [2:0]   forward_queue_nu_r;
reg [3:0]   forward_qid_r;

////// 
wire        replace_write_mem;
wire [31:0] replace_addr_wr_mem;
wire        uncache_acc_wr_mem;
wire        uncache_st_wr_mem;



////// uncache acc buffer
parameter   BLANK_ACC   = 2'b00;
parameter   IDLE_ACC    = 2'b01;
parameter   LINE_ACC    = 2'b10;
parameter   SPLIT_ACC   = 2'b11;

reg [  1:0] state_acc;
reg [ 26:0] addr_acc; //only the [31:5] of address
reg [ 31:0] ben_acc;
reg [255:0] data_acc;

reg  [2:0]  acc_word_cnt_r;


////// write back bus
wire        wb_valid;
wire [ 3:0] wb_ldq_v;
wire [ 1:0] wb_stq_v;
wire [ 3:0] wb_qid;
wire [ 7:0] wb_op;
wire [ 2:0] wb_fpqid;
wire [31:0] wb_value;
wire [31:0] wb_value_h;
wire        wb_bnt;
wire        wb_ex;
wire [ 2:0] wb_addr_offset;

wire        ldq_wb_valid;

////// replace broadcast bus
wire        replace_bc_valid;
wire [31:0] replace_bc_addr;


////// refill broadcast bus
wire        refill_bc_valid;
wire [31:0] refill_bc_addr;




wire tlb_has_ex = tlb_valid & tlb_ex;
wire tlb_op_prefetch = (tlb_op == `OP_PREF) || (tlb_op == `OP_PREFX);
wire tlb_op_sync = (tlb_op == `OP_SYNC);
wire tlb_op_cp0 = (tlb_op == `OP_MTC0)	 | (tlb_op == `OP_TLBR)	 |
                  (tlb_op == `OP_MFC0)	 | (tlb_op == `OP_TLBWI) |
                  (tlb_op == `OP_DI)	 | (tlb_op == `OP_TLBWR) |
                  (tlb_op == `OP_EI)	 | (tlb_op == `OP_ERET)	 |
                  (tlb_op == `OP_TLBP)	 | (tlb_op == `OP_DERET) ;
wire tlb_op_ldc1 = 1'b0;
wire tlb_op_sdc1 = 1'b0;
wire [1:0] tlb_offset = tlb_paddr[1:0];

wire tlb_op_cache29 = tlb_op == `OP_CACHE29;
wire tlb_op_dcache  = (tlb_op == `OP_CACHE1)  || (tlb_op == `OP_CACHE5) ||
                      (tlb_op == `OP_CACHE9)  || (tlb_op == `OP_CACHE17)||
                      (tlb_op == `OP_CACHE21) || tlb_op_cache29         ||
                      (tlb_op == `OP_SYNCI)   ;
wire tlb_op_icache  = (tlb_op == `OP_CACHE0)  || (tlb_op == `OP_CACHE8) ||
                      (tlb_op == `OP_CACHE16) || (tlb_op == `OP_CACHE28);
wire tlb_op_index_dcache = (tlb_op == `OP_CACHE1)  || (tlb_op == `OP_CACHE5) ||
                           (tlb_op == `OP_CACHE9);

wire        has_dcache_in_memqueue;

assign has_dcache_in_memqueue = head_op_dcache_stq & ~s_blank_st_q[head_st]; 

wire [3:0] load_byte_en;
wire [3:0] load_byte_en_h;

assign load_byte_en = tlb_bytelane;

assign load_byte_en_h = tlb_op_ldc1 ? 4'b1111 : 4'b0000;
// value from store queue
wire       head_add1_st = ~head_st;

wire [1:0] stq_op_sdc1;
assign stq_op_sdc1 = 2'b00;

wire [1:0] stq_addr_cmp;
assign stq_addr_cmp[0] = addr_st_q[0][31:3]==tlb_paddr[31:3];
assign stq_addr_cmp[1] = addr_st_q[1][31:3]==tlb_paddr[31:3];

wire [1:0] has_store = s_hit_st_q|s_miss_st_q;
wire [1:0] ld_hit_in_stq = has_store&stq_addr_cmp;

wire [3:0] stq_0_ben = (ld_hit_in_stq[0] & tlb_op_load &
                        (addr_st_q[0][2]==tlb_paddr[2]) ) ? ben_st_q[0] : 4'b0000;
wire [3:0] stq_0_h_ben = 4'b0000;

wire [3:0] stq_1_ben = (ld_hit_in_stq[1] & tlb_op_load &
                        (addr_st_q[1][2]==tlb_paddr[2]) ) ? ben_st_q[1] : 4'b0000;
wire [3:0] stq_1_h_ben = 4'b0000;

wire [3:0] stq_head_ben = head_st ? stq_1_ben : stq_0_ben;
wire [3:0] stq_head_h_ben = head_st ? stq_1_h_ben : stq_0_h_ben;
wire [3:0] stq_head_add1_ben = head_st ? stq_0_ben : stq_1_ben;
wire [3:0] stq_head_add1_h_ben = head_st ? stq_0_h_ben : stq_1_h_ben;

wire [7:0] stq_byte0, stq_byte1, stq_byte2, stq_byte3;
wire [3:0] stq_ben;
assign stq_byte0 = stq_head_add1_ben[0]   ? value_st_q[head_add1_st][7:0]   :
                                            value_st_q[head_st][7:0]; //stq_head_ben[0]
assign stq_byte1 = stq_head_add1_ben[1]   ? value_st_q[head_add1_st][15:8]   :
                                            value_st_q[head_st][15:8]; //stq_head_ben[1]

assign stq_byte2 = stq_head_add1_ben[2]   ? value_st_q[head_add1_st][23:16]   :
                                            value_st_q[head_st][23:16]; //stq_head_ben[2]
assign stq_byte3 = stq_head_add1_ben[3]   ? value_st_q[head_add1_st][31:24]   :
                                            value_st_q[head_st][31:24]; //stq_head_ben[3]
assign stq_ben = stq_0_ben | stq_1_ben; 
wire [7:0] stq_byte0_h, stq_byte1_h, stq_byte2_h, stq_byte3_h;
wire [3:0] stq_ben_h;
assign stq_byte0_h = 8'h00;
assign stq_byte1_h = 8'h00;
assign stq_byte2_h = 8'h00;
assign stq_byte3_h = 8'h00;
assign stq_ben_h   = 4'h0;

wire ld_in_memres_tmp = memres_valid & memres_is_block & tlb_op_load & 
                        (memres_addr_mid[31:3] == tlb_paddr[31:3]);

wire        ld_in_memres = ld_in_memres_tmp & (tlb_paddr[2]==memres_addr_mid[2]);
wire [3:0]  ld_in_memres_ben = ld_in_memres ? 4'b1111 : 4'b0000;
wire [31:0] ld_in_memres_value = memres_data;

wire [3:0]  ld_in_memres_ben_h   = 4'h0;
wire [31:0] ld_in_memres_value_h = 32'h0;

// value from miss queue
wire [26:0] select_addr = {27{memres_id==3'b000}}&addr_miss_q[0][31:5] |
                          {27{memres_id==3'b001}}&addr_miss_q[1][31:5] |
                          {27{memres_id==3'b010}}&addr_miss_q[2][31:5] ;
assign memres_addr_mid = {select_addr, memres_count[2:0], 2'b00};

wire [2:0] ld_in_missq;
assign ld_in_missq[0] = (state_miss_q[0]!=BLANK_MISSQ) & (addr_miss_q[0][31:5]==tlb_paddr[31:5]);
assign ld_in_missq[1] = (state_miss_q[1]!=BLANK_MISSQ) & (addr_miss_q[1][31:5]==tlb_paddr[31:5]);
assign ld_in_missq[2] = (state_miss_q[2]!=BLANK_MISSQ) & (addr_miss_q[2][31:5]==tlb_paddr[31:5]);

wire [31:0] ld_in_missq_ben_tmp = {32{tlb_op_load&ld_in_missq[0]}}&ben_miss_q[0] |
                                  {32{tlb_op_load&ld_in_missq[1]}}&ben_miss_q[1] |
                                  {32{tlb_op_load&ld_in_missq[2]}}&ben_miss_q[2] ;

wire [255:0] ld_in_missq_data_tmp = {256{tlb_op_load&ld_in_missq[0]}}&data_miss_q[0] |   
                                    {256{tlb_op_load&ld_in_missq[1]}}&data_miss_q[1] |
                                    {256{tlb_op_load&ld_in_missq[2]}}&data_miss_q[2] ;

wire [7:0] ld_in_missq_ben1 = {8{(tlb_paddr[4:3]==2'b00)}}&ld_in_missq_ben_tmp[ 7: 0] |
                              {8{(tlb_paddr[4:3]==2'b01)}}&ld_in_missq_ben_tmp[15: 8] |
                              {8{(tlb_paddr[4:3]==2'b10)}}&ld_in_missq_ben_tmp[23:16] |
                              {8{(tlb_paddr[4:3]==2'b11)}}&ld_in_missq_ben_tmp[31:24] ;
wire [63:0] ld_in_missq_data1 = {64{(tlb_paddr[4:3]==2'b00)}}&ld_in_missq_data_tmp[ 63:  0] |
                                {64{(tlb_paddr[4:3]==2'b01)}}&ld_in_missq_data_tmp[127: 64] |
                                {64{(tlb_paddr[4:3]==2'b10)}}&ld_in_missq_data_tmp[191:128] |
                                {64{(tlb_paddr[4:3]==2'b11)}}&ld_in_missq_data_tmp[255:192] ;

wire [ 3:0] ld_in_missq_ben = (tlb_paddr[2]==1'b0) ? ld_in_missq_ben1[3:0] : ld_in_missq_ben1[7:4];
wire [31:0] ld_in_missq_data = (tlb_paddr[2]==1'b0) ? ld_in_missq_data1[31:0] : ld_in_missq_data1[63:32];

wire [ 3:0] ld_in_missq_ben_h = 4'h0;
wire [31:0] ld_in_missq_data_h = 32'h0;


// load value in cpu, but not in cache now. the coming load should collect them
wire [ 3:0] ld_in_queue_ben;
wire [ 3:0] ld_in_queue_ben_h;

assign ld_in_queue_ben = stq_ben | ld_in_memres_ben | ld_in_missq_ben;
assign ld_in_queue_ben_h = 4'h0;

wire [7:0] in_queue_value1 = (stq_ben[0])    ? stq_byte0      :
                             (ld_in_missq_ben[0])  ? ld_in_missq_data[7:0] : ld_in_memres_value[7:0];

wire [7:0] in_queue_value2 = (stq_ben[1])    ? stq_byte1      :
                             (ld_in_missq_ben[1])  ? ld_in_missq_data[15:8] : ld_in_memres_value[15:8];

wire [7:0] in_queue_value3 = (stq_ben[2])    ? stq_byte2      :
                             (ld_in_missq_ben[2])  ? ld_in_missq_data[23:16] : ld_in_memres_value[23:16];

wire [7:0] in_queue_value4 = (stq_ben[3])    ? stq_byte3      :
                             (ld_in_missq_ben[3])  ? ld_in_missq_data[31:24] : ld_in_memres_value[31:24];

wire [7:0] in_queue_value_h1 = (stq_ben_h[0])    ? stq_byte0_h      :
                               (ld_in_missq_ben_h[0])  ? ld_in_missq_data_h[7:0] : ld_in_memres_value_h[7:0];

wire [7:0] in_queue_value_h2 = (stq_ben_h[1])    ? stq_byte1_h      :
                               (ld_in_missq_ben_h[1])  ? ld_in_missq_data_h[15:8] : ld_in_memres_value_h[15:8];

wire [7:0] in_queue_value_h3 = (stq_ben_h[2])    ? stq_byte2_h      :
                               (ld_in_missq_ben_h[2])  ? ld_in_missq_data_h[23:16] : ld_in_memres_value_h[23:16];

wire [7:0] in_queue_value_h4 = (stq_ben_h[3])    ? stq_byte3_h      :
                               (ld_in_missq_ben_h[3])  ? ld_in_missq_data_h[31:24] : ld_in_memres_value_h[31:24];


// to load queue bus
wire valid_to_ld_q = ~load_queue_full_o & ~miss_req_queue_full_o & tlb_valid_in & 
                     ( tlb_ex                                                    |
                      (tlb_cached&tlb_op_load)                                   |
                      (~tlb_cached&(tlb_op_load|tlb_op_store)&tlb_ejtag_dseg_en) |
                       tlb_op_cp0                                                |
                       tlb_op_icache                                             |
                       tlb_op_sync                                               |
                       tlb_op_prefetch) ;


wire [3:0] load_rdy   = ~load_byte_en;
wire [3:0] load_rdy_h = ~load_byte_en_h;

wire prefetch_hit = dcache_hit_i | (|ld_in_missq);
wire load_all_in_queue = &(load_rdy|ld_in_queue_ben);
wire load_hit = dcache_hit_i | load_all_in_queue;

wire [ 1:0] state_to_ld_q =((load_hit&tlb_op_load&tlb_cached)|
                            (tlb_op_prefetch & prefetch_hit) ) ?   READY_LDQ :  
                           (tlb_ex         | tlb_op_sync     |
                            tlb_op_cp0     | tlb_op_icache   |
                            (tlb_op_prefetch & ~tlb_cached)  |
                            (~tlb_cached&(tlb_op_load|tlb_op_store)&tlb_ejtag_dseg_en)) ? READY_LDQ : WAIT_BUF_LDQ;

wire        cond_true_to_ld_q = tlb_cond_true;
wire        ex_to_ld_q = tlb_ex;
wire        ddblimpr_to_ld_q = tlb_ex_ddblimpr;
wire        ddblimpr_h_to_ld_q = tlb_ex_ddblimpr_h;
wire [ 7:0] op_to_ld_q = tlb_op;
wire [ 2:0] fpqid_to_ld_q = tlb_fpqid;
wire [ 3:0] qid_to_ld_q = tlb_qid;
wire [ 2:0] brqid_to_ld_q = tlb_brqid;
wire [ 3:0] lock_to_ld_q = tlb_lock;
wire [31:0] addr_to_ld_q = tlb_paddr;
wire [ 3:0] brdy_to_ld_q = {4{tlb_ex|~tlb_cached|tlb_op_cp0|tlb_op_icache|tlb_op_sync}} |
                           load_rdy | 
                           (dcache_hit_i ? load_byte_en : ld_in_queue_ben);


wire [ 3:0] brdy_h_to_ld_q = {4{tlb_ex|~tlb_cached|tlb_op_cp0|tlb_op_icache|tlb_op_sync}} |
                             load_rdy_h | 
                             (dcache_hit_i ? load_byte_en_h : ld_in_queue_ben_h);



// to store queue bus
wire        valid_to_stq = ~store_queue_full_o & ~miss_req_queue_full_o & tlb_valid_in & ~tlb_ex &  
                           ((tlb_op_store&tlb_cached) | 
                            (~tlb_cached&(tlb_op_load|tlb_op_store)&~tlb_ejtag_dseg_en) | 
                            tlb_op_dcache);


wire [ 2:0] state_to_stq = (tlb_op_store&tlb_cached&dcache_hit_i | 
                            tlb_op_dcache&~tlb_op_index_dcache&tlb_cached&dcache_hit_i |
                            tlb_op_index_dcache)    ?  HIT_STQ   :
                           (tlb_op_store&tlb_cached&~dcache_hit_i |
                            tlb_op_dcache&~tlb_op_index_dcache&tlb_cached&~dcache_hit_i |
                            tlb_op_dcache&~tlb_op_index_dcache&~tlb_cached) ? MISS_STQ : WAIT_MEM_STQ;
wire        ddblimpr_to_stq = tlb_ex_ddblimpr;
wire        ddblimpr_h_to_stq = tlb_ex_ddblimpr_h;
wire [ 7:0] op_to_stq  = tlb_op;
wire [ 2:0] fpqid_to_stq = tlb_fpqid;
wire [ 3:0] qid_to_stq = tlb_qid;
wire [ 2:0] brqid_to_stq = tlb_brqid;
wire        uncache_acc_to_stq = tlb_uncache_acc;
wire [ 1:0] hit_set_to_stq = tlb_hit_set;
wire [ 3:0] lock_to_stq = tlb_lock;
wire [31:0] addr_to_stq = tlb_paddr;
wire [ 2:0] mrsq_index_to_stq = tail_mrsq;
wire [31:0] value_to_stq;
wire [31:0] value_h_to_stq;
wire [ 3:0] ben_to_stq;

assign ben_to_stq = tlb_bytelane;
assign value_to_stq = tlb_op_cache29 ? dcache_tag_i : tlb_value;
assign value_h_to_stq = tlb_value_h;


// load queue 
wire [3:0] new_entry_in_ldq;
wire [3:0] wr_back_ldq;
wire [3:0] enter_missq_ldq;
wire [3:0] hit_on_memres_tmp;
wire [3:0] hit_on_memres;
wire [3:0] hit_on_memres_h;
wire [3:0] prefetch_enter_missq;
wire [3:0] load_enter_missq;

wire [3:0] blank_ldq = {(state_ld_q[3]==BLANK_LDQ),
                        (state_ld_q[2]==BLANK_LDQ),
                        (state_ld_q[1]==BLANK_LDQ),
                        (state_ld_q[0]==BLANK_LDQ)
                       };
                       
wire [3:0] ready_ldq = {(state_ld_q[3]==READY_LDQ),
                        (state_ld_q[2]==READY_LDQ),
                        (state_ld_q[1]==READY_LDQ),
                        (state_ld_q[0]==READY_LDQ)
                       };

wire [3:0] wait_buf_ldq = {(state_ld_q[3]==WAIT_BUF_LDQ),
                           (state_ld_q[2]==WAIT_BUF_LDQ),
                           (state_ld_q[1]==WAIT_BUF_LDQ),
                           (state_ld_q[0]==WAIT_BUF_LDQ)
                          };
                          
wire [3:0] wait_result_ldq = {(state_ld_q[3]==WAIT_RESULT_LDQ),
                              (state_ld_q[2]==WAIT_RESULT_LDQ),
                              (state_ld_q[1]==WAIT_RESULT_LDQ),
                              (state_ld_q[0]==WAIT_RESULT_LDQ)
                             };

wire [3:0] prefetch_ldq = {((op_ld_q[3]==`OP_PREF)|(op_ld_q[3]==`OP_PREFX)),
                           ((op_ld_q[2]==`OP_PREF)|(op_ld_q[2]==`OP_PREFX)), 
                           ((op_ld_q[1]==`OP_PREF)|(op_ld_q[1]==`OP_PREFX)), 
                           ((op_ld_q[0]==`OP_PREF)|(op_ld_q[0]==`OP_PREFX)) 
                          };

assign load_queue_full_o = ~|blank_ldq;
assign ex_memqueue_o = (|ex_ld_q) | (|ddblimpr_ld_q) | (|ddblimpr_st_q) | (|ddblimpr_h_ld_q) | (|ddblimpr_h_st_q);
assign ldq_all_blank = &blank_ldq;

wire [3:0] brbus_cancel_ldq;
assign brbus_cancel_ldq[0] = ~blank_ldq[0] & brbus_brmask[brqid_ld_q[0]];
assign brbus_cancel_ldq[1] = ~blank_ldq[1] & brbus_brmask[brqid_ld_q[1]];
assign brbus_cancel_ldq[2] = ~blank_ldq[2] & brbus_brmask[brqid_ld_q[2]];
assign brbus_cancel_ldq[3] = ~blank_ldq[3] & brbus_brmask[brqid_ld_q[3]];

wire [3:0] in_ldq_index_v;
wire [1:0] in_ldq_index;

mem_tool_first_one_4_4_from_i u0_f1_4_4_from_i(.in(blank_ldq), .i(wr_p_ldq), .out(in_ldq_index_v));
mem_tool_encode_4_2 u0_enc_4_2(.in(in_ldq_index_v), .out(in_ldq_index));
assign new_entry_in_ldq = {4{~tlb_brbus_cancel}} & {4{valid_to_ld_q}} & in_ldq_index_v;

assign wr_back_ldq = wb_ldq_v;

assign enter_missq_ldq[3] = (wait_buf_ldq[3]) & to_missq_ready & 
                            (to_missq_queue_nu==3'b011) & missq_allowin;

assign enter_missq_ldq[2] = (wait_buf_ldq[2]) & to_missq_ready & 
                            (to_missq_queue_nu==3'b010) & missq_allowin;

assign enter_missq_ldq[1] = (wait_buf_ldq[1]) & to_missq_ready & 
                            (to_missq_queue_nu==3'b001) & missq_allowin;

assign enter_missq_ldq[0] = (wait_buf_ldq[0]) & to_missq_ready & 
                            (to_missq_queue_nu==3'b000) & missq_allowin;

assign load_enter_missq = enter_missq_ldq & ~prefetch_ldq;
assign prefetch_enter_missq = enter_missq_ldq & prefetch_ldq;

assign ldq_high_addr_match[0] = addr_ld_q[0][31:5]==memres_addr_mid[31:5];
assign ldq_high_addr_match[1] = addr_ld_q[1][31:5]==memres_addr_mid[31:5];
assign ldq_high_addr_match[2] = addr_ld_q[2][31:5]==memres_addr_mid[31:5];
assign ldq_high_addr_match[3] = addr_ld_q[3][31:5]==memres_addr_mid[31:5];

assign hit_on_memres_tmp[3] = memres_valid & memres_is_block & (addr_ld_q[3][4:3]==memres_addr_mid[4:3]);
assign hit_on_memres_tmp[2] = memres_valid & memres_is_block & (addr_ld_q[2][4:3]==memres_addr_mid[4:3]);
assign hit_on_memres_tmp[1] = memres_valid & memres_is_block & (addr_ld_q[1][4:3]==memres_addr_mid[4:3]);
assign hit_on_memres_tmp[0] = memres_valid & memres_is_block & (addr_ld_q[0][4:3]==memres_addr_mid[4:3]);

wire hit_on_memres_tmp3 = ldq_high_addr_match[3] & hit_on_memres_tmp[3] & (addr_ld_q[3][2]==memres_addr_mid[2]);
wire hit_on_memres_tmp2 = ldq_high_addr_match[2] & hit_on_memres_tmp[2] & (addr_ld_q[2][2]==memres_addr_mid[2]);
wire hit_on_memres_tmp1 = ldq_high_addr_match[1] & hit_on_memres_tmp[1] & (addr_ld_q[1][2]==memres_addr_mid[2]);
wire hit_on_memres_tmp0 = ldq_high_addr_match[0] & hit_on_memres_tmp[0] & (addr_ld_q[0][2]==memres_addr_mid[2]);

assign hit_on_memres[3] = hit_on_memres_tmp3 & (brdy_ld_q[3]!=4'b1111);
assign hit_on_memres[2] = hit_on_memres_tmp2 & (brdy_ld_q[2]!=4'b1111);
assign hit_on_memres[1] = hit_on_memres_tmp1 & (brdy_ld_q[1]!=4'b1111);
assign hit_on_memres[0] = hit_on_memres_tmp0 & (brdy_ld_q[0]!=4'b1111);

wire hit_on_memres_h_tmp3 = ldq_high_addr_match[3] & hit_on_memres_tmp[3] & (1'b1==memres_addr_mid[2]);
wire hit_on_memres_h_tmp2 = ldq_high_addr_match[2] & hit_on_memres_tmp[2] & (1'b1==memres_addr_mid[2]);
wire hit_on_memres_h_tmp1 = ldq_high_addr_match[1] & hit_on_memres_tmp[1] & (1'b1==memres_addr_mid[2]);
wire hit_on_memres_h_tmp0 = ldq_high_addr_match[0] & hit_on_memres_tmp[0] & (1'b1==memres_addr_mid[2]);


wire [3:0] prefetch_hit_on_memres;
assign prefetch_hit_on_memres[3] = ldq_high_addr_match[3] & prefetch_ldq[3] & memres_valid & memres_is_block;
assign prefetch_hit_on_memres[2] = ldq_high_addr_match[2] & prefetch_ldq[2] & memres_valid & memres_is_block;
assign prefetch_hit_on_memres[1] = ldq_high_addr_match[1] & prefetch_ldq[1] & memres_valid & memres_is_block;
assign prefetch_hit_on_memres[0] = ldq_high_addr_match[0] & prefetch_ldq[0] & memres_valid & memres_is_block;

wire [3:0] in_en_ex_ldq   = (reset|commitbus_ex_i) ? 4'b1111 : (new_entry_in_ldq | wr_back_ldq | brbus_cancel_ldq); 

wire [3:0] in_en_cond_true_ldq = new_entry_in_ldq;
wire [3:0] in_en_op_ldq   = new_entry_in_ldq; 
wire [3:0] in_en_qid_ldq  = new_entry_in_ldq; 
wire [3:0] in_en_brqid_ldq= new_entry_in_ldq;
wire [3:0] in_en_lock_ldq = new_entry_in_ldq; 
wire [3:0] in_en_brdy_ldq = new_entry_in_ldq | hit_on_memres;
wire [3:0] in_en_addr_ldq = new_entry_in_ldq; 
wire [3:0] in_en_value_ldq = new_entry_in_ldq | hit_on_memres;

wire [3:0] in_en_ddblimpr_ldq = {4{(reset|commitbus_ex_i)}} | new_entry_in_ldq | wr_back_ldq | brbus_cancel_ldq;


wire all_rdy_ldq3 = prefetch_hit_on_memres[3] | hit_on_memres_tmp3;
wire all_rdy_ldq2 = prefetch_hit_on_memres[2] | hit_on_memres_tmp2;
wire all_rdy_ldq1 = prefetch_hit_on_memres[1] | hit_on_memres_tmp1;
wire all_rdy_ldq0 = prefetch_hit_on_memres[0] | hit_on_memres_tmp0;

wire [3:0] all_rdy_ldq;
assign all_rdy_ldq[3] = all_rdy_ldq3 & (wait_buf_ldq[3] | wait_result_ldq[3]);
assign all_rdy_ldq[2] = all_rdy_ldq2 & (wait_buf_ldq[2] | wait_result_ldq[2]);
assign all_rdy_ldq[1] = all_rdy_ldq1 & (wait_buf_ldq[1] | wait_result_ldq[1]);
assign all_rdy_ldq[0] = all_rdy_ldq0 & (wait_buf_ldq[0] | wait_result_ldq[0]);

wire [3:0] in_en_state_ldq = (reset|commitbus_ex_i) ? 4'b1111 :
                                (new_entry_in_ldq | wr_back_ldq | brbus_cancel_ldq |
                                 enter_missq_ldq  | all_rdy_ldq);

wire [3:0] miss_ld_in_ldq;
assign miss_ld_in_ldq[3] = (wait_buf_ldq[3] | wait_result_ldq[3]);
assign miss_ld_in_ldq[2] = (wait_buf_ldq[2] | wait_result_ldq[2]);
assign miss_ld_in_ldq[1] = (wait_buf_ldq[1] | wait_result_ldq[1]);
assign miss_ld_in_ldq[0] = (wait_buf_ldq[0] | wait_result_ldq[0]);

wire [31:0] mem_to_ldq_l[3:0];
wire [31:0] mem_to_ldq_h[3:0];

assign mem_to_ldq_l[3] = memres_data;
assign mem_to_ldq_l[2] = memres_data;
assign mem_to_ldq_l[1] = memres_data;
assign mem_to_ldq_l[0] = memres_data;

assign mem_to_ldq_h[3] = memres_data;
assign mem_to_ldq_h[2] = memres_data;
assign mem_to_ldq_h[1] = memres_data;
assign mem_to_ldq_h[0] = memres_data;

integer ldq_i;
always @(posedge clock)
begin
    for (ldq_i=0; ldq_i<4; ldq_i=ldq_i+1)
    begin
        if (in_en_ex_ldq[ldq_i])
        begin
            ex_ld_q[ldq_i] <= (reset|commitbus_ex_i|wr_back_ldq[ldq_i]|brbus_cancel_ldq[ldq_i]) ? 1'b0 : ex_to_ld_q;
        end

        if (in_en_ddblimpr_ldq[ldq_i])
        begin
            ddblimpr_ld_q[ldq_i] <= (reset|commitbus_ex_i|wr_back_ldq[ldq_i]|brbus_cancel_ldq[ldq_i]) ? 1'b0 : ddblimpr_to_ld_q;
            ddblimpr_h_ld_q[ldq_i] <= (reset|commitbus_ex_i|wr_back_ldq[ldq_i]|brbus_cancel_ldq[ldq_i]) ? 1'b0 : ddblimpr_h_to_ld_q;
        end
        
        if (in_en_op_ldq[ldq_i])
        begin
            op_ld_q[ldq_i] <= op_to_ld_q;
        end

        if (in_en_qid_ldq[ldq_i])
        begin
            qid_ld_q[ldq_i] <= qid_to_ld_q;
        end

        if (in_en_brqid_ldq[ldq_i])
        begin
            brqid_ld_q[ldq_i] <= brqid_to_ld_q;
        end

        if (in_en_lock_ldq[ldq_i])
        begin
            lock_ld_q[ldq_i] <= lock_to_ld_q;
        end

        if (in_en_addr_ldq[ldq_i])
        begin
            addr_ld_q[ldq_i] <= addr_to_ld_q;
        end

        if (in_en_brdy_ldq[ldq_i])
        begin
            brdy_ld_q[ldq_i] <= new_entry_in_ldq[ldq_i] ? brdy_to_ld_q : 4'b1111;
        end

/*
        if (in_en_value_ldq[ldq_i])
        begin
            value_ld_q[ldq_i] <= new_entry_in_ldq[ldq_i] ? value_to_ld_q : 
                                 {(brdy_ld_q[ldq_i][3] ? value_ld_q[ldq_i][31:24] : memres_data[31:24]),
                                  (brdy_ld_q[ldq_i][2] ? value_ld_q[ldq_i][23:16] : memres_data[23:16]),
                                  (brdy_ld_q[ldq_i][1] ? value_ld_q[ldq_i][15: 8] : memres_data[15: 8]),
                                  (brdy_ld_q[ldq_i][0] ? value_ld_q[ldq_i][ 7: 0] : memres_data[ 7: 0])
                                 } ;
        end
*/
        if (in_en_value_ldq[ldq_i])
        begin
            value_ld_q[ldq_i][ 7: 0] <= (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en & 
                                         ~(stq_ben[0] |ld_in_missq_ben[0] | ld_in_memres_ben[0])) ? value_dcache[ 7: 0] :
                                        (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en &
                                          (stq_ben[0] |ld_in_missq_ben[0] | ld_in_memres_ben[0])) ? in_queue_value1     :
                                                                       (new_entry_in_ldq[ldq_i])  ?    tlb_value[ 7: 0] : 
                                                                       (~brdy_ld_q[ldq_i][0])     ? mem_to_ldq_l[ldq_i][ 7: 0] : 
                                                                                               value_ld_q[ldq_i][ 7: 0];
                                         
            value_ld_q[ldq_i][15: 8] <= (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en & 
                                         ~(stq_ben[1] |ld_in_missq_ben[1] | ld_in_memres_ben[1])) ? value_dcache[15: 8] :
                                        (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en &
                                          (stq_ben[1] |ld_in_missq_ben[1] | ld_in_memres_ben[1])) ? in_queue_value2     :
                                                                       (new_entry_in_ldq[ldq_i])  ?    tlb_value[15: 8] : 
                                                                       (~brdy_ld_q[ldq_i][1])     ? mem_to_ldq_l[ldq_i][15: 8] : 
                                                                                               value_ld_q[ldq_i][15: 8];

            value_ld_q[ldq_i][23:16] <= (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en & 
                                         ~(stq_ben[2] |ld_in_missq_ben[2] | ld_in_memres_ben[2])) ? value_dcache[23:16] :
                                        (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en &
                                          (stq_ben[2] |ld_in_missq_ben[2] | ld_in_memres_ben[2])) ? in_queue_value3     :
                                                                       (new_entry_in_ldq[ldq_i])  ?    tlb_value[23:16] : 
                                                                       (~brdy_ld_q[ldq_i][2])     ? mem_to_ldq_l[ldq_i][23:16] : 
                                                                                               value_ld_q[ldq_i][23:16];

            value_ld_q[ldq_i][31:24] <= (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en & 
                                         ~(stq_ben[3] |ld_in_missq_ben[3] | ld_in_memres_ben[3])) ? value_dcache[31:24] :
                                        (new_entry_in_ldq[ldq_i] & tlb_op_load &~tlb_ex & ~tlb_ejtag_dseg_en &
                                          (stq_ben[3] |ld_in_missq_ben[3] | ld_in_memres_ben[3])) ? in_queue_value4     :
                                                                       (new_entry_in_ldq[ldq_i])  ?    tlb_value[31:24] : 
                                                                       (~brdy_ld_q[ldq_i][3])     ? mem_to_ldq_l[ldq_i][31:24] : 
                                                                                               value_ld_q[ldq_i][31:24];
        end


        if (new_entry_in_ldq[ldq_i])
        begin
            value_h_ld_q[ldq_i] <= tlb_value_h;
        end

        if (in_en_state_ldq[ldq_i])
        begin
            state_ld_q[ldq_i] <= 
                            (reset|commitbus_ex_i|wr_back_ldq[ldq_i]|brbus_cancel_ldq[ldq_i]) ? BLANK_LDQ:
                            (all_rdy_ldq[ldq_i]|prefetch_enter_missq[ldq_i]) ? READY_LDQ : 
			    (enter_missq_ldq[ldq_i]) ? WAIT_RESULT_LDQ : state_to_ld_q;//new_entry_in_ldq[ldq_i]
        end

    end //for
end //always

always @(posedge clock)
begin
    if (reset|commitbus_ex_i)
    begin
        wr_p_ldq <= 2'b11;
        wb_p_ldq <= 2'b11;
    end
    else
    begin
        if (valid_to_ld_q & ~tlb_brbus_cancel)
            wr_p_ldq <= in_ldq_index;

        if (ldq_wb_valid)
            wb_p_ldq <= forward_queue_nu_r[1:0];
    end
end


////// store queue
assign head_state_stq = state_st_q[head_st];
assign head_set_stq = hit_set_st_q[head_st];
assign head_qid_stq = qid_st_q[head_st];
assign head_op_stq = op_st_q[head_st];
assign head_ben_stq = (!s_blank_st_q[head_st]) ? ben_st_q[head_st] : 4'b0000;
assign head_fpqid_stq = 3'b0;
assign head_ben_h_stq = 4'b0000;
assign head_addr_stq = addr_st_q[head_st];
assign head_value_stq = value_st_q[head_st];
assign head_value_h_stq = value_h_st_q[head_st];
assign head_st_ok_stq = st_ok_st_q[head_st];
assign head_ddblimpr_stq = ddblimpr_st_q[head_st];
assign head_ddblimpr_h_stq = ddblimpr_h_st_q[head_st];
assign head_acc_stq = uncache_acc_st_q[head_st];

assign s_blank_st_q    = {state_st_q[1]==BLANK_STQ,    state_st_q[0]==BLANK_STQ};
assign s_hit_st_q      = {state_st_q[1]==HIT_STQ,      state_st_q[0]==HIT_STQ};
assign s_miss_st_q     = {state_st_q[1]==MISS_STQ,     state_st_q[0]==MISS_STQ};
assign s_wait_mem_st_q = {state_st_q[1]==WAIT_MEM_STQ, state_st_q[0]==WAIT_MEM_STQ};
assign s_wait_res_st_q = {state_st_q[1]==WAIT_RESULT_STQ, state_st_q[0]==WAIT_RESULT_STQ};
assign s_ready_st_q    = {state_st_q[1]==READY_STQ,    state_st_q[0]==READY_STQ};

assign stq_all_blank = &s_blank_st_q;

assign not_store_ok_o  = ~s_blank_st_q[head_st] & ~head_st_ok_stq;

wire [1:0] brbus_cancel_stq;
assign brbus_cancel_stq[0] = ~s_blank_st_q[0] & brbus_brmask[brqid_st_q[0]];
assign brbus_cancel_stq[1] = ~s_blank_st_q[1] & brbus_brmask[brqid_st_q[1]];

wire [1:0] head_st_v = head_st ? 2'b10:2'b01;
wire [1:0] tail_st_v = tail_st ? 2'b10:2'b01;

assign store_queue_full_o = ~s_blank_st_q[tail_st];

assign store_qid_o[3:0] = (~s_blank_st_q[0]) ? qid_st_q[0] : tlb_qid;
assign store_qid_o[7:4] = (~s_blank_st_q[1]) ? qid_st_q[1] : tlb_qid;

wire [1:0] new_entry_in_stq;
assign new_entry_in_stq = {2{~tlb_brbus_cancel & valid_to_stq}} & tail_st_v;

wire [1:0] in_en_op_stq = new_entry_in_stq;
wire [1:0] in_en_qid_stq = new_entry_in_stq;
wire [1:0] in_en_brqid_stq = new_entry_in_stq;
wire [1:0] in_en_uncache_acc_stq = new_entry_in_stq;
wire [1:0] in_en_hit_set_stq;
wire [1:0] in_en_lock_stq = new_entry_in_stq;
wire [1:0] in_en_addr_stq = new_entry_in_stq;
wire [1:0] in_en_ben_stq  = new_entry_in_stq;

//load listen value from memres bus
wire head_op_ldc1_stq = 1'b0;

//wire ld_in_memres_stq_tmp1 = (addr_st_q[head_st][4:2]==memres_addr[4:2]) & stq_high_addr_match_r & 
//                             (head_op_stq==`OP_LWL | head_op_stq==`OP_LWR | addr_st_q[head_st][1:0]==memres_addr[1:0]);
wire ld_in_memres_stq_tmp1 = memres_id==3'b110;
wire ld_in_memres_stq_tmp = memres_valid & s_wait_res_st_q[head_st] & ld_in_memres_stq_tmp1;

wire ld_in_memres_stq = ld_in_memres_stq_tmp & 
                        ((head_op_ldc1_stq&(~is_dw_r)&memres_is_dw) | 
                         (~head_op_ldc1_stq&~memres_is_dw)) ;

wire ld_in_memres_stq_h = ld_in_memres_stq_tmp & head_op_ldc1_stq & is_dw_r & memres_is_dw;

wire [1:0] value_hit_memres = head_st_v & {2{ld_in_memres_stq}};
wire [1:0] value_h_hit_memres = head_st_v & {2{ld_in_memres_stq_h}};

wire [1:0] in_en_value_stq = new_entry_in_stq | value_hit_memres;
wire [1:0] in_en_value_h_stq = new_entry_in_stq | value_h_hit_memres;

////  msq to missq request bus
reg  [2:0] mrsq_head_point;
reg        mrsq_head_point_valid;
wire [2:0] mrsq_head_point_next;
wire       mrsq_head_point_valid_next;
reg  [2:0] first_load_point;
reg        first_load_point_valid;
wire [2:0] first_load_point_next;
wire       first_load_point_valid_next;
wire [2:0] mrsq_head_num;
wire [2:0] first_load_num;

wire load_cancel;
wire store_cancel;

wire head_op_cache29_stq = head_op_stq==`OP_CACHE29;
wire sc_not_write = head_op_stq==`OP_SC & ~|head_ben_stq;
//store queue head write back
wire head_stq_hit_ok = head_st_ok_stq & ~valid_replace_missq & ~valid_refill & ~forward_valid_r;
//hit 1 cycle can complete
wire st_wb_valid_hit_1 = s_hit_st_q[head_st] & head_stq_hit_ok &
                         (head_op_stq==`OP_SB   | head_op_stq==`OP_SH  | head_op_stq==`OP_SW  |
                          head_op_stq==`OP_SC   | head_op_stq==`OP_SWL | head_op_stq==`OP_SWR |
                          head_op_stq==`OP_CACHE5 | head_op_stq==`OP_CACHE9 |
                          head_op_stq==`OP_CACHE17 | head_op_cache29_stq);
//hit must 2 cycle complete
wire st_wb_valid_hit_2 = s_hit_st_q[head_st] & head_st_ok_stq & cache1_21_r &
                         (head_op_stq==`OP_CACHE1  | 
                          head_op_stq==`OP_CACHE21 |
                          head_op_stq==`OP_SYNCI   );
wire st_wb_valid_miss = s_miss_st_q[head_st] & 
                        (((head_op_stq==`OP_CACHE21 | head_op_stq==`OP_CACHE17 | head_op_stq==`OP_SYNCI) | 
                          (sc_not_write & ~forward_valid_r)) | 
                         ((mrsq_head_num[2:1]==2'b10) & (mrsq_head_num[0]==head_st) & head_stq_hit_ok & missq_allowin_st));

wire uncache_st_mem_req_tmp = s_wait_mem_st_q[head_st] & head_st_ok_stq & 
                              ~head_op_load & ~(forward_valid_r);

wire uncache_st_acc_req    = uncache_st_mem_req_tmp &  head_acc_stq;
wire uncache_st_no_acc_req_tmp1 = uncache_st_mem_req_tmp & ~head_acc_stq;
wire uncache_st_no_acc_req_tmp = uncache_st_no_acc_req_tmp1 & memres_wr_rdy & memres_uncache_rdy;


wire uncache_st_no_acc_req = uncache_st_no_acc_req_tmp & ~(valid_replace|replace_write_mem);

wire uncache_st_no_acc_req_real = uncache_st_no_acc_req & ~sc_not_write;

wire st_into_acc_allow;
wire st_wb_valid_uc_st = (uncache_st_acc_req & st_into_acc_allow) |
                         (uncache_st_no_acc_req & state_acc==BLANK_ACC) |
                         (s_wait_mem_st_q[head_st] & ~forward_valid_r & sc_not_write) ; 

wire head_ready_st_q = s_ready_st_q[head_st];
wire st_wb_valid_uc_ld = head_ready_st_q & forward_valid_r & (forward_queue_nu_r[2:0]=={2'b10, head_st});
assign st_wb_valid = (st_wb_valid_hit_1 | st_wb_valid_hit_2 | st_wb_valid_miss | st_wb_valid_uc_st | st_wb_valid_uc_ld);
wire [1:0] wr_back_stq = head_st_v & {2{st_wb_valid}};

assign duncache_valid_o = st_wb_valid_uc_st | st_wb_valid_uc_ld;

//change hit or miss state in store queue
wire [1:0] replace_refill_cmp_low;
assign replace_refill_cmp_low[0] = addr_st_q[0][11:5]==refill_bc_addr[11:5];
assign replace_refill_cmp_low[1] = addr_st_q[1][11:5]==refill_bc_addr[11:5];

wire [1:0] replace_cmp_stq;
assign replace_cmp_stq[0] = (addr_st_q[0][31:12]==replace_bc_addr[31:12]) & replace_refill_cmp_low[0];
assign replace_cmp_stq[1] = (addr_st_q[1][31:12]==replace_bc_addr[31:12]) & replace_refill_cmp_low[1];
wire [1:0] index_dcache_stq;
assign index_dcache_stq[0] = (op_st_q[0]==`OP_CACHE1) | (op_st_q[0]==`OP_CACHE5) | (op_st_q[0]==`OP_CACHE9);
assign index_dcache_stq[1] = (op_st_q[1]==`OP_CACHE1) | (op_st_q[1]==`OP_CACHE5) | (op_st_q[1]==`OP_CACHE9);
wire [1:0] hit_to_miss_stq;
assign hit_to_miss_stq[0] = s_hit_st_q[0] & replace_cmp_stq[0] & replace_bc_valid & ~index_dcache_stq[0];
assign hit_to_miss_stq[1] = s_hit_st_q[1] & replace_cmp_stq[1] & replace_bc_valid & ~index_dcache_stq[1];

wire [1:0] refill_cmp_stq;
assign refill_cmp_stq[0] = (addr_st_q[0][31:12]==refill_bc_addr[31:12]) & replace_refill_cmp_low[0];
assign refill_cmp_stq[1] = (addr_st_q[1][31:12]==refill_bc_addr[31:12]) & replace_refill_cmp_low[1];

wire [1:0] miss_to_hit_stq;
assign miss_to_hit_stq[0] = s_miss_st_q[0] & refill_cmp_stq[0] & refill_bc_valid;
assign miss_to_hit_stq[1] = s_miss_st_q[1] & refill_cmp_stq[1] & refill_bc_valid;

assign head_op_dcache_stq = (head_op_stq == `OP_CACHE1) || (head_op_stq == `OP_CACHE5) ||
                            (head_op_stq == `OP_CACHE9) || (head_op_stq == `OP_CACHE17)||
                            (head_op_stq == `OP_CACHE21)|| head_op_cache29_stq         ||
                            (head_op_stq == `OP_SYNCI)  ;


assign head_op_load = (head_op_stq==`OP_LB)      || (head_op_stq==`OP_LH)      ||
                      (head_op_stq==`OP_LW)      || (head_op_stq==`OP_LBU)     ||
                      (head_op_stq==`OP_LHU)     || (head_op_stq==`OP_LL)      ||
                      (head_op_stq==`OP_LBUX)    || (head_op_stq==`OP_LHX)     ||
                      (head_op_stq==`OP_LWX)     || 
                      (head_op_stq==`OP_LWL)     || (head_op_stq==`OP_LWR)     ;

wire ld_to_mem_req_tmp1 = s_wait_mem_st_q[head_st] & head_st_ok_stq & head_op_load;
wire ld_to_mem_req_tmp = ld_to_mem_req_tmp1 & memres_rd_rdy & memres_uncache_rdy;
wire ld_to_mem_req = ld_to_mem_req_tmp & ~missq_read_mem;
wire ld_to_mem = ld_to_mem_req & state_acc==BLANK_ACC;

wire [1:0] ld_to_mem_stq = head_st_v & {2{ld_to_mem}};

wire ld_all_back_stq = ld_in_memres_stq_tmp & ((~head_op_ldc1_stq) | (head_op_ldc1_stq & is_dw_r)); 

wire [1:0] ld_ready_stq = head_st_v & {2{ld_all_back_stq}};


wire [1:0] to_blank_stq =  wr_back_stq | brbus_cancel_stq;
wire [1:0] in_en_state_stq = {2{reset|commitbus_ex_i}} |
                             to_blank_stq    | new_entry_in_stq |
                             hit_to_miss_stq | miss_to_hit_stq  |
                             ld_to_mem_stq   | ld_ready_stq;


wire [1:0] in_en_ddblimpr_stq = {2{(reset|commitbus_ex_i)}} | new_entry_in_stq | wr_back_stq | brbus_cancel_stq;

assign in_en_hit_set_stq = new_entry_in_stq | miss_to_hit_stq;

wire [1:0] head_add1_st_v  = ~head_st_v;
wire [1:0] in_en_st_ok_stq;

assign in_en_st_ok_stq[0] = ~s_blank_st_q[0] | (~tlb_brbus_cancel & valid_to_stq & ~tail_st);
assign in_en_st_ok_stq[1] = ~s_blank_st_q[1] | (~tlb_brbus_cancel & valid_to_stq &  tail_st);

wire [31:0] mem_to_stq_l;
wire [31:0] mem_to_stq_h;

assign mem_to_stq_l = memres_data;
assign mem_to_stq_h = memres_data;

integer stq_i;
always @(posedge clock)
begin
    for (stq_i=0; stq_i<2; stq_i=stq_i+1)
    begin
        if (in_en_ddblimpr_stq[stq_i])
        begin
            ddblimpr_st_q[stq_i] <= (reset|commitbus_ex_i|wr_back_stq[stq_i]|brbus_cancel_stq[stq_i]) ? 1'b0 : ddblimpr_to_stq;
            ddblimpr_h_st_q[stq_i] <= (reset|commitbus_ex_i|wr_back_stq[stq_i]|brbus_cancel_stq[stq_i]) ? 1'b0 : ddblimpr_h_to_stq;
        end
        
        if (in_en_op_stq[stq_i])
            op_st_q[stq_i] <= op_to_stq;
            
        if (in_en_qid_stq[stq_i])
        begin
            qid_st_q[stq_i] <= qid_to_stq;
        end   
            
        if (in_en_brqid_stq[stq_i])
            brqid_st_q[stq_i] <= brqid_to_stq;
            
        if (in_en_uncache_acc_stq[stq_i])
            uncache_acc_st_q[stq_i] <= uncache_acc_to_stq; 

        if (in_en_hit_set_stq[stq_i])
            hit_set_st_q[stq_i] <= (new_entry_in_stq[stq_i]) ? hit_set_to_stq :
                                                               set_refill; //miss_to_hit_stq[stq_i]

        if (in_en_lock_stq[stq_i])
            lock_st_q[stq_i] <= lock_to_stq;

        if (in_en_addr_stq[stq_i])
            addr_st_q[stq_i] <= addr_to_stq;

        if (in_en_ben_stq[stq_i])
            ben_st_q[stq_i] <= ben_to_stq;
            
        if (reset | commitbus_ex_i | wr_back_stq[stq_i] | in_en_st_ok_stq[stq_i])
            st_ok_st_q[stq_i] <= (reset|commitbus_ex_i|wr_back_stq[stq_i]) ? 1'b0 : store_ok_i[stq_i];

        if (in_en_value_stq[stq_i])
            value_st_q[stq_i] <= new_entry_in_stq[stq_i] ? value_to_stq : mem_to_stq_l;

        if (new_entry_in_stq[stq_i])
            value_h_st_q[stq_i] <= value_h_to_stq;

        if (in_en_state_stq[stq_i])
            state_st_q[stq_i] <= 
                             (reset|commitbus_ex_i|to_blank_stq[stq_i]) ? BLANK_STQ :
                             (hit_to_miss_stq[stq_i]) ? MISS_STQ    :
                             (miss_to_hit_stq[stq_i]) ? HIT_STQ     :
                             (ld_to_mem_stq[stq_i]) ? WAIT_RESULT_STQ   :
                             (ld_ready_stq[stq_i]) ? READY_STQ      : state_to_stq; //new_entry_in_stq[stq_i]
    end //for
end //always

//not blank next cycle (not consider reset and commitbus_ex)
wire [1:0] n_blank_nc_stq = (~s_blank_st_q & ~to_blank_stq) | new_entry_in_stq;
wire head_st_next = (head_st==1'b0) ? (~n_blank_nc_stq[0]&n_blank_nc_stq[1]) : (n_blank_nc_stq[1] | ~n_blank_nc_stq[0]);

always @(posedge clock)
begin
    if (reset|commitbus_ex_i)
        head_st <= 1'b0;
    else
        head_st <= head_st_next;
end

assign tail_st = (head_st==1'b0) ? ~s_blank_st_q[0]&s_blank_st_q[1] : s_blank_st_q[1]|~s_blank_st_q[0];

wire [3:0] uncache_mem_width;

wire lb    = head_op_stq == `OP_LB;
wire lbu   = head_op_stq == `OP_LBU;
wire lbux  = head_op_stq == `OP_LBUX;
wire lh    = head_op_stq == `OP_LH;
wire lhu   = head_op_stq == `OP_LHU;
wire lhx   = head_op_stq == `OP_LHX;
wire lw    = (head_op_stq == `OP_LW) | (head_op_stq == `OP_LWC1);
wire lwx   = head_op_stq == `OP_LWX;
wire ll    = head_op_stq == `OP_LL;
wire lwl   = head_op_stq == `OP_LWL;
wire lwr   = head_op_stq == `OP_LWR;
wire ld    = 1'b0;
wire sb    = head_op_stq == `OP_SB;
wire sh    = head_op_stq == `OP_SH;
wire sw    = (head_op_stq == `OP_SW) | (head_op_stq == `OP_SWC1);
wire sc    = head_op_stq == `OP_SC;
wire swl00 = (head_op_stq == `OP_SWL)&&(head_addr_stq[1:0]==2'b00);
wire swl01 = (head_op_stq == `OP_SWL)&&(head_addr_stq[1:0]==2'b01);
wire swl10 = (head_op_stq == `OP_SWL)&&(head_addr_stq[1:0]==2'b10);
wire swl11 = (head_op_stq == `OP_SWL)&&(head_addr_stq[1:0]==2'b11);
wire swr00 = (head_op_stq == `OP_SWR)&&(head_addr_stq[1:0]==2'b00);
wire swr01 = (head_op_stq == `OP_SWR)&&(head_addr_stq[1:0]==2'b01);
wire swr10 = (head_op_stq == `OP_SWR)&&(head_addr_stq[1:0]==2'b10);
wire swr11 = (head_op_stq == `OP_SWR)&&(head_addr_stq[1:0]==2'b11);
wire sd    = 1'b0;

wire width_sel1=lb|lbu|lbux|sb|swl00|swr11;
wire width_sel2=lh|lhu|lhx |sh|swl01|swr10;
wire width_sel3=swl10|swr01;
wire width_sel4=lw|lwx|ll  |sw|sc   |swl11|swr00|lwl|lwr;
wire width_sel5=ld|sd;

// I don't know why encode width in this way, the old code of ambe interface is too 
// hard to read, so I have to copy the old code :(
assign uncache_mem_width[3]=1'b1;
assign uncache_mem_width[2]=width_sel5;
assign uncache_mem_width[1]=width_sel3|width_sel4|width_sel5;
assign uncache_mem_width[0]=width_sel2|width_sel4|width_sel5;



////// miss request queue
wire [2:0] select_mrsq_num;
wire [2:0] select_mrsq_point;
//wire    new_in_mrsq =  ((valid_to_ld_q & tlb_op_load & tlb_cached & ~dcache_hit_i & ~tlb_ex) | 
//                         (valid_to_stq & tlb_cached & (~tlb_op_dcache | tlb_op_cache29)) ) ;
wire new_in_mrsq = ((~dcache_hit_i & (tlb_op_load|tlb_op_prefetch) & ~load_queue_full_o) | 
                    ((tlb_op_store & ~(tlb_op==`OP_SC&~|tlb_bytelane) & ~store_queue_full_o) | tlb_op_cache29)) & 
                   ~tlb_ex & tlb_cached & tlb_valid_in & ~miss_req_queue_full_o;
wire    valid_to_mrsq = new_in_mrsq;
//wire    valid_to_mrsq = ~tlb_brbus_cancel & new_in_mrsq; 
            //even the new_in iterm is brbus canceled, we record it in mrsq, it will be invalided by
            //cancel_miss_mrsq or wr_out_mrsq

wire [2:0] num_new_in = (valid_to_stq) ? {2'b10, tail_st} : {1'b0, in_ldq_index}; 

//
//wire [5:0] brbus_cancel_queue = {brbus_cancel_stq, brbus_cancel_ldq};
//wire mrsq_brbus_cancel = ~(&select_mrsq_num[2:1]) & brbus_cancel_queue[select_mrsq_num];
wire [2:0] select_mrsq_brqid = (select_mrsq_num[2]==1'b0) ? (brqid_ld_q[select_mrsq_num[1:0]]) : 
                                                            (brqid_st_q[select_mrsq_num[0]]);
wire mrsq_brbus_cancel = ~(&select_mrsq_num[2:1]) & brbus_brmask[select_mrsq_brqid];

wire [5:0]  in_en_mrsq;
wire [5:0]  new_entry_in_mrsq;
wire [5:0]  wr_out_mrsq;
wire [5:0]  hit_to_miss_mrsq;
wire [5:0]  cancel_miss_mrsq;
wire [5:0]  brcancel_mrsq;
wire [5:0]  to_invalid_mrsq;

wire [5:0]  tail_mrsq_v;
mem_tool_decode_3_6 u0_dec_3_6(.in(tail_mrsq), .out(tail_mrsq_v));
assign new_entry_in_mrsq = tail_mrsq_v & {6{valid_to_mrsq}};

wire [5:0]  select_mrsq_p_v;
mem_tool_decode_3_6 u1_dec_3_6(.in(select_mrsq_point), .out(select_mrsq_p_v));
assign wr_out_mrsq = select_mrsq_p_v & {6{(to_missq_ready&missq_allowin)|mrsq_cancel_valid}};

assign cancel_miss_mrsq[0] = (valid_to_stq & mrsq[0]=={2'b10, tail_st}) | (valid_to_ld_q & mrsq[0]=={1'b0, in_ldq_index});
assign cancel_miss_mrsq[1] = (valid_to_stq & mrsq[1]=={2'b10, tail_st}) | (valid_to_ld_q & mrsq[1]=={1'b0, in_ldq_index});
assign cancel_miss_mrsq[2] = (valid_to_stq & mrsq[2]=={2'b10, tail_st}) | (valid_to_ld_q & mrsq[2]=={1'b0, in_ldq_index});
assign cancel_miss_mrsq[3] = (valid_to_stq & mrsq[3]=={2'b10, tail_st}) | (valid_to_ld_q & mrsq[3]=={1'b0, in_ldq_index});
assign cancel_miss_mrsq[4] = (valid_to_stq & mrsq[4]=={2'b10, tail_st}) | (valid_to_ld_q & mrsq[4]=={1'b0, in_ldq_index});
assign cancel_miss_mrsq[5] = (valid_to_stq & mrsq[5]=={2'b10, tail_st}) | (valid_to_ld_q & mrsq[5]=={1'b0, in_ldq_index});

assign brcancel_mrsq = {6{mrsq_brbus_cancel}} & select_mrsq_p_v;

assign to_invalid_mrsq = wr_out_mrsq | cancel_miss_mrsq | {6{(reset|commitbus_ex_i)}};
assign in_en_mrsq = wr_out_mrsq | new_entry_in_mrsq | cancel_miss_mrsq | {6{(reset|commitbus_ex_i)}};

integer mrsq_i;
always @(posedge clock)
begin
    for (mrsq_i=0; mrsq_i<6; mrsq_i=mrsq_i+1)
    begin
        if (in_en_mrsq[mrsq_i])
            mrsq[mrsq_i] <= (to_invalid_mrsq[mrsq_i]) ? INVALID_MRSQ  : num_new_in; //new_entry_in_mrsq[mrsq_i]
    end //for
end //always

wire [5:0] mrsq_invalid;
assign mrsq_invalid[0] = mrsq[0]==INVALID_MRSQ;
assign mrsq_invalid[1] = mrsq[1]==INVALID_MRSQ;
assign mrsq_invalid[2] = mrsq[2]==INVALID_MRSQ;
assign mrsq_invalid[3] = mrsq[3]==INVALID_MRSQ;
assign mrsq_invalid[4] = mrsq[4]==INVALID_MRSQ;
assign mrsq_invalid[5] = mrsq[5]==INVALID_MRSQ;

wire [5:0] mrsq_invalid_next;
assign mrsq_invalid_next[0] = to_invalid_mrsq[0] | mrsq_invalid[0];
assign mrsq_invalid_next[1] = to_invalid_mrsq[1] | mrsq_invalid[1];
assign mrsq_invalid_next[2] = to_invalid_mrsq[2] | mrsq_invalid[2];
assign mrsq_invalid_next[3] = to_invalid_mrsq[3] | mrsq_invalid[3];
assign mrsq_invalid_next[4] = to_invalid_mrsq[4] | mrsq_invalid[4];
assign mrsq_invalid_next[5] = to_invalid_mrsq[5] | mrsq_invalid[5];

assign miss_req_queue_full_o = ~mrsq_invalid[tail_mrsq];

////  msq to missq request bus
wire [5:0]  ld_miss_in_mrsq;
assign ld_miss_in_mrsq[0] = !mrsq[0][2];
assign ld_miss_in_mrsq[1] = !mrsq[1][2];
assign ld_miss_in_mrsq[2] = !mrsq[2][2];
assign ld_miss_in_mrsq[3] = !mrsq[3][2];
assign ld_miss_in_mrsq[4] = !mrsq[4][2];
assign ld_miss_in_mrsq[5] = !mrsq[5][2];

wire [5:0]  ld_miss_in_mrsq_next;
assign ld_miss_in_mrsq_next[0] = (ld_miss_in_mrsq[0] & ~to_invalid_mrsq[0]);
assign ld_miss_in_mrsq_next[1] = (ld_miss_in_mrsq[1] & ~to_invalid_mrsq[1]);
assign ld_miss_in_mrsq_next[2] = (ld_miss_in_mrsq[2] & ~to_invalid_mrsq[2]);
assign ld_miss_in_mrsq_next[3] = (ld_miss_in_mrsq[3] & ~to_invalid_mrsq[3]);
assign ld_miss_in_mrsq_next[4] = (ld_miss_in_mrsq[4] & ~to_invalid_mrsq[4]);
assign ld_miss_in_mrsq_next[5] = (ld_miss_in_mrsq[5] & ~to_invalid_mrsq[5]);


wire [2:0] tail_add1_mrsq = tail_mrsq + 1'b1;
wire [2:0] tail_mrsq_index = (reset|commitbus_ex_i) ? 3'b000 : tail_mrsq;

always @(posedge clock)
begin
    if (reset | commitbus_ex_i)
    begin
        tail_mrsq <= 3'b000;
    end
    else if (valid_to_mrsq)
    begin
        if (tail_mrsq==3'b101)
            tail_mrsq <= 3'b000;
        else
            tail_mrsq <= tail_add1_mrsq;
    end
end //always

// when no valid result, out==3'b111
mem_tool_first_one_6_3_from_i u0_f1_6_3_from_i(.in(~mrsq_invalid_next),
                                               .i(tail_mrsq_index),
                                               .out(mrsq_head_point_next),
                                               .valid(mrsq_head_point_valid_next));

always @(posedge clock)
begin
    if (reset)
    begin
        mrsq_head_point <= 3'b000;
        mrsq_head_point_valid <= 1'b0;
    end
    else
    begin
        if (mrsq_head_point_valid_next)
            mrsq_head_point <= mrsq_head_point_next;
        else
            mrsq_head_point <= tail_mrsq;
        
        mrsq_head_point_valid <= mrsq_head_point_valid_next | valid_to_mrsq;
    end
end

assign mrsq_head_num = mrsq[mrsq_head_point];

mem_tool_first_one_6_3_from_i u1_f1_6_3_from_i(.in(ld_miss_in_mrsq_next),
                                               .i(tail_mrsq_index),
                                               .out(first_load_point_next),
                                               .valid(first_load_point_valid_next));

always @(posedge clock)
begin
    if (reset)
    begin
        first_load_point <= 3'b000;
        first_load_point_valid <= 1'b0;
    end
    else
    begin
        if (first_load_point_valid_next)
            first_load_point <= first_load_point_next;
        else 
            first_load_point <= tail_mrsq;
        
        first_load_point_valid <= first_load_point_valid_next | valid_to_mrsq&(tlb_op_load|tlb_op_prefetch);
    end
end

assign first_load_num = first_load_point_valid ? mrsq[first_load_point] : INVALID_MRSQ;

wire st_hit_ok = s_hit_st_q[head_st] & head_stq_hit_ok & (|head_ben_stq);
/*
wire st_miss_ok = s_miss_st_q[head_st] &  
                  ~valid_replace & ~valid_refill;
*/

assign head_to_missq = (mrsq_head_num[2]==1'b0) ||
                       ((mrsq_head_num[2:1]==2'b10) & (mrsq_head_num[0]==head_st) &
                        head_st_ok_stq & ~valid_replace_missq & ~valid_refill & ~forward_valid_r & 
                        (s_hit_st_q[head_st] | s_miss_st_q[head_st])) ||
                       ((mrsq_head_num[2:1]==2'b10)&&(state_st_q[mrsq_head_num[0]]==BLANK_STQ));

assign select_mrsq_num = head_to_missq ? mrsq_head_num : first_load_num;
assign select_mrsq_point = head_to_missq ? mrsq_head_point : 
                           first_load_point_valid ? first_load_point : 3'b111;

assign to_missq_queue_nu = select_mrsq_num;
assign mrsq_cancel_queue_nu = select_mrsq_num;

assign load_cancel = (select_mrsq_num[2]==1'b0)&&(!wait_buf_ldq[select_mrsq_num[1:0]]); 
assign store_cancel = (select_mrsq_num[2:1]==2'b10)&&(state_st_q[select_mrsq_num[0]]!=MISS_STQ);

assign to_missq_ready    = ~(&to_missq_queue_nu[2:1])    & ~(load_cancel|store_cancel); //only used for st_wb_valid

assign to_missq_req      = ~mrsq_brbus_cancel & to_missq_ready;
assign mrsq_cancel_valid = ~(&mrsq_cancel_queue_nu[2:1]) &  (load_cancel|store_cancel);


assign to_missq_op = (to_missq_queue_nu[2]==1'b0) ? op_ld_q[first_load_num[1:0]] :
                                                    op_st_q[head_st];

assign to_missq_lock = (to_missq_queue_nu[2]==1'b0) ? lock_ld_q[first_load_num[1:0]] :
                                                      lock_st_q[head_st];

assign to_missq_paddr = (to_missq_queue_nu[2]==1'b0) ? addr_ld_q[first_load_num[1:0]] :
                                                       addr_st_q[head_st];

assign to_missq_set_lock = (to_missq_queue_nu[2:1]==2'b10 && head_op_cache29_stq) ? 1'b1 : 1'b0;

assign to_missq_value   = head_value_stq;
assign to_missq_value_h = head_value_h_stq;
form_cacheline_wen 
    u0_form_store_wen_line_h(.in(head_ben_stq), 
                             .offset(head_addr_stq[4:2]), 
                             .out(store_wen_line_l));

form_cacheline_wen 
    u1_form_store_wen_line_h(.in(head_ben_h_stq), 
                             .offset({head_addr_stq[4:3], 1'b1}), 
                             .out(store_wen_line_h));

assign store_wen_line = store_wen_line_l | store_wen_line_h;

assign st_sw_datain_missq = {to_missq_value, to_missq_value, to_missq_value, to_missq_value,
                             to_missq_value, to_missq_value, to_missq_value, to_missq_value};

assign st_dw_datain_missq = {to_missq_value_h, to_missq_value, to_missq_value_h, to_missq_value,
                             to_missq_value_h, to_missq_value, to_missq_value_h, to_missq_value};

assign store_datain_missq = st_sw_datain_missq;


////// miss queue
wire [2:0] blank_missq = {(state_miss_q[2]==BLANK_MISSQ),
                          (state_miss_q[1]==BLANK_MISSQ),
                          (state_miss_q[0]==BLANK_MISSQ)};

wire [2:0] valid_missq = {(state_miss_q[2]==VALID_MISSQ),
                          (state_miss_q[1]==VALID_MISSQ),
                          (state_miss_q[0]==VALID_MISSQ)};

wire [2:0] mem_read_missq = {(state_miss_q[2]==MEM_READ_MISSQ),
                             (state_miss_q[1]==MEM_READ_MISSQ),
                             (state_miss_q[0]==MEM_READ_MISSQ)};

wire [2:0] all_back_missq = {(state_miss_q[2]==ALL_BACK_MISSQ),
                             (state_miss_q[1]==ALL_BACK_MISSQ),
                             (state_miss_q[0]==ALL_BACK_MISSQ)};

wire [2:0] replace_missq = {(state_miss_q[2]==REPLACE_MISSQ),
                            (state_miss_q[1]==REPLACE_MISSQ),
                            (state_miss_q[0]==REPLACE_MISSQ)};

wire [2:0]  refill_missq = {(state_miss_q[2]==REFILL_MISSQ),
                            (state_miss_q[1]==REFILL_MISSQ),
                            (state_miss_q[0]==REFILL_MISSQ)};

wire cache_wr_ok;
assign valid_replace_missq = |replace_missq & memres_wr_rdy & cache_wr_ok;

wire [1:0] wr_in_num_missq = ({2{wr_in_p_r==2'b00}} & (blank_missq[0] ? 2'b00 :
                                                       blank_missq[1] ? 2'b01 :
                                                       blank_missq[2] ? 2'b10 : 2'b11
                                                       ))       |
                             ({2{wr_in_p_r==2'b01}} & (blank_missq[1] ? 2'b01 :
                                                       blank_missq[2] ? 2'b10 :
                                                       blank_missq[0] ? 2'b00 : 2'b11
                                                       ))       |
                             ({2{wr_in_p_r==2'b10}} & (blank_missq[2] ? 2'b10 :
                                                       blank_missq[0] ? 2'b00 :
                                                       blank_missq[1] ? 2'b01 : 2'b11
                                                       ))       ;

wire [2:0] wr_in_num_v = (wr_in_num_missq==2'b00) ? 3'b001 : 
                         (wr_in_num_missq==2'b01) ? 3'b010 :
                         (wr_in_num_missq==2'b10) ? 3'b100 : 3'b000;

wire [31:0] ld_to_missq_paddr = addr_ld_q[first_load_num[1:0]];
wire [2:0] ld_to_missq_match;

assign ld_to_missq_match[0] = ld_to_missq_paddr[31:5]==addr_miss_q[0][31:5];
assign ld_to_missq_match[1] = ld_to_missq_paddr[31:5]==addr_miss_q[1][31:5];
assign ld_to_missq_match[2] = ld_to_missq_paddr[31:5]==addr_miss_q[2][31:5];

wire [31:0] st_to_missq_paddr = addr_st_q[head_st];
wire [2:0] st_to_missq_match;

assign st_to_missq_match[0] = st_to_missq_paddr[31:5]==addr_miss_q[0][31:5];
assign st_to_missq_match[1] = st_to_missq_paddr[31:5]==addr_miss_q[1][31:5];
assign st_to_missq_match[2] = st_to_missq_paddr[31:5]==addr_miss_q[2][31:5];

wire [2:0] st_miss_in_missq;
assign st_miss_in_missq[0] = (st_to_missq_match[0]) && (!blank_missq[0]);
assign st_miss_in_missq[1] = (st_to_missq_match[1]) && (!blank_missq[1]);
assign st_miss_in_missq[2] = (st_to_missq_match[2]) && (!blank_missq[2]);

assign missq_allowin_st = (|st_miss_in_missq) | (~&wr_in_num_missq);

wire [2:0] to_missq_match = (to_missq_queue_nu[2]==1'b0) ? ld_to_missq_match : st_to_missq_match;

wire [2:0] miss_in_missq;
assign miss_in_missq[0] = (to_missq_match[0]) && (!blank_missq[0]);
assign miss_in_missq[1] = (to_missq_match[1]) && (!blank_missq[1]);
assign miss_in_missq[2] = (to_missq_match[2]) && (!blank_missq[2]);

assign missq_allowin = (|miss_in_missq) | (~&wr_in_num_missq);

assign missq_full_o = to_missq_req & ~missq_allowin;

// which iterm of missq should be written into by this store miss
wire [2:0] st_in_missq_v = (|miss_in_missq) ? miss_in_missq : wr_in_num_v;
wire [2:0] miss_wr_missq = (to_missq_req && to_missq_queue_nu[2:1]==2'b10) ? st_in_missq_v : 3'b000; 

wire [2:0] memres_in_missq_tmp;
wire [2:0] memres_in_missq;
assign memres_in_missq_tmp[0] = (memres_id==3'b000) && (mem_read_missq[0]);
assign memres_in_missq_tmp[1] = (memres_id==3'b001) && (mem_read_missq[1]);
assign memres_in_missq_tmp[2] = (memres_id==3'b010) && (mem_read_missq[2]);
assign memres_in_missq = {3{memres_valid&memres_is_block}}&memres_in_missq_tmp;

// which iterm of missq should be written by mem result
wire [2:0] mem_wr_missq = memres_in_missq;

wire [31:0] memres_wen_line;
wire [31:0] st_wen_line_missq[2:0];
wire [31:0] mem_wen_line_missq[2:0];
wire [31:0] wen_line_missq[2:0];
wire [2:0]  memres_offset;
wire [255:0] memres_datain_missq;

assign memres_offset = memres_addr_mid[4:2];

form_cacheline_wen
    u2_form_memres_wen_line(.in(4'b1111), 
                            .offset(memres_offset), 
                            .out(memres_wen_line));

assign memres_datain_missq = {memres_data, memres_data, memres_data, memres_data,
                              memres_data, memres_data, memres_data, memres_data};

assign st_wen_line_missq[0] = {32{miss_wr_missq[0]}} & store_wen_line;
assign st_wen_line_missq[1] = {32{miss_wr_missq[1]}} & store_wen_line;
assign st_wen_line_missq[2] = {32{miss_wr_missq[2]}} & store_wen_line;

assign mem_wen_line_missq[0] = {32{mem_wr_missq[0]}} & memres_wen_line & ~ben_miss_q[0]; 
assign mem_wen_line_missq[1] = {32{mem_wr_missq[1]}} & memres_wen_line & ~ben_miss_q[1]; 
assign mem_wen_line_missq[2] = {32{mem_wr_missq[2]}} & memres_wen_line & ~ben_miss_q[2]; 

assign wen_line_missq[0] = st_wen_line_missq[0] | mem_wen_line_missq[0];
assign wen_line_missq[1] = st_wen_line_missq[1] | mem_wen_line_missq[1]; 
assign wen_line_missq[2] = st_wen_line_missq[2] | mem_wen_line_missq[2];


wire [ 2:0] to_valid_missq;
wire [ 2:0] to_mem_read_missq;
wire [ 2:0] to_all_back_missq;
wire [ 2:0] to_replace_missq;
wire [ 2:0] to_refill_missq;
wire [ 2:0] to_blank_missq;

wire   wr_new_in_missq;
assign wr_new_in_missq = (to_missq_req&(~|miss_in_missq)&(|wr_in_num_v)); 
assign to_valid_missq = {3{wr_new_in_missq}} & wr_in_num_v;


wire [2:0] st_fill_index_v = (st_fill_index_r==2'b00) ? 3'b001    :
                             (st_fill_index_r==2'b01) ? 3'b010    :
                             (st_fill_index_r==2'b10) ? 3'b100    : 3'b000;

wire [2:0] rdy_mem_rd_missq = valid_missq & (~st_fill_index_v | ld_in_miss_q);
wire [1:0] iterm_rd_mem    = ({2{(wr_in_p_r==2'b00)}} & (rdy_mem_rd_missq[1] ? 2'b01 :       
                                                         rdy_mem_rd_missq[2] ? 2'b10 :       
                                                         rdy_mem_rd_missq[0] ? 2'b00 : 2'b11 
                                                         )) |
                             ({2{(wr_in_p_r==2'b01)}} & (rdy_mem_rd_missq[2] ? 2'b10 :
                                                         rdy_mem_rd_missq[0] ? 2'b00 :
                                                         rdy_mem_rd_missq[1] ? 2'b01 : 2'b11
                                                         )) |
                             ({2{(wr_in_p_r==2'b10)}} & (rdy_mem_rd_missq[0] ? 2'b00 :
                                                         rdy_mem_rd_missq[1] ? 2'b01 :
                                                         rdy_mem_rd_missq[2] ? 2'b10 : 2'b11
                                                         )) ;


wire [2:0] iterm_rd_mem_v = (iterm_rd_mem==2'b00) ? 3'b001   :
                            (iterm_rd_mem==2'b01) ? 3'b010   :
                            (iterm_rd_mem==2'b10) ? 3'b100   : 3'b000;
wire [2:0] all_rdy_at_valid;
assign all_rdy_at_valid[0] = &(ben_miss_q[0]) & valid_missq[0];
assign all_rdy_at_valid[1] = &(ben_miss_q[1]) & valid_missq[1];
assign all_rdy_at_valid[2] = &(ben_miss_q[2]) & valid_missq[2];


assign to_mem_read_missq = {3{memres_rd_rdy}} & 
                           (~all_rdy_at_valid|ld_in_miss_q) & iterm_rd_mem_v;

assign to_all_back_missq[0] = all_rdy_at_valid[0] & ~ld_in_miss_q[0] | 
                              back_count_miss_q[0][3] & mem_read_missq[0];
assign to_all_back_missq[1] = all_rdy_at_valid[1] & ~ld_in_miss_q[1] | 
                              back_count_miss_q[1][3] & mem_read_missq[1];
assign to_all_back_missq[2] = all_rdy_at_valid[2] & ~ld_in_miss_q[2] | 
                              back_count_miss_q[2][3] & mem_read_missq[2];

wire [2:0] to_replace_v = all_back_missq[0] ? 3'b001  :
                          all_back_missq[1] ? 3'b010  :
                          all_back_missq[2] ? 3'b100  : 3'b000;
assign to_replace_missq = to_replace_v & {3{~|replace_missq}};
    //(~valid_replace) : only one line can enter replace state at a time

assign to_refill_missq = {3{valid_replace_missq}}&replace_missq;

assign to_blank_missq = refill_missq;

wire [2:0] in_en_state_missq;
wire [2:0] in_en_addr_missq;
wire [2:0] in_en_lock_missq;
wire [2:0] in_en_set_lock_missq;
wire [2:0] in_en_dirty_missq;
wire [2:0] in_en_ld_in_missq;
wire [2:0] in_en_back_count_missq;
wire [31:0] in_en_ben_missq[2:0];

assign in_en_state_missq = (reset) ? 3'b111 :
                                        (to_valid_missq    | to_mem_read_missq |
                                         to_all_back_missq | to_replace_missq  |
                                         to_refill_missq   | to_blank_missq    );

assign in_en_addr_missq = to_valid_missq;

assign in_en_lock_missq = (reset) ? 3'b111  : to_valid_missq;

assign in_en_set_lock_missq = (reset) ? 3'b111 : to_valid_missq;

assign in_en_back_count_missq = (reset) ? 3'b111 : 
                                         (mem_wr_missq | to_mem_read_missq);

assign in_en_ben_missq[0] = (reset|to_blank_missq[0]) ? 32'hffff_ffff : wen_line_missq[0];
assign in_en_ben_missq[1] = (reset|to_blank_missq[1]) ? 32'hffff_ffff : wen_line_missq[1];
assign in_en_ben_missq[2] = (reset|to_blank_missq[2]) ? 32'hffff_ffff : wen_line_missq[2];

wire [2:0] set_dirty_missq;
wire head_wr_stq = (|head_ben_stq) | (|head_ben_h_stq);
assign set_dirty_missq[0] = miss_wr_missq[0] & head_wr_stq;
assign set_dirty_missq[1] = miss_wr_missq[1] & head_wr_stq;
assign set_dirty_missq[2] = miss_wr_missq[2] & head_wr_stq;
assign in_en_dirty_missq = (reset) ? 3'b111 : (to_blank_missq | set_dirty_missq);

wire [2:0] ld_fall_in_missq;
assign ld_fall_in_missq  = {3{to_missq_req & ~to_missq_queue_nu[2]}} & 
                           (miss_in_missq | to_valid_missq);
assign in_en_ld_in_missq = (reset) ? 3'b111 : (to_blank_missq | ld_fall_in_missq);

integer missq_i;
integer missq_j;
integer missq_k;

always @(posedge clock)
begin //miss_queue1
    for (missq_i=0; missq_i<3; missq_i=missq_i+1)
    begin //miss_queue1
        if (in_en_state_missq[missq_i])
            state_miss_q[missq_i] <= (reset|to_blank_missq[missq_i]) ?  BLANK_MISSQ  :
                                     (to_valid_missq[missq_i]) ? VALID_MISSQ  :
                                     (to_mem_read_missq[missq_i]) ? MEM_READ_MISSQ :
                                     (to_all_back_missq[missq_i]) ? ALL_BACK_MISSQ :
                                     (to_replace_missq[missq_i])  ? REPLACE_MISSQ  :
                                                                    REFILL_MISSQ   ; //to_refill_missq[missq_i]
        
        if (in_en_addr_missq[missq_i])
            addr_miss_q[missq_i] <= to_missq_paddr;

        if (in_en_lock_missq[missq_i])
            lock_miss_q[missq_i] <= (reset) ? 4'b0000 : to_missq_lock;

        if (in_en_set_lock_missq[missq_i])
            set_lock_miss_q[missq_i] <= (reset) ? 1'b0 : to_missq_set_lock;
        
        if (in_en_dirty_missq[missq_i])
            dirty_miss_q[missq_i] <= (reset|to_blank_missq[missq_i]) ? 1'b0 : 1'b1; // set_dirty_missq

        if (in_en_ld_in_missq[missq_i])
            ld_in_miss_q[missq_i] <= (reset|to_blank_missq[missq_i]) ? 1'b0 : 1'b1; // ld_fall_in_missq

        if (in_en_back_count_missq[missq_i])
            back_count_miss_q[missq_i] <= (reset|to_mem_read_missq[missq_i]) ? 4'b0000 :
                                                 back_count_miss_q[missq_i]+1'b1; //mem_wr_missq[missq_i]
        
        for (missq_j=0; missq_j<32; missq_j=missq_j+1)
        begin 
            if (in_en_ben_missq[missq_i][missq_j])
                ben_miss_q[missq_i][missq_j] <= (reset|to_blank_missq[missq_i]) ? 1'b0 : 1'b1;
        end
    end // for miss_queue1
end // always miss_queue1

always @(posedge clock)
begin //miss_queue_2
    for (missq_k=0; missq_k<3; missq_k=missq_k+1)
    begin
        if (wen_line_missq[missq_k][0])
            data_miss_q[missq_k][7:0] <= st_wen_line_missq[missq_k][0] ? 
                                         store_datain_missq[7:0] : memres_datain_missq[7:0];

        if (wen_line_missq[missq_k][1])
            data_miss_q[missq_k][15:8] <= st_wen_line_missq[missq_k][1] ? 
                                         store_datain_missq[15:8] : memres_datain_missq[15:8];

        if (wen_line_missq[missq_k][2])
            data_miss_q[missq_k][23:16] <= st_wen_line_missq[missq_k][2] ? 
                                         store_datain_missq[23:16] : memres_datain_missq[23:16];

        if (wen_line_missq[missq_k][3])
            data_miss_q[missq_k][31:24] <= st_wen_line_missq[missq_k][3] ? 
                                         store_datain_missq[31:24] : memres_datain_missq[31:24];

        if (wen_line_missq[missq_k][4])
            data_miss_q[missq_k][39:32] <= st_wen_line_missq[missq_k][4] ? 
                                         store_datain_missq[39:32] : memres_datain_missq[39:32];

        if (wen_line_missq[missq_k][5])
            data_miss_q[missq_k][47:40] <= st_wen_line_missq[missq_k][5] ? 
                                         store_datain_missq[47:40] : memres_datain_missq[47:40];

        if (wen_line_missq[missq_k][6])
            data_miss_q[missq_k][55:48] <= st_wen_line_missq[missq_k][6] ? 
                                         store_datain_missq[55:48] : memres_datain_missq[55:48];

        if (wen_line_missq[missq_k][7])
            data_miss_q[missq_k][63:56] <= st_wen_line_missq[missq_k][7] ? 
                                         store_datain_missq[63:56] : memres_datain_missq[63:56];

        if (wen_line_missq[missq_k][8])
            data_miss_q[missq_k][71:64] <= st_wen_line_missq[missq_k][8] ? 
                                         store_datain_missq[71:64] : memres_datain_missq[71:64];

        if (wen_line_missq[missq_k][9])
            data_miss_q[missq_k][79:72] <= st_wen_line_missq[missq_k][9] ? 
                                         store_datain_missq[79:72] : memres_datain_missq[79:72];

        if (wen_line_missq[missq_k][10])
            data_miss_q[missq_k][87:80] <= st_wen_line_missq[missq_k][10] ? 
                                         store_datain_missq[87:80] : memres_datain_missq[87:80];

        if (wen_line_missq[missq_k][11])
            data_miss_q[missq_k][95:88] <= st_wen_line_missq[missq_k][11] ? 
                                         store_datain_missq[95:88] : memres_datain_missq[95:88];

        if (wen_line_missq[missq_k][12])
            data_miss_q[missq_k][103:96] <= st_wen_line_missq[missq_k][12] ? 
                                         store_datain_missq[103:96] : memres_datain_missq[103:96];

        if (wen_line_missq[missq_k][13])
            data_miss_q[missq_k][111:104] <= st_wen_line_missq[missq_k][13] ? 
                                         store_datain_missq[111:104] : memres_datain_missq[111:104];

        if (wen_line_missq[missq_k][14])
            data_miss_q[missq_k][119:112] <= st_wen_line_missq[missq_k][14] ? 
                                         store_datain_missq[119:112] : memres_datain_missq[119:112];

        if (wen_line_missq[missq_k][15])
            data_miss_q[missq_k][127:120] <= st_wen_line_missq[missq_k][15] ? 
                                         store_datain_missq[127:120] : memres_datain_missq[127:120];

        if (wen_line_missq[missq_k][16])
            data_miss_q[missq_k][135:128] <= st_wen_line_missq[missq_k][16] ? 
                                         store_datain_missq[135:128] : memres_datain_missq[135:128];

        if (wen_line_missq[missq_k][17])
            data_miss_q[missq_k][143:136] <= st_wen_line_missq[missq_k][17] ? 
                                         store_datain_missq[143:136] : memres_datain_missq[143:136];

        if (wen_line_missq[missq_k][18])
            data_miss_q[missq_k][151:144] <= st_wen_line_missq[missq_k][18] ? 
                                         store_datain_missq[151:144] : memres_datain_missq[151:144];

        if (wen_line_missq[missq_k][19])
            data_miss_q[missq_k][159:152] <= st_wen_line_missq[missq_k][19] ? 
                                         store_datain_missq[159:152] : memres_datain_missq[159:152];

        if (wen_line_missq[missq_k][20])
            data_miss_q[missq_k][167:160] <= st_wen_line_missq[missq_k][20] ? 
                                         store_datain_missq[167:160] : memres_datain_missq[167:160];

        if (wen_line_missq[missq_k][21])
            data_miss_q[missq_k][175:168] <= st_wen_line_missq[missq_k][21] ? 
                                         store_datain_missq[175:168] : memres_datain_missq[175:168];

        if (wen_line_missq[missq_k][22])
            data_miss_q[missq_k][183:176] <= st_wen_line_missq[missq_k][22] ? 
                                         store_datain_missq[183:176] : memres_datain_missq[183:176];

        if (wen_line_missq[missq_k][23])
            data_miss_q[missq_k][191:184] <= st_wen_line_missq[missq_k][23] ? 
                                         store_datain_missq[191:184] : memres_datain_missq[191:184];

        if (wen_line_missq[missq_k][24])
            data_miss_q[missq_k][199:192] <= st_wen_line_missq[missq_k][24] ? 
                                         store_datain_missq[199:192] : memres_datain_missq[199:192];

        if (wen_line_missq[missq_k][25])
            data_miss_q[missq_k][207:200] <= st_wen_line_missq[missq_k][25] ? 
                                         store_datain_missq[207:200] : memres_datain_missq[207:200];

        if (wen_line_missq[missq_k][26])
            data_miss_q[missq_k][215:208] <= st_wen_line_missq[missq_k][26] ? 
                                         store_datain_missq[215:208] : memres_datain_missq[215:208];

        if (wen_line_missq[missq_k][27])
            data_miss_q[missq_k][223:216] <= st_wen_line_missq[missq_k][27] ? 
                                         store_datain_missq[223:216] : memres_datain_missq[223:216];

        if (wen_line_missq[missq_k][28])
            data_miss_q[missq_k][231:224] <= st_wen_line_missq[missq_k][28] ? 
                                         store_datain_missq[231:224] : memres_datain_missq[231:224];

        if (wen_line_missq[missq_k][29])
            data_miss_q[missq_k][239:232] <= st_wen_line_missq[missq_k][29] ? 
                                         store_datain_missq[239:232] : memres_datain_missq[239:232];

        if (wen_line_missq[missq_k][30])
            data_miss_q[missq_k][247:240] <= st_wen_line_missq[missq_k][30] ? 
                                         store_datain_missq[247:240] : memres_datain_missq[247:240];

        if (wen_line_missq[missq_k][31])
            data_miss_q[missq_k][255:248] <= st_wen_line_missq[missq_k][31] ? 
                                         store_datain_missq[255:248] : memres_datain_missq[255:248];

    end
end //always miss_queue2 


wire [1:0] iterm_rd_mem_add1 = iterm_rd_mem + 1'b1;
wire [1:0] wr_in_num_missq_add1 = wr_in_num_missq +1'b1;

always @(posedge clock)
begin
    if (reset)
    begin
        wr_in_p_r <= 2'b00;
    end
    else
    begin
        if (|to_valid_missq)
            wr_in_p_r <= (wr_in_num_missq_add1==2'b11) ? 2'b00 : wr_in_num_missq_add1;
    end
end

always @(posedge clock)
begin
    if (reset)
        st_fill_index_r <= 2'b11;
    else if (wr_new_in_missq && (to_missq_queue_nu[2:1]==2'b10) && ~head_op_cache29_stq)
        st_fill_index_r <= wr_in_num_missq;
    else if (inst_valid_at_addr_i&(inst_cache_at_addr_i|inst_sync_at_addr_i)&(~|rdy_mem_rd_missq) |
             to_missq_req & ~to_missq_queue_nu[2] & |(miss_in_missq&st_fill_index_v)
            )
        st_fill_index_r <= 2'b11;
end

assign missq_all_blank = &blank_missq;

////// uncache acc buffer
wire acc_write_mem_en  = (uncache_st_no_acc_req_tmp1 & ~sc_not_write) | ld_to_mem_req_tmp1 | 
                         uncache_st_acc_req & (head_addr_stq[31:5]!=addr_acc) |
                         inst_valid_at_addr_i&inst_sync_at_addr_i ;

wire write_port_no_conflict = ~(valid_replace | replace_write_mem);

wire acc_mem_rdy = memres_wr_rdy & memres_unc_acc_rdy & write_port_no_conflict;

wire [3:0] ben_current_word = {4{acc_word_cnt_r==3'b000}} & ben_acc[ 3: 0] |
                              {4{acc_word_cnt_r==3'b001}} & ben_acc[ 7: 4] |
                              {4{acc_word_cnt_r==3'b010}} & ben_acc[11: 8] |
                              {4{acc_word_cnt_r==3'b011}} & ben_acc[15:12] |
                              {4{acc_word_cnt_r==3'b100}} & ben_acc[19:16] |
                              {4{acc_word_cnt_r==3'b101}} & ben_acc[23:20] |
                              {4{acc_word_cnt_r==3'b110}} & ben_acc[27:24] |
                              {4{acc_word_cnt_r==3'b111}} & ben_acc[31:28] ;
wire current_word_all_one  = &ben_current_word;
wire current_word_all_zero = ~|ben_current_word;

wire acc_line_mem_can_out = (state_acc==LINE_ACC) & memres_wr_rdy & memres_unc_acc_rdy;
wire acc_line_mem_req  = acc_line_mem_can_out & write_port_no_conflict;

wire acc_split_mem_can_out = (state_acc==SPLIT_ACC) & memres_wr_rdy & memres_unc_acc_rdy & ~current_word_all_zero;

wire acc_split_mem_req = acc_split_mem_can_out & write_port_no_conflict; 

wire blank_to_idle_acc = (state_acc==BLANK_ACC) & uncache_st_acc_req;
wire idle_to_line_acc  = (state_acc==IDLE_ACC) & ( &ben_acc);
wire idle_to_split_acc = (state_acc==IDLE_ACC) & acc_write_mem_en & (~&ben_acc);
wire line_to_blank_acc = acc_line_mem_req;
wire split_to_blank_acc= (state_acc==SPLIT_ACC) & (acc_word_cnt_r==3'b111) & 
                         (current_word_all_zero | acc_mem_rdy);

assign st_into_acc_allow = state_acc==BLANK_ACC | (state_acc==IDLE_ACC & head_addr_stq[31:5]==addr_acc);
wire write_in_en_acc = uncache_st_acc_req & st_into_acc_allow & ~sc_not_write;

wire [31:0] ben_into_acc = {32{write_in_en_acc}} & store_wen_line;
wire [255:0] data_into_acc = store_datain_missq;

wire [31:0] ben_outof_acc_tmp;
wire [31:0] ben_outof_acc; 

wire [3:0] word_ben_outof_acc = ben_current_word;

form_cacheline_wen 
    u0_form_store_wen_acc(.in(word_ben_outof_acc),
                          .offset(acc_word_cnt_r),
                          .out(ben_outof_acc_tmp));

assign ben_outof_acc = {32{acc_split_mem_req}} & ben_outof_acc_tmp;

wire in_en_state_acc = (reset) ? 1'b1 : (blank_to_idle_acc | idle_to_line_acc  | 
                                         idle_to_split_acc | line_to_blank_acc |
                                         split_to_blank_acc);
wire in_en_addr_acc  = blank_to_idle_acc;
wire [31:0] in_en_ben_acc   = (reset|line_to_blank_acc) ? 32'hffffffff : (ben_into_acc | ben_outof_acc);

always @(posedge clock)
begin
    if (reset)
    begin
        acc_word_cnt_r <= 3'b000;
    end
    else if (state_acc==SPLIT_ACC)
    begin
        if (current_word_all_zero | acc_mem_rdy)
        begin
            acc_word_cnt_r <= acc_word_cnt_r + 1'b1;
        end
    end
end

always @(posedge clock)
begin
    if (in_en_state_acc)
        state_acc <= (reset | line_to_blank_acc | split_to_blank_acc) ? BLANK_ACC :
                                                  (blank_to_idle_acc) ? IDLE_ACC  :
                                                   (idle_to_line_acc) ? LINE_ACC  : SPLIT_ACC; //idle_to_split_acc
	
    if (in_en_addr_acc)
        addr_acc <= head_addr_stq[31:5];
    
    if (ben_into_acc[0])
		data_acc[7:0] <= data_into_acc[7:0];
	if (ben_into_acc[1])
		data_acc[15:8] <= data_into_acc[15:8];
	if (ben_into_acc[2])
		data_acc[23:16] <= data_into_acc[23:16];
	if (ben_into_acc[3])
		data_acc[31:24] <= data_into_acc[31:24];
	if (ben_into_acc[4])
		data_acc[39:32] <= data_into_acc[39:32];
	if (ben_into_acc[5])
		data_acc[47:40] <= data_into_acc[47:40];
	if (ben_into_acc[6])
		data_acc[55:48] <= data_into_acc[55:48];
	if (ben_into_acc[7])
		data_acc[63:56] <= data_into_acc[63:56];
	if (ben_into_acc[8])
		data_acc[71:64] <= data_into_acc[71:64];
	if (ben_into_acc[9])
		data_acc[79:72] <= data_into_acc[79:72];
	if (ben_into_acc[10])
		data_acc[87:80] <= data_into_acc[87:80];
	if (ben_into_acc[11])
		data_acc[95:88] <= data_into_acc[95:88];
	if (ben_into_acc[12])
		data_acc[103:96] <= data_into_acc[103:96];
	if (ben_into_acc[13])
		data_acc[111:104] <= data_into_acc[111:104];
	if (ben_into_acc[14])
		data_acc[119:112] <= data_into_acc[119:112];
	if (ben_into_acc[15])
		data_acc[127:120] <= data_into_acc[127:120];
	if (ben_into_acc[16])
		data_acc[135:128] <= data_into_acc[135:128];
	if (ben_into_acc[17])
		data_acc[143:136] <= data_into_acc[143:136];
	if (ben_into_acc[18])
		data_acc[151:144] <= data_into_acc[151:144];
	if (ben_into_acc[19])
		data_acc[159:152] <= data_into_acc[159:152];
	if (ben_into_acc[20])
		data_acc[167:160] <= data_into_acc[167:160];
	if (ben_into_acc[21])
		data_acc[175:168] <= data_into_acc[175:168];
	if (ben_into_acc[22])
		data_acc[183:176] <= data_into_acc[183:176];
	if (ben_into_acc[23])
		data_acc[191:184] <= data_into_acc[191:184];
	if (ben_into_acc[24])
		data_acc[199:192] <= data_into_acc[199:192];
	if (ben_into_acc[25])
		data_acc[207:200] <= data_into_acc[207:200];
	if (ben_into_acc[26])
		data_acc[215:208] <= data_into_acc[215:208];
	if (ben_into_acc[27])
		data_acc[223:216] <= data_into_acc[223:216];
	if (ben_into_acc[28])
		data_acc[231:224] <= data_into_acc[231:224];
	if (ben_into_acc[29])
		data_acc[239:232] <= data_into_acc[239:232];
	if (ben_into_acc[30])
		data_acc[247:240] <= data_into_acc[247:240];
	if (ben_into_acc[31])
		data_acc[255:248] <= data_into_acc[255:248];

end

integer acc_i;
always @(posedge clock)
begin
    for (acc_i=0; acc_i<32; acc_i=acc_i+1)
    begin
        if (in_en_ben_acc[acc_i])
            ben_acc[acc_i] <= (ben_into_acc[acc_i]) ? 1'b1 : 1'b0;
    end
end

wire acc_wr_mem = acc_line_mem_req | acc_split_mem_req; 
wire [3:0] acc_mem_width = (state_acc==LINE_ACC) ? 4'b0001 : 4'b0000;
wire [31:0] acc_mem_addr = (state_acc==LINE_ACC) ? {addr_acc, 5'b00000} : {addr_acc, acc_word_cnt_r, 2'b00};
wire [31:0]  select_word = {32{acc_word_cnt_r==3'b000/* || state_acc==LINE_ACC*/}} & data_acc[ 31:  0] |
                           {32{acc_word_cnt_r==3'b001}} & data_acc[ 63: 32] |
                           {32{acc_word_cnt_r==3'b010}} & data_acc[ 95: 64] |
                           {32{acc_word_cnt_r==3'b011}} & data_acc[127: 96] |
                           {32{acc_word_cnt_r==3'b100}} & data_acc[159:128] |
                           {32{acc_word_cnt_r==3'b101}} & data_acc[191:160] |
                           {32{acc_word_cnt_r==3'b110}} & data_acc[223:192] |
                           {32{acc_word_cnt_r==3'b111}} & data_acc[255:224] ;
                      //acc_word_cnt_r always equal to 3'b000 when state_acc==LINE_ACC
wire [255:0] acc_mem_data = {data_acc[255:32], select_word};
assign memwaddr_acc_wen = ben_current_word;


////// cp0_forward_bus_o bus
wire [5:0]  forward_qnum_inqueue_v;
wire [2:0]  forward_queue_nu;
wire [5:0]  forward_queue_nu_v;

wire [3:0]  ld_miss_forward_v;
wire [3:0]  ld_ready_forward_v;
wire [3:0]  ld_hit_forward_v;

mem_tool_first_one_4_4_from_i u2_f1_4_4_from_i(.in(~wb_ldq_v&ready_ldq), 
                                               .i(wb_p_ldq), 
                                               .out(ld_ready_forward_v));

assign ld_hit_forward_v = {4{tlb_forward_valid}} & in_ldq_index_v;

wire uc_ld_rdy_forward = head_ready_st_q & ~st_wb_valid_uc_ld;

assign forward_qnum_inqueue_v = /*(|all_rdy_ldq) ? {2'b00, ld_miss_forward_v}  : */
                                (ld_all_back_stq | uc_ld_rdy_forward) ? {head_st_v, 4'b0000}     :
                                (|(~wb_ldq_v&ready_ldq))? {2'b00, ld_ready_forward_v} : 6'b000000;

wire [2:0] forward_fpqid_ready_ldq = 3'b0;

wire [3:0] forward_qid_ready_ldq = {4{ld_ready_forward_v[0]}}&qid_ld_q[0]  |
                                   {4{ld_ready_forward_v[1]}}&qid_ld_q[1]  |
                                   {4{ld_ready_forward_v[2]}}&qid_ld_q[2]  |
                                   {4{ld_ready_forward_v[3]}}&qid_ld_q[3]  ;

wire [7:0] forward_op_ready_ldq = {8{ld_ready_forward_v[0]}}&op_ld_q[0]  |
                                  {8{ld_ready_forward_v[1]}}&op_ld_q[1]  |
                                  {8{ld_ready_forward_v[2]}}&op_ld_q[2]  |
                                  {8{ld_ready_forward_v[3]}}&op_ld_q[3]  ;

wire [7:0] forward_op_stq = head_op_stq;
wire [3:0] forward_qid_stq = head_qid_stq;
wire [2:0] forward_fpqid_stq = head_fpqid_stq;
wire fpq_ready_ldq;
wire fpq_stq;
assign fpq_ready_ldq = 1'b0;
assign fpq_stq =1'b0;

wire forward_inqueue_valid = ld_all_back_stq | uc_ld_rdy_forward | (|(~wb_ldq_v&ready_ldq));

assign forward_qid = (ld_all_back_stq | uc_ld_rdy_forward) ? forward_qid_stq   :
                     (|(~wb_ldq_v&ready_ldq)) ? forward_qid_ready_ldq : tlb_forward_qid;
assign forward_fpqid = (ld_all_back_stq | uc_ld_rdy_forward) ? forward_fpqid_stq   :
                       (|(~wb_ldq_v&ready_ldq)) ? forward_fpqid_ready_ldq : tlb_forward_fpqid;

assign forward_fpq = (ld_all_back_stq | uc_ld_rdy_forward) ? fpq_stq   :
                     (|(~wb_ldq_v&ready_ldq)) ? fpq_ready_ldq : fpq_forward;

wire tlb_op_can_in_ldq = tlb_valid_in & ~load_queue_full_o & ~miss_req_queue_full_o;

wire tlb_other_forward = (tlb_op==`OP_MFC0) & tlb_op_can_in_ldq;
wire tlb_load_can_in_ldq = tlb_cached & tlb_op_load & tlb_op_can_in_ldq;

wire load_hit_cache = (dcache_hit_i & tlb_load_can_in_ldq);

assign tlb_forward_valid = load_hit_cache | tlb_other_forward;
assign forward_valid = load_hit_cache | (forward_inqueue_valid | tlb_other_forward); 

assign forward_queue_nu_v = forward_inqueue_valid ? forward_qnum_inqueue_v : {2'b00, in_ldq_index_v};

mem_tool_encode_6_3 u0_enc_6_3(.in(forward_queue_nu_v), .out(forward_queue_nu));

wire [5:0] brbus_cancel_queue = {brbus_cancel_stq, brbus_cancel_ldq};
wire forward_valid_to_reg = (tlb_forward_valid & ~tlb_brbus_cancel) | 
                            (|(forward_qnum_inqueue_v & ~brbus_cancel_queue));

always @(posedge clock)
begin
    if (reset|commitbus_ex_i)
    begin
        forward_valid_r <= 1'b0;
        forward_queue_nu_r <= 3'b111;
        forward_qid_r <= 4'b0000;
    end
    else
    begin
        forward_valid_r <= forward_valid_to_reg;

        if (forward_valid)
        begin
            forward_queue_nu_r <= forward_queue_nu;
            forward_qid_r <= forward_qid;
        end
    end
end

////// cp0_cancel_bus_o bus
assign cancel_valid = 1'b0;
assign cancel_qid   = 4'b0; 

////// store_req_o bus
assign valid_st_req= st_hit_ok; //~(head_op_stq==`OP_SC&~cr_llbit_value_i);
assign op_st_req = head_op_stq;
assign set_st_req = head_set_stq;
assign laddr_st_req = head_addr_stq[11:0];
assign wen_st_req = store_wen_line;

assign value_st_req = store_datain_missq;


////// cache_req_o bus
assign valid_cache_req = s_hit_st_q[head_st] & head_st_ok_stq &
                        (( head_op_stq==`OP_CACHE9  |
                           head_op_stq==`OP_CACHE17 | 
                           head_op_cache29_stq             ) | 
                         ((head_op_stq==`OP_CACHE1  | 
                           head_op_stq==`OP_CACHE21 | 
                           head_op_stq==`OP_SYNCI) & cache1_21_r)
                        );
assign set_cache_req = head_set_stq;

assign laddr_cache_req = head_addr_stq[11:0];

assign tagout_cache_req[21] = (head_op_cache29_stq) ? 1'b1 : (head_op_stq==`OP_CACHE9) ? head_value_stq[23] : 1'b0;
assign tagout_cache_req[20:0] = (head_op_cache29_stq|head_op_stq==`OP_CACHE9) ? head_value_stq[22:2] : 21'b0;

////// replace_req_o bus
wire [3:0] lock_for_replace;
wire [3:0] available_set;
wire [2:0] available_set_num;
wire       has_0_set;
wire       has_1_set;
wire       has_2_set;
wire       has_3_set;
wire       has_4_set;

wire [1:0] first_1_set_num;
wire [1:0] second_1_set_num;
wire [1:0] third_1_set_num;
wire [1:0] fourth_1_set_num;

reg  [4:0] repcnt_r;
wire [1:0] random_num_radix4;
wire [1:0] random_num_radix3;
wire [3:0] random_mask4;
wire [2:0] random_mask3;
wire [1:0] random_mask2;


wire [1:0] select_set;

always @(posedge clock)
begin
    repcnt_r<= reset ? 5'b11111 : {repcnt_r[3]^repcnt_r[0],repcnt_r[4:1]};
end

assign lock_for_replace = {4{replace_missq[0]}}&lock_miss_q[0] |
                          {4{replace_missq[1]}}&lock_miss_q[1] |
                          {4{replace_missq[2]}}&lock_miss_q[2] ;

assign rand_num_o = repcnt_r[2:0];

assign available_set = ~lock_for_replace;
assign available_set_num = (available_set[0] + available_set[1]) + (available_set[2] + available_set[3]);
assign has_0_set = ~|available_set;
assign has_4_set =  &available_set;
assign has_1_set = available_set_num == 3'b001;
assign has_2_set = available_set_num == 3'b010;
assign has_3_set = available_set_num == 3'b011;

assign first_1_set_num = (available_set[0]) ? 2'b00 :
                         (available_set[1]) ? 2'b01 :
                         (available_set[2]) ? 2'b10 : 2'b11;
assign second_1_set_num = (available_set[0]) ? ((available_set[1]) ? 2'b01 :
                                                (available_set[2]) ? 2'b10 : 2'b11
                                               ) : 
                          (available_set[1]) ? ((available_set[2]) ? 2'b10 : 2'b11
                                               ) : 2'b11;

assign third_1_set_num = (available_set[2:0]==3'b111) ? 2'b10 : 2'b11;
assign fourth_1_set_num = 2'b11;

assign random_num_radix4 = repcnt_r[1:0];
assign random_mask3[0] = (repcnt_r==5'd1 ) || (repcnt_r==5'd4 ) || (repcnt_r==5'd7 ) ||
                         (repcnt_r==5'd10) || (repcnt_r==5'd13) || (repcnt_r==5'd16) ||
                         (repcnt_r==5'd19) || (repcnt_r==5'd22) || (repcnt_r==5'd25) ||
                         (repcnt_r==5'd28) || (repcnt_r==5'd31) ;
assign random_mask3[1] = (repcnt_r==5'd2 ) || (repcnt_r==5'd5 ) || (repcnt_r==5'd8 ) ||
                         (repcnt_r==5'd11) || (repcnt_r==5'd14) || (repcnt_r==5'd17) || 
                         (repcnt_r==5'd20) || (repcnt_r==5'd23) || (repcnt_r==5'd26) ||
                         (repcnt_r==5'd29) ;
assign random_mask3[2] = (repcnt_r==5'd3 ) || (repcnt_r==5'd6 ) || (repcnt_r==5'd9 ) ||
                         (repcnt_r==5'd12) || (repcnt_r==5'd15) || (repcnt_r==5'd18) ||
                         (repcnt_r==5'd21) || (repcnt_r==5'd24) || (repcnt_r==5'd27) ||
                         (repcnt_r==5'd30) || (repcnt_r==5'd0)  ; 

assign random_mask4[0] = (random_num_radix4==2'b00);
assign random_mask4[1] = (random_num_radix4==2'b01);
assign random_mask4[2] = (random_num_radix4==2'b10);
assign random_mask4[3] = (random_num_radix4==2'b11);

assign random_mask2[0] = ~random_num_radix4[0];
assign random_mask2[1] = random_num_radix4[0];


wire [1:0] set_1_in_4 = random_num_radix4; 
wire [1:0] set_1_in_3 = {2{random_mask3[0]}} & first_1_set_num  |
                        {2{random_mask3[1]}} & second_1_set_num |
                        {2{random_mask3[2]}} & third_1_set_num  ;
wire [1:0] set_1_in_2 = {2{random_mask2[0]}} & first_1_set_num  |
                        {2{random_mask2[1]}} & second_1_set_num ;
wire [1:0] set_1_in_1 = first_1_set_num;

wire [1:0] select_set_4way = (has_3_set) ? set_1_in_3 :
                             (has_2_set) ? set_1_in_2 :
                             (has_1_set) ? set_1_in_1 : random_num_radix4;
wire [1:0] select_set_2way;
assign select_set_2way[1] = 1'b0;
assign select_set_2way[0] = (lock_for_replace[1:0]==2'b01) ? 1'b1 :
                            (lock_for_replace[1:0]==2'b10) ? 1'b0 : repcnt_r[0];
assign select_set = (cr_cfg7_dcache_i==2'b01) ? 2'b00 :
                    (cr_cfg7_dcache_i==2'b10) ? select_set_2way : select_set_4way;

reg [1:0] replace_set_r;
always @(posedge clock)
begin
    if (reset)
        replace_set_r <= 2'b00;
    else if (valid_replace_missq)
        replace_set_r <= select_set;
end

assign replace_cache = s_hit_st_q[head_st] & head_st_ok_stq & memres_wr_rdy & cache_wr_ok & 
                       (head_op_stq==`OP_CACHE1  | 
                        head_op_stq==`OP_CACHE21 | 
                        head_op_stq==`OP_SYNCI   ) & ~cache1_21_r;
                                        // delete (~cp0_memrd_valid) because when cache begin to replace, 
                                        // no other operation will read memory
assign set_replace_cache = head_set_stq;
assign laddr_replace_cache = head_addr_stq[11:0];
 
always @(posedge clock)
begin
    if (reset|commitbus_ex_i)
        cache1_21_r <= 1'b0;
    else if (replace_cache)
        cache1_21_r <= 1'b1;
    else if (cache1_21_r)
        cache1_21_r <= 1'b0;
end

assign valid_replace = (valid_replace_missq | replace_cache); // & (~cp0_memrd_valid); //& memres_wr_rdy 
assign set_replace = (valid_replace_missq) ? select_set : set_replace_cache;
wire [11:0] laddr_replace_missq = {12{replace_missq[0]}} & addr_miss_q[0][11:0] |
                                  {12{replace_missq[1]}} & addr_miss_q[1][11:0] |
                                  {12{replace_missq[2]}} & addr_miss_q[2][11:0] ;

assign laddr_replace = (valid_replace_missq) ? laddr_replace_missq : laddr_replace_cache;

////// refill_req_o bus
assign valid_refill = (|refill_missq);
assign set_refill = replace_set_r;

wire [31:0] addr_refill_missq_tmp;
wire [31:0] addr_refill_missq;
assign addr_refill_missq_tmp = {32{refill_missq[0]}} & addr_miss_q[0] |
                               {32{refill_missq[1]}} & addr_miss_q[1] |
                               {32{refill_missq[2]}} & addr_miss_q[2] ;
assign addr_refill_missq = {addr_refill_missq_tmp[31:5], 5'b0};

assign laddr_refill = addr_refill_missq[11:0];

wire        set_lock_refill_missq;
wire [21:0] tag_refill_missq;
assign set_lock_refill_missq = refill_missq[0] & (set_lock_miss_q[0]) |
                               refill_missq[1] & (set_lock_miss_q[1]) |
                               refill_missq[2] & (set_lock_miss_q[2]) ;

assign dirty_refill = refill_missq[0] & dirty_miss_q[0] |
                      refill_missq[1] & dirty_miss_q[1] |
                      refill_missq[2] & dirty_miss_q[2] ;

assign tag_refill_missq = {set_lock_refill_missq, addr_refill_missq[31:12], 1'b1};

assign tagout_refill = tag_refill_missq;

assign dataout_refill = {256{refill_missq[0]}} & data_miss_q[0] |
                       {256{refill_missq[1]}} & data_miss_q[1] |
                       {256{refill_missq[2]}} & data_miss_q[2] ;

////// memq_to_dcache_o bus
assign st_set_memq = set_st_req;
assign st_laddr_memq = laddr_st_req;
assign set_memq = (valid_replace_missq) ? select_set : 
                  (valid_refill)  ? set_refill  : set_cache_req; //valid_cache_req

assign laddr_memq = (valid_replace_missq) ? laddr_replace_missq : 
                    (valid_refill ) ? laddr_refill  : laddr_cache_req; 

assign tag_wen_memq = valid_refill | valid_cache_req;

assign tagout_memq = (valid_refill) ? tagout_refill : tagout_cache_req;

assign data_wen_memq = (valid_refill) ? 32'hffff_ffff : wen_st_req;

assign dataout_memq = (valid_refill) ? dataout_refill : value_st_req;




////// replace broadcast bus
assign replace_bc_valid = valid_dump & tagout_dump[0];
wire [11:0] replace_bc_laddr = (|refill_missq) ? addr_refill_missq[11:0] : head_addr_stq[11:0];
assign replace_bc_addr = {tagout_dump[20:1], replace_bc_laddr[11:0]};


////// refill broadcast bus
assign refill_bc_valid = (|refill_missq);
assign refill_bc_addr  = addr_refill_missq;


////// cp0res_o bus
///result write back bus
assign wb_valid = forward_valid_r | st_wb_valid;

assign ldq_wb_valid = forward_valid_r & ~forward_queue_nu_r[2];

assign wb_ldq_v[0] = ldq_wb_valid & forward_queue_nu_r[1:0]==2'b00;
assign wb_ldq_v[1] = ldq_wb_valid & forward_queue_nu_r[1:0]==2'b01;
assign wb_ldq_v[2] = ldq_wb_valid & forward_queue_nu_r[1:0]==2'b10;
assign wb_ldq_v[3] = ldq_wb_valid & forward_queue_nu_r[1:0]==2'b11;

assign wb_qid = (ldq_wb_valid) ? qid_ld_q[forward_queue_nu_r[1:0]] : head_qid_stq;
assign wb_fpqid = 3'b0;

wire [31:0] ldq_value = value_ld_q[forward_queue_nu_r[1:0]];
wire [31:0] wb_raw_value = (ldq_wb_valid) ? ldq_value : head_value_stq;

wire [31:0] wb_raw_value_h = (ldq_wb_valid) ? value_h_ld_q[forward_queue_nu_r[1:0]] :
                                            head_value_h_stq;

wire [7:0] wb_load_op = op_ld_q[forward_queue_nu_r[1:0]];
assign  wb_op = (ldq_wb_valid) ? wb_load_op : head_op_stq;

assign wb_bnt = (wb_op==`OP_ERET || wb_op==`OP_DERET);

wire ldq_wb_ex = ex_ld_q[forward_queue_nu_r[1:0]];
wire has_ddblimpr = forward_valid_r & (~forward_queue_nu_r[2]&ddblimpr_ld_q[forward_queue_nu_r[1:0]] | 
                                       (forward_queue_nu_r[2:1]==2'b10)&head_ddblimpr_stq);
wire has_ddblimpr_h = forward_valid_r & (~forward_queue_nu_r[2]&ddblimpr_h_ld_q[forward_queue_nu_r[1:0]] | 
                                        (forward_queue_nu_r[2:1]==2'b10)&head_ddblimpr_h_stq);

assign wb_ex = ldq_wb_valid & ldq_wb_ex;

assign wb_addr_offset = (ldq_wb_valid) ? addr_ld_q[forward_queue_nu_r[1:0]][2:0]  :
                                            head_addr_stq[2:0] ;

wire [31:0] wb_value_no_sc;

form_mmres_value u0_form_mmres_value(.in(wb_raw_value),
                                     .in_h(wb_raw_value_h),
                                     .addr({1'b0, wb_addr_offset[1:0]}),
                                     .ex(wb_ex),
                                     .op(wb_op),
                                     .out(wb_value_no_sc));

assign wb_value   = ((wb_op==`OP_SC)&(~wb_ex)&(~ldq_wb_valid)) ? {31'b0, &head_ben_stq} : wb_value_no_sc;
assign wb_value_h = wb_raw_value_h;

assign op_ctc1_wb     = 1'b0;
assign eret_deret_wb  = ldq_wb_valid & wb_bnt;
assign condition_true = 1'b1;
assign ddbl_mmres = wb_ex & ddbl_r;
assign ddbs_mmres = wb_ex & ddbs_r;
assign ddblimpr_mmres = (has_ddblimpr | has_ddblimpr_h) & HB_DDBLIMPR;
assign ddbsimpr_mmres = wb_ex & ddbsimpr_r; 
assign bnt_mmres = wb_bnt;
assign valid_mmres = wb_valid;
assign op_mmres = wb_op;
assign qid_mmres = wb_qid;
assign fpqid_mmres = wb_fpqid;
assign value_h_mmres = wb_value_h;
assign value_mmres = wb_value;
assign mod_mmres = wb_ex & mod_r;
assign tlbli_mmres = wb_ex & tlbli_r;
assign tlblr_mmres = wb_ex & tlblr_r;
assign tlbsi_mmres = wb_ex & tlbsi_r; 
assign tlbsr_mmres = wb_ex & tlbsr_r;
assign adel_mmres = wb_ex & adel_r;
assign ades_mmres = wb_ex & ades_r;
assign dbe_mmres = 1'b0;
assign watch_mmres = 1'b0; //wb_ex & watch_r;
assign ri_mmres = wb_ex & ri_r;

///// mmres_to_mmrs_o
assign valid_res_to_mmrs = forward_valid_r;
assign qid_res_to_mmrs   = wb_qid;
assign value_res_to_mmrs = wb_raw_value;

///// ll_set_llbit_o 
assign ll_set_llbit_o = forward_valid_r & ~wb_ex & (wb_op==`OP_LL);

///// connect with hb module for EJTAG addr&data match
assign LOADDATA_VALID = has_ddblimpr;
assign LOADDATA = wb_raw_value;
assign LOADDATA_QID = wb_qid;
assign BYTELANE = bytelane_r;
assign LOADDATA_H_VALID = has_ddblimpr_h;
assign LOADDATA_H = wb_raw_value_h;

/////// memqueue_stall_o
reg cache29_miss_read_mem_r;
always @(posedge clock)
begin
    if (reset | commitbus_ex_i | cache29_miss_read_mem_r&missq_all_blank)
        cache29_miss_read_mem_r <= 1'b0;
    else if (st_wb_valid_miss & head_op_cache29_stq)
        cache29_miss_read_mem_r <= 1'b1;
end

assign memqueue_stall_o = has_dcache_in_memqueue  | 
                          cache29_miss_read_mem_r |
                          ((inst_cache_at_addr_i|inst_sync_at_addr_i)&
                           (~(missq_all_blank&stq_all_blank&ldq_all_blank))) |
                          (inst_sync_at_addr_i&(state_acc!=BLANK_ACC));

/////// memqueue_has_ll_o
wire [3:0] ll_ldq = {(op_ld_q[3]==`OP_LL),
                     (op_ld_q[2]==`OP_LL),
                     (op_ld_q[1]==`OP_LL),
                     (op_ld_q[0]==`OP_LL)};

wire [1:0] ll_stq = {(op_st_q[1]==`OP_LL), (op_st_q[0]==`OP_LL)};

assign memqueue_has_ll_o = (|(~blank_ldq & ll_ldq)) || (|(~s_blank_st_q & ll_stq)); 

/////// cp0_memraddr_o bus
wire [31:0] missq_rd_mem_addr;
wire [31:0] uncache_ld_mem_addr;
assign missq_read_mem = |to_mem_read_missq;
assign missq_rd_mem_addr = {32{to_mem_read_missq[0]}} & addr_miss_q[0]  |
                           {32{to_mem_read_missq[1]}} & addr_miss_q[1]  |
                           {32{to_mem_read_missq[2]}} & addr_miss_q[2]  ;
assign uncache_ld_mem_addr = {head_addr_stq[31:2], 
                              ((head_op_stq==`OP_LWL || head_op_stq==`OP_LWR) ? 2'b00 : head_addr_stq[1:0])};

assign cp0_memrd_valid =  missq_read_mem | ld_to_mem;
assign cp0_memrd_width = (missq_read_mem) ? 4'b0001 : uncache_mem_width;
assign cp0_memrd_addr  = (missq_read_mem) ? missq_rd_mem_addr : uncache_ld_mem_addr; 
wire [2:0] missq_read_id = (to_mem_read_missq[0]) ? 3'b000 : 
                           (to_mem_read_missq[1]) ? 3'b001 : 3'b010;
assign memraddr_id    = (missq_read_mem) ? missq_read_id : 3'b110;
assign memraddr_valid = cp0_memrd_valid;
assign memraddr_width = cp0_memrd_width;
assign memraddr_addr  = cp0_memrd_addr;


////// cp0_memwaddr_o bus
wire [31:0]  uncache_st_addr_wr_mem;
wire [255:0] uncache_st_data_wr_mem;
reg  wid_r;

assign replace_write_mem = valid_dump&dirty_dump;
assign uncache_st_wr_mem = uncache_st_no_acc_req_real & state_acc==BLANK_ACC;

assign cp0_memwr_valid = replace_write_mem | uncache_st_wr_mem | acc_wr_mem;

assign cp0_memwr_width = (replace_write_mem) ? 4'b0001 : 
                                (acc_wr_mem) ? acc_mem_width : uncache_mem_width;


assign replace_addr_wr_mem = replace_bc_addr;
assign uncache_st_addr_wr_mem = {head_addr_stq[31:2], ((head_op_stq==`OP_SWL) ? 2'b00 : head_addr_stq[1:0])};

assign cp0_memwr_addr = (replace_write_mem) ? replace_addr_wr_mem : 
                               (acc_wr_mem) ? acc_mem_addr : uncache_st_addr_wr_mem;

assign uncache_st_data_wr_mem = store_datain_missq; //(uncache_mem_width[2]) ? st_dw_datain_missq : st_sw_datain_missq;

assign cp0_memwr_data = (replace_write_mem) ? dataout_dump :
                               (acc_wr_mem) ? acc_mem_data : uncache_st_data_wr_mem;

always @(posedge clock)
begin
    if (reset)
        wid_r <= 1'b0;
    else if (replace_write_mem)
        wid_r <= ~wid_r;
end

assign cache_wr_ok = ~memres_wtq_valid[wid_r];

assign memwaddr_valid = cp0_memwr_valid;
assign memwaddr_width = cp0_memwr_width;
assign memwaddr_addr  = cp0_memwr_addr;
assign memwaddr_data  = cp0_memwr_data;
assign memwaddr_ben   = head_ben_stq;
assign memwaddr_id    = (replace_write_mem) ? {2'b10, wid_r} : 
                               (acc_wr_mem) ? 3'b111 : 3'b110;

////// cp0_mem_req_o

assign cp0_mem_req_o = cp0_memrd_valid;
assign inst_cache_block_o   = missq_read_mem | ld_to_mem;
assign inst_uncache_block_o = inst_cache_block_o | uncache_st_wr_mem | acc_wr_mem;

assign data_inter_conflict_o = (valid_replace|replace_write_mem) & (|((~all_rdy_at_valid|ld_in_miss_q) & iterm_rd_mem_v) | 
                                                                    ld_to_mem_req_tmp&(state_acc==BLANK_ACC)             |
                                                                    uncache_st_no_acc_req_tmp & state_acc==BLANK_ACC & ~sc_not_write  |
                                                                    acc_line_mem_can_out |
                                                                    acc_split_mem_can_out)  ||
                               missq_read_mem & (ld_to_mem_req_tmp&(state_acc==BLANK_ACC) |
                                                 uncache_st_no_acc_req_tmp & state_acc==BLANK_ACC & ~sc_not_write |
                                                 acc_line_mem_can_out | 
                                                 acc_split_mem_can_out);

endmodule //godson_memqueue_module


module form_mmres_value(in,in_h,addr,ex,op,out);
input [31:0] in,in_h;
input [2:0] addr;
input ex;
input [7:0] op;
output [31:0] out;

wire offset_d0,offset_d1,offset_d2,offset_d3,offset_d4,offset_d5,offset_d6,offset_d7;
wire byte0_sel_v0,byte0_sel_v1,byte0_sel_v2,byte0_sel_v3,byte0_sel_v4,byte0_sel_v5,byte0_sel_v6,byte0_sel_v7;
wire byte1_sel_v1,byte1_sel_v3,byte1_sel_v5,byte1_sel_v7;
wire byte2_sel_v2,byte2_sel_v6;
wire byte3_sel_v3,byte3_sel_v7;
wire byte4_sel_v4;
wire byte5_sel_v5;
wire byte6_sel_v6;
wire byte7_sel_v7;
wire [ 7:0] lb_sign, lh_sign, lw_sign;
wire [ 7:0] load_value_byte0,  load_value_byte1,  load_value_byte2,  load_value_byte3;
wire [ 7:0] load_value_byte4,  load_value_byte5,  load_value_byte6,  load_value_byte7;
wire [ 7:0] load_value_byte00, load_value_byte01, load_value_byte02, load_value_byte03;
wire [ 7:0] load_value_byte04, load_value_byte05, load_value_byte06, load_value_byte07;
wire [31:0] load_value, load_value_h, value_lwl, value_lwr, value_lwl_lwr;

wire op_lb  = op==`OP_LB;
wire op_lh  = op==`OP_LH  || op==`OP_LHX;
wire op_lw  = op==`OP_LW  || op==`OP_LWX || op==`OP_LWC1;
wire op_ll  = op==`OP_LL;
wire op_ld  = 1'b0;
wire op_lbu = op==`OP_LBU || op==`OP_LBUX;
wire op_lhu = op==`OP_LHU;
wire op_lwl = op==`OP_LWL;
wire op_lwr = op==`OP_LWR;
wire op_load= op_lb|op_lh|op_lw|op_ll|op_ld|op_lbu|op_lhu|op_lwl|op_lwr;

assign offset_d0        = addr[2:0]==3'b000;
assign offset_d1        = addr[2:0]==3'b001;
assign offset_d2        = addr[2:0]==3'b010;
assign offset_d3        = addr[2:0]==3'b011;
assign offset_d4        = addr[2:0]==3'b100;
assign offset_d5        = addr[2:0]==3'b101;
assign offset_d6        = addr[2:0]==3'b110;
assign offset_d7        = addr[2:0]==3'b111;

assign byte0_sel_v0     =offset_d0;
assign byte0_sel_v1     =offset_d1;
assign byte0_sel_v2     =offset_d2;
assign byte0_sel_v3     =offset_d3;
assign byte0_sel_v4     =offset_d4;
assign byte0_sel_v5     =offset_d5;
assign byte0_sel_v6     =offset_d6;
assign byte0_sel_v7     =offset_d7;
assign byte1_sel_v1     =offset_d0&(op_lh|op_lhu|op_lw|op_ll|op_ld);
assign byte1_sel_v3     =offset_d2&(op_lh|op_lhu);
assign byte1_sel_v5     =offset_d4&(op_lh|op_lhu|op_lw|op_ll);
assign byte1_sel_v7     =offset_d6&(op_lh|op_lhu);
assign byte2_sel_v2     =offset_d0&(op_lw|op_ll|op_ld);
assign byte2_sel_v6     =offset_d4&(op_lw|op_ll);
assign byte3_sel_v3     =offset_d0&(op_lw|op_ll|op_ld);
assign byte3_sel_v7     =offset_d4&(op_lw|op_ll);

assign load_value_byte0 =({8{byte0_sel_v0}}&  in[ 7: 0]) | ({8{byte0_sel_v1}}&  in[15: 8]) |
                         ({8{byte0_sel_v2}}&  in[23:16]) | ({8{byte0_sel_v3}}&  in[31:24]) |
                         ({8{byte0_sel_v4}}&in_h[ 7: 0]) | ({8{byte0_sel_v5}}&in_h[15: 8]) |
                         ({8{byte0_sel_v6}}&in_h[23:16]) | ({8{byte0_sel_v7}}&in_h[31:24]);
assign load_value_byte1 =({8{byte1_sel_v1}}&  in[15: 8]) | ({8{byte1_sel_v3}}&  in[31:24]) |
                         ({8{byte1_sel_v5}}&in_h[15: 8]) | ({8{byte1_sel_v7}}&in_h[31:24]);
assign load_value_byte2 =({8{byte2_sel_v2}}&  in[23:16]) | ({8{byte2_sel_v6}}&in_h[23:16]);
assign load_value_byte3 =({8{byte3_sel_v3}}&  in[31:24]) | ({8{byte3_sel_v7}}&in_h[31:24]);

assign lb_sign          =load_value_byte0[7]&op_lb? 8'hff:8'b0;
assign lh_sign          =load_value_byte1[7]&op_lh? 8'hff:8'b0;
assign lw_sign          =load_value_byte3[7]&(op_lw|op_ll)? 8'hff:8'b0;

assign load_value_byte00=load_value_byte0;
assign load_value_byte01=load_value_byte1|lb_sign;
assign load_value_byte02=load_value_byte2|lb_sign|lh_sign;
assign load_value_byte03=load_value_byte3|lb_sign|lh_sign;

form_value_lwl lwl00(.offset(addr[1:0]),.value_in(in),.value_in_h(in_h),.value_out(value_lwl));
form_value_lwr lwr00(.offset(addr[1:0]),.value_in(in),.value_in_h(in_h),.value_out(value_lwr));
assign load_value    = (op_lwl) ? value_lwl :
                       (op_lwr) ? value_lwr : {load_value_byte03, load_value_byte02, 
                                               load_value_byte01, load_value_byte00};

assign out           = op_load&(~ex)?load_value:in;

endmodule

module form_value_lwl(offset,value_in,value_in_h,value_out);
input  [1:0]  offset;
input  [31:0] value_in;
input  [31:0] value_in_h;
output [31:0] value_out;
reg    [31:0] value_out;

always @(offset or value_in or value_in_h)
   begin
     case(offset[1:0]) // synopsys full_case parallel_case
        2'b00:value_out={value_in[ 7:0],value_in_h[23: 0]};
        2'b01:value_out={value_in[15:0],value_in_h[15: 0]};
        2'b10:value_out={value_in[23:0],value_in_h[ 7: 0]};
        2'b11:value_out={value_in[31:0]};
     endcase
   end
endmodule

module form_value_lwr(offset,value_in,value_in_h,value_out);
input [1:0]  offset;
input [31:0] value_in;
input [31:0] value_in_h;
output[31:0] value_out;
reg   [31:0] value_out;

always @(offset or value_in or value_in_h)
   begin
     case(offset[1:0]) // synopsys full_case parallel_case
        2'b00:value_out={value_in[31:0]};
        2'b01:value_out={value_in_h[31:24],value_in[31: 8]};
        2'b10:value_out={value_in_h[31:16],value_in[31:16]};
        2'b11:value_out={value_in_h[31:8],value_in[31:24]};
     endcase
   end
endmodule

module mem_tool_first_one_4_4_from_i(in, i, out);
input  [3:0] in;
input  [1:0] i;
output [3:0] out;

wire [3:0] v_g_i;
wire [3:0] in_g_i;
wire [3:0] out_g_i;
wire [3:0] out_le_i;

assign in_g_i = (i==2'b00)&{in[3:1], 1'b0} |
                (i==2'b01)&{in[3:2], 2'b0} |
                (i==2'b10)&{in[3],   3'b0} ;

//mem_tool_first_one_4_4 u0_first_1_4_4(.in(in_g_i), .out(out_g_i));
assign out_g_i[0] = in_g_i[0];
assign out_g_i[1] = in_g_i[1] & (~in_g_i[0]);
assign out_g_i[2] = in_g_i[2] & (~|in_g_i[1:0]);
assign out_g_i[3] = in_g_i[3] & (~|in_g_i[2:0]);

//mem_tool_first_one_4_4 u1_first_1_4_4(.in(in),     .out(out_le_i));
assign out_le_i[0] = in[0];
assign out_le_i[1] = in[1] & (~in[0]);
assign out_le_i[2] = in[2] & (~|in[1:0]);
assign out_le_i[3] = in[3] & (~|in[2:0]);

assign out = (|in_g_i) ? out_g_i : out_le_i;

endmodule //mem_tool_first_one_4_4_from_i


module mem_tool_encode_4_2(in, out);
input  [3:0] in;
output [1:0] out;

assign out = ({2{in[0]}}&2'b00) |({2{in[1]}}&2'b01) |
             ({2{in[2]}}&2'b10) |({2{in[3]}}&2'b11) ;

endmodule

module mem_tool_encode_6_3(in, out);
input  [5:0] in;
output [2:0] out;

wire in_0 = ~(|in);
assign out = ({3{in[0]}}&3'b000) |({3{in[1]}}&3'b001) |
             ({3{in[2]}}&3'b010) |({3{in[3]}}&3'b011) |
             ({3{in[4]}}&3'b100) |({3{in[5]}}&3'b101) |
             ({3{in_0 }}&3'b111) ;

endmodule


module mem_tool_decode_3_6(in, out);
input  [2:0] in;
output [5:0] out;

assign out[0] = in==3'b000;
assign out[1] = in==3'b001;
assign out[2] = in==3'b010;
assign out[3] = in==3'b011;
assign out[4] = in==3'b100;
assign out[5] = in==3'b101;

endmodule //mem_tool_decode_3_6           

module form_cacheline_wen(in, offset, out);
input  [3:0]  in;
input  [2:0]  offset;
output [31:0] out;
reg    [31:0] out;

always @(in or offset)
begin
    case (offset) // synopsys full_case parallel_case
      3'b000:
        out = {28'h0000000, in};
      3'b001:
        out = {24'h000000, in, 4'h0};
      3'b010:
        out = {20'h00000, in, 8'h00};
      3'b011:
        out = {16'h0000, in, 12'h000};
      3'b100:
        out = {12'h000, in, 16'h0000};
      3'b101:
        out = {8'h00, in, 20'h00000};
      3'b110:
        out = {4'h0, in, 24'h000000};
      3'b111:
        out = {in, 28'h0000000};
    endcase
end

endmodule

module mem_tool_first_one_6_3_from_i (in,i,out,valid);
input  [5:0] in;
input  [2:0] i;
output [2:0] out;
output       valid; 

wire [5:0] v_ge_i;
wire [5:0] in_ge_i;
wire [2:0] out_ge_i;
wire [2:0] out_l_i;

assign in_ge_i = {6{i==3'b000}}&{in} |
                 {6{i==3'b001}}&{in[5:1], 1'b0} |
                 {6{i==3'b010}}&{in[5:2], 2'b0} |
                 {6{i==3'b011}}&{in[5:3], 3'b0} |
                 {6{i==3'b100}}&{in[5:4], 4'b0} |
                 {6{i==3'b101}}&{in[5],   5'b0} ;

//mem_tool_first_one_6_3 u0_first_1_6_3(.in(in_ge_i), .out(out_ge_i));
assign out_ge_i = (in_ge_i[0]==1'b1) ? 3'b000 :
                  (in_ge_i[1]==1'b1) ? 3'b001 :
                  (in_ge_i[2]==1'b1) ? 3'b010 :
                  (in_ge_i[3]==1'b1) ? 3'b011 :
                  (in_ge_i[4]==1'b1) ? 3'b100 : 3'b101;

//mem_tool_first_one_6_3 u1_first_1_6_3(.in(in),      .out(out_l_i ));
assign out_l_i  = (in[0]==1'b1) ? 3'b000 :
                  (in[1]==1'b1) ? 3'b001 :
                  (in[2]==1'b1) ? 3'b010 :
                  (in[3]==1'b1) ? 3'b011 :
                  (in[4]==1'b1) ? 3'b100 : 3'b101;

assign out = (|in_ge_i) ? out_ge_i : out_l_i;

assign valid = |in;

endmodule
