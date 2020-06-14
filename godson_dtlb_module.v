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

module godson_dtlb_module(
    clock,
    reset,
    commitbus0_i,
    commitbus1_i,
    brbus_to_cp0_i,
    int_in,
    int_out,
    status_bev_o,
    status_exl_o,
    status_erl_o,
    status_ksu_o,  
    status_cu_o,
    mode_user,
    mode_super,
    config_k0,
    HWREna,
    vector_int, 
    ebase,
    entryhi_vpn2_o,

////// connection with itlb
    itlb_req_i,
    tlb_to_itlb_o,
    itlb_flush_o,

////// connection with icache
    cache28_refill_ok_i,
    tlb_to_icache_o,

////// connection with interface
    filter_window_req_i,
    filter_window_result_o,

////// performance counter
    imemread_valid_i,
    dmemread_valid_i,
    duncache_valid_i,
    mreadreq_valid_i, 
    mwritereq_valid_i, 
    data_inst_conflict_i,
    data_inter_conflict_i,
    qissuebus_valid0_i,
    qissuebus_valid1_i,
    brq_full_i,
    stalled_fetch_i,
    missq_full_i,
    not_store_ok_i,
    st_ld_conflict_i,
    queue_full_i,
    icache_hit_i,
    icache_access_i,
    icache_update_i,
    icache_way_hit_i,
    itlb_access_i,
    flush_pipeline_cycle_i,
    inst_to_queue_cycle_i,
    insts_to_alu1_i,
    insts_to_alu2_i,
    insts_to_addr_i,
    insts_to_falu_i,
    insts_to_queue_i,
    insts_fetched_i,
    stalled_cycles_icachemiss_i,
    itlbmiss_tlbhit_i,
////// ejtag signals
    IBE_FROM_CACHE,
    HB_DDBL,
    HB_DDBS,
    HB_DDBSIMPR,
    PROBEN_IN,
    DMSEG_DFREE,
    
    DEBUG_MODE,
    DSS_ENABLE,
    NODCR,
    IEXI,
    PENDING_DBE,
    PENDING_IBE,
    FIRST_MEM_PASS_AFTER_DERET,
    EJTAGBRK_FROM_CORE,
    
    HB_REQBUS,
    DCR_REQBUS,
    DMSEG_DREQBUS,
    HB_DCOMPBUS,

    addr_to_tlb_i,
    inst_valid_at_addr_i,
    inst_cache_at_addr_i,
    inst_sync_at_addr_i,
    tlb_allowin_o,
    tlb_stall_o,
    tlb_has_ll_o,

    dcache_result_i,
    tag_to_tlb_i,
    conflict_to_tlb_i,
    dcachepaddr_o,
    tlb_read_again_o,


    load_queue_full_i,
    store_queue_full_i,
    miss_req_queue_full_i,
    ex_memqueue_i,
    rand_num_i,
    ll_set_llbit_i,
    tlb_to_memqueue_o,
    tlb_forward_bus_o,
    cr_llbit_value_o,
    cr_cfg6_brpred_type_o,
    cr_cfg6_rti_o,
    cr_cfg6_cache0_all_o,
    cr_cfg7_dcache_o,
    cr_cfg7_icache_o,

    ram_to_tlb_i,
    tlb_to_ram_o,
    DRESBUS_FROM_DRSEG, 
    DRESBUS_FROM_DCR  , 
    DRESBUS_FROM_DMSEG,

    HB_LOAD_ADDR_MATCH,
    DATA_FROM_TAP,
    hb_dbs_bs_i
);
parameter MMBUF_INVALID     = 2'b00;
parameter MMBUF_VALID       = 2'b01;
parameter MMBUF_READ_AGAIN  = 2'b10;

////// whole system signals
input           clock;
input           reset;
input   [`Lcommitbus_to_tlb-1:0]  commitbus0_i;
input   [`Lcommitbus_to_tlb-1:0]  commitbus1_i;
input   [  18:0] brbus_to_cp0_i;
input   [  5:0] int_in;

output          int_out;
output          status_bev_o;
output          status_exl_o;
output          status_erl_o;
output  [  1:0] status_ksu_o;  
output  [  3:0] status_cu_o;
output  [ 18:0] entryhi_vpn2_o;

output          mode_user;
output          mode_super;
output  [  2:0] config_k0;

output  [3:0 ]  HWREna;  
output  [8:0 ]  vector_int;  
output  [31:0]  ebase;  
///// connection with fetch and icache
input   [`Litlb_req-1:0] itlb_req_i;
output  [`Ltlb_to_itlb-1:0] tlb_to_itlb_o;
output          itlb_flush_o;

input           cache28_refill_ok_i;
output  [`Ltlb_to_icache-1:0] tlb_to_icache_o;

////// connection with interface
input   [31:0]  filter_window_req_i;
output          filter_window_result_o;
    
//////performance counter
input           imemread_valid_i;
input           dmemread_valid_i;
input           duncache_valid_i;
input           mreadreq_valid_i;
input           mwritereq_valid_i;
input           data_inst_conflict_i;
input           data_inter_conflict_i;
input           qissuebus_valid0_i;
input           qissuebus_valid1_i;
input           brq_full_i;
input           stalled_fetch_i;
input           missq_full_i;
input           not_store_ok_i;
input           st_ld_conflict_i;
input           queue_full_i;
input           icache_hit_i;
input           icache_access_i;
input           icache_update_i;
input           icache_way_hit_i;
input           itlb_access_i;

input           flush_pipeline_cycle_i;
input           inst_to_queue_cycle_i;
input           insts_to_alu1_i;
input           insts_to_alu2_i;
input           insts_to_addr_i;
input           insts_to_falu_i;
input[1:0]      insts_to_queue_i;
input[1:0]      insts_fetched_i;
input           stalled_cycles_icachemiss_i;
input           itlbmiss_tlbhit_i;

////// ejtag signals
input           IBE_FROM_CACHE;
input           HB_DDBL;
input           HB_DDBS;
input           HB_DDBSIMPR;
input           PROBEN_IN;
input           DMSEG_DFREE;

output          DEBUG_MODE;
output          DSS_ENABLE;
output          NODCR;
output          IEXI;
output          PENDING_DBE;
output          PENDING_IBE;
output          FIRST_MEM_PASS_AFTER_DERET;
output          EJTAGBRK_FROM_CORE;
output  [ 78:0] HB_REQBUS;
output  [ 78:0] DCR_REQBUS;
output  [ 78:0] DMSEG_DREQBUS;
output  [`Lhb_dcompbus-1:0] HB_DCOMPBUS;

input   [ 64:0] DRESBUS_FROM_DRSEG; 
input   [ 64:0] DRESBUS_FROM_DCR  ; 
input   [ 64:0] DRESBUS_FROM_DMSEG; 

input   [  1:0] HB_LOAD_ADDR_MATCH;

input   [`DBKP_NUM-1:0] hb_dbs_bs_i;

////// connect with addr module
input   [`Laddr_to_tlb-1:0] addr_to_tlb_i;
input           inst_valid_at_addr_i;
input           inst_cache_at_addr_i;
input           inst_sync_at_addr_i;
output          tlb_allowin_o;
output          tlb_stall_o;
output          tlb_has_ll_o;

////// connect with dcache module
input   [`Ldcache_result-1:0] dcache_result_i;
input   [ 31:0] tag_to_tlb_i;
input           conflict_to_tlb_i;
output  [ 31:0] dcachepaddr_o;
output  [`Ltlb_read_again-1:0] tlb_read_again_o;


////// connect with memqueue module
input           load_queue_full_i;
input           store_queue_full_i;
input           miss_req_queue_full_i;
input           ex_memqueue_i;
input   [  2:0] rand_num_i;
input           ll_set_llbit_i;
output  [`Ltlb_to_memq-1:0] tlb_to_memqueue_o;
output  [`Ltlb_forward-1:0] tlb_forward_bus_o;
output          cr_llbit_value_o;
output  [2:0]   cr_cfg6_brpred_type_o;
output          cr_cfg6_rti_o;
output          cr_cfg6_cache0_all_o;
output  [1:0]   cr_cfg7_dcache_o;
output  [1:0]   cr_cfg7_icache_o;

////// RAM access interface
input   [`Lram_to_tlb-1:0] ram_to_tlb_i;
output  [`Ltlb_to_ram-1:0] tlb_to_ram_o;

////input form tap buffer
input[32:0] DATA_FROM_TAP;

////// all fields of brbus_to_cp0_i
wire        brbus_brerr    = brbus_to_cp0_i[  0];
wire [ 5:0] brbus_brmask   = brbus_to_cp0_i[6:1];
wire        brbus_valid    = brbus_to_cp0_i[  7];
wire [ 7:0] brbus_op       = brbus_to_cp0_i[15:8];
wire        brbus_jr31     = brbus_to_cp0_i[  16];
wire        brbus_bht      = brbus_to_cp0_i[  17];
wire        brbus_static_br  = brbus_to_cp0_i[  18];

wire        mmbuf_brbus_cancel;
wire        addr_brbus_cancel;

////// all fields of tlb_forward_bus_o
wire        fpq_forward;
wire        valid_forward;
wire [ 3:0] qid_forward;
wire [ 2:0] fpqid_forward;

assign tlb_forward_bus_o[8]   = fpq_forward;
assign tlb_forward_bus_o[7:5] = fpqid_forward;
assign tlb_forward_bus_o[4  ] = valid_forward;
assign tlb_forward_bus_o[3:0] = qid_forward;

////// all fields of itlb_req_i bus
wire        itlb_lookup_req = itlb_req_i[32  ];
wire [31:0] itlb_lookup_pc  = itlb_req_i[31:0]; 

////// all fields of tlb_to_itlb_o bus
wire        tlb_valid_to_itlb;
wire        tlb_find_to_itlb;     
wire [ 7:0] asid_to_itlb;
wire [ 1:0] random_index_to_itlb;    
wire [15:0] tlb_pagemask_to_itlb; 
wire [ 7:0] tlb_asid_to_itlb;     
wire        tlb_g_to_itlb;        

assign tlb_to_itlb_o[36]    = tlb_valid_to_itlb;
assign tlb_to_itlb_o[35]    = tlb_find_to_itlb;
assign tlb_to_itlb_o[34:27] = asid_to_itlb;
assign tlb_to_itlb_o[26:25]	= random_index_to_itlb;
assign tlb_to_itlb_o[24:9]	= tlb_pagemask_to_itlb;
assign tlb_to_itlb_o[8:1]	= tlb_asid_to_itlb;
assign tlb_to_itlb_o[0]		= tlb_g_to_itlb;


////// all fields of tlb_to_icache_o
wire        vaddr_cache28_icache;
wire        cache28_icache;
wire        vaddr_cache16_icache;
wire        vaddr_valid_icache;
wire [ 6:0] vaddr_index_icache;
wire [31:0] taglow_icache;
wire        valid_icache;
wire [31:0] paddr_icache;
wire        cache16_icache;
wire        cache8_icache;
wire        cache0_icache;
wire [ 1:0] set_icache;

assign tlb_to_icache_o[81:80] = set_icache;
assign tlb_to_icache_o[79   ] = vaddr_cache28_icache;
assign tlb_to_icache_o[78   ] = cache28_icache;
assign tlb_to_icache_o[77   ] = vaddr_cache16_icache;
assign tlb_to_icache_o[76   ] = vaddr_valid_icache;
assign tlb_to_icache_o[75:69] = vaddr_index_icache;
assign tlb_to_icache_o[68:37] = taglow_icache;
assign tlb_to_icache_o[36   ] = valid_icache;
assign tlb_to_icache_o[34: 3] = paddr_icache;
assign tlb_to_icache_o[2    ] = cache16_icache;
assign tlb_to_icache_o[1    ] = cache8_icache;
assign tlb_to_icache_o[0    ] = cache0_icache;

///// all fields of itlb_flush_o bus
wire        itlb_flush_valid;

assign itlb_flush_o       = itlb_flush_valid | reset;


////// all fields of addr_to_tlb_i
wire [ 2:0] fpqid_addrout     = addr_to_tlb_i[117:115];
wire [ 2:0] brqid_addrout     = addr_to_tlb_i[114:112];
wire        cond_true_addrout = addr_to_tlb_i[111    ];
wire        valid_addrout_in  = addr_to_tlb_i[110    ]; 
wire        rd_tag_addrout    = addr_to_tlb_i[109    ];
wire        rd_data_addrout   = addr_to_tlb_i[108    ];
wire [ 3:0] qid_addrout       = addr_to_tlb_i[107:104]; 
wire [ 7:0] op_addrout        = addr_to_tlb_i[103: 96]; 
wire [31:0] vaddr_addrout     = addr_to_tlb_i[ 95: 64]; 
wire [31:0] value_addrout     = addr_to_tlb_i[ 63: 32]; 
wire [31:0] value_h_addrout   = addr_to_tlb_i[ 31:  0]; 

wire        valid_addrout   = valid_addrout_in & ~addr_brbus_cancel;

////// all fields of dcache_result_i
wire        hit_dcache     = dcache_result_i[70   ]; 
wire [ 1:0] hit_set_dcache = dcache_result_i[69:68];
wire [ 3:0] lock_dcache    = dcache_result_i[67:64];

////// all fields of tag_to_tlb_i
wire        taglo_lock = tag_to_tlb_i[23  ];  
wire [19:0] taglo_tag  = tag_to_tlb_i[22:3]; 
wire [ 1:0] taglo_cs   = tag_to_tlb_i[2 :1];  

////// all fields of tlb_read_again_o
wire        valid_tlb_rag;
wire        rd_tag_tlb_rag;
wire        rd_data_tlb_rag;
wire [ 7:0] op_tlb_rag;
wire [11:0] laddr_tlb_rag;

assign tlb_read_again_o[22   ] = valid_tlb_rag;
assign tlb_read_again_o[21   ] = rd_tag_tlb_rag;
assign tlb_read_again_o[20   ] = rd_data_tlb_rag;
assign tlb_read_again_o[19:12] = op_tlb_rag;
assign tlb_read_again_o[11: 0] = laddr_tlb_rag;


////// all fields of tlb_to_memqueue_o
wire        to_mq_valid;
wire        to_mq_ex;
wire        to_mq_op_load;
wire        to_mq_op_store;
wire        to_mq_cached;
wire        to_mq_uncache_acc;
wire        to_mq_hit;
wire [ 1:0] to_mq_hit_set;
wire [ 3:0] to_mq_qid;
wire [ 2:0] to_mq_fpqid;
wire [ 7:0] to_mq_op;
wire [31:0] to_mq_paddr;
wire [31:0] to_mq_value_h;
wire [31:0] to_mq_value;
wire [ 3:0] to_mq_lock;
wire        to_mq_ex_ddbl;
wire        to_mq_ex_ddbs;
wire        to_mq_ex_ddblimpr;
wire        to_mq_ex_ddblimpr_h;
wire        to_mq_ex_ddbsimpr;
wire        to_mq_ex_cacheerr;
wire        to_mq_ex_mcheck;
wire        to_mq_ex_watch;
wire        to_mq_ex_ades;
wire        to_mq_ex_adel;
wire        to_mq_ex_tlbsr;
wire        to_mq_ex_tlbsi;
wire        to_mq_ex_tlblr;
wire        to_mq_ex_tlbli;
wire        to_mq_ex_mod;
//wire        to_mq_ejtag_dseg_en;
wire        ejtag_dreq_valid;
wire        to_mq_cond_true;
wire [ 3:0] to_mq_bytelane;
wire [ 2:0] to_mq_brqid;
wire        to_mq_ex_ri;

assign tlb_to_memqueue_o[148    ] = to_mq_ex_ddblimpr_h;
assign tlb_to_memqueue_o[147:145] = to_mq_fpqid;
assign tlb_to_memqueue_o[144    ] = to_mq_ex_ri;
assign tlb_to_memqueue_o[143:141] = to_mq_brqid;
assign tlb_to_memqueue_o[140:137] = to_mq_bytelane;
assign tlb_to_memqueue_o[136    ] = to_mq_cond_true;
assign tlb_to_memqueue_o[135    ] = ejtag_dreq_valid;
assign tlb_to_memqueue_o[134    ] = to_mq_valid;
assign tlb_to_memqueue_o[133    ] = to_mq_ex;
assign tlb_to_memqueue_o[132    ] = to_mq_op_load;
assign tlb_to_memqueue_o[131    ] = to_mq_op_store;
assign tlb_to_memqueue_o[130    ] = to_mq_cached;
assign tlb_to_memqueue_o[129    ] = to_mq_uncache_acc;
assign tlb_to_memqueue_o[128    ] = to_mq_hit;
assign tlb_to_memqueue_o[127:126] = to_mq_hit_set;
assign tlb_to_memqueue_o[125:122] = to_mq_qid;
assign tlb_to_memqueue_o[121:114] = to_mq_op;
assign tlb_to_memqueue_o[113:82 ] = to_mq_paddr;
assign tlb_to_memqueue_o[81 :50 ] = to_mq_value_h;
assign tlb_to_memqueue_o[49 :18 ] = to_mq_value;
assign tlb_to_memqueue_o[17 :14 ] = to_mq_lock;
assign tlb_to_memqueue_o[13     ] = to_mq_ex_ddbl;
assign tlb_to_memqueue_o[12     ] = to_mq_ex_ddbs;
assign tlb_to_memqueue_o[11     ] = to_mq_ex_ddblimpr;
assign tlb_to_memqueue_o[10     ] = to_mq_ex_ddbsimpr;
assign tlb_to_memqueue_o[9      ] = to_mq_ex_cacheerr;
assign tlb_to_memqueue_o[8      ] = to_mq_ex_mcheck;
assign tlb_to_memqueue_o[7      ] = to_mq_ex_watch;
assign tlb_to_memqueue_o[6      ] = to_mq_ex_ades;
assign tlb_to_memqueue_o[5      ] = to_mq_ex_adel;
assign tlb_to_memqueue_o[4      ] = to_mq_ex_tlbsr;
assign tlb_to_memqueue_o[3      ] = to_mq_ex_tlbsi;
assign tlb_to_memqueue_o[2      ] = to_mq_ex_tlblr;
assign tlb_to_memqueue_o[1      ] = to_mq_ex_tlbli;
assign tlb_to_memqueue_o[0      ] = to_mq_ex_mod;



////// all fields of ram_to_tlb_i
wire        ne1  = ram_to_tlb_i[51   ];    
wire [19:0] pfn1 = ram_to_tlb_i[50:31];   
wire [ 2:0] c1   = ram_to_tlb_i[30:28];   
wire        d1   = ram_to_tlb_i[27   ];   
wire        v1   = ram_to_tlb_i[26   ];    
wire        ne0  = ram_to_tlb_i[25   ];    
wire [19:0] pfn0 = ram_to_tlb_i[24: 5];   
wire [ 2:0] c0   = ram_to_tlb_i[ 4: 2];   
wire        d0   = ram_to_tlb_i[ 1   ];   
wire        v0   = ram_to_tlb_i[ 0   ];  

wire [31:0] pfn0_from_ram = {1'b0, ne0, 4'b0, pfn0, c0, d0, v0, 1'b0};
wire [31:0] pfn1_from_ram = {1'b0, ne1, 4'b0, pfn1, c1, d1, v1, 1'b0};

////// all fields of tlb_to_ram_o
wire        tlb_ram_cen;
wire [ 5:0] tlb_ram_rindex;
wire        tlb_ram_wen;
wire [ 5:0] tlb_ram_windex;
wire [51:0] tlb_ram_wdata;

assign tlb_to_ram_o[    0] = ~tlb_ram_cen;
assign tlb_to_ram_o[ 6: 1] = tlb_ram_rindex;
assign tlb_to_ram_o[    7] = ~tlb_ram_wen;
assign tlb_to_ram_o[59: 8] = tlb_ram_wdata;
assign tlb_to_ram_o[65:60] = tlb_ram_windex;

////// all fields of commitbus0
wire           commitbus0_valid;
wire   [7:0]   commitbus0_op;
wire   [31:0]  commitbus0_value_h;
wire   [31:0]  commitbus0_value;
wire   [5:0]   commitbus0_excode;
wire           commitbus0_bd;
wire   [1:0]   commitbus0_ce;
wire           commitbus0_ex;
////// all fields of commitbus1
wire           commitbus1_valid;
wire   [7:0]   commitbus1_op;
wire   [31:0]  commitbus1_value_h;
wire   [31:0]  commitbus1_value;
wire   [5:0]   commitbus1_excode;
wire           commitbus1_bd;
wire   [1:0]   commitbus1_ce;
wire           commitbus1_ex;

wire           commitbus_valid;
wire   [7:0]   commitbus_op;
wire   [31:0]  commitbus_value_h;
wire   [31:0]  commitbus_value;
wire   [5:0]   commitbus_excode;
wire           commitbus_bd;
wire   [1:0]   commitbus_ce;
wire           commitbus_ex;

assign commitbus0_op      = commitbus0_i[82:75];
assign commitbus0_ce      = commitbus0_i[74:73];
assign commitbus0_bd      = commitbus0_i[72];
assign commitbus0_valid   = commitbus0_i[71];
assign commitbus0_value_h = commitbus0_i[70:39];
assign commitbus0_value   = commitbus0_i[38:7];
assign commitbus0_excode  = commitbus0_i[6:1];
assign commitbus0_ex      = commitbus0_i[0];

assign commitbus1_op      = commitbus1_i[82:75];
assign commitbus1_ce      = commitbus1_i[74:73];
assign commitbus1_bd      = commitbus1_i[72];
assign commitbus1_valid   = commitbus1_i[71];
assign commitbus1_value_h = commitbus1_i[70:39];
assign commitbus1_value   = commitbus1_i[38:7];
assign commitbus1_excode  = commitbus1_i[6:1];
assign commitbus1_ex      = commitbus1_i[0];

assign commitbus_valid    = commitbus0_valid|commitbus1_valid;
assign commitbus_op       = commitbus0_valid?commitbus0_op:commitbus1_op; //only for deret
assign commitbus_ce       = commitbus0_ex?commitbus0_ce:commitbus1_ce; //for cr_cause 
assign commitbus_bd       = commitbus0_ex?commitbus0_bd:commitbus1_bd;    //for cr_cause and cr_errorpc
assign commitbus_value_h  = commitbus0_ex?commitbus0_value_h:commitbus1_value_h; //for cr_errorpc
assign commitbus_value    = commitbus0_ex?commitbus0_value:commitbus1_value; //for cr_entryhi cr_badvaddr cr_context
assign commitbus_excode   = commitbus0_ex?commitbus0_excode:commitbus1_excode; //for cr_cause...
assign commitbus_ex       = commitbus0_ex|commitbus1_ex; //for ...

wire commit_deret = commitbus0_valid & (commitbus0_op==`OP_DERET) | commitbus1_valid & (commitbus1_op==`OP_DERET);
wire   ade;
//------------output signals to HardwareBreakpoint--------------------
wire   [31:0]  hb_reqbus_value;
wire   [7:0]   hb_reqbus_op;
wire   [31:0]  hb_reqbus_addr;
wire   [3:0]   hb_reqbus_qid;
wire           hb_reqbus_valid;
  
wire   [31:0]  dcr_reqbus_value;
wire   [7:0]   dcr_reqbus_op;
wire   [31:0]  dcr_reqbus_addr;
wire   [3:0]   dcr_reqbus_qid;
wire           dcr_reqbus_valid;

//------------output signals to DMSEG--------------------
wire   [31:0]  dmseg_dreqbus_value;
wire   [7:0]   dmseg_dreqbus_op;
wire   [31:0]  dmseg_dreqbus_addr;
wire   [3:0]   dmseg_dreqbus_qid;
wire           dmseg_dreqbus_valid;
                                                                                                                             
wire           dmseg_ireqbus_adei;
wire   [31:0]  dmseg_ireqbus_addr;
wire           dmseg_ireqbus_valid;


wire   [31:0]  hb_storedata;
wire   [3:0]   hb_bytelane;
wire           hb_type;
wire   [31:0]  hb_addr;
wire           hb_dvalid;
wire           hb_ivalid;


//all field of dresult_from_drseg
wire       dres_from_drseg_valid   = hb_reqbus_valid&!ade;//DRESBUS_FROM_DRSEG[0]; 
wire[31:0] dres_from_drseg_value   = DRESBUS_FROM_DRSEG[32:1]; 
wire[31:0] dres_from_drseg_value_h = DRESBUS_FROM_DRSEG[64:33]; 

//all field of dresult_from_dcr
wire       dres_from_dcr_valid   = dcr_reqbus_valid;//DRESBUS_FROM_DCR[0]; 
wire[31:0] dres_from_dcr_value   = DRESBUS_FROM_DCR[32:1]; 
wire[31:0] dres_from_dcr_value_h = DRESBUS_FROM_DCR[64:33]; 


//all field of dresult_from_dcr
wire       dres_from_dmseg_valid   = DRESBUS_FROM_DMSEG[0]; 
wire[31:0] dres_from_dmseg_value   = DRESBUS_FROM_DMSEG[32:1]; 
wire[31:0] dres_from_dmseg_value_h = DRESBUS_FROM_DMSEG[64:33]; 

wire           hb_reqbus_ades;
wire           hb_reqbus_adel;
wire           dcr_reqbus_ades;
wire           dcr_reqbus_adel;
wire           dmseg_dreqbus_ades;
wire           dmseg_dreqbus_adel;
wire           access_drseg;
wire           access_dcr;
wire           access_dmseg;
wire           op_cp1_dw;
wire           op_word;

wire       ejtag_valid  = access_drseg | access_dcr | access_dmseg&op_cp1_dw | dres_from_dmseg_valid;
//wire       ejtag_valid  = dres_from_drseg_valid | dres_from_dcr_valid | dres_from_dmseg_valid;
wire[31:0] ejtag_value  = dres_from_drseg_valid ? dres_from_drseg_value : 
                          dres_from_dcr_valid   ? dres_from_dcr_value   : dres_from_dmseg_value;
wire[31:0] ejtag_value_h = dres_from_drseg_valid ? dres_from_drseg_value_h : 
                          dres_from_dcr_valid   ? dres_from_dcr_value_h   : dres_from_dmseg_value_h;
wire       ejtag_ades    = dres_from_drseg_valid ? hb_reqbus_ades: 
                          dres_from_dcr_valid   ?  dcr_reqbus_ades : dmseg_dreqbus_ades;
wire       ejtag_adel    = dres_from_drseg_valid ? hb_reqbus_adel: 
                          dres_from_dcr_valid   ?  dcr_reqbus_adel : dmseg_dreqbus_adel;

wire        ejtag_dseg_dreq;
wire        ejtag_dcr;
wire        ejtag_drseg;
wire        ejtag_dmseg_dreq;
wire        dmseg_dreq_enable;
wire        drseg_enable;
wire [31:0] dmseg_daddr;
wire [31:0] drseg_daddr;
wire [31:0] dcr_daddr;


////// CP0 register
reg    [6:0]   cr_index;
reg    [5:0]   cr_random;
reg    [26:0]  cr_entrylo0;
wire   [31:0]  cr_entrylo0_value;
reg    [26:0]  cr_entrylo1;
wire   [31:0]  cr_entrylo1_value;
reg    [27:0]  cr_context; 
wire   [15:0]  cr_pagemask_value_out;
wire   [ 7:0]  cr_pagemask_value_in;
reg    [ 7:0]  cr_pagemask;
reg    [5:0]   cr_wired;  
reg    [3:0]   cr_HWREna;  
reg    [31:0]  cr_badvaddr;
reg    [31:0]  cr_count;
reg    [26:0]  cr_entryhi;
reg    [31:0]  cr_compare;
reg    [31:0]  cr_status;
reg    [ 4:0]  cr_intctrl;
reg    [31:0]  cr_cause;
reg    [31:0]  cr_epc;
reg    [17:0]  cr_ebase;
reg    [ 2:0]  cr_config;
reg    [ 9:0]  cr_config6;
reg    [ 31:0] cr_ejtag_value; //for improve download from tap, by xucp
reg    [ 5:0]  cr_config7; //SPECIAL_CFG
//reg    [31:0]  cr_lladdr;
//reg    [31:0]  cr_watchlo;
//reg    [17:0]  cr_watchhi;
//reg    [31:0]  cr_protmask;
//reg    [31:0]  cr_protaddr;
reg    [26:0]  cr_taglo;
reg    [31:0]  cr_taghi;
reg    [31:0]  cr_errorpc;
reg    [31:0]  cr_debug;
reg    [31:0]  cr_depc;
reg    [31:0]  cr_desave;
reg            cr_llbit;
//performance counter
reg    [10:0]  cr_perf_control0;
reg    [31:0]  cr_perf_count0;
reg    [10:0]  cr_perf_control1;
reg    [31:0]  cr_perf_count1;



reg            count_add_en;

reg  [`FILTER_WINDOW_NUM-1:0] cr_filter_en;  //reg 22, sel 7
reg  [`FILTER_WINDOW_DEPTH-1:0] cr_filter_addr; //reg 22, sel 5
reg  [31-`FILTER_MASK_UNIT:0] filter_window[`FILTER_WINDOW_NUM*2-1:0];
wire [31-`FILTER_MASK_UNIT:0] filter_window_data = filter_window[cr_filter_addr];

wire [31:0] cr_intctrl_init = `INIT_INTCTRL;
wire [31:0] cr_ebase_init   = 32'h80000000;
wire [31:0] cr_config_init  = `INIT_CONFIG;

wire   [31:0]  mfc_regi;

assign cr_entrylo0_value = {1'b0, cr_entrylo0[26], 4'b0, cr_entrylo0[25:0]};
assign cr_entrylo1_value = {1'b0, cr_entrylo1[26], 4'b0, cr_entrylo1[25:0]};

////// ITLB
wire        itlb_req_tlb_ack;
reg         itlb_req_tlb_ok_r;

////// DTLB
wire        doddpage;

wire [ 7:0] dtlb_asid;
wire [19:0] data_vpn;
wire        dtlb_wren;
wire [ 2:0] dtlb_wr_index;
wire [19:0] dtlb_wr_vpn;
wire [ 7:0] dtlb_wr_mask;
wire [ 7:0] dtlb_wr_asid;
wire        dtlb_wr_g;
wire [31:0] dtlb_wr_pfn;

wire        dtlb_flush_valid;
wire [18:0] dtlb_flush_vpn2;

wire [ 7:0] dtlb_pagemask;
wire        dtlb_find;
wire [31:0] dtlb_pfn;
wire [ 2:0] dtlb_find_index;

wire [31:0] dtlb_paddr;

wire        dv;
wire [ 2:0] dc;
wire        dd;
wire        dne;
wire        d_map;
wire        d_cached;
wire        d_uncache_acc;
wire [23:0] dpfn;
wire [31:0] dmask;

wire [31:0] dcache_paddr;

///// exception found during data access
wire        d_ex_mod;
wire        d_ex_tlblr;
wire        d_ex_tlbsr;
wire        d_ex_tlbli;
wire        d_ex_tlbsi;
wire        d_ex_adel;
wire        d_ex_ades;
wire        d_ex_watch;
wire        d_ex_mcheck;
wire        d_ex_cacheerr;
wire        d_ex_ddbl;
wire        d_ex_ddbs;
wire        d_ex_ddblimpr;
wire        d_ex_ddblimpr_h;
wire        d_ex_ddbsimpr;
wire        d_ex;
wire        d_ex_tlb;
wire        d_ex_ri;


reg         dtlb_req_tlb_ok_r;

////// TLB
// tlb_find registered, delayed to the same cycle of tlb ram data out
reg         tlb_find_r;
// tlb_pagemask_out registered
reg  [ 7:0] tlb_pagemask_r;
wire [15:0] tlb_pagemask_r_value;
// tlb_asid_out registered
reg  [ 7:0] tlb_asid_r;
// tlb_g_out registered
reg         tlb_g_r;

reg         tlb_find_ok_r;

// tlb cam read enable, when need not to look up tlb cam,
//  we set tlb_cam_index to be zero.
wire        tlb_data_lookup_en;
// when itlb miss occur, find it in tlb
wire        tlb_instr_lookup_en;
// vpn2 used to look up tlb cam 
wire [18:0] tlb_vpn2_lookup;
// asid used to look up tlb cam
wire [ 7:0] tlb_asid_lookup;
// whether tlb cam lookup is ok
wire        tlb_find;
// result index of tlb cam lookup
wire [ 5:0] tlb_index_out;
// pagemask of the hit tlb iterm
wire [ 7:0] tlb_pagemask_out;

wire [ 7:0] tlb_asid_out;

wire        tlb_g_out;

wire [18:0] tlb_vpn2_out;
// tlb cam write enable
wire        tlb_cam_wen;
// index used for TLBWI, TLBWR
wire [ 5:0] tlb_cam_windex;
// vpn2 value written into tlb cam
wire [18:0] tlbw_vpn2;
// pagemask value written into tlb cam
wire [ 7:0] tlbw_pagemask;
// asid value written into tlb cam
wire [ 7:0] tlbw_asid;
// g value written into tlb cam 
wire        tlbw_g;
// TLBR instr.
wire        tlbr_valid;
// index used for tlb cam read (TLBR) 
wire [ 5:0] tlb_cam_rindex;
// vpn2 value read from tlb cam
wire [18:0] tlbr_vpn2;
// pagemask value read from tlb cam
wire [ 7:0] tlbr_pagemask;
// asid value read from tlb cam
wire [ 7:0] tlbr_asid;
// g value read from tlb cam 
wire        tlbr_g;

wire [31:0] tlb_paddr;



///// some wires with cp0 register operation
wire [ 4:0] excode;
wire        random_wired_eq;
wire        count_compare_eq;
wire [ 5:0] cr_random_sub1;
wire [31:0] cr_count_add1;
wire [ 7:0] asid;
wire        g;   
wire        exl;
wire        erl;
wire [ 1:0] ksu;
wire [ 2:0] config_k0;
wire        mode_user;
wire        mode_super;
wire        mode_kernel;

//tlbr instr. is ready now
reg         tlbr_rdy_r;

// op can enter into its queue
wire        memqueue_allowin;
wire        normal_allowin;
wire        ld_queue_allowin;
wire        st_queue_allowin;


////// mmbuffer
reg  [ 1:0] mmbuf_state_r;
reg         mmbuf_rd_tag_r;
reg         mmbuf_rd_data_r;
reg  [ 3:0] mmbuf_qid_r;
reg  [ 2:0] mmbuf_brqid_r;
reg  [ 7:0] mmbuf_op_r;
reg  [31:0] mmbuf_vaddr_r;
reg  [31:0] mmbuf_value_h_r;
reg  [31:0] mmbuf_value_r;

reg         new_enter_r;
reg         addr_trans_ok_r;
reg  [ 2:0] select_index_r;
reg         d_ex_tlblr_r;
reg         d_ex_tlbsr_r;
reg         d_ex_tlbli_r;
reg         d_ex_tlbsi_r;

wire op_read_tag  = mmbuf_rd_tag_r;
wire op_read_data = mmbuf_rd_data_r;

wire [31:0] mmbuf_value_h = mmbuf_value_h_r;
wire [31:0] mmbuf_value = mmbuf_value_r;
wire [31:0] mmbuf_vaddr = mmbuf_vaddr_r;
wire [ 7:0] mmbuf_op    = mmbuf_op_r;

wire        mmbuf_invalid;
wire        mmbuf_valid;
wire        mmbuf_read_again;
wire        mmbuf_to_valid;
wire        mmbuf_to_invalid;
wire        mmbuf_to_read_again;

//the ram can not hold it's output
wire        ram_no_hold = 1'b0;


wire op_mtc0    = mmbuf_op == `OP_MTC0;
wire op_mfc0    = mmbuf_op == `OP_MFC0;
wire op_di      = mmbuf_op == `OP_DI;
wire op_ei      = mmbuf_op == `OP_EI;
wire op_tlbp    = mmbuf_op == `OP_TLBP;
wire op_tlbr    = mmbuf_op == `OP_TLBR;
wire op_tlbwi   = mmbuf_op == `OP_TLBWI;
wire op_tlbwr   = mmbuf_op == `OP_TLBWR;
wire op_eret    = mmbuf_op == `OP_ERET;
wire op_deret   = mmbuf_op == `OP_DERET;

wire op_lb      = mmbuf_op == `OP_LB;
wire op_lbu     = mmbuf_op == `OP_LBU;
wire op_lbux    = mmbuf_op == `OP_LBUX;
wire op_lh      = mmbuf_op == `OP_LH;
wire op_lhu     = mmbuf_op == `OP_LHU;
wire op_lhx     = mmbuf_op == `OP_LHX;
wire op_lw      = mmbuf_op == `OP_LW;
wire op_lwc1    = 1'b0;
wire op_lwx     = mmbuf_op == `OP_LWX;
wire op_ll      = mmbuf_op == `OP_LL;
wire op_lwl     = mmbuf_op == `OP_LWL;
wire op_lwr     = mmbuf_op == `OP_LWR;
wire op_ldc1    = 1'b0;
wire op_sb      = mmbuf_op == `OP_SB;
wire op_sh      = mmbuf_op == `OP_SH;
wire op_sw      = mmbuf_op == `OP_SW;
wire op_swc1    = 1'b0;
wire op_swl     = mmbuf_op == `OP_SWL;
wire op_swr     = mmbuf_op == `OP_SWR;
wire op_sc      = mmbuf_op == `OP_SC;
wire op_sdc1    = 1'b0;
wire op_prefetch = (mmbuf_op == `OP_PREF) || (mmbuf_op == `OP_PREFX);
wire op_sync    = mmbuf_op == `OP_SYNC;

wire op_cache1  = mmbuf_op == `OP_CACHE1;
wire op_cache5  = mmbuf_op == `OP_CACHE5;
wire op_cache9  = mmbuf_op == `OP_CACHE9;
wire op_cache29 = mmbuf_op == `OP_CACHE29;

wire op_cache16 = (mmbuf_op==`OP_CACHE16) || (mmbuf_op==`OP_SYNCI);
wire op_cache28 = (mmbuf_op==`OP_CACHE28);
wire op_cache8  = (mmbuf_op==`OP_CACHE8);
wire op_cache0  = (mmbuf_op==`OP_CACHE0);

wire di_op      = (mmbuf_op==`OP_DI) ||(mmbuf_op==`OP_EI);

wire op_cp0    = op_mtc0  | op_mfc0  | op_di    | op_ei    | op_tlbp  |
                 op_tlbr  | op_tlbwi | op_tlbwr | op_eret  | op_deret ;
wire op_dcache  = (mmbuf_op == `OP_CACHE1) || (mmbuf_op == `OP_CACHE5) ||
                  (mmbuf_op == `OP_CACHE9) || (mmbuf_op == `OP_CACHE17)||
                  (mmbuf_op == `OP_CACHE21)|| (mmbuf_op == `OP_CACHE29)||
                  (mmbuf_op == `OP_SYNCI)  ;

wire op_icache  = (mmbuf_op == `OP_CACHE0)  || (mmbuf_op == `OP_CACHE8)  ||
                  (mmbuf_op == `OP_CACHE16) || (mmbuf_op == `OP_CACHE28) ||
                  (mmbuf_op == `OP_SYNCI)   ;

wire op_cache   = op_dcache | op_icache;

wire op_index_dcache = (mmbuf_op == `OP_CACHE1) || (mmbuf_op == `OP_CACHE5) ||
                       (mmbuf_op == `OP_CACHE9);
wire op_index_icache = (mmbuf_op == `OP_CACHE0) || (mmbuf_op == `OP_CACHE8);
wire op_index_cache  = op_index_dcache | op_index_icache;

wire op_ld_b_align = op_lb  || op_lbu  ||
                     op_lwl || op_lwr  ||
                     op_lbux;
wire op_ld_hw_align = op_lh  || op_lhu ||
                      op_lhx ;
wire op_ld_w_align = op_lw || op_ll || op_lwx || op_lwc1;
wire op_ld_dw_align = op_ldc1;

wire op_st_b_align = op_sb  || 
                     op_swl || op_swr;
wire op_st_hw_align = op_sh;
wire op_st_w_align  = op_sw  || op_sc || op_swc1; 
wire op_st_dw_align = op_sdc1;

wire op_load   = op_read_data;
wire op_store  = op_read_tag & ~op_read_data & ~op_dcache & ~op_prefetch;

// tlb instruction
wire op_tlb    = op_tlbp | op_tlbr | op_tlbwi | op_tlbwr;

// 
wire paddr_ok  = addr_trans_ok_r | ~d_map;
wire paddr_ready = paddr_ok; // | d_ex_tlb;


assign mmbuf_invalid = mmbuf_state_r == MMBUF_INVALID;
assign mmbuf_valid = mmbuf_state_r == MMBUF_VALID;
assign mmbuf_read_again = mmbuf_state_r == MMBUF_READ_AGAIN;

wire has_dbs_bs_set = ~DEBUG_MODE & |hb_dbs_bs_i;

// if paddr is not find out, not allow in
//assign to_mq_ejtag_dseg_en = DEBUG_MODE & ejtag_dseg_dreq;
wire op_to_ld_q = (op_load&d_cached | 
                  ~d_cached&(op_load|op_store)&ejtag_dreq_valid | 
                  op_prefetch | op_cp0 | d_ex 
                  ); 
assign ld_queue_allowin = op_to_ld_q & (~load_queue_full_i);  

wire op_to_st_q = (~d_cached&(op_load|op_store)&~ejtag_dreq_valid | 
                  op_store&d_cached) & ~d_ex;
assign st_queue_allowin = op_to_st_q & (~store_queue_full_i);

assign normal_allowin  = (~miss_req_queue_full_i) & (ld_queue_allowin | st_queue_allowin);

assign memqueue_allowin = normal_allowin | op_sync | op_cache;

assign mmbuf_to_invalid = (~mmbuf_invalid & mmbuf_brbus_cancel) |      //valid/read_again -> invalid
                          (to_mq_valid & memqueue_allowin & (~valid_addrout | d_ex | to_mq_ex_ddblimpr | to_mq_ex_ddblimpr_h)); //valid->invalid 

assign mmbuf_to_valid = (mmbuf_invalid & ~ex_memqueue_i & ~has_dbs_bs_set & valid_addrout) |     //invalid->valid
                        (to_mq_valid & memqueue_allowin & ~d_ex & ~to_mq_ex_ddblimpr & ~to_mq_ex_ddblimpr_h & valid_addrout) |   //valid->valid 
                        (mmbuf_read_again & ~mmbuf_brbus_cancel & (valid_tlb_rag&~conflict_to_tlb_i | d_ex));  //read_again->valid

assign mmbuf_to_read_again = mmbuf_valid & ~mmbuf_brbus_cancel & 
                            ((conflict_to_tlb_i) |
                             (ram_no_hold & (~normal_allowin & paddr_ready) & ~op_cp0 & ~(op_index_cache|op_icache) & ~op_sync 
                             )
                            ) 
                           ;



wire in_en_state_mmbuf = mmbuf_to_invalid | mmbuf_to_valid | mmbuf_to_read_again;


assign mmbuf_brbus_cancel = brbus_brmask[mmbuf_brqid_r];
assign addr_brbus_cancel  = brbus_brmask[brqid_addrout];

always @(posedge clock)
begin
    if (reset | commitbus_ex)
    begin
        mmbuf_state_r <= MMBUF_INVALID;
        mmbuf_rd_tag_r  <= 1'b0;
        mmbuf_rd_data_r <= 1'b0;
        mmbuf_op_r      <= 8'b0;
        //mmbuf_qid_r     <= 4'b0;
        //mmbuf_vaddr_r   <= 32'b0;
        //mmbuf_value_h_r <= 32'b0;
        //mmbuf_value_r   <= 32'b0;
    end
    else
    begin
        if (in_en_state_mmbuf)
            mmbuf_state_r <= (mmbuf_to_invalid) ? MMBUF_INVALID :
                             (mmbuf_to_valid) ? MMBUF_VALID : MMBUF_READ_AGAIN; //mmbuf_to_read_again

        if (tlb_allowin_o & valid_addrout)
        begin
           mmbuf_rd_tag_r    <= rd_tag_addrout;
           mmbuf_rd_data_r   <= rd_data_addrout;
           mmbuf_op_r        <= op_addrout;
           mmbuf_qid_r       <= qid_addrout;
           mmbuf_brqid_r     <= brqid_addrout;
           mmbuf_vaddr_r     <= vaddr_addrout;
           mmbuf_value_h_r   <= value_h_addrout;
           mmbuf_value_r     <= value_addrout;
        end
    end
end

always @(posedge clock)
begin
    if (reset | commitbus_ex)
    begin
        new_enter_r <= 1'b0;
        addr_trans_ok_r <= 1'b0;
        //select_index_r <= 3'b0;
        tlb_find_ok_r<= 1'b0; //for particular prefetch whose TLB translation is failed but "dc" valued is wrong. 
    end
    else
    begin
        new_enter_r <= tlb_allowin_o & valid_addrout;

        if (dtlb_find & tlb_allowin_o & valid_addrout & ~dtlb_flush_valid)
            addr_trans_ok_r <= 1'b1;
        else if (mmbuf_brbus_cancel | (to_mq_valid & memqueue_allowin))
            addr_trans_ok_r <= 1'b0;
        else if (dtlb_req_tlb_ok_r & ~mmbuf_invalid)
            addr_trans_ok_r <= 1'b1;
        
        if (dtlb_find & tlb_allowin_o & valid_addrout & ~dtlb_flush_valid)
            tlb_find_ok_r <= 1'b1;
        else if (mmbuf_brbus_cancel | (to_mq_valid & memqueue_allowin))
            tlb_find_ok_r <= 1'b0;
        else if (dtlb_wren)
            tlb_find_ok_r <= 1'b1;
        
        //if (tlb_allowin_o & valid_addrout)
        //    select_index_r  <= dtlb_find_index;
        //else if (dtlb_wren)
        //    select_index_r  <= dtlb_wr_index;
    end
end

wire select_index_en_1 = tlb_allowin_o & valid_addrout;
wire select_index_en_2 = ~select_index_en_1 & dtlb_wren;
wire select_index_en_3 = ~select_index_en_1 & ~select_index_en_2;

wire [2:0] select_index_in_value = {3{~(reset|commitbus_ex)}} & 
                                   ({3{select_index_en_1}}&dtlb_find_index |
                                    {3{select_index_en_2}}&dtlb_wr_index   |
                                    {3{select_index_en_3}}&select_index_r);
always @(posedge clock)
begin
    select_index_r <= select_index_in_value;
end

assign tlb_allowin_o = (mmbuf_invalid & ~ex_memqueue_i & ~has_dbs_bs_set) |
                       (to_mq_valid & memqueue_allowin & ~d_ex & ~to_mq_ex_ddblimpr & ~to_mq_ex_ddblimpr_h);

assign tlb_stall_o = ~mmbuf_invalid & op_dcache   |
                     (inst_cache_at_addr_i|inst_sync_at_addr_i) & ~mmbuf_invalid;

assign tlb_has_ll_o = ~mmbuf_invalid & op_ll;

wire normal_to_mq_valid;
wire normal_to_mq_valid_0;

//// tlb_to_memqueue_o bus
reg cache28_ok_r;
always @(posedge clock)
begin
    if (reset | commitbus_ex | mmbuf_valid&op_cache28&cache28_ok_r)
        cache28_ok_r <= 1'b0;
    else if (mmbuf_valid & op_cache28 & (cache28_refill_ok_i | ~d_cached&paddr_ready))
        cache28_ok_r <= 1'b1;
end

reg cache16_28_r;
always @(posedge clock)
begin
    if (reset | commitbus_ex | normal_to_mq_valid&(op_cache16|op_cache28)&cache16_28_r)
        cache16_28_r <= 1'b0;
    else if (mmbuf_valid & ~cache16_28_r & (op_cache16 | op_cache28))
        cache16_28_r <= 1'b1;
end

assign normal_to_mq_valid_0 = mmbuf_valid  &
                           (op_index_cache | 
                            op_sync        |
                            (op_cp0&~op_tlbr) | 
                            (op_tlbr&tlbr_rdy_r) | 
                            paddr_ready&(((op_load|op_store)& ~ejtag_dreq_valid) | 
                                         (op_cache&~op_index_cache&~op_cache28&~op_cache16) | 
                                         ((op_cache28&cache28_ok_r | op_cache16) & cache16_28_r) |
                                         op_prefetch));

assign normal_to_mq_valid = normal_to_mq_valid_0;

assign to_mq_valid = normal_to_mq_valid | ejtag_valid;

assign to_mq_ex = d_ex;
assign to_mq_op_load = op_load;
assign to_mq_op_store = op_store;
assign to_mq_cached = d_cached; 
assign to_mq_uncache_acc = d_uncache_acc;
assign to_mq_hit = hit_dcache     | ~d_cached | op_cp0 | 
                   op_index_cache | op_icache | op_sync;
assign to_mq_hit_set = (op_cache1|op_cache5|op_cache9) ? mmbuf_vaddr[13:12] : hit_set_dcache;

assign to_mq_qid = mmbuf_qid_r;
assign to_mq_brqid = mmbuf_brqid_r;
assign to_mq_op = ((op_prefetch & (~tlb_find_ok_r|~d_cached)) |  
                   (op_dcache & ~op_index_dcache & ~d_cached)) ? `OP_MTC0 : mmbuf_op; //in memqueue MTC0 means do nothing
assign to_mq_paddr = dcache_paddr;
assign to_mq_fpqid = 3'b0;
assign to_mq_cond_true = 1'b1;
assign to_mq_value_h = (op_lwl|op_lwr) ? mmbuf_value :
                       (ejtag_valid)   ? ejtag_value_h : mmbuf_value_h;


wire [31:0] to_mq_value_sel0;
mux32b_7_1 u0_mux32b_7_1(.in1 (mfc_regi),
                         .in2 (erl ? cr_errorpc : cr_epc),
                         .in3 (cr_depc),
                         .in4 ({5'h00, cr_taglo}),
                         .in5 (mmbuf_vaddr),
                         .in6 (mmbuf_value),
                         .in7 (op_sc ? {31'h0, cr_llbit} : ejtag_value),
                         .sel1(op_mfc0 | op_di | op_ei),
                         .sel2(op_eret),
                         .sel3(op_deret),
                         .sel4(op_cache9),
                         .sel5((op_load|op_store|op_cache)&d_ex & ~ejtag_valid),
                         .sel6(op_store&~d_ex&~ejtag_valid),
                         .sel7(ejtag_valid & op_read_tag),
                         .out (to_mq_value_sel0));

assign to_mq_value = to_mq_value_sel0;

assign to_mq_lock = lock_dcache;

assign to_mq_ex_ddbl     = d_ex_ddbl;
assign to_mq_ex_ddbs     = d_ex_ddbs;
assign to_mq_ex_ddblimpr = d_ex_ddblimpr & ~d_ex_adel;
assign to_mq_ex_ddblimpr_h = d_ex_ddblimpr_h & ~d_ex_adel;
assign to_mq_ex_ddbsimpr = d_ex_ddbsimpr;
assign to_mq_ex_cacheerr = d_ex_cacheerr;
assign to_mq_ex_mcheck   = d_ex_mcheck;
assign to_mq_ex_watch    = d_ex_watch;
assign to_mq_ex_ades     = ejtag_valid ? ejtag_ades: d_ex_ades;
assign to_mq_ex_adel     = ejtag_valid ? ejtag_adel: d_ex_adel;
assign to_mq_ex_tlbsr    = d_ex_tlbsr_r;
assign to_mq_ex_tlbsi    = d_ex_tlbsi_r;
assign to_mq_ex_tlblr    = d_ex_tlblr_r;
assign to_mq_ex_tlbli    = d_ex_tlbli_r;
assign to_mq_ex_mod      = d_ex_mod;
assign to_mq_ex_ri       = d_ex_ri;


//// tlb_forward_bus_o bus
assign fpq_forward = 1'b0;
assign fpqid_forward = 3'b0;
assign valid_forward = (normal_to_mq_valid & normal_allowin & op_load & d_cached) & hit_dcache;
assign qid_forward   = mmbuf_qid_r;


//------------------------ TLB -----------------------------------

assign itlb_req_tlb_ack = ~(mmbuf_valid & op_tlbp) & ~(mmbuf_valid & op_tlbr) & ~tlb_data_lookup_en & ~tlb_cam_wen;

////// arbitration for ITLB
always @(posedge clock)
begin
    if (reset)
    begin
        itlb_req_tlb_ok_r <= 1'b0;
    end
    else
    begin
        itlb_req_tlb_ok_r <= tlb_instr_lookup_en & itlb_req_tlb_ack;
    end
end

// tlb_to_itlb_o bus
assign tlb_valid_to_itlb    = itlb_req_tlb_ack;
assign tlb_find_to_itlb     = itlb_req_tlb_ok_r & tlb_find_r;
assign asid_to_itlb         = asid;
assign random_index_to_itlb = rand_num_i[1:0]; //cr_random[1:0];
assign tlb_pagemask_to_itlb = {8'b0, tlb_pagemask_r};
assign tlb_asid_to_itlb     = tlb_asid_r;
assign tlb_g_to_itlb        = tlb_g_r;


// tlb_to_icache_o bus
assign vaddr_cache28_icache = ram_no_hold&mmbuf_valid&op_cache28 | mmbuf_valid&op_cache28;
assign vaddr_cache16_icache = ram_no_hold&mmbuf_valid&op_cache16 | mmbuf_valid&op_cache16;
assign vaddr_valid_icache = ram_no_hold&mmbuf_valid | 
                            (mmbuf_valid & (op_cache16|op_cache28) & (~paddr_ready | ~cache16_28_r) & ~cache28_ok_r);
//assign vaddr_index_icache = ram_no_hold&mmbuf_valid&(op_cache28|op_cache16) ? mmbuf_vaddr[11:5] : vaddr_addrout[11:5];
assign vaddr_index_icache = mmbuf_vaddr[11:5];
assign taglow_icache = cr_taglo;
assign valid_icache = mmbuf_valid & (op_index_icache | op_icache&paddr_ready&cache16_28_r&d_cached&~cache28_ok_r);
assign paddr_icache = dcache_paddr;
assign cache28_icache = op_cache28;
assign cache16_icache = op_cache16;
assign cache8_icache  = op_cache8;
assign cache0_icache  = op_cache0;
assign set_icache     = mmbuf_vaddr[13:12];

//// itlb_flush_o bus
assign itlb_flush_valid = tlb_cam_wen;

////// DTLB 
assign dtlb_asid = asid;  
assign data_vpn  = vaddr_addrout[31:12];
assign dtlb_wren = ~mmbuf_invalid & dtlb_req_tlb_ok_r & tlb_find_r & dv; 
assign dtlb_wr_index = rand_num_i; //cr_random[2:0];
assign dtlb_wr_vpn = mmbuf_vaddr[31:12]&({4'b1111, ~tlb_pagemask_r_value});
assign dtlb_wr_mask = tlb_pagemask_r; 
assign dtlb_wr_asid = tlb_asid_r;
assign dtlb_wr_g = tlb_g_r;
assign dtlb_wr_pfn = doddpage ? pfn1_from_ram : pfn0_from_ram;

wire [31:0] doddpage_1 = {3'h0, tlb_pagemask_r_value, 13'h1fff};
wire [31:0] doddpage_2 = doddpage_1&(~{1'b0, doddpage_1[31:1]});
assign doddpage = |(doddpage_2&mmbuf_vaddr);

assign dtlb_flush_valid = tlb_cam_wen | reset;

micro_dtlb u_micro_dtlb(
                     .clock(clock),
                     .asid(dtlb_asid),
                     .data_vpn(data_vpn),
                     
                     .wren(dtlb_wren),
                     .wr_index(dtlb_wr_index),
                     .wr_vpn(dtlb_wr_vpn),
                     .wr_mask(dtlb_wr_mask),
                     .wr_asid(dtlb_wr_asid),
                     .wr_g(dtlb_wr_g),
                     .wr_pfn(dtlb_wr_pfn),
                     
                     .dtlb_flush_i(dtlb_flush_valid),
                     
                     .d_find(dtlb_find),
                     .d_find_index(dtlb_find_index),
                     
                     .select_index(select_index_r),
                     .dpagemask_o(dtlb_pagemask),
                     .out_pfn(dtlb_pfn)
                    );


wire   d_valid = ~mmbuf_invalid & (~ejtag_dreq_valid); 

wire [15:0] dmask_tmp = 
                  {dtlb_pagemask[7], dtlb_pagemask[7], dtlb_pagemask[6], dtlb_pagemask[6],
                   dtlb_pagemask[5], dtlb_pagemask[5], dtlb_pagemask[4], dtlb_pagemask[4],
                   dtlb_pagemask[3], dtlb_pagemask[3], dtlb_pagemask[2], dtlb_pagemask[2],
                   dtlb_pagemask[1], dtlb_pagemask[1], dtlb_pagemask[0], dtlb_pagemask[0]};

assign dmask= {4'h0, dmask_tmp, 12'hfff};
assign dpfn = {4'b0, dtlb_pfn[25:6]};
assign dc   = dtlb_pfn[5:3];
assign dne  = dtlb_pfn[30];

assign dv   = doddpage  ? v1 : v0; 
assign dd   = dtlb_pfn[2]; 

wire [2:0] dseg = mmbuf_vaddr[31:29];
wire d_kseg3    = dseg[2]&dseg[1]&dseg[0];       //3'b111
wire d_ksseg    = dseg[2]&dseg[1]&(~dseg[0]);    //3'b110
wire d_kseg1    = dseg[2]&(~dseg[1])&dseg[0];    //3'b101
wire d_kseg0    = dseg[2]&(~dseg[1])&(~dseg[0]); //3'b100
wire d_useg     = ~dseg[2];                      //3'b0xx

assign d_map    = ~ejtag_dreq_valid&d_kseg3 | d_ksseg | (d_useg&(~erl));
assign d_cached = d_map&(dc==3'b011)&tlb_find_ok_r | d_kseg0&(config_k0==3'b011);
assign d_uncache_acc = d_map&(dc==3'b111)&tlb_find_ok_r | d_kseg0&(config_k0==3'b111);


////// the paddr which used by dcache to select the right way
wire d_addr_align_err = ((op_ld_hw_align|op_st_hw_align)&mmbuf_vaddr[0] |
                         (op_ld_w_align |op_st_w_align) &(mmbuf_vaddr[1:0]!=2'b00) |
                         (op_ld_dw_align|op_st_dw_align)&(mmbuf_vaddr[2:0]!=3'b000)
                        );
wire d_ex_ade = d_addr_align_err             |
                (mode_user & (~d_useg))      |
                (mode_super & (d_kseg0|d_kseg1|d_kseg3));
assign d_ex_adel  = d_valid&d_ex_ade&(op_load|op_cache);
assign d_ex_ades  = d_valid&d_ex_ade&op_store;

always @(posedge clock)
begin
    if (reset | commitbus_ex | (to_mq_valid & memqueue_allowin) | mmbuf_brbus_cancel)
    begin
        d_ex_tlblr_r <= 1'b0;
        d_ex_tlbsr_r <= 1'b0;
        d_ex_tlbli_r <= 1'b0;
        d_ex_tlbsi_r <= 1'b0;
    end
    else if (~mmbuf_invalid & dtlb_req_tlb_ok_r)
    begin
        if (op_load | (op_cache&~op_index_cache)) 
        begin
            d_ex_tlblr_r <= ~tlb_find_r;
            d_ex_tlbli_r <= tlb_find_r & ~dv;
        end

        if (op_store)
        begin
            d_ex_tlbsr_r <= ~tlb_find_r;
            d_ex_tlbsi_r <= tlb_find_r & ~dv;
        end
    end
end

assign d_ex_mod = ~mmbuf_invalid & op_store & tlb_find_ok_r & ~dd;

assign d_ex_watch = 1'b0;  // not implement 
assign d_ex_mcheck  = 1'b0; // not implement
assign d_ex_cacheerr= 1'b0;
assign d_ex_ddbl  = d_valid&HB_DDBL; 
assign d_ex_ddbs  = d_valid&HB_DDBS;
assign d_ex_ddblimpr = d_valid & op_load & HB_LOAD_ADDR_MATCH[0];  // doesn't influence d_ex
assign d_ex_ddblimpr_h = d_valid & op_ldc1 & HB_LOAD_ADDR_MATCH[1];  // doesn't influence d_ex
assign d_ex_ddbsimpr = d_valid & HB_DDBSIMPR;
assign d_ex_ri = (access_drseg&~op_word) | (access_dcr&~op_word) | (access_dmseg&op_cp1_dw); 
assign d_ex       = d_ex_adel | d_ex_ades | 
                    d_ex_ddbl | d_ex_ddbs | d_ex_ddbsimpr |
                    d_ex_tlb  | d_ex_ri;
assign d_ex_tlb   = d_ex_tlblr_r | d_ex_tlbsr_r | d_ex_tlbli_r | d_ex_tlbsi_r | d_ex_mod;

wire [19:0] d_paddr_unmap;
wire [19:0] d_paddr_map;

assign d_paddr_unmap = {3'b0, mmbuf_vaddr[28:12]};
assign d_paddr_map   = dpfn[19:0] | (mmbuf_vaddr[31:12]&dmask[31:12]);

assign dcache_paddr  = {(d_map ? d_paddr_map : d_paddr_unmap), mmbuf_vaddr[11:0]};

////// dcachepaddr_o output
assign dcachepaddr_o = dcache_paddr;

////// tlb_read_again_o bus
wire cache_ld_st_cango = (~miss_req_queue_full_i) & ~d_ex & d_cached &
                         (op_load&(~load_queue_full_i) | op_store&(~store_queue_full_i));

assign valid_tlb_rag   = (mmbuf_valid & addr_trans_ok_r & d_map & ram_no_hold) |
                         (mmbuf_read_again & (~ram_no_hold|cache_ld_st_cango));

assign rd_tag_tlb_rag  = op_read_tag;
assign rd_data_tlb_rag = op_read_data;
assign op_tlb_rag      = mmbuf_op_r;
assign laddr_tlb_rag   = mmbuf_vaddr[11:0];


////// TLB CAM
assign tlb_data_lookup_en  = new_enter_r & ~addr_trans_ok_r & d_map & (op_read_tag| op_icache&~op_index_cache); //dtlb miss request
assign tlb_instr_lookup_en = itlb_lookup_req;
assign tlb_vpn2_lookup = op_tlbp ? cr_entryhi[26:8]              :
                         tlb_data_lookup_en ? mmbuf_vaddr[31:13] : //only dtlb miss request, no TLBP
                         tlb_instr_lookup_en ? itlb_lookup_pc[31:13] : 19'b0;
assign tlb_asid_lookup = (op_tlbp|tlb_data_lookup_en|tlb_instr_lookup_en) ? asid : 8'b0;//entryhi_asid

assign tlb_cam_wen = mmbuf_valid & (op_tlbwi | op_tlbwr) & new_enter_r;
assign tlb_cam_windex = op_tlbwr ? cr_random[5:0] : cr_index[5:0];
assign tlbw_vpn2 = cr_entryhi[26:8]&({3'b111, ~cr_pagemask_value_out});
assign tlbw_pagemask = cr_pagemask[7:0];
assign tlbw_asid = asid; 
assign tlbw_g = g;

assign tlbr_valid = mmbuf_valid&op_tlbr;
assign tlb_cam_rindex = cr_index[5:0];


tlb_cam u_tlb_cam(
                .clock(clock),
                .vpn2_lookup_i(tlb_vpn2_lookup),
                .asid_lookup_i(tlb_asid_lookup),
                .find_o(tlb_find),
                .index_out_o(tlb_index_out),
                .pagemask_out_o(tlb_pagemask_out),
                .asid_out_o(tlb_asid_out),
                .g_out_o(tlb_g_out),
                .vpn2_out_o(tlb_vpn2_out),
                
                .wen_i(tlb_cam_wen),
                .windex_i(tlb_cam_windex),
                .w_vpn2_i(tlbw_vpn2),
                .w_pagemask_i(tlbw_pagemask),
                .w_asid_i(tlbw_asid),
                .w_g_i(tlbw_g),

                .tlbr_valid_i(tlbr_valid),
                .rindex_i(tlb_cam_rindex)
              );

assign tlbr_vpn2     = tlb_vpn2_out;
assign tlbr_pagemask = tlb_pagemask_out;
assign tlbr_asid     = tlb_asid_out;
assign tlbr_g        = tlb_g_out;

always @(posedge clock)
begin
    if (reset)
    begin
        tlb_find_r <= 1'b0;
        tlb_pagemask_r <= 8'b0;
        tlb_asid_r <= 8'b0;
        tlb_g_r <= 1'b0;
    end
    else
    begin
        tlb_find_r <= tlb_find;

        if (op_tlbp | tlb_data_lookup_en | tlb_instr_lookup_en)
        begin
            tlb_pagemask_r <= tlb_pagemask_out;
            tlb_asid_r <= tlb_asid_out; 
            tlb_g_r <= tlb_g_out;
        end
    end
end

assign tlb_pagemask_r_value = 
                  {tlb_pagemask_r[7], tlb_pagemask_r[7], tlb_pagemask_r[6], tlb_pagemask_r[6],
                   tlb_pagemask_r[5], tlb_pagemask_r[5], tlb_pagemask_r[4], tlb_pagemask_r[4],
                   tlb_pagemask_r[3], tlb_pagemask_r[3], tlb_pagemask_r[2], tlb_pagemask_r[2],
                   tlb_pagemask_r[1], tlb_pagemask_r[1], tlb_pagemask_r[0], tlb_pagemask_r[0]};

always @(posedge clock)
begin
    if (reset | commitbus_ex)
    begin
        dtlb_req_tlb_ok_r <= 1'b0;
    end
    else 
    begin
        dtlb_req_tlb_ok_r <= tlb_data_lookup_en;
    end
end


always @(posedge clock)
begin
    if (reset|commitbus_ex)
        tlbr_rdy_r <= 1'b0;
    else if (tlbr_valid & ~tlbr_rdy_r)
        tlbr_rdy_r <= 1'b1;
    else if ((to_mq_valid & memqueue_allowin & op_tlbr) | mmbuf_brbus_cancel)
        tlbr_rdy_r <= 1'b0;
end

////// TLB RAM 
// tlb_to_ram_o bus
assign tlb_ram_cen = tlb_ram_wen | tlbr_valid |
                    (tlb_data_lookup_en|tlb_instr_lookup_en)&tlb_find;
assign tlb_ram_rindex = (tlbr_valid) ? cr_index[5:0] : tlb_index_out;
assign tlb_ram_wen = tlb_cam_wen;
assign tlb_ram_windex = tlb_cam_windex;
assign tlb_ram_wdata = {{cr_entrylo1[26], (cr_entrylo1[25:6]&{4'hf, ~cr_pagemask_value_out}), cr_entrylo1[5:1]}, 
                        {cr_entrylo0[26], (cr_entrylo0[25:6]&{4'hf, ~cr_pagemask_value_out}), cr_entrylo0[5:1]}};



//----------------------------CP0 registers---------------------
////// CP0 control register outputs
wire config_ar     = cr_config_init[12:10]==3'b001 ? 1'b0 : 1'b1 ;         //if R2 return 0  else return 1
wire[7:0]cause_int = cr_cause[15:8] & cr_status[15:8];

assign int_out        = (|cause_int) & 
                        cr_status[0] & (~exl) & (~erl);
assign status_exl_o   = cr_status[1];
assign status_erl_o   = cr_status[2];
assign status_bev_o   = cr_status[22];
assign status_ksu_o   = cr_status[4:3];
assign HWREna         = cr_HWREna;
assign vector_int[4:0] = config_ar ? 5'b0: cr_intctrl[4:0];  //intctrl_vs
assign vector_int[7:5] = config_ar ? 3'b0:                   //intvec_no
                         cause_int[7] ? 3'b111 :
                         cause_int[6] ? 3'b110 :
                         cause_int[5] ? 3'b101 :
                         cause_int[4] ? 3'b100 :
                         cause_int[3] ? 3'b011 :
                         cause_int[2] ? 3'b010 :
                         cause_int[1] ? 3'b001 : 3'b000;
assign vector_int[8] = cr_cause[23];
assign ebase = {cr_ebase_init[31:30], cr_ebase[17:0], cr_ebase_init[11:0]};

assign status_cu_o    = cr_status[31:28];

assign entryhi_vpn2_o = cr_entryhi[26:8];

assign exl   = status_exl_o;
assign erl   = status_erl_o;
assign ksu   = status_ksu_o;
assign config_k0   = cr_config[2:0];
assign mode_user   = ksu[1]&(~ksu[0])&(~exl)&(~erl)&(~DEBUG_MODE);
assign mode_super  =(~ksu[1])&ksu[0]&(~exl)&(~erl)&(~DEBUG_MODE);
assign mode_kernel =((~ksu[1])&(~ksu[0]))|exl|erl|DEBUG_MODE;
assign asid = cr_entryhi[7:0];
assign g    = cr_entrylo0[0]&cr_entrylo1[0];

// cp0 reg number is in vl(vk=0 at the same time), so vl=vk+vl=vaddr=cp0 reg No.
wire [ 7:0] regi  =  mmbuf_vaddr_r[7:0];
wire [ 7:0] regi_rd= (op_mfc0|op_di|op_ei) ? mmbuf_vaddr_r[7:0] : 8'b00000000;
wire mfc_index    = (regi_rd==8'b00000000)&(mmbuf_vaddr_r[8]== 1'b0);
wire mfc_random   = (regi_rd==8'b00001000);
wire mfc_entrylo0 = (regi_rd==8'b00010000);
wire mfc_entrylo1 = (regi_rd==8'b00011000);
wire mfc_context  = (regi_rd==8'b00100000);
wire mfc_pagemask = (regi_rd==8'b00101000);
wire mfc_wired    = (regi_rd==8'b00110000);
wire mfc_HWREna   = (regi_rd==8'b00111000);
wire mfc_badvaddr = (regi_rd==8'b01000000);
wire mfc_count    = (regi_rd==8'b01001000)&(mmbuf_vaddr_r[8]== 1'b0);
wire mfc_entryhi  = (regi_rd==8'b01010000);
wire mfc_compare  = (regi_rd==8'b01011000);
wire mfc_ejtag_v  = (regi_rd==8'b10110110);
wire mfc_status   = (regi_rd==8'b01100000);
wire mfc_intctrl  = (regi_rd==8'b01100001);
wire mfc_srsctrl  = (regi_rd==8'b01100010);
wire mfc_srsmap   = (regi_rd==8'b01100011);
wire mfc_cause    = (regi_rd==8'b01101000);
wire mfc_epc      = (regi_rd==8'b01110000);
wire mfc_prid     = (regi_rd==8'b01111000);
wire mfc_ebase    = (regi_rd==8'b01111001)&(mmbuf_vaddr_r[8]== 1'b0);
wire mfc_config   = (regi_rd==8'b10000000);
wire mfc_config1  = (regi_rd==8'b10000001);
wire mfc_config2  = (regi_rd==8'b10000010);
wire mfc_config3  = (regi_rd==8'b10000011);
wire mfc_config6  = (regi_rd==8'b10000110);
wire mfc_config7  = (regi_rd==8'b10000111);
wire mfc_lladdr   = (regi_rd==8'b10001000);
wire mfc_watchlo  = (regi_rd==8'b10010000);
wire mfc_watchhi  = (regi_rd==8'b10011000);
//wire mfc_protmask = (regi_rd==8'b10101000);
//wire mfc_protaddr = (regi_rd==8'b10110000);
wire mfc_filter_en   = (regi_rd==8'b10110111);
wire mfc_filter_addr = (regi_rd==8'b10110101);
wire mfc_filter_data = (regi_rd==8'b10110100);
wire mfc_taglo    = (regi_rd==8'b11100000);
wire mfc_taghi    = (regi_rd==8'b11101000);
wire mfc_errorpc  = (regi_rd==8'b11110000);
wire mfc_debug    = (regi_rd==8'b10111000);
wire mfc_depc     = (regi_rd==8'b11000000);
wire mfc_desave   = (regi_rd==8'b11111000);
wire mfc_perf_control0= (regi_rd==8'b11001000);
wire mfc_perf_count0  = (regi_rd==8'b11001001);
wire mfc_perf_control1= (regi_rd==8'b11001010);
wire mfc_perf_count1  = (regi_rd==8'b11001011);

wire mfc_HWR0       =(regi_rd==8'b01111001)&(mmbuf_vaddr_r[8]== 1'b1); 
wire mfc_HWR1       =(regi_rd==8'b00000000)&(mmbuf_vaddr_r[8]== 1'b1); 
wire mfc_HWR2       =(regi_rd==8'b01001000)&(mmbuf_vaddr_r[8]== 1'b1); 
wire mfc_HWR3       =(regi_rd==8'b00000001)&(mmbuf_vaddr_r[8]== 1'b1); 


wire [31:0] mfc_index_value    = {32{mfc_index}}&{cr_index[6], 25'b0, cr_index[5:0]};
wire [31:0] mfc_random_value   = {32{mfc_random}}&{26'b0, cr_random};
wire [31:0] mfc_entrylo0_value = {32{mfc_entrylo0}}&cr_entrylo0_value;
wire [31:0] mfc_entrylo1_value = {32{mfc_entrylo1}}&cr_entrylo1_value;
wire [31:0] mfc_context_value  = {32{mfc_context}}&{cr_context, 4'b0};
wire [31:0] mfc_pagemask_value = {32{mfc_pagemask}}&{3'b000, cr_pagemask_value_out, 13'b0};
wire [31:0] mfc_wired_value    = {32{mfc_wired}}&{26'b0, cr_wired};
wire [31:0] mfc_HWREna_value   = {32{mfc_HWREna}}&{28'b0, cr_HWREna};
wire [31:0] mfc_badvaddr_value = {32{mfc_badvaddr}}&cr_badvaddr;
wire [31:0] mfc_count_value    = {32{mfc_count}}&cr_count;
wire [31:0] mfc_entryhi_value  = {32{mfc_entryhi}}&{cr_entryhi[26:8], 5'b0, cr_entryhi[7:0]};
wire [31:0] mfc_compare_value  = {32{mfc_compare}}&cr_compare;
wire [31:0] mfc_ejtag_value    = {32{mfc_ejtag_v}}&cr_ejtag_value;
wire [31:0] mfc_status_value   = {32{mfc_status}}&cr_status;
wire [31:0] mfc_intctrl_value  = {32{mfc_intctrl}}&{cr_intctrl_init[31:10], cr_intctrl[4:0], cr_intctrl_init[4:0]};
wire [31:0] mfc_srsctrl_value  = {32{mfc_srsctrl}}&`INIT_SRSCTRL;
wire [31:0] mfc_srsmap_value   = {32{mfc_srsmap}}&`INIT_SRSMAP;
wire [31:0] mfc_cause_value    = {32{mfc_cause}}&cr_cause;
wire [31:0] mfc_epc_value      = {32{mfc_epc}}&cr_epc;
wire [31:0] mfc_prid_value     = {32{mfc_prid}}&`INIT_PRID/*cr_prid*/;
wire [31:0] mfc_ebase_value    = {32{mfc_ebase}}& ebase;
wire [31:0] mfc_config_value   = {32{mfc_config}}&{cr_config_init[31:3], cr_config[2:0]};
wire [31:0] mfc_config1_value  = {32{mfc_config1}}&`INIT_CONFIG1/*cr_config1*/;
wire [31:0] mfc_config2_value  = {32{mfc_config2}}&`INIT_CONFIG2/*cr_config2*/;
wire [31:0] mfc_config3_value  = {32{mfc_config3}}&`INIT_CONFIG3/*cr_config3*/;
wire [31:0] mfc_config6_value  = {32{mfc_config6}}&{22'b0,cr_config6[9:8],3'b0,cr_config6[4:0]}/*cr_config6*/;
wire [31:0] mfc_config7_value  = {32{mfc_config7}}&{26'b0,cr_config7}; //SPECIAL_CFG
wire [31:0] mfc_filter_en_value   = {32{mfc_filter_en}}&{{(32-`FILTER_WINDOW_NUM){1'b0}}, cr_filter_en};
wire [31:0] mfc_filter_addr_value = {32{mfc_filter_addr}}&{{(32-`FILTER_WINDOW_DEPTH){1'b0}}, cr_filter_addr};
wire [31:0] mfc_filter_data_value = {32{mfc_filter_data}}&{filter_window_data, {`FILTER_MASK_UNIT{1'b0}}};

//wire [31:0] mfc_lladdr_value   = {32{mfc_lladdr}}&cr_lladdr;
//wire [31:0] mfc_watchlo_value  = {32{mfc_watchlo}}&cr_watchlo;
//wire [31:0] mfc_watchhi_value  = {32{mfc_watchhi}}&{1'b0, cr_watchhi[17], 6'b0, 
//                                                    cr_watchhi[16:9],4'b0, 
//                                                    cr_watchhi[8:0],3'b0};
//wire [31:0] mfc_protmask_value = {32{mfc_protmask}}&cr_protmask;
//wire [31:0] mfc_protaddr_value = {32{mfc_protaddr}}&cr_protaddr;
wire [31:0] mfc_taglo_value    = {32{mfc_taglo}}&{cr_taglo[26:1],5'b0,cr_taglo[0]};
wire [31:0] mfc_taghi_value    = {32{mfc_taghi}}&cr_taghi;
wire [31:0] mfc_errorpc_value  = {32{mfc_errorpc}}&cr_errorpc;
wire [31:0] mfc_debug_value    = {32{mfc_debug}}&cr_debug;
wire [31:0] mfc_depc_value     = {32{mfc_depc}}&cr_depc;
wire [31:0] mfc_desave_value   = {32{mfc_desave}}&cr_desave;
wire [31:0] mfc_perf_control_value0 = {32{mfc_perf_control0}}&{1'b1, 20'b0, cr_perf_control0[10:0]};
wire [31:0] mfc_perf_count_value0 = {32{mfc_perf_count0}}&cr_perf_count0;
wire [31:0] mfc_perf_control_value1 = {32{mfc_perf_control1}}&{21'b0, cr_perf_control1[10:0]};
wire [31:0] mfc_perf_count_value1 = {32{mfc_perf_count1}}&cr_perf_count1;


wire [31:0] mfc_HWR0_value = {32{mfc_HWR0}}&{22'b0,cr_ebase_init[9:0]};
wire [31:0] mfc_HWR1_value = {32{mfc_HWR1}}&{31'b0,1'b1};
wire [31:0] mfc_HWR2_value = {32{mfc_HWR2}}&{cr_count};
wire [31:0] mfc_HWR3_value = {32{mfc_HWR3}}&{31'b0,1'b1};

assign mfc_regi = mfc_index_value    | mfc_random_value  |mfc_entrylo0_value|
                  mfc_entrylo1_value | mfc_context_value |mfc_pagemask_value|
                  mfc_wired_value    | mfc_badvaddr_value|mfc_count_value   |
                  mfc_entryhi_value  | mfc_compare_value |mfc_status_value  |
                  mfc_cause_value    | mfc_epc_value     |mfc_prid_value    |
                  mfc_config_value   | mfc_config1_value |mfc_config6_value |
                  mfc_taglo_value    | mfc_taghi_value   |
                  mfc_errorpc_value  | mfc_debug_value   |mfc_depc_value    |
                  mfc_desave_value   | mfc_ejtag_value | 
                  mfc_perf_control_value0 | mfc_perf_count_value0 |
                  mfc_perf_control_value1 | mfc_perf_count_value1 | 
                  mfc_HWR0_value     | mfc_HWR1_value    | mfc_HWR2_value   |
                  mfc_HWR3_value     | mfc_ebase_value   |
                  mfc_config2_value  | mfc_config3_value | mfc_intctrl_value |
                  mfc_HWREna_value   | mfc_srsctrl_value | mfc_srsmap_value
                  |mfc_config7_value
                  |mfc_filter_en_value | mfc_filter_addr_value | mfc_filter_data_value
                  ;


////// commit exception and other events that can modify cp0 reg now
assign excode                = commitbus_excode[4:0];
assign random_wired_eq       = (cr_random== cr_wired);
assign count_compare_eq      = (cr_count==cr_compare);

assign cr_random_sub1        = cr_random - 1'b1; 
assign cr_count_add1         = cr_count + 1'b1 ;

wire [29:0] commitbus_h_sub4 = commitbus_value_h[31:2] - 1'b1 ;
wire [31:0] epc = commitbus_bd ? {commitbus_h_sub4,commitbus_value_h[1:0]}:commitbus_value_h;

wire ex_debug;
wire ex_softreset;
wire ex_ejtagboot;
wire ex_nmi;
wire ex_ddbl;
wire ex_ddbs;
wire ex_ddblimpr;
wire ex_ddbsimpr;
wire ex_dib;
wire ex_dss;
wire ex_dint;
wire ex_sdbbp;
wire ex_ibe;
wire ex_dbe;
wire lsnm;

assign ex_debug=(commitbus_excode[5:3]==3'b110)||ex_ejtagboot||ex_ddbsimpr;
assign ex_softreset=(commitbus_excode[5:0]==`EX_SOFTRESET);
assign ex_nmi=(commitbus_excode[5:0]==`EX_NMI);
assign ex_dint=(commitbus_excode[5:0]==`EX_DINT);
assign ex_dss=(commitbus_excode[5:0]==`EX_DSS);
assign ex_sdbbp=(commitbus_excode[5:0]==`EX_SDBBP);
assign ex_dib=(commitbus_excode[5:0]==`EX_DIB);
assign ex_ddbl=(commitbus_excode[5:0]==`EX_DDBL);
assign ex_ddbs=(commitbus_excode[5:0]==`EX_DDBS);
assign ex_ddblimpr=(commitbus_excode[5:0]==`EX_DDBLIMPR);
assign ex_ddbsimpr=(commitbus_excode[5:0]==`EX_DDBSIMPR);
assign ex_ejtagboot=(commitbus_excode[5:0]==`EX_EJTAGBOOT);
assign ex_ibe=(commitbus_excode[5:0]==`EX_IBE);
assign ex_dbe=(commitbus_excode[5:0]==`EX_DBE);


//// for performance counter
wire cr_pcount_exl0,cr_pcount_k0,cr_pcount_s0,cr_pcount_u0,cr_pcount_ie0;
wire cr_pcount_exl1,cr_pcount_k1,cr_pcount_s1,cr_pcount_u1,cr_pcount_ie1;
wire perf_in_en_0,pcount_ov_0;
wire perf_in_en_1,pcount_ov_1;
wire [4:0 ] cr_pcount_event0;
wire [4:0 ] cr_pcount_event1;
wire [31:0] pcount_event_d0;
wire [31:0] pcount_event_d1;

wire [31:0] perf_count_in0 ;
wire [31:0] perf_count_in1 ;
wire        pcount_inc0;
wire [1:0 ] pcount_inc1;
wire        pcount_inc0_gate;
wire [1:0 ] pcount_inc1_gate;

assign cr_pcount_exl0   = cr_perf_control0[0];
assign cr_pcount_k0     = cr_perf_control0[1];
assign cr_pcount_s0     = cr_perf_control0[2];
assign cr_pcount_u0     = cr_perf_control0[3];
assign cr_pcount_ie0    = cr_perf_control0[4];
assign cr_pcount_event0 = cr_perf_control0[9:5];

assign pcount_ov_0      = cr_perf_count0[31];
assign perf_in_en_0     = (cr_pcount_u0 & mode_user) |
                          (cr_pcount_s0 & mode_super)|
                          (cr_pcount_k0 & mode_kernel & ~exl & ~erl & ~DEBUG_MODE) |
                          (cr_pcount_exl0 & exl &~erl & ~DEBUG_MODE);
assign pcount_inc0_gate = perf_in_en_0 ? pcount_inc0 : 1'h0;
assign perf_count_in0   = cr_perf_count0 + pcount_inc0_gate;

assign cr_pcount_exl1   = cr_perf_control1[0];
assign cr_pcount_k1     = cr_perf_control1[1];
assign cr_pcount_s1     = cr_perf_control1[2];
assign cr_pcount_u1     = cr_perf_control1[3];
assign cr_pcount_ie1    = cr_perf_control1[4];
assign cr_pcount_event1 = cr_perf_control1[9:5];

assign pcount_ov_1      = cr_perf_count1[31];
assign perf_in_en_1     = (cr_pcount_u1 & mode_user) |
                          (cr_pcount_s1 & mode_super)|
                          (cr_pcount_k1 & mode_kernel & ~exl & ~erl & ~DEBUG_MODE) |
                          (cr_pcount_exl1 & exl &~erl & ~DEBUG_MODE);
assign pcount_inc1_gate = perf_in_en_1 ? pcount_inc1 : 2'h0;
assign perf_count_in1   = cr_perf_count1 + pcount_inc1_gate;

//event defination
reg exbus_ex_reg; 
reg brbus_brerr_reg; 
reg brbus_valid_reg; 
reg imemread_valid_reg; 
reg qissbus0_valid_reg;
reg qissbus1_valid_reg;
reg dmemread_valid_reg;
reg duncache_valid_reg;
reg mreadreq_valid_reg;
reg mwritereq_valid_reg;
reg data_inst_conflict_reg;
reg data_inter_conflict_reg;
reg not_store_ok_stall_reg;
reg st_ld_conflict_reg;
reg [1:0] commit_ld_st_reg;
reg cmtbus0_valid_reg;
reg cmtbus1_valid_reg;
reg queue_full_reg;
reg brq_full_reg;
reg loadq_full_reg  ; 
reg storeq_full_reg ;    
reg missq_full_reg  ;   
reg miss_req_queue_full_reg;
reg brbus_jr31_reg;
reg brbus_jr31miss_reg;
reg brbus_static_reg;
reg brbus_staticmiss_reg;
reg ex_tlbr_reg;
reg ex_int_reg;
reg brbus_bht_reg;
reg brbus_bhtmiss_reg;
reg icache_access_reg;
reg icache_hit_reg;
reg icache_way_hit_reg;
reg icache_update_reg;
reg dcache_access_reg;
reg dcache_hit_reg;
reg stalled_fetch_reg;
reg dtlb_miss_reg;
reg dtlb_access_reg;
reg itlb_access_reg;

reg flush_pipeline_cycle_reg;
reg inst_to_queue_cycle_reg;
reg insts_to_alu1_reg;
reg insts_to_alu2_reg;
reg insts_to_addr_reg;
reg insts_to_falu_reg;

reg[1:0] insts_to_queue_reg;
reg[1:0] insts_fetched_reg       ;
reg stalled_cycles_icachemiss_reg ;
reg itlbmiss_tlbhit_reg     ;

wire commit0_valid    = commitbus0_valid & ~commitbus0_ex;
wire commit0_ld_st;
LD_ST_OP u0_ld_st_op(.op(commitbus0_op), .ld_st_op(commit0_ld_st));

wire commit1_valid    = commitbus1_valid & ~commitbus1_ex;
wire commit1_ld_st;
LD_ST_OP u1_ld_st_op(.op(commitbus1_op), .ld_st_op(commit1_ld_st));
always @(posedge clock)
begin
        exbus_ex_reg          <= commitbus_ex; 
        brbus_brerr_reg       <= brbus_brerr; 
        brbus_valid_reg       <= brbus_valid; 
        imemread_valid_reg    <= imemread_valid_i ; 
        qissbus0_valid_reg    <= qissuebus_valid0_i;
        cmtbus0_valid_reg     <= commitbus0_valid;
        qissbus1_valid_reg    <= qissuebus_valid1_i;
        cmtbus1_valid_reg     <= commitbus1_valid;
        queue_full_reg        <= queue_full_i;
        brq_full_reg          <= brq_full_i;
        brbus_jr31_reg        <= brbus_valid && brbus_jr31;
        brbus_jr31miss_reg    <= brbus_brerr && brbus_jr31;
        brbus_static_reg      <= brbus_valid && brbus_static_br;
        brbus_staticmiss_reg  <= brbus_brerr && brbus_static_br;
        ex_tlbr_reg           <= commitbus_ex && ((commitbus_excode==`EX_TLBLR)||(commitbus_excode==`EX_TLBSR));
        ex_int_reg            <= commitbus_ex && (commitbus_excode == `EX_INTERRUPT);
        brbus_bht_reg         <= brbus_valid && brbus_bht;
        brbus_bhtmiss_reg     <= brbus_brerr && brbus_bht;
        icache_access_reg     <= icache_access_i;
        icache_hit_reg        <= icache_hit_i;
        icache_way_hit_reg    <= icache_way_hit_i;
        icache_update_reg     <= icache_update_i;
        stalled_fetch_reg     <= stalled_fetch_i;
        itlb_access_reg       <= itlb_access_i;
        
        flush_pipeline_cycle_reg <= flush_pipeline_cycle_i;
        inst_to_queue_cycle_reg <= inst_to_queue_cycle_i;
        insts_to_alu1_reg       <= insts_to_alu1_i;
        insts_to_alu2_reg       <= insts_to_alu2_i;
        insts_to_addr_reg       <= insts_to_addr_i;
        insts_to_falu_reg       <= insts_to_falu_i;
        
        insts_to_queue_reg      <= insts_to_queue_i;
        insts_fetched_reg       <= insts_fetched_i;
        stalled_cycles_icachemiss_reg <= stalled_cycles_icachemiss_i;
        itlbmiss_tlbhit_reg     <= itlbmiss_tlbhit_i;
    if (reset)
    begin
        mreadreq_valid_reg    <= 1'b0;
        duncache_valid_reg    <= 1'b0;
        dmemread_valid_reg    <= 1'b0;
        mwritereq_valid_reg   <= 1'b0;
        data_inst_conflict_reg<= 1'b0;
        data_inter_conflict_reg <= 1'b0;
        dcache_access_reg     <= 1'b0;
        dcache_hit_reg        <= 1'b0;
        missq_full_reg        <= 1'b0;
        loadq_full_reg        <= 1'b0;
        storeq_full_reg       <= 1'b0;
        miss_req_queue_full_reg <= 1'b0;
        not_store_ok_stall_reg<= 1'b0;
        dtlb_access_reg       <= 1'b0;
        dtlb_miss_reg         <= 1'b0;
        st_ld_conflict_reg    <= 1'b0;
        commit_ld_st_reg      <= 2'b0;
    end
    else
    begin
        if (pcount_event_d1[ 4])
            dmemread_valid_reg    <= dmemread_valid_i;
        if (pcount_event_d1[ 7])
            duncache_valid_reg    <= duncache_valid_i ;
        if (pcount_event_d0[ 9])
            mreadreq_valid_reg    <= mreadreq_valid_i;
        if (pcount_event_d1[ 9])
            mwritereq_valid_reg   <= mwritereq_valid_i;
        if (pcount_event_d0[29])
            data_inst_conflict_reg<= data_inst_conflict_i;
        if (pcount_event_d1[29])
            data_inter_conflict_reg <= data_inter_conflict_i;
        if (pcount_event_d0[19])
            loadq_full_reg        <= op_to_ld_q & load_queue_full_i;
        if (pcount_event_d0[20])
            storeq_full_reg       <= op_to_st_q & store_queue_full_i;
        if (pcount_event_d0[21])
            missq_full_reg        <= missq_full_i;
        if (pcount_event_d0[22])
            miss_req_queue_full_reg <= (ld_queue_allowin | st_queue_allowin) & miss_req_queue_full_i;
        if (pcount_event_d0[30])
            not_store_ok_stall_reg<= op_to_st_q & store_queue_full_i & not_store_ok_i;
        if (pcount_event_d0[31])
            st_ld_conflict_reg<= st_ld_conflict_i;
        if (pcount_event_d1[30])
            commit_ld_st_reg     <= (commit0_valid&commit0_ld_st) + (commit1_valid&commit1_ld_st);
 //       if (pcount_event_d1[30])
 //           commit_ld_st_reg     <= (commit0_valid&commit0_ld_st) + (commit1_valid&commit1_ld_st);
        if (pcount_event_d0[17])
            dcache_access_reg     <= normal_to_mq_valid & normal_allowin & to_mq_cached & mmbuf_rd_tag_r;
        if (pcount_event_d1[17])
            dcache_hit_reg        <= normal_to_mq_valid & normal_allowin & to_mq_cached & mmbuf_rd_tag_r & hit_dcache;
        if (pcount_event_d0[23])
            dtlb_access_reg       <= to_mq_valid & memqueue_allowin & d_map & ~d_ex_tlb;
        if (pcount_event_d1[23])
            dtlb_miss_reg         <= dtlb_wren;
    end
end
 

decoder_5_32  decoder0(.in(cr_pcount_event0), .out(pcount_event_d0));
decoder_5_32  decoder1(.in(cr_pcount_event1), .out(pcount_event_d1));

assign pcount_inc0=   (pcount_event_d0[ 0]) |
                      (pcount_event_d0[ 1] & brbus_valid_reg) |
                      (pcount_event_d0[ 3] & brbus_jr31_reg) |
                      (pcount_event_d0[ 4] & imemread_valid_reg) |
                      (pcount_event_d0[ 5] & qissbus0_valid_reg) |
                      (pcount_event_d0[ 6] & qissbus1_valid_reg) |
                      (pcount_event_d0[ 8] & brbus_bht_reg) |
                      (pcount_event_d0[ 9] & mreadreq_valid_reg) |
                      (pcount_event_d0[10] & stalled_fetch_reg)|
                      (pcount_event_d0[11] & queue_full_reg) |
                      (pcount_event_d0[12] & flush_pipeline_cycle_reg)|
                      (pcount_event_d0[13] & ex_tlbr_reg) |
                      (pcount_event_d0[14] & ex_int_reg) |
                      (pcount_event_d0[15] & inst_to_queue_cycle_reg)|
                      (pcount_event_d0[16] & icache_access_reg)|
                      (pcount_event_d0[17] & dcache_access_reg)|
                      (pcount_event_d0[18] & brbus_static_reg)|
                      (pcount_event_d0[19] & loadq_full_reg)|
                      (pcount_event_d0[20] & storeq_full_reg)|
                      (pcount_event_d0[21] & missq_full_reg)|
                      (pcount_event_d0[22] & miss_req_queue_full_reg)|
                      (pcount_event_d0[23] & dtlb_access_reg)|
                      (pcount_event_d0[24] & itlb_access_reg)|
                      (pcount_event_d0[25] & insts_to_alu1_reg)|
                      (pcount_event_d0[26] & insts_to_alu2_reg)|
                      (pcount_event_d0[27] & insts_to_addr_reg)|
                      (pcount_event_d0[28] & insts_to_falu_reg)|
                      (pcount_event_d0[29] & data_inst_conflict_reg)|
                      (pcount_event_d0[30] & not_store_ok_stall_reg)|
                      (pcount_event_d0[31] & st_ld_conflict_reg);



wire [1:0] pc_cmt_inc = cmtbus0_valid_reg + cmtbus1_valid_reg;
wire       has_commit = cmtbus0_valid_reg | cmtbus1_valid_reg;
wire       commit_two = cmtbus0_valid_reg & cmtbus1_valid_reg;

assign pcount_inc1=   ({2{pcount_event_d1[ 0]}} & { pc_cmt_inc }) |
                      ({2{pcount_event_d1[ 1]}} & {1'b0,brbus_brerr_reg}) |
                      ({2{pcount_event_d1[ 2]}} & {2'b01}) |   //clock
                      ({2{pcount_event_d1[ 3]}} & {1'b0,brbus_jr31miss_reg}) |
                      ({2{pcount_event_d1[ 4]}} & {1'b0,dmemread_valid_reg}) |
                      //({2{pcount_event_d1[ 5]}} & {1'b0,dmemread_valid_reg}) |
                      //({2{pcount_event_d1[ 6]}} & {1'b0,dmemread_valid_reg}) |
                      ({2{pcount_event_d1[ 7]}} & {1'b0,duncache_valid_reg}) |
                      ({2{pcount_event_d1[ 8]}} & {1'b0,brbus_bhtmiss_reg}) |
                      ({2{pcount_event_d1[ 9]}} & {1'b0,mwritereq_valid_reg}) |
                      ({2{pcount_event_d1[11]}} & {1'b0,brq_full_reg}) |
                      ({2{pcount_event_d1[13]}} & {1'b0,exbus_ex_reg}) | 
                      ({2{pcount_event_d1[15]}} & {insts_to_queue_reg}) | 
                      ({2{pcount_event_d1[16]}} & {1'b0,icache_hit_reg}) | 
                      ({2{pcount_event_d1[17]}} & {1'b0,dcache_hit_reg}) | 
                      ({2{pcount_event_d1[18]}} & {1'b0,brbus_staticmiss_reg}) | 
                      ({2{pcount_event_d1[19]}} & {1'b0,icache_way_hit_reg}) | 
                      ({2{pcount_event_d1[20]}} & {1'b0,icache_update_reg}) | 
                      ({2{pcount_event_d1[21]}} & {insts_fetched_reg}) | 
                      ({2{pcount_event_d1[22]}} & {1'b0,stalled_cycles_icachemiss_reg}) | 
                      ({2{pcount_event_d1[23]}} & {1'b0,dtlb_miss_reg}) | 
                      ({2{pcount_event_d1[24]}} & {1'b0,itlbmiss_tlbhit_reg}) | 
                      ({2{pcount_event_d1[25]}} & {1'b0,cmtbus0_valid_reg}) | 
                      ({2{pcount_event_d1[26]}} & {1'b0,cmtbus1_valid_reg}) | 
                      ({2{pcount_event_d1[27]}} & {1'b0,has_commit}) | 
                      ({2{pcount_event_d1[28]}} & {1'b0,commit_two}) | 
                      ({2{pcount_event_d1[29]}} & {1'b0, data_inter_conflict_reg}) |
                      ({2{pcount_event_d1[30]}} & commit_ld_st_reg);


//for cr_cause     
wire timer_hw0 =( cr_intctrl_init[31:29] == 3'b010);
wire timer_hw1 =( cr_intctrl_init[31:29] == 3'b011);
wire timer_hw2 =( cr_intctrl_init[31:29] == 3'b100);
wire timer_hw3 =( cr_intctrl_init[31:29] == 3'b101);
wire timer_hw4 =( cr_intctrl_init[31:29] == 3'b110);
wire timer_hw5 =( cr_intctrl_init[31:29] == 3'b111);

wire pc_hw0 =( cr_intctrl_init[28:26] == 3'b010);
wire pc_hw1 =( cr_intctrl_init[28:26] == 3'b011);
wire pc_hw2 =( cr_intctrl_init[28:26] == 3'b100);
wire pc_hw3 =( cr_intctrl_init[28:26] == 3'b101);
wire pc_hw4 =( cr_intctrl_init[28:26] == 3'b110);
wire pc_hw5 =( cr_intctrl_init[28:26] == 3'b111);

wire [5:0] timer_pc = ( ~cr_cause[23] | status_bev_o |(cr_intctrl==5'b000) ) ?  
                  {(cr_cause[30] | cr_cause[26]),5'b00000} :
                 ({timer_hw5&cr_cause[30],timer_hw4&cr_cause[30],timer_hw3&cr_cause[30],
                  timer_hw2&cr_cause[30],timer_hw1&cr_cause[30],timer_hw0&cr_cause[30]} | 
                 {pc_hw5&cr_cause[26],pc_hw4&cr_cause[26],pc_hw3&cr_cause[26],
                  pc_hw2&cr_cause[26],pc_hw1&cr_cause[26],pc_hw0&cr_cause[26]}) ;
                  
wire [5:0] ip  = int_in[5:0] |  timer_pc;

wire cp0_reg_wen = (~miss_req_queue_full_i) & (~load_queue_full_i);

//// writing of cp0 register
always @(posedge clock)
begin
  if (reset)
     begin
       count_add_en <=1'b0;
       cr_count     <=32'b0;
       cr_random    <=6'b011111;
       cr_cause     <=32'b0;
       cr_perf_count0  <=32'h0000000;
       cr_perf_count1  <=32'b0;
     end
  else
     begin
        //performance counter No. 25 sel: 1
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11001001))
            cr_perf_count0 <= mmbuf_value;
         else if(perf_in_en_0)
            cr_perf_count0 <= perf_count_in0;
        
         //performance counter No. 25 sel: 3
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11001011))
            cr_perf_count1 <= mmbuf_value;
         else if(perf_in_en_1)
            cr_perf_count1 <= perf_count_in1;

        //Random No.1 Page62
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00110000))//MTCO Wired Reg
               cr_random<=6'b011111;//TLBsize-1=31
        else if (commitbus_valid&&(~DEBUG_MODE))
               cr_random<=random_wired_eq?6'b011111: cr_random_sub1;

         //Cause  No.13 Page92
         if (commitbus_ex&&(~ex_softreset)&&(~ex_nmi)&&(~ex_debug)&&(~DEBUG_MODE)) 
         begin
             cr_cause[6:2]   <= excode;

             if (~exl) //"The processoe updates BD only if Status_EXL was zero when the exception occured"
             begin
                 cr_cause[31]    <= commitbus_bd;
             end
             
             if (excode==5'b01011)
             begin
                cr_cause[29:28] <= commitbus_ce;
             end
         end
            
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01101000))
         begin
             cr_cause[9:8] <= mmbuf_value[9:8];
             cr_cause[23]  <= mmbuf_value[23];
             cr_cause[27]  <= mmbuf_value[27];
         end

         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01011000)) //write Compare register
               cr_cause[30] <= 1'b0; //clear timer interrupt
         else if( count_compare_eq)
               cr_cause[30] <=1'b1;  //timer interrupt pending
         
         cr_cause[26] <= (pcount_ov_0 & cr_pcount_ie0) | (pcount_ov_1 & cr_pcount_ie1); 
      
         cr_cause[15:10] <=ip; 


        count_add_en<=~count_add_en;
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01001000))
           cr_count<=mmbuf_value;
        else   
           if (~(DEBUG_MODE&&(~cr_debug[25])))
              cr_count<=count_add_en?cr_count_add1:cr_count;
     end
end

assign cr_pagemask_value_out = 
                  {cr_pagemask[7], cr_pagemask[7], cr_pagemask[6], cr_pagemask[6],
                   cr_pagemask[5], cr_pagemask[5], cr_pagemask[4], cr_pagemask[4],
                   cr_pagemask[3], cr_pagemask[3], cr_pagemask[2], cr_pagemask[2],
                   cr_pagemask[1], cr_pagemask[1], cr_pagemask[0], cr_pagemask[0]};

assign cr_pagemask_value_in = {mmbuf_value[27], mmbuf_value[25], mmbuf_value[23], mmbuf_value[21],
                               mmbuf_value[19], mmbuf_value[17], mmbuf_value[15], mmbuf_value[13]};

always @(posedge clock)
begin
  if (reset)
     begin
       cr_index   <=7'b0; //Reset Value UNDEFINED
       cr_entrylo0<=27'b0;
       cr_entrylo1<=27'b0;
       cr_context <=28'b0;
       cr_pagemask<= 8'b0;
       cr_wired   <=6'b0;
       cr_HWREna  <=4'b0;
       cr_badvaddr<=32'b0;
       cr_entryhi <=27'b0;
       cr_compare <=32'b0;
       cr_ejtag_value<=32'b0;
       cr_status  <=`INIT_STATUS;
       cr_epc     <=32'b0;
       cr_ebase   <=cr_ebase_init[29:12];
      // cr_prid    <=`INIT_PRID;
       cr_config  <=cr_config_init[2:0]; 
       cr_config6 <=`INIT_CONFIG6;  
       cr_config7 <=`INIT_CONFIG7; //SPECIAL_CFG
       //cr_lladdr  <=32'b0;
       //cr_watchlo <=32'b0;
       //cr_watchhi <=18'b0;
       //cr_protmask<=32'b0;
       //cr_protaddr<=32'b0;
       cr_taglo   <=27'b0;
       cr_taghi   <=32'b0;
       cr_errorpc <=32'b0;
       cr_intctrl <=cr_intctrl_init[9:5];
       cr_perf_control0 <=11'b0;
       cr_perf_control1 <=11'b0;
       cr_filter_en <= {`FILTER_WINDOW_NUM{1'b0}};
       cr_filter_addr <= {`FILTER_WINDOW_DEPTH{1'b0}};
     end                      
  else
     begin
        //Config
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10000000))
            cr_config[2:0]<=mmbuf_value[2:0];
        
        //Config6, for branch prediction
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10000110))
            cr_config6[4:0]<=mmbuf_value[4:0];
        if (DATA_FROM_TAP[32])
            cr_config6[8] <= 1'b1; 
        else if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10000110)) 
            cr_config6[8]  <= 1'b0;
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10000110))
            cr_config6[9]<=mmbuf_value[9];

        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10000111))
            cr_config7[5:0]<=mmbuf_value[5:0]; //SPECIAL_CFG

        //Index No.0 Page61
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00000000))
           cr_index<={cr_index[6], mmbuf_value[5:0]};
        else 
           if (mmbuf_valid&(~commitbus_ex)&op_tlbp&cp0_reg_wen)
              cr_index<=tlb_find ? {1'b0,tlb_index_out}: {1'b1, cr_index[5:0]};

        //Wired  No.6 Page72
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00110000))
           cr_wired<=mmbuf_value[5:0];


        //HWREna  No.7 Page73
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00111000))
           cr_HWREna<=mmbuf_value[3:0];

       //Compare  No.11 Page78
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01011000))
           cr_compare<=mmbuf_value;


      //for fast ejtag   No.22,select 6 .unwritable, just be writen by tap,by xucp

          if(DATA_FROM_TAP[32])
            cr_ejtag_value<= DATA_FROM_TAP[31:0]; 
      
        //PageMask No.5 Page68
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00101000))
           cr_pagemask<= cr_pagemask_value_in;
        else 
           if (tlbr_rdy_r&(~commitbus_ex)&tlbr_valid)
               cr_pagemask<= tlbr_pagemask; 

        //EntryHi
        if (commitbus_ex&(~DEBUG_MODE)&((excode==5'b1)|(excode==5'b10)|(excode==5'b11)))
           cr_entryhi<={commitbus_value[31:13],cr_entryhi[7:0]};
        else 
           if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01010000))
              cr_entryhi<={mmbuf_value[31:13],mmbuf_value[7:0]};
           else
              if (tlbr_rdy_r&(~commitbus_ex)&tlbr_valid)
                 cr_entryhi<={tlbr_vpn2,tlbr_asid};

         //EntryLo0  No.2 Page63
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00010000))
            cr_entrylo0<={mmbuf_value[30], mmbuf_value[25:0]}; 
            //cr_entrylo0[30] is E bit(1'b1: non-executable,1'b0: executable)
         else 
            if (tlbr_rdy_r&(~commitbus_ex))
               cr_entrylo0<={ram_to_tlb_i[25:0],tlbr_g};

         //EntryLo1 No.3 Page63
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00011000))
            cr_entrylo1<={mmbuf_value[30], mmbuf_value[25:0]}; 
            //cr_entrylo1[30] is E bit(1'b1: non-executable,1'b0: executable)
         else
            if (tlbr_rdy_r&(~commitbus_ex))
               cr_entrylo1<={ram_to_tlb_i[51:26],tlbr_g};
                
         //Status  No.12 Page79
         if (commitbus_ex&&(~DEBUG_MODE))
            begin
               if (commitbus_excode==`EX_SOFTRESET) //change BEV,TS,SR,NMI, set ERL
                  cr_status<={cr_status[31:28],1'b0,cr_status[26:23],4'b1010,cr_status[18:3],1'b1,cr_status[1:0]};
               else if (commitbus_excode==`EX_NMI)
                  cr_status<={cr_status[31:23],4'b1001,cr_status[18:3],1'b1,cr_status[1:0]};
               else  //other exceptions
                  if (((excode[4]==1'b0)|(excode==5'b10111))&(~(excode==5'b01110))) 
                     cr_status<={cr_status[31:2],1'b1,cr_status[0]}; //set EXL
            end
         else
            if (mmbuf_valid&op_mtc0&cp0_reg_wen&(regi==8'b01100000))
            //HAVE_DSP_UNIT
               cr_status<={3'b0, mmbuf_value[28],3'b0,mmbuf_value[24],1'b0,
                           mmbuf_value[22:19],
                           3'b0,mmbuf_value[15:8], 3'b0,mmbuf_value[4:0]};
            else
               if (mmbuf_valid&op_di&cp0_reg_wen)
                cr_status <={cr_status[31:1],1'b0};
            else
               if (mmbuf_valid&op_ei&cp0_reg_wen)
                cr_status <={cr_status[31:1],1'b1};
            else
               if (mmbuf_valid&op_eret&cp0_reg_wen)
                  cr_status<=erl?
                             {cr_status[31:3],1'b0,cr_status[1:0]}: //clear ERL
                             {cr_status[31:2],1'b0,cr_status[0]};   //clear EXL
                
         //BadVAddr No.8 Page74
         if (commitbus_ex&(~DEBUG_MODE)&((excode==5'b1)|(excode==5'b10) //MOD TLBL TLBS ADEL ADES
                         |(excode==5'b11)|(excode==5'b100)|(excode==5'b101)))
            cr_badvaddr<=commitbus_value;

         //Context  No.4 Page67
         if (commitbus_ex&(~DEBUG_MODE)&((excode==5'b1)|(excode==5'b10)|(excode==5'b11)))
            cr_context<={cr_context[27:19],commitbus_value[31:13]};
         else 
            if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b00100000))
               cr_context<={mmbuf_value[31:23],cr_context[18:0]};

         //ErrorEPC No.30 Page130
         if (commitbus_ex&(~DEBUG_MODE)&((commitbus_excode==`EX_NMI)|(commitbus_excode==`EX_SOFTRESET)))//SoftReset, NMI
            cr_errorpc<=epc; //from commitbus
         else 
            if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11110000))
               cr_errorpc<=mmbuf_value;

         //INTCtrl No.12 select1
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01100001))
            cr_intctrl[4:0] <=mmbuf_value[9:5];

         //EPC  No.14 Page97
         if (commitbus_ex&&(~ex_debug)&&
             (~exl)&&(~DEBUG_MODE))
            cr_epc<=epc;
         else
            if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01110000))
               cr_epc<=mmbuf_value;

         //EBase No.15 select1
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b01111001))
            cr_ebase <=mmbuf_value[29:12];

         //LLaddr No.17 Page113 //FIX ME!!!! this register need not to be implemented
         //if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10001000))
         //   cr_lladdr<=mmbuf_value;

         //WatchLo No.18 Page114
         //if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10010000))
         //   cr_watchlo<=mmbuf_value[31:0];

         //WatchHi No.19 Page116
         //if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10011000))
         //   cr_watchhi<={mmbuf_value[30],mmbuf_value[23:16], mmbuf_value[11:3]};

         //ProtMask
         //if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10101000))
         //   cr_protmask<=mmbuf_value;

         //ProtAddr
         //if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10110000))
         //   cr_protaddr<=mmbuf_value;

         //TagLo
         if (mmbuf_valid&(~commitbus_ex)&op_cache5)
            cr_taglo <= {3'b0, taglo_lock, taglo_tag, taglo_cs, 1'b0};
         else if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11100000))
            cr_taglo <= {mmbuf_value[31:6],mmbuf_value[0]}; 

         //TagHi
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11101000))
            cr_taghi <= mmbuf_value;
        
         //performance control counter nu 25 sel: 0 
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11001000))
            cr_perf_control0[10:0]<=mmbuf_value[10:0]; 
        
         //performance control counter nu 25 sel: 2 
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b11001010))
            cr_perf_control1[10:0]<=mmbuf_value[10:0];
        
         //filter window enable, nu 22 sel: 7
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10110111))
            cr_filter_en <= mmbuf_value[`FILTER_WINDOW_NUM-1:0];

         //nu 22 sel: 5
         if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10110101))
            cr_filter_addr <= mmbuf_value[`FILTER_WINDOW_DEPTH-1:0];
      end
end       


integer fw_set_i;

always @(posedge clock)
begin
    for (fw_set_i=0; fw_set_i<`FILTER_WINDOW_NUM*2; fw_set_i=fw_set_i+1)
        if (mmbuf_valid&(~commitbus_ex)&op_mtc0&cp0_reg_wen&(regi==8'b10110100)) //No. 22, sel: 4
            filter_window[cr_filter_addr] <= mmbuf_value[31:`FILTER_MASK_UNIT];
end

wire [`FILTER_WINDOW_NUM-1:0] fw_cmp_result;

assign fw_cmp_result[0] = (filter_window[ 0]==(filter_window[ 1]&filter_window_req_i[31:`FILTER_MASK_UNIT])) & cr_filter_en[0];
assign fw_cmp_result[1] = (filter_window[ 2]==(filter_window[ 3]&filter_window_req_i[31:`FILTER_MASK_UNIT])) & cr_filter_en[1];
assign fw_cmp_result[2] = (filter_window[ 4]==(filter_window[ 5]&filter_window_req_i[31:`FILTER_MASK_UNIT])) & cr_filter_en[2];
assign fw_cmp_result[3] = (filter_window[ 6]==(filter_window[ 7]&filter_window_req_i[31:`FILTER_MASK_UNIT])) & cr_filter_en[3];
assign filter_window_result_o = |fw_cmp_result;


// LLbit 
always @(posedge clock)
begin
    if (reset)
        cr_llbit <= 1'b0;
    else
    begin
        if (~cr_cfg6_rti_o & ~mmbuf_invalid & op_ll & ~d_ex & (ejtag_valid | paddr_ready) & normal_allowin)
            cr_llbit <= 1'b1;
        else if (cr_cfg6_rti_o & ll_set_llbit_i)
            cr_llbit <= 1'b1;
        else if (mmbuf_valid & op_eret & cp0_reg_wen)
            cr_llbit <= 1'b0;
    end
end

assign cr_llbit_value_o = cr_llbit;
assign cr_cfg6_brpred_type_o = cr_config6[2:0];
assign cr_cfg6_rti_o = cr_config6[3];
assign cr_cfg6_cache0_all_o = cr_config6[4];
assign cr_cfg7_dcache_o = cr_config7[1:0];
assign cr_cfg7_icache_o = cr_config7[3:2];

// DESAVE register
always @(posedge clock)
begin
  if (reset)
    cr_desave<=32'h00000000;
  else
    if (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b11111000))
       cr_desave<=mmbuf_value;
end

// DEPC register
always @(posedge clock)
begin
  if (reset)
    cr_depc<=32'h00000000;
  else
    if (commitbus_ex&&ex_ejtagboot)
      cr_depc<=32'hbfc00000;
    else
      if ((commitbus_ex&&ex_debug)|| //debug exception
          (commitbus_ex&&DEBUG_MODE&&(~ex_softreset))) //debug mode exception
           cr_depc<=epc;
    else
      if (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b11000000))
          cr_depc<=mmbuf_value;
end

// DEBUG register
always @(posedge clock)
if (reset)
  cr_debug<=32'h00010000;
else
  begin
     // debug[31]:DBD bit
     if ((commitbus_ex&&ex_debug)||
         (commitbus_ex&&DEBUG_MODE&&(~ex_softreset)))
        cr_debug[31]<=commitbus_bd;

     // debug[30]:DM bit
     if (commitbus_ex&&ex_debug)
        cr_debug[30]<=1'b1;
     else
        if (commit_deret) //DERET 
        //replace (daddrbus_valid&&op_deret) with commit_deret to comply
        //with dss_flag in queue when committing deret.
           cr_debug[30]<=1'b0;

     // debug[29]:NoDCR bit
     // NoDCR is preset and fixed as 1'b0;

     // debug[28]:LSNM bit
     if (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b10111000))
        cr_debug[28]<=mmbuf_value[28];
                                                                                                                    
     // debug[27]:Doze bit
     // Doze is preset and fixed as 1'b0;
                                                                                                                     
     // debug[26]:Halt bit
     // Halt is preset and fixed as 1'b0;
                                                                                                                     
     // debug[25]:CountDM bit
     if (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b10111000))
        cr_debug[25]<=mmbuf_value[25];
                                                                                                                     
     // debug[24]:IBusEP bit
     if (commitbus_ex&&ex_ibe)
        cr_debug[24]<=1'b0;
     else
        if (IBE_FROM_CACHE||
            (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b10111000)&&mmbuf_value[24]))
           cr_debug[24]<=1'b1;

     // debug[23]:MCheckP bit
     // MCheckP is preset and fixed as 1'b0;
                                                                                                                     
     // debug[22]:CacheEP bit
     // CacheEP is preset and fixed as 1'b0;
                                                                                                                     
     // debug[21]:DBusEP bit
     // DBusEP is preset and fixed as 1'b0;
                                                                                                                    
     // debug[20]:IEXI bit
     if ((commitbus_ex&&ex_debug)||(commitbus_ex&&DEBUG_MODE&&(~ex_softreset)))
        cr_debug[20]<=1'b1;
     else
        if (mmbuf_valid&&op_deret&cp0_reg_wen)
           cr_debug[20]<=1'b0;
        else
           if (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b10111000))
              cr_debug[20]<=mmbuf_value[20];
                                                                                                                     
      // debug[19]:DDBSImpr bit
      if (commitbus_ex&&ex_ddbsimpr)
         cr_debug[19]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[19]<=1'b0;
                                                                                                                    
      // debug[18]:DDBLImpr bit
      if (commitbus_ex&&ex_ddblimpr)
         cr_debug[18]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[18]<=1'b0;

      // debug[17:15]:EJTAGver bits
      // EJTAGver bits is preset and fixed as 3'b010(version 2.6);
                                                                                                                    
      // debug[14:10]:DExcCode bits
      if (commitbus_ex&&DEBUG_MODE&&(~ex_softreset))
         cr_debug[14:10]<=commitbus_excode[4:0];
                                                                                                                    
      // debug[9]:NoSSt bit
      // NoSSt bit is preset and fixed as 1'b0 to indicate the single-step feature.
                                                                                                                   
      // debug[8]:SSt bit
      if (mmbuf_valid&&(~commitbus_ex)&&op_mtc0&cp0_reg_wen&&(regi==8'b10111000))
         cr_debug[8]<=mmbuf_value[8];
                                                                                                              
      // debug[5]:DINT bit
      if (commitbus_ex&&(ex_dint||ex_ejtagboot))
         cr_debug[5]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[5]<=1'b0;
                                                                                                                     
      // debug[4]:DIB bit
      if (commitbus_ex&&ex_dib)
         cr_debug[4]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[4]<=1'b0;
                                                                                                             
      // debug[3]:DDBS bit
      if (commitbus_ex&&ex_ddbs)
         cr_debug[3]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[3]<=1'b0;

      // debug[2]:DDBL bit
      if (commitbus_ex&&ex_ddbl)
         cr_debug[2]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[2]<=1'b0;

      // debug[1]:DBp bit
      if (commitbus_ex&&ex_sdbbp)
         cr_debug[1]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[1]<=1'b0;

      // debug[0]:DSS bit
      if (commitbus_ex&&ex_dss)
         cr_debug[0]<=1'b1;
      else
         if ((commitbus_ex&&DEBUG_MODE&&(~ex_softreset))||(mmbuf_valid&&op_deret))
            cr_debug[0]<=1'b0;

end


// output the ejtag control signals
assign DEBUG_MODE=cr_debug[30];
assign DSS_ENABLE=(~cr_debug[9])&&cr_debug[8];
assign NODCR=cr_debug[29];
assign IEXI=cr_debug[20];
assign PENDING_IBE=(~IEXI)&&cr_debug[24];
assign PENDING_DBE=(~IEXI)&&cr_debug[21];
assign EJTAGBRK_FROM_CORE=cr_debug[5];
assign lsnm=cr_debug[28];

//for d_valid
assign dmseg_daddr      = ({12'h002,mmbuf_vaddr_r[19:0]});
assign drseg_daddr      = ({12'h003,mmbuf_vaddr_r[19:0]});
assign dcr_daddr        = 32'h00000000;

assign ejtag_dseg_dreq  = (mmbuf_vaddr_r[31:21]==11'b11111111001);
assign ejtag_dcr        = ejtag_dseg_dreq&&mmbuf_vaddr_r[20]
                         &&(mmbuf_vaddr_r[15:2]==14'h0000);
assign ejtag_drseg      = ejtag_dseg_dreq&&mmbuf_vaddr_r[20]&&(~ejtag_dcr);
assign drseg_enable     = /*PROBEN_IN&&*/(~NODCR)&&(~lsnm)&&DEBUG_MODE;

assign ejtag_dmseg_dreq = ejtag_dseg_dreq&&(~mmbuf_vaddr_r[20]);
assign dmseg_dreq_enable= PROBEN_IN&&(~NODCR)&&(~lsnm)&&DEBUG_MODE;
assign ejtag_dreq_valid = ((ejtag_dcr||ejtag_drseg)&&drseg_enable)||
                          (ejtag_dmseg_dreq&&dmseg_dreq_enable);

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// hb_dcompbus
wire        lb,lh,lw,lbu,lhu,ll,sb,sh,sw,sc,lwl,lwr,swl,swr,ldc1,sdc1;
wire        op_mem;
wire        byte_loadop,byte_storeop;
wire        halfword_loadop,halfword_storeop;
wire        tribyte_loadop,tribyte_storeop;
wire        word_loadop,word_storeop;
wire        byte_memop,halfword_memop,tribyte_memop,word_memop;
wire [1:0]  d_offset;
reg  [3:0]  byte_bytelane;
reg  [3:0]  tribyte_bytelane;
reg  [3:0]  halfword_bytelane;
wire [3:0]  word_bytelane;
wire [31:0] mb_word,mh_word,mt_word,mw_word;
                                                                                                                             
assign op_mem   = op_load | op_store;
                                                                                                                             
assign d_offset = mmbuf_vaddr_r[1:0];
 
assign byte_loadop  = op_lb||op_lbu||
                      op_lbux ||
                      (op_lwl&&(d_offset==2'b00))||
                      (op_lwr&&(d_offset==2'b11));
assign byte_storeop = op_sb||
                      (op_swl&&(d_offset==2'b00))||
                      (op_swr&&(d_offset==2'b11));
assign byte_memop   = byte_loadop||byte_storeop;

assign halfword_loadop  = op_lh||op_lhu||
                          op_lhx ||
                          (op_lwl&&(d_offset==2'b01))||
                          (op_lwr&&(d_offset==2'b10));
assign halfword_storeop = op_sh||
                          (op_swl&&(d_offset==2'b01))||
                          (op_swr&&(d_offset==2'b10));
assign halfword_memop   = halfword_loadop||halfword_storeop;

assign tribyte_loadop   = (op_lwl&&(d_offset==2'b10))||
                          (op_lwr&&(d_offset==2'b01));
assign tribyte_storeop  = (op_swl&&(d_offset==2'b10))||
                          (op_swr&&(d_offset==2'b01));
assign tribyte_memop    = tribyte_loadop||tribyte_storeop;

assign word_loadop      = op_lw||op_ll||  op_lwc1||
                          op_lwx ||
                          (op_lwl&&(d_offset==2'b11))||
                          (op_lwr&&(d_offset==2'b00));
assign word_storeop     = op_sw||(op_sc&cr_llbit)||op_swc1 ||
                          (op_swl&&(d_offset==2'b11))||
                          (op_swr&&(d_offset==2'b00));
assign word_memop       = word_loadop||word_storeop;

always @(d_offset or op_lb or op_sb or op_lbu or op_lwr or op_swr
        or op_lbux
        )
begin
   if (op_lb||op_sb||op_lbu
       ||op_lbux
      )
      begin
           case (d_offset)
              2'b00:byte_bytelane=4'b0001;
              2'b01:byte_bytelane=4'b0010;
              2'b10:byte_bytelane=4'b0100;
              2'b11:byte_bytelane=4'b1000;
            endcase
       end
   else
      if ((op_swr&&(d_offset==2'b11))||(op_lwr&&(d_offset==2'b11)))
          byte_bytelane=4'b1000;
      else
          byte_bytelane=4'b0001;
end

always @(d_offset or op_lwr or op_swr or op_lh or op_sh or op_lhu
        or op_lhx
        )
begin
   if (op_lh||op_sh||op_lhu
       ||op_lhx
      )
      halfword_bytelane=d_offset[1]?4'b1100:4'b0011;
   else
      if ((op_swr&&(d_offset==2'b10))||(op_lwr&&(d_offset==2'b10)))
         halfword_bytelane=4'b1100;
      else
         halfword_bytelane=4'b0011;
end
                                                                                                                             
always @(d_offset or op_lwr or op_swr)
begin
  if ((op_swr&&(d_offset==2'b01))||(op_lwr&&(d_offset==2'b01)))
     tribyte_bytelane=4'b1110;
  else
     tribyte_bytelane=4'b0111;
end
                                                                                                                             
assign word_bytelane=4'b1111;

                                                                                                                            
wire   hb_is_dw     = op_ldc1 | op_sdc1;

assign hb_storedata = mmbuf_value;
assign hb_bytelane  = ({4{byte_memop}}&byte_bytelane)|
                      ({4{halfword_memop}}&halfword_bytelane)|
                      ({4{tribyte_memop}}&tribyte_bytelane)|
                      ({4{word_memop}}&word_bytelane)
                      ;
assign hb_type      = ~op_load;
assign hb_addr      = mmbuf_vaddr_r;
assign hb_dvalid    = d_valid & op_mem & ~(op_sc & ~cr_llbit);
wire [31:0] hb_storedata_h = mmbuf_value_h;

assign to_mq_bytelane = hb_bytelane;

assign HB_DCOMPBUS[120:89]= hb_storedata_h;
assign HB_DCOMPBUS[88]    = hb_is_dw;
assign HB_DCOMPBUS[87:85] = mmbuf_brqid_r;
assign HB_DCOMPBUS[84]    = op_load;
assign HB_DCOMPBUS[83:80] = mmbuf_qid_r;
assign HB_DCOMPBUS[79]    = d_ex_ades;
assign HB_DCOMPBUS[78]    = d_ex_adel;
assign HB_DCOMPBUS[77:46] = hb_storedata;
assign HB_DCOMPBUS[45:42] = hb_bytelane;
assign HB_DCOMPBUS[41]    = hb_type;//0-load,1-store
assign HB_DCOMPBUS[40:9]  = hb_addr;
assign HB_DCOMPBUS[8:1]   = asid;
assign HB_DCOMPBUS[0]     = hb_dvalid;                                                                                                                              
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// hardbreak registers access request
assign ade = d_addr_align_err;

assign op_word = op_lw||op_sw||op_lwx || op_lwc1;
assign op_cp1_dw  = op_ldc1||op_sdc1;

assign hb_reqbus_ades  = hb_reqbus_valid&&ade&&op_store;//only word operation
assign hb_reqbus_adel  = hb_reqbus_valid&&ade&&op_load;//only word operation
assign hb_reqbus_value = mmbuf_value_r;
assign hb_reqbus_op    = mmbuf_op_r;
assign hb_reqbus_addr  = drseg_daddr;
assign hb_reqbus_qid   = mmbuf_qid_r;
assign access_drseg    = mmbuf_valid&&drseg_enable&&ejtag_drseg;
assign hb_reqbus_valid = access_drseg&&op_word;
 
assign HB_REQBUS[78]    = hb_reqbus_ades;
assign HB_REQBUS[77]    = hb_reqbus_adel;
assign HB_REQBUS[76:45] = hb_reqbus_value;
assign HB_REQBUS[44:37] = hb_reqbus_op;
assign HB_REQBUS[36:5]  = hb_reqbus_addr;
assign HB_REQBUS[4:1]   = hb_reqbus_qid;
assign HB_REQBUS[0]     = hb_reqbus_valid;
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// DCR access request.
assign dcr_reqbus_ades   =  dcr_reqbus_valid&&ade&&op_store;//only word operation
assign dcr_reqbus_adel   =  dcr_reqbus_valid&&ade&&op_load;//only word operation
assign dcr_reqbus_value  =  mmbuf_value_r;
assign dcr_reqbus_op     =  mmbuf_op_r;
assign dcr_reqbus_addr   =  dcr_daddr;
assign dcr_reqbus_qid    =  mmbuf_qid_r;
assign access_dcr        =  mmbuf_valid&&drseg_enable&&ejtag_dcr;
assign dcr_reqbus_valid  =  access_dcr&&op_word;
 
assign DCR_REQBUS[78]    = dcr_reqbus_ades;
assign DCR_REQBUS[77]    = dcr_reqbus_adel;
assign DCR_REQBUS[76:45] = dcr_reqbus_value;
assign DCR_REQBUS[44:37] = dcr_reqbus_op;
assign DCR_REQBUS[36:5]  = dcr_reqbus_addr;
assign DCR_REQBUS[4:1]   = dcr_reqbus_qid;
assign DCR_REQBUS[0]     = dcr_reqbus_valid;
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// dmseg data access request.
assign dmseg_dreqbus_ades   = dmseg_dreqbus_valid&&ade&&op_store;
assign dmseg_dreqbus_adel   = dmseg_dreqbus_valid&&ade&&op_load;
assign dmseg_dreqbus_value  = mmbuf_value_r;
assign dmseg_dreqbus_op     = mmbuf_op_r;
assign dmseg_dreqbus_addr   = dmseg_daddr;
assign dmseg_dreqbus_qid    = mmbuf_qid_r;
assign access_dmseg         = mmbuf_valid&&dmseg_dreq_enable&&ejtag_dmseg_dreq;
assign dmseg_dreqbus_valid  = access_dmseg&&(~op_cp1_dw);
                                                                                                                             
assign DMSEG_DREQBUS[78]    = dmseg_dreqbus_ades;
assign DMSEG_DREQBUS[77]    = dmseg_dreqbus_adel;
assign DMSEG_DREQBUS[76:45] = dmseg_dreqbus_value;
assign DMSEG_DREQBUS[44:37] = dmseg_dreqbus_op;
assign DMSEG_DREQBUS[36:5]  = dmseg_dreqbus_addr;
assign DMSEG_DREQBUS[4:1]   = dmseg_dreqbus_qid;
assign DMSEG_DREQBUS[0]     = dmseg_dreqbus_valid;
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// FIRST_MEM_PASS_AFTER_DERET 
reg     deret_dflag;
wire    FIRST_MEM_PASS_AFTER_DERET;

always @(posedge clock)
if (reset)
   deret_dflag<=1'b0;
else
   if (commit_deret)
      deret_dflag<=1'b1;
   else
      if (FIRST_MEM_PASS_AFTER_DERET||(commitbus_ex&&ex_dib))
         deret_dflag<=1'b0;

assign FIRST_MEM_PASS_AFTER_DERET = deret_dflag & to_mq_valid & memqueue_allowin; 

endmodule //godson_dtlb_module


module tlb_cam(
    clock,
    vpn2_lookup_i,
    asid_lookup_i,
    find_o,
    index_out_o,
    pagemask_out_o,
    asid_out_o,
    g_out_o,
    vpn2_out_o,
    
    wen_i,
    windex_i,
    w_vpn2_i,
    w_pagemask_i,
    w_asid_i,
    w_g_i,
    
    tlbr_valid_i,
    rindex_i

);
input         clock;
// vpn2 used to look up tlb cam 
input  [18:0] vpn2_lookup_i;
// asid used to look up tlb cam
input  [ 7:0] asid_lookup_i;
// whether tlb cam lookup is ok
output        find_o;
// result index of tlb cam lookup
output [ 5:0] index_out_o;
// pagemask of the select tlb iterm
output [ 7:0] pagemask_out_o;
// asid of the select tlb iterm
output [ 7:0] asid_out_o;
// g of the select tlb iterm
output        g_out_o;
// vpn2 of the select tlb iterm
output [18:0] vpn2_out_o;

// tlb cam write enable
input         wen_i;
// index used for TLBWI, TLBWR
input  [ 5:0] windex_i;
// vpn2 value written into tlb cam
input  [18:0] w_vpn2_i;
// pagemask value written into tlb cam
input  [ 7:0] w_pagemask_i;
// asid value written into tlb cam
input  [ 7:0] w_asid_i;
// g value written into tlb cam 
input         w_g_i;

// TLBR instr.
input         tlbr_valid_i;
// index used for tlb cam read (TLBR) 
input  [ 5:0] rindex_i;

reg  [18:0] reg_vpn2[31:0];
reg  [ 7:0] reg_asid[31:0];
reg  [ 7:0] reg_mask[31:0];
reg  [31:0] reg_g;

// read enable for tlb lookup
wire ren0, ren1, ren2, ren3, ren4, ren5, ren6, ren7 ;
wire ren8, ren9, ren10,ren11,ren12,ren13,ren14,ren15;
wire ren16,ren17,ren18,ren19,ren20,ren21,ren22,ren23;
wire ren24,ren25,ren26,ren27,ren28,ren29,ren30,ren31;

// read enable for tlbr instr.
wire tlbr_en0, tlbr_en1, tlbr_en2, tlbr_en3, tlbr_en4, tlbr_en5, tlbr_en6, tlbr_en7 ;
wire tlbr_en8, tlbr_en9, tlbr_en10,tlbr_en11,tlbr_en12,tlbr_en13,tlbr_en14,tlbr_en15;
wire tlbr_en16,tlbr_en17,tlbr_en18,tlbr_en19,tlbr_en20,tlbr_en21,tlbr_en22,tlbr_en23;
wire tlbr_en24,tlbr_en25,tlbr_en26,tlbr_en27,tlbr_en28,tlbr_en29,tlbr_en30,tlbr_en31;

// read enable
wire rden0, rden1, rden2, rden3, rden4, rden5, rden6, rden7 ;
wire rden8, rden9, rden10,rden11,rden12,rden13,rden14,rden15;
wire rden16,rden17,rden18,rden19,rden20,rden21,rden22,rden23;
wire rden24,rden25,rden26,rden27,rden28,rden29,rden30,rden31;

// write enable for tlbwi or tlbwr
wire wen0, wen1, wen2, wen3, wen4, wen5, wen6, wen7 ;
wire wen8, wen9, wen10,wen11,wen12,wen13,wen14,wen15;
wire wen16,wen17,wen18,wen19,wen20,wen21,wen22,wen23;
wire wen24,wen25,wen26,wen27,wen28,wen29,wen30,wen31;

wire [31:0] asid_match;

wire [15:0] reg_mask_0 ;
wire [15:0] reg_mask_1 ;
wire [15:0] reg_mask_2 ;
wire [15:0] reg_mask_3 ;
wire [15:0] reg_mask_4 ;
wire [15:0] reg_mask_5 ;
wire [15:0] reg_mask_6 ;
wire [15:0] reg_mask_7 ;
wire [15:0] reg_mask_8 ;
wire [15:0] reg_mask_9 ;
wire [15:0] reg_mask_10;
wire [15:0] reg_mask_11;
wire [15:0] reg_mask_12;
wire [15:0] reg_mask_13;
wire [15:0] reg_mask_14;
wire [15:0] reg_mask_15;
wire [15:0] reg_mask_16;
wire [15:0] reg_mask_17;
wire [15:0] reg_mask_18;
wire [15:0] reg_mask_19;
wire [15:0] reg_mask_20;
wire [15:0] reg_mask_21;
wire [15:0] reg_mask_22;
wire [15:0] reg_mask_23;
wire [15:0] reg_mask_24;
wire [15:0] reg_mask_25;
wire [15:0] reg_mask_26;
wire [15:0] reg_mask_27;
wire [15:0] reg_mask_28;
wire [15:0] reg_mask_29;
wire [15:0] reg_mask_30;
wire [15:0] reg_mask_31;

assign reg_mask_0  = {reg_mask[ 0][7], reg_mask[ 0][7], reg_mask[ 0][6], reg_mask[ 0][6],
                      reg_mask[ 0][5], reg_mask[ 0][5], reg_mask[ 0][4], reg_mask[ 0][4],
                      reg_mask[ 0][3], reg_mask[ 0][3], reg_mask[ 0][2], reg_mask[ 0][2],
                      reg_mask[ 0][1], reg_mask[ 0][1], reg_mask[ 0][0], reg_mask[ 0][0]};

assign reg_mask_1  = {reg_mask[ 1][7], reg_mask[ 1][7], reg_mask[ 1][6], reg_mask[ 1][6],
                      reg_mask[ 1][5], reg_mask[ 1][5], reg_mask[ 1][4], reg_mask[ 1][4],
                      reg_mask[ 1][3], reg_mask[ 1][3], reg_mask[ 1][2], reg_mask[ 1][2],
                      reg_mask[ 1][1], reg_mask[ 1][1], reg_mask[ 1][0], reg_mask[ 1][0]};

assign reg_mask_2  = {reg_mask[ 2][7], reg_mask[ 2][7], reg_mask[ 2][6], reg_mask[ 2][6],
                      reg_mask[ 2][5], reg_mask[ 2][5], reg_mask[ 2][4], reg_mask[ 2][4],
                      reg_mask[ 2][3], reg_mask[ 2][3], reg_mask[ 2][2], reg_mask[ 2][2],
                      reg_mask[ 2][1], reg_mask[ 2][1], reg_mask[ 2][0], reg_mask[ 2][0]};

assign reg_mask_3  = {reg_mask[ 3][7], reg_mask[ 3][7], reg_mask[ 3][6], reg_mask[ 3][6],
                      reg_mask[ 3][5], reg_mask[ 3][5], reg_mask[ 3][4], reg_mask[ 3][4],
                      reg_mask[ 3][3], reg_mask[ 3][3], reg_mask[ 3][2], reg_mask[ 3][2],
                      reg_mask[ 3][1], reg_mask[ 3][1], reg_mask[ 3][0], reg_mask[ 3][0]};

assign reg_mask_4  = {reg_mask[ 4][7], reg_mask[ 4][7], reg_mask[ 4][6], reg_mask[ 4][6],
                      reg_mask[ 4][5], reg_mask[ 4][5], reg_mask[ 4][4], reg_mask[ 4][4],
                      reg_mask[ 4][3], reg_mask[ 4][3], reg_mask[ 4][2], reg_mask[ 4][2],
                      reg_mask[ 4][1], reg_mask[ 4][1], reg_mask[ 4][0], reg_mask[ 4][0]};

assign reg_mask_5  = {reg_mask[ 5][7], reg_mask[ 5][7], reg_mask[ 5][6], reg_mask[ 5][6],
                      reg_mask[ 5][5], reg_mask[ 5][5], reg_mask[ 5][4], reg_mask[ 5][4],
                      reg_mask[ 5][3], reg_mask[ 5][3], reg_mask[ 5][2], reg_mask[ 5][2],
                      reg_mask[ 5][1], reg_mask[ 5][1], reg_mask[ 5][0], reg_mask[ 5][0]};

assign reg_mask_6  = {reg_mask[ 6][7], reg_mask[ 6][7], reg_mask[ 6][6], reg_mask[ 6][6],
                      reg_mask[ 6][5], reg_mask[ 6][5], reg_mask[ 6][4], reg_mask[ 6][4],
                      reg_mask[ 6][3], reg_mask[ 6][3], reg_mask[ 6][2], reg_mask[ 6][2],
                      reg_mask[ 6][1], reg_mask[ 6][1], reg_mask[ 6][0], reg_mask[ 6][0]};

assign reg_mask_7  = {reg_mask[ 7][7], reg_mask[ 7][7], reg_mask[ 7][6], reg_mask[ 7][6],
                      reg_mask[ 7][5], reg_mask[ 7][5], reg_mask[ 7][4], reg_mask[ 7][4],
                      reg_mask[ 7][3], reg_mask[ 7][3], reg_mask[ 7][2], reg_mask[ 7][2],
                      reg_mask[ 7][1], reg_mask[ 7][1], reg_mask[ 7][0], reg_mask[ 7][0]};

assign reg_mask_8  = {reg_mask[ 8][7], reg_mask[ 8][7], reg_mask[ 8][6], reg_mask[ 8][6],
                      reg_mask[ 8][5], reg_mask[ 8][5], reg_mask[ 8][4], reg_mask[ 8][4],
                      reg_mask[ 8][3], reg_mask[ 8][3], reg_mask[ 8][2], reg_mask[ 8][2],
                      reg_mask[ 8][1], reg_mask[ 8][1], reg_mask[ 8][0], reg_mask[ 8][0]};

assign reg_mask_9  = {reg_mask[ 9][7], reg_mask[ 9][7], reg_mask[ 9][6], reg_mask[ 9][6],
                      reg_mask[ 9][5], reg_mask[ 9][5], reg_mask[ 9][4], reg_mask[ 9][4],
                      reg_mask[ 9][3], reg_mask[ 9][3], reg_mask[ 9][2], reg_mask[ 9][2],
                      reg_mask[ 9][1], reg_mask[ 9][1], reg_mask[ 9][0], reg_mask[ 9][0]};

assign reg_mask_10 = {reg_mask[10][7], reg_mask[10][7], reg_mask[10][6], reg_mask[10][6],
                      reg_mask[10][5], reg_mask[10][5], reg_mask[10][4], reg_mask[10][4],
                      reg_mask[10][3], reg_mask[10][3], reg_mask[10][2], reg_mask[10][2],
                      reg_mask[10][1], reg_mask[10][1], reg_mask[10][0], reg_mask[10][0]};

assign reg_mask_11 = {reg_mask[11][7], reg_mask[11][7], reg_mask[11][6], reg_mask[11][6],
                      reg_mask[11][5], reg_mask[11][5], reg_mask[11][4], reg_mask[11][4],
                      reg_mask[11][3], reg_mask[11][3], reg_mask[11][2], reg_mask[11][2],
                      reg_mask[11][1], reg_mask[11][1], reg_mask[11][0], reg_mask[11][0]};

assign reg_mask_12 = {reg_mask[12][7], reg_mask[12][7], reg_mask[12][6], reg_mask[12][6],
                      reg_mask[12][5], reg_mask[12][5], reg_mask[12][4], reg_mask[12][4],
                      reg_mask[12][3], reg_mask[12][3], reg_mask[12][2], reg_mask[12][2],
                      reg_mask[12][1], reg_mask[12][1], reg_mask[12][0], reg_mask[12][0]};

assign reg_mask_13 = {reg_mask[13][7], reg_mask[13][7], reg_mask[13][6], reg_mask[13][6],
                      reg_mask[13][5], reg_mask[13][5], reg_mask[13][4], reg_mask[13][4],
                      reg_mask[13][3], reg_mask[13][3], reg_mask[13][2], reg_mask[13][2],
                      reg_mask[13][1], reg_mask[13][1], reg_mask[13][0], reg_mask[13][0]};

assign reg_mask_14 = {reg_mask[14][7], reg_mask[14][7], reg_mask[14][6], reg_mask[14][6],
                      reg_mask[14][5], reg_mask[14][5], reg_mask[14][4], reg_mask[14][4],
                      reg_mask[14][3], reg_mask[14][3], reg_mask[14][2], reg_mask[14][2],
                      reg_mask[14][1], reg_mask[14][1], reg_mask[14][0], reg_mask[14][0]};

assign reg_mask_15 = {reg_mask[15][7], reg_mask[15][7], reg_mask[15][6], reg_mask[15][6],
                      reg_mask[15][5], reg_mask[15][5], reg_mask[15][4], reg_mask[15][4],
                      reg_mask[15][3], reg_mask[15][3], reg_mask[15][2], reg_mask[15][2],
                      reg_mask[15][1], reg_mask[15][1], reg_mask[15][0], reg_mask[15][0]};

assign reg_mask_16 = {reg_mask[16][7], reg_mask[16][7], reg_mask[16][6], reg_mask[16][6],
                      reg_mask[16][5], reg_mask[16][5], reg_mask[16][4], reg_mask[16][4],
                      reg_mask[16][3], reg_mask[16][3], reg_mask[16][2], reg_mask[16][2],
                      reg_mask[16][1], reg_mask[16][1], reg_mask[16][0], reg_mask[16][0]};

assign reg_mask_17 = {reg_mask[17][7], reg_mask[17][7], reg_mask[17][6], reg_mask[17][6],
                      reg_mask[17][5], reg_mask[17][5], reg_mask[17][4], reg_mask[17][4],
                      reg_mask[17][3], reg_mask[17][3], reg_mask[17][2], reg_mask[17][2],
                      reg_mask[17][1], reg_mask[17][1], reg_mask[17][0], reg_mask[17][0]};

assign reg_mask_18 = {reg_mask[18][7], reg_mask[18][7], reg_mask[18][6], reg_mask[18][6],
                      reg_mask[18][5], reg_mask[18][5], reg_mask[18][4], reg_mask[18][4],
                      reg_mask[18][3], reg_mask[18][3], reg_mask[18][2], reg_mask[18][2],
                      reg_mask[18][1], reg_mask[18][1], reg_mask[18][0], reg_mask[18][0]};

assign reg_mask_19 = {reg_mask[19][7], reg_mask[19][7], reg_mask[19][6], reg_mask[19][6],
                      reg_mask[19][5], reg_mask[19][5], reg_mask[19][4], reg_mask[19][4],
                      reg_mask[19][3], reg_mask[19][3], reg_mask[19][2], reg_mask[19][2],
                      reg_mask[19][1], reg_mask[19][1], reg_mask[19][0], reg_mask[19][0]};

assign reg_mask_20 = {reg_mask[20][7], reg_mask[20][7], reg_mask[20][6], reg_mask[20][6],
                      reg_mask[20][5], reg_mask[20][5], reg_mask[20][4], reg_mask[20][4],
                      reg_mask[20][3], reg_mask[20][3], reg_mask[20][2], reg_mask[20][2],
                      reg_mask[20][1], reg_mask[20][1], reg_mask[20][0], reg_mask[20][0]};

assign reg_mask_21 = {reg_mask[21][7], reg_mask[21][7], reg_mask[21][6], reg_mask[21][6],
                      reg_mask[21][5], reg_mask[21][5], reg_mask[21][4], reg_mask[21][4],
                      reg_mask[21][3], reg_mask[21][3], reg_mask[21][2], reg_mask[21][2],
                      reg_mask[21][1], reg_mask[21][1], reg_mask[21][0], reg_mask[21][0]};

assign reg_mask_22 = {reg_mask[22][7], reg_mask[22][7], reg_mask[22][6], reg_mask[22][6],
                      reg_mask[22][5], reg_mask[22][5], reg_mask[22][4], reg_mask[22][4],
                      reg_mask[22][3], reg_mask[22][3], reg_mask[22][2], reg_mask[22][2],
                      reg_mask[22][1], reg_mask[22][1], reg_mask[22][0], reg_mask[22][0]};

assign reg_mask_23 = {reg_mask[23][7], reg_mask[23][7], reg_mask[23][6], reg_mask[23][6],
                      reg_mask[23][5], reg_mask[23][5], reg_mask[23][4], reg_mask[23][4],
                      reg_mask[23][3], reg_mask[23][3], reg_mask[23][2], reg_mask[23][2],
                      reg_mask[23][1], reg_mask[23][1], reg_mask[23][0], reg_mask[23][0]};

assign reg_mask_24 = {reg_mask[24][7], reg_mask[24][7], reg_mask[24][6], reg_mask[24][6],
                      reg_mask[24][5], reg_mask[24][5], reg_mask[24][4], reg_mask[24][4],
                      reg_mask[24][3], reg_mask[24][3], reg_mask[24][2], reg_mask[24][2],
                      reg_mask[24][1], reg_mask[24][1], reg_mask[24][0], reg_mask[24][0]};

assign reg_mask_25 = {reg_mask[25][7], reg_mask[25][7], reg_mask[25][6], reg_mask[25][6],
                      reg_mask[25][5], reg_mask[25][5], reg_mask[25][4], reg_mask[25][4],
                      reg_mask[25][3], reg_mask[25][3], reg_mask[25][2], reg_mask[25][2],
                      reg_mask[25][1], reg_mask[25][1], reg_mask[25][0], reg_mask[25][0]};

assign reg_mask_26 = {reg_mask[26][7], reg_mask[26][7], reg_mask[26][6], reg_mask[26][6],
                      reg_mask[26][5], reg_mask[26][5], reg_mask[26][4], reg_mask[26][4],
                      reg_mask[26][3], reg_mask[26][3], reg_mask[26][2], reg_mask[26][2],
                      reg_mask[26][1], reg_mask[26][1], reg_mask[26][0], reg_mask[26][0]};

assign reg_mask_27 = {reg_mask[27][7], reg_mask[27][7], reg_mask[27][6], reg_mask[27][6],
                      reg_mask[27][5], reg_mask[27][5], reg_mask[27][4], reg_mask[27][4],
                      reg_mask[27][3], reg_mask[27][3], reg_mask[27][2], reg_mask[27][2],
                      reg_mask[27][1], reg_mask[27][1], reg_mask[27][0], reg_mask[27][0]};

assign reg_mask_28 = {reg_mask[28][7], reg_mask[28][7], reg_mask[28][6], reg_mask[28][6],
                      reg_mask[28][5], reg_mask[28][5], reg_mask[28][4], reg_mask[28][4],
                      reg_mask[28][3], reg_mask[28][3], reg_mask[28][2], reg_mask[28][2],
                      reg_mask[28][1], reg_mask[28][1], reg_mask[28][0], reg_mask[28][0]};

assign reg_mask_29 = {reg_mask[29][7], reg_mask[29][7], reg_mask[29][6], reg_mask[29][6],
                      reg_mask[29][5], reg_mask[29][5], reg_mask[29][4], reg_mask[29][4],
                      reg_mask[29][3], reg_mask[29][3], reg_mask[29][2], reg_mask[29][2],
                      reg_mask[29][1], reg_mask[29][1], reg_mask[29][0], reg_mask[29][0]};

assign reg_mask_30 = {reg_mask[30][7], reg_mask[30][7], reg_mask[30][6], reg_mask[30][6],
                      reg_mask[30][5], reg_mask[30][5], reg_mask[30][4], reg_mask[30][4],
                      reg_mask[30][3], reg_mask[30][3], reg_mask[30][2], reg_mask[30][2],
                      reg_mask[30][1], reg_mask[30][1], reg_mask[30][0], reg_mask[30][0]};

assign reg_mask_31 = {reg_mask[31][7], reg_mask[31][7], reg_mask[31][6], reg_mask[31][6],
                      reg_mask[31][5], reg_mask[31][5], reg_mask[31][4], reg_mask[31][4],
                      reg_mask[31][3], reg_mask[31][3], reg_mask[31][2], reg_mask[31][2],
                      reg_mask[31][1], reg_mask[31][1], reg_mask[31][0], reg_mask[31][0]};

wire [18:0] reg_vpn2_0  = reg_vpn2 [0 ];
wire [18:0] reg_vpn2_1  = reg_vpn2 [1 ];
wire [18:0] reg_vpn2_2  = reg_vpn2 [2 ];
wire [18:0] reg_vpn2_3  = reg_vpn2 [3 ];
wire [18:0] reg_vpn2_4  = reg_vpn2 [4 ];
wire [18:0] reg_vpn2_5  = reg_vpn2 [5 ];
wire [18:0] reg_vpn2_6  = reg_vpn2 [6 ];
wire [18:0] reg_vpn2_7  = reg_vpn2 [7 ];
wire [18:0] reg_vpn2_8  = reg_vpn2 [8 ];
wire [18:0] reg_vpn2_9  = reg_vpn2 [9 ];
wire [18:0] reg_vpn2_10 = reg_vpn2 [10];
wire [18:0] reg_vpn2_11 = reg_vpn2 [11];
wire [18:0] reg_vpn2_12 = reg_vpn2 [12];
wire [18:0] reg_vpn2_13 = reg_vpn2 [13];
wire [18:0] reg_vpn2_14 = reg_vpn2 [14];
wire [18:0] reg_vpn2_15 = reg_vpn2 [15];
wire [18:0] reg_vpn2_16 = reg_vpn2 [16];
wire [18:0] reg_vpn2_17 = reg_vpn2 [17];
wire [18:0] reg_vpn2_18 = reg_vpn2 [18];
wire [18:0] reg_vpn2_19 = reg_vpn2 [19];
wire [18:0] reg_vpn2_20 = reg_vpn2 [20];
wire [18:0] reg_vpn2_21 = reg_vpn2 [21];
wire [18:0] reg_vpn2_22 = reg_vpn2 [22];
wire [18:0] reg_vpn2_23 = reg_vpn2 [23];
wire [18:0] reg_vpn2_24 = reg_vpn2 [24];
wire [18:0] reg_vpn2_25 = reg_vpn2 [25];
wire [18:0] reg_vpn2_26 = reg_vpn2 [26];
wire [18:0] reg_vpn2_27 = reg_vpn2 [27];
wire [18:0] reg_vpn2_28 = reg_vpn2 [28];
wire [18:0] reg_vpn2_29 = reg_vpn2 [29];
wire [18:0] reg_vpn2_30 = reg_vpn2 [30];
wire [18:0] reg_vpn2_31 = reg_vpn2 [31];

wire [7:0] reg_asid_0   = reg_asid [0 ];
wire [7:0] reg_asid_1   = reg_asid [1 ];
wire [7:0] reg_asid_2   = reg_asid [2 ];
wire [7:0] reg_asid_3   = reg_asid [3 ];
wire [7:0] reg_asid_4   = reg_asid [4 ];
wire [7:0] reg_asid_5   = reg_asid [5 ];
wire [7:0] reg_asid_6   = reg_asid [6 ];
wire [7:0] reg_asid_7   = reg_asid [7 ];
wire [7:0] reg_asid_8   = reg_asid [8 ];
wire [7:0] reg_asid_9   = reg_asid [9 ];
wire [7:0] reg_asid_10  = reg_asid [10];
wire [7:0] reg_asid_11  = reg_asid [11];
wire [7:0] reg_asid_12  = reg_asid [12];
wire [7:0] reg_asid_13  = reg_asid [13];
wire [7:0] reg_asid_14  = reg_asid [14];
wire [7:0] reg_asid_15  = reg_asid [15];
wire [7:0] reg_asid_16  = reg_asid [16];
wire [7:0] reg_asid_17  = reg_asid [17];
wire [7:0] reg_asid_18  = reg_asid [18];
wire [7:0] reg_asid_19  = reg_asid [19];
wire [7:0] reg_asid_20  = reg_asid [20];
wire [7:0] reg_asid_21  = reg_asid [21];
wire [7:0] reg_asid_22  = reg_asid [22];
wire [7:0] reg_asid_23  = reg_asid [23];
wire [7:0] reg_asid_24  = reg_asid [24];
wire [7:0] reg_asid_25  = reg_asid [25];
wire [7:0] reg_asid_26  = reg_asid [26];
wire [7:0] reg_asid_27  = reg_asid [27];
wire [7:0] reg_asid_28  = reg_asid [28];
wire [7:0] reg_asid_29  = reg_asid [29];
wire [7:0] reg_asid_30  = reg_asid [30];
wire [7:0] reg_asid_31  = reg_asid [31];

assign asid_match[0]  = ((reg_asid[ 0]==asid_lookup_i)|reg_g[ 0]);
assign asid_match[1]  = ((reg_asid[ 1]==asid_lookup_i)|reg_g[ 1]);
assign asid_match[2]  = ((reg_asid[ 2]==asid_lookup_i)|reg_g[ 2]);
assign asid_match[3]  = ((reg_asid[ 3]==asid_lookup_i)|reg_g[ 3]);
assign asid_match[4]  = ((reg_asid[ 4]==asid_lookup_i)|reg_g[ 4]);
assign asid_match[5]  = ((reg_asid[ 5]==asid_lookup_i)|reg_g[ 5]);
assign asid_match[6]  = ((reg_asid[ 6]==asid_lookup_i)|reg_g[ 6]);
assign asid_match[7]  = ((reg_asid[ 7]==asid_lookup_i)|reg_g[ 7]);
assign asid_match[8]  = ((reg_asid[ 8]==asid_lookup_i)|reg_g[ 8]);
assign asid_match[9]  = ((reg_asid[ 9]==asid_lookup_i)|reg_g[ 9]);
assign asid_match[10] = ((reg_asid[10]==asid_lookup_i)|reg_g[10]);
assign asid_match[11] = ((reg_asid[11]==asid_lookup_i)|reg_g[11]);
assign asid_match[12] = ((reg_asid[12]==asid_lookup_i)|reg_g[12]);
assign asid_match[13] = ((reg_asid[13]==asid_lookup_i)|reg_g[13]);
assign asid_match[14] = ((reg_asid[14]==asid_lookup_i)|reg_g[14]);
assign asid_match[15] = ((reg_asid[15]==asid_lookup_i)|reg_g[15]);
assign asid_match[16] = ((reg_asid[16]==asid_lookup_i)|reg_g[16]);
assign asid_match[17] = ((reg_asid[17]==asid_lookup_i)|reg_g[17]);
assign asid_match[18] = ((reg_asid[18]==asid_lookup_i)|reg_g[18]);
assign asid_match[19] = ((reg_asid[19]==asid_lookup_i)|reg_g[19]);
assign asid_match[20] = ((reg_asid[20]==asid_lookup_i)|reg_g[20]);
assign asid_match[21] = ((reg_asid[21]==asid_lookup_i)|reg_g[21]);
assign asid_match[22] = ((reg_asid[22]==asid_lookup_i)|reg_g[22]);
assign asid_match[23] = ((reg_asid[23]==asid_lookup_i)|reg_g[23]);
assign asid_match[24] = ((reg_asid[24]==asid_lookup_i)|reg_g[24]);
assign asid_match[25] = ((reg_asid[25]==asid_lookup_i)|reg_g[25]);
assign asid_match[26] = ((reg_asid[26]==asid_lookup_i)|reg_g[26]);
assign asid_match[27] = ((reg_asid[27]==asid_lookup_i)|reg_g[27]);
assign asid_match[28] = ((reg_asid[28]==asid_lookup_i)|reg_g[28]);
assign asid_match[29] = ((reg_asid[29]==asid_lookup_i)|reg_g[29]);
assign asid_match[30] = ((reg_asid[30]==asid_lookup_i)|reg_g[30]);
assign asid_match[31] = ((reg_asid[31]==asid_lookup_i)|reg_g[31]);

assign ren0  = (({3'b111, ~reg_mask_0 }&vpn2_lookup_i)==(reg_vpn2[0] ))& asid_match[0];
assign ren1  = (({3'b111, ~reg_mask_1 }&vpn2_lookup_i)==(reg_vpn2[1] ))& asid_match[1];
assign ren2  = (({3'b111, ~reg_mask_2 }&vpn2_lookup_i)==(reg_vpn2[2] ))& asid_match[2];
assign ren3  = (({3'b111, ~reg_mask_3 }&vpn2_lookup_i)==(reg_vpn2[3] ))& asid_match[3];
assign ren4  = (({3'b111, ~reg_mask_4 }&vpn2_lookup_i)==(reg_vpn2[4] ))& asid_match[4];
assign ren5  = (({3'b111, ~reg_mask_5 }&vpn2_lookup_i)==(reg_vpn2[5] ))& asid_match[5];
assign ren6  = (({3'b111, ~reg_mask_6 }&vpn2_lookup_i)==(reg_vpn2[6] ))& asid_match[6];
assign ren7  = (({3'b111, ~reg_mask_7 }&vpn2_lookup_i)==(reg_vpn2[7] ))& asid_match[7];
assign ren8  = (({3'b111, ~reg_mask_8 }&vpn2_lookup_i)==(reg_vpn2[8] ))& asid_match[8];
assign ren9  = (({3'b111, ~reg_mask_9 }&vpn2_lookup_i)==(reg_vpn2[9] ))& asid_match[9];
assign ren10 = (({3'b111, ~reg_mask_10}&vpn2_lookup_i)==(reg_vpn2[10]))& asid_match[10];
assign ren11 = (({3'b111, ~reg_mask_11}&vpn2_lookup_i)==(reg_vpn2[11]))& asid_match[11];
assign ren12 = (({3'b111, ~reg_mask_12}&vpn2_lookup_i)==(reg_vpn2[12]))& asid_match[12];
assign ren13 = (({3'b111, ~reg_mask_13}&vpn2_lookup_i)==(reg_vpn2[13]))& asid_match[13];
assign ren14 = (({3'b111, ~reg_mask_14}&vpn2_lookup_i)==(reg_vpn2[14]))& asid_match[14];
assign ren15 = (({3'b111, ~reg_mask_15}&vpn2_lookup_i)==(reg_vpn2[15]))& asid_match[15];
assign ren16 = (({3'b111, ~reg_mask_16}&vpn2_lookup_i)==(reg_vpn2[16]))& asid_match[16];
assign ren17 = (({3'b111, ~reg_mask_17}&vpn2_lookup_i)==(reg_vpn2[17]))& asid_match[17];
assign ren18 = (({3'b111, ~reg_mask_18}&vpn2_lookup_i)==(reg_vpn2[18]))& asid_match[18];
assign ren19 = (({3'b111, ~reg_mask_19}&vpn2_lookup_i)==(reg_vpn2[19]))& asid_match[19];
assign ren20 = (({3'b111, ~reg_mask_20}&vpn2_lookup_i)==(reg_vpn2[20]))& asid_match[20];
assign ren21 = (({3'b111, ~reg_mask_21}&vpn2_lookup_i)==(reg_vpn2[21]))& asid_match[21];
assign ren22 = (({3'b111, ~reg_mask_22}&vpn2_lookup_i)==(reg_vpn2[22]))& asid_match[22];
assign ren23 = (({3'b111, ~reg_mask_23}&vpn2_lookup_i)==(reg_vpn2[23]))& asid_match[23];
assign ren24 = (({3'b111, ~reg_mask_24}&vpn2_lookup_i)==(reg_vpn2[24]))& asid_match[24];
assign ren25 = (({3'b111, ~reg_mask_25}&vpn2_lookup_i)==(reg_vpn2[25]))& asid_match[25];
assign ren26 = (({3'b111, ~reg_mask_26}&vpn2_lookup_i)==(reg_vpn2[26]))& asid_match[26];
assign ren27 = (({3'b111, ~reg_mask_27}&vpn2_lookup_i)==(reg_vpn2[27]))& asid_match[27];
assign ren28 = (({3'b111, ~reg_mask_28}&vpn2_lookup_i)==(reg_vpn2[28]))& asid_match[28];
assign ren29 = (({3'b111, ~reg_mask_29}&vpn2_lookup_i)==(reg_vpn2[29]))& asid_match[29];
assign ren30 = (({3'b111, ~reg_mask_30}&vpn2_lookup_i)==(reg_vpn2[30]))& asid_match[30];
assign ren31 = (({3'b111, ~reg_mask_31}&vpn2_lookup_i)==(reg_vpn2[31]))& asid_match[31];

assign find_o = ren0 |ren1 |ren2 |ren3 |ren4 |ren5 |ren6 |ren7 |
                ren8 |ren9 |ren10|ren11|ren12|ren13|ren14|ren15|
                ren16|ren17|ren18|ren19|ren20|ren21|ren22|ren23|
                ren24|ren25|ren26|ren27|ren28|ren29|ren30|ren31;

assign index_out_o = ({6{ren0 }}&6'b0    )|({6{ren1 }}&6'b1    )|({6{ren2 }}&6'b10   )|
                     ({6{ren3 }}&6'b11   )|({6{ren4 }}&6'b100  )|({6{ren5 }}&6'b101  )|
                     ({6{ren6 }}&6'b110  )|({6{ren7 }}&6'b111  )|({6{ren8 }}&6'b1000 )|
                     ({6{ren9 }}&6'b1001 )|({6{ren10}}&6'b1010 )|({6{ren11}}&6'b1011 )|
                     ({6{ren12}}&6'b1100 )|({6{ren13}}&6'b1101 )|({6{ren14}}&6'b1110 )|
                     ({6{ren15}}&6'b1111 )|({6{ren16}}&6'b10000)|({6{ren17}}&6'b10001)|
                     ({6{ren18}}&6'b10010)|({6{ren19}}&6'b10011)|({6{ren20}}&6'b10100)|
                     ({6{ren21}}&6'b10101)|({6{ren22}}&6'b10110)|({6{ren23}}&6'b10111)|
                     ({6{ren24}}&6'b11000)|({6{ren25}}&6'b11001)|({6{ren26}}&6'b11010)|
                     ({6{ren27}}&6'b11011)|({6{ren28}}&6'b11100)|({6{ren29}}&6'b11101)|
                     ({6{ren30}}&6'b11110)|({6{ren31}}&6'b11111);
 
assign wen0  = (windex_i==6'b0);    
assign wen1  = (windex_i==6'b1);
assign wen2  = (windex_i==6'b10);
assign wen3  = (windex_i==6'b11);
assign wen4  = (windex_i==6'b100);
assign wen5  = (windex_i==6'b101);
assign wen6  = (windex_i==6'b110);
assign wen7  = (windex_i==6'b111);
assign wen8  = (windex_i==6'b1000);
assign wen9  = (windex_i==6'b1001);
assign wen10 = (windex_i==6'b1010);
assign wen11 = (windex_i==6'b1011);
assign wen12 = (windex_i==6'b1100);
assign wen13 = (windex_i==6'b1101);
assign wen14 = (windex_i==6'b1110);
assign wen15 = (windex_i==6'b1111);
assign wen16 = (windex_i==6'b10000);
assign wen17 = (windex_i==6'b10001);
assign wen18 = (windex_i==6'b10010);
assign wen19 = (windex_i==6'b10011);
assign wen20 = (windex_i==6'b10100);    
assign wen21 = (windex_i==6'b10101);
assign wen22 = (windex_i==6'b10110);
assign wen23 = (windex_i==6'b10111);
assign wen24 = (windex_i==6'b11000);
assign wen25 = (windex_i==6'b11001);
assign wen26 = (windex_i==6'b11010);
assign wen27 = (windex_i==6'b11011);
assign wen28 = (windex_i==6'b11100);
assign wen29 = (windex_i==6'b11101);
assign wen30 = (windex_i==6'b11110);    
assign wen31 = (windex_i==6'b11111);

always @(posedge clock)     
begin
     if (wen0&wen_i)  begin reg_vpn2[ 0]<=w_vpn2_i; reg_asid[ 0]<=w_asid_i; reg_mask[ 0]<=w_pagemask_i; reg_g[ 0]<=w_g_i; end   
     if (wen1&wen_i)  begin reg_vpn2[ 1]<=w_vpn2_i; reg_asid[ 1]<=w_asid_i; reg_mask[ 1]<=w_pagemask_i; reg_g[ 1]<=w_g_i; end   
     if (wen2&wen_i)  begin reg_vpn2[ 2]<=w_vpn2_i; reg_asid[ 2]<=w_asid_i; reg_mask[ 2]<=w_pagemask_i; reg_g[ 2]<=w_g_i; end   
     if (wen3&wen_i)  begin reg_vpn2[ 3]<=w_vpn2_i; reg_asid[ 3]<=w_asid_i; reg_mask[ 3]<=w_pagemask_i; reg_g[ 3]<=w_g_i; end   
     if (wen4&wen_i)  begin reg_vpn2[ 4]<=w_vpn2_i; reg_asid[ 4]<=w_asid_i; reg_mask[ 4]<=w_pagemask_i; reg_g[ 4]<=w_g_i; end   
     if (wen5&wen_i)  begin reg_vpn2[ 5]<=w_vpn2_i; reg_asid[ 5]<=w_asid_i; reg_mask[ 5]<=w_pagemask_i; reg_g[ 5]<=w_g_i; end   
     if (wen6&wen_i)  begin reg_vpn2[ 6]<=w_vpn2_i; reg_asid[ 6]<=w_asid_i; reg_mask[ 6]<=w_pagemask_i; reg_g[ 6]<=w_g_i; end   
     if (wen7&wen_i)  begin reg_vpn2[ 7]<=w_vpn2_i; reg_asid[ 7]<=w_asid_i; reg_mask[ 7]<=w_pagemask_i; reg_g[ 7]<=w_g_i; end   
     if (wen8&wen_i)  begin reg_vpn2[ 8]<=w_vpn2_i; reg_asid[ 8]<=w_asid_i; reg_mask[ 8]<=w_pagemask_i; reg_g[ 8]<=w_g_i; end   
     if (wen9&wen_i)  begin reg_vpn2[ 9]<=w_vpn2_i; reg_asid[ 9]<=w_asid_i; reg_mask[ 9]<=w_pagemask_i; reg_g[ 9]<=w_g_i; end   
     if (wen10&wen_i) begin reg_vpn2[10]<=w_vpn2_i; reg_asid[10]<=w_asid_i; reg_mask[10]<=w_pagemask_i; reg_g[10]<=w_g_i; end   
     if (wen11&wen_i) begin reg_vpn2[11]<=w_vpn2_i; reg_asid[11]<=w_asid_i; reg_mask[11]<=w_pagemask_i; reg_g[11]<=w_g_i; end   
     if (wen12&wen_i) begin reg_vpn2[12]<=w_vpn2_i; reg_asid[12]<=w_asid_i; reg_mask[12]<=w_pagemask_i; reg_g[12]<=w_g_i; end   
     if (wen13&wen_i) begin reg_vpn2[13]<=w_vpn2_i; reg_asid[13]<=w_asid_i; reg_mask[13]<=w_pagemask_i; reg_g[13]<=w_g_i; end   
     if (wen14&wen_i) begin reg_vpn2[14]<=w_vpn2_i; reg_asid[14]<=w_asid_i; reg_mask[14]<=w_pagemask_i; reg_g[14]<=w_g_i; end   
     if (wen15&wen_i) begin reg_vpn2[15]<=w_vpn2_i; reg_asid[15]<=w_asid_i; reg_mask[15]<=w_pagemask_i; reg_g[15]<=w_g_i; end   
     if (wen16&wen_i) begin reg_vpn2[16]<=w_vpn2_i; reg_asid[16]<=w_asid_i; reg_mask[16]<=w_pagemask_i; reg_g[16]<=w_g_i; end   
     if (wen17&wen_i) begin reg_vpn2[17]<=w_vpn2_i; reg_asid[17]<=w_asid_i; reg_mask[17]<=w_pagemask_i; reg_g[17]<=w_g_i; end   
     if (wen18&wen_i) begin reg_vpn2[18]<=w_vpn2_i; reg_asid[18]<=w_asid_i; reg_mask[18]<=w_pagemask_i; reg_g[18]<=w_g_i; end   
     if (wen19&wen_i) begin reg_vpn2[19]<=w_vpn2_i; reg_asid[19]<=w_asid_i; reg_mask[19]<=w_pagemask_i; reg_g[19]<=w_g_i; end   
     if (wen20&wen_i) begin reg_vpn2[20]<=w_vpn2_i; reg_asid[20]<=w_asid_i; reg_mask[20]<=w_pagemask_i; reg_g[20]<=w_g_i; end   
     if (wen21&wen_i) begin reg_vpn2[21]<=w_vpn2_i; reg_asid[21]<=w_asid_i; reg_mask[21]<=w_pagemask_i; reg_g[21]<=w_g_i; end   
     if (wen22&wen_i) begin reg_vpn2[22]<=w_vpn2_i; reg_asid[22]<=w_asid_i; reg_mask[22]<=w_pagemask_i; reg_g[22]<=w_g_i; end   
     if (wen23&wen_i) begin reg_vpn2[23]<=w_vpn2_i; reg_asid[23]<=w_asid_i; reg_mask[23]<=w_pagemask_i; reg_g[23]<=w_g_i; end   
     if (wen24&wen_i) begin reg_vpn2[24]<=w_vpn2_i; reg_asid[24]<=w_asid_i; reg_mask[24]<=w_pagemask_i; reg_g[24]<=w_g_i; end   
     if (wen25&wen_i) begin reg_vpn2[25]<=w_vpn2_i; reg_asid[25]<=w_asid_i; reg_mask[25]<=w_pagemask_i; reg_g[25]<=w_g_i; end   
     if (wen26&wen_i) begin reg_vpn2[26]<=w_vpn2_i; reg_asid[26]<=w_asid_i; reg_mask[26]<=w_pagemask_i; reg_g[26]<=w_g_i; end   
     if (wen27&wen_i) begin reg_vpn2[27]<=w_vpn2_i; reg_asid[27]<=w_asid_i; reg_mask[27]<=w_pagemask_i; reg_g[27]<=w_g_i; end   
     if (wen28&wen_i) begin reg_vpn2[28]<=w_vpn2_i; reg_asid[28]<=w_asid_i; reg_mask[28]<=w_pagemask_i; reg_g[28]<=w_g_i; end   
     if (wen29&wen_i) begin reg_vpn2[29]<=w_vpn2_i; reg_asid[29]<=w_asid_i; reg_mask[29]<=w_pagemask_i; reg_g[29]<=w_g_i; end   
     if (wen30&wen_i) begin reg_vpn2[30]<=w_vpn2_i; reg_asid[30]<=w_asid_i; reg_mask[30]<=w_pagemask_i; reg_g[30]<=w_g_i; end   
     if (wen31&wen_i) begin reg_vpn2[31]<=w_vpn2_i; reg_asid[31]<=w_asid_i; reg_mask[31]<=w_pagemask_i; reg_g[31]<=w_g_i; end   
end     

assign tlbr_en0  = (rindex_i==6'b0);    
assign tlbr_en1  = (rindex_i==6'b1);
assign tlbr_en2  = (rindex_i==6'b10);
assign tlbr_en3  = (rindex_i==6'b11);
assign tlbr_en4  = (rindex_i==6'b100);
assign tlbr_en5  = (rindex_i==6'b101);
assign tlbr_en6  = (rindex_i==6'b110);
assign tlbr_en7  = (rindex_i==6'b111);
assign tlbr_en8  = (rindex_i==6'b1000);
assign tlbr_en9  = (rindex_i==6'b1001);
assign tlbr_en10 = (rindex_i==6'b1010);
assign tlbr_en11 = (rindex_i==6'b1011);
assign tlbr_en12 = (rindex_i==6'b1100);
assign tlbr_en13 = (rindex_i==6'b1101);
assign tlbr_en14 = (rindex_i==6'b1110);
assign tlbr_en15 = (rindex_i==6'b1111);
assign tlbr_en16 = (rindex_i==6'b10000);
assign tlbr_en17 = (rindex_i==6'b10001);
assign tlbr_en18 = (rindex_i==6'b10010);
assign tlbr_en19 = (rindex_i==6'b10011);
assign tlbr_en20 = (rindex_i==6'b10100);    
assign tlbr_en21 = (rindex_i==6'b10101);
assign tlbr_en22 = (rindex_i==6'b10110);
assign tlbr_en23 = (rindex_i==6'b10111);
assign tlbr_en24 = (rindex_i==6'b11000);
assign tlbr_en25 = (rindex_i==6'b11001);
assign tlbr_en26 = (rindex_i==6'b11010);
assign tlbr_en27 = (rindex_i==6'b11011);
assign tlbr_en28 = (rindex_i==6'b11100);
assign tlbr_en29 = (rindex_i==6'b11101);
assign tlbr_en30 = (rindex_i==6'b11110);    
assign tlbr_en31 = (rindex_i==6'b11111);

assign rden0  = (tlbr_valid_i) ? tlbr_en0  : ren0  ; 
assign rden1  = (tlbr_valid_i) ? tlbr_en1  : ren1  ; 
assign rden2  = (tlbr_valid_i) ? tlbr_en2  : ren2  ; 
assign rden3  = (tlbr_valid_i) ? tlbr_en3  : ren3  ; 
assign rden4  = (tlbr_valid_i) ? tlbr_en4  : ren4  ; 
assign rden5  = (tlbr_valid_i) ? tlbr_en5  : ren5  ; 
assign rden6  = (tlbr_valid_i) ? tlbr_en6  : ren6  ; 
assign rden7  = (tlbr_valid_i) ? tlbr_en7  : ren7  ; 
assign rden8  = (tlbr_valid_i) ? tlbr_en8  : ren8  ; 
assign rden9  = (tlbr_valid_i) ? tlbr_en9  : ren9  ; 
assign rden10 = (tlbr_valid_i) ? tlbr_en10 : ren10 ; 
assign rden11 = (tlbr_valid_i) ? tlbr_en11 : ren11 ; 
assign rden12 = (tlbr_valid_i) ? tlbr_en12 : ren12 ; 
assign rden13 = (tlbr_valid_i) ? tlbr_en13 : ren13 ; 
assign rden14 = (tlbr_valid_i) ? tlbr_en14 : ren14 ; 
assign rden15 = (tlbr_valid_i) ? tlbr_en15 : ren15 ; 
assign rden16 = (tlbr_valid_i) ? tlbr_en16 : ren16 ; 
assign rden17 = (tlbr_valid_i) ? tlbr_en17 : ren17 ; 
assign rden18 = (tlbr_valid_i) ? tlbr_en18 : ren18 ; 
assign rden19 = (tlbr_valid_i) ? tlbr_en19 : ren19 ; 
assign rden20 = (tlbr_valid_i) ? tlbr_en20 : ren20 ;    
assign rden21 = (tlbr_valid_i) ? tlbr_en21 : ren21 ; 
assign rden22 = (tlbr_valid_i) ? tlbr_en22 : ren22 ; 
assign rden23 = (tlbr_valid_i) ? tlbr_en23 : ren23 ; 
assign rden24 = (tlbr_valid_i) ? tlbr_en24 : ren24 ; 
assign rden25 = (tlbr_valid_i) ? tlbr_en25 : ren25 ; 
assign rden26 = (tlbr_valid_i) ? tlbr_en26 : ren26 ; 
assign rden27 = (tlbr_valid_i) ? tlbr_en27 : ren27 ; 
assign rden28 = (tlbr_valid_i) ? tlbr_en28 : ren28 ; 
assign rden29 = (tlbr_valid_i) ? tlbr_en29 : ren29 ; 
assign rden30 = (tlbr_valid_i) ? tlbr_en30 : ren30 ;    
assign rden31 = (tlbr_valid_i) ? tlbr_en31 : ren31 ; 


wire [ 7:0] reg_mask_out = {8{rden0 }}&reg_mask[0 ] | {8{rden1 }}&reg_mask[1 ] |
                           {8{rden2 }}&reg_mask[2 ] | {8{rden3 }}&reg_mask[3 ] |
                           {8{rden4 }}&reg_mask[4 ] | {8{rden5 }}&reg_mask[5 ] |
                           {8{rden6 }}&reg_mask[6 ] | {8{rden7 }}&reg_mask[7 ] |
                           {8{rden8 }}&reg_mask[8 ] | {8{rden9 }}&reg_mask[9 ] |
                           {8{rden10}}&reg_mask[10] | {8{rden11}}&reg_mask[11] |
                           {8{rden12}}&reg_mask[12] | {8{rden13}}&reg_mask[13] |
                           {8{rden14}}&reg_mask[14] | {8{rden15}}&reg_mask[15] |
                           {8{rden16}}&reg_mask[16] | {8{rden17}}&reg_mask[17] |
                           {8{rden18}}&reg_mask[18] | {8{rden19}}&reg_mask[19] |
                           {8{rden20}}&reg_mask[20] | {8{rden21}}&reg_mask[21] |
                           {8{rden22}}&reg_mask[22] | {8{rden23}}&reg_mask[23] |
                           {8{rden24}}&reg_mask[24] | {8{rden25}}&reg_mask[25] |
                           {8{rden26}}&reg_mask[26] | {8{rden27}}&reg_mask[27] |
                           {8{rden28}}&reg_mask[28] | {8{rden29}}&reg_mask[29] |
                           {8{rden30}}&reg_mask[30] | {8{rden31}}&reg_mask[31] ;

assign pagemask_out_o = reg_mask_out;

wire [ 7:0] reg_asid_out = {8{rden0 }}&reg_asid[0 ] | {8{rden1 }}&reg_asid[1 ] |
                           {8{rden2 }}&reg_asid[2 ] | {8{rden3 }}&reg_asid[3 ] |
                           {8{rden4 }}&reg_asid[4 ] | {8{rden5 }}&reg_asid[5 ] |
                           {8{rden6 }}&reg_asid[6 ] | {8{rden7 }}&reg_asid[7 ] |
                           {8{rden8 }}&reg_asid[8 ] | {8{rden9 }}&reg_asid[9 ] |
                           {8{rden10}}&reg_asid[10] | {8{rden11}}&reg_asid[11] |
                           {8{rden12}}&reg_asid[12] | {8{rden13}}&reg_asid[13] |
                           {8{rden14}}&reg_asid[14] | {8{rden15}}&reg_asid[15] |
                           {8{rden16}}&reg_asid[16] | {8{rden17}}&reg_asid[17] |
                           {8{rden18}}&reg_asid[18] | {8{rden19}}&reg_asid[19] |
                           {8{rden20}}&reg_asid[20] | {8{rden21}}&reg_asid[21] |
                           {8{rden22}}&reg_asid[22] | {8{rden23}}&reg_asid[23] |
                           {8{rden24}}&reg_asid[24] | {8{rden25}}&reg_asid[25] |
                           {8{rden26}}&reg_asid[26] | {8{rden27}}&reg_asid[27] |
                           {8{rden28}}&reg_asid[28] | {8{rden29}}&reg_asid[29] |
                           {8{rden30}}&reg_asid[30] | {8{rden31}}&reg_asid[31] ;

assign asid_out_o = reg_asid_out;

wire reg_g_out = {{rden0 }}&reg_g[0 ] | {{rden1 }}&reg_g[1 ] |
                 {{rden2 }}&reg_g[2 ] | {{rden3 }}&reg_g[3 ] |
                 {{rden4 }}&reg_g[4 ] | {{rden5 }}&reg_g[5 ] |
                 {{rden6 }}&reg_g[6 ] | {{rden7 }}&reg_g[7 ] |
                 {{rden8 }}&reg_g[8 ] | {{rden9 }}&reg_g[9 ] |
                 {{rden10}}&reg_g[10] | {{rden11}}&reg_g[11] |
                 {{rden12}}&reg_g[12] | {{rden13}}&reg_g[13] |
                 {{rden14}}&reg_g[14] | {{rden15}}&reg_g[15] |
                 {{rden16}}&reg_g[16] | {{rden17}}&reg_g[17] |
                 {{rden18}}&reg_g[18] | {{rden19}}&reg_g[19] |
                 {{rden20}}&reg_g[20] | {{rden21}}&reg_g[21] |
                 {{rden22}}&reg_g[22] | {{rden23}}&reg_g[23] |
                 {{rden24}}&reg_g[24] | {{rden25}}&reg_g[25] |
                 {{rden26}}&reg_g[26] | {{rden27}}&reg_g[27] |
                 {{rden28}}&reg_g[28] | {{rden29}}&reg_g[29] |
                 {{rden30}}&reg_g[30] | {{rden31}}&reg_g[31] ;

assign g_out_o = reg_g_out;

wire [18:0] reg_vpn2_out = {19{tlbr_en0 }}&reg_vpn2[0 ] | {19{tlbr_en1 }}&reg_vpn2[1 ] |
                           {19{tlbr_en2 }}&reg_vpn2[2 ] | {19{tlbr_en3 }}&reg_vpn2[3 ] |
                           {19{tlbr_en4 }}&reg_vpn2[4 ] | {19{tlbr_en5 }}&reg_vpn2[5 ] |
                           {19{tlbr_en6 }}&reg_vpn2[6 ] | {19{tlbr_en7 }}&reg_vpn2[7 ] |
                           {19{tlbr_en8 }}&reg_vpn2[8 ] | {19{tlbr_en9 }}&reg_vpn2[9 ] |
                           {19{tlbr_en10}}&reg_vpn2[10] | {19{tlbr_en11}}&reg_vpn2[11] |
                           {19{tlbr_en12}}&reg_vpn2[12] | {19{tlbr_en13}}&reg_vpn2[13] |
                           {19{tlbr_en14}}&reg_vpn2[14] | {19{tlbr_en15}}&reg_vpn2[15] |
                           {19{tlbr_en16}}&reg_vpn2[16] | {19{tlbr_en17}}&reg_vpn2[17] |
                           {19{tlbr_en18}}&reg_vpn2[18] | {19{tlbr_en19}}&reg_vpn2[19] |
                           {19{tlbr_en20}}&reg_vpn2[20] | {19{tlbr_en21}}&reg_vpn2[21] |
                           {19{tlbr_en22}}&reg_vpn2[22] | {19{tlbr_en23}}&reg_vpn2[23] |
                           {19{tlbr_en24}}&reg_vpn2[24] | {19{tlbr_en25}}&reg_vpn2[25] |
                           {19{tlbr_en26}}&reg_vpn2[26] | {19{tlbr_en27}}&reg_vpn2[27] |
                           {19{tlbr_en28}}&reg_vpn2[28] | {19{tlbr_en29}}&reg_vpn2[29] |
                           {19{tlbr_en30}}&reg_vpn2[30] | {19{tlbr_en31}}&reg_vpn2[31] ;

assign vpn2_out_o = reg_vpn2_out;

endmodule //tlb_cam



module micro_dtlb(
    clock,
    asid,
    data_vpn,
    
    wren,
    wr_index,
    wr_vpn,
    wr_mask,
    wr_asid,
    wr_g,
    wr_pfn,
    
    dtlb_flush_i,
    
    d_find,
    d_find_index,

    select_index,
    dpagemask_o,
    out_pfn
);
input         clock;
input  [7:0]  asid;
input  [19:0] data_vpn;

input         wren;
input  [2:0]  wr_index;
input  [19:0] wr_vpn;
input  [7:0]  wr_mask;
input  [7:0]  wr_asid;
input         wr_g;
input  [31:0] wr_pfn;

input         dtlb_flush_i;

output        d_find;
output [ 2:0] d_find_index;

input  [ 2:0] select_index;
output [ 7:0] dpagemask_o;
output [31:0] out_pfn;

wire drden0, drden1, drden2, drden3;
wire drden4, drden5, drden6, drden7;

reg  [19:0] reg_vpn[7:0];
reg  [7:0]  reg_asid[7:0];
reg  [7:0]  reg_mask[7:0];
reg  [7:0]  reg_g;
reg  [7:0]  reg_valid;
reg  [25:0] reg_pfn[7:0];

wire [25:0] reg_pfn_in = {wr_pfn[30], wr_pfn[25:1]};

wire [15:0] reg_mask_0;
wire [15:0] reg_mask_1;
wire [15:0] reg_mask_2;
wire [15:0] reg_mask_3;
wire [15:0] reg_mask_4;
wire [15:0] reg_mask_5;
wire [15:0] reg_mask_6;
wire [15:0] reg_mask_7;

assign reg_mask_0  = {reg_mask[ 0][7], reg_mask[ 0][7], reg_mask[ 0][6], reg_mask[ 0][6],
                      reg_mask[ 0][5], reg_mask[ 0][5], reg_mask[ 0][4], reg_mask[ 0][4],
                      reg_mask[ 0][3], reg_mask[ 0][3], reg_mask[ 0][2], reg_mask[ 0][2],
                      reg_mask[ 0][1], reg_mask[ 0][1], reg_mask[ 0][0], reg_mask[ 0][0]};

assign reg_mask_1  = {reg_mask[ 1][7], reg_mask[ 1][7], reg_mask[ 1][6], reg_mask[ 1][6],
                      reg_mask[ 1][5], reg_mask[ 1][5], reg_mask[ 1][4], reg_mask[ 1][4],
                      reg_mask[ 1][3], reg_mask[ 1][3], reg_mask[ 1][2], reg_mask[ 1][2],
                      reg_mask[ 1][1], reg_mask[ 1][1], reg_mask[ 1][0], reg_mask[ 1][0]};

assign reg_mask_2  = {reg_mask[ 2][7], reg_mask[ 2][7], reg_mask[ 2][6], reg_mask[ 2][6],
                      reg_mask[ 2][5], reg_mask[ 2][5], reg_mask[ 2][4], reg_mask[ 2][4],
                      reg_mask[ 2][3], reg_mask[ 2][3], reg_mask[ 2][2], reg_mask[ 2][2],
                      reg_mask[ 2][1], reg_mask[ 2][1], reg_mask[ 2][0], reg_mask[ 2][0]};

assign reg_mask_3  = {reg_mask[ 3][7], reg_mask[ 3][7], reg_mask[ 3][6], reg_mask[ 3][6],
                      reg_mask[ 3][5], reg_mask[ 3][5], reg_mask[ 3][4], reg_mask[ 3][4],
                      reg_mask[ 3][3], reg_mask[ 3][3], reg_mask[ 3][2], reg_mask[ 3][2],
                      reg_mask[ 3][1], reg_mask[ 3][1], reg_mask[ 3][0], reg_mask[ 3][0]};

assign reg_mask_4  = {reg_mask[ 4][7], reg_mask[ 4][7], reg_mask[ 4][6], reg_mask[ 4][6],
                      reg_mask[ 4][5], reg_mask[ 4][5], reg_mask[ 4][4], reg_mask[ 4][4],
                      reg_mask[ 4][3], reg_mask[ 4][3], reg_mask[ 4][2], reg_mask[ 4][2],
                      reg_mask[ 4][1], reg_mask[ 4][1], reg_mask[ 4][0], reg_mask[ 4][0]};

assign reg_mask_5  = {reg_mask[ 5][7], reg_mask[ 5][7], reg_mask[ 5][6], reg_mask[ 5][6],
                      reg_mask[ 5][5], reg_mask[ 5][5], reg_mask[ 5][4], reg_mask[ 5][4],
                      reg_mask[ 5][3], reg_mask[ 5][3], reg_mask[ 5][2], reg_mask[ 5][2],
                      reg_mask[ 5][1], reg_mask[ 5][1], reg_mask[ 5][0], reg_mask[ 5][0]};

assign reg_mask_6  = {reg_mask[ 6][7], reg_mask[ 6][7], reg_mask[ 6][6], reg_mask[ 6][6],
                      reg_mask[ 6][5], reg_mask[ 6][5], reg_mask[ 6][4], reg_mask[ 6][4],
                      reg_mask[ 6][3], reg_mask[ 6][3], reg_mask[ 6][2], reg_mask[ 6][2],
                      reg_mask[ 6][1], reg_mask[ 6][1], reg_mask[ 6][0], reg_mask[ 6][0]};

assign reg_mask_7  = {reg_mask[ 7][7], reg_mask[ 7][7], reg_mask[ 7][6], reg_mask[ 7][6],
                      reg_mask[ 7][5], reg_mask[ 7][5], reg_mask[ 7][4], reg_mask[ 7][4],
                      reg_mask[ 7][3], reg_mask[ 7][3], reg_mask[ 7][2], reg_mask[ 7][2],
                      reg_mask[ 7][1], reg_mask[ 7][1], reg_mask[ 7][0], reg_mask[ 7][0]};

wire [7:0]  asid_match;
wire [2:0]  pfn_index;

assign asid_match[0] = (reg_asid[0]==asid) | reg_g[0];
assign asid_match[1] = (reg_asid[1]==asid) | reg_g[1];
assign asid_match[2] = (reg_asid[2]==asid) | reg_g[2];
assign asid_match[3] = (reg_asid[3]==asid) | reg_g[3];
assign asid_match[4] = (reg_asid[4]==asid) | reg_g[4];
assign asid_match[5] = (reg_asid[5]==asid) | reg_g[5];
assign asid_match[6] = (reg_asid[6]==asid) | reg_g[6];
assign asid_match[7] = (reg_asid[7]==asid) | reg_g[7];


/*
assign drden0  = (({4'b1111, ~reg_mask[0]}&data_vpn )==({4'b1111, ~reg_mask[0]}&reg_vpn[0] ))&asid_match[0]&reg_valid[0];
assign drden1  = (({4'b1111, ~reg_mask[1]}&data_vpn )==({4'b1111, ~reg_mask[1]}&reg_vpn[1] ))&asid_match[1]&reg_valid[1];
assign drden2  = (({4'b1111, ~reg_mask[2]}&data_vpn )==({4'b1111, ~reg_mask[2]}&reg_vpn[2] ))&asid_match[2]&reg_valid[2];
assign drden3  = (({4'b1111, ~reg_mask[3]}&data_vpn )==({4'b1111, ~reg_mask[3]}&reg_vpn[3] ))&asid_match[3]&reg_valid[3];
assign drden4  = (({4'b1111, ~reg_mask[4]}&data_vpn )==({4'b1111, ~reg_mask[4]}&reg_vpn[4] ))&asid_match[4]&reg_valid[4];
assign drden5  = (({4'b1111, ~reg_mask[5]}&data_vpn )==({4'b1111, ~reg_mask[5]}&reg_vpn[5] ))&asid_match[5]&reg_valid[5];
assign drden6  = (({4'b1111, ~reg_mask[6]}&data_vpn )==({4'b1111, ~reg_mask[6]}&reg_vpn[6] ))&asid_match[6]&reg_valid[6];
assign drden7  = (({4'b1111, ~reg_mask[7]}&data_vpn )==({4'b1111, ~reg_mask[7]}&reg_vpn[7] ))&asid_match[7]&reg_valid[7];
*/
assign drden0  = (({4'b1111, ~reg_mask_0}&data_vpn )==reg_vpn[0]) & asid_match[0]&reg_valid[0];
assign drden1  = (({4'b1111, ~reg_mask_1}&data_vpn )==reg_vpn[1]) & asid_match[1]&reg_valid[1];
assign drden2  = (({4'b1111, ~reg_mask_2}&data_vpn )==reg_vpn[2]) & asid_match[2]&reg_valid[2];
assign drden3  = (({4'b1111, ~reg_mask_3}&data_vpn )==reg_vpn[3]) & asid_match[3]&reg_valid[3];
assign drden4  = (({4'b1111, ~reg_mask_4}&data_vpn )==reg_vpn[4]) & asid_match[4]&reg_valid[4];
assign drden5  = (({4'b1111, ~reg_mask_5}&data_vpn )==reg_vpn[5]) & asid_match[5]&reg_valid[5];
assign drden6  = (({4'b1111, ~reg_mask_6}&data_vpn )==reg_vpn[6]) & asid_match[6]&reg_valid[6];
assign drden7  = (({4'b1111, ~reg_mask_7}&data_vpn )==reg_vpn[7]) & asid_match[7]&reg_valid[7];

assign d_find = drden0 | drden1 | drden2 | drden3 |
                drden4 | drden5 | drden6 | drden7 ;
assign d_find_index = ({3{drden0}}&3'b000) | ({3{drden1}}&3'b001)|
                      ({3{drden2}}&3'b010) | ({3{drden3}}&3'b011)|
                      ({3{drden4}}&3'b100) | ({3{drden5}}&3'b101)|
                      ({3{drden6}}&3'b110) | ({3{drden7}}&3'b111);

wire [25:0] reg_pfn_out = reg_pfn[select_index];
assign out_pfn = {1'b0, reg_pfn_out[25], 4'b0, reg_pfn_out[24:0], 1'b0};

wire [7:0] mask_out = reg_mask[select_index];
assign dpagemask_o = mask_out;

wire wren0 = (wr_index == 3'b000);
wire wren1 = (wr_index == 3'b001);
wire wren2 = (wr_index == 3'b010);
wire wren3 = (wr_index == 3'b011);
wire wren4 = (wr_index == 3'b100);
wire wren5 = (wr_index == 3'b101);
wire wren6 = (wr_index == 3'b110);
wire wren7 = (wr_index == 3'b111);

wire [7:0] wren_tmp;
assign wren_tmp[0] = wren0;
assign wren_tmp[1] = wren1;
assign wren_tmp[2] = wren2;
assign wren_tmp[3] = wren3;
assign wren_tmp[4] = wren4;
assign wren_tmp[5] = wren5;
assign wren_tmp[6] = wren6;
assign wren_tmp[7] = wren7;



wire flush0 = dtlb_flush_i;
wire flush1 = dtlb_flush_i;
wire flush2 = dtlb_flush_i;
wire flush3 = dtlb_flush_i;
wire flush4 = dtlb_flush_i;
wire flush5 = dtlb_flush_i;
wire flush6 = dtlb_flush_i;
wire flush7 = dtlb_flush_i;

wire [7:0] reg_valid_value_new = {8{~dtlb_flush_i}} & (wren_tmp&{8{wren}});
wire [7:0] reg_in_en = {8{dtlb_flush_i}} | (wren_tmp&{8{wren}});
wire [7:0] reg_valid_value_in;
assign reg_valid_value_in = (reg_in_en & reg_valid_value_new) | (~reg_in_en & reg_valid);

always @(posedge clock)
begin
    reg_valid <= reg_valid_value_in;
end

always @(posedge clock)
begin
     if (wren0&wren)  
     begin 
        reg_vpn[0]<=wr_vpn; reg_asid[0]<=wr_asid; reg_mask[0]<=wr_mask; reg_g[0]<=wr_g; 
        reg_pfn[0]<=reg_pfn_in;
     end 

     if (wren1&wren)  
     begin 
        reg_vpn[1]<=wr_vpn; reg_asid[1]<=wr_asid; reg_mask[1]<=wr_mask; reg_g[1]<=wr_g; 
        reg_pfn[1]<=reg_pfn_in;
     end   

     if (wren2&wren)  
     begin 
        reg_vpn[2]<=wr_vpn; reg_asid[2]<=wr_asid; reg_mask[2]<=wr_mask; reg_g[2]<=wr_g; 
        reg_pfn[2]<=reg_pfn_in;
     end   

     if (wren3&wren)  
     begin 
        reg_vpn[3]<=wr_vpn; reg_asid[3]<=wr_asid; reg_mask[3]<=wr_mask; reg_g[3]<=wr_g; 
        reg_pfn[3]<=reg_pfn_in;
     end   

     if (wren4&wren)  
     begin 
        reg_vpn[4]<=wr_vpn; reg_asid[4]<=wr_asid; reg_mask[4]<=wr_mask; reg_g[4]<=wr_g; 
        reg_pfn[4]<=reg_pfn_in;
     end 

     if (wren5&wren)  
     begin 
        reg_vpn[5]<=wr_vpn; reg_asid[5]<=wr_asid; reg_mask[5]<=wr_mask; reg_g[5]<=wr_g; 
        reg_pfn[5]<=reg_pfn_in;
     end   

     if (wren6&wren)  
     begin 
        reg_vpn[6]<=wr_vpn; reg_asid[6]<=wr_asid; reg_mask[6]<=wr_mask; reg_g[6]<=wr_g; 
        reg_pfn[6]<=reg_pfn_in;
     end   

     if (wren7&wren)  
     begin 
        reg_vpn[7]<=wr_vpn; reg_asid[7]<=wr_asid; reg_mask[7]<=wr_mask; reg_g[7]<=wr_g; 
        reg_pfn[7]<=reg_pfn_in;
     end   
end


endmodule //micro_dtlb

module mux32b_7_1 (in1,in2,in3,in4,in5,in6,in7,sel1,sel2,sel3,sel4,sel5,sel6,sel7,out);
input  [31:0] in1,in2,in3,in4,in5,in6,in7;
input         sel1,sel2,sel3,sel4,sel5,sel6,sel7;
output [31:0] out;
assign out=({32{sel1}}&in1)|({32{sel2}}&in2)|({32{sel3}}&in3)|
           ({32{sel4}}&in4)|({32{sel5}}&in5)|({32{sel6}}&in6) | ({32{sel7}}&in7);
endmodule 
