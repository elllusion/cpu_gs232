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

module godson_cpu_core(
    coreclock,
    core_rst_,
    interrupt_i,nmi,
    
    aclk, areset_n,
    
    arid, araddr, arlen, arsize, arburst, arlock,
    arcache, arprot, arvalid, arready,
    
    rid, rdata, rresp, rlast, rvalid, rready,
    
    awid, awaddr, awlen, awsize, awburst, awlock,
    awcache, awprot, awvalid, awready,
    
    wid, wdata, wstrb, wlast, wvalid, wready,
    
    bid, bresp, bvalid, bready,
    
    ram_to_tlb,tlb_to_ram,
    ram_to_icache,icache_to_ram,
    ram_to_dcache,dcache_to_ram,
    EJTAG_TDI,EJTAG_TDO,EJTAG_TMS,EJTAG_TCK,EJTAG_TRST,prrst_to_core,
    testmode
);
input          coreclock;
output         core_rst_;
input [4:0]    interrupt_i;
input          nmi;
input          testmode;

//global
input           aclk;
input           areset_n;

//read address channel
input           arready;
output  [3:0]   arid;
output  [31:0]  araddr;
output  [3:0]   arlen;
output  [2:0]   arsize;
output  [1:0]   arburst;
output  [1:0]   arlock;
output  [3:0]   arcache;
output  [2:0]   arprot;
output          arvalid;

//read data channel
input   [3:0]   rid;
input   [31:0]  rdata;
input   [1:0]   rresp;
input           rlast;
input           rvalid;
output          rready;

//write address channel
input           awready;
output  [3:0]   awid;
output  [31:0]  awaddr;
output  [3:0]   awlen;
output  [2:0]   awsize;
output  [1:0]   awburst;
output  [1:0]   awlock;
output  [3:0]   awcache;
output  [2:0]   awprot;
output          awvalid;

//write data channel
input           wready;
output  [3:0]   wid;
output  [31:0]  wdata;
output  [3:0]   wstrb;
output          wlast;
output          wvalid;

//write response channel
input   [3:0]   bid;
input   [1:0]   bresp;
input           bvalid;
output          bready;

input  [`Lram_to_tlb-1   :0] ram_to_tlb;
output [`Ltlb_to_ram-1   :0] tlb_to_ram;
input  [`Lram_to_icache-1:0] ram_to_icache;
output [`Licache_to_ram-1:0] icache_to_ram;
output [`Ldcache_to_ram-1:0] dcache_to_ram;
input  [`Lram_to_dcache-1:0] ram_to_dcache;
input  EJTAG_TDI,EJTAG_TMS,EJTAG_TRST,EJTAG_TCK;
output EJTAG_TDO;
output prrst_to_core ;

wire [31:0] data_from_tap;
wire        pracc_from_tap;
wire        prrst_from_tap;
wire        proben_from_tap;
wire        trap_from_tap;
wire        ejtagbrk_from_tap;
wire [31:0] addr_to_tap;
wire [31:0] data_to_tap;
wire [1:0]  width_to_tap;
wire        write_to_tap;
wire        pracc_to_tap;
wire        ejtagbrk_to_tap;
wire        reset_to_tap;
wire        debugmode_to_tap;

wire mm_ok, falu_ok;
wire [`Lmemaddr-1:0] memaddr_to_io;
wire [1:0] ksu;
wire [3:0] cu;    
wire nmi_a;
wire[6:0]exp;       
wire interrupt;
wire [5:0]int_in_a;     
wire valin0;
wire valin1;
wire coreclock;
wire reset;

wire [`Lirbus_2issue-1:0] irbus;

wire [`Ldecbus_2issue-1:0] decbus;

wire [`Lqissuebus-1:0]  qissuebus0;
wire [`Lrissuebus0-1:0] rissuebus0;
wire [`Lqissuebus-1:0] qissuebus1;
wire [`Lrissuebus1-1:0] rissuebus1;
//wire [`Lresbus-1:0] mmres,mmres_h;
wire [`Lresbus3-1:0] resbus3;
wire [`Lresbus1-1:0] resbus1;
wire [`Lresbus2-1:0] resbus2;
wire [`Lresbus0-1:0] resbus0;
wire [31:0] alu_res;

assign core_rst_ = ~reset;

wire [`Lcommitbus-1:0] commitbus0, commitbus1;

wire [31:0] alures_jalr_target1;
wire [31:0] alures_jalr_target2;

wire [31:0] DSPCtl_value ;
wire [45:0] instout;
wire [`Liresbus_2issue-1:0] iresbus;

wire qstalli;
wire icache_stalli, icache_refill_ok;
wire[1:0] qfull ;
wire      brq_full ;
wire      brq_tail_next_valid ;
wire mmfull,alursfull1,alursfull2,alu2rsfull_tmp,mulfull,falufull,fmulfull;
wire mmbusrep,alubusrep,mulbusrep,falubusrep,fmulbusrep;
wire mmbusrep_h,mulbusrep_h,falubusrep_h,fmulbusrep_h;
wire [31:0] fcr31;
wire exl,erl,bev;
wire [8:0] vector_int;
wire [31:0]ebase;
wire [3:0] HWREna;
wire cp0_mod_ok;
wire [3:0] cp0_qid;
wire [1:0] store_ok;
wire [7:0] store_qid;
wire [3:0] acc_qid;
wire [18:0] entryhi_vpn2;
wire [1:0] addrout_bank_index;
wire addrout_cache_inst;
wire acc_write_ok;

wire [`Lrissuebus_to_fix-1:0] rissuebus_to_fix1;
wire [`Lrissuebus_to_fix-1:0] rissuebus_to_fix2;
wire [`Lrissuebus_to_float-1:0] rissuebus_to_float;
wire [`Lrissuebus_to_mm-1:0] rissuebus_to_mm;

wire [`Lcommitbus_to_fetch-1:0] commitbus_to_fetch;
wire [`Lcommitbus_to_gr-1:0] commitbus0_to_gr, commitbus1_to_gr;
wire [`Lcommitbus_to_fr-1:0] commitbus0_to_fr;
wire [`Lcommitbus_to_fr-1:0] commitbus1_to_fr;
wire [`Lcommitbus_to_tlb-1:0] commitbus0_to_tlb;
wire [`Lcommitbus_to_tlb-1:0] commitbus1_to_tlb;
wire [`Lcommitbus_to_hb-1:0] commitbus_to_hb;
wire [`Lcommitbus_to_itlb-1:0] commitbus_to_itlb;


wire commitbus_ex;
wire [`Lqissuebus0_to_gr-1:0] qissuebus0_to_gr;
wire [`Lqissuebus1_to_gr-1:0] qissuebus1_to_gr;
wire [15:0] qissuebus0_to_fr;

wire [`Lmemres-1:0] memres, memres_cache;
wire [`Lmemraddr-1:0] memraddr;
wire [`Lmemwaddr-1:0] memwaddr;

wire [103:0] memres_to_io;
wire [`Lmemraddr-1:0] memraddr_to_io;
wire [`Lmemwaddr-1:0] memwaddr_to_io;

wire [`Ldaddrbus-1:0] addrout;
wire [`Ldresbus-1:0] dresbus;     
wire [`Licachepaddr-1:0] icachepaddr;  
wire [13:0] d_laddr;
 
wire rsth;
wire coldrsth;

wire [31:0]   haddr;
wire [2:0]    hburst;
wire          hbusreq;
wire          hgrant;
wire          hlock;
wire [3:0]    hprot;
wire [31:0]   hrdata;
wire          hready;
wire [1:0]    hresp;
wire [2:0]    hsize;
wire [1:0]    htrans;
wire [31:0]   hwdata;
wire          hwrite;

wire        softreset;

wire allowin_dpaddrbuf,mmresbuf_free,dpaddrbuf_ex,conflict_delay;
wire mmqueue_free,mmresbuf_ex,allowin_mmqueue;

wire probtrap;
wire nodcr;
wire proben;
wire [`Liresbus-1:0] iresbus_from_dmseg;
wire debug_mode;
wire [`DBKP_NUM-1:0] hb_dbs_bs;
wire dss_enable;
wire ir_wait_bd;
wire ejtagbrk;
wire inte;
wire [64:0] dresbus_from_dmseg,dresbus_from_dcr,dresbus_from_drseg;
wire ibe_from_cache;
wire hb_ddbl,hb_ddbs,hb_ddblimpr,hb_ddbsimpr;
wire[1:0] hb_dib;
wire dmseg_dfree;
wire iexi;
wire pending_ibe,pending_dbe;
wire[1:0] icacherdy_after_deret;
wire first_mem_pass_after_deret;
wire dint;
wire [`Ldmseg_dreqbus-1:0] dmseg_dreqbus;
wire [`Ldmseg_ireqbus-1:0] dmseg_ireqbus;
wire [`Ldcr_reqbus-1:0] dcr_reqbus;
wire [`Lhb_reqbus-1:0] hb_reqbus;
wire [`Lhb_dcompbus-1:0] hb_dcompbus;
wire [`Lhb_icompbus-1:0] hb_icompbus;
wire [31:0] loaddata;
wire [ 3:0] loaddata_qid;
wire loaddata_valid;
wire [31:0] loaddata_h;
wire loaddata_h_valid;
wire [ 3:0] bytelane;
wire nmi_from_dcr;
wire proben_to_core;
wire ejtagboot;

wire[31:0] tx_data_from_tap;
wire       tr_from_tap;
wire       tr_to_tap;
wire       tr_to_core;

wire        brbus_deret_err;
wire [15:0] offset_to_mm;
wire [97:0] brbus;
wire [18:0]  brbus_to_cp0;
wire [47:0] brbus_to_hb;
wire [6:0] brbus_to_rs;
wire [6:0] brbus_to_fu;
wire [1:0]  brbus_to_icache;
wire [1:0]  hb_load_addr_match;
wire hb_load_value_compare;

wire DERET_IFLAG;

wire [17:0] qissuebus0_from_fpq;
wire [75:0] commitbus_from_fpq_0;
wire [75:0] commitbus_from_fpq_1; 


godson_qissuebus_module qissuebus_m(.qissuebus0(qissuebus0),
    .qissuebus1(qissuebus1),
    .qissuebus0_to_gr(qissuebus0_to_gr)
    ,.qissuebus1_to_gr(qissuebus1_to_gr)
    );
    
godson_commitbus_module commitbus_m(
    .commitbus0(commitbus0),
    .commitbus0_to_gr(commitbus0_to_gr),
    
    .commitbus1(commitbus1),
    .commitbus1_to_gr(commitbus1_to_gr),
    
    .commitbus_to_fetch(commitbus_to_fetch),
    .commitbus0_to_tlb(commitbus0_to_tlb),
    
    .commitbus1_to_tlb(commitbus1_to_tlb),
    
    .commitbus_to_hb(commitbus_to_hb),
    .commitbus_to_itlb(commitbus_to_itlb),
    .commitbus_ex_out(commitbus_ex));

godson_rissuebus_module rissuebus_m(.rissuebus0({rissuebus0[122:93], qissuebus0[100], rissuebus0[91:0]}),
    .rissuebus1({rissuebus1[99:93], qissuebus1[100], rissuebus1[91:0]}),
    .offset_to_mm(offset_to_mm),
    .rissuebus_to_fix1(rissuebus_to_fix1),
    .rissuebus_to_fix2(rissuebus_to_fix2),
    .rissuebus_to_mm(rissuebus_to_mm),
    .mm_ok(mm_ok),
    .falu_ok(falu_ok));

godson_brbus_module brbus_module(.brbus({brbus[97:96],brbus[93:0]}), 
                                  .brbus_to_rs(brbus_to_rs), 
                                  .brbus_to_fu(brbus_to_fu), 
                                  .brbus_to_icache(brbus_to_icache), 
                                  .brbus_to_cp0(brbus_to_cp0),
                                  .brbus_to_hb(brbus_to_hb));

wire irfull;//for performance counter
wire icache_hit_perf, icache_access;
wire icache_update, icache_way_hit;
wire dcache_hit, dcache_access;
wire itlb_access;
wire dtlb_miss, dtlb_access;
wire missq_full ;
wire not_store_ok;
wire st_ld_conflict;

wire           flush_pipeline_cycle;
wire           inst_to_queue_cycle;
wire           insts_to_alu1;
wire           insts_to_alu2;
wire           insts_to_addr;
wire           insts_to_falu;
wire[1:0]      insts_to_queue;
wire[1:0]      insts_to_fetched;
wire           stalled_cycles_icachemiss;
wire           itlbmiss_tlbhit;

wire  imemread_valid;
wire  dmemread_valid;
wire  duncache_valid;
wire  mreadreq_valid; 
wire  mwritereq_valid; 
wire  data_inst_conflict;
wire  data_inter_conflict;

assign  mreadreq_valid     = 1'b0; 
assign  data_inst_conflict = 1'b0;

wire[49:0] way_pred; //for_way_prediction
wire[10:0] src3_to_alu;
wire       irstalli_r;
wire[8:0]  cp0_forward_bus;
wire[4:0]  cp0_cancel_bus;
wire[2:0]  cr_cfg6_brpred_type;
wire       cr_cfg6_rti;
wire       cr_cfg6_cache0_all;
wire       mode_user;
wire       int_trigger;
wire       pc_en_nowayhit;
wire[1:0]  cr_cfg7_dcache;
wire[1:0]  cr_cfg7_icache;
wire qissuebus0_src1_rdy;

godson_fetch_module  fetch(.clock(coreclock),.reset(reset),
    .commitbus(commitbus_to_fetch),
    .qfull(qfull),.qstalli(qstalli),.brq_full(brq_full),.brq_tail_next_valid(brq_tail_next_valid),
    .exl(exl),.bev(bev),
    .vector_int(vector_int),
    .ebase(ebase),
    .cr_config6(cr_cfg6_brpred_type),
    .user_mode(mode_user), 
    .int_trigger(int_trigger),//for wait inst
    .icache_stalli(icache_stalli),.icache_refill_ok(icache_refill_ok),
    .insts_fetched_o(insts_to_fetched),
    .PROBTRAP(probtrap),
    .NODCR(nodcr),
    .PROBEN(proben),
    .IRESBUS_FROM_CACHE(iresbus),
    .IRESBUS_FROM_DMSEG(iresbus_from_dmseg),
    .DEBUG_MODE(debug_mode),
    .DSS_ENABLE(dss_enable),
    .IR_WAIT_BD(ir_wait_bd),
    .EJTAGBOOT(ejtagboot),
    .DERET_IFLAG(DERET_IFLAG), 
    //.DERET_IFLAG(1'b1),  //for single issue test, by xucp
    .instout(instout),
    .irbus(irbus),
    .irfull(irfull),
    .way_pred(way_pred),
    .irstalli_r(irstalli_r),
    .pc_en_nowayhit(pc_en_nowayhit),
    .brbus(brbus[95:0]));
           
godson_decoder_module decode(.reset(reset),.exl(exl),.erl(erl),.ksu(ksu),.cu(cu),.commitbus_ex(commitbus_ex),
    .HWREna(HWREna),.entryhi_vpn2(entryhi_vpn2),
    .irbus(irbus),
    .DEBUG_MODE(debug_mode),
    .decbus(decbus)
    );


wire [4:0] issue_to_mmrs_qj;     
wire fpqfull;
wire [104:0] resbus0_to_fpq;
wire [76:0] resbus2_to_fpq;
wire [49:0] queue_to_fpq;
wire [5:0] brq_brbus_vector;
wire [2:0] commitbus_to_fpq_0;
wire [2:0] commitbus_to_fpq_1;
wire nmi_2r;
wire int_2r;
godson_queue_module queue(.clock(coreclock),.reset(reset),
    .decbus(decbus),
    .resbus0(resbus0),
    .resbus3(resbus3),
    .resbus1(resbus1),
    .alures_jalr_target1(alures_jalr_target1),
    .insts_to_alu1_o(insts_to_alu1), //from_hsq
    .resbus2(resbus2),

    .alu_res(alu_res),
    .alures_jalr_target2(alures_jalr_target2),
    
    .cp0_qid(cp0_qid),
    .store_qid(store_qid),
    .mmfull(mmfull),.alu2full(alu2rsfull_tmp),.falufull(falufull),
    .DSS_ENABLE(dss_enable),
    .EJTAGBRK(ejtagbrk),
    .DEBUG_MODE(debug_mode),
    .INTE(inte),
    .fcr31(fcr31),
    .softreset(softreset),.nmi(nmi_from_dcr),.interrupt(interrupt),
    .cr_cfg6_rti_i(cr_cfg6_rti),
    .cp0_forward_bus_i(cp0_forward_bus), 
    .cp0_cancel_bus_i(cp0_cancel_bus),
    .qissuebus0(qissuebus0),
    
    .qissuebus1(qissuebus1),
    
    .qissuebus0_src1_rdy(qissuebus0_src1_rdy),
    .issue_to_mmrs_qj(issue_to_mmrs_qj),
    .commitbus0(commitbus0),
    
    .commitbus1(commitbus1),
    
    .qfull(qfull),.qstalli(qstalli),.brq_full(brq_full),.brq_tail_next_valid(brq_tail_next_valid),
    .EJTAGBOOT(ejtagboot),
    .store_ok(store_ok),
    .cp0_mod_ok(cp0_mod_ok),
    .src3_to_alu(src3_to_alu),
    .brbus(brbus),
    .brbus_deret_err(brbus_deret_err),
    .acc_write_ok(acc_write_ok),
    .acc_qid(acc_qid),
    .mm_ok(mm_ok),
    .falu_ok(falu_ok),
    
    .offset_to_mm(offset_to_mm),
    .int_trigger_o(int_trigger),

    .flush_pipeline_cycle_o(flush_pipeline_cycle), //from hsq
    .inst_to_queue_cycle_o(inst_to_queue_cycle), //from hsq
    .insts_to_alu2_o(insts_to_alu2), //from hsq
    .insts_to_addr_o(insts_to_addr), //from hsq
    .insts_to_falu_o(insts_to_falu), //from hsq
    .insts_to_queue_o(insts_to_queue) //from hsq
     );

wire [3:0] commitbus_to_rs_0;
wire [3:0] commitbus_to_rs_1;

godson_gr_module gr(.clock(coreclock),.reset(reset),
    .qissuebus0(qissuebus0_to_gr),
    .qissuebus1(qissuebus1_to_gr),
    .commitbus0(commitbus0_to_gr),
    
    .commitbus1(commitbus1_to_gr),
    
    .rissuebus0(rissuebus0)
    
     ,.rissuebus1(rissuebus1)
    
    ,.DSPCtl_value(DSPCtl_value)
    );

wire [84:0] alurs_to_alu1;
wire alu_allow2;
wire [87:0] alurs_to_alu2;

godson_alurs1_module alurs1(.clock(coreclock),.reset(reset),.commitbus_ex(commitbus_ex),
    .rissuebus(rissuebus_to_fix1),
    .allow(1'b1),
    .alursfull(alursfull1), // unused
    .alurs_to_alu(alurs_to_alu1),
    .brbus_to_rs(brbus_to_rs));

godson_alurs2_module alurs2(.clock(coreclock),.reset(reset),.commitbus_ex(commitbus_ex),
    .rissuebus(rissuebus_to_fix2),
    .allow(alu_allow2),
    .alursfull(alursfull2),  // unused
    .alurs_to_alu(alurs_to_alu2),
    .brbus_to_rs(brbus_to_rs));

godson_alu_module_nomul alu1(.clock(coreclock),
    .reset(reset),
    .commitbus_ex(commitbus_ex),
    .alurs_to_alu(alurs_to_alu1),
    .DSPCtl_value(DSPCtl_value),
    .alures(resbus1),
    .alures_jalr_target(alures_jalr_target1),
    .src3_to_alu(src3_to_alu),
    .brbus_to_fu(brbus_to_fu));

godson_alu_module alu2(.clock(coreclock),
    .reset(reset),
    .commitbus_ex(commitbus_ex),
    .alurs_to_alu(alurs_to_alu2),
    
    .alures_jalr_target(alures_jalr_target2),
    .alu_res(alu_res),

    .alu2rsfull_tmp(alu2rsfull_tmp),
    .DSPCtl_value(DSPCtl_value),
    .alures(resbus3),
    .src3_to_alu(src3_to_alu),
    .allow(alu_allow2),
    .brbus_to_fu(brbus_to_fu),
    .acc_write_ok(acc_write_ok),
    .acc_qid(acc_qid)
    );

wire[154:0] falurs_to_falu;
assign fcr31             =  31'b0;
assign falufull          =   1'b0;
assign resbus2           =  92'b0;
assign falurs_to_falu    = 154'b0;

wire addr_allowin;
wire [`Lmmrs-1:0] mmrs_to_addr;

wire [`Laddr_to_tlb-1:0] addr_to_tlb;
wire [`Laddr_to_dcache-1:0] addr_to_dcache;
wire [`Lmoveresult-1:0] move_resultbus;
wire tlb_allowin;
wire tlb_stall;
wire tlb_has_ll;
wire no_conflict_to_addr;
wire memqueue_stall;
wire memqueue_has_ll;
wire inst_valid_at_addr;
wire inst_cache_at_addr;
wire inst_sync_at_addr;
wire move_result_ack;

wire itlb_flush;
wire [`Litlb_req-1:0] itlb_req;
wire [`Ltlb_to_itlb-1:0] tlb_to_itlb;
wire [`Ltlb_to_icache-1:0] tlb_to_icache;
wire                       cache28_refill_ok;
wire [`Ldcache_result-1:0] dcache_result;
wire [31:0] dcache_tag;
wire [31:0] tag_to_tlb;
wire conflict_to_tlb;
wire [31:0] dcachepaddr;
wire [`Ltlb_read_again-1:0] tlb_read_again;
wire load_queue_full;
wire store_queue_full;
wire miss_req_queue_full;
wire ex_memqueue;
wire [2:0] rand_num;
wire ll_set_llbit;
wire [`Ltlb_to_memq-1:0] tlb_to_memqueue;
wire [`Ltlb_forward-1:0] tlb_forward;
wire cr_llbit_value;
wire store_req;
wire cache_req;
wire replace_req;
wire [1:0] refill_req;
wire [`Lmemq_to_dcache-1:0] memq_to_dcache;
wire [`Lreplace_dump-1:0] replace_dump;
wire [`Lmemraddr-1:0] cp0_memraddr;
wire [`Lmemwaddr-1:0] cp0_memwaddr;
wire [`Lmemres-1:0] cp0_memres;
wire inst_cache_block;
wire inst_uncache_block;
wire cp0_mem_req;

wire [`Lmemraddr-1:0] inst_memraddr;
wire [`Lmemres-1:0] inst_memres;

wire [36:0] mmres_to_mmrs;

wire [31:0] filter_window_req;
wire        filter_window_result;


godson_mmrs_module mmrs(
    .clock(coreclock),
    .reset(reset),
    .commitbus_ex(commitbus_ex),
    .rissuebus(rissuebus_to_mm),
    .qissuebus0_src1_rdy(qissuebus0_src1_rdy),
    .allow(addr_allowin),
    .mmres(mmres_to_mmrs),
    .mmrs_to_addr(mmrs_to_addr),
    .mmrsfull(mmfull),
    .from_queue_qj(issue_to_mmrs_qj),
    .brbus_to_rs(brbus_to_rs)
    ); 


godson_addr_module u_addr(
    .clock(coreclock),
    .reset(reset),
    .mmrs_to_addr_i(mmrs_to_addr),
    .addr_to_tlb_o(addr_to_tlb),
    .addr_to_dcache_o(addr_to_dcache),
    .tlb_allowin_i(tlb_allowin),
    .tlb_stall_i(tlb_stall),
    .tlb_has_ll_i(tlb_has_ll),
    .DEBUG_MODE(debug_mode),
    .cr_cfg6_rti_i(cr_cfg6_rti),
    .no_conflict_to_addr_i(no_conflict_to_addr),
    .memqueue_stall_i(memqueue_stall),
    .memqueue_has_ll_i(memqueue_has_ll),
    .inst_valid_at_addr_o(inst_valid_at_addr),
    .inst_cache_at_addr_o(inst_cache_at_addr),
    .inst_sync_at_addr_o(inst_sync_at_addr),
    .addr_allowin_o(addr_allowin),
    .cp0_mod_ok_i(cp0_mod_ok),
    .cp0_qid_o(cp0_qid));

wire mode_super;
wire[2:0] config_k0;
godson_dtlb_module u_dtlb(
    .clock(coreclock),
    .reset(reset),
    .commitbus0_i(commitbus0_to_tlb),
    
    .commitbus1_i(commitbus1_to_tlb),
    
    .brbus_to_cp0_i(brbus_to_cp0),
    .int_in(int_in_a),
    .int_out(interrupt),
    .status_bev_o(bev),
    .status_exl_o(exl),
    .status_erl_o(erl),
    .status_ksu_o(ksu),  
    .status_cu_o(cu),
    .mode_user(mode_user),
    .mode_super(mode_super),
    .config_k0(config_k0),
    .entryhi_vpn2_o(entryhi_vpn2),
    .vector_int(vector_int),
    .ebase(ebase),
    .HWREna(HWREna), 
    .itlb_req_i(itlb_req),
    .itlb_flush_o(itlb_flush),
    .tlb_to_itlb_o(tlb_to_itlb),
    .cache28_refill_ok_i(cache28_refill_ok),
    .tlb_to_icache_o(tlb_to_icache),
    .filter_window_req_i(filter_window_req),
    .filter_window_result_o(filter_window_result),

    .qissuebus_valid0_i(qissuebus0[100]),
    .qissuebus_valid1_i(qissuebus1[100]),
    .imemread_valid_i(inst_memraddr[36]), 
    .dmemread_valid_i(cp0_memraddr[36]), //from wwx
    .duncache_valid_i(duncache_valid), //from wwx
    .mreadreq_valid_i(mreadreq_valid), //from wwx
    .mwritereq_valid_i(cp0_memwaddr[36]), //from wwx
    .data_inst_conflict_i(data_inst_conflict),
    .data_inter_conflict_i(data_inter_conflict),
    .brq_full_i(brq_full), 
    .icache_hit_i(icache_hit_perf),
    .icache_access_i(icache_access),
    .icache_update_i(icache_update),
    .icache_way_hit_i(icache_way_hit),
    .queue_full_i(qfull[1]),
    .stalled_fetch_i(~instout[12]), 
    .missq_full_i(missq_full),       //from wwx
    .not_store_ok_i(not_store_ok),
    .st_ld_conflict_i(st_ld_conflict),
    .itlb_access_i(itlb_access),    
    .flush_pipeline_cycle_i(flush_pipeline_cycle), //from hsq
    .inst_to_queue_cycle_i(inst_to_queue_cycle), //from hsq
    .insts_to_alu1_i(insts_to_alu1), //from_hsq
    .insts_to_alu2_i(insts_to_alu2), //from hsq
    .insts_to_addr_i(insts_to_addr), //from hsq
    .insts_to_falu_i(insts_to_falu), //from hsq
    .insts_to_queue_i(insts_to_queue), //from hsq
    .insts_fetched_i(insts_to_fetched),
    .stalled_cycles_icachemiss_i(stalled_cycles_icachemiss),
    .itlbmiss_tlbhit_i(itlbmiss_tlbhit),

    .IBE_FROM_CACHE(ibe_from_cache),

    .HB_DDBL(hb_ddbl),
    .HB_DDBS(hb_ddbs),
    .HB_DDBSIMPR(hb_ddbsimpr),
    .PROBEN_IN(proben),
    .DMSEG_DFREE(dmseg_dfree),
    .DEBUG_MODE(debug_mode),
    .DSS_ENABLE(dss_enable),
    .NODCR(nodcr),
    .IEXI(iexi),
    .PENDING_DBE(pending_dbe),
    .PENDING_IBE(pending_ibe),
    .FIRST_MEM_PASS_AFTER_DERET(first_mem_pass_after_deret),
    .EJTAGBRK_FROM_CORE(dint),
    .HB_REQBUS(hb_reqbus),
    .DCR_REQBUS(dcr_reqbus),
    .DMSEG_DREQBUS(dmseg_dreqbus),
    .HB_DCOMPBUS(hb_dcompbus),
    .addr_to_tlb_i(addr_to_tlb),
    .inst_valid_at_addr_i(inst_valid_at_addr),
    .inst_cache_at_addr_i(inst_cache_at_addr),
    .inst_sync_at_addr_i(inst_sync_at_addr),
    .tlb_allowin_o(tlb_allowin),
    .tlb_stall_o(tlb_stall),
    .tlb_has_ll_o(tlb_has_ll),
    .dcache_result_i({dcache_result[70:64], 64'b0}),
    .tag_to_tlb_i(tag_to_tlb),
    .conflict_to_tlb_i(conflict_to_tlb),
    .dcachepaddr_o(dcachepaddr),
    .tlb_read_again_o(tlb_read_again),


    .load_queue_full_i(load_queue_full),
    .store_queue_full_i(store_queue_full),
    .miss_req_queue_full_i(miss_req_queue_full),
    .ex_memqueue_i(ex_memqueue),
    .rand_num_i(rand_num),
    .ll_set_llbit_i(ll_set_llbit),
    .tlb_to_memqueue_o(tlb_to_memqueue),
    .tlb_forward_bus_o(tlb_forward),
    .cr_llbit_value_o(cr_llbit_value),
    .cr_cfg6_brpred_type_o(cr_cfg6_brpred_type),
    .cr_cfg6_rti_o(cr_cfg6_rti),
    .cr_cfg6_cache0_all_o(cr_cfg6_cache0_all),
    .cr_cfg7_dcache_o(cr_cfg7_dcache),
    .cr_cfg7_icache_o(cr_cfg7_icache),
    .ram_to_tlb_i(ram_to_tlb),
    .tlb_to_ram_o(tlb_to_ram),
    .DRESBUS_FROM_DRSEG(dresbus_from_drseg),
    .DRESBUS_FROM_DCR(dresbus_from_dcr),
    .DRESBUS_FROM_DMSEG(dresbus_from_dmseg),
    .hb_dbs_bs_i(hb_dbs_bs),
    .HB_LOAD_ADDR_MATCH(hb_load_addr_match),
    .DATA_FROM_TAP({tr_to_core,tx_data_from_tap})
);

godson_dcache_module u_dcache(
    .clock(coreclock),
    .reset(reset),
    .addr_to_dcache_i(addr_to_dcache),
    .no_conflict_to_addr_o(no_conflict_to_addr),
    .tlb_read_again_i(tlb_read_again),
    .dcachepaddr_i(dcachepaddr),
    .conflict_to_tlb_o(conflict_to_tlb),
    .dcache_result_o(dcache_result),
    .tag_to_tlb_o(tag_to_tlb),
    .st_ld_conflict_o(st_ld_conflict),
    .dcache_tag_o(dcache_tag),
    .store_req_i(store_req),
    .cache_req_i(cache_req),
    .replace_req_i(replace_req),
    .refill_req_i(refill_req),
    .memq_to_dcache_i(memq_to_dcache),
    .replace_dump_o(replace_dump),
    .ram_to_dcache_i(ram_to_dcache),
    .dcache_to_ram_o(dcache_to_ram)
);

godson_itlb_module u_itlb(
    .clock(coreclock), 
    .reset(reset), 
    .erl(erl),
    .mode_user(mode_user),
    .mode_super(mode_super),
    .config_k0(config_k0),
    .instout_i(instout[45:12]),
    .icachepaddr_o(icachepaddr),
    .itlb_req(itlb_req), 
    .itlb_flush_i(itlb_flush),
    .tlb_to_itlb(tlb_to_itlb), 
    .ram_to_tlb_i(ram_to_tlb),
    .commitbus_to_itlb(commitbus_to_itlb),
    .brbus_deret_err(brbus_deret_err),
    .itlb_access_o(itlb_access),    
    .itlbmiss_tlbhit_o(itlbmiss_tlbhit),
    .HB_DIB(hb_dib),
    .HB_ICOMPBUS(hb_icompbus),
    .NODCR(nodcr),
    .PROBEN_IN(proben),
    .DEBUG_MODE(debug_mode),
    .IR_WAIT_BD(ir_wait_bd),
    .ICACHERDY_AFTER_DERET(icacherdy_after_deret),
    .DERET_IFLAG(DERET_IFLAG), 
    .DMSEG_IREQBUS(dmseg_ireqbus)
);

godson_icache_module u_icache(
    .clock(coreclock),
    .reset(reset),
	.commitbus_ex(commitbus_ex),
    .i_laddr({pc_en_nowayhit,irstalli_r,way_pred,instout[12:0]}),
    .icachepaddr(icachepaddr),
    .memres(inst_memres),
    .dicache(tlb_to_icache),
    .cache28_refill_ok(cache28_refill_ok),
    .inst_cache_block(inst_cache_block),
    .inst_uncache_block(inst_uncache_block),
    .PENDING_IBE(1'b0),//pending_ibe),//for ejtag, by wangwx
    .IEXI(iexi), 
    .DERET_IFLAG(DERET_IFLAG), 
   // .DERET_IFLAG(1'b1),  //for single issue test ,by xucp 
    .cr_cfg7_icache_i(cr_cfg7_icache),
    .cr_cfg6_cache0_all_i(cr_cfg6_cache0_all),
    .iresbus(iresbus),
    .imemraddr(inst_memraddr),
    .IBE_FROM_CACHE(ibe_from_cache),
    .ram_to_icache(ram_to_icache),
    .icache_to_ram(icache_to_ram),
    .icache_stalli(icache_stalli),.icache_refill_ok(icache_refill_ok),
    .icache_access(icache_access), 
    .icache_hit_perf(icache_hit_perf),
    .icache_update(icache_update), 
    .icache_way_hit(icache_way_hit),
    .stalled_cycles_icachemiss_o(stalled_cycles_icachemiss),
    .brbus_to_icache(brbus_to_icache));

godson_memqueue_module u_memqueue(
    .clock(coreclock),
    .reset(reset),
    .commitbus_ex_i(commitbus_ex),
    .cp0res_o(resbus0),
    .mmres_to_mmrs_o(mmres_to_mmrs),
    .store_ok_i(store_ok),
    .brbus_to_cp0_i(brbus_to_cp0[6:0]),
    .store_qid_o(store_qid),
    .cp0_forward_bus_o(cp0_forward_bus),
    .cp0_cancel_bus_o(cp0_cancel_bus),
    .inst_valid_at_addr_i(inst_valid_at_addr),
    .inst_cache_at_addr_i(inst_cache_at_addr),
    .inst_sync_at_addr_i(inst_sync_at_addr),
    .memqueue_stall_o(memqueue_stall),
    .memqueue_has_ll_o(memqueue_has_ll),
    .dcache_value_i(dcache_result[63:0]),
    .dcache_hit_i(dcache_result[70]),
    .dcache_tag_i(dcache_tag),
    .tlb_to_memqueue_i(tlb_to_memqueue),
    .tlb_forward_bus_i(tlb_forward), 
    .cr_llbit_value_i(cr_llbit_value),
    .cr_cfg7_dcache_i(cr_cfg7_dcache),
    .load_queue_full_o(load_queue_full),
    .store_queue_full_o(store_queue_full),
    .miss_req_queue_full_o(miss_req_queue_full),
    .ex_memqueue_o(ex_memqueue),
    .rand_num_o(rand_num),
    .ll_set_llbit_o(ll_set_llbit),
    .replace_dump_i(replace_dump),
    .store_req_o(store_req),
    .cache_req_o(cache_req),
    .replace_req_o(replace_req),
    .refill_req_o(refill_req),
    .memq_to_dcache_o(memq_to_dcache),
    .cp0_memres_i(cp0_memres),
    .cp0_memraddr_o(cp0_memraddr),
    .cp0_memwaddr_o(cp0_memwaddr),
    .cp0_mem_req_o(cp0_mem_req),
    .inst_cache_block_o(inst_cache_block),
    .inst_uncache_block_o(inst_uncache_block),
    .HB_DDBLIMPR(hb_ddblimpr),
    .LOADDATA(loaddata),
    .LOADDATA_QID(loaddata_qid),
    .LOADDATA_VALID(loaddata_valid),
    .LOADDATA_H_VALID(loaddata_h_valid),
    .LOADDATA_H(loaddata_h),
    .BYTELANE(bytelane),
    .duncache_valid_o(duncache_valid),
    .data_inter_conflict_o(data_inter_conflict),
    .missq_full_o(missq_full),
    .not_store_ok_o(not_store_ok)
);


godson_dcr_module dcr(.CLOCK(coreclock),
     .RESET(reset),
     .DCR_REQBUS(dcr_reqbus),
     .NMI_IN(nmi_a),
     .PROBEN_IN(proben_to_core),
     .DRESBUS_FROM_DCR(dresbus_from_dcr),
     .INTE(inte),
     .NMI_OUT(nmi_from_dcr),
     .PROBEN_OUT(proben));

godson_hb_module hb(.CLOCK(coreclock),
     .RESET(reset),
     .DEBUG_MODE(debug_mode),
     .HB_REQBUS(hb_reqbus),
     .HB_ICOMPBUS(hb_icompbus),
     .HB_DCOMPBUS(hb_dcompbus),
     .LOADDATA(loaddata),
     .LOADDATA_QID(loaddata_qid),
     .LOADDATA_VALID(loaddata_valid),
     .LOADDATA_H_VALID(loaddata_h_valid),
     .LOADDATA_H(loaddata_h),
     .COMMITBUS(commitbus_to_hb),
     .BRBUS_TO_HB(brbus_to_hb),
     // .DSS_ENABLE(1'b0),//dss_enable),
     .ICACHERDY_AFTER_DERET(icacherdy_after_deret),
     .FIRST_MEM_PASS_AFTER_DERET(first_mem_pass_after_deret),
     .HB_DIT(),
     .HB_DIB(hb_dib),
     .HB_DDT(),
     .HB_DDBL(hb_ddbl),
     .HB_DDBS(hb_ddbs),
     .HB_DDBLIMPR(hb_ddblimpr),
     .HB_DDBSIMPR(hb_ddbsimpr),
     .HB_LOAD_ADDR_MATCH(hb_load_addr_match),
     .BYTELANE(bytelane),
     .DRESBUS_FROM_HB(dresbus_from_drseg),
     .hb_dbs_bs_o(hb_dbs_bs));

 godson_tap_buffer_module tap_buffer(.CLOCK(coreclock),
     .RESET(reset),
     .SOFTRESET(softreset),
     .COMMITBUS_EX(commitbus_ex),
     .DMSEG_DREQBUS(dmseg_dreqbus),
     .DMSEG_IREQBUS(dmseg_ireqbus),
     .EJTAGBRK_FROM_CORE(dint),
     .DEBUGMODE_FROM_CORE(debug_mode),
     .DATA_FROM_TAP(data_from_tap),
     .PRACC_FROM_TAP(pracc_from_tap),
     .PROBEN_FROM_TAP(proben_from_tap),
     .TRAP_FROM_TAP(trap_from_tap),
     .EJTAGBRK_FROM_TAP(ejtagbrk_from_tap),
     .IRESBUS_FROM_DMSEG(iresbus_from_dmseg),
     .DRESBUS_FROM_DMSEG(dresbus_from_dmseg),
     .DMSEG_DFREE(dmseg_dfree),
     .EJTAGBRK_TO_CORE(ejtagbrk),
     .PROBEN_TO_CORE(proben_to_core),
     .TRAP_TO_CORE(probtrap),
     .ADDR_TO_TAP(addr_to_tap),
     .DATA_TO_TAP(data_to_tap),
     .WIDTH_TO_TAP(width_to_tap),
     .WRITE_TO_TAP(write_to_tap),
     .PRACC_TO_TAP(pracc_to_tap),
     .EJTAGBRK_TO_TAP(ejtagbrk_to_tap),
     .RESET_TO_TAP(reset_to_tap),
     .DEBUGMODE_TO_TAP(debugmode_to_tap),
     .TR_FROM_TAP(tr_from_tap),
     .TR_TO_TAP(tr_to_tap),
     .TR_TO_CORE(tr_to_core));

wire trst;
godson_ejtag_rstgen  ejtag_rst (.tck (EJTAG_TCK), .trst_in (EJTAG_TRST), .trst_out(trst),.testmode(testmode));

godson_ejtag_tap_module  ejtag_tap_U
     (.TCK             (EJTAG_TCK) ,
      .TRST            (trst) ,
      .TMS             (EJTAG_TMS) ,
      .TDI             (EJTAG_TDI) ,
      .TDO             (EJTAG_TDO) ,
      .DMSEG_ADDR      (addr_to_tap) ,
      .DMSEG_RDATA     (data_to_tap) ,
      .DMSEG_WDATA     (data_from_tap) ,
      .ROCC_IN         (reset_to_tap) ,
      .DMSEG_BE_IN     (width_to_tap) ,
      .PRNW            (write_to_tap) ,
      .PRACC_IN        (pracc_to_tap) ,
      .EJTAGBRK_IN     (ejtagbrk_to_tap) ,
      .DM              (debugmode_to_tap) ,
      .PRACC_OUT       (pracc_from_tap) ,
      .PRRST           (prrst_from_tap) ,
      .PROBEN          (proben_from_tap) ,
      .PROBTRAP        (trap_from_tap) ,
      .EJTAGBRK_OUT    (ejtagbrk_from_tap),
      .TR_OUT          (tr_from_tap),
      .PO_TX           (tx_data_from_tap),
      .TR_IN           (tr_to_tap)
      );

reg prrst_to_core_reg, prrst_to_core ;

always @(posedge aclk)
 begin
   prrst_to_core_reg <= prrst_from_tap ;
   prrst_to_core <=prrst_to_core_reg ;
 end


axi_interface m_axi_interface(
    .aclk(aclk),
//    .areset_n(areset_n),
    .areset_n(areset_n&&(~prrst_to_core)),

    .arid(arid),
    .araddr(araddr),
    .arlen(arlen),
    .arsize(arsize),
    .arburst(arburst),
    .arlock(arlock),
    .arcache(arcache),
    .arprot(arprot),
    .arvalid(arvalid),
    .arready(arready),

    .rid(rid),
    .rdata(rdata),
    .rresp(rresp),
    .rlast(rlast),
    .rvalid(rvalid),
    .rready(rready),

    .awid(awid),
    .awaddr(awaddr),
    .awlen(awlen),
    .awsize(awsize),
    .awburst(awburst),
    .awlock(awlock),
    .awcache(awcache),
    .awprot(awprot),
    .awvalid(awvalid),
    .awready(awready),

    .wid(wid),
    .wdata(wdata),
    .wstrb(wstrb),
    .wlast(wlast),
    .wvalid(wvalid),
    .wready(wready),

    .bid(bid),
    .bresp(bresp),
    .bvalid(bvalid),
    .bready(bready),

    .reset_o(reset),
 
    .softreset_i(1'b1), //not support real softreset now
    .softreset_o(softreset),

    .nmi_i(nmi),
    .nmi_o(nmi_a),

    .int_i(interrupt_i),
    .int_o(int_in_a),

    .filter_window_result_i(filter_window_result),
    .filter_window_req_o(filter_window_req),

    .inst_memraddr_i(inst_memraddr),
    .inst_memres_o(inst_memres),

    .cp0_mem_req_i(cp0_mem_req),
    .cp0_memraddr_i(cp0_memraddr),
    .cp0_memwaddr_i(cp0_memwaddr),
    .cp0_memres_o(cp0_memres)
);

endmodule 

module godson_ejtag_rstgen (tck , trst_in , trst_out, testmode);
input tck, trst_in;
input testmode;
output trst_out;
reg  rff, rff1; 

always @ (posedge tck or negedge trst_in)
  begin
    if(!trst_in) {rff1, rff} <= 2'b0;
    else         {rff1, rff} <= {rff, 1'b1}; 
  end

assign trst_out = testmode ? trst_in : rff1;

endmodule
