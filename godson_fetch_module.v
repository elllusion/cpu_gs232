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
`define BHTsize 256 
`define BHTbits 2

module godson_fetch_module(
    clock,reset,commitbus,qfull,qstalli,exl,bev,
    vector_int,ebase,brbus,cr_config6,user_mode,
    int_trigger, brq_full,brq_tail_next_valid,
    icache_stalli,icache_refill_ok,insts_fetched_o,
    PROBTRAP,NODCR,PROBEN,IRESBUS_FROM_CACHE,
    IRESBUS_FROM_DMSEG,DEBUG_MODE,DSS_ENABLE,
    DERET_IFLAG,IR_WAIT_BD,EJTAGBOOT,instout,
    irbus, irfull,way_pred, irstalli_r,pc_en_nowayhit
);
parameter  EX_ENTRY_RESET     = 32'hbfc00000;
parameter  EX_ENTRY_BEV0      = 32'h80000000;
parameter  EX_ENTRY_BEV1      = 32'hbfc00200;
parameter  EX_ENTRY_EJTAG_MEM = 32'hbfc00480;
parameter  EX_ENTRY_EJTAG_PROB= 32'hff200200;
  
input                                   reset      ; 
input                                   clock      ;
input [`Lcommitbus_to_fetch-1:0]        commitbus  ;
input [1:0]           qfull             ;
                                         //if qfull[1]==1,then the queue is full;
                                         //else if ~qfull[1]&qfull[0],then the queue has only one emty enty;
                                         //else if ~qfull[1]&~qfull[0],then the queue can accept two instruction. 
input                 brq_full          ;
input                 brq_tail_next_valid ;
input [95:0]          brbus             ;
input                 qstalli           ;
input                 icache_stalli     ;
input                 icache_refill_ok  ;
input                 exl               ;
input                 bev               ;
input[8:0]            vector_int        ;
input[31:0]           ebase;
input[2:0]            cr_config6;
input                 user_mode         ;
input                 int_trigger;

input                 PROBTRAP          ;
input                 NODCR             ;
input                 PROBEN            ;
input [`Liresbus_2issue-1:0] IRESBUS_FROM_CACHE;
input [`Liresbus-1:0] IRESBUS_FROM_DMSEG;  
input                 DEBUG_MODE        ;
input                 DSS_ENABLE        ;
input                 EJTAGBOOT;
input                 DERET_IFLAG;

output [45:0]         instout   ;
output [`Lirbus_2issue-1:0]  irbus     ;
output                IR_WAIT_BD;
output                irfull;
output[49:0]          way_pred ;
output                irstalli_r;
output                pc_en_nowayhit;
output[1:0]           insts_fetched_o;

wire [`Lirbus-1:0]   irbus_0;//the first instruction transfered to decoder;
wire [`Lirbus-1:0]   irbus_1;//the second instruction transfered to decoder;

wire [`Liresbus-1:0] IRESBUS_FROM_CACHE_0 = IRESBUS_FROM_CACHE[38:0];
wire [`Liresbus-1:0] IRESBUS_FROM_CACHE_1 = IRESBUS_FROM_CACHE[77:39];

wire [`Liresbus-1:0] IRESBUS_FROM_DMSEG_0 =  IRESBUS_FROM_DMSEG[38:0];

wire [1:0]irvalid;
wire iresvalid_0 , iresvalid_1;
wire ires_ex_0   , ires_ex_1;

reg  wait_bd     ; //next cycle, dispatched inst to decode is the delay slot intruction
wire irstalli    ;

wire pc_in_en    ;
wire tail_in_en_0; //instructions from Icache is valid for IR
wire tail_in_en_1;

wire pc_en_nowayhit ;

reg [`Lword-1:0] pc  ; //program counter
reg [`Lword-1:0] irstalli_pc  ; //when irstalli instruction commit, refetch's pc


// IR has 2 entries when two-issued
reg [`IRsize-1:0]ir_rdy;
reg [`IRsize-1:0]ir_adei;
reg [`IRsize-1:0]ir_tlbii;
reg [`IRsize-1:0]ir_tlbir;
reg [`IRsize-1:0]ir_dib;
reg [`IRsize-1:0]ibe;
reg [`Lword-1:0] ir_inst[`IRsize-1:0] ;
reg [`Lword-1:0] ir_pc;

reg [1:0]        ir_seq_link ;
reg [1:0]        ir_noseq_link[`IRsize-1:0]  ;
reg [9:0]        ir_link_info ; //by xucp
reg [19:0]       ir_paddr_tag ; //by xucp
reg [1:0]        ir_way_hit; //by xucp
reg [1:0]        ir_ex; //by xucp, for not update link
reg [1:0]        ir_v_lock; //by xucp
reg              ir_one_word;

reg              ir_int;
reg              block_begin; 

wire[31:0] ir_pc_plus4 = ir_pc + 3'b100;

wire [4:0] intctl_vs;
wire [2:0] intvec_no;
wire       cause_iv;

wire       commitbus0_valid  ;
wire [5:0] commitbus0_excode ;
wire       commitbus0_ex     ; 
wire [7:0] commitbus0_gshare;   //the prediction status of the branch;
wire [1:0] commitbus0_rashead;   //recieved ras head;

wire       commitbus1_valid  ;
wire [5:0] commitbus1_excode ;
wire       commitbus1_ex     ; 
wire [7:0] commitbus1_gshare;   //the prediction status of the branch;
wire [1:0] commitbus1_rashead;   //recieved ras head;

wire          brbus_valid_t    ;   //should come from queue
wire          brbus_valid      ;   //should come from queue
wire          brbus_err        ;   //should come from queue
wire [5: 0]   brbus_brvector   ;
wire [ 7: 0]  brbus_op         ;
wire [31: 0]  brbus_value      ;
wire [31: 0]  brbus_value_h    ;
wire [1 : 0]  brbus_old_status ;
wire [7 : 0]  brbus_gshare     ;
wire [1 : 0]  brbus_rashead    ;
wire [1 : 0]  brbus_other_link ;
wire          brbus_irstalli_bd ;
wire          brbus_brtaken;

wire              iresbus_valid_0   , iresbus_valid_1   ;
wire              iresbus_cacherdy_0, iresbus_cacherdy_1;
wire [`Lword-1:0] iresbus_value_0   , iresbus_value_1   ;
wire              iresbus_adei_0    , iresbus_adei_1    ;
wire              iresbus_tlbii_0   , iresbus_tlbii_1   ;
wire              iresbus_tlbir_0   , iresbus_tlbir_1   ;
wire              iresbus_ibe_0     , iresbus_ibe_1     ;
wire              iresbus_dib_0     , iresbus_dib_1     ;
wire[9:0]         iresbus_link_info;
wire[1:0]         iresbus_way_hit;
wire[19:0]        iresbus_paddr_tag;
wire[1:0]         iresbus_v_lock;
wire              iresbus_no_update;
wire              iresbus_one_word;

wire              irbus_valid_0      , irbus_valid_1;
wire [`Lword-1:0] irbus_pc_0         , irbus_pc_1   ;	
wire [`Lword-1:0] irbus_inst_0       , irbus_inst_1 ;	 
wire              irbus_bd_0         , irbus_bd_1   ;
wire              irbus_rdy_0        , irbus_rdy_1  ;
wire              irbus_adei_0       , irbus_adei_1 ;
wire              irbus_tlbii_0      , irbus_tlbii_1;
wire              irbus_tlbir_0      , irbus_tlbir_1;
wire              irbus_ibe_0        , irbus_ibe_1  ;	
wire              irbus_dib_0        , irbus_dib_1  ;	
wire [`Lword-1:0] irbus_taken_addr_0 , irbus_taken_addr_1;
wire [1:0]        irbus_pred_status_0, irbus_pred_status_1;
wire [7:0]        irbus_gshare_0     , irbus_gshare_1;
wire [1:0]        irbus_rashead_0    , irbus_rashead_1;
wire [1:0]        irbus_other_link_0 , irbus_other_link_1;
wire              irbus_block_begin_0 , irbus_block_begin_1;

wire              instout_valid   ;
wire              instout_pc_in_en;
wire [`Lword-1:0] instout_pc      ;

assign intctl_vs               =vector_int[4:0];
assign intvec_no               =vector_int[7:5];
assign cause_iv                =vector_int[8];

assign commitbus0_valid        = commitbus[0];
assign commitbus0_ex           = commitbus[1];
assign commitbus0_excode       = commitbus[ 7: 2];
assign commitbus0_gshare       = commitbus[15:8];
assign commitbus0_rashead      = commitbus[17:16];
assign commitbus1_valid        = commitbus[18];
assign commitbus1_ex           = commitbus[19];
assign commitbus1_excode       = commitbus[25:20];
assign commitbus1_gshare       = commitbus[33:26];
assign commitbus1_rashead      = commitbus[35:34];

wire       commitbus_ex             = commitbus0_ex | commitbus1_ex;
wire[5:0]  commitbus_excode         = commitbus0_ex ? commitbus0_excode       : commitbus1_excode;
wire[1:0]  commitbus_rashead        = commitbus0_ex ? commitbus0_rashead      : commitbus1_rashead;

assign  brbus_valid_t      =   brbus[0]   ;  
assign  brbus_err          =   brbus[1]   ;  
assign  brbus_brvector     =   brbus[7:2] ; 
assign  brbus_op           =   brbus[15:8];
assign  brbus_value        =   brbus[47:16];
assign  brbus_value_h      =   brbus[79:48]; 
assign  brbus_old_status   =   brbus[81:80];
assign  brbus_gshare       =   brbus[89:82];
assign  brbus_rashead      =   brbus[91:90];
assign  brbus_other_link   =   brbus[93:92];
assign  brbus_irstalli_bd  =   brbus[94];
assign  brbus_brtaken      =   brbus[95];

assign brbus_valid   = brbus_valid_t & ~commitbus_ex;

wire   iresbus_dib_0_t      = IRESBUS_FROM_CACHE_0[38]                                                  ;
wire   iresbus_valid_0_t    = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[37]  :IRESBUS_FROM_CACHE_0[37]  ;
wire   iresbus_cacherdy_0_t = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[36]  :IRESBUS_FROM_CACHE_0[36]  ; 
assign iresbus_value_0      = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[35:4]:
                              int_trigger ? 32'b0 : IRESBUS_FROM_CACHE_0[35:4];
wire iresbus_ibe_0_t        = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[3]   :IRESBUS_FROM_CACHE_0[3]   ;
wire iresbus_adei_0_t       = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[2]   :IRESBUS_FROM_CACHE_0[2]   ;
wire iresbus_tlbii_0_t      = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[1]   :IRESBUS_FROM_CACHE_0[1]   ;
wire iresbus_tlbir_0_t      = IRESBUS_FROM_DMSEG_0[37]?IRESBUS_FROM_DMSEG_0[0]   :IRESBUS_FROM_CACHE_0[0]   ;

assign iresbus_dib_0     = iresbus_dib_0_t & instout_valid;
assign iresbus_ibe_0     = iresbus_ibe_0_t & instout_valid;
assign iresbus_adei_0    = iresbus_adei_0_t & instout_valid;
assign iresbus_tlbii_0   = iresbus_tlbii_0_t & instout_valid;
assign iresbus_tlbir_0   = iresbus_tlbir_0_t & instout_valid;
assign iresbus_valid_0   = iresbus_valid_0_t& instout_valid;
assign iresbus_cacherdy_0   = iresbus_cacherdy_0_t& instout_valid;

wire   iresbus_dib_1_t     = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[38]  ;
wire   iresbus_valid_1_t   = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[37]  ;
assign iresbus_cacherdy_1  = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[36]  ; 
assign iresbus_value_1     = IRESBUS_FROM_DMSEG_0[37]? 32'b0 : 
                             int_trigger             ? 32'b0 : IRESBUS_FROM_CACHE_1[35:4];
wire  iresbus_ibe_1_t      = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[3]   ;
wire  iresbus_adei_1_t     = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[2]   ;
wire  iresbus_tlbii_1_t    = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[1]   ;
wire  iresbus_tlbir_1_t    = IRESBUS_FROM_DMSEG_0[37]? 1'b0  : IRESBUS_FROM_CACHE_1[0]   ;

assign iresbus_dib_1     = iresbus_dib_1_t & instout_valid;
assign iresbus_valid_1   = iresbus_valid_1_t & instout_valid;
assign iresbus_ibe_1     = iresbus_ibe_1_t & instout_valid;
assign iresbus_adei_1    = iresbus_adei_1_t & instout_valid;
assign iresbus_tlbii_1   = iresbus_tlbii_1_t & instout_valid;
assign iresbus_tlbir_1   = iresbus_tlbir_1_t & instout_valid;

assign iresbus_link_info  =IRESBUS_FROM_DMSEG_0[37]? 10'b0       : IRESBUS_FROM_CACHE[87:78];
assign iresbus_way_hit    =IRESBUS_FROM_DMSEG_0[37]? 2'b0        : IRESBUS_FROM_CACHE[89:88];
assign iresbus_paddr_tag  =IRESBUS_FROM_DMSEG_0[37]? 10'b0       : IRESBUS_FROM_CACHE[109:90];
assign iresbus_v_lock     =IRESBUS_FROM_DMSEG_0[37]? 2'b0        : IRESBUS_FROM_CACHE[111:110];
assign iresbus_no_update  =IRESBUS_FROM_DMSEG_0[37]? 1'b0        : IRESBUS_FROM_CACHE[112];

assign irbus_0[70]   =irbus_dib_0       ;
assign irbus_0[69]   =irbus_valid_0     ;
assign irbus_0[68:37]=irbus_pc_0[31:0]  ;
assign irbus_0[36:5] =irbus_inst_0[31:0];
assign irbus_0[4]    =irbus_bd_0        ;
assign irbus_0[3]    =irbus_adei_0      ;
assign irbus_0[2]    =irbus_tlbii_0     ;
assign irbus_0[1]    =irbus_tlbir_0     ;
assign irbus_0[0]    =irbus_ibe_0       ;
assign irbus_0[102:71]  = irbus_taken_addr_0;
assign irbus_0[104:103] = irbus_pred_status_0;
assign irbus_0[112:105] = irbus_gshare_0;
assign irbus_0[114:113] = irbus_rashead_0;
assign irbus_0[116:115] = irbus_other_link_0;
assign irbus_0[117]     = irbus_block_begin_0;
assign irbus_0[118]   =ir_int ;

assign irbus_1[70]      = irbus_dib_1       ;
assign irbus_1[69]      = irbus_valid_1     ;
assign irbus_1[68:37]   = irbus_pc_1[31:0]  ;
assign irbus_1[36:5]    = irbus_inst_1[31:0];
assign irbus_1[4]       = irbus_bd_1        ;
assign irbus_1[3]       = irbus_adei_1      ;
assign irbus_1[2]       = irbus_tlbii_1     ;
assign irbus_1[1]       = irbus_tlbir_1     ;
assign irbus_1[0]       = irbus_ibe_1       ;
assign irbus_1[102:71]  = irbus_taken_addr_1;
assign irbus_1[104:103] = irbus_pred_status_1;
assign irbus_1[112:105] = irbus_gshare_1;
assign irbus_1[114:113] = irbus_rashead_1;
assign irbus_1[116:115] = irbus_other_link_1;
assign irbus_1[117]     = irbus_block_begin_1;
assign irbus_1[118]     = 1'b0;

assign irbus[118:0]     = irbus_0;
assign irbus[237:119]   = irbus_1;

assign instout[45:14]   = instout_pc; //for tlb;
assign instout[13]      = instout_valid;
assign instout[12]      = instout_pc_in_en;
assign instout[11:0]    = instout_pc[11:0];


wire config_gshare_pred     = (cr_config6 == 3'b000);
wire config_pc_pred         = (cr_config6 == 3'b001);
wire config_static_taken    = (cr_config6 == 3'b010);
wire config_static_nottaken = (cr_config6 == 3'b011);
wire config_static_ftaken   = (cr_config6 == 3'b100); //front taken
wire config_static_btaken   = (cr_config6 == 3'b101); //back taken


/**************part1:predecode instructions in IR*******************/
wire[`Lword-1:0] ir_inst_0 , ir_inst_1;
wire             ir_rdy_0  , ir_rdy_1;
wire             ir_adei_0 , ir_adei_1;
wire             ir_tlbii_0, ir_tlbii_1;
wire             ir_tlbir_0, ir_tlbir_1;
wire             ir_dib_0  , ir_dib_1;
wire             ir_ibe_0  , ir_ibe_1;
wire[1:0]        ir_noseq_link_0, ir_noseq_link_1;

assign ir_inst_0  = ir_inst[0];     assign ir_inst_1  = ir_inst[1];

assign ir_rdy_0   = ir_rdy[0];      assign ir_rdy_1   = ir_rdy[1];

assign ir_adei_0  = ir_adei[0];     assign ir_adei_1  = ir_adei[1];

assign ir_tlbii_0 = ir_tlbii[0];    assign ir_tlbii_1 = ir_tlbii[1];

assign ir_tlbir_0 = ir_tlbir[0];    assign ir_tlbir_1 = ir_tlbir[1];

assign ir_dib_0   = ir_dib[0];      assign ir_dib_1   = ir_dib[1];

assign ir_ibe_0   = ibe[0];         assign ir_ibe_1   = ibe[1];

assign ir_noseq_link_0 = ir_noseq_link[0];   assign ir_noseq_link_1 = ir_noseq_link[1];

wire ir_noempty   = ir_rdy_0   | ir_rdy_1   ;
wire ir_adei_mid  = ir_adei_0  | ir_adei_1  ;
wire ir_tlbii_mid = ir_tlbii_0 | ir_tlbii_1 ;
wire ir_tlbir_mid = ir_tlbir_0 | ir_tlbir_1 ;
wire ir_dib_mid   = ir_dib_0   | ir_dib_1   ;
wire ir_ibe_mid   = ir_ibe_0   | ir_ibe_1   ;

assign irfull     = (ir_rdy_0 | ir_adei_0 | ir_tlbii_0 | ir_dib_0 | ir_ibe_0) &
                    (ir_rdy_1 | ir_adei_1 | ir_tlbii_1 | ir_dib_1 | ir_ibe_1); 

wire       ir_jump_0     ,  ir_jump_1     ;
wire       ir_jal_0      ,  ir_jal_1      ;
wire       ir_jalr31_0   ,  ir_jalr31_1   ;
wire       ir_blink_0    ,  ir_blink_1    ;
wire       ir_jreg_0     ,  ir_jreg_1     ;
wire       ir_jr31_0     ,  ir_jr31_1     ;
wire       ir_wait_bd_0  ,  ir_wait_bd_1  ;
wire       irstalli_0    ,  irstalli_1    ;
wire       ir_config_br_0,  ir_config_br_1; 
wire       ir_static_br_0,  ir_static_br_1;
wire       ir_back_br_0  ,  ir_back_br_1  ;
wire       ir_wait_0     ,  ir_wait_1     ;
wire       ir_ret_0      ,  ir_ret_1      ; //eret or deret
wire       ir_blikely_0  ,  ir_blikely_1  ;
wire[31:0] offsetaddr_0  ,  offsetaddr_1  ;

reg wait_pipeline; 

//when inst_wait in delayslot or in user_mode, do nothing
wire wait_in_ir =  (ir_wait_1&ir_rdy_1&~user_mode)&~ir_wait_bd_0 |
                   (ir_wait_0&ir_rdy_0&~user_mode)&~wait_bd;

assign  irstalli = ((irstalli_0 & ir_rdy_0) | (irstalli_1 & ir_rdy_1) |
                   ir_adei_mid| ir_tlbii_mid| ir_tlbir_mid|ir_ibe_mid|ir_dib_mid | wait_pipeline) |
                   wait_in_ir ;

                   
always @(posedge clock)
begin
if (reset)
    wait_pipeline <= 1'b0;
else if (int_trigger)
   wait_pipeline <= 1'b0;
else if (brbus_valid & brbus_err) //for wait is in wrong branch path
    wait_pipeline <= 1'b0;
else if (commitbus_ex)  //for wait sits after the ex instruction
    wait_pipeline <= 1'b0;
else if(wait_in_ir)  
   wait_pipeline <= 1'b1;
end


assign IR_WAIT_BD      = ir_wait_bd_0 & ((ir_pc[4:2] == 3'b111) | DERET_IFLAG); 


predecode_module ir_predec_0(.reset(reset),.DSS_ENABLE(DSS_ENABLE),.DEBUG_MODE(DEBUG_MODE),.ir_inst(ir_inst_0),
                             .ir_config_br(ir_config_br_0),.ir_static_br(ir_static_br_0),
                             .ir_jreg     (ir_jreg_0     ),.ir_jump     (ir_jump_0     ),
                             .ir_jal      (ir_jal_0      ),.ir_jalr31   (ir_jalr31_0   ),.ir_jr31(ir_jr31_0),
                             .ir_blink    (ir_blink_0    ),.ir_blikely  (ir_blikely_0  ),
                             .ir_back_br  (ir_back_br_0  ),.ir_ret      (ir_ret_0      ),.ir_wait(ir_wait_0),
                             .ir_wait_bd  (ir_wait_bd_0  ),.irstalli    (irstalli_0    ),.offsetaddr(offsetaddr_0));
predecode_module ir_predec_1(.reset(reset),.DSS_ENABLE(DSS_ENABLE),.DEBUG_MODE(DEBUG_MODE),.ir_inst(ir_inst_1),
                             .ir_config_br(ir_config_br_1),.ir_static_br(ir_static_br_1),
                             .ir_jreg     (ir_jreg_1     ),.ir_jump     (ir_jump_1     ),
                             .ir_jal      (ir_jal_1      ),.ir_jalr31   (ir_jalr31_1   ),.ir_jr31(ir_jr31_1),
                             .ir_blink    (ir_blink_1    ),.ir_blikely  (ir_blikely_1  ),
                             .ir_back_br  (ir_back_br_1  ),.ir_ret      (ir_ret_1      ),.ir_wait(ir_wait_1),
                             .ir_wait_bd  (ir_wait_bd_1  ),.irstalli    (irstalli_1    ),.offsetaddr(offsetaddr_1));

/******part2.1:BHT for common branch, but static prediction for branch likely(taken) and jr(not taken) */
wire [`BHTbits-1:0] status_mid;      //status from bht
wire [`BHTbits-1:0] status_0, status_1; 
reg  [7:0] gshare_reg;  

//brbus update bht
wire  brbus_config_br,brbus_config_br_t;
wire brbus_br_t;
wire[1:0] brbus_status_plus,brbus_status_minus;
wire      brbus_update_bht_en;
wire[1:0] brbus_update_status;
wire[7:0] brbus_bht_addr;

BR_CONFIG_OP      brbus_op_config_br(.op(brbus_op),.br_op(brbus_config_br_t));

assign brbus_config_br = brbus_valid & brbus_config_br_t;

assign brbus_status_plus[1]  = brbus_old_status[1] |  brbus_old_status[0];
assign brbus_status_plus[0]  = brbus_old_status[1] | ~brbus_old_status[0];
assign brbus_status_minus[1] = brbus_old_status[1] &  brbus_old_status[0];
assign brbus_status_minus[0] = brbus_old_status[1] & ~brbus_old_status[0];

assign brbus_update_status = brbus_brtaken ? brbus_status_plus : brbus_status_minus;

assign brbus_update_bht_en = ~(reset | EJTAGBOOT)&brbus_config_br&(config_gshare_pred | config_pc_pred)&
                             ~( brbus_brtaken &  brbus_old_status[1]&  brbus_old_status[0])&
                             ~(~brbus_brtaken & ~brbus_old_status[1]& ~brbus_old_status[0]);

assign brbus_bht_addr = (cr_config6[2:1]!=2'b00) ? 8'b0:
                         cr_config6[0]           ? brbus_value_h[10:3]: brbus_value_h[10:3] ^ brbus_gshare ;
                             

wire[7:0] bht_rd_addr   =  ir_pc[10:3]         ;
wire      bht_up_en     =  brbus_update_bht_en ;
wire[1:0] bht_up_status =  brbus_update_status ;
wire[7:0] bht_up_addr   =  brbus_bht_addr      ;

bht_module bht_op(.clock(clock),.reset(reset),.read_addr(bht_rd_addr), .gshare(gshare_reg),
                  .write_addr(bht_up_addr),.write_en(bht_up_en),.write_data(bht_up_status),
                  .status0(status_0),.status1(status_1));
assign  status_mid  = (ir_config_br_0 | ir_config_br_1&~ir_pc[2] ) ? status_0: status_1; 

//when new instruction is putted into IR,
// the status for it should be keep until another instruction are putted into the same entry in the IR
reg[1:0] status_reg;

reg[1:0] ir_in_en_delay;

wire tail_in_en_inst_0, tail_in_en_inst_1;

always@(posedge clock)
begin
 ir_in_en_delay <= {tail_in_en_inst_1,tail_in_en_inst_0};
end

wire status_select = ir_config_br_0 ? ir_in_en_delay[0] : ir_in_en_delay[1];

always@(posedge clock)
if(reset) 
    status_reg <= 2'b00;
else if(status_select)
    status_reg <= status_mid;

wire[1:0] status = (config_gshare_pred | config_pc_pred)&status_select  ?  status_mid  :
                   config_static_btaken  ? ((ir_config_br_0&ir_back_br_0  | ir_config_br_1&ir_back_br_1) ? 2'b11 : 2'b00) :
                   config_static_ftaken  ? ((ir_config_br_0&~ir_back_br_0 | ir_config_br_1&~ir_back_br_1) ? 2'b11 : 2'b00) :
                   config_static_taken   ?  2'b11: 
                   config_static_nottaken?  2'b00: status_reg;

always @ (posedge clock)
begin
   if (reset)
      gshare_reg <= 8'b0;  //when config_pc_pred, the bht rdaddr is pc
   else if (brbus_config_br &brbus_err & config_gshare_pred ) //branch prediction err end early
          gshare_reg <= {brbus_gshare[6:0],brbus_brtaken};
   else if (config_gshare_pred&brbus_err&(~brbus_config_br)) 
          gshare_reg <= brbus_gshare;
   else if (commitbus_ex&config_gshare_pred)                        //other  exceptions 
          gshare_reg <= commitbus0_ex ? commitbus0_gshare : commitbus1_gshare;
   else if ((irbus_valid_0&ir_config_br_0| irbus_valid_1&ir_config_br_1) &config_gshare_pred) //dipatch bht br to decode 
          gshare_reg <= {gshare_reg[6:0],status[1]};
end    

/***************part2.2: RAS******************************/
reg [31:0] RAS[3:0];
reg [1 :0] RAS_way[3:0];
reg [1 :0] ras_head;

wire[1:0] rashead_next,rashead_minus1;
wire push_ras, pop_ras;

wire blink_push_0, blink_push_1;
wire[ 1:0] push_way_0 , push_way_1;

wire[31:0] ras_push_pc;
wire[ 1:0] ras_push_way;

//wire [1:0] rashead_to_decode;
wire [31:0] irbus_pc0_plus8 = ir_pc + 32'b1000;
wire [31:0] irbus_pc1_plus8 = ir_pc + 32'b1100;

assign rashead_next = ras_head + 1'b1;
assign rashead_minus1 = ras_head - 1'b1;
 
assign blink_push_0 = ir_jal_0 | ir_blink_0 | ir_jalr31_0;
assign blink_push_1 = ir_jal_1 | ir_blink_1 | ir_jalr31_1;
assign push_ras = (irbus_valid_0 & blink_push_0) | (irbus_valid_1 & blink_push_1);
assign pop_ras  = (irbus_valid_0 & ir_jr31_0)    | (irbus_valid_1 & ir_jr31_1);

assign ras_push_pc = irbus_valid_0 & blink_push_0 ? irbus_pc0_plus8 : irbus_pc1_plus8; 

assign push_way_0   = (irbus_pc_0[4:3] == 2'b11)   ? ir_seq_link      : ir_way_hit; 
assign push_way_1   = (irbus_pc_1[4:3] == 2'b11)   ? ir_seq_link      : ir_way_hit; 
assign ras_push_way = irbus_valid_0 & blink_push_0 ? push_way_0       : push_way_1;


always @(posedge clock)
begin
if(reset)
   ras_head <= 2'b00;
else if(commitbus_ex)
   ras_head  <=commitbus_rashead;
else if(brbus_err & brbus_valid)
   ras_head  <=brbus_rashead;
else if (push_ras)
   ras_head <= rashead_next;
else if (pop_ras)
   ras_head <= rashead_minus1;
end

always @ (posedge clock )
begin 
if(push_ras & ~reset & ~commitbus_ex & ~(brbus_valid&brbus_err))
begin
    RAS[ras_head]     <= ras_push_pc;
    RAS_way[ras_head] <= ras_push_way; 
end
end
/*
assign rashead_to_decode = (ir_jalr31_0&irbus_valid_0 | ir_jalr31_1&irbus_valid_1) ?
                           rashead_next : ras_head;
*/
//when jr dispatch into queue, the rashead has already minus1, so this can't be index
wire [1:0] pop_ras_index = ((irvalid[0] & ir_jr31_0) | (irvalid[1] & ir_jr31_1))  ? rashead_minus1 : ras_head;

/**************part3: generate PC*************************/
wire [31:0] exceptionaddr_reset,exceptionaddr_normal;
wire [31:0] exceptionaddr_ejtag;
wire exception_reset,exception_normal;
wire exception_ejtag;

wire [31:0] branchaddr_br,branchaddr_jump,branchaddr_next, branchaddr_jr31;
wire branch_br,branch_jump,branch_next, branch_jr31;

wire clear, clear_t;

wire [31:0] pc_temp;

wire ir_static_br_mid;     //check if the instrution in IR is static predcit br 
wire ir_config_br_mid;     //check if the instrution in IR is config predcit br 
wire ir_jump_mid;         //check if the instrution in IR is jump br
wire ir_jr31_mid;

wire irstalli_next; //if the next instruction of irstalli is fetched in IR, then the instruction is invalid
assign irstalli_next  = (irstalli_0& ir_rdy_0|
                        ir_adei_0|ir_tlbii_0|
                        ir_tlbir_0|ir_ibe_0|ir_dib_0 |
                        ir_wait_0&ir_rdy_0&~wait_bd&~user_mode) & irvalid[1];
reg irstalli_next_reg;
reg irstalli_reg;

wire brbus_refetch;

always @ (posedge clock)
if (pc_in_en)
begin
   irstalli_reg <= 0;
   irstalli_next_reg <= 0;
end
else 
begin
if (irstalli_next)  //for cancel the next instruction of irstalli
   irstalli_next_reg <=1;
else if (brbus_valid & brbus_err & brbus_irstalli_bd) //irstalli is branch delayslot
   irstalli_next_reg <=0;

if (irstalli) //for stall instruction fetch 
   irstalli_reg <=1;
end


always @ (posedge clock)
begin
   if(reset | irstalli)
      irstalli_pc <= pc;
   else if(brbus_valid & brbus_err&brbus_irstalli_bd)
      irstalli_pc <= brbus_value;
end

assign  irstalli_r = irstalli_reg;

reg stalled_ex_r; //when cache miss and ex happen simutiously, then cache miss is dealed firstly.
reg stalled_ex_refetch; //brbus refetch are stalled.
reg[31:0] stalled_pc;

always @(posedge clock)
if(reset)
begin
   stalled_ex_r <=1'b0;
   stalled_ex_refetch <=1'b0;
   stalled_pc   <=32'b0;
end
else if (exception_reset & icache_stalli)
begin
   stalled_ex_r <=1'b1;
   stalled_pc   <=exceptionaddr_reset;
end
else if (exception_normal & icache_stalli)
begin
   stalled_ex_r <=1'b1;
   stalled_pc   <=exceptionaddr_normal;
end
else if (exception_ejtag & icache_stalli)
begin
   stalled_ex_r <=1'b1;
   stalled_pc   <=exceptionaddr_ejtag;
end
else if (brbus_refetch& icache_stalli)
begin
   stalled_ex_r <=1'b1;
   stalled_ex_refetch <=1'b1;
   stalled_pc   <=brbus_value;
end
else if(icache_refill_ok  )
begin
   stalled_ex_r <= 1'b0;
   stalled_ex_refetch <=1'b0;
end

assign ir_static_br_mid   = ir_static_br_0 | ir_static_br_1;
assign ir_config_br_mid   = ir_config_br_0 | ir_config_br_1;
assign ir_jump_mid        = ir_jump_0 | ir_jump_1;
assign ir_jr31_mid        = ir_jr31_0 | ir_jr31_1;

wire branch_two_in_ir = ir_wait_bd_0 & ir_wait_bd_1;
wire select_value = (ir_static_br_0 | ir_static_br_1 & ~ir_wait_bd_0) ? 1'b1 : 1'b0; 
wire br_taken = ~(ir_config_br_0 | ir_config_br_1&~ir_wait_bd_0) ? select_value : status[1];
wire jump  = branch_two_in_ir ? ir_jump_0  : ir_jump_mid ;
wire jr31  = branch_two_in_ir ? ir_jr31_0  : ir_jr31_mid; 

assign clear =  reset | commitbus_ex | brbus_valid&brbus_err | stalled_ex_r ;

reg ir_in_enable ;

reg bd_rdy;
always @(posedge clock)
begin
  if(reset)
     bd_rdy <= 1'b0;
  else if(ir_wait_bd_1 & iresvalid_0)
     bd_rdy <= 1'b1;
  else if(bd_rdy & irbus_valid_1 ) //why?? br is in ir_1, so when new irbus_1 is dispatched. this bits is unuseful
     bd_rdy <= 1'b0;
end

//when fetch the branch's next instruction, then pc_temp should be br's target. Orelse sequential fetch 
/*
wire   sequential_fetch = (pc == irbus_pc0_plus8)&ir_wait_bd_0 | 
                          (pc == irbus_pc0_plus8)&ir_wait_bd_1&ir_rdy[1] |
                          (pc == irbus_pc0_plus8)&ir_wait_bd_1&(wait_bd&~bd_rdy) |  
                          (ir_one_word | DERET_IFLAG)&ir_wait_bd_0;
*/

wire   sequential_fetch = (pc == irbus_pc0_plus8)&(ir_wait_bd_0 | ir_wait_bd_1&(wait_bd&~bd_rdy | ir_rdy[1]) )|
                         // (pc == ir_pc_plus4)&(ir_wait_bd_1& ~bd_rdy)| //??what does it mean?
                          ((ir_pc[4:2] == 3'b111) | DERET_IFLAG)&ir_wait_bd_0;
assign branch_br    =  br_taken &(~ir_in_enable&sequential_fetch&~irstalli_reg & ~clear);
assign branch_jump  =  jump     &(~ir_in_enable&sequential_fetch&~irstalli_reg & ~clear);
assign branch_jr31  =  jr31     &(~ir_in_enable&sequential_fetch&~irstalli_reg & ~clear);
assign branch_next  = ~(branch_br | branch_jump | branch_jr31) & ~(clear | irstalli_reg);

wire[31:0] branch_pc_t;
assign branch_pc_t[31:2] = ir_pc[31:2] + offsetaddr_0[31:2] + 1'b1;
assign branch_pc_t[1:0]  = 2'b00;

wire [31:0] pc_plus8 = pc + 4'b1000;
wire [31:0] pc_plus4 = pc + 4'b0100;

assign branchaddr_br   = (ir_config_br_0|ir_static_br_0) ?(branch_pc_t): (pc+offsetaddr_1); //~~~
assign branchaddr_jump =  ir_jump_0 ? {ir_pc_plus4[31:28],ir_inst_0[25:0],2'b00}: {pc[31:28],ir_inst_1[25:0],2'b00};

//before dret from debug, then fetch only one instrucion, and the iresbus_valid_1 also is 0

assign branchaddr_next =  ~(pc[4:2] ==3'b111)& ~DERET_IFLAG ?
                           (pc_plus8): (pc_plus4);

assign branchaddr_jr31 =  RAS[pop_ras_index];

wire commitbus_ex_refill    = ((commitbus_excode[5:0]==`EX_TLBLR)    | (commitbus_excode[5:0]==`EX_TLBSR));
wire commitbus_ex_softreset = ((commitbus_excode[5:0]==`EX_SOFTRESET)| (commitbus_excode[5:0]==`EX_NMI));
wire commitbus_ex_ejtag     = (commitbus_excode[5:0]==`EX_DINT)    | (commitbus_excode[5:0]==`EX_DSS)    |
                              (commitbus_excode[5:0]==`EX_SDBBP)   |(commitbus_excode[5:0]==`EX_DIB)     |
                              (commitbus_excode[5:0]==`EX_DDBL)    |(commitbus_excode[5:0]==`EX_DDBS)    |
                              (commitbus_excode[5:0]==`EX_DDBLIMPR)|(commitbus_excode[5:0]==`EX_DDBSIMPR)|
                              (DEBUG_MODE&&(~exception_reset));
//exception addr
wire[31:0] base_addr_bev1   = 32'Hbfc00200; 
wire[31:0] base_addr_bev0   = {ebase[31:12],12'b0};
wire[31:0] base_addr = bev ? base_addr_bev1: base_addr_bev0;  

wire[15:0] offset_vec;
wire[15:0] offset;
assign offset_vec  = bev ? 16'h0200:
                           16'h0200 + ({8'b0,({3{intctl_vs[0]}} & intvec_no),5'b0} |
                                       {7'b0,({3{intctl_vs[1]}} & intvec_no),6'b0} | 
                                       {6'b0,({3{intctl_vs[2]}} & intvec_no),7'b0} |
                                       {5'b0,({3{intctl_vs[3]}} & intvec_no),8'b0} |
                                       {4'b0,({3{intctl_vs[4]}} & intvec_no),9'b0} );

assign offset = commitbus_ex_refill&(exl==0) ? 16'H0000 : 
               (commitbus_excode[5:0] == `EX_INTERRUPT)&cause_iv ? offset_vec : 16'H180;

assign exceptionaddr_reset      = (commitbus_excode[5:0]==`EX_EJTAGBOOT)?EX_ENTRY_EJTAG_PROB:EX_ENTRY_RESET;
assign exceptionaddr_ejtag      = (~NODCR & PROBEN & PROBTRAP)?EX_ENTRY_EJTAG_PROB:EX_ENTRY_EJTAG_MEM;
assign exceptionaddr_normal     =  base_addr + offset; 

assign exception_reset          =  EJTAGBOOT | (commitbus_ex & commitbus_ex_softreset);
assign exception_normal         = ~(reset|EJTAGBOOT) & commitbus_ex  & ~commitbus_ex_softreset & 
                                   (~commitbus_ex_ejtag);
assign exception_ejtag          = ~(reset||EJTAGBOOT) & commitbus_ex & commitbus_ex_ejtag;

//when commitbus_ex , there isn't brbus_valid
assign brbus_refetch            = ~(reset|EJTAGBOOT |commitbus_ex) & brbus_valid&brbus_err & ~brbus_irstalli_bd ; 

//when older brbus_err is coming, stalled it. and the newer shouldn't be fetched 
         

wire stalled_refetch_clear = icache_refill_ok&(brbus_err&brbus_valid | commitbus_ex | reset)  ;

assign pc_temp =(({32{exception_reset &~icache_stalli | reset}} & exceptionaddr_reset)  |
                 ({32{exception_normal&~icache_stalli}} & exceptionaddr_normal) |
                 ({32{exception_ejtag &~icache_stalli}} & exceptionaddr_ejtag ) |
                 ({32{brbus_refetch   &~icache_stalli}} & brbus_value)          |
                 ({32{irstalli_next_reg& ~clear      }} & ir_pc_plus4)          | 
                 ({32{irstalli_reg & ~irstalli_next_reg&~clear }} & irstalli_pc) | 
                 ({32{stalled_ex_r & ~stalled_refetch_clear }} & stalled_pc)) |
                 ({32{branch_br  }} & branchaddr_br)        |
                 ({32{branch_jump}} & branchaddr_jump)      |
                 ({32{branch_jr31}} & branchaddr_jr31)      |
                 ({32{branch_next}} & branchaddr_next)     ; 

/**************way prediction**************************************************/
wire[1:0] iresbus_seq_link  , iresbus_noseq_link_0,iresbus_noseq_link_1;

wire[1:0] br_other_link;    //way prediction for branch ex refetch 
wire[1:0] other_seq_link;
//wire irbus_taken_br; 

wire [49 :0] way_pred;


assign iresbus_seq_link     = iresbus_link_info[9:8];
assign iresbus_noseq_link_0 = (pc[4:3] == 2'b00) ? iresbus_link_info[1:0]:
                              (pc[4:3] == 2'b01) ? iresbus_link_info[3:2]:
                              (pc[4:3] == 2'b10) ? iresbus_link_info[5:4]: iresbus_link_info[7:6];
assign iresbus_noseq_link_1 = (pc_plus4[4:3] == 2'b00) ? iresbus_link_info[1:0]: 
                              (pc_plus4[4:3] == 2'b01) ? iresbus_link_info[3:2]:
                              (pc_plus4[4:3] == 2'b10) ? iresbus_link_info[5:4]: iresbus_link_info[7:6];

wire [1:0] way_pred_seq = (branchaddr_next[4:2] == 3'b000)?  iresbus_seq_link: iresbus_way_hit;
wire [1:0] way_pred_noseq =ir_wait_bd_0 ? ir_noseq_link_0: ir_noseq_link_1; 

wire [1:0] way_pred_t0 = ((ir_pc[4:2] == 3'b111) | DERET_IFLAG) ? ir_seq_link  : ir_way_hit;

wire pre_sel = commitbus_ex | reset | EJTAGBOOT | brbus_valid&brbus_err | irstalli_r | branch_jr31 | branch_jump |
               icache_refill_ok&stalled_ex_r;

wire [1:0] pre_value = (commitbus_ex | reset | EJTAGBOOT | icache_refill_ok&stalled_ex_r&~stalled_ex_refetch)  ? 2'b00 :
                       (brbus_valid&brbus_err | icache_refill_ok&stalled_ex_refetch)   ? brbus_other_link   :
                       irstalli_r                ? way_pred_t0:
                       branch_jump               ? way_pred_noseq :RAS_way[pop_ras_index];

//for branch next and in the same cache line, we'll deal with in icache module for critical path
assign way_pred[1:0] =  pre_sel                  ? pre_value : 
                        branch_br                ? way_pred_noseq : way_pred_seq; 
//whether the way pred is sequential prediction
assign way_pred[2] = branch_next&(branchaddr_next[4:2] == 3'b000);  

assign way_pred[34:3]  = branch_next ? {iresbus_paddr_tag,pc[11:0]}:{ir_paddr_tag,ir_pc[11:0]};

assign way_pred[44:35] = branch_next ? iresbus_link_info :  ir_link_info;

assign way_pred[46:45] = branch_next ? iresbus_way_hit   :  ir_way_hit;

wire   update_link_disable = branch_next     ? (ires_ex_0 | ires_ex_1 | iresbus_no_update)   :
                             ir_wait_bd_0    ?  ir_ex[0]                  : ir_ex[1];

//when link came from ras, in-link_seq, irstalli_r ,we needn't update . 
//and especially, when the intruction where link came from has exception, we can't update link 
assign way_pred[47]    = branch_jr31 | branch_next&~(branchaddr_next[4:2] == 3'b000) | 
                         irstalli_r|
                         icache_refill_ok&stalled_ex_r |  //stalled pc , we no update
                         update_link_disable;

assign way_pred[49:48] = branch_next            ? iresbus_v_lock         : ir_v_lock;
                         
////// for branch prediction error resolved's way prediction
//assign irbus_taken_br   = ir_config_br_mid&status[1] | ir_static_br_mid ;

assign other_seq_link   = (ir_pc[4:3] == 2'b11) ? ir_seq_link : ir_way_hit;
assign br_other_link    = br_taken ? other_seq_link :
                          (ir_config_br_0 | ir_static_br_0) ? ir_noseq_link_0   : ir_noseq_link_1;  
/*****************part4: dispatch to decoder**************/     
wire ir_link_0  = ir_blink_0 | ir_jal_0 ;
wire ir_link_1  = ir_blink_1 | ir_jal_1; //to cancel blink stall issue

wire ir_link_mid = branch_two_in_ir  ? ir_link_0 :  (ir_link_1 | ir_link_0);
wire ir_jreg_mid = branch_two_in_ir  ? ir_jreg_0 :  (ir_jreg_1 | ir_jreg_0);

assign irvalid[0] = ir_rdy[0]   | ir_adei[0] | ir_tlbii[0] |
                    ir_tlbir[0] | ibe[0]     | ir_dib[0];

assign irvalid[1] = ir_rdy[1]   | ir_adei[1] | ir_tlbii[1] |
                    ir_tlbir[1] | ibe[1]     | ir_dib[1];

wire[31:0] jreg_pc = irbus_valid_0 & ir_jreg_0 ? irbus_pc0_plus8 : irbus_pc1_plus8;

wire ir_br_0 = ir_config_br_0 | ir_static_br_0;

wire ir_br_notaken_0 = ir_config_br_0&~status[1] ;
wire ir_br_notaken_1 = ir_config_br_1&~status[1] ;

wire ir_br_notaken =  branch_two_in_ir ? ir_br_notaken_0 : (ir_br_notaken_1 | ir_br_notaken_0);

wire[31:0] notaken_addr =  ir_br_0 ? irbus_pc0_plus8: irbus_pc1_plus8;

wire[31:0] taken_addr =  ir_br_notaken ? branchaddr_br : 
                         jr31          ? branchaddr_jr31:
                         ir_link_mid   ? ras_push_pc: 
                         ir_jreg_mid   ? jreg_pc: notaken_addr ; //for br ,it is other path's addr

 //blikely is block begin,brq should has  two empty entries
 // otherelse when blikely is sent to brq, brq shouldn't full
wire brq_no_out_0  = (brq_full&(irbus_block_begin_0| ir_blikely_0)) |
                     (brq_tail_next_valid&ir_blikely_0&irbus_block_begin_0);

wire brq_no_out_1  = (brq_full & (irbus_block_begin_1 | ir_blikely_1))  |
                     (brq_tail_next_valid &ir_blikely_1&(irbus_block_begin_1 | irbus_block_begin_0));

assign irbus_valid_0       = irvalid[0] & (~qfull[1]) & ~brq_no_out_0 &
                             (~(qfull[0]|brq_no_out_1)|~irvalid[1]); //the two inst should be sent to queue simutiously
assign irbus_inst_0[31:0]  = ir_inst [0];
assign irbus_pc_0[31:0]    = ir_pc   ; 
assign irbus_bd_0          = wait_bd   ;
assign irbus_rdy_0         = ir_rdy  [0];
assign irbus_adei_0        = ir_adei [0];
assign irbus_tlbii_0       = ir_tlbii[0];
assign irbus_tlbir_0       = ir_tlbir[0];
assign irbus_dib_0         = ir_dib  [0];
assign irbus_ibe_0         = ibe     [0];
assign irbus_taken_addr_0  = taken_addr; 
assign irbus_pred_status_0 = status;
assign irbus_gshare_0      = gshare_reg;
assign irbus_rashead_0     = ras_head; 
assign irbus_other_link_0  = br_other_link;
assign irbus_block_begin_0 = block_begin;

assign irbus_valid_1       = irvalid[1] & irbus_valid_0 & ~irstalli_next;
assign irbus_inst_1[31:0]  = ir_inst[1]; 
assign irbus_pc_1[31:0]    = ir_pc_plus4; 
assign irbus_bd_1          = ir_wait_bd_0&irbus_valid_0; 
assign irbus_rdy_1         = ir_rdy  [1];
assign irbus_adei_1        = ir_adei [1];
assign irbus_tlbii_1       = ir_tlbii[1];
assign irbus_tlbir_1       = ir_tlbir[1];
assign irbus_dib_1         = ir_dib[1];
assign irbus_ibe_1         = ibe[1];
assign irbus_taken_addr_1  = taken_addr; 
assign irbus_pred_status_1 = status;
assign irbus_gshare_1      = gshare_reg;
assign irbus_rashead_1     = ras_head; 
assign irbus_other_link_1  = br_other_link;
assign irbus_block_begin_1 = (wait_bd| ir_ret_0) ? 1'b1 : 1'b0; //when ir_0 is bd

/*******part 5: IR enable and PC enable************************/
assign instout_valid    = ~( (qfull[1]|brq_no_out_0)&ir_noempty | irvalid[1]&(qfull[0] |brq_no_out_1 )) &
                          ~(irstalli | stalled_ex_r | qstalli );

assign ires_ex_0        = (iresbus_adei_0|iresbus_tlbii_0|iresbus_tlbir_0|iresbus_ibe_0)| iresbus_dib_0 ;
assign iresvalid_0      = (ires_ex_0 |int_trigger)& (~irstalli_r &instout_valid)| iresbus_valid_0  ;
assign ires_ex_1        = (iresbus_adei_1|iresbus_tlbii_1|iresbus_tlbir_1|iresbus_ibe_1)| iresbus_dib_1;
assign iresvalid_1      = (ires_ex_1|int_trigger) & (~irstalli_r &instout_valid) | iresbus_valid_1 ;

assign  clear_t =  reset | commitbus_ex | brbus_valid&brbus_err;

assign pc_in_en         = ( reset|(commitbus_ex| brbus_valid&brbus_err&~brbus_irstalli_bd) & ~icache_stalli |
                            irstalli_r& instout_valid&~icache_stalli |
                            icache_refill_ok&stalled_ex_r |
                           (instout_valid&ires_ex_0 & ~(clear_t & icache_stalli)) ) |
                            iresbus_valid_0& ~(clear_t & icache_stalli) ; 

assign pc_en_nowayhit = ( reset|(commitbus_ex| brbus_valid&brbus_err&~brbus_irstalli_bd) & ~icache_stalli|
                          irstalli_r& instout_valid&~icache_stalli | 
                          icache_refill_ok&stalled_ex_r |
                          (instout_valid&ires_ex_0)& ~(clear_t & icache_stalli)) |
                          iresbus_cacherdy_0 &~(clear_t & icache_stalli); 

assign instout_pc_in_en = pc_in_en;
assign instout_pc[31:0] = pc_temp[31:0];

wire branch_taken = branch_br | branch_jump | branch_jr31 ;

wire br_taken_sequntial   = (pc == branchaddr_br);
wire jump_taken_sequntial = (pc == branchaddr_jump);
wire jr31_taken_sequntial = (pc == branchaddr_jr31);

//for taken target is equle to not taken target
always @(posedge clock)
begin
 if (pc_in_en)
     ir_in_enable <= sequential_fetch&(br_taken_sequntial&branch_br |jump_taken_sequntial&branch_jump |
                                        jr31_taken_sequntial&branch_jr31 ) ;
end


wire ir_in_en_0 = ~(branch_taken&ir_wait_bd_0 &~((ir_pc[4:2] == 3'b111) | DERET_IFLAG)) | ir_in_enable;
wire ir_in_en_1 = ~(branch_taken&(ir_wait_bd_0 | ir_wait_bd_1) )        | ir_in_enable;



assign tail_in_en_0     =  iresvalid_0& ir_in_en_0 ;
assign tail_in_en_1     =  iresvalid_1& ir_in_en_1;

assign tail_in_en_inst_0 = (iresbus_valid_0 |int_trigger&instout_valid)& ~irstalli_r & 
                            ir_in_en_0 ;
assign tail_in_en_inst_1 = (iresbus_valid_1 |int_trigger&instout_valid)& ~irstalli_r& 
                            ir_in_en_1;

always @ (posedge clock) begin
if (reset | commitbus_ex | brbus_valid&brbus_err| irbus_valid_0 &irstalli_next)
  begin
    ir_rdy[0] <= 1'b0;
    ir_adei[0] <=1'b0;
    ir_tlbii[0] <= 1'b0;
    ir_tlbir[0] <= 1'b0;
    ir_dib[0] <= 1'b0;
    ibe[0] <= 1'b0;
    ir_rdy[1] <= 1'b0;
    ir_adei[1] <=1'b0;
    ir_tlbii[1] <= 1'b0;
    ir_tlbir[1] <= 1'b0;
    ir_dib[1] <= 1'b0;
    ibe[1] <= 1'b0;
  end
else if (irstalli_next)
  begin
    ir_rdy[1]   <= 1'b0;
    ir_adei[1]  <= 1'b0;
    ir_tlbii[1] <= 1'b0;
    ir_tlbir[1] <= 1'b0;
    ir_dib[1]   <= 1'b0;
    ibe   [1]   <= 1'b0;
  end
else if(~irvalid[0]&tail_in_en_0 | irbus_valid_0)
 begin
    ir_rdy[0] <= (iresbus_valid_0 | int_trigger)&ir_in_en_0;
    ir_adei[0] <= iresbus_adei_0&ir_in_en_0;
    ir_tlbii[0] <= iresbus_tlbii_0&ir_in_en_0;
    ir_tlbir[0] <= iresbus_tlbir_0&ir_in_en_0;
    ir_dib[0] <= iresbus_dib_0&ir_in_en_0;
    ibe[0]    <= iresbus_ibe_0&ir_in_en_0;
    
    ir_rdy[1] <= iresbus_valid_1&ir_in_en_1;
    ir_adei[1] <= iresbus_adei_1&ir_in_en_1;
    ir_tlbii[1] <= iresbus_tlbii_1&ir_in_en_1;
    ir_tlbir[1] <= iresbus_tlbir_1&ir_in_en_1;
    ir_dib[1] <= iresbus_dib_1&ir_in_en_1;
    ibe[1]    <= iresbus_ibe_1&ir_in_en_1;
  end
end

always @(posedge clock)
begin
if(reset | commitbus_ex | brbus_valid&brbus_err)
begin     
     ir_noseq_link[0] <= 2'b00;
     ir_seq_link  <= 2'b00;
     ir_link_info <= 10'b00;
     ir_paddr_tag <= 20'b00;
     ir_v_lock    <= 2'b00;
     ir_way_hit   <= 2'b00;
     ir_ex[0]     <= 1'b0;
     ir_one_word  <= 1'b0;

     ir_noseq_link[1] <= 2'b00;
     ir_ex[1]     <= 1'b0;
end
else if (irstalli_next)
  begin
   ir_noseq_link[1] <= 2'b00;
   ir_ex[1]     <= 1'b0;
  end
else 
begin
   if (tail_in_en_0)
     begin
       ir_noseq_link[0] <= iresbus_noseq_link_0;
       ir_seq_link    <= iresbus_seq_link;
       ir_link_info   <= iresbus_link_info;
       ir_paddr_tag   <= iresbus_paddr_tag;
       ir_v_lock      <= iresbus_v_lock;
       ir_way_hit     <= iresbus_way_hit;
       ir_ex[0]       <= ires_ex_0 | iresbus_no_update;
       ir_pc          <= pc;
       ir_one_word    <= iresbus_one_word; 
     end
   if(tail_in_en_1) 
     begin
       ir_noseq_link[1] <= iresbus_noseq_link_1;
       ir_ex[1]       <= ires_ex_1 | iresbus_no_update;
     end
end  
end

always @(posedge clock)
begin
if(reset | commitbus_ex | brbus_valid&brbus_err)
begin
  ir_inst[0] <= 32'b0;
  ir_int     <= 1'b0;
  ir_inst[1] <= 32'b0;
end
else if (irstalli_next)
begin 
  ir_inst[1] <= 32'b0;
end
else 
begin
  if(tail_in_en_inst_0)
  begin
    ir_inst[0] <= iresbus_value_0;
    ir_int     <= int_trigger;
  end  
  if(tail_in_en_inst_1)
    ir_inst[1] <= iresbus_value_1;
end
end

always @(posedge clock)
begin
if (pc_in_en)    
    pc[31:0]      <= pc_temp[31:0]; 
end
 
always @ (posedge clock)
begin
 if(reset | EJTAGBOOT | commitbus_ex)
   block_begin  <= 1'b1;
 else if (brbus_valid&brbus_err)
   block_begin  <= 1'b1;
 else if(irbus_valid_1&(irbus_bd_1 | ir_ret_1) | irbus_valid_0&(irbus_bd_0 | ir_ret_0)&~irbus_valid_1)
   block_begin  <= 1'b1;
 else if (irbus_valid_0 &block_begin)
    block_begin <= 1'b0;
end

always @(posedge clock)
begin
  if(reset | commitbus_ex | brbus_valid&brbus_err | EJTAGBOOT)
     wait_bd <= 1'b0;
  else if (irbus_valid_0 & ~irbus_valid_1 &ir_wait_bd_0 | irbus_valid_1 & ir_wait_bd_1)
     wait_bd<= 1'b1;
  else if (wait_bd & irbus_valid_0)
     wait_bd<= 1'b0;
end

assign insts_fetched_o = ((pc[4:2] == 3'b111) | DERET_IFLAG)? {2{iresvalid_0}}&{2'b01} : {2{iresvalid_0}}&{2'b10};

endmodule 

module bht_module(clock,reset,read_addr,gshare, write_addr,write_en, write_data,status0, status1);
input clock;
input reset;
input[7:0] read_addr;
input[7:0] gshare;
input[7:0] write_addr;
input write_en;
input[`BHTbits-1:0] write_data;
output[`BHTbits-1:0] status0, status1;

reg[`BHTbits-1:0] bht_regfiles[`BHTsize-1:0];

wire [7:0]addr_even;
wire [7:0]addr_next_odd;

wire [`BHTbits-1:0] status_even, status_odd;
wire [`BHTbits-1:0] status_even_t, status_odd_t;

assign addr_even          = { read_addr[7:1] ^ gshare[7:1], 1'b0};
assign addr_next_odd      =  {read_addr[7:1] ^ gshare[7:1], 1'b1 } ;

assign status_even_t  = bht_regfiles[addr_even];
assign status_odd_t   = bht_regfiles[addr_next_odd];

assign status0 = gshare[0]^read_addr[0] ? status_odd_t  : status_even_t;
assign status1 = gshare[0]^read_addr[0] ? status_even_t : status_odd_t;

wire bht_gating_clock;
assign bht_gating_clock = clock;

integer i;
always @ (posedge bht_gating_clock)
if(reset)
  begin
   for(i = 0; i<`BHTsize; i=i+1)
    bht_regfiles[i] <= 2'b10;
  end
else if( write_en)
  begin
    bht_regfiles[write_addr] <= write_data;
  end
 
endmodule

module predecode_module(reset       , DEBUG_MODE   ,DSS_ENABLE ,ir_inst,
                        irstalli    , ir_wait_bd   , offsetaddr,
                        ir_config_br, ir_static_br , ir_jump   , ir_jreg ,
                        ir_blikely  , ir_blink     , 
                        ir_jr31     , ir_jalr31    , ir_jal    , 
                        ir_back_br  , ir_ret       , ir_wait  );

input        reset                  ;
input[31:0]  ir_inst                ;
input        DEBUG_MODE             ;
input        DSS_ENABLE             ;
output       irstalli               ;
output       ir_wait_bd             ;
output       ir_config_br           ;
output       ir_static_br           ;
output       ir_jump     , ir_jreg  ;
output       ir_blikely  , ir_blink ;
output       ir_jr31     , ir_jalr31;
output       ir_jal                 ;
output[31:0] offsetaddr             ;
output       ir_back_br             ;
output       ir_ret                 ;
output       ir_wait                ;

//Signal Declarations
wire         irstalli               ;
wire         ir_wait_bd             ;
wire         ir_config_br           ;
wire         ir_static_br           ;
wire         ir_jump     , ir_jreg  ;
wire         ir_blikely  , ir_blink ;
wire         ir_jr31     , ir_jalr31;
wire[31:0]   offsetaddr             ;
wire         ir_back_br             ;
wire         ir_ret                 ;
wire         ir_wait                ;

wire         ir_bposge32            ;
wire         dss_stalli             ;
wire         ir_beq  , ir_beql , ir_bne  , ir_bnel  , ir_blez  , ir_blezl  , ir_bgtz  , ir_bgtzl  ,
             ir_bltz , ir_bltzl, ir_bgez , ir_bgezl , ir_bltzal, ir_bltzall, ir_bgezal, ir_bgezall,
             ir_bc1f , ir_bc1t , ir_bc1fl, ir_bc1tl , ir_j     , ir_jr     , ir_jal   , 
             ir_mtc0 , ir_tlbr , ir_eret , ir_icache, ir_deret , ir_ll     , ir_synci , 
             ir_tlbwi, ir_tlbwr, ir_jalr , ir_di    , ir_ei;
wire         special_jr,regimm,cop1_bc,rt_zero,cop0,cop0_func;

assign rt_zero    = ~ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] & ~ir_inst[17] & ~ir_inst[16];
assign regimm     = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] & ~ir_inst[28] & ~ir_inst[27] &  ir_inst[26];
assign cop1_bc    = ~ir_inst[31] &  ir_inst[30] & ~ir_inst[29] & ~ir_inst[28] & ~ir_inst[27] &  ir_inst[26] &
                    ~ir_inst[25] &  ir_inst[24] & ~ir_inst[23] & ~ir_inst[22] & ~ir_inst[21];

//also for jalr.hb and jr.hb
assign special_jr = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] & ~ir_inst[28] & ~ir_inst[27] & ~ir_inst[26] &
                   ~ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] & ~ir_inst[17] & ~ir_inst[16] &
                   /* ~ir_inst[10] &*/ ~ir_inst[ 9] & ~ir_inst[ 8] & ~ir_inst[ 7] & ~ir_inst[ 6] &
                    ~ir_inst[ 5] & ~ir_inst[ 4] &  ir_inst[ 3] & ~ir_inst[ 2] & ~ir_inst[ 1] ;
assign cop0       = ~ir_inst[31] &  ir_inst[30] & ~ir_inst[29] & ~ir_inst[28] & ~ir_inst[27] & ~ir_inst[26] ;
assign cop0_func  = cop0 & rt_zero &
                    ir_inst[25] & ~ir_inst[24] & ~ir_inst[23] & ~ir_inst[22] & ~ir_inst[21] &
                    ~ir_inst[15] & ~ir_inst[14] & ~ir_inst[13] & ~ir_inst[12] & ~ir_inst[11] &
                    ~ir_inst[10] & ~ir_inst[9 ] & ~ir_inst[8 ] & ~ir_inst[ 7] & ~ir_inst[ 6];
                                                                                                                              
assign  ir_j      = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] & ~ir_inst[28] &  ir_inst[27] & ~ir_inst[26];
assign  ir_jal    = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] & ~ir_inst[28] &  ir_inst[27] &  ir_inst[26];
assign  ir_beq    = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] &  ir_inst[28] & ~ir_inst[27] & ~ir_inst[26];
assign  ir_beql   = ~ir_inst[31] &  ir_inst[30] & ~ir_inst[29] &  ir_inst[28] & ~ir_inst[27] & ~ir_inst[26];
assign  ir_bne    = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] &  ir_inst[28] & ~ir_inst[27] &  ir_inst[26];
assign  ir_bnel   = ~ir_inst[31] &  ir_inst[30] & ~ir_inst[29] &  ir_inst[28] & ~ir_inst[27] &  ir_inst[26];
assign  ir_blez   = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] &  ir_inst[28] &  ir_inst[27] & ~ir_inst[26] & rt_zero;
assign  ir_blezl  = ~ir_inst[31] &  ir_inst[30] & ~ir_inst[29] &  ir_inst[28] &  ir_inst[27] & ~ir_inst[26] & rt_zero;
assign  ir_bgtz   = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29] &  ir_inst[28] &  ir_inst[27] &  ir_inst[26] & rt_zero;
assign  ir_bgtzl  = ~ir_inst[31] &  ir_inst[30] & ~ir_inst[29] &  ir_inst[28] &  ir_inst[27] &  ir_inst[26] & rt_zero;
assign  ir_bltz   = regimm       & ~ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] & ~ir_inst[17] & ~ir_inst[16];
assign  ir_bltzl  = regimm       & ~ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] &  ir_inst[17] & ~ir_inst[16];
assign  ir_bgez   = regimm       & ~ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] & ~ir_inst[17] &  ir_inst[16];
assign  ir_bgezl  = regimm       & ~ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] &  ir_inst[17] &  ir_inst[16];
assign  ir_bltzal = regimm       &  ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] & ~ir_inst[17] & ~ir_inst[16];
assign  ir_bltzall= regimm       &  ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] &  ir_inst[17] & ~ir_inst[16];
assign  ir_bgezal = regimm       &  ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] & ~ir_inst[17] &  ir_inst[16];
assign  ir_bgezall= regimm       &  ir_inst[20] & ~ir_inst[19] & ~ir_inst[18] &  ir_inst[17] &  ir_inst[16];
assign  ir_bc1f   = cop1_bc      & ~ir_inst[17] & ~ir_inst[16];
assign  ir_bc1t   = cop1_bc      & ~ir_inst[17] &  ir_inst[16];
assign  ir_bc1fl  = cop1_bc      &  ir_inst[17] & ~ir_inst[16];
assign  ir_bc1tl  = cop1_bc      &  ir_inst[17] &  ir_inst[16];
assign  ir_jr     = special_jr   & ~ir_inst[15] & ~ir_inst[14] & ~ir_inst[13] & ~ir_inst[12] & ~ir_inst[11] & ~ir_inst[ 0];
assign  ir_jalr   = special_jr   &  ir_inst[ 0];
//for ras operation
assign  ir_jalr31 = special_jr   &  ir_inst[ 0] &  ir_inst[15] &  ir_inst[14] &  ir_inst[13] &  ir_inst[12] &  ir_inst[11];
assign  ir_jr31   = ir_jr        &  ir_inst[25] &  ir_inst[24] &  ir_inst[23] &  ir_inst[22] &  ir_inst[21];  


assign  ir_icache = ir_inst[31] & ~ir_inst[30] &  ir_inst[29] &  ir_inst[28] &  ir_inst[27] &  ir_inst[26] & ~ir_inst[16];
assign  ir_tlbr   = cop0_func   & ~ir_inst[5]  & ~ir_inst[ 4] & ~ir_inst[ 3] & ~ir_inst[ 2] & ~ir_inst[ 1] &  ir_inst[ 0];
assign  ir_tlbwi  = cop0_func&ir_inst[25] & ~ir_inst[5]  & ~ir_inst[ 4] & ~ir_inst[ 3] & ~ir_inst[ 2] & ir_inst[ 1] &  ~ir_inst[ 0];

assign  ir_tlbwr  = cop0_func&ir_inst[25] & ~ir_inst[5]  & ~ir_inst[ 4] & ~ir_inst[ 3] & ir_inst[ 2] & ir_inst[ 1] &  ~ir_inst[ 0];


assign  ir_eret   = cop0_func   & ~ir_inst[5]  &  ir_inst[ 4] &  ir_inst[ 3] & ~ir_inst[ 2] & ~ir_inst[ 1] & ~ir_inst[ 0];
assign  ir_deret  = cop0_func   & ~ir_inst[5]  &  ir_inst[ 4] &  ir_inst[ 3] &  ir_inst[ 2] &  ir_inst[ 1] &  ir_inst[ 0];
assign  ir_mtc0   = cop0    & ~ir_inst[25] & ~ir_inst[24] &  ir_inst[23] & ~ir_inst[22] & ~ir_inst[21] &
                                  ~ir_inst[10] & ~ir_inst[ 9] & ~ir_inst[ 8] & ~ir_inst[ 7] & ~ir_inst[ 6] &
                                  ~ir_inst[ 5] & ~ir_inst[ 4] & ~ir_inst[ 3]; 
assign  ir_di     = cop0    & ~ir_inst[25] & ir_inst[24] &  ~ir_inst[23] & ir_inst[22] & ir_inst[21] &
                                 ~ir_inst[15] & ir_inst[ 14] & ~ir_inst[ 13] & ~ir_inst[ 12] & ~ir_inst[11] &
                                 ~ir_inst[10] & ~ir_inst[ 9] & ~ir_inst[ 8] & ~ir_inst[ 7] & ~ir_inst[ 6] &
                                 ~ir_inst[ 5] & ~ir_inst[ 4] & ~ir_inst[ 3] &~ir_inst[2]&~ir_inst[1]&~ir_inst[0]; 
assign  ir_ei     = cop0    & ~ir_inst[25] & ir_inst[24] &  ~ir_inst[23] & ir_inst[22] & ir_inst[21] &
                                  ~ir_inst[15] & ir_inst[ 14] & ~ir_inst[ 13] & ~ir_inst[ 12] & ~ir_inst[11] &
                                  ~ir_inst[10] & ~ir_inst[ 9] & ~ir_inst[ 8] & ~ir_inst[ 7] & ~ir_inst[ 6] &
                                  ir_inst[ 5] & ~ir_inst[ 4] & ~ir_inst[ 3] &~ir_inst[2]&~ir_inst[1]&~ir_inst[0]; 
assign ir_bposge32 = ~ir_inst[31] & ~ir_inst[30] & ~ir_inst[29]& ~ir_inst[28] & ~ir_inst[27] & ir_inst[26] &
                     ~ir_inst[25] & ~ir_inst[24] & ~ir_inst[23]& ~ir_inst[22] & ~ir_inst[21] &
                      ir_inst[20] & ir_inst[19] & ir_inst[18]& ~ir_inst[17] & ~ir_inst[16];

assign ir_wait  = cop0 & ir_inst[25] & ir_inst[ 5] & ~ir_inst[ 4] & ~ir_inst[ 3] &~ir_inst[2]&~ir_inst[1]&~ir_inst[0]; 

assign ir_ll      = ir_inst[31]& ir_inst[30] & ~ir_inst[29] &~ir_inst[28]&~ir_inst[27]&~ir_inst[26];
assign ir_synci   = ~ir_inst[31]& ~ir_inst[30] & ~ir_inst[29] &~ir_inst[28]&~ir_inst[27]&ir_inst[26] &
                     ir_inst[20] & ir_inst[19] &ir_inst[18]&ir_inst[17]&ir_inst[16] ;
assign dss_stalli =(~ir_wait_bd)&&(~DEBUG_MODE)&&DSS_ENABLE;

assign  irstalli  = ir_synci| ir_icache  | ir_tlbr | ir_eret  | ir_deret |
                    ir_mtc0 | dss_stalli | ir_ll   | ir_tlbwr | ir_tlbwi |
                    ir_di   | ir_ei;
                                                                                                                              
assign  ir_wait_bd   = ir_static_br | ir_config_br | ir_jreg | ir_jump;
assign  ir_jreg      = ir_jr | ir_jalr;
assign  ir_jump      = ir_j  | ir_jal;
assign  ir_static_br = ir_beql  | ir_bnel   | ir_blezl | ir_bgtzl |
                       ir_bltzl | ir_bgezl  | ir_bc1fl | ir_bc1tl |
                       ir_blink ; 
assign  ir_config_br = ir_beq  |ir_bne   |ir_blez |ir_bgtz   |
                       ir_bltz | ir_bgez |ir_bc1f |ir_bc1t 
                       | ir_bposge32
                       ;
assign  ir_blink     = ir_bltzall | ir_bgezall | ir_bltzal  | ir_bgezal;
assign  ir_blikely   = ir_beql    | ir_bnel    | ir_blezl   | ir_bgtzl   |
                       ir_bltzl   | ir_bgezl   | ir_bltzall | ir_bgezall |
                       ir_bc1fl   | ir_bc1tl;

assign ir_back_br    = ir_inst[15];

assign offsetaddr[31:0]={{14{ir_inst[15]}},ir_inst[15:0],2'b00};

assign ir_ret = ir_eret | ir_deret;

endmodule
