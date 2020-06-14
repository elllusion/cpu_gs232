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

module godson_icache_module(
    clock,reset,
	commitbus_ex,i_laddr,icachepaddr,memres,
    dicache,  
    inst_cache_block,
    inst_uncache_block,
    PENDING_IBE,IEXI, DERET_IFLAG, iresbus,imemraddr,
    IBE_FROM_CACHE
	,ram_to_icache,icache_to_ram,
	icache_stalli,icache_refill_ok,
    //performance counter
    icache_access, icache_hit_perf,
    icache_update, icache_way_hit,
    cache28_refill_ok,
    stalled_cycles_icachemiss_o,
    cr_cfg7_icache_i,
    cr_cfg6_cache0_all_i,
    brbus_to_icache
);

input clock,reset;
input commitbus_ex;
input [`Licachepaddr-1:0] icachepaddr;
input [64:0] i_laddr;
input [`Lmemres-1:0] memres;
input [1:0] brbus_to_icache;

input PENDING_IBE;
input IEXI;
input DERET_IFLAG;
input[1:0] cr_cfg7_icache_i;
input      cr_cfg6_cache0_all_i;
output [`Liresbus_2issue-1:0] iresbus;//fetch two instruction
output [`Lmemraddr-1:0] imemraddr;
output IBE_FROM_CACHE;
output cache28_refill_ok;

input  [`Lram_to_icache-1:0] ram_to_icache;
output [`Licache_to_ram-1:0] icache_to_ram;


input  [`Ltlb_to_icache-1:0] dicache;  //modified for MIPS32
input  inst_cache_block;
input  inst_uncache_block;

output icache_stalli, icache_refill_ok;
output icache_hit_perf, icache_access;
output icache_way_hit, icache_update;
output  stalled_cycles_icachemiss_o;


//////////////////////////////////////////////////////////////
// transform the memres signals to memres_mid used internally.
wire [53:0] memres_mid;

wire is_block;

reg  [7:0] cs_a;
wire memres_dr_dw;
wire [7:0] memres_index;
wire [3:0] memres_width;
wire memres_block_rdy;


wire block_word7;
wire block_rrdy;
wire dw_dr_rrdy;
wire single_rrdy;

////// all fields of cp0_memres_i bus
wire        valid           = memres[0];
//wire        cancel          = memres[1];
wire [ 2:0] memres_id       = memres[4:2];
wire [ 2:0] counter         = memres[7:5];
wire        memres_rd_rdy   = memres[8];
wire        memres_wr_rdy   = memres[9];
wire        memres_uncache_rdy = memres[10];
wire        is_dw           = memres[11];
wire [31:0] memres_data     = memres[43:12];

assign is_block = (memres_id == 3'b011);

always @(counter)
begin
   case(counter)
             3'b000:cs_a=8'b00000001;
             3'b001:cs_a=8'b00000010;
             3'b010:cs_a=8'b00000100;
             3'b011:cs_a=8'b00001000;
             3'b100:cs_a=8'b00010000;
             3'b101:cs_a=8'b00100000;
             3'b110:cs_a=8'b01000000;
             3'b111:cs_a=8'b10000000;
    endcase
end


reg  dw_reg;
always @( posedge clock)
if (reset)
   dw_reg<=1'b0;
else
   if (valid&is_dw)
      dw_reg<=~dw_reg;

reg [2:0] base_counter;
always @(posedge clock)
if (reset)
   base_counter<=3'b0;
else
  if (block_rrdy)
     base_counter<=3'b000;
  else
     if (valid&is_block)
        base_counter<=base_counter+1;

assign block_word7=(base_counter[2:0]==3'b111)?1'b1:1'b0;
assign block_rrdy=block_word7&valid&is_block;

assign dw_dr_rrdy=dw_reg&valid&is_dw&(memres_id==3'b111);

assign memres_dr_dw=dw_dr_rrdy;
assign memres_block_rdy=block_rrdy;
assign memres_index={8{is_block}}&cs_a[7:0]&{8{valid}};
assign memres_width=(is_block)?4'b0001:4'b0000;//used only when is_block
wire   memres_is_icache  = is_block;

assign memres_mid[0]    = is_dw&(memres_id==3'b111);// whether is dw.
assign memres_mid[1]    = valid;
assign memres_mid[4:2]  = counter;
assign memres_mid[5]    = memres_is_icache;
assign memres_mid[6]    = memres_dr_dw;// indicates directly-read dw
assign memres_mid[7]    = memres_rd_rdy;
assign memres_mid[8]    = memres_uncache_rdy;
assign memres_mid[9]    = memres_block_rdy;
assign memres_mid[17:10] = memres_index;
assign memres_mid[21:18] = memres_width;
assign memres_mid[53:22]  = memres_data;

wire[5:0] memres_to_write = {memres_block_rdy , memres_width, memres_is_icache};

//////////////////////////////////////////////////////////////
wire brbus_brvalid = brbus_to_icache[0];
wire brbus_brerr   = brbus_to_icache[1];

wire commitbus_ex_br = commitbus_ex | (brbus_brerr&brbus_brvalid);
wire di_valid;
wire [19:0] i_tag;
wire [4:0] i_offset;
wire [6:0] di_index;
wire [21:0] iram_index;
wire cancel_write;
wire [19:0] di_tag;    //added for cache16
wire [31:0] di_taglow; //added for cache8
wire di_cache0, di_cache8, di_cache16, di_cache28;


wire [255:0] cacheline_writtendata;
wire read_instr_again;
wire [31:0] imemrqueue_addr;
wire current_imemreq;
wire di_vaddr_dicache_valid;
wire [2:0] icache_v_offset;

/*for way prediction*/
wire way_select0, way_select1, way_select2, way_select3;
wire second_read;
wire[39:0] update_link;
wire update_link_way0, update_link_way1, update_link_way2, update_link_way3;
wire[7:0] data_cen;
wire      di_cache28_miss;
wire      cache28_refill_ok;


wire [1:0] di_set;
wire i0_tag_match,i0_tag_valid,i1_tag_match,i1_tag_valid;
wire i2_tag_match,i2_tag_valid,i3_tag_match,i3_tag_valid;
wire [31:0] i0_tag,i1_tag;
wire [31:0] i2_tag,i3_tag;
wire [63:0] i0_data,i1_data;
wire [63:0] i2_data,i3_data;
wire i0_lock,i1_lock;
wire i2_lock,i3_lock;
wire cr_icachestate0_value,cr_icachestate1_value;
wire cr_icachestate2_value,cr_icachestate3_value;
wire cache0_validtime0,cache0_validtime1;
wire cache0_validtime2,cache0_validtime3;


icache_control icache_ctrl_4way(.clock(clock),.reset(reset),
        .DERET_IFLAG(DERET_IFLAG),
	.commitbus_ex(commitbus_ex_br),
	.i1_laddr(i_laddr),.icachepaddr(icachepaddr),
        .PENDING_IBE(PENDING_IBE),.IEXI(IEXI),
	.iresbus(iresbus),.memres(memres_mid),
	.imemraddr(imemraddr),
   .inst_cache_block(inst_cache_block),
   .inst_uncache_block(inst_uncache_block),
	.dicache(dicache),
	.cr_icachestate0_value(cr_icachestate0_value),
	.cr_icachestate1_value(cr_icachestate1_value),
        .cr_icachestate2_value(cr_icachestate2_value),
        .cr_icachestate3_value(cr_icachestate3_value),
	.iram_index(iram_index),
	.i_tag(i_tag),.i_offset(i_offset),
	.di_valid(di_valid),.di_index(di_index),.di_set(di_set),
        .di_tag(di_tag),.di_taglow(di_taglow),
        .di_cache0(di_cache0),.di_cache8(di_cache8),
        .di_cache16(di_cache16), .di_cache28(di_cache28),
        .di_vaddr_dicache_valid(di_vaddr_dicache_valid),
        .i0_tag_match(i0_tag_match),.i0_tag_valid(i0_tag_valid),
        .i0_tag(i0_tag),.i0_lock(i0_lock),.i0_data(i0_data),
        .i1_tag_match(i1_tag_match),.i1_tag_valid(i1_tag_valid),
        .i1_tag(i1_tag),.i1_lock(i1_lock),.i1_data(i1_data),
        .i2_tag_match(i2_tag_match),.i2_tag_valid(i2_tag_valid),
        .i2_tag(i2_tag),.i2_lock(i2_lock),.i2_data(i2_data),
        .i3_tag_match(i3_tag_match),.i3_tag_valid(i3_tag_valid),
        .i3_tag(i3_tag),.i3_lock(i3_lock),.i3_data(i3_data),
        .cr_cfg7_icache_i(cr_cfg7_icache_i),
        .cancel_write(cancel_write),
        .current_imemreq(current_imemreq),
        .imemrqueue_addr(imemrqueue_addr),
        .read_instr_again(read_instr_again),
        .cacheline_writtendata(cacheline_writtendata),
        .cache0_validtime0(cache0_validtime0),
        .cache0_validtime1(cache0_validtime1),
        .cache0_validtime2(cache0_validtime2),
        .cache0_validtime3(cache0_validtime3),
        .IBE_FROM_CACHE(IBE_FROM_CACHE),
        .icache_v_offset(icache_v_offset),
        .way_select0(way_select0),
        .way_select1(way_select1),
        .way_select2(way_select2),
        .way_select3(way_select3),
        .second_read(second_read),
        .update_link(update_link),
        .update_link_way0(update_link_way0),
        .update_link_way1(update_link_way1),
        .update_link_way2(update_link_way2),
        .update_link_way3(update_link_way3),
        .data_cen(data_cen),
        .di_cache28_miss(di_cache28_miss),
        .cache28_refill_ok(cache28_refill_ok),
        .icache_stalli(icache_stalli),.icache_refill_ok(icache_refill_ok),
        .icache_hit_perf(icache_hit_perf),
        .icache_access(icache_access),
        .icache_way_hit(icache_way_hit),
        .stalled_cycles_icachemiss_o(stalled_cycles_icachemiss_o),
        .icache_update(icache_update));

//ICache Read & Write//
//Set 0//
icache_read_write icache_wr0(.clock(clock),.reset(reset),.i_set(2'b00),
	.cr_icachestate_value(cr_icachestate0_value),
	.iram_index(iram_index),
        .icache_v_offset(icache_v_offset),
	.i_tag(i_tag),.i_offset(i_offset),
	.di_valid(di_valid),.di_index(di_index),.di_set(di_set),
        .di_tag(di_tag),.di_taglow(di_taglow),
        .di_cache0(di_cache0),.di_cache8(di_cache8),
        .di_cache16(di_cache16), .di_cache28(di_cache28),
	.di_vaddr_dicache_valid(di_vaddr_dicache_valid),
	.memres(memres_to_write),.cancel_write(cancel_write),
	.i_tag_match(i0_tag_match),.i_tag_valid(i0_tag_valid),
	.i_0_tag(i0_tag),.i_0_lock(i0_lock),.i_0_data(i0_data),
        .from_ram(ram_to_icache[287:0]),
        .to_ram(icache_to_ram[360:0]),
        .current_imemreq(current_imemreq),
        .imemrqueue_addr(imemrqueue_addr),
        .read_instr_again(read_instr_again),
        .cr_cfg6_cache0_all(cr_cfg6_cache0_all_i),
        .way_select(way_select0),
        .second_read(second_read),
        .update_link_way(update_link_way0),
        .update_link(update_link),
        .cacheline_writtendata(cacheline_writtendata),
        .data_cen({data_cen[7:4],data_cen[0]}),
        .di_cache28_miss(di_cache28_miss),
        .cache0_validtime(cache0_validtime0));

//Set 1//
icache_read_write icache_wr1(.clock(clock),.reset(reset),.i_set(2'b01),
	.cr_icachestate_value(cr_icachestate1_value),
	.iram_index(iram_index),
        .icache_v_offset(icache_v_offset),
	.i_tag(i_tag),.i_offset(i_offset),
	.di_valid(di_valid),.di_index(di_index),.di_set(di_set),
        .di_tag(di_tag),.di_taglow(di_taglow),
        .di_cache0(di_cache0),.di_cache8(di_cache8),
        .di_cache16(di_cache16), .di_cache28(di_cache28),
	.di_vaddr_dicache_valid(di_vaddr_dicache_valid),
	.memres(memres_to_write),.cancel_write(cancel_write),
	.i_tag_match(i1_tag_match),.i_tag_valid(i1_tag_valid),
	.i_0_tag(i1_tag),.i_0_lock(i1_lock),.i_0_data(i1_data),
        .from_ram(ram_to_icache[575:288]),
        .to_ram(icache_to_ram[721:361]),
        .current_imemreq(current_imemreq),
        .imemrqueue_addr(imemrqueue_addr),
        .read_instr_again(read_instr_again),
        .cr_cfg6_cache0_all(cr_cfg6_cache0_all_i),
        .way_select(way_select1),
        .second_read(second_read),
        .update_link_way(update_link_way1),
        .update_link(update_link),
        .cacheline_writtendata(cacheline_writtendata),
        .data_cen({data_cen[7:4],data_cen[1]}),
        .di_cache28_miss(di_cache28_miss),
        .cache0_validtime(cache0_validtime1));

//Set 2//
icache_read_write icache_wr2(.clock(clock),.reset(reset),.i_set(2'b10),
        .cr_icachestate_value(cr_icachestate2_value),
        .iram_index(iram_index),
        .icache_v_offset(icache_v_offset),
        .i_tag(i_tag),.i_offset(i_offset),
        .di_valid(di_valid),.di_index(di_index),.di_set(di_set),
        .di_tag(di_tag),.di_taglow(di_taglow),
        .di_cache0(di_cache0),.di_cache8(di_cache8),
        .di_cache16(di_cache16), .di_cache28(di_cache28),
	    .di_vaddr_dicache_valid(di_vaddr_dicache_valid),
        .memres(memres_to_write),.cancel_write(cancel_write),
        .i_tag_match(i2_tag_match),.i_tag_valid(i2_tag_valid),
        .i_0_tag(i2_tag),.i_0_lock(i2_lock),.i_0_data(i2_data),
        .from_ram(ram_to_icache[863:576]),
        .to_ram(icache_to_ram[1082:722]),
        .current_imemreq(current_imemreq),
        .imemrqueue_addr(imemrqueue_addr),
        .read_instr_again(read_instr_again),
        .cr_cfg6_cache0_all(cr_cfg6_cache0_all_i),
        .way_select(way_select2),
        .second_read(second_read),
        .update_link_way(update_link_way2),
        .update_link(update_link),
        .data_cen({data_cen[7:4],data_cen[2]}),
        .di_cache28_miss(di_cache28_miss),
        .cacheline_writtendata(cacheline_writtendata),
        .cache0_validtime(cache0_validtime2));

//Set 3//
icache_read_write icache_wr3(.clock(clock),.reset(reset),.i_set(2'b11),
        .cr_icachestate_value(cr_icachestate3_value),
        .iram_index(iram_index),
        .icache_v_offset(icache_v_offset),
        .i_tag(i_tag),.i_offset(i_offset),
        .di_valid(di_valid),.di_index(di_index),.di_set(di_set),
        .di_tag(di_tag),.di_taglow(di_taglow),
        .di_cache0(di_cache0),.di_cache8(di_cache8),
        .di_cache16(di_cache16), .di_cache28(di_cache28),
    	.di_vaddr_dicache_valid(di_vaddr_dicache_valid),
        .memres(memres_to_write),.cancel_write(cancel_write),
        .i_tag_match(i3_tag_match),.i_tag_valid(i3_tag_valid),
        .i_0_tag(i3_tag),.i_0_lock(i3_lock),.i_0_data(i3_data),
        .from_ram(ram_to_icache[1151:864]),
        .to_ram(icache_to_ram[1443:1083]),
        .current_imemreq(current_imemreq),
        .imemrqueue_addr(imemrqueue_addr),
        .read_instr_again(read_instr_again),
        .cr_cfg6_cache0_all(cr_cfg6_cache0_all_i),
        .way_select(way_select3),
        .second_read(second_read),
        .update_link_way(update_link_way3),
        .update_link(update_link),
        .data_cen({data_cen[7:4],data_cen[3]}),
        .di_cache28_miss(di_cache28_miss),
        .cacheline_writtendata(cacheline_writtendata),
        .cache0_validtime(cache0_validtime3));

endmodule

//////////////////////////////////////////////////////////////////////
//                                                                  //
//               Instruction Cache Control Module                   //
//                                                                  //
////////////////////////////////////////////////////////////////////// 


module icache_control(clock,reset,commitbus_ex,i1_laddr,
 icachepaddr,iresbus,memres,imemraddr,dicache,
 inst_cache_block, inst_uncache_block,
 cr_icachestate0_value,cr_icachestate1_value,
 cr_icachestate2_value,cr_icachestate3_value,
 iram_index,i_tag,i_offset,
 di_valid,di_index,di_set,cancel_write,
 di_tag,di_taglow,di_cache0,di_cache8,di_cache16,di_cache28,
 di_vaddr_dicache_valid, 
 i0_tag_match,i0_tag_valid,i0_tag,i0_lock,i0_data,
 i1_tag_match,i1_tag_valid,i1_tag,i1_lock,i1_data,
 i2_tag_match,i2_tag_valid,i2_tag,i2_lock,i2_data,
 i3_tag_match,i3_tag_valid,i3_tag,i3_lock,i3_data,
 cr_cfg7_icache_i,
 PENDING_IBE,IEXI,
 DERET_IFLAG,
 current_imemreq,
 imemrqueue_addr,
 cacheline_writtendata,
 read_instr_again,
 cache0_validtime0,
 cache0_validtime1,
 cache0_validtime2,
 cache0_validtime3,
 IBE_FROM_CACHE,
 icache_v_offset,
 way_select0,way_select1, way_select2,way_select3,
 second_read, update_link,
 update_link_way0, update_link_way1,update_link_way2, update_link_way3,
 //for critical path
 data_cen,
 di_cache28_miss,
 cache28_refill_ok,
 icache_stalli, icache_refill_ok,
//performance counter
 icache_hit_perf,
 icache_access,
 icache_way_hit,
 stalled_cycles_icachemiss_o,
 icache_update
 );

input clock,reset,commitbus_ex;
input [`Licachepaddr-1:0] icachepaddr;
input [64:0] i1_laddr;
input [53:0] memres;

input [`Ltlb_to_icache-1:0] dicache;
input inst_cache_block;
input inst_uncache_block;
input PENDING_IBE,IEXI;
input DERET_IFLAG;
output [`Liresbus_2issue-1:0] iresbus;
output [`Lmemraddr-1:0] imemraddr;
output[2:0] icache_v_offset;

input [31:0] i0_tag ,i1_tag, i2_tag, i3_tag; 
input        i0_lock,i1_lock,i2_lock,i3_lock;
input [63:0] i0_data,i1_data,i2_data,i3_data; 

input        i0_tag_match,i1_tag_match,i2_tag_match,i3_tag_match;
input        i0_tag_valid,i1_tag_valid,i2_tag_valid,i3_tag_valid;


input cache0_validtime0,cache0_validtime1,cache0_validtime2,cache0_validtime3;
input   [  1:0] cr_cfg7_icache_i;

output cr_icachestate0_value,cr_icachestate1_value;
output cr_icachestate2_value,cr_icachestate3_value;


output di_valid;
output [19:0] i_tag, di_tag; //added for cache16
output [4:0] i_offset;
output [6:0] di_index;
output [21:0] iram_index;
output [1:0] di_set;
output cancel_write;
output [31:0] di_taglow; //added for cache8
output di_cache0, di_cache8, di_cache16, di_cache28;
output di_vaddr_dicache_valid;

output [31:0] imemrqueue_addr;
output [255:0] cacheline_writtendata;
output read_instr_again;
output current_imemreq;
output IBE_FROM_CACHE;

//for data read 
output      way_select0, way_select1;
output      way_select2, way_select3;
/*for way predict miss and then read the hit data way */
output       second_read;
output[39:0] update_link;
output       update_link_way0, update_link_way1;
output       update_link_way2, update_link_way3;

//for critical path
output[7:0] data_cen;

//for cache28 miss
output      di_cache28_miss;
output      cache28_refill_ok;

output  icache_stalli, icache_refill_ok;

//for performance counter
output icache_hit_perf;
output icache_access;

output stalled_cycles_icachemiss_o;
output icache_way_hit;
output icache_update;

wire icachepaddr_valid_t; // it has no insout_valid, for critical path
wire icachepaddr_valid;
wire icachepaddr_cached;
wire [31:0] icachepaddr_addr;
wire icachepaddr_adei;
wire icachepaddr_tlbii;
wire icachepaddr_tlbir;
wire icachepaddr_dib_0;
wire icachepaddr_dib_1;

wire       pred_no_update;   //when prediction came from RAS or inline sequential, needn't update link  
wire[1:0]  pred_way_hit;     //the way link info came from  
wire[9:0]  pred_link_info;   //all the link info (five)
wire[19:0] pred_link_tag;    //the tag where link info came form 
wire[1:0]  pred_link_offset; //for select which no_sequential link for update 
wire[6:0]  pred_link_index;  //the index where link info came from
wire       pred_seq_link;    //wether the link info is sequential link
wire[1:0]  pred_v_lock;  

wire       irstalli;

wire       pc_en_nowayhit; //pc_in_en = pc_en_nowayhit | iresbus_way_hit
wire[1:0]  way_pred ;

wire        pc_in_en; 
wire [11:0] i_laddr;

wire      di_vaddr_cache28;
wire      di_vaddr_cache16;
wire      di_vaddr_valid;
wire[6:0] di_vaddr_index;


wire [63:0] iresbus_value; //for two instructions

wire iresbus_valid_0;
wire iresbus_cacherdy_0;
wire [31:0] iresbus_value_0;
wire iresbus_adei_0;
wire iresbus_tlbii_0;
wire iresbus_tlbir_0;
wire iresbus_ibe_0;
wire iresbus_dib_0;

wire iresbus_valid_1;
wire iresbus_cacherdy_1;
wire [31:0] iresbus_value_1;
wire iresbus_adei_1;
wire iresbus_tlbii_1;
wire iresbus_tlbir_1;
wire iresbus_ibe_1;
wire iresbus_dib_1;

//send them to fetch, for way prediction miss recover
wire[11:0] link_info_t;  //11: valid, 10: lock
wire[9:0] iresbus_link_info; 
wire[1:0] iresbus_way_hit;
wire[19:0] iresbus_paddr_tag;
wire[1:0] iresbus_v_lock;
wire      iresbus_no_update;
wire      iresbus_one_word;//for no dw align

reg cacheline_keyto8;
reg read_instr_again;

wire imemraddr_valid;
wire [3:0] imemraddr_width;
wire [31:0] imemraddr_addr;
wire [2:0] imemraddr_id;

assign icachepaddr_valid_t =icachepaddr[39];
assign icachepaddr_dib_1=icachepaddr[38];
assign icachepaddr_dib_0=icachepaddr[37];
assign icachepaddr_valid=icachepaddr[36];
assign icachepaddr_cached=icachepaddr[35];
assign icachepaddr_addr=icachepaddr[34:3];
assign icachepaddr_adei=icachepaddr[2];
assign icachepaddr_tlbii=icachepaddr[1];
assign icachepaddr_tlbir=icachepaddr[0];

wire [21:0] iram_index;
wire [31:0] i_addr;

assign pc_en_nowayhit   = i1_laddr[64];  
assign irstalli         = i1_laddr[63];  
assign pred_v_lock      = i1_laddr[62:61];  
assign pred_no_update   = i1_laddr[60:60];  
assign pred_way_hit     = i1_laddr[59:58];  
assign pred_link_info   = i1_laddr[57:48];  //all the link information which current way prediction come from
assign pred_link_tag    = i1_laddr[47:28];   //the physical tag which which current way prediction come from 
assign pred_link_index  = i1_laddr[27:21]; //select current way prediction come from which noseq link info 
assign pred_link_offset = i1_laddr[20:19];//the index which current way_prediction  come from
assign pred_seq_link    = i1_laddr[15];      //wether current way prediction is sequential link/

assign way_pred         = i1_laddr[14:13]; // current way prediction

assign pc_in_en         = i1_laddr[12];
assign i_laddr          = i1_laddr[11:0];



assign icache_v_offset  = pc_en_nowayhit ? i_laddr[4:2] :  i_addr[4:2]; //for bank read
                          //: read_instr_again

wire[31:0] di_paddr     = dicache[34:3];

assign di_vaddr_cache28 = dicache[79]; //vaddr cache28 
assign di_cache28       = dicache[78]; //paddr cache28
assign di_vaddr_cache16 = dicache[77];
assign di_vaddr_valid   = dicache[76];
assign di_vaddr_index   = dicache[75:69];
assign di_taglow        = dicache[68:37]; //added for cache8
assign di_valid         = dicache[36];
assign di_tag           = dicache[34:15]; //added for cache16, hit invlaid
assign di_index         = dicache[14:8];
assign di_set           = dicache[81:80];
assign di_cache16       = dicache[2];
assign di_cache8        = dicache[1];
assign di_cache0        = dicache[0];


wire      memres_is_dw       = memres[0];
wire      memres_valid       = memres[1];
wire[2:0] memres_counter     = memres[4:2];
wire      memres_is_icache   = memres[5]   ;
wire      memres_dr_dw       = memres[6]   ;
wire      memres_rd_rdy      = memres[7]   ;
wire      memres_uncache_rdy = memres[8]   ;
wire      memres_block_rdy   = memres[9]   ;
wire[7 :0]memres_index       = memres[17:10];
wire[3 :0]memres_width       = memres[21:18];
wire[31:0]memres_data        = memres[53:22] ;


assign iresbus[113]     = iresbus_one_word; // when data come from returning and uncached,  we needn't update link
assign iresbus[112]     = iresbus_no_update; // when data come from returning and uncached,  we needn't update link
assign iresbus[111:110] = iresbus_v_lock; // tag's valid bits of current fetch 
assign iresbus[109:90]  = iresbus_paddr_tag; //paddr of current fetch 
assign iresbus[89:88]   = iresbus_way_hit; //hit way of current fetch 
assign iresbus[87:78]   = iresbus_link_info; //link information for way predictiom miss recover 

assign iresbus[77]=iresbus_dib_1;
assign iresbus[76]=iresbus_valid_1;
assign iresbus[75]=iresbus_cacherdy_1;
assign iresbus[74:43]=iresbus_value_1;
assign iresbus[42]=iresbus_ibe_1;
assign iresbus[41]=iresbus_adei_1;
assign iresbus[40]=iresbus_tlbii_1;
assign iresbus[39]=iresbus_tlbir_1;

assign iresbus[38]=iresbus_dib_0;
assign iresbus[37]=iresbus_valid_0;
assign iresbus[36]=iresbus_cacherdy_0;
assign iresbus[35:4]=iresbus_value_0;
assign iresbus[3]=iresbus_ibe_0;
assign iresbus[2]=iresbus_adei_0;
assign iresbus[1]=iresbus_tlbii_0;
assign iresbus[0]=iresbus_tlbir_0;

assign imemraddr[39:37]=imemraddr_id;
assign imemraddr[36]=imemraddr_valid;
assign imemraddr[35:32]=imemraddr_width;
assign imemraddr[31:0]=imemraddr_addr;

wire        i_valid,i_cached,i_valid_t;
wire [19:0] i_tag; //physical tag
wire [4 :0] i_offset;

wire [31:0] i0_tag, i1_tag, i2_tag, i3_tag;
wire        i0_lock,i1_lock,i2_lock,i3_lock;
wire [63:0] i1_data,i0_data,i3_data,i2_data; 

wire        i0_tag_match,i1_tag_match,i2_tag_match,i3_tag_match;
wire        i0_tag_valid,i1_tag_valid,i2_tag_valid,i3_tag_valid;

reg imemrqueue_valid;
reg [31:0] imemrqueue_addr;
reg imemrqueue_dr;
reg[1:0] imemrqueue_way; //for refill way
reg      imemrqueue_cache28_miss;

reg miss_update_link; //for cache miss, need update link

reg fake_icache_hit;

wire i0_match = i0_tag_match& (i0_tag_valid& ~fake_icache_hit);
wire i1_match = i1_tag_match& (i1_tag_valid& ~fake_icache_hit);
wire i2_match = i2_tag_match& (i2_tag_valid& ~fake_icache_hit);
wire i3_match = i3_tag_match& (i3_tag_valid& ~fake_icache_hit);

wire icache_hit = i0_match | i1_match | i2_match | i3_match;

wire imemres_flag;

reg cr_icachestate0,cr_icachestate1,cr_icachestate2,cr_icachestate3,cr_icachestate5;
wire [1:0] select_set;

wire cr_icachestate0_value=cr_icachestate0;
wire cr_icachestate1_value=cr_icachestate1;
wire cr_icachestate2_value=cr_icachestate2;
wire cr_icachestate3_value=cr_icachestate3;
wire cr_icachestate5_value=cr_icachestate5;


wire ireq_block = i_cached&inst_cache_block | ~i_cached&inst_uncache_block;

assign icache_refill_ok = imemres_flag;
assign icache_stalli  = imemrqueue_valid &~imemres_flag | imemraddr_valid&~ireq_block;

/*****************************for way predictiion****************************/
reg      second_hit;  //when cache hit but way prediction miss, direct the second hit
reg[1:0] second_hit_way; //direct second hit way, because when second read, do link update. after link update, may be has two way match. we need know the right hit way 

reg[1:0] way_pred_reg; //current way prediction for judge whether way prediction miss and for second read 

reg[6:0] link_index_reg; //for link update 
reg[1:0] link_offset_reg;
reg[19:0] link_tag_reg;
reg      link_seq_reg;
reg[9:0] link_info_reg;
reg[1:0] link_way_reg;
reg[1:0] link_v_lock_reg;

reg        update_link_en; 
wire[31:0] update_link_value;

wire icache_hit_way_0, icache_hit_way_1,icache_hit_way_2,icache_hit_way_3;

always @ (posedge clock) 
if(pc_in_en)
begin
   way_pred_reg <= way_pred;

   link_index_reg<= pred_link_index;
   link_offset_reg<= pred_link_offset;
   link_tag_reg<= pred_link_tag;
   link_info_reg<= pred_link_info;
   link_seq_reg  <= pred_seq_link;
   link_way_reg  <= pred_way_hit;
   link_v_lock_reg  <= pred_v_lock;
   
//if link come from ex, jr31, inline seq,  needn't update link info.
//and if new instr fetch and refill cache conflict, needn't update link. Because,  the new cache line may be refill into the cache line which link came from 
   update_link_en <= ~(commitbus_ex | reset | pred_no_update)  ;
end

wire way_pred_hit0 = i0_match & (way_pred_reg==2'b00) ;
wire way_pred_hit1 = i1_match & (way_pred_reg==2'b01) ;
wire way_pred_hit2 = i2_match & (way_pred_reg==2'b10) ;
wire way_pred_hit3 = i3_match & (way_pred_reg==2'b11) ;

wire way_pred_hit =  way_pred_hit0 | way_pred_hit1 | way_pred_hit2 | way_pred_hit3;

wire way_pred_miss = ~(i0_match &(way_pred_reg==2'b00)|
                       i1_match &(way_pred_reg==2'b01)|
                       i2_match &(way_pred_reg==2'b10)|
                       i3_match &(way_pred_reg==2'b11));

assign icache_hit_way_0 = i0_match& (way_pred_reg!=2'b00);
assign icache_hit_way_1 = i1_match& (way_pred_reg!=2'b01);
assign icache_hit_way_2 = i2_match& (way_pred_reg!=2'b10);
assign icache_hit_way_3 = i3_match& (way_pred_reg!=2'b11);


wire way_select0 = pc_en_nowayhit     ? (way_pred     == 2'b00)    :  
                   read_instr_again   ? (way_pred_reg == 2'b00)    : icache_hit_way_0   ;
wire way_select1 = pc_en_nowayhit     ? (way_pred     == 2'b01)    :  
                   read_instr_again   ? (way_pred_reg == 2'b01)    : icache_hit_way_1   ;
wire way_select2 = pc_en_nowayhit     ? (way_pred     == 2'b10)    :  
                   read_instr_again   ? (way_pred_reg == 2'b10)    : icache_hit_way_2   ;
wire way_select3 = pc_en_nowayhit     ? (way_pred     == 2'b11)    :  
                   read_instr_again   ? (way_pred_reg == 2'b11)    : icache_hit_way_3   ;

wire way_select0_t = (way_pred == 2'b00); 
wire way_select1_t = (way_pred == 2'b01);
wire way_select2_t = (way_pred == 2'b10);
wire way_select3_t = (way_pred == 2'b11);

wire imemres_wayhit;

assign data_cen[3:0] = {way_select3_t,way_select2_t,way_select1_t,way_select0_t};
assign data_cen[4]   = imemres_wayhit;
assign data_cen[7:5] = i_laddr[4:2];

wire iresbus_returning_cacheline_hit;

assign second_read = (~reset&~commitbus_ex&~irstalli & i_cached & i_valid & ~read_instr_again & ~second_hit 
                      &~(imemres_flag & ~imemrqueue_dr)) & way_pred_miss & icache_hit; 
                      //when refill and second read conflict, needn't second read. just read instruction again

assign update_link_value[31:30] = link_v_lock_reg;
assign update_link_value[29:28] = ~link_seq_reg ? link_info_reg[9:8] :
                                   icache_hit_way_0  ? 2'b00              : 
                                   icache_hit_way_1  ? 2'b01              :
                                   icache_hit_way_2  ? 2'b10              : 
                                   icache_hit_way_3  ? 2'b11              : imemrqueue_way;  //for cache miss
assign update_link_value[27:26] = (~(link_offset_reg == 2'b11)& ~link_seq_reg | link_seq_reg) ? link_info_reg[7:6] :
                                   icache_hit_way_0 ? 2'b00                      :   
                                   icache_hit_way_1 ? 2'b01                      :   
                                   icache_hit_way_2 ? 2'b10                      :   
                                   icache_hit_way_3 ? 2'b11                      : imemrqueue_way;
assign update_link_value[25:24] = (~(link_offset_reg == 2'b10)& ~link_seq_reg | link_seq_reg)  ? link_info_reg[5:4] :
                                   icache_hit_way_0 ? 2'b00                      :   
                                   icache_hit_way_1 ? 2'b01                      :   
                                   icache_hit_way_2 ? 2'b10                      :   
                                   icache_hit_way_3 ? 2'b11                      : imemrqueue_way;
assign update_link_value[23:22] = (~(link_offset_reg == 2'b01)& ~link_seq_reg | link_seq_reg) ? link_info_reg[3:2] :
                                   icache_hit_way_0 ? 2'b00                      :   
                                   icache_hit_way_1 ? 2'b01                      :   
                                   icache_hit_way_2 ? 2'b10                      :   
                                   icache_hit_way_3 ? 2'b11                      : imemrqueue_way;
assign update_link_value[21:20] = (~(link_offset_reg == 2'b00)& ~link_seq_reg | link_seq_reg) ? link_info_reg[1:0] :
                                   icache_hit_way_0 ? 2'b00                      :   
                                   icache_hit_way_1 ? 2'b01                      :   
                                   icache_hit_way_2 ? 2'b10                      :    
                                   icache_hit_way_3 ? 2'b11                      : imemrqueue_way;
assign update_link_value[19:0]  =  link_tag_reg;

wire update_link_way0, update_link_way1, update_link_way2, update_link_way3;
wire [39:0] update_link;

reg[6:0] refill_index_reg_0;
reg[1:0] refill_way_reg_0;

reg[6:0] refill_index_reg_1;
reg[1:0] refill_way_reg_1;

assign update_link_way0 = (link_way_reg == 2'b00);
assign update_link_way1 = (link_way_reg == 2'b01);
assign update_link_way2 = (link_way_reg == 2'b10);
assign update_link_way3 = (link_way_reg == 2'b11);

assign update_link[39:33] = link_index_reg;
assign update_link[32]   = (update_link_en &
                           //~((imemrqueue_addr[11:5] == link_index_reg)&(imemrqueue_way == link_way_reg))&
                           ~((refill_index_reg_0 == link_index_reg)&(link_way_reg == refill_way_reg_0))  &
                           ~((refill_index_reg_1 == link_index_reg)&(link_way_reg == refill_way_reg_1)) ) &
                           //when dealing refill cache line has wrong link, needn't update
                           ( second_read | miss_update_link&~commitbus_ex) ;
                           //when way prediction miss and cache miss, update link
                      
assign update_link[31:0]  = update_link_value;


always@(posedge clock)
begin
if(reset)
begin
    refill_index_reg_0 <= 7'b0;
    refill_way_reg_0 <= 2'b0;
    refill_index_reg_1 <= 7'b0;
    refill_way_reg_1 <= 2'b0;
end
else if (imemres_flag)
begin
    refill_index_reg_0 <= imemrqueue_addr[11:5];
    refill_way_reg_0 <= imemrqueue_way;
    refill_index_reg_1 <= refill_index_reg_0;
    refill_way_reg_1 <= refill_way_reg_0;
end
else if (update_link[32])
begin
    refill_index_reg_0 <= 7'b0;
    refill_way_reg_0 <= 2'b0;
    refill_index_reg_1 <= 7'b0;
    refill_way_reg_1 <= 2'b0;
end
  
end

always @(posedge clock)
if (pc_in_en | read_instr_again)
begin
   second_hit<=1'b0;
   second_hit_way <= 2'b00;
end
else if (second_read)
begin
   second_hit     <= 1'b1; 
   second_hit_way <= way_select0 ? 2'b00:
                     way_select1 ? 2'b01:
                     way_select2 ? 2'b10: 2'b11;
end

reg icache_miss;
always @ (posedge clock)
if (pc_in_en | read_instr_again)
  fake_icache_hit <=0;
else if (update_link[32])
  fake_icache_hit <= 1;

//for performance counter
reg pc_in_en_r;
always @(posedge clock)
  pc_in_en_r <= pc_in_en;

assign icache_hit_perf = icache_hit & pc_in_en_r & i_cached;
assign icache_access = pc_in_en_r &  i_cached; 
assign icache_way_hit  = way_pred_hit & pc_in_en_r  & i_cached;
assign icache_update   = update_link[32] ;

/*************************************************************************************/

wire i_mem_allow;

//icachepaddr_valid = icachepaddr_valid_t&instout_valid
assign i_valid= (icachepaddr_valid&(~icachepaddr_tlbii)&(~icachepaddr_tlbir)&(~icachepaddr_dib_0) )&
                (~icachepaddr_adei);

assign i_valid_t= (icachepaddr_valid_t&(~icachepaddr_tlbii)&(~icachepaddr_tlbir)&(~icachepaddr_dib_0) )&
                (~icachepaddr_adei);
//when dib and cache miss ,it still mem
assign i_addr=icachepaddr_addr;
assign i_tag=i_addr[31:12];
assign i_offset=i_addr[4:0];
assign i_cached=icachepaddr_cached;

//to cache_read_write for rw_index
assign iram_index[0]     = pc_en_nowayhit ;
assign iram_index[7:1]   = i_laddr[11:5];
assign iram_index[14:8]  = i_addr[11:5];
assign iram_index[21:15] = di_vaddr_index;

assign di_vaddr_dicache_valid = di_vaddr_valid&(di_vaddr_cache16 | di_vaddr_cache28);
//when cache 28 miss, then need refill this cache line


wire cache28_miss_t  =  di_valid&di_cache28&
                     ~((di_tag == i3_tag[19:0])&i3_tag_valid| (di_tag == i2_tag[19:0])&i2_tag_valid|
                       (di_tag == i1_tag[19:0])&i1_tag_valid| (di_tag == i0_tag[19:0])&i0_tag_valid);
wire cache28_hit  =  di_valid&di_cache28&
                     ((di_tag == i3_tag[19:0])&i3_tag_valid| (di_tag == i2_tag[19:0])&i2_tag_valid|
                       (di_tag == i1_tag[19:0])&i1_tag_valid| (di_tag == i0_tag[19:0])&i0_tag_valid);
wire icache_miss_t =   i_valid&i_cached& ~read_instr_again &~irstalli & ~icache_hit&~second_hit | cache28_miss_t; 
reg cache28_miss;

wire  i_dr=i_valid&(~i_cached);

assign i_mem_allow=(~cr_icachestate0_value)&(~cr_icachestate1_value)&(~cr_icachestate2_value)&(~cr_icachestate3_value)&
                   (~cr_icachestate5_value)&~read_instr_again&
                   (memres_rd_rdy&i_cached | memres_rd_rdy&memres_uncache_rdy&~i_cached);

//Send Read Memory Request//
assign imemraddr_valid=i_mem_allow & icache_miss ;
assign imemraddr_addr= cache28_miss ? di_paddr : i_addr;
assign imemraddr_width=cache28_miss ? 4'b0001: {3'b000, i_cached} | {4{~i_cached}};
assign imemraddr_id=(cache28_miss | i_cached) ? 3'b011 : 3'b111;

wire [1:0] cr_iset_value = select_set;
 
always @(posedge clock)
if (reset)
  begin
    imemrqueue_valid   <=1'b0;
    imemrqueue_addr    <=32'b0;
    imemrqueue_dr      <=1'b0;
    imemrqueue_way     <=2'b0;
    miss_update_link   <=1'b0;
    imemrqueue_cache28_miss <=1'b0;
  end
else
  begin
    if (imemraddr_valid&(~ireq_block))
       imemrqueue_valid<=1'b1;
    else
       if (imemres_flag|iresbus_ibe_0)
       begin
          imemrqueue_valid<=1'b0;
          imemrqueue_cache28_miss <= 1'b0;
       end                                                                                 
    if (imemraddr_valid&~ireq_block)
       begin
          imemrqueue_addr<=imemraddr_addr;
          imemrqueue_dr<=i_dr;
          imemrqueue_way<=cr_iset_value;
          imemrqueue_cache28_miss <=di_valid & di_cache28;
          miss_update_link <= i_cached&~(di_valid &di_cache28);
       end
    else 
       miss_update_link <=1'b0;
    end
 

always @(posedge clock)
if(pc_in_en | read_instr_again | imemres_flag)
  begin
  icache_miss <= 1'b0;
  cache28_miss <= 1'b0;
  end
else 
begin
  icache_miss <= (icache_miss_t |i_dr) ;
  cache28_miss <= cache28_miss_t;
end
assign di_cache28_miss = imemrqueue_cache28_miss;
assign cache28_refill_ok = imemres_flag & imemrqueue_cache28_miss |cache28_hit ;
assign stalled_cycles_icachemiss_o = icache_miss&i_cached;
//////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
// assemble the returning 8 words from buffer into a whole cacheline.
reg [255:0] cacheline_backdata;
reg [255:0] cacheline_writtendata;
wire cacheline_wr_enable=memres_is_icache;
wire imemrefill_valid=imemrqueue_valid&memres_is_icache;

wire cache_backdata_gating_clock;
assign cache_backdata_gating_clock = clock;

always@(posedge cache_backdata_gating_clock)
if (reset)
   cacheline_backdata[255:0]<=256'b0;
else
   if (cacheline_wr_enable)
      begin
         if(memres_index[0])
             cacheline_backdata[31:0]<=memres_data[31:0];
         if(memres_index[1])
             cacheline_backdata[63:32]<=memres_data[31:0];
         if(memres_index[2])
             cacheline_backdata[95:64]<=memres_data[31:0];
         if(memres_index[3])
             cacheline_backdata[127:96]<=memres_data[31:0];
         if(memres_index[4])
             cacheline_backdata[159:128]<=memres_data[31:0];
         if(memres_index[5])
             cacheline_backdata[191:160]<=memres_data[31:0];
         if(memres_index[6])
             cacheline_backdata[223:192]<=memres_data[31:0];
         if(memres_index[7])
             cacheline_backdata[255:224]<=memres_data[31:0];
     end

// The whole cacheline is written into icache ram in the cycle when the last
// word is returning.Therefore the last word should be dealed with considerately.
always @(memres_block_rdy or memres_index or cacheline_backdata or memres_data or cacheline_wr_enable)
begin
    if (memres_block_rdy&cacheline_wr_enable)
       begin
           if(memres_index[0])
              begin
                 cacheline_writtendata[255:32]=cacheline_backdata[255:32];
                 cacheline_writtendata[31:0]=memres_data[31:0];
              end
           else if(memres_index[1])
              begin
                 cacheline_writtendata[255:64]=cacheline_backdata[255:64];
                 cacheline_writtendata[63:32]=memres_data[31:0];
                 cacheline_writtendata[31:0]=cacheline_backdata[31:0];
              end
           else if(memres_index[2])
              begin
                 cacheline_writtendata[255:96]=cacheline_backdata[255:96];
                 cacheline_writtendata[95:64]=memres_data[31:0];
                 cacheline_writtendata[63:0]=cacheline_backdata[63:0];
              end
           else if(memres_index[3])
              begin
                 cacheline_writtendata[255:128]=cacheline_backdata[255:128];
                 cacheline_writtendata[127:96]=memres_data[31:0];
                 cacheline_writtendata[95:0]=cacheline_backdata[95:0];
              end
           else if(memres_index[4])
              begin
                 cacheline_writtendata[255:160]=cacheline_backdata[255:160];
                 cacheline_writtendata[159:128]=memres_data[31:0];
                 cacheline_writtendata[127:0]=cacheline_backdata[127:0];
              end
           else if(memres_index[5])
              begin
                 cacheline_writtendata[255:192]=cacheline_backdata[255:192];
                 cacheline_writtendata[191:160]=memres_data[31:0];
                 cacheline_writtendata[159:0]=cacheline_backdata[159:0];
              end
           else if(memres_index[6])
              begin
                 cacheline_writtendata[255:224]=cacheline_backdata[255:224];
                 cacheline_writtendata[223:192]=memres_data[31:0];
                 cacheline_writtendata[191:0]=cacheline_backdata[191:0];
              end
           else if(memres_index[7])
              begin
                 cacheline_writtendata[255:224]=memres_data[31:0];
                 cacheline_writtendata[223:0]=cacheline_backdata[223:0];
              end
           else
                 cacheline_writtendata=256'b0;
         end
   else
       cacheline_writtendata=256'b0;
end
/////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////`
// indicate which word in the cacheline is valid
reg [7:0] word_valid;
always@(posedge clock)
begin
    if (reset)
       word_valid[7:0]<=8'b0;
    else
       if (imemres_flag|iresbus_ibe_0)
          word_valid[7:0]<=8'b0;
       else
          begin // the word can't be valid until memres_index[i] is high.
             if (memres_index[0]&imemrefill_valid)
                word_valid[0]<=1'b1;
             if (memres_index[1]&imemrefill_valid)
                word_valid[1]<=1'b1;
             if (memres_index[2]&imemrefill_valid)
                word_valid[2]<=1'b1;
             if (memres_index[3]&imemrefill_valid)
                word_valid[3]<=1'b1;
             if (memres_index[4]&imemrefill_valid)
                word_valid[4]<=1'b1;
             if (memres_index[5]&imemrefill_valid)
                word_valid[5]<=1'b1;
             if (memres_index[6]&imemrefill_valid)
                word_valid[6]<=1'b1;
             if (memres_index[7]&imemrefill_valid)
                word_valid[7]<=1'b1;
          end
end
/////////////////////////////////////////////////////




// The following logic deals with the case that the required data
// is in the returning cacheline.
wire  in_returning_cacheline_t= imemrqueue_valid&memres_is_icache;
                              
wire  in_cacheline =  (i_addr[31:5]==imemrqueue_addr[31:5]);
                                                                                     

wire  returning_cacheline_dw0_hit = (i_addr[4:2]==3'b000) &(word_valid[0])&(word_valid[1]|memres_index[1]);
wire  returning_cacheline_dw1_hit = (i_addr[4:2]==3'b001) &(word_valid[1])&(word_valid[2]|memres_index[2]);
wire  returning_cacheline_dw2_hit = (i_addr[4:2]==3'b010) &(word_valid[2])&(word_valid[3]|memres_index[3]);
wire  returning_cacheline_dw3_hit = (i_addr[4:2]==3'b011) &(word_valid[3])&(word_valid[4]|memres_index[4]);
wire  returning_cacheline_dw4_hit = (i_addr[4:2]==3'b100) &(word_valid[4])&(word_valid[5]|memres_index[5]);
wire  returning_cacheline_dw5_hit = (i_addr[4:2]==3'b101) &(word_valid[5])&(word_valid[6]|memres_index[6]);
wire  returning_cacheline_dw6_hit = (i_addr[4:2]==3'b110) &(word_valid[6])&(word_valid[7]|memres_index[7]);
wire  returning_cacheline_dw7_hit = (i_addr[4:2]==3'b111) &(word_valid[7]|memres_index[7]);

wire [31:0] returning_cacheline_hit_data0=
                (word_valid[0])?cacheline_backdata[31:0]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data1=
                (word_valid[1])?cacheline_backdata[63:32]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data2=
                (word_valid[2])?cacheline_backdata[95:64]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data3=
                (word_valid[3])?cacheline_backdata[127:96]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data4=
                (word_valid[4])?cacheline_backdata[159:128]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data5=
                (word_valid[5])?cacheline_backdata[191:160]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data6=
                (word_valid[6])?cacheline_backdata[223:192]:memres_data[31:0];
wire [31:0] returning_cacheline_hit_data7=
                (word_valid[7])?cacheline_backdata[255:224]:memres_data[31:0];


wire [63:0] returning_cacheline_hit_dw0;
assign returning_cacheline_hit_dw0[31:0] = cacheline_backdata[31:0];
assign returning_cacheline_hit_dw0[63:32] = returning_cacheline_hit_data1; 

wire [63:0] returning_cacheline_hit_dw1;
assign returning_cacheline_hit_dw1[31:0] = cacheline_backdata[63:32];
assign returning_cacheline_hit_dw1[63:32] = returning_cacheline_hit_data2; 

wire [63:0] returning_cacheline_hit_dw2;
assign returning_cacheline_hit_dw2[31:0] = cacheline_backdata[95:64];
assign returning_cacheline_hit_dw2[63:32] = returning_cacheline_hit_data3; 

wire [63:0] returning_cacheline_hit_dw3;
assign returning_cacheline_hit_dw3[31:0] = cacheline_backdata[127:96];
assign returning_cacheline_hit_dw3[63:32] = returning_cacheline_hit_data4; 

wire [63:0] returning_cacheline_hit_dw4;
assign returning_cacheline_hit_dw4[31:0] = cacheline_backdata[159:128];
assign returning_cacheline_hit_dw4[63:32] = returning_cacheline_hit_data5; 

wire [63:0] returning_cacheline_hit_dw5;
assign returning_cacheline_hit_dw5[31:0] = cacheline_backdata[191:160];
assign returning_cacheline_hit_dw5[63:32] = returning_cacheline_hit_data6; 

wire [63:0] returning_cacheline_hit_dw6;
assign returning_cacheline_hit_dw6[31:0] = cacheline_backdata[223:192];
assign returning_cacheline_hit_dw6[63:32] = returning_cacheline_hit_data7; 

wire [63:0] returning_cacheline_hit_dw7;
assign returning_cacheline_hit_dw7[31:0] = returning_cacheline_hit_data7;
assign returning_cacheline_hit_dw7[63:32] = returning_cacheline_hit_data7; 

wire  returning_cacheline_hit=(returning_cacheline_dw0_hit |
                              returning_cacheline_dw1_hit |
                              returning_cacheline_dw2_hit |
                              returning_cacheline_dw3_hit |
                              returning_cacheline_dw4_hit |
                              returning_cacheline_dw5_hit |
                              returning_cacheline_dw6_hit |
                              returning_cacheline_dw7_hit) & in_returning_cacheline_t;

wire [63:0] returning_cacheline_hit_value=
         ({64{returning_cacheline_dw0_hit}}&returning_cacheline_hit_dw0[63:0])|
         ({64{returning_cacheline_dw1_hit}}&returning_cacheline_hit_dw1[63:0])|
         ({64{returning_cacheline_dw2_hit}}&returning_cacheline_hit_dw2[63:0])|
         ({64{returning_cacheline_dw3_hit}}&returning_cacheline_hit_dw3[63:0])|
         ({64{returning_cacheline_dw4_hit}}&returning_cacheline_hit_dw4[63:0])|
         ({64{returning_cacheline_dw5_hit}}&returning_cacheline_hit_dw5[63:0])|
         ({64{returning_cacheline_dw6_hit}}&returning_cacheline_hit_dw6[63:0])|
         ({64{returning_cacheline_dw7_hit}}&returning_cacheline_hit_dw7[63:0]);
//////////////////////////////////////////////////////////////////////////////





/////////////////////////////////////////////////////////////////////////////
// indicate the load-value is from the returning cacheline. 
assign iresbus_returning_cacheline_hit= (i_valid_t&i_cached&imemrqueue_valid &(~imemrqueue_dr) & ~irstalli &
                (~(cache0_validtime0|cache0_validtime1|cache0_validtime2|cache0_validtime3)) & returning_cacheline_hit) & 
                in_cacheline;
/////////////////////////////////////////////////////////////////////////////
/*                                                                                      
wire memres_keyword  = (memres_counter == imemrqueue_addr[4:2])&memres_valid&memres_is_icache;
                                                                                      
                                                                                      
///////////////////////////////////////////////////
wire icache_ack = imemrqueue_valid & memres_is_icache&
                (cr_icachestate0_value|cr_icachestate1_value|cr_icachestate2_value|cr_icachestate3_value) ;
always @( posedge clock )
if ( reset )
   cacheline_keyto8<=1'b0;
else
   if ((imemres_flag&(cr_icachestate0_value|cr_icachestate1_value|cr_icachestate2_value|cr_icachestate3_value))
       ||iresbus_ibe_0)
            cacheline_keyto8<=1'b0;
   else
       if (icache_ack & memres_keyword)
           cacheline_keyto8<=1'b1;
///////////////////////////////////////////////////


*/
///////////////////////////////////////////////////
// necessary only for pipelined amba interface.
//assign current_imemreq=cacheline_keyto8;
assign current_imemreq=1'b1;
///////////////////////////////////////////////////





////////////////////////////////////////////////////
wire fetch_two_inst  =(imemres_flag&cr_icachestate5_value&memres_dr_dw) ?
                       ~(imemrqueue_addr[4:2] == 3'b111) :  ~(i_offset[4:2] == 3'b111); 
wire imemres_second_hit = (i_valid_t& i_cached&(~read_instr_again) & ~irstalli) & second_hit ;

wire imemres_dr_dw = i_valid_t&imemres_flag&cr_icachestate5_value&memres_dr_dw & ~irstalli;

wire imemres_cachehit=(i_valid_t& i_cached&(~read_instr_again) & ~irstalli) & (way_pred_hit | second_hit );
//Sedn to Instruction Result Bus//
assign iresbus_valid_0=(imemres_dr_dw |iresbus_returning_cacheline_hit)|imemres_cachehit;
assign iresbus_valid_1 = iresbus_valid_0 & fetch_two_inst &  ~DERET_IFLAG;

assign iresbus_cacherdy_0=imemres_dr_dw |iresbus_returning_cacheline_hit | imemres_second_hit ;
assign iresbus_cacherdy_1 = iresbus_cacherdy_0 & fetch_two_inst  & ~DERET_IFLAG; 

assign imemres_wayhit = (i_valid& i_cached&(~read_instr_again) & ~irstalli) &way_pred_hit;
////////////////////////////////////////////////////////
wire [31:0] dw_data0, dw_data1;
assign dw_data0 = memres_data;
assign dw_data1 = memres_data;
wire [63:0] iresbus_value1;

reg [31:0] dw0_reg;
always @( posedge clock )
if ( reset )
  dw0_reg <=32'b0;
else
  if (memres_valid & memres_is_dw)
     dw0_reg <=dw_data0;

assign iresbus_value1[31:0] =  dw0_reg;
assign iresbus_value1[63:32]= dw_data1;


///////////////////////////////////////////////////////
//Whenever the whole refill cacheline is back,the cache ram will be
//read again in the next cycle.And this strategy solves serveral
//complicated problems when nonblocking.
//reg read_instr_again_t;
always @(posedge clock)
if (reset)
   read_instr_again<=1'b0;
else 
   read_instr_again<=imemres_flag&(~imemrqueue_dr)&~imemrqueue_cache28_miss&
                     ~di_valid& ~di_vaddr_dicache_valid &
                     ~(cache0_validtime0 | cache0_validtime1 | cache0_validtime2 | cache0_validtime3); 
                     //when has dicache,no need read instr again

///////////////////////////////////////////////////////



////////////////////////////////////////////////////////
assign cancel_write=1'b0;
////////////////////////////////////////////////////////



//replace method
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

assign lock_for_replace = {i3_lock,i2_lock,i1_lock,i0_lock};

always @(posedge clock)
begin
    repcnt_r<= reset ? 5'b11111 : {repcnt_r[3]^repcnt_r[0],repcnt_r[4:1]};
end

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

wire[1:0] select_set_4way = (has_3_set) ? set_1_in_3 :
                    (has_2_set) ? set_1_in_2 :
                    (has_1_set) ? set_1_in_1 : random_num_radix4;
wire [1:0] select_set_2way;
assign select_set_2way[1] = 1'b0;
assign select_set_2way[0] = (lock_for_replace[1:0]==2'b01) ? 1'b1 :
                            (lock_for_replace[1:0]==2'b10) ? 1'b0 : repcnt_r[0];
assign select_set = (cr_cfg7_icache_i==2'b01) ? 2'b00 :
                    (cr_cfg7_icache_i==2'b10) ? select_set_2way : select_set_4way;

// if commitbus_ex happens 
// this cacheline refill would not be canceled and the next icache
// request won't be admitted into icache until the keyword instruction is back.

wire cr_state_en =imemraddr_valid &(i_cached | cache28_miss)&~inst_cache_block;
always @(posedge clock)
    //two ICacheState Register//
    if(imemres_flag|reset|iresbus_ibe_0)
                                                                          
    begin
      cr_icachestate0<=1'b0;
      cr_icachestate1<=1'b0;
      cr_icachestate2<=1'b0;
      cr_icachestate3<=1'b0;
      cr_icachestate5<=1'b0;
    end
    else if(cr_state_en&(cr_iset_value == 2'b00))
            cr_icachestate0<=1'b1;
    else if(cr_state_en&(cr_iset_value == 2'b01))
            cr_icachestate1<=1'b1;
    else if(cr_state_en&(cr_iset_value == 2'b10))
            cr_icachestate2<=1'b1;
    else if(cr_state_en&(cr_iset_value == 2'b11))
            cr_icachestate3<=1'b1;
    else if(imemraddr_valid&~i_cached&~inst_uncache_block&~cache28_miss)
            cr_icachestate5<=1'b1;


                                      

//Sedn to Instruction Result Bus//
assign iresbus_adei_0  = icachepaddr_adei ;
assign iresbus_tlbii_0 = icachepaddr_tlbii;
assign iresbus_tlbir_0 = icachepaddr_tlbir;
assign iresbus_ibe_0   = 1'b0;
assign iresbus_dib_0   = icachepaddr_dib_0 ;

assign iresbus_adei_1  = icachepaddr_adei  & fetch_two_inst;
assign iresbus_tlbii_1 = icachepaddr_tlbii & fetch_two_inst;
assign iresbus_tlbir_1 = icachepaddr_tlbir & fetch_two_inst;
assign iresbus_ibe_1   = iresbus_ibe_0     & fetch_two_inst;
assign iresbus_dib_1   = (icachepaddr_dib_1)& fetch_two_inst & (iresbus_valid_0);

assign imemres_flag=imemrqueue_valid&(memres_block_rdy&memres_is_icache | memres_dr_dw);

assign IBE_FROM_CACHE=iresbus_ibe_0;

wire [63:0] iresbus_value2,iresbus_value3,iresbus_value4,iresbus_value5;
assign iresbus_value2=i0_data;
assign iresbus_value3=i1_data;
assign iresbus_value4=i2_data;
assign iresbus_value5=i3_data;

wire in_returning_cacheline = in_cacheline & in_returning_cacheline_t;
//after update link(second read), there may be two way matche simutiously
wire icache_data_select0 = second_hit ? (second_hit_way ==2'b00) : i_cached&(way_pred_reg == 2'b00)&~in_returning_cacheline;
wire icache_data_select1 = second_hit ? (second_hit_way ==2'b01) : i_cached&(way_pred_reg == 2'b01)&~in_returning_cacheline;
wire icache_data_select2 = second_hit ? (second_hit_way ==2'b10) : i_cached&(way_pred_reg == 2'b10)&~in_returning_cacheline;
wire icache_data_select3 = second_hit ? (second_hit_way ==2'b11) : i_cached&(way_pred_reg == 2'b11)&~in_returning_cacheline;
busmux_6s1_64b iresbus_04(
        .in1(iresbus_value1),
        .in2(iresbus_value2),
        .in3(iresbus_value3),
        .in4(iresbus_value4),
        .in5(iresbus_value5),
        .in6(returning_cacheline_hit_value),
        .sel1(imemres_dr_dw),
        .sel2(icache_data_select0),
        .sel3(icache_data_select1),
        .sel4(icache_data_select2),
        .sel5(icache_data_select3),
        .sel6(iresbus_returning_cacheline_hit),
        .out(iresbus_value));
assign iresbus_value_0 = iresbus_value[31:0];
assign iresbus_value_1 = iresbus_value[63:32];
/*********for way prediction************/
assign iresbus_paddr_tag = i_tag;
assign iresbus_v_lock =link_info_t[11:10];
assign iresbus_link_info = link_info_t[9:0];


wire icache_hit_select0 = second_hit ? (second_hit_way ==2'b00) : i_cached&(way_pred_reg == 2'b00);
wire icache_hit_select1 = second_hit ? (second_hit_way ==2'b01) : i_cached&(way_pred_reg == 2'b01);
wire icache_hit_select2 = second_hit ? (second_hit_way ==2'b10) : i_cached&(way_pred_reg == 2'b10);
wire icache_hit_select3 = second_hit ? (second_hit_way ==2'b11) : i_cached&(way_pred_reg == 2'b11);
//`endif

busmux_4s1_12b res_link(
        .in1(i0_tag[31:20]),
        .in2(i1_tag[31:20]),
        .in3(i2_tag[31:20]),
        .in4(i3_tag[31:20]),
        .sel1(icache_hit_select0),
        .sel2(icache_hit_select1),
        .sel3(icache_hit_select2),
        .sel4(icache_hit_select3),
        .out(link_info_t));


busmux_4s1_2b way_hit(
        .in1(2'b00),
        .in2(2'b01),
        .in3(2'b10),
        .in4(2'b11),
        .sel1(icache_hit_select0),
        .sel2(icache_hit_select1),
        .sel3(icache_hit_select2),
        .sel4(icache_hit_select3),
        .out(iresbus_way_hit));

assign  iresbus_no_update = iresbus_returning_cacheline_hit | i_valid&imemres_dr_dw;

assign iresbus_one_word = ~fetch_two_inst;
/******************/
endmodule

/////////////////////////////////////////////////////////////
//                                                         //
//         Instruction Cache Read & Write Module           //
//                                                         //
/////////////////////////////////////////////////////////////

module icache_read_write(clock,reset,i_set,
	cr_icachestate_value,
	iram_index,i_tag,i_offset,
    icache_v_offset,
	di_valid,di_index,di_set,
        di_tag, di_taglow,di_cache0, di_cache8, di_cache16,di_cache28,
    di_vaddr_dicache_valid,
	memres,cancel_write,
	i_tag_match,i_tag_valid,
	i_0_tag,i_0_lock,i_0_data,
        from_ram,to_ram,
        current_imemreq,
        imemrqueue_addr,
        read_instr_again,
        cr_cfg6_cache0_all,
        way_select,
        second_read,
        update_link_way,
        update_link,
        data_cen,
        di_cache28_miss,
        cacheline_writtendata,
        cache0_validtime);

	
input clock,cr_icachestate_value;
input reset;
input di_valid;
input [19:0] i_tag;
input [4:0] i_offset;
input [6:0] di_index;
input [21:0] iram_index; 
input [19:0] di_tag;
input [31:0] di_taglow;
input di_cache0, di_cache8, di_cache16, di_cache28;
input di_vaddr_dicache_valid;
input [5:0] memres;
input cancel_write;
input [287:0] from_ram;
input [2:0] icache_v_offset;

input [31:0] imemrqueue_addr;
input [255:0] cacheline_writtendata;
input read_instr_again;
input cr_cfg6_cache0_all;
input current_imemreq;

input way_select;
input second_read;
input update_link_way;
input[39:0] update_link;
input[4:0] data_cen;
input [1:0]i_set,di_set;

input di_cache28_miss; //for cache28 miss, refill the cache 

output [31:0] i_0_tag;
output i_0_lock;
output [63:0] i_0_data; 
output i_tag_match,i_tag_valid;
output [360:0] to_ram;
output cache0_validtime;


wire memres_block_rdy;
wire [3:0] memres_width;
wire  memres_is_icache;

assign memres_block_rdy=memres[5];
assign memres_width=memres[4:1];
assign memres_is_icache=memres[0];

wire      pc_en_nowayhit         = iram_index[0];
wire[6:0] vaddr_index            = iram_index[7:1];
wire[6:0] paddr_index            = iram_index[14:8];
wire[6:0] di_vaddr_index         = iram_index[21:15];

wire       way_select_wayhit = data_cen[0];
wire       pc_en_wayhit      = data_cen[1];
wire[2:0]  offset_wayhit     = data_cen[4:2];

wire [31:0] icache_tag;
wire icache_tag_enable,icache_data_enable;
wire iset_match;


assign iset_match=(di_set==i_set);

wire di_tag_match = (i_0_tag[19:0] == di_tag)&i_tag_valid; 

assign i_tag_match=(i_0_tag[19:0]==i_tag) ;

wire di_match; //cache instruction index or hits.
wire cache0_valid = cr_cfg6_cache0_all ? di_cache0 : di_cache0&&iset_match;
assign di_match = di_valid & ((di_cache8&iset_match | cache0_valid) | (di_cache16 | di_cache28)&di_tag_match);

reg cache0_validtime_r;
always @(posedge clock)
if (reset)
    cache0_validtime_r<=1'b0;
else
  if (cr_icachestate_value& memres_block_rdy&memres_is_icache&(memres_width==4'b0001)&(~cancel_write)&current_imemreq)
     cache0_validtime_r<=1'b0;
  else
     if (di_match &cr_icachestate_value&(di_index==imemrqueue_addr[11:5]))
        cache0_validtime_r<=1'b1;

assign cache0_validtime = cache0_validtime_r;

//Icache Tag Set//
wire mm_tag_en; //when cache refill, write tag
assign mm_tag_en=cr_icachestate_value&memres_block_rdy&memres_is_icache&(memres_width==4'b0001)&
                 ~cancel_write&current_imemreq&
                 ( ~(di_match | cache0_validtime_r | di_vaddr_dicache_valid |(di_valid&di_cache28)) |
                   di_cache28_miss); 

assign icache_tag[31] = di_valid&di_cache8&iset_match    ? di_taglow[7]&di_taglow[6]:
                        mm_tag_en   ? 1'b1 :  1'b0;
assign icache_tag[30] = di_cache28_miss; //when cache 28 miss, lock the refill cache line
assign icache_tag[29:20] = 10'b0; //refill, the link info is initialled to 0 //by xucp  
assign icache_tag[19:0]  = mm_tag_en ? imemrqueue_addr[31:12] : di_taglow[27:8];

wire[6:0]  update_link_index = update_link[39:33];
wire       update_link_en = update_link[32] &( ~(di_vaddr_dicache_valid | di_match)&update_link_way );
wire[31:0] update_link_tag = update_link[31:0];

// data ram cen and wen ,index 
wire data_cen00, data_cen01, data_cen10, data_cen11;

wire bank0 = (offset_wayhit[2:1] == 2'b00);
wire bank1 = ~offset_wayhit[2]&~(offset_wayhit[1:0]==2'b00);
wire bank2 = ~offset_wayhit[2]&  offset_wayhit[1]&offset_wayhit[0] | offset_wayhit[2]&~offset_wayhit[1]; 
wire bank3 =  offset_wayhit[2]&~(offset_wayhit[1:0]==2'b00);

wire bank0_t = (icache_v_offset[2:1] == 2'b00);
wire bank1_t = ~icache_v_offset[2]&~(icache_v_offset[1:0]==2'b00);
wire bank2_t = ~icache_v_offset[2]&  icache_v_offset[1]&icache_v_offset[0] | icache_v_offset[2]&~icache_v_offset[1]; 
wire bank3_t =  icache_v_offset[2]&~(icache_v_offset[1:0]==2'b00);

assign data_cen00 = ((pc_en_nowayhit|read_instr_again|second_read )&way_select&bank0_t) |
                     way_select_wayhit&pc_en_wayhit &bank0 |
                     icache_data_enable;
assign data_cen01 = ((pc_en_nowayhit| read_instr_again | second_read) & way_select&bank1_t) |
                     way_select_wayhit&pc_en_wayhit &bank1 |
                     icache_data_enable;
assign data_cen10 = ((pc_en_nowayhit| read_instr_again | second_read) &  way_select&bank2_t) |
                     way_select_wayhit&pc_en_wayhit &bank2 |
                     icache_data_enable;
assign data_cen11 =  ((pc_en_nowayhit| read_instr_again | second_read)& way_select & bank3_t) |
                     way_select_wayhit&pc_en_wayhit &bank3 |
                     icache_data_enable;
assign icache_data_enable=mm_tag_en;

wire [6:0] rw_index_t  = icache_data_enable ? imemrqueue_addr[11:5]: 
                         pc_en_nowayhit     ? vaddr_index : paddr_index;

wire      data_pre_sel  = icache_data_enable | pc_en_nowayhit | read_instr_again ; 

wire[6:0] rw_index_addr = pc_en_wayhit ? vaddr_index : paddr_index;
wire[6:0] rw_index      = data_pre_sel ?  rw_index_t : rw_index_addr;

//wire di_match_wr = di_valid & ((di_cache0|di_cache8)&iset_match | (di_cache16 | di_cache28)&di_tag_match);
// tag ram cen and wen ,index 
assign icache_tag_enable = mm_tag_en | di_match;

//pc_in_en = pc_en_nowayhit | pc_en_wayhit
wire tag_cen=  pc_en_wayhit|update_link_en| 
               pc_en_nowayhit | read_instr_again|
               icache_tag_enable |   //tag write
               di_vaddr_dicache_valid ; 

wire icache_tag_wr_en   = icache_tag_enable ? 1'b1 : update_link_en;

wire[6:0] tag_rw_addr_t  = mm_tag_en? imemrqueue_addr[11:5]: 
                           di_vaddr_dicache_valid ? di_vaddr_index : 
                           pc_en_nowayhit         ? vaddr_index    : paddr_index;

wire[6:0] tag_rw_addr    = di_match ? di_index : tag_rw_addr_t; 

wire[6:0]  tag_rw_index_t  = pc_en_wayhit ?  vaddr_index: update_link_index;

wire tag_pre_sel = di_match |( mm_tag_en| di_vaddr_dicache_valid|pc_en_nowayhit|read_instr_again );
                    
wire[6:0]  tag_rw_index = tag_pre_sel ? tag_rw_addr: tag_rw_index_t;

wire icache_lock = di_valid&di_cache28&di_tag_match;

wire[31:0] icache_tag_t = icache_lock ? {i_0_tag[31],1'b1,i_0_tag[29:0]}:  
                          icache_tag_enable? icache_tag : update_link_tag  ;

// Instruction cache Mem
wire [255:0] cacheline_data ;
wire [31:0] cacheline_tag ;
//tag_ram
assign cacheline_tag = from_ram[31:0];
assign to_ram[0]     = ~tag_cen ;
assign to_ram[7: 1]  = tag_rw_index;
assign to_ram[8]     = ~icache_tag_wr_en;
assign to_ram[40: 9] = icache_tag_t;


//data ram
assign cacheline_data= from_ram[287 :32]; 
//bank0
assign to_ram[41]    = ~data_cen00; 
assign to_ram[48:42] = rw_index; 
assign to_ram[56:49] = {8{~icache_data_enable}}; 
assign to_ram[120:57] = cacheline_writtendata[63:0];
//bank1
assign to_ram[121]    = ~data_cen01; 
assign to_ram[128:122] = rw_index; 
assign to_ram[136:129] = {8{~icache_data_enable}}; 
assign to_ram[200:137] = cacheline_writtendata[127:64];
//bank2
assign to_ram[201]    = ~data_cen10; 
assign to_ram[208:202] = rw_index; 
assign to_ram[216:209] = {8{~icache_data_enable}}; 
assign to_ram[280:217] = cacheline_writtendata[191:128];
//bank3
assign to_ram[281]    = ~data_cen11; 
assign to_ram[288:282] = rw_index; 
assign to_ram[296:289] = {8{~icache_data_enable}}; 
assign to_ram[360:297] = cacheline_writtendata[255:192];

busmux_8s1_64b iresbus_01(.in(cacheline_data),
        .out(i_0_data),
        .addr(i_offset[4:2]));

assign i_tag_valid = cacheline_tag[31];
assign i_0_lock    = cacheline_tag[30];
assign i_0_tag     = cacheline_tag;
endmodule



/////////////////////////////////////////////////////////////
//                                                         //
//                     Bus Mux Selector                    //
//                                                         //
/////////////////////////////////////////////////////////////
module busmux_8s1_64b (in, out, addr);

input [255:0] in;
output [63:0] out;
input [2:0] addr;						   


reg [63:0] out1;
//wire [2:0] addr_n;
wire [2:0] addr_t;
wire [2:0] addr_n_t;
reg [31:0] temp0,temp1;

//assign addr_n   = addr + 1'b1;
assign addr_t   = {addr[0] ? (addr[2:1]+1'b1):addr[2:1],1'b0};
assign addr_n_t = {addr[2:1],1'b1};

always @ (addr_t or in)
begin
case (addr_t[2:1])
    2'b00: temp0 =in[31:0];
    2'b01: temp0 =in[95:64];
    2'b10: temp0 =in[159:128];
    2'b11: temp0 =in[223:192];
endcase
end

always @ (addr_n_t or in)
begin
case (addr_n_t[2:1])
    2'b00: temp1 =in[63:32];
    2'b01: temp1 =in[127:96];
    2'b10: temp1 =in[191:160];
    2'b11: temp1 =in[255:224];
endcase
end

assign out[31:0] = addr[0] ? temp1 : temp0;
assign out[63:32] = addr[0] ? temp0: temp1;
endmodule 


module busmux_6s1_64b(in1,in2,in3,in4,in5,in6,sel1,sel2,sel3,sel4,sel5,sel6,out);
input [63:0] in1,in2,in3,in4,in5,in6;
input sel1,sel2,sel3,sel4,sel5,sel6;
output [63:0] out;
assign out=({64{sel1}}&in1)|({64{sel2}}&in2)|({64{sel3}}&in3)|({64{sel4}}&in4)|({64{
sel5}}&in5)|({64{sel6}}&in6);
endmodule

//by xucp for way prediction
module busmux_4s1_12b(in1,in2,in3,in4,sel1,sel2,sel3,sel4,out);
input [11:0] in1,in2,in3,in4;
input sel1,sel2,sel3,sel4;
output [11:0] out;
assign out=({12{sel1}}&in1)|({12{sel2}}&in2)|({12{sel3}}&in3)|({12{sel4}}&in4);
endmodule

module busmux_4s1_2b(in1,in2,in3,in4,sel1,sel2,sel3,sel4,out);
input [1:0] in1,in2,in3,in4;
input sel1,sel2,sel3,sel4;
output [1:0] out;
assign out=({2{sel1}}&in1) |({2{sel2}}&in2)|({2{sel3}}&in3)|({2{sel4}}&in4);
endmodule
