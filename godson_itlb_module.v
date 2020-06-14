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

module godson_itlb_module(
    clock, reset, 
    instout_i,
    icachepaddr_o,
    
    erl,mode_user,mode_super,config_k0,
    itlb_req, 
    itlb_flush_i,
    tlb_to_itlb, 
    ram_to_tlb_i,
    commitbus_to_itlb,
    brbus_deret_err,
    HB_DIB,
    HB_ICOMPBUS,
    NODCR,
    PROBEN_IN,
    DEBUG_MODE,
    IR_WAIT_BD,
    DMSEG_IREQBUS,
    ICACHERDY_AFTER_DERET,
    DERET_IFLAG,
    itlb_access_o,
    itlbmiss_tlbhit_o);

input clock, reset;
// icache and itlb connection 
input   [ 33:0] instout_i;
output  [ 39:0] icachepaddr_o;

input       erl;
input       mode_user;
input       mode_super;
input [2:0] config_k0;

input                     itlb_flush_i;
input [`Lram_to_tlb-1:0]  ram_to_tlb_i;
input [`Ltlb_to_itlb-1:0] tlb_to_itlb;  
output[32:0]              itlb_req;

input[`Lcommitbus_to_itlb-1:0] commitbus_to_itlb;
input brbus_deret_err;

input [1:0]  HB_DIB;
output[40:0] HB_ICOMPBUS;
output[1:0]  ICACHERDY_AFTER_DERET ;
output       DERET_IFLAG;

input NODCR;
input PROBEN_IN;
input DEBUG_MODE;
input IR_WAIT_BD;
output [`Ldmseg_ireqbus-1:0]  DMSEG_IREQBUS;

output    itlb_access_o;
output    itlbmiss_tlbhit_o;
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

wire        itlb_req_valid;
wire [31:0] itlb_req_pc;

assign itlb_req[32]   = itlb_req_valid;
assign itlb_req[31:0] = itlb_req_pc;

reg         tlb_lookup_ok_r; //itlb  request TLB is successful.      

reg[31:0]   tlb_pc_r;
reg         pc_in_en_r;

wire        tlb_valid;  //TLB ack itlb's req
wire        tlb_find; // TLB find
wire [ 7:0] asid;
wire [ 1:0] cr_random;
wire [ 7:0] tlb_pagemask;
wire [15:0] tlb_pagemask_value;
wire [ 7:0] tlb_asid;
wire        tlb_g;

wire [31:0] pc_instout      = instout_i[33:2];       
wire        valid_instout   = instout_i[1];
wire        pc_in_en_instout= instout_i[0];

wire        commitbus_bd      = commitbus_to_itlb[8];
wire        commit_deret      = commitbus_to_itlb[7];
wire[5:0]   commitbus_excode  = commitbus_to_itlb[6:1];
wire        commitbus_ex      = commitbus_to_itlb[0];

wire        icachepaddr_valid_t;
wire        icachepaddr_valid;
wire        icachepaddr_cached;
wire [31:0] icachepaddr_addr;
wire        icachepaddr_adei;
wire        icachepaddr_tlbii;
wire        icachepaddr_tlbir;
wire        icachepaddr_dib_0;
wire        icachepaddr_dib_1;

assign icachepaddr_o[39]    = icachepaddr_valid_t;
assign icachepaddr_o[38]    = icachepaddr_dib_1;
assign icachepaddr_o[37]    = icachepaddr_dib_0;
assign icachepaddr_o[36]    = icachepaddr_valid;
assign icachepaddr_o[35]    = icachepaddr_cached;
assign icachepaddr_o[34:3]  = icachepaddr_addr;
assign icachepaddr_o[2]     = icachepaddr_adei;
assign icachepaddr_o[1]     = icachepaddr_tlbii;
assign icachepaddr_o[0]     = icachepaddr_tlbir;

assign tlb_valid    = tlb_to_itlb[36];
assign tlb_find     = tlb_to_itlb[35];
assign asid         = tlb_to_itlb[34:27];
assign cr_random    = tlb_to_itlb[26:25];
assign tlb_pagemask = tlb_to_itlb[16:9];
assign tlb_asid     = tlb_to_itlb[8:1];
assign tlb_g        = tlb_to_itlb[0];

assign tlb_pagemask_value = 
                  {tlb_pagemask[7], tlb_pagemask[7], tlb_pagemask[6], tlb_pagemask[6],
                   tlb_pagemask[5], tlb_pagemask[5], tlb_pagemask[4], tlb_pagemask[4],
                   tlb_pagemask[3], tlb_pagemask[3], tlb_pagemask[2], tlb_pagemask[2],
                   tlb_pagemask[1], tlb_pagemask[1], tlb_pagemask[0], tlb_pagemask[0]};

wire        ioddpage;

wire [ 7:0] itlb_asid;
wire [19:0] instr_vpn;
wire        itlb_wren;
wire [ 1:0] itlb_wr_index;
wire [19:0] itlb_wr_vpn;
wire [ 7:0] itlb_wr_mask;
wire [ 7:0] itlb_wr_asid;
wire        itlb_wr_g;
wire [31:0] itlb_wr_pfn;

wire [15:0] itlb_pagemask;
wire        itlb_find;
wire [31:0] itlb_pfn;

wire        iv;
wire [ 2:0] ic;
wire        id;
wire        ine;
wire        i_map;
wire [31:0] ipfn;
wire [31:0] imask;

///// exception found during instr fetch
wire        i_ex_adei  ;
wire        i_ex_tlbir ;
wire        i_ex_tlbii ;
wire        i_ex_dib_0 ;
wire        i_ex_dib_1 ;
wire        i_cached;

assign itlb_asid = asid;
assign instr_vpn = tlb_pc_r[31:12]; 

assign itlb_wren     = ~itlb_find & tlb_lookup_ok_r & tlb_find;
assign itlb_wr_index = cr_random[1:0]; 
assign itlb_wr_vpn   = tlb_pc_r[31:12]&({4'b1111, ~tlb_pagemask_value});
assign itlb_wr_mask  = tlb_pagemask;
assign itlb_wr_asid  = tlb_asid;
assign itlb_wr_g     = tlb_g;
assign itlb_wr_pfn   = ioddpage ? pfn1_from_ram : pfn0_from_ram;

wire [31:0] ioddpage_1 = {3'h0, tlb_pagemask_value, 13'h1fff};
wire [31:0] ioddpage_2 = ioddpage_1&(~{1'b0, ioddpage_1[31:1]});
assign ioddpage = |(ioddpage_2&tlb_pc_r);

assign itlb_access_o = pc_in_en_r &i_map; //for PCounter
micro_itlb u_micro_itlb(
                      .clock(clock),
                      .asid(itlb_asid),
                      .instr_vpn(instr_vpn),
                      
                      .wren(itlb_wren),
                      .wr_index(itlb_wr_index),
                      .wr_vpn(itlb_wr_vpn),
                      .wr_mask(itlb_wr_mask),
                      .wr_asid(itlb_wr_asid),
                      .wr_g(itlb_wr_g),
                      .wr_pfn(itlb_wr_pfn),

                      .itlb_flush_i(itlb_flush_i),
                      
                      .ipagemask(itlb_pagemask),
                      .i_find(itlb_find),
                      .out_pfn(itlb_pfn)
                    );

always @(posedge clock)
begin
    if (pc_in_en_instout)
    begin
        tlb_pc_r <= pc_instout;
    end
end

always @(posedge clock)
begin
    if (pc_in_en_instout)
        tlb_lookup_ok_r <= 1'b0;
    else if (itlb_req_valid & tlb_valid) //imply ~pc_in_en_instout
        tlb_lookup_ok_r <= 1'b1;
end

wire   ejtag_ireq_valid;
assign itlb_req_valid = valid_instout & ~itlb_find & ~tlb_lookup_ok_r & ~ejtag_ireq_valid &i_map;
assign itlb_req_pc    = tlb_pc_r;


reg tlb_find_ok; //for check tlb transfer complete

//wire   i_find  = itlb_find ;
//wire   i_paddr_valid =valid_instout& (itlb_find |tlb_find_ok |  ~i_map);
wire   i_paddr_valid_t = itlb_find |tlb_find_ok |  ~i_map;
wire   i_valid = i_paddr_valid_t & ~ejtag_ireq_valid; 

assign imask= {4'h0, itlb_pagemask, 12'hfff} ;
assign ipfn = itlb_pfn[29:6] ; 

assign itlbmiss_tlbhit_o = tlb_find & tlb_lookup_ok_r;

always @ (posedge clock)
begin
  if(pc_in_en_instout)
   begin
    tlb_find_ok <= 1'b0;
   end
  else if(tlb_lookup_ok_r)
   begin
    tlb_find_ok <= 1'b1;
   end
end


always @(posedge clock)
 begin
    pc_in_en_r = pc_in_en_instout;
 end

assign iv   = itlb_pfn[1]     ;
assign ic   = itlb_pfn[5:3]   ;
assign id   = itlb_pfn[2]     ;
assign ine  = itlb_pfn[30]    ;

wire [2:0] iseg = tlb_pc_r[31:29];
wire i_kseg3    = iseg[2]&iseg[1]&iseg[0];       //3'b111
wire i_ksseg    = iseg[2]&iseg[1]&(~iseg[0]);    //3'b110
wire i_kseg1    = iseg[2]&(~iseg[1])&iseg[0];    //3'b101
wire i_kseg0    = iseg[2]&(~iseg[1])&(~iseg[0]); //3'b100
wire i_useg     = ~iseg[2];                      //3'b0xx
assign i_map    = i_kseg3&(~ejtag_ireq_valid) | i_ksseg | (i_useg&(~erl)); 

assign i_ex_adei    = i_valid & ((tlb_pc_r[1:0]!=2'b0)| 
                                 (mode_user&(~i_useg))  |
                                 (mode_super&(i_kseg0|i_kseg1|i_kseg3)) |
                                 (i_map&itlb_find& iv & ine) ) ;

assign i_ex_tlbir   = i_valid&i_map&(~itlb_find);
assign i_ex_tlbii   = i_valid&i_map& itlb_find&(~iv);
assign i_ex_dib_0   = i_valid&HB_DIB[0];
assign i_ex_dib_1   = i_valid&HB_DIB[1];

assign i_cached     = i_map&(ic==3'b011) | i_kseg0&(config_k0==3'b011);

wire [19:0] i_addr_unmap, i_paddr_h;
assign i_addr_unmap = {1'b0, {2{i_useg}}&tlb_pc_r[30:29], tlb_pc_r[28:12]};
assign i_paddr_h    = i_map ? 
                      {ipfn[19:0]|(tlb_pc_r[31:12]&imask[31:12])} :
                      i_addr_unmap;

// icachepaddr_o bus
assign icachepaddr_valid_t = i_valid;
assign icachepaddr_valid   = i_valid & valid_instout;
assign icachepaddr_cached  = i_cached;
assign icachepaddr_addr    = {i_paddr_h, tlb_pc_r[11:0]};
assign icachepaddr_adei    = i_ex_adei;
assign icachepaddr_tlbii   = i_ex_tlbii;
assign icachepaddr_tlbir   = i_ex_tlbir;
assign icachepaddr_dib_0   = i_ex_dib_0;
assign icachepaddr_dib_1 = i_ex_dib_1;

// Instruction Hardware Breakpoint
assign HB_ICOMPBUS[40:9]  = tlb_pc_r;
assign HB_ICOMPBUS[8:1]   = asid;
assign HB_ICOMPBUS[0]     = i_valid;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// dmseg instruction access request.
///////////
wire        ejtag_dseg_ireq;
wire        ejtag_dmseg_ireq;
wire        dmseg_ireq_enable;

wire        dmseg_ireqbus_adei;
wire [31:0] dmseg_ireqbus_addr;
wire        dmseg_ireqbus_valid;
wire        dmseg_ireqbus_valid_t;

assign ejtag_dseg_ireq       = (tlb_pc_r[31:21]==11'b11111111001);
assign ejtag_dmseg_ireq      = ejtag_dseg_ireq&&(~tlb_pc_r[20]);
assign dmseg_ireq_enable     = PROBEN_IN&&(~NODCR)&&DEBUG_MODE;
assign ejtag_ireq_valid      = ejtag_dmseg_ireq&&dmseg_ireq_enable;

assign dmseg_ireqbus_adei    = dmseg_ireqbus_valid&&(tlb_pc_r[1:0]!=2'b00);
assign dmseg_ireqbus_addr    = ({12'h002,tlb_pc_r[19:0]}); 
assign dmseg_ireqbus_valid   = i_paddr_valid_t & ejtag_ireq_valid & valid_instout;
assign dmseg_ireqbus_valid_t = i_paddr_valid_t & ejtag_ireq_valid;
                                                                                                                             
assign DMSEG_IREQBUS[34]     = dmseg_ireqbus_valid_t;
assign DMSEG_IREQBUS[33]     = dmseg_ireqbus_adei;
assign DMSEG_IREQBUS[32:1]   = dmseg_ireqbus_addr;
assign DMSEG_IREQBUS[0]      = dmseg_ireqbus_valid;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

reg        deret_iflag;
reg        dib_in_delayslot;
reg        dib_flag;
reg        dbd_flag;
wire[1:0]  ICACHERDY_AFTER_DERET;

wire ex_ejtagboot = (commitbus_excode[5:0]==`EX_EJTAGBOOT);
wire ex_ddbsimpr  = (commitbus_excode[5:0]==`EX_DDBSIMPR);
wire ex_debug     = (commitbus_excode[5:3]==3'b110)||ex_ejtagboot||ex_ddbsimpr;
wire ex_dib       = (commitbus_excode[5:0]==`EX_DIB);

always @(posedge clock)
if (reset)
   deret_iflag<=1'b0;
else
   if (brbus_deret_err)
      deret_iflag<=1'b1;
   else
      if (ICACHERDY_AFTER_DERET[0])
         deret_iflag<=1'b0;
// dbd_flag is different from cr_debug[31](DBD) and used to comply with
// the dib_in_delayslot signal when dib in delayslot.
always @(posedge clock)
if (reset)
   dbd_flag<=1'b0;
else
   if (commitbus_ex&&ex_debug&&(~DEBUG_MODE))
      dbd_flag<=commitbus_bd;

// dib_flag is different from cr_debug[4](DIB) and used to comply with
// the dib_in_delayslot signal when dib in delayslot.
always @(posedge clock)
if (reset)
  dib_flag<=1'b0;
else
  if (commitbus_ex&&ex_dib)
     dib_flag<=1'b1;
  else
     if (brbus_deret_err)
        dib_flag<=1'b0;

always @(posedge clock)
if (reset)
   dib_in_delayslot<=1'b0;
else
   if (brbus_deret_err&&dib_flag&&dbd_flag)
      dib_in_delayslot<=1'b1;
   else
     if (|ICACHERDY_AFTER_DERET)
        dib_in_delayslot<=1'b0;

assign ICACHERDY_AFTER_DERET[0]=deret_iflag&pc_in_en_instout
                  &( ~dib_in_delayslot | dib_in_delayslot&~(tlb_pc_r[4:2]==3'b111) ) //for the dib inst refetch immetly
                    | deret_iflag & pc_in_en_instout & IR_WAIT_BD ; 

                                   //for the next inst after the dib
assign ICACHERDY_AFTER_DERET[1]=brbus_deret_err&pc_in_en_instout&~dbd_flag|
                                deret_iflag &dib_in_delayslot&pc_in_en_instout 
                                ;

//assign DERET_IFLAG =deret_iflag;


assign DERET_IFLAG = ejtag_ireq_valid;
endmodule 

module micro_itlb(
    clock,
    asid,
    instr_vpn,
    
    wren,
    wr_index,
    wr_vpn,
    wr_mask,
    wr_asid,
    wr_g,
    wr_pfn,

    itlb_flush_i,
    
    ipagemask,
    i_find,
    out_pfn
);

input         clock;
input  [7:0]  asid;
input  [19:0] instr_vpn;

input         wren;
input  [1:0]  wr_index;
input  [19:0] wr_vpn;
input  [7:0]  wr_mask;
input  [7:0]  wr_asid;
input         wr_g;
input  [31:0] wr_pfn;

input         itlb_flush_i;

output [15:0] ipagemask;
output        i_find;
output [31:0] out_pfn;

wire irden0, irden1, irden2,irden3;

reg  [19:0] reg_vpn[3:0];
reg  [7:0]  reg_asid[3:0];
reg  [7:0]  reg_mask[3:0];
reg         reg_g[3:0];
reg         reg_valid[3:0];
reg  [25:0] reg_pfn[3:0];

wire [25:0] reg_pfn_in = {wr_pfn[30], wr_pfn[25:1]};

wire [15:0] reg_mask_0;
wire [15:0] reg_mask_1;
wire [15:0] reg_mask_2;
wire [15:0] reg_mask_3;

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


wire [3:0]  asid_match;

assign asid_match[0] = (reg_asid[0]==asid) | reg_g[0];
assign asid_match[1] = (reg_asid[1]==asid) | reg_g[1];
assign asid_match[2] = (reg_asid[2]==asid) | reg_g[2];
assign asid_match[3] = (reg_asid[3]==asid) | reg_g[3];

assign irden0  = (({4'b1111, ~reg_mask_0}&instr_vpn )==reg_vpn[0])&(asid_match[0]&reg_valid[0]);
assign irden1  = (({4'b1111, ~reg_mask_1}&instr_vpn )==reg_vpn[1])&(asid_match[1]&reg_valid[1]);
assign irden2  = (({4'b1111, ~reg_mask_2}&instr_vpn )==reg_vpn[2])&(asid_match[2]&reg_valid[2]);
assign irden3  = (({4'b1111, ~reg_mask_3}&instr_vpn )==reg_vpn[3])&(asid_match[3]&reg_valid[3]);

assign i_find = irden0&reg_valid[0] | irden1&reg_valid[1] | irden2&reg_valid[2] | irden3&reg_valid[3];

wire wren0 = (wr_index == 2'b00);
wire wren1 = (wr_index == 2'b01);
wire wren2 = (wr_index == 2'b10);
wire wren3 = (wr_index == 2'b11);

wire flush0 = itlb_flush_i;
wire flush1 = itlb_flush_i;
wire flush2 = itlb_flush_i;
wire flush3 = itlb_flush_i;

always @ (posedge clock)
begin
    if (flush0)
        reg_valid[0] <= 1'b0;
    else if (wren0&wren)  
        reg_valid[0] <= 1'b1;
  
    if (flush1)
        reg_valid[1] <= 1'b0;
    else if (wren1&wren)  
        reg_valid[1] <= 1'b1;
  
    if (flush2)
        reg_valid[2] <= 1'b0;
    else if (wren2&wren)  
        reg_valid[2] <= 1'b1;
  
    if (flush3)
        reg_valid[3] <= 1'b0;
    else if (wren3&wren)  
        reg_valid[3] <= 1'b1;
  
end

always @ (posedge clock)
begin
     if (wren0&wren)  
     begin 
        reg_vpn[ 0]<=wr_vpn; reg_asid[ 0]<=wr_asid; reg_mask[ 0]<=wr_mask; reg_g[ 0]<=wr_g; 
        reg_pfn[ 0]<=reg_pfn_in;
     end 

     if (wren1&wren)  
     begin 
        reg_vpn[ 1]<=wr_vpn; reg_asid[ 1]<=wr_asid; reg_mask[ 1]<=wr_mask; reg_g[ 1]<=wr_g; 
        reg_pfn[ 1]<=reg_pfn_in;
     end   

     if (wren2&wren)  
     begin 
        reg_vpn[ 2]<=wr_vpn; reg_asid[ 2]<=wr_asid; reg_mask[ 2]<=wr_mask; reg_g[ 2]<=wr_g; 
        reg_pfn[ 2]<=reg_pfn_in;
     end   

     if (wren3&wren)  
     begin 
        reg_vpn[ 3]<=wr_vpn; reg_asid[ 3]<=wr_asid; reg_mask[ 3]<=wr_mask; reg_g[ 3]<=wr_g; 
        reg_pfn[ 3]<=reg_pfn_in;
     end   
end

wire [25:0] reg_pfn_out=   reg_pfn[0] &{26{irden0}}|
                           reg_pfn[1] &{26{irden1}}|
                           reg_pfn[2] &{26{irden2}}|
                           reg_pfn[3] &{26{irden3}}; 

assign out_pfn = {1'b0, reg_pfn_out[25], 4'b0, reg_pfn_out[24:0], 1'b0};

wire [7:0] ipagemask_tmp = reg_mask[0] &{8{irden0}}|
                           reg_mask[1] &{8{irden1}}|
                           reg_mask[2] &{8{irden2}}|
                           reg_mask[3] &{8{irden3}}; 

assign ipagemask = {ipagemask_tmp[7], ipagemask_tmp[7], ipagemask_tmp[6], ipagemask_tmp[6],
                    ipagemask_tmp[5], ipagemask_tmp[5], ipagemask_tmp[4], ipagemask_tmp[4],
                    ipagemask_tmp[3], ipagemask_tmp[3], ipagemask_tmp[2], ipagemask_tmp[2],
                    ipagemask_tmp[1], ipagemask_tmp[1], ipagemask_tmp[0], ipagemask_tmp[0]};

endmodule //micro_itlb
