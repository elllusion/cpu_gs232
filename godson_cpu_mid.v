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

module godson_cpu_mid(
    coreclock,
    interrupt_i,nmi,

    areset_n,
    
    arid, araddr, arlen, arsize, arburst, arlock,
    arcache, arprot, arvalid, arready,
    
    rid, rdata, rresp, rlast, rvalid, rready,
    
    awid, awaddr, awlen, awsize, awburst, awlock,
    awcache, awprot, awvalid, awready,
    
    wid, wdata, wstrb, wlast, wvalid, wready,
    
    bid, bresp, bvalid, bready,

    EJTAG_TCK,EJTAG_TDI,EJTAG_TMS,EJTAG_TRST,EJTAG_TDO,prrst_to_core,
    testmode
);

input          coreclock; 
input[4:0]     interrupt_i;
input          nmi;
input          testmode;
input          EJTAG_TCK,EJTAG_TDI,EJTAG_TMS,EJTAG_TRST;
output         EJTAG_TDO;
output         prrst_to_core ;

//global
wire            aclk;
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

assign aclk = coreclock;


wire [`Lram_to_tlb-1   :0] ram_to_tlb;
wire [`Ltlb_to_ram-1   :0] tlb_to_ram;
wire [`Lram_to_icache-1:0] ram_to_icache;
wire [`Licache_to_ram-1:0] icache_to_ram;
wire [`Lram_to_dcache-1:0] ram_to_dcache;
wire [`Ldcache_to_ram-1:0] dcache_to_ram;


wire core_rst_;

reg areset_n_1;
reg areset_n_2;

always @(posedge coreclock)
begin
  areset_n_1 <= areset_n;
  areset_n_2 <= areset_n_1;
end


godson_cpu_core  cpu_core(
            .coreclock(coreclock),
            .core_rst_(core_rst_),
            .interrupt_i(interrupt_i),.nmi(nmi),

            .aclk(aclk),  
            .areset_n(areset_n_2),
        
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
        
            .ram_to_tlb(ram_to_tlb),
            .tlb_to_ram(tlb_to_ram),
            .ram_to_icache(ram_to_icache),
            .icache_to_ram(icache_to_ram),
            .ram_to_dcache(ram_to_dcache),
            .dcache_to_ram(dcache_to_ram),
            .EJTAG_TDI(EJTAG_TDI),
            .EJTAG_TDO(EJTAG_TDO),
            .EJTAG_TMS(EJTAG_TMS),
            .EJTAG_TCK(EJTAG_TCK),
            .EJTAG_TRST(EJTAG_TRST),
            .prrst_to_core(prrst_to_core),
            .testmode(testmode)
            );

godson_ram_bist ram_bist(.clock(coreclock),
            .ram_to_tlb(ram_to_tlb),
            .tlb_to_ram(tlb_to_ram),
            .ram_to_icache(ram_to_icache),
            .icache_to_ram(icache_to_ram),
            .ram_to_dcache(ram_to_dcache),
            .dcache_to_ram(dcache_to_ram)
            );
endmodule


