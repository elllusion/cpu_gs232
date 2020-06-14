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

module godson_ram_bist
(
    input  wire                       clock,
    input  wire [   `Ltlb_to_ram-1:0] tlb_to_ram,
    output wire [   `Lram_to_tlb-1:0] ram_to_tlb,
    input  wire [`Ldcache_to_ram-1:0] dcache_to_ram,
    output wire [`Lram_to_dcache-1:0] ram_to_dcache,
    input  wire [`Licache_to_ram-1:0] icache_to_ram,
    output wire [`Lram_to_icache-1:0] ram_to_icache
);

genvar iway, ibank;
generate
    for (iway = 0; iway < `GS232_N_ICWAY; iway = iway + 1)
    begin : gen_icache_rambist_way
        sram_128x32 tag
            (
            .clka   ( clock                                ),
            .ena    (~icache_to_ram[361*iway+0            ]), //Chip Enable. ena high enables all operations.
            .addra  ( icache_to_ram[361*iway+7 :361*iway+1]), //Address. Synchronous.
            .wea    (~icache_to_ram[361*iway+8            ]), //Write Enable. Synchronous. Active high.
            .dina   ( icache_to_ram[361*iway+40:361*iway+9]), //Input data. Synchronous.
            .douta  ( ram_to_icache[288*iway+31:288*iway+0])  //Output Data. Synchronous.
            );

        for (ibank = 0; ibank < 4; ibank = ibank + 1)
        begin : gen_icache_rambist_data
            sram_128x64 bank
                (
                .clka   ( clock                                                         ),
                .ena    (~icache_to_ram[                        361*iway+41+80*ibank+0 ]), //Chip Enable. ena high enables all operations.
                .addra  ( icache_to_ram[361*iway+41+80*ibank+7 :361*iway+41+80*ibank+1 ]), //Address. Synchronous.
                .wea    (~icache_to_ram[361*iway+41+80*ibank+15:361*iway+41+80*ibank+8 ]), //Write Enable. Synchronous. Active high.
                .dina   ( icache_to_ram[361*iway+41+80*ibank+79:361*iway+41+80*ibank+16]), //Input data. Synchronous.
                .douta  ( ram_to_icache[288*iway+32+64*ibank+63:288*iway+32+64*ibank+0 ])  //Output Data. Synchronous.
                );
        end
    end
endgenerate


genvar dway, dbank;
generate
    for (dway = 0; dway < `GS232_N_DCWAY; dway = dway + 1)
    begin : gen_dcache_rambist_way
        sram_128x22 tag
            (
            .clka   ( clock                                ),
            .ena    (~dcache_to_ram[351*dway+0            ]), //Chip Enable. ena high enables all operations.
            .addra  ( dcache_to_ram[351*dway+7 :351*dway+1]), //Address. Synchronous.
            .wea    (~dcache_to_ram[351*dway+8            ]), //Write Enable. Synchronous. Active high.
            .dina   ( dcache_to_ram[351*dway+30:351*dway+9]), //Input data. Synchronous.
            .douta  ( ram_to_dcache[278*dway+21:278*dway+0])  //Output Data. Synchronous.
            );

        for (dbank = 0; dbank < 4; dbank = dbank + 1)
        begin : gen_dcache_rambist_data
            sram_128x64 bank
                (
                .clka   ( clock                                                         ),
                .ena    (~dcache_to_ram[                        351*dway+31+80*dbank+0 ]), //Chip Enable. ena high enables all operations.
                .addra  ( dcache_to_ram[351*dway+31+80*dbank+7 :351*dway+31+80*dbank+1 ]), //Address. Synchronous.
                .wea    (~dcache_to_ram[351*dway+31+80*dbank+15:351*dway+31+80*dbank+8 ]), //Write Enable. Synchronous. Active high.
                .dina   ( dcache_to_ram[351*dway+31+80*dbank+79:351*dway+31+80*dbank+16]), //Input data. Synchronous.
                .douta  ( ram_to_dcache[278*dway+22+64*dbank+63:278*dway+22+64*dbank+0 ])  //Output Data. Synchronous.
                );
        end
    end
endgenerate

wire        tlb_cen;
wire        tlb_wen;
wire [51:0] tlb_din;
wire [ 4:0] tlb_addr;
assign tlb_cen  = tlb_to_ram[0]; 
assign tlb_wen  = tlb_to_ram[7]; 
assign tlb_din  = tlb_to_ram[59:8];
assign tlb_addr = tlb_wen ? tlb_to_ram[5:1] : tlb_to_ram[64:60];

 sram_32x52bit tlb_reg_file
     (
     .clka   ( clock      ),
     .ena    (~tlb_cen    ), //Chip Enable. ena high enables all operations.
     .addra  ( tlb_addr   ), //Address. Synchronous.
     .wea    (~tlb_wen    ), //Write Enable. Synchronous. Active high.
     .dina   ( tlb_din    ), //Input data. Synchronous.
     .douta  ( ram_to_tlb )  //Output Data. Synchronous.
     );
endmodule

