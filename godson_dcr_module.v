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

module godson_dcr_module(
    CLOCK,RESET,
    DCR_REQBUS,
    NMI_IN,
    PROBEN_IN,
    DRESBUS_FROM_DCR,
    INTE,
    NMI_OUT,
    PROBEN_OUT
);

input                   CLOCK     ;
input                   RESET     ;
input [`Ldcr_reqbus-1:0] DCR_REQBUS;
input                   NMI_IN    ;
input                   PROBEN_IN ;

output [64:0] DRESBUS_FROM_DCR;
output                 INTE            ;
output                 NMI_OUT         ;
output                 PROBEN_OUT      ;

wire        dcr_reqbus_ades;
wire        dcr_reqbus_adel;
wire [31:0] dcr_reqbus_value;
wire [ 7:0] dcr_reqbus_op   ;
wire [31:0] dcr_reqbus_addr ;
wire [ 3:0] dcr_reqbus_qid  ;
wire        dcr_reqbus_valid;

wire        dresbus_from_dcr_valid   ;
wire [31:0] dresbus_from_dcr_value   ;
wire [31:0] dresbus_from_dcr_value_h ;
wire        dresbus_from_dcr_adel    ;
wire        dresbus_from_dcr_ades    ;

reg  [3:0] dcr_reg;

assign dcr_reqbus_ades  = DCR_REQBUS[78];
assign dcr_reqbus_adel  = DCR_REQBUS[77];
assign dcr_reqbus_value = DCR_REQBUS[76:45];
assign dcr_reqbus_op    = DCR_REQBUS[44:37];
assign dcr_reqbus_addr  = DCR_REQBUS[36: 5];
assign dcr_reqbus_qid   = DCR_REQBUS[ 4: 1];
assign dcr_reqbus_valid = DCR_REQBUS[ 0]   ;

assign dresbus_from_dcr_valid        =dcr_reqbus_valid       ;
assign dresbus_from_dcr_value        ={27'h0001800, dcr_reg[3:1],1'b1,dcr_reg[0]};
assign dresbus_from_dcr_value_h      =32'b0                  ;

assign DRESBUS_FROM_DCR[0]    =dresbus_from_dcr_valid        ;
assign DRESBUS_FROM_DCR[32:1] =dresbus_from_dcr_value        ;
assign DRESBUS_FROM_DCR[64:33]=dresbus_from_dcr_value_h      ;

assign INTE       = dcr_reg[3]            ;
assign NMI_OUT    = dcr_reg[2]&&dcr_reg[1];  // = NMIE && NMIpend
assign PROBEN_OUT = dcr_reg[0]            ;

always @(posedge CLOCK)
begin
    if(RESET)
//        dcr_reg[31:1]<=31'h0001800d;
          dcr_reg[3:1]<=3'b110;
    else
        begin
            dcr_reg[1]<=NMI_IN;
            if(dcr_reqbus_valid & (dcr_reqbus_op==`OP_SW))
                dcr_reg[3:2]<=dcr_reqbus_value[4:3]; // = [IntE:NMIE]
        end
    dcr_reg[0]<=PROBEN_IN;
end

endmodule
