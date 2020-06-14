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

module godson_tap_buffer_module(
    CLOCK , RESET , SOFTRESET , COMMITBUS_EX , DMSEG_DREQBUS ,
    DMSEG_IREQBUS , EJTAGBRK_FROM_CORE , DEBUGMODE_FROM_CORE ,
    DATA_FROM_TAP , PRACC_FROM_TAP , 
    PROBEN_FROM_TAP , TRAP_FROM_TAP , EJTAGBRK_FROM_TAP ,
    IRESBUS_FROM_DMSEG , DRESBUS_FROM_DMSEG , 
    DMSEG_DFREE , EJTAGBRK_TO_CORE , 
    PROBEN_TO_CORE , TRAP_TO_CORE , ADDR_TO_TAP , DATA_TO_TAP ,
    WIDTH_TO_TAP , WRITE_TO_TAP , PRACC_TO_TAP , EJTAGBRK_TO_TAP ,
    RESET_TO_TAP , DEBUGMODE_TO_TAP,
    //for cpu download from tap
    TR_FROM_TAP, TR_TO_TAP,TR_TO_CORE
);

input                     CLOCK              ;
input                     RESET              ;
input                     SOFTRESET          ;
input                     COMMITBUS_EX       ;
input [`Ldmseg_dreqbus-1:0] DMSEG_DREQBUS      ;
input [`Ldmseg_ireqbus-1:0] DMSEG_IREQBUS      ;
input                     EJTAGBRK_FROM_CORE ;
input                     DEBUGMODE_FROM_CORE;


input [31:0] DATA_FROM_TAP    ;
input        PRACC_FROM_TAP   ;
//input        PRRST_FROM_TAP   ;
input        PROBEN_FROM_TAP  ;
input        TRAP_FROM_TAP    ;
input        EJTAGBRK_FROM_TAP;
input        TR_FROM_TAP       ;

output [`Liresbus-1:0] IRESBUS_FROM_DMSEG;
output [64:0]        DRESBUS_FROM_DMSEG; 
output               DMSEG_DFREE       ;
output               EJTAGBRK_TO_CORE  ;
//output               PRRST_TO_CORE     ;
output               PROBEN_TO_CORE    ;
output               TRAP_TO_CORE      ;
output               TR_TO_CORE;

output [31:0] ADDR_TO_TAP     ;
output [31:0] DATA_TO_TAP     ;
output [ 1:0] WIDTH_TO_TAP    ;
output        WRITE_TO_TAP    ;
output        PRACC_TO_TAP    ;
output        EJTAGBRK_TO_TAP ;
output        RESET_TO_TAP    ;
output        DEBUGMODE_TO_TAP;
output        TR_TO_TAP       ;

wire        dreqbus_ades ;
wire        dreqbus_adel ;
wire [31:0] dreqbus_value;
wire [ 7:0] dreqbus_op   ;
wire [31:0] dreqbus_addr ;
wire [ 3:0] dreqbus_qid  ;
wire        dreqbus_valid;

wire        ireqbus_adei ;
wire [31:0] ireqbus_addr ;
wire        ireqbus_valid;
wire        ireqbus_valid_t;

wire        iresbus_valid   ;
wire        iresbus_cacherdy;
wire [31:0] iresbus_value   ;
wire        iresbus_adei    ;
wire        iresbus_tlbii   ;
wire        iresbus_tlbir   ;
wire        iresbus_ibe     ;
wire        iresbus_dib     ;

wire        dresbus_valid   ;
wire [ 3:0] dresbus_qid     ;
wire [ 7:0] dresbus_op      ;
wire [31:0] dresbus_value   ;
wire [31:0] dresbus_value_h ;
wire        dresbus_dbe     ;
wire        dresbus_adel    ;
wire        dresbus_ades    ;
wire        dresbus_tlbli   ;
wire        dresbus_tlblr   ;
wire        dresbus_tlbsi   ;
wire        dresbus_tlbsr   ;
wire        dresbus_mod     ;
wire        dresbus_watch   ;
wire        dresbus_cacherdy;
wire [ 2:0] dresbus_offset  ;
wire        dresbus_ddbl    ;
wire        dresbus_ddbs    ;
wire        dresbus_ddblimpr;
wire        dresbus_ddbsimpr;

wire width_sel1,width_sel2,width_sel3,width_sel4    ;
wire posi_sel1,posi_sel2     ;
wire lb,lh,lw,lbu,lhu,ll,sb,sh,sw,sc,lwl,lwr,swl,swr;
wire [ 7:0] d_op          ;
wire [ 1:0] d_offset,width;
wire        op_store      ;
wire [31:0] d_value       ;
wire [31:0]right_posi_value,mb_word,mh_word;

reg        pracc_from_tap_0_reg   , pracc_from_tap_reg   ;
reg        prrst_from_tap_0_reg   , prrst_from_tap_reg   ;
reg        proben_from_tap_0_reg  , proben_from_tap_reg  ;
reg        trap_from_tap_0_reg    , trap_from_tap_reg    ;
reg        ejtagbrk_from_tap_0_reg, ejtagbrk_from_tap_reg;
reg        tr_from_tap_0_reg,      tr_from_tap_reg;

reg pracc_to_tap_reg   ;
reg ejtagbrk_to_tap_reg;
reg keep_pracc_reg     ;
reg keep_dint_reg      ;
reg dreq_reg           ;
reg busy_reg           ;

reg keep_tr_reg     ;
reg tr_to_tap_reg   ;
reg tr_busy_reg     ;

assign dreqbus_ades  = DMSEG_DREQBUS[78];
assign dreqbus_adel  = DMSEG_DREQBUS[77];
assign dreqbus_value = DMSEG_DREQBUS[76:45];
assign dreqbus_op    = DMSEG_DREQBUS[44:37];
assign dreqbus_addr  = DMSEG_DREQBUS[36: 5];
assign dreqbus_qid   = DMSEG_DREQBUS[ 4: 1];
assign dreqbus_valid = DMSEG_DREQBUS[ 0];

assign ireqbus_valid_t  = DMSEG_IREQBUS[34];
assign ireqbus_adei  = DMSEG_IREQBUS[33];
assign ireqbus_addr  = DMSEG_IREQBUS[32: 1];
assign ireqbus_valid = DMSEG_IREQBUS[ 0];

// width of visiting dmseg
assign d_op     = dreqbus_op       ;
assign d_offset = dreqbus_addr[1:0];
assign d_value  = dreqbus_value    ;
//Mem Operations
assign lb      = (~d_op[5])&d_op[4]&(~d_op[3])&(~d_op[2])&(~d_op[1])&(~d_op[0]);
assign lh      = (~d_op[5])&d_op[4]&(~d_op[3])&(~d_op[2])&(~d_op[1])&d_op[0]   ;
assign lw      = (~d_op[5])&d_op[4]&(~d_op[3])&(~d_op[2])&d_op[1]&(~d_op[0])   ;
assign lbu     = (~d_op[5])&d_op[4]&(~d_op[3])&d_op[2]&(~d_op[1])&(~d_op[0])   ;
assign lhu     = (~d_op[5])&d_op[4]&(~d_op[3])&d_op[2]&(~d_op[1])&d_op[0]      ;
assign ll      = (~d_op[5])&d_op[4]&(~d_op[3])&d_op[2]&d_op[1]&(~d_op[0])      ;
assign sb      = (~d_op[5])&d_op[4]&d_op[3]&(~d_op[2])&(~d_op[1])&(~d_op[0])   ;
assign sh      = (~d_op[5])&d_op[4]&d_op[3]&(~d_op[2])&(~d_op[1])&d_op[0]      ; 
assign sw      = (~d_op[5])&d_op[4]&d_op[3]&(~d_op[2])&d_op[1]&(~d_op[0])      ;
assign sc      = (~d_op[5])&d_op[4]&d_op[3]&d_op[2]&d_op[1]&(~d_op[0])         ;
assign lwl     = d_op[5]&d_op[4]&d_op[3]&(~d_op[2])&(~d_op[1])&(~d_op[0])      ;
assign lwr     = d_op[5]&d_op[4]&d_op[3]&(~d_op[2])&(~d_op[1])&d_op[0]         ;
assign swl     = d_op[5]&d_op[4]&d_op[3]&d_op[2]&(~d_op[1])&(~d_op[0])         ;
assign swr     = d_op[5]&d_op[4]&d_op[3]&d_op[2]&(~d_op[1])&d_op[0]            ;
assign op_store= (~d_op[5])&d_op[4]&d_op[3]|swl|swr                            ;

assign width_sel1 = lb|lbu|sb|(swl&(d_offset==2'b00))|(swr&(d_offset==2'b11));
assign width_sel2 = lh|lhu|sh|(swl&(d_offset==2'b01))|(swr&(d_offset==2'b10));
assign width_sel3 = (swl&(d_offset==2'b10))|(swr&(d_offset==2'b01))          ;
assign width_sel4 = lw|ll|sw|sc|(swl&(d_offset==2'b11))|(swr&(d_offset==2'b00))|lwl|lwr;

assign width[1] = width_sel3 | width_sel4;
assign width[0] = width_sel2 | width_sel3;

assign posi_sel1 = sb|(swl&(d_offset==2'b00))|(swr&(d_offset==2'b11));
assign posi_sel2 = sh|(swl&(d_offset==2'b01))|(swr&(d_offset==2'b10));

assign mb_word = {d_value[7:0],d_value[7:0],d_value[7:0],d_value[7:0]};
assign mh_word = {d_value[15:0],d_value[15:0]};
//value of triple and word is equal to d_value

assign right_posi_value = posi_sel1 ? mb_word : (posi_sel2 ? mh_word : d_value);

assign iresbus_dib      = 1'b0;
assign iresbus_valid    = ireqbus_valid_t&&keep_pracc_reg&&~pracc_from_tap_reg&&~dreq_reg;
assign iresbus_cacherdy = ireqbus_valid_t&&keep_pracc_reg&&~pracc_from_tap_reg&&~dreq_reg;
assign iresbus_value    = DATA_FROM_TAP;
assign iresbus_ibe      = 1'b0;
assign iresbus_adei     = ireqbus_adei ;
assign iresbus_tlbii    = 1'b0;
assign iresbus_tlbir    = 1'b0;

assign IRESBUS_FROM_DMSEG[38]  = iresbus_dib     ;
assign IRESBUS_FROM_DMSEG[37]  = iresbus_valid   ;
assign IRESBUS_FROM_DMSEG[36]  = iresbus_cacherdy;
assign IRESBUS_FROM_DMSEG[35:4]= iresbus_value   ; 
assign IRESBUS_FROM_DMSEG[3]   = iresbus_ibe     ; 
assign IRESBUS_FROM_DMSEG[2]   = iresbus_adei    ; 
assign IRESBUS_FROM_DMSEG[1]   = iresbus_tlbii   ;
assign IRESBUS_FROM_DMSEG[0]   = iresbus_tlbir   ;


assign dresbus_valid    = dreqbus_valid&&(dreqbus_adel||dreqbus_ades||
                          (keep_pracc_reg&&~pracc_from_tap_reg&&dreq_reg));
assign dresbus_value    = (dreqbus_adel||dreqbus_ades)?
                          dreqbus_value:DATA_FROM_TAP;
assign dresbus_value_h  = (lwl||lwr)?dreqbus_value:DATA_FROM_TAP;

assign DRESBUS_FROM_DMSEG[0]    = dresbus_valid    ;
assign DRESBUS_FROM_DMSEG[32:1] = dresbus_value    ;
assign DRESBUS_FROM_DMSEG[64:33]= dresbus_value_h  ;

//assign PRRST_TO_CORE    = prrst_from_tap_reg   ;
assign PROBEN_TO_CORE   = proben_from_tap_reg  ;
assign TRAP_TO_CORE     = trap_from_tap_reg    ;
assign EJTAGBRK_TO_CORE = ejtagbrk_from_tap_reg;
assign DMSEG_DFREE      = dreqbus_valid&&(dreqbus_adel||dreqbus_ades||
                          (keep_pracc_reg&&~pracc_from_tap_reg&&dreq_reg));

assign ADDR_TO_TAP      = dreq_reg?{dreqbus_addr[31:2],(lwl||lwr||swl)?2'b0:dreqbus_addr[1:0]}:ireqbus_addr ;
assign DATA_TO_TAP      = right_posi_value    ;
assign WIDTH_TO_TAP     = dreq_reg?width:2'b10;
assign WRITE_TO_TAP     = dreq_reg && op_store;
assign PRACC_TO_TAP     = pracc_to_tap_reg    ;
assign EJTAGBRK_TO_TAP  = EJTAGBRK_FROM_CORE  ;
assign RESET_TO_TAP     = RESET||SOFTRESET    ;
assign DEBUGMODE_TO_TAP = DEBUGMODE_FROM_CORE ;

assign TR_TO_CORE      = keep_tr_reg &&~tr_from_tap_reg;
assign TR_TO_TAP       = tr_to_tap_reg;

always @(posedge CLOCK)
begin
pracc_from_tap_0_reg   <= PRACC_FROM_TAP   ;
//prrst_from_tap_0_reg   <= PRRST_FROM_TAP   ;
proben_from_tap_0_reg  <= PROBEN_FROM_TAP  ;
trap_from_tap_0_reg    <= TRAP_FROM_TAP    ;
ejtagbrk_from_tap_0_reg<= EJTAGBRK_FROM_TAP;
tr_from_tap_0_reg      <= TR_FROM_TAP;

pracc_from_tap_reg   <= pracc_from_tap_0_reg   ;
//prrst_from_tap_reg   <= prrst_from_tap_0_reg   ;
proben_from_tap_reg  <= proben_from_tap_0_reg  ;
trap_from_tap_reg    <= trap_from_tap_0_reg    ;
ejtagbrk_from_tap_reg<= ejtagbrk_from_tap_0_reg;

tr_from_tap_reg      <= tr_from_tap_0_reg   ;
keep_tr_reg          <= tr_from_tap_reg    ;

keep_pracc_reg <= pracc_from_tap_reg;
keep_dint_reg  <= EJTAGBRK_FROM_CORE;

if(RESET||COMMITBUS_EX)
begin
    pracc_to_tap_reg   <=1'b0;
    busy_reg           <=1'b0;
    ejtagbrk_to_tap_reg<=1'b0;
    tr_to_tap_reg   <=1'b0;
    tr_busy_reg     <= 1'b0; 
end
else
begin
    if(((ireqbus_valid&&~ireqbus_adei) || 
        (dreqbus_valid&&~dreqbus_ades&&~dreqbus_adel)) && 
       ~pracc_from_tap_reg && ~pracc_to_tap_reg && ~busy_reg)
        begin
        pracc_to_tap_reg<=1'b1;
        busy_reg        <=1'b1;
        if(dreqbus_valid)
            dreq_reg<=1'b1;
        else
            dreq_reg<=1'b0;
        end
    else 
        begin
         if(~keep_pracc_reg && pracc_from_tap_reg)
             pracc_to_tap_reg<=1'b0;
         if( keep_pracc_reg && ~pracc_from_tap_reg && ~pracc_to_tap_reg)
             busy_reg        <=1'b0;
        end

    if(keep_dint_reg && ~EJTAGBRK_FROM_CORE && ejtagbrk_from_tap_reg)
        ejtagbrk_to_tap_reg<=1'b1;
    else if(~ejtagbrk_from_tap_reg)
             ejtagbrk_to_tap_reg<=1'b0;

//for accelerate cpu download, by xucp
//we think the cpu is much faster than tap.so we should get data from tap one by one without interrupt
   if(~tr_from_tap_reg & ~tr_to_tap_reg & ~tr_busy_reg)
     begin
      tr_to_tap_reg <= 1'b1;
      tr_busy_reg   <= 1'b1;
     end
   else 
     begin
       if(~keep_tr_reg & tr_from_tap_reg)
          tr_to_tap_reg <= 1'b0;
       if(keep_tr_reg & ~tr_from_tap_reg & ~tr_to_tap_reg)
          tr_busy_reg  <= 1'b0;
     end
end

end
endmodule
