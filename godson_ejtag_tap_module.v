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

`timescale 1ns / 1ps
//fsm encode
`define EJTAG_TAP_fsm_width       3'd4

`define EJTAG_TAP_fsm_test         4'd0
`define EJTAG_TAP_fsm_idle         4'd1
`define EJTAG_TAP_fsm_seldr        4'd2
`define EJTAG_TAP_fsm_capturedr    4'd3
`define EJTAG_TAP_fsm_shiftdr      4'd4
`define EJTAG_TAP_fsm_exit1dr      4'd5
`define EJTAG_TAP_fsm_pausedr      4'd6
`define EJTAG_TAP_fsm_exit2dr      4'd7
`define EJTAG_TAP_fsm_updatadr     4'd8
`define EJTAG_TAP_fsm_selir        4'd9
`define EJTAG_TAP_fsm_captureir    4'd10
`define EJTAG_TAP_fsm_shiftir      4'd11
`define EJTAG_TAP_fsm_exit1ir      4'd12
`define EJTAG_TAP_fsm_pauseir      4'd13
`define EJTAG_TAP_fsm_exit2ir      4'd14
`define EJTAG_TAP_fsm_updatair     4'd15
//inst encode
//`define EJTAG_TAP_inst_EXTEST         5'b0_0000
`define EJTAG_TAP_inst_IDCODE         5'b0_0001
//`define EJTAG_TAP_inst_SAMPLE         5'b0_0010
`define EJTAG_TAP_inst_IMPCODE        5'b0_0011
//`define EJTAG_TAP_inst_HI_Z           5'b0_0101
//`define EJTAG_TAP_inst_CLAMP          5'b0_0110
`define EJTAG_TAP_inst_BYPASS1        5'b0_0111
`define EJTAG_TAP_inst_ADDRESS        5'b0_1000
`define EJTAG_TAP_inst_DATA           5'b0_1001
`define EJTAG_TAP_inst_CONTROL        5'b0_1010
`define EJTAG_TAP_inst_ALL            5'b0_1011
`define EJTAG_TAP_inst_EJTAGBOOT      5'b0_1100
`define EJTAG_TAP_inst_NORMALBOOT     5'b0_1101
`define EJTAG_TAP_inst_FASTDATA       5'b0_1110
`define EJTAG_TAP_inst_TCBCONTROLA    5'b1_0000
`define EJTAG_TAP_inst_TCBCONTROLB    5'b1_0001
`define EJTAG_TAP_inst_TCBADDRESS     5'b1_0010
`define EJTAG_TAP_inst_DBTX           5'b1_1100  //for fast download, by xucp
`define EJTAG_TAP_inst_BYPASS2        5'b1_1111

module godson_ejtag_tap_module(TCK              ,
                     TRST             ,
                     TMS              ,
                     TDI              ,
                     TDO              ,

                     DMSEG_ADDR       ,

                     DMSEG_RDATA      ,
                     DMSEG_WDATA      ,

                     ROCC_IN          ,
                     DMSEG_BE_IN      ,
                     PRNW             ,
                     PRACC_IN         ,
                     EJTAGBRK_IN      ,
                     DM               ,
                     TR_IN            ,

                     PRACC_OUT        ,
                     PRRST            ,
                     PROBEN           ,
                     PROBTRAP         ,
                     EJTAGBRK_OUT     ,
                     TR_OUT           ,
                     PO_TX           
                   );

input TCK;
input TRST;
input TMS;
input TDI;
output TDO;

input [31:0] DMSEG_ADDR;
input [31:0] DMSEG_RDATA;

output [31:0] DMSEG_WDATA;

input ROCC_IN;
input [1:0] DMSEG_BE_IN;
input PRNW;
input PRACC_IN;
input EJTAGBRK_IN ;
input DM;
input TR_IN;

output PRACC_OUT;
output PRRST;
output PROBEN;
output PROBTRAP;
output EJTAGBRK_OUT;
output TR_OUT;
output[31:0] PO_TX ;
wire ind;
wire cap_i;
wire shift_i;
wire updata_i;
wire cap_d;
wire shift_d;
wire updata_d;
wire tlr_sig;
wire tdo_i;
wire tdo_d;

wire [4:0] inst;

wire [13:0] sel; //add fast upload and download, by xucp
//wire [11:0] sel;
wire norboot;
wire boot;

wire pracc_en;


//2005-4-30 TDO register out
reg  TDO;
wire  tdo_tmp;

assign tdo_tmp = ind ? tdo_i : tdo_d;

always @ (negedge TCK or negedge TRST)
if(!TRST)
   TDO <= 1'b0;
else
   TDO <= tdo_tmp;

//assign TDO = ind ? tdo_i : tdo_d;

EJTAG_TAP_fsm FSM   (.TCK            (   TCK      ),
                     .TRST           (   TRST     ),
                     .TMS            (   TMS      ),
                     .IND            (   ind      ),
                     .CAPTURE_IR     (   cap_i    ),
                     .SHIFT_IR       (   shift_i  ),
                     .UPDATA_IR      (   updata_i ),
                     .CAPTURE_DR     (   cap_d    ),
                     .SHIFT_DR       (   shift_d  ),
                     .UPDATA_DR      (   updata_d ),
                     .TLR_SIG        (   tlr_sig  ),
                     .PRACC_EN       (   pracc_en ));


EJTAG_TAP_instreg instreg (.CLK     (TCK         ),
                          .RST_N    (TRST        ),
                          .S_RST    (tlr_sig     ),
                          .EN       (1'b1        ),
                          .CAP      (cap_i       ),
                          .SHIFT    (shift_i     ),
                          .UPDATA   (updata_i    ),
                          .SI       (TDI         ),
                          .PI       (inst        ),
                          .SO       (tdo_i       ),
                          .PO       (inst        ));

EJTAG_TAP_decode decode (.INST           (    inst     ),
                         .SEL            (    sel      ),
                         .NORBOOT        (    norboot   ),
                         .BOOT           (    boot     ));



EJTAG_TAP_reg_group reggroup(.TCK              (  TCK             ),
                             .TRST             (  TRST            ),
                             .TDI              (  TDI             ),
                             .TDO              (  tdo_d           ),

                             .CAPTURE_DR       (  cap_d           ),
                             .SHIFT_DR         (  shift_d         ),
                             .UPDATA_DR        (  updata_d        ),

                             .SEL              (  sel             ),
                             .NORBOOT          (  norboot|tlr_sig ),
                             .BOOT             (  boot            ),

                             .DMSEG_ADDR       (  DMSEG_ADDR      ),

                             .DMSEG_RDATA      (  DMSEG_RDATA     ),
                             .DMSEG_WDATA      (  DMSEG_WDATA     ),

                             .ROCC_IN          (  ROCC_IN         ),
                             .DMSEG_BE_IN      (  DMSEG_BE_IN     ),
                             .PRNW             (  PRNW            ),
                             .PRACC_IN         (  PRACC_IN        ),
                             .EJTAGBRK_IN      (  EJTAGBRK_IN     ),
                             .DM               (  DM              ),

                             .PRACC_OUT        (  PRACC_OUT       ),
                             .PRRST            (  PRRST           ),
                             .PROBEN           (  PROBEN          ),
                             .PROBTRAP         (  PROBTRAP        ),
                             .EJTAGBRK_OUT     (  EJTAGBRK_OUT    ),

                             .PRACC_EN         (  pracc_en        ),
                             .TR_IN            (  TR_IN           ),
                             .TR_OUT           (  TR_OUT          ),
                             .PO_TX            (  PO_TX           )
                             );


endmodule

module EJTAG_TAP_instreg (CLK           ,
                          RST_N         ,
                          S_RST         ,
                          EN            ,
                          CAP           ,
                          SHIFT         ,
                          UPDATA        ,
                          SI            ,
                          PI            ,
                          SO            ,
                          PO            );

input CLK, RST_N, S_RST;
input EN;
input CAP, SHIFT, UPDATA;
input SI;
output SO;
input [4 : 0] PI;
output [4 : 0] PO;

wire SO;

reg [4 : 0] PO;

reg [4 : 0] sr;


assign SO = sr[0];

always@(posedge CLK or negedge RST_N)
begin
  if(!RST_N)
    sr <= `EJTAG_TAP_inst_IDCODE;
  else
    begin
      if(S_RST)
        sr <= `EJTAG_TAP_inst_IDCODE; 
      else if(CAP && EN)
        sr <= PI;
      else if(SHIFT && EN)
        sr <= {SI, sr[4 : 1]};
    end
end

always@(posedge CLK or negedge RST_N)
 if(!RST_N)
   PO <= `EJTAG_TAP_inst_IDCODE;
 else
   begin
    if(S_RST)
       PO <= `EJTAG_TAP_inst_IDCODE;
    else if(UPDATA && EN)
       PO <= sr;
   end



endmodule

module EJTAG_TAP_reg_group(TCK              ,
                           TRST             ,
                           TDI              ,
                           TDO              ,

                           CAPTURE_DR       ,
                           SHIFT_DR         ,
                           UPDATA_DR        ,

                           SEL              ,
                           NORBOOT          ,
                           BOOT             ,

                           DMSEG_ADDR       ,

                           DMSEG_RDATA      ,
                           DMSEG_WDATA      ,

                           ROCC_IN          ,
                           DMSEG_BE_IN      ,
                           PRNW             ,
                           PRACC_IN         ,
                           EJTAGBRK_IN      ,
                           DM               ,

                           PRACC_OUT        ,
                           PRRST            ,
                           PROBEN           ,
                           PROBTRAP         ,
                           EJTAGBRK_OUT     ,

                           PRACC_EN         ,  
                          
                          //for accelerate cpu download
                           TR_IN,
                           TR_OUT,
                           PO_TX
                           );

input TCK, TRST;
input TDI;
output TDO;

input CAPTURE_DR, SHIFT_DR, UPDATA_DR;
input  [13:0] SEL;
input  NORBOOT;
input  BOOT;


input [31:0] DMSEG_ADDR;
input [31:0] DMSEG_RDATA;

output [31:0] DMSEG_WDATA;

input ROCC_IN;
input [1:0] DMSEG_BE_IN;
input PRNW;
input PRACC_IN;
input EJTAGBRK_IN ;
input DM;

output PRACC_OUT;
output PRRST;
output PROBEN;
output PROBTRAP;
output EJTAGBRK_OUT;

input         TR_IN;
output [31:0] PO_TX;
output        TR_OUT;

input PRACC_EN;

reg EJTAGBRK_OUT;
reg PRACC_OUT;
//reg TDO; //2005-4-30
reg tdo;

reg [31:0] dmseg_addr;
reg [31:0] dmseg_rdata;

//wire rst_n; //2005-7-15

wire tdo_id;
wire [31:0] temp_id;

wire tdo_imp;
wire [31:0] temp_imp;

wire tdo_addr;

wire tdi_data;
wire tdo_data;

wire tdo_ecr;

reg tdo_bypass;

wire tdo_tcbca;
wire [31:0] temp_tcbca;

wire tdo_tcbcb;
wire [31:0] temp_tcbcb;

wire tdo_tcbaddr;
wire [31:0] temp_tcbaddr;

//tx register and tr register
wire        tdo_tx;
wire [31:0] PO_TX;
reg         TXCTL_TR;
reg         tdo_tr;
reg         tr_buffer_1, tr_buffer_2;

//ECR
wire tdi_ecr;
reg [31:0] ecr_in;
reg        ecr_rocc_buffer_1, ecr_rocc_buffer_2;
reg        ecr_dm_buffer_1, ecr_dm_buffer_2;
reg        ecr_pracc_buffer_1, ecr_pracc_buffer_2;
wire tdo_ecr1;


//PrAcc ECR[18]

reg tdo_pracc;  //shift
wire pracc_sin;  //?¶ÁªÊäÈë


//ECR[17:13]

wire tdo_ecr2;
reg [4:0] ecr2;
reg [4:0] secr2;

//EjtagBrk   ECR[12]
reg tdo_brk;   //brk?¶ÁªÊä³ö

//normalboot only do clear ejtagboot indication
//do not reset other registers
//assign rst_n = TRST & ~ NORBOOT;

assign tdi_data = SEL[10] ? tdo_addr : TDI;
assign tdi_ecr = SEL[10] ? tdo_data : TDI;

assign pracc_sin = SEL[11] ? tdo_data : tdo_ecr1;

assign PRRST = ecr2[3];
assign PROBEN = ecr2[2];
assign PROBTRAP = ecr2[1];
assign tdo_ecr2 = secr2[0];
wire [31:0] data_pi;

assign data_pi = ecr_in[18] ? dmseg_rdata : DMSEG_WDATA;

always @(posedge TCK or negedge TRST)
  if(!TRST)
    begin
      ecr_rocc_buffer_1 <= 1'b0; 
      ecr_rocc_buffer_2 <= 1'b0; 
      ecr_dm_buffer_1 <= 1'b0; 
      ecr_dm_buffer_2 <= 1'b0; 
      ecr_pracc_buffer_1 <= 1'b0; 
      ecr_pracc_buffer_2 <= 1'b0; 
    end
  else
    begin
      ecr_rocc_buffer_1 <= ROCC_IN; 

      if(!(PRACC_EN & ( SEL[4] | SEL[5])& (PROBEN==1'b0)))
          ecr_pracc_buffer_1 <= PRACC_IN; 
      else
          ecr_pracc_buffer_1 <= 1'b0; 

      ecr_dm_buffer_1 <= DM; 
      
      ecr_rocc_buffer_2 <= ecr_rocc_buffer_1; 
      ecr_dm_buffer_2 <= ecr_dm_buffer_1; 
      ecr_pracc_buffer_2 <= ecr_pracc_buffer_1; 
    end

    

always @(posedge TCK or negedge TRST)
  if(!TRST)
      ecr_in <= 32'h0000_0000;
  else
    begin
      ecr_in[2:0] <= 3'b000;
      ecr_in[3] <= ecr_dm_buffer_2;
      ecr_in[17:4] <= {1'b0, PRRST, PROBEN, PROBTRAP,1'b0, EJTAGBRK_IN, 8'b0000_0000};
      if(ecr_pracc_buffer_2)
        begin
          ecr_in[18] <= 1'b1;
          ecr_in[30:19] <= {DMSEG_BE_IN, 9'b0_0000_0000, PRNW};
        end
      else
        if(UPDATA_DR && (SEL[4] || SEL[5]) && !tdo_pracc)
            ecr_in[18] <= 1'b0;
      
      ecr_in[31] <= ecr_rocc_buffer_2;
    end



always @(posedge TCK or negedge TRST)
  if(!TRST)
    dmseg_addr <= 32'h0000_0000;
  else
    begin
      if(ecr_pracc_buffer_2)
         dmseg_addr <= DMSEG_ADDR;
    end


always @(posedge TCK or negedge TRST)
  if(!TRST)
    dmseg_rdata <= 32'h0000_0000;
  else
    begin
      if(ecr_pracc_buffer_2)
         dmseg_rdata <= DMSEG_RDATA;
    end


EJTAG_TAP_basicreg #(32, 32'h20010819) ID     (.CLK          ( TCK          ),
                           .RST_N        ( TRST         ),
                           .EN           ( SEL[0]       ),
                           .CAP          ( CAPTURE_DR   ),
                           .SHIFT        ( SHIFT_DR     ),
                           .UPDATA       ( 1'b0         ),
                           .SI           ( tdo_id       ),
                           .PI           ( temp_id      ),
                           .SO           ( tdo_id       ),
                           .PO           ( temp_id      ));
//                   defparam ID.WIDTH = 32;
 //                  defparam ID.RST_VAL = 32'h5a5a_5a5a;

EJTAG_TAP_basicreg #(32, 32'h40404000) IMP    (.CLK          ( TCK          ),
                           .RST_N        ( TRST        ),
                           .EN           ( SEL[1]       ),
                           .CAP          ( CAPTURE_DR   ),
                           .SHIFT        ( SHIFT_DR     ),
                           .UPDATA       ( 1'b0         ),
                           .SI           ( tdo_imp      ),
                           .PI           ( temp_imp     ),
                           .SO           ( tdo_imp      ),
                           .PO           ( temp_imp     ));
  //                 defparam IMP.WIDTH = 32;
  //                 defparam IMP.RST_VAL = 32'ha5a5_a5a5;

EJTAG_TAP_basicreg #(32, 32'h0000_0000) ADDR   (.CLK          ( TCK          ),
                           .RST_N        ( TRST         ),
                           .EN           ( SEL[2]       ),
                           .CAP          ( CAPTURE_DR   ),
                           .SHIFT        ( SHIFT_DR     ),
                           .UPDATA       ( 1'b0         ),
                           .SI           ( TDI          ),
                           .PI           ( (dmseg_addr|32'hff00_0000) ),
                           .SO           ( tdo_addr     ),
                           .PO           (              ));
//                   defparam ADDR.WIDTH = 32;
//                   defparam ADDR.RST_VAL = 32'h0000_0000;




EJTAG_TAP_basicreg #(32, 32'h0000_0000) DATA   (.CLK          ( TCK          ),
                           .RST_N        ( TRST         ),
                           .EN           ( SEL[3]       ),
                           .CAP          ( CAPTURE_DR   ),
                           .SHIFT        ( SHIFT_DR     ),
                           .UPDATA       ( UPDATA_DR    ),
                           .SI           ( tdi_data     ),
                           .PI           ( data_pi      ),
                           .SO           ( tdo_data     ),
                           .PO           ( DMSEG_WDATA  ));
//                   defparam DATA.WIDTH = 32;
//                   defparam DATA.RST_VAL = 32'h0000_0000;

//ECR


//ECR[31:19]  Ö»¶Á
EJTAG_TAP_basicreg #(13, 13'h0000) ECR1   (.CLK          ( TCK           ),
                           .RST_N        ( TRST          ),
                           .EN           ( SEL[4]        ),
                           .CAP          ( CAPTURE_DR    ),
                           .SHIFT        ( SHIFT_DR      ),
                           .UPDATA       ( 1'b0          ),
                           .SI           ( tdi_ecr       ),
                           .PI           ( ecr_in[31:19] ),
                           .SO           ( tdo_ecr1      ),
                           .PO           (               ));
//                   defparam ECR1.WIDTH = 13;
//                   defparam ECR1.RST_VAL = 13'b0_0000_0000;


//PrAcc ECR[18]


always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    tdo_pracc <= 1'b0;
  else
    begin
      if(CAPTURE_DR && (SEL[4] || SEL[5]))
        tdo_pracc <= ecr_in[18];
      else if(SHIFT_DR && (SEL[4] || SEL[5]))
        tdo_pracc <= pracc_sin;
    end
end

always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    PRACC_OUT <= 1'b0;
  else
    begin
      if(UPDATA_DR && (SEL[4] || SEL[5]))
        begin
          if(!tdo_pracc)
            PRACC_OUT <= 1'b0;
        end
      else
        if(ecr_in[18])  PRACC_OUT <= 1'b1;
    end
end

//ECR[17:13]



always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    secr2 <= 5'b0_0000;
  else
    begin
      if(NORBOOT)  //normalboot do clear ejtagboot indication
          secr2 <= {1'b0,secr2[3],3'b000};  //2005-7-15
      else if(BOOT)
        begin
       //   secr2 <= 5'b0_0110;   //ejtagboot may not clear prrst
          secr2 <= {1'b0,secr2[3],3'b110};  //2005-4-20
        end
      else
        begin
          if(CAPTURE_DR && SEL[4])
            secr2 <= ecr2;
          else if(SHIFT_DR && SEL[4])
            secr2 <= {tdo_pracc, secr2[4:1]};
        end
    end
end

always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    ecr2 <= 5'b0_0000;
  else
    begin
     if(NORBOOT) //normalboot do clear ejtagboot indication
       ecr2 <= {1'b0,ecr2[3],3'b000}; //2005-7-15
     else if(BOOT)
       //ecr2 <= 5'b0_0110;  //ejtagboot may not clear prrst
       ecr2 <= {1'b0,ecr2[3],3'b110};  //2005-4-20
     else
     begin
       if(UPDATA_DR && SEL[4])
         ecr2 <= secr2;
     end
    end
end

//EjtagBrk   ECR[12]


always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    tdo_brk <= 1'b0;
  else
    begin
      if(NORBOOT)  //normalboot do clear ejtagboot indication
          tdo_brk <= 1'b0; //2005-7-15
      else if(BOOT)
        begin
          tdo_brk <= 1'b1;
        end
      else
          begin
           if(CAPTURE_DR && SEL[4])
             tdo_brk <= EJTAGBRK_OUT; //ecr_in[12];  //2005-3-22
           else if(SHIFT_DR && SEL[4])
             tdo_brk <= tdo_ecr2;
        end
    end
end

always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    EJTAGBRK_OUT <= 1'b0;
  else
    begin
      if(NORBOOT)  //normalboot do clear ejtagboot indication
          EJTAGBRK_OUT <= 1'b0;   //2005-7-15
      else if(BOOT)
        begin
          EJTAGBRK_OUT <= 1'b1;
        end
      else
        begin
          if(UPDATA_DR && SEL[4])
            begin
              if(tdo_brk)
                EJTAGBRK_OUT <= 1'b1;
            end
          else
 //           if (ecr_in[12]) EJTAGBRK_OUT <= 1'b0;
            if (ecr_in[3]) EJTAGBRK_OUT <= 1'b0;     //2005-3-22
        end
    end
end

//ECR[11:0]


EJTAG_TAP_basicreg #(12, 12'h000) ECR3   (.CLK          ( TCK           ),
                           .RST_N        ( TRST          ),
                           .EN           ( SEL[4]        ),
                           .CAP          ( CAPTURE_DR    ),
                           .SHIFT        ( SHIFT_DR      ),
                           .UPDATA       ( 1'b0          ),
                           .SI           ( tdo_brk       ),
                           .PI           ( ecr_in[11:0]  ),
                           .SO           ( tdo_ecr       ),
                           .PO           (               ));
//                   defparam ECR3.WIDTH = 12;
//                   defparam ECR3.RST_VAL = 12'b0000_0000;

///////////////////////////////////
//for cpu download,by xucp, 2008-12-23

EJTAG_TAP_basicreg #(32, 32'h000) TX   
                          (.CLK          ( TCK           ),
                           .RST_N        ( TRST          ),
                           .EN           ( SEL[12]       ),
                           .CAP          ( CAPTURE_DR    ),
                           .SHIFT        ( SHIFT_DR      ),
                           .UPDATA       ( UPDATA_DR     ),
                           .SI           ( TDI           ),
                           .PI           ( PO_TX         ),
                           .SO           ( tdo_tx        ),
                           .PO           ( PO_TX         ));

always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
  begin
    tr_buffer_1 <= 1'b0;
    tr_buffer_2 <= 1'b0;
  end
  else
  begin
    if (!(PRACC_EN & SEL[12]&(PROBEN==1'b0)))
      tr_buffer_1 <= TR_IN;
    else
      tr_buffer_1 <= 1'b0;
     
     tr_buffer_2 <= tr_buffer_1;
  end
end

always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    tdo_tr <= 1'b0;
  else
    begin
      if(CAPTURE_DR && SEL[12])
        tdo_tr <= TXCTL_TR;
      else if(SHIFT_DR && SEL[12])
        tdo_tr <= tdo_tx;
    end
end

always @(posedge TCK or negedge TRST)
begin
  if(!TRST)
      TXCTL_TR <= 1'b0;
  else
   begin
      if(tr_buffer_2)
         TXCTL_TR <= 1'b1;
      else if(UPDATA_DR && SEL[12]& !tdo_tr)
         TXCTL_TR <= 1'b0;
      end
   end
reg TR_OUT;
always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    TR_OUT <= 1'b0;
  else
    begin
      if(UPDATA_DR && SEL[12])
        begin
          if(!tdo_tr)
            TR_OUT <= 1'b0;
        end
      else
        if(TXCTL_TR) TR_OUT <= 1'b1;
    end
end


/////////////////////////////////////////////////
//always@(posedge TCK or negedge rst_n)
always@(posedge TCK or negedge TRST)
begin
  //if(!rst_n) //2005-4-29
  if(!TRST)
    tdo_bypass <= 1'b0;
  else
    begin
     if(SHIFT_DR && SEL[6])
        tdo_bypass <= TDI;
    end
end


EJTAG_TAP_basicreg #(32, 32'h0000_0000) TCBCONTROLA(.CLK          ( TCK          ),
                               .RST_N        ( TRST        ),
                               .EN           ( SEL[7]       ),
                               .CAP          ( CAPTURE_DR   ),
                               .SHIFT        ( SHIFT_DR     ),
                               .UPDATA       ( UPDATA_DR    ),
                               .SI           ( TDI          ),
                               .PI           ( temp_tcbca   ),
                               .SO           ( tdo_tcbca    ),
                               .PO           ( temp_tcbca   ));
//                       defparam TCBCONTROLA.WIDTH = 32;
//                       defparam TCBCONTROLA.RST_VAL = 32'h0000_0000;

EJTAG_TAP_basicreg #(32, 32'h0000_0000) TCBCONTROLB(.CLK          ( TCK          ),
                               .RST_N        ( TRST         ),
                               .EN           ( SEL[8]       ),
                               .CAP          ( CAPTURE_DR   ),
                               .SHIFT        ( SHIFT_DR     ),
                               .UPDATA       ( UPDATA_DR    ),
                               .SI           ( TDI          ),
                               .PI           ( temp_tcbcb   ),
                               .SO           ( tdo_tcbcb    ),
                               .PO           ( temp_tcbcb   ));
//                       defparam TCBCONTROLB.WIDTH = 32;
//                       defparam TCBCONTROLB.RST_VAL = 32'h0000_0000;

EJTAG_TAP_basicreg #(32, 32'h0000_0000) TCBADDRESS (.CLK          ( TCK          ),
                               .RST_N        ( TRST         ),
                               .EN           ( SEL[9]       ),
                               .CAP          ( CAPTURE_DR   ),
                               .SHIFT        ( SHIFT_DR     ),
                               .UPDATA       ( UPDATA_DR    ),
                               .SI           ( TDI          ),
                               .PI           ( temp_tcbaddr ),
                               .SO           ( tdo_tcbaddr  ),
                               .PO           ( temp_tcbaddr ));
//                       defparam TCBADDRESS.WIDTH = 32;
//                       defparam TCBADDRESS.RST_VAL = 32'h0000_0000;


always@(SEL[9:0] or tdo_id or tdo_imp or tdo_addr or tdo_data or tdo_ecr or tdo_pracc
       or tdo_bypass or tdo_tcbca or tdo_tcbcb or tdo_tcbaddr  )
 case(SEL[9:0])
    10'b00_0000_0001   :  tdo = tdo_id;
    10'b00_0000_0010   :  tdo = tdo_imp;
    10'b00_0100_0000   :  tdo = tdo_bypass;
    10'b00_0000_0100   :  tdo = tdo_addr;
    10'b00_0000_1000   :  tdo = tdo_data;
    10'b00_0001_0000   :  tdo = tdo_ecr;
    10'b00_0001_1100   :  tdo = tdo_ecr;
    10'b00_0010_1000   :  tdo = tdo_pracc;
    10'b00_1000_0000   :  tdo = tdo_tcbca;
    10'b01_0000_0000   :  tdo = tdo_tcbcb;
    10'b10_0000_0000   :  tdo = tdo_tcbaddr;
    //14'b01_0010_0000_0000   :  tdo = tdo_tr; //noneed, because cpu is faster than ejtag
   default  : tdo = 1'b0;
  endcase

//2005-4-30 TDO register out
/*
always@(negedge TCK or negedge TRST)
if(!TRST)
  TDO <= 1'b0;
else
  TDO <= tdo;
*/

assign TDO = tdo;


endmodule

module EJTAG_TAP_fsm(TCK            ,
                     TRST           ,
                     TMS            ,
                     IND            ,
                     CAPTURE_IR     ,
                     SHIFT_IR       ,
                     UPDATA_IR      ,
                     CAPTURE_DR     ,
                     SHIFT_DR       ,
                     UPDATA_DR      ,
                     TLR_SIG        ,
                     PRACC_EN        );

input TCK, TRST;
input TMS;
output IND;
output CAPTURE_IR, SHIFT_IR, UPDATA_IR;
output CAPTURE_DR, SHIFT_DR, UPDATA_DR;
output TLR_SIG;
output PRACC_EN;

reg IND;
reg CAPTURE_IR, SHIFT_IR, UPDATA_IR;
reg CAPTURE_DR, SHIFT_DR, UPDATA_DR;

reg PRACC_EN;

reg [`EJTAG_TAP_fsm_width - 1 : 0] curr_state, next_state;

assign  TLR_SIG = (curr_state == `EJTAG_TAP_fsm_test) ? 1'b1 : 1'b0;

always@(TMS or curr_state)
  case(curr_state)
    `EJTAG_TAP_fsm_test         :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_test : `EJTAG_TAP_fsm_idle ;
    `EJTAG_TAP_fsm_idle         :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_seldr : `EJTAG_TAP_fsm_idle ;
    `EJTAG_TAP_fsm_seldr        :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_selir : `EJTAG_TAP_fsm_capturedr ;
    `EJTAG_TAP_fsm_capturedr    :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_exit1dr : `EJTAG_TAP_fsm_shiftdr ;
    `EJTAG_TAP_fsm_shiftdr      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_exit1dr : `EJTAG_TAP_fsm_shiftdr ;
    `EJTAG_TAP_fsm_exit1dr      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_updatadr : `EJTAG_TAP_fsm_pausedr ;
    `EJTAG_TAP_fsm_pausedr      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_exit2dr : `EJTAG_TAP_fsm_pausedr ;
    `EJTAG_TAP_fsm_exit2dr      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_updatadr : `EJTAG_TAP_fsm_shiftdr ;
    `EJTAG_TAP_fsm_updatadr     :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_seldr : `EJTAG_TAP_fsm_idle ;
    `EJTAG_TAP_fsm_selir        :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_test : `EJTAG_TAP_fsm_captureir ;
    `EJTAG_TAP_fsm_captureir    :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_exit1ir : `EJTAG_TAP_fsm_shiftir ;
    `EJTAG_TAP_fsm_shiftir      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_exit1ir : `EJTAG_TAP_fsm_shiftir ;
    `EJTAG_TAP_fsm_exit1ir      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_updatair : `EJTAG_TAP_fsm_pauseir ;
    `EJTAG_TAP_fsm_pauseir      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_exit2ir : `EJTAG_TAP_fsm_pauseir ;
    `EJTAG_TAP_fsm_exit2ir      :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_updatair : `EJTAG_TAP_fsm_shiftir ;
    default                     :  next_state = (TMS == 1'b1) ? `EJTAG_TAP_fsm_seldr : `EJTAG_TAP_fsm_idle ;
        //`EJTAG_TAP_fsm_updatair
  endcase

always@(posedge TCK or negedge TRST)
begin
  if(!TRST)
    begin
      curr_state <= `EJTAG_TAP_fsm_test;
      PRACC_EN <= 1'b0;
      IND <= 0;
      CAPTURE_IR <= 0;
      SHIFT_IR <= 0;
      UPDATA_IR <= 0;
      CAPTURE_DR <= 0;
      SHIFT_DR <= 0;
      UPDATA_DR <= 0;
    end
  else
    begin
      curr_state <= next_state;

      PRACC_EN <= ((next_state == `EJTAG_TAP_fsm_capturedr) |
                   (next_state == `EJTAG_TAP_fsm_shiftdr) |
                   (next_state == `EJTAG_TAP_fsm_exit1dr) |    
                   (next_state == `EJTAG_TAP_fsm_pausedr) |
                   (next_state == `EJTAG_TAP_fsm_exit2dr) |
                   (next_state == `EJTAG_TAP_fsm_updatadr)); 

      case(next_state)
        `EJTAG_TAP_fsm_test, `EJTAG_TAP_fsm_idle, `EJTAG_TAP_fsm_seldr,
        `EJTAG_TAP_fsm_exit1dr, `EJTAG_TAP_fsm_pausedr, `EJTAG_TAP_fsm_exit2dr,
        `EJTAG_TAP_fsm_selir   :
                                      begin
                                       IND <= 0;
                                        CAPTURE_IR <= 0;
                                        SHIFT_IR <= 0;
                                        UPDATA_IR <= 0;
                                        CAPTURE_DR <= 0;
                                        SHIFT_DR <= 0;
                                        UPDATA_DR <= 0;
                                      end

         `EJTAG_TAP_fsm_exit1ir, `EJTAG_TAP_fsm_pauseir, `EJTAG_TAP_fsm_exit2ir     :
                                      begin
                                        IND <= 1;
                                        CAPTURE_IR <= 0;
                                        SHIFT_IR <= 0;
                                        UPDATA_IR <= 0;
                                        CAPTURE_DR <= 0;
                                        SHIFT_DR <= 0;
                                        UPDATA_DR <= 0;
                                      end

        `EJTAG_TAP_fsm_capturedr    : begin
                                        IND <= 0;
                                        CAPTURE_DR <= 1;
                                      end
        `EJTAG_TAP_fsm_shiftdr      : begin
                                        CAPTURE_DR <= 0;
                                        SHIFT_DR <= 1;
                                      end
        `EJTAG_TAP_fsm_updatadr     : begin
                                        SHIFT_DR <= 0;
                                        UPDATA_DR <= 1;
                                      end
        `EJTAG_TAP_fsm_captureir    : begin
                                        IND <= 1;
                                        CAPTURE_IR <= 1;
                                      end
        `EJTAG_TAP_fsm_shiftir      : begin
                                        CAPTURE_IR <= 0;
                                        SHIFT_IR <= 1;
                                      end
        default                     : begin  //`EJTAG_TAP_fsm_updatair
                                        SHIFT_IR <= 0;
                                        UPDATA_IR <= 1;
                                      end
      endcase
    end
end


endmodule


module EJTAG_TAP_decode(INST             ,
                        SEL              ,
                        NORBOOT          ,
                        BOOT             );

input [4:0] INST;

output [13:0] SEL;
output NORBOOT;
output BOOT;


reg [13:0] SEL;
reg NORBOOT;
reg BOOT;


always@(INST)
  case (INST)
    `EJTAG_TAP_inst_IDCODE        :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0000_0001,1'b0,1'b0};
    `EJTAG_TAP_inst_IMPCODE       :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0000_0010,1'b0,1'b0};
    `EJTAG_TAP_inst_BYPASS1       :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0100_0000,1'b0,1'b0};
    `EJTAG_TAP_inst_ADDRESS       :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0000_0100,1'b0,1'b0};
    `EJTAG_TAP_inst_DATA          :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0000_1000,1'b0,1'b0};
    `EJTAG_TAP_inst_CONTROL       :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0001_0000,1'b0,1'b0};
    `EJTAG_TAP_inst_ALL           :   {SEL,NORBOOT,BOOT} = {14'b00_0100_0001_1100,1'b0,1'b0};
    `EJTAG_TAP_inst_EJTAGBOOT     :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0100_0000,1'b0,1'b1};
    `EJTAG_TAP_inst_NORMALBOOT    :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0100_0000,1'b1,1'b0};
    `EJTAG_TAP_inst_FASTDATA      :   {SEL,NORBOOT,BOOT} = {14'b00_1000_0010_1000,1'b0,1'b0};
    `EJTAG_TAP_inst_TCBCONTROLA   :   {SEL,NORBOOT,BOOT} = {14'b00_0000_1000_0000,1'b0,1'b0};
    `EJTAG_TAP_inst_TCBCONTROLB   :   {SEL,NORBOOT,BOOT} = {14'b00_0001_0000_0000,1'b0,1'b0};
    `EJTAG_TAP_inst_TCBADDRESS    :   {SEL,NORBOOT,BOOT} = {14'b00_0010_0000_0000,1'b0,1'b0};
    `EJTAG_TAP_inst_BYPASS2       :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0100_0000,1'b0,1'b0};
    `EJTAG_TAP_inst_DBTX          :   {SEL,NORBOOT,BOOT} = {14'b01_0000_0000_0000,1'b0,1'b0};
    default      :   {SEL,NORBOOT,BOOT} = {14'b00_0000_0000_0000,1'b0,1'b0};
 endcase

endmodule



module EJTAG_TAP_basicreg(CLK           ,
                          RST_N         ,
                          EN            ,
                          CAP           ,
                          SHIFT         ,
                          UPDATA        ,
                          SI            ,
                          PI            ,
                          SO            ,
                          PO            );
parameter WIDTH = 32;
parameter RST_VAL = 32'h0000_0000;

input CLK, RST_N;
input EN;
input CAP, SHIFT, UPDATA;
input SI;
output SO;
input [WIDTH - 1 : 0] PI;
output [WIDTH - 1 : 0] PO;

wire SO;

reg [WIDTH - 1 : 0] PO;

reg [WIDTH - 1 : 0] sr;


assign SO = sr[0];

always@(posedge CLK or negedge RST_N)
begin
  if(!RST_N)
    sr <= RST_VAL;
  else
    begin
      if(CAP && EN)
        sr <= PI;
      else if(SHIFT && EN)
        sr <= {SI, sr[WIDTH - 1 : 1]};
    end
end

always@(posedge CLK or negedge RST_N)
 if(!RST_N)
   PO <= RST_VAL;
 else
   begin
    if(UPDATA && EN)
       PO <= sr;
   end



endmodule
