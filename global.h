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

`ifdef GODSON_GLOBAL_H
`else
`define GODSON_GLOBAL_H

`timescale 1ns/10ps

`define GS232_N_ICWAY 4 

`define GS232_N_DCWAY 4 

`define FILTER_WINDOW_NUM 4
`define FILTER_WINDOW_DEPTH 3 //uplimit(FILTER_WINDOW_DEPTH=log2(FILTER_WINDOW_NUM*2))
`define FILTER_MASK_UNIT 8 //mask unit is (2^FILTER_MASK_UNIT) bytes

`define DBKP_NUM  4'd2 //in hb
`define IBKP_NUM  4'd6 //in hb
//----------------------------------------------------------

`define INIT_STATUS     32'H00400004
`define INIT_CONFIG     32'H80000480 //for releas2
`define INIT_PRID       32'H00004220
`define INIT_CONFIG2    32'H80000000
 
`define INIT_CONFIG3    32'H00000420 //for dsp implement

`define INIT_SRSCTRL    32'H00000000
`define INIT_SRSMAP     32'H00000000
`define INIT_INTCTRL    32'Hfc000000
//bits 31-31: (1) Config1 exist.
//bits 30-28: (0) non-fix mapping MMU for kseg2 and kseg3.
//bits 27-25: (0) non-fix mapping MMU for kuseg.
//bits 24-16: (0) reserved for implementation.
//bits 15-15: (0) little endian.
//bits 14-13: (0) MIPS32.
//bits 12-10: (0) release 1.
//bits  9- 7: (1) standard TLB.
//bits  6- 4: (0) reserved.
//bits  3- 3: (0) instruction cache is not virtual.
//bits  2- 0: (0) kseg0 coherency algorithm.

//the folowing INIT_CONFIG1 is valid only for TLB_32_ENTRIES

`define CFG1_RELEASE  1'h1  //1'b1 for config2 is implements 

`define CFG1_FLOAT    1'h0

`define CFG1_MMU      6'h1f 

`define CFG1_ICACHE 9'h63  //icache 4way
`define CFG1_DCACHE 9'h63  //dcache 4way

`define CFG1_C2 1'b0
`define CFG1_MD 1'b0
`define CFG1_PC 1'b1 
`define CFG1_WR 1'b0
`define CFG1_CA 1'b0
`define CFG1_EP 1'b1 

`define INIT_CONFIG1 {`CFG1_RELEASE, `CFG1_MMU,`CFG1_ICACHE, `CFG1_DCACHE,`CFG1_C2,`CFG1_MD,`CFG1_PC,`CFG1_WR,`CFG1_CA,      `CFG1_EP, `CFG1_FLOAT} 
//bits 30-25: (47) MMU entries 32.
//bits 24-22: (1) icache sets per way 128.
//bits 21-19: (0/4) No icache/icache line size 32 bytes.
//bits 18-16: (0/1/3) Direct mapped/icache two/four-way set associated.
//bits 15-13: (1) dcache sets per way 128.
//bits 12-10: (0/4) No dcache/dcache line size 32 bytes.
//bits  9- 7: (0/1/3) Direct mapped/dcache two/four-way set associated.
//bits  6- 6: (0) no cop2 implemented.
//bits  5- 5: (0) no MDMX ASE implemented.
//bits  4- 4: (1) performance counter implemented.
//bits  3- 3: (0) no watch register implemented.
//bits  2- 2: (0) no MIPS16e implemented.
//bits  1- 1: (1) EJTAG implemented.
//bits  0- 0: (1) FPU implemented.

`define CFG6_TAP_DONE 1'b0
`define CFG6_TAP_TR   1'b0
`define CFG6_CACHE0_ALL 1'b0
//1 indicate the data from tap can be used
`define CFG6_RT_INT   1'b0
//0 -- normal interrupt handle mechanism; 1 -- real time interrupt handle mechanism
`define CFG6_BR_PREDICT 3'b000

`define INIT_CONFIG6 {`CFG6_TAP_DONE,`CFG6_TAP_TR,3'b0,`CFG6_CACHE0_ALL,`CFG6_RT_INT, `CFG6_BR_PREDICT}

`define CFG7_DCACHE_WAY 2'b00 //4-WAY
`define CFG7_ICACHE_WAY 2'b00 //4-WAY
`define CFG7_DCACHE_ECC_EN 1'b0 //1--ON; 0--OFF
`define CFG7_ICACHE_PARITY_EN 1'b0 //1--ON; 0--OFF
`define INIT_CONFIG7 {`CFG7_ICACHE_PARITY_EN, `CFG7_DCACHE_ECC_EN, `CFG7_ICACHE_WAY, `CFG7_DCACHE_WAY}

 /*queue state*/
 `define EMPTY    2'B00
 `define UNISSUED 2'B01
 `define ISSUED   2'B11
 
 /*Exception encoding*/
 `define EX_RESET       6'D36
 `define EX_SOFTRESET   6'D37
 `define EX_NMI         6'D38
 `define EX_INTERRUPT    6'D0
 `define EX_MOD          6'D1
 `define EX_TLBLR       6'D34
 `define EX_TLBLI        6'D2
 `define EX_TLBSR       6'D35
 `define EX_TLBSI        6'D3
 `define EX_ADEL         6'D4
 `define EX_ADES         6'D5
 `define EX_IBE          6'D6
 `define EX_DBE          6'D7
 `define EX_SYS          6'D8
 `define EX_BP           6'D9
 `define EX_RI           6'D10
 `define EX_CPU          6'D11
 `define EX_OV           6'D12
 `define EX_TRAP         6'D13
 `define EX_FPE          6'D15
 `define EX_WATCH        6'D23
 `define EX_CACHEERR     6'D30
 `define EX_BNT          6'D63
 `define EX_DINT         6'D48
 `define EX_DSS          6'D49
 `define EX_SDBBP        6'D50
 `define EX_DIB          6'D51
 `define EX_DDBL         6'D52
 `define EX_DDBS         6'D53
 `define EX_DDBLIMPR     6'D54
 `define EX_DDBSIMPR     6'D56
 `define EX_EJTAGBOOT    6'D57
 
 /*Exception Entry Addr*/
 `define EX_ENTRY_RESET  32'Hbfc00000
 `define EX_ENTRY_BEV1   32'Hbfc00200
 `define EX_ENTRY_BEV0   32'H80000000
 
 /*Control register name*/
 `define CR_INDEX        5'D0
 `define CR_RANDOM       5'D1
 `define CR_ENTRYLO0     5'D2
 `define CR_ENTRYLO1     5'D3
 `define CR_CONTEXT      5'D4
 `define CR_PAGEMASK     5'D5
 `define CR_WIRED        5'D6
 `define CR_BADVADDR     5'D8
 `define CR_COUNT        5'D9
 `define CR_ENTRYHI      5'D10
 `define CR_COMPARE      5'D11
 `define CR_STATUS       5'D12
 `define CR_CAUSE        5'D13
 `define CR_EPC          5'D14
 `define CR_PRID         5'D15
 `define CR_CONFIG       5'D16
 `define CR_LLADDR       5'D17
 `define CR_WATCHLO      5'D18
 `define CR_WATCHHI      5'D19
 `define CR_TAGLO        5'D28
 `define CR_TAGHI        5'D29
 `define CR_ERRORPC      5'D30
 `define CR_DEBUG        5'D23
 `define CR_DEPC         5'D24
 `define CR_DESAVE       5'D31
 
 
 /*Fix point operation*/
 `define OP_CLO      8'H00 
 `define OP_CLZ      8'H01 
 `define OP_EXT      8'H02
 `define OP_INS      8'H03
 `define OP_WSBH     8'H04
 `define OP_ROTR     8'H06 //include ROTRV
 `define OP_SEB      8'H08
 `define OP_SEH      8'H09
 `define OP_MOVN     8'H0a 
 `define OP_MOVZ     8'H0b 
 `define OP_MFHI     8'H0c 
 `define OP_MFLO     8'H0d 
 `define OP_MTHI     8'H0e 
 `define OP_MTLO     8'H0f 
 `define OP_MUL      8'H10 
 `define OP_SLL      8'H11 //include NOP,SSNOP,EHB,SLLV
 `define OP_SRL      8'H12 //SRLV
 `define OP_SRA      8'H13
 `define OP_MULT     8'H14 
 `define OP_MULTU    8'H15 
 `define OP_DIV      8'H16
 `define OP_DIVU     8'H17
 `define OP_ADD      8'H18 //ADDI
 `define OP_ADDU     8'H19 //ADDIU, LUI, RDPGPR, WRPGPR
 `define OP_SUB      8'H1a
 `define OP_SUBU     8'H1b
 `define OP_AND      8'H1c //ANDI
 `define OP_OR       8'H1d //ORI
 `define OP_XOR      8'H1e //XORI
 `define OP_NOR      8'H1f
 `define OP_TEQ      8'H20 //TEQI
 `define OP_TNE      8'H21 //TNEI
 `define OP_TLT      8'H22 //TLTI
 `define OP_TLTU     8'H23 //TLTIU
 `define OP_TGE      8'H24 //TGEI`
 `define OP_TGEU     8'H25 //TEEI
 `define OP_SLT      8'H26 //SLTI
 `define OP_SLTU     8'H27 //SLTIU
 `define OP_MADD     8'H28 
 `define OP_MADDU    8'H29 
 `define OP_MSUB     8'H2a 
 `define OP_MSUBU    8'H2b 
 `define OP_J        8'H2c
 `define OP_JR       8'H2d //JR.HB
 `define OP_JAL      8'H2e 
 `define OP_JALR     8'H2f //JALR.HB
 `define OP_BEQ      8'H30
 `define OP_BNE      8'H31
 `define OP_BLEZ     8'H32
 `define OP_BGTZ     8'H33
 `define OP_BLTZ     8'H34
 `define OP_BGEZ     8'H35
 `define OP_BLTZAL   8'H36
 `define OP_BGEZAL   8'H37
 `define OP_BEQL     8'H38
 `define OP_BNEL     8'H39
 `define OP_BLEZL    8'H3a
 `define OP_BGTZL    8'H3b
 `define OP_BLTZL    8'H3c
 `define OP_BGEZL    8'H3d
 `define OP_BLTZALL  8'H3e
 `define OP_BGEZALL  8'H3f
 
 /*CP0 operation*/
 `define OP_MFC1     8'H80 
 `define OP_CFC1     8'H81 
 `define OP_MTC1     8'H82 
 `define OP_CTC1     8'H83 
 `define OP_DI       8'H84 //release2 
 `define OP_EI       8'H85 //release2
 `define OP_SYNC     8'H86
 `define OP_ERET     8'H87
 `define OP_TLBP     8'H88
 `define OP_TLBR     8'H89
 `define OP_TLBWI    8'H8a
 `define OP_TLBWR    8'H8b
 `define OP_MFC0     8'H8c //include RDHWR
 `define OP_MTC0     8'H8d
 `define OP_SYNCI    8'H8e 
 `define OP_DERET    8'H8f
 `define OP_LB       8'H90
 `define OP_LH       8'H91
 `define OP_LW       8'H92 
 `define OP_LDC1     8'H93
 `define OP_LBU      8'H94
 `define OP_LHU      8'H95
 `define OP_LL       8'H96
 `define OP_PREF     8'H97
 `define OP_SB       8'H98
 `define OP_SH       8'H99
 `define OP_SW       8'H9a 
 `define OP_SDC1     8'H9b 
 `define OP_LBUX     8'H9c 
 `define OP_LHX      8'H9d 
 `define OP_SC       8'H9e
 `define OP_LWX      8'H9f 
 `define OP_CACHE0   8'Ha0
 `define OP_CACHE1   8'Ha1
 `define OP_LWC1     8'Ha2
 `define OP_SWC1     8'Ha3
//`define OP_CACHE2   8'Ha2
//`define OP_CACHE3   8'Ha3
 `define OP_CACHE4   8'Ha4
 `define OP_CACHE5   8'Ha5
 `define OP_PREFX    8'Ha6
 `define OP_CACHE7   8'Ha7
 `define OP_CACHE8   8'Ha8
 `define OP_CACHE9   8'Ha9
 `define OP_CACHE10  8'Haa
 `define OP_CACHE11  8'Hab
 `define OP_CACHE12  8'Hac
 `define OP_CACHE13  8'Had
 `define OP_CACHE14  8'Hae
 `define OP_CACHE15  8'Haf
 `define OP_CACHE16  8'Hb0
 `define OP_CACHE17  8'Hb1
 `define OP_CACHE18  8'Hb2
 `define OP_CACHE19  8'Hb3
 `define OP_CACHE20  8'Hb4
 `define OP_CACHE21  8'Hb5
 `define OP_CACHE22  8'Hb6
 `define OP_CACHE23  8'Hb7
 `define OP_LWL      8'Hb8
 `define OP_LWR      8'Hb9
 `define OP_SWL      8'Hba
 `define OP_SWR      8'Hbb
 `define OP_CACHE28  8'Hbc
 `define OP_CACHE29  8'Hbd
 `define OP_CACHE30  8'Hbe
 `define OP_CACHE31  8'Hbf
 
 /*Floating point operation*/
 `define OP_FADD     8'H40
 `define OP_FSUB     8'H41
 `define OP_FMUL     8'H42
 `define OP_FDIV     8'H43
 `define OP_FSQRT    8'H44
 `define OP_FABS     8'H45
 `define OP_FMOV     8'H46
 `define OP_FNEG     8'H47
 `define OP_ROUNDL   8'H48
 `define OP_TRUNCL   8'H49
 `define OP_CEILL    8'H4a
 `define OP_FLOORL   8'H4b
 `define OP_ROUNDW   8'H4c
 `define OP_TRUNCW   8'H4d
 `define OP_CEILW    8'H4e
 `define OP_FLOORW   8'H4f
 `define OP_RECIP    8'H5c
 `define OP_RSQRT    8'H5d
 `define OP_CVTS     8'H60
 `define OP_CVTD     8'H61
 `define OP_CVTW     8'H64
 `define OP_CVTL     8'H65
 `define OP_MOVF     8'H66
 `define OP_MOVT     8'H67
 `define OP_BC1F     8'H68
 `define OP_BC1T     8'H69
 `define OP_BC1FL    8'H6a
 `define OP_BC1TL    8'H6b
 `define OP_FMOVF    8'H6c
 `define OP_FMOVT    8'H6d
 `define OP_FMOVZ    8'H6e
 `define OP_FMOVN    8'H6f
 `define OP_CF       8'H70
 `define OP_CUN      8'H71
 `define OP_CEQ      8'H72
 `define OP_CUEQ     8'H73
 `define OP_COLT     8'H74
 `define OP_CULT     8'H75
 `define OP_COLE     8'H76
 `define OP_CULE     8'H77
 `define OP_CSF      8'H78
 `define OP_CNGLE    8'H79
 `define OP_CSEQ     8'H7a
 `define OP_CNGL     8'H7b
 `define OP_CLT      8'H7c
 `define OP_CNGE     8'H7d
 `define OP_CLE      8'H7e
 `define OP_CNGT     8'H7f

 `define OP_ADDQ               8'Hc0 //include addu.qb, addu.ph
 `define OP_ADDQ_S             8'Hc1 //include addu_s.qb, addq_s.ph, addq_s.w
 `define OP_SUBQ               8'Hc2 //include subu.qb, subq.ph
 `define OP_SUBQ_S             8'Hc3 //include subu_s.qb, subq_s.ph, subq_s.w
 `define OP_ABSQ_S             8'Hc4 //include absq_s.ph, absq_s.w
 `define OP_RADDU              8'Hc5 //adduu.w.qb
 `define OP_ADDSC              8'Hc6 
 `define OP_ADDWC              8'Hc7
 `define OP_PRECRQ_QB_PH       8'Hc8
 `define OP_PRECRQ_PH_W        8'Hc9
 `define OP_PRECRQ_RS_PH_W     8'Hca
 `define OP_PRECRQU_S_QB_PH    8'Hcb
 `define OP_PRECEQ_W_PHL       8'Hcc
 `define OP_PRECEQ_W_PHR       8'Hcd
 `define OP_PACKRL_PH          8'Hce
 `define OP_MODSUB             8'Hcf
 `define OP_PRECEQU_PH_QBL     8'Hd0 
 `define OP_PRECEQU_PH_QBR     8'Hd1 
 `define OP_PRECEQU_PH_QBLA    8'Hd2 
 `define OP_PRECEQU_PH_QBRA    8'Hd3 
 `define OP_PRECEU_PH_QBL      8'Hd4
 `define OP_PRECEU_PH_QBR      8'Hd5
 `define OP_PRECEU_PH_QBLA     8'Hd6 
 `define OP_PRECEU_PH_QBRA     8'Hd7
 `define OP_SHLLV               8'Hd8 //include SLL(V).QB,SLL(V).PH
 `define OP_SHLLVS              8'Hd9 //include SLL(V)_S.W,SLL(V)_S.PH
 `define OP_SHRAV              8'Hda //include shrl(v).qb, shra(v).ph
 `define OP_SHRAVR             8'Hdb //inclue shra(v)_R.ph, shra(v).w
 `define OP_CMP_EQ             8'Hdc //include cmp(g)u.eq.qb, cmp.eq.ph
 `define OP_CMP_LT             8'Hdd //include cm(g)u.lt.qb, cmp.lt.ph
 `define OP_CMP_LE             8'Hde //include cmp(g)u.le.gb, cm.le.ph
 `define OP_PICK               8'Hdf //include pick.ph, pick.qb
 `define OP_MULEU_S_PH_QBL     8'He0 
 `define OP_MULEU_S_PH_QBR     8'He1 
 `define OP_MULQ_RS_PH         8'He2 
 `define OP_MULEQ_S_W_PHL      8'He3 
 `define OP_MULEQ_S_W_PHR      8'He4 
 `define OP_BITREV             8'He5
 `define OP_INSV               8'He6
 `define OP_REPL               8'He7 //include repl(v).ph ,repl(v).qb
 `define OP_MAQ_S_W_PHL        8'He8
 `define OP_MAQ_S_W_PHR        8'He9
 `define OP_MAQ_SA_W_PHL       8'Hea
 `define OP_MAQ_SA_W_PHR       8'Heb
 `define OP_MULSAQ_S_W_PH      8'Hec
 `define OP_EXTP               8'Hed //include extp, extpv
 `define OP_EXTPDP             8'Hee //include extpdp, extpdpv
 `define OP_SHILO              8'Hef //include shilo, shilov
 `define OP_DPAU_H_QBL         8'Hf0 
 `define OP_DPAU_H_QBR         8'Hf1 
 `define OP_DPAQ_S_W_PH        8'Hf2
 `define OP_DPAQ_SA_L_W        8'Hf3
 `define OP_DPSU_H_QBL         8'Hf4
 `define OP_DPSU_H_QBR         8'Hf5
 `define OP_DPSQ_S_W_PH        8'Hf6
 `define OP_DPSQ_SA_L_W        8'Hf7
 `define OP_EXTR_W             8'Hf8
 `define OP_EXTR_R_W           8'Hf9
 `define OP_EXTR_RS_W          8'Hfa
 `define OP_EXTR_S_H           8'Hfb
 `define OP_MTHLIP             8'Hfc
 `define OP_BPOSGE32          8'Hfd
 `define OP_RDDSP              8'Hfe
 `define OP_WRDSP              8'Hff

 /*Cache state*/
 `define DCACHE_INV      0
 `define DCACHE_SHARED   1
 `define DCACHE_EXC      2
 `define DCACHE_DIRTY    3
 `define ICACHE_INV      0
 `define ICACHE_VALID    1
 
`endif
