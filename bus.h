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

`ifdef GODSON_BUS_H
`else
`define GODSON_BUS_H

`include "global.h"

`define Lword                32
`define Ldword               64
`define Lirbus              119 
`define Lirbus_2issue       238 
`define Ldecbus             145
`define Ldecbus_2issue      290 
`define Lqissuebus          118
`define Lqissuebus0_to_gr   125
`define Lqissuebus1_to_gr   102
`define Lrissuebus0         123 //299 //331
`define Lrissuebus1         100//299 //331
`define Lrissuebus_to_fix	 82//89
`define Lrissuebus_to_float	176
`define Lrissuebus_to_mm	137
`define Lresbus             125 
`define Lresbus3            123 
`define Lresbus1            122 
`define Lresbus2            92 
`define Lresbus0            99 
`define Lcommitbus          176
`define Lcommitbus_random   240

`define Lcommitbus_to_fetch 36 

`define Lcommitbus_to_gr    85 
`define Lcommitbus_to_gr_random    149 
`define Lcommitbus_to_fr    101
`define Lcommitbus_to_tlb    83
`define Lcommitbus_to_hb     51
`define Lcommitbus_to_itlb   9 
`define Liaddrbus            34
`define Liresbus             39
`define Liresbus_2issue      114//90//88//82//78

`define Ldaddrbus           109
`define Ldresbus             94
`define Ldcachepaddr        123
`define Licachepaddr        40 
`define Lhb_reqbus           79
`define Ldcr_reqbus          79
`define Ldmseg_dreqbus       79
`define Ldmseg_ireqbus       35
`define Lhb_dcompbus         121
`define Lhb_icompbus         41

`define Lmemwaddr           304 

`define Lmemraddr            40

`define Lmemres              46 

`define Lmemaddr            294 
`define Lmmrs               179
`define Laddr_to_tlb        118
`define Laddr_to_dcache     25
`define Lmoveresult         70
`define Litlb_req           33
`define Ltlb_to_itlb        37
`define Ltlb_to_icache      82
`define Ldcache_result      71
`define Ltlb_read_again     23
`define Ltlb_to_memq        149
`define Ltlb_forward        9
`define Lmemq_to_dcache     339
`define Lreplace_dump       280

`define Ltlb_to_ram         66
`define Lram_to_tlb         52

`define Licache_to_ram       1444 

`define Ldcache_to_ram      1404

`define Lram_to_icache     1152

`define Lram_to_dcache      1112

`endif
