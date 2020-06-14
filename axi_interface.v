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
`include "reg.h"
`include "bus.h"

module axi_interface (
    aclk,
    areset_n,
    
    arid,
    araddr,
    arlen,
    arsize,
    arburst,
    arlock,
    arcache,
    arprot,
    arvalid,
    arready,

    rid,
    rdata,
    rresp,
    rlast,
    rvalid,
    rready,

    awid,
    awaddr,
    awlen,
    awsize,
    awburst,
    awlock,
    awcache,
    awprot,
    awvalid,
    awready,

    wid,
    wdata,
    wstrb,
    wlast,
    wvalid,
    wready,

    bid,
    bresp,
    bvalid,
    bready,

    reset_o,
    
    softreset_i,
    softreset_o,
    
    nmi_i,
    nmi_o,
    
    int_i,
    int_o,
    
    filter_window_result_i,
    filter_window_req_o,

    inst_memraddr_i,
    inst_memres_o,
    
    cp0_mem_req_i,
    cp0_memraddr_i,
    cp0_memwaddr_i,
    cp0_memres_o
);
//burst type
parameter BURST_FIXED   = 2'b00;
parameter BURST_INCR    = 2'b01;
parameter BURST_WRAP    = 2'b10;

//response type
parameter RESP_OKAY     = 2'b00;
parameter RESP_EXOKAY   = 2'b01;
parameter RESP_SLVERR   = 2'b10;
parameter RESP_DECERR   = 2'b11;

//burst length
parameter LEN_1         = 4'h0;
parameter LEN_2         = 4'h1;
parameter LEN_4         = 4'h3;
parameter LEN_8         = 4'h7;

//burst size
parameter SIZE_1B       = 3'b000;
parameter SIZE_2B       = 3'b001;
parameter SIZE_4B       = 3'b010;
parameter SIZE_8B       = 3'b011;
parameter SIZE_16B      = 3'b100;

//width type
parameter WIDTH_BLOCK   = 4'b0001;
parameter WIDTH_64BIT   = 4'b1111;
parameter WIDTH_32BIT   = 4'b1011;
parameter WIDTH_24BIT   = 4'b1010;
parameter WIDTH_16BIT   = 4'b1001;
parameter WIDTH_8BIT    = 4'b1000;
parameter WIDTH_FULL    = 4'b0000; 

//global
input   aclk;
input   areset_n;

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

reg  [2:0]   arid_r;
reg  [31:0]  araddr;
reg  [3:0]   arlen;
reg  [2:0]   arsize;
reg  [1:0]   arburst;
wire [1:0]   arlock;
wire [3:0]   arcache;
wire [2:0]   arprot;
reg          arprot_2_r;
reg          arvalid;

assign arid    = {1'b0, arid_r};
assign arlock  = 2'b00; //normal access
assign arcache = 4'h0;  //noncacheable and nonbufferable
assign arprot[0] = 1'b0; //normal access
assign arprot[1] = 1'b1; //nonsesure access
assign arprot[2] = arprot_2_r;

//read data channel
input   [3:0]   rid;
input   [31:0]  rdata;
input   [1:0]   rresp;
input           rlast;
input           rvalid;
output          rready;

wire            rready;
assign rready = 1'b1;

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

reg  [2:0]   awid_r;
reg  [31:0]  awaddr;
reg  [3:0]   awlen;
reg  [2:0]   awsize;
reg  [1:0]   awburst;
wire [1:0]   awlock;
wire [3:0]   awcache;
wire [2:0]   awprot;
reg          awvalid;

assign awid = {1'b0, awid_r};
assign awlock  = 2'b00; //normal access
assign awcache = 4'h0;  //noncacheable and nonbufferable
assign awprot[0] = 1'b0; //normal access
assign awprot[1] = 1'b1; //nonsesure access
assign awprot[2] = 1'b1; //data access

//write data channel
input           wready;
output  [3:0]   wid;
output  [31:0]  wdata;
output  [3:0]   wstrb;
output          wlast;
output          wvalid;

reg [31:0]  wdata;
reg [3:0]   wstrb;
reg         wlast;
reg         wvalid;
assign wid = awid;

//write response channel
input   [3:0]   bid;
input   [1:0]   bresp;
input           bvalid;
output          bready;

wire            bready;
assign bready = 1'b1;

//reset, softreset, nmi and ex_int for core
output          reset_o;
assign reset_o = ~areset_n;

input           softreset_i;
output          softreset_o;
assign softreset_o = reset_o;

input           nmi_i;
output          nmi_o;
assign nmi_o = ~nmi_i;

input   [4:0]   int_i;
output  [5:0]   int_o;
assign int_o = {1'b0, ~int_i};

input           filter_window_result_i;
output  [31:0]  filter_window_req_o;

input   [`Lmemraddr-1:0] inst_memraddr_i;
output  [`Lmemres-1:0]   inst_memres_o;

input                    cp0_mem_req_i;
input   [`Lmemraddr-1:0] cp0_memraddr_i;
input   [`Lmemwaddr-1:0] cp0_memwaddr_i;
output  [`Lmemres-1:0]   cp0_memres_o;

wire [`Lmemraddr-1:0] mem_rd_req = (cp0_mem_req_i) ? cp0_memraddr_i : inst_memraddr_i;
wire [2:0]  rd_id    = mem_rd_req[39:37];
wire        rd_valid = mem_rd_req[36];
wire [3:0]  rd_width = mem_rd_req[35:32];
wire [31:0] rd_addr  = mem_rd_req[31:0];

wire [3:0] wr_acc_ben = cp0_memwaddr_i[`Lmemwaddr-1:300];

wire [255:0]wr_data  = cp0_memwaddr_i[299:44];
wire [3:0]  wr_ben   = cp0_memwaddr_i[43:40];
wire [2:0]  wr_id    = cp0_memwaddr_i[39:37];
wire        wr_valid = cp0_memwaddr_i[36];
wire [3:0]  wr_width = cp0_memwaddr_i[35:32];
wire [31:0] wr_addr  = cp0_memwaddr_i[31:0];

reg         filter_out_flag_r;
reg  [2:0]  filter_out_id_r;
wire [2:0]  filter_out_cnt;
wire [2:0]  filter_out_index;
reg  [2:0]  filter_out_cnt_r;
assign filter_out_cnt = filter_out_cnt_r;
reg  [2:0]  filter_out_index_r;
assign filter_out_index = filter_out_index_r;
reg         filter_out_isdw_r;

wire [`Lmemres-1:0]  memres;
wire        memres_valid;
wire        memres_unc_acc_rdy;
wire [2:0]  memres_id;
wire [2:0]  memres_count;
wire        memres_rd_rdy;
wire        memres_wr_rdy;
wire        memres_uncache_rdy;
wire        memres_unc_acc_clean;
wire        memres_is_dw;
wire [1:0]  memres_wtq_valid;
wire [31:0] memres_data;

assign memres[0]     = memres_valid | filter_out_flag_r;
assign memres[1]     = memres_unc_acc_rdy;
assign memres[4:2]   = memres_valid ? memres_id : filter_out_id_r;
assign memres[7:5]   = memres_valid ? memres_count : filter_out_index;
assign memres[8]     = memres_rd_rdy;
assign memres[9]     = memres_wr_rdy;
assign memres[10]    = memres_uncache_rdy;
assign memres[11]    = memres_valid ? memres_is_dw : filter_out_isdw_r;
assign memres[43:12] = memres_data;
assign memres[45:44] = memres_wtq_valid;

assign cp0_memres_o  = memres;
assign inst_memres_o = memres;

reg   [3:0]   rid_r;
reg   [31:0]  rdata_r;
reg   [1:0]   rresp_r;
reg           rlast_r;
reg           rvalid_r;

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        rid_r   <= 4'b0;
        rresp_r <= RESP_OKAY;
        rlast_r <= 1'b0;
        rvalid_r<= 1'b0;
    end
    else
    begin
        rvalid_r<= rvalid & ~rid[3];
        if (rvalid)
        begin
            rid_r   <= rid;
            rresp_r <= rresp; 
            rlast_r <= rlast; 
            rdata_r <= rdata;
        end
    end
end

wire noncache_wr_valid;

//transaction of write data channel is cache block write
reg         cache_wr_r;
reg  [ 1:0] wtq_valid_r;
reg  [26:0] wtq_addr_r[1:0];


//read address channel
reg rd_buf_valid_r;

wire cache_rd = rd_width==WIDTH_BLOCK;
wire [31:0] araddr_value = {rd_addr[31:2], ((cache_rd) ? 2'b00 : rd_addr[1:0])};
wire [3:0] arlen_value   = (cache_rd) ? LEN_8 :
                           (rd_width==WIDTH_64BIT) ? LEN_2 : LEN_1;
wire [2:0] arsize_value  = (rd_width==WIDTH_8BIT)  ? SIZE_1B : 
                           (rd_width==WIDTH_16BIT) ? SIZE_2B : SIZE_4B;
wire [1:0] arburst_value = (cache_rd) ? BURST_WRAP : BURST_INCR;

//assign memres_rd_rdy = ~rd_buf_valid_r | (arvalid & arready); 
assign memres_rd_rdy = ~rd_buf_valid_r; 
wire new_rd_in = memres_rd_rdy & rd_valid & (cache_rd | memres_uncache_rdy&~noncache_wr_valid);


assign filter_window_req_o = rd_addr;
wire mem_duncache_rd = rd_id == 3'b110;
wire mem_req_cancel  = ~mem_duncache_rd & filter_window_result_i;

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        filter_out_flag_r <= 1'b0;
        filter_out_id_r   <= 3'b0;
        filter_out_isdw_r <= 1'b0;
    end
    else if (new_rd_in & mem_req_cancel)
    begin
        filter_out_flag_r <= 1'b1;
        filter_out_id_r   <= rd_id;
        filter_out_isdw_r <= rd_width[3]&rd_width[2]; //rd_width==WIDTH_64BIT
    end
    else if (filter_out_flag_r & ~memres_valid & filter_out_cnt==3'b000)
    begin
        filter_out_flag_r <= 1'b0;
    end

    if (new_rd_in & mem_req_cancel)
    begin
        filter_out_cnt_r <= ~rd_width[3] ? 3'b111 : 
                             rd_width[2] ? 3'b001 : 3'b000;
    end
    else if (filter_out_flag_r & ~memres_valid & filter_out_cnt!=3'b000)
    begin
        filter_out_cnt_r <= filter_out_cnt_r - 1'b1;
    end

    if (~areset_n)
    begin
        filter_out_index_r <= 3'b000;
    end
    else if (new_rd_in & mem_req_cancel)
    begin
        filter_out_index_r <= rd_addr[4:2];
    end
    else if (filter_out_flag_r & ~memres_valid & filter_out_cnt!=3'b000)
    begin
        filter_out_index_r <= (filter_out_index_r==3'b111) ? 3'b0 : filter_out_index_r+1'b1;
    end
end

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        rd_buf_valid_r <= 1'b0;
        arid_r  <= 3'b000;
        araddr  <= 32'h0;
        arlen   <= 4'h0;
        arsize  <= 3'b000;
        arburst <= 2'b00;
        arprot_2_r <= 1'b1;
    end
    else
    begin
        if (new_rd_in)
        begin
            rd_buf_valid_r <= 1'b1;
            arid_r  <= rd_id;
            araddr  <= araddr_value;
            arlen   <= arlen_value;
            arsize  <= arsize_value;
            arburst <= arburst_value;
            arprot_2_r <= ~cp0_mem_req_i;
        end
        else if ((~filter_out_flag_r & arvalid & arready) |
                 ( filter_out_flag_r & ~memres_valid & filter_out_cnt==3'b000))
        begin
            rd_buf_valid_r <= 1'b0;
        end
    end
end

wire [26:0] select_addr = (memres_rd_rdy) ? rd_addr[31:5] : araddr[31:5];
wire cache_rd_wait = (wtq_valid_r[0] & (select_addr==wtq_addr_r[0])) |
                     (wtq_valid_r[1] & (select_addr==wtq_addr_r[1])) ;

always @(posedge aclk)
begin
    if (~areset_n)
        arvalid <= 1'b0;
    else if (new_rd_in & ~(cache_rd & cache_rd_wait) & ~mem_req_cancel)
        arvalid <= 1'b1;
    else if (rd_buf_valid_r & ~arvalid & ~cache_rd_wait & ~filter_out_flag_r)
        arvalid <= 1'b1;
    else if (arvalid & arready)
        arvalid <= 1'b0;
end

//write address channel & write data channel
wire cache_wr   = wr_id==3'b100 || wr_id==3'b101;
wire uncache_wr = wr_id==3'b110;
wire unc_acc_wr = wr_id==3'b111;
wire block_wr   = wr_width==WIDTH_BLOCK;
wire last_wr;

reg  [223:0] data_buf_r;
reg  [2:0] data_cnt_r;
wire [2:0] data_select = ~data_cnt_r; // 3'h7-data_cnt_r

wire [31:0] awaddr_value = {wr_addr[31:5], ((block_wr) ? 5'h00 : wr_addr[4:0])}; 
wire [3:0] awlen_value   = (block_wr) ? LEN_8 :
                           (wr_width==WIDTH_64BIT) ? LEN_2 : LEN_1;
wire [2:0] awsize_value  = (wr_width==WIDTH_8BIT)  ? SIZE_1B :
                           (wr_width==WIDTH_16BIT) ? SIZE_2B : SIZE_4B;   
wire [1:0] awburst_value = (block_wr) ? BURST_WRAP : BURST_INCR;

wire   last_wr_ok = wvalid & wlast & wready;
assign memres_wr_rdy = ~awvalid & ~wvalid;
wire   new_wr_in = memres_wr_rdy & wr_valid & ((cache_wr   & ~wtq_valid_r[wr_id[0]]) | 
                                               (uncache_wr & memres_uncache_rdy)     |
                                               (unc_acc_wr & memres_unc_acc_rdy)     );

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        awvalid <= 1'b0;
        awid_r  <= 3'b000;
        awaddr  <= 32'h0;
        awlen   <= 4'h0;
        awsize  <= 3'b000;
        awburst <= 2'b00;
    end
    else
    begin
        if (new_wr_in)
        begin
            awvalid <= 1'b1;
            awid_r  <= wr_id;
            awaddr  <= awaddr_value;
            awlen   <= awlen_value;
            awsize  <= awsize_value;
            awburst <= awburst_value;
        end
        else if (awvalid & awready)
        begin
            awvalid <= 1'b0;
        end
    end
end

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        wvalid     <= 1'b0;
        cache_wr_r <= 1'b0;
    end
    else if (new_wr_in)
    begin
        wvalid     <= 1'b1;
        cache_wr_r <= cache_wr;
    end
    else
    begin
        if (last_wr_ok)
            wvalid <= 1'b0;
    end
end

always @(posedge aclk) 
begin
    if (~areset_n)
    begin
        wtq_valid_r <= 2'b00;
    end
    else begin
        if (new_wr_in && (wr_id==3'b100)) begin
            wtq_valid_r[0] <= 1'b1;
            wtq_addr_r[0]  <= awaddr_value[31:5];
        end
        else if (bvalid && bready && (bid==4'b0100))
            wtq_valid_r[0] <= 1'b0;

        if (new_wr_in && (wr_id==3'b101)) begin
            wtq_valid_r[1] <= 1'b1;
            wtq_addr_r[1]  <= awaddr_value[31:5];
        end
        else if (bvalid && bready && (bid==4'b0101))
            wtq_valid_r[1] <= 1'b0;

    end
end

assign memres_wtq_valid = wtq_valid_r;

wire [3:0] wstrb_value = (wr_width==WIDTH_BLOCK || wr_width==WIDTH_64BIT || wr_width==WIDTH_32BIT) ? 4'b1111 : 
                         (wr_width==WIDTH_FULL) ? wr_acc_ben : wr_ben;
assign last_wr = data_cnt_r==3'b001;

always @(posedge aclk)
begin
    if (~areset_n)
        data_cnt_r <= 3'b000;
    else if (new_wr_in)
        data_cnt_r <= awlen_value[2:0];
    else if (wvalid & wready & ~wlast)
        data_cnt_r <= data_cnt_r - 1'b1;
end

always @(posedge aclk)
begin
    if (~areset_n)
        wlast <= 1'b0;
    else if (new_wr_in)
        wlast <= ~(wr_width==WIDTH_BLOCK || wr_width==WIDTH_64BIT);
    else if (wvalid & wready & ~wlast & last_wr)
        wlast <= 1'b1;
    else if (last_wr_ok)
        wlast <= 1'b0;
end

always @(posedge aclk)
begin
    if (~areset_n)
        wstrb <= 4'h0;
    else if (new_wr_in)
        wstrb <= wstrb_value;
end

wire [31:0] new_wdata  = ({32{data_select==3'b000}}&data_buf_r[ 31:  0]) |
                         ({32{data_select==3'b001}}&data_buf_r[ 63: 32]) |
                         ({32{data_select==3'b010}}&data_buf_r[ 95: 64]) |
                         ({32{data_select==3'b011}}&data_buf_r[127: 96]) |
                         ({32{data_select==3'b100}}&data_buf_r[159:128]) |
                         ({32{data_select==3'b101}}&data_buf_r[191:160]) |
                         ({32{data_select==3'b110}}&data_buf_r[223:192]);

always @(posedge aclk)
begin
    if (new_wr_in)
    begin
        wdata <= wr_data[31:0];
        data_buf_r <= wr_data[255:32];
    end
    else if (wvalid & wready & ~wlast)
    begin
        wdata <= new_wdata;
    end
end

//uncache record
reg     uncache_valid_r;
reg     uncache_wr_r; //1--write, 0--read
reg     uncache_id_r; //the [0] bit of id

wire uncache_complete = uncache_valid_r & ((uncache_wr_r  & bvalid   & (bid==4'b0110)        ) | 
                                           (~uncache_wr_r & rvalid_r & (rid_r=={3'b011, uncache_id_r}) & rlast_r));
assign memres_uncache_rdy = ~uncache_valid_r & memres_unc_acc_clean;
wire new_uncache_in = memres_uncache_rdy & (wr_valid & uncache_wr & memres_wr_rdy |
                                            rd_valid & ~cache_rd  & memres_rd_rdy & ~noncache_wr_valid);
assign noncache_wr_valid = wr_valid & ~cache_wr;

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        uncache_valid_r <= 1'b0;
        uncache_wr_r    <= 1'b0;
        uncache_id_r    <= 1'b0;
    end
    else if (new_uncache_in && !(mem_req_cancel && rd_valid))
    begin
        uncache_valid_r <= 1'b1;
        uncache_wr_r    <= wr_valid & uncache_wr; //wr_valid imply memres_wr_rdy=1
        uncache_id_r    <= rd_id[0];
    end
    else if (uncache_complete)
    begin
        uncache_valid_r <= 1'b0;
    end
end

//uncache acc record
reg  [4:0] unc_acc_cnt_r;

wire new_acc_in = wr_valid && unc_acc_wr && memres_wr_rdy && memres_unc_acc_rdy;
wire acc_b_resp = bvalid && bready && bid==4'b0111;

always @(posedge aclk)
begin
    if (~areset_n)
        unc_acc_cnt_r <= 5'h0;
    else if (new_acc_in && ~acc_b_resp)
        unc_acc_cnt_r <= unc_acc_cnt_r + 1'b1;
    else if (~new_acc_in && acc_b_resp)
        unc_acc_cnt_r <= unc_acc_cnt_r - 1'b1;
end

assign memres_unc_acc_rdy   = unc_acc_cnt_r!=5'b10000;
assign memres_unc_acc_clean = unc_acc_cnt_r==5'b00000;

//memres
wire rid_0 = rid_r==4'b0000;
wire rid_1 = rid_r==4'b0001;
wire rid_2 = rid_r==4'b0010;
wire rid_3 = rid_r==4'b0011;

assign memres_valid  = rvalid_r & rready;
assign memres_id     = rid_r[2:0];
assign memres_data   = rdata_r;

reg [2:0] rd_cnt_0;
reg [2:0] rd_cnt_1;
reg [2:0] rd_cnt_2;
reg [2:0] rd_cnt_3;
reg       has_dw_rd_r;

always @(posedge aclk)
begin
    if (~areset_n)
    begin
        rd_cnt_0 <= 3'h0;
        rd_cnt_1 <= 3'h0;
        rd_cnt_2 <= 3'h0;
        rd_cnt_3 <= 3'h0;
        has_dw_rd_r <= 1'b0;
    end
    else
    begin
        if (new_rd_in & (rd_id==3'b000))
            rd_cnt_0 <= rd_addr[4:2];
        else if (rvalid_r & rready & ~rlast_r & rid_0)
            rd_cnt_0 <= (rd_cnt_0==3'b111) ? 3'b000 : rd_cnt_0 + 1'b1;

        if (new_rd_in & (rd_id==3'b001))
            rd_cnt_1 <= rd_addr[4:2];
        else if (rvalid_r & rready & ~rlast_r & rid_1)
            rd_cnt_1 <= (rd_cnt_1==3'b111) ? 3'b000 : rd_cnt_1 + 1'b1;

        if (new_rd_in & (rd_id==3'b010))
            rd_cnt_2 <= rd_addr[4:2];
        else if (rvalid_r & rready & ~rlast_r & rid_2)
            rd_cnt_2 <= (rd_cnt_2==3'b111) ? 3'b000 : rd_cnt_2 + 1'b1;

        if (new_rd_in & (rd_id==3'b011))
            rd_cnt_3 <= rd_addr[4:2];
        else if (rvalid_r & rready & ~rlast_r & rid_3)
            rd_cnt_3 <= (rd_cnt_3==3'b111) ? 3'b000 : rd_cnt_3 + 1'b1;

        if (new_rd_in & (rd_id[2:1]==2'b11) & (rd_width==WIDTH_64BIT))
            has_dw_rd_r <= 1'b1;
        else if (rvalid_r & rready & rlast_r & (rid_r[3:1]==3'b011))
            has_dw_rd_r <= 1'b0;
    end
end

assign memres_count = {3{rid_0}}&rd_cnt_0 |
                      {3{rid_1}}&rd_cnt_1 |
                      {3{rid_2}}&rd_cnt_2 |
                      {3{rid_3}}&rd_cnt_3 ;
assign memres_is_dw = (rid_r[3:1]==3'b011) & has_dw_rd_r;



endmodule
