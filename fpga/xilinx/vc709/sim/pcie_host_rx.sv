//-----------------------------------------------------------------------------
//
// (c) Copyright 2012-2012 Xilinx, Inc. All rights reserved.
//
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
//
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
//
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
//
//-----------------------------------------------------------------------------
//
// Project    : Virtex-7 FPGA Gen3 Integrated Block for PCI Express
// File       : pci_exp_usrapp_rx.v
// Version    : 4.2
//--
//--------------------------------------------------------------------------------

`include "board_common.vh"

`define EXPECT_FINISH_CHECK board.RP.tx_usrapp.expect_finish_check
module pcie_host_rx #( parameter C_DATA_WIDTH = 64,
                        parameter AXISTEN_IF_RQ_ALIGNMENT_MODE   = "FALSE",
                        parameter AXISTEN_IF_CC_ALIGNMENT_MODE   = "FALSE",
                        parameter AXISTEN_IF_RQ_ALIGNMENT_MODE   = "FALSE",
                        parameter AXISTEN_IF_CC_ALIGNMENT_MODE   = "FALSE",
                        parameter STRB_WIDTH   = C_DATA_WIDTH / 8, // TSTRB width
                        parameter KEEP_WIDTH   = C_DATA_WIDTH / 32,
                        parameter PARITY_WIDTH = C_DATA_WIDTH / 8)  // TPARITY width 
(
   input                            user_clk,
   input                            user_reset,
   input                            user_lnk_up,

   // 接收请求（在原始的ex design中接收来自rp_model的请求，rp_model作为host（完成者））
   // 把本模块作为host时，该接口用作接收请求，接收来自FPGA device的请求，此时该接口和FPGA device的rq相连
   input      [C_DATA_WIDTH-1:0]    s_axis_rq_tdata,
   input                            s_axis_rq_tlast,
   input                            s_axis_rq_tvalid,
   input                  [84:0]    s_axis_rq_tuser,
   input        [KEEP_WIDTH-1:0]    s_axis_rq_tkeep,
   output reg                       s_axis_rq_tready,
   // 接收cmplD（在原始的ex design中接收来自rp_model的comletion data，rp_model作为host（回复请求者FPGA的完成消息））
   // 把本模块作为host时，该接口用作接收完成消息，接收来自FPGA device的完成消息，此时该接口和FPGA device的cc相连
   input      [C_DATA_WIDTH-1:0]    s_axis_cc_tdata,
   input                            s_axis_cc_tlast,
   input                            s_axis_cc_tvalid,
   input                  [74:0]    s_axis_cc_tuser,
   input        [KEEP_WIDTH-1:0]    s_axis_cc_tkeep,
   output reg                       s_axis_cc_tready,

   input                   [5:0]    pcie_rq_np_req_count,
   output reg                       pcie_rq_np_req
);
   parameter   Tcq = 1;
   /* Local variables */
   reg   [31:0]      rx_file_ptr;
   reg   [7:0]       frame_store_rx[5119:0];
   integer           frame_store_rx_idx;

   reg  [11:0]       byte_count_fbe = 12'b0;
   reg  [11:0]       byte_count_lbe = 12'b0;
   reg [11:0]        byte_count = 12'b0;
   reg  [06:0]       lower_addr = 7'b0;

   reg               req_compl_wd;
   reg               req_compl = 1'b0;
   reg               req_compl_ur = 1'b0;
   reg               req_compl_q = 1'b0;
   reg               req_compl_qq = 1'b0;
   reg               req_compl_wd_q = 1'b0;
   reg               req_compl_wd_qq = 1'b0;
   reg               req_compl_ur_q = 1'b0;
   reg               req_compl_ur_qq = 1'b0;

   reg     [4:0]     rq_rx_state = 5'b0;
   reg     [4:0]     cc_rx_state = 5'b0;
   reg     [63:0]    rq_data = 64'b0;
   reg     [63:0]    cc_data = 64'b0;
   reg     [7:0]     rq_be = 8'b0;
   reg     [7:0]     cc_be = 8'b0;
   reg     [31:0]    next_rq_rx_timeout = 32'b0;
   reg               rq_beat0_valid = 1'b0;
   reg               cc_beat0_valid = 1'b0;
   reg     [7:0]     ii = 8'b0;
   wire              user_reset_n;

   assign user_reset_n  = ~user_reset;

   reg [7:0] tkeep;
   reg [7:0] tkeep_q;
   reg [7:0] tkeep_qq;

   always @  (s_axis_cc_tdata[4:2]) begin
      casex (s_axis_cc_tdata[4:2])
         3'b000 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'h1 :16'h1; 
         3'b001 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'h3 :16'h1; 
         3'b010 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'h7 :16'h1; 
         3'b011 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'hf :16'h1; 
         3'b100 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'h1f :16'h1; 
         3'b101 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'h3f :16'h1; 
         3'b110 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'h7f :16'h1; 
         3'b111 : tkeep = (AXISTEN_IF_CC_ALIGNMENT_MODE == "TRUE" ) ? 16'hff :16'h1; 
      endcase
   end

   always @(posedge user_clk or negedge user_reset_n) begin
      if (user_reset_n == 1'b0) begin
         tkeep_q <= 8'h0;
         tkeep_qq <= 8'h0;
      end else begin 
         tkeep_q <= tkeep;
         tkeep_qq <= tkeep_q;
      end
   end

   /* State variables */
   `define           RQ_RX_RESET    5'b00001
   `define           RQ_RX_DOWN     5'b00010
   `define           RQ_RX_IDLE     5'b00100
   `define           RQ_RX_ACTIVE   5'b01000
   `define           RQ_RX_SRC_DSC  5'b10000

   `define           CC_RX_RESET    5'b00001
   `define           CC_RX_DOWN     5'b00010
   `define           CC_RX_IDLE     5'b00100
   `define           CC_RX_ACTIVE   5'b01000
   `define           CC_RX_SRC_DSC  5'b10000

   /* Transaction Receive User Interface State Machine */
   always @(posedge user_clk or negedge user_reset_n) begin
      if (user_reset_n == 1'b0) begin
         rq_rx_state     <= #(Tcq)  `RQ_RX_RESET;
      end
      else begin
         case (rq_rx_state)
            `RQ_RX_RESET :  begin
               if (user_reset_n == 1'b0)
                  rq_rx_state <= #(Tcq) `RQ_RX_RESET;
               else
                  rq_rx_state <= #(Tcq) `RQ_RX_DOWN;
            end

            `RQ_RX_DOWN : begin
               if (user_lnk_up == 1'b0)
                  rq_rx_state <= #(Tcq) `RQ_RX_DOWN;
               else begin
                  rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
               end
            end

            `RQ_RX_IDLE : begin
               if (user_reset_n == 1'b0)
                  rq_rx_state <= #(Tcq) `RQ_RX_RESET;
               else if (user_lnk_up == 1'b0)
                  rq_rx_state <= #(Tcq) `RQ_RX_DOWN;
               else begin
                  if (  (s_axis_rq_tuser[40] == 1'b1) && (s_axis_rq_tvalid == 1'b1) && (s_axis_rq_tready == 1'b1)  ) begin
                     rq_data         =  s_axis_rq_tdata;
                     rq_be           = s_axis_rq_tuser[7:0];
                     rq_beat0_valid  = s_axis_rq_tuser[40];

                     if(C_DATA_WIDTH==64)begin
                        rq_rx_state <= #(Tcq) `RQ_RX_ACTIVE;
                     end
                     else if(C_DATA_WIDTH==128)begin
                        if(s_axis_rq_tlast == 1'b1) begin
                           TSK_BUILD_RQ_TO_PCIE_PKT(s_axis_rq_tdata[63:0],rq_be,s_axis_rq_tdata[C_DATA_WIDTH-1:C_DATA_WIDTH/2]);
                           TSK_PARSE_FRAME(`RX_LOG);
                           rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
                        end
                     end
                     else if(C_DATA_WIDTH==256)begin
                        if(s_axis_rq_tlast == 1'b1) begin
                              TSK_BUILD_RQ_TO_PCIE_PKT(s_axis_rq_tdata[63:0],rq_be,s_axis_rq_tdata[(C_DATA_WIDTH/2)-1:C_DATA_WIDTH/4]);
                              for(ii=4; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                                 if(s_axis_rq_tkeep[ii] == 1'b1 ||s_axis_rq_tkeep[ii+1] == 1'b1 )
                                    TSK_READ_DATA(~(s_axis_rq_tkeep[ii] & s_axis_rq_tkeep[ii+1]), `RX_LOG, s_axis_rq_tdata[(ii+2)*32-1 -:64], ~(s_axis_rq_tkeep[ii+1]&s_axis_rq_tkeep[ii]));
                              end

                              TSK_PARSE_FRAME(`RX_LOG);
                              rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
                        end
                     end
                  end
                  else begin
                     rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
                  end
               end
            end

            `RQ_RX_ACTIVE : begin
               if (user_reset_n == 1'b0)
                  rq_rx_state <= #(Tcq) `RQ_RX_RESET;
               else if (user_lnk_up == 1'b0)
                  rq_rx_state <= #(Tcq) `RQ_RX_DOWN;
               else if ( (s_axis_rq_tvalid == 1'b1) && (s_axis_rq_tlast == 1'b1) && (s_axis_rq_tready == 1'b1)  ) begin
                  if(C_DATA_WIDTH==64)begin
                     if(rq_beat0_valid != s_axis_rq_tuser[40]) begin
                             TSK_BUILD_RQ_TO_PCIE_PKT(rq_data,rq_be,s_axis_rq_tdata);
                        rq_beat0_valid <= s_axis_rq_tuser[40];
                     end
                     else begin
                        TSK_READ_DATA(1, `RX_LOG, s_axis_rq_tdata,~s_axis_rq_tkeep[1] );
                     end
                        TSK_PARSE_FRAME(`RX_LOG);
                        rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
                  end
                  else if(C_DATA_WIDTH==128)begin
                     for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                        if(s_axis_rq_tkeep[ii] == 1'b1 ||s_axis_rq_tkeep[ii+1] == 1'b1 )
                           TSK_READ_DATA(~(s_axis_rq_tkeep[ii] & s_axis_rq_tkeep[ii+1]), `RX_LOG, s_axis_rq_tdata[(ii+2)*32-1 -:64], ~(s_axis_rq_tkeep[ii+1]&s_axis_rq_tkeep[ii]));
                     end

                     TSK_PARSE_FRAME(`RX_LOG);
                     rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
                  end
                  else if(C_DATA_WIDTH==256)begin
                     for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                        if(s_axis_rq_tkeep[ii] == 1'b1 ||s_axis_rq_tkeep[ii+1] == 1'b1 )
                           TSK_READ_DATA(~(s_axis_rq_tkeep[ii] & s_axis_rq_tkeep[ii+1]), `RX_LOG, s_axis_rq_tdata[(ii+2)*32-1 -:64],~(s_axis_rq_tkeep[ii] & s_axis_rq_tkeep[ii+1]) );
                     end

                     TSK_PARSE_FRAME(`RX_LOG);
                     rq_rx_state <= #(Tcq) `RQ_RX_IDLE;
                  end
               end 
               else if (  (s_axis_rq_tvalid == 1'b1) && (s_axis_rq_tready == 1'b1)  ) begin
                  if(C_DATA_WIDTH==64)begin
                     if(rq_beat0_valid != s_axis_rq_tuser[40]) begin
                        TSK_BUILD_RQ_TO_PCIE_PKT(rq_data,rq_be,s_axis_rq_tdata);
                        rq_beat0_valid <= s_axis_rq_tuser[40];
                     end
                     else begin
                        TSK_READ_DATA(1, `RX_LOG, s_axis_rq_tdata, 1'b0);
                     end
                        rq_rx_state <= #(Tcq) `RQ_RX_ACTIVE;
                  end
                  else if(C_DATA_WIDTH==128)begin
                     for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                           TSK_READ_DATA(0, `RX_LOG, s_axis_rq_tdata[(ii+2)*32-1 -:64],~(s_axis_rq_tkeep[ii] & s_axis_rq_tkeep[ii+1]) );
                     end

                     rq_rx_state <= #(Tcq) `RQ_RX_ACTIVE;
                  end
                  else if(C_DATA_WIDTH==256)begin
                     for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                        TSK_READ_DATA(0, `RX_LOG, s_axis_rq_tdata[(ii+2)*32-1 -:64], ~(s_axis_rq_tkeep[ii] & s_axis_rq_tkeep[ii+1]));
                     end

                     rq_rx_state <= #(Tcq) `RQ_RX_ACTIVE;
                  end
               end
            end
         endcase
      end
   end

   always @(posedge user_clk or negedge user_reset_n) begin
      if (user_reset_n == 1'b0) begin
         cc_rx_state <= #(Tcq)  `CC_RX_RESET;
      end 
      else begin
         case (cc_rx_state)
            `CC_RX_RESET :  begin
               if (user_reset_n == 1'b0)
                  cc_rx_state <= #(Tcq) `CC_RX_RESET;
               else
                  cc_rx_state <= #(Tcq) `CC_RX_DOWN;
            end
            `CC_RX_DOWN : begin
               if (user_lnk_up == 1'b0)
                  cc_rx_state <= #(Tcq) `CC_RX_DOWN;
               else begin
                  cc_rx_state <= #(Tcq) `CC_RX_IDLE;
               end
            end
            `CC_RX_IDLE : begin
               if (user_reset_n == 1'b0)
                  cc_rx_state <= #(Tcq) `CC_RX_RESET;
               else if (user_lnk_up == 1'b0)
                  cc_rx_state <= #(Tcq) `CC_RX_DOWN;
               else begin
                  if (  (s_axis_cc_tuser[32] == 1'b1) && (s_axis_cc_tvalid == 1'b1) && (s_axis_cc_tready == 1'b1)  ) begin
                     cc_data <=  s_axis_cc_tdata;
                     cc_be <= s_axis_cc_tuser[7:0];
                     cc_beat0_valid <= s_axis_cc_tuser[32];

                     if(C_DATA_WIDTH==64)begin
                        cc_rx_state <= #(Tcq) `CC_RX_ACTIVE;
                     end
                     else if(C_DATA_WIDTH==128)begin
                        if(AXISTEN_IF_CC_ALIGNMENT_MODE  == "TRUE" ) begin
                           TSK_BUILD_CC_TO_PCIE_PKT(s_axis_cc_tdata[63:0],s_axis_cc_tdata[C_DATA_WIDTH-1:C_DATA_WIDTH/2],4'b0111,s_axis_cc_tlast);
                        end else begin 
                           TSK_BUILD_CC_TO_PCIE_PKT(s_axis_cc_tdata[63:0],s_axis_cc_tdata[C_DATA_WIDTH-1:C_DATA_WIDTH/2],s_axis_cc_tkeep[KEEP_WIDTH-1:0],s_axis_cc_tlast);
                        end
                        if(s_axis_cc_tlast == 1'b1) begin
                           TSK_PARSE_FRAME(`RX_LOG);
                           cc_rx_state <= #(Tcq) `CC_RX_IDLE;
                        end 
                        else
                           cc_rx_state <= #(Tcq) `CC_RX_ACTIVE;
                     end
                     else if(C_DATA_WIDTH==256)begin
                        if(AXISTEN_IF_CC_ALIGNMENT_MODE  == "TRUE" ) begin
                           TSK_BUILD_CC_TO_PCIE_PKT(s_axis_cc_tdata[63:0],s_axis_cc_tdata[(C_DATA_WIDTH/2)-1:C_DATA_WIDTH/4],8'h07,s_axis_cc_tlast);
                        end 
                        else begin 
                           TSK_BUILD_CC_TO_PCIE_PKT(s_axis_cc_tdata[63:0],s_axis_cc_tdata[(C_DATA_WIDTH/2)-1:C_DATA_WIDTH/4],s_axis_cc_tkeep[KEEP_WIDTH-1:0],s_axis_cc_tlast);
                        end 

                        if(s_axis_cc_tlast == 1'b1) begin
                           for(ii=4; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                              if(s_axis_cc_tkeep[ii] == 1'b1 ||s_axis_cc_tkeep[ii+1] == 1'b1 )
                                 TSK_READ_DATA(~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]), `RX_LOG, s_axis_cc_tdata[(ii+2)*32-1 -:64],~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]) );
                              end

                              TSK_PARSE_FRAME(`RX_LOG);
                              cc_rx_state <= #(Tcq) `CC_RX_IDLE;
                        end 
                        else
                           cc_rx_state <= #(Tcq) `CC_RX_ACTIVE;
                     end
                  end 
                  else begin
                     cc_rx_state <= #(Tcq) `CC_RX_IDLE;
                  end
               end
            end
         `CC_RX_ACTIVE : begin
            if (user_reset_n == 1'b0)
               cc_rx_state <= #(Tcq) `CC_RX_RESET;
            else if (user_lnk_up == 1'b0)
               cc_rx_state <= #(Tcq) `CC_RX_DOWN;
            else if ( (s_axis_cc_tvalid == 1'b1) && (s_axis_cc_tlast == 1'b1) && (s_axis_cc_tready == 1'b1) ) begin
               if(C_DATA_WIDTH==64)begin
                  if(cc_beat0_valid != s_axis_cc_tuser[32]) begin
                     TSK_BUILD_CC_TO_PCIE_PKT(cc_data,s_axis_cc_tdata,s_axis_cc_tkeep[KEEP_WIDTH-1:0],s_axis_cc_tlast);
                     cc_beat0_valid <= s_axis_cc_tuser[32];
                  end
                  else begin
                     TSK_READ_DATA(1, `RX_LOG, {s_axis_cc_tdata[31:0],s_axis_cc_tdata[63:32]},~s_axis_cc_tkeep[1] );
                  end

                  TSK_PARSE_FRAME(`RX_LOG);
                  cc_rx_state <= #(Tcq) `CC_RX_IDLE;
               end
               else if(C_DATA_WIDTH==128)begin
                  for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                     if(s_axis_cc_tkeep[ii] == 1'b1 ||s_axis_cc_tkeep[ii+1] == 1'b1 )
                        TSK_READ_DATA(~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]), `RX_LOG, {s_axis_cc_tdata[(ii+1)*32-1 -:32],s_axis_cc_tdata[(ii+2)*32-1 -:32]}, ~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]));
                  end

                  TSK_PARSE_FRAME(`RX_LOG);
                  cc_rx_state <= #(Tcq) `CC_RX_IDLE;
               end
               else if(C_DATA_WIDTH==256)begin
                  for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                     if(s_axis_cc_tkeep[ii] == 1'b1 ||s_axis_cc_tkeep[ii+1] == 1'b1 )
                        TSK_READ_DATA(~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]), `RX_LOG, {s_axis_cc_tdata[(ii+1)*32-1 -:32],s_axis_cc_tdata[(ii+2)*32-1 -:32]}, ~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]));
                  end

                  TSK_PARSE_FRAME(`RX_LOG);
                  cc_rx_state <= #(Tcq) `CC_RX_IDLE;
               end
            end 
            else if (  (s_axis_cc_tvalid == 1'b1) && (s_axis_cc_tready == 1'b1)  ) begin
               if(C_DATA_WIDTH==64)begin
                  if(cc_beat0_valid != s_axis_cc_tuser[32]) begin
                     TSK_BUILD_CC_TO_PCIE_PKT(cc_data,s_axis_cc_tdata,2'b01,s_axis_cc_tlast);
                     cc_beat0_valid <= s_axis_cc_tuser[32];
                  end
                  else begin
                     TSK_READ_DATA(1, `RX_LOG, s_axis_cc_tdata, 1'b0);
                  end
                  
                  cc_rx_state <= #(Tcq) `CC_RX_ACTIVE;
               end
               else if(C_DATA_WIDTH==128)begin
                  for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                     TSK_READ_DATA(0, `RX_LOG, {s_axis_cc_tdata[(ii+1)*32-1 -:32],s_axis_cc_tdata[(ii+2)*32-1 -:32]}, ~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]));
                  end
                  cc_rx_state <= #(Tcq) `CC_RX_ACTIVE;
               end
               else if(C_DATA_WIDTH==256)begin
                  for(ii=0; ii<KEEP_WIDTH ; ii = ii+ 2)begin 
                     TSK_READ_DATA(0, `RX_LOG, {s_axis_cc_tdata[(ii+1)*32-1 -:32],s_axis_cc_tdata[(ii+2)*32-1 -:32]}, ~(s_axis_cc_tkeep[ii] & s_axis_cc_tkeep[ii+1]));
                  end
                  cc_rx_state <= #(Tcq) `CC_RX_ACTIVE;
               end
            end 
         end
      endcase
   end
end
//end
//endgenerate


reg [1:0]   trn_rdst_rdy_toggle_count;
reg [8:0]   trn_rnp_ok_toggle_count;

reg [31:0]  sim_timeout;

initial begin
   sim_timeout       = `RQ_RX_TIMEOUT;
   s_axis_rq_tready  =1'b1;
   s_axis_cc_tready  =1'b1;
   pcie_rq_np_req    =1'b1;
end

/* Transaction Receive Timeout */

task TSK_BUILD_CC_TO_PCIE_PKT;
   input [63:0] cc_data_QW0;
   input [63:0] cc_data_QW1;
   input [KEEP_WIDTH-1:0] s_axis_cc_tkeep;
   input s_axis_cc_tlast;

   reg [127:0] pcie_pkt;

   integer index;

   begin
   if(C_DATA_WIDTH == 64) begin
      index=1;
   end
   else begin
      index=3;
   end

   if((C_DATA_WIDTH == 64 && s_axis_cc_tkeep[index]==1'b1) || (C_DATA_WIDTH > 64 && s_axis_cc_tkeep[index]==1'b1)) begin
      pcie_pkt = { 1'b0,
                   2'b10,
                   5'b01010,
                   1'b0,
                   cc_data_QW1[27:25],
                   1'b0,
                   cc_data_QW1[30],
                   1'b0,
                   1'b0,
                   1'b0,
                   cc_data_QW0[46],
                   cc_data_QW1[29:28],
                   2'b00,
                   cc_data_QW0[41:32],    // 32
                   cc_data_QW1[23:8],
                   cc_data_QW0[45:43],
                   1'b0,
                   cc_data_QW0[27:16],    // 64
                   cc_data_QW0[63:48],
                   cc_data_QW1[7:0],
                   1'b0,
                   cc_data_QW0[6:0],      // 96
                   cc_data_QW1[63:32] };  // 128

         TSK_READ_DATA(0, `RX_LOG, pcie_pkt[127:64], 1'b0);
         TSK_READ_DATA(1, `RX_LOG, pcie_pkt[63:0], 1'b0);
      end
   else begin
      pcie_pkt   = { 1'b0,                         // 0
                     {~s_axis_cc_tlast,1'b0},
                     //2m_axis_cc_tlast;'b00,
                     5'b01010,
                     1'b0,
                     cc_data_QW1[27:25],
                     1'b0,
                     cc_data_QW1[30],
                     1'b0,
                     1'b0,
                     1'b0,
                     cc_data_QW0[46],
                     cc_data_QW1[29:28],
                     2'b00,
                     cc_data_QW0[41:32],           // 32
                     cc_data_QW1[23:8],
                     cc_data_QW0[45:43],
                     1'b0,
                     cc_data_QW0[27:16],           // 64
                     cc_data_QW0[63:48],
                     cc_data_QW1[7:0],
                     1'b0,
                     cc_data_QW0[6:0],             // 96
                     32'h00000000                  // 128
                     };
      TSK_READ_DATA(0, `RX_LOG, pcie_pkt[127:64], 1'b0);
      TSK_READ_DATA(1, `RX_LOG, pcie_pkt[63:0], 1'b1);
   end
   end
endtask

//----------------------------------------------------------------------------------------------------//
task TSK_BUILD_RQ_TO_PCIE_PKT;
   input [63:0] rq_data;
   input [7:0]  rq_be;
   input [63:0] s_axis_rq_tdata;

   reg [127:0] pcie_pkt;

   begin
      req_compl_wd = (s_axis_rq_tdata[10:0] != 11'h000 && s_axis_rq_tdata[14:11] == 4'b0000) ? 1'b1 : 1'b0;

      //--------------------------------------------------------------//
      // Calculate byte count based on byte enable
      //--------------------------------------------------------------//
      casex ({rq_be[3:0],rq_be[7:4]})
         8'b1xx10000 : byte_count = 12'h004;
         8'b01x10000 : byte_count = 12'h003;
         8'b1x100000 : byte_count = 12'h003;
         8'b00110000 : byte_count = 12'h002;
         8'b01100000 : byte_count = 12'h002;
         8'b11000000 : byte_count = 12'h002;
         8'b00010000 : byte_count = 12'h001;
         8'b00100000 : byte_count = 12'h001;
         8'b01000000 : byte_count = 12'h001;
         8'b10000000 : byte_count = 12'h001;
         8'b00000000 : byte_count = 12'h001;
         8'bxxx11xxx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 0;
         8'bxxx101xx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 1;
         8'bxxx1001x : byte_count = (s_axis_rq_tdata[10:0] * 4) - 2;
         8'bxxx10001 : byte_count = (s_axis_rq_tdata[10:0] * 4) - 3;
         8'bxx101xxx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 1;
         8'bxx1001xx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 2;
         8'bxx10001x : byte_count = (s_axis_rq_tdata[10:0] * 4) - 3;
         8'bxx100001 : byte_count = (s_axis_rq_tdata[10:0] * 4) - 4;
         8'bx1001xxx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 2;
         8'bx10001xx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 3;
         8'bx100001x : byte_count = (s_axis_rq_tdata[10:0] * 4) - 4;
         8'bx1000001 : byte_count = (s_axis_rq_tdata[10:0] * 4) - 5;
         8'b10001xxx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 3;
         8'b100001xx : byte_count = (s_axis_rq_tdata[10:0] * 4) - 4;
         8'b1000001x : byte_count = (s_axis_rq_tdata[10:0] * 4) - 5;
         8'b10000001 : byte_count = (s_axis_rq_tdata[10:0] * 4) - 6;
      endcase

      // Calculate lower address based on  byte enable
      casex ({req_compl_wd,rq_be[3:0]})

         5'b0_xxxx : lower_addr = 8'h0;
         5'bx_0000 : lower_addr = {rq_data[6:2], 2'b00};
         5'bx_xxx1 : lower_addr = {rq_data[6:2], 2'b00};
         5'bx_xx10 : lower_addr = {rq_data[6:2], 2'b01};
         5'bx_x100 : lower_addr = {rq_data[6:2], 2'b10};
         5'bx_1000 : lower_addr = {rq_data[6:2], 2'b11};

      endcase

      case(s_axis_rq_tdata[14:11])
         4'b0000: begin // Memory Read Request 收到读memory请求，构造
            pcie_pkt = {
                        ((rq_data[63:32] == 32'h0) ? 3'b000 : 3'b001), // Fmt (32-bit or 64-bit)
                        5'b00000,               // Type
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[59:57], // TC
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[62],    // Attr {ID Based Ordering}
                        2'b00,                  // TLP Processing Hint (TPH)
                        1'b0,                   // Digest Present
                        1'b0,                   // Error Poisoned
                        s_axis_rq_tdata[61:60], // Attributes {Relaxed Ordering, No Snoop}
                        2'b00,                  // Address Translation
                        s_axis_rq_tdata[9:0],   // Length
                        s_axis_rq_tdata[31:16], // Requester ID
                        s_axis_rq_tdata[39:32], // Tag
                        rq_be[7:4],             // Last DW Byte Enable
                        rq_be[3:0],             // First DW Byte Enable
                        ((rq_data[63:32] == 32'h0) ? {rq_data[31:2], 32'b0} : rq_data[63:2]), // Address (32-bit or 64-bit)
                        2'b00                   // *Reserved*
                       };
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[127:64], 1'b0); // 把pcie_pkt data存入frame_store数组
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[63:0], 1'b0);   // 把pcie_pkt data存入frame_store数组
            TSK_TX_COMPLETION_DATA( s_axis_rq_tdata[31:16], // req_id_
                                    s_axis_rq_tdata[39:32], // tag_
                                    s_axis_rq_tdata[59:57], // tc_
                                    s_axis_rq_tdata[10:0], // len_
                                    byte_count, // byte_count_
                                    lower_addr, //lower_addr_
                                    3'b000, // comp_status_
                                    1'b0 ); // ep_
         end
         4'b0001: begin //Memory Write Request
            pcie_pkt = {
                        ((rq_data[63:32] == 32'h0) ? 3'b010 : 3'b001), // Fmt (32-bit or 64-bit)
                        5'b00000,               // Type
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[59:57], // TC
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[62],    // Attr {ID Based Ordering}
                        2'b00,                  // TLP Processing Hint (TPH)
                        1'b0,                   // Digest Present
                        1'b0,                   // Error Poisoned
                        s_axis_rq_tdata[61:60], // Attributes {Relaxed Ordering, No Snoop}
                        2'b00,                  // Address Translation
                        s_axis_rq_tdata[9:0],   // Length
                        s_axis_rq_tdata[31:16], // Requester ID
                        s_axis_rq_tdata[39:32], // Tag
                        rq_be[7:4],             // Last DW Byte Enable
                        rq_be[3:0],             // First DW Byte Enable
                        ((rq_data[63:32] == 32'h0) ? {rq_data[31:2], 32'b0} : rq_data[63:2]), // Address (32-bit or 64-bit)
                        2'b00                   // *Reserved*
                       }; /* Only provide header -- Payload data is not presented */
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[127:64], 1'b0);
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[63:0], 1'b0);
         end

         4'b1101:begin // Vendor Defined Message
            pcie_pkt = {
                        3'b001,                 // Fmt
                        2'b10,                  // Type
                        s_axis_rq_tdata[50:48], // Message Routing
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[59:57], // TC
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[62],    // Attr {ID Based Ordering}
                        2'b00,                  // TLP Processing Hint (TPH)
                        1'b0,                   // Digest Present
                        1'b0,                   // Error Poisoned
                        s_axis_rq_tdata[61:60], // Attributes {Relaxed Ordering, No Snoop}
                        2'b00,                  // Address Translation
                        s_axis_rq_tdata[9:0],   // Length
                        s_axis_rq_tdata[31:16], // Requester ID
                        s_axis_rq_tdata[39:32], // Tag
                        s_axis_rq_tdata[47:40], // Message Code
                        rq_data[15:0],          // Destination ID (Bus/Device/Function)
                        rq_data[31:16],         // Vendor ID
                        rq_data[63:32]          // Vendor Message
                       };
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[127:64], 1'b0);
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[63:0], 1'b0);
         end

         4'b1110,4'b1100:begin // ATS Message and Other Messages
            pcie_pkt = {
                        3'b001,                 // Fmt
                        2'b10,                  // Type
                        s_axis_rq_tdata[50:48], // Message Routing
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[59:57], // TC
                        1'b0,                   // *Reserved*
                        s_axis_rq_tdata[62],    // Attr {ID Based Ordering}
                        2'b00,                  // TLP Processing Hint (TPH)
                        1'b0,                   // Digest Present
                        1'b0,                   // Error Poisoned
                        s_axis_rq_tdata[61:60], // Attributes {Relaxed Ordering, No Snoop}
                        2'b00,                  // Address Translation
                        s_axis_rq_tdata[9:0],   // Length
                        s_axis_rq_tdata[31:16], // Requester ID
                        s_axis_rq_tdata[39:32], // Tag
                        s_axis_rq_tdata[47:40], // Message Code
                        rq_data[63:0]           // Messages (ATS or non-Vendor Defined)
                       };
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[127:64], 1'b0);
            TSK_READ_DATA(0, `RX_LOG, pcie_pkt[63:0], 1'b0);
         end
      endcase
   end
endtask //TSK_BUILD_RQ_TO_PCIE_PKT

/************************************************************
   Task : TSK_READ_DATA
   Inputs : None
   Outputs : None
   Description : Consume clocks.
*************************************************************/
task TSK_READ_DATA;
   input          last;
   input          txrx;
   input  [63:0]  trn_d;
   input          trn_rem;

   integer        _i;
   reg  [7:0]     _byte;
   reg  [63:0]    _msk;
   reg  [3:0]     _rem;

   begin
      _msk = 64'hff00000000000000;
      _rem = (last ? ((trn_rem == 1) ? 4 : 8) : 8);

      for (_i = 0; _i < _rem; _i = _i + 1) begin
         _byte = (trn_d & (_msk >> (_i * 8))) >> (((7) - _i) * 8);
         if (txrx) begin
            frame_store_tx[frame_store_tx_idx] = _byte;
            frame_store_tx_idx = frame_store_tx_idx + 1;
         end else begin
            frame_store_rx[frame_store_rx_idx] = _byte;
            frame_store_rx_idx = frame_store_rx_idx + 1;
         end
      end 
   end
endtask // TSK_READ_DATA

endmodule // pcie_host_rx

