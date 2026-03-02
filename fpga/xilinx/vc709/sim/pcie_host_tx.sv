module pcie_host_tx #(  parameter        ATTR_AXISTEN_IF_ENABLE_CLIENT_TAG = 0,
                        parameter        ATTR_AXISTEN_IF_CQ_PARITY_CHECK = 0,
                        parameter        ATTR_AXISTEN_IF_CC_PARITY_CHECK = 0,
                        parameter        AXISTEN_IF_RQ_ALIGNMENT_MODE   = "FALSE",
                        parameter        AXISTEN_IF_CC_ALIGNMENT_MODE   = "FALSE",
                        parameter        AXISTEN_IF_CQ_ALIGNMENT_MODE   = "FALSE",
                        parameter        AXISTEN_IF_RC_ALIGNMENT_MODE   = "FALSE",

                        parameter        DEV_CAP_MAX_PAYLOAD_SUPPORTED = 1,
                        parameter        C_DATA_WIDTH = 128,
                        parameter        KEEP_WIDTH = C_DATA_WIDTH / 32,
                        parameter        STRB_WIDTH   = C_DATA_WIDTH / 8,
                        parameter [3:0]  LINK_CAP_MAX_LINK_WIDTH = 4'h8,
                        parameter [2:0]  LINK_CAP_MAX_LINK_SPEED = 3'h2,
                        parameter        EP_DEV_ID = 16'h7700,
                        parameter        REM_WIDTH  = ((C_DATA_WIDTH == 256) ? 3 : ((C_DATA_WIDTH == 128) ? 2 : 1)))
(
   output reg                                 m_axis_cq_tlast,
   output reg  [C_DATA_WIDTH-1:0]             m_axis_cq_tdata,
   output reg              [59:0]             m_axis_cq_tuser,
   output reg    [KEEP_WIDTH-1:0]             m_axis_cq_tkeep,
   output reg                                 m_axis_cq_tvalid,
   input                                      m_axis_cq_tready,

   output reg  [C_DATA_WIDTH-1:0]             m_axis_rc_tdata,
   output reg              [32:0]             m_axis_rc_tuser,
   output reg                                 m_axis_rc_tlast,
   output reg    [KEEP_WIDTH-1:0]             m_axis_rc_tkeep,
   output reg                                 m_axis_rc_tvalid,
   input                                      m_axis_rc_tready,

   input                  [3:0]               pcie_rq_seq_num,
   input                                      pcie_rq_seq_num_vld,
   input                  [5:0]               pcie_rq_tag,
   input                                      pcie_rq_tag_vld,

   input                  [1:0]               pcie_tfc_nph_av,
   input                  [1:0]               pcie_tfc_npd_av,

   input                                      speed_change_done_n,

   input                                      user_clk,
   input                                      reset,
   input                                      user_lnk_up
);


   reg   [31:0]                  tx_file_ptr;

   reg   [7:0]                   frame_store_tx[5119:0];
   integer                       frame_store_tx_idx;

   reg  [7:0]                    DATA_STORE [4095:0];
   reg  [(C_DATA_WIDTH - 1):0]   pcie_tlp_data;
   reg  [(REM_WIDTH - 1):0]      pcie_tlp_rem;

   reg   [31:0]           log_file_ptr;
   integer                _frame_store_idx;

  // Logic to compute the Parity of the CC and the RQ channel

  generate
    if(ATTR_AXISTEN_IF_CQ_PARITY_CHECK == 1)
    begin

      genvar a;
      for(a=0; a< STRB_WIDTH; a = a + 1) // Parity needs to be computed for every byte of data
      begin : parity_assign
        assign m_axis_cq_tparity[a] = !(  m_axis_cq_tdata[(8*a)+ 0] ^ m_axis_cq_tdata[(8*a)+ 1]
                                 ^ m_axis_cq_tdata[(8*a)+ 2] ^ m_axis_cq_tdata[(8*a)+ 3]
                                 ^ m_axis_cq_tdata[(8*a)+ 4] ^ m_axis_cq_tdata[(8*a)+ 5]
                                 ^ m_axis_cq_tdata[(8*a)+ 6] ^ m_axis_cq_tdata[(8*a)+ 7]);

        assign m_axis_rc_tparity[a] = !(  m_axis_rc_tdata[(8*a)+ 0] ^ m_axis_rc_tdata[(8*a)+ 1]
                                 ^ m_axis_rc_tdata[(8*a)+ 2] ^ m_axis_rc_tdata[(8*a)+ 3]
                                 ^ m_axis_rc_tdata[(8*a)+ 4] ^ m_axis_rc_tdata[(8*a)+ 5]
                                 ^ m_axis_rc_tdata[(8*a)+ 6] ^ m_axis_rc_tdata[(8*a)+ 7]);
      end
    end
  endgenerate

/************************************************************
Task        : TSK_TX_MEMORY_READ_32
Inputs      : Tag, Length, Address, Last Byte En, First Byte En
Outputs     : Transaction Tx Interface Signaling
Description : Generates a Memory Read 32 TLP
*************************************************************/
task TSK_TX_MEMORY_READ_32;
   input    [7:0]    tag_;
   input    [2:0]    tc_;
   input    [10:0]   len_;
   input    [31:0]   addr_;
   input    [3:0]    last_dw_be_;
   input    [3:0]    first_dw_be_;

   begin
      //-----------------------------------------------------------------------\\
      if (user_lnk_up_n) begin
          $display("[%t] :  interface is MIA", $realtime);
          $finish(1);
      end
      //-----------------------------------------------------------------------\\
      TSK_TX_SYNCHRONIZE(0, 0, 0, `SYNC_RQ_RDY);
      //-----------------------------------------------------------------------\\
      m_axis_cq_tvalid <= #(Tcq) 1'b1;
      m_axis_cq_tlast  <= #(Tcq) 1'b1;
      m_axis_cq_tkeep  <= #(Tcq) 4'hF;          // 2DW Descriptor for Memory Transactions alone
      m_axis_cq_tuser  <= #(Tcq) {(ATTR_AXISTEN_IF_CQ_PARITY_CHECK ?  m_axis_cq_tparity : 32'b0), // Parity
                                    4'b1010,      // Seq Number
                                    8'h00,        // TPH Steering Tag
                                    1'b0,         // TPH indirect Tag Enable
                                    2'b0,         // TPH Type
                                    1'b0,         // TPH Present
                                    1'b0,         // Discontinue
                                    3'b000,       // Byte Lane number in case of Address Aligned mode
                                    last_dw_be_,    // Last BE of the Read Data
                                    first_dw_be_ }; // First BE of the Read Data
      m_axis_cq_tdata  <= #(Tcq) {  1'b0,        // Force ECRC       //128
                                    3'b000,      // Attributes
                                    tc_,         // Traffic Class
                                    1'b1,        // RID Enable to use the Client supplied Bus/Device/Func No
                                    EP_BUS_DEV_FNS, //
                                    (ATTR_AXISTEN_IF_ENABLE_CLIENT_TAG ? tag_ : 8'hCC),
                                    RP_BUS_DEV_FNS, // Requester ID -- Used only when RID enable = 1  //96
                                    1'b0,       // Poisoned Req
                                    4'b0000,    // Req Type for MRd Req
                                    len_ ,     // DWORD Count
                                    32'b0,        // 32-bit Addressing. So, bits[63:32] = 0  //64
                                    addr_[31:2],  // Memeory read address 32-bits             //32
                                    2'b00};      // AT -> 00 : Untranslated Address
      //---------------------------------------------------------------\\
      pcie_tlp_data     <= #(Tcq)  {
                            1'b0,
                            2'b00,
                            5'b00000,
                            1'b0,
                            tc_,
                            4'b0000,
                            1'b0,
                            1'b0,
                            2'b00,
                            2'b00,
                            len_[9:0],             // 32
                            RP_BUS_DEV_FNS,
                            tag_,
                            last_dw_be_,
                            first_dw_be_,          // 64
                            addr_[31:2],
                            2'b00,                 // 96
                            32'b0};                // 128

      pcie_tlp_rem <= #(Tcq)  2'b01;
      //-----------------------------------------------------------------------\\
      TSK_TX_SYNCHRONIZE(1, 1, 1, `SYNC_RQ_RDY);
      //-----------------------------------------------------------------------\\
      m_axis_cq_tvalid     <= #(Tcq) 1'b0;
      m_axis_cq_tlast      <= #(Tcq) 1'b0;
      m_axis_cq_tkeep      <= #(Tcq) 4'h0;
      m_axis_cq_tuser      <= #(Tcq) 60'b0;
      m_axis_cq_tdata      <= #(Tcq) 128'b0;
      //-----------------------------------------------------------------------\\
      pcie_tlp_rem         <= #(Tcq)  0;
      //-----------------------------------------------------------------------\\
   end
endtask // TSK_TX_MEMORY_READ_32

/************************************************************
Task        : TSK_TX_MEMORY_READ_64
Inputs      : Tag, Length, Address, Last Byte En, First Byte En
Outputs     : Transaction Tx Interface Signaling
Description : Generates a Memory Read 64 TLP
*************************************************************/
task TSK_TX_MEMORY_READ_64;
   input    [7:0]    tag_;
   input    [2:0]    tc_;
   input    [10:0]   len_;
   input    [63:0]   addr_;
   input    [3:0]    last_dw_be_;
   input    [3:0]    first_dw_be_;
   
   begin
      //-----------------------------------------------------------------------\\
      if (user_lnk_up_n) begin
          $display("[%t] :  interface is MIA", $realtime);
          $finish(1);
      end

      TSK_TX_SYNCHRONIZE(0, 0, 0, `SYNC_RQ_RDY);

      m_axis_cq_tvalid <= #(Tcq) 1'b1;
      m_axis_cq_tlast  <= #(Tcq) 1'b1;
      m_axis_cq_tkeep  <= #(Tcq) 4'hF; // 2DW Descriptor for Memory Transactions alone
      m_axis_cq_tuser  <= #(Tcq) {(ATTR_AXISTEN_IF_CQ_PARITY_CHECK ?  m_axis_cq_tparity : 32'b0), // Parity
                                    4'b1010,      // Seq Number
                                    8'h00,        // TPH Steering Tag
                                    1'b0,         // TPH indirect Tag Enable
                                    2'b0,         // TPH Type
                                    1'b0,         // TPH Present
                                    1'b0,         // Discontinue
                                    3'b000,       // Byte Lane number in case of Address Aligned mode
                                    last_dw_be_,    // Last BE of the Read Data
                                    first_dw_be_ }; // First BE of the Read Data
      m_axis_cq_tdata  <= #(Tcq) {1'b0,        // Force ECRC       //128
                                  3'b000,      // Attributes
                                  tc_,         // Traffic Class
                                  1'b1,        // RID Enable to use the Client supplied Bus/Device/Func No
                                  EP_BUS_DEV_FNS, //
                                  (ATTR_AXISTEN_IF_ENABLE_CLIENT_TAG ? tag_ : 8'hCC), //

                                  RP_BUS_DEV_FNS, // Requester ID -- Used only when RID enable = 1  //96
                                   1'b0,       // Poisoned Req
                                   4'b0000,    // Req Type for MRd Req
                                   len_ ,     // DWORD Count

                                   addr_[63:2],  // Memeory read address 64-bits  //64
                                   2'b00};

      pcie_tlp_data  <= (Tcq)  {
                                 1'b0,
                                 2'b01,
                                 5'b00000,
                                 1'b0,
                                 tc_,
                                 4'b0000,
                                 1'b0,
                                 1'b0,
                                 2'b00,
                                 2'b00,
                                 len_[9:0],           // 32

                                 RP_BUS_DEV_FNS,
                                 tag_,
                                 last_dw_be_,
                                 first_dw_be_,        // 64

                                 addr_[63:2],
                                 2'b00 };             //128

      pcie_tlp_rem         <= #(Tcq)  2'b00;

      TSK_TX_SYNCHRONIZE(1, 1, 1, `SYNC_RQ_RDY);

      m_axis_cq_tvalid <= #(Tcq) 1'b0;
      m_axis_cq_tlast  <= #(Tcq) 1'b0;
      m_axis_cq_tkeep  <= #(Tcq) 4'h0;
      m_axis_cq_tuser  <= #(Tcq) 60'b0;
      m_axis_cq_tdata  <= #(Tcq) 128'b0;

      pcie_tlp_rem     <= #(Tcq)  2'b00;

   end
endtask // TSK_TX_MEMORY_READ_64

/************************************************************
Task        : TSK_TX_MEMORY_WRITE_32
Inputs      : Tag, Length, Address, Last Byte En, First Byte En
Outputs     : Transaction Tx Interface Signaling
Description : Generates a Memory Write 32 TLP
*************************************************************/
task TSK_TX_MEMORY_WRITE_32;
   input  [7:0]    tag_;         // Tag
   input  [2:0]    tc_;          // Traffic Class
   input  [10:0]   len_;         // Length (in DW)
   input  [31:0]   addr_;        // Address
   input  [3:0]    last_dw_be_;  // Last DW Byte Enable
   input  [3:0]    first_dw_be_; // First DW Byte Enable
   input           ep_;          // Poisoned Data: Payload is invalid if set

   reg    [10:0]   _len;         // Length Info on pcie_tlp_data -- Used to count how many times to loop
   reg    [10:0]   len_i;        // Length Info on m_axis_cq_tdata -- Used to count how many times to loop
   reg    [2:0]    aa_dw;        // Adjusted DW Count for Address Aligned Mode
   reg    [127:0]  aa_data;      // Adjusted Data for Address Aligned Mode
   reg    [31:0]   data_pcie_i;  // Data Info for pcie_tlp_data
   reg             log_last_dw;  // A switch to turn on or off data logging on the last beat
   integer         _j;

   begin
      log_last_dw     = 1'b1;
      //-----------------------------------------------------------------------\\
      if (AXISTEN_IF_RQ_ALIGNMENT_MODE=="TRUE")
          aa_dw = {1'b0, addr_[3:2]};
      else
          aa_dw = 3'b000;
      
      len_i = len_ + aa_dw;
      _len  = len_;
      //-----------------------------------------------------------------------\\
      if (user_lnk_up_n) begin
          $display("[%t] :  interface is MIA", $realtime);
          $finish(1);
      end
      //-----------------------------------------------------------------------\\
      TSK_TX_SYNCHRONIZE(0, 0, 0, `SYNC_RQ_RDY);
      //-----------------------------------------------------------------------\\
      // Start of First Data Beat
      m_axis_cq_tvalid <= #(Tcq) 1'b1;
      m_axis_cq_tlast  <= #(Tcq) 1'b0; // 4DWs Descriptor for Memory Transactions alone, so
      m_axis_cq_tkeep  <= #(Tcq) 4'hF; // it will never complete on the first beat
      m_axis_cq_tuser  <= #(Tcq) {
                                 (ATTR_AXISTEN_IF_CQ_PARITY_CHECK ?  m_axis_cq_tparity : 32'b0), // Parity
                                 4'b1010,      // Seq Number
                                 8'h00,        // TPH Steering Tag
                                 1'b0,         // TPH indirect Tag Enable
                                 2'b0,         // TPH Type
                                 1'b0,         // TPH Present
                                 1'b0,         // Discontinue
                                 aa_dw,        // Byte Lane number in case of Address Aligned mode
                                 last_dw_be_,  // Last BE of the Read Data
                                 first_dw_be_  // First BE of the Read Data
                                 };
      m_axis_cq_tdata  <= #(Tcq) { //128
                                 1'b0,             // Force ECRC
                                 3'b010,           // Attributes
                                 tc_,              // Traffic Class
                                 1'b1,             // RID Enable to use the Client supplied Bus/Device/Func No
                                 EP_BUS_DEV_FNS, // Completer ID
                                 (ATTR_AXISTEN_IF_ENABLE_CLIENT_TAG ? tag_ : 8'hCC), // Tag
                                   //96
                                 RP_BUS_DEV_FNS, // Requester ID -- Used only when RID enable = 1
                                 ep_,              // Poisoned Req
                                 4'b0001,          // Req Type for MWr Req
                                 (set_malformed ? (len_+11'h4) : len_), // DWORD Count - length does not include padded zeros
                                  //64
                                 32'b0,        // 32-bit Addressing. So, bits[63:32] = 0
                                 addr_[31:2],  // Memory Write address 64-bits
                                 2'b00         // AT -> 00 : Untranslated Address
                                 };
      //-----------------------------------------------------------------------\\
      data_pcie_i    = {
                       DATA_STORE[0],
                       DATA_STORE[1],
                       DATA_STORE[2],
                       DATA_STORE[3]
                       };
      
      pcie_tlp_data <= #(Tcq) {
                              3'b010,       // Fmt for 32-bit MWr Req
                              5'b00000,     // Type for 32-bit Mwr Req
                              1'b0,         // *reserved*
                              tc_,          // 3-bit Traffic Class
                              1'b0,         // *reserved*
                              1'b0,         // Attributes {ID Based Ordering)
                              1'b0,         // *reserved*
                              1'b0,         // TLP Processing Hints
                              1'b0,         // TLP Digest Present
                              ep_,          // Poisoned Req
                              2'b00,        // Attributes {Relaxed Ordering, No Snoop}
                              2'b00,        // Address Translation
                              (set_malformed ? (len_[9:0]+10'h4) : len_[9:0]), // DWORD Count
                              RP_BUS_DEV_FNS,   // Requester ID
                              tag_,             // Tag
                              last_dw_be_,      // Last DW Byte Enable
                              first_dw_be_,     // First DW Byte Enable
                               //64
                              addr_[31:2],      // Req Address
                              2'b00,            // *reserved* or Processing Hint
                               //96
                              data_pcie_i       // Payload Data
                               //128
                              };
      
      pcie_tlp_rem  <= #(Tcq) 2'b00;
      set_malformed <= #(Tcq) 1'b0;
      _len           = (_len > 0) ? (_len - 11'h1) : 11'h0;
      //-----------------------------------------------------------------------\\
      // The following check is required because AXIS RQ may be one beat
      // longer than the actual PCIe TLP due to the fact that AXIS RQ header is always 4DW long.
      // When it happens do not log the last clock beat, but just send the packet on AXIS RQ interface
      if (_len == 0) begin
          log_last_dw      = 1'b0;
          TSK_TX_SYNCHRONIZE(1, 1, 1, `SYNC_RQ_RDY);
      end else begin
          TSK_TX_SYNCHRONIZE(1, 1, 0, `SYNC_RQ_RDY);
      end
      // End of First Data Beat
      //-----------------------------------------------------------------------\\
      // Start of Second and Subsequent Data Beat
      for (_j = 0; len_i != 0; _j = _j + 16) begin
          if(_j == 0) begin 
              aa_data = {
                        DATA_STORE[_j + 15],
                        DATA_STORE[_j + 14],
                        DATA_STORE[_j + 13],
                        DATA_STORE[_j + 12],
                        DATA_STORE[_j + 11],
                        DATA_STORE[_j + 10],
                        DATA_STORE[_j +  9],
                        DATA_STORE[_j +  8],
                        DATA_STORE[_j +  7],
                        DATA_STORE[_j +  6],
                        DATA_STORE[_j +  5],
                        DATA_STORE[_j +  4],
                        DATA_STORE[_j +  3],
                        DATA_STORE[_j +  2],
                        DATA_STORE[_j +  1],
                        DATA_STORE[_j +  0]
                        } << (aa_dw*4*8);
          end else begin 
              aa_data = {
                        DATA_STORE[_j + 15 - (aa_dw*4)],
                        DATA_STORE[_j + 14 - (aa_dw*4)],
                        DATA_STORE[_j + 13 - (aa_dw*4)],
                        DATA_STORE[_j + 12 - (aa_dw*4)],
                        DATA_STORE[_j + 11 - (aa_dw*4)],
                        DATA_STORE[_j + 10 - (aa_dw*4)],
                        DATA_STORE[_j +  9 - (aa_dw*4)],
                        DATA_STORE[_j +  8 - (aa_dw*4)],
                        DATA_STORE[_j +  7 - (aa_dw*4)],
                        DATA_STORE[_j +  6 - (aa_dw*4)],
                        DATA_STORE[_j +  5 - (aa_dw*4)],
                        DATA_STORE[_j +  4 - (aa_dw*4)],
                        DATA_STORE[_j +  3 - (aa_dw*4)],
                        DATA_STORE[_j +  2 - (aa_dw*4)],
                        DATA_STORE[_j +  1 - (aa_dw*4)],
                        DATA_STORE[_j +  0 - (aa_dw*4)]
                        };
          end

          m_axis_cq_tdata  <= #(Tcq) aa_data;

          if ((len_i)/4 == 0) begin
              case ((len_i) % 4)
                  1 : begin len_i = len_i - 1; m_axis_cq_tkeep <= #(Tcq) 4'h1; end // D0---------
                  2 : begin len_i = len_i - 2; m_axis_cq_tkeep <= #(Tcq) 4'h3; end // D0-D1------
                  3 : begin len_i = len_i - 3; m_axis_cq_tkeep <= #(Tcq) 4'h7; end // D0-D1-D2---
                  0 : begin len_i = len_i - 4; m_axis_cq_tkeep <= #(Tcq) 4'hF; end // D0-D1-D2-D3
              endcase 
          end 
          else begin
              len_i = len_i - 4; m_axis_cq_tkeep <= #(Tcq) 4'hF;     // D0-D1-D2-D3
          end
          //-----------------------------------------------------------------------\\
          pcie_tlp_data <= #(Tcq) { // Note: _j+4 because the first 1DW payload data is already sent earlier
                                  DATA_STORE[_j+4 + 0],
                                  DATA_STORE[_j+4 + 1],
                                  DATA_STORE[_j+4 + 2],
                                  DATA_STORE[_j+4 + 3],
                                  DATA_STORE[_j+4 + 4],
                                  DATA_STORE[_j+4 + 5],
                                  DATA_STORE[_j+4 + 6],
                                  DATA_STORE[_j+4 + 7]
                                  };
          
          if ((_len)/4 == 0) begin
              case ((_len) % 4)
                  1 : begin _len = _len - 1; pcie_tlp_rem <= #(Tcq) 2'b11; end // D0---------
                  2 : begin _len = _len - 2; pcie_tlp_rem <= #(Tcq) 2'b10; end // D0-D1------
                  3 : begin _len = _len - 3; pcie_tlp_rem <= #(Tcq) 2'b01; end // D0-D1-D2---
                  0 : begin _len = _len - 4; pcie_tlp_rem <= #(Tcq) 2'b00; end // D0-D1-D2-D3
              endcase 
          end 
          else begin
              _len = _len - 4; 
              pcie_tlp_rem <= #(Tcq) 2'b00;     // D0-D1-D2-D3
          end
          
          if (len_i != 0) begin
              m_axis_cq_tlast     <= #(Tcq) 1'b0;

              // The following check is required because AXIS RQ may be one beat
              // longer than the actual PCIe TLP due to the fact that AXIS RQ header is always 4DW long.
              // When it happens do not log the last clock beat, but just send the packet on AXIS RQ interface
              if (_len != 0) begin
                  TSK_TX_SYNCHRONIZE(0, 1, 0, `SYNC_RQ_RDY);
              end else begin
                  log_last_dw      = 1'b0;
                  TSK_TX_SYNCHRONIZE(0, 1, 1, `SYNC_RQ_RDY);
              end
          end else begin
              m_axis_cq_tlast     <= #(Tcq) 1'b1;
              TSK_TX_SYNCHRONIZE(0, log_last_dw, log_last_dw, `SYNC_RQ_RDY);
          end // len_i

      end // for loop
      // End of Second and Subsequent Data Beat
      //-----------------------------------------------------------------------\\
      // Packet Complete - Drive 0s
      m_axis_cq_tvalid         <= #(Tcq) 1'b0;
      m_axis_cq_tlast          <= #(Tcq) 1'b0;
      m_axis_cq_tkeep          <= #(Tcq) 4'h0;
      m_axis_cq_tuser          <= #(Tcq) 60'b0;
      m_axis_cq_tdata          <= #(Tcq) 128'b0;
      //-----------------------------------------------------------------------\\
      pcie_tlp_rem             <= #(Tcq) 2'b00;
      //-----------------------------------------------------------------------\\
   end
endtask // TSK_TX_MEMORY_WRITE_32

/************************************************************
Task        : TSK_TX_MEMORY_WRITE_64
Inputs      : Tag, Length, Address, Last Byte En, First Byte En
Outputs     : Transaction Tx Interface Signaling
Description : Generates a Memory Write 64 TLP
*************************************************************/
task TSK_TX_MEMORY_WRITE_64;
   input  [7:0]    tag_;         // Request Tag
   input  [2:0]    tc_;          // Request Traffic Class
   input  [10:0]   len_;         // Request Length (in DW)
   input  [63:0]   addr_;        // Request Address
   input  [3:0]    last_dw_be_;  // Request Last DW Byte Enable
   input  [3:0]    first_dw_be_; // Request First DW Byte Enable
   input           ep_;          // Poisoned Data: Payload is invalid if set

   reg    [10:0]   _len;         // Length Info on pcie_tlp_data -- Used to count how many times to loop
   reg    [10:0]   len_i;        // Length Info on m_axis_cq_tdata -- Used to count how many times to loop
   reg    [2:0]    aa_dw;        // Adjusted DW Count for Address Aligned Mode
   reg    [127:0]  aa_data;      // Adjusted Data for Address Aligned Mode
   integer         _j;           // Byte Index
   
   begin
      //-----------------------------------------------------------------------\\
      if (AXISTEN_IF_RQ_ALIGNMENT_MODE=="TRUE")
         aa_dw = {1'b0, addr_[3:2]};
      else
         aa_dw = 3'b000;

      len_i = len_ + aa_dw;
      _len  = len_;
      //-----------------------------------------------------------------------\\
      if (user_lnk_up_n) begin
          $display("[%t] :  interface is MIA", $realtime);
          $finish(1);
      end
      //-----------------------------------------------------------------------\\
      TSK_TX_SYNCHRONIZE(0, 0, 0, `SYNC_RQ_RDY);
      //-----------------------------------------------------------------------\\
      // Start of First Data Beat
      m_axis_cq_tvalid <= #(Tcq) 1'b1;
      m_axis_cq_tlast  <= #(Tcq) 1'b0;  // 4DW Descriptor for memory transactions alone, so
      m_axis_cq_tkeep  <= #(Tcq) 4'hF;  // it will never complete on the first beat
      m_axis_cq_tuser  <= #(Tcq) { (ATTR_AXISTEN_IF_CQ_PARITY_CHECK ?  m_axis_cq_tparity : 32'b0), // Parity
                                    4'b1010,      // Seq Number
                                    8'h00,        // TPH Steering Tag
                                    1'b0,         // TPH indirect Tag Enable
                                    2'b0,         // TPH Type
                                    1'b0,         // TPH Present
                                    1'b0,         // Discontinue
                                    aa_dw,        // Byte Lane number in case of Address Aligned mode
                                    last_dw_be_,  // Last BE of the Read Data
                                    first_dw_be_  // First BE of the Read Data
                                };
      
      m_axis_cq_tdata  <= #(Tcq) {//128
                                1'b0,         // Force ECRC
                                3'b010,       // Attributes
                                tc_,          // Traffic Class
                                1'b1,         // RID Enable to use the Client supplied Bus/Device/Func No
                                EP_BUS_DEV_FNS, // Completer ID
                                (ATTR_AXISTEN_IF_ENABLE_CLIENT_TAG ? tag_ : 8'hCC), // Tag
                                 //96
                                RP_BUS_DEV_FNS, // Requester ID -- Used only when RID enable = 1
                                ep_,          // Poisoned Req
                                4'b0001,      // Req Type for MWr Req
                                (set_malformed ? (len_+11'h4) : len_), // DWORD Count - length does not include padded zeros
                                 //64
                                addr_[63:2],  // Memory Write address 64-bits
                                2'b00         // AT -> 00 : Untranslated Address
                                };
      //-----------------------------------------------------------------------\\
      pcie_tlp_data <= #(Tcq) {
                          3'b011,      // Fmt for 64-bit MWr Req
                          5'b00000,    // Type for 64-bit Mwr Req
                          1'b0,        // *reserved*
                          tc_,         // 3-bit Traffic Class
                          1'b0,        // *reserved*
                          1'b0,        // Attributes {ID Based Ordering)
                          1'b0,        // *reserved*
                          1'b0,        // TLP Processing Hints
                          1'b0,        // TLP Digest Present
                          ep_,         // Poisoned Req
                          2'b00,       // Attributes {Relaxed Ordering, No Snoop}
                          2'b00,       // Address Translation
                          (set_malformed ? (len_[9:0]+10'h4) : len_[9:0]), // DWORD Count
                          RP_BUS_DEV_FNS,   // Requester ID
                          tag_,             // Tag
                          last_dw_be_,      // Last DW Byte Enable
                          first_dw_be_,     // First DW Byte Enable
                           //64
                          addr_[63:2],      // Req Address
                          2'b00             // *reserved* or Processing Hint
                           //128
                          };

      pcie_tlp_rem         <= #(Tcq) 2'b00;
      set_malformed        <= #(Tcq) 1'b0;
      //-----------------------------------------------------------------------\\
      TSK_TX_SYNCHRONIZE(1, 1, 0, `SYNC_RQ_RDY);
      // End of First Data Beat
      //-----------------------------------------------------------------------\\
      // Start of Second and Subsequent Data Beat
      for (_j = 0; len_i != 0; _j = _j + 16) begin
         if(_j == 0) begin 
             aa_data = {  DATA_STORE[_j + 15],
                          DATA_STORE[_j + 14],
                          DATA_STORE[_j + 13],
                          DATA_STORE[_j + 12],
                          DATA_STORE[_j + 11],
                          DATA_STORE[_j + 10],
                          DATA_STORE[_j +  9],
                          DATA_STORE[_j +  8],
                          DATA_STORE[_j +  7],
                          DATA_STORE[_j +  6],
                          DATA_STORE[_j +  5],
                          DATA_STORE[_j +  4],
                          DATA_STORE[_j +  3],
                          DATA_STORE[_j +  2],
                          DATA_STORE[_j +  1],
                          DATA_STORE[_j +  0]
                       } << (aa_dw*4*8);
         end 
         else begin 
             aa_data = {  DATA_STORE[_j + 15 - (aa_dw*4)],
                          DATA_STORE[_j + 14 - (aa_dw*4)],
                          DATA_STORE[_j + 13 - (aa_dw*4)],
                          DATA_STORE[_j + 12 - (aa_dw*4)],
                          DATA_STORE[_j + 11 - (aa_dw*4)],
                          DATA_STORE[_j + 10 - (aa_dw*4)],
                          DATA_STORE[_j +  9 - (aa_dw*4)],
                          DATA_STORE[_j +  8 - (aa_dw*4)],
                          DATA_STORE[_j +  7 - (aa_dw*4)],
                          DATA_STORE[_j +  6 - (aa_dw*4)],
                          DATA_STORE[_j +  5 - (aa_dw*4)],
                          DATA_STORE[_j +  4 - (aa_dw*4)],
                          DATA_STORE[_j +  3 - (aa_dw*4)],
                          DATA_STORE[_j +  2 - (aa_dw*4)],
                          DATA_STORE[_j +  1 - (aa_dw*4)],
                          DATA_STORE[_j +  0 - (aa_dw*4)]
                       };
         end
         
         m_axis_cq_tdata  <= #(Tcq) aa_data;
         
         if ((len_i)/4 == 0) begin
             case ((len_i) % 4)
                 1 : begin len_i = len_i - 1; m_axis_cq_tkeep <= #(Tcq) 4'h1; end // D0---------
                 2 : begin len_i = len_i - 2; m_axis_cq_tkeep <= #(Tcq) 4'h3; end // D0-D1------
                 3 : begin len_i = len_i - 3; m_axis_cq_tkeep <= #(Tcq) 4'h7; end // D0-D1-D2---
                 0 : begin len_i = len_i - 4; m_axis_cq_tkeep <= #(Tcq) 4'hF; end // D0-D1-D2-D3
             endcase 
         end 
         else begin
             len_i = len_i - 4; m_axis_cq_tkeep <= #(Tcq) 4'hF;     // D0-D1-D2-D3
         end
         //-----------------------------------------------------------------------\\
         pcie_tlp_data <= #(Tcq) { DATA_STORE[_j + 0],
                                   DATA_STORE[_j + 1],
                                   DATA_STORE[_j + 2],
                                   DATA_STORE[_j + 3],
                                   DATA_STORE[_j + 4],
                                   DATA_STORE[_j + 5],
                                   DATA_STORE[_j + 6],
                                   DATA_STORE[_j + 7],
                                   DATA_STORE[_j + 8],
                                   DATA_STORE[_j + 9],
                                   DATA_STORE[_j + 10],
                                   DATA_STORE[_j + 11],
                                   DATA_STORE[_j + 12],
                                   DATA_STORE[_j + 13],
                                   DATA_STORE[_j + 14],
                                   DATA_STORE[_j + 15]
                                   };
         if ((_len)/4 == 0) begin
           case ((_len) % 4)
               1 : begin _len = _len - 1; pcie_tlp_rem <= #(Tcq) 2'b11; end // D0---------
               2 : begin _len = _len - 2; pcie_tlp_rem <= #(Tcq) 2'b10; end // D0-D1------
               3 : begin _len = _len - 3; pcie_tlp_rem <= #(Tcq) 2'b01; end // D0-D1-D2---
               0 : begin _len = _len - 4; pcie_tlp_rem <= #(Tcq) 2'b00; end // D0-D1-D2-D3
           endcase 
         end 
         else begin
           _len = _len - 4; 
           pcie_tlp_rem <= #(Tcq) 2'b00;     // D0-D1-D2-D3
         end

         if (len_i != 0) begin
            m_axis_cq_tlast <= #(Tcq) 1'b0;
            TSK_TX_SYNCHRONIZE(0, 1, 0, `SYNC_RQ_RDY);
         end 
         else begin
            m_axis_cq_tlast <= #(Tcq) 1'b1;
            TSK_TX_SYNCHRONIZE(0, 1, 1, `SYNC_RQ_RDY);
         end // len_
      end // for loop

      // End of Second and Subsequent Data Beat
      //-----------------------------------------------------------------------\\
      // Packet Complete - Drive 0s
      m_axis_cq_tvalid         <= #(Tcq) 1'b0;
      m_axis_cq_tlast          <= #(Tcq) 1'b0;
      m_axis_cq_tkeep          <= #(Tcq) 4'h0;
      m_axis_cq_tuser          <= #(Tcq) 60'b0;
      m_axis_cq_tdata          <= #(Tcq) 128'b0;
      //-----------------------------------------------------------------------\\
      pcie_tlp_rem             <= #(Tcq) 2'b00;
      //-----------------------------------------------------------------------\\
   end
endtask // TSK_TX_MEMORY_WRITE_64

/************************************************************
Task        : TSK_TX_COMPLETION
Inputs      : Tag, TC, Length, Completion ID
Outputs     : Transaction Tx Interface Signaling
Description : Generates a Completion TLP
*************************************************************/
task TSK_TX_COMPLETION;
   input    [15:0]   req_id_;
   input    [7:0]    tag_;
   input    [2:0]    tc_;
   input    [10:0]   len_;
   input    [2:0]    comp_status_;

   begin
      if (user_lnk_up_n) begin
          $display("[%t] :  interface is MIA", $realtime);
          $finish(1);
      end

      TSK_TX_SYNCHRONIZE(0, 0, 0, `SYNC_CC_RDY);

      m_axis_rc_tvalid <= #(Tcq) 1'b1;
      m_axis_rc_tlast  <= #(Tcq) 1'b1;
      m_axis_rc_tkeep  <= #(Tcq) 4'h7;
      m_axis_rc_tuser  <= #(Tcq) {(ATTR_AXISTEN_IF_RC_PARITY_CHECK ? m_axis_rc_tparity : 32'b0),1'b0};
      m_axis_rc_tdata  <= #(Tcq) {32'b0,    // Tied to 0 for 3DW Completion Descriptor  //128

                                  1'b0,     // Force ECRC                               //96
                                  3'b0,     // Attr
                                  tc_,      //
                                  1'b1,      // Completer ID to Control Selection of Client
                                  RP_BUS_DEV_FNS, //Bus, Device/Fun No
                                  tag_,

                                  req_id_,        // Requester ID                      //64
                                  1'b0,           // Rsvd
                                  1'b0,           // Posioned Completion
                                  comp_status_,   //SuccessFull Completion
                                  len_,           //DWORD Count

                                  2'b0,           // Rsvd                              //32
                                  1'b0,           // Locked Read Completion
                                  13'h0004,       // Byte Count
                                  6'b0,           // Rsvd
                                  2'b0,           // Address Type
                                  1'b0,           // Rsvd
                                  7'b0 };         // Starting Address of the Mem Byte
      //-----------------------------------------------------------------------\\
      pcie_tlp_data  <= #(Tcq)    {
                           1'b0,
                           2'b00,
                           5'b01010,
                           1'b0,
                           tc_,
                           4'b0000,
                           1'b0,
                           1'b0,
                           2'b00,
                           2'b00,
                           len_[9:0],             // 32

                           RP_BUS_DEV_FNS,
                           comp_status_,
                           1'b0,
                           12'b0,            //64

                           req_id_,
                           tag_,
                           8'b00,            //96

                           32'b0             //128
                           };
      pcie_tlp_rem         <= #(Tcq)    2'b01;

      TSK_TX_SYNCHRONIZE(1, 1, 1, `SYNC_CC_RDY);

      m_axis_rc_tvalid <= #(Tcq) 1'b0;
      m_axis_rc_tlast  <= #(Tcq) 1'b0;
      m_axis_rc_tkeep  <= #(Tcq) 4'h0;
      m_axis_rc_tuser  <= #(Tcq) 60'b0;
      m_axis_rc_tdata  <= #(Tcq) 128'b0;

      pcie_tlp_rem   <= #(Tcq) 2'b00;

   end
endtask // TSK_TX_COMPLETION

/************************************************************
Task        : TSK_TX_COMPLETION_DATA
Inputs      : Tag, TC, Length, Completion ID
Outputs     : Transaction Tx Interface Signaling
Description : Generates a Completion TLP
*************************************************************/
task TSK_TX_COMPLETION_DATA;
   input    [15:0]   req_id_;
   input    [7:0]    tag_;
   input    [2:0]    tc_;
   input    [10:0]   len_;
   input    [11:0]   byte_count_;
   input    [6:0]    lower_addr_;
   input    [2:0]    comp_status_;
   input             ep_;

   reg    [10:0]     _len;
   reg    [10:0]     len_i;
   reg    [31:0]     data_axis_i;
   reg    [31:0]     data_pcie_i;
   integer           _j;

   begin
      //-----------------------------------------------------------------------\\
      data_axis_i = 0;
      data_pcie_i = 0;
      _len = len_;
      //-----------------------------------------------------------------------\\
      if (user_lnk_up_n) begin
         $display("[%t] :  interface is MIA", $realtime);
         $finish(1);
      end
      //-----------------------------------------------------------------------\\
      TSK_TX_SYNCHRONIZE(0, 0, 0, `SYNC_CC_RDY);
      //-----------------------------------------------------------------------\\
      m_axis_rc_tvalid         <= #(Tcq) 1'b1;

      data_axis_i  =  {
                       DATA_STORE[3],
                       DATA_STORE[2],
                       DATA_STORE[1],
                       DATA_STORE[0]
                       };

      data_pcie_i  =  {
                      DATA_STORE[0],
                      DATA_STORE[1],
                      DATA_STORE[2],
                      DATA_STORE[3]
                     };

      m_axis_rc_tuser <= #(Tcq) {(ATTR_AXISTEN_IF_CC_PARITY_CHECK ? m_axis_rc_tparity : 32'b0),1'b0};
      m_axis_rc_tdata <= #(Tcq) {data_axis_i,      // completion data first DW                  // 128
                                 1'b0,             // Force ECRC                                //96
                                 3'b0,             // Attr
                                 tc_,              //
                                 1'b1,             // Completer ID to Control Selection of Client
                                 RP_BUS_DEV_FNS,   //Bus, Device/Fun No
                                 tag_ ,

                                 req_id_,        // Requester ID                                //64
                                 1'b0,           // Rsvd
                                 1'b0,           // Posioned Completion
                                 comp_status_,   //SuccessFull Completion
                                 len_,           //DWORD Count

                                 2'b0,           // Rsvd                                        //32
                                 1'b0,           // Locked Read Completion
                                 1'b0,           // Byte Count MSB
                                 byte_count_,    // Byte Count
                                 6'b0,           // Rsvd
                                 2'b0,           // Address Type
                                 1'b0,           // Rsvd
                                 lower_addr_ };  // Starting Address of the Mem Byte
      //-----------------------------------------------------------------------\\
      pcie_tlp_data  <= #(Tcq) { 1'b0,
                                 2'b10,
                                 5'b01010,
                                 1'b0,
                                 tc_,
                                 4'b0000,
                                 1'b0,
                                 1'b0,
                                 2'b00,
                                 2'b00,
                                 len_[9:0],                        // 32

                                 RP_BUS_DEV_FNS,
                                 comp_status_,
                                 1'b0,
                                 byte_count_,                     // 64

                                 req_id_,
                                 tag_,
                                 1'b0,
                                 lower_addr_,                     //96

                                 data_pcie_i };// completion data first DW    // 128

          pcie_tlp_rem <= #(Tcq) 2'b00;
      //-----------------------------------------------------------------------\\
      if (_len > 1) begin
         len_i = len_ - 11'h1;
         m_axis_rc_tlast          <= #(Tcq) 1'b0;
         m_axis_rc_tkeep          <= #(Tcq) 4'hF;
         TSK_TX_SYNCHRONIZE(1, 1, 0, `SYNC_CC_RDY);
      end
      else begin
         len_i = len_;
         m_axis_rc_tlast          <= #(Tcq) 1'b1;
         if (_len == 1)
             m_axis_rc_tkeep      <= #(Tcq) 4'hF;
         TSK_TX_SYNCHRONIZE(1, 1, 1, `SYNC_CC_RDY);
      end
      //-----------------------------------------------------------------------\\
      if (_len > 1) begin
         for (_j = 4; _j < (_len * 4); _j = _j + 16) begin
            pcie_tlp_data <= #(Tcq) { DATA_STORE[_j + 0],
                                      DATA_STORE[_j + 1],
                                      DATA_STORE[_j + 2],
                                      DATA_STORE[_j + 3],
                                      DATA_STORE[_j + 4],
                                      DATA_STORE[_j + 5],
                                      DATA_STORE[_j + 6],
                                      DATA_STORE[_j + 7],
                                      DATA_STORE[_j + 8],
                                      DATA_STORE[_j + 9],
                                      DATA_STORE[_j + 10],
                                      DATA_STORE[_j + 11],
                                      DATA_STORE[_j + 12],
                                      DATA_STORE[_j + 13],
                                      DATA_STORE[_j + 14],
                                      DATA_STORE[_j + 15] };

            m_axis_rc_tdata   <= #(Tcq) { DATA_STORE[_j + 15],
                                          DATA_STORE[_j + 14],
                                          DATA_STORE[_j + 13],
                                          DATA_STORE[_j + 12],
                                          DATA_STORE[_j + 11],
                                          DATA_STORE[_j + 10],
                                          DATA_STORE[_j +  9],
                                          DATA_STORE[_j +  8],
                                          DATA_STORE[_j +  7],
                                          DATA_STORE[_j +  6],
                                          DATA_STORE[_j +  5],
                                          DATA_STORE[_j +  4],
                                          DATA_STORE[_j +  3],
                                          DATA_STORE[_j +  2],
                                          DATA_STORE[_j +  1],
                                          DATA_STORE[_j +  0] };

            if ((_j + 15)  >=  ((_len * 4) - 1)) begin
               if (ep_ == 1'b0) begin
                  case ((_len - 11'h1) % 4)
                     1 : begin len_i = len_i - 1; pcie_tlp_rem   <= #(Tcq) 2'b11; end  // D0---------
                     2 : begin len_i = len_i - 2; pcie_tlp_rem   <= #(Tcq) 2'b10; end  // D0-D1------
                     3 : begin len_i = len_i - 3; pcie_tlp_rem   <= #(Tcq) 2'b01; end  // D0-D1-D2---
                     0 : begin len_i = len_i - 4; pcie_tlp_rem   <= #(Tcq) 2'b00; end  // D0-D1-D2-D3
                  endcase 
               end 
            end
            else begin 
               len_i = len_i - 4; 
               pcie_tlp_rem   <= #(Tcq) 2'b00; 
               m_axis_rc_tkeep <= #(Tcq) 4'hF; 
            end  // D0-D1-D2-D3

            if (len_i == 0) begin
               m_axis_rc_tlast <= #(Tcq) 1'b1;
               TSK_TX_SYNCHRONIZE(0, 1, 1, `SYNC_CC_RDY);
            end
            else
               TSK_TX_SYNCHRONIZE(0, 1, 0, `SYNC_CC_RDY);
         end
      end
      //-----------------------------------------------------------------------\\
      m_axis_rc_tvalid <= #(Tcq) 1'b0;
      m_axis_rc_tlast  <= #(Tcq) 1'b0;
      m_axis_rc_tkeep  <= #(Tcq) 4'h0;
      m_axis_rc_tuser  <= #(Tcq) 60'b0;
      m_axis_rc_tdata  <= #(Tcq) 128'b0;
      //-----------------------------------------------------------------------\\
      pcie_tlp_rem         <= #(Tcq) 0;
      //-----------------------------------------------------------------------\\
   end
endtask // TSK_TX_COMPLETION_DATA


endmodule