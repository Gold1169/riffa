`include "trellis.vh"
`include "riffa.vh"
`include "ultrascale.vh"
`include "functions.vh"
`timescale 1ps / 1ps

module tb_top;
     
   localparam C_FPGA_NAME = "REGT"; // This is not yet exposed in the driver
   localparam C_MAX_READ_REQ_BYTES = C_MAX_PAYLOAD_BYTES * 2;
   // ALTERA, XILINX or ULTRASCALE
   localparam C_VENDOR = "ULTRASCALE";
   localparam C_KEEP_WIDTH = C_PCI_DATA_WIDTH / 32;
   localparam C_PIPELINE_OUTPUT = 1;
   localparam C_PIPELINE_INPUT = 1;
   localparam C_DEPTH_PACKETS = 4;

   logic                                              user_clk;
   logic                                              user_reset;
   logic                                              rst_out;
   
   wire                                               clk;
   wire                                               rst_in;

   logic                                              s_axis_cc_tvalid;
   logic                                              s_axis_cc_tlast;
   logic    [C_PCI_DATA_WIDTH-1:0]                    s_axis_cc_tdata;
   logic    [(C_PCI_DATA_WIDTH/32)-1:0]               s_axis_cc_tkeep;
   logic    [`SIG_CC_TUSER_W-1:0]                     s_axis_cc_tuser;
   logic                                              s_axis_cc_tready;

   logic                                              s_axis_rq_tvalid;
   logic                                              s_axis_rq_tlast;
   logic    [C_PCI_DATA_WIDTH-1:0]                    s_axis_rq_tdata;
   logic    [(C_PCI_DATA_WIDTH/32)-1:0]               s_axis_rq_tkeep;
   logic    [`SIG_RQ_TUSER_W-1:0]                     s_axis_rq_tuser;
   logic                                              s_axis_rq_tready;

   logic                                              m_axis_cq_tvalid;
   logic                                              m_axis_cq_tlast;
   logic    [C_PCI_DATA_WIDTH-1:0]                    m_axis_cq_tdata;
   logic    [(C_PCI_DATA_WIDTH/32)-1:0]               m_axis_cq_tkeep;
   logic    [`SIG_CQ_TUSER_W-1:0]                     m_axis_cq_tuser;
   logic                                              m_axis_cq_tready;

   logic                                              m_axis_rc_tvalid;
   logic                                              m_axis_rc_tlast;
   logic    [C_PCI_DATA_WIDTH-1:0]                    m_axis_rc_tdata;
   logic    [(C_PCI_DATA_WIDTH/32)-1:0]               m_axis_rc_tkeep;
   logic    [`SIG_RC_TUSER_W-1:0]                     m_axis_rc_tuser;
   logic                                              m_axis_rc_tready;

   logic    [3:0]                                     cfg_interrupt_int;
   logic    [1:0]                                     cfg_interrupt_pending;
   logic    [3:0]                                     cfg_interrupt_msi_select;
   logic    [31:0]                                    cfg_interrupt_msi_int;
   logic    [63:0]                                    cfg_interrupt_msi_pending_status;
   logic    [2:0]                                     cfg_interrupt_msi_attr;
   logic                                              cfg_interrupt_msi_tph_present;
   logic    [1:0]                                     cfg_interrupt_msi_tph_type;
   logic    [8:0]                                     cfg_interrupt_msi_tph_st_tag;
   logic    [2:0]                                     cfg_interrupt_msi_function_number;
   logic    [2:0]                                     cfg_fc_sel;

   logic    [1:0]                                     cfg_interrupt_msi_enable;
   logic                                              cfg_interrupt_msi_mask_update;
   logic    [31:0]                                    cfg_interrupt_msi_data;
   logic                                              cfg_interrupt_msi_sent;
   logic                                              cfg_interrupt_msi_fail;
   logic    [7:0]                                     cfg_fc_cplh;
   logic    [11:0]                                    cfg_fc_cpld;
   logic    [3:0]                                     cfg_negotiated_width;
   logic    [2:0]                                     cfg_current_speed;
   logic    [2:0]                                     cfg_max_payload;
   logic    [2:0]                                     cfg_max_read_req;
   logic    [7:0]                                     cfg_function_status;
   logic    [1:0]                                     cfg_rcb_status;

   logic                                              pcie_cq_np_req;

   logic    [C_NUM_CHNL-1:0]                          chnl_rx;
   logic    [C_NUM_CHNL-1:0]                          chnl_rx_clk;
   logic    [C_NUM_CHNL-1:0]                          chnl_rx_ack;
   logic    [C_NUM_CHNL-1:0]                          chnl_rx_last;
   logic    [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]     chnl_rx_len;
   logic    [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]     chnl_rx_off;
   logic    [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]       chnl_rx_data;
   logic    [C_NUM_CHNL-1:0]                          chnl_rx_data_valid;
   logic    [C_NUM_CHNL-1:0]                          chnl_rx_data_ren;

   logic    [C_NUM_CHNL-1:0]                          chnl_tx_clk;
   logic    [C_NUM_CHNL-1:0]                          chnl_tx;
   logic    [C_NUM_CHNL-1:0]                          chnl_tx_ack;
   logic    [C_NUM_CHNL-1:0]                          chnl_tx_data_ren;
   logic    [C_NUM_CHNL-1:0]                          chnl_tx_last;
   logic    [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]     chnl_tx_len;
   logic    [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]     chnl_tx_off;
   logic    [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]       chnl_tx_data;
   logic    [C_NUM_CHNL-1:0]                          chnl_tx_data_valid;


   if (interrupt_done)
      cfg_interrupt_msi_sent = 1;
      cfg_interrupt_msi_fail = 0;

   pcie_host_tx #( .ATTR_AXISTEN_IF_ENABLE_CLIENT_TAG  ( 0                   )
                   .ATTR_AXISTEN_IF_RQ_PARITY_CHECK    ( 0                   )
                   .ATTR_AXISTEN_IF_CC_PARITY_CHECK    ( 0                   )
                   .AXISTEN_IF_RQ_ALIGNMENT_MODE       ( "FALSE"             )
                   .AXISTEN_IF_CC_ALIGNMENT_MODE       ( "FALSE"             )
                   .AXISTEN_IF_CQ_ALIGNMENT_MODE       ( "FALSE"             )
                   .AXISTEN_IF_RC_ALIGNMENT_MODE       ( "FALSE"             )

                   .DEV_CAP_MAX_PAYLOAD_SUPPORTED      (  1                  )
                   .C_DATA_WIDTH                       ( 128)
                   .KEEP_WIDTH                         ( C_DATA_WIDTH / 32   )
                   .STRB_WIDTH                         ( C_DATA_WIDTH / 8    )
                   .LINK_CAP_MAX_LINK_WIDTH            ( 4'h8                )
                   .LINK_CAP_MAX_LINK_SPEED            ( 3'h2                )
                   .EP_DEV_ID                          ( 16'h7700            )
                   .REM_WIDTH                          ( ((C_DATA_WIDTH == 256) ? 3 : ((C_DATA_WIDTH == 128) ? 2 : 1))))
   pcie_host_tx (
      .user_clk                        (  ),
      .reset                           (  ),
      .user_lnk_up                     (  ),

      .m_axis_cq_tvalid                ( m_axis_cq_tuser    ),
      .m_axis_cq_tdata                 ( m_axis_cq_tdata    ),
      .m_axis_cq_tlast                 ( m_axis_cq_tlast    ),
      .m_axis_cq_tuser                 ( m_axis_cq_tvalid   ),
      .m_axis_cq_tkeep                 ( m_axis_cq_tkeep    ),
      .m_axis_cq_tready                ( m_axis_cq_tready   ),

      .m_axis_rc_tvalid                ( m_axis_rc_tuser    ),
      .m_axis_rc_tdata                 ( m_axis_rc_tvalid   ),
      .m_axis_rc_tlast                 ( m_axis_rc_tdata    ),
      .m_axis_rc_tkeep                 ( m_axis_rc_tkeep    ),
      .m_axis_rc_tuser                 ( m_axis_rc_tlast    ),
      .m_axis_rc_tready                ( m_axis_rc_tready   ),

      .pcie_rq_seq_num                 (  ),
      .pcie_rq_seq_num_vld             (  ),
      .pcie_rq_tag                     (  ),
      .pcie_rq_tag_vld                 (  ),

      .pcie_tfc_nph_av                 (  ),
      .pcie_tfc_npd_av                 (  ),

      .speed_change_done_n             (  )
   );

   pcie_host_rx #( .C_DATA_WIDTH                 ( 64     )
                   .AXISTEN_IF_RQ_ALIGNMENT_MODE ( "FALSE")
                   .AXISTEN_IF_CC_ALIGNMENT_MODE ( "FALSE")
                   .AXISTEN_IF_CQ_ALIGNMENT_MODE ( "FALSE")
                   .AXISTEN_IF_RC_ALIGNMENT_MODE ( "FALSE")
                   .STRB_WIDTH                   ( C_DATA_WIDTH / 8 ), // TSTRB width
                   .KEEP_WIDTH                   ( C_DATA_WIDTH / 32),
                   .PARITY_WIDTH                 ( C_DATA_WIDTH / 8 ) );  // TPARITY width 
   pcie_host_rx(
      .user_clk                  (  ),
      .user_reset                (  ),
      .user_lnk_up               (  ),

      .s_axis_rq_tvalid          ( s_axis_rq_tvalid ),
      .s_axis_rq_tdata           ( s_axis_rq_tdata  ),
      .s_axis_rq_tlast           ( s_axis_rq_tlast  ),
      .s_axis_rq_tkeep           ( s_axis_rq_tkeep  ),
      .s_axis_rq_tuser           ( s_axis_rq_tuser  ),
      .s_axis_rq_tready          ( s_axis_rq_tready ),

      .s_axis_cc_tvalid          ( s_axis_cc_tvalid ),
      .s_axis_cc_tdata           ( s_axis_cc_tdata  ),
      .s_axis_cc_tlast           ( s_axis_cc_tlast  ),
      .s_axis_cc_tkeep           ( s_axis_cc_tkeep  ),
      .s_axis_cc_tuser           ( s_axis_cc_tuser  ),
      .s_axis_cc_tready          ( s_axis_cc_tready ),

      .pcie_cq_np_req_count      (  ),
      .pcie_cq_np_req            (  )
   );

   pcie_host_com_task com_task   ();

   riffa_wrapper_vc709 #(
         .C_NUM_CHNL             ( 1      ), // Number of RIFFA Channels
         .C_PCI_DATA_WIDTH       ( 128    ), // Bit-Width from Vivado IP Generator
         .C_MAX_PAYLOAD_BYTES    ( 25     ), // 4-Byte Name for this FPGA
         .C_LOG_NUM_TAGS         ( 5      ),
         .C_FPGA_ID              ( "V709" )) 
   dut_riffa(
      .USER_CLK                            (),
      .USER_RESET                          (),

      //Interface: CQ Ultrascale (RXR)
      .M_AXIS_CQ_TVALID                    ( m_axis_cq_tvalid   ),
      .M_AXIS_CQ_TDATA                     ( m_axis_cq_tdata    ),
      .M_AXIS_CQ_TLAST                     ( m_axis_cq_tlast    ),
      .M_AXIS_CQ_TKEEP                     ( m_axis_cq_tkeep    ),
      .M_AXIS_CQ_TUSER                     ( m_axis_cq_tuser    ),
      .M_AXIS_CQ_TREADY                    ( m_axis_cq_tready   ),

      .M_AXIS_RC_TVALID                    ( m_axis_rc_tvalid   ),
      .M_AXIS_RC_TDATA                     ( m_axis_rc_tdata    ),
      .M_AXIS_RC_TLAST                     ( m_axis_rc_tlast    ),
      .M_AXIS_RC_TKEEP                     ( m_axis_rc_tkeep    ),
      .M_AXIS_RC_TUSER                     ( m_axis_rc_tuser    ),
      .M_AXIS_RC_TREADY                    ( m_axis_rc_tready   ),

      .S_AXIS_CC_TVALID                    ( s_axis_cc_tvalid   ),
      .S_AXIS_CC_TDATA                     ( s_axis_cc_tdata   ),
      .S_AXIS_CC_TLAST                     ( s_axis_cc_tlast   ),
      .S_AXIS_CC_TKEEP                     ( s_axis_cc_tkeep   ),
      .S_AXIS_CC_TUSER                     ( s_axis_cc_tuser  ),
      .S_AXIS_CC_TREADY                    ( s_axis_cc_tready  ),

      .S_AXIS_RQ_TVALID                    ( s_axis_rq_tvalid  ), // o TXR
      .S_AXIS_RQ_TDATA                     ( s_axis_rq_tdata   ), // o TXR
      .S_AXIS_RQ_TLAST                     ( s_axis_rq_tlast    ), // o TXR
      .S_AXIS_RQ_TKEEP                     ( s_axis_rq_tkeep   ), // o TXR
      .S_AXIS_RQ_TUSER                     ( s_axis_rq_tuser  ), // o TXR
      .S_AXIS_RQ_TREADY                    ( s_axis_rq_tready  ), // o TXR

      .CFG_INTERRUPT_MSI_ENABLE            ( 1 ),
      .CFG_INTERRUPT_MSI_MASK_UPDATE       (),
      .CFG_INTERRUPT_MSI_SENT              ( cfg_interrupt_msi_sent ),
      .CFG_INTERRUPT_MSI_FAIL              ( cfg_interrupt_msi_fail ),
      .CFG_INTERRUPT_MSI_DATA              (),

      .CFG_FC_CPLH                         (),
      .CFG_FC_CPLD                         (),
      .CFG_FC_SEL                          (),

      .CFG_NEGOTIATED_WIDTH                ( 4'b1000 ), // CONFIG_LINK_WIDTH
      .CFG_CURRENT_SPEED                   ( 3'b001 ), // CONFIG_LINK_RATE
      .CFG_MAX_PAYLOAD                     ( 3'b010 ), // CONFIG_MAX_PAYLOAD 512BYTE
      .CFG_MAX_READ_REQ                    ( 3'b010 ), // CONFIG_MAX_READ_REQUEST
      .CFG_FUNCTION_STATUS                 ( 8'b0000_1111 ), // [2] = CONFIG_BUS_MASTER_ENABLE
      .CFG_RCB_STATUS                      (),

      .CFG_INTERRUPT_INT                   ( cfg_interrupt_int ),
      .CFG_INTERRUPT_PENDING               (),
      .CFG_INTERRUPT_MSI_SELECT            (),
      .CFG_INTERRUPT_MSI_INT               (),
      .CFG_INTERRUPT_MSI_PENDING_STATUS    ( cfg_interrupt_msi_pending_status ),
      .CFG_INTERRUPT_MSI_ATTR              (),
      .CFG_INTERRUPT_MSI_TPH_PRESENT       (),
      .CFG_INTERRUPT_MSI_TPH_TYPE          (),
      .CFG_INTERRUPT_MSI_TPH_ST_TAG        (),
      .CFG_INTERRUPT_MSI_FUNCTION_NUMBER   (),

      .PCIE_CQ_NP_REQ                      (),

        // RIFFA Interface Signals
      .RST_OUT                             (  ),
      .CHNL_RX_CLK                         ( chnl_rx_clk        ), // Channel read clock
      .CHNL_RX                             ( chnl_rx            ), // Channel read receive signal
      .CHNL_RX_ACK                         ( chnl_rx_ack        ), // Channel read received signal
      .CHNL_RX_LAST                        ( chnl_rx_last       ), // Channel last read
      .CHNL_RX_LEN                         ( chnl_rx_len        ), // Channel read length
      .CHNL_RX_OFF                         ( chnl_rx_off        ), // Channel read offset
      .CHNL_RX_DATA                        ( chnl_rx_data       ), // Channel read data
      .CHNL_RX_DATA_VALID                  ( chnl_rx_data_valid ), // Channel read data valid
      .CHNL_RX_DATA_REN                    ( chnl_rx_data_ren   ), // Channel read data has been recieved

      .CHNL_TX_CLK                         ( chnl_tx_clk        ), // Channel write clock
      .CHNL_TX                             ( chnl_tx            ), // Channel write receive signal
      .CHNL_TX_ACK                         ( chnl_tx_ack        ), // Channel write acknowledgement signal
      .CHNL_TX_LAST                        ( chnl_tx_last       ), // Channel last write
      .CHNL_TX_LEN                         ( chnl_tx_len        ), // Channel write length (in 32 bit words)
      .CHNL_TX_OFF                         ( chnl_tx_off        ), // Channel write offset
      .CHNL_TX_DATA                        ( chnl_tx_data       ), // Channel write data
      .CHNL_TX_DATA_VALID                  ( chnl_tx_data_valid ), // Channel write data valid
      .CHNL_TX_DATA_REN                    ( chnl_tx_data_ren   )  // Channel write data has been recieved
   );


generate
   for (chnl = 0; chnl < C_NUM_CHNL; chnl = chnl + 1) begin : test_channels
      chnl_tester #( .C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH) ) 
      module1  (
         .CLK(user_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK               (chnl_rx_clk        [chnl]                                        ), 
         .CHNL_RX                   (chnl_rx            [chnl]                                        ), 
         .CHNL_RX_ACK               (chnl_rx_ack        [chnl]                                        ), 
         .CHNL_RX_LAST              (chnl_rx_last       [chnl]                                        ), 
         .CHNL_RX_LEN               (chnl_rx_len        [32*chnl +:32]                                ), 
         .CHNL_RX_OFF               (chnl_rx_off        [31*chnl +:31]                                ), 
         .CHNL_RX_DATA              (chnl_rx_data       [C_PCI_DATA_WIDTH*chnl +:C_PCI_DATA_WIDTH]    ), 
         .CHNL_RX_DATA_VALID        (chnl_rx_data_valid [chnl]                                        ), 
         .CHNL_RX_DATA_REN          (chnl_rx_data_ren   [chnl]                                        ),
         // Tx interface
         .CHNL_TX_CLK               (chnl_tx_clk        [chnl]                                        ), 
         .CHNL_TX                   (chnl_tx            [chnl]                                        ), 
         .CHNL_TX_ACK               (chnl_tx_ack        [chnl]                                        ), 
         .CHNL_TX_LAST              (chnl_tx_last       [chnl]                                        ), 
         .CHNL_TX_LEN               (chnl_tx_len        [32*chnl +:32]                                ), 
         .CHNL_TX_OFF               (chnl_tx_off        [31*chnl +:31]                                ), 
         .CHNL_TX_DATA              (chnl_tx_data       [C_PCI_DATA_WIDTH*chnl +:C_PCI_DATA_WIDTH]    ), 
         .CHNL_TX_DATA_VALID        (chnl_tx_data_valid [chnl]                                        ), 
         .CHNL_TX_DATA_REN          (chnl_tx_data_ren   [chnl]                                        )
      );
   end
endgenerate

   // 定义1个二维数组模拟host内存，用4byte模拟host内存中的1页，
   // 假设一次传输加入为10byte，则需要跨3页，10/4=2余2，此时需要3个entry，即生成3个描述符写入sg_buffer
   logic [31:0] mem_array [0:4095]; 

   //---------------------------------------------------------------------
   // interrup_model
   //---------------------------------------------------------------------
   task proc_interrupt;
      input interrupt;
      input [3:0] chnl_no
      output proc_done;

      // 5种消息，
      // - TX_TXN，FPGA发送新的transaction
      // - TX_SG_BUF_RECVD，TX sg buffer 用尽
      // - TX_TXN_DONE，tx transaction 完成
      // - RX_SG_BUF_RECVD，RX sg buffer 用尽
      // - RX_TXN_DONE，RX transaction 完成
      bit[31:0] TXN_OFFSET[$];
      bit[31:0] TXN_LEN[$];
      bit[31:0] TX_TXN_DONE[$];
      bit[31:0] RX_TXN_DONE[$];
      bit       TX_SG_BUF_RECVD[$];
      bit       RX_SG_BUF_RECVD[$];


      if(interrupt) begin
         read_reg(ADDR_INTR_VECTOR_0, vect0);
         read_reg(ADDR_INTR_VECTOR_1, vect1);
      end

      if(chon_no<6) begin
         intr[0] <= vect0[4:0];
         intr[1] <= vect0[9:5];
         intr[2] <= vect0[14:10];
         intr[3] <= vect0[19:15];
         intr[4] <= vect0[24:20];
         intr[5] <= vect0[29:25];
      end
      else begin
         intr[0] <= vect1[4:0];
         intr[1] <= vect1[9:5];
         intr[2] <= vect1[14:10];
         intr[3] <= vect1[19:15];
         intr[4] <= vect1[24:20];
         intr[5] <= vect1[29:25];
      end

      for (i = 0; i<6; i++ ) begin
         case( intr[i] )
            5'b00001:begin   // TX_TXN New TX (PC receive) transaction.
               read_reg({chnl_no,TX_OFFLAST_REG_OFF}, offset);
               read_reg({chnl_no, TX_LEN_REG_OFF}, len);
               TXN_OFFSET.push_back(offset ) ;
               TXN_LEN.push_back(len ) ;
            end
            5'b00010:begin   // TX_SG_BUF_RECVD
               TX_SG_BUF_RECVD.push_back( counter_tx );
               counter_tx++;
            end
            5'b00100:begin   // TX_TXN_DONE  TX (PC receive) transaction done.
               len = read_reg({chnl_no, TX_TNFR_LEN_REG_OFF}, len);
               TX_TXN_DONE.push_back(len ) ;
            end
            5'b01000:begin   // RX_SG_BUF_RECVD
               RX_SG_BUF_RECVD.push_back( counter_rx);
               counter_rx++;
            end 
            5'b10000:begin   // RX_TXN_DONE 
               len = read_reg({chnl_no, RX_TNFR_LEN_REG_OFF}, len);
               RX_TXN_DONE.push_back(len) ;
            end
         endcase
      end
   endtask

   // 从FPGA接收数据:FPGA TX
   task driver_recv; 
      input       interrupt;
      input [3:0] chnl_no;

      proc_interrupt(interrupt, chnl_no);
      // 定义两个二维数组模拟host内存，用4byte模拟host内存中的1页，
      // 假设一次传输加入为10byte，则需要跨3页，10/4=2余2，此时需要3个entry，即生成3个描述如写入sg_buffer
      logic [31:0] addr=4; 

      while(1) begin
         if( !TXN_OFFSET.empty()) begin 
            // Read the offset and last flags (always before reading length)
            {offset, last} = TXN_OFFSET.pop_front();
         end
         if( !TXN_LEN.empty()) begin
            // Read the length
            offset = 0;
            length = TXN_LEN.pop_front(); // 32bit为单位
            len_byte = length/4; // byte为单位
            // 根据FPGA要发送的长度申请一段地址，把这个地址以及偏移写进sg_buffer
            mem_pages = len_byte / 4 + (len_byte % 4 != 0);
            offset = (len_byte + offset) % 4;
            mem_array[0]=0;// TODO：更新地址
            mem_array[1]=0; // mem[0][1]用作sg_buf。
            addr = addr + mem_pages + offset;
            // 获取一个内存地址，用在保存sg_buffer的地址: mem_array1:
            // (TX_SG_ADDR_LO_REG_OFF),12'h001(TX_SG_ADDR_HI_REG_OFF),
            // 12'h002(RX_SG_ADDR_LO_REG_OFF),12'h003(RX_SG_ADDR_HI_REG_OFF)
            //（在仿真中这个地址可以是一个固定值（地址0），在真实的驱动中，每次启动后，在初始化设备的过程中分配一个地址）
            // Use the recv common buffer to share the scatter gather elements.
            write_reg({chnl_no, TX_SG_ADDR_LO_REG_OFF}, 32'h0000_0000); // 告诉FPGA TX_SG_buffer 的低32bit地址
            write_reg({chnl_no, TX_SG_ADDR_HI_REG_OFF}, 32'h0000_0001); // 告诉FPGA TX_SG_buffer 的32高bit地址
            write_reg({chnl_no, TX_SG_LEN_REG_OFF}, mem_pages); // 告诉FPGA本次传输需要读取几个entry，TODO确认这个值的大小
         end
         if( !TX_SG_BUF_RECVD.empty()) begin 
               // Ignore if we haven't received offlast/len.
               if (last == -1)
                  break;
            // Read the length
            offset = 0;
            length = TXN_LEN.pop_front(); // 32bit为单位
            len_byte = length/4; // byte为单位
            // 根据FPGA要发送的长度申请一段地址，把这个地址以及偏移写进sg_buffer
            mem_page = len_byte / 4 + (len_byte % 4 != 0);
            offset = (len_byte + offset) % 4;
            mem_array[0]=addr;
            mem_array[1]=0; // 仿真平台中申请的二维数组比较小，所以高位地址用不到，固定为0。
            // Use the recv common buffer to share the scatter gather elements.
            write_reg({chon_no, TX_SG_ADDR_LO_REG_OFF}, 32'h0000_0000);
            write_reg({chon_no, TX_SG_ADDR_HI_REG_OFF}, 32'h0000_0001);
            write_reg({chon_no, TX_SG_LEN_REG_OFF}, 32'd64);// 告诉FPGA本次传输需要读取几个entry，TODO确认这个值的大小
         end
         if( !TX_TXN_DONE.empty()) begin
            // Ignore if we haven't received offlast/len.
            if (last == -1)
               break;
            // Update with the true value of words transferred.
            recvd_len_word = TX_TXN_DONE.pop_front();
            recvd_len_byte = recvd_len_word<<2;
            if (last)
               break;
         end
      end
   endtask

   // 向FPGA发送数据: FPGA RX
   task driver_send; 
      input        interrupt;
      input [3:0]  chnl_no;
      input [31:0] addr; // 要发送的数据的地址
      input [31:0] len; // 要发送的数据长度

      // Let FPGA know about transfer.
      // 当前的tansfer在地址中的偏移，是否是last，需要传输的长度（单位为word）。分别写入相应寄存器。
      write_reg({chnl_no, RX_OFFLAST_REG_OFF}, ((offset<<1) | last));
      write_reg({chnl_no, RX_LEN_REG_OFF}, len);

      // Let FPGA know about the scatter gather buffer.
      // 下发buffer地址以及buffer长度。
      write_reg({chnl, RX_SG_ADDR_LO_REG_OFF}, 32'h0000_0002);
      write_reg({chnl, RX_SG_ADDR_HI_REG_OFF}, 32'h0000_0003);
      write_reg({chnl, RX_SG_LEN_REG_OFF}, );

      proc_interrupt(interrupt, chnl_no);

      offset = 0;
      length = len;
      len_byte = length/4; // byte为单位
      //把要发送的数据的地址以及偏移写进sg_buffer
      mem_page = len_byte / 4 + (len_byte % 4 != 0);
      offset = (len_byte + offset) % 4;
      mem_array[2]=addr;
      mem_array[3]=0; // 仿真平台中申请的二维数组比较小，所以高位地址用不到，固定为0。

      // Use the recv common buffer to share the scatter gather elements.
      write_reg({chnl_no, RX_SG_ADDR_LO_REG_OFF}, 32'h0000_0000);
      write_reg({chnl_no, RX_SG_ADDR_HI_REG_OFF}, 32'h0000_0001);
      write_reg({chnl_no, RX_SG_LEN_REG_OFF}, 32'd64); // 告诉FPGA本次传输需要读取几个entry，TODO确认这个值的大小

      while(1) begin
         if( !RX_SG_BUF_RECVD.empty()) begin 
               // Ignore if we haven't received offlast/len.
               if (last == -1)
                  break;
            // Read the length
            offset = 0;
            length = TXN_LEN.pop_front(); // 32bit为单位
            len_byte = length/4; // byte为单位
            // 根据FPGA要发送的长度申请一段地址，把这个地址以及偏移写进sg_buffer
            mem_page = len_byte / 4 + (len_byte % 4 != 0);
            offset = (len_byte + offset) % 4;
            mem_array[0]=addr;
            mem_array[1]=0; // 仿真平台中申请的二维数组比较小，所以高位地址用不到，固定为0。
            // Use the recv common buffer to share the scatter gather elements.
            write_reg({chnl_no, RX_SG_ADDR_LO_REG_OFF}, 32'h0000_0000);
            write_reg({chnl_no, RX_SG_ADDR_HI_REG_OFF}, 32'h0000_0001);
            write_reg({chnl_no, RX_SG_LEN_REG_OFF}, 32'd64); // 告诉FPGA本次传输需要读取几个entry，TODO确认这个值的大小
         end
         if( !RX_TXN_DONE.empty()) begin
            // Ignore if we haven't received offlast/len.
            if (last == -1)
               break;
            // Update with the true value of words transferred.
            sent_len_word = RX_TXN_DONE.pop_front();
            sent_len_byte = sent_len_word<<2;
            if (last)
               break;
         end
      end

      // // 向FPGA发送数据，先发送数据所在地址，等待FPGA返回memrd请求，再通过DMA返回数据。
      // // 等待来自FPGA的请求数据的请求，并返回需要返回的数据。
      // pcie_host_rx(); // 通过RX模块监控所接收到的所有pcie数据。

      // wait_for_next = 1'b1; //haven't found any matching tag yet

      // while(wait_for_next) begin
      //    @ rcvd_memrd64; //wait for a rcvd_memrd64 event
      //    traffic_class_ = frame_store_rx[1] >> 4;
      //    td_ = frame_store_rx[2] >> 7;
      //    ep_ = frame_store_rx[2] >> 6;
      //    attr_ = frame_store_rx[2] >> 4;
      //    length_ = frame_store_rx[2];
      //    length_ = (length_ << 8) | (frame_store_rx[3]);
      //    requester_id_= {frame_store_rx[4], frame_store_rx[5]};
      //    tag_= frame_store_rx[6];
      //    last_dw_be_= frame_store_rx[7] >> 4;
      //    first_dw_be_= frame_store_rx[7];
      //    address_[61:6] = {frame_store_rx[8],  frame_store_rx[9],
      //                      frame_store_rx[10], frame_store_rx[11],
      //                      frame_store_rx[12], frame_store_rx[13],
      //                      frame_store_rx[14]};
      //    address_[5:0] = frame_store_rx[15] >> 2;

      //    $display("[%t] : Received MEMRD64 --- Tag 0x%h", $realtime, tag_);
      //    if(tag == tag_) begin //find matching tag
      //       wait_for_next = 1'b0;
      //       pcie_host_tx.TSK_TX_COMPLETION_DATA(requester_id_, tag_, traffic_class_, length_, byte_count_, address_[5:0], comp_status_, ep_);
      //    end
      // end
      

   endtask

   task write_reg;
      input [63:0]   address;
      input [31:0]   wr_data;

      input [2:0]    chnl_no;

      begin
         pcie_host_tx.TSK_TX_MEMORY_WRITE_64(0, 0, 1, address, 4'h0, 4'hF, 0);
         pcie_host_tx.DATA_STORE[0] = wr_data[7:0];
         pcie_host_tx.DATA_STORE[1] = wr_data[15:8];
         pcie_host_tx.DATA_STORE[2] = wr_data[23:16];
         pcie_host_tx.DATA_STORE[3] = wr_data[31:24];

         pcie_host_tx.TSK_TX_CLK_EAT(100);
         pcie_host_tx.DEFAULT_TAG = pcie_host_tx.DEFAULT_TAG + 1;
      end
   endtask

   //--------------------------------------------------------------------------
   // task read_reg
   // 发送读取寄存器的请求-->fpga 收到请求，返回寄存器值--> pcie_host_rx收到cc包，
   // 并调用TSK_PARSE_FRAME解析包--> 返回的完成包会触发rcvd_cpl event --> 
   // 监测rcvd_cpl event，比较tag，如果和发送tag一致，则提取payload，赋值给rd_data
   //--------------------------------------------------------------------------

   task read_reg;
      input  [63:0]   address;
      output [31:0]   rd_data;

      begin
         pcie_host_tx.TSK_TX_MEMORY_READ_64(0, 0, 1, address, 4'h0, 4'hF, 0);
         pcie_host_tx.TSK_TX_CLK_EAT(100);
         pcie_host_tx.DEFAULT_TAG = pcie_host_tx.DEFAULT_TAG + 1;

         wait_for_next = 1'b1; //haven't found any matching tag yet
         while(wait_for_next) begin
            @ rcvd_cpld; //wait for a rcvd_cpld event
            // traffic_class_ = frame_store_rx[1] >> 4;
            // td_ = frame_store_rx[2] >> 7;
            // ep_ = frame_store_rx[2] >> 6;
            // attr_ = frame_store_rx[2] >> 4;
            // length_ = frame_store_rx[2];
            // length_ = (length_ << 8) | (frame_store_rx[3]);
            // bcm_ = frame_store_rx[6] >> 4;
            // completion_status_= frame_store_rx[6] >> 5;
            // byte_count_ = (frame_store_rx[6]);
            // byte_count_ = (byte_count_ << 8) | frame_store_rx[7];
            // completer_id_ = {frame_store_rx[4], frame_store_rx[5]};
            // requester_id_= {frame_store_rx[8], frame_store_rx[9]};
            tag_= frame_store_rx[10];
            // address_low_ = frame_store_rx[11];

            payload_len = (bcm_) ? byte_count_ : (length << 2);
            if (payload_len==0) 
               payload_len = 4096;

            $display("[%t] : Received CPLD --- Tag 0x%h", $realtime, tag_);
            if(pcie_host_tx.DEFAULT_TAG == tag_) begin//find matching tag
               wait_for_next = 1'b0;
               for (i_ = 12; i_ < payload_len; i_ = i_ + 1)
                  rd_data[i*7+:8] = frame_store_rx[12 + i_];
            end
         end
      end
   endtask



endmodule