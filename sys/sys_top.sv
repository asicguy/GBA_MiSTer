//============================================================================
//
//  MiSTer hardware abstraction module
//  (c)2017-2020 Alexey Melnikov
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

`ifndef ARCADE_SYS
        `define USE_DDRAM
        `define USE_SDRAM
`endif

module sys_top
  #
  (
   parameter FPGA_FAMILY = "CYCLONEV"
   )
  (
   /////////// CLOCK //////////
   input         FPGA_CLK1_50,
   input         FPGA_CLK2_50,
   input         FPGA_CLK3_50,

   //////////// HDMI //////////
   output        HDMI_I2C_SCL,
   inout         HDMI_I2C_SDA,

   output        HDMI_MCLK,
   output        HDMI_SCLK,
   output        HDMI_LRCLK,
   output        HDMI_I2S,

   output        HDMI_TX_CLK,
   output        HDMI_TX_DE,
   output [23:0] HDMI_TX_D,
   output        HDMI_TX_HS,
   output        HDMI_TX_VS,

   input         HDMI_TX_INT,

   //////////// SDR ///////////
   output [12:0] SDRAM_A,
   inout [15:0]  SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nWE,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nCS,
   output [1:0]  SDRAM_BA,
   output        SDRAM_CLK,
   output        SDRAM_CKE,

`ifdef DUAL_SDRAM
   ////////// SDR #2 //////////
   output [12:0] SDRAM2_A,
   inout [15:0]  SDRAM2_DQ,
   output        SDRAM2_nWE,
   output        SDRAM2_nCAS,
   output        SDRAM2_nRAS,
   output        SDRAM2_nCS,
   output [1:0]  SDRAM2_BA,
   output        SDRAM2_CLK,

`else
   //////////// VGA ///////////
   output [5:0]  VGA_R,
   output [5:0]  VGA_G,
   output [5:0]  VGA_B,
   inout         VGA_HS, // VGA_HS is secondary SD card detect when VGA_EN = 1 (inactive)
   output        VGA_VS,
   input         VGA_EN, // active low

   /////////// AUDIO //////////
   output        AUDIO_L,
   output        AUDIO_R,
   output        AUDIO_SPDIF,

   //////////// SDIO ///////////
   inout [3:0]   SDIO_DAT,
   inout         SDIO_CMD,
   output        SDIO_CLK,

   //////////// I/O ///////////
   output        LED_USER,
   output        LED_HDD,
   output        LED_POWER,
   input         BTN_USER,
   input         BTN_OSD,
   input         BTN_RESET,
`endif

   ////////// I/O ALT /////////
   output        SD_SPI_CS,
   input         SD_SPI_MISO,
   output        SD_SPI_CLK,
   output        SD_SPI_MOSI,

   inout         SDCD_SPDIF,
   output        IO_SCL,
   inout         IO_SDA,

   ////////// ADC //////////////
   output        ADC_SCK,
   input         ADC_SDO,
   output        ADC_SDI,
   output        ADC_CONVST,

   ////////// MB KEY ///////////
   input [1:0]   KEY,

   ////////// MB SWITCH ////////
   input [3:0]   SW,

   ////////// MB LED ///////////
   output [7:0]  LED,

   ///////// USER IO ///////////
   inout [6:0]   USER_IO
   );

  logic [31:0]  gp_in;
  logic [31:0]  gp_out;
  logic         uart_dtr;
  logic         uart_dsr;
  logic         uart_cts;
  logic         uart_rts;
  logic         uart_rxd;
  logic         uart_txd;
  logic         aspi_sck;
  logic         aspi_mosi;
  logic         aspi_ss;
  logic         aspi_miso;
  logic [63:0]  f2h_irq;
  logic [1:0]   clkselect;
  logic [2:0]   inclk;
  logic         outclk;
  logic         ram_clk;
  logic [28:0]  ram_address;
  logic [7:0]   ram_burstcount;
  logic         ram_waitrequest;
  logic [63:0]  ram_readdata;
  logic         ram_readdatavalid;
  logic         ram_read;
  logic [63:0]  ram_writedata;
  logic [7:0]   ram_byteenable;
  logic         ram_write;

  logic         clk_audio;
  logic [28:0]  ram2_address;
  logic [7:0]   ram2_burstcount;
  logic         ram2_waitrequest;
  logic [63:0]  ram2_readdata;
  logic         ram2_readdatavalid;
  logic         ram2_read;
  logic [63:0]  ram2_writedata;
  logic [7:0]   ram2_byteenable;
  logic         ram2_write;

  logic [27:0]  vbuf_address;
  logic [7:0]   vbuf_burstcount;
  logic         vbuf_waitrequest;
  logic [127:0] vbuf_readdata;
  logic         vbuf_readdatavalid;
  logic         vbuf_read;
  logic [127:0] vbuf_writedata;
  logic [15:0]  vbuf_byteenable;
  logic         vbuf_write;

  logic [10:0]  pll_axi_awaddr;
  logic         pll_axi_awvalid;
  logic         pll_axi_awready;
  logic [31:0]  pll_axi_wdata;
  logic [3:0]   pll_axi_wstrb;
  logic         pll_axi_wvalid;
  logic         pll_axi_wready;
  logic [1:0]   pll_axi_bresp;
  logic         pll_axi_bvalid;
  logic         pll_axi_bready;
  logic [10:0]  pll_axi_araddr;
  logic         pll_axi_arvalid;
  logic         pll_axi_arready;
  logic [31:0]  pll_axi_rdata;
  logic [1:0]   pll_axi_rresp;
  logic         pll_axi_rvalid;
  logic         pll_axi_rready;

  subsys
    #
    (
     .FPGA_FAMILY         (FPGA_FAMILY)
     )
  u_subsys
    (
     .FPGA_CLK1_50        (FPGA_CLK1_50),
     .FPGA_CLK2_50        (FPGA_CLK2_50),
     .FPGA_CLK3_50        (FPGA_CLK3_50),

     .HDMI_I2C_SCL        (HDMI_I2C_SCL),
     .HDMI_I2C_SDA        (HDMI_I2C_SDA),

     .HDMI_SCLK           (HDMI_SCLK),
     .HDMI_LRCLK          (HDMI_LRCLK),
     .HDMI_I2S            (HDMI_I2S),

     .HDMI_TX_CLK         (HDMI_TX_CLK),
     .HDMI_TX_DE          (HDMI_TX_DE),
     .HDMI_TX_D           (HDMI_TX_D),
     .HDMI_TX_HS          (HDMI_TX_HS),
     .HDMI_TX_VS          (HDMI_TX_VS),

     .HDMI_TX_INT         (HDMI_TX_INT),

     .SDRAM_A             (SDRAM_A),
     .SDRAM_DQ            (SDRAM_DQ),
     .SDRAM_DQML          (SDRAM_DQML),
     .SDRAM_DQMH          (SDRAM_DQMH),
     .SDRAM_nWE           (SDRAM_nWE),
     .SDRAM_nCAS          (SDRAM_nCAS),
     .SDRAM_nRAS          (SDRAM_nRAS),
     .SDRAM_nCS           (SDRAM_nCS),
     .SDRAM_BA            (SDRAM_BA),
     .SDRAM_CLK           (SDRAM_CLK),
     .SDRAM_CKE           (SDRAM_CKE),

`ifdef DUAL_SDRAM
     .SDRAM2_A            (SDRAM2_A),
     .SDRAM2_DQ           (SDRAM2_DQ),
     .SDRAM2_nWE          (SDRAM2_nWE),
     .SDRAM2_nCAS         (SDRAM2_nCAS),
     .SDRAM2_nRAS         (SDRAM2_nRAS),
     .SDRAM2_nCS          (SDRAM2_nCS),
     .SDRAM2_BA           (SDRAM2_BA),
     .SDRAM2_CLK          (SDRAM2_CLK),

`else
     .VGA_R               (VGA_R),
     .VGA_G               (VGA_G),
     .VGA_B               (VGA_B),
     .VGA_HS              (VGA_HS),
     .VGA_VS              (VGA_VS),
     .VGA_EN              (VGA_EN),

     .AUDIO_L             (AUDIO_L),
     .AUDIO_R             (AUDIO_R),
     .AUDIO_SPDIF         (AUDIO_SPDIF),

     .SDIO_DAT            (SDIO_DAT),
     .SDIO_CMD            (SDIO_CMD),
     .SDIO_CLK            (SDIO_CLK),

     .LED_USER            (LED_USER),
     .LED_POWER           (LED_POWER),
     .LED_HDD             (LED_DISK),
     .BTN_USER            (BTN_USER),
     .BTN_OSD             (BTN_OSD),
     .BTN_RESET           (BTN_RESET),
`endif

     .SD_SPI_CS           (SD_SPI_CS),
     .SD_SPI_MISO         (SD_SPI_MISO),
     .SD_SPI_CLK          (SD_SPI_CLK),
     .SD_SPI_MOSI         (SD_SPI_MOSI),

     .SDCD_SPDIF          (SDCD_SPDIF),
     .IO_SCL              (IO_SCL),
     .IO_SDA              (IO_SDA),

     .ADC_SCK             (ADC_SCK),
     .ADC_SDO             (ADC_SDO),
     .ADC_SDI             (ADC_SDI),
     .ADC_CONVST          (ADC_CONVST),

     ////////// MB KEY ///////////
     .KEY                 (KEY),

     ////////// MB SWITCH ////////
     .SW                  (SW),

     ////////// MB LED ///////////
     .LED                 (LED),

     ///////// USER IO ///////////
     .USER_IO             (USER_IO),

     // HPS connections
     .gp_in               (gp_in),
     .gp_out              (gp_out),

     .uart_cts            (uart_rts),
     .uart_rts            (uart_cts),
     .uart_rxd            (uart_txd),
     .uart_txd            (uart_rxd),
     .uart_dtr            (uart_dsr),
     .uart_dsr            (uart_dtr),

     .aspi_sck            (aspi_sck),
     .aspi_mosi           (aspi_mosi),
     .aspi_miso           (aspi_miso),
     .aspi_ss             (aspi_ss),

     .f2h_irq             (f2h_irq),

     .reset_req           (reset_req),
     .clk_100m            (clk_100m),
     .reset               (reset),

     .reset_hps_cold_req  (reset_hps_cold_req),

     .ram_clk             (ram_clk),
     .ram_address         (ram_address),
     .ram_burstcount      (ram_burstcount),
     .ram_waitrequest     (ram_waitrequest),
     .ram_readdata        (ram_readdata),
     .ram_readdatavalid   (ram_readdatavalid),
     .ram_read            (ram_read),
     .ram_writedata       (ram_writedata),
     .ram_byteenable      (ram_byteenable),
     .ram_write           (ram_write),

     .clk_audio           (clk_audio),
     .ram2_address        (ram2_address),
     .ram2_burstcount     (ram2_burstcount),
     .ram2_waitrequest    (ram2_waitrequest),
     .ram2_readdata       (ram2_readdata),
     .ram2_readdatavalid  (ram2_readdatavalid),
     .ram2_read           (ram2_read),
     .ram2_writedata      (ram2_writedata),
     .ram2_byteenable     (ram2_byteenable),
     .ram2_write          (ram2_write),

     .vbuf_address        (vbuf_address),
     .vbuf_burstcount     (vbuf_burstcount),
     .vbuf_waitrequest    (vbuf_waitrequest),
     .vbuf_readdata       (vbuf_readdata),
     .vbuf_readdatavalid  (vbuf_readdatavalid),
     .vbuf_read           (vbuf_read),
     .vbuf_writedata      (vbuf_writedata),
     .vbuf_byteenable     (vbuf_byteenable),
     .vbuf_write          (vbuf_write),

     .s_axi_awaddr        (pll_axi_awaddr),
     .s_axi_awvalid       (pll_axi_awvalid),
     .s_axi_awready       (pll_axi_awready),
     .s_axi_wdata         (pll_axi_wdata),
     .s_axi_wstrb         (pll_axi_wstrb),
     .s_axi_wvalid        (pll_axi_wvalid),
     .s_axi_wready        (pll_axi_wready),
     .s_axi_bresp         (pll_axi_bresp),
     .s_axi_bvalid        (pll_axi_bvalid),
     .s_axi_bready        (pll_axi_bready),
     .s_axi_araddr        (pll_axi_araddr),
     .s_axi_arvalid       (pll_axi_arvalid),
     .s_axi_arready       (pll_axi_arready),
     .s_axi_rdata         (pll_axi_rdata),
     .s_axi_rresp         (pll_axi_rresp),
     .s_axi_rvalid        (pll_axi_rvalid),
     .s_axi_rready        (pll_axi_rready)
     );

  assign HDMI_MCLK = '0;

  cyclonev_hps_interface_mpu_general_purpose h2f_gp
    (
     .gp_in               (gp_in),
     .gp_out              (gp_out)
     );

  cyclonev_hps_interface_peripheral_uart uart
    (
     .ri                  (0)
`ifndef ARCADE_SYS
     ,
     .dsr                 (uart_dsr),
     .dcd                 (uart_dsr),
     .dtr                 (uart_dtr),

     .cts                 (uart_cts),
     .rts                 (uart_rts),
     .rxd                 (uart_rxd),
     .txd                 (uart_txd)
`endif //  `ifndef ARCADE_SYS
     );

  cyclonev_hps_interface_peripheral_spi_master spi
    (
     .sclk_out            (aspi_sck),
     .txd                 (aspi_mosi), // mosi
     .rxd                 (aspi_miso), // miso

     .ss_0_n              (aspi_ss),
     .ss_in_n             (1)
     );

  cyclonev_hps_interface_interrupts interrupts
    (
     .irq                 (f2h_irq)
     );

  sysmem_lite sysmem
    (
     //Reset/Clock
     .reset_core_req      (reset_req),
     .reset_out           (reset),
     .clock               (clk_100m),

     //DE10-nano has no reset signal on GPIO, so core has to emulate cold reset button.
     .reset_hps_cold_req  (reset_hps_cold_req),

`ifdef USE_DDRAM
     //64-bit DDR3 RAM access
     .ram1_clk            (ram_clk),
     .ram1_address        (ram_address),
     .ram1_burstcount     (ram_burstcount),
     .ram1_waitrequest    (ram_waitrequest),
     .ram1_readdata       (ram_readdata),
     .ram1_readdatavalid  (ram_readdatavalid),
     .ram1_read           (ram_read),
     .ram1_writedata      (ram_writedata),
     .ram1_byteenable     (ram_byteenable),
     .ram1_write          (ram_write),
`endif

     //64-bit DDR3 RAM access
     .ram2_clk            (clk_audio),
     .ram2_address        (ram2_address),
     .ram2_burstcount     (ram2_burstcount),
     .ram2_waitrequest    (ram2_waitrequest),
     .ram2_readdata       (ram2_readdata),
     .ram2_readdatavalid  (ram2_readdatavalid),
     .ram2_read           (ram2_read),
     .ram2_writedata      (ram2_writedata),
     .ram2_byteenable     (ram2_byteenable),
     .ram2_write          (ram2_write),

     //128-bit DDR3 RAM access
     // HDMI frame buffer
     .vbuf_clk            (clk_100m),
     .vbuf_address        (vbuf_address),
     .vbuf_burstcount     (vbuf_burstcount),
     .vbuf_waitrequest    (vbuf_waitrequest),
     .vbuf_writedata      (vbuf_writedata),
     .vbuf_byteenable     (vbuf_byteenable),
     .vbuf_write          (vbuf_write),
     .vbuf_readdata       (vbuf_readdata),
     .vbuf_readdatavalid  (vbuf_readdatavalid),
     .vbuf_read           (vbuf_read)
     );

endmodule
