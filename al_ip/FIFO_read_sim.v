// Verilog netlist created by Tang Dynasty v6.2.178840
// Wed Apr 15 16:52:45 2026

`timescale 1ns / 1ps
module FIFO_read  // FIFO_read.v(14)
  (
  clk,
  di,
  re,
  rst,
  we,
  afull_flag,
  do,
  empty_flag,
  full_flag
  );

  input clk;  // FIFO_read.v(24)
  input [31:0] di;  // FIFO_read.v(23)
  input re;  // FIFO_read.v(25)
  input rst;  // FIFO_read.v(22)
  input we;  // FIFO_read.v(24)
  output afull_flag;  // FIFO_read.v(29)
  output [63:0] do;  // FIFO_read.v(27)
  output empty_flag;  // FIFO_read.v(28)
  output full_flag;  // FIFO_read.v(29)

  wire empty_flag_syn_2;  // FIFO_read.v(28)
  wire full_flag_syn_2;  // FIFO_read.v(29)

  EF2_PHY_CONFIG #(
    .DONE_PERSISTN("ENABLE"),
    .INIT_PERSISTN("ENABLE"),
    .JTAG_PERSISTN("DISABLE"),
    .PROGRAMN_PERSISTN("DISABLE"))
    config_inst ();
  not empty_flag_syn_1 (empty_flag_syn_2, empty_flag);  // FIFO_read.v(28)
  EF2_PHY_FIFO #(
    .AE(32'b00000000000000000000000001101000),
    .AEP1(32'b00000000000000000000000001111000),
    .AF(32'b00000000000000000001111111010000),
    .AFM1(32'b00000000000000000001111111001000),
    .ASYNC_RESET_RELEASE("SYNC"),
    .DATA_WIDTH_A("9"),
    .DATA_WIDTH_B("18"),
    .E(32'b00000000000000000000000000001000),
    .EP1(32'b00000000000000000000000000011000),
    .F(32'b00000000000000000010000000000000),
    .FM1(32'b00000000000000000001111111111000),
    .GSR("DISABLE"),
    .MODE("FIFO8K"),
    .REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),
    .RESETMODE("SYNC"))
    fifo_inst_syn_1 (
    .clkr(clk),
    .clkw(clk),
    .csr({2'b11,empty_flag_syn_2}),
    .csw({2'b11,full_flag_syn_2}),
    .dia(di[8:0]),
    .orea(1'b0),
    .oreb(1'b0),
    .re(re),
    .rprst(rst),
    .rst(rst),
    .we(we),
    .afull_flag(afull_flag),
    .doa(do[40:32]),
    .dob(do[8:0]),
    .empty_flag(empty_flag),
    .full_flag(full_flag));  // FIFO_read.v(42)
  EF2_PHY_FIFO #(
    .AE(32'b00000000000000000000000001101000),
    .AEP1(32'b00000000000000000000000001111000),
    .AF(32'b00000000000000000001111111010000),
    .AFM1(32'b00000000000000000001111111001000),
    .ASYNC_RESET_RELEASE("SYNC"),
    .DATA_WIDTH_A("9"),
    .DATA_WIDTH_B("18"),
    .E(32'b00000000000000000000000000001000),
    .EP1(32'b00000000000000000000000000011000),
    .F(32'b00000000000000000010000000000000),
    .FM1(32'b00000000000000000001111111111000),
    .GSR("DISABLE"),
    .MODE("FIFO8K"),
    .REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),
    .RESETMODE("SYNC"))
    fifo_inst_syn_2 (
    .clkr(clk),
    .clkw(clk),
    .csr({2'b11,empty_flag_syn_2}),
    .csw({2'b11,full_flag_syn_2}),
    .dia(di[17:9]),
    .orea(1'b0),
    .oreb(1'b0),
    .re(re),
    .rprst(rst),
    .rst(rst),
    .we(we),
    .doa(do[49:41]),
    .dob(do[17:9]));  // FIFO_read.v(42)
  EF2_PHY_FIFO #(
    .AE(32'b00000000000000000000000001101000),
    .AEP1(32'b00000000000000000000000001111000),
    .AF(32'b00000000000000000001111111010000),
    .AFM1(32'b00000000000000000001111111001000),
    .ASYNC_RESET_RELEASE("SYNC"),
    .DATA_WIDTH_A("9"),
    .DATA_WIDTH_B("18"),
    .E(32'b00000000000000000000000000001000),
    .EP1(32'b00000000000000000000000000011000),
    .F(32'b00000000000000000010000000000000),
    .FM1(32'b00000000000000000001111111111000),
    .GSR("DISABLE"),
    .MODE("FIFO8K"),
    .REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),
    .RESETMODE("SYNC"))
    fifo_inst_syn_3 (
    .clkr(clk),
    .clkw(clk),
    .csr({2'b11,empty_flag_syn_2}),
    .csw({2'b11,full_flag_syn_2}),
    .dia(di[26:18]),
    .orea(1'b0),
    .oreb(1'b0),
    .re(re),
    .rprst(rst),
    .rst(rst),
    .we(we),
    .doa(do[58:50]),
    .dob(do[26:18]));  // FIFO_read.v(42)
  EF2_PHY_FIFO #(
    .AE(32'b00000000000000000000000001101000),
    .AEP1(32'b00000000000000000000000001111000),
    .AF(32'b00000000000000000001111111010000),
    .AFM1(32'b00000000000000000001111111001000),
    .ASYNC_RESET_RELEASE("SYNC"),
    .DATA_WIDTH_A("9"),
    .DATA_WIDTH_B("18"),
    .E(32'b00000000000000000000000000001000),
    .EP1(32'b00000000000000000000000000011000),
    .F(32'b00000000000000000010000000000000),
    .FM1(32'b00000000000000000001111111111000),
    .GSR("DISABLE"),
    .MODE("FIFO8K"),
    .REGMODE_A("NOREG"),
    .REGMODE_B("NOREG"),
    .RESETMODE("SYNC"))
    fifo_inst_syn_4 (
    .clkr(clk),
    .clkw(clk),
    .csr({2'b11,empty_flag_syn_2}),
    .csw({2'b11,full_flag_syn_2}),
    .dia({open_n83,open_n84,open_n85,open_n86,di[31:27]}),
    .orea(1'b0),
    .oreb(1'b0),
    .re(re),
    .rprst(rst),
    .rst(rst),
    .we(we),
    .doa({open_n98,open_n99,open_n100,open_n101,do[63:59]}),
    .dob({open_n102,open_n103,open_n104,open_n105,do[31:27]}));  // FIFO_read.v(42)
  not full_flag_syn_1 (full_flag_syn_2, full_flag);  // FIFO_read.v(29)

endmodule 

