/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Mar 20 2026 15:37:28
**	TD version	:	6.2.178840
************************************************************/
module UART_0
(
  input                         clk,
  input                         rst_n,
  output                        rx_vld,
  output  [7:0]                 rx_data,
  output                        rx_err,
  input                         rxd,
  output                        tx_rdy,
  input                         tx_en,
  input   [7:0]                 tx_data,
  output                        txd
);

  uart_2bd8831e6282
  #(
      .CLK_DIV_NUM(27),
      .DATA_BIT(8),
      .PARITY_BIT("None"),
      .STOP_BIT("1")
  )uart_2bd8831e6282_Inst
  (
      .clk(clk),
      .rst_n(rst_n),
      .rx_vld(rx_vld),
      .rx_data(rx_data),
      .rx_err(rx_err),
      .rxd(rxd),
      .tx_rdy(tx_rdy),
      .tx_en(tx_en),
      .tx_data(tx_data),
      .txd(txd)
  );
endmodule
