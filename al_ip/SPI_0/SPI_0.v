/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Mar 28 2026 09:39:55
**	TD version	:	6.2.178840
************************************************************/
module SPI_0
(
  input                         Sys_Clk,
  input                         Rst_n,
  input   [1:0]                 Address,
  input   [31:0]                Data_to_IP,
  input                         Write,
  output  [31:0]                Data_from_IP,
  input                         Read,
  output                        TOE,
  output                        ROE,
  output                        TRDY,
  output                        RRDY,
  output                        SCLK,
  output                        MOSI,
  input                         MISO,
  output  [0:0]                 SS_n
);

  SPI_Master_c06a3a2e2c87
  #(
      .Number_of_Select_Signals(1),
      .Divide_Factor(10),
      .SS_n_Setup_Time(1),
      .SS_n_Hold_Time(1),
      .Data_Width(8)
  )SPI_Master_c06a3a2e2c87_Inst
  (
      .Sys_Clk(Sys_Clk),
      .Rst_n(Rst_n),
      .Address(Address),
      .Data_to_IP(Data_to_IP),
      .Write(Write),
      .Data_from_IP(Data_from_IP),
      .Read(Read),
      .TOE(TOE),
      .ROE(ROE),
      .TRDY(TRDY),
      .RRDY(RRDY),
      .SCLK(SCLK),
      .MOSI(MOSI),
      .MISO(MISO),
      .SS_n(SS_n)
  );
endmodule
