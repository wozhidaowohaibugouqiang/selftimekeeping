/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Mar 31 2026 19:06:33
**	TD version	:	6.2.178840
************************************************************/
module ChipWatcher_0
(
  input   [0:0]                 probe0,
  input   [0:0]                 probe1,
  input   [0:0]                 probe2,
  input   [0:0]                 probe3,
  input   [0:0]                 probe4,
  input   [0:0]                 probe5,
  input   [0:0]                 probe6,
  input   [0:0]                 probe7,
  input   [0:0]                 probe8,
  input   [0:0]                 probe9,
  input                         clk
);

  ChipWatcher_8e2139500150  ChipWatcher_8e2139500150_Inst
  (
      .probe0(probe0),
      .probe1(probe1),
      .probe2(probe2),
      .probe3(probe3),
      .probe4(probe4),
      .probe5(probe5),
      .probe6(probe6),
      .probe7(probe7),
      .probe8(probe8),
      .probe9(probe9),
      .clk(clk)
  );
endmodule
