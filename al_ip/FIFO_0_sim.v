// Verilog netlist created by Tang Dynasty v6.2.178840
// Wed Jan 21 16:16:40 2026

`timescale 1ns / 1ps
module FIFO_0  // FIFO_0.v(14)
  (
  clk,
  di,
  re,
  rrst,
  we,
  wrst,
  dout,
  empty_flag,
  fifo_rdpointer,
  fifo_wrpointer,
  full_flag
  );

  input clk;  // FIFO_0.v(20)
  input [7:0] di;  // FIFO_0.v(19)
  input re;  // FIFO_0.v(22)
  input rrst;  // FIFO_0.v(18)
  input we;  // FIFO_0.v(21)
  input wrst;  // FIFO_0.v(17)
  output [7:0] dout;  // FIFO_0.v(23)
  output empty_flag;  // FIFO_0.v(24)
  output [13:0] fifo_rdpointer;  // FIFO_0.v(27)
  output [13:0] fifo_wrpointer;  // FIFO_0.v(26)
  output full_flag;  // FIFO_0.v(25)

  wire [13:0] fifo_inst_syn_109;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_124;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_138;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_153;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_167;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_17;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_182;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_196;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_3;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_37;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_51;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_66;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_80;  // FIFO_0.v(38)
  wire [13:0] fifo_inst_syn_95;  // FIFO_0.v(38)
  wire fifo_inst_syn_31;  // FIFO_0.v(38)
  wire fifo_inst_syn_32;  // FIFO_0.v(38)
  wire fifo_inst_syn_33;  // FIFO_0.v(38)
  wire fifo_inst_syn_34;  // FIFO_0.v(38)

  PH1_PHY_CONFIG_V2 #(
    .JTAG_PERSISTN("DISABLE"),
    .SPIX4_PERSISTN("ENABLE"))
    config_inst ();
  PH1_PHY_FIFOCTRL #(
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y0Z0"),
    .FIFO_AE(14'b00000001100000),
    .FIFO_AF(14'b11111110011111),
    .FIFO_ASYNC_RESET_RELEASE("SYNC"),
    .FIFO_DATA_WIDTH("1"),
    .FIFO_FIRSTWRITE_RD("DISABLE"),
    .FIFO_SYNC("SYNC"))
    fifo_inst (
    .fifo_rclk(clk),
    .fifo_re(re),
    .fifo_rrst(rrst),
    .fifo_wclk(clk),
    .fifo_we(we),
    .fifo_wrst(wrst),
    .fifo_empty(empty_flag),
    .fifo_full(full_flag),
    .fifo_ore(fifo_inst_syn_34),
    .fifo_owe(fifo_inst_syn_33),
    .fifo_rdpointer({open_n206,fifo_rdpointer}),
    .fifo_wrpointer({open_n207,fifo_wrpointer}),
    .fifoctrl_raddr(fifo_inst_syn_17),
    .fifoctrl_re(fifo_inst_syn_32),
    .fifoctrl_waddr(fifo_inst_syn_3),
    .fifoctrl_we(fifo_inst_syn_31));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y8Z1"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSB0("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .HADDRCAS_A("CASFB"),
    .HADDRCAS_B("CASFB"),
    .LADDRCAS_A("CASFB"),
    .LADDRCAS_B("CASFB"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_123 (
    .addra_casin(fifo_inst_syn_124),
    .addrb_casin(fifo_inst_syn_138),
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,fifo_inst_syn_31,open_n236}),
    .csb({fifo_inst_syn_34,fifo_inst_syn_32,open_n237}),
    .dia({open_n238,open_n239,open_n240,open_n241,open_n242,open_n243,open_n244,open_n245,open_n246,open_n247,open_n248,open_n249,open_n250,open_n251,open_n252,di[5]}),
    .addra_casout(fifo_inst_syn_153),
    .addrb_casout(fifo_inst_syn_167),
    .dob({open_n335,open_n336,open_n337,open_n338,open_n339,open_n340,open_n341,open_n342,open_n343,open_n344,open_n345,open_n346,open_n347,open_n348,open_n349,dout[5]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y12Z0"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSB0("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .HADDRCAS_A("CASFB"),
    .HADDRCAS_B("CASFB"),
    .LADDRCAS_A("CASFB"),
    .LADDRCAS_B("CASFB"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_152 (
    .addra_casin(fifo_inst_syn_153),
    .addrb_casin(fifo_inst_syn_167),
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,fifo_inst_syn_31,open_n384}),
    .csb({fifo_inst_syn_34,fifo_inst_syn_32,open_n385}),
    .dia({open_n386,open_n387,open_n388,open_n389,open_n390,open_n391,open_n392,open_n393,open_n394,open_n395,open_n396,open_n397,open_n398,open_n399,open_n400,di[6]}),
    .addra_casout(fifo_inst_syn_182),
    .addrb_casout(fifo_inst_syn_196),
    .dob({open_n483,open_n484,open_n485,open_n486,open_n487,open_n488,open_n489,open_n490,open_n491,open_n492,open_n493,open_n494,open_n495,open_n496,open_n497,dout[6]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y12Z1"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSB0("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .HADDRCAS_A("CASFB"),
    .HADDRCAS_B("CASFB"),
    .LADDRCAS_A("CASFB"),
    .LADDRCAS_B("CASFB"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_181 (
    .addra_casin(fifo_inst_syn_182),
    .addrb_casin(fifo_inst_syn_196),
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,fifo_inst_syn_31,open_n532}),
    .csb({fifo_inst_syn_34,fifo_inst_syn_32,open_n533}),
    .dia({open_n534,open_n535,open_n536,open_n537,open_n538,open_n539,open_n540,open_n541,open_n542,open_n543,open_n544,open_n545,open_n546,open_n547,open_n548,di[7]}),
    .dob({open_n659,open_n660,open_n661,open_n662,open_n663,open_n664,open_n665,open_n666,open_n667,open_n668,open_n669,open_n670,open_n671,open_n672,open_n673,dout[7]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y0Z0"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSA1("1"),
    .CSB0("1"),
    .CSB1("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .FIFOMODE("ENABLE"),
    .HADDRCAS_A("FIFO"),
    .HADDRCAS_B("FIFO"),
    .LADDRCAS_A("FIFO"),
    .LADDRCAS_B("FIFO"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_2 (
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,open_n736,open_n737}),
    .csb({fifo_inst_syn_34,open_n738,open_n739}),
    .dia({open_n740,open_n741,open_n742,open_n743,open_n744,open_n745,open_n746,open_n747,open_n748,open_n749,open_n750,open_n751,open_n752,open_n753,open_n754,di[0]}),
    .fifoctrl_raddr(fifo_inst_syn_17),
    .fifoctrl_re(fifo_inst_syn_32),
    .fifoctrl_waddr(fifo_inst_syn_3),
    .fifoctrl_we(fifo_inst_syn_31),
    .dob({open_n835,open_n836,open_n837,open_n838,open_n839,open_n840,open_n841,open_n842,open_n843,open_n844,open_n845,open_n846,open_n847,open_n848,open_n849,dout[0]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y0Z1"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSA1("1"),
    .CSB0("1"),
    .CSB1("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .FIFOMODE("ENABLE"),
    .HADDRCAS_A("FIFO"),
    .HADDRCAS_B("FIFO"),
    .LADDRCAS_A("FIFO"),
    .LADDRCAS_B("FIFO"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_35 (
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,open_n912,open_n913}),
    .csb({fifo_inst_syn_34,open_n914,open_n915}),
    .dia({open_n916,open_n917,open_n918,open_n919,open_n920,open_n921,open_n922,open_n923,open_n924,open_n925,open_n926,open_n927,open_n928,open_n929,open_n930,di[1]}),
    .fifoctrl_raddr(fifo_inst_syn_17),
    .fifoctrl_re(fifo_inst_syn_32),
    .fifoctrl_waddr(fifo_inst_syn_3),
    .fifoctrl_we(fifo_inst_syn_31),
    .addra_casout(fifo_inst_syn_37),
    .addrb_casout(fifo_inst_syn_51),
    .dob({open_n983,open_n984,open_n985,open_n986,open_n987,open_n988,open_n989,open_n990,open_n991,open_n992,open_n993,open_n994,open_n995,open_n996,open_n997,dout[1]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y4Z0"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSB0("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .HADDRCAS_A("CASFB"),
    .HADDRCAS_B("CASFB"),
    .LADDRCAS_A("CASFB"),
    .LADDRCAS_B("CASFB"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_36 (
    .addra_casin(fifo_inst_syn_37),
    .addrb_casin(fifo_inst_syn_51),
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,fifo_inst_syn_31,open_n1032}),
    .csb({fifo_inst_syn_34,fifo_inst_syn_32,open_n1033}),
    .dia({open_n1034,open_n1035,open_n1036,open_n1037,open_n1038,open_n1039,open_n1040,open_n1041,open_n1042,open_n1043,open_n1044,open_n1045,open_n1046,open_n1047,open_n1048,di[2]}),
    .addra_casout(fifo_inst_syn_66),
    .addrb_casout(fifo_inst_syn_80),
    .dob({open_n1131,open_n1132,open_n1133,open_n1134,open_n1135,open_n1136,open_n1137,open_n1138,open_n1139,open_n1140,open_n1141,open_n1142,open_n1143,open_n1144,open_n1145,dout[2]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y4Z1"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSB0("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .HADDRCAS_A("CASFB"),
    .HADDRCAS_B("CASFB"),
    .LADDRCAS_A("CASFB"),
    .LADDRCAS_B("CASFB"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_65 (
    .addra_casin(fifo_inst_syn_66),
    .addrb_casin(fifo_inst_syn_80),
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,fifo_inst_syn_31,open_n1180}),
    .csb({fifo_inst_syn_34,fifo_inst_syn_32,open_n1181}),
    .dia({open_n1182,open_n1183,open_n1184,open_n1185,open_n1186,open_n1187,open_n1188,open_n1189,open_n1190,open_n1191,open_n1192,open_n1193,open_n1194,open_n1195,open_n1196,di[3]}),
    .addra_casout(fifo_inst_syn_95),
    .addrb_casout(fifo_inst_syn_109),
    .dob({open_n1279,open_n1280,open_n1281,open_n1282,open_n1283,open_n1284,open_n1285,open_n1286,open_n1287,open_n1288,open_n1289,open_n1290,open_n1291,open_n1292,open_n1293,dout[3]}));  // FIFO_0.v(38)
  PH1_PHY_ERAM #(
    //.ADDRCASENA("ENABLE"),
    //.ADDRCASENB("ENABLE"),
    //.MACRO("fifo_inst"),
    //.R_POSITION("X0Y8Z0"),
    .CLKMODE("SYNC"),
    .CSA0("1"),
    .CSB0("1"),
    .DATA_WIDTH_A("1"),
    .DATA_WIDTH_B("1"),
    .ECC_DECODE("DISABLE"),
    .ECC_ENCODE("DISABLE"),
    .HADDRCAS_A("CASFB"),
    .HADDRCAS_B("CASFB"),
    .LADDRCAS_A("CASFB"),
    .LADDRCAS_B("CASFB"),
    .MODE("FIFO20K"),
    .OCEAMUX("1"),
    .OCEBMUX("1"),
    .REGMODE_A("OUTREG"),
    .REGMODE_B("OUTREG"),
    .RSTAMUX("0"),
    .RSTBMUX("0"),
    .SSROVERCE("DISABLE"),
    .WEAMUX("1"),
    .WEBMUX("0"))
    fifo_inst_syn_94 (
    .addra_casin(fifo_inst_syn_95),
    .addrb_casin(fifo_inst_syn_109),
    .clka(clk),
    .clkb(clk),
    .csa({fifo_inst_syn_33,fifo_inst_syn_31,open_n1328}),
    .csb({fifo_inst_syn_34,fifo_inst_syn_32,open_n1329}),
    .dia({open_n1330,open_n1331,open_n1332,open_n1333,open_n1334,open_n1335,open_n1336,open_n1337,open_n1338,open_n1339,open_n1340,open_n1341,open_n1342,open_n1343,open_n1344,di[4]}),
    .addra_casout(fifo_inst_syn_124),
    .addrb_casout(fifo_inst_syn_138),
    .dob({open_n1427,open_n1428,open_n1429,open_n1430,open_n1431,open_n1432,open_n1433,open_n1434,open_n1435,open_n1436,open_n1437,open_n1438,open_n1439,open_n1440,open_n1441,dout[4]}));  // FIFO_0.v(38)

  // synthesis translate_off
  glbl glbl();
  always @(*) begin
    glbl.gsr <= PH1_PHY_GSR.gsr;
    glbl.gsrn <= PH1_PHY_GSR.gsrn;
    glbl.done_gwe <= PH1_PHY_GSR.done_gwe;
    glbl.usr_gsrn_en <= PH1_PHY_GSR.usr_gsrn_en;
  end
  // synthesis translate_on

endmodule 

