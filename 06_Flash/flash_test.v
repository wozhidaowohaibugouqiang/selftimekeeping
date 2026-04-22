/************************************************************\
 ** 例程	:	SPI→FLASH存储器W25Q32测试程序；
 ** 现象	:	W25Q64测试读写测试成功L2灯点亮；
 ** 板卡	:	FA202 V1.0
 ** 日期	:	2023/03
 ** 版本	:	V1.2 (TD4.6.7-64bit)
 ** 作者	:	大科电子工作室（ https://shop117015787.taobao.com/ ）	
\************************************************************/
module Flash
(
    input          CLK,
	 input          RSTn,


    output flash_clk,                 //spi flash clock 
	 output flash_cs,                  //spi flash cs 
	 output flash_datain,              //spi flash data input  
	 input  flash_dataout,             //spi flash data output
	 
	 output led
);

reg [3:0] Flash_i;
reg [7:0] flash_cmd;
reg [23:0] flash_addr;

reg clock25M;
reg [3:0] cmd_type;

reg [7:0] time_delay;

wire Done_Sig;
wire [7:0] mydata_o;
wire myvalid_o;
wire [2:0] spi_state;

reg [21:0] counter;   
reg reset;


always @(posedge CLK ) 
begin    
   if(RSTn==1'b1) counter<=0;   
	else if (!counter[21]) counter <= counter + 1'b1;
	else counter <= counter;
	
	reset<=counter[21];

end

always @ ( posedge clock25M or posedge reset )
    if( reset ) begin
			Flash_i <= 4'd0;
			flash_addr <= 24'd0;
			flash_cmd <= 8'd0;
			cmd_type <= 4'b0000;
			time_delay<=0;
	 end
	 else 
	     case( Flash_i )

	      4'd0:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
			else begin  flash_cmd <= 8'h9f; flash_addr <= 24'd0; cmd_type <= 4'b1000; end	
		  
	      4'd1:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
			else begin flash_cmd <= 8'h06; cmd_type <= 4'b1001; end
	
			4'd2:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type<=4'b0000; end
			else begin flash_cmd <= 8'hd8; flash_addr <= 24'd0; cmd_type <= 4'b1010; end
			
	      4'd3:
			if( time_delay<8'd100 ) begin flash_cmd <= 8'h00; time_delay<=time_delay+1'b1; cmd_type <= 4'b0000; end
			else begin Flash_i <= Flash_i + 1'b1; time_delay<=0; end	
					
			4'd4:
			if( Done_Sig ) begin 
			    if (mydata_o[0]==1'b0) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
				 else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end
			end
			else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end

	      4'd5:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
			else begin flash_cmd <= 8'h04; cmd_type <= 4'b1100; end			

			4'd6:
			if( Done_Sig ) begin 
			    if (mydata_o[0]==1'b0) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
				 else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end
			end
			else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end

	      4'd7:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
			else begin flash_cmd <= 8'h06; cmd_type <= 4'b1001; end 

	      4'd8:
			if( time_delay<8'd100 ) begin flash_cmd <= 8'h00; time_delay<=time_delay+1'b1; cmd_type <= 4'b0000; end
			else begin Flash_i <= Flash_i + 1'b1; time_delay<=0; end	

	      4'd9:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1;cmd_type <= 4'b0000; end
			else begin flash_cmd <= 8'h02; flash_addr <= 24'd0; cmd_type <= 4'b1101; end
			
	      4'd10:
			if( time_delay<8'd100 ) begin flash_cmd <= 8'h00; time_delay<=time_delay+1'b1; cmd_type <= 4'b0000; end
			else begin Flash_i <= Flash_i + 1'b1; time_delay<=0; end	

			4'd11:
			if( Done_Sig ) begin 
			    if (mydata_o[0]==1'b0) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
				 else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end
			end
			else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end

	      4'd12:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
			else begin flash_cmd <= 8'h04; cmd_type <= 4'b1100; end		

			4'd13:
			if( Done_Sig ) begin 
			    if (mydata_o[0]==1'b0) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
				 else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end
			end
			else begin flash_cmd <= 8'h05; cmd_type <= 4'b1011; end
					
			4'd14:
			if( Done_Sig ) begin flash_cmd <= 8'h00; Flash_i <= Flash_i + 1'b1; cmd_type <= 4'b0000; end
			else begin flash_cmd <= 8'h03; flash_addr <= 24'd0; cmd_type <= 4'b1110; end

			4'd15://idle
			Flash_i <= 4'd15;

	      endcase
			
  
always @ ( posedge CLK )
    if( !RSTn ) clock25M<=1'b0;
	 else clock25M <= ~clock25M;  
 	 
flash_spi U1
(
	     .flash_clk(flash_clk ),
		  .flash_cs( flash_cs ),
		  .flash_datain( flash_datain ),  
		  .flash_dataout( flash_dataout ),    
		  
		  .clock25M( clock25M ),           //input clock
		  .flash_rstn( ~reset ),             //input reset 
		  .cmd_type( cmd_type ),           // flash command type		  
		  .Done_Sig( Done_Sig ),           //output done signal
		  .flash_cmd( flash_cmd ),         // input flash command 
		  .flash_addr( flash_addr ),       // input flash address 
		  .mydata_o( mydata_o ),           // output flash data 
		  .myvalid_o( myvalid_o ),         // output flash data valid 		
        .spi_state(spi_state),
        .Flash_LED(led)		  
		  

);
	 

endmodule
