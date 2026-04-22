
module flash_spi(
                  output flash_clk,
						output reg flash_cs,
						output reg flash_datain,
						input  flash_dataout,
						
                  input clock25M,
						input flash_rstn,
						input [3:0] cmd_type,
						output reg Done_Sig,
						input [7:0]  flash_cmd,
						input  [23:0] flash_addr,
						output reg [7:0] mydata_o,
						output myvalid_o,
						output reg [2:0] spi_state,
						output Flash_LED
						);



assign myvalid_o=myvalid;

assign flash_clk=spi_clk_en?clock25M:0;


reg myvalid;
reg [7:0] mydata;

reg spi_clk_en=1'b0;
reg data_come;
reg Flash_LED_reg;

parameter idle=3'b000;
parameter cmd_send=3'b001;
parameter address_send=3'b010;
parameter read_wait=3'b011;
parameter write_data=3'b101;
parameter finish_done=3'b110;


reg [7:0] cmd_reg;
reg [23:0] address_reg;
reg [7:0] cnta;
reg [8:0] write_cnt;
reg [7:0] cntb;
reg [8:0] read_cnt;
reg [8:0] read_num;

reg read_finish;

assign Flash_LED = Flash_LED_reg;

always @(negedge clock25M)
begin
if(!flash_rstn)
	begin
		flash_cs<=1'b1;		
		spi_state<=idle;
		cmd_reg<=0;
		address_reg<=0;
	   spi_clk_en<=1'b0;             
		cnta<=0;
      write_cnt<=0;
      read_num<=0;	
      address_reg<=0;
		Done_Sig<=1'b0;
	end
else
	begin
	case(spi_state)
		idle: begin		  
				spi_clk_en<=1'b0;
				flash_cs<=1'b1;
				flash_datain<=1'b1;	
			   cmd_reg<=flash_cmd;
            address_reg<=flash_addr;
		      Done_Sig<=1'b0;				
				if(cmd_type[3]==1'b1) begin              
						spi_state<=cmd_send;
                  cnta<=7;		
                  write_cnt<=0;
                  read_num<=0;					
				end
		end
		cmd_send:begin 
			   spi_clk_en<=1'b1;                         
				flash_cs<=1'b0;                            
			   if(cnta>0) begin                          
						flash_datain<=cmd_reg[cnta];      
                  cnta<=cnta-1'b1;						
				end				
				else begin                              
						flash_datain<=cmd_reg[0];
						if ((cmd_type[2:0]==3'b001) | (cmd_type[2:0]==3'b100)) begin   
 						    spi_state<=finish_done;
                  end							 
                  else if (cmd_type[2:0]==3'b011)  begin   
						 	 spi_state<=read_wait;
							 cnta<=7;
							 read_num<=1;                      
						end
                  else if (cmd_type[2:0]==3'b000)  begin   
						 	 spi_state<=read_wait;                  
							 cnta<=7;
							 read_num<=17;                    
						end						
						else begin	                          
							 spi_state<=address_send;
						    cnta<=23;
					   end
				end
		end
		address_send:begin 
			   if(cnta>0)  begin                                
					flash_datain<=address_reg[cnta];           
               cnta<=cnta-1;						
				end				
				else begin                                     
					flash_datain<=address_reg[0];   
               if(cmd_type[2:0]==3'b010) begin             
 						 spi_state<=finish_done;	
               end
               else if (cmd_type[2:0]==3'b101) begin	      			
				       spi_state<=write_data;
						 cnta<=7;                       
					end			 
					else begin
					    spi_state<=read_wait;
						 read_num<=256;                        					 
               end						 
				end
		end
		read_wait: begin  
		     if(read_finish)  begin
			     spi_state<=finish_done;
				  data_come<=1'b0;
			  end
			  else
			     data_come<=1'b1;
		end		
		write_data: begin  
		   if(write_cnt<256) begin                    
			   if(cnta>0) begin                    
						flash_datain<=write_cnt[cnta];     
                  cnta<=cnta-1'b1;						
				end				
				else begin                                 
						flash_datain<=write_cnt[0];   
					   cnta<=7;
					   write_cnt<=write_cnt+1'b1;
				end
			end
         else begin
				 spi_state<=finish_done;
				 spi_clk_en<=1'b0;
			end
				 
		end
		finish_done:begin  
 		     flash_cs<=1'b1;
			  flash_datain<=1'b1;
			  spi_clk_en<=1'b0;
			  Done_Sig<=1'b1;
			  spi_state<=idle;
		end
		default:spi_state<=idle;
		endcase;		
	end
end
	

always @(posedge clock25M)
begin
	if(!flash_rstn)begin
			read_cnt<=0;
			cntb<=0;
			read_finish<=1'b0;
			myvalid<=1'b0;
			mydata<=0;
			mydata_o<=0;
			Flash_LED_reg <= 1'b1;
	end
	else
		 if(data_come)   begin
				if(read_cnt<read_num) begin 		  
						if(cntb<7) begin                	  
							 myvalid<=1'b0;
							 mydata<={mydata[6:0],flash_dataout};
							 cntb<=cntb+1'b1;
						end
						else  begin
							 myvalid<=1'b1;        
							 mydata_o<={mydata[6:0],flash_dataout}; 
							 cntb<=0;
							 if( mydata_o == read_cnt - 1 )
							 	begin
							 		Flash_LED_reg <= 1'b0;
							 		read_cnt<=read_cnt+1'b1;
							 	end
							 else 
							 	begin
							 		Flash_LED_reg <= 1'b1;
							 		read_cnt <= 256; 
							 	end
							 read_cnt<=read_cnt+1'b1;
						end
				end				 			 
				else begin 
					 read_cnt<=0;
					 read_finish<=1'b1;
					 myvalid<=1'b0;
				end
			end
		 else begin
			  read_cnt<=0;
			  cntb<=0;
			  read_finish<=1'b0;
			  myvalid<=1'b0;
			  mydata<=0;
		 end
end			

endmodule

