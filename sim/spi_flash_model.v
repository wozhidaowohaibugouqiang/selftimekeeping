`timescale 1ns / 1ps

module spi_flash_model #(
    parameter MEM_BYTES = 65536
)(
    input  wire       sys_clk,
    input  wire       cs_n,
    input  wire       sclk,
    input  wire       mosi,
    output reg        miso,
    output wire [7:0] dbg_mem_1000,
    output wire [7:0] dbg_mem_1001,
    output wire [7:0] dbg_mem_1002
);

    reg [7:0] mem [0:MEM_BYTES-1];

    assign dbg_mem_1000 = mem[16'h1000];
    assign dbg_mem_1001 = mem[16'h1001];
    assign dbg_mem_1002 = mem[16'h1002];

    initial begin
        integer i;
        $display("FLASH_MODEL: Initializing %0d bytes...", MEM_BYTES);
        for (i=0; i<MEM_BYTES; i=i+1) mem[i] = 8'hFF;
        $display("FLASH_MODEL: Initialization done.");
    end

    reg [7:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [7:0] cmd;
    reg [23:0] addr;
    reg [3:0] state; 
    // State encoding:
    // 0: CMD
    // 1: ADDR 23:16
    // 2: ADDR 15:8
    // 3: ADDR 7:0
    // 4: DATA (Read/Write)
    
    reg wel = 0;
    reg wip = 0;

    // Shift In (MOSI)
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            bit_cnt <= 0;
            state <= 0;
            cmd <= 0;
        end else begin
            shift_reg <= {shift_reg[6:0], mosi};
            bit_cnt <= bit_cnt + 1;
            
            if (bit_cnt == 7) begin // Byte received
                case (state)
                    0: begin // CMD
                        cmd <= {shift_reg[6:0], mosi};
                        if ({shift_reg[6:0], mosi} == 8'h06) wel <= 1; // WREN
                        else if ({shift_reg[6:0], mosi} == 8'h04) wel <= 0; // WRDI
                        else if ({shift_reg[6:0], mosi} == 8'h02) state <= 1; // PP
                        else if ({shift_reg[6:0], mosi} == 8'h03) state <= 1; // READ
                        else if ({shift_reg[6:0], mosi} == 8'h20) state <= 1; // SE
                        else if ({shift_reg[6:0], mosi} == 8'h05) begin // RDSR
                            // Prepare status to shift out?
                        end
                    end
                    1: begin addr[23:16] <= {shift_reg[6:0], mosi}; state <= 2; end
                    2: begin addr[15:8] <= {shift_reg[6:0], mosi}; state <= 3; end
                    3: begin 
                        addr[7:0] <= {shift_reg[6:0], mosi}; 
                        state <= 4; 
                        
                        // Execute SE (Sector Erase) immediately for simplicity
                        if (cmd == 8'h20 && wel) begin
                            // Erase 4KB sector
                            // For simulation, just loop. NOTE: This is blocking and takes 0 time.
                            // Real flash takes time. Controller polls WIP.
                            // We should set WIP=1 and clear it later?
                            // For now, instant erase.
                             integer k;
                             // Use addr logic
                             // addr & FFF000
                             // Cannot use variable index in simple loop in always block easily
                        end
                    end
                    4: begin
                        if (cmd == 8'h02 && wel) begin // PP
                            mem[addr] <= {shift_reg[6:0], mosi};
                            addr <= addr + 1;
                        end
                    end
                endcase
            end
        end
    end
    
    // Sector Erase Logic (Separate block to handle looping)
    // Triggered by state transition to 4 with cmd 20
    // Using a flag or just doing it in the always block if synthesis allows (this is sim model)
    // ModelSim handles loops fine.
    always @(posedge sclk) begin
        if (!cs_n && bit_cnt == 7 && state == 3 && cmd == 8'h20 && wel) begin
             // Erase
             // Note: using non-blocking assignment for array in loop
             // mem[k] <= 8'hFF
        end
    end

    // MISO Output
    // Mode 0: Data sampled on rising, shifted out on falling.
    reg [7:0] out_byte;
    
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n) begin
            miso <= 1'bz;
            out_byte <= 0;
        end else begin
            if (cmd == 8'h05) begin // RDSR
                // Status Register: 0000_00(WEL)(WIP)
                // If we are here, we received 05.
                // We need to output status.
                // Bit 7 comes out first?
                // RDSR returns 8 bits. MSB first.
                // Status = {6'b0, wel, wip}
                if (bit_cnt == 0) out_byte <= {6'b0, wel, wip}; // Load at byte boundary
                miso <= out_byte[7];
                out_byte <= {out_byte[6:0], 1'b0};
            end else if (state == 4 && cmd == 8'h03) begin // READ
                // Read memory
                if (bit_cnt == 0) begin
                     reg [7:0] load_val;
                     if (addr < MEM_BYTES) begin
                         if (mem[addr] === 8'hxx) 
                             load_val = 8'hFF;
                         else
                             load_val = mem[addr];
                         $display("[%0t] FLASH_MODEL: READ addr=%h data=%h", $time, addr, load_val);
                     end else begin
                         load_val = 8'hFF;
                         $display("[%0t] FLASH_MODEL: READ OOB addr=%h", $time, addr);
                     end
                     miso <= load_val[7];
                     out_byte <= {load_val[6:0], 1'b0};
                end else begin
                    miso <= out_byte[7];
                    out_byte <= {out_byte[6:0], 1'b0};
                end
            end else begin
                miso <= 1'bz;
            end
        end
    end
    
    // Refined Erase Logic
    // We can just check the condition
    always @(negedge cs_n) begin
        // Reset logic
    end
    
    // Let's implement Erase properly
    always @(posedge sclk) begin
        if (!cs_n && bit_cnt == 7 && state == 3) begin
            if (cmd == 8'h20 && wel) begin
                // Sector Erase
                // We use a task or loop
                erase_sector({addr[23:8], shift_reg[6:0], mosi});
            end
        end
    end
    
    task erase_sector;
        input [23:0] s_addr;
        integer k;
        reg [23:0] base;
        begin
            base = s_addr & 24'hFFF000;
            for (k=0; k<4096; k=k+1) begin
                mem[base + k] = 8'hFF;
            end
            //$display("Flash Sector Erase at %h", base);
        end
    endtask

endmodule
