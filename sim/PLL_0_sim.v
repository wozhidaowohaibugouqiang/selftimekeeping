`timescale 1ns/1ps

module PLL_0 (
    input  refclk,
    input  reset,
    output reg extlock,
    output reg clk0_out
);

    // 100MHz clock generation (Period = 10ns)
    initial begin
        clk0_out = 1'b0;
        extlock  = 1'b0;
        
        // Wait for reset release simulation
        wait(reset == 1'b0); // Reset is active high in PLL_0 (from top.v: ~rst_n)
                             // Wait, top.v: .reset(~rst_n). So reset is high active.
                             // If reset is high, PLL is reset.
                             
        #500; // Lock time
        extlock = 1'b1;
    end

    // Toggle every 5ns -> 10ns period -> 100MHz
    always #5 clk0_out = ~clk0_out;

endmodule
