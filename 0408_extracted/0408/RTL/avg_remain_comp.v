`timescale 1ns / 1ps

// 模块名称：avg_remain_comp
// 功能说明：动态补偿步进生成（除法 + 余数均匀分布）
//          每次收到16秒累积误差后，计算 q=diff/16 与 r=diff-q*16。
//          后续每秒输出一次补偿步进：
//            comp_step = q (+/-1 by Bresenham remainder distribution)

module avg_remain_comp (
    input                     clk,
    input                     rst_n,
    input                     diff_update_valid,
    input      signed [31:0]  diff_value,
    input                     sec_tick,
    output reg                comp_valid,
    output reg signed [15:0]  comp_step
);

    reg signed [15:0] q;
    reg signed [4:0]  r;        // -15 ~ 15
    reg signed [5:0]  rem_acc;  // 余数累加器，留1bit裕量

    reg signed [31:0] q_ext;
    reg signed [31:0] r_calc;
    reg signed [6:0]  rem_next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q         <= 16'sd0;
            r         <= 5'sd0;
            rem_acc   <= 6'sd0;
            comp_valid <= 1'b0;
            comp_step <= 16'sd0;
            q_ext     <= 32'sd0;
            r_calc    <= 32'sd0;
            rem_next  <= 7'sd0;
        end else begin
            comp_valid <= 1'b0;

            if (diff_update_valid) begin
                q_ext   = ($signed(diff_value) >>> 4);
                r_calc  = $signed(diff_value) - (q_ext <<< 4);

                q       <= q_ext[15:0];
                r       <= r_calc[4:0];
                rem_acc <= 6'sd0;
            end

            if (sec_tick) begin
                comp_valid <= 1'b1;
                comp_step <= q;

                rem_next = $signed(rem_acc) + $signed(r);
                if (rem_next >= 7'sd16) begin
                    comp_step <= q + 16'sd1;
                    rem_acc   <= $signed(rem_next[5:0]) - 6'sd16;
                end else if (rem_next <= -7'sd16) begin
                    comp_step <= q - 16'sd1;
                    rem_acc   <= $signed(rem_next[5:0]) + 6'sd16;
                end else begin
                    rem_acc <= $signed(rem_next[5:0]);
                end
            end
        end
    end

endmodule
