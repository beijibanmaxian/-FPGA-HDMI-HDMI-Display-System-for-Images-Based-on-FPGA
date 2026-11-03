module uart_tx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD = 115200
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD;

    reg [31:0] bit_timer;
    reg [3:0] bit_index;
    reg [9:0] shift_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx        <= 1'b1;
            busy      <= 1'b0;
            bit_timer <= 32'd0;
            bit_index <= 4'd0;
            shift_reg <= 10'h3ff;
        end else if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                shift_reg <= {1'b1, data, 1'b0};
                tx        <= 1'b0;
                busy      <= 1'b1;
                bit_index <= 4'd0;
                bit_timer <= CLKS_PER_BIT - 1;
            end
        end else if (bit_timer == 32'd0) begin
            if (bit_index == 4'd9) begin
                tx   <= 1'b1;
                busy <= 1'b0;
            end else begin
                tx        <= shift_reg[1];
                shift_reg <= {1'b1, shift_reg[9:1]};
                bit_index <= bit_index + 4'd1;
                bit_timer <= CLKS_PER_BIT - 1;
            end
        end else begin
            bit_timer <= bit_timer - 32'd1;
        end
    end
endmodule
