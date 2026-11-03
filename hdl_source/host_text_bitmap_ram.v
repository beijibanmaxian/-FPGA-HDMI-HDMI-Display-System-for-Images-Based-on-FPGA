module host_text_bitmap_ram (
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [11:0] wr_addr,
    input  wire [7:0]  wr_data,
    input  wire        rd_clk,
    input  wire [11:0] rd_addr,
    output reg  [7:0]  rd_data
);
    reg [7:0] bitmap [0:1919];

    always @(posedge wr_clk) begin
        if (wr_en && (wr_addr < 12'd1920))
            bitmap[wr_addr] <= wr_data;
    end

    always @(posedge rd_clk)
        rd_data <= (rd_addr < 12'd1920) ? bitmap[rd_addr] : 8'd0;
endmodule
