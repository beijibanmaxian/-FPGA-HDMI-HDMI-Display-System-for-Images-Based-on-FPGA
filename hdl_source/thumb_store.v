// ============================================================================
// 模块名 : thumb_store.v
// 功能   : 转场缩略图缓冲（双 bank，每 bank 128x96，RGB332 各 1 字节）
//          - 写口：显示当前帧时每 5x5 像素采样一点写入
//          - 读口：换图后读取上一张图的缩略图做淡入淡出混合
// 容量   : 2 x 12288 x 8bit = 196608 bit（约 22 个 9Kb ERAM 块）
//          用 RGB332 而不是 RGB565，是为了把 ERAM 占用压到 EG4S20 的 64 块限制内
//
// 工程   : HX4S20C 选题一 自有工程（topic1_self_engineered）V3.0
// 日期   : 2026-10-17
// ============================================================================
module thumb_store (
    input  wire        clk,
    input  wire        we,
    input  wire [14:0] waddr,
    input  wire [7:0]  wdata,
    input  wire [14:0] raddr,
    output reg  [7:0]  rdata
);

    reg [7:0] mem [0:24575];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

endmodule
