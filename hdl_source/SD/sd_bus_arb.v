// ============================================================================
// 模块名 : sd_bus_arb.v
// 功能   : TF 卡扇区读总线的三客户端仲裁
//
// 三个消费者共用一套 sd_card_top：
//   c0 = 音频流播放（优先级最高，但 FIFO 快满时自己就不请求了）
//   c1 = 视频帧读取（640x480 输出，需要连续带宽）
//   c2 = 图片扫描/加载缩放（最低，用户按键时才抢一下）
//
// 仲裁粒度是"一个扇区"：一旦授权，就保持到该扇区读完（sd_end），
// 期间地址由客户端自己保持在 sd_req_addr 上。请求是电平信号：
//   客户端置 sd_req=1 → 等 sd_gnt=1 → 本扇区数据在 cX_data/cX_valid 上流出
//   → 收到 cX_end=1 表示本扇区结束，客户端可以改地址继续请求。
//
// 防饿死：音频连续拿到 STREAK_MAX 个授权后会被强制让出一次，
// 避免音频 FIFO 长期"刚好没满"把视频/图片彻底饿死。
//
// ★ 这个值原来是 8，等于允许音频连吃 8 个扇区（约 1.5ms）才让一次。
//   音频在每次 reload 后要把环形缓冲填到一半，会**连发十几个扇区**
//   （约 3ms），期间视频/图片只能拿到 1/9 的带宽 ——
//   于是 video_stream_reader 的 600us 看门狗、bmp_read_scaler 的 3ms
//   读超时都被触发：视频那几帧被"补黑"、图片被"用一个像素填满"，
//   表现就是「视频开头黑屏」「轮播几轮后某张图颜色完全变了」。
//   音频只需要约 3.5% 的带宽，让成 2 个扇区（约 0.4ms）就能彻底消掉饿死。
// ============================================================================
module sd_bus_arb #(
    parameter [3:0] STREAK_MAX = 4'd2
)(
    input  wire        clk,
    input  wire        rst,

    // ---------------- 客户端 0：音频 ----------------
    input  wire        c0_req,
    input  wire [31:0] c0_addr,
    output wire        c0_gnt,
    output wire [7:0]  c0_data,
    output wire        c0_valid,
    output wire        c0_end,

    // ---------------- 客户端 1：视频 ----------------
    input  wire        c1_req,
    input  wire [31:0] c1_addr,
    output wire        c1_gnt,
    output wire [7:0]  c1_data,
    output wire        c1_valid,
    output wire        c1_end,

    // ---------------- 客户端 2：图像 ----------------
    input  wire        c2_req,
    input  wire [31:0] c2_addr,
    output wire        c2_gnt,
    output wire [7:0]  c2_data,
    output wire        c2_valid,
    output wire        c2_end,

    // ---------------- 到 sd_card_top ----------------
    output reg         sd_read,
    output reg  [31:0] sd_addr,
    input  wire [7:0]  sd_data,
    input  wire        sd_valid,
    input  wire        sd_end
);

    localparam [1:0] C_AUD = 2'd0;
    localparam [1:0] C_VID = 2'd1;
    localparam [1:0] C_IMG = 2'd2;

    // 扇区之间留几拍空隙：控制器要回到 S_WAIT_READ_WRITE 才能接下一次请求，
    // 同时保证客户端在这几拍里已把 sd_req_addr 更新成新扇区。
    localparam [2:0] GAP_MAX = 3'd4;

    reg [1:0] cur;          // 当前被授权的客户端
    reg       active;       // 正在传输中（授权有效）
    reg [2:0] gap;          // 扇区之间的强制空隙
    reg [3:0] streak;       // 音频连续授权计数

    wire aud_blocked = (streak >= STREAK_MAX);

    assign c0_gnt   = active && (cur == C_AUD);
    assign c1_gnt   = active && (cur == C_VID);
    assign c2_gnt   = active && (cur == C_IMG);

    assign c0_data  = sd_data;
    assign c1_data  = sd_data;
    assign c2_data  = sd_data;

    assign c0_valid = sd_valid && (cur == C_AUD);
    assign c1_valid = sd_valid && (cur == C_VID);
    assign c2_valid = sd_valid && (cur == C_IMG);

    assign c0_end   = sd_end && (cur == C_AUD);
    assign c1_end   = sd_end && (cur == C_VID);
    assign c2_end   = sd_end && (cur == C_IMG);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cur    <= C_IMG;
            active <= 1'b0;
            gap    <= 3'd0;
            streak <= 4'd0;
            sd_read <= 1'b0;
            sd_addr <= 32'd0;
        end else begin
            if (active) begin
                if (sd_end) begin
                    active  <= 1'b0;
                    sd_read <= 1'b0;
                    gap     <= GAP_MAX;
                end
            end else if (gap != 3'd0) begin
                gap <= gap - 3'd1;
            end else begin
                // 静态优先级：音频 > 视频 > 图像
                if (c0_req && !aud_blocked) begin
                    cur     <= C_AUD;
                    active  <= 1'b1;
                    sd_addr <= c0_addr;
                    sd_read <= 1'b1;
                    if (streak != 4'd15) streak <= streak + 4'd1;
                end else if (c1_req) begin
                    cur     <= C_VID;
                    active  <= 1'b1;
                    sd_addr <= c1_addr;
                    sd_read <= 1'b1;
                    streak  <= 4'd0;
                end else if (c2_req) begin
                    cur     <= C_IMG;
                    active  <= 1'b1;
                    sd_addr <= c2_addr;
                    sd_read <= 1'b1;
                    streak  <= 4'd0;
                end else begin
                    streak <= 4'd0;
                end
            end
        end
    end

endmodule