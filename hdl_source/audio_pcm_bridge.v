// ============================================================================
// 模块名 : audio_pcm_bridge.v
// 功能   : 把 audio_mclk 域解析好的 (L,R) PCM 样本搬到 video_clk 域，
//          并且**严格按 48kHz 的节拍**交给 HDMI 音频包发生器。
//
// 为什么必须"按节拍"给（这是之前所有噪声的根源）：
//   HDMI 音频包发生器每 1/48000 秒只取一个样本（ACR N=6144 固定）。
//   如果像普通 FIFO 那样"只要非空就整块倒出来"，25MHz 的读时钟会在
//   200us 内把一个扇区（128ms）的音频全部抽干，随后长时间空转。
//   发生器只会取走其中极少数样本，其余全部丢弃 ->
//   听感就是"沙沙/嘈杂、速度不对、人声破碎"。
//   所以这里用分数分频产生精确的 48kHz tick，每个 tick 只取 1 个样本。
//
// 与官方 PCM->I2S->I2S_receiver 回环的区别：
//   那条路用 25MHz 采 6.144MHz 的 BCLK，半周期只有 2.03 个时钟，
//   而边沿检测需要 3 拍，天生漏边沿（音色发闷/杂音）。
//   这里用 Gray 码异步 FIFO 直接搬 16bit 样本，位级无损。
// ============================================================================
module audio_pcm_bridge #(
    parameter integer AW      = 6,          // FIFO 深度 = 2^AW 个样本（64 ≈ 1.3ms）
    parameter integer RCLK_HZ = 25_000_000, // 读侧（像素）时钟
    parameter integer FS_HZ   = 48_000      // 音频采样率
)(
    // ---- 写侧：audio_mclk 域（样本到达就写）----
    input  wire        wclk,
    input  wire        wrst,
    input  wire        wr_valid,
    input  wire signed [23:0] wr_l,
    input  wire signed [23:0] wr_r,

    // ---- 读侧：video_clk 域（每 1/FS_HZ 秒取 1 个样本）----
    input  wire        rclk,
    input  wire        rrst,
    output reg         rd_valid,
    output reg  [23:0] rd_l,
    output reg  [23:0] rd_r,
    output reg  [15:0] underrun_cnt,      // 调试：tick 到但没数据（欠载）次数低 16 位
    output wire        wr_ready           // 有空间：上游必须看到它才允许取下一个样本
);

    localparam integer DEPTH = (1 << AW);

    reg [31:0]   mem [0:DEPTH-1];        // {L16, R16}
    reg [AW:0]   wbin, wgray;
    reg [AW:0]   rbin, rgray;
    reg [AW:0]   rgray_s0, rgray_s1;     // 同步到写域
    reg [AW:0]   wgray_s0, wgray_s1;     // 同步到读域
    reg [31:0]   rdata;

    function [AW:0] gray_of;
        input [AW:0] b;
        begin
            gray_of = b ^ (b >> 1);
        end
    endfunction

    function [AW:0] bin_of_gray;
        input [AW:0] g;
        integer k;
        begin
            bin_of_gray[AW] = g[AW];
            for (k = AW - 1; k >= 0; k = k - 1)
                bin_of_gray[k] = bin_of_gray[k + 1] ^ g[k];
        end
    endfunction

    // 输入是 **24bit 左对齐** 样本（与官方 I2S_receiver 输出、HDMI 核心要求一致），
    // 所以这里只能取高 16bit 存 FIFO，绝不能按 16bit 量程去饱和！
    // （曾经写成 sat16 判 ±32767：任何 |样本|>127 都被削到满量程 ->
    //   输出变成方波 -> 音箱里就是"嗡嗡 + 嘈杂"，这正是上一版杂音的真正原因）
    function [15:0] sat16of24;
        input signed [23:0] s;
        begin
            if (s > 24'sd8388352)        sat16of24 = 16'sh7FFF;   // > 0x7FFF00
            else if (s < -24'sd8388608)  sat16of24 = 16'sh8000;
            else                         sat16of24 = s[23:8];
        end
    endfunction

    wire [AW:0] wbin_nxt = wbin + {{AW{1'b0}}, 1'b1};
    wire [AW:0] rbin_nxt = rbin + {{AW{1'b0}}, 1'b1};

    wire [AW:0] rbin_w = bin_of_gray(rgray_s1);
    wire [AW:0] wused  = wbin - rbin_w;
    wire        wfull  = (wused == DEPTH[AW:0]);

    // 给上游的握手：还有空位才允许它出下一个样本（关键！）
    // 播放器的 I2S 帧率约 96kHz，而这里每 1/48kHz 才消费一个；
    // 若不给握手，FIFO 会在 1.3ms 内被灌满，后续样本被静默丢弃 = 嘈杂。
    assign wr_ready = !wfull;

    wire [AW:0] wbin_r = bin_of_gray(wgray_s1);
    wire        rempty = (wbin_r == rbin);

    // ---------------- 写侧 ----------------
    always @(posedge wclk or posedge wrst) begin
        if (wrst) begin
            wbin     <= {(AW+1){1'b0}};
            wgray    <= {(AW+1){1'b0}};
            rgray_s0 <= {(AW+1){1'b0}};
            rgray_s1 <= {(AW+1){1'b0}};
        end else begin
            rgray_s0 <= rgray;
            rgray_s1 <= rgray_s0;
            if (wr_valid && !wfull) begin
                mem[wbin[AW-1:0]] <= {sat16of24(wr_l), sat16of24(wr_r)};
                wbin  <= wbin_nxt;
                wgray <= gray_of(wbin_nxt);
            end
        end
    end

    // ---------------- 读侧：精确 48kHz 节拍 ----------------
    // 累加器：每拍加 FS_HZ，满 RCLK_HZ 就产生一次 tick 并减去 RCLK_HZ。
    // 25MHz / 48kHz = 520.83 -> 平均每 520.83 拍一个 tick，长期无误差。
    reg [31:0] acc;
    reg [31:0] acc_sum;

    always @* acc_sum = acc + FS_HZ[31:0];

    wire tick = (acc_sum >= RCLK_HZ[31:0]);

    always @(posedge rclk or posedge rrst) begin
        if (rrst) begin
            acc          <= 32'd0;
            rbin         <= {(AW+1){1'b0}};
            rgray        <= {(AW+1){1'b0}};
            wgray_s0     <= {(AW+1){1'b0}};
            wgray_s1     <= {(AW+1){1'b0}};
            rd_valid     <= 1'b0;
            rd_l         <= 24'd0;
            rd_r         <= 24'd0;
            rdata        <= 32'd0;
            underrun_cnt <= 16'd0;
        end else begin
            wgray_s0 <= wgray;
            wgray_s1 <= wgray_s0;
            rd_valid <= 1'b0;
            rdata    <= mem[rbin[AW-1:0]];

            if (tick) begin
                if (!rempty) begin
                    rd_l     <= {rdata[31:16], 8'd0};   // 写侧 {L,R}：高半字是 L
                    rd_r     <= {rdata[15:0],  8'd0};
                    rd_valid <= 1'b1;
                    rbin     <= rbin_nxt;
                    rgray    <= gray_of(rbin_nxt);
                end else begin
                    underrun_cnt <= underrun_cnt + 16'd1;
                end
            end

            // 关键：累加器必须**每拍**推进（tick 时回卷），否则 acc 永远停在 0，
            // tick 永远不会产生 -> 一个样本都出不来（曾导致完全没声音）。
            acc <= (acc_sum >= RCLK_HZ[31:0]) ? (acc_sum - RCLK_HZ[31:0]) : acc_sum;
        end
    end

endmodule