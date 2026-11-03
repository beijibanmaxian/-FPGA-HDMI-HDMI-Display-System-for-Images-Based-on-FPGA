// ============================================================================
// 模块名 : audio_stream_player.v
// 功能   : 从 TF 卡流式播放 48kHz/16bit **单声道** PCM，输出标准 I2S（64fs）
//
// 素材格式（与 tools/make_media_assets.py 一致）：
//   文件头 54 字节（伪 BMP 头，供扫描器识别），
//   之后是 **单声道** 16bit 小端 PCM 样本流，48kHz。
//
// ★ 关键：素材是单声道！每个 I2S 帧只消费 **2 字节**，把这个样本同时送到
//   L 和 R。原实现按"立体声 4 字节/帧"消费，等于每次吃掉两个样本 →
//   播放速度 2 倍、音高升八度、并且是抽 1 点不滤波的降采样（混叠）→
//   "语速变快、调变了、音色变了、很嘈杂"。
//
// 结构：
//   · clk 域（100MHz）：经 sd_bus_arb 以扇区为单位取数据，每 2 字节拼一个
//     16bit 样本，复制成 {s,s} 写入异步 FIFO（32bit/帧）。
//   · FIFO：1024 x 32bit（约 21ms 音频），Gray 码指针跨时钟域。
//   · audio_mclk 域（12.288MHz）：完全照官方 hdmi_audio_tone_i2s_64fs 的时序，
//     BCLK = mclk/2 = 6.144MHz，每 64 个 BCLK 切一次 LRCK，128 BCLK/帧 = 48kHz。
// ============================================================================
module audio_stream_player #(
    parameter integer FIFO_AW = 10              // FIFO 深度 = 2^FIFO_AW 个立体声帧
)(
    input  wire        clk,                     // 100MHz（SD 卡域）
    input  wire        rst,

    input  wire        reload,                  // 换曲：重新从 start_sector 开始
    input  wire [31:0] start_sector,
    input  wire [31:0] sector_count,
    input  wire        loop_en,                 // 图片音轨循环；视频音轨只播一遍
    input  wire [12:0] byte_off,                // 文件头字节数（54），必须跳过
    input  wire [3:0]  volume,                  // 0=静音 8=原始音量 15≈2x

    // SD 读总线（接 sd_bus_arb 的 c0）
    output reg         sd_req,
    output reg  [31:0] sd_req_addr,
    input  wire        sd_gnt,
    input  wire [7:0]  rd_data,
    input  wire        rd_valid,
    input  wire        rd_end,

    // I2S 输出
    input  wire        audio_mclk,              // 12.288MHz
    input  wire        audio_rst,
    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT,

    // 直接输出 PCM 样本对（audio_mclk 域，1 拍 = 一个样本）
    // 走这条通路给 HDMI 音频包，避免 "PCM->I2S->接收器" 回环的位级重采样风险
    output reg         O_pcm_valid,
    output reg  [23:0] O_pcm_l,
    output reg  [23:0] O_pcm_r,
    // 下游（audio_pcm_bridge）还有空位才出样本；否则保持 have_frame=0 等待，
    // 绝不"出了却装不进去"（否则样本被丢 = 杂音/破音）
    input  wire        I_pcm_ready
);

    localparam integer DEPTH = (1 << FIFO_AW);

    // ==================================================================
    // 一、异步 FIFO（写：clk 域；读：audio_mclk 域）
    // ==================================================================
    reg  [31:0] fifo [0:DEPTH-1];

    reg  [FIFO_AW:0] wbin, wgray;
    reg  [FIFO_AW:0] rbin, rgray;
    reg  [FIFO_AW:0] rgray_s0, rgray_s1;        // clk 域同步读指针
    reg  [FIFO_AW:0] wgray_s0, wgray_s1;        // audio 域同步写指针

    reg  [31:0] fifo_rdata;
    reg         rd_take;

    reg  [31:0] wr_data;
    reg         wr_en;

    // 同步过来的读指针是 Gray 码，必须换算回二进制才能做减法。
    //（直接 wbin - rgray 会得到垃圾值，导致 full / fifo_room 判断全错：
    //  FIFO 满了还继续写 → 数据被覆盖 → 声音刺啦；
    //  又因为总以为还有空位而不断申请扇区 → 长期霸占 SD 总线 → 图片饿死黑屏。）
    function [FIFO_AW:0] bin_of_gray;
        input [FIFO_AW:0] g;
        integer k;
        begin
            bin_of_gray[FIFO_AW] = g[FIFO_AW];
            for (k = FIFO_AW - 1; k >= 0; k = k - 1)
                bin_of_gray[k] = bin_of_gray[k + 1] ^ g[k];
        end
    endfunction

    function [FIFO_AW:0] gray_of;
        input [FIFO_AW:0] b;
        begin
            gray_of = b ^ (b >> 1);
        end
    endfunction

    wire [FIFO_AW:0] rbin_sync = bin_of_gray(rgray_s1);
    wire [FIFO_AW:0] wused     = wbin - rbin_sync;
    wire             full      = (wused == DEPTH[FIFO_AW:0]);
    wire             empty     = (rgray == wgray_s1);

    wire [FIFO_AW:0] wbin_nxt = wbin + {{FIFO_AW{1'b0}}, 1'b1};
    wire [FIFO_AW:0] rbin_nxt = rbin + {{FIFO_AW{1'b0}}, 1'b1};

    always @(posedge clk) begin
        if (wr_en)
            fifo[wbin[FIFO_AW-1:0]] <= wr_data;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wbin      <= {(FIFO_AW+1){1'b0}};
            wgray     <= {(FIFO_AW+1){1'b0}};
            rgray_s0  <= {(FIFO_AW+1){1'b0}};
            rgray_s1  <= {(FIFO_AW+1){1'b0}};
        end else begin
            rgray_s0 <= rgray;
            rgray_s1 <= rgray_s0;
            if (wr_en) begin
                wbin  <= wbin_nxt;
                wgray <= gray_of(wbin_nxt);
            end
        end
    end

    always @(posedge audio_mclk or posedge audio_rst) begin
        if (audio_rst) begin
            rbin       <= {(FIFO_AW+1){1'b0}};
            rgray      <= {(FIFO_AW+1){1'b0}};
            wgray_s0   <= {(FIFO_AW+1){1'b0}};
            wgray_s1   <= {(FIFO_AW+1){1'b0}};
            fifo_rdata <= 32'd0;
        end else begin
            wgray_s0 <= wgray;
            wgray_s1 <= wgray_s0;
            if (!empty)
                fifo_rdata <= fifo[rbin[FIFO_AW-1:0]];
            if (rd_take && !empty) begin
                rbin  <= rbin_nxt;
                rgray <= gray_of(rbin_nxt);
            end
        end
    end

    // ==================================================================
    // 二、clk 域：按扇区取数，2 字节拼一个单声道样本
    // ==================================================================
    reg [31:0] cur_sec;
    reg [31:0] sec_left;
    reg [31:0] sec_base;
    reg [31:0] sec_total;
    reg        loop_latched;
    reg [7:0]  b0;
    reg        half;             // 0=等样本低字节 1=等高字节（单声道 16bit）
    reg        fetching;
    reg [12:0] skip_left;

    // 只填到一半，留出安全余量（音频请求因此是"间歇"的，不会长期霸占总线）
    wire fifo_room = (wused < DEPTH[FIFO_AW:0] / 2);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sd_req      <= 1'b0;
            sd_req_addr <= 32'd0;
            cur_sec     <= 32'd0;
            sec_left    <= 32'd0;
            sec_base    <= 32'd0;
            sec_total   <= 32'd0;
            loop_latched<= 1'b0;
            b0          <= 8'd0;


            half        <= 1'b0;
            skip_left   <= 13'd0;
            wr_en       <= 1'b0;
            wr_data     <= 32'd0;
            fetching    <= 1'b0;
        end else begin
            wr_en <= 1'b0;

            if (reload) begin
                sec_base  <= start_sector;
                sec_total <= sector_count;
                loop_latched <= loop_en;
                cur_sec   <= start_sector;
                sec_left  <= sector_count;
                half        <= 1'b0;
                skip_left <= byte_off;
                sd_req    <= 1'b0;
                fetching  <= 1'b0;
            end else if (sec_total == 32'd0) begin
                sd_req   <= 1'b0;
                fetching <= 1'b0;
            end else begin
                if (!fetching && fifo_room && (sec_left != 32'd0)) begin
                    sd_req_addr <= cur_sec;
                    sd_req      <= 1'b1;
                    fetching    <= 1'b1;
                    half        <= 1'b0;
                end

                if (fetching && sd_gnt)
                    sd_req <= 1'b0;

                // 音频素材是 **单声道 16bit 48kHz**（song1.wav: ch=1 rate=48000 bits=16，
                // 2880000 字节 = 30.000 秒，与用户原始文件逐字节一致）。
                // 所以每个 I2S 帧只消费 2 字节（一个样本），再复制到左右声道。
                // 之前误按"立体声 4 字节/帧"处理，等效采样率翻倍 → 30s 的歌 15s 播完、
                // 人声像开了变声器（变调 + 变速 + 刺啦）。
                if (fetching && rd_valid) begin
                    if (skip_left != 13'd0) begin
                        skip_left <= skip_left - 13'd1;
                    end else if (!half) begin
                        b0   <= rd_data;
                        half <= 1'b1;
                    end else begin
                        half <= 1'b0;
                        if (!full) begin
                            wr_en   <= 1'b1;
                            // 单声道样本同时写入 L 与 R
                            wr_data <= {rd_data, b0, rd_data, b0};
                        end
                    end
                end

                if (fetching && rd_end) begin
                    fetching <= 1'b0;
                    half        <= 1'b0;
                    if (sec_left == 32'd1) begin
                        if (loop_latched) begin
                            cur_sec   <= sec_base;
                            sec_left  <= sec_total;
                            skip_left <= byte_off;
                        end else begin
                            sec_left  <= 32'd0;
                            sec_total <= 32'd0;
                            sd_req    <= 1'b0;
                            fetching  <= 1'b0;
                        end
                    end else begin
                        cur_sec  <= cur_sec + 32'd1;
                        sec_left <= sec_left - 32'd1;
                    end
                end
            end
        end
    end

    // ==================================================================
    // 三、audio_mclk 域：I2S 发送（时序与官方 tone 模块一致）
    // ==================================================================
    reg [5:0]  bit_cnt;
    reg [63:0] shreg;
    reg signed [23:0] smp_l, smp_r;
    reg        have_frame;
    reg [3:0]  vol_s0, vol_s1;

    function signed [23:0] apply_vol;
        input signed [23:0] s;
        input [3:0]         v;
        reg signed [27:0] t;
        begin
            if (v == 4'd0)
                apply_vol = 24'sd0;
            else begin
                t = s * $signed({1'b0, v});      // 原始 *v
                t = t >>> 3;                     // /8 → v=8 时为 1.0x
                // 24bit 左对齐样本的满量程是 ±8388608（不是 ±4194304，
                // 后者只有半量程）——按 ±4194304 饱和会在音量 >8 时提前削波
                if (t > 28'sd8388607)       apply_vol = 24'sd8388607;
                else if (t < -28'sd8388608) apply_vol = -24'sd8388608;
                else                        apply_vol = t[23:0];
            end
        end
    endfunction

    wire signed [23:0] l24 = {fifo_rdata[15:0],  8'd0};
    wire signed [23:0] r24 = {fifo_rdata[31:16], 8'd0};

    always @(posedge audio_mclk or posedge audio_rst) begin
        if (audio_rst) begin
            O_i2s_BCLK <= 1'b0;
            O_i2s_LRCK <= 1'b0;
            O_i2s_DOUT <= 1'b0;
            bit_cnt    <= 6'd0;
            shreg      <= 64'd0;
            smp_l      <= 24'sd0;
            smp_r      <= 24'sd0;
            have_frame <= 1'b0;
            rd_take    <= 1'b0;
            vol_s0     <= 4'd8;
            vol_s1     <= 4'd8;
            O_pcm_valid <= 1'b0;
            O_pcm_l     <= 24'd0;
            O_pcm_r     <= 24'd0;
        end else begin
            rd_take <= 1'b0;
            vol_s0  <= volume;
            vol_s1  <= vol_s0;

            O_i2s_BCLK <= ~O_i2s_BCLK;

            if (O_i2s_BCLK == 1'b1) begin
                O_i2s_DOUT <= shreg[63];
                shreg      <= {shreg[62:0], 1'b0};

                if (bit_cnt == 6'd63) begin
                    bit_cnt <= 6'd0;
                    if (O_i2s_LRCK == 1'b0) begin
                        // 左声道发完 → 发右声道
                        O_i2s_LRCK <= 1'b1;
                        shreg      <= {smp_r, 40'd0};
                    end else begin
                        // 右声道发完 → 用当前缓存发左声道
                        O_i2s_LRCK <= 1'b0;
                        shreg      <= have_frame ? {smp_l, 40'd0} : 64'd0;
                        if (have_frame) have_frame <= 1'b0;
                    end
                end else begin
                    bit_cnt <= bit_cnt + 6'd1;
                end
            end

            // 常态化预装下一帧（优先级高于上面的"消耗"分支，因此消耗的永远是
            // 上一拍装好的旧值，逻辑自洽）。必须等下游有空位（I_pcm_ready）才出，
            // 否则样本会丢失（这是"人声像变声器/沙沙杂音"的根源之一）。
            O_pcm_valid <= 1'b0;
            if (!have_frame && !empty && I_pcm_ready) begin
                smp_l       <= apply_vol(l24, vol_s1);
                smp_r       <= apply_vol(r24, vol_s1);
                O_pcm_l     <= apply_vol(l24, vol_s1);
                O_pcm_r     <= apply_vol(r24, vol_s1);
                O_pcm_valid <= 1'b1;
                rd_take     <= 1'b1;
                have_frame  <= 1'b1;
            end
        end
    end

endmodule
