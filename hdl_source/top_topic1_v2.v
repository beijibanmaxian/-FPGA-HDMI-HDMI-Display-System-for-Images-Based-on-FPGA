// ============================================================================
// 工程名 : HX4S20C 选题一 自有工程 —— topic1_self_engineered V3.0
// 文件   : top_topic1_v2.v（顶层，替换官方 lab_ex5_i2s 的 top_tf_hdmi_audio.v）
// 功能   : 基于安路 EG4S20（HX4S20C 开发板）的 HDMI 多媒体播放系统
//          - TF 卡读取 BMP 图片（支持任意分辨率，自动缩放到 640x480）
//          - SDRAM 双缓冲帧缓存
//          - HDMI 视频显示 + HDMI 内嵌音频输出（多音阶测试音）
//          - 按键：手动切图 / 自动轮播 / 轮播周期 / 亮度对比度参数调节 / OSD 开关
//          - OSD 图层：运行时间、播放模式、亮度对比度数值、底部滚动标语
//          - 切图淡入淡出转场
//          - 音频波形实时叠加
//          - USB-UART 中英文文字叠加 + FPGA 接收确认
//          - SW4..SW1 选择播放状态显示；SW2 独立控制 PC 字幕显示
//          - 右上角常驻（随 OSD 显隐）的安陆 Logo
//
// 相对官方例程的自主实现（“自主编程实现”证据）：
//   1) app_ctrl.v            自有顶层控制状态机（播放模式 / 切图 / 参数分发）
//   2) key_proc.v            自有按键处理（去抖 + 长短按识别）
//   3) video_compose.v       自有显示合成层（亮度对比度 + OSD + 转场 + 波形）
//   4) thumb_store.v         自有转场缩略图缓冲（双 bank）
//   5) osd_font_rom.v        自有 5x7 字模 ROM（自动生成）
//   6) osd_text_rom.v        自有 128 字符滚动标语 ROM（自动生成）
//   7) bmp_read_scaler.v     自有 BMP 读取 + 任意分辨率缩放
//   8) sd_card_bmp_v2.v      自有 TF 多图管理（在官方版本上增加提交事件输出）
//
// 保留的官方底层模块（未改动，保证基础功能稳定）：
//   sys_pll / video_pll / PLL_HDMI_AUDIO / seg_decoder / seg_scan / sdram
//   video_timing_data / video_delay / frame_read_write
//   hdmi_audio_tone_i2s_64fs / I2S_receiver / audio_arc_calculate
//   video_rgb_to_axis_640x480 / startup_pulse / hdmi_1_4b_transmitter_core_wrapper
//   hdmi_phy_wrapper / sd_card_top / sd_card_cmd / sd_card_sec_read_write / spi_master
//
// 端口说明 : HDMI/TF/按键沿用官方引脚；增加 SW1..SW4 与 UART_TX 输出引脚
// 日期   : 2026-10-17
// ============================================================================
module top_topic1_v2(
    input                       clk,          // 板载主时钟
    input                       rst_n,        // 复位（低有效）
    input                       key1,         // 物理 KEY2(B2)：短按切节目；长按进/退菜单
    input                       key2,         // 物理 KEY4(C1)：短按=自动轮播开关；菜单 +（长按连发）
    input                       key3,         // 物理 KEY3(B1)：短按=OSD 开关；菜单 −（长按连发）
    input  [3:0]                sw,           // SW1..SW4 选择数码管字段；SW2 控制字幕显示

    output [5:0]                seg_sel,      // 数码管位选
    output [7:0]                seg_data,     // 数码管段码

    // HDMI TMDS 差分输出（P 端）
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC（EDID 读取）
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF 卡 SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    // USB-UART（串口控制 + PC 文字显示）
    input                       uart_rx,
    output                      uart_tx
);

    // ================= 常量（与官方一致） =================
    parameter MEM_DATA_BITS = 32;
    parameter ADDR_BITS     = 21;
    parameter BUSRT_BITS    = 10;
    parameter FRAME_PIXELS  = 24'd307200;   // 640*480
    parameter BUF0_ADDR     = 24'd0;
    parameter BUF1_ADDR     = FRAME_PIXELS;

    // ================= 时钟/状态信号 =================
    wire Sdr_init_done;
    wire Sdr_init_ref_vld;
    wire Sdr_busy;

    wire sd_card_clk;       // TF 卡时钟域（100MHz）
    wire ext_mem_clk;       // SDRAM 时钟域
    wire ext_mem_clk_sft;
    wire video_clk;         // 像素时钟域
    wire hdmi_5x_clk;       // HDMI 5x 串行时钟

    wire hs, vs, de;
    wire comp_vs, comp_de;      // video_compose 输出（与处理后的像素对齐）

    wire [23:0] vout_data_raw;   // video_delay 原始输出（未处理）
    wire [23:0] vout_data_post;  // 经参数调节/OSD 处理后的像素
    wire [23:0] vout_data;       // 最终送入 HDMI 的像素（未提交首图前为黑）
    wire        display_valid;   // 首图是否已提交显示
    wire [3:0]  img_count;       // 已扫描到的合法 BMP 数量（OSD 显示）
    wire        auto_on;         // 播放器实际的自动轮播状态（OSD 显示）
    wire        img_load_busy;   // 正在加载图片（OSD 显示）

    wire [3:0]  state_code;
    wire [6:0]  seg_data_0;
    wire [6:0]  seg_program_ones_code;
    wire [6:0]  seg_program_tens_code;
    wire [3:0]  seg_program_no;
    wire [3:0]  seg_program_ones;
    wire [3:0]  seg_program_tens;
    wire [3:0]  shown_program_no;
    wire [3:0]  display_idx;
    reg  [3:0]  display_idx_meta_clk, display_idx_sync_clk;
    reg         display_valid_meta_clk, display_valid_sync_clk;
    reg         item_kind_meta_clk, item_kind_sync_clk;
    wire [7:0]  seg_blank = {1'b1, 7'b111_1111};
    wire [7:0]  seg_digit_0;
    wire [7:0]  seg_digit_1;
    wire [7:0]  seg_digit_2;
    wire [7:0]  seg_digit_3;

    wire        video_read_req;
    wire        video_read_req_ack;
    wire        video_read_en;
    wire [31:0] video_read_data;

    wire        sd_card_write_en;
    wire [31:0] sd_card_write_data;
    wire        sd_card_write_req;
    wire        sd_card_write_req_ack;
    wire        write_data_ready;
    wire        frame_write_finish;
    reg         frame_write_toggle_mem;

    wire [1:0]  write_buf_idx;
    wire [1:0]  disp_buf_idx;

    wire App_rd_en;
    wire [ADDR_BITS-1:0] App_rd_addr;
    wire Sdr_rd_en;
    wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
    wire App_wr_en;
    wire [ADDR_BITS-1:0] App_wr_addr;
    wire [MEM_DATA_BITS-1:0] App_wr_din;
    wire [3:0] App_wr_dm;

    wire hs_0, vs_0, de_0;

    // ================= 音频通路 =================
    wire        audio_pll_lock;
    wire        audio_mclk;
    wire        audio_i2s_bclk;
    wire        audio_i2s_lrck;
    wire        audio_i2s_dout;
    wire        audio_valid;
    wire [23:0] audio_left_data;
    wire [23:0] audio_right_data;
    wire        aud_pcm_valid;
    wire [23:0] aud_pcm_l;
    wire [23:0] aud_pcm_r;
    wire [15:0] aud_underrun;
    wire        aud_fifo_ready;
    wire [3:0]  aud_found_cnt;
    wire        vid_ok;
    wire        gen_relaxed;
    wire        acr_valid;
    wire [19:0] acr_cts;
    wire [19:0] acr_n;

    wire        axis_s_user;
    wire        axis_s_valid;
    wire        axis_s_last;
    wire [23:0] axis_s_data;
    wire        axis_s_ready;

    wire        edid_trig;
    wire        edid_valid;
    wire [7:0]  edid_data;

    wire [9:0]  tmds_ch0_data;
    wire [9:0]  tmds_ch1_data;
    wire [9:0]  tmds_ch2_data;
    wire [9:0]  tmds_clk_data;

    // ================= 上电延时与各时钟域复位 =================
    wire [3:0]  sw_cfg;
    reg  [3:0]  sw_meta, sw_sync, sw_stable;
    reg  [20:0] sw_filter_cnt;
    wire reset_request;
    wire rst_sd, rst_mem, rst_video, rst_audio;

    startup_reset_guard #(
        .WAIT_CYCLES(23'd5_000_000) // 100ms @50MHz
    ) u_startup_reset_guard (
        .refclk        (clk),
        .reset_n       (rst_n),
        .pll_lock      (audio_pll_lock),
        .reset_request (reset_request)
    );

    reset_release_sync u_rst_sd (
        .clk(sd_card_clk), .async_reset(reset_request), .reset_out(rst_sd)
    );
    reset_release_sync u_rst_mem (
        .clk(ext_mem_clk), .async_reset(reset_request), .reset_out(rst_mem)
    );
    reset_release_sync u_rst_video (
        .clk(video_clk), .async_reset(reset_request), .reset_out(rst_video)
    );
    reset_release_sync u_rst_audio (
        .clk(audio_mclk), .async_reset(reset_request), .reset_out(rst_audio)
    );
    assign sw_cfg = sw_stable;

    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin
            sw_meta       <= 4'b0000;
            sw_sync       <= 4'b0000;
            sw_stable     <= 4'b0000;
            sw_filter_cnt <= 21'd0;
        end else begin
            sw_meta <= sw;
            sw_sync <= sw_meta;
            if (sw_sync == sw_stable) begin
                sw_filter_cnt <= 21'd0;
            end else if (sw_filter_cnt == 21'd1_999_999) begin
                sw_stable     <= sw_sync;
                sw_filter_cnt <= 21'd0;
            end else begin
                sw_filter_cnt <= sw_filter_cnt + 21'd1;
            end
        end
    end

    // ================= 自有控制层新增信号 =================
    wire        ctrl_key_next_n;    // 单拍低有效事件（切下一张）
    wire        ctrl_key_auto_n;    // 单拍低有效事件（自动轮播开关）
    wire [7:0]  ctrl_brightness;    // 亮度参数
    wire [7:0]  ctrl_contrast;      // 对比度参数
    wire [11:0] ctrl_brt_bcd;       // 亮度十进制三位
    wire [11:0] ctrl_con_bcd;       // 对比度十进制三位
    wire [1:0]  ctrl_work_mode;     // 播放模式（0=手动 1=自动 2=参数）
    wire [2:0]  ctrl_para_item;     // 调节项（0=亮度 1=对比度 2=周期 3=音量 4=OSD 5=转场）
    wire [1:0]  ctrl_auto_period;   // 轮播周期档位（5/10/15/20 秒）
    wire [3:0]  ctrl_volume;        // 音量 0~15
    wire [7:0]  ctrl_vol_bcd;       // 音量十进制两位
    wire        ctrl_osd_en;        // OSD 图层开关
    wire        ctrl_host_text_show;// PC 文字图层开关
    wire [1:0]  ctrl_trans_mode;    // 转场：0=滑动 1=关闭
    wire        ctrl_adj_pulse;     // 参数被调节（浮层提示）
    wire        img_commit_tgl;     // 新图提交事件（转场触发，sd_clk 域）
    wire [3:0]  img_idx;            // 当前节目槽位编号（0 起）
wire        img_scaled;         // 当前图片是否经过缩放（非 640x480 源图）
wire        img_scan_busy;      // 后台仍在补充扫描（TF 卡上还有图没认完）
wire        item_kind;          // 0=图片 1=视频
wire        aud_sd_req;
wire [31:0] aud_sd_addr;
wire        aud_sd_gnt;
wire [7:0]  aud_sd_rd_data;
wire        aud_sd_rd_valid;
wire        aud_sd_rd_end;
wire [31:0] aud_start_sector;
wire [31:0] aud_sector_count;
wire        aud_reload;  // 是否走了宽松扫描（OSD 诊断用）

    // UART 命令脉冲（sd_card_clk 域）
    wire        u_cmd_next, u_cmd_auto, u_cmd_item, u_cmd_inc, u_cmd_dec, u_cmd_osd;
    wire        uart_rx_valid;
    wire [7:0]  uart_rx_data;
    wire        host_text_busy;
    wire        host_text_valid;
    wire        host_text_wr_en;
    wire [11:0] host_text_wr_addr;
    wire [7:0]  host_text_wr_data;
    wire        host_text_ack;
    wire [7:0]  host_text_ack_code;

    // 音频采样（video_clk 域，来自 I2S 接收器）
    // audio_valid / audio_left_data 在下方音频通路中产生

    // ================= 时钟管理（官方 IP，原样保留） =================
    sys_pll sys_pll_m0(
        .refclk     (clk),
        .clk0_out   (sd_card_clk),
        .clk1_out   (ext_mem_clk),
        .clk2_out   (ext_mem_clk_sft),
        .reset      (1'b0)
    );

    video_pll video_pll_m0(
        .refclk     (clk),
        .clk0_out   (video_clk),
        .clk1_out   (hdmi_5x_clk),
        .reset      (1'b0)
    );

    PLL_HDMI_AUDIO u_audio_pll(
        .refclk     (clk),
        .reset      (1'b0),
        .extlock    (audio_pll_lock),
        .clk0_out   (),
        .clk1_out   (),
        .clk2_out   (audio_mclk)
    );

    // ================= mem_clk 域写完成 toggle 同步 =================
    always @(posedge ext_mem_clk or posedge rst_mem) begin
        if (rst_mem)
            frame_write_toggle_mem <= 1'b0;
        else if (frame_write_finish)
            frame_write_toggle_mem <= ~frame_write_toggle_mem;
    end

    // ================= 自有应用控制层（本次核心改动） =================
    // KEY2/KEY4 经 app_ctrl 去抖后，以单拍事件驱动底层播放控制器。
    app_ctrl #(
        .CLK_FREQ_HZ   (100_000_000),
        .LONG_PRESS_MS (800)
    ) u_app_ctrl (
        .clk           (sd_card_clk),
        .rst_n         (~rst_sd),                 // 低有效复位
        .key1_n        (key1),
        .key2_n        (key2),
        .key3_n        (key3),
        .auto_state    (auto_on),        // 轮播实际状态来自 sd_card_bmp_v2，不再本地维护
        .interaction_switch(sw_cfg[1]),
        .key_next_n    (ctrl_key_next_n),
        .key_auto_n    (ctrl_key_auto_n),
        .brightness    (ctrl_brightness),
        .contrast      (ctrl_contrast),
        .brt_bcd       (ctrl_brt_bcd),
        .con_bcd       (ctrl_con_bcd),
        .vol_bcd       (ctrl_vol_bcd),
        .work_mode     (ctrl_work_mode),
        .para_item     (ctrl_para_item),
        .auto_period   (ctrl_auto_period),
        .volume        (ctrl_volume),
        .osd_en        (ctrl_osd_en),
        .host_text_show(ctrl_host_text_show),
        .trans_mode    (ctrl_trans_mode),
        .adj_pulse     (ctrl_adj_pulse),
        .uart_next     (u_cmd_next),
        .uart_auto     (u_cmd_auto),
        .uart_item     (u_cmd_item),
        .uart_inc      (u_cmd_inc),
        .uart_dec      (u_cmd_dec),
        .uart_osd      (u_cmd_osd)
    );

    always @(posedge clk or posedge reset_request) begin
        if (reset_request) begin
            display_idx_meta_clk <= 4'd0;
            display_idx_sync_clk <= 4'd0;
            display_valid_meta_clk <= 1'b0;
            display_valid_sync_clk <= 1'b0;
            item_kind_meta_clk <= 1'b0;
            item_kind_sync_clk <= 1'b0;
        end else begin
            display_idx_meta_clk <= display_idx;
            display_idx_sync_clk <= display_idx_meta_clk;
            display_valid_meta_clk <= display_valid;
            display_valid_sync_clk <= display_valid_meta_clk;
            item_kind_meta_clk <= item_kind;
            item_kind_sync_clk <= item_kind_meta_clk;
        end
    end

    assign seg_program_no = display_idx_sync_clk + 4'd1;
    assign seg_program_tens = (seg_program_no >= 4'd10) ? 4'd1 : 4'd0;
    assign seg_program_ones = (seg_program_no >= 4'd10)
                            ? (seg_program_no - 4'd10) : seg_program_no;
    assign shown_program_no = display_valid_sync_clk ? seg_program_no : 4'd0;

    // UART 输入抗噪：板载串口不接时 RX 脚悬空，噪声会被当成合法帧解出来；
    // 一旦偶尔解出 A（自动轮播），就会在用户毫无操作时切进自动模式，
    // 表现为按 KEY2 却开始自动轮播、轮播顺序乱掉。
    // 这里做 2us 稳定滤波（115200 一个位 8.68us，不影响正常通信）。
    reg uart_rx_f;
    reg [7:0] urx_cnt;
    always @(posedge sd_card_clk or posedge rst_sd) begin
        if (rst_sd) begin
            uart_rx_f <= 1'b1;
            urx_cnt   <= 8'd0;
        end else if (uart_rx == uart_rx_f) begin
            urx_cnt   <= 8'd0;
        end else if (urx_cnt == 8'd199) begin
            uart_rx_f <= uart_rx;
            urx_cnt   <= 8'd0;
        end else begin
            urx_cnt   <= urx_cnt + 8'd1;
        end
    end

    // ================= 串口命令接收（扩展要求 3） =================
    uart_rx #(
        .CLK_FREQ_HZ (100_000_000),
        .BAUD        (115200)
    ) u_uart_rx (
        .clk      (sd_card_clk),
        .rst      (rst_sd),
        .rx       (uart_rx_f),
        .rx_valid (uart_rx_valid),
        .rx_data  (uart_rx_data)
    );

    // ================= 串口文字接收（XCOM 支持 ASCII、UTF-8 与常用 GBK 中文） =================
    host_text_rx #(
        .CLK_FREQ_HZ (100_000_000),
        .TIMEOUT_MS  (200)
    ) u_host_text_rx (
        .clk         (sd_card_clk),
        .rst         (rst_sd),
        .rx_valid    (uart_rx_valid),
        .rx_data     (uart_rx_data),
        .busy        (host_text_busy),
        .text_valid  (host_text_valid),
        .text_wr_en  (host_text_wr_en),
        .text_wr_addr(host_text_wr_addr),
        .text_wr_data(host_text_wr_data),
        .ack_pulse   (host_text_ack),
        .ack_code    (host_text_ack_code)
    );

    uart_tx #(
        .CLK_FREQ_HZ (100_000_000),
        .BAUD        (115200)
    ) u_uart_tx (
        .clk   (sd_card_clk),
        .rst   (rst_sd),
        .start (1'b0),
        .data  (8'd0),
        .tx    (uart_tx),
        .busy  ()
    );

    // 旧版按单字节 opcode 的串口命令已停用：XCOM 里输入的文字会被当成命令，
    // 例如打一个 "A" 就会切到自动轮播。控制请用板上按键。
    assign u_cmd_next = 1'b0;
    assign u_cmd_auto = 1'b0;
    assign u_cmd_item = 1'b0;
    assign u_cmd_inc  = 1'b0;
    assign u_cmd_dec  = 1'b0;
    assign u_cmd_osd  = 1'b0;

    // ================= TF 多图扫描与缓存（自有版本：支持任意分辨率缩放） =================
    sd_card_bmp_v2 #(
        .CLK_FREQ_HZ       (100_000_000),
        .SCAN_START_SECTOR (32'd0),
        .SCAN_MAX_SECTOR   (32'd262143),   // 前 128 MiB；覆盖全部 TF_FPGA 素材
        .SCAN_TARGET_COUNT (5'd22),       // 9 张图 + 11 条音轨 + 2 个视频
        .IMAGE_GEN         (8'h31),       // 与 tools/make_ad_assets.py 的 GEN 一致
        .VIDEO_FPS         (10),
        .VID_PACE_CYC      (32'd10000000), // 10fps；320x180 降低读卡带宽
        .VID_W             (16'd320),
        .VID_H             (16'd180),
        .VID_DST_Y0        (0),            // 视频拉伸铺满 640x480，不补黑边
        .VID_DST_H         (480),
        .VID_FRAME_SECS    (32'd225),      // 320x180 RGB565 帧的固定扇区步长
        .VID_HDR_SECS      (32'd1),
        .VID_FRAMES        (16'd256),      // 保留旧接口；每段视频的帧数按素材扇区数动态读取
        .VID_TOTAL_SECS    (32'd57601)
    ) sd_card_bmp_m0(
        .clk               (sd_card_clk),
        .rst               (rst_sd),
        .key_next          (ctrl_key_next_n),   // ← 由 app_ctrl 虚拟按键驱动
        .key_auto          (ctrl_key_auto_n),   // ← 由 app_ctrl 虚拟按键驱动
        .auto_period_sel   (ctrl_auto_period),   // 轮播周期档位
        .state_code        (state_code),
        .bmp_width         (16'd640),
        .bmp_height        (16'd480),
        .display_valid     (display_valid),
        .commit_tgl        (img_commit_tgl),
        .img_count         (img_count),
        .img_idx_o         (img_idx),
        .display_idx_o     (display_idx),
        .img_scaled_o      (img_scaled),
        .scan_busy_o       (img_scan_busy),
        .auto_on           (auto_on),
        .load_busy_o       (img_load_busy),
        .item_kind_o       (item_kind),
        .aud_count         (aud_found_cnt),
        .vid_ok            (vid_ok),
        .gen_relaxed       (gen_relaxed),

        // 音频播放器：把它的总线请求送进来仲裁，并把授权后的数据还给它
        .aud_req           (aud_sd_req),
        .aud_addr          (aud_sd_addr),
        .aud_gnt           (aud_sd_gnt),
        .aud_rd_data       (aud_sd_rd_data),
        .aud_rd_valid      (aud_sd_rd_valid),
        .aud_rd_end        (aud_sd_rd_end),
        .aud_start_sector  (aud_start_sector),
        .aud_sector_count  (aud_sector_count),
        .aud_reload        (aud_reload),

        .write_finish_toggle(frame_write_toggle_mem),
        .write_buf_idx     (write_buf_idx),
        .disp_buf_idx      (disp_buf_idx),

        .write_req         (sd_card_write_req),
        .write_req_ack     (sd_card_write_req_ack),
        .write_data_ready  (write_data_ready),
        .write_en          (sd_card_write_en),
        .write_data        (sd_card_write_data),
        .SD_nCS            (sd_ncs),
        .SD_DCLK           (sd_dclk),
        .SD_MOSI           (sd_mosi),
        .SD_MISO           (sd_miso)
    );

    // ================= 数码管：拨码选择独立播放状态，不复用 KEY 功能 =================
    seg_decoder seg_decoder_m0(
        .bin_data          (state_code),
        .seg_data          (seg_data_0)
    );

    seg_decoder seg_decoder_program(
        .bin_data          (seg_program_ones),
        .seg_data          (seg_program_ones_code)
    );

    seg_decoder seg_decoder_program_tens(
        .bin_data          (seg_program_tens),
        .seg_data          (seg_program_tens_code)
    );

    assign seg_digit_0 = (sw_cfg[3] && (shown_program_no != 4'd0))
                       ? {1'b1, seg_program_ones_code} : seg_blank;
    assign seg_digit_1 = sw_cfg[3] && (shown_program_no >= 4'd10)
                       ? {1'b1, seg_program_tens_code}
                       : (!sw_cfg[3] && sw_cfg[2] && display_valid_sync_clk)
                       ? {1'b1, (item_kind_sync_clk ? 7'b100_0001 : 7'b111_1001)} : seg_blank;
    assign seg_digit_2 = sw_cfg[1] && display_valid_sync_clk
                       ? {1'b1, (ctrl_work_mode == 2'd1 ? 7'b000_1000 :
                                 ctrl_work_mode == 2'd2 ? 7'b000_1100 : 7'b000_1001)}
                       : seg_blank;
    assign seg_digit_3 = sw_cfg[0] ? {1'b1, seg_data_0} : seg_blank;

    seg_scan seg_scan_m0(
        .clk               (clk),
        .rst_n             (~reset_request),
        .seg_sel           (seg_sel),
        .seg_data          (seg_data),
        .seg_data_0        (seg_digit_0),
        .seg_data_1        (seg_digit_1),
        .seg_data_2        (seg_digit_2),
        .seg_data_3        (seg_digit_3),
        .seg_data_4        (seg_blank),
        .seg_data_5        (seg_blank)
    );

    // ================= 图像时序与帧缓存（官方，原样保留） =================
    video_timing_data video_timing_data_m0(
        .video_clk         (video_clk),
        .rst               (rst_video),
        .read_req          (video_read_req),
        .read_req_ack      (video_read_req_ack),
        .hs                (hs_0),
        .vs                (vs_0),
        .de                (de_0)
    );

    video_delay video_delay_m0(
        .video_clk         (video_clk),
        .rst               (rst_video),
        .read_en           (video_read_en),
        .read_data         (video_read_data[31:8]),
        .hs                (hs_0),
        .vs                (vs_0),
        .de                (de_0),
        .hs_r              (hs),
        .vs_r              (vs),
        .de_r              (de),
        .vout_data         (vout_data_raw)
    );

    // ================= 自有显示合成层（本次核心改动） =================
    // 官方原句：assign vout_data = display_valid ? vout_data_raw : 24'd0;
    // 本工程改为：亮度/对比度调节 + 转场混合 + OSD 图层 + 音频波形，
    //             再统一门控黑屏。
    video_compose #(
        .H_ACTIVE     (640),
        .V_ACTIVE     (480),
        .SLIDE_FRAMES (20),
        .SEC_CYCLES   (25_000_000)
    ) u_video_compose (
        .clk          (video_clk),
        .rst          (rst_video),
        .vs           (vs),
        .de           (de),
        .rgb_in       (vout_data_raw),

        .brightness   (ctrl_brightness),
        .contrast     (ctrl_contrast),
        .brt_bcd      (ctrl_brt_bcd),
        .con_bcd      (ctrl_con_bcd),
        .work_mode    (ctrl_work_mode),
        .para_item    (ctrl_para_item),
        .auto_period  (ctrl_auto_period),
        .volume       (ctrl_volume),
        .vol_bcd      (ctrl_vol_bcd),
        .img_count    (img_count),
        .img_idx      (img_idx),
        .img_scaled   (img_scaled),
        .scan_busy    (img_scan_busy),
        .item_kind    (item_kind),
        .auto_on      (auto_on),
        .load_busy    (img_load_busy),
        .osd_show     (ctrl_osd_en),
        .trans_mode   (ctrl_trans_mode),
        .adj_pulse    (ctrl_adj_pulse),

        .commit_tgl   (img_commit_tgl),
        .audio_valid  (audio_valid),
        .audio_sample (audio_left_data),
        .disp_valid   (display_valid),
        .aud_count    (aud_found_cnt),
        .vid_ok       (vid_ok),
        .gen_relaxed  (gen_relaxed),
        .text_wr_clk  (sd_card_clk),
        .text_enable  (host_text_valid & ctrl_host_text_show),
        .text_wr_en   (host_text_wr_en),
        .text_wr_addr (host_text_wr_addr),
        .text_wr_data (host_text_wr_data),

        .rgb_out      (vout_data_post),
        .vs_out       (comp_vs),
        .de_out       (comp_de)
    );

    // 注意：这里**不能**再用 display_valid 把画面压成纯黑。
    // 之前写成 `assign vout_data = display_valid ? vout_data_post : 24'd0;`
    // 会把 video_compose 输出的诊断彩条一起屏蔽掉 ——
    // 于是"HDMI 正常但素材没读到"也表现为纯黑，现场完全无法区分原因。
    // 现在 compose 自己会在无内容时输出彩条，画面直接透出。
    assign vout_data = vout_data_post;

    // ================= SDRAM 帧缓存（官方，原样保留） =================
    frame_read_write #(
        .WRITE_V_FLIP     (1),
        .FRAME_WIDTH      (640),
        .FRAME_HEIGHT     (480)
    ) frame_read_write_m0(
        .mem_clk           (ext_mem_clk),
        .rst               (rst_mem),
        .Sdr_init_done     (Sdr_init_done),
        .Sdr_init_ref_vld  (Sdr_init_ref_vld),
        .Sdr_busy          (Sdr_busy),

        .App_rd_en         (App_rd_en),
        .App_rd_addr       (App_rd_addr),
        .Sdr_rd_en         (Sdr_rd_en),
        .Sdr_rd_dout       (Sdr_rd_dout),

        .read_clk          (video_clk),
        .read_req          (video_read_req),
        .read_req_ack      (video_read_req_ack),
        .read_finish       (),
        .read_addr_0       (BUF0_ADDR),
        .read_addr_1       (BUF1_ADDR),
        .read_addr_2       (24'd0),
        .read_addr_3       (24'd0),
        .read_addr_index   (disp_buf_idx),
        .read_len          (FRAME_PIXELS),
        .read_en           (video_read_en),
        .read_data         (video_read_data),

        .App_wr_en         (App_wr_en),
        .App_wr_addr       (App_wr_addr),
        .App_wr_din        (App_wr_din),
        .App_wr_dm         (App_wr_dm),

        .write_clk         (sd_card_clk),
        .write_data_ready  (write_data_ready),
        .write_req         (sd_card_write_req),
        .write_req_ack     (sd_card_write_req_ack),
        .write_finish      (frame_write_finish),
        .write_addr_0      (BUF0_ADDR),
        .write_addr_1      (BUF1_ADDR),
        .write_addr_2      (24'd0),
        .write_addr_3      (24'd0),
        .write_addr_index  (write_buf_idx),
        .write_len         (FRAME_PIXELS),
        .write_en          (sd_card_write_en),
        .write_data        (sd_card_write_data)
    );

    sdram U3(
        .Clk               (ext_mem_clk),
        .Clk_sft           (ext_mem_clk_sft),
        .Rst               (rst_mem),
        .Sdr_init_done     (Sdr_init_done),
        .Sdr_init_ref_vld  (Sdr_init_ref_vld),
        .Sdr_busy          (Sdr_busy),
        .App_wr_en         (App_wr_en),
        .App_wr_addr       (App_wr_addr),
        .App_wr_dm         (App_wr_dm),
        .App_wr_din        (App_wr_din),
        .App_rd_en         (App_rd_en),
        .App_rd_addr       (App_rd_addr),
        .Sdr_rd_en         (Sdr_rd_en),
        .Sdr_rd_dout       (Sdr_rd_dout)
    );

    // ================= 音频：图片联动旋律（扩展要求 2，自有实现） =================
    // 替换官方固定 1kHz 测试音：每张图片对应一段独立旋律，切图时旋律同步切换，
    // 并插入一声提示音；音量由参数菜单 / UART 控制。
    // 音频源：从 TF 卡流式播放 48 kHz 单声道 PCM（真正的音画同步）
    //   · 环形缓冲 4096 样本（85 ms），音频在 SD 仲裁里优先级最高，不会断音
    //   · 切图/切到视频时由 sd_card_bmp_v2 给出 aud_reload + 新的曲子扇区
    audio_stream_player u_audio_stream (
        .clk            (sd_card_clk),
        .rst            (rst_sd),
        .reload         (aud_reload),
        .start_sector   (aud_start_sector),
        .sector_count   (aud_sector_count),
        .loop_en        (~item_kind),
        .byte_off       (13'd54),        // 素材头 54 字节，必须跳过
        .volume         (ctrl_volume),
        .sd_req         (aud_sd_req),
        .sd_req_addr    (aud_sd_addr),
        .sd_gnt         (aud_sd_gnt),
        .rd_data        (aud_sd_rd_data),
        .rd_valid       (aud_sd_rd_valid),
        .rd_end         (aud_sd_rd_end),
        .audio_mclk     (audio_mclk),
        .audio_rst      (rst_audio),
        .O_i2s_BCLK     (audio_i2s_bclk),
        .O_i2s_LRCK     (audio_i2s_lrck),
        .O_i2s_DOUT     (audio_i2s_dout),
        .O_pcm_valid    (aud_pcm_valid),
        .O_pcm_l        (aud_pcm_l),
        .O_pcm_r        (aud_pcm_r),
        .I_pcm_ready    (aud_fifo_ready)
    );

    // 音频样本跨域：audio_mclk -> video_clk，直接搬 PCM（不走 I2S 回环）
    //   原官方做法是 PCM -> I2S -> I2S_receiver -> PCM，用 25MHz 采 6.144MHz 的 BCLK，
    //   只有 4.07 倍过采样，余量几乎为零；一旦边沿漏采就会整帧错位，
    //   听感就是"变调、音色发闷、沙沙噪声"。这里用 Gray 码异步 FIFO 直接搬样本。
    audio_pcm_bridge #(
        // 深度 2^10 = 1024 样本 ≈ 21ms：SD 总线被视频帧/图片占用时，
        // 21ms 缓冲足以盖过仲裁延迟；原来只有 64 样本(1.3ms)，
        // 一旦被拖住就欠载 -> 周期性咔哒/爆音。
        .AW      (10),
        .RCLK_HZ (25_000_000),
        .FS_HZ   (48_000)
    ) u_audio_pcm_bridge (
        .wclk     (audio_mclk),
        .wrst     (rst_audio),
        .wr_valid (aud_pcm_valid),
        .wr_l     (aud_pcm_l),
        .wr_r     (aud_pcm_r),
        .rclk     (video_clk),
        .rrst     (rst_video),
        .rd_valid (audio_valid),
        .rd_l     (audio_left_data),
        .rd_r     (audio_right_data),
        .underrun_cnt (aud_underrun),
        .wr_ready     (aud_fifo_ready)      // 反压：FIFO 没空位就不许播放器出样本
    );

    audio_arc_calculate #(
        .ACR_N         (6144)
    ) u_audio_arc_calculate (
        .I_clk         (video_clk),
        .I_rst         (rst_video),
        .I_audio_valid (audio_valid),
        .O_acr_valid   (acr_valid),
        .O_acr_cts     (acr_cts),
        .O_acr_n       (acr_n)
    );

    // ================= RGB/DE 转 AXIS 视频（官方，原样保留） =================
    video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
        .I_clk         (video_clk),
        .I_rst         (rst_video),
        .I_vs          (comp_vs),
        .I_de          (comp_de),
        .I_rgb         (vout_data),
        .O_video_user  (axis_s_user),
        .O_video_valid (axis_s_valid),
        .O_video_last  (axis_s_last),
        .O_video_data  (axis_s_data)
    );

    // 上电自动打一拍，触发 EDID 读取（官方，原样保留）
    startup_pulse #(
        .CNT_MAX(20'd100000)
    ) u_startup_pulse (
        .I_clk   (video_clk),
        .I_rst   (rst_video),
        .O_pulse (edid_trig)
    );

    // ================= 带音频的 HDMI 1.4b 发射（官方，原样保留） =================
    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE                 ( "EG"       ),
        .HTOTAL                 ( 800        ),
        .HSA                    ( 96         ),
        .HFP                    ( 16         ),
        .HBP                    ( 48         ),
        .HACTIVE                ( 640        ),
        .VTOTAL                 ( 525        ),
        .VSA                    ( 2          ),
        .VFP                    ( 10         ),
        .VBP                    ( 33         ),
        .VACTIVE                ( 480        ),
        .VIDEO_VIC              ( 1          ),
        .VIDEO_TPG              ( "Disable"  ),
        .VIDEO_FORMAT           ( "RGB"      ),
        .AUDIO_SAMPLE_RATE      ( "48K"      ),
        .IIC_SCL_DIV            ( 250        )
    ) u_hdmi_1_4b_transmitter_core_wrapper(
        .I_pixel_clk        (video_clk),
        .I_rst              (rst_video),
        .I_edid_read_trig   (edid_trig),
        .O_edid_read_valid  (edid_valid),
        .O_edid_read_data   (edid_data),

        .I_axis_s_user      (axis_s_user),
        .I_axis_s_valid     (axis_s_valid),
        .I_axis_s_last      (axis_s_last),
        .I_axis_s_data      (axis_s_data),
        .O_axis_s_ready     (axis_s_ready),

        .I_audio_valid      (audio_valid),
        .I_audio_left_data  (audio_left_data),
        .I_audio_right_data (audio_right_data),
        .I_acr_valid        (acr_valid),
        .I_acr_cts          (acr_cts),
        .I_acr_n            (acr_n),

        .O_video_locked     (),
        .O_ddc_scl          (HDMI_DDC_SCL),
        .IO_ddc_sda         (HDMI_DDC_SDA),

        .O_ch0_tmds_data    (tmds_ch0_data),
        .O_ch1_tmds_data    (tmds_ch1_data),
        .O_ch2_tmds_data    (tmds_ch2_data),
        .O_clk_tmds_data    (tmds_clk_data)
    );

    hdmi_phy_wrapper #(
        .DEVICE ( "EG" )
    ) u_hdmi2phy_wrapper(
        .I_pixel_clk        (video_clk),
        .I_serial_clk       (hdmi_5x_clk),
        .I_rst              (rst_video),
        .I_tmds_channel_0   (tmds_ch0_data),
        .I_tmds_channel_1   (tmds_ch1_data),
        .I_tmds_channel_2   (tmds_ch2_data),
        .I_tmds_channel_clk (tmds_clk_data),
        .O_tmds_ch0_p       (HDMI_D0_P),
        .O_tmds_ch1_p       (HDMI_D1_P),
        .O_tmds_ch2_p       (HDMI_D2_P),
        .O_tmds_clk_p       (HDMI_CLK_P)
    );

endmodule

module startup_reset_guard #(
    parameter [22:0] WAIT_CYCLES = 23'd5_000_000
)(
    input  wire refclk,
    input  wire reset_n,
    input  wire pll_lock,
    output wire reset_request
);
    reg pll_lock_meta, pll_lock_sync;
    reg [22:0] wait_count;
    reg startup_ready;

    always @(posedge refclk or negedge reset_n) begin
        if (!reset_n) begin
            pll_lock_meta <= 1'b0;
            pll_lock_sync <= 1'b0;
            wait_count <= 23'd0;
            startup_ready <= 1'b0;
        end else begin
            pll_lock_meta <= pll_lock;
            pll_lock_sync <= pll_lock_meta;
            if (!pll_lock_sync) begin
                wait_count <= 23'd0;
                startup_ready <= 1'b0;
            end else if (!startup_ready) begin
                if (wait_count == WAIT_CYCLES - 23'd1)
                    startup_ready <= 1'b1;
                else
                    wait_count <= wait_count + 23'd1;
            end
        end
    end

    assign reset_request = !reset_n | !pll_lock_sync | !startup_ready;
endmodule

module reset_release_sync(
    input  wire clk,
    input  wire async_reset,
    output wire reset_out
);
    reg [1:0] release_pipe;

    always @(posedge clk or posedge async_reset) begin
        if (async_reset)
            release_pipe <= 2'b11;
        else
            release_pipe <= {release_pipe[0], 1'b0};
    end

    assign reset_out = release_pipe[1];
endmodule
