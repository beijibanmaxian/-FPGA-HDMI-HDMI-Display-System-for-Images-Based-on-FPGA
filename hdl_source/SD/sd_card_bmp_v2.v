// ============================================================================
// 模块名 : sd_card_bmp_v2.v
// 功能   : TF 卡素材管理与播放调度（官方 sd_card_bmp 思路上的完全重写版本）
//
// 管理的三种素材（靠 BMP 头第 28 字节 biBitCount 区分，见 bmp_read_scaler.v）：
//   · 图片 24bit：任意分辨率，缩放到 640x480 播放
//   · 音频  8bit ：48kHz/16bit 立体声 PCM 流，与"同槽位"图片联动播放
//   · 视频 16bit ：RGB565 原始帧序列，逐帧送进帧缓存，2x 放大到 640x480
// 三类文件的公共标记（保留字段偏移 6..9）：'H' 'X' GEN SLOT
//   GEN  = IMAGE_GEN 参数（换代用：换一批素材就改这个常量，旧素材自然失效）
//   SLOT = 节目号 1..11（每个图片/视频节目与同槽位音轨配对）
//
// 播放列表：槽位 1..9 为图片，槽位 10..11 为视频；播放顺序按槽位号，
//   与 FAT 目录/物理扫描顺序无关。
// 按键：手动切下一条会退出自动轮播；KEY4 事件切换自动轮播。
//       手动模式的视频播完后停在末帧，等 KEY2。
//
// 读总线：三个消费者（图像缩放、视频帧、音频流）经 sd_bus_arb 共用 sd_card_top。
// ============================================================================
module sd_card_bmp_v2 #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd262143,
    parameter [4:0]   SCAN_TARGET_COUNT = 5'd22,
    parameter [7:0]   IMAGE_GEN         = 8'h31,
    parameter integer VIDEO_FPS         = 10,        // 仅文档用途
    parameter [31:0]  VID_PACE_CYC     = 32'd10000000, // 帧间隔（cycles @CLK_FREQ_HZ）
                                                      //   10fps @100MHz
    parameter [15:0]  VID_W             = 16'd320,
    parameter [15:0]  VID_H             = 16'd180,
    parameter [9:0]   VID_BYTE_OFF      = 10'd54,
    parameter [31:0]  VID_FRAME_SECS    = 32'd225,   // 一帧占多少扇区（320x180x2/512）
    parameter [31:0]  VID_HDR_SECS      = 32'd1,
    // 视频画面铺满输出，不额外补上下黑边。
    parameter integer VID_DST_Y0        = 0,
    parameter integer VID_DST_H         = 480,
    parameter [15:0]  VID_FRAMES        = 16'd256,     // 兼容旧工程；实际帧数按文件扇区动态确定
    // 文件应有总扇区数 = 头扇区 + 帧数 × 每帧扇区；用于校验卡上视频是否与本版固件匹配。
    // 不符时**主动不播视频**（只播图片/音频），避免"换了素材没换 bit"导致乱画面。
    parameter [31:0]  VID_TOTAL_SECS    = 32'd57601
)(
    input                       clk,
    input                       rst,

    // 按键事件（由上层 app_ctrl 去抖后产生的单拍低有效脉冲）
    input                       key_next,
    input                       key_auto,
    input  [1:0]                auto_period_sel,   // 轮播周期：0=5s 1=10s 2=15s 3=20s

    output [3:0]                state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  display_valid,
    output reg                  commit_tgl,      // 换素材时翻转（图片切换做滑动转场）
    output [3:0]                img_count,
    output [3:0]                img_idx_o,
    output reg [3:0]            display_idx_o,   // 已提交到显示帧缓存的节目槽位
    output                      img_scaled_o,
    output                      scan_busy_o,
    output                      auto_on,
    output                      load_busy_o,
    output                      item_kind_o,     // 0=图片 1=视频

    // 扫描诊断（供屏上"扫描状态页"显示：卡上到底找到了什么）
    output [3:0]                aud_count,       // 找到的音频轨数（0..4）
    output                      vid_ok,          // 视频项可用（尺寸/帧数校验通过）
    output                      gen_relaxed,     // 1=严格代号没找到，放宽后重扫成功

    // 音频播放器（经仲裁器共用 SD 总线）
    input  wire        aud_req,
    input  wire [31:0] aud_addr,
    output wire        aud_gnt,
    output wire [7:0]  aud_rd_data,
    output wire        aud_rd_valid,
    output wire        aud_rd_end,
    output reg  [31:0] aud_start_sector,
    output reg  [31:0] aud_sector_count,
    output reg         aud_reload,

    // 帧缓存写侧
    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,
    output                      write_req,
    input                       write_req_ack,
    input                       write_data_ready,
    output                      write_en,
    output [31:0]               write_data,

    // TF 卡
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

    function integer sector_width_for_limit;
        input [31:0] max_sector;
        integer bit_pos;
        begin
            sector_width_for_limit = 1;
            for (bit_pos = 0; bit_pos < 32; bit_pos = bit_pos + 1)
                if (max_sector[bit_pos])
                    sector_width_for_limit = bit_pos + 2;
            if (sector_width_for_limit > 32)
                sector_width_for_limit = 32;
        end
    endfunction

    // Include one extra bit for a sector count that reaches the scan limit.
    localparam integer SECTOR_BITS = sector_width_for_limit(SCAN_MAX_SECTOR);
    localparam integer MAX_PROGRAMS = 11;
    localparam integer MAX_AUDIO_SLOTS = 11;

    // ---------------- 状态编码 ----------------
    localparam [2:0] ST_INIT   = 3'd0;
    localparam [2:0] ST_IDLE   = 3'd1;
    localparam [2:0] ST_START  = 3'd2;
    localparam [2:0] ST_IMGWT  = 3'd3;
    localparam [2:0] ST_VIDREQ = 3'd4;
    localparam [2:0] ST_VIDRUN = 3'd5;
    localparam [2:0] ST_VPACE  = 3'd6;


    wire [31:0] VID_PACE = VID_PACE_CYC;
    localparam [29:0] LOAD_WD_MAX = 30'd800_000_000;   // 单个状态最多 8s @100MHz（大图 1.5s，留足余量）
                                                       // 1280x720 图 + 音频抢总线时 1.2s 才读完，
                                                       // 原来的 2s 看门狗会中途放弃 → 按键/轮播看起来失灵

    // ==================================================================
    // 一、内部连线（必须先声明，后面例化要用）
    // ==================================================================
    wire        sd_read;
    wire [31:0] sd_addr;
    wire [7:0]  sd_data;
    wire        sd_valid;
    wire        sd_end;
    wire        sd_init_done;

    wire        img_sd_req;
    wire [31:0] img_sd_addr;
    wire        img_sd_gnt;
    wire [7:0]  img_rd_data;
    wire        img_rd_valid;
    wire        img_rd_end;
    wire        bmp_write_req;
    wire        bmp_data_wr_en;
    wire [23:0] bmp_data;
    wire        img_ready;
    wire        img_scaled;

    wire        scan_done;
    wire        scan_aborted;
    wire        scan_found_valid;
    wire [31:0] scan_found_sector;
    wire [31:0] scan_found_sectors;
    wire [1:0]  scan_found_kind;
    wire [3:0]  scan_found_slot;
    wire [4:0]  scan_found_total;
    wire [31:0] scan_resume;

    wire        vid_sd_req;
    wire [31:0] vid_sd_addr;
    wire        vid_sd_gnt;
    wire [7:0]  vid_rd_data;
    wire        vid_rd_valid;
    wire        vid_rd_end;
    wire        vid_pix_wr_en;
    wire [23:0] vid_pix;
    wire        vid_busy, vid_done;



    // ==================================================================
    // 二、寄存器声明
    // ==================================================================
    reg [2:0]  st;

    // 调度
    reg        scan_start_pulse;
    reg        load_start_pulse;
    reg [SECTOR_BITS-1:0] load_sector;
    wire [31:0] load_sector_ext;
    assign load_sector_ext = {{(32-SECTOR_BITS){1'b0}}, load_sector};
    reg        scan_kicked;
    reg [3:0]  scan_done_d;
    reg        scan_keep_sel;
    reg        scan_resume_pending;
    wire [31:0] scan_cursor_start =
        scan_keep_sel ? scan_resume : SCAN_START_SECTOR;

    reg        first_committed;
    reg        auto_play_en;
    reg [14:0] auto_cnt;
    reg [16:0] auto_subcnt;
    reg [SECTOR_BITS-1:0] vid_off;
    reg [15:0] vid_fno;               // 已播到第几帧（0 基）
    reg [31:0] vid_pace;             // 帧时钟分频器（0..VID_PACE）
    reg [15:0] vid_due;              // 绝对时间轴：现在应该播到第几帧
    reg        frame_rdy;            // 视频帧已写进帧缓存、还没上屏（读帧与上屏解耦）

    reg [1:0]  pending_buf;
    reg        new_item;
    reg        next_pend;          // 单深度 pending 标志（不做队列，避免连按后一次性冲出去）

    reg [2:0]  wrfin_sync;
    reg [29:0]       load_wd;
    reg [2:0]        wd_st;          // 超时判定用的上一状态

    // 兜底：严格（按素材代号）扫描一张都没找到时，放宽一次重扫，
    // 避免卡上还是上一批素材时直接黑屏（宁可显示旧素材，也不要没画面）
    reg        gen_any_r;
    // retry_done 已删除：不再放宽素材代号

    reg        vid_start;
    reg        vid_src_done;
    reg        frame_toggle_start;
    reg  [SECTOR_BITS-1:0]  vid_base_sector;
    wire [31:0] vid_base_sector_ext;
    assign vid_base_sector_ext = {{(32-SECTOR_BITS){1'b0}}, vid_base_sector};
    reg        vid_wr_req;

    // 素材表按节目槽位索引，不依赖 FAT 文件物理扫描顺序。
    reg [SECTOR_BITS-1:0] program_sector [0:MAX_PROGRAMS-1];
    reg [SECTOR_BITS-1:0] program_secs [0:MAX_PROGRAMS-1];
    reg [MAX_PROGRAMS-1:0] program_valid;
    reg [MAX_PROGRAMS-1:0] program_video;
    reg [SECTOR_BITS-1:0] aud_sector [0:MAX_AUDIO_SLOTS-1];
    reg [SECTOR_BITS-1:0] aud_secs   [0:MAX_AUDIO_SLOTS-1];
    reg [MAX_AUDIO_SLOTS-1:0] aud_valid;

    reg [SECTOR_BITS-1:0] vid_off_max;
    localparam [SECTOR_BITS:0] VID_FRAME_SECTORS = VID_FRAME_SECS;
    localparam [31:0] VID_MIN_SECTORS = VID_HDR_SECS + VID_FRAME_SECS;
    wire [SECTOR_BITS:0] vid_next_frame_end =
        {1'b0, vid_off} + VID_FRAME_SECTORS;
    wire vid_next_frame_fits =
        ({1'b0, vid_off_max} >= vid_next_frame_end);
    reg        vid_found;
    reg        vid_bad_size;      // 视频尺寸/帧数与固件参数不符（状态页显示用）

    reg [3:0]  img_found_cnt;
    reg [3:0]  program_found_cnt;
    reg [3:0]  cur_idx;
    reg        cur_is_vid;

    // 扫描结果打一拍（把跨模块长组合链从 100MHz 关键路径上摘掉）
    reg        sc_valid_r;
    reg [1:0]  sc_kind_r;
    reg [SECTOR_BITS-1:0] sc_sec_r;
    reg [SECTOR_BITS-1:0] sc_secs_r;
    reg [3:0]  sc_m1;
    reg        sc_slot_in;

    // 当前项译码结果
    reg [SECTOR_BITS-1:0] sel_program_sec;
    reg [SECTOR_BITS-1:0] sel_program_secs;
    reg        sel_program_video;
    reg [SECTOR_BITS-1:0] sel_aud_sec;
    reg [SECTOR_BITS-1:0] sel_aud_cnt;
    reg        sel_aud_ok;

    // sel_* 是组合译码（组合链很长：img_found_cnt → 循环比较 → 存储器选择）。
    // 在 ST_START 里再直接用它会形成 100MHz 下的最差路径（实测 -3.7ns）。
    // 这里先打一拍，ST_START 改成两拍：第一拍只等寄存值稳定，第二拍才真正发命令。
    reg [SECTOR_BITS-1:0] q_program_sec;
    reg [SECTOR_BITS-1:0] q_program_secs;
    reg        q_program_video;
    reg [SECTOR_BITS-1:0] q_aud_sec;
    reg [SECTOR_BITS-1:0] q_aud_cnt;
    reg        q_aud_ok;
    reg        start_ph;
    reg        aud_pending;      // 本项待切换的曲子（等画面提交时一起切）
    reg        vid_stopped;      // 视频被 KEY2 手动停止（再按一次才跳到下一项）

    integer i;

    // ==================================================================
    // 三、工具函数（必须在引用它们的语句之前）
    // ==================================================================
    function [3:0] first_valid_program;
        input [3:0] unused_arg;
        integer slot;
        reg found;
        begin
            first_valid_program = 4'd0;
            found = 1'b0;
            for (slot = 0; slot < MAX_PROGRAMS; slot = slot + 1) begin
                if (!found && program_valid[slot]) begin
                    first_valid_program = slot[3:0];
                    found = 1'b1;
                end
            end
        end
    endfunction

    function [3:0] next_valid_program;
        input [3:0] cur;
        integer offset;
        integer slot;
        reg found;
        begin
            next_valid_program = cur;
            found = 1'b0;
            for (offset = 1; offset <= MAX_PROGRAMS; offset = offset + 1) begin
                slot = cur + offset;
                if (slot >= MAX_PROGRAMS)
                    slot = slot - MAX_PROGRAMS;
                if (!found && program_valid[slot]) begin
                    next_valid_program = slot[3:0];
                    found = 1'b1;
                end
            end
        end
    endfunction

    function [31:0] period_cycles;
        input [1:0] sel;
        begin
            case (sel)
                2'd0:    period_cycles = CLK_FREQ_HZ * 5;      // 广告牌场景用秒级间隔：
                2'd1:    period_cycles = CLK_FREQ_HZ * 10;     // 5 / 10 / 15 / 20 秒
                2'd2:    period_cycles = CLK_FREQ_HZ * 15;     // （菜单里可调）
                default: period_cycles = CLK_FREQ_HZ * 20;
            endcase
        end
    endfunction

    function [14:0] period_milliseconds;
        input [1:0] sel;
        begin
            case (sel)
                2'd0:    period_milliseconds = 15'd5000;
                2'd1:    period_milliseconds = 15'd10000;
                2'd2:    period_milliseconds = 15'd15000;
                default: period_milliseconds = 15'd20000;
            endcase
        end
    endfunction

    localparam integer CYCLES_PER_MS =
        (CLK_FREQ_HZ >= 1000) ? (CLK_FREQ_HZ / 1000) : 1;
    wire [14:0] auto_period_ms = period_milliseconds(auto_period_sel);

    reg [1:0] auto_period_q;
    wire auto_period_changed = (auto_period_sel != auto_period_q);

    // ==================================================================
    // 四、三个消费者 + 仲裁 + 卡控制器
    // ==================================================================
    bmp_read_scaler #(
        .SCAN_GAP (16384),
        .STOP_AFTER_FIRST_IMAGE (1)
    ) bmp_read_m0 (
        .clk                    (clk),
        .rst                    (rst),
        .ready                  (img_ready),
        .scan_start             (scan_start_pulse),
        .scan_start_sector      (scan_cursor_start),
        .scan_max_sector        (SCAN_MAX_SECTOR),
        .scan_target_count      (SCAN_TARGET_COUNT),
        .image_gen              (IMAGE_GEN),
        .image_gen_any          (gen_any_r),
        .scan_keep              (scan_keep_sel),
        .scan_done              (scan_done),
        .scan_aborted           (scan_aborted),
        .scan_found_valid       (scan_found_valid),
        .scan_found_sector      (scan_found_sector),
        .scan_found_sectors     (scan_found_sectors),
        .scan_found_kind        (scan_found_kind),
        .scan_found_slot        (scan_found_slot),
        .scan_found_total       (scan_found_total),
        .scan_resume            (scan_resume),
        .load_start             (load_start_pulse),
        .load_sector            (load_sector_ext),
        .sd_init_done           (sd_init_done),
        .state_code             (state_code),
        .bmp_width              (bmp_width),
        .bmp_height             (bmp_height),
        .scaled_o               (img_scaled),
        .write_req              (bmp_write_req),
        .write_req_ack          (write_req_ack),
        .write_data_ready       (write_data_ready),
        .sd_req                 (img_sd_req),
        .sd_req_addr            (img_sd_addr),
        .rd_data                (img_rd_data),
        .rd_valid               (img_rd_valid),
        .rd_end                 (img_rd_end),
        .bmp_data_wr_en         (bmp_data_wr_en),
        .bmp_data               (bmp_data)
    );

    video_stream_reader #(
        .OUT_W  (640),
        .OUT_H  (480),
        .DST_Y0 (VID_DST_Y0),
        .DST_H  (VID_DST_H),
        .TOP_EDGE_CROP_ROWS (0),
        .LB_AW  (10)
    ) u_vid (
        .clk         (clk),
        .rst         (rst),
        .start       (vid_start),
        .base_sector (vid_base_sector_ext),
        .byte_off    ({3'd0, VID_BYTE_OFF}),
        .src_w       (VID_W[12:0]),
        .src_h       (VID_H[12:0]),
        .vflip       (1'b0),   // 视频素材在转换时已按 BMP 习惯写成自下而上，这里不再翻转
        .sd_req      (vid_sd_req),
        .sd_addr     (vid_sd_addr),
        .sd_gnt      (vid_sd_gnt),
        .rd_data     (vid_rd_data),
        .rd_valid    (vid_rd_valid),
        .rd_end      (vid_rd_end),
        .pix_wr_ready(write_data_ready),
        .pix_wr_en   (vid_pix_wr_en),
        .pix_data    (vid_pix),
        .busy        (vid_busy),
        .done        (vid_done)
    );

    sd_bus_arb u_arb (
        .clk      (clk),
        .rst      (rst),
        .c0_req   (aud_req),
        .c0_addr  (aud_addr),
        .c0_gnt   (aud_gnt),
        .c0_data  (aud_rd_data),
        .c0_valid (aud_rd_valid),
        .c0_end   (aud_rd_end),
        .c1_req   (vid_sd_req),
        .c1_addr  (vid_sd_addr),
        .c1_gnt   (vid_sd_gnt),
        .c1_data  (vid_rd_data),
        .c1_valid (vid_rd_valid),
        .c1_end   (vid_rd_end),
        .c2_req   (img_sd_req),
        .c2_addr  (img_sd_addr),
        .c2_gnt   (img_sd_gnt),
        .c2_data  (img_rd_data),
        .c2_valid (img_rd_valid),
        .c2_end   (img_rd_end),
        .sd_read  (sd_read),
        .sd_addr  (sd_addr),
        .sd_data  (sd_data),
        .sd_valid (sd_valid),
        .sd_end   (sd_end)
    );

    sd_card_top sd_card_top_m0 (
        .clk                    (clk),
        .rst                    (rst),
        .SD_nCS                 (SD_nCS),
        .SD_DCLK                (SD_DCLK),
        .SD_MOSI                (SD_MOSI),
        .SD_MISO                (SD_MISO),
        .sd_init_done           (sd_init_done),
        .sd_sec_read            (sd_read),
        .sd_sec_read_addr       (sd_addr),
        .sd_sec_read_data       (sd_data),
        .sd_sec_read_data_valid (sd_valid),
        .sd_sec_read_end        (sd_end),
        .sd_sec_write           (1'b0),
        .sd_sec_write_addr      (32'd0),
        .sd_sec_write_data      (8'd0),
        .sd_sec_write_data_req  (),
        .sd_sec_write_end       ()
    );

    // app_ctrl 已完成机械按键去抖。同步器后的下降沿检测保留单拍事件，
    // 避免长脉冲重复触发，也能让短事件跨模块可靠传递。
    reg key_next_s0, key_next_s1, key_next_prev;
    reg key_auto_s0, key_auto_s1, key_auto_prev;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            key_next_s0 <= 1'b1;
            key_next_s1 <= 1'b1;
            key_next_prev <= 1'b1;
            key_auto_s0 <= 1'b1;
            key_auto_s1 <= 1'b1;
            key_auto_prev <= 1'b1;
        end else begin
            key_next_s0 <= key_next;
            key_next_s1 <= key_next_s0;
            key_next_prev <= key_next_s1;
            key_auto_s0 <= key_auto;
            key_auto_s1 <= key_auto_s0;
            key_auto_prev <= key_auto_s1;
        end
    end

    wire key_next_press = key_next_prev & ~key_next_s1;
    wire key_auto_press = key_auto_prev & ~key_auto_s1;

    // ==================================================================
    // 五、写侧多路选择（图片走 scaler，视频走帧读取器）
    // ==================================================================
    assign write_req  = cur_is_vid ? vid_wr_req   : bmp_write_req;
    assign write_en   = cur_is_vid ? vid_pix_wr_en: bmp_data_wr_en;
    assign write_data = cur_is_vid ? {vid_pix, 8'b0} : {bmp_data, 8'b0};

    // ==================================================================
    // 六、对外状态
    // ==================================================================
    wire [3:0] item_total = program_found_cnt;
    wire       wrfin_pulse = wrfin_sync[2] ^ wrfin_sync[1];

    assign img_count   = img_found_cnt;
    assign img_idx_o   = cur_idx;
    assign img_scaled_o= img_scaled;
    assign item_kind_o = cur_is_vid;
    // 诊断输出
    assign aud_count    = {3'b000, aud_valid[0]} + {3'b000, aud_valid[1]} +
                          {3'b000, aud_valid[2]} + {3'b000, aud_valid[3]} +
                          {3'b000, aud_valid[4]} + {3'b000, aud_valid[5]} +
                          {3'b000, aud_valid[6]} + {3'b000, aud_valid[7]} +
                          {3'b000, aud_valid[8]} + {3'b000, aud_valid[9]} +
                          {3'b000, aud_valid[10]};
    assign vid_ok       = vid_found;
    // gen_relaxed 现在只表示"视频文件尺寸/帧数与本版固件不符"（该视频已被主动跳过）
    assign gen_relaxed  = vid_bad_size;
    assign load_busy_o = (st != ST_IDLE) && (st != ST_INIT);
    assign auto_on     = auto_play_en;
    assign scan_busy_o = !scan_kicked || (scan_done_d != 4'd8);

    wire [31:0] auto_period_cycles = period_cycles(auto_period_sel);
    wire        auto_timer_run = auto_play_en && display_valid &&
                                 first_committed && !cur_is_vid &&
                                 (item_total > 4'd1) && (st == ST_IDLE) &&
                                 !auto_period_changed &&
                                 !key_auto_press && !key_next_press;
    wire        auto_ms_tick = auto_timer_run &&
                               (auto_subcnt >= (CYCLES_PER_MS - 1));
    wire        auto_tick = auto_ms_tick &&
                            (auto_cnt >= (auto_period_ms - 15'd1));

    // ==================================================================
    // 七、当前节目和同槽音轨的扇区译码
    // ==================================================================
    always @* begin
        sel_program_sec = 32'd0;
        sel_program_secs = 32'd0;
        sel_program_video = 1'b0;
        sel_aud_sec = 32'd0;
        sel_aud_cnt = 32'd0;
        sel_aud_ok  = 1'b0;
        for (i = 0; i < MAX_PROGRAMS; i = i + 1) begin
            if (cur_idx == i[3:0]) begin
                sel_program_sec = program_sector[i];
                sel_program_secs = program_secs[i];
                sel_program_video = program_video[i];
                sel_aud_sec = aud_sector[i];
                sel_aud_cnt = aud_secs[i];
                sel_aud_ok  = aud_valid[i];
            end
        end
    end

    // 把选择结果打一拍（见上方说明）
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            q_program_sec <= 32'd0;
            q_program_secs <= 32'd0;
            q_program_video <= 1'b0;
            q_aud_sec <= 32'd0;
            q_aud_cnt <= 32'd0;
            q_aud_ok  <= 1'b0;
        end else begin
            q_program_sec <= sel_program_sec;
            q_program_secs <= sel_program_secs;
            q_program_video <= sel_program_video;
            q_aud_sec <= sel_aud_sec;
            q_aud_cnt <= sel_aud_cnt;
            q_aud_ok  <= sel_aud_ok;
        end
    end

    // ==================================================================
    // 八、主状态机
    // ==================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st               <= ST_INIT;
            scan_start_pulse <= 1'b0;
            load_start_pulse <= 1'b0;
            load_sector      <= 32'd0;
            scan_kicked      <= 1'b0;
            scan_done_d      <= 4'd0;
            scan_keep_sel    <= 1'b0;
            scan_resume_pending <= 1'b0;
            first_committed  <= 1'b0;
            auto_play_en     <= 1'b0;
            auto_cnt         <= 15'd0;
            auto_subcnt      <= 17'd0;
            auto_period_q    <= 2'd3;
            vid_off          <= 32'd0;
            vid_fno   <= 16'd0;
            vid_pace         <= 32'd0;
            vid_due          <= 16'd0;
            frame_rdy        <= 1'b0;
            pending_buf      <= 2'd0;
            new_item         <= 1'b0;
            next_pend        <= 1'b0;
            wrfin_sync       <= 3'b000;
            load_wd          <= 30'd0;
            wd_st            <= ST_INIT;
            gen_any_r        <= 1'b0;

            start_ph         <= 1'b0;
            aud_pending      <= 1'b0;
            vid_stopped      <= 1'b0;
            display_valid    <= 1'b0;
            commit_tgl       <= 1'b0;
            write_buf_idx    <= 2'd0;
            disp_buf_idx     <= 2'd0;
            cur_idx          <= 4'd0;
            display_idx_o    <= 4'd0;
            cur_is_vid       <= 1'b0;
            img_found_cnt    <= 4'd0;
            program_found_cnt <= 4'd0;
            vid_wr_req       <= 1'b0;
            vid_start        <= 1'b0;
            vid_src_done     <= 1'b0;
            frame_toggle_start <= 1'b0;
            vid_base_sector  <= 32'd0;
            aud_start_sector <= 32'd0;
            aud_sector_count <= 32'd0;
            aud_reload       <= 1'b0;

            vid_found        <= 1'b0;
            vid_bad_size     <= 1'b0;
            vid_off_max      <= 32'd0;
            program_valid    <= {MAX_PROGRAMS{1'b0}};
            program_video    <= {MAX_PROGRAMS{1'b0}};
            aud_valid        <= {MAX_AUDIO_SLOTS{1'b0}};
            sc_valid_r       <= 1'b0;
            sc_kind_r        <= 2'd0;
            sc_sec_r         <= 32'd0;
            sc_secs_r        <= 32'd0;
            sc_m1            <= 4'd0;
            sc_slot_in       <= 1'b0;
            for (i = 0; i < MAX_PROGRAMS; i = i + 1) begin
                program_sector[i] <= 32'd0;
                program_secs[i] <= 32'd0;
            end
            for (i = 0; i < MAX_AUDIO_SLOTS; i = i + 1) begin
                aud_sector[i] <= 32'd0;
                aud_secs[i]   <= 32'd0;
            end
        end else begin
            wrfin_sync       <= {wrfin_sync[1:0], write_finish_toggle};
            scan_start_pulse <= 1'b0;
            load_start_pulse <= 1'b0;
            vid_start        <= 1'b0;
            aud_reload       <= 1'b0;

            if (auto_period_changed) begin
                auto_period_q <= auto_period_sel;
                auto_cnt      <= 15'd0;
                auto_subcnt   <= 17'd0;
            end

            // ---------------- 扫描结果打一拍 ----------------
            sc_valid_r <= 1'b0;
            if (scan_found_valid) begin
                sc_valid_r <= 1'b1;
                sc_kind_r  <= scan_found_kind;
                sc_sec_r   <= scan_found_sector;
                sc_secs_r  <= scan_found_sectors;
                sc_m1      <= scan_found_slot - 4'd1;
                sc_slot_in <= (scan_found_slot >= 4'd1) &&
                              (scan_found_slot <= MAX_PROGRAMS[3:0]);
            end

            // ---------------- 扫描结果入库 ----------------
            if (sc_valid_r) begin
                case (sc_kind_r)
                    2'd0: begin     // 图片
                        if (sc_slot_in && !program_valid[sc_m1]) begin
                            program_sector[sc_m1] <= sc_sec_r;
                            program_secs[sc_m1] <= sc_secs_r;
                            program_video[sc_m1] <= 1'b0;
                            program_valid[sc_m1] <= 1'b1;
                            program_found_cnt <= program_found_cnt + 4'd1;
                            if (img_found_cnt < 4'd9)
                                img_found_cnt <= img_found_cnt + 4'd1;
                        end
                    end
                    2'd1: begin     // 音频
                        if (sc_slot_in) begin
                            aud_sector[sc_m1] <= sc_sec_r;
                            aud_secs[sc_m1]   <= sc_secs_r;
                            aud_valid[sc_m1]  <= 1'b1;
                            if (display_valid && !cur_is_vid &&
                                (sc_m1 == cur_idx) && !aud_valid[sc_m1] &&
                                !aud_pending) begin
                                aud_start_sector <= sc_sec_r;
                                aud_sector_count <= sc_secs_r;
                                aud_reload       <= 1'b1;
                            end
                        end
                    end
                    default: begin  // 视频
                        if (sc_slot_in && !program_valid[sc_m1] &&
                            (sc_secs_r >= VID_MIN_SECTORS)) begin
                            program_sector[sc_m1] <= sc_sec_r;
                            program_secs[sc_m1] <= sc_secs_r;
                            program_video[sc_m1] <= 1'b1;
                            program_valid[sc_m1] <= 1'b1;
                            program_found_cnt <= program_found_cnt + 4'd1;
                            vid_found <= 1'b1;
                        end else begin
                            vid_bad_size <= 1'b1;   // 供状态页显示
                        end
                    end
                endcase
            end

            // ---------------- 扫描完成稳定计数 ----------------
            if (scan_done) begin
                if (scan_done_d != 4'd8) scan_done_d <= scan_done_d + 4'd1;
            end else begin
                scan_done_d <= 4'd0;
            end

            // ---------------- 状态超时保护 ----------------
            // 只在"瞬态等待"状态计时；ST_IDLE 正常空闲、ST_VPACE 合法等帧，
            // 都不能计时（否则 43s 的视频会在 8s 时被强行打断 = 视频时长不足的根因）
            if (st != wd_st) begin
                wd_st    <= st;
                load_wd  <= 30'd0;
            end else if ((st == ST_START) || (st == ST_IMGWT) ||
                         (st == ST_VIDREQ) || (st == ST_VIDRUN)) begin
                if (load_wd == LOAD_WD_MAX) begin
                    // 该素材读不出来（卡坏块 / 尺寸超限 / 总线异常）：
                    // 必须放弃这一项回到空闲，否则整个系统会卡死 -> 按键全失灵
                    load_wd    <= 30'd0;
                    st         <= ST_IDLE;
                    vid_wr_req <= 1'b0;
                end else begin
                    load_wd <= load_wd + 30'd1;
                end
            end else begin
                load_wd <= 30'd0;
            end

            // 视频卡死保护已移除：video_stream_reader 内部有 15ms 的递减看门狗，
            // 而这里若再判 15ms 就会比一帧读卡时间(约 60ms)还短，
            // 会在帧读到一半时被判定"卡死"并重启 -> 视频永远播不出来。

            // ---------------- 视频帧时间轴（自由运行 + 迟到丢帧） ----------------
            // vid_pace 当分频器用：满 VID_PACE 就 vid_due +1（"现在应该播到第几帧"）。
            // vid_due 是**绝对时间轴**，不会被读卡速度拖慢；
            // ST_VPACE 里一旦发现 vid_fno 落后于 vid_due 就空跳丢帧追齐。
            // 这样即使读卡偶尔超时，也只会掉帧，绝不会"越播越慢、跟音频脱节"。
            // （早期版本用 12+16 位两级计数器 + 独立 CATCH 状态，功能对了但 TD 布线器崩溃；
            //   这里复用 vid_pace 本身做分频，不新增状态，资源与风险都最小）
            if ((st == ST_VIDREQ) || (st == ST_VIDRUN) ||
                (st == ST_VPACE) || (st == ST_START)) begin
                if (!vid_stopped) begin
                    if (vid_pace >= VID_PACE) begin
                        vid_pace <= 32'd0;
                        if (vid_due != 16'hFFFF) vid_due <= vid_due + 16'd1;
                    end else begin
                        vid_pace <= vid_pace + 32'd1;
                    end
                end
            end else begin
                vid_pace <= 32'd0;
            end

            // ---------------- 按键 -> 请求队列 ----------------
            // 视频项的特殊处理：
            //   第一次按 KEY2 -> 停止视频（定住最后一帧），不排队
            //   第二次按 KEY2 -> 真正跳到下一项
            if (key_next_press) begin
                auto_play_en <= 1'b0;
                auto_cnt     <= 15'd0;
                auto_subcnt  <= 17'd0;
                if (cur_is_vid && !vid_stopped)
                    vid_stopped <= 1'b1;
                else
                    next_pend   <= 1'b1;   // 加载中按下的也算，空闲时消费一次
            end

            if (key_auto_press) begin
                auto_play_en <= ~auto_play_en;
                auto_cnt     <= 15'd0;
                auto_subcnt  <= 17'd0;
                next_pend    <= 1'b0;      // 切模式时丢掉残留请求，避免刚开就跳一张
            end

            // Count only while an image is fully committed and visible. Pausing
            // during item loads keeps the requested visible dwell time exact.
            if (auto_timer_run) begin
                if (auto_ms_tick) begin
                    auto_subcnt <= 17'd0;
                    if (auto_cnt < (auto_period_ms - 15'd1))
                        auto_cnt <= auto_cnt + 15'd1;
                end else begin
                    auto_subcnt <= auto_subcnt + 17'd1;
                end
            end

            // 到点后在"安全点"排队一次，绝不重入。
            // 只在 ST_IDLE 排队：视频项要**完整播完**再换（视频末帧会自动跳到下一项），
            // 否则自动模式下 5s 就把 43s 的视频切走了。
            if (auto_tick && (st == ST_IDLE) && !next_pend &&
                auto_play_en && (item_total > 4'd1)) begin
                next_pend <= 1'b1;
                auto_cnt  <= 15'd0;
                auto_subcnt <= 17'd0;
            end

            // ---------------- 状态机 ----------------
            case (st)
                ST_INIT: begin
                    if (!scan_kicked && img_ready && sd_init_done) begin
                        scan_kicked      <= 1'b1;
                        scan_keep_sel    <= 1'b0;
                        scan_start_pulse <= 1'b1;
                    end else if ((scan_done_d == 4'd8) && img_ready) begin
                        if (!scan_keep_sel &&
                            (scan_found_total < SCAN_TARGET_COUNT))
                            scan_resume_pending <= 1'b1;
                        if (item_total != 4'd0) begin
                            cur_idx    <= first_valid_program(4'd0);
                            new_item   <= 1'b1;
                            vid_off    <= 32'd0;
                            vid_fno   <= 16'd0;
                            st         <= ST_START;
                        end
                        // 认不到就**停在状态页**，绝不放宽代号重扫。
                        // 曾经的"放宽代号再扫一遍"会把握手时卡上残留的旧素材
                        // （别的 GENERATION）收进来播放 —— 外部表现就是
                        // "素材换过了，但屏幕上还是旧图片、音箱里还是旧音乐"。
                        // 现在严格匹配：认不到就显示扫描状态页，其中 GEN 位显示 X，
                        // 一眼就能看出"卡上的素材不是本版固件要的那一批"。
                    end
                end

                ST_IDLE: begin
                    if (scan_resume_pending && img_ready && !vid_busy) begin
                        scan_start_pulse <= 1'b1;
                        scan_keep_sel <= 1'b1;
                        scan_resume_pending <= 1'b0;
                    end else if (next_pend && (item_total != 4'd0) && img_ready && !vid_busy) begin
                        cur_idx       <= next_valid_program(cur_idx);
                        new_item      <= 1'b1;
                        vid_off       <= 32'd0;
                        vid_fno   <= 16'd0;
                        st            <= ST_START;
                        next_pend     <= 1'b0;
                        vid_stopped   <= 1'b0;
                        auto_cnt      <= 15'd0;
                        auto_subcnt   <= 17'd0;
                    end
                end

                ST_START: begin
                    // 两拍：第一拍只等 q_* 寄存值稳定（打断 img_found_cnt 的长组合链），
                    // 第二拍才真正发加载命令。
                    if (!start_ph) begin
                        start_ph <= 1'b1;
                    end else begin
                        start_ph      <= 1'b0;
                        pending_buf   <= first_committed ? (disp_buf_idx ^ 2'd1) : 2'd0;
                        write_buf_idx <= first_committed ? (disp_buf_idx ^ 2'd1) : 2'd0;

                        // 只有"换素材"时才换曲；视频的下一帧不换。
                        // 注意：这里只**登记**，等画面真正提交时再发 aud_reload，
                        // 否则音乐已经换了、图还在读（大图要 1 秒多），音画就对不上。
                        if (new_item) begin
                            new_item    <= 1'b0;
                            aud_pending <= 1'b1;
                            // 每一项的开头都把帧节拍与上屏标志清零：
                            // 视频项必须从"第 0 帧 + 时间轴 0 点"开始，
                            // 否则会带着上一次播放残留的 vid_due 直接空跳掉整段视频。
                            vid_pace    <= 32'd0;
                            vid_due     <= 16'd0;
                            frame_rdy   <= 1'b0;
                            vid_stopped <= 1'b0;
                        end

                        if (q_program_video) begin
                            cur_is_vid      <= 1'b1;
                            vid_base_sector <= q_program_sec + vid_off;
                            vid_off_max     <= q_program_secs - VID_HDR_SECS;
                            // 注意：这里**不要**清帧节拍。逐帧重进 ST_START 时清零会让
                            // vid_pace 永远长不起来，视频一帧都推不动；
                            // 0 点由 ST_VIDRUN 在"第一帧上屏 + 起音频"那一拍统一清。
                            vid_wr_req      <= 1'b1;
                            st              <= ST_VIDREQ;
                        end else begin
                            cur_is_vid       <= 1'b0;
                            load_sector      <= q_program_sec;
                            load_start_pulse <= 1'b1;
                            st               <= ST_IMGWT;
                        end
                    end
                end

                ST_IMGWT: begin
                    if (wrfin_pulse) begin
                        disp_buf_idx    <= pending_buf;
                        display_valid   <= 1'b1;
                        display_idx_o   <= cur_idx;
                        first_committed <= 1'b1;
                        commit_tgl      <= ~commit_tgl;     // 图片切换 → 滑动转场
                        st              <= ST_IDLE;
                        // 画面已提交 → 同一拍换曲，保证"换图"和"换歌"同步
                        if (aud_pending && q_aud_ok) begin
                            aud_start_sector <= q_aud_sec;
                            aud_sector_count <= q_aud_cnt;
                            aud_reload       <= 1'b1;
                        end
                        aud_pending <= 1'b0;
                    end
                end

                ST_VIDREQ: begin
                    if (write_req_ack) begin
                        vid_wr_req <= 1'b0;
                        frame_toggle_start <= wrfin_sync[2];
                        vid_src_done <= 1'b0;
                        vid_start  <= 1'b1;
                        st         <= ST_VIDRUN;
                    end
                end

                ST_VIDRUN: begin
                    // ★ 读帧与上屏彻底分开：
                    //   这里只把这一帧**写进帧缓存**（frame_rdy=1），真正的上屏
                    //   在 ST_VPACE 里按绝对时间轴 vid_due 做。
                    //   原来的写法一收到 vid_done 就切 disp_buf_idx，于是
                    //   "读多慢就显示多慢"，而且边界上会在旧图/半写视频缓冲
                    //   之间反复切换 —— 屏上就是"前面几张图片快速闪过"。
                    if (vid_done)
                        vid_src_done <= 1'b1;

                    // vid_done only means the pixel generator has finished. The
                    // asynchronous FIFO may still contain pixels not yet committed
                    // to SDRAM, so do not expose this buffer until its write toggle
                    // has advanced beyond the value captured at request acceptance.
                    if ((vid_src_done || vid_done) &&
                        (wrfin_sync[2] != frame_toggle_start)) begin
                        vid_off   <= vid_off + VID_FRAME_SECS;
                        vid_fno   <= vid_fno + 16'd1;
                        frame_rdy <= 1'b1;
                        vid_src_done <= 1'b0;
                        st        <= ST_VPACE;
                    end
                end

                ST_VPACE: begin
                    // 手动模式下，视频自然结束或由 KEY2 停止后都停留末帧；
                    // 仅下一次 KEY2 才跳到下一项。
                    if (vid_stopped) begin
                        if (next_pend && img_ready && !vid_busy) begin
                            cur_idx     <= next_valid_program(cur_idx);
                            new_item    <= 1'b1;
                            vid_off     <= 32'd0;
                            vid_fno   <= 16'd0;
                            frame_rdy   <= 1'b0;
                            vid_stopped <= 1'b0;
                            next_pend   <= 1'b0;
                            st          <= ST_START;
                        end
                    end else if (next_pend && img_ready && !vid_busy) begin
                        // 切项前必须等视频解码器空闲，否则帧缓存会被
                        // "图像像素 + 残留视频像素" 混写 -> 画面乱码
                        cur_idx     <= next_valid_program(cur_idx);
                        new_item    <= 1'b1;
                        vid_off     <= 32'd0;
                        vid_fno   <= 16'd0;
                        frame_rdy   <= 1'b0;
                        next_pend   <= 1'b0;
                        st          <= ST_START;
                    end else if (frame_rdy) begin
                        // 2) 有帧待上屏。该帧的序号 = vid_fno-1，它的"到点时刻"是
                        //    vid_due == vid_fno-1。到点就切缓冲，音频也在这**同一拍**
                        //    起播 -> 音画从第 0 帧就对齐。
                        //    没到点就等（不读卡、不动缓冲），所以显示节奏严格由
                        //    固定帧时钟决定，不会因为读卡快慢而忽快忽慢。
                        if ((vid_due > vid_fno) && !aud_pending) begin
                            // 已经落后一帧以上：这一帧直接丢掉（读卡跟不上时才会发生），
                            // 只追时间轴，保证画面不会越来越晚、与音频脱节。
                            // 注意：本项第 0 帧（aud_pending=1）**永远不丢** ——
                            // 它定义时间轴 0 点并同时起播音频，丢了就音画对不齐。
                            frame_rdy <= 1'b0;
                        end else if (({1'b0, vid_due} + 16'd1) >= {1'b0, vid_fno}) begin
                            disp_buf_idx  <= pending_buf;
                            display_valid <= 1'b1;
                            display_idx_o <= cur_idx;
                            first_committed <= 1'b1;
                            frame_rdy     <= 1'b0;
                            if (aud_pending) begin
                                if (q_aud_ok) begin
                                    aud_start_sector <= q_aud_sec;
                                    aud_sector_count <= q_aud_cnt;
                                    aud_reload       <= 1'b1;
                                end
                                aud_pending <= 1'b0;
                                // 以"第 0 帧上屏的这一刻"为时间轴 0 点
                                vid_pace    <= 32'd0;
                                vid_due     <= 16'd0;
                            end
                        end
                    end else if (!vid_next_frame_fits) begin
                        frame_rdy <= 1'b0;
                        if (auto_play_en && !key_next_press) begin
                            cur_idx     <= next_valid_program(cur_idx);
                            new_item    <= 1'b1;
                            vid_off     <= {SECTOR_BITS{1'b0}};
                            vid_fno     <= 16'd0;
                            vid_stopped <= 1'b0;
                            auto_cnt    <= 15'd0;
                            auto_subcnt <= 17'd0;
                            st          <= ST_START;
                        end else begin
                            // 手动模式：末帧保持显示，直到用户再次按 KEY2。
                            vid_stopped <= 1'b1;
                        end
                    end else if (vid_due > vid_fno) begin
                        // 时间轴已经走到这一帧之后了（读卡慢/总线被音频占用）：
                        // 直接把这一帧空跳掉（只推进偏移，不读卡不上屏），
                        // 下一拍继续判断 —— 结果是**只掉帧，绝不拖慢时间轴**，
                        // 也就不会越播越晚、跟音频脱节。
                        vid_off <= vid_off + VID_FRAME_SECS;
                        vid_fno <= vid_fno + 16'd1;
                    end else begin
                        // 到点读下一帧
                        st <= ST_START;
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule

// ============================================================================
// 按键去抖：松开=1，按下=0，稳定 DEBOUNCE_MS 后输出一个周期宽的 press_pulse
// ============================================================================
module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 8
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,
    output reg  press_pulse
);

    localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

    reg button_sync0;
    reg button_sync1;
    reg button_stable;
    reg [31:0] cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            button_sync0  <= 1'b1;
            button_sync1  <= 1'b1;
            button_stable <= 1'b1;
            cnt           <= 32'd0;
            press_pulse   <= 1'b0;
        end else begin
            button_sync0 <= button_in;
            button_sync1 <= button_sync0;
            press_pulse  <= 1'b0;

            if (button_sync1 == button_stable) begin
                cnt <= 32'd0;
            end else begin
                if (cnt >= DEBOUNCE_CYCLES - 1) begin
                    cnt           <= 32'd0;
                    button_stable <= button_sync1;
                    if (button_sync1 == 1'b0)
                        press_pulse <= 1'b1;      // 只在"按下"的瞬间产生脉冲
                end else begin
                    cnt <= cnt + 32'd1;
                end
            end
        end
    end

endmodule
