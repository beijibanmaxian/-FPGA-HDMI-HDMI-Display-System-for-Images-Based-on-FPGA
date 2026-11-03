// ============================================================================
// 模块名 : video_compose.v
// 功能   : 显示合成层（video_clk 25MHz 单时钟域）
//          1) 实时亮度 / 对比度调节
//          2) 顶部 OSD 状态条（时间 / 模式 / 参数 / 自检指示）
//          3) 参数菜单面板（2 倍字号，右侧弹出，带游标）
//          4) 参数调节时的数值提示浮层（2 倍字号，停留约 2 秒）
//          5) 底部滚动标语（约 1 字符/秒）
//          6) 音频频谱实时叠加（16 段对数频段柱状图，音频驱动）
//          7) 切图滑动转场（新图自右侧滑入，前缘高亮）
//
// 设计说明：
//   · 菜单与浮层用 2 倍字体（每个字形像素放大 2×2），640×480 下清晰可读
//   · 所有面板原点都按 16 像素对齐，避免字形跨格错位（曾出现乱码的原因）
//   · 像素通路两级流水：亮度/对比度一级，最终合成一级
// ============================================================================
module video_compose #(
    parameter H_ACTIVE    = 640,
    parameter V_ACTIVE    = 480,
    parameter SLIDE_FRAMES= 20,
    parameter SEC_CYCLES  = 25_000_000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        vs,
    input  wire        de,
    input  wire [23:0] rgb_in,

    input  wire [7:0]  brightness,
    input  wire [7:0]  contrast,
    input  wire [11:0] brt_bcd,
    input  wire [11:0] con_bcd,
    input  wire [7:0]  vol_bcd,
    input  wire [1:0]  work_mode,
    input  wire [2:0]  para_item,
    input  wire [1:0]  auto_period,
    input  wire [3:0]  volume,
    input  wire [1:0]  trans_mode,
    input  wire [3:0]  img_count,
    input  wire [3:0]  img_idx,         // 当前节目槽位编号（0 起）
    input  wire        img_scaled,      // 当前图片是否经过缩放（OSD 显示 S/F）
    input  wire        scan_busy,       // 后台还在补充扫描（OSD 显示 +）
    input  wire        item_kind,       // 0=图片 1=视频（OSD 显示 IMG/VID）
    input  wire        auto_on,
    input  wire        load_busy,
    input  wire        osd_show,
    input  wire        adj_pulse,       // 参数被调节（弹出数值提示）

    input  wire        commit_tgl,
    input  wire        audio_valid,
    input  wire [23:0] audio_sample,
    input  wire        disp_valid,      // 帧缓存里已经有内容（否则显示"扫描状态页"）
    input  wire [3:0]  aud_count,       // 找到的音频轨数
    input  wire        vid_ok,          // 视频可用（尺寸校验通过）
    input  wire        gen_relaxed,     // 1=放宽代号后才扫到（或视频尺寸不符）
    input  wire        text_wr_clk,
    input  wire        text_enable,
    input  wire        text_wr_en,
    input  wire [11:0] text_wr_addr,
    input  wire [7:0]  text_wr_data,

    output wire [23:0] rgb_out,
    output wire        vs_out,
    output wire        de_out
);

    // ========================================================================
    // 一、像素坐标
    // ========================================================================
    reg [9:0] x;
    reg [8:0] y;
    reg       vs_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x    <= 10'd0;
            y    <= 9'd0;
            vs_d <= 1'b1;
        end else begin
            vs_d <= vs;
            if (de) begin
                if (x == H_ACTIVE - 1) begin
                    x <= 10'd0;
                    y <= (y == V_ACTIVE - 1) ? 9'd0 : (y + 9'd1);
                end else begin
                    x <= x + 10'd1;
                end
            end else begin
                x <= 10'd0;
                if (vs & ~vs_d)
                    y <= 9'd0;
            end
        end
    end

    wire frame_tick = vs & ~vs_d;
    wire [10:0] xx  = {1'b0, x};
    wire [10:0] yy  = {2'b00, y};
    wire no_content = (img_count == 4'd0) || !disp_valid;

    reg [31:0] uptime_cycle_count;
    reg [7:0] uptime_hour_bcd;
    reg [7:0] uptime_minute_bcd;
    reg [7:0] uptime_second_bcd;

    function [7:0] increment_bcd2;
        input [7:0] value;
        begin
            increment_bcd2 = (value[3:0] == 4'd9)
                           ? {value[7:4] + 4'd1, 4'd0}
                           : {value[7:4], value[3:0] + 4'd1};
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            uptime_cycle_count <= 32'd0;
            uptime_hour_bcd <= 8'd0;
            uptime_minute_bcd <= 8'd0;
            uptime_second_bcd <= 8'd0;
        end else if (uptime_cycle_count >= SEC_CYCLES - 1) begin
            uptime_cycle_count <= 32'd0;
            if (uptime_second_bcd == 8'h59) begin
                uptime_second_bcd <= 8'd0;
                if (uptime_minute_bcd == 8'h59) begin
                    uptime_minute_bcd <= 8'd0;
                    uptime_hour_bcd <= (uptime_hour_bcd == 8'h99)
                                     ? 8'd0 : increment_bcd2(uptime_hour_bcd);
                end else begin
                    uptime_minute_bcd <= increment_bcd2(uptime_minute_bcd);
                end
            end else begin
                uptime_second_bcd <= increment_bcd2(uptime_second_bcd);
            end
        end else begin
            uptime_cycle_count <= uptime_cycle_count + 32'd1;
        end
    end

    wire [6:0] logo_x = x - 10'd544;
    wire [5:0] logo_y = y - 9'd4;
    wire [1:0] logo_color;
    wire logo_area = osd_show && (x >= 10'd544) && (x < 10'd624) &&
                     (y >= 9'd4) && (y < 9'd56);

    osd_logo_rom u_logo_rom (
        .x     (logo_x),
        .y     (logo_y),
        .color (logo_color)
    );

    localparam [9:0] HOST_X0 = 10'd16;
    localparam [8:0] HOST_Y0 = 9'd64;
    wire host_text_area = (x >= HOST_X0) && (x < (HOST_X0 + 10'd480)) &&
                          (y >= HOST_Y0) && (y < (HOST_Y0 + 9'd64));
    wire [9:0] host_x_offset = x - HOST_X0;
    wire [8:0] host_y_offset = y - HOST_Y0;
    // A 512-pixel wrap gives the 480-pixel bitmap a 32-pixel gap between repeats.
    reg [8:0] host_scroll_px;
    reg [1:0] host_scroll_div;
    wire [8:0] host_source_x = host_x_offset[8:0] + host_scroll_px;
    wire host_source_valid = host_source_x < 9'd480;
    wire [11:0] host_rd_addr = host_text_area && host_source_valid
        ? ((host_y_offset * 12'd60) + {6'd0, host_source_x[8:3]})
        : 12'd0;
    wire [7:0] host_bitmap_byte;
    reg [2:0] host_text_enable_sync;

    host_text_bitmap_ram u_host_text_bitmap (
        .wr_clk  (text_wr_clk),
        .wr_en   (text_wr_en),
        .wr_addr (text_wr_addr),
        .wr_data (text_wr_data),
        .rd_clk  (clk),
        .rd_addr (host_rd_addr),
        .rd_data (host_bitmap_byte)
    );

    // ========================================================================
    // 三、滑动转场
    // ========================================================================
    reg [2:0] cmt_sync;
    reg [3:0] img_idx_meta, img_idx_sync;
    reg       slide_act;
    reg       slide_vertical;
    reg [5:0] slide_cnt;

    wire cmt_pulse = cmt_sync[2] ^ cmt_sync[1];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cmt_sync <= 3'b000;
            img_idx_meta <= 4'd0;
            img_idx_sync <= 4'd0;
        end else begin
            cmt_sync <= {cmt_sync[1:0], commit_tgl};
            img_idx_meta <= img_idx;
            img_idx_sync <= img_idx_meta;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            slide_act <= 1'b0;
            slide_vertical <= 1'b0;
            slide_cnt <= 6'd0;
        end else if (cmt_pulse) begin
            slide_cnt <= 6'd0;
            slide_act <= (trans_mode == 2'd0);
            slide_vertical <= (img_idx_sync >= 4'd5);
        end else if (slide_act && frame_tick) begin
            if (slide_cnt >= SLIDE_FRAMES[5:0] - 6'd1) begin
                slide_act <= 1'b0;
                slide_cnt <= 6'd0;
            end else begin
                slide_cnt <= slide_cnt + 6'd1;
            end
        end
    end

    localparam integer SLIDE_PX_X = H_ACTIVE / SLIDE_FRAMES;
    localparam integer SLIDE_PX_Y = V_ACTIVE / SLIDE_FRAMES;
    wire [10:0] slide_shift_x = slide_act
                              ? ((H_ACTIVE - SLIDE_PX_X) -
                                 {5'd0, slide_cnt} * SLIDE_PX_X[10:0])
                              : 11'd0;
    wire [10:0] slide_shift_y = slide_act
                              ? ((V_ACTIVE - SLIDE_PX_Y) -
                                 {5'd0, slide_cnt} * SLIDE_PX_Y[10:0])
                              : 11'd0;
    wire        in_new = slide_vertical ? (yy >= slide_shift_y)
                                         : (xx >= slide_shift_x);
    wire        slide_edge = slide_act && in_new &&
                            (slide_vertical ? ((yy - slide_shift_y) < 11'd3)
                                             : ((xx - slide_shift_x) < 11'd3));

    // ========================================================================
    // 四、亮度 / 对比度
    //     out = ((c - 128) * cst >> 7) + brt
    // ========================================================================
    function [7:0] post_chan;
        input [7:0] c;
        input [7:0] brt;
        input [7:0] cst;
        reg signed [8:0]  c_off;
        reg signed [17:0] prod;
        reg signed [17:0] y_tmp;
        reg [7:0]         v;
        begin
            c_off = $signed({1'b0, c}) - 9'sd128;
            prod  = c_off * $signed({1'b0, cst});
            y_tmp = (prod >>> 7) + $signed({1'b0, brt});
            if (y_tmp > 18'sd255)      v = 8'd255;
            else if (y_tmp < 18'sd0)   v = 8'd0;
            else                       v = y_tmp[7:0];
            post_chan = v;
        end
    endfunction

    wire [7:0] adj_r = post_chan(rgb_in[23:16], brightness, contrast);
    wire [7:0] adj_g = post_chan(rgb_in[15:8],  brightness, contrast);
    wire [7:0] adj_b = post_chan(rgb_in[7:0],   brightness, contrast);

    reg [7:0] adj1_r, adj1_g, adj1_b;
    reg       vs_p1, de_p1;
    reg       vs_p2, de_p2;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            adj1_r <= 8'd0; adj1_g <= 8'd0; adj1_b <= 8'd0;
            vs_p1  <= 1'b0; de_p1  <= 1'b0;
            vs_p2  <= 1'b0; de_p2  <= 1'b0;
        end else begin
            adj1_r <= adj_r;
            adj1_g <= adj_g;
            adj1_b <= adj_b;
            vs_p1  <= vs;
            de_p1  <= de;
            vs_p2  <= vs_p1;
            de_p2  <= de_p1;
        end
    end

    wire [7:0] mix_r = slide_edge ? 8'hFF : (in_new ? adj1_r : 8'h00);
    wire [7:0] mix_g = slide_edge ? 8'hFF : (in_new ? adj1_g : 8'h00);
    wire [7:0] mix_b = slide_edge ? 8'hFF : (in_new ? adj1_b : 8'h00);

    // ========================================================================
    // 五、参数值（BCD / 档位）
    // ========================================================================
    wire [3:0] brt_h = brt_bcd[11:8];
    wire [3:0] brt_t = brt_bcd[7:4];
    wire [3:0] brt_o = brt_bcd[3:0];
    wire [3:0] con_h = con_bcd[11:8];
    wire [3:0] con_t = con_bcd[7:4];
    wire [3:0] con_o = con_bcd[3:0];
    // 轮播间隔 5/10/15/20 秒 -> 两位十进制
    wire [3:0] per_tens = (auto_period == 2'd0) ? 4'd0 :
                          (auto_period == 2'd1) ? 4'd1 :
                          (auto_period == 2'd2) ? 4'd1 : 4'd2;
    wire [3:0] per_ones = (auto_period == 2'd0) ? 4'd5 :
                          (auto_period == 2'd1) ? 4'd0 :
                          (auto_period == 2'd2) ? 4'd5 : 4'd0;
    wire [3:0] img_cnt_d = img_count;
    wire [3:0] img_idx_d = img_idx_sync + 4'd1;
    wire [3:0] img_idx_tens = (img_idx_d >= 4'd10) ? 4'd1 : 4'd0;
    wire [3:0] img_idx_ones = (img_idx_d >= 4'd10) ? (img_idx_d - 4'd10) : img_idx_d;

    function [7:0] mode_char;
        input [2:0] i;
        begin
            case (i)
                3'd0: mode_char = (work_mode == 2'd0) ? "M" : (work_mode == 2'd1) ? "A" : "P";
                3'd1: mode_char = (work_mode == 2'd0) ? "A" : (work_mode == 2'd1) ? "U" : "A";
                3'd2: mode_char = (work_mode == 2'd0) ? "N" : (work_mode == 2'd1) ? "T" : "R";
                3'd3: mode_char = (work_mode == 2'd0) ? "U" : (work_mode == 2'd1) ? "O" : "A";
                3'd4: mode_char = (work_mode == 2'd0) ? "A" : (work_mode == 2'd1) ? " " : "M";
                default: mode_char = " ";
            endcase
        end
    endfunction

    // ========================================================================
    // 六、顶部 OSD（1 倍字体）
    // ========================================================================
    localparam [10:0] Y_BAR_TOP  = 11'd4;
    localparam [10:0] Y_BAR_BOT  = 11'd34;
    localparam [10:0] Y_LINE_A   = 11'd8;
    localparam [10:0] Y_LINE_B   = 11'd24;
    // 底部字幕条：2 倍字体（16 px 高）＋ 纯深色底，保证在亮画面上也看得清。
    // 原先是 1 倍字体（8 px 高）压在“压暗但仍会动的画面”上，所以糊成一团。
    localparam [10:0] Y_BAR2_TOP = 11'd428;
    localparam [10:0] Y_BAR2_BOT = 11'd455;
    localparam [10:0] BAR2_EDGE  = 11'd428;
    localparam [10:0] Y_LINE_C   = 11'd432;   // 必须是 16 的倍数，行对齐才不会错位

    wire [9:0] xo_a   = x - 10'd8;
    wire [6:0] cidx_a = xo_a[9:3];
    wire [2:0] col_a  = xo_a[2:0];
    wire       in_a   = (yy >= Y_LINE_A) && (yy < (Y_LINE_A + 11'd8))
                        && (x >= 10'd8) && (cidx_a < 7'd34);

    wire [9:0] xo_b   = x - 10'd8;
    wire [6:0] cidx_b = xo_b[9:3];
    wire [2:0] col_b  = xo_b[2:0];
    wire       in_b   = (yy >= Y_LINE_B) && (yy < (Y_LINE_B + 11'd8))
                        && (x >= 10'd8) && (cidx_b < 7'd64);

    wire [10:0] xc_off  = x;
    wire [6:0]  cidx_c0 = xc_off[10:4];      // 16 px 一格
    wire [3:0]  ccol    = xc_off[3:0];
    wire [2:0]  col_c   = ccol[3:1];         // 字形列 0..7（2 倍放大）
    wire [2:0]  row_c   = yy[3:1];           // 字形行 0..7（2 倍放大）
    wire        in_c    = (yy >= Y_LINE_C) && (yy < (Y_LINE_C + 11'd16));

    // 滚动：每 6 帧移动 1 像素（约 10 像素/秒 ≈ 1.2 字符/秒）
    // 滚动：每 6 帧移动 1 像素（约 1.2 字符/秒）
    // 注意 scroll_off 必须按"有效文本长度"回绕，否则滚到 114..127 那段空白时
    // 屏幕会整体空掉 —— 这就是"字幕不全"的原因。
    localparam [7:0] TEXT_LEN = 8'd114;
    reg [7:0] scroll_off;
    reg [2:0] scroll_div;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            scroll_off <= 8'd0;
            scroll_div <= 3'd0;
        end else if (frame_tick) begin
            if (scroll_div == 3'd5) begin
                scroll_div <= 3'd0;
                scroll_off <= (scroll_off == (TEXT_LEN - 8'd1)) ? 8'd0 : (scroll_off + 8'd1);
            end else begin
                scroll_div <= scroll_div + 3'd1;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            host_scroll_px <= 9'd0;
            host_scroll_div <= 2'd0;
        end else if (!host_text_enable_sync[2] || !osd_show) begin
            host_scroll_px <= 9'd0;
            host_scroll_div <= 2'd0;
        end else if (frame_tick) begin
            if (host_scroll_div == 2'd3) begin
                host_scroll_div <= 2'd0;
                host_scroll_px <= (host_scroll_px == 9'd511)
                    ? 9'd0 : (host_scroll_px + 9'd1);
            end else begin
                host_scroll_div <= host_scroll_div + 2'd1;
            end
        end
    end
    wire [7:0] cidx_sum = {1'b0, cidx_c0} + scroll_off;
    wire [7:0] cidx_c   = (cidx_sum >= TEXT_LEN) ? (cidx_sum - TEXT_LEN) : cidx_sum;

    function [7:0] uptime_char;
        input [6:0] i;
        begin
            case (i)
                7'd0:  uptime_char = "U";
                7'd1:  uptime_char = "P";
                7'd2:  uptime_char = " ";
                7'd3:  uptime_char = {4'h3, uptime_hour_bcd[7:4]};
                7'd4:  uptime_char = {4'h3, uptime_hour_bcd[3:0]};
                7'd5:  uptime_char = ":";
                7'd6:  uptime_char = {4'h3, uptime_minute_bcd[7:4]};
                7'd7:  uptime_char = {4'h3, uptime_minute_bcd[3:0]};
                7'd8:  uptime_char = ":";
                7'd9:  uptime_char = {4'h3, uptime_second_bcd[7:4]};
                7'd10: uptime_char = {4'h3, uptime_second_bcd[3:0]};
                default: uptime_char = " ";
            endcase
        end
    endfunction

    function [7:0] text_char_a;
        input [6:0] i;
        begin
            text_char_a = uptime_char(i);
        end
    endfunction

    function [7:0] text_char_b;
        input [5:0] i;
        begin
            if (no_content) begin
                // 状态页第 2 行：VID O/X(视频可用)  GEN O/X(代号/尺寸)
                case (i)
                    6'd0: text_char_b = "V";
                    6'd1: text_char_b = "I";
                    6'd2: text_char_b = "D";
                    6'd3: text_char_b = " ";
                    6'd4: text_char_b = vid_ok ? "O" : "X";
                    6'd5: text_char_b = " ";
                    6'd6: text_char_b = " ";
                    6'd7: text_char_b = "G";
                    6'd8: text_char_b = "E";
                    6'd9: text_char_b = "N";
                    6'd10: text_char_b = " ";
                    6'd11: text_char_b = gen_relaxed ? "X" : "O";
                    default: text_char_b = " ";
                endcase
            end else case (i)
                6'd0:  text_char_b = "M";
                6'd1:  text_char_b = "O";
                6'd2:  text_char_b = "D";
                6'd3:  text_char_b = "E";
                6'd4:  text_char_b = " ";
                6'd5:  text_char_b = mode_char(3'd0);
                6'd6:  text_char_b = mode_char(3'd1);
                6'd7:  text_char_b = mode_char(3'd2);
                6'd8:  text_char_b = mode_char(3'd3);
                6'd9:  text_char_b = mode_char(3'd4);
                6'd10: text_char_b = mode_char(3'd5);
                6'd11: text_char_b = " ";
                6'd12: text_char_b = "B";
                6'd13: text_char_b = "R";
                6'd14: text_char_b = "T";
                6'd15: text_char_b = " ";
                6'd16: text_char_b = {4'h3, brt_h};
                6'd17: text_char_b = {4'h3, brt_t};
                6'd18: text_char_b = {4'h3, brt_o};
                6'd19: text_char_b = " ";
                6'd20: text_char_b = "C";
                6'd21: text_char_b = "O";
                6'd22: text_char_b = "N";
                6'd23: text_char_b = " ";
                6'd24: text_char_b = {4'h3, con_h};
                6'd25: text_char_b = {4'h3, con_t};
                6'd26: text_char_b = {4'h3, con_o};
                6'd27: text_char_b = " ";
                6'd28: text_char_b = "V";
                6'd29: text_char_b = {4'h3, vol_bcd[7:4]};
                6'd30: text_char_b = {4'h3, vol_bcd[3:0]};
                6'd31: text_char_b = " ";
                6'd32: text_char_b = "P";
                6'd33: text_char_b = (per_tens == 4'd0) ? " " : {4'h3, per_tens};
                6'd34: text_char_b = {4'h3, per_ones};
                6'd35: text_char_b = "S";
                6'd36: text_char_b = " ";
                6'd37: text_char_b = {4'h3, img_cnt_d};
                6'd38: text_char_b = " ";
                6'd39: text_char_b = "A";
                6'd40: text_char_b = {4'h3, {3'b000, auto_on}};
                6'd41: text_char_b = " ";
                6'd42: text_char_b = "B";
                6'd43: text_char_b = {4'h3, {3'b000, load_busy}};
                6'd44: text_char_b = " ";
                6'd45: text_char_b = "I";
                6'd46: text_char_b = (img_idx_tens == 4'd0) ? " " : {4'h3, img_idx_tens};
                6'd47: text_char_b = {4'h3, img_idx_ones};
                6'd48: text_char_b = img_scaled ? "S" : "F";   // S=走缩放, F=640x480 直通
                6'd49: text_char_b = " ";
                6'd50: text_char_b = item_kind ? "V" : "P";   // V=视频 P=图片
                6'd51: text_char_b = " ";
                6'd52: text_char_b = scan_busy ? "+" : " ";   // "+" = 后台还在补扫
                default: text_char_b = " ";
            endcase
        end
    endfunction

    // ========================================================================
    // 七、参数菜单（2 倍字体）
    //     面板 13 字符 × 16 px = 208 宽，8 行 × 16 px = 128 高
    //     原点 (416,112)：两个方向都按 16 对齐 → 字形不会跨格错位
    // ========================================================================
    localparam [10:0] M_X0 = 11'd416;
    localparam [10:0] M_Y0 = 11'd112;
    localparam [10:0] M_W  = 11'd208;
    localparam [10:0] M_H  = 11'd128;

    wire in_menu_box = (x >= M_X0) && (x < (M_X0 + M_W))
                       && (yy >= M_Y0) && (yy < (M_Y0 + M_H));
    wire in_menu     = (work_mode == 2'd2) && in_menu_box;

    wire [10:0] mx_off  = xx - M_X0;
    wire [10:0] my_off  = yy - M_Y0;
    wire [3:0]  mcidx   = mx_off[7:4];       // 字符列 0..12
    wire [3:0]  mcol    = mx_off[3:0];       // 格内列 0..15
    wire [2:0]  mline   = my_off[6:4];       // 行 0..7
    wire [3:0]  mrow    = my_off[3:0];       // 格内行 0..15
    wire [2:0]  mfont_r = mrow[3:1];         // 字形行 0..7
    wire [2:0]  mfont_c = mcol[3:1];         // 字形列 0..7

    function [7:0] menu_name;
        input [2:0] li;
        input [1:0] i;
        begin
            case (li)
                3'd1: menu_name = (i == 2'd0) ? "B" : (i == 2'd1) ? "R" : "T";
                3'd2: menu_name = (i == 2'd0) ? "C" : (i == 2'd1) ? "O" : "N";
                3'd3: menu_name = (i == 2'd0) ? "P" : (i == 2'd1) ? "E" : "R";
                3'd4: menu_name = (i == 2'd0) ? "V" : (i == 2'd1) ? "O" : "L";
                3'd5: menu_name = (i == 2'd0) ? "O" : (i == 2'd1) ? "S" : "D";
                default: menu_name = (i == 2'd0) ? "M" : (i == 2'd1) ? "O" : "V";
            endcase
        end
    endfunction

    function [7:0] menu_char;
        input [2:0] li;
        input [3:0] ci;
        reg   [2:0] item;
        begin
            item      = li - 3'd1;
            menu_char = " ";
            case (li)
                3'd0: case (ci)                       // "  MENU"
                        4'd2: menu_char = "M";
                        4'd3: menu_char = "E";
                        4'd4: menu_char = "N";
                        4'd5: menu_char = "U";
                        default: menu_char = " ";
                      endcase
                3'd1, 3'd2, 3'd3, 3'd4, 3'd5, 3'd6: case (ci)
                        4'd0: menu_char = " ";            // 游标单独画（见下）
                        4'd1: menu_char = {4'h3, item + 3'd1};
                        4'd2: menu_char = " ";
                        4'd3: menu_char = menu_name(li, 2'd0);
                        4'd4: menu_char = menu_name(li, 2'd1);
                        4'd5: menu_char = menu_name(li, 2'd2);
                        4'd6: case (li)
                                  3'd3: menu_char = (per_tens == 4'd0) ? " " : {4'h3, per_tens};
                                  default: menu_char = " ";
                                endcase
                        4'd7: case (li)
                                 3'd1: menu_char = {4'h3, brt_h};
                                 3'd2: menu_char = {4'h3, con_h};
                                 3'd3: menu_char = {4'h3, per_ones};
                                 3'd4: menu_char = {4'h3, vol_bcd[7:4]};
                                 3'd5: menu_char = osd_show ? "O" : "O";
                                 default: menu_char = (trans_mode == 2'd0) ? "S" : "O";
                               endcase
                        4'd8: case (li)
                                 3'd1: menu_char = {4'h3, brt_t};
                                 3'd2: menu_char = {4'h3, con_t};
                                 3'd3: menu_char = "S";
                                 3'd4: menu_char = {4'h3, vol_bcd[3:0]};
                                 3'd5: menu_char = osd_show ? "N" : "F";
                                 default: menu_char = (trans_mode == 2'd0) ? "L" : "F";
                               endcase
                        4'd9: case (li)
                                 3'd1: menu_char = {4'h3, brt_o};
                                 3'd2: menu_char = {4'h3, con_o};
                                 3'd5: menu_char = osd_show ? " " : "F";
                                 default: menu_char = (trans_mode == 2'd0) ? "D" : "F";
                               endcase
                        default: menu_char = " ";
                      endcase
                3'd7: case (ci)                       // "K4:+ K3:-"
                        4'd0: menu_char = "K";
                        4'd1: menu_char = "4";
                        4'd2: menu_char = ":";
                        4'd3: menu_char = "+";
                        4'd4: menu_char = " ";
                        4'd5: menu_char = "K";
                        4'd6: menu_char = "3";
                        4'd7: menu_char = ":";
                        4'd8: menu_char = "-";
                        default: menu_char = " ";
                      endcase
                default: menu_char = " ";
            endcase
        end
    endfunction

    wire [7:0] ch_m   = menu_char(mline, mcidx);
    wire [7:0] bits_m;
    wire       mcur   = (mline != 3'd0) && (mline != 3'd7) &&
                        (para_item == (mline - 3'd1));
    // 游标：第 0 列第 1..3 字形列画一个箭头块
    wire       mcursor = mcur && (mcidx == 4'd0) && (mfont_c < 3'd3);
    wire       mpix    = in_menu && (mcidx <= 4'd12) && (mfont_c < 3'd5) &&
                         bits_m[3'd7 - mfont_c];

    // ========================================================================
    // 八、参数调节浮层（2 倍字体，居中偏下，停留约 2 秒）
    //     13 字符 × 16 = 208 宽，2 行 × 16 = 32 高，原点 (208,392) 均 16 对齐
    // ========================================================================
    localparam [10:0] O_X0 = 11'd208;
    localparam [10:0] O_Y0 = 11'd392;
    localparam [10:0] O_W  = 11'd208;
    localparam [10:0] O_H  = 11'd32;

    reg [7:0] ovl_cnt;          // 剩余显示帧数
    always @(posedge clk or posedge rst) begin
        if (rst)
            ovl_cnt <= 8'd0;
        else if (adj_pulse)
            ovl_cnt <= 8'd120;                       // 约 2 秒 @60Hz
        else if (frame_tick && (ovl_cnt != 8'd0))
            ovl_cnt <= ovl_cnt - 8'd1;
    end

    wire in_ovl_box = (ovl_cnt != 8'd0) && (x >= O_X0) && (x < (O_X0 + O_W))
                      && (yy >= O_Y0) && (yy < (O_Y0 + O_H));
    wire [10:0] ox_off = xx - O_X0;
    wire [10:0] oy_off = yy - O_Y0;
    wire [3:0]  ocidx  = ox_off[7:4];
    wire [3:0]  ocol   = ox_off[3:0];
    wire [2:0]  oline  = oy_off[4:1];   // 行仅 0/1
    wire [2:0]  ofont_r= oy_off[3:1];
    wire [2:0]  ofont_c= ocol[3:1];

    function [7:0] ovl_name;
        input [1:0] i;
        begin
            case (para_item)
                3'd0: ovl_name = (i == 2'd0) ? "B" : (i == 2'd1) ? "R" : "T";
                3'd1: ovl_name = (i == 2'd0) ? "C" : (i == 2'd1) ? "O" : "N";
                3'd2: ovl_name = (i == 2'd0) ? "P" : (i == 2'd1) ? "E" : "R";
                3'd3: ovl_name = (i == 2'd0) ? "V" : (i == 2'd1) ? "O" : "L";
                3'd4: ovl_name = (i == 2'd0) ? "O" : (i == 2'd1) ? "S" : "D";
                default: ovl_name = (i == 2'd0) ? "M" : (i == 2'd1) ? "O" : "V";
            endcase
        end
    endfunction

    function [7:0] ovl_val;
        input [2:0] i;
        begin
            ovl_val = " ";
            case (para_item)
                3'd0: case (i)
                        3'd0: ovl_val = {4'h3, brt_h};
                        3'd1: ovl_val = {4'h3, brt_t};
                        3'd2: ovl_val = {4'h3, brt_o};
                        default: ovl_val = " ";
                      endcase
                3'd1: case (i)
                        3'd0: ovl_val = {4'h3, con_h};
                        3'd1: ovl_val = {4'h3, con_t};
                        3'd2: ovl_val = {4'h3, con_o};
                        default: ovl_val = " ";
                      endcase
                3'd2: case (i)
                        3'd0: ovl_val = (per_tens == 4'd0) ? " " : {4'h3, per_tens};
                        3'd1: ovl_val = {4'h3, per_ones};
                        3'd2: ovl_val = "S";
                        default: ovl_val = " ";
                      endcase
                3'd3: case (i)
                        3'd0: ovl_val = {4'h3, vol_bcd[7:4]};
                        3'd1: ovl_val = {4'h3, vol_bcd[3:0]};
                        default: ovl_val = " ";
                      endcase
                3'd4: case (i)
                        3'd0: ovl_val = osd_show ? "O" : "O";
                        3'd1: ovl_val = osd_show ? "N" : "F";
                        3'd2: ovl_val = osd_show ? " " : "F";
                        default: ovl_val = " ";
                      endcase
                default: case (i)
                        3'd0: ovl_val = (trans_mode == 2'd0) ? "S" : "O";
                        3'd1: ovl_val = (trans_mode == 2'd0) ? "L" : "F";
                        3'd2: ovl_val = (trans_mode == 2'd0) ? "D" : "F";
                        default: ovl_val = " ";
                      endcase
            endcase
        end
    endfunction

    function [7:0] ovl_char;
        input [2:0] li;
        input [3:0] ci;
        begin
            ovl_char = " ";
            if (li == 3'd0) begin
                case (ci)
                    4'd1: ovl_char = ovl_name(2'd0);
                    4'd2: ovl_char = ovl_name(2'd1);
                    4'd3: ovl_char = ovl_name(2'd2);
                    default: ovl_char = " ";
                endcase
            end else begin
                ovl_char = ovl_val(ci);
            end
        end
    endfunction

    wire [7:0] ch_o    = ovl_char(oline, ocidx);
    wire [7:0] bits_o;
    wire       opix    = in_ovl_box && (ocidx <= 4'd12) && (ofont_c < 3'd5) &&
                         bits_o[3'd7 - ofont_c];

    // ========================================================================
    // 九、音频频谱（16 段对数频段柱状图，叠加在画面下部）
    //     · 频谱由 audio_spectrum 用真实 PCM 样本做正交下变频 + 二级低通得到
    //     · 16 根柱子，每根 32 px 宽、间隔 8 px，共 16*40 = 640 px
    //     · 柱高 0..63，底线 y=446，最高到 y=383
    // ========================================================================
    localparam [10:0] SP_BASE = 11'd416;
    localparam [10:0] SP_TOP  = 11'd356;

    wire [8*16-1:0] sp_bars;

    audio_spectrum #(
        .NBANDS  (16),
        .SH      (7),
        .BAR_MAX (63)
    ) u_spectrum (
        .clk          (clk),
        .rst          (rst),
        .audio_valid  (audio_valid),
        .audio_sample (audio_sample),
        .frame_tick   (frame_tick),
        .bars         (sp_bars)
    );

    // 柱序号 / 柱内列：随 de 推进，与 x 同步（每 40 像素一根柱）
    reg [3:0] sp_bi;
    reg [5:0] sp_bx;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sp_bi <= 4'd0;
            sp_bx <= 6'd0;
        end else if (de) begin
            if (x == (H_ACTIVE - 1)) begin
                sp_bi <= 4'd0;
                sp_bx <= 6'd0;
            end else if (sp_bx == 6'd39) begin
                sp_bx <= 6'd0;
                sp_bi <= sp_bi + 4'd1;
            end else begin
                sp_bx <= sp_bx + 6'd1;
            end
        end
    end

    function [7:0] bar_sel;
        input [3:0] i;
        begin
            case (i)
                4'd0 : bar_sel = sp_bars[7:0];
                4'd1 : bar_sel = sp_bars[15:8];
                4'd2 : bar_sel = sp_bars[23:16];
                4'd3 : bar_sel = sp_bars[31:24];
                4'd4 : bar_sel = sp_bars[39:32];
                4'd5 : bar_sel = sp_bars[47:40];
                4'd6 : bar_sel = sp_bars[55:48];
                4'd7 : bar_sel = sp_bars[63:56];
                4'd8 : bar_sel = sp_bars[71:64];
                4'd9 : bar_sel = sp_bars[79:72];
                4'd10: bar_sel = sp_bars[87:80];
                4'd11: bar_sel = sp_bars[95:88];
                4'd12: bar_sel = sp_bars[103:96];
                4'd13: bar_sel = sp_bars[111:104];
                4'd14: bar_sel = sp_bars[119:112];
                default: bar_sel = sp_bars[127:120];
            endcase
        end
    endfunction

    wire [7:0]  sp_h     = bar_sel(sp_bi);
    wire [10:0] sp_hx    = {{3{1'b0}}, sp_h};
    wire [10:0] sp_top_y = (SP_BASE >= sp_hx) ? (SP_BASE - sp_hx) : 11'd0;

    wire sp_area  = osd_show && (yy >= SP_TOP) && (yy <= SP_BASE);
    // 只在每根柱子背后压一条半透明暗底（柱子之间保留画面），像真正的频谱槽
    wire sp_panel = sp_area && (sp_bx < 6'd32);
    wire sp_bar   = sp_area && (sp_bx < 6'd32) && (yy > sp_top_y);
    wire sp_base  = sp_area && (yy == SP_BASE);

    // 柱体按高度分三色（低绿 / 中黄 / 高红），看起来更接近常见频谱仪
    wire [23:0] sp_col = (yy > 11'd420) ? 24'h30FF60 :
                         (yy > 11'd400) ? 24'hFFE040 : 24'hFF5040;

    // ========================================================================
    // 十、最终合成
    // ========================================================================
    wire in_bar1 = (yy >= Y_BAR_TOP)  && (yy <= Y_BAR_BOT);
    wire in_bar2 = (yy >= Y_BAR2_TOP) && (yy <= Y_BAR2_BOT);

    wire [23:0] bar_rgb = {2'b00, mix_r[7:2], 2'b00, mix_g[7:2], 2'b00, mix_b[7:2]};

    wire [7:0] ch_a = text_char_a(cidx_a);
    wire [7:0] ch_b = text_char_b(cidx_b[5:0]);
    wire [7:0] ch_c;
    wire [7:0] bits_a, bits_b, bits_c;

    osd_text_rom u_slogan_rom (
        .idx (cidx_c[6:0]),
        .ch  (ch_c)
    );

    osd_font_rom u_font_a (.ch(ch_a), .row(y[2:0]), .bits(bits_a));
    osd_font_rom u_font_b (.ch(ch_b), .row(y[2:0]), .bits(bits_b));
    osd_font_rom u_font_c (.ch(ch_c), .row(row_c), .bits(bits_c));
    osd_font_rom u_font_m (.ch(ch_m), .row(mfont_r), .bits(bits_m));
    osd_font_rom u_font_o (.ch(ch_o), .row(ofont_r), .bits(bits_o));

    wire pix_a = in_a & (col_a < 3'd5) & bits_a[3'd7 - col_a];
    wire pix_b = in_b & (col_b < 3'd5) & bits_b[3'd7 - col_b];
    wire pix_c = in_c & (col_c < 3'd5) & bits_c[3'd7 - col_c];

    reg [23:0] stage1;

    // ---- 扫描状态页：无内容时把"扫到了什么"显示在顶部两条文字行上 ----
    // 复用已有的顶部 OSD 文本通道（text_char_a / text_char_b + u_font_a/u_font_b），
    // 不再单独实例化 font ROM —— 那样会额外吃 LUT，导致布线拥塞。
    wire osd_eff    = osd_show || no_content;      // 状态页强制显示顶部文字


    always @* begin
        stage1 = {mix_r, mix_g, mix_b};

        if (no_content) begin
            stage1 = 24'h0A1428;                 // 深海军蓝底，不是彩条
        end

        // 顶部信息条 + 1 倍字体
        if (osd_eff) begin
            if (in_bar1)
                stage1 = bar_rgb;
            if (pix_a || pix_b)
                stage1 = 24'hFFFF40;
        end

        // 音频频谱（暗底 + 柱体 + 基线）
        if (sp_panel) begin
            stage1 = {1'b0, mix_r[7:1], 1'b0, mix_g[7:1], 1'b0, mix_b[7:1]};
            if (sp_base)
                stage1 = 24'h205040;
            if (sp_bar)
                stage1 = sp_col;
        end

        // 参数菜单（2 倍字体，深底亮字）
        if (in_menu_box && (work_mode == 2'd2)) begin
            stage1 = 24'h0C1428;
            if (mpix)
                stage1 = 24'hC0E8FF;
            if (mcursor)
                stage1 = 24'hFFC020;
        end

        // 参数调节浮层（2 倍字体）
        if (in_ovl_box) begin
            stage1 = 24'h0C1428;
            if (opix)
                stage1 = 24'hFFE060;
        end

        // 底部字幕条放在**最后**：它必须盖住频谱柱/菜单，否则文字被柱体切断就看不清。
        // 底：纯黑，字：纯黄，对比度拉满。
        if (osd_show && !no_content && in_bar2) begin
            stage1 = 24'h000000;
            if (yy == BAR2_EDGE)
                stage1 = 24'h3060A0;          // 顶部一条细亮边，边界更清楚
            if (pix_c)
                stage1 = 24'hFFFF00;
        end

        if (logo_area && (logo_color != 2'b00)) begin
            case (logo_color)
                2'b01: stage1 = 24'hE82030;
                2'b10: stage1 = 24'h18345C;
                default: stage1 = 24'hE82030;
            endcase
        end
    end

    reg [23:0] stage1_d;
    reg host_pixel_en_d;
    reg [2:0] host_pixel_bit_d;
    reg [23:0] rgb_r;
    wire host_text_pixel = host_pixel_en_d &&
                           host_bitmap_byte[3'd7 - host_pixel_bit_d];
    wire [23:0] stage2 = host_text_pixel ? 24'hFFE040 : stage1_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            stage1_d <= 24'd0;
            host_text_enable_sync <= 3'b000;
            host_pixel_en_d <= 1'b0;
            host_pixel_bit_d <= 3'd0;
            rgb_r <= 24'd0;
        end else begin
            stage1_d <= stage1;
            host_text_enable_sync <= {host_text_enable_sync[1:0], text_enable};
            host_pixel_en_d <= host_text_enable_sync[2] && osd_show &&
                               !no_content && host_text_area && host_source_valid;
            host_pixel_bit_d <= host_source_x[2:0];
            rgb_r <= stage2;
        end
    end

    // Logo 与 OSD 共用开关；同步文字位图使整条像素通路延迟到两拍。
    assign rgb_out = rgb_r;
    assign vs_out  = vs_p2;
    assign de_out  = de_p2;

endmodule
