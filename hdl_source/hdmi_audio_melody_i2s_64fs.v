// ============================================================================
// 模块名 : hdmi_audio_melody_i2s_64fs.v
// 功能   : 音视频联动音频源（扩展要求 2）
//          · 每张图片对应一段独立旋律，调性/速度/音区都不同
//          · 切图时旋律立即从第一个音重新开始，并插入一声 1kHz 提示音
//          · 音量 0~15 可调（幅度 = BASE_AMP * vol / 8）
//          · 输出与官方 hdmi_audio_tone_i2s_64fs 完全相同的 I2S 64fs 时序，
//            下游 I2S_receiver / audio_arc_calculate / HDMI 发射核无需改动
//
// 设计说明：
//   · I_mclk = 12.288 MHz，内部按 48 kHz 帧率产生样本
//   · 图片编号/音量来自 sd_card_clk 域（慢变）：2 级同步 + commit 翻转握手
//   · 方波音源（24-bit 有符号），音序器每帧推进，休止符只把样本置 0
// ============================================================================
module hdmi_audio_melody_i2s_64fs #(
    parameter signed [23:0] BASE_AMP    = 24'sd2000000,
    parameter integer       BEEP_FRAMES = 5760        // 提示音 120ms @48kHz
)(
    input  wire        I_mclk,       // 12.288MHz
    input  wire        I_rst,
    input  wire [1:0]  I_img_idx,    // 当前图片编号（慢变）
    input  wire        I_img_tgl,    // 图片提交事件（每次提交翻转一次）
    input  wire [3:0]  I_volume,     // 音量 0~15（8 = 原始幅度）
    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT
);

    // ------------------------------------------------------------------
    // 音符频率表：0~14 = C4 ~ C6 半音，15 = 休止
    // 相位增量 = f / 48000 * 2^32
    // ------------------------------------------------------------------
    function [31:0] note_inc_lut;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:  note_inc_lut = 32'd23410256; // C4  261.63Hz
                4'd1:  note_inc_lut = 32'd26276252; // D4  293.66Hz
                4'd2:  note_inc_lut = 32'd29494793; // E4  329.63Hz
                4'd3:  note_inc_lut = 32'd31248571; // F4  349.23Hz
                4'd4:  note_inc_lut = 32'd35075566; // G4  392.00Hz
                4'd5:  note_inc_lut = 32'd39370534; // A4  440.00Hz
                4'd6:  note_inc_lut = 32'd44191634; // B4  493.88Hz
                4'd7:  note_inc_lut = 32'd46819617; // C5  523.25Hz
                4'd8:  note_inc_lut = 32'd52553399; // D5  587.33Hz
                4'd9:  note_inc_lut = 32'd58988691; // E5  659.25Hz
                4'd10: note_inc_lut = 32'd62497143; // F5  698.46Hz
                4'd11: note_inc_lut = 32'd70150238; // G5  783.99Hz
                4'd12: note_inc_lut = 32'd78741067; // A5  880.00Hz
                4'd13: note_inc_lut = 32'd88384163; // B5  987.77Hz
                4'd14: note_inc_lut = 32'd93639235; // C6 1046.50Hz
                default: note_inc_lut = 32'd0;      // 休止
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // 每张图片对应的旋律（8 个音循环）
    // ------------------------------------------------------------------
    function [3:0] melody_note;
        input [1:0] img;
        input [2:0] step;
        begin
            case (img)
                // 图1：C 大调，明亮上行琶音（高音区）
                2'd0: case (step)
                        3'd0: melody_note = 4'd7;    // C5
                        3'd1: melody_note = 4'd9;    // E5
                        3'd2: melody_note = 4'd11;   // G5
                        3'd3: melody_note = 4'd14;   // C6
                        3'd4: melody_note = 4'd11;   // G5
                        3'd5: melody_note = 4'd9;    // E5
                        3'd6: melody_note = 4'd7;    // C5
                        default: melody_note = 4'd4; // G4
                      endcase
                // 图2：a 小调，缓慢下行（低音区）
                2'd1: case (step)
                        3'd0: melody_note = 4'd12;   // A5
                        3'd1: melody_note = 4'd9;    // E5
                        3'd2: melody_note = 4'd5;    // A4
                        3'd3: melody_note = 4'd3;    // F4
                        3'd4: melody_note = 4'd1;    // D4
                        3'd5: melody_note = 4'd0;    // C4
                        3'd6: melody_note = 4'd15;   // 休止（换气）
                        default: melody_note = 4'd15;
                      endcase
                // 图3：G 大调快速琶音跳进（音域最宽）
                2'd2: case (step)
                        3'd0: melody_note = 4'd4;    // G4
                        3'd1: melody_note = 4'd8;    // D5
                        3'd2: melody_note = 4'd11;   // G5
                        3'd3: melody_note = 4'd13;   // B5
                        3'd4: melody_note = 4'd11;   // G5
                        3'd5: melody_note = 4'd8;    // D5
                        3'd6: melody_note = 4'd6;    // B4
                        default: melody_note = 4'd4; // G4
                      endcase
                // 图4：C 大调五声音阶，低音区沉稳
                default: case (step)
                        3'd0: melody_note = 4'd0;    // C4
                        3'd1: melody_note = 4'd2;    // E4
                        3'd2: melody_note = 4'd4;    // G4
                        3'd3: melody_note = 4'd5;    // A4
                        3'd4: melody_note = 4'd4;    // G4
                        3'd5: melody_note = 4'd2;    // E4
                        3'd6: melody_note = 4'd4;    // G4
                        default: melody_note = 4'd0; // C4
                      endcase
            endcase
        end
    endfunction

    // 每张图的速度：每个音持续的 48kHz 帧数
    function [15:0] note_len_lut;
        input [1:0] img;
        begin
            case (img)
                2'd0: note_len_lut = 16'd12000;   // 0.25 s 快板
                2'd1: note_len_lut = 16'd26400;   // 0.55 s 慢板
                2'd2: note_len_lut = 16'd8640;    // 0.18 s 急板
                default: note_len_lut = 16'd28800; // 0.60 s 广板
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // 跨时钟域同步
    // ------------------------------------------------------------------
    reg [1:0] img_s0, img_s1;
    reg       tgl_s0, tgl_s1, tgl_s2;
    reg [3:0] vol_s0, vol_s1;

    wire img_changed = tgl_s1 ^ tgl_s2;

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            img_s0 <= 2'd0; img_s1 <= 2'd0;
            tgl_s0 <= 1'b0; tgl_s1 <= 1'b0; tgl_s2 <= 1'b0;
            vol_s0 <= 4'd8; vol_s1 <= 4'd8;
        end else begin
            img_s0 <= I_img_idx;  img_s1 <= img_s0;
            tgl_s0 <= I_img_tgl;  tgl_s1 <= tgl_s0;  tgl_s2 <= tgl_s1;
            vol_s0 <= I_volume;   vol_s1 <= vol_s0;
        end
    end

    reg [1:0] cur_img;
    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst)            cur_img <= 2'd0;
        else if (img_changed) cur_img <= img_s1;
    end

    // ------------------------------------------------------------------
    // 幅度
    // ------------------------------------------------------------------
    wire signed [27:0] amp_scaled = BASE_AMP * $signed({1'b0, vol_s1});
    wire signed [23:0] amp_cur    = amp_scaled >>> 3;

    // ------------------------------------------------------------------
    // 音序器
    // ------------------------------------------------------------------
    reg [2:0]  note_step;
    reg [15:0] note_cnt;
    reg [15:0] beep_cnt;
    reg        beeping;

    wire [3:0]  note_code    = melody_note(cur_img, note_step);
    wire [31:0] note_inc     = note_inc_lut(note_code);
    wire [15:0] note_len_cur = note_len_lut(cur_img);
    wire [31:0] play_inc     = beeping ? 32'd89478485 : note_inc;   // 提示音 1 kHz

    // I2S 发送
    reg [5:0]  S_bit_cnt;
    reg [63:0] S_shift_reg;
    reg [31:0] S_phase_acc;
    reg signed [23:0] S_sample_word;
    reg signed [23:0] S_sample_next;

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            O_i2s_BCLK    <= 1'b0;
            O_i2s_LRCK    <= 1'b0;
            O_i2s_DOUT    <= 1'b0;
            S_bit_cnt     <= 6'd0;
            S_shift_reg   <= 64'd0;
            S_phase_acc   <= 32'd0;
            S_sample_word <= 24'sd0;
            S_sample_next <= 24'sd0;
            note_step     <= 3'd0;
            note_cnt      <= 16'd0;
            beep_cnt      <= 16'd0;
            beeping       <= 1'b0;
        end else begin
            O_i2s_BCLK <= ~O_i2s_BCLK;

            // 切图：提示音 + 旋律从头开始
            if (img_changed) begin
                beeping   <= 1'b1;
                beep_cnt  <= BEEP_FRAMES[15:0] - 16'd1;
                note_step <= 3'd0;
                note_cnt  <= 16'd0;
            end

            if (O_i2s_BCLK == 1'b1) begin
                O_i2s_DOUT  <= S_shift_reg[63];
                S_shift_reg <= {S_shift_reg[62:0], 1'b0};

                if (S_bit_cnt == 6'd63) begin
                    S_bit_cnt <= 6'd0;

                    if (O_i2s_LRCK == 1'b0) begin
                        // 左声道结束，接着发右声道（同一份样本）
                        O_i2s_LRCK  <= 1'b1;
                        S_shift_reg <= {S_sample_word[23:0], 40'd0};
                    end else begin
                        // 右声道结束 → 计算下一个样本（一帧一个样本）
                        O_i2s_LRCK <= 1'b0;

                        // 提示音按帧计时
                        if (beeping) begin
                            if (beep_cnt == 16'd0) beeping <= 1'b0;
                            else                   beep_cnt <= beep_cnt - 16'd1;
                        end

                        S_phase_acc <= S_phase_acc + play_inc;
                        if (!beeping && (note_inc == 32'd0))
                            S_sample_next <= 24'sd0;          // 休止符
                        else if (S_phase_acc[31])
                            S_sample_next <= amp_cur;
                        else
                            S_sample_next <= -amp_cur;

                        S_sample_word <= S_sample_next;
                        S_shift_reg   <= {S_sample_next[23:0], 40'd0};

                        // 音序推进：每帧无条件推进（休止符也要计时，否则会卡死）
                        if (!beeping) begin
                            if (note_cnt >= (note_len_cur - 16'd1)) begin
                                note_cnt  <= 16'd0;
                                note_step <= (note_step == 3'd7) ? 3'd0
                                                                : (note_step + 3'd1);
                            end else begin
                                note_cnt <= note_cnt + 16'd1;
                            end
                        end
                    end
                end else begin
                    S_bit_cnt <= S_bit_cnt + 6'd1;
                end
            end
        end
    end

endmodule
