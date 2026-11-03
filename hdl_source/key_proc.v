// ============================================================================
// 模块名 : key_proc.v
// 功能   : 按键去抖 + 短按 / 长按 / 按住自动连发
//          - press_pulse : 去抖确认"按下"的那一拍（低有效输入，稳定变 0 时）
//                          ★ 立即响应：不用等松手，按下去就有动作
//          - short_pulse : 按下后短于 LONG_PRESS_MS 就松开，产生一拍
//          - long_pulse  : 按住达到 LONG_PRESS_MS 时产生一拍
//          - rep_pulse   : 按住超过 REPEAT_DELAY_MS 后，每 REPEAT_MS 产生一拍
//                          （用于参数连续加减，避免"调高了调不回去"）
// 按键极性 : 低有效（按下=0，松开=1）
// ============================================================================
module key_proc #(
    parameter integer CLK_FREQ_HZ     = 100_000_000,
    parameter integer DEBOUNCE_MS     = 6,
    parameter integer LONG_PRESS_MS   = 1000,
    parameter integer REPEAT_DELAY_MS = 600,
    parameter integer REPEAT_MS       = 120
)(
    input  wire clk,
    input  wire rst_n,
    input  wire button_in,     // 低有效
    output reg  press_pulse,
    output reg  short_pulse,
    output reg  long_pulse,
    output reg  rep_pulse
);

    localparam integer CYCLES_PER_MS   = (CLK_FREQ_HZ >= 1000) ? (CLK_FREQ_HZ / 1000) : 1;
    localparam integer MS_DIV_WIDTH    = (CYCLES_PER_MS > 1) ? $clog2(CYCLES_PER_MS) : 1;
    localparam integer DB_CNT_WIDTH    = (DEBOUNCE_MS > 1) ? $clog2(DEBOUNCE_MS) : 1;
    localparam integer HOLD_LIMIT_MS   = (LONG_PRESS_MS > REPEAT_DELAY_MS)
                                       ? LONG_PRESS_MS : REPEAT_DELAY_MS;
    localparam integer HELD_CNT_WIDTH  = (HOLD_LIMIT_MS > 1) ? $clog2(HOLD_LIMIT_MS + 1) : 1;
    localparam integer REP_CNT_WIDTH   = (REPEAT_MS > 1) ? $clog2(REPEAT_MS) : 1;

    reg sync0, sync1, stable;
    reg [MS_DIV_WIDTH-1:0] ms_div_cnt;
    reg                    ms_tick;
    reg [DB_CNT_WIDTH-1:0] db_cnt;

    reg [HELD_CNT_WIDTH-1:0] held_cnt;
    reg        long_done;
    reg [REP_CNT_WIDTH-1:0] rep_cnt;
    reg        stable_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_div_cnt <= {MS_DIV_WIDTH{1'b0}};
            ms_tick    <= 1'b0;
        end else if (ms_div_cnt == CYCLES_PER_MS - 1) begin
            ms_div_cnt <= {MS_DIV_WIDTH{1'b0}};
            ms_tick    <= 1'b1;
        end else begin
            ms_div_cnt <= ms_div_cnt + 1'b1;
            ms_tick    <= 1'b0;
        end
    end

    // ---------------- 去抖 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync0  <= 1'b1;
            sync1  <= 1'b1;
            stable <= 1'b1;
            db_cnt <= {DB_CNT_WIDTH{1'b0}};
        end else begin
            sync0 <= button_in;
            sync1 <= sync0;
            if (sync1 == stable) begin
                db_cnt <= {DB_CNT_WIDTH{1'b0}};
            end else if (ms_tick) begin
                if (db_cnt >= DEBOUNCE_MS - 1) begin
                    stable <= sync1;
                    db_cnt <= {DB_CNT_WIDTH{1'b0}};
                end else begin
                    db_cnt <= db_cnt + 1'b1;
                end
            end
        end
    end

    // 去抖后"稳定按下"的那一拍（1 -> 0）：按键立即生效，不等松手
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stable_d    <= 1'b1;
            press_pulse <= 1'b0;
        end else begin
            stable_d    <= stable;
            press_pulse <= stable_d & ~stable;
        end
    end

    // ---------------- 短按 / 长按 / 连发 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            held_cnt    <= {HELD_CNT_WIDTH{1'b0}};
            long_done   <= 1'b0;
            rep_cnt     <= {REP_CNT_WIDTH{1'b0}};
            short_pulse <= 1'b0;
            long_pulse  <= 1'b0;
            rep_pulse   <= 1'b0;
        end else begin
            short_pulse <= 1'b0;
            long_pulse  <= 1'b0;
            rep_pulse   <= 1'b0;

            if (stable == 1'b0) begin
                if (ms_tick) begin
                    if (held_cnt < HOLD_LIMIT_MS)
                        held_cnt <= held_cnt + 1'b1;

                    if (!long_done && (held_cnt >= LONG_PRESS_MS - 1)) begin
                        long_pulse <= 1'b1;
                        long_done  <= 1'b1;
                    end

                    // 连发和长按共享毫秒计数，计数到所需阈值后饱和。
                    if (held_cnt >= REPEAT_DELAY_MS) begin
                        if (rep_cnt >= REPEAT_MS - 1) begin
                            rep_cnt   <= {REP_CNT_WIDTH{1'b0}};
                            rep_pulse <= 1'b1;
                        end else begin
                            rep_cnt <= rep_cnt + 1'b1;
                        end
                    end else begin
                        rep_cnt <= {REP_CNT_WIDTH{1'b0}};
                    end
                end
            end else begin
                // 松开
                if ((held_cnt != {HELD_CNT_WIDTH{1'b0}}) && !long_done)
                    short_pulse <= 1'b1;
                held_cnt  <= {HELD_CNT_WIDTH{1'b0}};
                long_done <= 1'b0;
                rep_cnt   <= {REP_CNT_WIDTH{1'b0}};
            end
        end
    end

endmodule
