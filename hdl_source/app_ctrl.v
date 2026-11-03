// ============================================================================
// 模块名 : app_ctrl.v
// 功能   : 自有应用控制层 —— 顶层状态机（播放 / 切图 / 参数命令分发）
//          这是“自主编程实现”的核心证据：官方例程把控制逻辑藏在
//          sd_card_bmp 内部，本工程在顶层新增一套独立控制状态机，
//          统一管理播放模式、手动/自动切图、亮度/对比度参数调节，
//          并通过单拍同步脉冲驱动底层 sd_card_bmp。
//
// 交互逻辑（板载丝印按键 → 功能）：
//   KEY2 (FPGA 引脚 B2) —— 手动切下一张
//   KEY4 (FPGA 引脚 C1) —— 自动轮播 开/关
//   KEY1 (FPGA 引脚 A2) —— 硬件复位（沿用官方 rst_n，不复用为功能键）
//
//   [普通播放模式]
//     KEY2 短按 → 手动切到下一节目，并退出自动轮播
//     SW2=1 → 显示 PC 字幕；SW2=0 → 隐藏 PC 字幕
//     KEY4 短按  → 自动轮播 开/关 切换
//     KEY2 长按800ms → 进入参数调节模式（SW2 状态不影响）
//   [参数调节模式]
//     KEY2 长按1s→ 退出参数调节，回到普通模式
//     KEY2 短按  → 切换调节项（亮度 → 对比度 → 轮播周期 → OSD → 转场 → 亮度…）
//     KEY4 短按  → 当前项 +8 / 周期加一档 / 开关翻转
//     KEY4 长按1s→ 当前项 -8 / 周期减一档 / 开关翻转
//
// 输出给上层的意义：
//   - brightness/contrast : 供 video_compose 对像素做亮度/对比度运算
//   - auto_period          : 供 sd_card_bmp_v2 配置 5/10/15/20 秒轮播间隔
//   - work_mode/para_item : 供 OSD 显示当前模式与调节项
//
// 工程   : HX4S20C 选题一 自有工程（topic1_self_engineered）V2.0
// 日期   : 2026-10-16
// ============================================================================
module app_ctrl #(
    parameter integer CLK_FREQ_HZ  = 100_000_000, // 与 sd_card_bmp 同域时钟
    parameter integer LONG_PRESS_MS = 1200,       // 长按阈值（毫秒）
    parameter [7:0]  PARA_STEP     = 8'd8         // 参数每次调节步进
)(
    input  wire        clk,           // 时钟（sd_card_clk / 100MHz）
    input  wire        rst_n,         // 低有效复位（接顶层 rst_all 取反后的复位）
    input  wire        key1_n,        // KEY2 (B2)：切图 / 菜单换项 / 长按进出菜单
    input  wire        key2_n,        // KEY4 (C1)：轮播开关 / 菜单项 +
    input  wire        key3_n,        // KEY3 (B1)：OSD 开关 / 菜单项 −
    input  wire        auto_state,    // 播放器内部自动轮播的**实际**状态（唯一真相）
    input  wire        interaction_switch, // SW2=1 时显示 PC 字幕
    output reg         key_next_n,    // 单拍低有效：切下一张（供 sd_card_bmp）
    output reg         key_auto_n,    // 单拍低有效：自动播放开关（供 sd_card_bmp）
    output reg  [7:0]  brightness,    // 亮度偏置参数（0~255，默认128=无偏置）
    output reg  [7:0]  contrast,      // 对比度增益参数（0~255，默认128=1.0倍）
    output reg  [11:0] brt_bcd,       // 亮度十进制三位 {百,十,个}
    output reg  [11:0] con_bcd,       // 对比度十进制三位
    output reg  [7:0]  vol_bcd,       // 音量十进制两位 {十,个}（0~15）
    output wire [1:0]  work_mode,     // 播放模式：0=手动 1=自动 2=参数调节
    output wire [2:0]  para_item,     // 当前调节项：0=亮度 1=对比度 2=周期 3=音量 4=OSD 5=转场
    output reg  [1:0]  auto_period,   // 轮播间隔档位：0=5s 1=10s 2=15s 3=20s
    output reg  [3:0]  volume,        // 音量 0~15（8 = 原始幅度）
    output reg         osd_en,        // OSD 图层开关
    output wire        host_text_show,// PC 发送文字的可见开关
    output reg  [1:0]  trans_mode,    // 转场：0=滑动 1=关闭
    output reg         adj_pulse,     // 参数被调节（供画面弹出数值提示）

    // ---- UART 控制接口（扩展要求 3）：均已在本时钟域做过同步/去抖，为单拍脉冲 ----
    input  wire        uart_next,     // 下一张
    input  wire        uart_auto,     // 自动轮播开关
    input  wire        uart_item,     // 切换调节项（并进入参数模式）
    input  wire        uart_inc,      // 当前项 +
    input  wire        uart_dec,      // 当前项 -
    input  wire        uart_osd      // OSD 开关
);

    // ================= 内部状态定义 =================
    localparam [1:0] S_NORMAL = 2'd0;  // 普通播放模式
    localparam [1:0] S_PARAM  = 2'd1;  // 参数调节模式
    reg [1:0] state;
    reg [31:0] idle_cnt;        // 菜单内无操作计时（防呆自动退出）

    // 注意：不再自维护"自动播放镜像"。原来本地翻转 + 向底层发脉冲的做法，
    // 一旦任何一次脉冲被底层漏掉，两份状态就永久错位 —— 表现为
    // "OSD 显示 AUTO 但不轮播" 或 "OSD 显示 MANUAL 却在轮播"。
    // 现在直接用底层返回的 auto_state 作为唯一真相。

    // 按键脉冲
    wire k1_press, k1_short, k1_long, k1_rep;
    wire k2_press, k2_short, k2_long, k2_rep;
    wire k3_press, k3_short, k3_long, k3_rep;

    // 模式输出（组合）
    reg [2:0] para_item_reg;

    assign work_mode = (state == S_PARAM) ? 2'd2 :
                       (auto_state == 1'b1) ? 2'd1 : 2'd0;
    assign para_item = para_item_reg;
    assign host_text_show = interaction_switch;

    // 参数单步（亮度/对比度用 4 更细腻，配合连发足够快）
    localparam [7:0] STEP8  = 8'd4;

    // ================= 按键实例化 =================
    key_proc #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .DEBOUNCE_MS     (15),
        .LONG_PRESS_MS   (LONG_PRESS_MS),
        .REPEAT_DELAY_MS (60000),       // KEY2 不连发：按住只是为了进/出菜单
        .REPEAT_MS       (90)
    ) u_key1 (
        .clk         (clk),
        .rst_n       (rst_n),
        .button_in   (key1_n),
        .press_pulse (k1_press),
        .short_pulse (k1_short),
        .long_pulse  (k1_long),
        .rep_pulse   (k1_rep)
    );

    key_proc #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .DEBOUNCE_MS     (15),          // KEY4 在去抖确认按下时响应，长按不会重复触发
        .LONG_PRESS_MS   (LONG_PRESS_MS),
        .REPEAT_DELAY_MS (450),
        .REPEAT_MS       (90)
    ) u_key2 (
        .clk         (clk),
        .rst_n       (rst_n),
        .button_in   (key2_n),
        .press_pulse (k2_press),
        .short_pulse (k2_short),
        .long_pulse  (k2_long),
        .rep_pulse   (k2_rep)
    );

    key_proc #(
        .CLK_FREQ_HZ     (CLK_FREQ_HZ),
        .DEBOUNCE_MS     (15),
        .LONG_PRESS_MS   (LONG_PRESS_MS),
        .REPEAT_DELAY_MS (450),
        .REPEAT_MS       (90)
    ) u_key3 (
        .clk         (clk),
        .rst_n       (rst_n),
        .button_in   (key3_n),
        .press_pulse (k3_press),
        .short_pulse (k3_short),
        .long_pulse  (k3_long),
        .rep_pulse   (k3_rep)
    );

    // ================= 参数单步动作（按键/连发/UART 共用） =================
    task param_step;
        input        up;          // 1=加 0=减
        begin
            case (para_item_reg)
                3'd0: brightness  <= up ? sat_add(brightness, STEP8) : sat_sub(brightness, STEP8);
                3'd1: contrast    <= up ? sat_add(contrast,   STEP8) : sat_sub(contrast,   STEP8);
                3'd2: auto_period <= up ? ((auto_period == 2'd3) ? 2'd0 : (auto_period + 2'd1))
                                        : ((auto_period == 2'd0) ? 2'd3 : (auto_period - 2'd1));
                3'd3: volume      <= up ? ((volume == 4'd15) ? 4'd15 : (volume + 4'd1))
                                        : ((volume == 4'd0)  ? 4'd0  : (volume - 4'd1));
                3'd4: osd_en      <= ~osd_en;
                default: trans_mode <= (trans_mode == 2'd0) ? 2'd1 : 2'd0;
            endcase
        end
    endtask

    // ================= 状态机主逻辑 =================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_NORMAL;
            idle_cnt     <= 32'd0;
            brightness   <= 8'd128;
            contrast     <= 8'd128;
            para_item_reg<= 3'd0;
            auto_period  <= 2'd3;    // 默认 20s：让每页的音频能完整播完
            volume       <= 4'd8;
            osd_en       <= 1'b1;
            trans_mode   <= 2'd0;   // 默认滑动
            adj_pulse    <= 1'b0;
            key_next_n  <= 1'b1;
            key_auto_n  <= 1'b1;
        end else begin
            // 默认空闲为高电平；按键事件只拉低一个时钟周期。
            key_next_n   <= 1'b1;
            key_auto_n   <= 1'b1;
            adj_pulse     <= 1'b0;

            case (state)
                // ---------------- 普通播放模式 ----------------
                // 菜单 = **长按 KEY2**（用户习惯的入口）。
                // 之前"长按 KEY2 进菜单后 KEY2/KEY4 像坏了一样"的真正原因不是这个入口，
                // 而是菜单里 KEY2/KEY4 被改成了"换参数项/调数值"且没有明显的退出方式。
                // 现在：菜单内 KEY2 长按退出 + 15 秒无操作自动退出 + 屏幕上有明确菜单框，
                // 所以长按 KEY2 作为菜单入口是安全的。
                S_NORMAL: begin
                    if (k1_long) begin
                        state      <= S_PARAM;     // 长按 KEY2 进入参数菜单
                        idle_cnt   <= 32'd0;
                        adj_pulse  <= 1'b1;
                    end else if (k1_short) begin
                        key_next_n <= 1'b0;
                    end else if (k2_press) begin
                        key_auto_n <= 1'b0;
                    end else if (k3_short) begin
                        osd_en <= ~osd_en;
                        adj_pulse <= 1'b1;
                    end

                end

                // ---------------- 参数菜单模式 ----------------
                // 长按 KEY2 退出；KEY2 短按换参数项；KEY4 = +；KEY3 = −；长按 KEY4 恢复默认
                S_PARAM: begin
                    if (k1_long) begin
                        state     <= S_NORMAL;     // 长按 KEY2 退出菜单
                        adj_pulse <= 1'b1;
                    end else if (k1_short) begin
                        para_item_reg <= (para_item_reg == 3'd5) ? 3'd0
                                                                 : (para_item_reg + 3'd1);
                        idle_cnt  <= 32'd0;
                        adj_pulse <= 1'b1;
                    end else if (k2_short || k2_rep) begin
                        param_step(1'b1);          // KEY4：+
                        idle_cnt  <= 32'd0;
                        adj_pulse <= 1'b1;
                    end else if (k3_short || k3_rep) begin
                        param_step(1'b0);          // KEY3：−
                        idle_cnt  <= 32'd0;
                        adj_pulse <= 1'b1;
                    end else if (k2_long) begin
                        // 长按 KEY4：当前项恢复默认值
                        case (para_item_reg)
                            3'd0: brightness  <= 8'd128;
                            3'd1: contrast    <= 8'd128;
                            3'd2: auto_period <= 2'd3;   // 恢复 20s（本版默认值）
                            3'd3: volume      <= 4'd8;
                            3'd4: osd_en      <= 1'b1;
                            default: trans_mode <= 2'd0;
                        endcase
                        idle_cnt  <= 32'd0;
                        adj_pulse <= 1'b1;
                    end else begin
                        // 15s 无操作自动退出，避免"停在一个没注意到的菜单里"
                        if (idle_cnt != 32'd1_500_000_000)
                            idle_cnt <= idle_cnt + 32'd1;
                        if (idle_cnt >= 32'd1_500_000_000)
                            state <= S_NORMAL;
                    end
                end

                default: state <= S_NORMAL;
            endcase

            // ---------------- UART 命令（任何模式都生效） ----------------
            if (uart_item) begin
                state         <= S_PARAM;
                para_item_reg <= (para_item_reg == 3'd5) ? 3'd0
                                                          : (para_item_reg + 3'd1);
                adj_pulse     <= 1'b1;
            end
            if (uart_next) begin
                key_next_n <= 1'b0;
            end
            if (uart_auto) begin
                key_auto_n <= 1'b0;
            end
            if (uart_osd) begin
                osd_en    <= ~osd_en;
                adj_pulse <= 1'b1;
            end
            if (uart_inc || uart_dec) begin
                if (state != S_PARAM)
                    volume <= uart_inc ? ((volume == 4'd15) ? 4'd15 : (volume + 4'd1))
                                       : ((volume == 4'd0)  ? 4'd0  : (volume - 4'd1));
                else
                    param_step(uart_inc);
                adj_pulse <= 1'b1;
            end
        end
    end

    // ================= 十进制三位转换（0~255 → {百,十,个}） =================
    // 用常数乘法近似除 100 / 除 10，避免循环带来的长组合链：
    //   v/100 = (v*41)>>12      v/10 = (r*205)>>11   （0..255 / 0..99 内精确）
    // 分两级寄存器实现：第一级算百位与余数，第二级算十位与个位，
    // 保证 100MHz 控制域不会出现超长组合路径（OSD 显示对 2 拍延迟不敏感）
    reg [3:0] brt_h_r, con_h_r;
    reg [7:0] brt_rem, con_rem;

    function [3:0] div100;
        input [7:0] v;
        reg [15:0] m;
        begin
            m      = {8'd0, v} * 16'd41;
            div100 = m[15:12];
        end
    endfunction

    function [3:0] div10;
        input [7:0] r;
        reg [15:0] m;
        begin
            m     = {8'd0, r} * 16'd205;
            div10 = m[15:11];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brt_h_r <= 4'd1;
            con_h_r <= 4'd1;
            brt_rem <= 8'd28;
            con_rem <= 8'd28;
        end else begin
            brt_h_r <= div100(brightness);
            con_h_r <= div100(contrast);
            brt_rem <= brightness - ({4'd0, div100(brightness)} * 8'd100);
            con_rem <= contrast   - ({4'd0, div100(contrast)}   * 8'd100);
        end
    end

    wire [3:0] brt_u_w = brt_rem - ({4'd0, div10(brt_rem)} * 8'd10);
    wire [3:0] con_u_w = con_rem - ({4'd0, div10(con_rem)} * 8'd10);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brt_bcd <= 12'h128;
            con_bcd <= 12'h128;
        end else begin
            // 注意：三段都要是 4bit，否则拼接宽度变成 16bit 会被截断
            brt_bcd <= {brt_h_r, div10(brt_rem), brt_u_w};
            con_bcd <= {con_h_r, div10(con_rem), con_u_w};
        end
    end

    // ================= 音量两位十进制（0~15） =================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            vol_bcd <= 8'h08;
        else
            vol_bcd <= (volume >= 4'd10) ? {4'd1, (volume - 4'd10)}
                                         : {4'd0, volume};
    end

    // ================= 饱和加减函数 =================
    // 返回 value ± step，结果限制在 [0,255]
    function [7:0] sat_add;
        input [7:0] value;
        input [7:0] step;
        begin
            sat_add = (value > (8'd255 - step)) ? 8'd255 : (value + step);
        end
    endfunction

    function [7:0] sat_sub;
        input [7:0] value;
        input [7:0] step;
        begin
            sat_sub = (value < step) ? 8'd0 : (value - step);
        end
    endfunction

endmodule
