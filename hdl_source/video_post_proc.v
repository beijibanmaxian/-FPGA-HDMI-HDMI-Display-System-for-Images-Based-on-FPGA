// ============================================================================
// 模块名 : video_post_proc.v
// 功能   : 视频后处理 —— 实时亮度 / 对比度调节（纯组合逻辑，不引入时序延迟）
//          - 对比度公式 : y = (c - 128) * (contrast - 128) / 128 + 128
//          - 亮度公式   : y = y + (brightness - 128)
//          - 结果饱和到 [0, 255]
//          - osd_en=1 时直接输出 osd_rgb（为扩展要求(1)OSD叠加预留接口，
//            后续 B 队员的 OSD 模块只需把字符像素接到 osd_en/osd_rgb 即可）
//
// 时序说明 : 本模块输出与输入同一像素节拍（组合逻辑直通），
//            不破坏 video_delay → video_rgb_to_axis 的 vs/de 同步关系。
//
// 工程   : HX4S20C 选题一 自有工程（topic1_self_engineered）V2.0
// 日期   : 2026-10-16
// ============================================================================
module video_post_proc #(
    parameter integer PW = 8          // 每通道位宽（RGB 各 8bit）
)(
    input  wire        video_clk,     // 像素时钟（本模块仅用于寄存器输出，当前为组合）
    input  wire        rst_n,         // 低有效复位（预留，未使用寄存器）
    input  wire [23:0] rgb_in,        // 原始 RGB 像素（来自 video_delay.vout_data_raw）
    input  wire [7:0]  brightness,    // 亮度参数（0~255，128=无偏置）
    input  wire [7:0]  contrast,      // 对比度参数（0~255，128=1.0倍增益）
    input  wire        osd_en,        // OSD 使能（1=用 OSD 像素覆盖）
    input  wire [23:0] osd_rgb,       // OSD 像素 RGB（由 OSD 模块产生）
    output wire [23:0] rgb_out        // 处理后的 RGB 像素
);

    // ================= 单通道处理函数 =================
    // 输入 c : 通道原始值（0~255）
    // 输出   : 对比度+亮度处理并饱和后的值（0~255）
    function [7:0] post_chan;
        input [7:0]  c;
        input [7:0]  brt;
        input [7:0]  cst;
        reg signed [8:0]  c_off;   // c - 128，范围 -128..127
        reg signed [8:0]  g_off;   // contrast - 128，范围 -128..127
        reg signed [17:0] prod;    // 有符号乘积
        reg signed [17:0] y_tmp;   // 处理中间值
        reg [7:0]   y;             // 饱和后输出
        begin
            // --- 对比度：以 128 为中心缩放 ---
            c_off = $signed({1'b0, c})   - 9'sd128;
            g_off = $signed({1'b0, cst}) - 9'sd128;
            prod  = c_off * g_off;                       // 18bit 有符号
            y_tmp = (prod >>> 7) + 9'sd128;              // 除以128(算术右移7位)+回中心

            // --- 亮度：整体加偏置 ---
            y_tmp = y_tmp + ($signed({1'b0, brt}) - 9'sd128);

            // --- 饱和到 0~255 ---
            if (y_tmp > 18'sd255)
                y = 8'd255;
            else if (y_tmp < 18'sd0)
                y = 8'd0;
            else
                y = y_tmp[7:0];

            post_chan = y;
        end
    endfunction

    // ================= 三通道并行处理 =================
    wire [7:0] r_adj = post_chan(rgb_in[23:16], brightness, contrast);
    wire [7:0] g_adj = post_chan(rgb_in[15:8],  brightness, contrast);
    wire [7:0] b_adj = post_chan(rgb_in[7:0],   brightness, contrast);

    wire [23:0] adj_rgb = {r_adj, g_adj, b_adj};

    // ================= OSD 叠加：使能时用 OSD 像素覆盖 =================
    // 说明：这里先做“覆盖式”叠加（OSD 像素直接替换背景），
    //       B 队员做扩展(1)时，可在此升级为“透明度混合”（alpha 混帧）。
    assign rgb_out = osd_en ? osd_rgb : adj_rgb;

endmodule
