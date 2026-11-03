// ============================================================================
// 模块名 : audio_spectrum.v
// 功能   : 音频频谱分析（赛题扩展要求「音频可视化」——频谱叠加于视频画面）
//
// 原理   : 16 个频段做“正交下变频 + 二级一阶低通”，即每个频点的能量包络：
//            I[k] += ( x*sin(2*pi*f_k*n/fs) - I[k] ) / 2^SH
//            Q[k] += ( x*cos(2*pi*f_k*n/fs) - Q[k] ) / 2^SH
//            （I/Q 各串联两级，等效 12 dB/oct，频段之间不互相糊）
//          幅度 mag[k] = |I2[k]| + |Q2[k]|，就是该频段的幅度谱。
//          16 个频点按对数等分覆盖 150 Hz ~ 6 kHz，正好覆盖 C4~C6 及前几次谐波。
//
// 时序   : 48 kHz 样本，每样本 32 拍（16 段 x 2 相）串行处理；
//          视频时钟 25 MHz 下每样本有 520 拍可用，余量充足；
//          只用 2 个乘法器（I/Q 各一路），不抢 DSP 资源。
//
// 显示   : bars 为 16 x 8bit 柱高。上升立即跟随，下降每视频帧回落 1 像素。
// ============================================================================
module audio_spectrum #(
    parameter integer NBANDS  = 16,          // 频段数（显示成 16 根柱子）
    parameter integer SH      = 7,           // 一阶低通系数 1/2^SH
    parameter integer BAR_MAX = 63           // 柱高上限（像素）
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        audio_valid,          // 单拍脉冲 @48 kHz
    input  wire [23:0] audio_sample,         // 有符号 PCM
    input  wire        frame_tick,           // 每视频帧一个脉冲（柱高回落用）
    output reg  [8*NBANDS-1:0] bars          // 每 8 bit 一根柱的高度 0..BAR_MAX
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_MUL  = 2'd1;
    localparam [1:0] S_MAG  = 2'd2;

    reg [1:0]  st;
    reg [3:0]  b;
    reg signed [15:0] xs;             // 当前样本（24 -> 16 bit）
    reg        ft_flag;               // 本批处理内是否需要回落

    reg [31:0] pha   [0:NBANDS-1];    // NCO 相位
    reg signed [23:0] ai1 [0:NBANDS-1];
    reg signed [23:0] ai2 [0:NBANDS-1];
    reg signed [23:0] aq1 [0:NBANDS-1];
    reg signed [23:0] aq2 [0:NBANDS-1];

    // ----------------------------------------------------------------
    // 各频段 NCO 频率字：inc = f * 2^32 / 48000
    // ----------------------------------------------------------------
    function [31:0] inc_lut;
        input [3:0] i;
        begin
            case (i)
                4'd0 : inc_lut = 32'd15061709  ;   //   168.3 Hz
                4'd1 : inc_lut = 32'd18967191  ;   //   212.0 Hz
                4'd2 : inc_lut = 32'd23885358  ;   //   266.9 Hz
                4'd3 : inc_lut = 32'd30078800  ;   //   336.2 Hz
                4'd4 : inc_lut = 32'd37878193  ;   //   423.3 Hz
                4'd5 : inc_lut = 32'd47699959  ;   //   533.1 Hz
                4'd6 : inc_lut = 32'd60068495  ;   //   671.3 Hz
                4'd7 : inc_lut = 32'd75644177  ;   //   845.4 Hz
                4'd8 : inc_lut = 32'd95258613  ;   //  1064.6 Hz
                4'd9 : inc_lut = 32'd119959046 ;   //  1340.6 Hz
                4'd10: inc_lut = 32'd151064269 ;   //  1688.3 Hz
                4'd11: inc_lut = 32'd190235035 ;   //  2126.0 Hz
                4'd12: inc_lut = 32'd239562729 ;   //  2677.3 Hz
                4'd13: inc_lut = 32'd301681029 ;   //  3371.5 Hz
                4'd14: inc_lut = 32'd379906523 ;   //  4245.8 Hz
                4'd15: inc_lut = 32'd478415784 ;   //  5346.7 Hz
                default: inc_lut = 32'd0;
            endcase
        end
    endfunction

    // ----------------------------------------------------------------
    // 1/4 周期正弦表：tab[t] = round(127*sin(2*pi*t/256))，t = 0..63
    // ----------------------------------------------------------------
    function signed [7:0] sin_tab;
        input [5:0] t;
        begin
            case (t)
                6'd0 : sin_tab = 8'sd0  ;  6'd1 : sin_tab = 8'sd3  ;  6'd2 : sin_tab = 8'sd6  ;  6'd3 : sin_tab = 8'sd9  ;
                6'd4 : sin_tab = 8'sd12 ;  6'd5 : sin_tab = 8'sd16 ;  6'd6 : sin_tab = 8'sd19 ;  6'd7 : sin_tab = 8'sd22 ;
                6'd8 : sin_tab = 8'sd25 ;  6'd9 : sin_tab = 8'sd28 ;  6'd10: sin_tab = 8'sd31 ;  6'd11: sin_tab = 8'sd34 ;
                6'd12: sin_tab = 8'sd37 ;  6'd13: sin_tab = 8'sd40 ;  6'd14: sin_tab = 8'sd43 ;  6'd15: sin_tab = 8'sd46 ;
                6'd16: sin_tab = 8'sd49 ;  6'd17: sin_tab = 8'sd51 ;  6'd18: sin_tab = 8'sd54 ;  6'd19: sin_tab = 8'sd57 ;
                6'd20: sin_tab = 8'sd60 ;  6'd21: sin_tab = 8'sd63 ;  6'd22: sin_tab = 8'sd65 ;  6'd23: sin_tab = 8'sd68 ;
                6'd24: sin_tab = 8'sd71 ;  6'd25: sin_tab = 8'sd73 ;  6'd26: sin_tab = 8'sd76 ;  6'd27: sin_tab = 8'sd78 ;
                6'd28: sin_tab = 8'sd81 ;  6'd29: sin_tab = 8'sd83 ;  6'd30: sin_tab = 8'sd85 ;  6'd31: sin_tab = 8'sd88 ;
                6'd32: sin_tab = 8'sd90 ;  6'd33: sin_tab = 8'sd92 ;  6'd34: sin_tab = 8'sd94 ;  6'd35: sin_tab = 8'sd96 ;
                6'd36: sin_tab = 8'sd98 ;  6'd37: sin_tab = 8'sd100;  6'd38: sin_tab = 8'sd102;  6'd39: sin_tab = 8'sd104;
                6'd40: sin_tab = 8'sd106;  6'd41: sin_tab = 8'sd107;  6'd42: sin_tab = 8'sd109;  6'd43: sin_tab = 8'sd111;
                6'd44: sin_tab = 8'sd112;  6'd45: sin_tab = 8'sd113;  6'd46: sin_tab = 8'sd115;  6'd47: sin_tab = 8'sd116;
                6'd48: sin_tab = 8'sd117;  6'd49: sin_tab = 8'sd118;  6'd50: sin_tab = 8'sd120;  6'd51: sin_tab = 8'sd121;
                6'd52: sin_tab = 8'sd122;  6'd53: sin_tab = 8'sd122;  6'd54: sin_tab = 8'sd123;  6'd55: sin_tab = 8'sd124;
                6'd56: sin_tab = 8'sd125;  6'd57: sin_tab = 8'sd125;  6'd58: sin_tab = 8'sd126;  6'd59: sin_tab = 8'sd126;
                6'd60: sin_tab = 8'sd126;  6'd61: sin_tab = 8'sd127;  6'd62: sin_tab = 8'sd127;  6'd63: sin_tab = 8'sd127;
                default: sin_tab = 8'sd0;
            endcase
        end
    endfunction

    // 8 位相位 -> 有符号正弦（象限折叠）
    function signed [7:0] sin_lut;
        input [7:0] a;
        reg   [5:0] t;
        reg         q1;
        reg  signed [7:0] v;
        begin
            q1 = a[6];
            t  = a[5:0];
            v  = q1 ? sin_tab(6'd63 - t) : sin_tab(t);
            sin_lut = a[7] ? -v : v;
        end
    endfunction

    wire [7:0]  ph8   = pha[b][31:24];
    wire signed [7:0] s_sin = sin_lut(ph8);
    wire signed [7:0] s_cos = sin_lut(ph8 + 8'd64);

    // 注意：乘积不再右移 8 位，而是把它当作 Q8 定点（低 8 位是小数）。
    // 这样一阶低通的步进精度是 1/256 个 LSB，负残差也能收敛回 0；
    // 若直接右移 8 位，残差落在 -128..-1 时增量被截断为 0，会永久留下直流偏置。
    wire signed [23:0] prod_i = (xs * s_sin);
    wire signed [23:0] prod_q = (xs * s_cos);

    wire [23:0] mag_i = ai2[b][23] ? (~ai2[b] + 24'd1) : ai2[b];
    wire [23:0] mag_q = aq2[b][23] ? (~aq2[b] + 24'd1) : aq2[b];
    wire [24:0] mag   = {1'b0, mag_i} + {1'b0, mag_q};
    wire [22:0] hm    = mag >> 14;         // Q8 定点 -> 柱高（满幅基波接近顶部）
    wire [7:0]  hnew  = (hm > BAR_MAX) ? BAR_MAX[7:0] : hm[7:0];

    reg  [7:0] cur_bar;
    always @* begin
        case (b)
            4'd0 : cur_bar = bars[7:0];
            4'd1 : cur_bar = bars[15:8];
            4'd2 : cur_bar = bars[23:16];
            4'd3 : cur_bar = bars[31:24];
            4'd4 : cur_bar = bars[39:32];
            4'd5 : cur_bar = bars[47:40];
            4'd6 : cur_bar = bars[55:48];
            4'd7 : cur_bar = bars[63:56];
            4'd8 : cur_bar = bars[71:64];
            4'd9 : cur_bar = bars[79:72];
            4'd10: cur_bar = bars[87:80];
            4'd11: cur_bar = bars[95:88];
            4'd12: cur_bar = bars[103:96];
            4'd13: cur_bar = bars[111:104];
            4'd14: cur_bar = bars[119:112];
            4'd15: cur_bar = bars[127:120];
            default: cur_bar = 8'd0;
        endcase
    end

    task set_bar;
        input [3:0] idx;
        input [7:0] val;
        begin
            case (idx)
                4'd0 : bars[7:0] <= val;
                4'd1 : bars[15:8] <= val;
                4'd2 : bars[23:16] <= val;
                4'd3 : bars[31:24] <= val;
                4'd4 : bars[39:32] <= val;
                4'd5 : bars[47:40] <= val;
                4'd6 : bars[55:48] <= val;
                4'd7 : bars[63:56] <= val;
                4'd8 : bars[71:64] <= val;
                4'd9 : bars[79:72] <= val;
                4'd10: bars[87:80] <= val;
                4'd11: bars[95:88] <= val;
                4'd12: bars[103:96] <= val;
                4'd13: bars[111:104] <= val;
                4'd14: bars[119:112] <= val;
                4'd15: bars[127:120] <= val;
                default: ;
            endcase
        end
    endtask

    integer k;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st      <= S_IDLE;
            b       <= 4'd0;
            xs      <= 16'sd0;
            ft_flag <= 1'b0;
            bars    <= {8*NBANDS{1'b0}};
            for (k = 0; k < NBANDS; k = k + 1) begin
                pha[k] <= 32'd0;
                ai1[k] <= 24'sd0;  ai2[k] <= 24'sd0;
                aq1[k] <= 24'sd0;  aq2[k] <= 24'sd0;
            end
        end else begin
            if (frame_tick)
                ft_flag <= 1'b1;

            case (st)
                S_IDLE: begin
                    if (audio_valid) begin
                        xs <= audio_sample[23:8];
                        b  <= 4'd0;
                        st <= S_MUL;
                    end
                end
                // 正交下变频 + 二级一阶低通（每拍一个频段）
                S_MUL: begin
                    pha[b] <= pha[b] + inc_lut(b);
                    ai1[b] <= ai1[b] + ((prod_i - ai1[b]) >>> SH);
                    aq1[b] <= aq1[b] + ((prod_q - aq1[b]) >>> SH);
                    ai2[b] <= ai2[b] + ((ai1[b] - ai2[b]) >>> SH);
                    aq2[b] <= aq2[b] + ((aq1[b] - aq2[b]) >>> SH);
                    if (b == NBANDS-1) begin
                        b  <= 4'd0;
                        st <= S_MAG;
                    end else begin
                        b <= b + 4'd1;
                    end
                end
                // 求幅度并更新柱高（上升立即跟随，下降每帧回落 1 像素）
                S_MAG: begin
                    if (hnew >= cur_bar)
                        set_bar(b, hnew);
                    else if (ft_flag && (cur_bar != 8'd0))
                        set_bar(b, cur_bar - 8'd1);
                    if (b == NBANDS-1) begin
                        b       <= 4'd0;
                        st      <= S_IDLE;
                        ft_flag <= 1'b0;
                    end else begin
                        b <= b + 4'd1;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
