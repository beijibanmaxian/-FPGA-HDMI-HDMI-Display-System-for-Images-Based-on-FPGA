// ============================================================================
// 模块名 : uart_rx.v
// 功能   : 极简 UART 接收器（115200 8N1），用于扩展要求(3)的串口参数配置
//          · 16 倍过采样 + 中点采样，抗干扰
//          · 输出 rx_valid + rx_data（1 拍脉冲）
//          · 端口名与官方 uart 例程一致，引脚见 uart_test.adc（RX=F12）
// ============================================================================
module uart_rx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD        = 115200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,          // 外部串口输入（板载 USB-UART）
    output reg        rx_valid,    // 收到一个字节（1 拍）
    output reg  [7:0] rx_data
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD;   // 868 @100MHz
    localparam integer HALF_BIT     = CLKS_PER_BIT / 2;

    reg [1:0]  rx_sync;
    reg [15:0] cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  shift;

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0] state;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_sync  <= 2'b11;
            state    <= S_IDLE;
            cnt      <= 16'd0;
            bit_idx  <= 4'd0;
            shift    <= 8'd0;
            rx_valid <= 1'b0;
            rx_data  <= 8'd0;
        end else begin
            rx_sync  <= {rx_sync[0], rx};
            rx_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (!rx_sync[1]) begin      // 检测到起始位（低）
                        state <= S_START;
                        cnt   <= HALF_BIT[15:0];
                    end
                end
                S_START: begin
                    if (cnt != 16'd0) begin
                        cnt <= cnt - 16'd1;
                    end else if (!rx_sync[1]) begin
                        // 中点仍为低 → 确认起始位
                        state   <= S_DATA;
                        cnt     <= CLKS_PER_BIT[15:0] - 16'd1;
                        bit_idx <= 4'd0;
                    end else begin
                        state <= S_IDLE;        // 毛刺，丢弃
                    end
                end
                S_DATA: begin
                    if (cnt != 16'd0) begin
                        cnt <= cnt - 16'd1;
                    end else begin
                        shift   <= {rx_sync[1], shift[7:1]};   // LSB 先到
                        bit_idx <= bit_idx + 4'd1;
                        cnt     <= CLKS_PER_BIT[15:0] - 16'd1;
                        if (bit_idx == 4'd7)
                            state <= S_STOP;
                    end
                end
                S_STOP: begin
                    if (cnt != 16'd0) begin
                        cnt <= cnt - 16'd1;
                    end else begin
                        if (rx_sync[1]) begin      // 停止位为高 → 数据有效
                            rx_valid <= 1'b1;
                            rx_data  <= shift;
                        end
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
