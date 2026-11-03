// ============================================================================
// 模块名 : uart_cmd_decode.v
// 功能   : 把串口收到的 ASCII 命令翻译成单拍控制脉冲（扩展要求 3）
//
// 命令（串口终端 115200 8N1，回车即可）：
//   n  或  N : 下一张图片
//   a  或  A : 自动轮播 开/关
//   m  或  M : 切换调节项（进入参数菜单）
//   +  或  = : 当前项 +
//   -  或  _ : 当前项 -
//   o  或  O : OSD 图层 开/关
// 其它字符忽略。所有输出均为单拍脉冲，可直接接 app_ctrl。
// ============================================================================
module uart_cmd_decode (
    input  wire        clk,
    input  wire        rst,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output reg         cmd_next,
    output reg         cmd_auto,
    output reg         cmd_item,
    output reg         cmd_inc,
    output reg         cmd_dec,
    output reg         cmd_osd
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cmd_next <= 1'b0;
            cmd_auto <= 1'b0;
            cmd_item <= 1'b0;
            cmd_inc  <= 1'b0;
            cmd_dec  <= 1'b0;
            cmd_osd  <= 1'b0;
        end else begin
            cmd_next <= 1'b0;
            cmd_auto <= 1'b0;
            cmd_item <= 1'b0;
            cmd_inc  <= 1'b0;
            cmd_dec  <= 1'b0;
            cmd_osd  <= 1'b0;

            if (rx_valid) begin
                case (rx_data)
                    8'h6E, 8'h4E: cmd_next <= 1'b1;   // 'n' 'N'
                    8'h61, 8'h41: cmd_auto <= 1'b1;   // 'a' 'A'
                    8'h6D, 8'h4D: cmd_item <= 1'b1;   // 'm' 'M'
                    8'h2B, 8'h3D: cmd_inc  <= 1'b1;   // '+' '='
                    8'h2D, 8'h5F: cmd_dec  <= 1'b1;   // '-' '_'
                    8'h6F, 8'h4F: cmd_osd  <= 1'b1;   // 'o' 'O'
                    default: ;
                endcase
            end
        end
    end

endmodule
