// ============================================================================
// 模块名 : video_stream_reader.v
// 功能   : 从 TF 卡流式读取 RGB565 原始帧，缩放成 OUT_W x OUT_H 的 RGB888 像素流
//
// 素材格式（与 tools/make_ad_assets.py 一致）：
//   文件头 54 字节（伪 BMP 头，用于让扫描器识别），
//   之后是 src_w*src_h 个 16bit 小端 RGB565 像素，按行**自下而上**排列（BMP 习惯）。
//
// 全屏显示：
//   DST_Y0=0 且 DST_H=OUT_H 时，源画面通过纵横 DDA 拉伸到整个输出区域，
//   不补黑边，也不裁切素材内容。非 4:3 素材会发生比例拉伸。
//
// 每帧像素数严格等于 OUT_W*OUT_H：
//   帧缓存写侧的写指针只有整帧收满才会回到帧首，一旦某一帧少发/多发像素，
//   后面所有帧都会整体错位（屏幕表现：画面上下翻滚、"来回晃动"）。
//   所以这里用 pix_cnt 计数，任何异常中断都走 S_FLUSH 补黑，保证**帧对齐**。
//
// 结构：
//   · 扇区缓冲 sec_buf[512]：把"SD 卡读时序"与"像素处理"解耦。
//   · 行缓存 linebuf：一行的 RGB888 像素（LB_DEPTH = 2^LB_AW）。
//   · 纵向 DDA：内容第 oy 行输出取源图第 sy 行，sy = floor(oy*src_h/DST_H)。
//   · 横向 DDA：ox -> sx = floor(ox*src_w/OUT_W)。
// ============================================================================
module video_stream_reader #(
    parameter integer OUT_W  = 640,         // 输出宽（= 帧缓存宽）
    parameter integer OUT_H  = 480,         // 输出高（= 帧缓存高）
    parameter integer DST_Y0 = 0,           // 画面顶部空白行
    parameter integer DST_H  = 480,         // 画面占用的输出行数
    parameter integer TOP_EDGE_CROP_ROWS = 0, // 丢弃 BMP 顶边的解码器绿色扫描行
    parameter integer LB_AW  = 10           // 行缓存深度 = 2^LB_AW 像素
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [31:0] base_sector,
    input  wire [12:0] byte_off,            // 有效载荷在首扇区内的字节偏移
    input  wire [12:0] src_w,
    input  wire [12:0] src_h,
    input  wire        vflip,               // 1 = 素材自上而下（本工程一律 0）

    output reg         sd_req,
    output reg  [31:0] sd_addr,
    input  wire        sd_gnt,
    input  wire [7:0]  rd_data,
    input  wire        rd_valid,
    input  wire        rd_end,

    input  wire        pix_wr_ready,
    output reg         pix_wr_en,
    output reg  [23:0] pix_data,
    output reg         busy,
    output reg         done
);

    localparam integer LB_DEPTH = (1 << LB_AW);

    localparam [3:0] S_IDLE  = 4'd0;
    localparam [3:0] S_REQ   = 4'd1;
    localparam [3:0] S_GNT   = 4'd2;
    localparam [3:0] S_FILL  = 4'd3;
    localparam [3:0] S_CONS  = 4'd4;      // 发读地址（读扇区缓冲）
    localparam [3:0] S_CON2  = 4'd5;      // 用上一拍读出的字节
    localparam [3:0] S_EMIT  = 4'd6;      // 输出画面本体
    localparam [3:0] S_BAND  = 4'd7;      // 输出黑边（前段/后段共用）
    localparam [3:0] S_FLUSH = 4'd8;      // 收尾：补黑到整帧
    localparam [3:0] S_FIN   = 4'd9;      // 判定本帧是否已经发满

    reg [3:0]  st;

    localparam [15:0]  OUT_W16      = OUT_W;
    localparam [15:0]  OUT_H16      = OUT_H;
    localparam [15:0]  DST_H16      = DST_H;
    localparam [15:0]  BAND_FIRST16 = (OUT_H - DST_Y0 - DST_H);
    localparam [15:0]  BAND_LAST16  = DST_Y0;
    localparam [18:0]  OUT_PIXELS_C = OUT_W * OUT_H;
    localparam [12:0]  LB_DEPTH13   = LB_DEPTH;

    // ---------------- 扇区缓冲 ----------------
    reg [7:0]  sec_buf [0:511];
    reg [9:0]  fill_ptr;
    reg [9:0]  cons_ptr;
    reg [7:0]  cons_byte;               // sec_buf 的寄存输出

    // ---------------- 流位置 ----------------
    reg [31:0] sec;
    reg [12:0] skip;                    // 首扇区要丢掉的头部字节
    reg [12:0] w_limit;                 // 实际写入行缓存的宽度

    // ---------------- 行缓存 ----------------
    reg [23:0] linebuf [0:LB_DEPTH-1];
    reg [LB_AW-1:0]  waddr;
    reg [LB_AW-1:0]  raddr;
    reg [23:0]       rdata;

    // ---------------- 像素拼装 / 计数 ----------------
    reg        half;
    reg [18:0] pix_cnt;                 // 本帧已发出的像素数（帧对齐的命根子）

    // ---------------- 黑边输出 ----------------
    reg [15:0] band_rows;               // 本次要发多少行黑边
    reg        band_lead;               // 1 = 前段（发完去读卡）0 = 后段（发完收尾）
    reg [15:0] bl_row;
    reg [15:0] bl_col;
    reg        bl_ph;                   // 2 拍发 1 像素（与 S_EMIT 同速率）

    // 超时保护：等 SD 授权/数据太久就放弃本帧的**读取部分**
    // 这个超时必须远大于"被音频挤开"的时间。仲裁器把音频连发限到 2 个扇区后，
    // 视频最长等待约 0.4ms；这里给 10ms，只用来兜住"卡真的没响应"。
    reg [16:0] wd;
    localparam [16:0] WD_RELOAD = 17'd120_000;    // 约 1.2ms 无进展才判定卡死
    wire wd_expire = ~|wd;

    reg [7:0]  lob;
    reg [12:0] x;                       // 当前行已收像素数
    reg [12:0] row;                     // 当前源行号

    // ---------------- 纵向 DDA ----------------
    reg [15:0] oy;
    reg [15:0] sy;
    reg [15:0] vacc;

    // ---------------- 横向 DDA ----------------
    reg [15:0] e_ox;
    reg [15:0] e_sx;
    reg [15:0] e_acc;
    reg        e_ph;

    wire [12:0] src_w_c  = (src_w > LB_DEPTH13) ? LB_DEPTH13 : src_w;
    wire        row_last = ({1'b0, x} + 14'd1 >= {1'b0, src_w});
    wire        src_last = ({1'b0, row} + 14'd1 >= {1'b0, src_h});

    wire [16:0] v_next  = {1'b0, vacc} + {1'b0, src_h};
    wire        v_carry = (v_next >= {1'b0, DST_H16});

    // 下一个输出像素要取的源列（水平 DDA 的"下一拍值"，供行缓存地址提前一拍给）
    wire [15:0] e_sx_next = (({1'b0, e_acc} + {1'b0, src_w_c}) >= {1'b0, OUT_W16})
                            ? (e_sx + 16'd1) : e_sx;
    wire source_top_edge_row = vflip
        ? (row < TOP_EDGE_CROP_ROWS)
        : ({1'b0, row} + TOP_EDGE_CROP_ROWS >= {1'b0, src_h});

    // 行缓存：写口在 S_CON2，读口在 S_EMIT（简单双口 RAM）
    always @(posedge clk) begin
        if (st == S_CON2 && half && (x < w_limit))
            linebuf[waddr] <= rgb565_to_888({cons_byte, lob});
        rdata <= linebuf[raddr];
    end

    // 扇区缓冲：写口来自 SD，读口在 S_CONS（简单双口 RAM，可推断成 BRAM）
    always @(posedge clk) begin
        if (rd_valid && (fill_ptr < 10'd512))
            sec_buf[fill_ptr[8:0]] <= rd_data;
        if (st == S_CONS && (cons_ptr < 10'd512))
            cons_byte <= sec_buf[cons_ptr[8:0]];
    end

    // 已发像素计数：pix_wr_en 是寄存器输出，这里数的是**上一拍**发出的那个像素，
    // 因此 pix_cnt 报满时正好还有 1 拍余量，S_FIN/S_FLUSH 不会多补。
    // ★ 每帧开始必须清零：不清零的话第二帧一进来就"已满"，
    //   于是只发一半像素就收工 -> 帧缓存写指针错位 -> 屏上画面上下翻滚。
    always @(posedge clk or posedge rst) begin
        if (rst)
            pix_cnt <= 19'd0;
        else if (st == S_IDLE)
            pix_cnt <= 19'd0;
        else if (pix_wr_en)
            pix_cnt <= pix_cnt + 19'd1;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st         <= S_IDLE;
            sd_req     <= 1'b0;
            sd_addr    <= 32'd0;
            fill_ptr   <= 10'd0;
            cons_ptr   <= 10'd0;
            sec        <= 32'd0;
            skip       <= 13'd0;
            w_limit    <= 13'd0;
            waddr      <= {LB_AW{1'b0}};
            raddr      <= {LB_AW{1'b0}};
            half       <= 1'b0;
            lob        <= 8'd0;
            x          <= 13'd0;
            row        <= 13'd0;
            oy         <= 16'd0;
            sy         <= 16'd0;
            vacc       <= 16'd0;
            e_ox       <= 16'd0;
            e_sx       <= 16'd0;
            e_acc      <= 16'd0;
            e_ph       <= 1'b0;
            band_rows  <= 16'd0;
            band_lead  <= 1'b1;
            bl_row     <= 16'd0;
            bl_col     <= 16'd0;
            bl_ph      <= 1'b0;
            pix_wr_en  <= 1'b0;
            pix_data   <= 24'd0;
            wd         <= WD_RELOAD;
            busy       <= 1'b0;
            done       <= 1'b0;
        end else begin
            pix_wr_en <= 1'b0;
            done      <= 1'b0;

            if (((st == S_GNT) || (st == S_FILL)) && !rd_valid) begin
                if (wd != 17'd0) wd <= wd - 17'd1;
            end else begin
                wd <= WD_RELOAD;
            end

            // 扇区缓冲写指针（只在本块里驱动，避免多驱动）
            if (rd_valid && (fill_ptr < 10'd512))
                fill_ptr <= fill_ptr + 10'd1;

            // 看门狗：只在等授权 / 等数据时有效；长时间无进展就放弃读取。
            // 但**不能**就地结束 —— 那样帧缓存写指针会停在半帧位置，
            // 之后每一帧都会整体错位（屏上"画面来回晃"就是这么来的）。
            // 统一转到 S_FLUSH 把剩下的像素补黑，保证整帧对齐。
            if (wd_expire && ((st == S_GNT) || (st == S_FILL))) begin
                sd_req <= 1'b0;
                st     <= S_FLUSH;
            end else case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start && (src_w != 13'd0) && (src_h != 13'd0)) begin
                        busy      <= 1'b1;
                        sec       <= base_sector;
                        skip      <= byte_off;
                        w_limit   <= src_w_c;
                        cons_ptr  <= 10'd0;
                        half      <= 1'b0;
                        x         <= 13'd0;
                        row       <= 13'd0;
                        oy        <= 16'd0;
                        sy        <= vflip ? ({3'd0, src_h} - 16'd1) : 16'd0;
                        vacc      <= 16'd0;
                        e_ox      <= 16'd0;
                        e_sx      <= 16'd0;
                        e_acc     <= 16'd0;
                        e_ph      <= 1'b0;
                        bl_row    <= 16'd0;
                        bl_col    <= 16'd0;
                        bl_ph     <= 1'b0;
                        waddr     <= {LB_AW{1'b0}};
                        // 先发"流前段"黑边，再读画面
                        band_rows <= BAND_FIRST16;
                        band_lead <= 1'b1;
                        st        <= (BAND_FIRST16 != 16'd0) ? S_BAND : S_REQ;
                    end
                end

                // 黑边：整行整行地发黑像素
                S_BAND: begin
                    if (!bl_ph) begin
                        bl_ph <= 1'b1;
                    end else if (pix_wr_ready) begin
                        bl_ph     <= 1'b0;
                        pix_wr_en <= 1'b1;
                        pix_data  <= 24'd0;
                        if (bl_col + 16'd1 >= OUT_W16) begin
                            bl_col <= 16'd0;
                            if (bl_row + 16'd1 >= band_rows) begin
                                bl_row <= 16'd0;
                                st     <= band_lead ? S_REQ : S_FIN;
                            end else begin
                                bl_row <= bl_row + 16'd1;
                            end
                        end else begin
                            bl_col <= bl_col + 16'd1;
                        end
                    end
                end

                S_REQ: begin
                    sd_addr  <= sec;
                    sd_req   <= 1'b1;
                    fill_ptr <= 10'd0;
                    st       <= S_GNT;
                end

                S_GNT: begin
                    if (sd_gnt) begin
                        sd_req   <= 1'b0;
                        st       <= S_FILL;
                    end
                end

                S_FILL: begin
                    if (rd_end) begin
                        sec      <= sec + 32'd1;
                        cons_ptr <= 10'd0;
                        st       <= S_CONS;
                    end
                end

                S_CONS: begin
                    if (cons_ptr >= 10'd512) begin
                        st <= S_REQ;                    // 本扇区吃完，要下一块
                    end else begin
                        cons_ptr <= cons_ptr + 10'd1;
                        st       <= S_CON2;
                    end
                end

                S_CON2: begin
                    if (skip != 13'd0) begin
                        skip <= skip - 13'd1;
                        st   <= S_CONS;
                    end else if (!half) begin
                        lob  <= cons_byte;
                        half <= 1'b1;
                        st   <= S_CONS;
                    end else begin
                        half <= 1'b0;
                        if (x < w_limit)
                            waddr <= waddr + 1'b1;
                        if (row_last) begin
                            // 一行收齐 → 输出所有映射到本行的输出行
                            x     <= 13'd0;
                            waddr <= {LB_AW{1'b0}};
                            raddr <= {LB_AW{1'b0}};
                            e_ox  <= 16'd0;
                            e_sx  <= 16'd0;
                            e_acc <= 16'd0;
                            e_ph  <= 1'b0;
                            st    <= S_EMIT;
                        end else begin
                            x  <= x + 13'd1;
                            st <= S_CONS;
                        end
                    end
                end

                S_EMIT: begin
                    if ((oy < DST_H16) && (sy == {3'd0, row})) begin
                        // 两拍一个像素：A 拍等行缓存数据（raddr 上一拍已给好），B 拍输出。
                        // 注意 raddr 必须"提前一拍"指向**下一个**像素，否则输出会整体
                        // 右移一个像素（raddr -> rdata -> pix_data 是三级流水）。
                        if (!e_ph) begin
                            e_ph <= 1'b1;
                        end else if (pix_wr_ready) begin
                            e_ph      <= 1'b0;
                            pix_data  <= source_top_edge_row ? 24'd0 :
                                         ((e_sx < {3'd0, w_limit}) ? rdata : 24'd0);
                            pix_wr_en <= 1'b1;
                            if ({1'b0, e_acc} + {1'b0, src_w_c} >= {1'b0, OUT_W16}) begin
                                e_acc <= e_acc + {1'b0, src_w_c} - OUT_W16;
                                e_sx  <= e_sx + 16'd1;
                            end else begin
                                e_acc <= e_acc + {1'b0, src_w_c};
                            end
                            if (e_ox + 16'd1 >= OUT_W16) begin
                                // 本输出行结束 → 推进纵向 DDA，下一行从头取列
                                e_ox  <= 16'd0;
                                e_sx  <= 16'd0;
                                e_acc <= 16'd0;
                                e_ph  <= 1'b0;
                                raddr <= {LB_AW{1'b0}};
                                oy    <= oy + 16'd1;
                                if (v_carry) begin
                                    vacc <= v_next - {1'b0, DST_H16};
                                    sy   <= vflip ? (sy - 16'd1) : (sy + 16'd1);
                                end else begin
                                    vacc <= v_next[15:0];
                                end
                            end else begin
                                e_ox  <= e_ox + 16'd1;
                                raddr <= (e_sx_next < {3'd0, w_limit}) ? e_sx_next[LB_AW-1:0]
                                                                       : {LB_AW{1'b0}};
                            end
                        end
                    end else begin
                        // 本源行对应的输出行全部发完
                        row <= row + 13'd1;
                        if (src_last) begin
                            // 画面发完 → 补"流后段"黑边
                            band_rows <= BAND_LAST16;
                            band_lead <= 1'b0;
                            bl_row    <= 16'd0;
                            bl_col    <= 16'd0;
                            bl_ph     <= 1'b0;
                            st        <= (BAND_LAST16 != 16'd0) ? S_BAND : S_FIN;
                        end else begin
                            st <= S_CONS;
                        end
                    end
                end

                // 收尾判定：满一帧就结束，否则补黑
                // pix_cnt 比 pix_wr_en 晚一拍（它在下一拍才累加），
                // 所以这里要用 pix_cnt+1 才能正确判断"刚好发满"，
                // 否则每次都会白跑一趟 S_FLUSH（多 2 拍，不影响内容但不干净）。
                S_FIN: begin
                    if ((pix_cnt + 19'd1) >= OUT_PIXELS_C) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        st   <= S_IDLE;
                    end else begin
                        bl_ph <= 1'b0;
                        st    <= S_FLUSH;
                    end
                end

                // 补黑到整帧（读取中断 / 素材比预期短 / 任何异常都走这里）
                S_FLUSH: begin
                    if (pix_cnt >= OUT_PIXELS_C) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        st   <= S_IDLE;
                    end else if (!bl_ph) begin
                        bl_ph <= 1'b1;
                    end else if (pix_wr_ready) begin
                        bl_ph     <= 1'b0;
                        pix_wr_en <= 1'b1;
                        pix_data  <= 24'd0;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // RGB565 -> RGB888
    function [23:0] rgb565_to_888;
        input [15:0] p;
        reg   [4:0]  r5;
        reg   [5:0]  g6;
        reg   [4:0]  b5;
        begin
            r5 = p[15:11];
            g6 = p[10:5];
            b5 = p[4:0];
            rgb565_to_888 = {r5, r5[4:2], g6, g6[5:4], b5, b5[4:2]};
        end
    endfunction

endmodule
