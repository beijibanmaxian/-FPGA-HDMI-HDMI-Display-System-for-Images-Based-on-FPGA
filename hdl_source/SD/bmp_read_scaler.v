// ============================================================================
// 模块名 : bmp_read_scaler.v
// 功能   : BMP 读取 + 任意分辨率缩放（赛题扩展要求(3)：图片缩放适配）
//          · 扫描 TF 卡中的 24bit 无压缩 BMP（不限分辨率）
//          · 读取时缩放到 640x480 后写入帧缓存
//              水平：先整行读入行缓存，再按 xo*src_w/640 做 Bresenham 采样，
//                    相邻两个源像素之间做线性插值（缩放质量优于最近邻）
//              垂直：按 yo*src_h/480 做 Bresenham 行采样；同一行重复输出时
//                    直接复用行缓存，不需要重新读卡
//              行缓存把"读卡速率"和"输出速率"彻底解耦：放大时不会丢像素，
//              缩小时不会因为等数据而错位
//          · 每行固定输出 640 像素、每帧固定 480 行，保证帧缓存严格对齐
//
// 与官方 bmp_read.v 的区别（端口完全兼容，可直接替换）：
//   官方只接受 640x480 并把文件字节原样写入帧缓存；
//   本模块接受任意分辨率，写入前完成缩放，输出恒为 640x480。
//
// 工程   : HX4S20C 选题一 自有工程（topic1_self_engineered）V3.0
// 日期   : 2026-10-17
// ============================================================================
module bmp_read_scaler #(
    parameter integer OUT_W = 640,              // 输出宽
    parameter integer OUT_H = 480,              // 输出高
    parameter integer MIN_W = 32,               // 接受的最小源宽
    parameter integer MAX_W = 4096,             // 接受的最大源宽
    // 连续这么多个扇区没找到 BMP 头就放弃本次扫描（由上层重复调用做"分段续扫"）。
    // 仿真里可以调小它来构造"碎片化卡"场景。
    parameter integer SCAN_GAP = 16384,
    parameter integer STOP_AFTER_FIRST_IMAGE = 0
)(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [4:0]                scan_target_count,
    input  [7:0]                image_gen,        // 只接受本代号的素材（旧素材自动失效）
    input                       image_gen_any,    // 1 = 放宽：任何 "HX"+合法槽位 的素材都收
                                                  //     严格扫描一张都没找到时由上层重试一次，
                                                  //     避免卡上是上一批素材时直接黑屏
    input                       scan_keep,        // 1=追加模式（不清零已找到的张数）
    output reg                  scan_done,
    output reg                  scan_aborted,     // 本次扫描因越界/放弃而结束（非达到目标）
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [31:0]           scan_found_sectors,   // 该文件占用的扇区数（用于跳转）
    output reg [1:0]            scan_found_kind,      // 0=图片 1=音频 2=视频
    output reg [3:0]            scan_found_slot,      // 图片、音频、视频素材的节目槽位 1..11
    output reg [4:0]            scan_found_total,
    output reg [31:0]           scan_resume,      // 下一次应从哪个扇区继续扫

    // 按指定扇区加载一张图（自动缩放到 OUT_W x OUT_H）
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,      // 保留接口兼容，本模块不限分辨率
    input  [15:0]               bmp_height,

    output                      scaled_o,       // 1 = 本次加载走了缩放路径（源图非 OUT_W x OUT_H）
    output reg                  write_req,
    input                       write_req_ack,
    input                       write_data_ready,

    // SD 读总线（经 sd_bus_arb 仲裁：请求电平保持，数据在授权期间有效）
    output reg                  sd_req,
    output reg [31:0]           sd_req_addr,
    input  [7:0]                rd_data,
    input                       rd_valid,
    input                       rd_end,

    output reg                  bmp_data_wr_en,
    output reg  [23:0]          bmp_data
);

    // ------------------------------------------------------------------
    // 状态定义
    // ------------------------------------------------------------------
    localparam [3:0] S_IDLE      = 4'd0;
    localparam [3:0] S_SCAN      = 4'd1;
    localparam [3:0] S_LOAD_HDR  = 4'd2;
    localparam [3:0] S_LOAD_WAIT = 4'd3;
    localparam [3:0] S_ROWREQ    = 4'd4;      // 决定本行是重新读卡还是复用行缓存
    localparam [3:0] S_ROWEND    = 4'd5;
    localparam [3:0] S_VMAP      = 4'd6;
    localparam [3:0] S_ROWEMIT   = 4'd7;      // 从行缓存插值输出 640 列
    localparam [3:0] S_FAST      = 4'd8;      // 640x480 直通：整文件顺序读，不做行重定位
    localparam [3:0] S_ROWFILL   = 4'd9;      // 顺序读一整行源像素进行缓存
    localparam [3:0] S_BPB0      = 4'd10;     // 读 0 号扇区（MBR / 引导扇区）
    localparam [3:0] S_BPB1      = 4'd11;     // 读分区引导扇区（BPB），得到数据区起点与簇大小
    localparam [3:0] S_BPB2      = 4'd12;     // 流水级 1：FAT 总扇区数 / 根目录扇区数
    localparam [3:0] S_BPB3      = 4'd13;     // 流水级 2：保留区 + FAT + 根目录
    localparam [3:0] S_BPB4      = 4'd14;     // 流水级 3：加分区起始 → 数据区起点

    localparam integer LB_DEPTH = 4096;       // 行缓存深度（像素），等于最大源宽
    localparam integer AW       = 12;         // 行缓存地址位宽

    localparam [18:0] OUT_PIXELS = OUT_W * OUT_H;   // 640*480 = 307200

    // 读超时：50ms 内的 SD 空隙视为暂时等待；超时后补黑并完成整帧。
    // SD 卡可能因音频仲裁或卡内重试出现较长空隙。50ms 门限前不允许
    // 提前补像素；旧 19 位计数器实际只能表达约 5.2ms。
    localparam [22:0] RD_TIMEOUT = 23'd5_000_000;

    // 扫描空档上限：找到上一张图后，连续这么多扇区（2 MB）没有新图就认为扫完了。
    // 没有这个限制时，把 target 设成 8 而后卡里只有 4 张图，就会一路扫到 64 MB
    // （131071 个扇区，实测 30 s 以上），导致上电迟迟不出图。
    // 连续这么多扇区没找到 BMP 头就放弃扫描（防止卡里没有图时无限扫）。
    // 实机反馈 N 只有 5：卡上 8 张图分批拷入后，文件之间的空档超过了旧容差，
    // 后面的图被"放弃扫描"直接跳过。现放宽到 32767 扇区（32 MB），
    // 最坏多花约 6.5 s 启动时间，但能跨过绝大多数碎片；配合 sync_to_sd.py 整卡重写更稳。
    localparam [15:0] SCAN_GAP_SECTORS = SCAN_GAP[15:0];

    reg [3:0] state;
    assign ready = (state == S_IDLE);

    // 全局看门狗：一整行都推进不了就放弃本次加载
    // 说明：用"收到卡数据(rd_valid)/刚输出完一行/正在冲刷扇区"作为进展判据，
    // 这些在 S_SCAN（扫卡）里持续发生，所以扫描永远不会被误杀；
    // 只有"既没有数据、也没有进展"的卡死状态才会累计到超时。
    reg  [23:0] wd;
    localparam [23:0] WD_MAX = 24'd10_000_000;     // 100ms @100MHz，必须晚于读超时兜底
    reg        row_progress_d;
    wire       row_progress = (state == S_ROWEND) && !row_progress_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            row_progress_d <= 1'b0;
        end else begin
            row_progress_d <= (state == S_ROWEND);
        end
    end

    // ==================================================================
    // 扫描步进：按"簇"跳着扫，而不是一个扇区一个扇区扫
    //
    // 为什么可以这样：任何文件的首扇区一定落在某个簇的起点上，
    // 而簇起点 = 数据区起点 + k*每簇扇区数。所以只要从 BPB 里读出
    // "每簇扇区数 spc" 和 "数据区起点 data_start"，就可以每隔 spc
    // 个扇区读一次，扫描速度直接快 spc 倍（4KB 簇 8 倍、32KB 簇 64 倍），
    // 而且不会漏掉任何文件头。
    //
    // 8GB 的卡上 31MB 素材原来要逐扇区扫 6 万多次、启动等 15s 以上，
    // 现在几千次就读完（1~2s），并且能把整张卡扫完（不再受"只扫前 32MB"限制）。
    //
    // 读不到合法 BPB（exFAT/NTFS/非 512 字节扇区）时自动退回逐扇区，
    // 并把范围限制在 40MB 内，保证一定能出图。
    // ==================================================================
    reg  [9:0]  bpb_cnt;
    reg  [7:0]  mbr_bps_l, mbr_bps_h, mbr_ptype, mbr_sig_l, mbr_sig_h;
    reg  [31:0] mbr_lba;
    reg  [7:0]  bpb_bps_l, bpb_bps_h, bpb_spc, bpb_nfat;
    reg  [15:0] bpb_rsvd, bpb_rootent, bpb_fatsz16;
    reg  [31:0] bpb_fatsz32;

    reg  [31:0] scan_base;      // 数据区起始扇区
    reg  [4:0]  scan_sh;        // log2(每簇扇区数)
    reg  [31:0] scan_step;      // 每簇扇区数（=1<<scan_sh）
    // 扫描地址的"下一拍值"单独寄存：原来直接在状态机里写
    //   sd_req_addr <= sd_req_addr + scan_step;
    // 32 位加法器会和状态译码挤进 sd_req_addr 的**时钟使能**路径
    // （实测 9.864ns / 10ns，布线占 65%）。这里把加法提前一级流水，
    // 使能路径就只剩比较器，sd_card_clk 余量从 −0.144ns 恢复到正值。
    reg  [31:0] adv_addr;       // = sd_req_addr + scan_step（寄存一级）
    reg         adv_settled;    // 进入 S_SCAN 后先等一拍，确保 adv_addr 已跟上
    // 扫描退出判据寄存一拍：它们是 sd_req_addr 的纯函数，而 sd_req_addr 在两次
    // 判断之间至少稳定一个扇区（约 190us），所以晚一拍完全等价；
    // 但这样能把两个 32 位比较器从 sd_req_addr 的**时钟使能**路径里摘出去
    // （原来 sd_card_clk 的 -0.874ns/9 端点就是这么来的）。
    reg         addr_ge_lim_r;
    reg         addr_ge_max_r;
    reg  [31:0] part_lba;       // 分区起始扇区
    // rd_end 与最后一个字节同拍到达，此时刚捕获的签名还没写进寄存器，
    // 所以判定要推迟一拍，否则会把合法 MBR 判成"无效"。
    reg         rd_end_d;

    wire [16:0] bpb_bps    = {bpb_bps_h, bpb_bps_l};
    wire [31:0] bpb_fatsz  = (bpb_fatsz32 != 32'd0) ? bpb_fatsz32 : {16'd0, bpb_fatsz16};
    wire [31:0] bpb_fattot = (bpb_nfat >= 8'd2) ? (bpb_fatsz << 1) : bpb_fatsz;
    wire [31:0] bpb_rootsec= {27'd0, bpb_rootent[15:4]} + ((bpb_rootent[3:0] != 4'd0) ? 32'd1 : 32'd0);
    reg         bpb_ok_r;

    reg  [4:0]  spc_sh;
    always @* begin
        case (bpb_spc)
            8'd1:    spc_sh = 5'd0;
            8'd2:    spc_sh = 5'd1;
            8'd4:    spc_sh = 5'd2;
            8'd8:    spc_sh = 5'd3;
            8'd16:   spc_sh = 5'd4;
            8'd32:   spc_sh = 5'd5;
            8'd64:   spc_sh = 5'd6;
            8'd128:  spc_sh = 5'd7;
            default: spc_sh = 5'd31;        // 非法
        endcase
    end

    wire bpb_ok = (bpb_bps == 17'd512) && (spc_sh != 5'd31) &&
                  (bpb_nfat != 8'd0) && (bpb_fatsz != 32'd0);

    // 分区表是否可信：签名 0x55AA + 类型是常见 FAT 类型 + 起始 LBA 非 0。
    // 不可信时把 0 号扇区当引导扇区再读一次（超级软盘），再不行就退回逐扇区扫描。
    // 这一步是为了避免"TF 卡里就是素材、0 号扇区并不是 MBR"时误判出天大的起始扇区。
    wire mbr_valid = (mbr_sig_l == 8'h55) && (mbr_sig_h == 8'hAA) &&
                     (mbr_lba != 32'd0) &&
                     ((mbr_ptype == 8'h01) || (mbr_ptype == 8'h04) || (mbr_ptype == 8'h06) ||
                      (mbr_ptype == 8'h0B) || (mbr_ptype == 8'h0C) || (mbr_ptype == 8'h0E));
    wire [31:0] part_lba_nxt = mbr_valid ? mbr_lba : 32'd0;

    // ------------------------------------------------------------------
    // BPB 数值 → 数据区起点：拆成三级流水寄存器，避免"加法→加法→加法→比较→移位"
    // 串成一条 26 ns 的组合长链（综合后 WNS 会直接掉到 -16 ns）。
    // 状态机里用 S_BPB2/S_BPB3/S_BPB4 三个周期把下面三级依次打拍。
    // ------------------------------------------------------------------
    reg  [31:0] fattot_r, rootsec_r, data_lo_r;
    reg  [31:0] step_m1;            // scan_step - 1（掩码用）
    wire [31:0] step_mask = ~step_m1;

    // 命中素材后要跳过整个文件，并保持落在簇网格上：
    //   advance = ((file_secs + step - 1) & ~(step-1))
    // 预先算好存寄存器，扫描状态机里就只剩一次加法。
    reg  [31:0] file_adv_r;
    wire [31:0] file_adv_c = (file_secs + scan_step - 32'd1) & step_mask;

    // 退化为逐扇区扫描时，把范围压到 40MB，避免在 exFAT 卡上扫几分钟
    wire [31:0] scan_max_eff = (scan_sh == 5'd0 && scan_step == 32'd1)
                               ? ((scan_max_sector > 32'd81920) ? 32'd81920 : scan_max_sector)
                               : scan_max_sector;

    // ------------------------------------------------------------------
    // BMP 头解析结果
    // ------------------------------------------------------------------
    reg [5:0]  hdr_cnt;
    reg [31:0] scan_last_hit;    // 上一张找到的图所在扇区
    reg [15:0] gap_cnt;          // 自上次命中以来"逐扇区"扫过的步数（缩短判定组合链）
    reg [7:0]  hdr_b0, hdr_b1;
    // BMP 保留字段（偏移 6..9）被复用成素材索引：
    //   6..7 = "HX" 固定标记，8 = 生成代号 gen，9 = 节目槽位 1..11
    reg [7:0]  hdr_t0, hdr_t1, hdr_gen, hdr_slot;
    reg        hdr_topdown;
    reg [31:0] file_len;
    reg [31:0] pixel_off;
    reg [12:0] src_w;
    reg [12:0] src_h;
    reg [15:0] bit_count;
    reg [31:0] compress;

    // 工程标记：扫描时只认带标记的图片，避免 TF 卡里已删除旧图的残留扇区被误扫到。
    // 首次扫描找不到时，上层会再发一次 scan_accept_any=1 的宽松扫描。
    wire tag_ok = (hdr_t0 == "H") && (hdr_t1 == "X") &&
                  ((hdr_gen == image_gen) || image_gen_any) && (hdr_slot >= 8'd1) && (hdr_slot <= 8'd11);

    // 三种素材靠 biBitCount 区分：24=图片（真 BMP）、8=音频流、16=视频流
    wire [1:0] kind = (bit_count == 16'd24) ? 2'd0 :
                      (bit_count == 16'd8)  ? 2'd1 :
                      (bit_count == 16'd16) ? 2'd2 : 2'd3;

    // 图片要求宽高合法；音频/视频只要求长度合法（宽高字段留空）
    wire size_ok = (kind == 2'd0) ? ((src_w >= MIN_W) && (src_w <= MAX_W) &&
                                     (src_h >= 13'd1) && (src_h <= 13'd4096))
                                  : (kind != 2'd3);
    wire hdr_ok = (hdr_b0 == "B") && (hdr_b1 == "M") && tag_ok &&
                  (compress == 32'd0) && (!hdr_topdown) &&
                  (file_len > 32'd54) && size_ok;

    wire [13:0] pix_bytes_c = {1'b0, src_w} * 14'd3;                        // 行有效像素字节
    wire [13:0] stride_c    = (({1'b0, src_w} * 14'd3) + 14'd3) & 14'h3FFC; // 行字节数(4 字节对齐)
    wire [31:0] file_secs_c = (file_len + 32'd511) >> 9;

    // 这些量只与 BMP 头有关，打成寄存器后用，避免"乘法→加法→比较"串成长组合链
    reg         hdr_ok_r;
    reg  [13:0] pix_bytes;
    reg  [13:0] stride;
    reg  [31:0] file_secs;
    reg  [31:0] scan_lim;        // scan_max_sector - file_secs（预寄存，缩短扫描判定组合链）

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            hdr_ok_r    <= 1'b0;
            pix_bytes   <= 14'd0;
            stride      <= 14'd0;
            file_secs   <= 32'd1;
            scan_lim    <= 32'd0;
        end else begin
            hdr_ok_r  <= hdr_ok;
            pix_bytes <= pix_bytes_c;
            stride    <= stride_c;
            file_secs <= file_secs_c;
            scan_lim  <= scan_max_sector - file_secs_c;
        end
    end

    // ------------------------------------------------------------------
    // TF 卡字节流处理寄存器
    // ------------------------------------------------------------------
    reg  [13:0] row_byte;        // 当前源行内字节偏移
    reg  [12:0] src_col;         // 当前源像素列
    reg  [1:0]  pix_cnt;         // 像素内字节计数
    reg  [15:0] pix_shift;       // 像素字节拼装
    reg         cur_scaled;      // 本次加载的源图是否需要缩放
    reg         sect_wait;       // 行读完后等待本扇区自然结束
    reg  [9:0]  flush_cnt;       // 冲刷计数：最多再读 512 字节就一定退出
    reg  [8:0]  skip_cnt;        // 定位行首需要跳过的字节数
    reg  [31:0] row_off;         // 当前源行起始的文件字节偏移
    reg  [31:0] base_sector;     // 当前图片首扇区

    // 640x480 直通路径
    reg  [18:0] fast_cnt;        // 已输出像素数（在写口 always 块里累加）
    reg         fast_pad;        // 读超时后补黑以保持整帧对齐
    reg  [22:0] rd_idle_cnt;     // 无数据周期计数（超时判断）
    wire        fast_mode = (src_w == OUT_W[12:0]) && (src_h == OUT_H[12:0]);
    assign      scaled_o  = cur_scaled;
    wire        rd_timeout = (rd_idle_cnt >= RD_TIMEOUT);

    // 垂直映射
    reg  [8:0]  passes;          // 已输出行数 0..479
    reg  [12:0] cur_row;         // 当前源行号
    reg  [12:0] v_acc;           // 垂直 Bresenham 余数

    wire        byte_vld = rd_valid;
    wire [7:0]  byte_d   = rd_data;

    // ------------------------------------------------------------------
    // 行缓存（简单双口 RAM：写口由读卡侧驱动，读口给缩放发射用）
    //   · 同步读，1 拍延迟，综合成 BRAM
    // ------------------------------------------------------------------
    reg [23:0]   lb [0:LB_DEPTH-1];
    reg          lb_wr_en;
    reg [AW-1:0] lb_wr_addr;
    reg [23:0]   lb_wr_data;
    reg          lb_rd_en;
    reg [AW-1:0] lb_rd_addr;
    reg [23:0]   lb_rd_data;

    always @(posedge clk) begin
        if (lb_wr_en)
            lb[lb_wr_addr] <= lb_wr_data;
        if (lb_rd_en)
            lb_rd_data <= lb[lb_rd_addr];
    end

    // 行缓存状态：buf_row 是哪一行（垂直放大时用来复用）
    reg          fill_done;      // 本行像素已收齐
    reg          buf_valid;
    reg  [12:0]  buf_row;

    // 缩放发射侧
    reg  [12:0]  cur_xs;         // 当前输出列对应的源列
    reg  [12:0]  acc;            // 水平 Bresenham 余数（也是插值权重来源）
    reg  [9:0]   xo;             // 当前输出列 0..639
    reg  [12:0]  rd_ptr;         // 行缓存读指针（只前进）
    reg  [23:0]  px0, px1;       // 当前 / 下一个源像素
    reg  [1:0]   eph;            // 0=发射 1=等读命令 2=取同步读数据
    reg          emit_done;

    // ------------------------------------------------------------------
    // 水平线性插值：把相邻两个源像素按权重混合，改善缩放质量
    //   w = 0 → 完全用 a；w = 128 → 完全用 b
    // ------------------------------------------------------------------
    function [23:0] lerp24;
        input [23:0] a;
        input [23:0] b;
        input [7:0]  w;
        reg signed [8:0]  dr, dg, db;
        reg signed [17:0] tr, tg, tb;
        reg [7:0] r, g, bl;
        begin
            dr = $signed({1'b0, b[23:16]}) - $signed({1'b0, a[23:16]});
            dg = $signed({1'b0, b[15:8]})  - $signed({1'b0, a[15:8]});
            db = $signed({1'b0, b[7:0]})   - $signed({1'b0, a[7:0]});
            tr = dr * $signed({1'b0, w});
            tg = dg * $signed({1'b0, w});
            tb = db * $signed({1'b0, w});
            r  = a[23:16] + tr[14:7];
            g  = a[15:8]  + tg[14:7];
            bl = a[7:0]   + tb[14:7];
            lerp24 = {r, g, bl};
        end
    endfunction

    // 水平插值权重：w = acc * 128 / 640 ≈ acc / 5（acc 为该输出列的余量 0..639）
    // 用寄存器提前一拍算好（acc 归一化结束时得出），使下面的 lerp 乘法不再串联一级乘法器
    reg  [7:0] lerp_w_r;

    // ------------------------------------------------------------------
    // 输出口：S_FAST 直通写像素；S_ROWEMIT 从行缓存插值缩放输出
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            fast_cnt       <= 19'd0;
            bmp_data_wr_en <= 1'b0;
            bmp_data       <= 24'd0;
            cur_xs         <= 13'd0;
            acc            <= 13'd0;
            xo             <= 10'd0;
            rd_ptr         <= 13'd0;
            px0            <= 24'd0;
            px1            <= 24'd0;
            eph            <= 2'd0;
            emit_done      <= 1'b0;
            lb_rd_en       <= 1'b0;
            lb_rd_addr     <= {AW{1'b0}};
            lerp_w_r       <= 8'd0;
        end else if (state == S_FAST) begin
            // 640x480 直通：每收齐 3 字节直接写一个像素
            if (fast_cnt >= OUT_PIXELS) begin
                bmp_data_wr_en <= 1'b0;
            end else if (fast_pad) begin
                // 读超时：补黑到完整帧，保持帧缓存写指针对齐
                bmp_data_wr_en <= 1'b1;
                bmp_data       <= 24'd0;
                fast_cnt       <= fast_cnt + 19'd1;
            end else if (byte_vld && (skip_cnt == 9'd0) && (pix_cnt == 2'd2) && !fast_pad) begin
                bmp_data_wr_en <= 1'b1;
                bmp_data       <= {byte_d, pix_shift[15:0]};
                fast_cnt       <= fast_cnt + 19'd1;
            end else begin
                bmp_data_wr_en <= 1'b0;
            end
        end else if (state == S_ROWEMIT) begin
            bmp_data_wr_en <= 1'b0;
            lb_rd_en       <= 1'b0;
            if (emit_done) begin
                // 本行输出完成，等待主状态机切换
            end else if (eph == 2'd0) begin
                if (rd_ptr <= (cur_xs + 13'd1)) begin
                    // 需要 cur_xs 与 cur_xs+1 两个源像素：顺序把行缓存读出来
                    // （最后会多读一拍 cur_xs+1 = src_w，其数据只进 px1，不参与运算）
                    lb_rd_en   <= 1'b1;
                    lb_rd_addr <= rd_ptr[AW-1:0];
                    rd_ptr     <= rd_ptr + 13'd1;
                    eph        <= 2'd1;
                end else if (write_data_ready) begin
                    // 两个像素都到齐：按余量 acc 做线性插值后输出
                    bmp_data_wr_en <= 1'b1;
                    bmp_data       <= (cur_xs < (src_w - 13'd1)) ? lerp24(px0, px1, lerp_w_r)
                                                                 : px0;
                    xo             <= xo + 10'd1;
                    acc            <= {1'b0, acc} + {1'b0, src_w};
                    eph            <= 2'd3;
                    if (xo == (OUT_W[9:0] - 10'd1))
                        emit_done <= 1'b1;
                end
            end else if (eph == 2'd1) begin
                // 读命令已发出，等 BRAM 把数据打出来
                eph <= 2'd2;
            end else if (eph == 2'd2) begin
                // 同步读数据有效
                px0 <= px1;
                px1 <= lb_rd_data;
                eph <= 2'd0;
            end else begin
                // 缩小倍数 >1 时一步可能跨过多个源列，余数要循环归一化
                if (acc >= OUT_W[12:0]) begin
                    acc    <= acc - OUT_W[12:0];
                    cur_xs <= cur_xs + 13'd1;
                end else begin
                    // 余数已定：提前一拍算好下一列的插值权重
                    lerp_w_r <= (acc * 19'd205) >> 10;
                    eph      <= 2'd0;
                end
            end
        end else begin
            // 不在输出状态：清空发射侧状态
            bmp_data_wr_en <= 1'b0;
            fast_cnt       <= 19'd0;
            cur_xs         <= 13'd0;
            acc            <= 13'd0;
            xo             <= 10'd0;
            rd_ptr         <= 13'd0;
            px0            <= 24'd0;
            px1            <= 24'd0;
            eph            <= 2'd0;
            emit_done      <= 1'b0;
            lb_rd_en       <= 1'b0;
            lerp_w_r       <= 8'd0;
        end
    end

    // ------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state             <= S_IDLE;
            state_code        <= 4'd0;
            wd                <= 24'd0;
            sd_req       <= 1'b0;
            sd_req_addr  <= 32'd0;
            adv_addr     <= 32'd0;
            adv_settled  <= 1'b0;
            addr_ge_lim_r <= 1'b0;
            addr_ge_max_r <= 1'b0;
            write_req         <= 1'b0;
            scan_done         <= 1'b0;
            scan_found_valid  <= 1'b0;
            scan_found_sector <= 32'd0;
            scan_found_total  <= 5'd0;
            scan_last_hit     <= 32'd0;
            gap_cnt           <= 16'd0;
            hdr_cnt           <= 6'd0;
            hdr_b0            <= 8'd0;
            hdr_b1            <= 8'd0;
            hdr_t0            <= 8'd0;
            hdr_t1            <= 8'd0;
            hdr_gen           <= 8'd0;
            hdr_slot          <= 8'd0;
            hdr_topdown       <= 1'b0;
            file_len          <= 32'd0;
            pixel_off         <= 32'd54;
            src_w             <= 13'd0;
            src_h             <= 13'd0;
            bit_count         <= 16'd0;
            compress          <= 32'd0;
            row_byte          <= 14'd0;
            src_col           <= 13'd0;
            pix_cnt           <= 2'd0;
            pix_shift         <= 16'd0;
            skip_cnt          <= 9'd0;
            row_off           <= 32'd0;
            base_sector       <= 32'd0;
            passes            <= 9'd0;
            cur_row           <= 13'd0;
            v_acc             <= 13'd0;
            fast_pad          <= 1'b0;
            rd_idle_cnt       <= 23'd0;
            fill_done         <= 1'b0;
            buf_valid         <= 1'b0;
            buf_row           <= 13'd0;
            lb_wr_en          <= 1'b0;
            lb_wr_addr        <= {AW{1'b0}};
            lb_wr_data        <= 24'd0;
            sect_wait         <= 1'b0;
            flush_cnt         <= 10'd0;
            cur_scaled        <= 1'b0;
        end else if (!sd_init_done) begin
            state            <= S_IDLE;
            state_code       <= 4'd0;
            sd_req      <= 1'b0;
            sd_req_addr <= 32'd0;
            write_req        <= 1'b0;
            scan_done        <= 1'b0;
            scan_aborted     <= 1'b0;
            scan_found_valid <= 1'b0;
            bpb_cnt    <= 10'd0;
            mbr_sig_l  <= 8'd0;
            mbr_sig_h  <= 8'd0;
            mbr_bps_l  <= 8'd0;
            mbr_bps_h  <= 8'd0;
            mbr_ptype  <= 8'd0;
            mbr_lba    <= 32'd0;
            part_lba   <= 32'd0;
            bpb_bps_l  <= 8'd0;
            bpb_bps_h  <= 8'd0;
            bpb_spc    <= 8'd0;
            bpb_nfat   <= 8'd0;
            bpb_rsvd   <= 16'd0;
            bpb_rootent<= 16'd0;
            bpb_fatsz16<= 16'd0;
            bpb_fatsz32<= 32'd0;
            scan_base  <= 32'd0;
            scan_sh    <= 5'd0;
            scan_step  <= 32'd1;
            rd_end_d   <= 1'b0;
            fattot_r   <= 32'd0;
            rootsec_r  <= 32'd0;
            data_lo_r  <= 32'd0;
            step_m1    <= 32'd0;
            file_adv_r <= 32'd1;
            bpb_ok_r   <= 1'b0;
            scan_found_total <= 5'd0;
        end else begin
            scan_found_valid <= 1'b0;
            lb_wr_en         <= 1'b0;

            // 扫描地址的下一拍值（一级流水：把 32 位加法移出 sd_req_addr 的时钟使能路径）
            adv_addr <= sd_req_addr + scan_step;
            // 退出判据寄存（见声明处说明）
            addr_ge_lim_r <= (sd_req_addr >= scan_lim);
            addr_ge_max_r <= (sd_req_addr >= scan_max_eff);

            rd_end_d   <= rd_end;
            step_m1    <= scan_step - 32'd1;
            file_adv_r <= file_adv_c;

            // ---------------- 引导扇区 / BPB 字节捕获 ----------------
            if (byte_vld && (state == S_BPB0)) begin
                case (bpb_cnt)
                    10'd11:  mbr_bps_l <= byte_d;
                    10'd12:  mbr_bps_h <= byte_d;
                    10'd450: mbr_ptype <= byte_d;      // 分区表第 0 项的类型（0x1C2）
                    10'd454: mbr_lba[7:0]   <= byte_d; // 起始 LBA（0x1C6）
                    10'd455: mbr_lba[15:8]  <= byte_d;
                    10'd456: mbr_lba[23:16] <= byte_d;
                    10'd457: mbr_lba[31:24] <= byte_d;
                    10'd510: mbr_sig_l <= byte_d;      // 0x55
                    10'd511: mbr_sig_h <= byte_d;      // 0xAA
                    default: ;
                endcase
            end
            if (byte_vld && (state == S_BPB1)) begin
                case (bpb_cnt)
                    10'd11:  bpb_bps_l <= byte_d;
                    10'd12:  bpb_bps_h <= byte_d;
                    10'd13:  bpb_spc   <= byte_d;
                    10'd14:  bpb_rsvd[7:0]  <= byte_d;
                    10'd15:  bpb_rsvd[15:8] <= byte_d;
                    10'd16:  bpb_nfat  <= byte_d;
                    10'd17:  bpb_rootent[7:0]  <= byte_d;
                    10'd18:  bpb_rootent[15:8] <= byte_d;
                    10'd22:  bpb_fatsz16[7:0]  <= byte_d;
                    10'd23:  bpb_fatsz16[15:8] <= byte_d;
                    10'd36:  bpb_fatsz32[7:0]   <= byte_d;
                    10'd37:  bpb_fatsz32[15:8]  <= byte_d;
                    10'd38:  bpb_fatsz32[23:16] <= byte_d;
                    10'd39:  bpb_fatsz32[31:24] <= byte_d;
                    default: ;
                endcase
            end
            if (byte_vld && ((state == S_BPB0) || (state == S_BPB1)))
                bpb_cnt <= bpb_cnt + 10'd1;

            // ---------------- BMP 头解析 ----------------
            if (byte_vld && (hdr_cnt < 6'd54) &&
                ((state == S_SCAN) || (state == S_LOAD_HDR))) begin
                hdr_cnt <= hdr_cnt + 6'd1;
                case (hdr_cnt)
                    6'd0 : hdr_b0 <= byte_d;
                    6'd1 : hdr_b1 <= byte_d;
                    6'd6 : hdr_t0 <= byte_d;
                    6'd7 : hdr_t1 <= byte_d;
                    6'd8 : hdr_gen  <= byte_d;
                    6'd9 : hdr_slot <= byte_d;
                    6'd2 : file_len[7:0]   <= byte_d;
                    6'd3 : file_len[15:8]  <= byte_d;
                    6'd4 : file_len[23:16] <= byte_d;
                    6'd5 : file_len[31:24] <= byte_d;
                    6'd10: pixel_off[7:0]   <= byte_d;
                    6'd11: pixel_off[15:8]  <= byte_d;
                    6'd12: pixel_off[23:16] <= byte_d;
                    6'd13: pixel_off[31:24] <= byte_d;
                    6'd18: src_w[7:0]   <= byte_d;
                    6'd19: src_w[12:8]  <= byte_d[4:0];
                    6'd22: src_h[7:0]   <= byte_d;
                    6'd23: src_h[12:8]  <= byte_d[4:0];
                    6'd25: hdr_topdown  <= (byte_d != 8'd0);   // 高度高字节非 0 → 自顶向下格式
                    6'd28: bit_count[7:0]  <= byte_d;
                    6'd29: bit_count[15:8] <= byte_d;
                    6'd30: compress[7:0]   <= byte_d;
                    6'd31: compress[15:8]  <= byte_d;
                    default: ;
                endcase
            end
            if (rd_end)
                hdr_cnt <= 6'd0;

            // ---------------- 行内字节处理（缩放路径：把整行写进行缓存） ----------------
            if (byte_vld && (state == S_ROWFILL)) begin
                if (skip_cnt != 9'd0) begin
                    skip_cnt <= skip_cnt - 9'd1;
                end else if (!fill_done) begin
                    if (row_byte < pix_bytes) begin
                        pix_shift <= {byte_d, pix_shift[15:8]};   // 新字节进高字节
                        if (pix_cnt == 2'd2) begin
                            pix_cnt    <= 2'd0;
                            src_col    <= src_col + 13'd1;
                            lb_wr_en   <= 1'b1;
                            lb_wr_addr <= src_col[AW-1:0];
                            lb_wr_data <= {byte_d, pix_shift[15:0]};   // BMP 为 B,G,R
                            if ((src_col + 13'd1) >= src_w)
                                fill_done <= 1'b1;
                        end else begin
                            pix_cnt <= pix_cnt + 2'd1;
                        end
                    end
                    if (row_byte == (stride - 14'd1))
                        fill_done <= 1'b1;                 // 行尾（含 4 字节对齐填充）
                    else
                        row_byte <= row_byte + 14'd1;
                end
            end

            // ---------------- 640x480 直通字节处理 ----------------
            if (byte_vld && (state == S_FAST)) begin
                if (skip_cnt != 9'd0) begin
                    skip_cnt <= skip_cnt - 9'd1;
                end else if (!fast_pad) begin
                    pix_shift <= {byte_d, pix_shift[15:8]};
                    pix_cnt   <= (pix_cnt == 2'd2) ? 2'd0 : (pix_cnt + 2'd1);
                end
            end

            // ---------------- 读超时计数 ----------------
            if (byte_vld && ((state == S_FAST) || (state == S_ROWFILL) || (state == S_LOAD_HDR)))
                rd_idle_cnt <= 23'd0;
            else if ((state == S_FAST) || (state == S_ROWFILL) || (state == S_LOAD_HDR)) begin
                if (rd_idle_cnt != 23'h7F_FFFF)
                    rd_idle_cnt <= rd_idle_cnt + 23'd1;
            end else
                rd_idle_cnt <= 23'd0;

            // ---------------- 主状态 ----------------
            case (state)
                // ============ 空闲 ============
                S_IDLE: begin
                    state_code  <= 4'd1;
                    sd_req <= 1'b0;
                    write_req   <= 1'b0;

                    if (scan_start) begin
                        scan_done        <= 1'b0;
                        scan_aborted     <= 1'b0;
                        if (!scan_keep)
                            scan_found_total <= 5'd0;
                        scan_last_hit    <= scan_start_sector;
                        scan_resume      <= scan_start_sector;
                        gap_cnt          <= 16'd0;
                        hdr_cnt          <= 6'd0;
                        bpb_cnt          <= 10'd0;
                        sd_req_addr      <= 32'd0;
                        sd_req           <= 1'b1;
                        state            <= S_BPB0;
                    end else if (load_start) begin
                        base_sector      <= load_sector;
                        hdr_cnt          <= 6'd0;
                        sd_req_addr <= load_sector;
                        sd_req      <= 1'b1;
                        state            <= S_LOAD_HDR;
                    end
                end

                // ============ 读 0 号扇区：找分区表 ============
                S_BPB0: begin
                    state_code <= 4'd2;
                    if (rd_end_d) begin
                        bpb_cnt <= 10'd0;
                        part_lba    <= part_lba_nxt;
                        sd_req_addr <= part_lba_nxt;
                        state       <= S_BPB1;
                    end
                end

                // ============ 读分区引导扇区：拿到 BPB ============
                S_BPB1: begin
                    state_code <= 4'd2;
                    if (rd_end_d) begin
                        bpb_cnt  <= 10'd0;
                        sd_req   <= 1'b0;
                        bpb_ok_r <= bpb_ok;
                        state    <= S_BPB2;
                    end
                end

                // ============ 流水级 1：FAT 总扇区数 / 根目录扇区数 ============
                S_BPB2: begin
                    state_code <= 4'd2;
                    fattot_r   <= bpb_fattot;
                    rootsec_r  <= bpb_rootsec;
                    state      <= S_BPB3;
                end

                // ============ 流水级 2：保留区 + FAT + 根目录 ============
                S_BPB3: begin
                    state_code <= 4'd2;
                    data_lo_r  <= {16'd0, bpb_rsvd} + fattot_r + rootsec_r;
                    state      <= S_BPB4;
                end

                // ============ 流水级 3：加分区起始 = 数据区起点 ============
                S_BPB4: begin
                    state_code <= 4'd2;
                    if (bpb_ok_r) begin
                        scan_base   <= part_lba + data_lo_r;
                        scan_sh     <= spc_sh;
                        scan_step   <= (32'd1 << spc_sh);
                    end else begin
                        // 认不出文件系统：退回逐扇区扫描（范围由 scan_max_eff 限制）
                        scan_base   <= 32'd0;
                        scan_sh     <= 5'd0;
                        scan_step   <= 32'd1;
                    end
                    sd_req_addr <= scan_start_sector;
                    adv_settled <= 1'b0;
                    sd_req      <= 1'b1;
                    state       <= S_SCAN;
                end

                // ============ 扫描候选图片 ============
                S_SCAN: begin
                    state_code <= 4'd2;
                    // 进扫描后先空转一拍，让 adv_addr（= sd_req_addr + scan_step）跟上新参数
                    if (!adv_settled) begin
                        adv_settled <= 1'b1;
                    end else if (rd_end) begin
                        sd_req <= 1'b1;
                        if (hdr_ok_r) begin
                            scan_found_valid  <= 1'b1;
                            scan_found_sector  <= sd_req_addr;
                            scan_found_sectors <= file_secs;
                            scan_found_kind    <= kind;
                            scan_found_slot    <= hdr_slot[3:0];
                            scan_found_total   <= scan_found_total + 5'd1;
                            scan_last_hit     <= sd_req_addr;
                            gap_cnt           <= 16'd0;
                            if (((scan_found_total + 5'd1) >= scan_target_count) ||
                                addr_ge_lim_r ||
                                ((STOP_AFTER_FIRST_IMAGE != 0) && !scan_keep &&
                                 (kind == 2'd0) && (hdr_slot == 4'd1))) begin
                                scan_done   <= 1'b1;
                                scan_resume <= sd_req_addr + file_adv_r;
                                sd_req <= 1'b0;
                                state       <= S_IDLE;
                            end else begin
                                // 命中后跳过整个文件（file_adv_r 已对齐到簇网格）
                                sd_req_addr <= sd_req_addr + file_adv_r;
                                scan_resume <= sd_req_addr + file_adv_r;
                            end
                        end else begin
                            if (addr_ge_max_r ||
                                ((scan_found_total != 5'd0) && (gap_cnt >= SCAN_GAP_SECTORS))) begin
                                scan_done    <= 1'b1;
                                scan_aborted <= 1'b1;      // 越界/放弃：告知上层"还没扫完"
                                sd_req  <= 1'b0;
                                state        <= S_IDLE;
                            end else begin
                                // 按簇步进：文件首扇区一定在簇边界上，不会漏
                                sd_req_addr <= adv_addr;
                                scan_resume <= adv_addr;
                                if (gap_cnt > (16'd65535 - scan_step[15:0]))
                                    gap_cnt <= 16'd65535;
                                else
                                    gap_cnt <= gap_cnt + scan_step[15:0];
                            end
                        end
                    end
                end

                // ============ 读图片头 ============
                S_LOAD_HDR: begin
                    state_code <= 4'd2;
                    if (rd_end) begin
                        sd_req <= 1'b0;
                        if (hdr_ok_r) begin
                            write_req <= 1'b1;
                            state     <= S_LOAD_WAIT;
                        end else begin
                            state_code <= 4'd9;          // 格式不支持（数码管显示 9）
                            state      <= S_IDLE;
                        end
                    end
                end

                // ============ 等帧缓存响应 ============
                S_LOAD_WAIT: begin
                    state_code <= 4'd3;
                    if (write_req_ack) begin
                        write_req        <= 1'b0;
                        row_off          <= pixel_off;
                        cur_row          <= 13'd0;
                        v_acc            <= 13'd0;
                        sd_req_addr <= base_sector + (pixel_off >> 9);
                        skip_cnt         <= pixel_off[8:0];
                        row_byte         <= 14'd0;
                        src_col          <= 13'd0;
                        pix_cnt          <= 2'd0;
                        pix_shift        <= 16'd0;
                        passes           <= 9'd1;
                        fast_pad         <= 1'b0;
                        rd_idle_cnt      <= 23'd0;
                        sd_req      <= 1'b1;

                        cur_scaled <= ~fast_mode;
                        // Use the row-buffered path even for native-size images.
                        // It can pause cleanly when the SDRAM write FIFO is nearly full.
                        buf_valid <= 1'b0;
                        state     <= S_ROWREQ;
                    end
                end

                // ============ 640x480 直通 ============
                S_FAST: begin
                    state_code <= 4'd5;

                    // 读超时：由 fast_pad 补黑到整帧，避免 framebuffer 指针错位
                    if (rd_timeout)
                        fast_pad <= 1'b1;

                    if (fast_pad && (fast_cnt >= OUT_PIXELS)) begin
                        // 已经补齐一整帧，直接收工（不再等 SD 扇区结束）
                        sd_req <= 1'b0;
                        state       <= S_IDLE;
                    end else if (rd_end) begin
                        if (fast_cnt >= OUT_PIXELS) begin
                            sd_req <= 1'b0;
                            state       <= S_IDLE;
                        end else begin
                            sd_req_addr <= sd_req_addr + 32'd1;
                            sd_req      <= 1'b1;
                        end
                    end else if (fast_cnt >= OUT_PIXELS) begin
                        sd_req <= 1'b0;          // 数据已够，等本扇区自然结束
                    end else begin
                        sd_req <= 1'b1;
                    end
                end

                // ============ 决定本行：读卡 or 复用行缓存 ============
                S_ROWREQ: begin
                    state_code <= 4'd4;
                    if (buf_valid && (buf_row == cur_row)) begin
                        // 垂直放大：同一源行重复输出，直接用行缓存
                        state <= S_ROWEMIT;
                    end else begin
                        sd_req_addr <= base_sector + (row_off >> 9);
                        skip_cnt         <= row_off[8:0];
                        row_byte         <= 14'd0;
                        src_col          <= 13'd0;
                        pix_cnt          <= 2'd0;
                        pix_shift        <= 16'd0;
                        fill_done        <= 1'b0;
                        buf_valid        <= 1'b0;
                        sect_wait        <= 1'b0;
                        rd_idle_cnt      <= 23'd0;
                        sd_req      <= 1'b1;
                        state            <= S_ROWFILL;
                    end
                end

                // ============ 顺序读一整行源像素进行缓存 ============
                S_ROWFILL: begin
                    state_code <= 4'd4;
                    if (rd_timeout && !fill_done) begin
                        // SD 卡长时间无数据：按本行已读完处理，避免整机卡死
                        fill_done <= 1'b1;
                    end
                    if (!fill_done) begin
                        if (rd_end) begin
                            sd_req_addr <= sd_req_addr + 32'd1;   // 跨扇区继续
                            sd_req      <= 1'b1;
                        end
                    end else if (!sect_wait) begin
                        // 行已收齐，但仍要把本扇区读完，保证下次定位时字节流是干净的。
                        // 注意：这里不能立刻撤销 sd_req，否则真实 SD 控制器会把该扇区
                        // 判为放弃、不再产生 rd_end，本状态就会永久卡死。
                        sect_wait <= 1'b1;
                        flush_cnt <= 10'd0;
                    end else if (rd_end || (flush_cnt >= 10'd511) ||
                                 (rd_idle_cnt >= RD_TIMEOUT)) begin
                        // 三种收工条件：扇区自然结束 / 已读掉最多 512 字节 / 卡长时间无响应。
                        // 后两条是死锁兜底 —— 缺了它们，卡一旦不吐数据，
                        // load_busy 就永远为 1，表现为"所有按键都失灵"。
                        sd_req <= 1'b0;
                        buf_row     <= cur_row;
                        buf_valid   <= 1'b1;
                        state       <= S_ROWEMIT;
                    end else begin
                        sd_req <= 1'b1;      // 继续把本扇区剩余字节读掉（丢弃）
                        if (byte_vld && (flush_cnt != 10'd511))
                            flush_cnt <= flush_cnt + 10'd1;
                    end
                end

                // ============ 从行缓存插值输出 640 列 ============
                S_ROWEMIT: begin
                    state_code <= 4'd4;
                    if (emit_done)
                        state <= S_ROWEND;
                end

                // ============ 行结束：累计垂直映射 ============
                S_ROWEND: begin
                    state_code <= 4'd4;
                    v_acc <= v_acc + src_h;
                    state <= S_VMAP;
                end

                // ============ 垂直 Bresenham ============
                S_VMAP: begin
                    state_code <= 4'd4;
                    if (passes >= OUT_H[8:0]) begin
                        sd_req <= 1'b0;
                        state       <= S_IDLE;           // 整帧输出完成
                    end else if (v_acc >= OUT_H[12:0]) begin
                        if (cur_row < (src_h - 13'd1)) begin
                            v_acc   <= v_acc - OUT_H[12:0];
                            cur_row <= cur_row + 13'd1;
                            row_off <= row_off + {18'd0, stride};
                        end else begin
                            v_acc <= 13'd0;              // 已到最后一行，重复最后一行
                        end
                    end else begin
                        passes <= passes + 9'd1;
                        state  <= S_ROWREQ;
                    end
                end

                default: state <= S_IDLE;
            endcase

            // ============ 全局看门狗（最后写，优先级最高） ============
            // 目的：行读取/插值输出任何一处卡住时，不让 ready 永远为 0
            //       （ready=0 会让 sd_card_bmp_v2 永远等不到加载完成 -> 按键全失灵）。
            //
            // ★ 血的教训：这个看门狗**绝对不能**在长时间合法运行的状态里计时。
            //   第一版只按"没有输出完整行"判断，结果 S_SCAN（按簇步进扫整张卡）
            //   每 80ms 就被强行踢回 S_IDLE -> 扫描永远扫不完
            //   -> 一个素材都找不到 -> 黑屏 + 无音频。
            //   现在改为：只要**收到卡数据**或**状态发生迁移**就清零计数，
            //   只有在"既没数据、状态也不动"时才累计 -> S_SCAN 永远不会被误杀。
            if ((state == S_IDLE) || rd_valid || row_progress || sect_wait) begin
                wd <= 24'd0;
            end else if (wd == WD_MAX) begin
                wd     <= 24'd0;
                sd_req <= 1'b0;
                state  <= S_IDLE;               // 放弃本次加载，让上层重新发命令
            end else begin
                wd <= wd + 24'd1;
            end
        end
    end

endmodule
