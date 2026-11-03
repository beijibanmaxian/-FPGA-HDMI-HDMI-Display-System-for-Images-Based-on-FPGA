// ============================================================================
// XCOM UTF-8/GBK text receiver.
// ASCII plus the built-in 16x16 Chinese glyph set is rendered into a 480x64 bitmap.
// CRLF is treated as one line ending; unsupported characters become '?'.
// Chinese glyphs are rasterized from Noto Sans SC under SIL OFL 1.1.
// ============================================================================
module host_text_rx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer TIMEOUT_MS  = 200,
    parameter integer COLS        = 30,
    parameter integer ROWS        = 2
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    output wire        busy,
    output reg         text_valid,
    output reg         text_wr_en,
    output reg  [11:0] text_wr_addr,
    output reg  [7:0]  text_wr_data,
    output reg         ack_pulse,
    output reg  [7:0]  ack_code
);
    assign busy = 1'b0;

    reg [7:0] line0 [0:COLS-1];
    reg [7:0] line1 [0:COLS-1];
    reg [4:0] len0, len1;
    reg [1:0] cur_line;
    reg [5:0] col_ptr;
    reg       redraw;
    reg       last_was_cr;
    reg [1:0] utf8_need;
    reg [20:0] utf8_codepoint;
    reg [7:0]  utf8_first_byte;
    reg        gbk_pending;
    reg [7:0]  gbk_lead;

    reg  [7:0] g_ch;
    reg  [2:0] g_row;
    wire [7:0] g_bits;
    wire [7:0] rd_char;
    wire [15:0] cjk_pixels;

    osd_font_rom u_font (
        .ch   (g_ch),
        .row  (g_row),
        .bits (g_bits)
    );

    reg [4:0] rd_col;
    reg [3:0] rd_r;
    reg       rd_half;
    reg [1:0] rd_line;
    reg [1:0] st;

    localparam [1:0] S_SETTLE = 2'd0;
    localparam [1:0] S_WRITE  = 2'd1;
    localparam [1:0] S_NEXT   = 2'd2;

    wire [4:0] rd_col_lim = (rd_line == 2'd0) ? len0 : len1;
    assign rd_char = (rd_line == 2'd0) ? line0[rd_col] : line1[rd_col];
    wire [7:0] cell_char = (rd_col < rd_col_lim) ? rd_char : 8'h20;
    wire [11:0] row_base  = (rd_line == 2'd0) ? 12'd0 : 12'd960;
    wire [11:0] byte_off  = row_base + (rd_r * 12'd60) +
                            (rd_col * 12'd2) + {11'd0, rd_half};

    wire [7:0] cell_left  = {g_bits[7], g_bits[7], g_bits[6], g_bits[6],
                             g_bits[5], g_bits[5], g_bits[4], g_bits[4]};
    wire [7:0] cell_right = {g_bits[3], g_bits[3], 6'b000000};
    wire [15:0] glyph_pixels = cell_char[7]
                             ? cjk_pixels : {cell_left, cell_right};

    function [7:0] utf8_glyph_id;
        input [15:0] cp;
        begin
            case (cp)
                16'h4E2D: utf8_glyph_id = 8'h80;
                16'h6587: utf8_glyph_id = 8'h81;
                16'h4F60: utf8_glyph_id = 8'h82;
                16'h597D: utf8_glyph_id = 8'h83;
                16'h56FE: utf8_glyph_id = 8'h84;
                16'h7247: utf8_glyph_id = 8'h85;
                16'h64AD: utf8_glyph_id = 8'h86;
                16'h653E: utf8_glyph_id = 8'h87;
                16'h8F6E: utf8_glyph_id = 8'h88;
                16'h56DE: utf8_glyph_id = 8'h89;
                16'h6D4B: utf8_glyph_id = 8'h8A;
                16'h8BD5: utf8_glyph_id = 8'h8B;
                16'h5B57: utf8_glyph_id = 8'h8C;
                16'h5E55: utf8_glyph_id = 8'h8D;
                default: utf8_glyph_id = 8'h3F;
            endcase
        end
    endfunction

    function [7:0] gbk_glyph_id;
        input [15:0] pair;
        begin
            case (pair)
                16'hD6D0: gbk_glyph_id = 8'h80;
                16'hCEC4: gbk_glyph_id = 8'h81;
                16'hC4E3: gbk_glyph_id = 8'h82;
                16'hBAC3: gbk_glyph_id = 8'h83;
                16'hCDBC: gbk_glyph_id = 8'h84;
                16'hC6AC: gbk_glyph_id = 8'h85;
                16'hB2A5: gbk_glyph_id = 8'h86;
                16'hB7C5: gbk_glyph_id = 8'h87;
                16'hC2D6: gbk_glyph_id = 8'h88;
                16'hBBD8: gbk_glyph_id = 8'h89;
                16'hB2E2: gbk_glyph_id = 8'h8A;
                16'hCAD4: gbk_glyph_id = 8'h8B;
                16'hD7D6: gbk_glyph_id = 8'h8C;
                16'hC4BB: gbk_glyph_id = 8'h8D;
                default: gbk_glyph_id = 8'd0;
            endcase
        end
    endfunction

    function [15:0] cjk_row;
        input [7:0] glyph_id;
        input [3:0] row;
        reg [255:0] glyph;
        reg [7:0] shift_amt;
        begin
            case (glyph_id)
                8'h80: glyph = 256'h0080008000803FFE208220822082208220823FFE208200800080008000800000;
                8'h81: glyph = 256'h0100010000807FFE1010101008200420044002400180018006600810700E0000;
                8'h82: glyph = 256'h08000900110011FE32225220902011241124122212221020102010E000000000;
                8'h83: glyph = 256'h100020FE20047E082410241045FF4410281018100C1012102010407000000000;
                8'h84: glyph = 256'h7FFC4004440447E44C44528443044CC4522441844C0443C440047FFC00000000;
                8'h85: glyph = 256'h10401040104010401FFE100010001FF010103010201020104010000000000000;
                8'h86: glyph = 256'h103C11E410A47C2813FF10B011281A2675FF112211FE112211FE710200000000;
                8'h87: glyph = 256'h082000207F40107F10441EC412C81228122832302210222842444F8200000000;
                8'h88: glyph = 256'h102020607C502088330450827C88109010E0148078801082108200FE00000000;
                8'h89: glyph = 256'h7FFC4004400440044FE448244824482448244FE4400440047FFC400400000000;
                8'h8A: glyph = 256'h200217EA042A052A452A252A052A052A052A252A210242424422480E00000000;
                8'h8B: glyph = 256'h001020141012001007FF0010701013F0109010901088108819C9200E00060000;
                8'h8C: glyph = 256'h010000807FFE400240020FF00020004001807FFE010001000100010007000000;
                8'h8D: glyph = 256'h04207FFE04201FFC10041FFC1FFC02007FFF08883FFC508B108810B800000000;
                default: glyph = 256'd0;
            endcase
            shift_amt = {4'd15 - row, 4'b0000};
            cjk_row = glyph[shift_amt +: 16];
        end
    endfunction

    function [7:0] ascii_upper;
        input [7:0] value;
        begin
            ascii_upper = ((value >= 8'h61) && (value <= 8'h7A))
                        ? value - 8'h20 : value;
        end
    endfunction

    assign cjk_pixels = cjk_row(cell_char, rd_r);

    task text_byte;
        input [7:0] value;
        integer c;
        begin
            redraw <= 1'b1;
            if ((value == 8'h0D) || (value == 8'h0A)) begin
                if (cur_line == 2'd0) begin
                    cur_line <= 2'd1;
                    col_ptr <= len1;
                end else begin
                    for (c = 0; c < COLS; c = c + 1)
                        line0[c] <= line1[c];
                    len0 <= len1;
                    len1 <= 5'd0;
                    cur_line <= 2'd1;
                    col_ptr <= 6'd0;
                    text_valid <= (len1 != 5'd0);
                end
            end else if ((value == 8'h08) || (value == 8'h7F)) begin
                if (col_ptr != 6'd0) begin
                    col_ptr <= col_ptr - 6'd1;
                    if (cur_line == 2'd0) begin
                        if (len0 != 5'd0) len0 <= len0 - 5'd1;
                    end else begin
                        if (len1 != 5'd0) len1 <= len1 - 5'd1;
                    end
                end
            end else if (value == 8'h0C) begin
                len0 <= 5'd0;
                len1 <= 5'd0;
                cur_line <= 2'd0;
                col_ptr <= 6'd0;
                text_valid <= 1'b0;
            end else if (((value >= 8'h20) && (value <= 8'h7E)) ||
                         ((value >= 8'h80) && (value <= 8'hAF))) begin
                if (col_ptr < COLS) begin
                    if (cur_line == 2'd0) begin
                        line0[col_ptr[4:0]] <= value;
                        if (len0 < (col_ptr[4:0] + 5'd1))
                            len0 <= col_ptr[4:0] + 5'd1;
                    end else begin
                        line1[col_ptr[4:0]] <= value;
                        if (len1 < (col_ptr[4:0] + 5'd1))
                            len1 <= col_ptr[4:0] + 5'd1;
                    end
                    col_ptr <= col_ptr + 6'd1;
                    text_valid <= 1'b1;
                end
            end
        end
    endtask

    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            len0 <= 5'd0;
            len1 <= 5'd0;
            cur_line <= 2'd0;
            col_ptr <= 6'd0;
            redraw <= 1'b1;
            last_was_cr <= 1'b0;
            utf8_need <= 2'd0;
            utf8_codepoint <= 21'd0;
            utf8_first_byte <= 8'd0;
            gbk_pending <= 1'b0;
            gbk_lead <= 8'd0;
            text_valid <= 1'b0;
            text_wr_en <= 1'b0;
            text_wr_addr <= 12'd0;
            text_wr_data <= 8'd0;
            ack_pulse <= 1'b0;
            ack_code <= 8'd0;
            rd_col <= 5'd0;
            rd_r <= 4'd0;
            rd_half <= 1'b0;
            rd_line <= 2'd0;
            st <= S_SETTLE;
            g_ch <= 8'h20;
            g_row <= 3'd0;
            for (i = 0; i < COLS; i = i + 1) begin
                line0[i] <= 8'h20;
                line1[i] <= 8'h20;
            end
        end else begin
            text_wr_en <= 1'b0;
            ack_pulse <= 1'b0;

            if (rx_valid) begin
                if ((rx_data == 8'h0D) || (rx_data == 8'h0A)) begin
                    utf8_need <= 2'd0;
                    gbk_pending <= 1'b0;
                    last_was_cr <= (rx_data == 8'h0D);
                    if ((rx_data == 8'h0A) && last_was_cr) begin
                        last_was_cr <= 1'b0;
                    end else begin
                        text_byte(8'h0A);
                    end
                end else begin
                    last_was_cr <= 1'b0;
                    if (gbk_pending) begin
                        gbk_pending <= 1'b0;
                        if (gbk_glyph_id({gbk_lead, rx_data}) != 8'd0)
                            text_byte(gbk_glyph_id({gbk_lead, rx_data}));
                        else
                            text_byte(8'h3F);
                    end else if (utf8_need != 2'd0) begin
                        if ((utf8_need == 2'd1) &&
                            (gbk_glyph_id({utf8_first_byte, rx_data}) != 8'd0)) begin
                            utf8_need <= 2'd0;
                            text_byte(gbk_glyph_id({utf8_first_byte, rx_data}));
                        end else if ((rx_data[7:6] == 2'b10) &&
                                     (utf8_need == 2'd1)) begin
                            utf8_need <= 2'd0;
                            if ({utf8_codepoint[14:0], rx_data[5:0]} <= 21'h0FFFF)
                                text_byte(utf8_glyph_id({utf8_codepoint[9:0], rx_data[5:0]}));
                            else
                                text_byte(8'h3F);
                        end else if ((rx_data[7:6] == 2'b10) &&
                                     (utf8_need > 2'd1)) begin
                            utf8_codepoint <= {utf8_codepoint[14:0], rx_data[5:0]};
                            utf8_need <= utf8_need - 2'd1;
                        end else begin
                            utf8_need <= 2'd0;
                            text_byte(8'h3F);
                        end
                    end else if ((rx_data >= 8'h81) && (rx_data <= 8'hBF)) begin
                        gbk_lead <= rx_data;
                        gbk_pending <= 1'b1;
                    end else if ((rx_data >= 8'h20) && (rx_data <= 8'h7E)) begin
                        text_byte(rx_data);
                    end else if ((rx_data >= 8'hC2) && (rx_data <= 8'hDF)) begin
                        utf8_first_byte <= rx_data;
                        utf8_codepoint <= {16'd0, rx_data[4:0]};
                        utf8_need <= 2'd1;
                    end else if ((rx_data >= 8'hE0) && (rx_data <= 8'hEF)) begin
                        utf8_first_byte <= rx_data;
                        utf8_codepoint <= {17'd0, rx_data[3:0]};
                        utf8_need <= 2'd2;
                    end else if ((rx_data >= 8'hF0) && (rx_data <= 8'hF4)) begin
                        utf8_first_byte <= rx_data;
                        utf8_codepoint <= {17'd0, rx_data[3:0]};
                        utf8_need <= 2'd3;
                    end else if (rx_data[7]) begin
                        text_byte(8'h3F);
                    end
                end
            end

            case (st)
                S_SETTLE: begin
                    if (redraw) begin
                        g_ch <= cell_char[7] ? 8'h20 : ascii_upper(cell_char);
                        g_row <= rd_r[3:1];
                        st <= S_WRITE;
                    end
                end
                S_WRITE: begin
                    text_wr_en <= 1'b1;
                    text_wr_addr <= byte_off;
                    text_wr_data <= rd_half ? glyph_pixels[7:0] : glyph_pixels[15:8];
                    st <= S_NEXT;
                end
                S_NEXT: begin
                    if (!rd_half) begin
                        rd_half <= 1'b1;
                        st <= S_SETTLE;
                    end else begin
                        rd_half <= 1'b0;
                        if (rd_r == 4'd15) begin
                            rd_r <= 4'd0;
                            if (rd_col + 5'd1 >= COLS) begin
                                rd_col <= 5'd0;
                                if (rd_line == 2'd0) begin
                                    rd_line <= 2'd1;
                                    if (!rx_valid)
                                        text_valid <= (len0 != 5'd0) || (len1 != 5'd0);
                                end else begin
                                    rd_line <= 2'd0;
                                    if (!rx_valid)
                                        redraw <= 1'b0;
                                end
                            end else begin
                                rd_col <= rd_col + 5'd1;
                            end
                        end else begin
                            rd_r <= rd_r + 4'd1;
                        end
                        st <= S_SETTLE;
                    end
                end
                default: st <= S_SETTLE;
            endcase
        end
    end
endmodule
