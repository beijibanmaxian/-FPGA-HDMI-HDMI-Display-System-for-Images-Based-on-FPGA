module rtc_clock #(
    parameter integer CLK_FREQ_HZ = 25_000_000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        set_valid,
    input  wire [63:0] set_payload,
    output reg         time_valid,
    output wire [15:0] year_bcd,
    output reg  [7:0]  month_bcd,
    output reg  [7:0]  day_bcd,
    output reg  [7:0]  hour_bcd,
    output reg  [7:0]  minute_bcd,
    output reg  [7:0]  second_bcd,
    output reg  [2:0]  weekday
);
    reg [7:0] year_low_bcd;
    reg [31:0] tick_count;

    assign year_bcd = {4'd2, 4'd0, year_low_bcd};

    function [7:0] bcd2_to_bin;
        input [7:0] value;
        begin
            bcd2_to_bin = (value[7:4] * 8'd10) + value[3:0];
        end
    endfunction

    function [7:0] increment_bcd2;
        input [7:0] value;
        begin
            if (value[3:0] == 4'd9)
                increment_bcd2 = {value[7:4] + 4'd1, 4'd0};
            else
                increment_bcd2 = {value[7:4], value[3:0] + 4'd1};
        end
    endfunction

    function valid_bcd2;
        input [7:0] value;
        begin
            valid_bcd2 = (value[7:4] <= 4'd9) && (value[3:0] <= 4'd9);
        end
    endfunction

    wire [7:0] set_year_low = set_payload[55:48];
    wire [7:0] set_month = set_payload[47:40];
    wire [7:0] set_day = set_payload[39:32];
    wire [7:0] set_hour = set_payload[31:24];
    wire [7:0] set_minute = set_payload[23:16];
    wire [7:0] set_second = set_payload[15:8];
    wire [7:0] set_month_bin = bcd2_to_bin(set_month);
    wire [7:0] set_day_bin = bcd2_to_bin(set_day);
    wire [7:0] set_year_low_bin = bcd2_to_bin(set_year_low);
    wire set_leap_year = ((set_year_low_bin % 8'd4) == 8'd0);

    reg [7:0] month_bin;
    reg [7:0] day_bin;
    reg [7:0] days_this_month;
    always @* begin
        month_bin = bcd2_to_bin(month_bcd);
        day_bin = bcd2_to_bin(day_bcd);
        case (month_bin)
            8'd1, 8'd3, 8'd5, 8'd7, 8'd8, 8'd10, 8'd12:
                days_this_month = 8'd31;
            8'd4, 8'd6, 8'd9, 8'd11:
                days_this_month = 8'd30;
            8'd2:
                days_this_month = (bcd2_to_bin(year_low_bcd) % 8'd4 == 0)
                    ? 8'd29 : 8'd28;
            default:
                days_this_month = 8'd0;
        endcase
    end

    reg [7:0] set_days_in_month;
    always @* begin
        case (set_month_bin)
            8'd1, 8'd3, 8'd5, 8'd7, 8'd8, 8'd10, 8'd12:
                set_days_in_month = 8'd31;
            8'd4, 8'd6, 8'd9, 8'd11:
                set_days_in_month = 8'd30;
            8'd2:
                set_days_in_month = set_leap_year ? 8'd29 : 8'd28;
            default:
                set_days_in_month = 8'd0;
        endcase
    end

    wire set_payload_valid =
        (set_payload[63:60] == 4'd2) &&
        (set_payload[59:56] == 4'd0) &&
        valid_bcd2(set_year_low) &&
        valid_bcd2(set_month) && (set_month_bin >= 8'd1) &&
        (set_month_bin <= 8'd12) &&
        valid_bcd2(set_day) && (set_day_bin >= 8'd1) &&
        (set_day_bin <= set_days_in_month) &&
        valid_bcd2(set_hour) && (bcd2_to_bin(set_hour) <= 8'd23) &&
        valid_bcd2(set_minute) && (bcd2_to_bin(set_minute) <= 8'd59) &&
        valid_bcd2(set_second) && (bcd2_to_bin(set_second) <= 8'd59) &&
        (set_payload[2:0] <= 3'd6);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            time_valid <= 1'b0;
            year_low_bcd <= 8'd0;
            month_bcd <= 8'd0;
            day_bcd <= 8'd0;
            hour_bcd <= 8'd0;
            minute_bcd <= 8'd0;
            second_bcd <= 8'd0;
            weekday <= 3'd0;
            tick_count <= 32'd0;
        end else if (set_valid) begin
            tick_count <= 32'd0;
            if (set_payload_valid) begin
                time_valid <= 1'b1;
                year_low_bcd <= set_year_low;
                month_bcd <= set_month;
                day_bcd <= set_day;
                hour_bcd <= set_hour;
                minute_bcd <= set_minute;
                second_bcd <= set_second;
                weekday <= set_payload[2:0];
            end
        end else if (time_valid) begin
            if (tick_count >= CLK_FREQ_HZ - 1) begin
                tick_count <= 32'd0;
                if (second_bcd != 8'h59)
                    second_bcd <= increment_bcd2(second_bcd);
                else begin
                    second_bcd <= 8'd0;
                    if (minute_bcd != 8'h59)
                        minute_bcd <= increment_bcd2(minute_bcd);
                    else begin
                        minute_bcd <= 8'd0;
                        if (hour_bcd != 8'h23)
                            hour_bcd <= increment_bcd2(hour_bcd);
                        else begin
                            hour_bcd <= 8'd0;
                            weekday <= (weekday == 3'd6) ? 3'd0 : weekday + 3'd1;
                            if (day_bin >= days_this_month) begin
                                day_bcd <= 8'h01;
                                if (month_bin == 8'd12) begin
                                    month_bcd <= 8'h01;
                                    year_low_bcd <=
                                        (year_low_bcd == 8'h99) ? 8'h00 :
                                        increment_bcd2(year_low_bcd);
                                end else begin
                                    month_bcd <= increment_bcd2(month_bcd);
                                end
                            end else begin
                                day_bcd <= increment_bcd2(day_bcd);
                            end
                        end
                    end
                end
            end else begin
                tick_count <= tick_count + 32'd1;
            end
        end else begin
            tick_count <= 32'd0;
        end
    end
endmodule
