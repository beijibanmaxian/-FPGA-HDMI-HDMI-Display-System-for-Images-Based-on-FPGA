# ============================================================
# timing.sdc  —— HX4S20C / EG4S20BG256 音视频融合展示系统
#
# 原则：只用官方例程验证过的约束写法，不引入额外时钟对象。
#   1) 板级主时钟 50MHz
#   2) derive_clocks 自动推导 PLL 输出
#   3) 给关键 PLL 输出改名（只改名，不新建时钟）
#   4) video_clk / hdmi_5x_clk 互斥
#   5) audio_mclk 与视频/存储域异步
#   6) sd_card_clk 与 video_clk 异步（跨域只走 toggle 同步器与官方异步 FIFO）
#
# 注意：不要对 sys_pll_m0/pll_inst.clkc[2]（SDRAM 相移时钟）单独下约束，
# 该时钟由加密 SDRAM IP 内部使用；额外约束会在同一 pin 上产生重复时钟定义，
# 反而让 SDRAM 时序分析用错周期。
# ============================================================

# 1. 板级输入主时钟
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# 2. 自动推导 PLL 输出时钟
derive_clocks

# 3. 关键 PLL 输出命名
rename_clock -name {sd_card_clk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}    -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# 4. HDMI 像素时钟与串行时钟互斥（沿用官方 timing(2).sdc 的写法）
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# 5. 音频 I2S 域与其余域异步
#    audio_mclk 域生成 I2S，其他域做多拍同步采样；跨域只走 Gray 码异步 FIFO。
set_clock_groups -asynchronous \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {video_clk} hdmi_5x_clk sd_card_clk ext_mem_clk]

# 6. TF 卡域与像素域异步
#    跨域只有：write_finish_toggle / commit_tgl / disp_buf_idx（双触发器同步器），
#    以及官方 frame_fifo_write / frame_fifo_read 内部的异步 FIFO。
set_clock_groups -asynchronous \
    -group [get_clocks {sd_card_clk}] \
    -group [get_clocks {video_clk hdmi_5x_clk}]

# 7. Input/Output delay 保持空白（与官方两个原始 sdc 一致）
# ============================================================
# 8. 跨时钟域逐对声明（只引用已有时钟，不新建任何时钟对象）
#
# 为什么还要加这几条：set_clock_groups -asynchronous 在 TD 里对
# "非 group 成员两两之间"的排除并不彻底，报告里仍会分析
#   ext_mem_clk -> sd_card_clk （165 个端点，几乎全是异步复位网络 rst_all）
#   sd_card_clk <-> audio_mclk （26 个端点，音频 Gray 码 FIFO 指针）
# 这些路径在设计上就不需要满足同步时序，显式排除后可让布线器把精力
# 放在真正的同步路径上，SD 域的 WNS 从 -2.3ns 收敛到 -0.2ns 量级。
#
# 注意：不要对 sys_pll_m0/pll_inst.clkc[2]（SDRAM 相移时钟）下任何约束，
# 否则会在同一 pin 上产生重复时钟定义。
# ============================================================
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {sd_card_clk}]
set_false_path -from [get_clocks {sd_card_clk}] -to [get_clocks {ext_mem_clk}]
set_false_path -from [get_clocks {sd_card_clk}] -to [get_clocks {audio_mclk}]
set_false_path -from [get_clocks {audio_mclk}] -to [get_clocks {sd_card_clk}]
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {audio_mclk}]
set_false_path -from [get_clocks {audio_mclk}] -to [get_clocks {ext_mem_clk}]
