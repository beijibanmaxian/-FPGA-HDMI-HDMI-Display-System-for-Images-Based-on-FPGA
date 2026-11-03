# -FPGA-HDMI-HDMI-Display-System-for-Images-Based-on-FPGA
本项目基于安路HX4S20C开发板（EG4S20国产FPGA），面向数字广告牌、应急信息发布场景，设计纯硬件HDMI多媒体播放系统。全部逻辑均在FPGA内部Verilog实现，不使用额外处理器。系统完成TF卡BMP图片解析、SDRAM帧缓存管理、HDMI音视频输出，支持按键手动切图与图片自动轮播。在此基础上实现图层字幕叠加、图片转场特效、OSD状态信息叠加扩展功能，充分发挥FPGA并行处理、低延迟、音画同步的优势，系统运行稳定，具备完整实物演示能力。
