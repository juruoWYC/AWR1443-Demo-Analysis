########################################
AWR1443 mmWave Demo 源代码解析与中文注释
########################################

1.主要文件说明

main.c
mmWave Demo主程序

mmw.h
mmWave Demo主头文件，包含所有需要的头文件，定义重要功能函数、结构体

data_path.c
数据处理路径主程序，包含对ADC数据的收集，1D-FFT及2D-FFT处理

config_edma_util.c
实现配置EDMA功能的API函数

config_hwa_util.c
实现配置FFT、CFAR硬件加速器的API函数

mmw_cli.c
实现对User Port中输入的命令行参数的处理

post_processing.c
调用硬件加速器，对FFT输出的数据进行CFAR检测，方位角、仰角估测

sensor_mgmt.c
实现对雷达前端的管理，可以通过CLI启动或停止雷达前端

rx_ch_bias_measure.c
实现天线制造误差校准（1.1版本新功能）

mmw.cfg
SYS/BIOS实时内核的RTSC软件组件配置文件