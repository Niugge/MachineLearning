log.zip存放静态采样的IMU数据

csvload.m可以导入log.zip中的所有csv文件

运行Calibate_tool.m文件，每个轴选最高两个点，最低两个点；总共12个点

自动运行，计算加速度计的偏移量和scale

计算陀螺仪的静态偏移量