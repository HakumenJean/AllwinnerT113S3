# Allwinner T113S3板级支持包

## 1. 简介

T113S3 是由全志公司推出的ARM A7内核的SOC
包括如下硬件特性：

| 硬件 | 描述 |
| -- | -- |
|芯片型号| tina系列 |
|CPU| Cortex-A7 |
|主频| 1200MHz |
|片内DDR | 128MB |
|板载SPI Nor Flash | 8/16MB|

## 2. 编译说明

| 环境 | 说明 |
| --- | --- |
|PC操作系统|Linux/MacOS|
|编译器|arm-none-eabi-gcc version 10.2.1 (release)|
|构建工具|scons|
1) 下载源码

```
    git clone https://github.com/HakumenJean/AllwinnerT113S3.git
```
2) 配置工程并准备env
```
    scons --menuconfig
    source ~/.env/env.sh
    pkgs --upgrade
    
```
3) 安装下载工具
```
    xfel
```
4) 编译
```
    scons
```
如果编译正确无误，会产生rtthread.elf、rtthread.bin文件。其中rtthread.bin需要烧写到设备中进行运行。

## 3. 执行
当正确编译产生出rtthread.bin映像文件后可以使用下面的方式在设备的DDR中运行。

```
将设备进入FEL模式
1.短接flash 1、4脚(当flash中无可引导代码时无需此步骤)
2.连接USB
3.松开短接的引脚
4.输入下列指令
```

```
    xfel ddr t113-s3
    xfel write 0x40000000 .\rtthread.bin
    xfel exec 0x40000000
```

### 3.1 运行结果

如果编译 & 烧写无误，会在串口3上看到RT-Thread的启动logo信息：

```
 \ | /
- RT -     Thread Operating System
 / | \     5.1.0 build Jan  6 2026 10:02:18
 2006 - 2024 Copyright by RT-Thread team
[D/main] CPU CLK: 1200MHz
[D/main]m sh />Periph CLK: (1X) = 600MHz, (2X) = 1200MHz, (800M) = 800MHz
[D/main] DDR CLK: 1584MHz
[D/main] AHB CLK: 200MHz
[D/main] APB CLK: (APB0) = 100MHz, (APB1) = 24MHz
msh />
```


## 4. 驱动支持情况及计划

| 驱动 | 支持情况  |  备注  |
| ------ | :----:  | :------:  |
| SMP | 支持 | / |
| USB | 支持 | CherryUSB 1.5.2 |
| UART | 支持 | UART0/1/2/3/4/5 |
| GPIO | 支持 | / |
| CLOCK | 支持 | / |
| MMU | 支持 | / |
| CE | 支持 | / |
| ETH | 支持 | / |
| G2D | 支持 | / |
| GPADC | 支持 | / |
| RGB LCD | 支持 | / |
| MIPI LCD | 支持 | / |
| TWI | 支持 | / |
| PWM | 支持 | / |
| RTC | 支持 | / |
| SMHC | 支持 | / |
| SPI | 支持 | / |
| TIMER | 支持 | / |


### 4.1 IO在板级支持包中的映射情况

| IO号 | 板级包中的定义 |
| -- | -- |
| PB7 | USART3 RX |
| PB6 | USART3 TX |
| PC2 | SPI0 CLK |
| PC3 | SPI0 CS |
| PC4 | SPI0 MOSI |
| PC5 | SPI0 MISO |
| PF2 | SMHC0 CLK |
| PF3 | SMHC0 CMD |
| PF1 | SMHC0 D0 |
| PF0 | SMHC0 D1 |
| PF5 | SMHC0 D2 |
| PF6 | SMHC0 D3 |


## 5. 联系人信息

维护人:
[HakumenJean][4] < [1696015776@qq.com][5] >


  [1]: https://www.rt-thread.org/page/download.html
  [4]: https://github.com/HakumenJean
