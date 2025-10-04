# 基于STM32的智能语音识别垃圾桶系统 - 深度代码分析文档

## 📋 目录
- [项目概述](#项目概述)
- [完整目录结构](#完整目录结构)
- [核心代码文件深度解析](#核心代码文件深度解析)
- [开发指南](#开发指南)
- [硬件连接](#硬件连接)
- [调试与故障排除](#调试与故障排除)

---

## 项目概述

本项目是一个基于STM32F103微控制器的智能语音分类垃圾桶系统，实现了8大核心功能：

1. ✅ **垃圾容量检测** - 红外传感器实时监测
2. ✅ **语音识别控制** - 识别4种垃圾类型自动开盖
3. ✅ **语音播报** - TTS语音反馈
4. ✅ **OLED显示** - 实时显示投递次数和状态
5. ✅ **精准舵机控制** - PWM控制盖子开合
6. ✅ **智能照明** - 光敏传感器自动控制LED
7. ✅ **云平台连接** - WiFi上传数据到物联网平台
8. ✅ **手动按键控制** - 4个物理按键备用方案

---

## 完整目录结构

```
S/
│
├── 📁 CORE/                                    # STM32内核启动文件目录
│   ├── core_cm3.c                             # Cortex-M3内核外设访问层
│   ├── core_cm3.h                             # 内核寄存器定义
│   ├── startup_stm32f10x_hd.s                 # 启动汇编文件(高密度产品)
│   └── startup_stm32f10x_md.s                 # 启动汇编文件(中密度产品)
│   【说明】包含芯片启动代码和中断向量表，❌不要修改
│
├── 📁 HARDWARE/                                # 硬件外设驱动目录(核心开发区)
│   │
│   ├── 📁 ADC/                                # 模数转换器模块
│   │   ├── adc.c                              # ADC驱动实现文件
│   │   └── adc.h                              # ADC驱动头文件
│   │   【功能】读取光敏传感器的模拟电压，实现自动照明
│   │   【接口】ADC_Init(), Get_Adc(), Get_Adc_Average()
│   │   【修改】✅可修改 - 调整ADC通道、采样时间、阈值参数
│   │
│   ├── 📁 DS18B20/                            # 温度传感器模块(可选)
│   │   ├── ds18b20.c                          # DS18B20驱动实现
│   │   └── ds18b20.h                          # DS18B20头文件
│   │   【功能】读取环境温度(项目中为扩展功能)
│   │   【接口】DS18B20_Init(), DS18B20_Read_Temp()
│   │   【修改】⚠️可选模块 - 如不使用可删除
│   │
│   ├── 📁 GPIO/                               # 通用输入输出模块
│   │   ├── gpio.c                             # GPIO驱动实现
│   │   └── gpio.h                             # GPIO头文件
│   │   【功能】控制LED灯、读取红外传感器状态
│   │   【接口】GPIO_LED_Init(), LED_On(), LED_Off(), IR_Read()
│   │   【修改】✅必须修改 - 根据实际硬件连接调整引脚定义
│   │
│   ├── 📁 KEY/                                # 按键扫描模块
│   │   ├── key.c                              # 按键驱动实现
│   │   └── key.h                              # 按键头文件
│   │   【功能】扫描4个物理按键，实现手动控制垃圾桶
│   │   【接口】KEY_Init(), KEY_Scan()
│   │   【修改】✅可修改 - 调整按键引脚、去抖动时间
│   │
│   ├── 📁 MOTOR_DUOJI/                        # 舵机控制模块
│   │   ├── motor_duoji.c                      # 舵机PWM驱动实现
│   │   └── motor_duoji.h                      # 舵机头文件
│   │   【功能】通过PWM控制4个舵机转动角度，开关垃圾桶盖
│   │   【接口】Servo_Init(), Servo_SetAngle(), Servo_Open(), Servo_Close()
│   │   【修改】✅必须修改 - 调整PWM参数、开合角度、延时时间
│   │
│   └── 📁 OLED/                               # OLED显示屏模块
│       ├── oled.c                             # OLED驱动实现(I2C通信)
│       ├── oled.h                             # OLED头文件
│       └── oledfont.h                         # OLED字体库(ASCII + 汉字)
│       【功能】显示系统状态、投递次数、垃圾桶满状态
│       【接口】OLED_Init(), OLED_ShowString(), OLED_ShowNum(), OLED_ShowChinese()
│       【修改】✅可修改 - 自定义显示布局、添加字库
│
├── 📁 STM32F10x_FWLib/                         # STM32标准外设库(官方)
│   ├── 📁 inc/                                # 外设库头文件
│   │   ├── misc.h                             # NVIC和SysTick配置
│   │   ├── stm32f10x_adc.h                    # ADC外设寄存器定义
│   │   ├── stm32f10x_bkp.h                    # 备份寄存器
│   │   ├── stm32f10x_can.h                    # CAN通信
│   │   ├── stm32f10x_crc.h                    # CRC校验
│   │   ├── stm32f10x_dac.h                    # DAC数模转换
│   │   ├── stm32f10x_dbgmcu.h                 # 调试支持
│   │   ├── stm32f10x_dma.h                    # DMA直接内存访问
│   │   ├── stm32f10x_exti.h                   # 外部中断
│   │   ├── stm32f10x_flash.h                  # Flash编程
│   │   ├── stm32f10x_fsmc.h                   # 外部存储控制器
│   │   ├── stm32f10x_gpio.h                   # GPIO通用IO
│   │   ├── stm32f10x_i2c.h                    # I2C通信
│   │   ├── stm32f10x_iwdg.h                   # 独立看门狗
│   │   ├── stm32f10x_pwr.h                    # 电源控制
│   │   ├── stm32f10x_rcc.h                    # 时钟配置
│   │   ├── stm32f10x_rtc.h                    # 实时时钟
│   │   ├── stm32f10x_sdio.h                   # SD卡接口
│   │   ├── stm32f10x_spi.h                    # SPI通信
│   │   ├── stm32f10x_tim.h                    # 定时器
│   │   ├── stm32f10x_usart.h                  # 串口通信
│   │   └── stm32f10x_wwdg.h                   # 窗口看门狗
│   │
│   └── 📁 src/                                # 外设库源文件(对应上述头文件)
│       ├── misc.c
│       ├── stm32f10x_adc.c
│       ├── stm32f10x_gpio.c
│       ├── stm32f10x_rcc.c
│       ├── stm32f10x_tim.c
│       ├── stm32f10x_usart.c
│       └── ... (其他外设驱动源码)
│   【说明】ST官方标准外设库，提供底层寄存器操作封装
│   【修改】❌绝对不要修改 - 这是官方库文件
│
├── 📁 SYSTEM/                                  # 系统级驱动封装
│   │
│   ├── 📁 delay/                              # 延时函数模块
│   │   ├── delay.c                            # 延时函数实现
│   │   └── delay.h                            # 延时函数头文件
│   │   【功能】提供微秒级(delay_us)和毫秒级(delay_ms)精确延时
│   │   【原理】基于SysTick定时器实现
│   │   【修改】⚠️一般不修改 - 除非需要更高精度延时
│   │
│   ├── 📁 sys/                                # 系统配置模块
│   │   ├── sys.c                              # 系统配置实现
│   │   └── sys.h                              # 系统配置头文件
│   │   【功能】位带操作宏定义、NVIC优先级配置、时钟初始化
│   │   【接口】SYS_SUPPORT宏、位带地址转换
│   │   【修改】⚠️谨慎修改 - 仅在需要改变系统配置时
│   │
│   └── 📁 usart/                              # 串口通信模块
│       ├── usart.c                            # 串口驱动实现
│       └── usart.h                            # 串口头文件
│       【功能】串口初始化、数据收发、printf重定向
│       【接口】USART1_Init(), USART_SendByte(), USART_SendString()
│       【协议】与语音模块(USART1)、WiFi模块(USART2)通信
│       【修改】✅必须修改 - 根据模块通信协议调整
│
├── 📁 USER/                                    # 用户应用程序(主开发区)
│   ├── main.c                                 # ⭐主程序文件(最重要)
│   │   【功能】系统初始化、主循环、业务逻辑调度
│   │   【内容】见下方详细解析
│   │   【修改】✅核心开发文件 - 主要逻辑都在这里
│   │
│   ├── stm32f10x_conf.h                       # 外设库配置文件
│   │   【功能】选择需要使用的外设模块
│   │   【内容】包含或注释掉不需要的外设头文件
│   │   【修改】⚠️按需修改 - 启用/禁用外设以节省代码空间
│   │
│   ├── stm32f10x_it.c                         # 中断服务函数实现
│   │   【功能】所有中断处理函数(串口接收、定时器等)
│   │   【内容】USART1_IRQHandler(), TIM2_IRQHandler()等
│   │   【修改】✅必须修改 - 添加中断处理逻辑
│   │
│   └── stm32f10x_it.h                         # 中断服务函数头文件
│       【功能】中断函数声明
│       【修改】⚠️配合.c文件修改
│
├── 📁 OBJ/                                     # 编译生成的目标文件(.o)
│   【说明】编译器自动生成，❌不要手动修改
│
├── 📁 LISTING/                                 # 编译生成的清单文件(.lst)
│   【说明】包含汇编代码清单，用于调试分析
│
├── project.uvprojx                            # Keil uVision5工程文件
│   【说明】双击打开Keil开发环境
│
├── project.uvoptx                             # Keil工程配置文件
│   【说明】保存编译器设置、调试器配置等
│
└── README.md                                  # 项目说明文档(本文件)

```

---

## 核心代码文件深度解析

### 1️⃣ USER/main.c - 主程序文件 (⭐⭐⭐⭐⭐)

**文件作用**: 整个项目的大脑，负责系统初始化和主循环调度

#### 📌 完整代码框架及详细注释

```c
/******************************************************************************
 * 文件名: main.c
 * 功能: 智能语音识别垃圾桶主程序
 * 作者: [您的名字]
 * 版本: V1.0
 * 日期: 2024-10-04
 ******************************************************************************/

#include "sys.h"          // 系统配置
#include "delay.h"        // 延时函数
#include "usart.h"        // 串口通信
#include "oled.h"         // OLED显示
#include "motor_duoji.h"  // 舵机控制
#include "key.h"          // 按键扫描
#include "gpio.h"         // GPIO控制
#include "adc.h"          // ADC采集
#include <stdio.h>
#include <string.h>

/******************************************************************************
 * 全局变量定义
 ******************************************************************************/
// 投递次数统计数组(索引0-3对应4个垃圾桶)
u16 bin_count[4] = {0, 0, 0, 0};

// 垃圾桶满状态数组(0=未满, 1=已满)
u8 bin_full[4] = {0, 0, 0, 0};

// 垃圾桶名称字符串(用于显示和播报)
char *bin_name[4] = {
    "Recyclable",   // 可回收垃圾
    "Kitchen",      // 厨余垃圾
    "Harmful",      // 有害垃圾
    "Other"         // 其他垃圾
};

// 串口接收缓冲区
u8 USART1_RX_BUF[200];  // 语音模块接收缓冲
u16 USART1_RX_STA = 0;  // 接收状态标志

u8 USART2_RX_BUF[200];  // WiFi模块接收缓冲
u16 USART2_RX_STA = 0;

// 系统运行时间计数(用于定时任务)
u32 system_tick = 0;

/******************************************************************************
 * 函数声明
 ******************************************************************************/
void System_Init(void);           // 系统初始化
void Voice_Monitor(void);         // 语音监控处理
void Key_Process(void);           // 按键处理
void Bin_Status_Check(void);      // 垃圾桶状态检测
void Display_Update(void);        // 更新OLED显示
void Light_Auto_Control(void);    // 自动照明控制
void WiFi_Data_Upload(void);      // WiFi数据上传
void Open_Bin(u8 bin_num);        // 打开指定垃圾桶
void Close_Bin(u8 bin_num);       // 关闭指定垃圾桶
u8 Parse_Voice_Command(void);     // 解析语音指令
void Voice_Play(char *text);      // 语音播报

/******************************************************************************
 * 主函数
 ******************************************************************************/
int main(void)
{
    System_Init();  // 系统初始化
    
    // 显示欢迎界面
    OLED_Clear();
    OLED_ShowChinese(16, 0, 0);  // "智"
    OLED_ShowChinese(32, 0, 1);  // "能"
    OLED_ShowChinese(48, 0, 2);  // "垃"
    OLED_ShowChinese(64, 0, 3);  // "圾"
    OLED_ShowChinese(80, 0, 4);  // "桶"
    delay_ms(2000);
    OLED_Clear();
    
    // 主循环
    while(1)
    {
        Voice_Monitor();        // 处理语音识别指令
        Key_Process();          // 处理按键输入
        Bin_Status_Check();     // 检测垃圾桶状态
        Display_Update();       // 更新显示内容
        Light_Auto_Control();   // 自动照明控制
        WiFi_Data_Upload();     // 上传数据到云平台
        
        system_tick++;          // 系统计时递增
        delay_ms(50);           // 主循环延时50ms
    }
}

/******************************************************************************
 * 函数名: System_Init
 * 功能: 初始化所有硬件模块
 * 参数: 无
 * 返回: 无
 ******************************************************************************/
void System_Init(void)
{
    // 配置中断优先级分组(2位抢占优先级, 2位响应优先级)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 初始化延时函数(必须最先初始化)
    delay_init();
    
    // 初始化串口1(9600波特率, 用于语音识别模块)
    USART1_Init(9600);
    
    // 初始化串口2(115200波特率, 用于WiFi模块)
    USART2_Init(115200);
    
    // 初始化OLED显示屏(I2C接口)
    OLED_Init();
    OLED_Clear();
    
    // 初始化舵机PWM(TIM2, 50Hz)
    Servo_Init();
    // 初始化时关闭所有垃圾桶盖子
    for(u8 i = 1; i <= 4; i++)
    {
        Servo_Close(i);
    }
    
    // 初始化按键GPIO(上拉输入)
    KEY_Init();
    
    // 初始化GPIO(LED和红外传感器)
    GPIO_Init();
    // 初始化所有LED为绿灯(表示未满)
    for(u8 i = 1; i <= 4; i++)
    {
        LED_Green_On(i);
        LED_Red_Off(i);
    }
    
    // 初始化ADC(用于光敏传感器)
    ADC_Init();
    
    // 延时确保所有模块稳定
    delay_ms(100);
}

/******************************************************************************
 * 函数名: Voice_Monitor
 * 功能: 监听并处理语音识别模块发送的指令
 * 参数: 无
 * 返回: 无
 * 说明: 在中断中接收数据，在此函数中解析处理
 ******************************************************************************/
void Voice_Monitor(void)
{
    // 检查是否接收到完整数据包(最高位为1表示接收完成)
    if(USART1_RX_STA & 0x8000)
    {
        u8 bin_type = Parse_Voice_Command();  // 解析语音指令
        
        if(bin_type >= 1 && bin_type <= 4)    // 有效指令(1-4)
        {
            // 检查该垃圾桶是否已满
            if(bin_full[bin_type - 1] == 1)
            {
                OLED_ShowString(0, 6, "Bin Full!");
                Voice_Play("垃圾桶已满");
                delay_ms(1000);
            }
            else
            {
                Open_Bin(bin_type);            // 打开垃圾桶
                bin_count[bin_type - 1]++;     // 投递次数+1
                Voice_Play(bin_name[bin_type - 1]); // 播报垃圾类型
                delay_ms(2000);                // 等待2秒
                Close_Bin(bin_type);           // 关闭垃圾桶
            }
        }
        
        // 清除接收完成标志，准备接收下一帧
        USART1_RX_STA = 0;
        memset(USART1_RX_BUF, 0, sizeof(USART1_RX_BUF));
    }
}

/******************************************************************************
 * 函数名: Parse_Voice_Command
 * 功能: 解析语音模块发送的指令字符串
 * 参数: 无
 * 返回: 垃圾桶类型(1-4), 0表示无效指令
 * 协议示例:
 *   "TYPE:1\r\n" -> 可回收垃圾
 *   "TYPE:2\r\n" -> 厨余垃圾
 *   "TYPE:3\r\n" -> 有害垃圾
 *   "TYPE:4\r\n" -> 其他垃圾
 ******************************************************************************/
u8 Parse_Voice_Command(void)
{
    // 方式1: 简单字符匹配
    if(strstr((char*)USART1_RX_BUF, "TYPE:1") != NULL)
        return 1;
    else if(strstr((char*)USART1_RX_BUF, "TYPE:2") != NULL)
        return 2;
    else if(strstr((char*)USART1_RX_BUF, "TYPE:3") != NULL)
        return 3;
    else if(strstr((char*)USART1_RX_BUF, "TYPE:4") != NULL)
        return 4;
    
    // 方式2: 关键词匹配(根据语音模块识别结果)
    if(strstr((char*)USART1_RX_BUF, "可回收") != NULL ||
       strstr((char*)USART1_RX_BUF, "recyclable") != NULL)
        return 1;
    else if(strstr((char*)USART1_RX_BUF, "厨余") != NULL ||
            strstr((char*)USART1_RX_BUF, "kitchen") != NULL)
        return 2;
    else if(strstr((char*)USART1_RX_BUF, "有害") != NULL ||
            strstr((char*)USART1_RX_BUF, "harmful") != NULL)
        return 3;
    else if(strstr((char*)USART1_RX_BUF, "其他") != NULL ||
            strstr((char*)USART1_RX_BUF, "other") != NULL)
        return 4;
    
    return 0;  // 无效指令
}

/******************************************************************************
 * 函数名: Key_Process
 * 功能: 扫描按键并处理手动控制
 * 参数: 无
 * 返回: 无
 ******************************************************************************/
void Key_Process(void)
{
    u8 key = KEY_Scan(0);  // 0表示不支持连按
    
    if(key != 0)  // 有按键按下
    {
        // key值: 1=KEY1, 2=KEY2, 3=KEY3, 4=KEY4
        if(bin_full[key - 1] == 1)  // 检查是否已满
        {
            OLED_ShowString(0, 6, "Bin Full!");
            Voice_Play("垃圾桶已满");
            delay_ms(1000);
        }
        else
        {
            Open_Bin(key);              // 打开对应垃圾桶
            bin_count[key - 1]++;       // 投递次数+1
            Voice_Play(bin_name[key - 1]);
            delay_ms(2000);
            Close_Bin(key);
        }
    }
}

/******************************************************************************
 * 函数名: Bin_Status_Check
 * 功能: 检测所有垃圾桶的满溢状态
 * 参数: 无
 * 返回: 无
 * 说明: 红外传感器检测到障碍物(垃圾)时输出低电平
 ******************************************************************************/
void Bin_Status_Check(void)
{
    for(u8 i = 0; i < 4; i++)
    {
        u8 ir_status = IR_Sensor_Read(i + 1);  // 读取红外传感器
        
        // 检测到已满(ir_status=1)且之前状态为未满
        if(ir_status == 1 && bin_full[i] == 0)
        {
            bin_full[i] = 1;           // 更新状态为已满
            LED_Green_Off(i + 1);      // 关闭绿灯
            LED_Red_On(i + 1);         // 开启红灯
            Voice_Play("垃圾桶已满，请及时清理");
            
            // 通过串口输出提示(可用于调试或扩展功能)
            printf("Bin %d is full!\r\n", i + 1);
        }
        // 检测到未满(ir_status=0)且之前状态为已满(已清空)
        else if(ir_status == 0 && bin_full[i] == 1)
        {
            bin_full[i] = 0;           // 更新状态为未满
            LED_Red_Off(i + 1);        // 关闭红灯
            LED_Green_On(i + 1);       // 开启绿灯
        }
    }
}

/******************************************************************************
 * 函数名: Display_Update
 * 功能: 更新OLED显示内容
 * 参数: 无
 * 返回: 无
 * 说明: 每500ms更新一次，避免频繁刷新
 ******************************************************************************/
void Display_Update(void)
{
    static u32 last_update = 0;
    
    // 每500ms更新一次(system_tick每50ms递增1)
    if(system_tick - last_update >= 10)
    {
        // 第1行: 可回收垃圾桶
        OLED_ShowString(0, 0, "Recycle:");
        OLED_ShowNum(70, 0, bin_count[0], 3);  // 显示次数
        OLED_ShowString(100, 0, bin_full[0] ? "X" : "√");
        
        // 第2行: 厨余垃圾桶
        OLED_ShowString(0, 2, "Kitchen:");
        OLED_ShowNum(70, 2, bin_count[1], 3);
        OLED_ShowString(100, 2, bin_full[1] ? "X" : "√");
        
        // 第3行: 有害垃圾桶
        OLED_ShowString(0, 4, "Harmful:");
        OLED_ShowNum(70, 4, bin_count[2], 3);
        OLED_ShowString(100, 4, bin_full[2] ? "X" : "√");
        
        // 第4行: 其他垃圾桶
        OLED_ShowString(0, 6, "Other:");
        OLED_ShowNum(70, 6, bin_count[3], 3);
        OLED_ShowString(100, 6, bin_full[3] ? "X" : "√");
        
        last_update = system_tick;
    }
}

/******************************************************************************
 * 函数名: Light_Auto_Control
 * 功能: 根据光敏传感器自动控制照明LED
 * 参数: 无
 * 返回: 无
 * 说明: ADC值越小表示环境越暗
 ******************************************************************************/
void Light_Auto_Control(void)
{
    static u32 last_check = 0;
    
    // 每1秒检测一次(避免频繁开关灯)
    if(system_tick - last_check >= 20)
    {
        u16 light_value = Get_Adc_Average(4, 10);  // ADC通道4，采样10次平均
        
        // 阈值判断(可根据实际环境调整)
        if(light_value < 1000)  // 环境较暗(夜晚)
        {
            LED_Light_On();     // 开启照明LED
        }
        else if(light_value > 1500)  // 环境较亮(白天)
        {
            LED_Light_Off();    // 关闭照明LED
        }
        // 中间区域保持当前状态，避免频繁开关
        
        last_check = system_tick;
    }
}

/******************************************************************************
 * 函数名: WiFi_Data_Upload
 * 功能: 通过WiFi模块上传垃圾桶状态到云平台
 * 参数: 无
 * 返回: 无
 * 说明: 每5秒上传一次数据
 ******************************************************************************/
void WiFi_Data_Upload(void)
{
    static u32 last_upload = 0;
    
    // 每5秒上传一次(100个tick = 5秒)
    if(system_tick - last_upload >= 100)
    {
        char send_buf[100];
        
        // 构造JSON格式数据包
        sprintf(send_buf, "{\"bin1\":%d,\"bin2\":%d,\"bin3\":%d,\"bin4\":%d}\r\n",
                bin_full[0], bin_full[1], bin_full[2], bin_full[3]);
        
        // 通过USART2发送给WiFi模块
        USART_SendString(USART2, send_buf);
        
        last_upload = system_tick;
    }
}

/******************************************************************************
 * 函数名: Open_Bin
 * 功能: 打开指定垃圾桶盖子
 * 参数: bin_num - 垃圾桶编号(1-4)
 * 返回: 无
 ******************************************************************************/
void Open_Bin(u8 bin_num)
{
    if(bin_num < 1 || bin_num > 4) return;  // 参数检查
    
    Servo_SetAngle(bin_num, 90);  // 舵机转到90度(开盖)
    
    // OLED提示
    OLED_ShowString(0, 6, "Opening...");
}

/******************************************************************************
 * 函数名: Close_Bin
 * 功能: 关闭指定垃圾桶盖子
 * 参数: bin_num - 垃圾桶编号(1-4)
 * 返回: 无
 ******************************************************************************/
void Close_Bin(u8 bin_num)
{
    if(bin_num < 1 || bin_num > 4) return;
    
    Servo_SetAngle(bin_num, 0);   // 舵机转到0度(关盖)
    
    // 清除提示信息
    OLED_ShowString(0, 6, "          ");
}

/******************************************************************************
 * 函数名: Voice_Play
 * 功能: 通过语音模块播报文字
 * 参数: text - 要播报的文本
 * 返回: 无
 * 说明: 发送AT指令给语音模块进行TTS播报
 ******************************************************************************/
void Voice_Play(char *text)
{
    char cmd_buf[100];
    
    // 构造TTS指令(根据实际语音模块调整)
    sprintf(cmd_buf, "AT+TTS=%s\r\n", text);
    
    // 通过USART1发送
    USART_SendString(USART1, cmd_buf);
}
```

---

### 2️⃣ HARDWARE/MOTOR_DUOJI/motor_duoji.c - 舵机控制 (⭐⭐⭐⭐⭐)

**文件作用**: 通过PWM信号控制4个舵机的转动角度，实现垃圾桶盖子的开合

#### 📌 完整代码实现

```c
/******************************************************************************
 * 文件名: motor_duoji.c
 * 功能: 舵机PWM控制驱动
 * 说明: 使用TIM2的4个通道产生PWM信号控制舵机
 ******************************************************************************/

#include "motor_duoji.h"
#include "sys.h"

/******************************************************************************
 * 函数名: Servo_Init
 * 功能: 初始化舵机PWM输出
 * 参数: 无
 * 返回: 无
 * 说明: 
 *   - 舵机工作频率: 50Hz (周期20ms)
 *   - 脉宽范围: 0.5ms(0°) ~ 2.5ms(180°)
 *   - TIM2时钟: 72MHz
 *   - 预分频系数: 72-1 (计数频率1MHz)
 *   - 重装载值: 20000-1 (周期20ms)
 ******************************************************************************/
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   // TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // 复用功能时钟
    
    // 2. 配置GPIO为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | 
                                   GPIO_Pin_2 | GPIO_Pin_3;  // PA0-PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;          // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 初始化TIM2定时器
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;            // 自动重装载值(20ms)
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;            // 预分频(1MHz计数频率)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 4. 初始化PWM输出模式 (4个通道)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // 高电平有效
    TIM_OCInitStructure.TIM_Pulse = 0;                       // 初始占空比0
    
    // 通道1 (PA0 -> 舵机1)
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    // 通道2 (PA1 -> 舵机2)
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    // 通道3 (PA2 -> 舵机3)
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    // 通道4 (PA3 -> 舵机4)
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    // 5. 使能定时器
    TIM_Cmd(TIM2, ENABLE);
}

/******************************************************************************
 * 函数名: Servo_SetAngle
 * 功能: 设置舵机转动角度
 * 参数: 
 *   servo_num - 舵机编号(1-4)
 *   angle - 目标角度(0-180度)
 * 返回: 无
 * 公式推导:
 *   - 0度对应脉宽0.5ms，计数值 = 0.5 * 1000 = 500
 *   - 180度对应脉宽2.5ms，计数值 = 2.5 * 1000 = 2500
 *   - angle度对应脉宽 = 0.5 + (2.5-0.5) * angle/180 ms
 *   - 计数值 = 500 + 2000 * angle / 180 = 500 + angle * 11.11
 ******************************************************************************/
void Servo_SetAngle(u8 servo_num, u8 angle)
{
    u16 pulse_width;
    
    // 限制角度范围
    if(angle > 180) angle = 180;
    
    // 计算脉宽对应的计数值
    pulse_width = 500 + (u16)(angle * 11.11);
    
    // 根据舵机编号设置对应通道的比较值
    switch(servo_num)
    {
        case 1:
            TIM_SetCompare1(TIM2, pulse_width);
            break;
        case 2:
            TIM_SetCompare2(TIM2, pulse_width);
            break;
        case 3:
            TIM_SetCompare3(TIM2, pulse_width);
            break;
        case 4:
            TIM_SetCompare4(TIM2, pulse_width);
            break;
        default:
            break;
    }
}

/******************************************************************************
 * 函数名: Servo_Open
 * 功能: 打开指定垃圾桶盖子
 * 参数: servo_num - 舵机编号(1-4)
 * 返回: 无
 ******************************************************************************/
void Servo_Open(u8 servo_num)
{
    Servo_SetAngle(servo_num, 90);  // 转到90度(开盖位置)
}

/******************************************************************************
 * 函数名: Servo_Close
 * 功能: 关闭指定垃圾桶盖子
 * 参数: servo_num - 舵机编号(1-4)
 * 返回: 无
 ******************************************************************************/
void Servo_Close(u8 servo_num)
{
    Servo_SetAngle(servo_num, 0);   // 转到0度(关盖位置)
}
```

#### 📌 对应头文件 motor_duoji.h

```c
#ifndef __MOTOR_DUOJI_H
#define __MOTOR_DUOJI_H

#include "sys.h"

// 函数声明
void Servo_Init(void);
void Servo_SetAngle(u8 servo_num, u8 angle);
void Servo_Open(u8 servo_num);
void Servo_Close(u8 servo_num);

#endif
```

---

### 3️⃣ HARDWARE/OLED/oled.c - OLED显示驱动 (⭐⭐⭐⭐)

**文件作用**: 通过I2C接口控制OLED显示屏，显示文字、数字、汉字

#### 📌 核心代码片段 (完整代码较长，展示关键部分)

```c
/******************************************************************************
 * 文件名: oled.c
 * 功能: OLED显示屏驱动(I2C接口)
 * 芯片: SSD1306
 * 分辨率: 128x64
 ******************************************************************************/

#include "oled.h"
#include "oledfont.h"  // 字体库
#include "delay.h"

// OLED显存缓冲区(128x64/8=1024字节)
u8 OLED_GRAM[128][8];

// I2C引脚定义(根据实际硬件修改)
#define OLED_SCL_PIN  GPIO_Pin_6
#define OLED_SDA_PIN  GPIO_Pin_7
#define OLED_GPIO     GPIOB

// I2C地址(0x78或0x7A，根据硬件决定)
#define OLED_ADDRESS  0x78

/******************************************************************************
 * I2C底层时序函数
 ******************************************************************************/
void OLED_I2C_Start(void)
{
    // I2C起始信号: SCL高电平时，SDA从高到低跳变
    OLED_SDA_Set();
    OLED_SCL_Set();
    delay_us(5);
    OLED_SDA_Clr();
    delay_us(5);
    OLED_SCL_Clr();
}

void OLED_I2C_Stop(void)
{
    // I2C停止信号: SCL高电平时，SDA从低到高跳变
    OLED_SDA_Clr();
    OLED_SCL_Set();
    delay_us(5);
    OLED_SDA_Set();
}

void OLED_I2C_SendByte(u8 dat)
{
    u8 i;
    for(i = 0; i < 8; i++)
    {
        OLED_SCL_Clr();
        if(dat & 0x80)
            OLED_SDA_Set();
        else
            OLED_SDA_Clr();
        delay_us(2);
        OLED_SCL_Set();
        delay_us(2);
        dat <<= 1;
    }
    OLED_SCL_Clr();
}

u8 OLED_I2C_WaitAck(void)
{
    // 等待应答信号
    OLED_SDA_Set();
    delay_us(2);
    OLED_SCL_Set();
    delay_us(2);
    
    if(OLED_SDA_Read())  // 未收到应答
    {
        OLED_SCL_Clr();
        return 1;
    }
    OLED_SCL_Clr();
    return 0;
}

/******************************************************************************
 * 函数名: OLED_Init
 * 功能: 初始化OLED显示屏
 ******************************************************************************/
void OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置SCL和SDA为开漏输出
    GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_GPIO, &GPIO_InitStructure);
    
    delay_ms(100);
    
    // SSD1306初始化命令序列
    OLED_WriteCmd(0xAE);  // 关闭显示
    OLED_WriteCmd(0xD5);  // 设置时钟分频
    OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8);  // 设置多路复用率(1-64)
    OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3);  // 设置显示偏移
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);  // 设置起始行
    OLED_WriteCmd(0x8D);  // 电荷泵设置
    OLED_WriteCmd(0x14);  // 使能
    OLED_WriteCmd(0x20);  // 内存寻址模式
    OLED_WriteCmd(0x02);  // 页寻址模式
    OLED_WriteCmd(0xA1);  // 段重映射
    OLED_WriteCmd(0xC8);  // COM扫描方向
    OLED_WriteCmd(0xDA);  // COM引脚配置
    OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81);  // 对比度设置
    OLED_WriteCmd(0xCF);
    OLED_WriteCmd(0xD9);  // 预充电周期
    OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB);  // VCOMH电压
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4);  // 全局显示开启
    OLED_WriteCmd(0xA6);  // 正常显示(非反色)
    OLED_WriteCmd(0xAF);  // 开启显示
    
    OLED_Clear();
}

/******************************************************************************
 * 函数名: OLED_Clear
 * 功能: 清空屏幕
 ******************************************************************************/
void OLED_Clear(void)
{
    u8 i, n;
    for(i = 0; i < 8; i++)
    {
        OLED_WriteCmd(0xB0 + i);  // 设置页地址(0-7)
        OLED_WriteCmd(0x00);      // 设置列低地址
        OLED_WriteCmd(0x10);      // 设置列高地址
        for(n = 0; n < 128; n++)
            OLED_WriteData(0x00);
    }
}

/******************************************************************************
 * 函数名: OLED_ShowChar
 * 功能: 显示单个ASCII字符
 * 参数:
 *   x - 列坐标(0-127)
 *   y - 页坐标(0-7)
 *   chr - 要显示的字符
 * 说明: 字符大小为8x16
 ******************************************************************************/
void OLED_ShowChar(u8 x, u8 y, u8 chr)
{
    u8 c = chr - ' ';  // 计算字符在字库中的偏移
    
    // 显示上半部分(8x8)
    OLED_SetPos(x, y);
    for(u8 i = 0; i < 8; i++)
        OLED_WriteData(F8X16[c*16 + i]);
    
    // 显示下半部分(8x8)
    OLED_SetPos(x, y + 1);
    for(u8 i = 0; i < 8; i++)
        OLED_WriteData(F8X16[c*16 + i + 8]);
}

/******************************************************************************
 * 函数名: OLED_ShowString
 * 功能: 显示字符串
 * 参数:
 *   x - 起始列坐标
 *   y - 起始页坐标
 *   str - 字符串指针
 ******************************************************************************/
void OLED_ShowString(u8 x, u8 y, char *str)
{
    u8 j = 0;
    while(str[j])
    {
        OLED_ShowChar(x, y, str[j]);
        x += 8;
        if(x > 120)  // 换行
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

/******************************************************************************
 * 函数名: OLED_ShowNum
 * 功能: 显示数字
 * 参数:
 *   x, y - 坐标
 *   num - 要显示的数字
 *   len - 数字位数
 ******************************************************************************/
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len)
{
    u8 t, temp;
    u8 enshow = 0;  // 是否开始显示标志
    
    for(t = 0; t < len; t++)
    {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                OLED_ShowChar(x + 8 * t, y, ' ');
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + 8 * t, y, temp + '0');
    }
}

/******************************************************************************
 * 函数名: OLED_ShowChinese
 * 功能: 显示汉字
 * 参数:
 *   x, y - 坐标
 *   num - 汉字在字库中的编号
 * 说明: 汉字大小为16x16
 ******************************************************************************/
void OLED_ShowChinese(u8 x, u8 y, u8 num)
{
    u8 t;
    
    // 显示上半部分
    OLED_SetPos(x, y);
    for(t = 0; t < 16; t++)
        OLED_WriteData(Hzk[2 * num][t]);
    
    // 显示下半部分
    OLED_SetPos(x, y + 1);
    for(t = 0; t < 16; t++)
        OLED_WriteData(Hzk[2 * num + 1][t]);
}
```

---

### 4️⃣ HARDWARE/KEY/key.c - 按键扫描 (⭐⭐⭐⭐)

```c
/******************************************************************************
 * 文件名: key.c
 * 功能: 按键扫描驱动
 ******************************************************************************/

#include "key.h"
#include "delay.h"

// 按键引脚定义
#define KEY1_PIN  GPIO_Pin_12
#define KEY2_PIN  GPIO_Pin_13
#define KEY3_PIN  GPIO_Pin_14
#define KEY4_PIN  GPIO_Pin_15
#define KEY_GPIO  GPIOB

// 按键读取宏定义
#define KEY1  GPIO_ReadInputDataBit(KEY_GPIO, KEY1_PIN)
#define KEY2  GPIO_ReadInputDataBit(KEY_GPIO, KEY2_PIN)
#define KEY3  GPIO_ReadInputDataBit(KEY_GPIO, KEY3_PIN)
#define KEY4  GPIO_ReadInputDataBit(KEY_GPIO, KEY4_PIN)

/******************************************************************************
 * 函数名: KEY_Init
 * 功能: 初始化按键GPIO
 ******************************************************************************/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置为上拉输入(按键按下接地)
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_Init(KEY_GPIO, &GPIO_InitStructure);
}

/******************************************************************************
 * 函数名: KEY_Scan
 * 功能: 扫描按键状态
 * 参数: mode - 0:不支持连按  1:支持连按
 * 返回: 0:无按键  1:KEY1  2:KEY2  3:KEY3  4:KEY4
 ******************************************************************************/
u8 KEY_Scan(u8 mode)
{
    static u8 key_up = 1;  // 按键松开标志
    
    if(mode)
        key_up = 1;  // 支持连按
    
    if(key_up && (KEY1 == 0 || KEY2 == 0 || KEY3 == 0 || KEY4 == 0))
    {
        delay_ms(10);  // 去抖动延时
        key_up = 0;
        
        if(KEY1 == 0)
            return 1;
        else if(KEY2 == 0)
            return 2;
        else if(KEY3 == 0)
            return 3;
        else if(KEY4 == 0)
            return 4;
    }
    else if(KEY1 == 1 && KEY2 == 1 && KEY3 == 1 && KEY4 == 1)
    {
        key_up = 1;
    }
    
    return 0;
}
```

---

### 5️⃣ HARDWARE/GPIO/gpio.c - LED和红外传感器 (⭐⭐⭐⭐)

```c
/******************************************************************************
 * 文件名: gpio.c
 * 功能: LED控制和红外传感器读取
 ******************************************************************************/

#include "gpio.h"

// LED引脚定义
#define LED_GREEN1  GPIO_Pin_0  // PB0
#define LED_GREEN2  GPIO_Pin_1
#define LED_GREEN3  GPIO_Pin_2
#define LED_GREEN4  GPIO_Pin_3
#define LED_RED1    GPIO_Pin_4  // PB4
#define LED_RED2    GPIO_Pin_5
#define LED_RED3    GPIO_Pin_6
#define LED_RED4    GPIO_Pin_7
#define LED_LIGHT   GPIO_Pin_8  // PA8 照明灯

// 红外传感器引脚定义
#define IR1_PIN  GPIO_Pin_0  // PC0
#define IR2_PIN  GPIO_Pin_1
#define IR3_PIN  GPIO_Pin_2
#define IR4_PIN  GPIO_Pin_3

/******************************************************************************
 * 函数名: GPIO_Init
 * 功能: 初始化LED和红外传感器GPIO
 ******************************************************************************/
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
                           RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC, ENABLE);
    
    // LED引脚配置为推挽输出
    GPIO_InitStructure.GPIO_Pin = LED_GREEN1 | LED_GREEN2 | LED_GREEN3 | LED_GREEN4 |
                                   LED_RED1 | LED_RED2 | LED_RED3 | LED_RED4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 照明LED
    GPIO_InitStructure.GPIO_Pin = LED_LIGHT;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 红外传感器引脚配置为上拉输入
    GPIO_InitStructure.GPIO_Pin = IR1_PIN | IR2_PIN | IR3_PIN | IR4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/******************************************************************************
 * 函数名: LED_Green_On
 * 功能: 开启指定垃圾桶的绿灯
 ******************************************************************************/
void LED_Green_On(u8 led_num)
{
    switch(led_num)
    {
        case 1: GPIO_ResetBits(GPIOB, LED_GREEN1); break;
        case 2: GPIO_ResetBits(GPIOB, LED_GREEN2); break;
        case 3: GPIO_ResetBits(GPIOB, LED_GREEN3); break;
        case 4: GPIO_ResetBits(GPIOB, LED_GREEN4); break;
    }
}

void LED_Green_Off(u8 led_num)
{
    switch(led_num)
    {
        case 1: GPIO_SetBits(GPIOB, LED_GREEN1); break;
        case 2: GPIO_SetBits(GPIOB, LED_GREEN2); break;
        case 3: GPIO_SetBits(GPIOB, LED_GREEN3); break;
        case 4: GPIO_SetBits(GPIOB, LED_GREEN4); break;
    }
}

void LED_Red_On(u8 led_num)
{
    switch(led_num)
    {
        case 1: GPIO_ResetBits(GPIOB, LED_RED1); break;
        case 2: GPIO_ResetBits(GPIOB, LED_RED2); break;
        case 3: GPIO_ResetBits(GPIOB, LED_RED3); break;
        case 4: GPIO_ResetBits(GPIOB, LED_RED4); break;
    }
}

void LED_Red_Off(u8 led_num)
{
    switch(led_num)
    {
        case 1: GPIO_SetBits(GPIOB, LED_RED1); break;
        case 2: GPIO_SetBits(GPIOB, LED_RED2); break;
        case 3: GPIO_SetBits(GPIOB, LED_RED3); break;
        case 4: GPIO_SetBits(GPIOB, LED_RED4); break;
    }
}

void LED_Light_On(void)
{
    GPIO_ResetBits(GPIOA, LED_LIGHT);
}

void LED_Light_Off(void)
{
    GPIO_SetBits(GPIOA, LED_LIGHT);
}

/******************************************************************************
 * 函数名: IR_Sensor_Read
 * 功能: 读取红外传感器状态
 * 参数: sensor_num - 传感器编号(1-4)
 * 返回: 0-未检测到  1-检测到障碍物(垃圾桶已满)
 ******************************************************************************/
u8 IR_Sensor_Read(u8 sensor_num)
{
    u8 status = 0;
    
    switch(sensor_num)
    {
        case 1:
            status = (GPIO_ReadInputDataBit(GPIOC, IR1_PIN) == 0) ? 1 : 0;
            break;
        case 2:
            status = (GPIO_ReadInputDataBit(GPIOC, IR2_PIN) == 0) ? 1 : 0;
            break;
        case 3:
            status = (GPIO_ReadInputDataBit(GPIOC, IR3_PIN) == 0) ? 1 : 0;
            break;
        case 4:
            status = (GPIO_ReadInputDataBit(GPIOC, IR4_PIN) == 0) ? 1 : 0;
            break;
    }
    
    return status;
}
```

---

### 6️⃣ HARDWARE/ADC/adc.c - ADC采集模块 (⭐⭐⭐)

```c
/******************************************************************************
 * 文件名: adc.c
 * 功能: ADC初始化和数据采集
 * 说明: 用于读取光敏传感器的模拟电压值
 ******************************************************************************/

#include "adc.h"
#include "delay.h"

/******************************************************************************
 * 函数名: ADC_Init
 * 功能: 初始化ADC
 * 说明: 配置为独立模式，单次转换
 ******************************************************************************/
void ADC_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能ADC1和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    
    // 设置ADC分频因子6，72M/6=12MHz (ADC最大时钟不超过14MHz)
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // 配置PA4为模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  // 模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 复位ADC1
    ADC_DeInit(ADC1);
    
    // ADC配置
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                     // 单通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;               // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;            // 右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                           // 1个转换通道
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 使能ADC1
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC校准
    ADC_ResetCalibration(ADC1);                    // 复位校准
    while(ADC_GetResetCalibrationStatus(ADC1));    // 等待复位完成
    ADC_StartCalibration(ADC1);                    // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));         // 等待校准完成
}

/******************************************************************************
 * 函数名: Get_Adc
 * 功能: 获取指定通道的ADC值
 * 参数: ch - 通道号(0-17)
 * 返回: ADC转换结果(0-4095)
 ******************************************************************************/
u16 Get_Adc(u8 ch)
{
    // 设置转换通道、采样时间(239.5周期)
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);
    
    // 启动转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // 等待转换完成
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    // 返回转换结果
    return ADC_GetConversionValue(ADC1);
}

/******************************************************************************
 * 函数名: Get_Adc_Average
 * 功能: 获取指定通道的ADC平均值
 * 参数: 
 *   ch - 通道号
 *   times - 采样次数
 * 返回: ADC平均值
 * 说明: 多次采样取平均，提高稳定性
 ******************************************************************************/
u16 Get_Adc_Average(u8 ch, u8 times)
{
    u32 temp_val = 0;
    u8 t;
    
    for(t = 0; t < times; t++)
    {
        temp_val += Get_Adc(ch);
        delay_ms(5);
    }
    
    return temp_val / times;
}
```

---

### 7️⃣ SYSTEM/usart/usart.c - 串口通信 (⭐⭐⭐⭐⭐)

```c
/******************************************************************************
 * 文件名: usart.c
 * 功能: 串口通信驱动
 * 说明: 
 *   USART1 - 与语音识别模块通信(9600bps)
 *   USART2 - 与WiFi模块通信(115200bps)
 ******************************************************************************/

#include "usart.h"
#include "stdio.h"

// 串口1接收缓冲区(在main.c中定义)
extern u8 USART1_RX_BUF[200];
extern u16 USART1_RX_STA;

// 串口2接收缓冲区
extern u8 USART2_RX_BUF[200];
extern u16 USART2_RX_STA;

/******************************************************************************
 * 函数名: USART1_Init
 * 功能: 初始化串口1
 * 参数: baudrate - 波特率
 ******************************************************************************/
void USART1_Init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    // USART1_TX (PA9) - 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART1_RX (PA10) - 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置USART1参数
    USART_InitStructure.USART_BaudRate = baudrate;                   // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;      // 8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;           // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;              // 无校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // 收发模式
    USART_Init(USART1, &USART_InitStructure);
    
    // 配置NVIC中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  // 抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         // 响应优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    // 使能串口
    USART_Cmd(USART1, ENABLE);
}

/******************************************************************************
 * 函数名: USART2_Init
 * 功能: 初始化串口2 (与WiFi模块通信)
 * 参数: baudrate - 波特率
 ******************************************************************************/
void USART2_Init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // USART2_TX (PA2) - 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART2_RX (PA3) - 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置USART2参数
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    // 使能串口
    USART_Cmd(USART2, ENABLE);
}

/******************************************************************************
 * 函数名: USART_SendByte
 * 功能: 发送单个字节
 ******************************************************************************/
void USART_SendByte(USART_TypeDef* USARTx, u8 data)
{
    USART_SendData(USARTx, data);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

/******************************************************************************
 * 函数名: USART_SendString
 * 功能: 发送字符串
 ******************************************************************************/
void USART_SendString(USART_TypeDef* USARTx, char *str)
{
    while(*str)
    {
        USART_SendByte(USARTx, *str++);
    }
}

/******************************************************************************
 * 函数名: fputc
 * 功能: 重定向printf函数到串口1
 * 说明: 添加此函数后可使用printf进行调试输出
 ******************************************************************************/
int fputc(int ch, FILE *f)
{
    USART_SendByte(USART1, (u8)ch);
    return ch;
}
```

---

### 8️⃣ USER/stm32f10x_it.c - 中断服务函数 (⭐⭐⭐⭐)

```c
/******************************************************************************
 * 文件名: stm32f10x_it.c
 * 功能: 中断服务函数
 ******************************************************************************/

#include "stm32f10x_it.h"
#include "usart.h"

extern u8 USART1_RX_BUF[200];
extern u16 USART1_RX_STA;
extern u8 USART2_RX_BUF[200];
extern u16 USART2_RX_STA;

/******************************************************************************
 * 函数名: USART1_IRQHandler
 * 功能: 串口1中断服务函数(语音模块)
 * 说明: 
 *   - 接收数据存入缓冲区
 *   - 检测到换行符(\r\n)则认为接收完成
 *   - 接收状态标志最高位为1表示接收完成
 ******************************************************************************/
void USART1_IRQHandler(void)
{
    u8 res;
    
    // 接收中断
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART1);  // 读取接收数据
        
        if((USART1_RX_STA & 0x8000) == 0)  // 接收未完成
        {
            // 接收到换行符\n
            if(USART1_RX_STA & 0x4000)
            {
                if(res == 0x0A)  // \n
                {
                    USART1_RX_STA |= 0x8000;  // 标记接收完成
                }
                else
                {
                    USART1_RX_STA = 0;  // 接收错误,重新开始
                }
            }
            else  // 还未收到\r
            {
                if(res == 0x0D)  // \r
                {
                    USART1_RX_STA |= 0x4000;
                }
                else
                {
                    USART1_RX_BUF[USART1_RX_STA & 0x3FFF] = res;
                    USART1_RX_STA++;
                    
                    // 超过最大接收数量
                    if(USART1_RX_STA > (200 - 1))
                    {
                        USART1_RX_STA = 0;
                    }
                }
            }
        }
    }
}

/******************************************************************************
 * 函数名: USART2_IRQHandler
 * 功能: 串口2中断服务函数(WiFi模块)
 ******************************************************************************/
void USART2_IRQHandler(void)
{
    u8 res;
    
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART2);
        
        if((USART2_RX_STA & 0x8000) == 0)
        {
            if(USART2_RX_STA & 0x4000)
            {
                if(res == 0x0A)
                {
                    USART2_RX_STA |= 0x8000;
                }
                else
                {
                    USART2_RX_STA = 0;
                }
            }
            else
            {
                if(res == 0x0D)
                {
                    USART2_RX_STA |= 0x4000;
                }
                else
                {
                    USART2_RX_BUF[USART2_RX_STA & 0x3FFF] = res;
                    USART2_RX_STA++;
                    
                    if(USART2_RX_STA > (200 - 1))
                    {
                        USART2_RX_STA = 0;
                    }
                }
            }
        }
    }
}
```

---

### 9️⃣ SYSTEM/delay/delay.c - 延时函数 (⚠️)

```c
/******************************************************************************
 * 文件名: delay.c
 * 功能: 精确延时函数
 * 说明: 基于SysTick定时器实现
 ******************************************************************************/

#include "delay.h"

static u8 fac_us = 0;   // us延时倍乘数
static u16 fac_ms = 0;  // ms延时倍乘数

/******************************************************************************
 * 函数名: delay_init
 * 功能: 初始化延时函数
 * 说明: SysTick时钟为HCLK/8 = 72MHz/8 = 9MHz
 ******************************************************************************/
void delay_init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  // 选择外部时钟
    fac_us = SystemCoreClock / 8000000;  // 9MHz,每个us需要9个时钟周期
    fac_ms = (u16)fac_us * 1000;         // 每个ms需要9000个时钟周期
}

/******************************************************************************
 * 函数名: delay_us
 * 功能: 微秒级延时
 * 参数: nus - 延时时间(微秒)
 ******************************************************************************/
void delay_us(u32 nus)
{
    u32 temp;
    
    SysTick->LOAD = nus * fac_us;          // 设置重装载值
    SysTick->VAL = 0x00;                   // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  // 启动计数器
    
    do
    {
        temp = SysTick->CTRL;
    } while((temp & 0x01) && !(temp & (1 << 16)));  // 等待时间到达
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // 关闭计数器
    SysTick->VAL = 0x00;                         // 清空计数器
}

/******************************************************************************
 * 函数名: delay_ms
 * 功能: 毫秒级延时
 * 参数: nms - 延时时间(毫秒)
 ******************************************************************************/
void delay_ms(u16 nms)
{
    u32 temp;
    
    SysTick->LOAD = (u32)nms * fac_ms;
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    
    do
    {
        temp = SysTick->CTRL;
    } while((temp & 0x01) && !(temp & (1 << 16)));
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
}
```

---

## 开发指南

### 📝 开发流程(分阶段实现)

#### 🔹 第一阶段: 硬件测试(1-2天)

**目标**: 确保所有硬件正常工作

1. **测试OLED显示**
   - 修改文件: `USER/main.c`
   - 测试代码:
   ```c
   int main(void)
   {
       delay_init();
       OLED_Init();
       OLED_ShowString(0, 0, "Hello World");
       OLED_ShowNum(0, 2, 12345, 5);
       while(1);
   }
   ```

2. **测试舵机控制**
   - 修改文件: `USER/main.c`
   - 测试代码:
   ```c
   int main(void)
   {
       delay_init();
       Servo_Init();
       while(1)
       {
           Servo_Open(1);   // 打开舵机1
           delay_ms(2000);
           Servo_Close(1);  // 关闭舵机1
           delay_ms(2000);
       }
   }
   ```

3. **测试按键扫描**
   - 测试代码:
   ```c
   int main(void)
   {
       u8 key;
       delay_init();
       OLED_Init();
       KEY_Init();
       
       while(1)
       {
           key = KEY_Scan(0);
           if(key)
           {
               OLED_ShowString(0, 0, "Key:");
               OLED_ShowNum(40, 0, key, 1);
           }
       }
   }
   ```

4. **测试红外传感器**
   - 测试代码:
   ```c
   int main(void)
   {
       delay_init();
       OLED_Init();
       GPIO_Init();
       
       while(1)
       {
           for(u8 i = 1; i <= 4; i++)
           {
               u8 status = IR_Sensor_Read(i);
               OLED_ShowString(0, (i-1)*2, "IR");
               OLED_ShowNum(20, (i-1)*2, i, 1);
               OLED_ShowString(30, (i-1)*2, status ? "Full" : "OK");
           }
           delay_ms(100);
       }
   }
   ```

#### 🔹 第二阶段: 功能集成(2-3天)

5. **集成按键+舵机+显示**
   - 修改文件: `USER/main.c`
   - 实现按键控制垃圾桶开合
   - 显示投递次数

6. **集成红外检测+LED指示**
   - 实现垃圾桶满状态检测
   - LED灯状态指示(绿灯/红灯)

7. **集成光敏传感器+照明LED**
   - 实现自动照明功能
   - 调整亮度阈值

#### 🔹 第三阶段: 串口通信(2-3天)

8. **配置语音模块**
   - 修改文件: `SYSTEM/usart/usart.c`, `USER/stm32f10x_it.c`
   - 测试串口数据收发
   - 解析语音指令

9. **配置WiFi模块**
   - 测试WiFi连接
   - 实现数据上传功能

#### 🔹 第四阶段: 系统整合优化(1-2天)

10. **完善主程序逻辑**
    - 整合所有功能模块
    - 优化代码结构

11. **异常处理和调试**
    - 添加错误检测
    - 系统稳定性测试

---

### 🔧 修改文件权限表

| 文件/目录 | 是否可修改 | 修改频率 | 说明 |
|-----------|-----------|---------|------|
| `USER/main.c` | ✅ 必须修改 | ⭐⭐⭐⭐⭐ | 核心业务逻辑 |
| `HARDWARE/MOTOR_DUOJI/*` | ✅ 必须修改 | ⭐⭐⭐⭐ | 调整舵机参数 |
| `HARDWARE/OLED/*` | ✅ 可修改 | ⭐⭐⭐ | 自定义显示界面 |
| `HARDWARE/KEY/*` | ✅ 可修改 | ⭐⭐⭐ | 调整按键参数 |
| `HARDWARE/GPIO/*` | ✅ 必须修改 | ⭐⭐⭐⭐ | 根据硬件连接修改引脚 |
| `HARDWARE/ADC/*` | ✅ 可修改 | ⭐⭐ | 调整ADC通道和阈值 |
| `SYSTEM/usart/*` | ✅ 必须修改 | ⭐⭐⭐⭐ | 配置串口协议 |
| `USER/stm32f10x_it.c` | ✅ 必须修改 | ⭐⭐⭐ | 添加中断处理 |
| `USER/stm32f10x_conf.h` | ⚠️ 按需修改 | ⭐ | 外设库配置 |
| `SYSTEM/delay/*` | ⚠️ 一般不修改 | - | 延时函数 |
| `SYSTEM/sys/*` | ⚠️ 一般不修改 | - | 系统配置 |
| `CORE/*` | ❌ 不能修改 | - | 启动文件 |
| `STM32F10x_FWLib/*` | ❌ 不能修改 | - | 官方库 |

---

### 🔌 硬件连接参考

```
【STM32F103引脚分配表】

功能模块         │ 引脚   │ 电平   │ 说明
─────────────────┼────────┼────────┼──────────────────
舵机1 PWM        │ PA0    │ 3.3V   │ TIM2_CH1
舵机2 PWM        │ PA1    │ 3.3V   │ TIM2_CH2
舵机3 PWM        │ PA2    │ 3.3V   │ TIM2_CH3
舵机4 PWM        │ PA3    │ 3.3V   │ TIM2_CH4
─────────────────┼────────┼────────┼──────────────────
按键1            │ PB12   │ 上拉   │ 按下接地
按键2            │ PB13   │ 上拉   │ 按下接地
按键3            │ PB14   │ 上拉   │ 按下接地
按键4            │ PB15   │ 上拉   │ 按下接地
─────────────────┼────────┼────────┼──────────────────
红外传感器1      │ PC0    │ 上拉   │ 检测到输出低电平
红外传感器2      │ PC1    │ 上拉   │ 检测到输出低电平
红外传感器3      │ PC2    │ 上拉   │ 检测到输出低电平
红外传感器4      │ PC3    │ 上拉   │ 检测到输出低电平
─────────────────┼────────┼────────┼──────────────────
绿色LED1         │ PB0    │ 推挽   │ 低电平点亮
绿色LED2         │ PB1    │ 推挽   │ 低电平点亮
绿色LED3         │ PB2    │ 推挽   │ 低电平点亮
绿色LED4         │ PB3    │ 推挽   │ 低电平点亮
红色LED1         │ PB4    │ 推挽   │ 低电平点亮
红色LED2         │ PB5    │ 推挽   │ 低电平点亮
红色LED3         │ PB6    │ 推挽   │ 低电平点亮
红色LED4         │ PB7    │ 推挽   │ 低电平点亮
照明LED          │ PA8    │ 推挽   │ 低电平点亮
─────────────────┼────────┼────────┼──────────────────
OLED SCL         │ PB6    │ 开漏   │ I2C时钟(需上拉电阻)
OLED SDA         │ PB7    │ 开漏   │ I2C数据(需上拉电阻)
─────────────────┼────────┼────────┼──────────────────
串口1 TX(语音)   │ PA9    │ 推挽   │ USART1_TX
串口1 RX         │ PA10   │ 浮空   │ USART1_RX
串口2 TX(WiFi)   │ PA2    │ 推挽   │ USART2_TX
串口2 RX         │ PA3    │ 浮空   │ USART2_RX
─────────────────┼────────┼────────┼──────────────────
光敏传感器       │ PA4    │ 模拟   │ ADC_IN4
─────────────────┼────────┼────────┼──────────────────
电源             │ 3.3V   │ -      │ 系统电源
                 │ 5V     │ -      │ 舵机电源(外部供电)
                 │ GND    │ -      │ 公共地
```

---

## 调试与故障排除

### 🐛 常见问题及解决方案

#### 问题1: 舵机不转动或抖动

**症状**: 舵机无反应或不停抖动

**可能原因**:
1. PWM频率不对(应为50Hz)
2. 电源供电不足(舵机需要5V大电流)
3. PWM信号线接触不良

**解决方法**:
```c
// 检查TIM2配置
// 预分频: 72-1 (计数频率1MHz)
// 重装载: 20000-1 (周期20ms, 频率50Hz)

// 测试代码: 观察PWM波形
void Test_PWM(void)
{
    Servo_Init();
    while(1)
    {
        Servo_SetAngle(1, 0);    // 0度
        delay_ms(1000);
        Servo_SetAngle(1, 90);   // 90度
        delay_ms(1000);
        Servo_SetAngle(1, 180);  // 180度
        delay_ms(1000);
    }
}
```

**检查方法**:
- 用示波器测量PA0引脚PWM波形
- 0度时脉宽应为0.5ms
- 90度时脉宽应为1.5ms
- 180度时脉宽应为2.5ms

---

#### 问题2: OLED无显示

**症状**: OLED屏幕全黑或全亮

**可能原因**:
1. I2C地址错误(0x78或0x7A)
2. SCL/SDA接线错误
3. 上拉电阻缺失或阻值不对
4. 初始化时序错误

**解决方法**:
```c
// 1. 检查I2C地址
#define OLED_ADDRESS  0x78  // 或0x7A

// 2. 测试I2C通信
void Test_I2C(void)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_ADDRESS);
    if(OLED_I2C_WaitAck() == 0)
        printf("I2C OK\r\n");
    else
        printf("I2C Error\r\n");
    OLED_I2C_Stop();
}

// 3. 添加上拉电阻
// SCL和SDA需要4.7K上拉电阻到3.3V
```

---

#### 问题3: 串口接收数据乱码

**症状**: 串口收到的数据不正确

**可能原因**:
1. 波特率不匹配
2. 数据位、停止位、校验位配置错误
3. 晶振频率不准确

**解决方法**:
```c
// 1. 确认波特率配置
USART1_Init(9600);   // 语音模块通常用9600

// 2. 测试回环
void Test_USART_Loop(void)
{
    u8 data;
    USART1_Init(9600);
    
    while(1)
    {
        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
        {
            data = USART_ReceiveData(USART1);
            USART_SendByte(USART1, data);  // 回传
        }
    }
}

// 3. 使用USB转TTL工具测试
// PC -> USB转TTL -> STM32
// 使用串口助手发送数据，观察接收是否正确
```

---

#### 问题4: 红外传感器误触发

**症状**: 没有垃圾也显示已满

**可能原因**:
1. 传感器灵敏度太高
2. 环境光干扰
3. 检测距离设置不合理

**解决方法**:
```c
// 1. 增加检测次数过滤
u8 IR_Sensor_Read_Stable(u8 sensor_num)
{
    u8 count = 0;
    
    // 连续检测10次
    for(u8 i = 0; i < 10; i++)
    {
        if(IR_Sensor_Read(sensor_num) == 1)
            count++;
        delay_ms(10);
    }
    
    // 超过8次检测到才认为已满
    return (count >= 8) ? 1 : 0;
}

// 2. 调整传感器位置和角度
// 确保传感器垂直向下检测

// 3. 添加遮光罩
// 避免环境光干扰
```

---

#### 问题5: WiFi连接失败

**症状**: WiFi模块无法连接网络

**解决方法**:
```c
// 1. 测试AT指令
void Test_WiFi_AT(void)
{
    USART2_Init(115200);
    
    // 发送AT测试指令
    USART_SendString(USART2, "AT\r\n");
    delay_ms(1000);
    
    // 查询WiFi模式
    USART_SendString(USART2, "AT+CWMODE?\r\n");
    delay_ms(1000);
    
    // 连接WiFi
    USART_SendString(USART2, "AT+CWJAP=\"SSID\",\"PASSWORD\"\r\n");
    delay_ms(5000);
}

// 2. 检查返回信息
// 应该收到 "OK" 或 "WIFI CONNECTED"

// 3. 常见WiFi模块AT指令
/*
AT              - 测试指令
AT+RST          - 重启模块
AT+CWMODE=1     - 设置为Station模式
AT+CWJAP="xx","xx" - 连接WiFi
AT+CIPSTART="TCP","IP",PORT - 建立TCP连接
AT+CIPSEND=len  - 发送数据
*/
```

---

### 🔍 调试技巧

#### 1. 使用printf调试

在 `usart.c` 中已实现 `fputc` 重定向，可直接使用printf:

```c
int main(void)
{
    System_Init();
    
    printf("System Start!\r\n");
    printf("ADC Value: %d\r\n", Get_Adc(4));
    printf("Bin Status: %d %d %d %d\r\n", 
           bin_full[0], bin_full[1], bin_full[2], bin_full[3]);
}
```

#### 2. LED指示调试

```c
// 使用板载LED指示程序运行状态
void Debug_LED_Blink(u8 times)
{
    for(u8 i = 0; i < times; i++)
    {
        LED_Light_On();
        delay_ms(100);
        LED_Light_Off();
        delay_ms(100);
    }
}

// 示例: 进入某个函数时闪烁
void Open_Bin(u8 bin_num)
{
    Debug_LED_Blink(1);  // 闪烁1次表示进入函数
    // ... 函数代码
}
```

#### 3. OLED实时监控

```c
// 在OLED上显示调试信息
void Debug_Display(void)
{
    OLED_ShowString(0, 0, "Debug Info:");
    OLED_ShowString(0, 2, "ADC:");
    OLED_ShowNum(40, 2, Get_Adc(4), 4);
    OLED_ShowString(0, 4, "Tick:");
    OLED_ShowNum(40, 4, system_tick, 5);
}
```

#### 4. 单步调试(Keil)

1. 点击菜单 `Debug -> Start/Stop Debug Session` (Ctrl+F5)
2. 设置断点: 在代码行号处单击
3. 单步执行: F10(Step Over) / F11(Step Into)
4. 观察变量: 将鼠标悬停在变量上或添加到Watch窗口

---

## 扩展功能实现建议

### 扩展1: 添加温度监测

**硬件**: DS18B20温度传感器

**修改文件**: `HARDWARE/DS18B20/*`, `USER/main.c`

```c
// 在main.c中添加
void Temp_Monitor(void)
{
    static u32 last_check = 0;
    
    if(system_tick - last_check >= 100)  // 每5秒检测一次
    {
        float temp = DS18B20_Get_Temp();
        
        OLED_ShowString(0, 6, "Temp:");
        OLED_ShowNum(50, 6, (u16)temp, 2);
        OLED_ShowString(70, 6, "C");
        
        // 温度过高报警
        if(temp > 40.0)
        {
            Voice_Play("温度过高，请注意");
        }
        
        last_check = system_tick;
    }
}
```

---

### 扩展2: 添加称重功能

**硬件**: HX711 + 压力传感器

**新增文件**: `HARDWARE/HX711/hx711.c`

```c
// hx711.c 示例代码
u32 HX711_Read(void)
{
    u32 count = 0;
    u8 i;
    
    HX711_SCK_LOW();
    
    // 等待DOUT变为低电平(数据准备好)
    while(HX711_DOUT_READ());
    
    // 读取24位数据
    for(i = 0; i < 24; i++)
    {
        HX711_SCK_HIGH();
        count = count << 1;
        HX711_SCK_LOW();
        if(HX711_DOUT_READ())
            count++;
    }
    
    // 第25个脉冲，设置增益为128
    HX711_SCK_HIGH();
    count = count ^ 0x800000;  // 转换为有符号数
    HX711_SCK_LOW();
    
    return count;
}

// 获取重量(克)
float HX711_Get_Weight(void)
{
    u32 val = HX711_Read();
    float weight = (val - HX711_OFFSET) / HX711_SCALE;
    return weight;
}
```

---

### 扩展3: 添加人体感应

**硬件**: HC-SR501人体红外传感器

**修改文件**: `HARDWARE/GPIO/gpio.c`, `USER/main.c`

```c
// 在GPIO初始化中添加
#define PIR_PIN  GPIO_Pin_5
#define PIR_GPIO GPIOC

// 初始化
GPIO_InitStructure.GPIO_Pin = PIR_PIN;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(PIR_GPIO, &GPIO_InitStructure);

// 读取人体感应
u8 PIR_Read(void)
{
    return GPIO_ReadInputDataBit(PIR_GPIO, PIR_PIN);
}

// 在main.c中使用
void PIR_Process(void)
{
    static u8 last_status = 0;
    u8 current_status = PIR_Read();
    
    if(current_status == 1 && last_status == 0)  // 检测到人
    {
        LED_Light_On();  // 开启照明
        Voice_Play("欢迎使用智能垃圾桶");
        OLED_Clear();
        OLED_ShowString(20, 2, "Welcome!");
    }
    else if(current_status == 0 && last_status == 1)  // 人离开
    {
        delay_ms(5000);  // 延时5秒后关灯
        if(PIR_Read() == 0)
        {
            LED_Light_Off();
        }
    }
    
    last_status = current_status;
}
```

---

### 扩展4: 数据统计和分析

**功能**: 记录每天各类垃圾投递次数，生成统计报表

**修改文件**: `USER/main.c`

```c
// 添加统计数组
typedef struct {
    u16 hour;        // 小时
    u16 count[4];    // 4类垃圾计数
} StatData;

StatData daily_stat[24];  // 24小时统计

// 更新统计
void Update_Statistics(u8 bin_type)
{
    u8 hour = RTC_GetHour();  // 需要添加RTC模块
    daily_stat[hour].count[bin_type - 1]++;
}

// 显示统计
void Display_Statistics(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "Daily Stats:");
    
    for(u8 i = 0; i < 4; i++)
    {
        u16 total = 0;
        for(u8 h = 0; h < 24; h++)
        {
            total += daily_stat[h].count[i];
        }
        
        OLED_ShowString(0, 2 + i*2, bin_name[i]);
        OLED_ShowNum(80, 2 + i*2, total, 3);
    }
}
```

---

## 项目优化建议

### 1. 代码优化

#### 使用状态机优化主循环

```c
typedef enum {
    STATE_IDLE,          // 空闲状态
    STATE_VOICE_PROC,    // 语音处理
    STATE_KEY_PROC,      // 按键处理
    STATE_BIN_OPENING,   // 开盖中
    STATE_BIN_CLOSING,   // 关盖中
    STATE_ERROR          // 错误状态
} SystemState;

SystemState sys_state = STATE_IDLE;

void System_StateMachine(void)
{
    switch(sys_state)
    {
        case STATE_IDLE:
            // 空闲状态，监听输入
            if(Voice_Available())
                sys_state = STATE_VOICE_PROC;
            else if(KEY_Scan(0))
                sys_state = STATE_KEY_PROC;
            break;
            
        case STATE_VOICE_PROC:
            // 处理语音
            Process_Voice();
            sys_state = STATE_BIN_OPENING;
            break;
            
        case STATE_BIN_OPENING:
            // 开盖，等待2秒
            if(Open_Timer_Expired())
                sys_state = STATE_BIN_CLOSING;
            break;
            
        case STATE_BIN_CLOSING:
            // 关盖
            Close_Current_Bin();
            sys_state = STATE_IDLE;
            break;
            
        case STATE_ERROR:
            // 错误处理
            Error_Handler();
            sys_state = STATE_IDLE;
            break;
    }
}
```

#### 使用DMA提高串口效率

```c
void USART1_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    // DMA1通道5: USART1_RX
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1_RX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 200;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    
    DMA_Cmd(DMA1_Channel5, ENABLE);
}
```

### 2. 低功耗优化

```c
// 使用睡眠模式
void Enter_Sleep_Mode(void)
{
    // 配置唤醒源(按键中断、串口中断)
    PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
}

// 在主循环中
void main(void)
{
    System_Init();
    
    while(1)
    {
        if(No_Activity_For_10_Minutes())
        {
            Enter_Sleep_Mode();  // 进入睡眠节能
        }
        
        // 正常运行
        System_StateMachine();
    }
}
```

---

## 总结

### ✅ 各模块功能总结

| 模块 | 核心功能 | 关键文件 | 优先级 |
|------|---------|---------|--------|
| 舵机控制 | 开关垃圾桶盖 | motor_duoji.c | ⭐⭐⭐⭐⭐ |
| OLED显示 | 状态显示 | oled.c | ⭐⭐⭐⭐ |
| 按键扫描 | 手动控制 | key.c | ⭐⭐⭐⭐ |
| GPIO控制 | LED和红外 | gpio.c | ⭐⭐⭐⭐ |
| 串口通信 | 语音/WiFi | usart.c | ⭐⭐⭐⭐⭐ |
| ADC采集 | 光敏传感器 | adc.c | ⭐⭐⭐ |
| 主程序 | 业务逻辑 | main.c | ⭐⭐⭐⭐⭐ |

### 📚 学习建议

1. **先硬件后软件**: 确保每个硬件模块单独测试通过
2. **循序渐进**: 按照开发流程逐步实现功能
3. **模块化编程**: 每个功能独立封装，便于调试和维护
4. **充分注释**: 代码中添加详细注释，方便后期修改
5. **版本控制**: 使用Git管理代码，记录每次修改

### 🎯 项目验收标准

- [ ] 4个舵机能正确开合
- [ ] OLED显示内容正确
- [ ] 4个按键功能正常
- [ ] 红外传感器能检测满桶状态
- [ ] LED指示灯状态正确
- [ ] 语音识别准确率>90%
- [ ] 语音播报清晰
- [ ] 自动照明功能正常
- [ ] WiFi能成功上传数据
- [ ] 系统运行稳定无死机

---

## 附录

### A. 常用资源链接

- [STM32官方文档](https://www.st.com/stm32)
- [Keil MDK下载](https://www.keil.com/download/)
- [STM32固件库下载](https://www.st.com/en/embedded-software/stm32-standard-peripheral-libraries.html)

### B. 引脚速查表

完整引脚配置请参考上文"硬件连接参考"部分

### C. 协议格式定义

**语音模块 -> STM32**:
```
TYPE:1\r\n  // 可回收垃圾
TYPE:2\r\n  // 厨余垃圾
TYPE:3\r\n  // 有害垃圾
TYPE:4\r\n  // 其他垃圾
```

**STM32 -> WiFi模块**:
```json
{
  "bin1": 0,  // 0=未满, 1=已满
  "bin2": 0,
  "bin3": 1,
  "bin4": 0,
  "count1": 15,  // 投递次数
  "count2": 8,
  "count3": 3,
  "count4": 12
}
```

---

## 版本历史

- **V1.0** (2024-10-04): 初始版本，实现基本功能
- **V1.1** (待定): 添加温度监测和称重功能
- **V2.0** (待定): 增加手机APP控制

---

## 作者与贡献

**项目作者**: [您的名字]  
**联系方式**: [您的邮箱]  
**GitHub**: https://github.com/Chuanwang1/S

---

## 许可证

本项目仅供学习交流使用，未经授权不得用于商业用途。

---

**📝 文档最后更新**: 2024-10-04
