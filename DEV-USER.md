## USER 目录详细解读与 main.c 框架示例

### 一、USER 目录各文件作用说明

| 文件名           | 作用简介                                                         |
|------------------|------------------------------------------------------------------|
| main.c           | 项目主入口，负责系统初始化、主循环、各功能模块调用               |
| gpio.c/h         | GPIO（通用输入输出）端口配置与操作，控制LED、继电器、按键等      |
| key.c/h          | 按键检测与消抖，提供按键事件接口                                 |
| motor_duoji.c/h  | 舵机/电机驱动，控制垃圾桶盖子的开关                              |
| oled.c/h         | OLED显示屏驱动，显示垃圾桶状态、投放次数等                       |
| adc.c/h          | 模数转换（ADC），垃圾容量检测、环境参数采集                       |
| usart.c/h        | 串口通信，语音模块/WiFi模块数据传输                              |
| delay.c/h        | 毫秒/微秒级延时函数                                              |
| sys.c/h          | 系统相关功能（如定时器、全局配置等）                             |
| ds18b20.c/h      | DS18B20温度传感器驱动                                            |
| system_stm32f10x.h | 系统初始化头文件                                               |
| Template.BAT     | 编译脚本，无需修改                                               |

---

### 二、main.c 典型框架示例

```c
#include "stm32f10x.h"
#include "gpio.h"
#include "key.h"
#include "motor_duoji.h"
#include "oled.h"
#include "adc.h"
#include "usart.h"
#include "sys.h"
#include "delay.h"

int main(void)
{
    // 1. 系统时钟初始化
    SystemInit();
    delay_init();

    // 2. 各模块初始化
    GPIO_Init();
    OLED_Init();
    ADC_Init();
    USART_Init();
    Motor_Duoji_Init();
    Key_Init();

    // 3. 变量声明
    uint8_t key_val;
    uint16_t trash_level;
    char trash_type[16];

    while (1)
    {
        // 4. 检测垃圾桶容量
        trash_level = ADC_Read();
        if (trash_level > THRESHOLD_FULL) {
            OLED_ShowString(0, 0, "垃圾桶已满");
            // WiFi上传状态
            SendStatusToCloud("full");
        }

        // 5. 语音识别垃圾类型
        if (Voice_Recognize(trash_type)) {
            Motor_Duoji_Open(trash_type);     // 打开相应垃圾桶盖
            OLED_ShowString(0, 2, trash_type);
            Voice_Broadcast(trash_type);      // 语音播报
            Record_TrashCount(trash_type);    // 投放次数统计
        }

        // 6. 按键手动开盖
        key_val = Key_Scan();
        if (key_val == KEY_TRASH_OPEN) {
            Motor_Duoji_Open("default");
        }

        // 7. 智能照明控制
        Lighting_Control();

        delay_ms(50);
    }
}
```

---

### 三、各模块接口示例与解读

#### 1. gpio.c/h
- **功能**：端口初始化、控制单片机IO口
- **典型接口**：
  ```c
  void GPIO_Init(void);
  void LED_ON(void);
  void LED_OFF(void);
  ```

#### 2. key.c/h
- **功能**：按键检测与消抖
- **典型接口**：
  ```c
  void Key_Init(void);
  uint8_t Key_Scan(void);  // 返回按键值
  ```

#### 3. motor_duoji.c/h
- **功能**：控制舵机或电机，实现盖子开关
- **典型接口**：
  ```c
  void Motor_Duoji_Init(void);
  void Motor_Duoji_Open(char* trash_type);   // 根据垃圾类型打开盖子
  void Motor_Duoji_Close(void);
  ```

#### 4. oled.c/h
- **功能**：OLED显示屏驱动与显示内容刷新
- **典型接口**：
  ```c
  void OLED_Init(void);
  void OLED_ShowString(uint8_t x, uint8_t y, char* str);
  void OLED_ShowTrashCount(uint8_t count);
  ```

#### 5. adc.c/h
- **功能**：ADC采集垃圾桶容量或其它模拟信号
- **典型接口**：
  ```c
  void ADC_Init(void);
  uint16_t ADC_Read(void);  // 返回采样值
  ```

#### 6. usart.c/h
- **功能**：串口通信，语音识别、WiFi模块等数据交互
- **典型接口**：
  ```c
  void USART_Init(void);
  void USART_SendData(uint8_t* data, uint16_t len);
  uint8_t USART_ReceiveData(uint8_t* buf);
  int Voice_Recognize(char* trash_type); // 识别语音返回垃圾类型
  void Voice_Broadcast(char* trash_type); // 播报垃圾类型
  void SendStatusToCloud(char* status);   // 上传状态到云端
  ```

#### 7. delay.c/h
- **功能**：延时函数
- **典型接口**：
  ```c
  void delay_init(void);
  void delay_ms(uint16_t nms);
  ```

#### 8. sys.c/h
- **功能**：系统相关功能扩展，如定时器、全局变量等
- **典型接口**：
  ```c
  void SysTick_Init(void);
  ```

#### 9. ds18b20.c/h
- **功能**：温度传感器驱动
- **典型接口**：
  ```c
  void DS18B20_Init(void);
  float DS18B20_GetTemp(void);
  ```

---

### 四、开发建议

- **主流程在 main.c 完成，业务功能封装在各自 .c/.h 文件**，便于模块化开发与维护。
- **新增功能建议新增模块文件，并在 main.c 调用。**
- **调试时建议逐步测试各模块，确保硬件驱动与业务逻辑稳定。**

如需更详细代码模板或某模块实现，可进一步补充说明。
