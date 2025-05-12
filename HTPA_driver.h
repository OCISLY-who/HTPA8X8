/* HTPA_driver.h */
#ifndef __HTPA_DRIVER_H
#define __HTPA_DRIVER_H

#include "stm32f10x.h"

/* 传感器配置参数 */
#define HTPA8X8_ROWS          8       // 传感器行数
#define HTPA8X8_COLS          8       // 传感器列数
#define HTPA8X8_PIXELS        64      // 总像素数
#define HTPA8X8_ADDR          0x1A    // 传感器I2C地址(7位)
#define HTPA8X8_MEASURE_DELAY 50      // 单次测量延时(ms)

/* I2C接口配置 */
#define IIC_GPIO              GPIOB
#define IIC_SCL_PIN           GPIO_Pin_6
#define IIC_SDA_PIN           GPIO_Pin_7

/* 函数声明 */
void HTPA8x8_Init(void);                          // 初始化传感器
void HTPA8x8_StartMeasure(void);                  // 启动单次测量
void HTPA8x8_StopMeasure(void);                   // 停止测量
uint8_t HTPA8x8_ReadRaw(uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS]); // 读取原始数据
void HTPA8x8_CalcTemp_Formula(const uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS], 
                              float temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // 计算温度
float HTPA8x8_GetMaxTemp(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // 获取最高温度
uint8_t HTPA8x8_ContinuousMeasure(uint8_t count, float avg_temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // 连续测量
void HTPA8x8_PrintTemperature(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // 打印温度矩阵

#endif      
