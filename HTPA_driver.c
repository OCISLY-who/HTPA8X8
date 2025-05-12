/* HTPA_driver.c */
#include "HTPA_driver.h"
#include "OLED.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>

/*---------------------- 软件IIC实现 --------------------------*/
/* I2C信号控制宏定义 */
#define SDA_H()    GPIO_SetBits(IIC_GPIO, IIC_SDA_PIN)
#define SDA_L()    GPIO_ResetBits(IIC_GPIO, IIC_SDA_PIN)
#define SCL_H()    GPIO_SetBits(IIC_GPIO, IIC_SCL_PIN)
#define SCL_L()    GPIO_ResetBits(IIC_GPIO, IIC_SCL_PIN)
#define SDA_READ() GPIO_ReadInputDataBit(IIC_GPIO, IIC_SDA_PIN)

/**
 * @brief 配置SDA引脚为输入模式
 */
static void SDA_IN(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
}

/**
 * @brief 配置SDA引脚为输出模式
 */
static void SDA_OUT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
}

/**
 * @brief I2C总线延时函数
 */
static void IIC_Delay(void) {
    Delay_us(5); // 根据系统时钟调整延时
}

/**
 * @brief 产生I2C起始信号
 */
static void IIC_Start(void) {
    SDA_OUT();
    SDA_H(); 
    SCL_H(); 
    IIC_Delay();
    SDA_L(); // SDA在SCL高电平时下降，产生起始信号
    IIC_Delay();
    SCL_L(); // 钳住I2C总线，准备发送数据
    IIC_Delay();
}

/**
 * @brief 产生I2C停止信号
 */
static void IIC_Stop(void) {
    SDA_OUT();
    SCL_L();
    SDA_L();
    IIC_Delay();
    SCL_H(); // SDA在SCL高电平时上升，产生停止信号
    SDA_H(); 
    IIC_Delay();
}

/**
 * @brief 等待从设备应答信号
 * @return 0-应答成功，1-应答失败
 */
static uint8_t IIC_Wait_Ack(void) {
    uint8_t t = 0;
    SDA_IN();
    SDA_H(); 
    IIC_Delay();
    SCL_H(); 
    IIC_Delay();
    
    while(SDA_READ()) { // 等待SDA被从设备拉低
        t++;
        if(t > 250) { // 超时处理
            IIC_Stop();
            return 1;
        }
    }
    
    SCL_L(); // 时钟线拉低，结束应答位
    IIC_Delay();
    return 0;
}

/**
 * @brief 产生ACK应答信号
 */
static void IIC_Ack(void) {
    SCL_L();
    SDA_OUT();
    SDA_L(); // SDA拉低表示应答
    IIC_Delay();
    SCL_H(); // 时钟线拉高，发送应答信号
    IIC_Delay();
    SCL_L(); // 时钟线拉低，结束应答
    IIC_Delay();
}

/**
 * @brief 产生NACK非应答信号
 */
static void IIC_NAck(void) {
    SCL_L();
    SDA_OUT();
    SDA_H(); // SDA拉高表示非应答
    IIC_Delay();
    SCL_H(); // 时钟线拉高，发送非应答信号
    IIC_Delay();
    SCL_L(); // 时钟线拉低，结束非应答
    IIC_Delay();
}

/**
 * @brief 发送一个字节数据
 * @param txd 要发送的字节数据
 */
static void IIC_Send_Byte(uint8_t txd) {
    SDA_OUT();
    SCL_L(); // 拉低时钟线开始数据传输
    
    for(uint8_t t = 0; t < 8; t++) {
        if(txd & 0x80) 
            SDA_H(); 
        else 
            SDA_L();
        
        txd <<= 1;
        IIC_Delay();
        SCL_H(); // 时钟线拉高，让从设备读取数据
        IIC_Delay();
        SCL_L(); // 时钟线拉低，准备发送下一位
        IIC_Delay();
    }
}

/**
 * @brief 读取一个字节数据
 * @param ack 1-发送ACK应答，0-发送NACK非应答
 * @return 读取的字节数据
 */
static uint8_t IIC_Read_Byte(unsigned char ack) {
    unsigned char i, receive = 0;
    SDA_IN(); // 设置SDA为输入模式
    
    for(i = 0; i < 8; i++) {
        SCL_L(); // 时钟线拉低，准备接收数据
        IIC_Delay();
        SCL_H(); // 时钟线拉高，从设备发送数据
        receive <<= 1;
        
        if(SDA_READ()) 
            receive++;
            
        IIC_Delay();
    }
    
    if(!ack) 
        IIC_NAck(); // 发送NACK，结束读取
    else 
        IIC_Ack();  // 发送ACK，继续读取
    
    return receive;
}

/*---------------------- 传感器驱动函数 --------------------------*/

/**
 * @brief 初始化HTPA8x8传感器
 * 配置I2C接口和传感器基本设置
 */
void HTPA8x8_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置I2C引脚
    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
    
    // 初始化I2C总线状态
    SDA_H();
    SCL_H();
}

/**
 * @brief 启动单次温度测量
 */
void HTPA8x8_StartMeasure(void) {
    // 发送测量启动命令
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 0); // 发送写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x01); // 控制寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x09); // 启动测量 (START=1, WAKEUP=1)
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
 * @brief 停止温度测量，进入低功耗模式
 */
void HTPA8x8_StopMeasure(void) {
    // 发送测量停止命令
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 0); // 发送写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x01); // 控制寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00); // 停止测量
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
 * @brief 读取64像素的原始数据
 * @param raw 存储原始数据的二维数组
 * @return 0-成功，非0-失败
 */
uint8_t HTPA8x8_ReadRaw(uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    // 发送读取数据命令
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 0); // 发送写地址
    if(IIC_Wait_Ack()) return 1;
    
    IIC_Send_Byte(0x0A); // 数据寄存器起始地址
    if(IIC_Wait_Ack()) return 2;
    
    // 重新开始，准备读取数据
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 1); // 发送读地址
    if(IIC_Wait_Ack()) return 3;
    
    // 读取64个像素数据(每个像素16位)
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            uint8_t high = IIC_Read_Byte(1); // 读取高字节
            uint8_t low = IIC_Read_Byte((i == HTPA8X8_ROWS-1 && j == HTPA8X8_COLS-1)?0:1); // 读取低字节
            raw[i][j] = ((uint16_t)high << 8) | low; // 组合成16位数据
						OLED_ShowFNum (4,1,raw[i][j] ,6,3);
        }
    }
    IIC_Stop();
    return 0; // 成功
}

/**
 * @brief 根据原始数据计算温度值
 * @param raw 原始数据数组
 * @param temp 计算后的温度值数组(单位:°C)
 */
void HTPA8x8_CalcTemp_Formula(const uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS], 
                              float temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    // 根据数据手册公式: T = (ADC / 16.0) - 273.15
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            temp[i][j] = (raw[i][j] / 16.0f) - 273.15f;
        }
    }
}

/**
 * @brief 获取温度矩阵中的最高温度
 * @param temp 温度矩阵
 * @return 最高温度值
 */
float HTPA8x8_GetMaxTemp(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    float max = temp[0][0];
    
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            if(temp[i][j] > max) 
                max = temp[i][j];
        }
    }
    
    return max;
}

/**
 * @brief 连续测量多次并计算平均值
 * @param count 测量次数
 * @param avg_temp 存储平均温度的数组
 * @return 0-成功，非0-失败(错误发生时的测量次数)
 */
uint8_t HTPA8x8_ContinuousMeasure(uint8_t count, float avg_temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    uint16_t raw_data[HTPA8X8_ROWS][HTPA8X8_COLS];
    float temp_data[HTPA8X8_ROWS][HTPA8X8_COLS];
    float sum_temp[HTPA8X8_ROWS][HTPA8X8_COLS] = {0};
    
    // 初始化累加器
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            sum_temp[i][j] = 0.0f;
        }
    }
    
    // 连续测量指定次数
    for(uint8_t c = 0; c < count; c++) {
        // 启动单次测量
        HTPA8x8_StartMeasure();
        
        // 等待测量完成
        Delay_ms(HTPA8X8_MEASURE_DELAY);
        
        // 读取原始数据
        if(HTPA8x8_ReadRaw(raw_data) != 0) {
            return c + 1; // 返回错误发生时的测量次数
        }
        
        // 计算温度
        HTPA8x8_CalcTemp_Formula(raw_data, temp_data);
        
        // 累加温度值
        for(int i = 0; i < HTPA8X8_ROWS; i++) {
            for(int j = 0; j < HTPA8X8_COLS; j++) {
                sum_temp[i][j] += temp_data[i][j];
            }
        }
        
        // 短暂延时，避免过于频繁的测量
        Delay_ms(10);
    }
    
    // 计算平均值
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            avg_temp[i][j] = sum_temp[i][j] / (float)count;
        }
    }
    
    return 0; // 成功完成所有测量
}

/**
 * @brief 打印温度矩阵
 * @param temp 温度矩阵
 */
void HTPA8x8_PrintTemperature(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    printf("\nHTPA8x8 Temperature Matrix (°C):\n");
    printf("=================================\n");
    
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            printf("%6.1f ", temp[i][j]);
        }
        printf("\n");
    }
    
    printf("=================================\n");
    printf("Max Temperature: %.1f°C\n", HTPA8x8_GetMaxTemp(temp));
}       
