/* HTPA_driver.h */
#ifndef __HTPA_DRIVER_H
#define __HTPA_DRIVER_H

#include "stm32f10x.h"

/* ���������ò��� */
#define HTPA8X8_ROWS          8       // ����������
#define HTPA8X8_COLS          8       // ����������
#define HTPA8X8_PIXELS        64      // ��������
#define HTPA8X8_ADDR          0x1A    // ������I2C��ַ(7λ)
#define HTPA8X8_MEASURE_DELAY 50      // ���β�����ʱ(ms)

/* I2C�ӿ����� */
#define IIC_GPIO              GPIOB
#define IIC_SCL_PIN           GPIO_Pin_6
#define IIC_SDA_PIN           GPIO_Pin_7

/* �������� */
void HTPA8x8_Init(void);                          // ��ʼ��������
void HTPA8x8_StartMeasure(void);                  // �������β���
void HTPA8x8_StopMeasure(void);                   // ֹͣ����
uint8_t HTPA8x8_ReadRaw(uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS]); // ��ȡԭʼ����
void HTPA8x8_CalcTemp_Formula(const uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS], 
                              float temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // �����¶�
float HTPA8x8_GetMaxTemp(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // ��ȡ����¶�
uint8_t HTPA8x8_ContinuousMeasure(uint8_t count, float avg_temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // ��������
void HTPA8x8_PrintTemperature(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]); // ��ӡ�¶Ⱦ���

#endif      
