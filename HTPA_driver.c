/* HTPA_driver.c */
#include "HTPA_driver.h"
#include "OLED.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>

/*---------------------- ���IICʵ�� --------------------------*/
/* I2C�źſ��ƺ궨�� */
#define SDA_H()    GPIO_SetBits(IIC_GPIO, IIC_SDA_PIN)
#define SDA_L()    GPIO_ResetBits(IIC_GPIO, IIC_SDA_PIN)
#define SCL_H()    GPIO_SetBits(IIC_GPIO, IIC_SCL_PIN)
#define SCL_L()    GPIO_ResetBits(IIC_GPIO, IIC_SCL_PIN)
#define SDA_READ() GPIO_ReadInputDataBit(IIC_GPIO, IIC_SDA_PIN)

/**
 * @brief ����SDA����Ϊ����ģʽ
 */
static void SDA_IN(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
}

/**
 * @brief ����SDA����Ϊ���ģʽ
 */
static void SDA_OUT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
}

/**
 * @brief I2C������ʱ����
 */
static void IIC_Delay(void) {
    Delay_us(5); // ����ϵͳʱ�ӵ�����ʱ
}

/**
 * @brief ����I2C��ʼ�ź�
 */
static void IIC_Start(void) {
    SDA_OUT();
    SDA_H(); 
    SCL_H(); 
    IIC_Delay();
    SDA_L(); // SDA��SCL�ߵ�ƽʱ�½���������ʼ�ź�
    IIC_Delay();
    SCL_L(); // ǯסI2C���ߣ�׼����������
    IIC_Delay();
}

/**
 * @brief ����I2Cֹͣ�ź�
 */
static void IIC_Stop(void) {
    SDA_OUT();
    SCL_L();
    SDA_L();
    IIC_Delay();
    SCL_H(); // SDA��SCL�ߵ�ƽʱ����������ֹͣ�ź�
    SDA_H(); 
    IIC_Delay();
}

/**
 * @brief �ȴ����豸Ӧ���ź�
 * @return 0-Ӧ��ɹ���1-Ӧ��ʧ��
 */
static uint8_t IIC_Wait_Ack(void) {
    uint8_t t = 0;
    SDA_IN();
    SDA_H(); 
    IIC_Delay();
    SCL_H(); 
    IIC_Delay();
    
    while(SDA_READ()) { // �ȴ�SDA�����豸����
        t++;
        if(t > 250) { // ��ʱ����
            IIC_Stop();
            return 1;
        }
    }
    
    SCL_L(); // ʱ�������ͣ�����Ӧ��λ
    IIC_Delay();
    return 0;
}

/**
 * @brief ����ACKӦ���ź�
 */
static void IIC_Ack(void) {
    SCL_L();
    SDA_OUT();
    SDA_L(); // SDA���ͱ�ʾӦ��
    IIC_Delay();
    SCL_H(); // ʱ�������ߣ�����Ӧ���ź�
    IIC_Delay();
    SCL_L(); // ʱ�������ͣ�����Ӧ��
    IIC_Delay();
}

/**
 * @brief ����NACK��Ӧ���ź�
 */
static void IIC_NAck(void) {
    SCL_L();
    SDA_OUT();
    SDA_H(); // SDA���߱�ʾ��Ӧ��
    IIC_Delay();
    SCL_H(); // ʱ�������ߣ����ͷ�Ӧ���ź�
    IIC_Delay();
    SCL_L(); // ʱ�������ͣ�������Ӧ��
    IIC_Delay();
}

/**
 * @brief ����һ���ֽ�����
 * @param txd Ҫ���͵��ֽ�����
 */
static void IIC_Send_Byte(uint8_t txd) {
    SDA_OUT();
    SCL_L(); // ����ʱ���߿�ʼ���ݴ���
    
    for(uint8_t t = 0; t < 8; t++) {
        if(txd & 0x80) 
            SDA_H(); 
        else 
            SDA_L();
        
        txd <<= 1;
        IIC_Delay();
        SCL_H(); // ʱ�������ߣ��ô��豸��ȡ����
        IIC_Delay();
        SCL_L(); // ʱ�������ͣ�׼��������һλ
        IIC_Delay();
    }
}

/**
 * @brief ��ȡһ���ֽ�����
 * @param ack 1-����ACKӦ��0-����NACK��Ӧ��
 * @return ��ȡ���ֽ�����
 */
static uint8_t IIC_Read_Byte(unsigned char ack) {
    unsigned char i, receive = 0;
    SDA_IN(); // ����SDAΪ����ģʽ
    
    for(i = 0; i < 8; i++) {
        SCL_L(); // ʱ�������ͣ�׼����������
        IIC_Delay();
        SCL_H(); // ʱ�������ߣ����豸��������
        receive <<= 1;
        
        if(SDA_READ()) 
            receive++;
            
        IIC_Delay();
    }
    
    if(!ack) 
        IIC_NAck(); // ����NACK��������ȡ
    else 
        IIC_Ack();  // ����ACK��������ȡ
    
    return receive;
}

/*---------------------- �������������� --------------------------*/

/**
 * @brief ��ʼ��HTPA8x8������
 * ����I2C�ӿںʹ�������������
 */
void HTPA8x8_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ʹ��GPIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // ����I2C����
    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
    
    // ��ʼ��I2C����״̬
    SDA_H();
    SCL_H();
}

/**
 * @brief ���������¶Ȳ���
 */
void HTPA8x8_StartMeasure(void) {
    // ���Ͳ�����������
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 0); // ����д��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(0x01); // ���ƼĴ�����ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(0x09); // �������� (START=1, WAKEUP=1)
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
 * @brief ֹͣ�¶Ȳ���������͹���ģʽ
 */
void HTPA8x8_StopMeasure(void) {
    // ���Ͳ���ֹͣ����
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 0); // ����д��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(0x01); // ���ƼĴ�����ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00); // ֹͣ����
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
 * @brief ��ȡ64���ص�ԭʼ����
 * @param raw �洢ԭʼ���ݵĶ�ά����
 * @return 0-�ɹ�����0-ʧ��
 */
uint8_t HTPA8x8_ReadRaw(uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    // ���Ͷ�ȡ��������
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 0); // ����д��ַ
    if(IIC_Wait_Ack()) return 1;
    
    IIC_Send_Byte(0x0A); // ���ݼĴ�����ʼ��ַ
    if(IIC_Wait_Ack()) return 2;
    
    // ���¿�ʼ��׼����ȡ����
    IIC_Start();
    IIC_Send_Byte((HTPA8X8_ADDR << 1) | 1); // ���Ͷ���ַ
    if(IIC_Wait_Ack()) return 3;
    
    // ��ȡ64����������(ÿ������16λ)
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            uint8_t high = IIC_Read_Byte(1); // ��ȡ���ֽ�
            uint8_t low = IIC_Read_Byte((i == HTPA8X8_ROWS-1 && j == HTPA8X8_COLS-1)?0:1); // ��ȡ���ֽ�
            raw[i][j] = ((uint16_t)high << 8) | low; // ��ϳ�16λ����
						OLED_ShowFNum (4,1,raw[i][j] ,6,3);
        }
    }
    IIC_Stop();
    return 0; // �ɹ�
}

/**
 * @brief ����ԭʼ���ݼ����¶�ֵ
 * @param raw ԭʼ��������
 * @param temp �������¶�ֵ����(��λ:��C)
 */
void HTPA8x8_CalcTemp_Formula(const uint16_t raw[HTPA8X8_ROWS][HTPA8X8_COLS], 
                              float temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    // ���������ֲṫʽ: T = (ADC / 16.0) - 273.15
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            temp[i][j] = (raw[i][j] / 16.0f) - 273.15f;
        }
    }
}

/**
 * @brief ��ȡ�¶Ⱦ����е�����¶�
 * @param temp �¶Ⱦ���
 * @return ����¶�ֵ
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
 * @brief ����������β�����ƽ��ֵ
 * @param count ��������
 * @param avg_temp �洢ƽ���¶ȵ�����
 * @return 0-�ɹ�����0-ʧ��(������ʱ�Ĳ�������)
 */
uint8_t HTPA8x8_ContinuousMeasure(uint8_t count, float avg_temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    uint16_t raw_data[HTPA8X8_ROWS][HTPA8X8_COLS];
    float temp_data[HTPA8X8_ROWS][HTPA8X8_COLS];
    float sum_temp[HTPA8X8_ROWS][HTPA8X8_COLS] = {0};
    
    // ��ʼ���ۼ���
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            sum_temp[i][j] = 0.0f;
        }
    }
    
    // ��������ָ������
    for(uint8_t c = 0; c < count; c++) {
        // �������β���
        HTPA8x8_StartMeasure();
        
        // �ȴ��������
        Delay_ms(HTPA8X8_MEASURE_DELAY);
        
        // ��ȡԭʼ����
        if(HTPA8x8_ReadRaw(raw_data) != 0) {
            return c + 1; // ���ش�����ʱ�Ĳ�������
        }
        
        // �����¶�
        HTPA8x8_CalcTemp_Formula(raw_data, temp_data);
        
        // �ۼ��¶�ֵ
        for(int i = 0; i < HTPA8X8_ROWS; i++) {
            for(int j = 0; j < HTPA8X8_COLS; j++) {
                sum_temp[i][j] += temp_data[i][j];
            }
        }
        
        // ������ʱ���������Ƶ���Ĳ���
        Delay_ms(10);
    }
    
    // ����ƽ��ֵ
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            avg_temp[i][j] = sum_temp[i][j] / (float)count;
        }
    }
    
    return 0; // �ɹ�������в���
}

/**
 * @brief ��ӡ�¶Ⱦ���
 * @param temp �¶Ⱦ���
 */
void HTPA8x8_PrintTemperature(const float temp[HTPA8X8_ROWS][HTPA8X8_COLS]) {
    printf("\nHTPA8x8 Temperature Matrix (��C):\n");
    printf("=================================\n");
    
    for(int i = 0; i < HTPA8X8_ROWS; i++) {
        for(int j = 0; j < HTPA8X8_COLS; j++) {
            printf("%6.1f ", temp[i][j]);
        }
        printf("\n");
    }
    
    printf("=================================\n");
    printf("Max Temperature: %.1f��C\n", HTPA8x8_GetMaxTemp(temp));
}       
