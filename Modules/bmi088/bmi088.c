#include "bmi088.h"
#include "bsp_iic.h"
#include "bsp_delay.h"
#include "gpio.h"

// 静态定义总线对象（关联硬件引脚）
static iic_bus_t bmi088_bus = {
    .IIC_SDA_PORT = SDA_GPIO_Port,
    .IIC_SDA_PIN = SDA_Pin,
    .IIC_SCL_PORT = SCL_GPIO_Port,
    .IIC_SCL_PIN = SCL_Pin
};

/**
 * @brief  BMI088 初始化 (bsp_iic 风格)
 * @return 0:成功, 1:ACC错误, 2:GYRO错误, 3:配置校验失败
 */
uint8_t BMI088_Init(void)
{
    uint8_t status = 0;
    uint8_t read_val = 0;

    // --- 1. 加速度计初始化 ---
    // 软复位
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_SOFTRESET, 0xB6);
    delay_ms(50);
    
    // 开启电源 (顺序极其关键)
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04);
    delay_ms(10); 
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00);
    delay_ms(10);

    // 校验 ACC ID
    read_val = IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CHIP_ID);
    if(read_val != 0x1E) return 1;

    // 配置 ACC：±6g, ODR 400Hz, OSR4 滤波
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_RANGE, 0x01);
    IIC_Write_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CONF, 0x8A);
    
    // 验证配置是否写入成功
    if(IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CONF) != 0x8A) return 3;

    // --- 2. 陀螺仪初始化 ---
    // 软复位
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_SOFTRESET, 0xB6);
    delay_ms(50);

    // 校验 GYRO ID
    read_val = IIC_Read_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_CHIP_ID);
    if(read_val != 0x0F) return 2;

    // 配置 GYRO：±2000dps, ODR 1000Hz, BW 116Hz, Normal Mode
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_RANGE, 0x00);
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_BANDWIDTH, 0x02);
    IIC_Write_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_LPM1, 0x00);

    return 0; // 全部成功
}

uint8_t BMI088_Get_Acc_ID(void)
{
    // 假设宏定义 #define ACC_CHIP_ID 0x00
    return IIC_Read_One_Byte(&bmi088_bus, BMI088_ACC_ADDR, ACC_CHIP_ID);
}

uint8_t BMI088_Get_Gyro_ID(void)
{
    // 假设宏定义 #define GYRO_CHIP_ID 0x00
    return IIC_Read_One_Byte(&bmi088_bus, BMI088_GYRO_ADDR, GYRO_CHIP_ID);
}