#include "flow.h"
#include "bsp_delay.h"
#include "usart.h"
#include "led.h"

static FLOW_DATA_t rx_data;
struct _flow_ mini;
struct _pixel_flow_ pixel_flow;
float pixel_cpi = 1.0f; 
uint8_t Flow_SSI, Flow_SSI_CNT, Flow_Err;
_st_AngE Angle;

void Flow_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_data.data, 1);
}

/**
 * @brief 串口接收解析函数（解析FE 04协议帧）
 */
void Flow_Receive(uint8_t data) 
{
    static uint8_t RxBuffer[12]; // 协议最大11字节，减少内存占用
    static uint8_t _data_cnt = 0;
    static uint8_t state = 0; 
    static uint8_t fault_cnt = 0;
    
    switch(state)
    {
        case 0:
            if(data == 0xFE) { state = 1; RxBuffer[0] = data; _data_cnt = 1; }
            break;
        case 1:
            if(data == 0x04) { state = 2; RxBuffer[1] = data; _data_cnt = 2; }
            else state = 0;
            break;
        case 2:
            RxBuffer[_data_cnt++] = data;
            if(_data_cnt == 11)
            {
                state = 0;
                uint8_t sum = 0;
                // 计算 DATA0-DATA5 的校验和
                for(uint8_t i=2; i<8; i++) sum += RxBuffer[i];

                if((0xAA == data) && (sum == RxBuffer[8])) 
                {
                    Flow_SSI_CNT++;
                    // 解析原始数据
                    mini.flow_x = -((int16_t)(RxBuffer[3]<<8 | RxBuffer[2]));
                    mini.flow_y =  ((int16_t)(RxBuffer[5]<<8 | RxBuffer[4]));
                    mini.qual   = RxBuffer[9];
                    
                    // 高度处理 (mm 转 cm)
                    float raw_high = (int16_t)(RxBuffer[7]<<8 | RxBuffer[6]) * 0.1f;
                    mini.flow_High = raw_high;

                    // 优化 CPI 计算：预计算常量 ((50*0.01)/11.914)*2.54
                    const float base_cpi = 0.106597f;
                    static float high_filter = 0;
                    high_filter += (raw_high - high_filter) * 0.1f;
                    
                    // 计算当前高度下的实际像素比例
                    pixel_cpi = base_cpi * (1.0f + (high_filter - 50.0f) * 0.002f); 

                    // 积分累加
                    mini.flow_x_iOUT += mini.flow_x;
                    mini.flow_y_iOUT += mini.flow_y;
                    
                    // 积分前完成高度融合
                    mini.flow_x_i += (mini.flow_x * pixel_cpi);
                    mini.flow_y_i += (mini.flow_y * pixel_cpi);
                    
                    // 质量判断逻辑
                    if(mini.qual < 25) {
                        if(++fault_cnt > 60) { fault_cnt = 60; mini.ok = 0; }
                    } else {
                        fault_cnt = 0; mini.ok = 1;
                    }
                }
            }
            break;
        default: state = 0; break;
    }
}
/**
 * @brief 光流数据融合与修正主函数
 * @param dT 调用周期（秒），如10ms调用则传入0.01f
 */
// 光流数据融合修正 (建议 10ms 调用一次)
void Pixel_Flow_Fix(float dT) 
{
    // 异常处理
    if(Flow_Err == 1 || !mini.ok)
    {		
        mini.flow_x_i = mini.flow_y_i = 0;
        pixel_flow.fix_x = pixel_flow.fix_y = 0;
        pixel_flow.err1_cnt++;
        return;
    }

    // 预计算除法
    float inv_dt = 1.0f / dT;

    // 1. 位移低通滤波
    pixel_flow.fix_x_i += (mini.flow_x_i - pixel_flow.fix_x_i) * 0.2f;
    pixel_flow.fix_y_i += (mini.flow_y_i - pixel_flow.fix_y_i) * 0.2f;

    // 2. 姿态补偿计算：减少 tan() 调用
    // 600.0f 为根据焦距等参数确定的补偿系数
    float comp_target_x = 600.0f * tan(-Angle.pitch * ANG_2_RAD);
    float comp_target_y = 600.0f * tan(-Angle.roll * ANG_2_RAD);

    pixel_flow.ang_x += (comp_target_x - pixel_flow.ang_x) * 0.2f;
    pixel_flow.ang_y += (comp_target_y - pixel_flow.ang_y) * 0.2f;

    // 3. 融合补偿（计算最终位移）
    pixel_flow.out_x_i = pixel_flow.fix_x_i - (pixel_flow.ang_x * pixel_cpi);
    pixel_flow.out_y_i = pixel_flow.fix_y_i - (pixel_flow.ang_y * pixel_cpi);

    // 4. 微分求速度
    pixel_flow.x = (pixel_flow.out_x_i - pixel_flow.out_x_i_o) * inv_dt;
    pixel_flow.y = (pixel_flow.out_y_i - pixel_flow.out_y_i_o) * inv_dt;
    
    // 更新历史值
    pixel_flow.out_x_i_o = pixel_flow.out_x_i;
    pixel_flow.out_y_i_o = pixel_flow.out_y_i;

    // 5. 速度滤波
    pixel_flow.fix_x += (pixel_flow.x - pixel_flow.fix_x) * 0.1f;
    pixel_flow.fix_y += (pixel_flow.y - pixel_flow.fix_y) * 0.1f;

    // 6. 输出结果赋值
    pixel_flow.loc_x = pixel_flow.out_x_i;
    pixel_flow.loc_y = pixel_flow.out_y_i;
    pixel_flow.loc_xs = pixel_flow.fix_x;
    pixel_flow.loc_ys = pixel_flow.fix_y;

    // 7. 坐标溢出保护：当位移超过 10m 时移动坐标轴
    if(fabs(pixel_flow.loc_x) > 1000.0f || fabs(pixel_flow.loc_y) > 1000.0f)
    {
        float shift_x = (pixel_flow.loc_x > 1000.0f) ? 1000.0f : (pixel_flow.loc_x < -1000.0f ? -1000.0f : 0);
        float shift_y = (pixel_flow.loc_y > 1000.0f) ? 1000.0f : (pixel_flow.loc_y < -1000.0f ? -1000.0f : 0);
        // Pixel_Flow_set_zero(shift_x, shift_y);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)   // 判断是USART2触发的回调
    {
        Flow_Receive(rx_data.data);  // 处理接收到的数据
        rx_data.flag = 1;  // 标记接收完成
        HAL_UART_Receive_IT(&huart1, &rx_data.data, 1);
    }
}
