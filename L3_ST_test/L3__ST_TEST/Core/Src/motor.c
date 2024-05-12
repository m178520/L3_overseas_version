#include "motor.h"
#include "deal_data.h"

void vcu_Trans_LGD(double Angle,double Speed)
{
	static uint8_t vcu_data[8] = {0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	/*对数组进行赋值*/
	vcu_data[1] = Angle;
	vcu_data[2] = Speed;
	if(vcu_data[3] >= 255) vcu_data[3]++;
	
	usart2_send_data_apply(vcu_data,8);
}

