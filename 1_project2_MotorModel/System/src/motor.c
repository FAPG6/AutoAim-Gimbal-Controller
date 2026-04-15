#include "motor.h"
#include "dm_drv.h"
#include "pid.h"
#include "BMI088driver.h"
#include "ws2812.h"

Joint_Motor_t underMotor = { 0 };
Joint_Motor_t upperMotor = { 0 };

extern redrPos redr;
extern uint16_t x, y, w, h;



void fdcan1_rx_callback(void)
{
	canFrame frame = { 0 };
	fdcanx_receive_frame(&hfdcan1, &frame, FDCAN_RX_FIFO0);
	
	dmCallback(&underMotor, &frame);
}


void fdcan2_rx_callback(void)
{
	canFrame frame = { 0 };
	fdcanx_receive_frame(&hfdcan2, &frame, FDCAN_RX_FIFO0);

}


static float limitFp(float input, float max, float min)
{
	if ( input > max) {
		return max;
	}else if( input < min) {
		return min;
	}
	
	return input;
}



// static  160 112
//#define xAbsCenter 173
//#define yAbsCenter 117

//  50cm  xy 173 102 h 120
// 100cm  xy 173 114 h 51
// 150cm  xy 173 118 h 34

// 图像处理以及回复的Task
float posRingSpeed = 0;	// under电机的位置环输出速度

float xAbsCenter = 173;	
float yAbsCenter = 118;

PID_t yawPID = { 0 };
PID_t pitchPID = { 0 };
void motorTask(void* param)
{
	
	
	
	PID_Init(&yawPID, POS_PID, 0.02, 0, 0, 90, 0, 100);
	PID_Init(&pitchPID, POS_PID, 0.01, 0, 0.025, 12, 0, 100);

	int32_t no_frame_times = 0;
	while(1) {
		while(Mode_Motor == 2){
			vTaskDelay(1);
		}
		
		while(redr.upDate == false){
			vTaskDelay(1);
			no_frame_times++;
			if(no_frame_times > 100){
				posRingSpeed = 0;
				speed_ctrl(&upperMotor, 0);

				no_frame_times = 0;
			}
		}

		redr.upDate = false;
		
		yAbsCenter = 120 - ((h - 34.0) / 86.0) * 16.0;
		if( Mode_Motor != 0){
			posRingSpeed = PID_Calc(&yawPID, xAbsCenter - redr.xCenter, 70);
			speed_ctrl(&upperMotor, PID_Calc(&pitchPID, -yAbsCenter + redr.yCenter, 70));
		}else{
			speed_ctrl(&upperMotor, 0);
		}
	}
}


float gy = 0;
void canTask(void* param)
{
	const TickType_t xFrequency = pdMS_TO_TICKS(2);
	

	joint_motor_init(&underMotor, 0x01, 0x11, SPEED_MODE, &hfdcan1, DM6220);
	joint_motor_init(&upperMotor, 0x02, 0x12, SPEED_MODE, &hfdcan2, DM4310);
	
	vTaskDelay(2000);
	
	while (BMI088_init()) {
		vTaskDelay(100);
	}
	
	enable_motor_mode(&underMotor, 1);
	enable_motor_mode(&upperMotor, 1);


	vTaskDelay(1);
	speed_ctrl(&underMotor, 0);
	speed_ctrl(&upperMotor, 0);
	vTaskDelay(1000);


	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){

		if(Mode_Motor == 0){
			speed_ctrl(&underMotor,0);
			HAL_GPIO_WritePin(RealLaser_GPIO_Port, RealLaser_Pin, 0);
			
		}else if(Mode_Motor == 1){
			for(int i = 0; i < 1000; i++){
				speed_ctrl(&underMotor, posRingSpeed);
				vTaskDelay(1);
			}
			
			Mode_Motor = 0;
			speed_ctrl(&underMotor,0);
			speed_ctrl(&upperMotor,0);
			HAL_GPIO_WritePin(RealLaser_GPIO_Port, RealLaser_Pin, 1);
			vTaskDelay(1000);
			HAL_GPIO_WritePin(RealLaser_GPIO_Port, RealLaser_Pin, 0);
		}else if(Mode_Motor == 2){
			for(int i = 0; i < 3000; i++){
				speed_ctrl(&underMotor, 3);
				vTaskDelay(1);
				if(redr.upDate == true){
					break;
				}
			}
			Mode_Motor = 1;

		}else if(Mode_Motor == 3){
			yawPID.kp = 0.04;
			pitchPID.kp = 0.02;
			
			for(int i = 0; i < 3000; i++){
				speed_ctrl(&underMotor, 3);
				vTaskDelay(1);
				if(redr.upDate == true){
					break;
				}
			}
			
			
			for(int i = 0; i<40*1000;i++){
				if(i > 2*1000){
					HAL_GPIO_WritePin(RealLaser_GPIO_Port, RealLaser_Pin, 1);
				}
					
				gy = BMI088_read_gy();
				speed_ctrl(&underMotor, posRingSpeed + posRingSpeed + gy/180*3.1415926);
				vTaskDelay(1);
			}
			Mode_Motor = 0;
			yawPID.kp = 0.02;
			pitchPID.kp = 0.01;
		}
		
		//speed_ctrl(&underMotor, lastVel );
//		speed_ctrl(&underMotor, posRingSpeed + posRingSpeed + gy/180*3.1415926);
		
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

uint8_t Mode_Motor = 0;
void keyTask(void* param)
{
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
	

	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){
		//key1 2S启动开关
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == 0)
		{
			Mode_Motor = 1;
			vTaskDelay(1000);
		}
		//key2 4S启动开关
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9) == 0)
		{
			Mode_Motor = 2;
			vTaskDelay(1000);
		}
		//key3 加陀螺仪在小车上面跑
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == 0)
		{
			Mode_Motor = 3;
			vTaskDelay(1000);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


/*

void gambalAngle(float* pitch, float* yaw)
{
	const float Pi = 3.1415926;
	
	*pitch = limitFp(*pitch, 12, -12);
	*yaw = limitFp(*yaw, 360, -360);
	
	float pitchR = *pitch / 180 * Pi;
	float yawR = *yaw / 180 * Pi;

	//pos_speed_ctrl(&underMotor, yawR, 6);
	mit_ctrl(&underMotor, yawR, 0, 1.2, 0.025, 0);
	mit_ctrl(&upperMotor, pitchR, 0, 16, 0.08, 0);
	//pos_speed_ctrl(&upperMotor, pitchR, 6);
}


float pitch = 0, yaw = 0;

void motorTask(void* param)
{	
	PID_t yawPID = { 0 };
	PID_t pitchPID = { 0 };
	
	joint_motor_init(&underMotor, 0x01, 0x11, MIT_MODE, &hfdcan1, DM6220);
	joint_motor_init(&upperMotor, 0x02, 0x12, MIT_MODE, &hfdcan2, DM4310);
	
	
	PID_Init(&yawPID, POS_PID, 0.07, 0, 1, 90, 0, 100);
	PID_Init(&pitchPID, POS_PID, 0.01, 0, 0, 12, 0, 100);

	vTaskDelay(2000);

	enable_motor_mode(&underMotor, 1);
	enable_motor_mode(&upperMotor, 1);

	vTaskDelay(1);
	gambalAngle(&pitch, &yaw);
	vTaskDelay(1000);

	while(1) {
		while(redr.upDate == false){
			vTaskDelay(1);
		}
		redr.upDate = false;
		
		
		yaw += PID_Calc(&yawPID, xAbsCenter - redr.xCenter, 70);
		pitch += PID_Calc(&yawPID, -yAbsCenter + redr.yCenter, 70);
		
		
		//yaw += (xAbsCenter - redr.xCenter) * 0.03;
		//pitch += (-yAbsCenter + redr.yCenter) * 0.03;

		gambalAngle(&pitch, &yaw);
		print("xerr=%2.2f, yerr=%2.2f, yaw=%2.2f, pitch=%2.2f\r\n",
				(float)(xAbsCenter - redr.xCenter),
				(float)(-yAbsCenter + redr.yCenter),
				yaw,
				pitch
		);
		
		
	}
}
*/

/*
void motorTask(void* param)
{
	
	float pitch = 0, yaw = 0;
	PID_t yawPID = { 0 };
	PID_t pitchPID = { 0 };
	
	joint_motor_init(&underMotor, 0x01, 0x11, POS_MODE, &hfdcan1, DM6220);
	joint_motor_init(&upperMotor, 0x02, 0x12, POS_MODE, &hfdcan2, DM4310);
	
	
	PID_Init(&yawPID, POS_PID, 0.07, 0, 0.05, 90, 0, 100);
	PID_Init(&pitchPID, POS_PID, 0.07, 0, 0.05, 12, 0, 100);

	vTaskDelay(2000);

	enable_motor_mode(&underMotor, 1);
	enable_motor_mode(&upperMotor, 1);

	vTaskDelay(1);
	gambalAngle(&pitch, &yaw);
	vTaskDelay(1000);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1) {
		while(redr.upDate == false){
			vTaskDelay(1);
		}
		redr.upDate = false;
		
		
		yaw += PID_Calc(&yawPID, xAbsCenter - redr.xCenter, xTaskGetTickCount() - xLastWakeTime);
		pitch += PID_Calc(&yawPID, -yAbsCenter + redr.yCenter, xTaskGetTickCount() - xLastWakeTime);
		xLastWakeTime = xTaskGetTickCount();
		
		
		//yaw += (xAbsCenter - redr.xCenter) * 0.03;
		//pitch += (-yAbsCenter + redr.yCenter) * 0.03;

		gambalAngle(&pitch, &yaw);
		print("xerr=%2.2f, yerr=%2.2f, yaw=%2.2f, pitch=%2.2f\r\n",
				(float)(xAbsCenter - redr.xCenter),
				(float)(-yAbsCenter + redr.yCenter),
				yaw,
				pitch
		);
		
		
	}
}
*/
