#include "dm_drv.h"


motorType DM6220 = {
	.P_MIN = -12.5, .P_MAX = 12.5,
	.V_MIN = -45, .V_MAX = 45,
	.KP_MIN = 0, .KP_MAX = 500,
	.KD_MIN = 0, .KD_MAX = 5,
	.T_MIN = -10, .T_MAX = 10
};

motorType DM4310 = {
	.P_MIN = -12.5, .P_MAX = 12.5,
	.V_MIN = -30, .V_MAX = 30,
	.KP_MIN = 0, .KP_MAX = 500,
	.KD_MIN = 0, .KD_MAX = 5,
	.T_MIN = -10, .T_MAX = 10
};

/**
************************************************************************
* @brief:      	fp_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int fp_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_fp: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_fp(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t recvId, DMmotorMode mode, FDCAN_HandleTypeDef *hfdcan, motorType type)
{
	motor->mode=mode;
	motor->para.id=id;
	motor->para.recvId = recvId;
	motor->type = type;
	motor->hfdcan = hfdcan;

}



static void dmCanSend(Joint_Motor_t *motor, uint16_t id, uint8_t* data, uint32_t len)
{
	canFrame txFrame = { 0 };
	txFrame.frame_id = id;
	memcpy(txFrame.frame_data, data, len);
	txFrame.data_len = len;
	
	fdcanx_send_frame(motor->hfdcan, &txFrame);
}


void dmCallback(Joint_Motor_t *motor, canFrame* frame)
{ 
	if( frame->data_len == 8 && frame->frame_id == motor->para.recvId) {
		
		motor->para.id = (frame->frame_data[0])&0x0F;
		motor->para.state = (frame->frame_data[0])>>4;
		motor->para.p_int=(frame->frame_data[1]<<8)|frame->frame_data[2];
		motor->para.v_int=(frame->frame_data[3]<<4)|(frame->frame_data[4]>>4);
		motor->para.t_int=((frame->frame_data[4]&0xF)<<8)|frame->frame_data[5];
		motor->para.pos = uint_to_fp(motor->para.p_int, motor->type.P_MIN, motor->type.P_MAX, 16);
		motor->para.vel = uint_to_fp(motor->para.v_int, motor->type.V_MIN, motor->type.V_MAX, 12);
		motor->para.tor = uint_to_fp(motor->para.t_int, motor->type.T_MIN, motor->type.T_MAX, 12);
		motor->para.Tmos = (float)(frame->frame_data[6]);
		motor->para.Tcoil = (float)(frame->frame_data[7]);
	}
}

void enable_motor_mode(Joint_Motor_t *motor, uint8_t state)
{
	uint8_t data[8];
	uint16_t id = motor->para.id + motor->mode;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	
	data[7] = state ? 0xFC : 0xFD;
	
	dmCanSend(motor, id, data, 8);
}


void mit_ctrl(Joint_Motor_t *motor, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor->para.id + MIT_MODE;

	pos_tmp = fp_to_uint(pos,  motor->type.P_MIN,  motor->type.P_MAX,  16);
	vel_tmp = fp_to_uint(vel,  motor->type.V_MIN,  motor->type.V_MAX,  12);
	kp_tmp  = fp_to_uint(kp,   motor->type.KP_MIN, motor->type.KP_MAX, 12);
	kd_tmp  = fp_to_uint(kd,   motor->type.KD_MIN, motor->type.KD_MAX, 12);
	tor_tmp = fp_to_uint(torq, motor->type.T_MIN,  motor->type.T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	dmCanSend(motor, id, data, 8);
}

void pos_speed_ctrl(Joint_Motor_t *motor, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor->para.id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	dmCanSend(motor, id, data, 8);
}

void speed_ctrl(Joint_Motor_t *motor, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor->para.id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	dmCanSend(motor, id, data, 4);
}


