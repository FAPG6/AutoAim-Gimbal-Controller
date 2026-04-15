#ifndef __DM_DRV_H__
#define __DM_DRV_H__
#include "main.h"
#include "can_bsp.h"



typedef enum DMmotorMode{
    MIT_MODE = 0x000,
    POS_MODE = 0x100,
    SPEED_MODE = 0x200,
}DMmotorMode;


typedef struct motorType {
    float P_MIN;
    float P_MAX;
    float V_MIN;
    float V_MAX;
    float KP_MIN;
    float KP_MAX;
    float KD_MIN;
    float KD_MAX;
    float T_MIN;
    float T_MAX;
} motorType;


typedef struct motor_fbpara_t{
    uint16_t id;
	uint16_t recvId;
    uint16_t state;

    int p_int;
    int v_int;
    int t_int;

    float pos;
    float vel;
    float tor;

    float Tmos;
    float Tcoil;
}motor_fbpara_t;


typedef struct Joint_Motor_t{
    uint16_t mode;
    motor_fbpara_t para;
    motorType type;
	FDCAN_HandleTypeDef *hfdcan;

}Joint_Motor_t ;


extern motorType DM6220;
extern motorType DM4310;

// 功能
void dmCallback(Joint_Motor_t *motor, canFrame* frame);
void enable_motor_mode(Joint_Motor_t *motor, uint8_t state);

// 控制
void mit_ctrl(Joint_Motor_t *motor, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(Joint_Motor_t *motor, float pos, float vel);
void speed_ctrl(Joint_Motor_t *motor, float vel);

// 初始化
void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t recvId, DMmotorMode mode, FDCAN_HandleTypeDef *hfdcan, motorType type);

    

#endif /* __DM_DRV_H__ */

