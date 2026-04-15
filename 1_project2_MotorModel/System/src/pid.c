#include "pid.h"

#define LIMIT(input, max)		\
do{								\
	if( input > max) {			\
		input = max;			\
	}else if( input < -max) {	\
		input = -max;			\
	}							\
}while(0)


#define OUTLIMIT(input, min)				\
do{											\
	if( input < min && input >= 0) {		\
		input = min;						\
	}else if( input > -min && input < 0) {	\
		input = -min;						\
	}										\
}while(0)



void PID_Init(PID_t *pid, PID_type type, float kp, float ki, float kd,
	float out_max, float out_min, float i_add_time)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_add = 0.0f;
	pid->last_error = 0.0f;
	pid->i_add_max = 100.0f;
	pid->out_max = out_max;
	pid->out_min = out_min;
	pid->i_add_time = i_add_time;
	pid->type = type;
	pid->last_output = 0.0f;	
}



/**
 * @brief 进行一次pid计算
 * @param pid: pid结构体
 * @param error: goal - real 被计算的error值
 * @param dt: 距离上次计算的时间(ms)
 */
float PID_Calc(PID_t *pid, float error, float dt)
{
	float i_change = dt * pid->i_add_max / pid->i_add_time;
	if( error > 0){
		pid->i_add += i_change;
	}else if ( error < 0){
		pid->i_add -= i_change;
	}
	
	LIMIT(pid->i_add, pid->i_add_max);

	float d_error = (error - pid->last_error) / dt;
	pid->last_error = error;

	float output =	pid->kp * error + 
					pid->ki * pid->i_add + 
					pid->kd * d_error;
	if (output >= 0) {
		output += pid->out_min;
	}else{
		output -= pid->out_min;
	}
	
	if( pid->type == ADD_PID) {		// 若为增量式
		output += pid->last_output;	// 则加上上次数值
	}
	
	LIMIT(output, pid->out_max);

	pid->last_output = output;
	return output;
}

