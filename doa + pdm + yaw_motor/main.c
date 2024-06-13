#include <stdio.h>
#include "include/board.h"
#include "lib/io.h"
#include "lib/uart.h"
#include "lib/timer.h"
#include "lib/tick.h"
#include "dynamixel.h"
#include <string.h>



#define delay(ms)	timer_wait_ms(_TIM2,ms,NULL)

#define motor_yaw 2
#define motor_pitch 1

#define position_yaw_init 2800
#define position_pitch_init 0

#define position_yaw_max 0
#define position_pitch_max 500


int position_yaw = 0;		//variable globale 
int position_pitch = 0;		//variable globale

int convert_tr_min_to_num(int tr_min)
{
	return (int)(1.1*(tr_min*(256/60))); //1.1 est un coefficient de correction
}

float convert_tr_min_to_deg_sec(int tr_min)
{
	return (tr_min*(360/60));
}


int convert_to_deg(int num)
{
	return (int)((num*360)/4096);
}

int convert_to_num(int deg){
	return (int)((deg*4096)/360);
}

int convert_deg_to_num(int deg){
	return (int)(((deg*4096)/360));
}

void motor_init()
{
	//initialisation et récupération de la position
	dxl_init();
	dxl_set_operating_mode(motor_yaw, POSITION_MODE);
	dxl_set_operating_mode(motor_pitch, POSITION_MODE);
	position_yaw = dxl_get_cur_position(motor_yaw);
	position_pitch = dxl_get_cur_position(motor_pitch);

	//déplacement du pitch pour atteindre la position initiale voulue
	while (position_pitch>1300){
		dxl_set_operating_mode(motor_pitch, VELOCITY_MODE);
		dxl_set_goal_velocity(motor_pitch,100);
		delay(50);
		dxl_set_operating_mode(motor_pitch, POSITION_MODE);
		position_pitch = dxl_get_cur_position(motor_pitch);
	}
	dxl_set_goal_position(motor_pitch, position_pitch_init);

	//déplacement du yaw pour atteindre la position initiale voulue
	while ((position_yaw>800) & (position_yaw<2800)){
		dxl_set_operating_mode(motor_yaw, VELOCITY_MODE);
		dxl_set_goal_velocity(motor_yaw,100);
		delay(50);
		dxl_set_operating_mode(motor_yaw, POSITION_MODE);
		position_yaw = dxl_get_cur_position(motor_yaw);
	}
	dxl_set_goal_position(motor_yaw, position_yaw_init);
}

// déplacement du PITCH en fonction de l'angle fourni en argument 
int move_pitch(int angle){	
	if (((angle>40)&(angle<-40))|(angle==0)){
		return 0;
	}

	//on récupère la position actuelle
	dxl_set_operating_mode(motor_pitch, POSITION_MODE);
	position_pitch = dxl_get_cur_position(motor_pitch);

	int angle_num = convert_to_num(angle);
	int position_pitch_goal = position_pitch-angle_num;
	
	//on s'assure que la position voulu n'est pas au dela que la buté mécanique
	if ((position_pitch_goal<500) & (position_pitch_goal>-700) | ((position_pitch_goal<4496) & (position_pitch_goal>3100))){
		dxl_set_operating_mode(motor_pitch, VELOCITY_MODE);
		//conversion des vitesses de rotation
		int speed_tr_min = 8; 
		if((angle>15)&(angle<-15)){
			speed_tr_min = 20;	
		}
		float speed_deg_sec = convert_tr_min_to_deg_sec(speed_tr_min);
		int speed_num = convert_tr_min_to_num(speed_tr_min);	

		int rotation_duration = (int)(1000*(abs(angle)/speed_deg_sec));	//renvoi une durée en ms

		if (angle<0){		
			dxl_set_goal_velocity(motor_pitch,speed_num);
		}
		else{
			dxl_set_goal_velocity(motor_pitch,-speed_num);
		}
		delay(rotation_duration);
		dxl_set_goal_velocity(motor_pitch,0);
	}
	return 1;
}

// déplacement du YAW en fonction de l'angle fourni en argument 
int move_yaw(int angle){	
	if (((angle>120)&(angle<-120))|(angle==0)){
		return 0;
	}
	//on récupère la position actuelle
	dxl_set_operating_mode(motor_yaw, POSITION_MODE);
	position_yaw = dxl_get_cur_position(motor_yaw);
	int angle_num = convert_to_num(angle);
	int position_yaw_goal = position_yaw + angle_num;

	//on s'assure que la position voulu n'est pas au dela que la buté mécanique
	if ((position_yaw_goal<4096) & (position_yaw_goal>1000)){
		dxl_set_operating_mode(motor_yaw, VELOCITY_MODE);

		//la vitesse s'adapte en fonction du déplacement angulaire a réaliser
		int speed_tr_min = 10; 
		if((angle>30)&(angle<-30)){
			speed_tr_min = 20;	
		}

		//conversion des vitesses de rotation
		float speed_deg_sec = convert_tr_min_to_deg_sec(speed_tr_min);
		int speed_num = convert_tr_min_to_num(speed_tr_min);	
		int rotation_duration = (int)(1000*(abs(angle)/speed_deg_sec));	//renvoi une durée en ms

		if (angle<0){		
			dxl_set_goal_velocity(motor_yaw,-speed_num);
		}
		else{
			dxl_set_goal_velocity(motor_yaw,speed_num);
		}
		delay(rotation_duration);
		dxl_set_goal_velocity(motor_yaw,0);
	}
	return 1;
}


int test()
{
	motor_init();
	move_pitch(-20);
	move_pitch(40);
	move_pitch(-20);
	move_yaw(-20);
	move_yaw(40);
	move_yaw(-20);


	return 0;
}



int chartoint(char caractere){
    return caractere - '0';
}

int power(int value, int power){
    int result = 1;
    for(int i = 0; i<power; i++){
        result*=value;
    }
    return result;
}

int stringtoint(char *string, int len){
    int result = 0;
    for(int i=1; i<len; i++){
        result += chartoint(string[i])*power(10, len-i-1);
    }
    
    if (string[0] == '-'){
        result*=-1;
    }
    return result;
}

int get_pitch(char *originalstring){
    char tmp[5] = "0000";
    memcpy(tmp, originalstring, 4);
    return stringtoint(tmp, 4);
}

int get_yaw(char *originalstring){
    char tmp[5] = "0000";
    memcpy(tmp , originalstring + 4, 4);
    return stringtoint(tmp, 4);
}




int main()
{
	// execution de la fonction test
	//test();

	motor_init();
   	uart_init(_USART2, 115200, UART_8N1);
    USART_t *uart = (USART_t *)USART2;
	char string[9] = "00000000";

   	while(1){
		uart_read(uart,string,8);
		int pitch = get_pitch(string);
		int yaw = get_yaw(string);
		move_pitch(pitch);
		move_yaw(yaw);

		//delay(1000);
	}

}
