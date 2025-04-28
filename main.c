#include "at32f425_board.h"
#include "at32f425_clock.h"
#include "spi_flash.h"
#include "sfud.h"
#include "sensor.h"
#include "cantest.h"
#include "rs485.h"
#include "uart.h"
#include "string.h"
#include "wdt.h"
#include "flashtest.h"
#include "systick.h"
#include "can_iap_user.h"
#include "SETTIME.h"
#include "handset.h"
#include "LISDH12.h"
#include "SPL06.h"
#include "TMF8001.h"
#include "acctest.h"
#include "kalman_filter.h"
#include "math.h"
#include "altitude.h"
#define PRESSURE_QUEUE_COUNT 30
#define PRESURE_INITIAL_ESTIMATE 100788.0
#define PRESURE_INITIAL_ERROR 1000.0
#define ACCELERATE_INITIAL_ESTIMATE 9.8
#define ACCELERATE_INITIAL_ERROR 0.1
#define THRESHOLD_ACCELERATE 0.2
#define ELEVATOR_STATE_UP 1
#define ELEVATOR_STATE_DOWN -1
#define ELEVATOR_STATE_STATIONARY -2
#define ELEVATOR_STATE_UNKNOWN 2000

#define ELEVATOR_MOVE_UNKNOWN 2000
#define ELEVATOR_MOVE_MOVING 1000
#define THRESHOLD_SPEED 0.05

#define STATE_SIGN_NEGATIVE -1
#define STATE_SIGN_POSITIVE 1
#define STATE_SIGN_ZERO 0
//#define BASE_PRESSURE 101540
const float BASE_PRESSURE=102687.05372053712;
static const uint8_t HISTORY_SIZE = 48;

int accelerate_sign = STATE_SIGN_ZERO;
int last_accelerate_sign = STATE_SIGN_ZERO;
int accelerate_state = ELEVATOR_MOVE_UNKNOWN;
int accelerate_moving_state = ELEVATOR_MOVE_UNKNOWN;
extern can_rx_message_type rx_message_struct;
extern float accelThreshold;
extern float sigmaAccel;
extern float sigmaGyro;
extern float sigmaBaro;
extern float ca;
extern uint16_t timestamp[2];
extern uint16_t tick_count[2];
extern unsigned char is_reset;
static float groundAltitude = 0;
static float groundPressure = 0;
static float pressureSum = 0;
static float history[HISTORY_SIZE];
static uint8_t historyIdx = 0;
static uint32_t endCalibration = 120;
typedef struct {
	float altitude;
	float dt;
} ALTITUDE_DATA;
float calculateAltitude(float pressure)
{
	float altitude = 0;
	//altitude = (44330.0 * (1.0 - pow((float)(pressure) / BASE_PRESSURE, 1.0 / 5.255)));
	altitude = 8500 * log(BASE_PRESSURE / pressure);
	return altitude;
}

int get_accelerate_sign(float baseValue, float data)
{
	float result = data - baseValue;
	if (fabs(result) > THRESHOLD_ACCELERATE)
	{
		if (result > 0)
		{
			return STATE_SIGN_POSITIVE;
		}
		if (result < 0)
		{
			return STATE_SIGN_NEGATIVE;
		}
	}
	return STATE_SIGN_ZERO;
}
int get_accelerate_status(int last_state, float baseValue, float data)
{
	float delta = 0;
	float average_data = 0;
	delta = data - baseValue;
	int state = last_state;
	accelerate_sign = get_accelerate_sign(baseValue, data);
	if (accelerate_sign != last_accelerate_sign)
	{
		if (accelerate_moving_state == ELEVATOR_MOVE_UNKNOWN)
		{
			if (accelerate_sign != STATE_SIGN_ZERO)
			{
				accelerate_moving_state = ELEVATOR_MOVE_MOVING;
				if (accelerate_sign == STATE_SIGN_POSITIVE)
				{
					state = ELEVATOR_STATE_UP;
				}
				else if (accelerate_sign == STATE_SIGN_NEGATIVE)
				{
					state = ELEVATOR_STATE_DOWN;
				}
			}
		}
		else if (accelerate_moving_state == ELEVATOR_MOVE_MOVING)
		{
			if (accelerate_sign != STATE_SIGN_ZERO)
			{
				accelerate_moving_state = ELEVATOR_MOVE_UNKNOWN;
				state = ELEVATOR_STATE_STATIONARY;
			}
		}
		else if (accelerate_sign == STATE_SIGN_ZERO)
		{
			if (accelerate_moving_state != ELEVATOR_MOVE_MOVING)
			{
				state = ELEVATOR_STATE_STATIONARY;
			}
		}
	}

	last_accelerate_sign = accelerate_sign;
	return state;
}
void init_device()
{
	system_clock_config();
	at32_board_init();
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	usart_configuration(115200);
	TMF8001_I2C_Init();
	Lis2dh12_Init();
	SysTime1Init();
	iap_init();
	if (spl06_init() == 0)
	{
		printf("SPL06 init 0k\r\n");
	}
	delay_ms(100);
	for (uint8_t i = 0; i < 100; i++)
	{
		Collect_Orignal_Data();
	}
}
float get_micros(void)
{
	uint32_t ms = tick_count[0];
	uint32_t ms_ticks = SysTick->VAL;
	if (ms_ticks < (SystemCoreClock / 1000000.0 - 1000.0))
	{
		ms++;
	}
	return ms * 1000.0 + (SystemCoreClock / 1000000.0 - 1.0 - ms_ticks) / (SystemCoreClock / 1000000000.0);
}
static float pa2mbar(float pressurePa)
{
	return pressurePa * 1013.25;
}
static float millibarsToMeters(float mbar)
{
	return (1.0f - powf(mbar / 1013.25f, 0.190295f)) * 44330.0f;
}
static void calibrate(float pressure)
{
	history[historyIdx] = pa2mbar(pressure);
	pressureSum += pa2mbar(pressure);
	uint8_t nextIndex = (historyIdx + 1) % HISTORY_SIZE;
	pressureSum -= history[nextIndex];
	historyIdx = nextIndex;
	groundPressure -= groundPressure / 8;
	groundPressure += pressureSum / (HISTORY_SIZE - 1);
	groundAltitude = millibarsToMeters(groundPressure / 8);
}

static float getAltitudeFromPressure(float pressure)
{
	return (millibarsToMeters(pa2mbar(pressure - BASE_PRESSURE)) - groundAltitude);
}
void setup()
{
	uint32_t count = 0;
	uint32_t pressure;
	uint32_t temperature;
	spl06_update(&temperature, &Air_Pressure);
	Calibrate_Other();
	calibrate(Air_Pressure);
	for (uint8_t k = 0; k < HISTORY_SIZE; ++k)
	{
		history[k] = 0;
	}
	while (count < endCalibration)
	{
		spl06_update(&temperature, &pressure);
		calibrate(pressure);
		count++;
	}
}

float getAvarageSpeed(ALTITUDE_DATA *data){
	float averageSpeed = 0;
	int counter=0;
	int isSlow =0;
	for(int i =9;i>0;i--){		
		if(fabs(data[i-1].altitude)>0.00001){
			counter++;
			averageSpeed = averageSpeed + fabs(data[i].altitude - data[i-1].altitude) * data[i].dt;		
			if(fabs(data[i].altitude-data[i-1].altitude)<0.01){
				 isSlow++;
				//printf("---- slow speed %d -----",isSlow);
			}
		}		
	}
	if(isSlow>8){
		 averageSpeed = (data[9].altitude-data[0].altitude)/2;    
     isSlow =0;
	}
	else{
		if(averageSpeed>0.0001){
			averageSpeed =  ((data[9].altitude-data[8].altitude)/data[9].dt+(data[8].altitude-data[7].altitude)/data[8].dt+(data[7].altitude-data[6].altitude)/data[7].dt)/3; 
			//printf("---- last 3 speed %.4f -----",averageSpeed);
			counter = 0;
		}
		else{
			averageSpeed=0;
		}
	}
	
	return fabs(averageSpeed);
	
}
void addAltitudeToQueue(ALTITUDE_DATA *dataQueue,ALTITUDE_DATA data){
	  if(dataQueue[9].altitude<0.0001)
		{		
			dataQueue[9].altitude=data.altitude;
			dataQueue[9].dt=data.dt;
		}
		else{
			for(int i =0;i<9;++i){
				dataQueue[i].altitude=dataQueue[i+1].altitude;
				dataQueue[i].dt=dataQueue[i+1].dt;
			}
			dataQueue[9].altitude=data.altitude;
			dataQueue[9].dt=data.dt;		
		}
		
}
int main(void)
{
	float acc_value = 0;
	float last_accelerate = 0;
	float last_altitude = 0;
	float speed = 0;
	double altitude = 0;
	float acc_base_value = 0;
	int state = ELEVATOR_STATE_UNKNOWN;
	int last_state = ELEVATOR_STATE_UNKNOWN;
	int counter = 0;
	float timer = get_micros();
	float dt = 0;
	float accelData[3];
	float speedData[10]={0};
	ALTITUDE_DATA altitude_data[10]={{.altitude=0,.dt=0},{.altitude=0,.dt=0},
																	 {.altitude=0,.dt=0},{.altitude=0,.dt=0},
																	{.altitude=0,.dt=0},{.altitude=0,.dt=0},{.altitude=0,.dt=0},
																	{.altitude=0,.dt=0},{.altitude=0,.dt=0},{.altitude=0,.dt=0}};
	ALTITUDE_DATA altitude_data_temp={.altitude=0,.dt=0};
	int speedDataCounter = 0;
	KalmanFilter altitude_kf, acceleration_kf;
  kalman_init(&altitude_kf);
  kalman_init(&acceleration_kf);
	init_device();
	setup();
	spl06_update(&temperature, &Air_Pressure);
	Collect_Orignal_Data();
	altitude = calculateAltitude(Air_Pressure);
	
	for(int i=0;i<10;i++){
		altitude_data[i].altitude=altitude;
		altitude_data[i].dt =  (get_micros() - timer) / 1000 / 1000;
	}
	accelData[0] = acc_average_value(collect_data.x, COLLECT_ACC_TIMES);
	accelData[1] = acc_average_value(collect_data.y, COLLECT_ACC_TIMES);
	accelData[2] = acc_average_value(collect_data.z, COLLECT_ACC_TIMES);
	acc_value = accelData[2] * 9.8;
	acc_base_value = acc_value;
	last_altitude = altitude;
	last_accelerate = acc_value;
	while (1)
	{
		if (delay_100tick[0] > delay_100tick[1])
		{
			spl06_update(&temperature, &Air_Pressure);
			Collect_Orignal_Data();
			altitude = calculateAltitude(Air_Pressure);
			float estimated_altitude = kalman_update(&altitude_kf,altitude);
			dt = (get_micros() - timer) / 1000 / 1000;
			accelData[0] = acc_average_value(collect_data.x, COLLECT_ACC_TIMES);
			accelData[1] = acc_average_value(collect_data.y, COLLECT_ACC_TIMES);
			accelData[2] = acc_average_value(collect_data.z, COLLECT_ACC_TIMES);
			acc_value = accelData[2] * 9.8;
			altitude_data_temp.altitude=estimated_altitude;
			altitude_data_temp.dt = dt;
			addAltitudeToQueue(&altitude_data[0],altitude_data_temp);
			speed = getAvarageSpeed(&altitude_data[0]);
			if (speed < THRESHOLD_SPEED)
			{
				counter++;
				if (counter >= 5)
				{
					state = ELEVATOR_STATE_STATIONARY;
					last_state = ELEVATOR_STATE_STATIONARY;
					accelerate_state = ELEVATOR_MOVE_UNKNOWN;
					counter = 0;
				}
			}
			//speedData[speedDataCounter]=speed;
			//speed = (speedData[0]+speedData[1]+speedData[2])/3.0;
			state = get_accelerate_status(last_state, acc_base_value, acc_value);
			/**/
			if (state == ELEVATOR_STATE_UP)
			{
				printf("UP  ,%.6f, %.2f, %.2f, %.2f, %.2f, %.2f\n", timer,dt, estimated_altitude, last_altitude, speed,acc_value);
			}
			else if (state == ELEVATOR_STATE_DOWN)
			{
				printf("DOWN,%.6f,%.2f, %.2f, %.2f, %.2f, %.2f\n",timer, dt, estimated_altitude, last_altitude, speed,acc_value);
			}
			else if (state == ELEVATOR_STATE_STATIONARY)
			{
				speed = 0;
				printf("STAT,%.6f,%.2f, %.2f, %.2f, %.2f, %.2f\n",timer, dt, estimated_altitude, last_altitude, speed,acc_value);
			}
			else
			{
				printf("UNKN,%.6f,%.2f, %.2f, %.2f, %.2f, %.2f\n",timer, dt, estimated_altitude, last_altitude, speed,acc_value);
			}
			/**/
			speedDataCounter ++;
			if(speedDataCounter>=125){
				speedDataCounter=0;
			}
			delay_100tick[0] = 0;
			last_state = state;
			last_accelerate = acc_value;
			last_altitude = estimated_altitude;
			timer = get_micros();
		}
	}
}
