/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void IMU_Init();
void Get_Angle();
void Limit_PWM();
void Modify_Param();
void Check_Bluetooth();
void PWM_Set_Val(TIM_HandleTypeDef *htim, uint32_t Channel, float val);
void Set_PWM(int moto1, int moto2);

int balance(float angle, float gyro);
int velocity(int encoder_left, int encoder_right);
int turn(int encoder_left, int encoder_right, float gyro);

int myabs(int a);
float myabs(float a);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//* Sensors Data
int Encoder_Left, Encoder_Right;							  // encoder counters
float Angle_Balance, Gyro_Balance, Gyro_Turn, Acceleration_Z; // control variables

//* IMU Variables
MPU9250 mpu9250;
uint8_t whoami; //imu id

//* Bluetooth Variables
uint8_t RxBuff[1] = {0};
uint8_t DataBuff[10] = {0};
int RxLine = 0;
uint8_t Recieve_flag = 0;
uint8_t Recieve_val = 0;

//* PWM Variables
int Motor1, Motor2; // relative PWM
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
uint16_t pwm1 = 500;
uint16_t pwm2 = 500;
uint16_t pwm3 = 500;
uint16_t pwm4 = 500;

//* Mechanical Parameters
int ZHONGZHI = 6;

//* PID Parameters
float Target_Velocity = 0, Target_Turn = 0;
float Balance_Kp = 70, Balance_Kd = -3;
float Velocity_Kp = 50, Velocity_Ki = 0.25;
float Turn_Kp = 42, Turn_Kd = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	IMU_Init();
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// update IMU data
		Get_Angle();
		Check_Bluetooth();
		// Calculate PWM
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);
		Turn_Pwm = turn(Encoder_Left, Encoder_Right, Gyro_Turn);
		
		Motor1 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;
		Motor2 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
		
		Limit_PWM(); //===limit maximum PWM to 1000
		Set_PWM(Motor1, Motor2);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
  */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void Check_Bluetooth()
{
	if (Recieve_flag == 1 && RxLine != 0)
	{
		Modify_Param();
		memset(DataBuff, 0, sizeof(DataBuff));
		RxLine = 0;
		Recieve_flag = 0;
	}
}

void Modify_Param()
{
	// no turnning while translation, no translation while turnning
	sscanf((const char *)(&DataBuff[1]), "%d", &Recieve_val);
	switch (DataBuff[0])
	{
	case 'f':
		Target_Velocity = Recieve_val;
		Target_Turn = 0;
		// send in nonblocking mode
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)"forward", 7); 
		break;
	case 'b':
		Target_Velocity = -Recieve_val;
		Target_Turn = 0;
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)"backward", 8);
		break;
	case 'l':
		Target_Turn = Recieve_val;
		Target_Velocity = 0;
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)"left", 4);
		break;
	case 'r':
		Target_Turn = -Recieve_val;
		Target_Velocity = 0;
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)"right", 5);
		break;
	}
	Recieve_val = 0;
}

// bluetooth communication format: f/b/l/r + val + A
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		if (RxBuff[0] == 'A')
		{
			Recieve_flag = 1;
		}
		else
		{
			RxLine++;
			DataBuff[RxLine - 1] = RxBuff[0];
		}
		RxBuff[0] = 0;

		HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim4.Instance)
	{
		//* update encoder data
		Encoder_Left = __HAL_TIM_GET_COUNTER(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		Encoder_Right = __HAL_TIM_GET_COUNTER(&htim1);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
	}
}

int balance(float Angle, float Gyro)
{
	static int Bias, balance;
	// Compute pitch angle bias
	Bias = Angle - ZHONGZHI;
	// Compute PWM
	balance = Balance_Kp * Bias + Gyro * Balance_Kd;
	return balance;
}

int velocity(int encoder_left, int encoder_right)
{
	static int Velocity, Encoder_Least, Encoder;
	static int Encoder_Integral, Encoder_I_max;
	// Compute body velocity bias
	Encoder_Least = (encoder_left+encoder_right)/2*100*60/4/448 - Target_Velocity;
	Encoder *= 0.7;
	Encoder += Encoder_Least * 0.3;
	Encoder_Integral += Encoder;
	// Integral limit
	Encoder_I_max = 1000 / Velocity_Ki;
	if (Encoder_Integral > Encoder_I_max)
		Encoder_Integral = Encoder_I_max;
	if (Encoder_Integral < -Encoder_I_max)
		Encoder_Integral = -Encoder_I_max;
	// Compute PWM
	Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;
	return Velocity;
}

int turn(int encoder_left, int encoder_right, float gyro)
{
	static int Turn, Bias;
	// Compute z-axis angular velocity bias
	Bias = Target_Turn-gyro;
	// Compute PWM
	Turn = Bias * Turn_Kp;
	return Turn;
}

void Set_PWM(int moto1, int moto2)
{
	if (moto1 == 0 && moto2 == 0)
	{
		pwm1 = 0;
		pwm2 = 0;
		pwm3 = 0;
		pwm4 = 0;
		PWM_Set_Val(&htim3, TIM_CHANNEL_1, pwm1);
		PWM_Set_Val(&htim3, TIM_CHANNEL_2, pwm2);
		PWM_Set_Val(&htim3, TIM_CHANNEL_3, pwm3);
		PWM_Set_Val(&htim3, TIM_CHANNEL_4, pwm4);
		return;
	}

	if (moto1 < 0)
	{
		pwm1 = 1000;
		pwm2 = 1000 - myabs(moto1);
	}
	else
	{
		pwm2 = 1000;
		pwm1 = 1000 - myabs(moto1);
	}

	if (moto2 < 0)
	{
		pwm3 = 1000;
		pwm4 = 1000 - myabs(moto2);
	}
	else
	{
		pwm4 = 1000;
		pwm3 = 1000 - myabs(moto2);
	}

	PWM_Set_Val(&htim3, TIM_CHANNEL_1, pwm1);
	PWM_Set_Val(&htim3, TIM_CHANNEL_2, pwm2);
	PWM_Set_Val(&htim3, TIM_CHANNEL_3, pwm3);
	PWM_Set_Val(&htim3, TIM_CHANNEL_4, pwm4);
}

void PWM_Set_Val(TIM_HandleTypeDef *htim, uint32_t Channel, float val)
{
	__HAL_TIM_SET_COMPARE(htim, Channel, val);
}

void Limit_PWM()
{
	int Amplitude = 1000;
	if (Motor1 < -Amplitude)
		Motor1 = -Amplitude;
	if (Motor1 > Amplitude)
		Motor1 = Amplitude;
	if (Motor2 < -Amplitude)
		Motor2 = -Amplitude;
	if (Motor2 > Amplitude)
		Motor2 = Amplitude;
}
void IMU_Init()
{
	whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

	if (whoami == 0x71) //WHO_AM_I should always be 0x68
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)"imu_ok\n", 7);
		HAL_Delay(100);
		mpu9250.resetMPU9250(); //reset registers to default status
		mpu9250.calibrateMPU9250(gyroBias, accelBias);
		HAL_Delay(200);
		mpu9250.initMPU9250();
		mpu9250.initAK8963(magCalibration);
		HAL_Delay(50);
	}
	else
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)"imu_error\n", 10);
		while (1)
			; //loop forever if communication doesn't work
	}
	mpu9250.getAres(); //get accelerometer sensitivity
	mpu9250.getGres(); //get gyro sensitivity
	mpu9250.getMres(); //get magnetometer sensitivity

	magbias[0] = +470.; //user environmental x-axis
	magbias[1] = +120.; //user environmental x-axis
	magbias[2] = +125.; //user environmental x-axis
}
void Get_Angle()
{
	if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) //on interrupt, check if data ready interrupt
	{
		mpu9250.readAccelData(accelCount); //read the x/y/z adc values

		// now we need to calculate the acceleration value into actual value
		ax = (float)accelCount[0] * aRes;
		ay = (float)accelCount[1] * aRes;
		az = (float)accelCount[2] * aRes;

		mpu9250.readGyroData(gyroCount); //read the x/y/z adc values

		//calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0] * gRes;
		gy = (float)gyroCount[1] * gRes;
		gz = (float)gyroCount[2] * gRes;

		mpu9250.readMagData(magCount); //read the x/y/z adc values

		//calculate the magnetometer values in milliGuass
		mx = (float)magCount[0] * mRes * magCalibration[0];
		my = (float)magCount[1] * mRes * magCalibration[1];
		mz = (float)magCount[2] * mRes * magCalibration[2];

		deltat = 0.005;
		mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, mx, my, mz);

		yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

		//tranform into degrees unit
		pitch *= 180.0f / PI;
		yaw *= 180.0f / PI;
		yaw -= 3.0f; //Declination at Danville, california is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll *= 180.0f / PI;
	}

	Gyro_Balance = -gy;
	Angle_Balance = roll;
	Gyro_Turn = gz;
	Acceleration_Z = az;
}

int myabs(int a)
{
	int temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

float myabs(float a)
{
	float temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
