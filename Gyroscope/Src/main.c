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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <math.h>

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
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */




uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
float getMres();

#define MPU9250_ADDRESS 0x68// Device address when ADO = 0
#define AK8963_ADDRESS   0x0C


#define WHO_AM_I 0x75 // Should return 0x71
#define MPU9250_WHOAMI_DEFAULT_VALUE 0x71
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define MPU_CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define PWR_MGMT_2 0x6C
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define USER_CTRL 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define FIFO_COUNTH 0x72
#define FIFO_R_W 0x74
#define XG_OFFSET_H 0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define INT_STATUS 0x3A


#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_WHOAMI_DEFAULT_VALUE 0x48
#define AK8963_CNTL 0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX 0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY 0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12  // Fuse ROM z-axis sensitivity adjustment value


#define GFSSEL 0x00 //0 - ± 250°/s; 1 - ± 500°/s; 2 - ±1000°/s; ± 2000°/s 
#define AFSSEL 0x00 //0 - ±2g; 1 - ±4g; 2 - ±8g; 3 - ±16g;

#define magnetic_declination 11.3

#define Mmode 0x06 // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer
#define AK8963_ST1 0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data

float mRes = 10. * 4912. / 32760.0; //MFSSEL: 0 - 10. * 4912. / 8190.0; 1 - 10. * 4912. / 32760.0;
float aRes = 2.0 / 32768.0; //±2g - 2.0 / 32768.0; ±4g - 4.0 / 32768.0; ±8g - 8.0 / 32768.0;±16g - 16.0 / 32768.0
float gRes = 250.0 / 32768.0; // ± 250°/s - 250.0 / 32768.0; ± 500°/s - 500.0 / 32768.0; ± 1000°/s - 1000.0 / 32768.0; ± 2000°/s - 2000.0 / 32768.0;

float magCalibration[3] = {0, 0, 0}; //factory mag calibration
float magBias[3] = {158, 134, -344};
float magScale[3]  = {0.98636, 0.9344, 1.12148}; // Bias corrections for gyro and accelerometer

float gyroBias[3] = {-2.00, -0.2, -1.1}; // bias corrections
float accelBias[3] = {0, 0, 0}; // bias corrections

int16_t tempCount;      // temperature raw count output
float temperature;    // Stores the real internal chip temperature in degrees Celsius
float SelfTestResult[6];    // holds results of gyro and accelerometer self test

int flagStartRead = 1;
float a[3], g[3], m[3];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float pitch, yaw, roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)


#define modeMedianAndKalman 0
const int medianVal = 3;
const float kalmanVal = 0.5;
float axMedian[medianVal], ayMedian[medianVal], azMedian[medianVal];
float gxMedian[medianVal], gyMedian[medianVal], gzMedian[medianVal];
float mxMedian[medianVal], myMedian[medianVal], mzMedian[medianVal];
int iterMedian = 0;



const double PI = 3.14159265358979f;
const float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
const float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta;   // compute beta
float zeta;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
const float Kp = 2.0f * 5.0f; // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
const float Ki = 0.0f;

float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

// for mahony only
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


int flagMPU = 0;
char bufUsb[127];
int b_ahrs = 1;
int flagPrint = 0;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void qsortRecursive(float *mas, int size) 
{
	int i = 0;
	int j = size - 1;
	float mid = mas[size / 2];

	do 
	{
		
			while(mas[i] < mid)
			{
				i++;
			}

			while(mas[j] > mid) 
			{
				j--;
			}

			if (i <= j) 
			{
					float tmp = mas[i];
					mas[i] = mas[j];
					mas[j] = tmp;

					i++;
					j--;
			}
	} 
	while (i <= j);

	if(j > 0) 
	{
		qsortRecursive(mas, j + 1);
	}
	if (i < size)
	{
		qsortRecursive(&mas[i], size - i);
	}
}


float medianMas(float *mas, int size)
{
	qsortRecursive(mas,size);
	return mas[size/2];
}

void valInit(void)
{
	beta =  sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
 zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
}

void usbSend(char* str_tx)
{
	HAL_GPIO_WritePin(blueLed_GPIO_Port,blueLed_Pin,GPIO_PIN_SET);
	CDC_Transmit_FS((unsigned char*)str_tx, strlen(str_tx));
	HAL_Delay(1);
	HAL_GPIO_WritePin(blueLed_GPIO_Port,blueLed_Pin,GPIO_PIN_RESET);

}

void qFilterUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q)
{
	// updateParams()
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;
	gx *= PI / 180.f;
	gy *= PI / 180.f;
	gz *= PI / 180.f;

	// updateTime()
	Now = HAL_GetTick();
	deltat = ((Now - lastUpdate) / 1000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
	
	
}
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q)
    {
        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        float norm;
        float hx, hy, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
        float pa, pb, pc;

        // Auxiliary variables to avoid repeated arithmetic
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrtf(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
        hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
        bx = sqrtf((hx * hx) + (hy * hy));
        bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

        // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q2q4 - q1q3);
        vy = 2.0f * (q1q2 + q3q4);
        vz = q1q1 - q2q2 - q3q3 + q4q4;
        wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
        wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
        wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if (Ki > 0.0f)
        {
            eInt[0] += ex;      // accumulate integral error
            eInt[1] += ey;
            eInt[2] += ez;
        }
        else
        {
            eInt[0] = 0.0f;     // prevent integral wind up
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }

        // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

        // Normalise quaternion
        norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;
    }

void MPU9250Write(uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
	
	if (HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS<<1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 1000) != HAL_OK)
	{
		sprintf(bufUsb, "I2C write error: %d\r\n",MemAddress);
	}
}

void MPU9250Read(uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
	
	if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS<<1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 1000) !=HAL_OK)
	{
			sprintf(bufUsb, "I2C read error: %d\r\n", MemAddress);
	}
}

void AK8963Write(uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
	
	if (HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS<<1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 1000) != HAL_OK)
	{
		sprintf(bufUsb, "I2C write error: %d\r\n",MemAddress);
	}
}

void AK8963Read(uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
	
	if (HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS<<1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 1000) !=HAL_OK)
	{
			sprintf(bufUsb, "I2C read error: %d\r\n", MemAddress);
	}
}

int isConnectedMPU9250(void)
{
	
	uint8_t c[1];
	MPU9250Read(WHO_AM_I, c, 1);
	
	//sprintf(bufUsb,"MPU9250 WHO AM I = 0x%02X\r\n", c[0]);
	//usbSend(bufUsb);
	if (c[0] == (uint8_t) MPU9250_WHOAMI_DEFAULT_VALUE)
	{
		flagMPU = 1;
		return 1;
	}
	else
	{
		flagMPU = 0;
		return 0;
	}
	
}

void MPUInit(void)
{	

	uint8_t pwrOff[1]={0x00};
	uint8_t pwrOn[1]={0x01};
	uint8_t mpuConfigVal[1]={0x03};
	uint8_t smplrtDivVal[1]={0x04};
	uint8_t intPinCfgVal[1]={0x22};
	uint8_t intEnableVal[1]={0x01};
	
	// wake up device
	MPU9250Write(PWR_MGMT_1, pwrOff, 1);
	HAL_Delay(100);// Wait for all registers to reset
	
	// get stable time source
	MPU9250Write(PWR_MGMT_1, pwrOn, 1);
	HAL_Delay(200);
	
	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	MPU9250Write(MPU_CONFIG, mpuConfigVal, 1);
	
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  MPU9250Write(SMPLRT_DIV, smplrtDivVal, 1); // Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above
	
	// Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c[1];
	MPU9250Read(GYRO_CONFIG, c,1); // get current GYRO_CONFIG register value
	// c &= ~0xE0; // Clear self-test bits [7:5]
	c[0] &= ~0x03; // Clear Fchoice bits [1:0]
  c[0] &= ~0x18; // Clear GFS bits [4:3]
  c[0] |= (uint8_t)GFSSEL << 3; // Set full scale range for the gyro
	// c |= 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	MPU9250Write(GYRO_CONFIG, c,1); // Write new GYRO_CONFIG value to register
  
	// Set accelerometer full-scale range configuration
	MPU9250Read(ACCEL_CONFIG, c,1); // get current ACCEL_CONFIG register value
	// c &= ~0xE0; // Clear self-test bits [7:5]
  c[0] &= ~0x18;  // Clear AFS bits [4:3]
  c[0] |= (uint8_t) AFSSEL << 3; // Set full scale range for the accelerometer
	MPU9250Write(ACCEL_CONFIG, c,1); // Write new ACCEL_CONFIG register value
	
	MPU9250Read(ACCEL_CONFIG2,c,1); // get current ACCEL_CONFIG2 register value
	c[0] &= ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c[0] |= 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	MPU9250Write(ACCEL_CONFIG2,c,1); // Write new ACCEL_CONFIG2 register value  
	
  
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	MPU9250Write(INT_PIN_CFG, intPinCfgVal, 1);
	MPU9250Write(INT_ENABLE, intEnableVal, 1);	// Enable data ready (bit 0) interrupt
	HAL_Delay(100);      
	
}



int isConnectedAK8963(void)
{
	
	uint8_t c[1];
	AK8963Read(AK8963_WHO_AM_I, c,1);
	
	//sprintf(bufUsb, "AK8963 WHO AM I =  %02X\r\n",c[0]);
	//usbSend(bufUsb);
	if (c[0] == (uint8_t) AK8963_WHOAMI_DEFAULT_VALUE)
	{
		flagMPU = 3;
		return 3;
	}
	else
	{
		flagMPU = 0;
		return 0;
	}
	
}

void AK8963Init(float* destination)
{
	
	uint8_t rawData[3];
	uint8_t AK8963Off[1]={0x00};
	uint8_t AK8963AccMod[1]={0x0F};
	uint8_t AK8963AcqMod[1]={0x16};
	
	
	AK8963Write(AK8963_CNTL, &AK8963Off[0], 1);// Power down magnetometer
	HAL_Delay(10);
	
	AK8963Write(AK8963_CNTL, &AK8963AccMod[0], 1);// Enter Fuse ROM access mode
	HAL_Delay(10);
	
	// Read the x-, y-, and z-axis calibration values
	AK8963Read(AK8963_ASAX, &rawData[0], 3);
	
	destination[0]= (float) (rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
	destination[1]= (float) (rawData[1] - 128) / 256. + 1.; // Return y-axis sensitivity adjustment values, etc.
	destination[2]= (float) (rawData[2] - 128) / 256. + 1.; // Return z-axis sensitivity adjustment values, etc.
	
	// Power down magnetometer
	AK8963Read(AK8963_CNTL, &AK8963Off[0],1);
	HAL_Delay(10);
	
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	AK8963Write(AK8963_CNTL, &AK8963AcqMod[0], 1);
	HAL_Delay(10);
	
	usbSend("Calibration value:\r\n");
	sprintf(bufUsb, "X-Axis sensitivity adjustment value: %.4f \r\n", destination[0]);
	usbSend(bufUsb);
	sprintf(bufUsb, "Y-Axis sensitivity adjustment value: %.4f \r\n", destination[1]);
	usbSend(bufUsb);
	sprintf(bufUsb, "Z-Axis sensitivity adjustment value: %.4f \r\n", destination[2]);
	usbSend(bufUsb);
	sprintf(bufUsb, "X-Axis sensitivity offset value: %.4f \r\n", magBias[0]);
	usbSend(bufUsb);
	sprintf(bufUsb, "Y-Axis sensitivity offset value: %.4f \r\n", magBias[1]);
	usbSend(bufUsb);
	sprintf(bufUsb, "Z-Axis sensitivity offset value: %.4f \r\n", magBias[2]);
	usbSend(bufUsb);
	
}
void MPUSetup()
{
	int m_whoami = 0;
	int a_whoami = 0;
	
	m_whoami = isConnectedMPU9250();
	
	if (m_whoami == 1)
	{
		usbSend("MPU9250 is online...\r\n");
		
		MPUInit();
		
		a_whoami = isConnectedAK8963();
		
		if (a_whoami)
		{
			
			AK8963Init(magCalibration);
		}
		else
		{
			
			usbSend("Could not connect to AK8963: 0x");
			sprintf(bufUsb, "%d\r\n",a_whoami);
			usbSend(bufUsb);
			
		}
	}
	else
	{
		
		sprintf(bufUsb, "Could not connect to MPU9250: 0x%d\r\n", m_whoami);
		usbSend(bufUsb);
		
	}
}




// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.    
void calibrateMPU9250(float* dest1,float* dest2)
{
	
	uint8_t resetDevice[1] = {0x80};
	uint8_t powerOn[1] = {0x01};
	uint8_t powerOff[1] = {0x00};
	uint8_t resetFIFOandDMP[1] = {0x0C};
	uint8_t fifoEnable[1] = {0x40};
	uint8_t gyroAndAccelerometrEnable[1] = {0x78};
	
	uint8_t data[12]; //data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0,0,0}, accel_bias[3] = {0,0,0};
	
	// reset device
	MPU9250Write(PWR_MGMT_1, resetDevice, 1);
	HAL_Delay(100);
	
	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
	MPU9250Write(PWR_MGMT_1, powerOn, 1);
	MPU9250Write(PWR_MGMT_2, powerOff, 1);
	HAL_Delay(200);
	
	// Configure device for bias calculation
	MPU9250Write(INT_ENABLE, powerOff, 1); // Disable all interrupts
	MPU9250Write(FIFO_EN, powerOff, 1); // Disable FIFO
	MPU9250Write(PWR_MGMT_1, powerOff, 1); // Turn on internal clock source
	MPU9250Write(I2C_MST_CTRL, powerOff, 1); // Disable I2C master
	MPU9250Write(USER_CTRL, powerOff, 1); // Disable FIFO and I2C master modes
	MPU9250Write(USER_CTRL, resetFIFOandDMP, 1); // Reset FIFO and DMP
	HAL_Delay(15);
	
	// Configure MPU6050 gyro and accelerometer for bias calculation
	MPU9250Write(MPU_CONFIG, powerOn, 1); // Set low-pass filter to 188 Hz
	MPU9250Write(SMPLRT_DIV, powerOff, 1); // Set sample rate to 1 kHz
	MPU9250Write(GYRO_CONFIG, powerOff, 1); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU9250Write(ACCEL_CONFIG, powerOff, 1); // Set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
	
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU9250Write(USER_CTRL, fifoEnable, 1); // Enable FIFO
	MPU9250Write(FIFO_EN, gyroAndAccelerometrEnable, 1); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
	
	// At end of sample accumulation, turn off FIFO sensor read
	MPU9250Write(FIFO_EN, powerOff, 1); // Disable gyro and accelerometer sensors for FIFO
	MPU9250Read(FIFO_COUNTH, &data[0], 2); // read FIFO sample count
	fifo_count = ( (uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging
	
	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		MPU9250Read(FIFO_R_W, &data[0], 12); // read data for averaging
		
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}
	
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) 
	{
		accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
	}  
  else 
	{
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4)       & 0xFF;
	data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4)       & 0xFF;

	// Push gyro biases to hardware registers
  MPU9250Write(XG_OFFSET_H, &data[0], 1);
	MPU9250Write(XG_OFFSET_L, &data[1], 1);
	MPU9250Write(YG_OFFSET_H, &data[2], 1);
	MPU9250Write(YG_OFFSET_L, &data[3], 1);
	MPU9250Write(ZG_OFFSET_H, &data[4], 1);
	MPU9250Write(ZG_OFFSET_L, &data[5], 1);
	
	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	/* int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
		 readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
		 accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	   readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	   accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	   readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	   accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	   uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	   uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	   for(ii = 0; ii < 3; ii++) {
	       if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	   }

	   Construct total accelerometer bias, including calculated average accelerometer bias from above
	   accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	   accel_bias_reg[1] -= (accel_bias[1] / 8);
	   accel_bias_reg[2] -= (accel_bias[2] / 8);

	   data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	   data[1] = (accel_bias_reg[0])      & 0xFF;
	   data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	   data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	   data[3] = (accel_bias_reg[1])      & 0xFF;
	   data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	   data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	   data[5] = (accel_bias_reg[2])      & 0xFF;
	   data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	   Apparently this is not working for the acceleration biases in the MPU-9250
	   Are we handling the temperature correction bit properly?
	   Push accelerometer biases to hardware registers
	   writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	   writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	   writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	   writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	   writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	   writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);*/

	   //Output scaled accelerometer biases for display in the main program
		dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

		usbSend("MPU9250 bias\r\n");
    usbSend(" x   y   z  \r\n");
		sprintf(bufUsb, "%d %d %d mg\r\n", (int)(1000 * accelBias[0]), (int)(1000 * accelBias[1]), (int)(1000 * accelBias[2]));
		usbSend(bufUsb);
		sprintf(bufUsb, "%f %f %f o/s\r\n", gyroBias[0],gyroBias[1],gyroBias[2]);
		usbSend(bufUsb);
		
   HAL_Delay(100);
	 MPUInit();
	 HAL_Delay(1000);

	
}

void calibrateAccelGyro(void)
{
	calibrateMPU9250(gyroBias, accelBias);
}

void readMagData(int16_t* destination)
{
	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t AK8963STVal[1];
	
	AK8963Read(AK8963_ST1, &AK8963STVal[0], 1);
	
	if (AK8963STVal[0] && 0x01) // wait for magnetometer data ready bit to be set
	{
		AK8963Read(AK8963_XOUT_L, &rawData[0], 7); // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c &0x08)) // Check if magnetic sensor overflow set, if not then report data
		{
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}

}
void magcalMPU9250(float *dest1, float *dest2)
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
		
	usbSend("Mag Calibration: Wave device in a figure eight until done!\r\n");
	HAL_Delay(4000);
	
	// shoot for ~fifteen seconds of mag data
  if (Mmode == 0x02)
	{		
		sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	}
	else 
	{
		if (Mmode == 0x06) 
		{
			sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
		}
	}
	
	for(ii = 0; ii < sample_count; ii++)
	{
			readMagData(mag_temp);  // Read the mag data
			for (int jj = 0; jj < 3; jj++)
			{
					if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
					if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
			if(Mmode == 0x02) 
			{
				HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
			}
			if(Mmode == 0x06)
			{
				HAL_Delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
			}
	}
	
	sprintf(bufUsb, "mag x min/max: %d/%d\r\n", mag_min[0], mag_max[0]);
	usbSend(bufUsb);
	sprintf(bufUsb, "mag y min/max: %d/%d\r\n", mag_min[1], mag_max[1]);
	usbSend(bufUsb);
	sprintf(bufUsb, "mag z min/max: %d/%d\r\n", mag_min[2], mag_max[2]);
	usbSend(bufUsb);
	
	if ((mag_min[0] == mag_max[0]) || (mag_min[1] == mag_max[1]) || (mag_min[2] == mag_max[2]))
	{
		HAL_GPIO_WritePin(orangeLed_GPIO_Port,orangeLed_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(redLed_GPIO_Port,redLed_Pin,GPIO_PIN_SET);
	}
	else
	{
	
		// Get hard iron correction
		mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
		mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
		mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

		dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
		dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
		dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

		// Get soft iron correction estimate
		mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
		mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
		mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

		float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
		avg_rad /= 3.0;

		dest2[0] = avg_rad/((float)mag_scale[0]);
		dest2[1] = avg_rad/((float)mag_scale[1]);
		dest2[2] = avg_rad/((float)mag_scale[2]);
		
		usbSend("Mag Calibration done!\r\n");
		usbSend("AK8963 magbiases (mG)\r\n");
		sprintf(bufUsb, "%f, %f, %f\r\n", magBias[0], magBias[1], magBias[2]);
		usbSend(bufUsb);
		usbSend("AK8963 mag scale (mG)\r\n");
		sprintf(bufUsb, "%f, %f, %f\r\n", magScale[0], magScale[1], magScale[2]);
		usbSend(bufUsb);
		
		
	}

}
	

void calibrateMag(void)
{
	magcalMPU9250(magBias, magScale);
}
void readMPU9250Data(int16_t* destination)
{
	
	uint8_t rawData[14]; // x/y/z accel register data stored here
	MPU9250Read(ACCEL_XOUT_H, &rawData[0], 14); // Read the 14 raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;

}

int16_t readTempData(void)
{
	
	uint8_t rawData[2];// x/y/z gyro register data stored here
	MPU9250Read(TEMP_OUT_H, &rawData[0], 2);
	return ((int16_t)rawData[0] << 8) | rawData[1] ;
	
}

void updateAccelGyro(void)
{
	int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
	readMPU9250Data(MPU9250Data);// INT cleared on any read
	
	float aNew[3], gNew[3];
	
	// Now we'll calculate the accleration value into actual g's
	aNew[0] = (float)MPU9250Data[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
	aNew[1] = (float)MPU9250Data[1] * aRes - accelBias[1];
	aNew[2] = (float)MPU9250Data[2] * aRes - accelBias[2];

	// Calculate the gyro value into actual degrees per second
	gNew[0] = (float)MPU9250Data[4] * gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
	gNew[1] = (float)MPU9250Data[5] * gRes - gyroBias[1];
	gNew[2] = (float)MPU9250Data[6] * gRes - gyroBias[2];
	
	if (modeMedianAndKalman)
	{
		if (flagStartRead)
		{
			a[0] = aNew[0];
			a[1] = aNew[1];
			a[2] = aNew[2];
			
			
			
			g[0] = gNew[0];
			g[1] = gNew[1];
			g[2] = gNew[2];
			
			for(int i=0; i<medianVal; i++)
			{
				axMedian[i] = aNew[0];
				ayMedian[i] = aNew[1];
				azMedian[i] = aNew[2];
				
				gxMedian[i] = gNew[0];
				gyMedian[i] = gNew[1];
				gzMedian[i] = gNew[2];
				
			}
			
			flagStartRead = 0;
		}
		else
		{
			a[0] = kalmanVal*aNew[0]+(1-kalmanVal)*a[0];
			a[1] = kalmanVal*aNew[1]+(1-kalmanVal)*a[1];
			a[2] = kalmanVal*aNew[2]+(1-kalmanVal)*a[2];
			
			g[0] = kalmanVal*gNew[0]+(1-kalmanVal)*g[0];
			g[1] = kalmanVal*gNew[1]+(1-kalmanVal)*g[1];
			g[2] = kalmanVal*gNew[2]+(1-kalmanVal)*g[2];
			
			axMedian[iterMedian] = a[0];
			ayMedian[iterMedian] = a[1];
			azMedian[iterMedian] = a[2];
			
			gxMedian[iterMedian] = g[0];
			gyMedian[iterMedian] = g[1];
			gzMedian[iterMedian] = g[2];
			
			iterMedian++;
			
			if (iterMedian == 25)
			{
				iterMedian = 0;
			}
			
			a[0] = medianMas(axMedian, medianVal);
			a[1] = medianMas(ayMedian, medianVal);
			a[2] = medianMas(azMedian, medianVal);
			
			g[0] = medianMas(gxMedian, medianVal);
			g[1] = medianMas(gyMedian, medianVal);
			g[2] = medianMas(gzMedian, medianVal);
			
		}
	}
	else
	{
		
		a[0] = aNew[0];
		a[1] = aNew[1];
		a[2] = aNew[2];
		
		
		
		g[0] = gNew[0];
		g[1] = gNew[1];
		g[2] = gNew[2];
	}
	
}
void updateMag(void)
{
	int16_t magCount[3] = {0, 0, 0};    // Stores the 16-bit signed magnetometer sensor output
	readMagData(magCount);  // Read the x/y/z adc values

	float mNew[3];
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	mNew[0] = (float)(magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];  // get actual magnetometer value, this depends on scale being set
	mNew[1] = (float)(magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];
	mNew[2] = (float)(magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];
	
	if (modeMedianAndKalman)
	{
		if (flagStartRead)
		{
			m[0] = mNew[0];
			m[1] = mNew[1];
			m[2] = mNew[2];

			for(int i=0; i<medianVal; i++)
			{
				mxMedian[i] = mNew[0];
				myMedian[i] = mNew[1];
				mzMedian[i] = mNew[2];
			}
			flagStartRead = 0;
		}
		else
		{
			m[0] = kalmanVal*mNew[0]+(1-kalmanVal)*m[0];
			m[1] = kalmanVal*mNew[1]+(1-kalmanVal)*m[1];
			m[2] = kalmanVal*mNew[2]+(1-kalmanVal)*m[2];
			
			mxMedian[iterMedian] = m[0];
			myMedian[iterMedian] = m[1];
			mzMedian[iterMedian] = m[2];
			
			iterMedian++;
			
			if (iterMedian == 25)
			{
				iterMedian = 0;
			}
			
			m[0] = medianMas(mxMedian, medianVal);
			m[1] = medianMas(myMedian, medianVal);
			m[2] = medianMas(mzMedian, medianVal);
		}
	}
	else
	{
			m[0] = mNew[0];
			m[1] = mNew[1];
			m[2] = mNew[2];
	}
}

void updateRPY(void)
{
		a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
		a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
		a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
		a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
		a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
		pitch = -asinf(a32);
		roll  = atan2f(a31, a33);
		yaw   = atan2f(a12, a22);
		pitch *= 180.0f / PI;
		roll  *= 180.0f / PI;
		yaw   *= 180.0f / PI;
		yaw   += magnetic_declination;
		if (yaw >= +180.f)
		{			
			yaw -= 360.f;
		}
		else 
		{
			if (yaw <  -180.f)
			{				
				yaw += 360.f;
			}
		}

		lin_ax = a[0] + a31;
		lin_ay = a[1] + a32;
		lin_az = a[2] - a33;
}

void printCalibration(void)
{
	usbSend("< calibration parametrs >\r\n");
	usbSend("Accel bias [g]:\r\n");
	sprintf(bufUsb,"%f, %f, %f\r\n",accelBias[0] * 1000.f, accelBias[1] * 1000.f, accelBias[2] * 1000.f);
	usbSend(bufUsb);
	usbSend("Gyro bias [deg/s]: \r\n");
	sprintf(bufUsb,"%f, %f, %f\r\n", gyroBias[0], gyroBias[1], gyroBias[2]);
	usbSend(bufUsb);
	usbSend("Mag bias [mG]: \r\n");
	sprintf(bufUsb,"%f, %f, %f\r\n", magBias[0], magBias[1], magBias[2]);
	usbSend(bufUsb);
	usbSend("Mag scale []: \r\n");
	sprintf(bufUsb,"%f, %f, %f\r\n", magScale[0], magScale[1], magScale[2]);
	usbSend(bufUsb);

}


void printRollPitchYaw(void)
{
	sprintf(bufUsb, "4 %.2f %.2f %.2f\r\n", yaw, pitch, roll);
	usbSend(bufUsb);
}



void printRawData(void)
{
	if (flagPrint == 1)
	{
		 // Print acceleration values in milligs!
		sprintf(bufUsb, "0 %d %d %d\r\n",(int)(1000 * a[0]),(int)(1000 * a[1]),(int)(1000 * a[2]));
		usbSend(bufUsb);
	}
	
	if (flagPrint == 2)
	{
		// Print gyro values in degree/sec
		sprintf(bufUsb, "1 %.2f %.2f %.2f\r\n",g[0],g[1],g[2]);
		usbSend(bufUsb);
	}
	
	if (flagPrint == 3)
	{
		// Print mag values in degree/sec
		sprintf(bufUsb, "2 %d %d %d\r\n", (int)m[0], (int)m[1], (int)m[2]);
		usbSend(bufUsb);
	}
	
	if (flagPrint == 4)
	{
		sprintf(bufUsb, "3 %.2f %.2f %.2f %.2f\r\n",q[0],q[1],g[2],q[3]);
		usbSend(bufUsb);
	}
}
void print(void)
{
	printRawData();
	printRollPitchYaw();
	printCalibration();
}
int available(void)
{
	uint8_t intStatusVal[1];
	MPU9250Read(INT_STATUS, &intStatusVal[0],1);
	
	return (intStatusVal[0] & 0x01);
}	
void update(void)
{
	//if (available())
	//{// On interrupt, check if data ready interrupt
		updateAccelGyro();
		updateMag();
	//}
	
	// Madgwick function needs to be fed North, East, and Down direction like
	// (AN, AE, AD, GN, GE, GD, MN, ME, MD)
	// Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
	// Magneto direction is Right-Hand, Y-Forward, Z-Down
	// So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
	// we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
	// but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
	// because gravity is by convention positive down, we need to ivnert the accel data

	// get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
	// acc[mg], gyro[deg/s], mag [mG]
	// gyro will be convert from [deg/s] to [rad/s] inside of this function
	qFilterUpdate(-a[0], a[1], a[2], g[0], -g[1], -g[2], m[1], -m[0], m[2], q);
	
	if (!b_ahrs)
	{
			tempCount = readTempData();  // Read the adc values
			temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
	}
	else
	{
		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		updateRPY();
	}

}
        
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
	valInit();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(orangeLed_GPIO_Port,orangeLed_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(greenLed_GPIO_Port,greenLed_Pin,GPIO_PIN_RESET);
	
	HAL_Delay(3000);
	usbSend("Start!\r\n");
	HAL_Delay(1000);
	usbSend("3!\r\n");
	HAL_Delay(1000);
	usbSend("2!\r\n");
	HAL_Delay(1000);
	usbSend("1!\r\n");
	HAL_Delay(1000);
	
	usbSend("MPU setup:\r\n");
	MPUSetup();
	HAL_Delay(500);
	
	
	if (flagMPU)
	{
		
		
		usbSend("Take blue button for calibrate\r\n");
		HAL_Delay(2000);
		if (HAL_GPIO_ReadPin(blueButton_GPIO_Port, blueButton_Pin)==GPIO_PIN_SET)
		{
			
			usbSend("MPU gyro and accel calibrate:\r\n");
			calibrateAccelGyro();
			HAL_Delay(500);
			
			usbSend("AK8963 calibrate:\r\n");
			calibrateMag();
			HAL_Delay(500);
		}
		
		HAL_GPIO_WritePin(orangeLed_GPIO_Port,orangeLed_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(greenLed_GPIO_Port,greenLed_Pin,GPIO_PIN_SET);
		
	}
	else
	{
		HAL_GPIO_WritePin(orangeLed_GPIO_Port,orangeLed_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(redLed_GPIO_Port,redLed_Pin,GPIO_PIN_SET);
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	usbSend("While go!\r\n");
	char trans_str[64] = {0,};
	int flagNoClickButton = 1;
	uint32_t prevTime = HAL_GetTick();
	
  while (1)
  {
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if ((isConnectedAK8963() == 3) & (isConnectedMPU9250()==1))
		{
			
			HAL_GPIO_WritePin(redLed_GPIO_Port,redLed_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(greenLed_GPIO_Port,greenLed_Pin,GPIO_PIN_SET);
			
			if (HAL_GPIO_ReadPin(blueButton_GPIO_Port, blueButton_Pin)==GPIO_PIN_SET && flagNoClickButton)
			{
				
				if (flagNoClickButton)
				{
					
					flagPrint = flagPrint + 1;
					
					if (flagPrint > 5)
					{
						flagPrint = 1;
					}
					flagNoClickButton = 0;
					
				}
			}
			
			if (HAL_GPIO_ReadPin(blueButton_GPIO_Port, blueButton_Pin)==GPIO_PIN_RESET)
			{
				flagNoClickButton = 1;
			}
			
			update();
			if (HAL_GetTick() - prevTime > 984)
			{
				if (flagPrint > 0 && flagPrint < 5)
				{
					printRawData();
				}
				
				if (flagPrint == 5)
				{
					printRollPitchYaw();
				}
				prevTime = HAL_GetTick();
			}	
		}
		else
		{
			HAL_GPIO_WritePin(greenLed_GPIO_Port,greenLed_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(redLed_GPIO_Port,redLed_Pin,GPIO_PIN_SET);
			usbSend("Error connection MPU\r\n");
		}
		HAL_Delay(16);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, greenLed_Pin|orangeLed_Pin|redLed_Pin|blueLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blueButton_Pin */
  GPIO_InitStruct.Pin = blueButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blueButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : greenLed_Pin orangeLed_Pin redLed_Pin blueLed_Pin */
  GPIO_InitStruct.Pin = greenLed_Pin|orangeLed_Pin|redLed_Pin|blueLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
