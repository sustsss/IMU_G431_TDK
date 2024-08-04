/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "invn_algo_asf.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SPI1_IRQHandler(void);
void SysTick_Handler(void);


void SYS_LED_ON()
{
	LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_3);
}

void SYS_LED_OFF()
{
	LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_3);
}

void SYS_LED_TOG()
{
	LL_GPIO_TogglePin(GPIOA,LL_GPIO_PIN_3);
}

void CAN_LED_ON()
{
	LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_4);
}

void CAN_LED_OFF()
{
	LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_4);
}

void CAN_LED_TOG()
{
	LL_GPIO_TogglePin(GPIOA,LL_GPIO_PIN_4);
}

void SystemClock_Config(void);


void ICM_SPI_CS_LOW(void)
{
		LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_4);
}

void ICM_SPI_CS_HIGH(void)
{
		LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_4);
}

void UART1_SendByte(char c)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&c,1,0xFFFF);
}

void UART1_SendFloat(float f)
{
	char *p = (char *)&f;
	for(int i=0;i<4;i++)
	{
		UART1_SendByte(*p);
		p++;
	}
}

void UART1_SendData(uint8_t *data,uint16_t len)
{
	HAL_UART_Transmit(&huart1,data,len,0xFFFF);
}

uint8_t data[64]  = {0};
uint8_t data_count = 0;

struct HighResFIFOPacket
{
	uint8_t head;
	int16_t acc_x_h;
	int16_t acc_y_h;
	int16_t acc_z_h;
	int16_t gyro_x_h;
	int16_t gyro_y_h;
	int16_t gyro_z_h;
	uint16_t temperature;
	uint16_t time_stamp;
	uint8_t acc_x_l:4;
	uint8_t gyro_x_l:4;
	uint8_t acc_y_l:4;
	uint8_t gyro_y_l:4;
	uint8_t acc_z_l:4;
	uint8_t gyro_z_l:4;
}__attribute__((packed));

union HighResFIFOPacketUnion
{
	uint8_t packet_data[20];
	struct HighResFIFOPacket packet_struct;
};

void SPI1_IRQHandler(void)
{
	data[data_count%64] = LL_SPI_ReceiveData8(SPI1);
	data_count++;
}

uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{

    ICM_SPI_CS_LOW();
	
	for(int i=0;i<500;i++){;}

	LL_SPI_TransmitData8(SPI1,reg);
	LL_SPI_TransmitData8(SPI1,value);

	for(int i=0;i<500;i++){;}
		
    ICM_SPI_CS_HIGH();

    return 0;
}

 uint8_t icm42688_read_reg(uint8_t reg, uint8_t len)
{

    ICM_SPI_CS_LOW();
	
	for(int i=0;i<100;i++){;}

	reg |= 0x80;
	LL_SPI_TransmitData8(SPI1,reg);
		
	for(int i=0;i<len;i++)
	{
		//for(int i=0;i<10;i++){;}
		if(len>10)
		{
			for(int j=0;j<10;j++){;}
		}
		LL_SPI_TransmitData8(SPI1,0);
	}

	for(int i=0;i<50*len;i++){;}
		
    ICM_SPI_CS_HIGH();

    return 0;
}

uint8_t reg_val = 0;
static int32_t acc_bias[3] = {0,0,0};
static int32_t gyr_bias[3] = {0,0,0};
static int32_t acc_accuracy;
static int32_t gyr_accuracy;

static int read_reg_asf(uint8_t reg, uint8_t *buf, uint32_t len)
{
	data_count = 0;
	icm42688_read_reg(reg,len);
	
	for(int i=0;i<20*len;i++){;}
	
	if(reg == 0x75)
	{
		buf[0] = 0XDB;
	}
	else
	{	
		for(int i=0;i<len;i++)
		{
			buf[i] = data[i+1];
		}
	}
	
	return 0;
}

static int write_reg_asf(uint8_t reg, const uint8_t *buf, uint32_t len)
{
	for(int i=0;i<len;i++)
	{
		icm42688_write_reg(reg,buf[i]);
	}
	return 0;
}

static void sleep_us_asf(uint32_t us)
{
	for(int i = us * 170;i>0;i--)
	{
	}
}

uint8_t icm_init_state = 0;
uint8_t icm_state = 0;
InvnAlgoASFConfig config;
InvnAlgoASFSerif  serif;

void ICM_INT(void)
{
	icm_state = 1;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
    ICM_SPI_CS_LOW();
	for(int i=0;i<100;i++){;}
		
	LL_SPI_TransmitData8(SPI1,ICM42688_WHO_AM_I|0x80);
	LL_SPI_TransmitData8(SPI1,0);
	
	for(int i=0;i<100;i++){;}
	ICM_SPI_CS_HIGH();
		
	for(int i=0;i<100;i++){;}
		
    while(1)
	{
		if(data_count != 0)
		{
			if(data[1] == 0x47)
			{
				break;
			}
			else
			{
				data_count = 0;
				ICM_SPI_CS_LOW();
				for(int i=0;i<100;i++){;}
					
				LL_SPI_TransmitData8(SPI1,ICM42688_WHO_AM_I|0x80);
				LL_SPI_TransmitData8(SPI1,0);
				
				for(int i=0;i<100;i++){;}
				ICM_SPI_CS_HIGH();
			}
		}
		else
		{
			
			ICM_SPI_CS_LOW();
			for(int i=0;i<100;i++){;}
				
			LL_SPI_TransmitData8(SPI1,ICM42688_WHO_AM_I|0x80);
			LL_SPI_TransmitData8(SPI1,0);
			
			for(int i=0;i<100;i++){;}
			ICM_SPI_CS_HIGH();
		}
	}
	data_count = 0;
	
	icm42688_write_reg(ICM42688_REG_BANK_SEL, 0);
	icm42688_write_reg(ICM42688_PWR_MGMT0, 0x00);

	for(int i=0;i<10000;i++){;}
	
	icm42688_write_reg(ICM42688_FIFO_CONFIG,0x40);
	icm42688_write_reg(ICM42688_FIFO_CONFIG2, 0x00); // watermark
    icm42688_write_reg(ICM42688_FIFO_CONFIG3, 0x02); // watermark
	icm42688_write_reg(ICM42688_FIFO_CONFIG1, 0x10); // Enable the accel and gyro to the FIFO
	icm42688_write_reg(ICM42688_REG_BANK_SEL, 0);
	icm42688_write_reg(ICM42688_ACCEL_CONFIG0, 0x06);
	icm42688_write_reg(ICM42688_GYRO_CONFIG0, 0x26);
    icm42688_write_reg(ICM42688_INTF_CONFIG0, 0x00);
		
	for(int i=0;i<10000;i++){;}
		
	icm42688_write_reg(ICM42688_PWR_MGMT0, 0x0F);
		
    for(int i=0;i<10000;i++){;}	
	
    serif.read_reg_asf  = read_reg_asf;
	serif.write_reg_asf = write_reg_asf;
	serif.sleep_us_asf  = sleep_us_asf;
		
	icm_state = invn_algo_asf_generate_config(&config, INVN_ALGO_ASF_HIGH_ACCURACY);
		
	config.acc_fsr = 16;
	config.gyr_fsr = 2000;
		
	config.mag_sc_q16 = 9830;
    config.mag_odr_us = 1000;
		
	config.acc_odr_us = 4000;
	config.gyr_odr_us = 4000;
	
	config.temp_offset = 25<<16;
	config.temp_sensitivity = (int32_t)((int64_t)((int64_t)100 << 30) / 13248); // high-res
	config.predictive_time_us = 4000;
	config.acc_bias_q16 = acc_bias;
	config.gyr_bias_q16 = gyr_bias;
	config.acc_accuracy = 0;
	config.gyr_accuracy = 0;
	
	config.gyr_cal_stationary_duration_us = 100000;
	config.fus_low_speed_drift_yaw = 10;
	
	icm_state = invn_algo_asf_init(&serif, &config);
	
	while(icm_state!=0)
	{	
		icm_state = invn_algo_asf_init(&serif, &config);
		for(int i=0;i<10000;i++){;}
			
	}
	data_count = 0;
}

FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
 uint8_t can_rxbuf2[8];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, can_rxbuf2) != HAL_OK)
    {
		Error_Handler();
    }
}

uint32_t time_count = 0;
uint32_t time_compute =  0;
uint32_t time_end = 750;
uint32_t test_counts = 0;
float y_Init = 0;
float y_final = 0;
uint16_t flag = 0;
int data_Min = 0;
float q[4] = {0.0f};
float rpy[3] = {0.0f};
float r,p,y = 0.0f;
float r_record[5],p_record[5], y_record[5]={0.0f};
//int r,p,y=0;
uint8_t asf_state;
uint8_t MSB_LSB_set_flag = 0;
uint8_t MSB_LSB_state = 0;
float bias = 0.0f;
float biasf = 0.0f;
float temp = 0.0f;
static InvnAlgoASFInput input;
static InvnAlgoASFOutput output;
uint8_t test_flag = 0;
uint8_t sendHeader[4] = {0xA5,0x00,0x00,0xff};

static int32_t icm_mounting_matrix[9] = { (1 << 30), 0, 0, 0, (1 << 30), 0, 0, 0, (1 << 30) };

static void apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3])
{
	unsigned i;
	int64_t  data_q30[3];

	for (i = 0; i < 3; i++) {
		data_q30[i] = ((int64_t)matrix[3 * i + 0] * raw[0]);
		data_q30[i] += ((int64_t)matrix[3 * i + 1] * raw[1]);
		data_q30[i] += ((int64_t)matrix[3 * i + 2] * raw[2]);
	}
	raw[0] = (int32_t)(data_q30[0] >> 30);
	raw[1] = (int32_t)(data_q30[1] >> 30);
	raw[2] = (int32_t)(data_q30[2] >> 30);
}

float get_yaw(float m_q[4]){

    float q0 = m_q[0];
    float q1 = m_q[1];
    float q2 = m_q[2];
    float q3 = m_q[3];

    return atan2f(2.0f*(q1*q2+q0*q3),(q1*q1+q0*q0-q3*q3-q2*q2));

}

float get_pitch(float m_q[4]){
    
    float q0 = m_q[0];
    float q1 = m_q[1];
    float q2 = m_q[2];
    float q3 = m_q[3];

    return atan2f(2.0f*(q0*q1+q2*q3),(q3*q3-q2*q2-q1*q1+q0*q0));

}

float get_roll(float m_q[4]){
    
    float q0 = m_q[0];
    float q1 = m_q[1];
    float q2 = m_q[2];
    float q3 = m_q[3];

    return asinf(2.0f*q0*q2-2.0f*q1*q3);
}

void WXYPrintData(int data)
{
	if(data<0)
	{
		data=-data;
		UART1_SendByte('-');
	}
	else
	{
		UART1_SendByte('+');
	}
	int a=100000;
	while(a>1)
	{
		data=data%a;
		a=a/10;
		UART1_SendByte((char)('0'+(data/a)));
	}
}

union HighResFIFOPacketUnion packet_union;
static uint16_t time_stamp_test = 0;
static uint16_t last_time_stamp_test = 0;
static uint32_t systick = 0;
static float use_rate = 0.0f;
static float last_yaw = 0.0f;
static float abs_yaw = 0.0f;
static int32_t time_stamp = 0;



void SysTick_Handler(void)//2ms运行一次?
{
	SYS_LED_TOG();
	CAN_LED_TOG();
	test_counts++;
	
	
	if(test_counts < 5000)
	{
		return;
	}
	
	if(icm_state == 1)
	{
		return;
	}
	
	time_count++;
	
	
	
	float t = 0.002f;
	if(time_count%2 == 0)
	{
		data_count = 0;
		icm42688_read_reg(0X30 ,21);
	}
	else if (time_count%2 == 1)
	{
		if(data_count == 0 )
		{
			return;
		}
		
		for(uint8_t i = 0; i<20 ; i++)
		{
			packet_union.packet_data[i] = data[i+1];
		}
		input.mask = 3;

		input.sRacc_data[0] = ((((int32_t)packet_union.packet_struct.acc_x_h)<<4)+(int32_t)packet_union.packet_struct.acc_x_l);
		input.sRacc_data[1] = ((((int32_t)packet_union.packet_struct.acc_y_h)<<4)+(int32_t)packet_union.packet_struct.acc_y_l);
		
		input.sRacc_data[2] = ((((int32_t)packet_union.packet_struct.acc_z_h)<<4)+(int32_t)packet_union.packet_struct.acc_z_l);
		
		input.sRgyr_data[0] = ((((int32_t)packet_union.packet_struct.gyro_x_h)<<4)+(int32_t)packet_union.packet_struct.gyro_x_l);
		input.sRgyr_data[1] = ((((int32_t)packet_union.packet_struct.gyro_y_h)<<4)+(int32_t)packet_union.packet_struct.gyro_y_l);
		input.sRgyr_data[2] = ((((int32_t)packet_union.packet_struct.gyro_z_h)<<4)+(int32_t)packet_union.packet_struct.gyro_z_l);
	
		time_stamp += 4000;
		
		input.sRtemp_data = packet_union.packet_struct.temperature;
		input.sRimu_time_us = time_stamp;
		
		apply_mounting_matrix(icm_mounting_matrix, input.sRacc_data);
	    apply_mounting_matrix(icm_mounting_matrix, input.sRgyr_data);
		
		invn_algo_asf_process(&input, &output);
	
		for(uint8_t i = 0;i<4;i++)
		{
			q[i] = output.grv_quat_q30[i]*0.000000001F;
		}
		float sum = 0.0f;
		
		rpy[2] = get_yaw(q);
		rpy[1] = get_pitch(q);
		rpy[0] = get_roll(q);
		
//		r = (int)(rpy[0]*57.29578f*1000);
//		p = (int)(rpy[1]*57.29578f*1000);
//		y = (int)(rpy[2]*57.29578f*1000);
		r = rpy[0]*57.29578f;
		p = rpy[1]*57.29578f;
		y = rpy[2]*57.29578f;
		
		bias = output.gyr_bias_q16[2]*0.0001f;
		
		
		last_yaw = y;
		
		float delta_yaw = y - last_yaw;
		
		if(delta_yaw<-180.0f)
		{
			delta_yaw =  360.0f + delta_yaw;
		}
		else if(delta_yaw>180.0f)
		{
			delta_yaw = 360.0f - delta_yaw;
		}
		
		abs_yaw +=delta_yaw;
		
		temp = packet_union.packet_struct.temperature /132.48f + 25.0f;
	}
	
	   if((input.sRacc_data[1] > 400000&&flag == 0)||(time_compute < time_end && time_compute > 0))
	    {
			flag = 1;
			
			if(time_compute == 0){
				y_Init = y;
			}

			r_record[time_compute%10] = r;
			p_record[time_compute%10] = p;//连续记录5次pitch数据，用最后一次减去初始的，看是否变化超过了10度，假如超过了10度，基本可以确定是落地了。
			y_record[time_compute%10] = y;

			if(time_compute > 500){
				if(abs(r_record[4]-r_record[0])>10||abs(p_record[4]-p_record[0])>10||abs(y_record[4]-y_record[0])>10){
					time_compute = time_end;
				}
			}

		    time_compute++;

			if(abs(r) < 70){
				UART1_Printf("%.2f,", r);
				UART1_Printf("%.2f,", p);
				if(y-y_Init>180){
					y_final=360+y_Init-y;
				}else if (y-y_Init<-180)
				{
					y_final=360+y-y_Init;
					
				}else{
			
					y_final= y-y_Init;
				}
				UART1_Printf("%.2f,", y_final);
				UART1_Printf("%d", input.sRacc_data[1]);//通过打印此项能够观察到飞镖发射过程中加速是否平稳。

				UART1_SendByte('\n');
			}else{
				time_compute = time_end;
			}
	   }

	if(time_compute == time_end) //传输数据的时间为1.6s
	{ 
		UART1_SendByte('\n');
		time_compute = 0;
	}

	if(flag > 0){
		flag++;
		if(flag == 3000){
			flag = 0;    //此功能是防止飞镖落地后因装击再次触发数据传输，开始传输到下次能传输的时间间隔为6s
		}
	}
	
}



//			UART1_SendData(sendHeader,4);//标志�?
//			UART1_SendFloat(r);
//			UART1_SendFloat(p);
//			UART1_SendFloat(y);
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
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ICM_INT();
  
  for(int i=0;i<1000000;i++){;}

  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enable SysTick Interrupt   
  SysTick->LOAD = 85000000/250;//由250变为500
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PCLK1);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
