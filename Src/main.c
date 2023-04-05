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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stdlib.h"
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
	
PUTCHAR_PROTOTYPE{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
	return ch;
}
enum gsm_status{
  INIT, ATE0, CMEE, CIPSTART, CIPSEND, CIPCLOSE, NOTHING, PUBLIH, DONE_PUBISH
};
enum gsm_status device = INIT;

#define MAX_GSM_SERIAL_BUFFER 100
#define MAX_GSM_RESET_INTERVAL 100 
int gsm_reset_interval_cnt = 0;

uint8_t gsm_serial_data[1];
char gsm_serial_buffer[MAX_GSM_SERIAL_BUFFER];
int gsm_serial_buffer_idx = 0;

int gsm_rst_tim_cnt = 0;
int timer_counter;

int gsm_send_period = 12;
char* gsm_response1 = "+CREG: 1,5";
char* gsm_response2 = "+CREG: 1,1";
char* gsm_response3 = NULL;

char MQTT_STR [750];
int MQTT_STR_idx = 0;

char* MQTT_BROKER = "193.176.241.102";
//char* MQTT_BROKER = "broker.hivemq.com";
int MQTT_PORT = 1883;
char* MQTT_PCKT_ID = "REZA";
char* MQTT_PCKT_TOPIC = "mqtt_gs";
#define MQTT_SEND_INTERVAL	12
int mqtt_send_interval_cnt = 0;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

#define gps_tag_size  70
#define mpu_packe_size  70
#define mpu_pcket_in_each_second   5
TM_GPS_t GPS_Data;
TM_GPS_Result_t result;
TM_GPS_Float_t GPS_Float;
struct GPS_tag {
  int year;
  int month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  float latitude;
  float longitude;
  float altitude;
	
	float custom_latitude;
  float custom_longitude;

  uint8_t satelite_number;
  uint8_t speed;
};
struct GPS_tag gps_tag;
uint8_t gps_serial_data[1];


#define MQTT_ARRAY_LENGTH  	gps_tag_size * 10
char MQTT_ARRAY[MQTT_ARRAY_LENGTH];
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t hex(int _int){
  uint8_t res;
  res = _int/16;
  res = res << 4;
  res = res | (_int%16);
  return res;
}
int parse_gsm_rec_buffer(){
	int res = -1;
	if(gsm_response1 != NULL)
		if (strstr(gsm_serial_buffer, gsm_response1) != NULL)
			res = 1;
	if(gsm_response2 != NULL)
		if (strstr(gsm_serial_buffer, gsm_response2) != NULL)
			res = 2;
  if(gsm_response3 != NULL)
		if (strstr(gsm_serial_buffer, gsm_response3) != NULL)
			res = 3;
		
	if(res != -1){
		printf("Res: ");
		switch(res){
			case 1:
				printf(gsm_response1);
				break;
			case 2:
				printf(gsm_response2);
				break;	
			case 3:
				printf(gsm_response3);
				break;	
			}
		printf("\n");
		}

	return res;
}
void MQTT_PUBLISH(){
	MQTT_STR_idx = 0;
	memset(MQTT_STR, 0, sizeof(MQTT_STR));
  MQTT_STR[MQTT_STR_idx++] = 0x10;
  MQTT_STR[MQTT_STR_idx++] = hex(10 + 2 + strlen(MQTT_PCKT_ID));
  
  MQTT_STR[MQTT_STR_idx++] = 0x00;
  MQTT_STR[MQTT_STR_idx++] = 0x04;
  MQTT_STR[MQTT_STR_idx++] = 0x4D;
  MQTT_STR[MQTT_STR_idx++] = 0x51;
  MQTT_STR[MQTT_STR_idx++] = 0x54;
  MQTT_STR[MQTT_STR_idx++] = 0x54;

  MQTT_STR[MQTT_STR_idx++] = 0x04;
  MQTT_STR[MQTT_STR_idx++] = 0x02;
  MQTT_STR[MQTT_STR_idx++] = 0x00;
  MQTT_STR[MQTT_STR_idx++] = 0x3c;

  MQTT_STR[MQTT_STR_idx++] = hex(strlen(MQTT_PCKT_ID)/16);
  MQTT_STR[MQTT_STR_idx++] = hex(strlen(MQTT_PCKT_ID)%16);
    
  for(int i=0; i<strlen(MQTT_PCKT_ID); i++)
		MQTT_STR[MQTT_STR_idx++] = hex(MQTT_PCKT_ID[i]);

	int pckt_length = 2 + strlen(MQTT_PCKT_TOPIC) + MQTT_ARRAY_LENGTH;
	
	MQTT_STR[MQTT_STR_idx++] = hex(48);
	
	if(pckt_length <= 127)
		MQTT_STR[MQTT_STR_idx++] = hex(pckt_length);
	
  else {
    uint8_t tmp;
    tmp = hex(pckt_length%128);
		unsigned int b = 128;
    tmp = b | tmp;
		MQTT_STR[MQTT_STR_idx++] = hex(tmp);
    tmp = hex(pckt_length/128);
		MQTT_STR[MQTT_STR_idx++] = hex(tmp);
  }
	
	MQTT_STR[MQTT_STR_idx++] = hex(0);
	MQTT_STR[MQTT_STR_idx++] = hex(strlen(MQTT_PCKT_TOPIC));
	for(int i = 0; i < strlen(MQTT_PCKT_TOPIC); i++)
		MQTT_STR[MQTT_STR_idx++] = hex(MQTT_PCKT_TOPIC[i]);

	for(int i = 0; i < MQTT_ARRAY_LENGTH; i++)
		MQTT_STR[MQTT_STR_idx++] = hex(MQTT_ARRAY[i]);
	
	
	HAL_UART_Transmit(&huart1, (uint8_t*)MQTT_STR, MQTT_STR_idx, 1000);	
	HAL_Delay(150);
	char str[1];
	sprintf (str, "%c", 26);
	HAL_UART_Transmit(&huart1, (uint8_t*)str, sizeof(char), 1000);	
}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool dummy_send_flag = true;
int dummy_send__cntr = 1;
void packetize_mqtt(struct GPS_tag pckt, char pckt_array[]){
  double dec, fract;
  int fractt, decc;
  int idx = 0;
  
  fract = modf(pckt.latitude, &dec);
  decc = dec;
  pckt_array[idx++] = (decc/10) + '0';
  pckt_array[idx++] = (decc%10) + '0';
  pckt_array[idx++] = '.';
  fractt = fract * 10000000;//10.000.000
  pckt_array[idx++] = (fractt/1000000) + '0';   fractt%=1000000;
  pckt_array[idx++] = (fractt/100000) + '0';   fractt%=100000;
  pckt_array[idx++] = (fractt/10000) + '0';   fractt%=10000;
  pckt_array[idx++] = (fractt/1000) + '0';   fractt%=1000;
  pckt_array[idx++] = (fractt/100) + '0';   fractt%=100;
  pckt_array[idx++] = (fractt/10) + '0';   fractt%=10;
  pckt_array[idx++] = ',';

  fract = modf(pckt.longitude, &dec);
  decc = dec;
  pckt_array[idx++] = (decc/10) + '0';
  pckt_array[idx++] = (decc%10) + '0';
  pckt_array[idx++] = '.';
  fractt = fract * 10000000;//10.000.000
  pckt_array[idx++] = (fractt/1000000) + '0';   fractt%=1000000;
  pckt_array[idx++] = (fractt/100000) + '0';   fractt%=100000;
  pckt_array[idx++] = (fractt/10000) + '0';   fractt%=10000;
  pckt_array[idx++] = (fractt/1000) + '0';   fractt%=1000;
  pckt_array[idx++] = (fractt/100) + '0';   fractt%=100;
  pckt_array[idx++] = (fractt/10) + '0';   fractt%=10;
  pckt_array[idx++] = ',';
	
	fract = modf(pckt.custom_latitude, &dec);
  decc = dec;
  pckt_array[idx++] = (decc/10) + '0';
  pckt_array[idx++] = (decc%10) + '0';
  pckt_array[idx++] = '.';
  fractt = fract * 10000000;//10.000.000
  pckt_array[idx++] = (fractt/1000000) + '0';   fractt%=1000000;
  pckt_array[idx++] = (fractt/100000) + '0';   fractt%=100000;
  pckt_array[idx++] = (fractt/10000) + '0';   fractt%=10000;
  pckt_array[idx++] = (fractt/1000) + '0';   fractt%=1000;
  pckt_array[idx++] = (fractt/100) + '0';   fractt%=100;
  pckt_array[idx++] = (fractt/10) + '0';   fractt%=10;
  pckt_array[idx++] = ',';

  fract = modf(pckt.custom_longitude, &dec);
  decc = dec;
  pckt_array[idx++] = (decc/10) + '0';
  pckt_array[idx++] = (decc%10) + '0';
  pckt_array[idx++] = '.';
  fractt = fract * 10000000;//10.000.000
  pckt_array[idx++] = (fractt/1000000) + '0';   fractt%=1000000;
  pckt_array[idx++] = (fractt/100000) + '0';   fractt%=100000;
  pckt_array[idx++] = (fractt/10000) + '0';   fractt%=10000;
  pckt_array[idx++] = (fractt/1000) + '0';   fractt%=1000;
  pckt_array[idx++] = (fractt/100) + '0';   fractt%=100;
  pckt_array[idx++] = (fractt/10) + '0';   fractt%=10;
  pckt_array[idx++] = ',';
  
  decc = pckt.altitude * 10;
  pckt_array[idx++] = (decc/10000) + '0';   decc%=10000;
  pckt_array[idx++] = (decc/1000) + '0';   decc%=1000;
  pckt_array[idx++] = (decc/100) + '0';   decc%=100;
  pckt_array[idx++] = (decc/10) + '0';   decc%=10;
  pckt_array[idx++] = '.';
  pckt_array[idx++] = (decc%10) + '0';
  pckt_array[idx++] = ',';

  
  pckt_array[idx++] = (pckt.satelite_number / 10) + '0';
  pckt_array[idx++] = (pckt.satelite_number % 10) + '0';
  pckt_array[idx++] = ',';

  pckt_array[idx++] = (pckt.year/1000) + '0';   pckt.year%=1000;
  pckt_array[idx++] = (pckt.year/100) + '0';   pckt.year%=100;
  pckt_array[idx++] = (pckt.year/10) + '0';   pckt.year%=10;
  pckt_array[idx++] = (pckt.year % 10) + '0';
  pckt_array[idx++] = '/';

  pckt_array[idx++] = (pckt.month/10) + '0';   pckt.month%=10;
  pckt_array[idx++] = (pckt.month % 10) + '0';
  pckt_array[idx++] = '/';

  pckt_array[idx++] = (pckt.day/10) + '0';   pckt.day%=10;
  pckt_array[idx++] = (pckt.day % 10) + '0';
  pckt_array[idx++] = ',';

  pckt_array[idx++] = (pckt.hour/10) + '0';   pckt.hour%=10;
  pckt_array[idx++] = (pckt.hour % 10) + '0';
  pckt_array[idx++] = ':';

  pckt_array[idx++] = (pckt.minute/10) + '0';   pckt.minute%=10;
  pckt_array[idx++] = (pckt.minute % 10) + '0';
  pckt_array[idx++] = ':';

  pckt_array[idx++] = (pckt.second/10) + '0';   pckt.second%=10;
  pckt_array[idx++] = (pckt.second % 10) + '0';
  pckt_array[idx++] = '*';

  pckt_array[idx] = '\0';
}
void gsm_transmit_at(){
	int STRMAX =100;
	char dummy[1];
	char ss[50];
	char str[STRMAX];
	
	if(device != NOTHING && device != DONE_PUBISH){
		switch(device){
			case (INIT):
//				sprintf (dummy, "%c", 26);
//				HAL_UART_Transmit(&huart1, (uint8_t*)dummy, sizeof(char), 1000);	
				strncpy(str, "AT+CREG?\r\n", STRMAX);
				printf("Sending: %s", str);
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
			break;
		
			case(ATE0):
				strncpy(str, "ATE0\r\n", STRMAX);
				printf("Sending: %s", str);
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
			break;
		
			case (CMEE):
				strncpy(str, "AT+CMEE=2\r\n", STRMAX);
				printf("Sending: %s", str);
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
			break;
			
			case(CIPSTART):
				sprintf(ss, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", MQTT_BROKER, MQTT_PORT);
				strncpy(str, ss, STRMAX);
				printf("Sending: %s", str);
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
			break;
				
			case(CIPSEND):
				strncpy(str, "AT+CIPSEND\r\n", STRMAX);
				printf("Sending: %s", str);
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
			break;
			
			case(PUBLIH):
				printf("Generating MQTT pckt\n");
			
//				printf(MQTT_ARRAY);
//				printf("\n\n");
			
				MQTT_PUBLISH();
				sprintf (dummy, "%c", 26);
				HAL_UART_Transmit(&huart1, (uint8_t*)dummy, sizeof(char), 1000);	
				dummy_send_flag = true;
//				device = CIPCLOSE;
			break;
			
			case(CIPCLOSE):
				sprintf (dummy, "%c", 26);
				HAL_UART_Transmit(&huart1, (uint8_t*)dummy, sizeof(char), 1000);	
				strncpy(str, "AT+CIPCLOSE\r\n", STRMAX);
				printf("Sending: %s", str);
				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
				device = NOTHING;
			break;
		}
		gsm_serial_buffer_idx = 0;
		memset(gsm_serial_buffer, 0, sizeof gsm_serial_buffer);
	}
}
void gsm_next_status(){
	switch(device){
		case INIT:
			device = ATE0;
			gsm_response1 = "OK";
			gsm_response2 = NULL;
			gsm_response3 = NULL;
			gsm_send_period = 4;
		case ATE0:
			device = CMEE;
			gsm_response1 = "OK";
			gsm_response2 = NULL;
			gsm_response3 = NULL;
			gsm_send_period = 4;
			break;
		case CMEE:
			device = CIPSTART;
			gsm_response1 = "CONNECT OK";
			gsm_response2 = "ALREAY CONNECT";
			gsm_response3 = NULL;
			gsm_send_period = 16;
			break;
		case CIPSTART:
			device = CIPSEND;
			gsm_response1 = ">";
			gsm_response2 = NULL;
			gsm_response3 = NULL;
			gsm_send_period = 8;
			break;
		case CIPSEND:
			device = PUBLIH;
			gsm_response1 = "SEND OK";
			gsm_response2 = NULL;
			gsm_response3 = NULL;
			gsm_send_period = 8;
			break;
//		case PUBLIH:
//			device = NOTHING;
//			gsm_response1 = "SEND OK";
//			gsm_response2 = NULL;
//			gsm_response3 = NULL;
//			gsm_send_period = 4;
//			break;	
//		case PUBLIH:
//			device = NOTHING;
//			gsm_response1 = "CLOSED";
//			gsm_response2 = "CLOSE OK";
//			gsm_response3 = "ERROR";
//			gsm_send_period = 4;
//			timer_counter = 1;
//			break;
	}
	
	
	
//	timer_counter = 1;
}
void reset_gsm_device(){
	printf("Reseting GSM\n");
	HAL_GPIO_WritePin(GSM_rst_GPIO_Port, GSM_rst_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GSM_rst_GPIO_Port, GSM_rst_Pin, GPIO_PIN_RESET);
	timer_counter = 1;
	gsm_reset_interval_cnt = 0;
	gsm_serial_buffer_idx = 0;
	memset(gsm_serial_buffer, 0, sizeof gsm_serial_buffer);
	
	device = INIT;
	gsm_send_period = 12;
	gsm_response1 = "+CREG: 1,5";
	gsm_response2 = "+CREG: 1,1";
	gsm_response3 = NULL;
	gsm_transmit_at();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	
	timer_counter++;
	gsm_reset_interval_cnt++;

	
	if(dummy_send_flag)
		dummy_send__cntr++;
	if(dummy_send__cntr%6 ==0){
		dummy_send__cntr =1;
		dummy_send_flag = false;
		if(device == PUBLIH)
			device = CIPCLOSE;
	}
	if(timer_counter % gsm_send_period == 0){
		gsm_transmit_at();
		timer_counter = 1;
	}
	if(device == NOTHING){
		mqtt_send_interval_cnt++;
		if(mqtt_send_interval_cnt % MQTT_SEND_INTERVAL == MQTT_SEND_INTERVAL -1){
			mqtt_send_interval_cnt = 0;
			device = CMEE;
			gsm_next_status();
			gsm_transmit_at();
			timer_counter = 1;
		}
	}
	if(gsm_reset_interval_cnt % MAX_GSM_RESET_INTERVAL == MAX_GSM_RESET_INTERVAL-1){
		reset_gsm_device();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){ // GSM
		gsm_serial_buffer[gsm_serial_buffer_idx++] = gsm_serial_data[0];
			int ret = parse_gsm_rec_buffer();
			if(ret != -1){
				timer_counter = 1;
				gsm_reset_interval_cnt = 0;
				gsm_serial_buffer_idx = 0;
				memset(gsm_serial_buffer, 0, sizeof gsm_serial_buffer);
				gsm_next_status();
				gsm_transmit_at();
			}
			if(MAX_GSM_SERIAL_BUFFER < gsm_serial_buffer_idx){
					gsm_serial_buffer_idx = 0;
					memset(gsm_serial_buffer, 0, sizeof gsm_serial_buffer);
				}
		HAL_UART_Receive_IT(&huart1, gsm_serial_data, sizeof(gsm_serial_data));
	}
	if(huart->Instance == USART3){ // GSM
		result = TM_GPS_Update(&GPS_Data, gps_serial_data[0]);
		if (result == TM_GPS_Result_NewData) {
			
			__HAL_UART_DISABLE_IT(&huart1, false);
			TM_GPS_ConvertFloat(GPS_Data.Latitude, &GPS_Float, 6);
			gps_tag.latitude = GPS_Float.Integer;
			gps_tag.latitude += GPS_Float.Decimal * 1e-6;
			
			TM_GPS_ConvertFloat(GPS_Data.Longitude, &GPS_Float, 6);
			gps_tag.longitude = GPS_Float.Integer;
			gps_tag.longitude += GPS_Float.Decimal * 1e-6;
			
			TM_GPS_ConvertFloat(GPS_Data.Altitude, &GPS_Float, 6);
			gps_tag.altitude = GPS_Float.Integer;
			gps_tag.altitude += GPS_Float.Decimal * 1e-6;

			gps_tag.satelite_number = GPS_Data.Satellites;

			
			char _array [gps_tag_size];
			gps_tag.hour = GPS_Data.Time.Hours;
			gps_tag.minute = GPS_Data.Time.Minutes;
			gps_tag.second = GPS_Data.Time.Seconds;
			packetize_mqtt(gps_tag, _array);
			_array[strlen(_array)] = '\0';
			for(int i=gps_tag_size; i<MQTT_ARRAY_LENGTH; i++)
				MQTT_ARRAY[i-gps_tag_size] = MQTT_ARRAY[i];
				
			int dummy = MQTT_ARRAY_LENGTH - gps_tag_size;
			for(int i=dummy; i<MQTT_ARRAY_LENGTH; i++)
				MQTT_ARRAY[i] = _array[i - dummy];
				
			MQTT_ARRAY[strlen(MQTT_ARRAY)] = '\0';
//			printf(_array);
//			printf("\n\n");
		}
		HAL_UART_Receive_IT(&huart1, gsm_serial_data, sizeof(gsm_serial_data));
		HAL_UART_Receive_IT(&huart3, gps_serial_data, sizeof(gps_serial_data));
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
	for(int i=0; i<MQTT_ARRAY_LENGTH; i++)
		MQTT_ARRAY[i] = 'b';
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	printf("Device Started\n");
	HAL_UART_Receive_IT(&huart1, gsm_serial_data, sizeof(gsm_serial_data));
	HAL_GPIO_WritePin(GSM_rst_GPIO_Port, GSM_rst_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart3, gps_serial_data, sizeof(gps_serial_data));
	reset_gsm_device();

	TM_GPS_Init(&GPS_Data, 115200);
	
	/* Convert float number */
	
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
