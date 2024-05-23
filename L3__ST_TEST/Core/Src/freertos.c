/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "spi.h"
#include "deal_data.h"
#include "nav.h"
#include "sbus.h"
#include "motor.h"
#include "WGS84ToAngle.h"

#include "fdcan.h"
#include "motor.h"

extern TIM_HandleTypeDef htim6;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
cJSON * EC600U_MQTT_SEND_STATUS;
cJSON * EC600U_HTTP_jobPause;
cJSON * EC600U_HTTP_updateRoute;

uint32_t GPS_REC_block_time    = portMAX_DELAY;
uint32_t EC600U_REC_block_time = portMAX_DELAY;
uint32_t APP_Info_Submit_time  = portMAX_DELAY;
uint32_t Device_unusual_time   = portMAX_DELAY;
Device_Poweron_status_t Device_Poweron_status = Check_poweron;

Change_Status_t Device_Run_Status =
{
	Poweron,
	Poweron,
	Job_Wait
};

extern uint8_t request_num;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 const uint32_t BIT_0 =	( 1 << 0 );
 const uint32_t BIT_1	= ( 1 << 1 );
 const uint32_t BIT_2	= ( 1 << 2 );
 const uint32_t BIT_3	= ( 1 << 3 );
 const uint32_t BIT_4	= ( 1 << 4 );
 const uint32_t BIT_5	= ( 1 << 5 );
 const uint32_t BIT_6	= ( 1 << 6 );
 const uint32_t BIT_7	= ( 1 << 7 );
 const uint32_t BIT_8	= ( 1 << 8 );
 const uint32_t BIT_9	= ( 1 << 9 );
 const uint32_t BIT_10=	( 1 << 10 );
 const uint32_t BIT_11= ( 1 << 11 );
 const uint32_t BIT_12=	( 1 << 12 );
 const uint32_t BIT_13=	( 1 << 13 );
 const uint32_t BIT_14=	( 1 << 14 );
 const uint32_t BIT_15=	( 1 << 15 );
 const uint32_t BIT_16=	( 1 << 16 );
 const uint32_t BIT_17=	( 1 << 17 );
 const uint32_t BIT_18=	( 1 << 18 );
 const uint32_t BIT_19=	( 1 << 19 );
 const uint32_t BIT_20=	( 1 << 20 );
 const uint32_t BIT_21=	( 1 << 21 );
 const uint32_t BIT_22=	( 1 << 22 );
 const uint32_t BIT_23=	( 1 << 23 );
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EC600U_REC */
osThreadId_t EC600U_RECHandle;
const osThreadAttr_t EC600U_REC_attributes = {
  .name = "EC600U_REC",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for APP_Info_Submit */
osThreadId_t APP_Info_SubmitHandle;
const osThreadAttr_t APP_Info_Submit_attributes = {
  .name = "APP_Info_Submit",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Device_Run */
osThreadId_t Device_RunHandle;
const osThreadAttr_t Device_Run_attributes = {
  .name = "Device_Run",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for EC600U_SEND */
osThreadId_t EC600U_SENDHandle;
const osThreadAttr_t EC600U_SEND_attributes = {
  .name = "EC600U_SEND",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for HTTP_REQUEST */
osThreadId_t HTTP_REQUESTHandle;
const osThreadAttr_t HTTP_REQUEST_attributes = {
  .name = "HTTP_REQUEST",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Device_unusual */
osThreadId_t Device_unusualHandle;
const osThreadAttr_t Device_unusual_attributes = {
  .name = "Device_unusual",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Power_Check */
osThreadId_t Power_CheckHandle;
const osThreadAttr_t Power_Check_attributes = {
  .name = "Power_Check",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GPS_REC */
osThreadId_t GPS_RECHandle;
const osThreadAttr_t GPS_REC_attributes = {
  .name = "GPS_REC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VCU_send */
osThreadId_t VCU_sendHandle;
const osThreadAttr_t VCU_send_attributes = {
  .name = "VCU_send",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for VCU_REC */
osThreadId_t VCU_RECHandle;
const osThreadAttr_t VCU_REC_attributes = {
  .name = "VCU_REC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SBUS_Parse */
osThreadId_t SBUS_ParseHandle;
const osThreadAttr_t SBUS_Parse_attributes = {
  .name = "SBUS_Parse",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for VCU_send_semp_queue */
osMessageQueueId_t VCU_send_semp_queueHandle;
const osMessageQueueAttr_t VCU_send_semp_queue_attributes = {
  .name = "VCU_send_semp_queue"
};
/* Definitions for VCU_recv_semp_queue */
osMessageQueueId_t VCU_recv_semp_queueHandle;
const osMessageQueueAttr_t VCU_recv_semp_queue_attributes = {
  .name = "VCU_recv_semp_queue"
};
/* Definitions for usart1_recv_semp_queue */
osMessageQueueId_t usart1_recv_semp_queueHandle;
const osMessageQueueAttr_t usart1_recv_semp_queue_attributes = {
  .name = "usart1_recv_semp_queue"
};
/* Definitions for usart1_send_semp_queue */
osMessageQueueId_t usart1_send_semp_queueHandle;
const osMessageQueueAttr_t usart1_send_semp_queue_attributes = {
  .name = "usart1_send_semp_queue"
};
/* Definitions for HTTP_REQUEST_queue */
osMessageQueueId_t HTTP_REQUEST_queueHandle;
const osMessageQueueAttr_t HTTP_REQUEST_queue_attributes = {
  .name = "HTTP_REQUEST_queue"
};
/* Definitions for SPI1_recv_semp_queue */
osMessageQueueId_t SPI1_recv_semp_queueHandle;
const osMessageQueueAttr_t SPI1_recv_semp_queue_attributes = {
  .name = "SPI1_recv_semp_queue"
};
/* Definitions for usart3_recv_semp_queue */
osMessageQueueId_t usart3_recv_semp_queueHandle;
const osMessageQueueAttr_t usart3_recv_semp_queue_attributes = {
  .name = "usart3_recv_semp_queue"
};
/* Definitions for APP_Info_Submit_Semp */
osSemaphoreId_t APP_Info_Submit_SempHandle;
const osSemaphoreAttr_t APP_Info_Submit_Semp_attributes = {
  .name = "APP_Info_Submit_Semp"
};
/* Definitions for SBUS_RUN_Semp */
osSemaphoreId_t SBUS_RUN_SempHandle;
const osSemaphoreAttr_t SBUS_RUN_Semp_attributes = {
  .name = "SBUS_RUN_Semp"
};
/* Definitions for CAN_send_semp */
osSemaphoreId_t CAN_send_sempHandle;
const osSemaphoreAttr_t CAN_send_semp_attributes = {
  .name = "CAN_send_semp"
};
/* Definitions for Device_Run_status_event */
osEventFlagsId_t Device_Run_status_eventHandle;
const osEventFlagsAttr_t Device_Run_status_event_attributes = {
  .name = "Device_Run_status_event"
};
/* Definitions for Device_unusual_status_event */
osEventFlagsId_t Device_unusual_status_eventHandle;
const osEventFlagsAttr_t Device_unusual_status_event_attributes = {
  .name = "Device_unusual_status_event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void EC600U_REC_task(void *argument);
void APP_Info_Submit_task(void *argument);
void Device_Run_task(void *argument);
void EC600U_SEND_task(void *argument);
void HTTP_REQUEST_task(void *argument);
void Device_unusual_task(void *argument);
void Power_Check_task(void *argument);
void GPS_REC_task(void *argument);
void VCU_send_task(void *argument);
void VCU_REC_task(void *argument);
void SBUS_Parse_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of APP_Info_Submit_Semp */
  APP_Info_Submit_SempHandle = osSemaphoreNew(1, 0, &APP_Info_Submit_Semp_attributes);

  /* creation of SBUS_RUN_Semp */
  SBUS_RUN_SempHandle = osSemaphoreNew(15, 0, &SBUS_RUN_Semp_attributes);

  /* creation of CAN_send_semp */
  CAN_send_sempHandle = osSemaphoreNew(15, 0, &CAN_send_semp_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of VCU_send_semp_queue */
  VCU_send_semp_queueHandle = osMessageQueueNew (15, sizeof(uint16_t), &VCU_send_semp_queue_attributes);

  /* creation of VCU_recv_semp_queue */
  VCU_recv_semp_queueHandle = osMessageQueueNew (15, sizeof(uint16_t), &VCU_recv_semp_queue_attributes);

  /* creation of usart1_recv_semp_queue */
  usart1_recv_semp_queueHandle = osMessageQueueNew (15, sizeof(uint16_t), &usart1_recv_semp_queue_attributes);

  /* creation of usart1_send_semp_queue */
  usart1_send_semp_queueHandle = osMessageQueueNew (15, sizeof(uint16_t), &usart1_send_semp_queue_attributes);

  /* creation of HTTP_REQUEST_queue */
  HTTP_REQUEST_queueHandle = osMessageQueueNew (16, sizeof(uint16_t), &HTTP_REQUEST_queue_attributes);

  /* creation of SPI1_recv_semp_queue */
  SPI1_recv_semp_queueHandle = osMessageQueueNew (15, sizeof(uint16_t), &SPI1_recv_semp_queue_attributes);

  /* creation of usart3_recv_semp_queue */
  usart3_recv_semp_queueHandle = osMessageQueueNew (30, sizeof(uint16_t), &usart3_recv_semp_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of EC600U_REC */
  EC600U_RECHandle = osThreadNew(EC600U_REC_task, NULL, &EC600U_REC_attributes);

  /* creation of APP_Info_Submit */
  APP_Info_SubmitHandle = osThreadNew(APP_Info_Submit_task, NULL, &APP_Info_Submit_attributes);

  /* creation of Device_Run */
  Device_RunHandle = osThreadNew(Device_Run_task, NULL, &Device_Run_attributes);

  /* creation of EC600U_SEND */
  EC600U_SENDHandle = osThreadNew(EC600U_SEND_task, NULL, &EC600U_SEND_attributes);

  /* creation of HTTP_REQUEST */
  HTTP_REQUESTHandle = osThreadNew(HTTP_REQUEST_task, NULL, &HTTP_REQUEST_attributes);

  /* creation of Device_unusual */
  Device_unusualHandle = osThreadNew(Device_unusual_task, NULL, &Device_unusual_attributes);

  /* creation of Power_Check */
  Power_CheckHandle = osThreadNew(Power_Check_task, NULL, &Power_Check_attributes);

  /* creation of GPS_REC */
  GPS_RECHandle = osThreadNew(GPS_REC_task, NULL, &GPS_REC_attributes);

  /* creation of VCU_send */
  VCU_sendHandle = osThreadNew(VCU_send_task, NULL, &VCU_send_attributes);

  /* creation of VCU_REC */
  VCU_RECHandle = osThreadNew(VCU_REC_task, NULL, &VCU_REC_attributes);

  /* creation of SBUS_Parse */
  SBUS_ParseHandle = osThreadNew(SBUS_Parse_task, NULL, &SBUS_Parse_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Device_Run_status_event */
  Device_Run_status_eventHandle = osEventFlagsNew(&Device_Run_status_event_attributes);

  /* creation of Device_unusual_status_event */
  Device_unusual_status_eventHandle = osEventFlagsNew(&Device_unusual_status_event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
//	uint8_t test[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
  /* Infinite loop */
  for(;;)
  {
//		usart2_send_data_apply("usart2_ok\r\n",11);
//		osDelay(2000);
//		cJSON* test = Json_data_Change(EC600U_MQTT_SEND_STATUS,"%f%s%s",1.1,"property","lat");
//		printf("%f",test->valuedouble);
		/*��Ҫ���ڴ�ܴ󣬽���1024*/
//		WGS84_axis_t Start_To_Distance_Angle	= GPStoXY(125,43,126,44);
//		printf("%f,%f,%f\r\n",Start_To_Distance_Angle.s12,Start_To_Distance_Angle.azi1,Start_To_Distance_Angle.azi2);
		
//		can_SendPacket(test, DRIVEID);
		
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
		osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_EC600U_REC_task */
/**
* @brief Function implementing the EC600U_REC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EC600U_REC_task */
void EC600U_REC_task(void *argument)
{
  /* USER CODE BEGIN EC600U_REC_task */
	osStatus_t err;
	uint16_t Recv_Len;
	//�������ڿ���DMA�����ж�
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, USART1RxData[UART1_fifo.usRxWrite],USART1_Max_Rxbuf_size);
  /* Infinite loop */
  for(;;)
  {
		err = osMessageQueueGet (usart1_recv_semp_queueHandle, &Recv_Len, 0, EC600U_REC_block_time);
		if(err == osOK)
		{
			if(Recv_Len != 0) //˵�����ڷ�����������
			{
//				printf("%s\r\n",USART1RxData[UART1_fifo.usRxRead]);
				usart1_rec_data_apply(USART1RxData[UART1_fifo.usRxRead],Recv_Len);
		
				memset(USART1RxData[UART1_fifo.usRxRead],0,Recv_Len);
				if (++UART1_fifo.usRxRead >= UART1_fifo.usRxBufSize)
				{
					UART1_fifo.usRxRead = 0;
				}
			}
			else  //˵��ֻ��Ϊ��������������Դ��ж��Ƿ�ʱ
				printf("�������\r\n");
		}
		else // ��ʱ����
		{
			if(Device_Run_Status.Curstatus == Poweron) //�������̻ظ���ʱ
			{
				if(Device_Poweron_status == Check_poweron)
				{
					printf("4Gģ���ϵ�ʧ��");
					osEventFlagsClear(Device_Run_status_eventHandle,BIT_1);                //����4Gģ���ϵ�ʧ�ܱ�־
					EC600U_REC_block_time = portMAX_DELAY;
				}
				else if(Device_Poweron_status == Check_Authen)
				{
					if((osEventFlagsGet(Device_Run_status_eventHandle) & BIT_3) != 0)    //�����ʱ����HTTP��ʱ�������ظ��������粻��ʱ�����ȴ����ȴ���������ʱ�ٽ��в���
					{
						if(request_num <= 5)
						{
							printf("HTTP��Ȩ����ʧ��\r\n");
							osMessageQueuePut(HTTP_REQUEST_queueHandle, &BIT_0 , 0 ,10);
							request_num ++;
						}
						else
						{
							printf("��Ȩʧ��");
							osEventFlagsClear(Device_Run_status_eventHandle,BIT_3);                //���ü�Ȩʧ�ܱ�־
							EC600U_REC_block_time = portMAX_DELAY;
						}
					}
				}
				else if(Device_Poweron_status == Check_MQTT_APP)
				{
					printf("����MQTT��Ϣʧ��");
					osEventFlagsClear(Device_Run_status_eventHandle,BIT_6);                //����RTK��Ϣ��ʧ��־
				}
			}
		}
  }
  /* USER CODE END EC600U_REC_task */
}

/* USER CODE BEGIN Header_APP_Info_Submit_task */
/**
* @brief Function implementing the APP_Info_Submit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APP_Info_Submit_task */
void APP_Info_Submit_task(void *argument)
{
  /* USER CODE BEGIN APP_Info_Submit_task */
	osStatus_t err;
	EC600U_MQTT_SEND_STATUS = StringToObject(DEVICE_MQTT_STATUS);
  /* Infinite loop */
  for(;;)
  {
		err = osSemaphoreAcquire (APP_Info_Submit_SempHandle, APP_Info_Submit_time);
		if(err == osOK)
		{
			if((osEventFlagsGet(Device_Run_status_eventHandle) & BIT_6) != 0)
			{
				APP_Info_Submit();
			}
		}
		else
		{
			APP_Info_Submit_time  = portMAX_DELAY;
			//�տ�ʼ���ܵ�gps���ݣ����ȶ����ȴ�һ��ʱ����ٿ����ϴ�
			HAL_TIM_Base_Start_IT(&htim6);                             //MQTT_APP�򿪳ɹ������յ�GPS���ݣ�����APP��Ϣ�ϱ�
		}
  }
  /* USER CODE END APP_Info_Submit_task */
}

/* USER CODE BEGIN Header_Device_Run_task */
/**
* @brief Function implementing the Device_Run thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Device_Run_task */
void Device_Run_task(void *argument)
{
  /* USER CODE BEGIN Device_Run_task */
	uint32_t uxBits;
	osStatus_t err;
	NAV_output_t NAV_output = {0};
	CAN_Msg_t    CAN_Msg = {0};
  /* Infinite loop */
  for(;;)
  {
		if(controlFlag == NALCont)
		{
			uxBits = osEventFlagsWait(Device_Run_status_eventHandle,  BIT_1 | BIT_3 |BIT_4 | BIT_23,osFlagsWaitAll | osFlagsNoClear, 1000); //����bit0��ʱ�Ȳ�����BIT_0 |  �ȴ�1s����ʱ����ӡ����������������ת����sbus���ƾ�ת��ģʽ
		
			if( (uxBits & ( BIT_1 | BIT_3 | BIT_4 | BIT_23))  == ( BIT_1 | BIT_3 | BIT_4 | BIT_23)) //�Ƿ���������������
			{
				if(Device_Run_Status.Curstatus == Job_Working)                                                      //�Ƿ�����˿ɹ�����׼��
				{
					NAV_output = NAV_Control();
					printf("���֣�%f,���֣�%f\r\n",NAV_output.LSpeed,NAV_output.RSpeed);
					CAN_Msg = Direct_Drive_motor(NAV_output.RSpeed,NAV_output.LSpeed);
					CAN1_send_data_apply(CAN_Msg.L_Msg);
					CAN1_send_data_apply(CAN_Msg.R_Msg);
					HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);	
				}
			}
			else //��ʱ�䲻��������
			{
				printf("�Զ���������������");
			}
		}
		else if(controlFlag == sbusCont)
		{
			err = osSemaphoreAcquire (SBUS_RUN_SempHandle, 1000);
			if(err == osOK)
			{
				CAN_Msg = Direct_Drive_motor(SBUS_CH.RSpeed,SBUS_CH.LSpeed);
				CAN1_send_data_apply(CAN_Msg.L_Msg);
				CAN1_send_data_apply(CAN_Msg.R_Msg);
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);	
			}
		}
		osDelay(1);
  }
  /* USER CODE END Device_Run_task */
}

/* USER CODE BEGIN Header_EC600U_SEND_task */
/**
* @brief Function implementing the EC600U_SEND thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EC600U_SEND_task */
void EC600U_SEND_task(void *argument)
{
  /* USER CODE BEGIN EC600U_SEND_task */
	osStatus_t err;
	uint16_t Send_Len;
  /* Infinite loop */
  for(;;)
  {
		err = osMessageQueueGet (usart1_send_semp_queueHandle, &Send_Len, 0, portMAX_DELAY);
		if(err == osOK)
		{
			HAL_UART_Transmit_DMA(&huart1,USART1TxData[UART1_fifo.usTxRead],Send_Len );
			if (++UART1_fifo.usTxRead >= UART1_fifo.usTxBufSize)
			{
				UART1_fifo.usTxRead = 0;
			}
			osDelay(100);
		}
  }
  /* USER CODE END EC600U_SEND_task */
}

/* USER CODE BEGIN Header_HTTP_REQUEST_task */
/**
* @brief Function implementing the HTTP_REQUEST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HTTP_REQUEST_task */
void HTTP_REQUEST_task(void *argument)
{
  /* USER CODE BEGIN HTTP_REQUEST_task */
	uint32_t Device_Run_Bits;
	uint32_t uxBits;
	osStatus_t err;
	EC600U_HTTP_updateRoute = StringToObject(Update_Route_param);
	EC600U_HTTP_jobPause    = StringToObject(Pause_param);
  /* Infinite loop */
  for(;;)
  {
		Device_Run_Bits = osEventFlagsWait(Device_Run_status_eventHandle,BIT_2,osFlagsNoClear, portMAX_DELAY);
		err = osMessageQueueGet (HTTP_REQUEST_queueHandle, &uxBits, 0, portMAX_DELAY);
		if(err == osOK)
		{
			if((Device_Run_Bits & BIT_2)  != 0)
			{
				/*��Ȩָ��*/
				if( (uxBits & BIT_0)  != 0 )
				{
					HTTP_Authen_Request();
				}
				/*��ʼ��ҵָ��*/
				else if((uxBits & BIT_1)  != 0)
				{
					HTTP_jobStart_Request();
				}
				/*��ͣ��ҵָ��*/
				else if((uxBits & BIT_2)  != 0)
				{
					HTTP_jobPause_Request();
				}
				/*������ҵָ��*/
				else if((uxBits & BIT_3)  != 0)
				{
					HTTP_jobContinue_Request();
				}
				/*�������ָ��*/
				else if((uxBits & BIT_4)  != 0)
				{
					HTTP_jobFinish_Request();
				}
				/*��ȡ�ֶκ���ָ��*/
				else if((uxBits & BIT_5)  != 0)
				{
					HTTP_updateRoute_Request();
				}
				/*��ȡ���׮λ��ָ��*/
				else if((uxBits & BIT_6)  != 0)
				{
					HTTP_goToCharge_Request();
				}
			}
		}
//    osDelay(1);
  }
  /* USER CODE END HTTP_REQUEST_task */
}

/* USER CODE BEGIN Header_Device_unusual_task */
/**
* @brief Function implementing the Device_unusual thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Device_unusual_task */
void Device_unusual_task(void *argument)
{
  /* USER CODE BEGIN Device_unusual_task */
	uint32_t uxBits;
  /* Infinite loop */
  for(;;)
  {
		uxBits = osEventFlagsWait(Device_unusual_status_eventHandle, BIT_0 | BIT_1 ,osFlagsWaitAny, Device_unusual_time);
		if( (uxBits & BIT_0)  != 0 )               //ǿ��ֹͣ
		{
			osEventFlagsClear(Device_Run_status_eventHandle,BIT_23);                //��������
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
			osDelay(1000);
		}
		else if( (uxBits & BIT_1)  != 0 )          //״̬�仯
		{
			if(Device_Run_Status.Curstatus == Job_Wait) // ����ǵȴ��׶�ת��Ϊ�����׶�
			{
				/*�Ƚ���http�õ��ĺ�����зָ�*/
				waypoints_Parse(HTTP_Task_Msg.waypoints,",");
				/*���õ�ǰ״̬*/
				Device_Run_Status.Prestatus = Device_Run_Status.Curstatus;
				Device_Run_Status.Curstatus = Device_Run_Status.Alterstatus;
				/*���õ�23λ���豸��������*/
				osEventFlagsSet(Device_Run_status_eventHandle,BIT_23);
				
			}
			else if(Device_Run_Status.Curstatus == Job_Working) //�����׶�ת��Ϊ ��������ͣ ��������� ���ϰ�������
			{
				
			}
			else
			{
				
			}
			
		}
		else if( (uxBits & BIT_2)  != 0 )          //RTKʧȥ�ź�
		{
			osEventFlagsClear(Device_Run_status_eventHandle,BIT_23);                //��������
		}
		else //�����ʱ
		{
			osEventFlagsSet(Device_Run_status_eventHandle,BIT_23);                //ת��Ϊ����
		}
//    osDelay(1);
  }
  /* USER CODE END Device_unusual_task */
}

/* USER CODE BEGIN Header_Power_Check_task */
/**
*        �����Ϣ�ĳ�����ȡ,���һ��ʱ��δ�ܻ�ȡ�����Ϣ���������޵�
* @brief Function implementing the Power_Check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Power_Check_task */
void Power_Check_task(void *argument)
{
  /* USER CODE BEGIN Power_Check_task */
//	osStatus_t err;
//	uint8_t  run_num = 0;
//	uint32_t len = 0;
  /* Infinite loop */
  for(;;)
  {
//		err = osMessageQueueGet (VCU_recv_semp_queueHandle, &len, 0, portMAX_DELAY); //���Խ׶�δʹ�ã���ʽ�׶ν�����ʱ������Ϊ1000 ��ʱʹ�ô��ڶ�
//		if(err == osOK)
//		{
//			/*�����������*/
//			if(len <= 100 && len>30)
//			{
//				if(run_num != 0) continue;
//				osEventFlagsSet(Device_Run_status_eventHandle,BIT_0);                 //���õ��������־λ
//				/*��EC600U�ϵ�*/
//				/*��GPS�ϵ�*/
//				EC600U_REC_block_time = 1000;                                         //�ж�4Gģ���Ƿ��ϵ糬ʱ
//				osMessageQueuePut(usart1_recv_semp_queueHandle,&len,0,10);
//				run_num = 1;
//			}
//			/*�������Ƿ��*/
//			else if(len<=30 && len >20)
//			{
//				/*����ڿ����Լ�׶�û��*/
//				if(Device_Run_Status.Curstatus == Poweron)
//				{
//					if(run_num != 0) continue;
//					printf("����û����,������������\r\n");
//					osEventFlagsSet(Device_Run_status_eventHandle,BIT_0);                 //���õ��������־λ
//					
//					/*��EC600U�ϵ�*/
//					/*��GPS�ϵ�*/
//					EC600U_REC_block_time = 1000;                                         //�ж�4Gģ���Ƿ��ϵ糬ʱ
//					osMessageQueuePut(usart1_recv_semp_queueHandle,&len,0,10);
//					run_num = 1;
//				}
//				/*��������ҵ������ҵ��ɻ��������ϰ���ֹͣ�����û�磬���Ϸ���*/
//				else if(Device_Run_Status.Curstatus == Job_Working || Device_Run_Status.Curstatus == Job_Wait || Device_Run_Status.Curstatus == Job_Block)
//				{
//					printf("����û����,��ʼ����\r\n");
//					/*���÷���״̬*/
//					osEventFlagsSet(Device_unusual_status_eventHandle,BIT_1);              //ת��״̬
//				}
//				else if(Device_Run_Status.Curstatus == Job_Return)
//				{
//					printf("����û����,���ڷ���\r\n");
//				}
//				else //������� ��������
//				{
//					
//				}
//			}
//			/*û�е��˽���ǿ��ֹͣ*/
//			else  
//			{
//				osEventFlagsClear(Device_Run_status_eventHandle,BIT_0);                //���õ���û���־λ
//				osEventFlagsSet(Device_unusual_status_eventHandle,BIT_0);              //ǿ��ֹͣ
//			}
//		}
//		else                                                                      //��ʱ�������޵� ǿ��ֹͣ
//		{
//			osEventFlagsClear(Device_Run_status_eventHandle,BIT_0);               	//���õ���û���־λ
//			osEventFlagsSet(Device_unusual_status_eventHandle,BIT_0);              //ǿ��ֹͣ
//		}
    osDelay(1000);
  }
  /* USER CODE END Power_Check_task */
}

/* USER CODE BEGIN Header_GPS_REC_task */
/**
* @brief Function implementing the GPS_REC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_REC_task */
void GPS_REC_task(void *argument)
{
  /* USER CODE BEGIN GPS_REC_task */
	osStatus_t err;
	uint32_t len = 0;
	HAL_SPI_Receive_DMA(&hspi1 , SPI1RxData[SPI1_fifo.usRxWrite] ,180 );
  /* Infinite loop */
  for(;;)
  {
		err = osMessageQueueGet (SPI1_recv_semp_queueHandle, &len, 0, GPS_REC_block_time); //���Խ׶�δʹ�ã���ʽ�׶ν�����ʱ������Ϊ1000
		if(err == osOK)
		{
			if(len != 0)
			{
//				printf("%s\r\n",SPI1RxData[SPI1_fifo.usRxRead]);
				
				SPI1_GPS_data(SPI1RxData[SPI1_fifo.usRxRead]);
				
				memset(SPI1RxData[SPI1_fifo.usRxRead],0,SPI1_Max_Rxbuf_size);
				if (++SPI1_fifo.usRxRead >= SPI1_fifo.usRxBufSize)
				{
					SPI1_fifo.usRxRead = 0;
				}
			}
			else  //˵��ֻ��Ϊ��������������Դ��ж��Ƿ�ʱ
				printf("�������\r\n");
		}
		else
		{
//			if(Device_Poweron_status == Check_RTK || Device_Run_Status.Curstatus != Poweron)
//			{
				printf("����RTK��Ϣʧ��\r\n");
//				HAL_TIM_Base_Stop_IT(&htim6);
				osEventFlagsClear(Device_Run_status_eventHandle,BIT_4);                //����RTK��Ϣ��ʧ��־
				osEventFlagsClear(Device_Run_status_eventHandle,BIT_5);                //����RTK��Ϣ��ʧ��־
				osEventFlagsSet(Device_unusual_status_eventHandle,BIT_2);              //�����쳣
//			}
		}
//    osDelay(1);
  }
  /* USER CODE END GPS_REC_task */
}

/* USER CODE BEGIN Header_VCU_send_task */
/**
* @brief Function implementing the VCU_send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VCU_send_task */
void VCU_send_task(void *argument)
{
  /* USER CODE BEGIN VCU_send_task */
	osStatus_t err;
	CAN_Msg_t Msg;
//	uint16_t Send_Len;
  /* Infinite loop */
  for(;;)
  {
//		err = osMessageQueueGet (VCU_send_semp_queueHandle, &Send_Len, 0, portMAX_DELAY);
//		if(err == osOK)
//		{
//			HAL_UART_Transmit_DMA(&huart2,USART2TxData[UART2_fifo.usTxRead],Send_Len );
//			if (++UART2_fifo.usTxRead >= UART2_fifo.usTxBufSize)
//			{
//				UART2_fifo.usTxRead = 0;
//			}
//			osDelay(20);
//		}
		/* ��ѯʽ can����*/
		err = osSemaphoreAcquire (CAN_send_sempHandle, 25);
		if(err == osOK)
		{
			/* ������Ϣ */
			can_SendPacket(CAN1TxData[CAN1_fifo.usTxRead],DRIVEID);
			memset(CAN1TxData[CAN1_fifo.usTxRead],0,8);
			if (++CAN1_fifo.usTxRead >= CAN1_fifo.usTxBufSize)
			{
				CAN1_fifo.usTxRead = 0;
			}
			osDelay(3);
		}
		else
		{
			Msg = Direct_Drive_motor(0, 0);
			can_SendPacket((uint8_t *)Msg.L_Msg,DRIVEID);
			osDelay(3);
			can_SendPacket((uint8_t *)Msg.R_Msg,DRIVEID);
		}
		//�������ַ���������
		//�����ٽ���
//		taskENTER_CRITICAL();
//		if(CAN1_fifo.usTxLen > 0)
//		{
//			/* ������Ϣ */
//			can_SendPacket(CAN1TxData[CAN1_fifo.usTxRead],DRIVEID);
//			memset(CAN1TxData[CAN1_fifo.usTxRead],0,8);
//			if (++CAN1_fifo.usTxRead >= CAN1_fifo.usTxBufSize)
//			{
//				CAN1_fifo.usTxRead = 0;
//			}
//			CAN1_fifo.usTxLen--;
//			osDelay(6);
//		}
//		else
//		{
//			can_SendPacket((uint8_t *)Direct_Drive_motor(0, 0).L_Msg,DRIVEID);
//			can_SendPacket((uint8_t *)Direct_Drive_motor(0, 0).R_Msg,DRIVEID);
//			osDelay(25);
//		}
//		/*�˳��ٽ���*/
//		taskEXIT_CRITICAL();
  }
  /* USER CODE END VCU_send_task */
}

/* USER CODE BEGIN Header_VCU_REC_task */
/**
* @brief Function implementing the VCU_REC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VCU_REC_task */
void VCU_REC_task(void *argument)
{
  /* USER CODE BEGIN VCU_REC_task */
	osStatus_t err;
	uint16_t Recv_Len;
	//�������ڿ���DMA�����ж�
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2RxData[UART2_fifo.usRxWrite],USART2_Max_Rxbuf_size);
  /* Infinite loop */
  for(;;)
  {
		err = osMessageQueueGet (VCU_recv_semp_queueHandle, &Recv_Len, 0, portMAX_DELAY);
		if(err == osOK)
		{
			if(Recv_Len != 0) //˵�����ڷ�����������
			{
				printf("%s   %d\r\n",USART2RxData[UART2_fifo.usRxRead],Recv_Len);
//				usart2_rec_data_apply(USART2RxData[UART2_fifo.usRxRead],Recv_Len);
				
				memset(USART2RxData[UART2_fifo.usRxRead],0,Recv_Len);
				if (++UART2_fifo.usRxRead >= UART2_fifo.usRxBufSize)
				{
					UART2_fifo.usRxRead = 0;
				}
			}
			else  //˵��ֻ��Ϊ��������������Դ��ж��Ƿ�ʱ
				printf("�������\r\n");
		}
		else //��ʱ����
		{
			
		}
  }
  /* USER CODE END VCU_REC_task */
}

/* USER CODE BEGIN Header_SBUS_Parse_task */
/**
* @brief Function implementing the SBUS_Parse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SBUS_Parse_task */
void SBUS_Parse_task(void *argument)
{
  /* USER CODE BEGIN SBUS_Parse_task */
	osStatus err;
	uint16_t Recv_Len;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3RxData[UART3_fifo.usRxWrite],USART3_Max_Rxbuf_size);
  /* Infinite loop */
  for(;;)
  {
		err = osMessageQueueGet (usart3_recv_semp_queueHandle, &Recv_Len, 0, portMAX_DELAY);
		if(err == osOK)
		{
//			printf("%x\r\n",USART3RxData[UART3_fifo.usRxRead]); //����ʹ��
			
			sbus_parse(USART3RxData[UART3_fifo.usRxRead],Recv_Len);
			
			memset(&USART3RxData[UART3_fifo.usRxRead],0,Recv_Len);
			if (++UART3_fifo.usRxRead >= UART3_fifo.usRxBufSize)
			{
				UART3_fifo.usRxRead = 0;
			}
		}
		else
		{
			SBUS_CH.Start = 0;
			printf("SBUS�źŶ�ʧ\r\n");
		}
  }
  /* USER CODE END SBUS_Parse_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

