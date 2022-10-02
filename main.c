/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define Button_1_Monitor_PERIOD 		50//task A period
#define Button_2_Monitor_PERIOD 		50 //task B period
#define Periodic_Transmitter_PERIOD 100
#define Uart_Receiver_PERIOD 							20
#define Load_1_Simulation_PERIOD 							10 //Execution time: 5ms
#define Load_2_Simulation_PERIOD 							100 //Execution time: 12ms


TaskHandle_t task_1_Handler = NULL;
TaskHandle_t task_2_Handler = NULL;
TaskHandle_t task_3_Handler = NULL;
TaskHandle_t task_4_Handler = NULL;
TaskHandle_t task_5_Handler = NULL;
TaskHandle_t task_6_Handler = NULL;
//    x                    y                    z
int task_1_in_time = 0,task_1_out_time = 0,task_1_total_time = 0;
int task_2_in_time = 0,task_2_out_time = 0,task_2_total_time = 0;
int task_3_in_time = 0,task_3_out_time = 0,task_3_total_time = 0;
int task_4_in_time = 0,task_4_out_time = 0,task_4_total_time = 0;
int task_5_in_time = 0,task_5_out_time = 0,task_5_total_time = 0;
int task_6_in_time = 0,task_6_out_time = 0,task_6_total_time = 0;
int system_time = 0;
int cpu_load = 0;
char runTimeStatsBuff[190];
//volatile int messes =0;
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
void Button_1_Monitor (void *pvParameters);
void Button_2_Monitor (void *pvParameters);
void Periodic_Transmitter(void *pvParameters);
void Uart_Receiver(void *pvParameters);
void Load_1_Simulation(void *pvParameters);
void Load_2_Simulation(void *pvParameters);
/*-----------------------------------------------------------*/
/* Task to be created. */

pinState_t buttonState;
pinState_t button1State = 0;
pinState_t button2State = 0;
QueueHandle_t xQueue1;/* Declare a variable of type QueueHandle_t. This is used to store the handleto the queue that is accessed by all three tasks. */
QueueHandle_t xQueue2;
QueueHandle_t xQueueTx;
/* Define an enumerated type used to identify the source of the data. */

typedef struct DT{
	 char* pdata;
	 uint8_t len;
}xData;

int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	xQueue1 = xQueueCreate( 8, sizeof(xData) );
    /* Create Tasks here */
 xTaskPeriodicCreate( Button_1_Monitor,               //TaskFunction_t
										( const char * ) "Button_1_Monitor",  //pcName
											configMINIMAL_STACK_SIZE,  //usStackDepth
											NULL,											//xTaskParameters
											1, 												//uxPriority
											&task_1_Handler											//Handler
											,Button_1_Monitor_PERIOD 
										);								//period
 xTaskPeriodicCreate( Button_2_Monitor, 
										( const char * ) "Button_2_Monitor",
											configMINIMAL_STACK_SIZE,
											NULL,													//xTaskParameters
											1,
											&task_2_Handler 
										  ,Button_2_Monitor_PERIOD 
										 );
 xTaskPeriodicCreate( Periodic_Transmitter, 
										( const char * ) "Transmit_Tsk",
											configMINIMAL_STACK_SIZE,
											NULL,												//xTaskParameters
											1,
											&task_3_Handler
										  ,Periodic_Transmitter_PERIOD 
										 );
 xTaskPeriodicCreate( Uart_Receiver, 
										( const char * ) "Uart_Receiver",
											configMINIMAL_STACK_SIZE,
											NULL,												//xTaskParameters
											1,
											&task_4_Handler
										  ,Uart_Receiver_PERIOD 
										 );
 xTaskPeriodicCreate( Load_1_Simulation, 
										( const char * ) "Load_1_Simulation",
											configMINIMAL_STACK_SIZE,
											NULL,												//xTaskParameters
											1,
											&task_5_Handler
										  ,Load_1_Simulation_PERIOD 
										 );
 xTaskPeriodicCreate( Load_2_Simulation, 
										( const char * ) "Load_2_Simulation",
											configMINIMAL_STACK_SIZE,
											NULL,												//xTaskParameters
											1,
											&task_6_Handler
										  ,Load_2_Simulation_PERIOD 
										 );
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);


	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1(); //this timer is configured to be 20x the speed of systick

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}

void vApplicationTickHook(void)
{
	GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
}
//when there is no task running, idle task calls idle hook in infinte loop
void vApplicationIdleHook(void)
{
//	GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
	
}
/*
This task will monitor rising and falling edge on button 1 and (send this 
	[event] to the consumer task). (Note: The rising and failling edges are 
	treated as separate [events], hence they have separate strings
*/

void Button_1_Monitor (void *pvParameters)
 {
	 BaseType_t xStatus;
	 TickType_t xLastWakeTimeA;	 	
	 char AString[20];// = "This is TSK 1\n";
	 uint8_t siz = sizeof(AString);
	 xData data;
	 const TickType_t xFrequency = Button_1_Monitor_PERIOD; //tsk A frequency
	 static int count = 0 ;	 
	 vTaskSetApplicationTaskTag(
                        NULL, //NULL means the current task we are working from
                        (void*)1 //tag idintifier 
													);
 // Initialise the xLastWakeTime variable with the current time.
	data.pdata= AString;
	data.len =	siz;
	xLastWakeTimeA = xTaskGetTickCount();
 while(1)
 {
//	 TickType_t xTime = xTaskGetTickCount ();
	 
	 button1State = GPIO_read(PORT_0,PIN0);
	 if(button1State == PIN_IS_HIGH)
	 {
		 memcpy(AString,"BT1_Pressed\n",sizeof("BT1_Pressed\n"));
		 data.len = sizeof("BT1_Pressed\n");
		 xQueueSend(xQueue1, (void*) &data, (TickType_t)0);//don't block if queue is already full
	 }else{
		 memcpy(AString,"BT1_released\n",sizeof("BT1_released\n"));
		 data.len = sizeof("BT1_released\n");
		 xQueueSend(xQueue1, (void*) &data, (TickType_t)0); //don't block if queue is already full
	 }
	 
	 vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
 }

 }
/*
This task will monitor rising and falling edge on button 2 and (send this 
	[event] to the consumer task). (Note: The rising and failling edges are 
	treated as separate [events], hence they have separate strings
*/
void Button_2_Monitor (void *pvParameters)
{
  TickType_t xLastWakeTimeB;
	BaseType_t xStatus;
	char AString[20] ;//= "This is TSK 2\n";
	 uint8_t siz = sizeof(AString);
	 xData data;
  const TickType_t xFrequency = Button_2_Monitor_PERIOD; //tsk B frequency
	vTaskSetApplicationTaskTag(
                        NULL, //NULL means the current task we are working from
                        (void*)2 //tag idintifier 
													);
  // Initialise the xLastWakeTime variable with the current time.
	data.pdata= AString;
	data.len =	siz;
	xLastWakeTimeB = xTaskGetTickCount();
  while(1)
  {
		 button2State = GPIO_read(PORT_0,PIN1);
		 if(button2State == PIN_IS_HIGH)
		 {
			 memcpy(AString,"BT2_Pressed\n",sizeof("BT2_Pressed\n"));
			 data.len = sizeof("BT2_Pressed\n");
			 xQueueSend(xQueue1, (void*) &data, (TickType_t)0);
		 }else{
			 memcpy(AString,"BT2_released\n",sizeof("BT2_released\n"));
			 data.len = sizeof("BT2_released\n");
			 xQueueSend(xQueue1, (void*) &data, (TickType_t)0);
		 }
		
		 vTaskDelayUntil( &xLastWakeTimeB, xFrequency );
  }

 }
 /*
 This task will (send preiodic string) every 100ms (to the consumer task)
 */
 void Periodic_Transmitter(void *pvParameters)
 {
	  TickType_t xLastWakeTimeTx ;
	  BaseType_t xStatus;
	  const TickType_t xFrequency = Periodic_Transmitter_PERIOD; //tsk B frequency
	  char arr[30] = "cpu_load:";
	  uint8_t siz = sizeof("cpu_load:");
	  xData data;
	  data.pdata	= arr;
	  data.len = siz;
	  vTaskSetApplicationTaskTag(
                        NULL, //NULL means the current task we are working from
                        (void*)3 //tag idintifier 
													);
  xLastWakeTimeTx = xTaskGetTickCount();
		while(1)
		{
				xQueueSend(xQueue1, (void*) &data, (TickType_t)10);	
				vTaskDelayUntil( &xLastWakeTimeTx, xFrequency );
		}
												
 }
 /*
 ((This is the consumer task)) which will write on UART any received string from other tasks
 */
//char ReceiveArr[60];
 void Uart_Receiver(void *pvParameters)
 {
	  TickType_t xLastWakeTimeReceive;
	  BaseType_t xStatus;
	  portCHAR *pcReceivedString;
	  uint8_t msgLen  = 0;
	  const TickType_t xFrequency = Uart_Receiver_PERIOD; //tsk B frequency
	  xData data1;
	  xData data2;
	  xData dataTx;
	  vTaskSetApplicationTaskTag(
                        NULL, //NULL means the current task we are working from
                        (void*)4 //tag idintifier 
													);
	 xLastWakeTimeReceive = xTaskGetTickCount();		
		while(1)
		{
				if(xQueueReceive(xQueue1, &data1, (TickType_t)10))
				{
					vSerialPutString(data1.pdata,data1.len-1);
					if(!memcmp(data1.pdata,"cpu_load:",data1.len-1)){
						xSerialPutChar((cpu_load/10)+48);
						xSerialPutChar((cpu_load%10)+48);
						xSerialPutChar('\n');
					}
				}

				vTaskDelayUntil( &xLastWakeTimeReceive, xFrequency );
		}
												
 }
//Execution time: 5ms
 void Load_1_Simulation(void *pvParameters)
 {
	  TickType_t xLastWakeTimeSimu1;
	  uint32_t i;
	  const TickType_t xFrequency = Load_1_Simulation_PERIOD; //tsk B frequency
		vTaskSetApplicationTaskTag(
                        NULL, //NULL means the current task we are working from
                        (void*)5 //tag idintifier 
													);
		xLastWakeTimeSimu1 = xTaskGetTickCount();
		while(1)
		{
			for(i=0;i<37000;i++);
			vTaskDelayUntil( &xLastWakeTimeSimu1, xFrequency );
		}
												
 }
//Execution time: 12ms
  void Load_2_Simulation(void *pvParameters)
 {
	  TickType_t xLastWakeTimeSimu2 ;
	  uint32_t i;
	  const TickType_t xFrequency = Load_2_Simulation_PERIOD; //tsk B frequency
	 
		vTaskSetApplicationTaskTag(
                        NULL, //NULL means the current task we are working from
                        (void*)6 //tag idintifier 
													);
		xLastWakeTimeSimu2 = xTaskGetTickCount();
		while(1)
		{
			for(i=0;i<88000;i++);
			vTaskDelayUntil( &xLastWakeTimeSimu2, xFrequency );
		}
 }

