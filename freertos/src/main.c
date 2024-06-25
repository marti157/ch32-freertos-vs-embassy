/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/29
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "debug.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

void I2C2_EV_IRQHandler(void) __attribute__((interrupt()));
void I2C2_ER_IRQHandler(void) __attribute__((interrupt()));

__attribute__( ( always_inline ) ) RV_STATIC_INLINE void enable_interrupts()
{
    I2C2->CTLR2 |= 0x700;
}

__attribute__( ( always_inline ) ) RV_STATIC_INLINE void disable_interrupts()
{
    I2C2->CTLR2 &= (uint16_t)~0x700;
}

/* Global define */
#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      256
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      256

/* Global Variable */
TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;
SemaphoreHandle_t xI2CSemaphore = NULL;

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0/1
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      IIC_Init
 *
 * @brief   Initializes the IIC peripheral.
 *
 *  PB11------SDA
 *  PB10------SCL
 *
 * @return  none
 */
void IIC_Init(u32 bound, u16 address)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C2, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C2, &I2C_InitTSturcture );

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    I2C_Cmd( I2C2, ENABLE );
}

/*********************************************************************
 * @fn      I2C_ReadOneByte
 *
 * @brief   Read one data via I2C.
 *
 * @param   ReadAddr - Read frist address.
 *
 * @return  temp - Read data.
 */
u8 I2C_ReadOneByte(u16 ReadAddr)
{
    u8 temp = 0;
    while( I2C_GetFlagStatus( I2C2, I2C_FLAG_BUSY ) != RESET );

    disable_interrupts();

    GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
    
    I2C_GenerateSTART( I2C2, ENABLE );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C2->DATAR = (0x77 << 1); // Send address

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

    I2C_SendData( I2C2, (u8)(ReadAddr&0x00FF) );
    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

    I2C_GenerateSTART( I2C2, ENABLE );

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_MODE_SELECT ) );

    I2C2->DATAR = (0x77 << 1) + 1; // Send address

    while( !I2C_CheckEvent( I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) );

    // Clear condition by reading SR2
    ((void)I2C_ReadRegister(I2C2, I2C_Register_STAR2));

    enable_interrupts();

    I2C2->CTLR1 = 0x201; // ACK + STOP

    xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);

    temp = I2C_ReceiveData( I2C2 );

    return temp;
}

/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
    int i;
    u8 id;
    while(1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);

        for (i = 0; i < 1000; i++) {
            id = I2C_ReadOneByte(0xD0);
        }

        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);

        Delay_Ms(1);
    }
}

/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 */
void task2_task(void *pvParameters)
{
    while(1)
    {
        printf("Long running task\n");
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        vTaskDelay(250);
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        vTaskDelay(250);
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    IIC_Init(100000, 0XA0);
    printf("SystemClk:%d\n",SystemCoreClock);
    printf( "ChipID:%08x\n", DBGMCU_GetCHIPID() );
    printf("FreeRTOS Kernel Version:%s\n",tskKERNEL_VERSION_NUMBER);
    GPIO_Toggle_INIT();

    xI2CSemaphore = xSemaphoreCreateBinary();

    /* create two tasks */
    xTaskCreate((TaskFunction_t )task2_task,
                        (const char*    )"task2",
                        (uint16_t       )TASK2_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TASK2_TASK_PRIO,
                        (TaskHandle_t*  )&Task2Task_Handler);

    xTaskCreate((TaskFunction_t )task1_task,
                    (const char*    )"task1",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
    vTaskStartScheduler();

    while(1)
    {
        printf("Shouldn't reach here\n");
        Delay_Ms(1000);
    }
}

void I2C2_EV_IRQHandler(void)
{
    I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
    I2C_ITConfig(I2C2, I2C_IT_EVT, DISABLE);
    I2C_ITConfig(I2C2, I2C_IT_ERR, DISABLE);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Give the semaphore to unblock the I2C task
    xSemaphoreGiveFromISR(xI2CSemaphore, &xHigherPriorityTaskWoken);

    // Request a context switch if giving the semaphore unblocked a higher priority task
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void I2C2_ER_IRQHandler(void)
{
    printf("I2C2_ER\n");
    I2C_ITConfig(I2C2, I2C_IT_BUF, DISABLE);
    I2C_ITConfig(I2C2, I2C_IT_EVT, DISABLE);
    I2C_ITConfig(I2C2, I2C_IT_ERR, DISABLE);
}
