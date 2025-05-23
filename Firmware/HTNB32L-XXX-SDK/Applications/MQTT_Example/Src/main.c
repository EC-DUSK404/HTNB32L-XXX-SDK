/**
 *
 * Copyright (c) 2023 HT Micron Semicondutores S.A.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "slpman_qcx212.h"
#include "pad_qcx212.h"
#include "HT_gpio_qcx212.h"
#include "ic_qcx212.h"
#include "HT_ic_qcx212.h"
#include "semphr.h"
#include "htnb32lxxx_hal_usart.h"
#include "stdlib.h"
#include "adc_qcx212.h"
#include <stdio.h>


static uint32_t uart_cntrl = (ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE |
                                ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE);

extern USART_HandleTypeDef huart1;

SemaphoreHandle_t xSemaforo;

//GPIO10 - BUTTON

#define BUTTON_INSTANCE          0                  /**</ Button pin instance. */
#define BUTTON_PIN               10                 /**</ Button pin number. */
#define BUTTON_PAD_ID            25                 /**</ Button Pad ID. */
#define BUTTON_PAD_ALT_FUNC      PAD_MuxAlt0        /**</ Button pin alternate function. */

//GPIO3 - LED
#define LED_INSTANCE             0                  /**</ LED pin instance. */
#define LED_GPIO_PIN             3                  /**</ LED pin number. */
#define LED_PAD_ID               14                 /**</ LED Pad ID. */
#define LED_PAD_ALT_FUNC         PAD_MuxAlt0        /**</ LED pin alternate function. */

//GPIO4 - LED
#define LED2_INSTANCE             0                  /**</ LED pin instance. */
#define LED2_GPIO_PIN             4                  /**</ LED pin number. */
#define LED2_PAD_ID               15                 /**</ LED Pad ID. */
#define LED2_PAD_ALT_FUNC         PAD_MuxAlt0        /**</ LED pin alternate function. */

//GPIO5 - LED
#define LED3_INSTANCE             0                  /**</ LED pin instance. */
#define LED3_GPIO_PIN             5                  /**</ LED pin number. */
#define LED3_PAD_ID               16                 /**</ LED Pad ID. */
#define LED3_PAD_ALT_FUNC         PAD_MuxAlt0        /**</ LED pin alternate function. */

#define LED_ON  1                                   /**</ LED on. */
#define LED_OFF 0                                   /**</ LED off. */

volatile bool button_state = false;
volatile uint32_t led_time = 500;
// char cmdRxBuffer[255] = {0};

// volatile uint8_t rx_callback = 0;
// volatile uint8_t tx_callback = 0;

// void HT_USART_Callback(uint32_t event) {
//     if(event & ARM_USART_EVENT_RECEIVE_COMPLETE)
//         rx_callback = 1;
// }

static void HT_GPIO_InitButton(void) {
  GPIO_InitType GPIO_InitStruct = {0};

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = BUTTON_PAD_ID;
  GPIO_InitStruct.gpio_pin = BUTTON_PIN;
  GPIO_InitStruct.pin_direction = GPIO_DirectionInput;
  GPIO_InitStruct.pull = PAD_InternalPullUp;
  GPIO_InitStruct.instance = BUTTON_INSTANCE;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;
  GPIO_InitStruct.interrupt_config = GPIO_InterruptFallingEdge;

  HT_GPIO_Init(&GPIO_InitStruct);
}

static void HT_GPIO_InitLed(void) {
  GPIO_InitType GPIO_InitStruct = {0};

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = LED_PAD_ID;
  GPIO_InitStruct.gpio_pin = LED_GPIO_PIN;
  GPIO_InitStruct.pin_direction = GPIO_DirectionOutput;
  GPIO_InitStruct.init_output = 1;
  GPIO_InitStruct.pull = PAD_AutoPull;
  GPIO_InitStruct.instance = LED_INSTANCE;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;

  HT_GPIO_Init(&GPIO_InitStruct);
}

static void HT_GPIO_InitLed2(void) {
  GPIO_InitType GPIO_InitStruct = {0};

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = LED2_PAD_ID;
  GPIO_InitStruct.gpio_pin = LED2_GPIO_PIN;
  GPIO_InitStruct.pin_direction = GPIO_DirectionOutput;
  GPIO_InitStruct.init_output = 1;
  GPIO_InitStruct.pull = PAD_AutoPull;
  GPIO_InitStruct.instance = LED2_INSTANCE;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;

  HT_GPIO_Init(&GPIO_InitStruct);
}

static void HT_GPIO_InitLed3(void) {
  GPIO_InitType GPIO_InitStruct = {0};
  
  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = LED3_PAD_ID;
  GPIO_InitStruct.gpio_pin = LED3_GPIO_PIN;
  GPIO_InitStruct.pin_direction = GPIO_DirectionOutput;
  GPIO_InitStruct.init_output = 1;
  GPIO_InitStruct.pull = PAD_AutoPull;
  GPIO_InitStruct.instance = LED3_INSTANCE;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;

  HT_GPIO_Init(&GPIO_InitStruct);
}


void Task1(void *pvParameters) {
    bool estado_anterior = 1;
    while (1) {
        bool estado_atual = !HT_GPIO_PinRead(BUTTON_INSTANCE, BUTTON_PIN);  // Pressionado = 0
        if (estado_atual == 0 && estado_anterior == 1) {  // Transição de solto → pressionado
            xSemaphoreGive(xSemaforo);  // Sinaliza para a Task2
        }
        estado_anterior = estado_atual;
        vTaskDelay(pdMS_TO_TICKS(50));  // Pequeno delay para debounce
    }
}



void Task2(void *pvParameters) {
    while (1) {
        // if (rx_callback == 1)
        // {
        //   led_time = atoi(cmdRxBuffer);
        //   rx_callback = 0;
        // }
        printf("led_time: %d \n",led_time );
        HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_ON);
        vTaskDelay(pdMS_TO_TICKS(led_time));
        HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(led_time));
    }
}

void TaskUart(void *pvParameters)
{
  uint8_t rx_buffer_usart;
  char cmdRxBuffer[20] = {0};
  uint8_t cmdRxBufferIdx = 0;
  while (1)
  {
    HAL_USART_ReceivePolling(&huart1, &rx_buffer_usart, 1);
    if (rx_buffer_usart != 0)
    {
      printf("%c\n",rx_buffer_usart);
      cmdRxBuffer[cmdRxBufferIdx] = rx_buffer_usart;
      cmdRxBufferIdx++;
      if (rx_buffer_usart == '\r')
      {
        printf("Recebido UART: %s \n",cmdRxBuffer);
        led_time = atoi(cmdRxBuffer);
        printf("Recebido UART: %s \n",cmdRxBuffer);
        memset(&cmdRxBuffer, 0, cmdRxBufferIdx);
        cmdRxBufferIdx = 0;
        HAL_USART_Control(ARM_USART_CONTROL_PURGE_COMM, 0, &huart1);
      }
     
      rx_buffer_usart = 0;
    }
  }
}

void HT_App(void) {
    char case_num;
    char stop_1;
    char stop_2;
    
    while(1) {
        printf("Case 0: 100ms\r\n");
        printf("Case 1: 400ms\r\n");
        printf("Case 2: 700ms\r\n");
        printf("Case 3: 1000ms\r\n");
        
        __set_BASEPRI(0);

        case_num = (char)fgetc(NULL);
        printf("%c\r\n",case_num);

        switch(case_num) {
            case '0':
                while(1){
                    printf("Quantas repetiçoes deseja?");
                    stop_1 = (char)fgetc(NULL);
                    printf("%c\r\n",stop_1);
                    if(stop_1 == NULL){
                    
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_ON);
                      delay_us(100000);
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_OFF);
                      delay_us(100000);
                    }
                    else{
                      break;
                    }
                }
            break;
            case '1':
                while(1){
                    stop_1 = (char)fgetc(NULL);
                    printf("%c\r\n",stop_1);
                    if(stop_1 == NULL){
                    
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_OFF);
                      delay_us(400000);
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_ON);
                      delay_us(400000);
                    }
                    else{
                      break;
                    }
                }
            break;    
            case '2':
                while(1){
                    stop_1 = (char)fgetc(NULL);
                    printf("%c\r\n",stop_1);
                    if(stop_1 == NULL){
                    
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_ON);
                      delay_us(700000);
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_OFF);
                      delay_us(700000);
                    }
                    else{
                      break;
                    }
                }
            break;
            case '3':
                while(1){
                     stop_1 = (char)fgetc(NULL);
                     printf("%c\r\n",stop_1);
                     if(stop_1 == 1){
                   
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_ON);
                      delay_us(1000000);
                      HT_GPIO_WritePin(LED2_GPIO_PIN, LED2_INSTANCE, LED_OFF);
                      delay_us(1000000);
                    }
                    else{
                      break;
                    }
                }
            break;
            default: 
                printf("Error! Wrong option!\n");
            break;
        }
      printf("SE DESEJAR SAIR, DIGITE *PARA*");
      stop_2 = (char)fgetc(NULL);
      printf("%c\r\n",stop_2);
      if(stop_2 == "PARA"){
        break;
      }
    }
}



/**
  \fn          int main_entry(void)
  \brief       main entry function.
  \return
*/
void main_entry(void) {
    HAL_USART_InitPrint(&huart1, GPR_UART1ClkSel_26M, uart_cntrl, 115200);
    // HAL_USART_IRQnEnable(&huart1, (USART_IER_RX_DATA_REQ_Msk | USART_IER_RX_TIMEOUT_Msk | USART_IER_RX_LINE_STATUS_Msk));
    // HAL_USART_Receive_IT(cmdRxBuffer, 10);

    printf("Exemplo FreeRTOS\n");
//  xSemaforo = xSemaphoreCreateBinary();
// if (xSemaforo == NULL) {
//     printf("Erro ao criar a semáforo.\n");
//     while (1);
// }

    HT_GPIO_InitButton();
    HT_GPIO_InitLed();
    HT_GPIO_InitLed2();
    HT_GPIO_InitLed3();
    // HT_App();
    slpManNormalIOVoltSet(IOVOLT_3_30V);



    // xTaskCreate(Task1, "Button", 128, NULL, 1, NULL);
    xTaskCreate(Task2, "Blink", 1024, NULL, 2, NULL);
    xTaskCreate(TaskUart, "TaskUart", 1024, NULL, 1, NULL);
    
    printf("Inicializou Tasks\n");

    vTaskStartScheduler();
    
    printf("Nao deve chegar aqui.\n");

    while(1);

}

/******** HT Micron Semicondutores S.A **END OF FILE*/