/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "pid.h"
//#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(a,b) (((a)>(b))?(a):(b))

// Mapeia linearmente um valor (x) que está numa escala (in_min a in_max) 
// para uma nova escala proporcional (out_min a out_max)
#define MAP(x, in_min, in_max, out_min, out_max) (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

// Constante de resolução do Hardware ADC de 12 bits
#define ADC_MAX_VAL 4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t sistema_liberado;
extern uint32_t pot1;
extern uint32_t pot2;
extern float pot1_fiir;
extern float pot2_fiir;
extern uint16_t pwm_max;
extern float alpha;
extern volatile uint16_t valores_adc[3];
extern TIM_HandleTypeDef htim1;
uint8_t DelayControle = 5;
extern uint32_t emfmot1;
extern uint32_t emfmot2;
uint8_t divisor_lcd = 0;
/* USER CODE END Variables */
/* Definitions for TaskControle */
osThreadId_t TaskControleHandle;
const osThreadAttr_t TaskControle_attributes = {
  .name = "TaskControle",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskIHM */
osThreadId_t TaskIHMHandle;
const osThreadAttr_t TaskIHM_attributes = {
  .name = "TaskIHM",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void funcao_51(void);
extern void verifica_botao(void);
extern void inicializacao(void);
extern void emf(void);
extern void zera_pwm(void);
//extern void atualiza_display(void);
/* USER CODE END FunctionPrototypes */

void StartTaskControle(void *argument);
void StartTaskIHM(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskControle */
  TaskControleHandle = osThreadNew(StartTaskControle, NULL, &TaskControle_attributes);

  /* creation of TaskIHM */
  TaskIHMHandle = osThreadNew(StartTaskIHM, NULL, &TaskIHM_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskControle */
/**
  * @brief  Function implementing the TaskControle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskControle */
void StartTaskControle(void *argument)
{
  /* USER CODE BEGIN StartTaskControle */

  PID_Controller pid_motor1;
  pid_motor1.Kp = 1.0f;     // Valores iniciais para você sintonizar depois
  pid_motor1.Ki = 0.0f;
  pid_motor1.Kd = 0.0f;
  pid_motor1.Ts = DelayControle/1000.0f;   // 5 milissegundos do osDelay(5)
  pid_motor1.erro_anterior = 0.0f;
  pid_motor1.soma_integral = 0.0f;
  pid_motor1.limite_max = (float)(1000 - pwm_max); // Seu limite de hardware
  pid_motor1.limite_min = 0.0f;


  PID_Controller pid_motor2;
  pid_motor2.Kp = 1.0f;     // Valores iniciais para você sintonizar depois
  pid_motor2.Ki = 0.0f;
  pid_motor2.Kd = 0.0f;
  pid_motor2.Ts = DelayControle/1000.0f;   // 5 milissegundos do osDelay(5)
  pid_motor2.erro_anterior = 0.0f;
  pid_motor2.soma_integral = 0.0f;
  pid_motor2.limite_max = (float)(1000 - pwm_max); // Seu limite de hardware
  pid_motor2.limite_min = 0.0f;

  /* Infinite loop */
  for(;;)
  {
    funcao_51(); 
    
    if(sistema_liberado == 1){ 

      //valores_adc é um valor que vem direto da memória do STM do ADC dos potenciometros
      //valores vao de 1 a 1000
      pot1 = MAP(valores_adc[0], 0, ADC_MAX_VAL, 0, 1000 - pwm_max); //comanda a velocidade da bola de 0 ate pwm max para ter folga nos moteres
      pot2 = MAP(valores_adc[1], 0, ADC_MAX_VAL, 0, 1000);  // comanda o spin da bola
      emf();

      //filtragem do input, uso de Filtro IIR
      //y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
      pot1_fiir = (alpha * pot1) + ((1.0f - alpha) * pot1_fiir);
      pot2_fiir = (alpha * pot2) + ((1.0f - alpha) * pot2_fiir); 

      float fator_spin = pot2_fiir / 10000.0f; //permitindo adicionar até 10% de velocidade a mais no motor 1
      float setpoint_m1 = pot1_fiir * (1.0f + fator_spin); 
      float setpoint_m2 = pot1_fiir;
      
      float limite_hardware = (float)(1000 - pwm_max);
      
      if (setpoint_m1 > limite_hardware) {
        float over = setpoint_m1 - limite_hardware; 
        setpoint_m1 = limite_hardware;
        
        if (over > setpoint_m2) {
          setpoint_m2 = 0.0f;
        } else {
          setpoint_m2 = setpoint_m2 - over;
        }
      }

      float velocidade_real_m1 = (float)emfmot1; // precisamos analisar dps se o valor medido é realmente linear com o RPM
      float velocidade_real_m2 = (float)emfmot2;


      //PID

      // Comente o cálculo do PID:
      // float esforco_m1 = calcular_PID(&pid_motor1, setpoint_m1, velocidade_real_m1);
      // float esforco_m2 = calcular_PID(&pid_motor2, setpoint_m2, velocidade_real_m2);

      // BYPASS PARA BANCADA: Jogue a leitura do potenciômetro direto para o atuador (LED)
      float esforco_m1 = setpoint_m1;
      float esforco_m2 = setpoint_m2;

      //PWM PA8 (Mantém igual)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MAX(1000-esforco_m1,pwm_max) );
      //PWM PA9 (Mantém igual)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MAX(1000-esforco_m2,pwm_max) );

      //o trigger motor é colocado no CCR do tim1 ch4, após a metade do tempo em que o PWM está em 1, o ch4 vai para 0 é a falling edge causa o trigger no ADC
      //o trigger da leitura do EMF é feito no motor 1 por ele pode estar mais acelerado que o motor 2, portanto, ele tem menos t para ler a EMF
      
      uint16_t trigger_motor = MAX(1000-esforco_m1, pwm_max) / 2;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger_motor);
    } else {
      //se o sistema não está liberado -> pwm = 0
      zera_pwm();
    }
    
    osDelay(DelayControle);
  }
  /* USER CODE END StartTaskControle */
}

/* USER CODE BEGIN Header_StartTaskIHM */
/**
* @brief Function implementing the TaskIHM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIHM */
void StartTaskIHM(void *argument)
{
  /* USER CODE BEGIN StartTaskIHM */
  /* Infinite loop */
  for(;;)
  {

    pot1 = MAP(valores_adc[0], 0, ADC_MAX_VAL, 0, 1000 - pwm_max); //comanda a velocidade da bola de 0 ate pwm max para ter folga nos moteres
    pot2 = MAP(valores_adc[1], 0, ADC_MAX_VAL, 0, 1000);  // comanda o spin da bola
    verifica_botao(); 
    
    if (sistema_liberado == 0) { 
      inicializacao();
    }

    /*
  // Só envia os textos para a tela a cada 300 ms (15 * 20 ms)
    divisor_lcd++;
    if (divisor_lcd >= 15) {
        atualiza_display();
        divisor_lcd = 0;
    }
    */
    
    osDelay(20);
  }
  /* USER CODE END StartTaskIHM */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

