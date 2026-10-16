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
extern volatile uint8_t sistema_liberado;
extern uint32_t pot1;
extern uint32_t pot2;
extern float pot1_fiir;
extern float pot2_fiir;
extern uint16_t pwm_max;
extern float alpha;
extern volatile uint16_t valores_adc[3];
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern uint16_t tensao_barramento1;
extern uint16_t tensao_barramento2;

uint8_t DelayControle = 5;  // 5 ms (200 Hz) para proteção e filtro rápido
uint8_t DelayIHM = 20;      // 20 ms (50 Hz) para botão e IHM

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
  TaskIHMHandle = osThreadNew(StartTaskIHM, NULL, &TaskControle_attributes); // Mantém atributos ou TaskIHM_attributes
  /* Correção: usa TaskIHM_attributes na criação da TaskIHM */

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

  // =========================================================================
  // SINTONIA DO CONTROLADOR PI (Alterar Kp e Ki AQUI durante os testes)
  // =========================================================================
  PID_Controller pid_motor1;
  pid_motor1.Kp = 1.0f;     // Ganho Proporcional do Motor 1
  pid_motor1.Ki = 0.0f;     // Ganho Integral do Motor 1
  pid_motor1.Ts = 0.2f;     // 200 ms (5 Hz = 40 ciclos de 5 ms)
  pid_motor1.soma_integral = 0.0f;
  pid_motor1.limite_max = (float)(1000 - pwm_max); // Limite de hardware (800)
  pid_motor1.limite_min = 0.0f;

  PID_Controller pid_motor2;
  pid_motor2.Kp = 1.0f;     // Ganho Proporcional do Motor 2
  pid_motor2.Ki = 0.0f;     // Ganho Integral do Motor 2
  pid_motor2.Ts = 0.2f;     // 200 ms (5 Hz = 40 ciclos de 5 ms)
  pid_motor2.soma_integral = 0.0f;
  pid_motor2.limite_max = (float)(1000 - pwm_max); // Limite de hardware (800)
  pid_motor2.limite_min = 0.0f;

  // Variáveis de estado da malha de 5 Hz (40 ciclos * 5 ms = 200 ms)
  uint8_t ciclo_pi = 0;
  float esforco_m1 = 0.0f;
  float esforco_m2 = 0.0f;

  /* Infinite loop */
  uint32_t tick_controle = osKernelGetTickCount();
  for(;;)
  {
    // Proteção ANSI 51 vigia a corrente todos os 5 ms (200 Hz)
    funcao_51(); 
    
    if (sistema_liberado == 1) { 

      // 1. Fotografa as leituras do DMA localmente para evitar Race Condition
      uint16_t raw_adc0 = valores_adc[0];
      uint16_t raw_adc1 = valores_adc[1];

      pot1 = MAP((int32_t)raw_adc0, 0, ADC_MAX_VAL, 0, 1000 - pwm_max); // Velocidade (0 a 800)
      pot2 = MAP((int32_t)raw_adc1, 0, ADC_MAX_VAL, 0, 1000);           // Spin (0 a 1000)

      // 2. Filtro IIR passa-baixas nos potenciômetros (atualizado a 200 Hz para resposta suave)
      pot1_fiir = (alpha * (float)pot1) + ((1.0f - alpha) * pot1_fiir);
      pot2_fiir = (alpha * (float)pot2) + ((1.0f - alpha) * pot2_fiir); 

      // 3. Cálculo dos Setpoints com compensação de saturação do Spin
      float fator_spin = pot2_fiir / 10000.0f; // Até +10% no Motor 1
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

      // =====================================================================
      // MÁQUINA DE TEMPORIZAÇÃO DE 5 Hz (Período = 200 ms = 40 ciclos de 5 ms)
      // =====================================================================
      if (ciclo_pi < 38) 
      {
        // --- CICLOS 0 A 37 (0 a 190 ms): Motores alimentados normalmente ---
        
        // (Opcional em bancada sem motor): Para acompanhar o potenciômetro em malha aberta,
        // descomente as duas linhas abaixo. Quando ativar o PI no Ciclo 39, deixe-as comentadas:
        esforco_m1 = setpoint_m1;
        esforco_m2 = setpoint_m2;

        uint16_t ccr_m1 = (uint16_t)MAX(1000.0f - esforco_m1, (float)pwm_max);
        uint16_t ccr_m2 = (uint16_t)MAX(1000.0f - esforco_m2, (float)pwm_max);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_m1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_m2);

        // Posiciona o trigger na metade do tempo de chave aberta (off-time) do Motor 1
        // Aqui o diodo de roda livre está conduzindo em 1 kHz (mede a tensão do barramento)
        uint16_t trigger_motor = ccr_m1 / 2;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger_motor);

        ciclo_pi++;
      }
      else if (ciclo_pi == 38) 
      {
        // --- CICLO 38 (Aos 195 ms): 1ª Medição (Barramento) e Abertura por 5 ms ---
        
        // 1ª Medição: Antes de desligar o PWM de 1 kHz, o ADC Injetado acabou de ler
        // a tensão com o diodo de roda livre conduzindo (Tensão Instantânea do Barramento)
        tensao_barramento1 = (uint16_t)MAP((int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1), 0, ADC_MAX_VAL, 0, 1000);
        tensao_barramento2 = (uint16_t)MAP((int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2), 0, ADC_MAX_VAL, 0, 1000);

        // Corta os PWMs para iniciar a janela de 5 ms de descarga da corrente do indutor
        zera_pwm();

        // Mantém o Canal 4 gerando trigger no meio do período de 1 kHz (CCR4 = 500)
        // para que o ADC Injetado continue amostrando durante os 5 ms de chave aberta
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 500);

        ciclo_pi++;
      }
      else 
      {
        // --- CICLO 39 (Aos 200 ms): 2ª Medição (Back-EMF), Cálculo do PI (5 Hz) e Religação ---
        
        // Após 5 ms de chave aberta, a corrente do diodo caiu a zero.
        // A função emf() faz a 2ª medição (Vbus - Vbemf) e calcula a diferença contra tensao_barramento1/2
        emf();

        float velocidade_real_m1 = (float)emfmot1;
        float velocidade_real_m2 = (float)emfmot2;

        // Cálculo do PI a 5 Hz (Descomente quando for fechar a malha com os motores):
        // esforco_m1 = calcular_PID(&pid_motor1, setpoint_m1, velocidade_real_m1);
        // esforco_m2 = calcular_PID(&pid_motor2, setpoint_m2, velocidade_real_m2);

        // BYPASS PARA BANCADA (Comente as duas linhas abaixo quando descomentar o calcular_PID acima):
        esforco_m1 = setpoint_m1;
        esforco_m2 = setpoint_m2;

        // Religa imediatamente os PWMs com o novo esforço calculado pelo PI
        uint16_t ccr_m1 = (uint16_t)MAX(1000.0f - esforco_m1, (float)pwm_max);
        uint16_t ccr_m2 = (uint16_t)MAX(1000.0f - esforco_m2, (float)pwm_max);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_m1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_m2);

        uint16_t trigger_motor = ccr_m1 / 2;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, trigger_motor);

        // Reinicia o ciclo de 200 ms (5 Hz)
        ciclo_pi = 0;
      }

    } else {
      // 1. Corta imediatamente o chaveamento de potência (CCR = 1000 na lógica invertida)
      zera_pwm();

      // 2. Mantém o canal fantasma (CH4) em 500 para o ADC Injetado funcionar na calibração
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 500);

      // 3. Reseta a memória integral dos controladores PI (Motor 1 e Motor 2)
      resetar_PID(&pid_motor1);
      resetar_PID(&pid_motor2);

      // 4. Zera os esforços, o contador de ciclo de 5 Hz e a memória do filtro IIR
      esforco_m1 = 0.0f;
      esforco_m2 = 0.0f;
      ciclo_pi = 0;
      pot1_fiir = 0.0f;
      pot2_fiir = 0.0f;
    }
    
    tick_controle += DelayControle;
    osDelayUntil(tick_controle);
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
  uint32_t tick_IHM = osKernelGetTickCount();
  for(;;)
  {
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
    tick_IHM += DelayIHM;
    osDelayUntil(tick_IHM);
  }
  /* USER CODE END StartTaskIHM */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */