#include "pid.h"

/**
  * @brief  Calcula a ação de controle utilizando um controlador PI digital (5 Hz / Ts = 0.2s).
  * @details Implementa as ações Proporcional e Integral no tempo discreto.
  * Possui proteção de saturação dinâmica da ação integral (Anti-Windup) e 
  * corte imediato em referência nula para garantir parada total do motor.
  * @param  pid Ponteiro para a estrutura PID_Controller que guarda os ganhos (Kp, Ki),
  * o tempo de amostragem (Ts = 0.2f) e a memória de estado do motor.
  * @param  setpoint Valor de referência desejado pelo utilizador (escala 0 a 800).
  * @param  leitura_atual Variável de processo (PV) medida via Back-EMF.
  * @retval float Retorna o sinal de esforço de controle final, já saturado entre
  * limite_min e limite_max, pronto para ser aplicado ao PWM.
  */
float calcular_PID(PID_Controller *pid, float setpoint, float leitura_atual) {
    
    // 0. Proteção de Parada: se a referência for zero (ou faixa morta inicial),
    // limpa a memória integral imediatamente para o motor não continuar girando sozinho
    if (setpoint <= 0.0f) {
        pid->soma_integral = 0.0f;
        return 0.0f;
    }

    // 1. Calcula o Erro (Desejado - Real)
    float erro = setpoint - leitura_atual;

    // 2. Ação Proporcional (P)
    float P = pid->Kp * erro;

    // 3. Ação Integral Discreta (I)
    // Para 5 Hz, pid->Ts deve ser configurado como 0.2f (200 ms)
    pid->soma_integral += (pid->Ki * erro * pid->Ts);
    
    // ANTI-WINDUP (Clamping da Integral): Garante que a integral nunca ultrapasse os limites físicos
    if (pid->soma_integral > pid->limite_max) {
        pid->soma_integral = pid->limite_max;
    } else if (pid->soma_integral < pid->limite_min) {
        pid->soma_integral = pid->limite_min;
    }

    // 4. Soma a Ação de Controle PI (Sinal de esforço sem parcela Derivativa)
    float saida = P + pid->soma_integral;

    // 5. Saturação final da saída e Anti-Windup Dinâmico (Back-Calculation)
    // Impede que a soma (P + I) acumule integral além do limite máximo do PWM
    if (saida > pid->limite_max) {
        saida = pid->limite_max;
        // Se P sozinho for menor que o teto, ajusta a integral para o teto exato restante
        if (P < pid->limite_max) {
            pid->soma_integral = pid->limite_max - P;
        } else {
            pid->soma_integral = 0.0f;
        }
    } else if (saida < pid->limite_min) {
        saida = pid->limite_min;
        if (P > pid->limite_min) {
            pid->soma_integral = pid->limite_min - P;
        } else {
            pid->soma_integral = 0.0f;
        }
    }

    return saida;
}

/**
  * @brief  Reseta o acumulador integral do controlador PI.
  * @note   Deve ser chamada quando sistema_liberado == 0 ou após trip da ANSI 50/51.
  * @param  pid Ponteiro para a estrutura PID_Controller do motor.
  * @retval None
  */
void resetar_PID(PID_Controller *pid) {
    pid->soma_integral = 0.0f;
}