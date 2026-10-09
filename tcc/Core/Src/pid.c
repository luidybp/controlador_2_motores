#include "pid.h"
/**
  * @brief  Calcula a ação de controle utilizando um controlador PID digital.
  * * @details Implementa as ações Proporcional, Integral e Derivativa no tempo discreto. 
  * Possui proteção de saturação da ação integral (Anti-Windup) e limitação 
  * da saída final para garantir que o esforço de controle respeite os limites 
  * físicos do hardware (ex: limites de PWM).
  * * @param  pid Ponteiro para a estrutura PID_Controller que guarda os ganhos (Kp, Ki, Kd), 
  * o tempo de amostragem (Ts) e a memória de estado do motor.
  * @param  setpoint Valor de referência desejado pelo utilizador (ex: velocidade ideal).
  * @param  leitura_atual Variável de processo (PV) medida atualmente no sistema (ex: Back-EMF).
  * * @retval float Retorna o sinal de esforço de controle final, já saturado e pronto 
  * para ser aplicado ao atuador.
  */
float calcular_PID(PID_Controller *pid, float setpoint, float leitura_atual) {
    
    // 1. Calcula o Erro (Desejado - Real)
    float erro = setpoint - leitura_atual;

    // 2. Ação Proporcional (P)
    float P = pid->Kp * erro;

    // 3. Ação Integral Discreta (I)
    // O erro é multiplicado pelo tempo de amostragem (Ts)
    pid->soma_integral += (pid->Ki * erro * pid->Ts);
    
    // ANTI-WINDUP: Trava a integral se ela estourar os limites físicos
    if (pid->soma_integral > pid->limite_max) {
        pid->soma_integral = pid->limite_max;
    } else if (pid->soma_integral < pid->limite_min) {
        pid->soma_integral = pid->limite_min;
    }

    // 4. Ação Derivativa Discreta (D)
    // Taxa de variação do erro dividida pelo tempo de amostragem
    float D = pid->Kd * ((erro - pid->erro_anterior) / pid->Ts);
    
    // Salva o erro atual para o próximo ciclo
    pid->erro_anterior = erro;

    // 5. Soma a Ação de Controle (Sinal de esforço)
    float saida = P + pid->soma_integral + D;

    // Saturação final da saída de controle
    if (saida > pid->limite_max) saida = pid->limite_max;
    else if (saida < pid->limite_min) saida = pid->limite_min;

    return saida;
}


