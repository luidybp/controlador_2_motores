#ifndef PID_H
#define PID_H

// Declaração do Molde do Controlador PI (5 Hz)
typedef struct {
    float Kp;            // Ganho Proporcional
    float Ki;            // Ganho Integral
    float Ts;            // Tempo de amostragem em segundos (0.2f para 5 Hz)
    float soma_integral; // Acumulador da ação integral
    float limite_max;    // Limite superior de saída (1000 - pwm_max)
    float limite_min;    // Limite inferior de saída (0.0f)
} PID_Controller;

// Assinaturas das funções para que outros arquivos saibam que elas existem
float calcular_PID(PID_Controller *pid, float setpoint, float leitura_atual);
void resetar_PID(PID_Controller *pid);

#endif