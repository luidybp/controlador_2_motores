#ifndef PID_H
#define PID_H

// Declaração do Molde
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ts;
    float erro_anterior;
    float soma_integral;
    float limite_max;
    float limite_min;
} PID_Controller;

// Assinatura da função para que outros arquivos saibam que ela existe
float calcular_PID(PID_Controller *pid, float setpoint, float leitura_atual);

#endif