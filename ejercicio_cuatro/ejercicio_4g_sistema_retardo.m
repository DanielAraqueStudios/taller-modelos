%% EJERCICIO 4G: SISTEMA CON RETARDO - PRESIÓN
clear all; close all; clc;
load('ejercicio_cuatro/datos_sistema.mat');
load('ejercicio_cuatro/pid_ziegler_nichols.mat');
fprintf('EJERCICIO 4G: Sistema con Retardo\n\n');
tau_delay = 0.7;
pade_num = [1, -tau_delay/2]; pade_den = [1, tau_delay/2];
H_delay = tf(pade_num, pade_den);
G_delay = G*H_delay;
Kp_delay1 = 0.7*Kp_zn; Ki_delay1 = 0.65*Ki_zn; Kd_delay1 = 0.85*Kd_zn;
C_delay1 = pid(Kp_delay1, Ki_delay1, Kd_delay1);
sys_cl_delay1 = feedback(C_delay1*G_delay, 1);
fprintf('tau_delay=%.2f s\nKp=%.2f, Ki=%.2f, Kd=%.2f\n', tau_delay, Kp_delay1, Ki_delay1, Kd_delay1);
figure; step(sys_cl_zn, sys_cl_delay1, 100); legend('Sin retardo', 'Con retardo conservador'); grid on;
save('ejercicio_cuatro/sistema_retardo.mat', 'tau_delay', 'G_delay', 'H_delay', 'C_delay1', 'Kp_delay1', 'Ki_delay1', 'Kd_delay1', 'sys_cl_delay1');
saveas(gcf, 'ejercicio_cuatro/sistema_retardo.png');
fprintf('Resultados guardados\n');
