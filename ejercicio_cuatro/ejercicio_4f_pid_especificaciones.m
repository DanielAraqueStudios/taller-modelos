%% EJERCICIO 4F: PID CON ESPECIFICACIONES - SISTEMA DE PRESIÓN
clear all; close all; clc;
load('ejercicio_cuatro/datos_sistema.mat');
load('ejercicio_cuatro/resultados_lazo_abierto.mat');
fprintf('EJERCICIO 4F: PID con Especificaciones\n\n');
ts_objetivo = 0.92*ts_lazo_abierto_2; zeta_objetivo = 2;
Kp_spec = 25; Ki_spec = 1.5; Kd_spec = 90;
fprintf('ts objetivo=%.2f s, zeta=%.2f\nKp=%.2f, Ki=%.2f, Kd=%.2f\n', ts_objetivo, zeta_objetivo, Kp_spec, Ki_spec, Kd_spec);
C_spec = pid(Kp_spec, Ki_spec, Kd_spec);
sys_cl_spec = feedback(C_spec*G, 1);
polos_cl = pole(sys_cl_spec);
ess_escalon = 1 - dcgain(sys_cl_spec);
figure; subplot(2,2,1); step(sys_cl_spec, 100); title('PID Espec - Escalón'); grid on;
subplot(2,2,2); step(G, sys_cl_spec, 100); legend('LA', 'LC'); grid on;
subplot(2,2,[3,4]); rlocus(C_spec*G); grid on;
save('ejercicio_cuatro/pid_especificaciones.mat', 'C_spec', 'sys_cl_spec', 'Kp_spec', 'Ki_spec', 'Kd_spec', 'polos_cl', 'ess_escalon');
saveas(gcf, 'ejercicio_cuatro/pid_especificaciones.png');
fprintf('ess=%.6f\nResultados guardados\n', ess_escalon);
