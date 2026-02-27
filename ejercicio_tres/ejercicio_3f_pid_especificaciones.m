%% EJERCICIO 3F: PID CON ESPECIFICACIONES - MASA-CILINDRO
clear all; close all; clc;
load('ejercicio_tres/datos_sistema.mat');
load('ejercicio_tres/resultados_lazo_abierto.mat');
fprintf('EJERCICIO 3F: PID con Especificaciones\n\n');
ts_objetivo = 0.85*ts_lazo_abierto_2; zeta_objetivo = 0.5;
Kp_spec = 150; Ki_spec = 300; Kd_spec = 18; Ki2 = 80; Ki3 = 20;
fprintf('ts objetivo=%.4f s, zeta=%.2f\nKp=%.2f, Ki=%.2f, Kd=%.2f\n', ts_objetivo, zeta_objetivo, Kp_spec, Ki_spec, Kd_spec);
C_spec_num = conv(conv([Kd_spec Kp_spec Ki_spec Ki2 Ki3], [1 0]), [1 0 0]);
C_spec_den = [1 0 0 0 0 0];
C_spec = tf(C_spec_num, C_spec_den);
sys_cl_spec = feedback(C_spec*G, 1);
polos_cl = pole(sys_cl_spec);
figure; subplot(2,2,1); step(sys_cl_spec); title('PID Espec - Escalón'); grid on;
subplot(2,2,2); t=0:0.01:10; lsim(sys_cl_spec, t.^2, t); title('Respuesta Parábola'); grid on;
subplot(2,2,[3,4]); rlocus(C_spec*G); grid on;
save('ejercicio_tres/pid_especificaciones.mat', 'C_spec', 'sys_cl_spec', 'Kp_spec', 'Ki_spec', 'Kd_spec', 'Ki2', 'Ki3', 'polos_cl');
saveas(gcf, 'ejercicio_tres/pid_especificaciones.png');
fprintf('Resultados guardados\n');
