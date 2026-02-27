%% EJERCICIO 4E: PID ZIEGLER-NICHOLS - SISTEMA DE PRESIÓN
clear all; close all; clc;
load('ejercicio_cuatro/datos_sistema.mat');
fprintf('EJERCICIO 4E: PID Ziegler-Nichols\n\n');
Ku = 45; Tu = 30;
Kp_zn = 0.6*Ku; Ki_zn = Kp_zn/(Tu/2); Kd_zn = Kp_zn*(Tu/8);
fprintf('Ku=%.2f, Tu=%.2f s\nKp=%.2f, Ki=%.2f, Kd=%.2f\n', Ku, Tu, Kp_zn, Ki_zn, Kd_zn);
C_zn = pid(Kp_zn, Ki_zn, Kd_zn);
sys_cl_zn = feedback(C_zn*G, 1);
polos_cl = pole(sys_cl_zn);
figure; subplot(2,2,1); step(sys_cl_zn, 100); title('PID ZN - Escalón'); grid on;
subplot(2,2,2); rlocus(C_zn*G); title('Lugar Raíces'); grid on;
subplot(2,2,[3,4]); margin(C_zn*G); grid on;
save('ejercicio_cuatro/pid_ziegler_nichols.mat', 'C_zn', 'sys_cl_zn', 'Kp_zn', 'Ki_zn', 'Kd_zn', 'Ku', 'Tu', 'polos_cl');
saveas(gcf, 'ejercicio_cuatro/pid_ziegler_nichols.png');
fprintf('Resultados guardados\n');
