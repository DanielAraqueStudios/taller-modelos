%% EJERCICIO 5H: SIMULACIONES COMPLETAS - CIRCUITO RL
% Comparación de todos los controladores diseñados
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR TODOS LOS DATOS
load('ejercicio_cinco/datos_sistema.mat');
load('ejercicio_cinco/pid_ziegler_nichols.mat');
load('ejercicio_cinco/pid_especificaciones.mat');
load('ejercicio_cinco/sistema_retardo.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 5H: SIMULACIONES COMPLETAS\n');
fprintf('  Circuito RL\n');
fprintf('========================================\n\n');

%% SISTEMAS EN LAZO CERRADO
sys_cl_zn = feedback(C_zn * G, 1);
sys_cl_spec = feedback(C_spec * G, 1);
sys_cl_delay_cons = feedback(C_delay1 * G_delay, 1);
sys_cl_delay_imc = feedback(C_imc * G_delay, 1);

%% FIGURA 1: COMPARACIÓN DE RESPUESTAS
figure('Name', 'Comparación Completa - Circuito RL', 'Position', [50 50 1400 900]);

% Escalón - sin retardo
subplot(2,2,1);
step(G * 1000, 0.02);
hold on;
step(sys_cl_zn * 1000, 0.02);
step(sys_cl_spec * 1000, 0.02);
grid on;
title('Respuesta al Escalón - Controladores sin Retardo');
legend('Lazo Abierto', 'PID ZN', 'PID Especificaciones', 'Location', 'best');
ylabel('Corriente [mA]');
xlabel('Tiempo [s]');

% Escalón - con retardo
subplot(2,2,2);
step(sys_cl_zn * 1000, 1);
hold on;
step(sys_cl_delay_cons * 1000, 1);
step(sys_cl_delay_imc * 1000, 1);
grid on;
title('Respuesta al Escalón - Con Retardo τ=0.7s');
legend('ZN sin retardo', 'Conservador', 'IMC', 'Location', 'best');
ylabel('Corriente [mA]');
xlabel('Tiempo [s]');

% Respuesta a rampa
subplot(2,2,3);
t = 0:0.0001:0.05;
u_ramp = t;

[y_zn, ~] = lsim(sys_cl_zn * 1000, u_ramp, t);
[y_spec, ~] = lsim(sys_cl_spec * 1000, u_ramp, t);

plot(t, u_ramp, 'k--', 'LineWidth', 2);
hold on;
plot(t, y_zn, 'b-', 'LineWidth', 1.5);
plot(t, y_spec, 'r-', 'LineWidth', 1.5);
grid on;
title('Respuesta a Entrada Rampa');
legend('Referencia', 'PID ZN', 'PID Especificaciones', 'Location', 'best');
ylabel('Corriente [mA]');
xlabel('Tiempo [s]');

% Respuesta senoidal
subplot(2,2,4);
t_sin = 0:0.0001:0.1;
freq = 50;  % 50 Hz
u_sin = sin(2*pi*freq*t_sin);

[y_zn_sin, ~] = lsim(sys_cl_zn * 1000, u_sin, t_sin);
[y_spec_sin, ~] = lsim(sys_cl_spec * 1000, u_sin, t_sin);

plot(t_sin, u_sin, 'k--', 'LineWidth', 1.5);
hold on;
plot(t_sin, y_zn_sin, 'b-', 'LineWidth', 1);
plot(t_sin, y_spec_sin, 'r-', 'LineWidth', 1);
grid on;
title(sprintf('Respuesta a Senoide (%d Hz)', freq));
legend('Referencia', 'PID ZN', 'PID Especificaciones', 'Location', 'best');
ylabel('Corriente [mA]');
xlabel('Tiempo [s]');

%% FIGURA 2: ANÁLISIS DE FRECUENCIA
figure('Name', 'Análisis de Frecuencia - Circuito RL', 'Position', [100 100 1400 800]);

subplot(2,3,1);
bode(C_zn * G);
grid on;
title('Bode - PID Ziegler-Nichols');

subplot(2,3,2);
bode(C_spec * G);
grid on;
title('Bode - PID Especificaciones');

subplot(2,3,3);
bode(C_delay1 * G_delay);
grid on;
title('Bode - PID con Retardo (Conservador)');

subplot(2,3,4);
rlocus(C_zn * G);
grid on;
title('Lugar Raíces - PID ZN');

subplot(2,3,5);
rlocus(C_spec * G);
grid on;
title('Lugar Raíces - PID Espec');

subplot(2,3,6);
rlocus(C_imc * G_delay);
grid on;
title('Lugar Raíces - IMC con Retardo');

%% TABLA DE MÉTRICAS
fprintf('\n=== MÉTRICAS DE DESEMPEÑO ===\n\n');

info_zn = stepinfo(sys_cl_zn);
info_spec = stepinfo(sys_cl_spec);

fprintf('%-25s | %-15s | %-15s\n', 'Métrica', 'PID ZN', 'PID Espec');
fprintf('-------------------------------------------------------------------------------\n');
fprintf('%-25s | %15.6f | %15.6f\n', 'Tiempo de subida (s)', info_zn.RiseTime, info_spec.RiseTime);
fprintf('%-25s | %15.6f | %15.6f\n', 'Tiempo establecim. (s)', info_zn.SettlingTime, info_spec.SettlingTime);
fprintf('%-25s | %15.2f | %15.2f\n', 'Sobrepico (%)', info_zn.Overshoot, info_spec.Overshoot);
fprintf('%-25s | %15.6f | %15.6f\n', 'Pico', info_zn.Peak, info_spec.Peak);

%% ANÁLISIS DE ROBUSTEZ
fprintf('\n=== ANÁLISIS DE ROBUSTEZ (MÁRGENES) ===\n\n');

[Gm_zn, Pm_zn, ~, ~] = margin(C_zn * G);
[Gm_spec, Pm_spec, ~, ~] = margin(C_spec * G);
[Gm_delay, Pm_delay, ~, ~] = margin(C_delay1 * G_delay);

fprintf('%-25s | %-15s | %-15s | %-15s\n', 'Controlador', 'Margen Gan (dB)', 'Margen Fase (°)', 'Estabilidad');
fprintf('-----------------------------------------------------------------------------------\n');
fprintf('%-25s | %15.2f | %15.2f | %-15s\n', 'PID ZN', 20*log10(Gm_zn), Pm_zn, ...
    char((Gm_zn>1 && Pm_zn>0)*'Estable' + ~(Gm_zn>1 && Pm_zn>0)*'Límite'));
fprintf('%-25s |  %15.2f | %15.2f | %-15s\n', 'PID Espec', 20*log10(Gm_spec), Pm_spec, ...
    char((Gm_spec>1 && Pm_spec>0)*'Estable' + ~(Gm_spec>1 && Pm_spec>0)*'Límite'));
fprintf('%-25s | %15.2f | %15.2f | %-15s\n', 'PID Retardo', 20*log10(Gm_delay), Pm_delay, ...
    char((Gm_delay>1 && Pm_delay>0)*'Estable' + ~(Gm_delay>1 && Pm_delay>0)*'Límite'));

%% RESUMEN DE CONTROLADORES
fprintf('\n=== RESUMEN DE TODOS LOS CONTROLADORES ===\n\n');
fprintf('%-25s | %10s | %12s | %10s\n', 'Controlador', 'Kp', 'Ki', 'Kd');
fprintf('-----------------------------------------------------------------------\n');
fprintf('%-25s | %10.2f | %12.2f | %10.2f\n', 'PID ZN', Kp_zn, Ki_zn, Kd_zn);
fprintf('%-25s | %10.2f | %12.2f | %10.2f\n', 'PID Especificaciones', Kp_spec, Ki_spec, Kd_spec);
fprintf('%-25s | %10.2f | %12.2f | %10.2f\n', 'PID Retardo (Conserv)', Kp_delay1, Ki_delay1, Kd_delay1);
fprintf('%-25s | %10.2f | %12.2f | %10.2f\n', 'PI IMC', Kp_imc, Ki_imc, Kd_imc);

%% GUARDAR
saveas(figure(1), 'ejercicio_cinco/simulaciones_completas_1.fig');
saveas(figure(1), 'ejercicio_cinco/simulaciones_completas_1.png');
saveas(figure(2), 'ejercicio_cinco/simulaciones_completas_2.fig');
saveas(figure(2), 'ejercicio_cinco/simulaciones_completas_2.png');

metricas.info_zn = info_zn;
metricas.info_spec = info_spec;
metricas.margenes_ganancia = [Gm_zn, Gm_spec, Gm_delay];
metricas.margenes_fase = [Pm_zn, Pm_spec, Pm_delay];

save('ejercicio_cinco/metricas_desempeno.mat', 'metricas');

fprintf('\n✓ Simulaciones y análisis completados\n');
fprintf('✓ Figuras y datos guardados en "ejercicio_cinco/"\n');
fprintf('\n=== EJERCICIO 5H COMPLETADO ===\n');
fprintf('\n=== TODOS LOS EJERCICIOS DEL CIRCUITO RL FINALIZADOS ===\n');
