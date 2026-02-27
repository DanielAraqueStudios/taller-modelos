%% EJERCICIO 2H: SIMULACIONES COMPLETAS - SISTEMA PÉNDULO
% Comparación de todos los controladores diseñados
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR TODOS LOS DATOS
load('ejercicio_dos/datos_sistema.mat');
load('ejercicio_dos/pid_ziegler_nichols.mat');
load('ejercicio_dos/pid_especificaciones.mat');
load('ejercicio_dos/sistema_retardo.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 2H: SIMULACIONES COMPLETAS\n');
fprintf('  Sistema Péndulo con Resortes\n');
fprintf('========================================\n\n');

%% SISTEMAS EN LAZO CERRADO
sys_cl_zn = feedback(C_zn * G, 1);
sys_cl_spec = feedback(C_spec * G, 1);
sys_cl_delay_cons = feedback(C_delay1 * G_delay, 1);
sys_cl_delay_chr = feedback(C_chr * G_delay, 1);

%% FIGURA 1: COMPARACIÓN DE RESPUESTAS AL ESCALÓN
figure('Name', 'Comparación Completa - Péndulo', 'Position', [50 50 1400 900]);

subplot(2,2,1);
step(G, 5);
hold on;
step(sys_cl_zn, 5);
step(sys_cl_spec, 5);
grid on;
title('Respuesta al Escalón - Controladores sin Retardo');
legend('Lazo Abierto', 'PID ZN', 'PI²D Especificaciones', 'Location', 'best');
ylabel('Salida y(t)');
xlabel('Tiempo [s]');

subplot(2,2,2);
step(sys_cl_zn, 5);
hold on;
step(sys_cl_delay_cons, 5);
step(sys_cl_delay_chr, 5);
grid on;
title('Respuesta al Escalón - Con Retardo τ=0.1s');
legend('ZN sin retardo', 'Conservador', 'CHR', 'Location', 'best');
ylabel('Salida y(t)');
xlabel('Tiempo [s]');

%% RESPUESTA A RAMPA
subplot(2,2,3);
t = 0:0.01:10;
u_ramp = t;

[y_zn, ~] = lsim(sys_cl_zn, u_ramp, t);
[y_spec, ~] = lsim(sys_cl_spec, u_ramp, t);

plot(t, t, 'k--', 'LineWidth', 2);
hold on;
plot(t, y_zn, 'b-', 'LineWidth', 1.5);
plot(t, y_spec, 'r-', 'LineWidth', 1.5);
grid on;
title('Respuesta a Entrada Rampa');
legend('Referencia', 'PID ZN', 'PI²D Especificaciones', 'Location', 'best');
ylabel('Salida y(t)');
xlabel('Tiempo [s]');

%% RESPUESTA A ENTRADA SENOIDAL
subplot(2,2,4);
t_sin = 0:0.01:20;
freq = 0.5;  % Hz
u_sin = sin(2*pi*freq*t_sin);

[y_zn_sin, ~] = lsim(sys_cl_zn, u_sin, t_sin);
[y_spec_sin, ~] = lsim(sys_cl_spec, u_sin, t_sin);

plot(t_sin, u_sin, 'k--', 'LineWidth', 1.5);
hold on;
plot(t_sin, y_zn_sin, 'b-', 'LineWidth', 1);
plot(t_sin, y_spec_sin, 'r-', 'LineWidth', 1);
grid on;
title(sprintf('Respuesta a Senoide (%.1f Hz)', freq));
legend('Referencia', 'PID ZN', 'PI²D Especificaciones', 'Location', 'best');
ylabel('Salida y(t)');
xlabel('Tiempo [s]');

%% FIGURA 2: DIAGRAMAS DE BODE Y LUGARES DE RAÍCES
figure('Name', 'Análisis de Frecuencia y Raíces - Péndulo', 'Position', [100 100 1400 800]);

subplot(2,3,1);
bode(C_zn * G);
grid on;
title('Bode - PID Ziegler-Nichols');

subplot(2,3,2);
bode(C_spec * G);
grid on;
title('Bode - PI²D Especificaciones');

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
title('Lugar Raíces - PI²D Espec');

subplot(2,3,6);
rlocus(C_chr * G_delay);
grid on;
title('Lugar Raíces - CHR con Retardo');

%% TABLA DE MÉTRICAS DE DESEMPEÑO
fprintf('\n=== MÉTRICAS DE DESEMPEÑO ===\n\n');

% Calcular métricas para cada controlador
info_zn = stepinfo(sys_cl_zn);
info_spec = stepinfo(sys_cl_spec);

fprintf('%-25s | %-15s | %-15s\n', 'Métrica', 'PID ZN', 'PI²D Espec');
fprintf('-------------------------------------------------------------------------------\n');
fprintf('%-25s | %15.4f | %15.4f\n', 'Tiempo de subida (s)', info_zn.RiseTime, info_spec.RiseTime);
fprintf('%-25s | %15.4f | %15.4f\n', 'Tiempo establecim. (s)', info_zn.SettlingTime, info_spec.SettlingTime);
fprintf('%-25s | %15.2f | %15.2f\n', 'Sobrepico (%)', info_zn.Overshoot, info_spec.Overshoot);
fprintf('%-25s | %15.4f | %15.4f\n', 'Pico', info_zn.Peak, info_spec.Peak);

%% ANÁLISIS DE ROBUSTEZ
fprintf('\n=== ANÁLISIS DE ROBUSTEZ (MÁRGENES) ===\n\n');

[Gm_zn, Pm_zn, ~, ~] = margin(C_zn * G);
[Gm_spec, Pm_spec, ~, ~] = margin(C_spec * G);
[Gm_delay, Pm_delay, ~, ~] = margin(C_delay1 * G_delay);

fprintf('%-25s | %-15s | %-15s | %-15s\n', 'Controlador', 'Margen Gan (dB)', 'Margen Fase (°)', 'Estabilidad');
fprintf('-----------------------------------------------------------------------------------\n');
fprintf('%-25s | %15.2f | %15.2f | %-15s\n', 'PID ZN', 20*log10(Gm_zn), Pm_zn, ...
    char((Gm_zn>1 && Pm_zn>0)*'Estable' + ~(Gm_zn>1 && Pm_zn>0)*'Límite'));
fprintf('%-25s | %15.2f | %15.2f | %-15s\n', 'PI²D Espec', 20*log10(Gm_spec), Pm_spec, ...
    char((Gm_spec>1 && Pm_spec>0)*'Estable' + ~(Gm_spec>1 && Pm_spec>0)*'Límite'));
fprintf('%-25s | %15.2f | %15.2f | %-15s\n', 'PID Retardo', 20*log10(Gm_delay), Pm_delay, ...
    char((Gm_delay>1 && Pm_delay>0)*'Estable' + ~(Gm_delay>1 && Pm_delay>0)*'Límite'));

%% GUARDAR RESULTADOS
saveas(figure(1), 'ejercicio_dos/simulaciones_completas_1.fig');
saveas(figure(1), 'ejercicio_dos/simulaciones_completas_1.png');
saveas(figure(2), 'ejercicio_dos/simulaciones_completas_2.fig');
saveas(figure(2), 'ejercicio_dos/simulaciones_completas_2.png');

% Guardar métricas
metricas.info_zn = info_zn;
metricas.info_spec = info_spec;
metricas.margenes_ganancia = [Gm_zn, Gm_spec, Gm_delay];
metricas.margenes_fase = [Pm_zn, Pm_spec, Pm_delay];

save('ejercicio_dos/metricas_desempeno.mat', 'metricas');

fprintf('\n✓ Simulaciones y análisis completados\n');
fprintf('✓ Figuras y datos guardados en "ejercicio_dos/"\n');
fprintf('\n=== EJERCICIO 2H COMPLETADO ===\n');
