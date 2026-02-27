%% EJERCICIO 5D: TIEMPO DE ESTABLECIMIENTO - CIRCUITO RL
% Simulación de la respuesta al escalón
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_cinco/datos_sistema.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 5D: TIEMPO DE ESTABLECIMIENTO\n');
fprintf('  Circuito RL\n');
fprintf('========================================\n\n');

%% RESPUESTA AL ESCALÓN
figure('Name', 'Respuesta al Escalón - Circuito RL', 'Position', [100 100 1000 600]);

[y, t]= step(G_mA, 0.015);  % 15 ms de simulación

subplot(2,1,1);
plot(t*1000, y, 'b-', 'LineWidth', 2);
grid on;
xlabel('Tiempo [ms]');
ylabel('Corriente [mA/V]');
title('Respuesta al Escalón Unitario - Lazo Abierto');

%% ANÁLISIS
yss = dcgain(G_mA);
fprintf('Ganancia DC: %.6f mA/V\n', yss);

% Tiempos teóricos para sistema de primer orden
ts_2_teorico = 4 * tau;
ts_5_teorico = 3 * tau;

fprintf('\n=== TIEMPOS TEÓRICOS (PRIMER ORDEN) ===\n');
fprintf('ts(2%%) teórico = 4τ = %.4f s = %.2f ms\n', ts_2_teorico, ts_2_teorico*1000);
fprintf('ts(5%%) teórico = 3τ = %.4f s = %.2f ms\n', ts_5_teorico, ts_5_teorico*1000);

% Simular ts
tolerance_2 = 0.02;
band_upper_2 = yss * (1 + tolerance_2);
band_lower_2 = yss * (1 - tolerance_2);

idx = find(y > band_upper_2 | y < band_lower_2, 1, 'last');
if ~isempty(idx) && idx < length(t)
    ts_2_simulado = t(idx);
else
    ts_2_simulado = t(end);
end

tolerance_5 = 0.05;
band_upper_5 = yss * (1 + tolerance_5);
band_lower_5 = yss * (1 - tolerance_5);

idx = find(y > band_upper_5 | y < band_lower_5, 1, 'last');
if ~isempty(idx) && idx < length(t)
    ts_5_simulado = t(idx);
else
    ts_5_simulado = t(end);
end

fprintf('\n=== TIEMPOS SIMULADOS ===\n');
fprintf('ts(2%%) simulado = %.4f s = %.2f ms\n', ts_2_simulado, ts_2_simulado*1000);
fprintf('ts(5%%) simulado = %.4f s = %.2f ms\n', ts_5_simulado, ts_5_simulado*1000);

% Marcar bandas
hold on;
plot([0 t(end)]*1000, [band_upper_2 band_upper_2], 'r--', 'LineWidth', 1);
plot([0 t(end)]*1000, [band_lower_2 band_lower_2], 'r--', 'LineWidth', 1);
plot([0 t(end)]*1000, [band_upper_5 band_upper_5], 'g--', 'LineWidth', 1);
plot([0 t(end)]*1000, [band_lower_5 band_lower_5], 'g--', 'LineWidth', 1);
plot([0 t(end)]*1000, [yss yss], 'k--', 'LineWidth', 1.5);

if ts_2_simulado < t(end)
    plot([ts_2_simulado ts_2_simulado]*1000, [0 yss*1.1], 'r:', 'LineWidth', 1.5);
end

legend('Respuesta', 'Banda ±2%', '', 'Banda ±5%', '', ...
       sprintf('y_{ss} = %.4f', yss), 'Location', 'best');

%% INFO
info_step = stepinfo(G_mA);

subplot(2,1,2);
axis off;
text(0.1, 0.9, '\bf Información del Sistema de Primer Orden:', 'FontSize', 12);

info_text = {
    sprintf('Constante de tiempo: τ = %.4f s = %.2f ms', tau, tau*1000);
    sprintf('Tiempo de subida (10%%-90%%): %.4f s = %.2f ms', info_step.RiseTime, info_step.RiseTime*1000);
    sprintf('Tiempo de establecimiento (2%%): %.4f s = %.2f ms', info_step.SettlingTime, info_step.SettlingTime*1000);
    sprintf('Ganancia DC: %.6f mA/V', yss);
    '';
    'Sistema de PRIMER ORDEN - Sin sobrepico';
    'Respuesta exponencial pura';
};

text(0.1, 0.75, info_text, 'FontSize', 10, 'VerticalAlignment', 'top');

%% GUARDAR
ts_lazo_abierto_2 = ts_2_simulado;
ts_lazo_abierto_5 = ts_5_simulado;

save('ejercicio_cinco/resultados_lazo_abierto.mat', 'ts_lazo_abierto_2', 'ts_lazo_abierto_5', ...
     'yss', 'info_step', 't', 'y', 'ts_2_teorico', 'ts_5_teorico');

saveas(gcf, 'ejercicio_cinco/respuesta_lazo_abierto.fig');
saveas(gcf, 'ejercicio_cinco/respuesta_lazo_abierto.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 5D COMPLETADO ===\n');
