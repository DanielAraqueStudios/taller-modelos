%% EJERCICIO 4D: TIEMPO DE ESTABLECIMIENTO EN LAZO ABIERTO
% Simulación de la respuesta al escalón del sistema de presión
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS DEL SISTEMA
load('ejercicio_cuatro/datos_sistema.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 4D: TIEMPO DE ESTABLECIMIENTO\n');
fprintf('  Sistema de Control de Presión\n');
fprintf('========================================\n\n');

%% RESPUESTA AL ESCALÓN EN LAZO ABIERTO
figure('Name', 'Respuesta al Escalón - Sistema de Presión', 'Position', [100 100 1000 600]);

% Simular respuesta al escalón
[y, t] = step(G, 50);  % 50 segundos para sistemas lentos

% Graficar respuesta
subplot(2,1,1);
plot(t, y, 'b-', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('Presión p_2(t) [Pa]');
title('Respuesta al Escalón Unitario - Lazo Abierto');

% Calcular valor final estacionario
yss = dcgain(G);
fprintf('Valor en estado estacionario: %.6f Pa\n', yss);

% Calcular tiempo de establecimiento (criterio 2%)
tolerance_2 = 0.02;
band_upper_2 = yss * (1 + tolerance_2);
band_lower_2 = yss * (1 - tolerance_2);

% Encontrar tiempo de establecimiento al 2%
idx = find(y > band_upper_2 | y < band_lower_2, 1, 'last');
if ~isempty(idx) && idx < length(t)
    ts_2_simulado = t(idx);
else
    ts_2_simulado = t(end);
end

fprintf('Tiempo de establecimiento (2%%): %.4f s\n', ts_2_simulado);

% Calcular tiempo de establecimiento (criterio 5%)
tolerance_5 = 0.05;
band_upper_5 = yss * (1 + tolerance_5);
band_lower_5 = yss * (1 - tolerance_5);

idx = find(y > band_upper_5 | y < band_lower_5, 1, 'last');
if ~isempty(idx) && idx < length(t)
    ts_5_simulado = t(idx);
else
    ts_5_simulado = t(end);
end

fprintf('Tiempo de establecimiento (5%%): %.4f s\n', ts_5_simulado);

% Marcar bandas de tolerancia
hold on;
plot([0 t(end)], [band_upper_2 band_upper_2], 'r--', 'LineWidth', 1);
plot([0 t(end)], [band_lower_2 band_lower_2], 'r--', 'LineWidth', 1);
plot([0 t(end)], [band_upper_5 band_upper_5], 'g--', 'LineWidth', 1);
plot([0 t(end)], [band_lower_5 band_lower_5], 'g--', 'LineWidth', 1);
plot([0 t(end)], [yss yss], 'k--', 'LineWidth', 1.5);

% Marcar tiempos de establecimiento
if ts_2_simulado < t(end)
    plot([ts_2_simulado ts_2_simulado], [0 yss*1.1], 'r:', 'LineWidth', 1.5);
end
if ts_5_simulado < t(end)
    plot([ts_5_simulado ts_5_simulado], [0 yss*1.1], 'g:', 'LineWidth', 1.5);
end

legend('Respuesta', 'Banda ±2%', '', 'Banda ±5%', '', ...
       sprintf('y_{ss} = %.4f', yss), ...
       sprintf('t_s(2%%) = %.2f s', ts_2_simulado), ...
       sprintf('t_s(5%%) = %.2f s', ts_5_simulado), ...
       'Location', 'best');

%% ANÁLISIS DE SOBREPICO
[y_max, idx_max] = max(y);
t_max = t(idx_max);
Mp_percent = ((y_max - yss)/yss) * 100;

fprintf('\nSobrepico máximo: %.2f%% (a t = %.4f s)\n', Mp_percent, t_max);

%% INFO DEL SISTEMA
info_step = stepinfo(G);

subplot(2,1,2);
axis off;
text(0.1, 0.9, '\bf Información de la Respuesta al Escalón:', 'FontSize', 12);

info_text = {
    sprintf('Tiempo de subida (10%%-90%%): %.4f s', info_step.RiseTime);
    sprintf('Tiempo de establecimiento (2%%): %.4f s', info_step.SettlingTime);
    sprintf('Sobrepico: %.2f %%', info_step.Overshoot);
    sprintf('Tiempo de pico: %.4f s', info_step.PeakTime);
    sprintf('Valor pico: %.6f Pa', info_step.Peak);
    sprintf('Ganancia DC: %.6f', dcgain(G));
    '';
    sprintf('Ts(2%%) simulado: %.4f s', ts_2_simulado);
    sprintf('Ts(5%%) simulado: %.4f s', ts_5_simulado);
};

text(0.1, 0.75, info_text, 'FontSize', 10, 'VerticalAlignment', 'top');

%% GUARDAR RESULTADOS
ts_lazo_abierto_2 = ts_2_simulado;
ts_lazo_abierto_5 = ts_5_simulado;

save('ejercicio_cuatro/resultados_lazo_abierto.mat', 'ts_lazo_abierto_2', 'ts_lazo_abierto_5', ...
     'yss', 'Mp_percent', 'info_step', 't', 'y');

% Guardar figura
saveas(gcf, 'ejercicio_cuatro/respuesta_lazo_abierto.fig');
saveas(gcf, 'ejercicio_cuatro/respuesta_lazo_abierto.png');

fprintf('\n✓ Resultados guardados en "ejercicio_cuatro/resultados_lazo_abierto.mat"\n');
fprintf('\n=== EJERCICIO 4D COMPLETADO ===\n');
