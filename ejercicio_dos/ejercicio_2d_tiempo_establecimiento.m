%% EJERCICIO 2D: TIEMPO DE ESTABLECIMIENTO - SISTEMA PÉNDULO
% Simulación de la respuesta al escalón
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_dos/datos_sistema.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 2D: TIEMPO DE ESTABLECIMIENTO\n');
fprintf('  Sistema Péndulo con Resortes\n');
fprintf('========================================\n\n');

%% RESPUESTA AL ESCALÓN
figure('Name', 'Respuesta al Escalón - Péndulo', 'Position', [100 100 1000 600]);

[y, t] = step(G, 10);  

subplot(2,1,1);
plot(t, y, 'b-', 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('Salida y(t) = L\theta [m]');
title('Respuesta al Escalón Unitario - Lazo Abierto');

%% ANÁLISIS
yss = dcgain(G);
fprintf('Ganancia DC: %.6f\n', yss);

% Tiempo de establecimiento (2%)
tolerance_2 = 0.02;
band_upper_2 = yss * (1 + tolerance_2);
band_lower_2 = yss * (1 - tolerance_2);

idx = find(y > band_upper_2 | y < band_lower_2, 1, 'last');
if ~isempty(idx) && idx < length(t)
    ts_2_simulado = t(idx);
else
    ts_2_simulado = inf;  % Sistema oscilatorio sin establecimiento
end

fprintf('Tiempo de establecimiento (2%%): ');
if isinf(ts_2_simulado)
    fprintf('∞ (sistema marginalmente estable)\n');
else
    fprintf('%.4f s\n', ts_2_simulado);
end

% Tiempo de establecimiento (5%)
tolerance_5 = 0.05;
band_upper_5 = yss * (1 + tolerance_5);
band_lower_5 = yss * (1 - tolerance_5);

idx = find(y > band_upper_5 | y < band_lower_5, 1, 'last');
if ~isempty(idx) && idx < length(t)
    ts_5_simulado = t(idx);
else
    ts_5_simulado = inf;
end

fprintf('Tiempo de establecimiento (5%%): ');
if isinf(ts_5_simulado)
    fprintf('∞ (sistema marginalmente estable)\n');
else
    fprintf('%.4f s\n', ts_5_simulado);
end

%% SOBREPICO
[y_max, idx_max] = max(y);
t_max = t(idx_max);
if abs(yss) > 1e-6
    Mp_percent = ((y_max - yss)/abs(yss)) * 100;
else
    Mp_percent = 0;
end

fprintf('\nSobrepico máximo: %.2f%% (a t = %.4f s)\n', Mp_percent, t_max);

% Marcar en gráfica
hold on;
if ~isinf(ts_2_simulado)
    plot([0 t(end)], [band_upper_2 band_upper_2], 'r--', 'LineWidth', 1);
    plot([0 t(end)], [band_lower_2 band_lower_2], 'r--', 'LineWidth', 1);
end
if ~isinf(ts_5_simulado)
    plot([0 t(end)], [band_upper_5 band_upper_5], 'g--', 'LineWidth', 1);
    plot([0 t(end)], [band_lower_5 band_lower_5], 'g--', 'LineWidth', 1);
end
plot([0 t(end)], [yss yss], 'k--', 'LineWidth', 1.5);

legend('Respuesta', 'Location', 'best');

%% INFO
info_step = stepinfo(G, 'SettlingTimeThreshold', 0.02);

subplot(2,1,2);
axis off;
text(0.1, 0.9, '\bf Información del Sistema:', 'FontSize', 12);

info_text = {
    sprintf('Sistema marginalmente estable (polos imaginarios puros)');
    sprintf('Frecuencia natural: %.4f rad/s', wn);
    sprintf('Ganancia DC: %.6f', yss);
    '';
    'Nota: Sistema oscilatorio sin amortiguamiento';
    'Se requiere control para estabilizar';
};

text(0.1, 0.75, info_text, 'FontSize', 10, 'VerticalAlignment', 'top');

%% GUARDAR
ts_lazo_abierto_2 = ts_2_simulado;
ts_lazo_abierto_5 = ts_5_simulado;

save('ejercicio_dos/resultados_lazo_abierto.mat', 'ts_lazo_abierto_2', 'ts_lazo_abierto_5', ...
     'yss', 'Mp_percent', 't', 'y', 'wn');

saveas(gcf, 'ejercicio_dos/respuesta_lazo_abierto.fig');
saveas(gcf, 'ejercicio_dos/respuesta_lazo_abierto.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 2D COMPLETADO ===\n');
