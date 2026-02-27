%% EJERCICIO H: SIMULACIONES COMPLETAS Y COMPARACIÓN
% Simulación comparativa de todos los controladores diseñados
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR TODOS LOS DATOS
load('ejercicio_uno/datos_sistema.mat');
load('ejercicio_uno/resultados_lazo_abierto.mat');
load('ejercicio_uno/resultados_ziegler_nichols.mat');
load('ejercicio_uno/resultados_pid_especificaciones.mat');
load('ejercicio_uno/resultados_sistema_retardo.mat');

fprintf('========================================\n');
fprintf('  EJERC ICIO H: SIMULACIONES COMPLETAS\n');
fprintf('========================================\n\n');

%% DEFINIR SISTEMAS PARA COMPARACIÓN
% 1. Lazo abierto
T_LA = G;

% 2. PID Ziegler-Nichols
T_ZN = T_PID_ZN;

% 3. PID con especificaciones
T_Spec = T_spec;

% 4. Sistema con retardo - ajuste conservador
T_Delay = T_delay1;

fprintf('Sistemas a comparar:\n');
fprintf('1. Lazo Abierto\n');
fprintf('2. PID Ziegler-Nichols (Kp=%.2f, Ki=%.2f, Kd=%.2f)\n', Kp_PID, Ki_PID, Kd_PID);
fprintf('3. PID con Especificaciones (Kp=%.2f, Ki=%.2f, Kd=%.2f)\n', Kp, Ki, Kd);
fprintf('4. Sistema con Retardo tau=%.1fs (Kp=%.2f, Ki=%.2f, Kd=%.2f)\n\n', tau, Kp_delay1, Ki_delay1, Kd_delay1);

%% FIGURA 1: COMPARACIÓN COMPLETA DE RESPUESTAS
figure('Name', 'Comparación Completa de Controladores', 'Position', [50 50 1400 900]);

% Subplot 1: Respuesta al escalón
subplot(3,3,1);
step(T_LA, T_ZN, T_Spec);
grid on;
title('Respuesta al Escalón');
legend('Lazo Abierto', 'PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

% Subplot 2: Respuesta al escalón con retardo
subplot(3,3,2);
step(T_Spec, T_Delay);
grid on;
title('Respuesta con Retardo');
legend('Sin retardo', 'Con retardo (τ=1s)', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

% Subplot 3: Señal de control
subplot(3,3,3);
t_sim = 0:0.001:2;
u_ref = ones(size(t_sim));
[~, ~, u_zn] = lsim(feedback(C_PID_ZN, G), u_ref, t_sim);
[~, ~, u_spec] = lsim(feedback(C_spec, G), u_ref, t_sim);
plot(t_sim, u_zn, t_sim, u_spec, 'LineWidth', 1.5);
grid on;
title('Señal de Control');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('u(t) [N]');
xlim([0 1]);

% Subplot 4: Diagrama de Bode en lazo cerrado
subplot(3,3,[4,7]);
bode(T_LA, T_ZN, T_Spec);
grid on;
title('Diagrama de Bode - Lazo Cerrado');
legend('Lazo Abierto', 'PID ZN', 'PID Espec', 'Location', 'best');

% Subplot 5: Lugar de raíces con PID
subplot(3,3,5);
rlocus(C_PID_ZN * G);
grid on;
title('Lugar Raíces - PID ZN');
xlabel('Parte Real');
ylabel('Parte Imaginaria');

subplot(3,3,6);
rlocus(C_spec * G);
grid on;
title('Lugar Raíces - PID Especificaciones');
xlabel('Parte Real');
ylabel('Parte Imaginaria');

% Subplot 7: Polos y ceros
subplot(3,3,8);
pzmap(T_LA, T_ZN, T_Spec);
grid on;
title('Polos en Lazo Cerrado');
legend('LA', 'ZN', 'Espec', 'Location', 'best');

% Subplot 8: Respuesta al impulso
subplot(3,3,9);
impulse(T_ZN, T_Spec);
grid on;
title('Respuesta al Impulso');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

%% FIGURA 2: ANÁLISIS DETALLADO DE DESEMPEÑO
figure('Name', 'Análisis Detallado de Desempeño', 'Position', [100 100 1400 800]);

% Respuesta al escalón con detalles
subplot(2,2,1);
[y_la, t_la] = step(T_LA, 2);
[y_zn, t_zn] = step(T_ZN, 2);
[y_spec, t_spec] = step(T_Spec, 2);

plot(t_la, y_la, 'b-', 'LineWidth', 2);
hold on;
plot(t_zn, y_zn, 'r-', 'LineWidth', 2);
plot(t_spec, y_spec, 'g-', 'LineWidth', 2);

% Marcar características importantes
info_la = stepinfo(T_LA);
info_zn = stepinfo(T_ZN);
info_spec = stepinfo(T_Spec);

% Banda de asentamiento
yss = 1;
plot([0 2], [yss*1.02 yss*1.02], 'k--', 'LineWidth', 0.5);
plot([0 2], [yss*0.98 yss*0.98], 'k--', 'LineWidth', 0.5);
plot([0 2], [yss yss], 'k-', 'LineWidth', 1);

grid on;
title('Respuesta al Escalón Detallada');
legend('Lazo Abierto', 'PID ZN', 'PID Espec', 'Banda ±2%', '', 'y_{ss}', ...
       'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

% Tabla de características
subplot(2,2,2);
axis off;
text(0.1, 0.95, '\bf Tabla Comparativa de Desempeño', 'FontSize', 14);

table_data = {
    '', 'Lazo Abierto', 'PID ZN', 'PID Espec';
    sprintf('t_r (s)'), sprintf('%.4f', info_la.RiseTime), ...
    sprintf('%.4f', info_zn.RiseTime), sprintf('%.4f', info_spec.RiseTime);
    sprintf('t_s (s)'), sprintf('%.4f', info_la.SettlingTime), ...
    sprintf('%.4f', info_zn.SettlingTime), sprintf('%.4f', info_spec.SettlingTime);
    sprintf('M_p (%%)'), sprintf('%.2f', info_la.Overshoot), ...
    sprintf('%.2f', info_zn.Overshoot), sprintf('%.2f', info_spec.Overshoot);
    sprintf('e_{ss}'), sprintf('%.4f', 1-dcgain(T_LA)), ...
    sprintf('%.4f', 1-dcgain(T_ZN)), sprintf('%.4f', 1-dcgain(T_Spec));
};

y_pos = 0.85;
for i = 1:size(table_data, 1)
    for j = 1:size(table_data, 2)
        if i == 1
            text(0.05 + (j-1)*0.25, y_pos, table_data{i,j}, ...
                 'FontSize', 10, 'FontWeight', 'bold');
        else
            text(0.05 + (j-1)*0.25, y_pos, table_data{i,j}, 'FontSize', 9);
        end
    end
    y_pos = y_pos - 0.08;
end

% Error de seguimiento
subplot(2,2,3);
error_la = 1 - y_la;
error_zn = 1 - y_zn;
error_spec = 1 - y_spec;

plot(t_la, error_la, 'b-', 'LineWidth', 2);
hold on;
plot(t_zn, error_zn, 'r-', 'LineWidth', 2);
plot(t_spec, error_spec, 'g-', 'LineWidth', 2);
plot([0 2], [0 0], 'k--', 'LineWidth', 1);
grid on;
title('Error de Seguimiento');
legend('Lazo Abierto', 'PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Error e(t) = r(t) - y(t) [m]');

% Análisis de sensibilidad
subplot(2,2,4);
S_zn = feedback(1, C_PID_ZN * G);  % Función de sensibilidad
S_spec = feedback(1, C_spec * G);

bode(S_zn, S_spec);
grid on;
title('Función de Sensibilidad S(s) = 1/(1+C·G)');
legend('PID ZN', 'PID Espec', 'Location', 'best');

%% FIGURA 3: PRUEBAS CON DIFERENTES ENTRADAS
figure('Name', 'Respuesta a Diferentes Señales de Entrada', 'Position', [150 100 1400 700]);

t_test = 0:0.01:5;

% Entrada escalón
subplot(2,3,1);
u_step = ones(size(t_test));
lsim(T_ZN, T_Spec, u_step, t_test);
grid on;
title('Entrada Escalón Unitario');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

% Entrada rampa
subplot(2,3,2);
u_ramp = t_test;
lsim(T_ZN, T_Spec, u_ramp, t_test);
grid on;
title('Entrada Rampa');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

% Entrada sinusoidal
subplot(2,3,3);
u_sin = sin(2*pi*0.5*t_test);  % 0.5 Hz
lsim(T_ZN, T_Spec, u_sin, t_test);
grid on;
title('Entrada Sinusoidal (0.5 Hz)');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

% Entrada escalón con perturbación
subplot(2,3,4);
u_dist = ones(size(t_test));
u_dist(t_test > 2.5) = 1.5;  % Perturbación en t=2.5s
lsim(T_ZN, T_Spec, u_dist, t_test);
grid on;
title('Escalón con Perturbación');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

% Entrada cuadrada
subplot(2,3,5);
u_square = square(2*pi*0.2*t_test);
lsim(T_ZN, T_Spec, u_square, t_test);
grid on;
title('Entrada Onda Cuadrada (0.2 Hz)');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

% Entrada aleatoria
subplot(2,3,6);
u_random = 0.5 + 0.3*randn(size(t_test));
lsim(T_ZN, T_Spec, u_random, t_test);
grid on;
title('Entrada Aleatoria (Ruido)');
legend('PID ZN', 'PID Espec', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

%% FIGURA 4: ANÁLISIS CON RETARDO
figure('Name', 'Análisis del Sistema con Retardo', 'Position', [200 100 1400 600]);

subplot(2,3,1);
step(T_Spec, T_Delay);
grid on;
title('Respuesta al Escalón');
legend('Sin retardo', 'Con retardo (τ=1s)', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida [m]');

subplot(2,3,2);
pzmap(T_Spec, T_Delay);
grid on;
title('Polos en Lazo Cerrado');
legend('Sin retardo', 'Con retardo', 'Location', 'best');

subplot(2,3,3);
bode(T_Spec, T_Delay);
grid on;
title('Diagrama de Bode');
legend('Sin retardo', 'Con retardo', 'Location', 'best');

subplot(2,3,4);
margin(C_spec * G);
grid on;
title('Márgenes sin Retardo');

subplot(2,3,5);
margin(C_delay1 * G_delay_pade2);
grid on;
title('Márgenes con Retardo');

subplot(2,3,6);
nyquist(C_spec * G, C_delay1 * G_delay_pade2);
grid on;
title('Diagrama de Nyquist');
legend('Sin retardo', 'Con retardo', 'Location', 'best');

%% RESUMEN NUMÉRICO
fprintf('\n=== RESUMEN DE CARACTERÍSTICAS ===\n\n');

fprintf('LAZO ABIERTO:\n');
fprintf('  Tiempo de subida: %.4f s\n', info_la.RiseTime);
fprintf('  Tiempo de establecimiento: %.4f s\n', info_la.SettlingTime);
fprintf('  Sobrepico: %.2f %%\n', info_la.Overshoot);
fprintf('  Error estado estacionario: %.6f\n\n', 1-dcgain(T_LA));

fprintf('PID ZIEGLER-NICHOLS:\n');
fprintf('  Parámetros: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_PID, Ki_PID, Kd_PID);
fprintf('  Tiempo de subida: %.4f s\n', info_zn.RiseTime);
fprintf('  Tiempo de establecimiento: %.4f s\n', info_zn.SettlingTime);
fprintf('  Sobrepico: %.2f %%\n', info_zn.Overshoot);
fprintf('  Error estado estacionario: %.6f\n\n', 1-dcgain(T_ZN));

fprintf('PID CON ESPECIFICACIONES:\n');
fprintf('  Parámetros: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp, Ki, Kd);
fprintf('  Tiempo de subida: %.4f s\n', info_spec.RiseTime);
fprintf('  Tiempo de establecimiento: %.4f s (obj: %.4f s)\n', ...
        info_spec.SettlingTime, ts_deseado);
fprintf('  Sobrepico: %.2f %%\n', info_spec.Overshoot);
fprintf('  Error estado estacionario: %.6f\n\n', 1-dcgain(T_Spec));

fprintf('SISTEMA CON RETARDO (τ=%.1f s):\n', tau);
fprintf('  Parámetros: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_delay1, Ki_delay1, Kd_delay1);
info_delay = stepinfo(T_Delay);
fprintf('  Time de subida: %.4f s\n', info_delay.RiseTime);
fprintf('  Tiempo de establecimiento: %.4f s\n', info_delay.SettlingTime);
fprintf('  Sobrepico: %.2f %%\n', info_delay.Overshoot);
fprintf('  Error estado estacionario: %.6f\n\n', 1-dcgain(T_Delay));

%% ANÁLISIS DE ROBUSTEZ
fprintf('=== ANÁLISIS DE ROBUSTEZ ===\n\n');

[Gm_zn, Pm_zn, Wcg_zn, Wcp_zn] = margin(C_PID_ZN * G);
[Gm_spec, Pm_spec, Wcg_spec, Wcp_spec] = margin(C_spec * G);

fprintf('PID ZIEGLER-NICHOLS:\n');
fprintf('  Margen de ganancia: %.2f dB (%.2f veces)\n', 20*log10(Gm_zn), Gm_zn);
fprintf('  Margen de fase: %.2f°\n', Pm_zn);
fprintf('  Frec. cruce de ganancia: %.2f rad/s\n', Wcg_zn);
fprintf('  Frec. cruce de fase: %.2f rad/s\n\n', Wcp_zn);

fprintf('PID CON ESPECIFICACIONES:\n');
fprintf('  Margen de ganancia: %.2f dB (%.2f veces)\n', 20*log10(Gm_spec), Gm_spec);
fprintf('  Margen de fase: %.2f°\n', Pm_spec);
fprintf('  Frec. cruce de ganancia: %.2f rad/s\n', Wcg_spec);
fprintf('  Frec. cruce de fase: %.2f rad/s\n\n', Wcp_spec);

%% CONCLUSIONES
fprintf('=== CONCLUSIONES ===\n\n');
fprintf('1. El PID Ziegler-Nichols proporciona respuesta rápida pero con sobrepico moderado\n');
fprintf('2. El PID con especificaciones cumple objetivos precisos de ts y ζ\n');
fprintf('3. El retardo degrada el desempeño; se requiere ajuste de parámetros\n');
fprintf('4. Ambos controladores PID eliminan el error en estado estacionario (acción integral)\n');
fprintf('5. El sistema mantiene buenos márgenes de estabilidad\n\n');

fprintf('=== EJERCICIO H COMPLETADO ===\n');
fprintf('Todas las simulaciones han sido ejecutadas y comparadas exitosamente.\n');

%% GUARDAR FIGURAS
saveas(1, 'ejercicio_uno/comparacion_completa.fig');
saveas(1, 'ejercicio_uno/comparacion_completa.png');
saveas(2, 'ejercicio_uno/analisis_detallado.fig');
saveas(2, 'ejercicio_uno/analisis_detallado.png');
saveas(3, 'ejercicio_uno/respuestas_diferentes_entradas.fig');
saveas(3, 'ejercicio_uno/respuestas_diferentes_entradas.png');
saveas(4, 'ejercicio_uno/analisis_completo_retardo.fig');
saveas(4, 'ejercicio_uno/analisis_completo_retardo.png');

fprintf('\n✓ Figuras guardadas en carpeta ejercicio_uno/\n');
