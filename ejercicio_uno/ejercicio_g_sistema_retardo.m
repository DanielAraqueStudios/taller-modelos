%% EJERCICIO G: SISTEMA CON RETARDO
% Análisis y diseño de controlador para sistema con retardo de 1 segundo
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_uno/datos_sistema.mat');
load('ejercicio_uno/resultados_pid_especificaciones.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO G: SISTEMA CON RETARDO\n');
fprintf('========================================\n\n');

%% PARÁMETROS DEL RETARDO
tau = 1.0;  % Retardo de 1 segundo

fprintf('Retardo del sistema: %.2f s\n\n', tau);

%% SISTEMA CON RETARDO - APROXIMACIÓN DE PADÉ
% Aproximación de Padé de primer orden: e^(-tau*s) ≈ (1 - tau*s/2) / (1 + tau*s/2)
% Aproximación de Padé de segundo orden: e^(-tau*s) ≈ (1 - tau*s/2 + tau^2*s^2/12) / (1 + tau*s/2 + tau^2*s^2/12)

% Primer orden
num_pade1 = [1 - tau/2];
den_pade1 = [1 tau/2];
Pade1 = tf(num_pade1, den_pade1);

% Segundo orden
num_pade2 = [tau^2/12, -tau/2, 1];
den_pade2 = [tau^2/12, tau/2, 1];
Pade2 = tf(num_pade2, den_pade2);

fprintf('--- Aproximaciones de Padé ---\n');
fprintf('Primer orden:\n');
disp(Pade1);
fprintf('Segundo orden:\n');
disp(Pade2);

%% SISTEMA CON RETARDO
% G_delay(s) = G(s) * e^(-tau*s)
G_delay_pade1 = G * Pade1;
G_delay_pade2 = G * Pade2;

fprintf('\n--- Sistema con Retardo (Padé 2do orden) ---\n');
disp(G_delay_pade2);

%% ANÁLISIS DEL SISTEMA CON RETARDO SIN CONTROL
fprintf('\n--- Análisis del Sistema con Retardo en Lazo Abierto ---\n');

% Respuesta al escalón
figure('Name', 'Sistema con Retardo - Lazo Abierto', 'Position', [100 100 1200 400]);

subplot(1,2,1);
step(G, G_delay_pade2);
grid on;
title('Respuesta al Escalón - Comparación');
legend('Sin retardo', 'Con retardo (Padé 2)', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

subplot(1,2,2);
bode(G, G_delay_pade2);
grid on;
title('Diagrama de Bode - Comparación');
legend('Sin retardo', 'Con retardo', 'Location', 'best');

%% ESTRATEGIA 1: AJUSTE CONSERVADOR DE PARÁMETROS PID
fprintf('\n=== ESTRATEGIA 1: AJUSTE CONSERVADOR ===\n');
fprintf('Reducir Kp, aumentar Ti, reducir Td para mayor robustez\n\n');

% Factores de ajuste conservadores
factor_Kp = 0.7;    % Reducir Kp en 30%
factor_Ti = 1.5;    % Aumentar Ti (reducir Ki)
factor_Td = 0.75;   % Reducir Td

Kp_delay1 = Kp * factor_Kp;
Ki_delay1 = Ki / factor_Ti;
Kd_delay1 = Kd * factor_Td;

fprintf('Parámetros PID ajustados (conservador):\n');
fprintf('Kp = %.4f (original: %.4f, factor: %.2f)\n', Kp_delay1, Kp, factor_Kp);
fprintf('Ki = %.4f (original: %.4f, factor: %.2f)\n', Ki_delay1, Ki, 1/factor_Ti);
fprintf('Kd = %.4f (original: %.4f, factor: %.2f)\n', Kd_delay1, Kd, factor_Td);

C_delay1 = pid(Kp_delay1, Ki_delay1, Kd_delay1);
T_delay1 = feedback(C_delay1 * G_delay_pade2, 1);

% Verificar estabilidad
polos_delay1 = pole(T_delay1);
fprintf('\nPolos del sistema con controlador ajustado:\n');
for i = 1:length(polos_delay1)
    if abs(imag(polos_delay1(i))) < 1e-6
        fprintf('  p%d = %.4f\n', i, real(polos_delay1(i)));
    else
        fprintf('  p%d = %.4f %+.4fi\n', i, real(polos_delay1(i)), imag(polos_delay1(i)));
    end
end

if all(real(polos_delay1) < 0)
    fprintf('✓ Sistema ESTABLE con controlador ajustado\n');
else
    fprintf('✗ Sistema INESTABLE - requiere más ajuste\n');
end

%% ESTRATEGIA 2: PREDICTOR DE SMITH
fprintf('\n=== ESTRATEGIA 2: PREDICTOR DE SMITH ===\n');

% El predictor de Smith elimina el efecto del retardo del lazo de control
% Estructura: C(s) * [G(s) - G(s)*e^(-tau*s)] + C(s)*G(s)*e^(-tau*s)
% Simplificado: Usamos el controlador original

% En la práctica, el predictor de Smith requiere conocer el modelo exacto
% Para simulación, asumimos modelo perfecto

fprintf('Usando controlador PID original con Predictor de Smith\n');
fprintf('El predictor elimina el retardo del lazo de realimentación\n\n');

% Sistema equivalente con predictor de Smith
% T_smith(s) = C(s)*G(s)*e^(-tau*s) / [1 + C(s)*G(s)]
C_smith = C_spec;  % Controlador original

% El sistema en lazo cerrado con Smith es:
T_smith_num = C_smith * G;
T_smith_inner = feedback(C_smith * G, 1);
% Multiplicar por el retardo
T_smith = T_smith_inner * Pade2;

fprintf('Controlador (mismo que sin retardo):\n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp, Ki, Kd);

%% ESTRATEGIA 3: RESINTONIZACIÓN COMPLETA
fprintf('\n=== ESTRATEGIA 3: RESINTONIZACIÓN COMPLETA ===\n');
fprintf('Diseñar nuevo controlador considerando el retardo\n\n');

% Usar método de sintonización específico para sistemas con retardo
% Método de Chien, Hrones y Reswick (CHR) para sistemas con retardo

% Aproximar el sistema con retardo como First Order Plus Dead Time (FOPDT)
% G(s) ≈ K / (Ts + 1) * e^(-L*s)

% Obtener respuesta al escalón del sistema original
[y_step, t_step] = step(G);

% Método de dos puntos para identificar FOPDT
y_final = dcgain(G);
t_28 = interp1(y_step, t_step, 0.283*y_final);
t_63 = interp1(y_step, t_step, 0.632*y_final);

K_fopdt = y_final;
T_fopdt = 1.5 * (t_63 - t_28);
L_fopdt = t_63 - T_fopdt;

fprintf('Modelo FOPDT identificado:\n');
fprintf('K = %.4f, T = %.4f s, L = %.4f s\n', K_fopdt, T_fopdt, L_fopdt);

% Agregar retardo adicional
L_total = L_fopdt + tau;
fprintf('Retardo total (incluyendo tau = %.2f s): %.4f s\n', tau, L_total);

% Método CHR para rechazo de perturbaciones (0% overshoot)
Kp_chr = 0.6 * T_fopdt / (K_fopdt * L_total);
Ti_chr = T_fopdt;
Td_chr = 0.5 * L_total;

Ki_chr = Kp_chr / Ti_chr;
Kd_chr = Kp_chr * Td_chr;

fprintf('\nParámetros PID (método CHR para retardo):\n');
fprintf('Kp = %.4f\n', Kp_chr);
fprintf('Ki = %.4f\n', Ki_chr);
fprintf('Kd = %.4f\n', Kd_chr);

C_delay3 = pid(Kp_chr, Ki_chr, Kd_chr);
T_delay3 = feedback(C_delay3 * G_delay_pade2, 1);

% Verificar estabilidad
polos_delay3 = pole(T_delay3);
if all(real(polos_delay3) < 0)
    fprintf('✓ Sistema ESTABLE con controlador CHR\n');
else
    fprintf('✗ Sistema INESTABLE con controlador CHR\n');
end

%% COMPARACIÓN DE ESTRATEGIAS
figure('Name', 'Comparación de Estrategias con Retardo', 'Position', [100 100 1400 900]);

% Respuesta al escalón
subplot(2,2,1);
step(T_spec, T_delay1, T_smith, T_delay3);
grid on;
title('Respuesta al Escalón - Comparación de Controladores');
legend('Original (sin retardo)', 'Ajuste conservador', 'Predictor Smith', ...
       'Resintonización CHR', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

% Zoom en el inicio para ver el retardo
subplot(2,2,2);
step(T_spec, T_delay1, T_smith, T_delay3);
grid on;
xlim([0 3]);
title('Respuesta al Escalón - Zoom Inicial');
legend('Original', 'Conservador', 'Smith', 'CHR', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

% Esfuerzo de control
subplot(2,2,3);
t = 0:0.01:5;
u = ones(size(t));
[~, ~, u1] = lsim(feedback(C_delay1, G_delay_pade2), u, t);
[~, ~, u2] = lsim(feedback(C_smith, G_delay_pade2), u, t);
[~, ~, u3] = lsim(feedback(C_delay3, G_delay_pade2), u, t);
plot(t, u1, t, u2, t, u3, 'LineWidth', 1.5);
grid on;
title('Señal de Control - Comparación');
legend('Conservador', 'Smith', 'CHR', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('u(t) [N]');

% Diagrama de Bode
subplot(2,2,4);
bode(T_delay1, T_smith, T_delay3);
grid on;
title('Diagrama de Bode en Lazo Cerrado');
legend('Conservador', 'Smith', 'CHR', 'Location', 'best');

%% ANÁLISIS DE DESEMPEÑO
fprintf('\n--- Análisis de Desempeño ---\n\n');

info_delay1 = stepinfo(T_delay1);
info_smith = stepinfo(T_smith);
info_delay3 = stepinfo(T_delay3);

fprintf('Estrategia 1 - Ajuste Conservador:\n');
fprintf('  ts = %.4f s, Mp = %.2f %%, ess = %.6f\n', ...
        info_delay1.SettlingTime, info_delay1.Overshoot, 1-dcgain(T_delay1));

fprintf('\nEstrategia 2 - Predictor de Smith:\n');
fprintf('  ts = %.4f s, Mp = %.2f %%, ess = %.6f\n', ...
        info_smith.SettlingTime, info_smith.Overshoot, 1-dcgain(T_smith));

fprintf('\nEstrategia 3 - Resintonización CHR:\n');
fprintf('  ts = %.4f s, Mp = %.2f %%, ess = %.6f\n', ...
        info_delay3.SettlingTime, info_delay3.Overshoot, 1-dcgain(T_delay3));

%% RECOMENDACIÓN
fprintf('\n=== RECOMENDACIÓN ===\n');
fprintf('Para un retardo de %.2f s:\n', tau);
fprintf('1. Si el modelo es preciso: usar Predictor de Smith\n');
fprintf('2. Si hay incertidumbre: usar ajuste conservador\n');
fprintf('3. Para robustez máxima: usar resintonización CHR\n');

%% GUARDAR RESULTADOS
save('ejercicio_uno/resultados_sistema_retardo.mat', 'tau', 'G_delay_pade2', ...
     'C_delay1', 'C_smith', 'C_delay3', ...
     'T_delay1', 'T_smith', 'T_delay3', ...
     'Kp_delay1', 'Ki_delay1', 'Kd_delay1', ...
     'Kp_chr', 'Ki_chr', 'Kd_chr');

% Guardar figuras
saveas(1, 'ejercicio_uno/sistema_retardo_analisis.fig');
saveas(1, 'ejercicio_uno/sistema_retardo_analisis.png');
saveas(2, 'ejercicio_uno/comparacion_estrategias_retardo.fig');
saveas(2, 'ejercicio_uno/comparacion_estrategias_retardo.png');

fprintf('\n✓ Resultados guardados en "ejercicio_uno/resultados_sistema_retardo.mat"\n');
fprintf('\n=== EJERCICIO G COMPLETADO ===\n');
