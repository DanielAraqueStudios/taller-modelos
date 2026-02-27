%% EJERCICIO E: DISEÑO DE PID USANDO ZIEGLER-NICHOLS
% Método de sintonización de Ziegler-Nichols (método de ganancia última)
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS DEL SISTEMA
load('ejercicio_uno/datos_sistema.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO E: PID ZIEGLER-NICHOLS\n');
fprintf('========================================\n\n');

%% MÉTODO 1: ZIEGLER-NICHOLS - GANANCIA ÚLTIMA (Closed Loop)
fprintf('--- Método de Ganancia Última ---\n\n');

% Sistema en lazo cerrado con ganancia proporcional
% Encontrar ganancia crítica Ku (ganancia que hace al sistema marginalmente estable)

% Usar el lugar de las raíces para encontrar Ku
figure('Name', 'Lugar de Raíces para encontrar Ku', 'Position', [100 100 800 600]);
rlocus(G);
grid on;
title('Lugar de las Raíces - Determinación de Ganancia Crítica K_u');
xlabel('Parte Real');
ylabel('Parte Imaginaria');

% Encontrar ganancia crítica (donde los polos cruzan el eje imaginario)
[K_rlocus, poles_rlocus] = rlocus(G);

% Buscar ganancia donde los polos tienen parte real cercana a cero
tolerance = 1e-3;
idx_critical = [];
for i = 1:size(poles_rlocus, 2)
    poles_at_k = poles_rlocus(:, i);
    if any(abs(real(poles_at_k)) < tolerance & abs(imag(poles_at_k)) > 0)
        idx_critical = i;
        break;
    end
end

if ~isempty(idx_critical)
    Ku = K_rlocus(idx_critical);
    critical_poles = poles_rlocus(:, idx_critical);
    % Encontrar el polo complejo
    complex_pole = critical_poles(find(abs(imag(critical_poles)) > 0, 1));
    wu = abs(imag(complex_pole));
    Tu = 2*pi / wu;
    
    fprintf('Ganancia crítica (Ku): %.4f\n', Ku);
    fprintf('Frecuencia crítica (ωu): %.4f rad/s\n', wu);
    fprintf('Período crítico (Tu): %.4f s\n', Tu);
else
    % Método alternativo: barrido de ganancia
    fprintf('Buscando Ku mediante barrido de frecuencia...\n');
    
    K_test = logspace(-2, 4, 10000);
    margin_phase = zeros(size(K_test));
    
    for i = 1:length(K_test)
        sys_temp = feedback(K_test(i)*G, 1);
        p = pole(sys_temp);
        margin_phase(i) = max(real(p));
    end
    
    % Encontrar donde cambia de estabilidad
    idx_stable = find(margin_phase < 0, 1, 'last');
    if ~isempty(idx_stable)
        Ku = (K_test(idx_stable) + K_test(idx_stable+1))/2;
        
        % Calcular Tu con margen de fase
        [~, ~, ~, wcp] = margin(Ku*G);
        wu = wcp;
        Tu = 2*pi/wu;
        
        fprintf('Ganancia crítica (Ku): %.4f\n', Ku);
        fprintf('Frecuencia crítica (ωu): %.4f rad/s\n', wu);
        fprintf('Período crítico (Tu): %.4f s\n', Tu);
    else
        % Valores por defecto si no se encuentra
        fprintf('⚠ No se encontró Ku automáticamente. Usando análisis de Bode...\n');
        [Gm, Pm, Wcg, Wcp] = margin(G);
        Ku = Gm;
        wu = Wcg;
        Tu = 2*pi/wu;
        
        fprintf('Margen de ganancia: %.4f (%.2f dB)\n', Gm, 20*log10(Gm));
        fprintf('Ganancia crítica estimada (Ku): %.4f\n', Ku);
        fprintf('Frecuencia de cruce de ganancia (ωu): %.4f rad/s\n', wu);
        fprintf('Período crítico (Tu): %.4f s\n', Tu);
    end
end

%% TABLA DE ZIEGLER-NICHOLS (2da regla - Ganancia Última)
fprintf('\n--- Tabla de Ziegler-Nichols (Ganancia Última) ---\n');

% Controlador P
Kp_P = 0.5 * Ku;
Ki_P = 0;
Kd_P = 0;

% Controlador PI
Kp_PI = 0.45 * Ku;
Ki_PI = Kp_PI / (0.83 * Tu);
Kd_PI = 0;

% Controlador PID
Kp_PID = 0.6 * Ku;
Ki_PID = Kp_PID / (0.5 * Tu);
Kd_PID = Kp_PID * (0.125 * Tu);

fprintf('\nControlador P:\n');
fprintf('  Kp = %.4f\n', Kp_P);

fprintf('\nControlador PI:\n');
fprintf('  Kp = %.4f, Ki = %.4f\n', Kp_PI, Ki_PI);

fprintf('\nControlador PID (Ziegler-Nichols):\n');
fprintf('  Kp = %.4f\n', Kp_PID);
fprintf('  Ki = %.4f\n', Ki_PID);
fprintf('  Kd = %.4f\n', Kd_PID);

%% CREAR CONTROLADORES
% PID en forma paralela: C(s) = Kp + Ki/s + Kd*s
C_P = pid(Kp_P);
C_PI = pid(Kp_PI, Ki_PI);
C_PID_ZN = pid(Kp_PID, Ki_PID, Kd_PID);

fprintf('\nFunción de transferencia PID (Ziegler-Nichols):\n');
disp(C_PID_ZN);

%% SISTEMAS EN LAZO CERRADO
T_P = feedback(C_P*G, 1);
T_PI = feedback(C_PI*G, 1);
T_PID_ZN = feedback(C_PID_ZN*G, 1);

%% ANÁLISIS DE ESTABILIDAD
fprintf('\n--- Análisis de Estabilidad ---\n');

polos_P = pole(T_P);
polos_PI = pole(T_PI);
polos_PID_ZN = pole(T_PID_ZN);

fprintf('\nPolos en lazo cerrado (P):\n');
disp(polos_P);

fprintf('Polos en lazo cerrado (PI):\n');
disp(polos_PI);

fprintf('Polos en lazo cerrado (PID ZN):\n');
disp(polos_PID_ZN);

if all(real(polos_PID_ZN) < 0)
    fprintf('\n✓ Sistema con PID ZN es ESTABLE\n');
else
    fprintf('\n✗ Sistema con PID ZN es INESTABLE\n');
end

%% SIMULACIÓN Y COMPARACIÓN
figure('Name', 'Comparación de Controladores Ziegler-Nichols', 'Position', [100 100 1200 800]);

% Respuesta al escalón
subplot(2,2,1);
step(T_P, T_PI, T_PID_ZN);
grid on;
title('Respuesta al Escalón - Comparación');
legend('P', 'PI', 'PID (ZN)', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');

% Esfuerzo de control
subplot(2,2,2);
t = 0:0.01:10;
u = ones(size(t));
[~, ~, u_P] = lsim(C_P, u, t);
[~, ~, u_PI] = lsim(C_PI, u, t);
[~, ~, u_PID] = lsim(C_PID_ZN, u, t);
plot(t, u_P, t, u_PI, t, u_PID);
grid on;
title('Señal de Control');
legend('P', 'PI', 'PID (ZN)', 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('u(t) [N]');

% Diagrama de Bode en lazo cerrado
subplot(2,2,3);
bode(T_P, T_PI, T_PID_ZN);
grid on;
title('Diagrama de Bode - Lazo Cerrado');
legend('P', 'PI', 'PID (ZN)');

% Polos en lazo cerrado
subplot(2,2,4);
pzmap(T_P, T_PI, T_PID_ZN);
grid on;
title('Polos en Lazo Cerrado');
legend('P', 'PI', 'PID (ZN)', 'Location', 'best');

%% CARACTERÍSTICAS DE DESEMPEÑO
info_PID_ZN = stepinfo(T_PID_ZN);

fprintf('\n--- Características del PID Ziegler-Nichols ---\n');
fprintf('Tiempo de subida: %.4f s\n', info_PID_ZN.RiseTime);
fprintf('Tiempo de establecimiento: %.4f s\n', info_PID_ZN.SettlingTime);
fprintf('Sobrepico: %.2f %%\n', info_PID_ZN.Overshoot);
fprintf('Tiempo de pico: %.4f s\n', info_PID_ZN.PeakTime);

% Error en estado estacionario
ess_ZN = 1 - dcgain(T_PID_ZN);
fprintf('Error en estado estacionario: %.6f\n', ess_ZN);

%% GUARDAR RESULTADOS
save('ejercicio_uno/resultados_ziegler_nichols.mat', 'Ku', 'Tu', 'wu', ...
     'Kp_PID', 'Ki_PID', 'Kd_PID', 'C_PID_ZN', 'T_PID_ZN', ...
     'info_PID_ZN', 'ess_ZN', 'polos_PID_ZN');

% Guardar figuras
saveas(gcf, 'ejercicio_uno/pid_ziegler_nichols.fig');
saveas(gcf, 'ejercicio_uno/pid_ziegler_nichols.png');

fprintf('\n✓ Resultados guardados en "ejercicio_uno/resultados_ziegler_nichols.mat"\n');
fprintf('\n=== EJERCICIO E COMPLETADO ===\n');
