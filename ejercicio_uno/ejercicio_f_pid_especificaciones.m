%% EJERCICIO F: DISEÑO DE PID CON ESPECIFICACIONES
% Diseño de controlador PID con especificaciones de desempeño:
% - ts = 95% del tiempo de establecimiento en lazo abierto
% - zeta = 0.9
% - ess = 0 para entrada escalón
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS DEL SISTEMA Y RESULTADOS PREVIOS
load('ejercicio_uno/datos_sistema.mat');
load('ejercicio_uno/resultados_lazo_abierto.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO F: PID CON ESPECIFICACIONES\n');
fprintf('========================================\n\n');

%% ESPECIFICACIONES DE DISEÑO
ts_lazo_abierto = ts_lazo_abierto_2;  % Tiempo de establecimiento en lazo abierto (2%)
ts_deseado = 0.95 * ts_lazo_abierto;   % 95% del ts en lazo abierto
zeta_deseado = 0.9;                    % Factor de amortiguamiento
ess_deseado = 0;                       % Error en estado estacionario = 0

fprintf('--- Especificaciones de Diseño ---\n');
fprintf('Tiempo de establecimiento lazo abierto: %.4f s\n', ts_lazo_abierto);
fprintf('Tiempo de establecimiento deseado: %.4f s (95%% del LA)\n', ts_deseado);
fprintf('Factor de amortiguamiento deseado: %.2f\n', zeta_deseado);
fprintf('Error estado estacionario: %.0f (entrada escalón)\n\n', ess_deseado);

%% CÁLCULO DE FRECUENCIA NATURAL
% Para un sistema de segundo orden: ts = 4/(zeta*wn)
wn_deseado = 4 / (zeta_deseado * ts_deseado);

fprintf('--- Parámetros del Sistema de Segundo Orden Equivalente ---\n');
fprintf('Frecuencia natural deseada (ωn): %.4f rad/s\n', wn_deseado);
fprintf('Factor de amortiguamiento (ζ): %.2f\n', zeta_deseado);

%% POLOS DOMINANTES DESEADOS
% Polos complejos conjugados
sigma = -zeta_deseado * wn_deseado;
wd = wn_deseado * sqrt(1 - zeta_deseado^2);

polo_deseado_1 = sigma + 1j*wd;
polo_deseado_2 = sigma - 1j*wd;

% Tercer polo (debe ser más rápido que los dominantes)
% Regla típica: 5 a 10 veces más rápido
polo_rapido = -10 * zeta_deseado * wn_deseado;

fprintf('\nPolos deseados en lazo cerrado:\n');
fprintf('  p1, p2 = %.4f ± j%.4f\n', sigma, wd);
fprintf('  p3 = %.4f (polo rápido)\n', polo_rapido);

%% ECUACIÓN CARACTERÍSTICA DESEADA
% (s - p1)(s - p2)(s - p3) = 0
% (s^2 + 2*zeta*wn*s + wn^2)(s - p3) = 0

a2_des = 2*zeta_deseado*wn_deseado;
a1_des = wn_deseado^2;

% Expandir: (s^2 + a2_des*s + a1_des)(s - p3)
% s^3 + (a2_des - p3)*s^2 + (a1_des - p3*a2_des)*s - p3*a1_des

coef_s3 = 1;
coef_s2 = a2_des - polo_rapido;
coef_s1 = a1_des - polo_rapido*a2_des;
coef_s0 = -polo_rapido*a1_des;

fprintf('\nEcuación característica deseada:\n');
fprintf('s^3 + %.4f*s^2 + %.4f*s + %.4f = 0\n', coef_s2, coef_s1, coef_s0);

%% DISEÑO DEL CONTROLADOR PID
% Sistema: G(s) = (b1*s + b0) / (a3*s^3 + a2*s^2 + a1*s + a0)
% Controlador: C(s) = Kd*s^2 + Kp*s + Ki / s
% Lazo cerrado: 1 + C(s)*G(s) = 0

% Ecuación característica en lazo cerrado:
% s*(a3*s^3 + a2*s^2 + a1*s + a0) + (Kd*s^2 + Kp*s + Ki)*(b1*s + b0) = 0
% a3*s^4 + a2*s^3 + a1*s^2 + a0*s + Kd*b1*s^3 + Kd*b0*s^2 + Kp*b1*s^2 + Kp*b0*s + Ki*b1*s + Ki*b0 = 0
% a3*s^4 + (a2 + Kd*b1)*s^3 + (a1 + Kd*b0 + Kp*b1)*s^2 + (a0 + Kp*b0 + Ki*b1)*s + Ki*b0 = 0

% Coeficientes del sistema
num_G = [b1 b0];
den_G = [a3 a2 a1 a0];

fprintf('\n--- Coeficientes del Sistema ---\n');
fprintf('G(s) = (%.2f*s + %.2f) / (%.2f*s^3 + %.2f*s^2 + %.2f*s + %.0f)\n', ...
        b1, b0, a3, a2, a1, a0);

% Para un sistema de orden 3, queremos igualar con la característica deseada
% Multiplicamos la ecuación deseada por s para tener orden 4:
% s^4 + coef_s2*s^3 + coef_s1*s^2 + coef_s0*s = 0

% Igualando coeficientes (método simplificado):
% Usaremos el método de colocación de polos con controlador PID

% Sistema aumentado con integrador
% G_aug(s) = G(s)/s
num_aug = num_G;
den_aug = conv(den_G, [1 0]);  % Multiplica por s

% Usar el comando 'place' no es directo para PID, así que usaremos un enfoque iterativo
% o diseño por igualación de coeficientes

% Método alternativo: Usar SISOTOOL o diseño manual
% Para este caso, usaremos valores iniciales y refinamiento

% Aproximación inicial basada en la ubicación de polos
% Usando el método de igualación aproximada

% De la ecuación característica deseada (orden 3):
% s^3 + coef_s2*s^2 + coef_s1*s + coef_s0 = 0

% Sistema en lazo cerrado con PID:
% Denominador = den_G + Kd*b1*s^3 + (Kd*b0 + Kp*b1)*s^2 + (Kp*b0 + Ki*b1)*s + Ki*b0

% Igualando:
% a3 + Kd*b1 = coef_s3 = 1
% a2 + Kd*b0 + Kp*b1 = coef_s2
% a1 + Kp*b0 + Ki*b1 = coef_s1
% a0 + Ki*b0 = coef_s0

% Pero esto da orden 3, necesitamos considerar que el sistema real será de orden 4
% Ajustemos el enfoque:

% Usaremos optimización numérica para encontrar Kp, Ki, Kd
% tal que los polos dominantes estén cerca de los deseados

% Función objetivo: minimizar la distancia de los polos a los deseados
objetivo = @(K) calcular_error_polos(K, G, polo_deseado_1, polo_deseado_2, polo_rapido);

% Valores iniciales (heurística)
K0 = [80; 950; 5];  % [Kp; Ki; Kd]

% Optimización
options = optimset('Display', 'off', 'TolX', 1e-6, 'MaxIter', 1000);
K_opt = fminsearch(objetivo, K0, options);

Kp = K_opt(1);
Ki = K_opt(2);
Kd = K_opt(3);

fprintf('\n--- Parámetros del PID Optimizado ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

%% CREAR CONTROLADOR PID
C_spec = pid(Kp, Ki, Kd);

fprintf('\nControlador PID:\n');
fprintf('C(s) = %.4f + %.4f/s + %.4f*s\n', Kp, Ki, Kd);

%% SISTEMA EN LAZO CERRADO
T_spec = feedback(C_spec*G, 1);

fprintf('\n--- Sistema en Lazo Cerrado ---\n');
fprintf('Función de transferencia T(s) = Y(s)/R(s):\n');
disp(T_spec);

%% ANÁLISIS DE POLOS EN LAZO CERRADO
polos_CL = pole(T_spec);
fprintf('\nPolos en lazo cerrado:\n');
for i = 1:length(polos_CL)
    if abs(imag(polos_CL(i))) < 1e-6
        fprintf('  p%d = %.4f (real)\n', i, real(polos_CL(i)));
    else
        fprintf('  p%d = %.4f %+.4fi\n', i, real(polos_CL(i)), imag(polos_CL(i)));
    end
end

% Verificar estabilidad
if all(real(polos_CL) < 0)
    fprintf('\n✓ Sistema en lazo cerrado ESTABLE\n');
else
    fprintf('\n✗ Sistema en lazo cerrado INESTABLE\n');
end

%% VERIFICACIÓN DE ESPECIFICACIONES
info_spec = stepinfo(T_spec);

fprintf('\n--- Verificación de Especificaciones ---\n');
fprintf('Tiempo de establecimiento (2%%): %.4f s (objetivo: %.4f s)\n', ...
        info_spec.SettlingTime, ts_deseado);
fprintf('Sobrepico: %.2f %%\n', info_spec.Overshoot);

% Calcular zeta y wn de los polos dominantes
polos_complejos = polos_CL(abs(imag(polos_CL)) > 1e-6);
if ~isempty(polos_complejos)
    polo_dom = polos_complejos(1);
    wn_real = abs(polo_dom);
    zeta_real = -real(polo_dom) / wn_real;
    fprintf('Factor de amortiguamiento real: %.4f (objetivo: %.2f)\n', zeta_real, zeta_deseado);
    fprintf('Frecuencia natural real: %.4f rad/s (objetivo: %.4f rad/s)\n', wn_real, wn_deseado);
end

% Error en estado estacionario
ess_real = 1 - dcgain(T_spec);
fprintf('Error en estado estacionario: %.6f (objetivo: %.0f)\n', ess_real, ess_deseado);

%% GRÁFICAS
figure('Name', 'PID con Especificaciones', 'Position', [100 100 1200 800]);

% Respuesta al escalón
subplot(2,2,1);
step(T_spec);
grid on;
hold on;
yline(1, 'k--', 'LineWidth', 1);
yline(1.02, 'r--', 'LineWidth', 0.5);
yline(0.98, 'r--', 'LineWidth', 0.5);
xline(ts_deseado, 'g--', sprintf('t_s objetivo = %.3f s', ts_deseado), 'LineWidth', 1.5);
xline(info_spec.SettlingTime, 'b--', sprintf('t_s real = %.3f s', info_spec.SettlingTime), 'LineWidth', 1.5);
title('Respuesta al Escalón - Lazo Cerrado');
xlabel('Tiempo [s]');
ylabel('Salida y(t) [m]');
legend('Respuesta', 'Referencia', 'Banda ±2%', '', ...
       sprintf('t_s obj = %.3f s', ts_deseado), ...
       sprintf('t_s real = %.3f s', info_spec.SettlingTime), ...
       'Location', 'best');

% Esfuerzo de control
subplot(2,2,2);
t = 0:0.001:2;
u = ones(size(t));
[y_ctrl, t_ctrl, u_ctrl] = lsim(feedback(C_spec, G), u, t);
plot(t_ctrl, u_ctrl, 'b-', 'LineWidth', 1.5);
grid on;
title('Señal de Control');
xlabel('Tiempo [s]');
ylabel('u(t) [N]');

% Diagrama de Bode
subplot(2,2,3);
bode(T_spec);
grid on;
title('Diagrama de Bode - Lazo Cerrado');

% Polos y ceros
subplot(2,2,4);
pzmap(T_spec);
grid on;
hold on;
% Marcar polos deseados
plot(real(polo_deseado_1), imag(polo_deseado_1), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
plot(real(polo_deseado_2), imag(polo_deseado_2), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
plot(polo_rapido, 0, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
legend('Polos reales', 'Ceros', 'Polos deseados', 'Location', 'best');
title('Diagrama de Polos y Ceros');

%% GUARDAR RESULTADOS
save('ejercicio_uno/resultados_pid_especificaciones.mat', 'Kp', 'Ki', 'Kd', 'C_spec', 'T_spec', ...
     'info_spec', 'polos_CL', 'ts_deseado', 'zeta_deseado', 'wn_deseado');

% Guardar figuras
saveas(gcf, 'ejercicio_uno/pid_especificaciones.fig');
saveas(gcf, 'ejercicio_uno/pid_especificaciones.png');

fprintf('\n✓ Resultados guardados en "ejercicio_uno/resultados_pid_especificaciones.mat"\n');
fprintf('\n=== EJERCICIO F COMPLETADO ===\n');

%% FUNCIÓN AUXILIAR PARA OPTIMIZACIÓN
function error = calcular_error_polos(K, G, p1_des, p2_des, p3_des)
    % Calcula el error entre los polos deseados y los reales
    Kp = K(1);
    Ki = K(2);
    Kd = K(3);
    
    % Crear controlador
    C = pid(Kp, Ki, Kd);
    
    % Sistema en lazo cerrado
    T = feedback(C*G, 1);
    
    % Polos reales
    polos_reales = pole(T);
    
    % Calcular distancia a los polos deseados
    polos_deseados = [p1_des; p2_des; p3_des];
    
    error = 0;
    for i = 1:length(polos_deseados)
        % Distancia mínima al polo deseado
        dist = min(abs(polos_reales - polos_deseados(i)));
        error = error + dist^2;
    end
    
    % Penalizar polos inestables
    if any(real(polos_reales) > 0)
        error = error + 1e6;
    end
end
