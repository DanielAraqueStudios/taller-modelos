%% CIRCUITO RL - EJERCICIO 5
% Sistema eléctrico con resistencias e inductor
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% PARÁMETROS DEL CIRCUITO
Vs = 10;      % Voltaje de fuente [V]
R1 = 4000;    % Resistencia 1 [Ω] = 4 kΩ
R2 = 2000;    % Resistencia 2 [Ω] = 2 kΩ  
L = 18;       % Inductancia [H] - determinada de la gráfica
iL0 = 0.003;  % Corriente inicial [A] = 3 mA

fprintf('=== PARÁMETROS DEL CIRCUITO RL ===\n');
fprintf('Vs = %.2f V\n', Vs);
fprintf('R1 = %.2f Ω = %.2f kΩ\n', R1, R1/1000);
fprintf('R2 = %.2f Ω = %.2f kΩ\n', R2, R2/1000);
fprintf('L = %.2f H\n', L);
fprintf('iL(0) = %.3f A = %.2f mA\n\n', iL0, iL0*1000);

%% RESISTENCIA EQUIVALENTE
Req = R1 + R2;
fprintf('Resistencia equivalente: Req = %.2f Ω = %.2f kΩ\n\n', Req, Req/1000);

%% CONSTANTE DE TIEMPO
tau = L / Req;
fprintf('Constante de tiempo: τ = %.6f s = %.2f ms\n', tau, tau*1000);

%% GANANCIA DC
K = 1 / Req;
fprintf('Ganancia DC: K = %.6e A/V\n', K);
fprintf('Corriente en estado estacionario: iL(∞) = %.6f A = %.3f mA\n\n', Vs*K, Vs*K*1000);

%% FUNCIÓN DE TRANSFERENCIA
% G(s) = K / (τs + 1) = (1/Req) / ((L/Req)s + 1)
num = [K];
den = [tau, 1];

G = tf(num, den);

fprintf('=== FUNCIÓN DE TRANSFERENCIA G(s) ===\n');
disp(G);

%% FORMA ESTÁNDAR
num_std = [1/Req];
den_std = [L, Req];

G_std = tf(num_std, den_std);

fprintf('\nForma estándar (corriente en mA por V de entrada):\n');
G_mA = G * 1000;  % Convertir a mA
disp(G_mA);

%% POLOS
polos = pole(G);
fprintf('\n=== POLOS DEL SISTEMA ===\n');
fprintf('polo = %.4f (real negativo)\n', polos);
fprintf('Sistema ESTABLE de primer orden\n');

%% ESPACIO DE ESTADOS
A = -Req/L;
B = 1/L;
C = 1;
D = 0;

sys_ss = ss(A, B, C, D);

fprintf('\n=== ESPACIO DE ESTADOS ===\n');
fprintf('A = %.4f\n', A);
fprintf('B = %.4f\n', B);
fprintf('C = %.4f\n', C);
fprintf('D = %.4f\n\n', D);

%% SIMULACIÓN DE LA RESPUESTA
% Con condición inicial iL(0) = 3 mA
t = 0:0.0001:0.015;  % 0 a 15 ms

% Respuesta temporal con CI
iL_inf = Vs * K;
iL = iL_inf + (iL0 - iL_inf) * exp(-t/tau);

%% GRÁFICAS
figure('Name', 'Análisis del Circuito RL', 'Position', [100 100 1200 800]);

% Respuesta temporal
subplot(2,3,1);
plot(t*1000, iL*1000, 'b-', 'LineWidth', 2);
hold on;
yline(iL_inf*1000, 'r--', 'LineWidth', 1.5);
yline(iL0*1000, 'g--', 'LineWidth', 1.5);
% Marcar constante de tiempo
plot(tau*1000, (iL_inf + (iL0-iL_inf)*exp(-1))*1000, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
xlabel('Tiempo [ms]');
ylabel('Corriente i_L(t) [mA]');
title('Respuesta Temporal con CI');
legend('i_L(t)', 'i_L(\infty)', 'i_L(0)', '\tau', 'Location', 'best');

% Respuesta al escalón (sin CI)
subplot(2,3,2);
step(G_mA);
grid on;
title('Respuesta al Escalón Unitario');
ylabel('Corriente [mA/V]');

% Diagrama de polos y ceros
subplot(2,3,3);
pzmap(G);
grid on;
title('Diagrama de Polos y Ceros');

% Diagrama de Bode
subplot(2,3,[4,5]);
bode(G);
grid on;
title('Diagrama de Bode');

% Análisis de la respuesta
subplot(2,3,6);
axis off;
text(0.1, 0.9, '\bf Características del Sistema:', 'FontSize', 12);

info_text = {
    sprintf('Constante de tiempo: τ = %.2f ms', tau*1000);
    sprintf('Frecuencia de corte: fc = %.2f Hz', 1/(2*pi*tau));
    sprintf('Tiempo establecimiento (2%%): ts ≈ %.2f ms', 4*tau*1000);
    sprintf('Tiempo establecimiento (5%%): ts ≈ %.2f ms', 3*tau*1000);
    '';
    sprintf('Ganancia DC: K = %.6f A/V', K);
    sprintf('iL(∞) para Vs=10V: %.3f mA', Vs*K*1000);
    '';
    'Sistema de PRIMER ORDEN';
    'Sin oscilaciones (polo real)';
    'ESTABLE';
};

text(0.1, 0.8, info_text, 'FontSize', 10, 'VerticalAlignment', 'top');

%% GUARDAR DATOS
save('ejercicio_cinco/datos_sistema.mat', 'G', 'G_mA', 'sys_ss', 'Vs', 'R1', 'R2', 'L', 'Req', 'tau', 'K', 'polos', 'iL0', 'A', 'B', 'C', 'D');

saveas(gcf, 'ejercicio_cinco/analisis_sistema.fig');
saveas(gcf, 'ejercicio_cinco/analisis_sistema.png');

fprintf('✓ Datos guardados en "ejercicio_cinco/datos_sistema.mat"\n');
fprintf('\n=== FIN DEL ANÁLISIS ===\n');
