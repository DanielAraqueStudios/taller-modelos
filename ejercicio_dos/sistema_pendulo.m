%% SISTEMA PÉNDULO CON RESORTES - EJERCICIO 2
% Sistema de péndulo con resortes K1 y K2
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% PARÁMETROS FÍSICOS DEL SISTEMA
M = 2.0;    % Masa del péndulo [kg]
L = 1.5;    % Longitud de la cuerda [m]
K1 = 100;   % Constante del resorte 1 [N/m]
K2 = 150;   % Constante del resorte 2 [N/m]
g = 9.81;   % Aceleración gravitacional [m/s²]

fprintf('=== PARÁMETROS DEL SISTEMA PÉNDULO ===\n');
fprintf('M = %.2f kg\n', M);
fprintf('L = %.2f m\n', L);
fprintf('K1 = %.2f N/m\n', K1);
fprintf('K2 = %.2f N/m\n', K2);
fprintf('g = %.2f m/s²\n\n', g);

%% CONSTANTE EQUIVALENTE
Keq = (K2 * (2*K1 + K2)) / (K1 + K2);
fprintf('Constante equivalente: K_eq = %.2f N/m\n\n', Keq);

%% FUNCIÓN DE TRANSFERENCIA
% G(s) = 1/M / (s^2 + g/L + Keq/M)
num = [1/M];
den = [1, 0, g/L + Keq/M];

% Crear función de transferencia
G = tf(num, den);

fprintf('=== FUNCIÓN DE TRANSFERENCIA G(s) ===\n');
disp(G);

%% ANÁLISIS DE POLOS
polos = pole(G);
fprintf('\n=== POLOS DEL SISTEMA ===\n');
for i = 1:length(polos)
    if imag(polos(i)) == 0
        fprintf('p%d = %.6f (real)\n', i, real(polos(i)));
    else
        fprintf('p%d = %.6f %+.6fi\n', i, real(polos(i)), imag(polos(i)));
    end
end

% Frecuencia natural
wn = sqrt(g/L + Keq/M);
fprintf('\nFrecuencia natural: ωn = %.4f rad/s\n', wn);

if abs(imag(polos(1))) > 1e-6
    fprintf('Sistema con polos complejos conjugados\n');
    if abs(real(polos(1))) < 1e-6
        fprintf('Sistema MARGINALMENTE ESTABLE\n');
    end
else
    fprintf('Sistema con polos reales\n');
end

%% REPRESENTACIÓN EN ESPACIO DE ESTADOS
% dx/dt = Ax + Bu
% y = Cx + Du

A = [0, 1; 
     -(g/L + Keq/M), 0];
B = [0; 1/(M*L)];
C = [L, 0];
D = 0;

sys_ss = ss(A, B, C, D);

fprintf('\n=== ESPACIO DE ESTADOS ===\n');
fprintf('Matriz A:\n');
disp(A);
fprintf('Matriz B:\n');
disp(B);
fprintf('Matriz C:\n');
disp(C);
fprintf('Matriz D:\n');
disp(D);

%% GRÁFICAS
figure('Name', 'Análisis del Sistema Péndulo', 'Position', [100 100 1200 800]);

% Diagrama de polos y ceros
subplot(2,3,1);
pzmap(G);
grid on;
title('Diagrama de Polos y Ceros');

% Respuesta al impulso
subplot(2,3,2);
impulse(G);
grid on;
title('Respuesta al Impulso');

% Respuesta al escalón
subplot(2,3,3);
step(G);
grid on;
title('Respuesta al Escalón');

% Diagrama de Bode
subplot(2,3,[4,5]);
bode(G);
grid on;
title('Diagrama de Bode');

% Lugar de las raíces
subplot(2,3,6);
rlocus(G);
grid on;
title('Lugar de las Raíces');

%% GUARDAR DATOS
save('ejercicio_dos/datos_sistema.mat', 'G', 'sys_ss', 'M', 'L', 'K1', 'K2', 'g', 'Keq', 'polos', 'wn', 'A', 'B', 'C', 'D');

saveas(gcf, 'ejercicio_dos/analisis_sistema.fig');
saveas(gcf, 'ejercicio_dos/analisis_sistema.png');

fprintf('\n✓ Datos guardados en "ejercicio_dos/datos_sistema.mat"\n');
fprintf('\n=== FIN DEL ANÁLISIS ===\n');
