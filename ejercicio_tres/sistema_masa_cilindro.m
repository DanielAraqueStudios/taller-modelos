%% SISTEMA MASA-CILINDRO ACOPLADO - EJERCICIO 3
% Sistema vibratorio con masa y cilindro
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% PARÁMETROS FÍSICOS
M = 5;      % Masa [kg]
J = 0.1;    % Momento de inercia del cilindro [kg·m²]
R = 0.2;    % Radio del cilindro [m]
K = 200;    % Constante del resorte principal [N/m]
K2 = 100;   % Constante del segundo resorte [N/m]

fprintf('=== PARÁMETROS DEL SISTEMA MASA-CILINDRO ===\n');
fprintf('M = %.2f kg\n', M);
fprintf('J = %.2f kg·m²\n', J);
fprintf('R = %.2f m\n', R);
fprintf('K = %.2f N/m\n', K);
fprintf('K2 = %.2f N/m\n\n', K2);

%% MASAS Y CONSTANTES EQUIVALENTES
Meq = M + J/(R^2);
Keq = K + K2;

fprintf('Masa equivalente: M_eq = %.2f kg\n', Meq);
fprintf('Constante equivalente: K_eq = %.2f N/m\n\n', Keq);

%% FUNCIÓN DE TRANSFERENCIA
% G(s) = 1 / (Meq*s^2 + Keq)
num = [1/Meq];
den = [1, 0, Keq/Meq];

G = tf(num, den);

fprintf('=== FUNCIÓN DE TRANSFERENCIA G(s) ===\n');
disp(G);

%% ANÁLISIS DE POLOS
polos = pole(G);
wn = sqrt(Keq/Meq);

fprintf('\n=== POLOS DEL SISTEMA ===\n');
for i = 1:length(polos)
    if imag(polos(i)) == 0
        fprintf('p%d = %.6f (real)\n', i, real(polos(i)));
    else
        fprintf('p%d = %.6f %+.6fi\n', i, real(polos(i)), imag(polos(i)));
    end
end

fprintf('\nFrecuencia natural: ωn = %.4f rad/s\n', wn);
fprintf('Sistema marginalmente estable (polos puramente imaginarios)\n');

%% ESPACIO DE ESTADOS
A = [0, 1; 
     -Keq/Meq, 0];
B = [0; 1/Meq];
C = [1, 0];
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
figure('Name', 'Análisis del Sistema Masa-Cilindro', 'Position', [100 100 1200 800]);

% Diagrama de polos y ceros
subplot(2,3,1);
pzmap(G);
grid on;
title('Diagrama de Polos y Ceros');

% Respuesta al impulso
subplot(2,3,2);
impulse(G, 5);
grid on;
title('Respuesta al Impulso');

% Respuesta al escalón
subplot(2,3,3);
step(G, 5);
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

%% GUAR DAR DATOS
save('ejercicio_tres/datos_sistema.mat', 'G', 'sys_ss', 'M', 'J', 'R', 'K', 'K2', 'Meq', 'Keq', 'polos', 'wn', 'A', 'B', 'C', 'D');

saveas(gcf, 'ejercicio_tres/analisis_sistema.fig');
saveas(gcf, 'ejercicio_tres/analisis_sistema.png');

fprintf('\n✓ Datos guardados en "ejercicio_tres/datos_sistema.mat"\n');
fprintf('\n=== FIN DEL ANÁLISIS ===\n');
