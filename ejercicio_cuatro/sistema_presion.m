%% SISTEMA DE CONTROL DE PRESIÓN - EJERCICIO 4
% Sistema hidráulico con tanques y válvulas
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% PARÁMETROS FÍSICOS DEL SISTEMA
R1 = 10;     % Resistencia hidráulica 1 [Pa·s/m³]
R3 = 15;     % Resistencia hidráulica 3 [Pa·s/m³]
C1 = 0.5;    % Capacitancia tanque 1 [m³/Pa]
C2 = 0.3;    % Capacitancia tanque 2 [m³/Pa]

fprintf('=== PARÁMETROS DEL SISTEMA DE PRESIÓN ===\n');
fprintf('R1 = %.2f Pa·s/m³\n', R1);
fprintf('R3 = %.2f Pa·s/m³\n', R3);
fprintf('C1 = %.2f m³/Pa\n', C1);
fprintf('C2 = %.2f m³/Pa\n\n', C2);

%% COEFICIENTES DE LA FUNCIÓN DE TRANSFERENCIA
% G(s) = 1 / (a2*s^2 + a1*s + a0)

a2 = R1 * R3 * C1 * C2;
a1 = R1*C1 + R3*C2;
a0 = 1;

fprintf('=== COEFICIENTES DE LA FUNCIÓN DE TRANSFERENCIA ===\n');
fprintf('Denominador: a2 = %.4f, a1 = %.4f, a0 = %.4f\n\n', a2, a1, a0);

%% FUNCIÓN DE TRANSFERENCIA
num = [1];
den = [a2 a1 a0];

% Crear objeto de función de transferencia
G = tf(num, den);

fprintf('=== FUNCIÓN DE TRANSFERENCIA G(s) = P2(s)/Pi(s) ===\n');
disp(G);

%% FORMA SIMPLIFICADA (dividiendo por a2)
num_simp = num/a2;
den_simp = den/a2;

G_simp = tf(num_simp, den_simp);
fprintf('\n=== FUNCIÓN DE TRANSFERENCIA SIMPLIFICADA ===\n');
disp(G_simp);

%% ANÁLISIS DE POLOS Y CEROS
polos = pole(G);
ceros = zero(G);

fprintf('\n=== ANÁLISIS DE POLOS Y CEROS ===\n');
fprintf('Polos del sistema:\n');
for i = 1:length(polos)
    if imag(polos(i)) == 0
        fprintf('  p%d = %.6f (real)\n', i, real(polos(i)));
    else
        fprintf('  p%d = %.6f %+.6fi\n', i, real(polos(i)), imag(polos(i)));
    end
end

if isempty(ceros)
    fprintf('\nNo hay ceros en el numerador (sistema sin ceros)\n');
else
    fprintf('\nCeros del sistema:\n');
    for i = 1:length(ceros)
        fprintf('  z%d = %.6f\n', i, ceros(i));
    end
end

% Verificar estabilidad
if all(real(polos) < 0)
    fprintf('\n✓ SISTEMA ESTABLE (todos los polos tienen parte real negativa)\n');
else
    fprintf('\n✗ SISTEMA INESTABLE\n');
end

%% CARACTERÍSTICAS DEL SISTEMA DE SEGUNDO ORDEN
if length(polos) == 2
    % Calcular parámetros de segundo orden
    if abs(imag(polos(1))) > 1e-6
        % Polos complejos conjugados
        wn = abs(polos(1));
        zeta = -real(polos(1))/wn;
        wd = abs(imag(polos(1)));
        
        fprintf('\n=== SISTEMA DE SEGUNDO ORDEN (POLOS COMPLEJOS) ===\n');
        fprintf('Frecuencia natural (ωn): %.6f rad/s\n', wn);
        fprintf('Factor de amortiguamiento (ζ): %.6f\n', zeta);
        fprintf('Frecuencia amortiguada (ωd): %.6f rad/s\n', wd);
        
        if zeta < 1
            fprintf('Sistema SUBAMORTIGUADO (ζ < 1)\n');
        elseif abs(zeta - 1) < 1e-6
            fprintf('Sistema CRÍTICAMENTE AMORTIGUADO (ζ = 1)\n');
        else
            fprintf('Sistema SOBREAMORTIGUADO (ζ > 1)\n');
        end
        
        % Predicciones teóricas
        ts_2 = 4/(zeta*wn);
        ts_5 = 3/(zeta*wn);
        
        if zeta < 1
            Mp = exp(-pi*zeta/sqrt(1-zeta^2)) * 100;
            tp = pi/wd;
            fprintf('\nSobrepico máximo teórico: %.2f%%\n', Mp);
            fprintf('Tiempo de pico teórico: %.4f s\n', tp);
        end
        
        fprintf('Tiempo de asentamiento (2%%): %.4f s\n', ts_2);
        fprintf('Tiempo de asentamiento (5%%): %.4f s\n', ts_5);
    else
        % Polos reales
        fprintf('\n=== SISTEMA DE SEGUNDO ORDEN (POLOS REALES) ===\n');
        fprintf('Polo 1: %.6f\n', polos(1));
        fprintf('Polo 2: %.6f\n', polos(2));
        fprintf('Sistema SOBREAMORTIGUADO (dos polos reales distintos)\n');
        
        % Tiempo de asentamiento aproximado (polo dominante)
        polo_dominante = max(real(polos));
        ts_aprox = 4/abs(polo_dominante);
        fprintf('Tiempo de asentamiento aproximado: %.4f s\n', ts_aprox);
    end
end

%% REPRESENTACIÓN EN ESPACIO DE ESTADOS
% Estados: x = [p1; p2]
% Entrada: u = pi
% Salida: y = p2

% Matriz A
A = [-(R1+R3)/(R1*R3*C1),  1/C1;
      1/(R3*C2),           -(1)/( R3*C2)];

% Matriz B
B = [1/(R1*C1); 0];

% Matriz C (salida es p2)
C = [0, 1];

% Matriz D
D = 0;

% Crear sistema en espacio de estados
sys_ss = ss(A, B, C, D);

fprintf('\n\n=== REPRESENTACIÓN EN ESPACIO DE ESTADOS ===\n');
fprintf('dx/dt = Ax + Bu\n');
fprintf('y = Cx + Du\n\n');

fprintf('Matriz A (2x2):\n');
disp(A);

fprintf('Matriz B (2x1):\n');
disp(B);

fprintf('Matriz C (1x2):\n');
disp(C);

fprintf('Matriz D:\n');
disp(D);

% Verificar equivalencia
fprintf('\n=== VERIFICACIÓN: Conversión SS -> TF ===\n');
G_from_ss = tf(sys_ss);
disp(G_from_ss);

%% GRÁFICAS DE CARACTERIZACIÓN
figure('Name', 'Análisis del Sistema de Presión en Lazo Abierto', 'Position', [100 100 1200 800]);

% Diagrama de polos y ceros
subplot(2,3,1);
pzmap(G);
grid on;
title('Diagrama de Polos y Ceros');
xlabel('Parte Real');
ylabel('Parte Imaginaria');

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

%% GUARDAR DATOS DEL SISTEMA
save('ejercicio_cuatro/datos_sistema.mat', 'G', 'sys_ss', 'R1', 'R3', 'C1', 'C2', ...
     'polos', 'ceros', 'A', 'B', 'C', 'D', 'num', 'den', 'a2', 'a1', 'a0');

% Guardar figura
saveas(gcf, 'ejercicio_cuatro/analisis_sistema.fig');
saveas(gcf, 'ejercicio_cuatro/analisis_sistema.png');

fprintf('\n✓ Datos del sistema guardados en "ejercicio_cuatro/datos_sistema.mat"\n');
fprintf('\n=== FIN DEL ANÁLISIS ===\n');
