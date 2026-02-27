%% SISTEMA MECÁNICO VIBRATORIO - DEFINICIÓN DE PARÁMETROS Y MODELO
% Sistema de barra articulada con resortes y amortiguadores acoplados
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% PARÁMETROS FÍSICOS DEL SISTEMA
L = 1.0;        % Longitud de la barra [m]
J = 0.5;        % Momento de inercia [kg·m²]
K1 = 500;       % Constante del resorte 1 [N/m]
K2 = 300;       % Constante del resorte 2 [N/m]
B1 = 20;        % Constante del amortiguador 1 [N·s/m]
B2 = 15;        % Constante del amortiguador 2 [N·s/m]

fprintf('=== PARÁMETROS DEL SISTEMA ===\n');
fprintf('L  = %.2f m\n', L);
fprintf('J  = %.2f kg·m²\n', J);
fprintf('K1 = %.0f N/m\n', K1);
fprintf('K2 = %.0f N/m\n', K2);
fprintf('B1 = %.0f N·s/m\n', B1);
fprintf('B2 = %.0f N·s/m\n\n', B2);

%% COEFICIENTES DE LA FUNCIÓN DE TRANSFERENCIA
% G(s) = (b1*s + b0) / (a3*s^3 + a2*s^2 + a1*s + a0)

% Numerador
b1 = L^2 * B2;
b0 = L^2 * (K1 + K2);

% Denominador
a3 = J * B2;
a2 = J*(K1 + K2) + (L^2 * B1 * B2)/4;
a1 = (L^2 * B1 * (K1 + K2))/4 + K1 * L^2 * B2;
a0 = K1 * L^2 * K2;

fprintf('=== COEFICIENTES DE LA FUNCIÓN DE TRANSFERENCIA ===\n');
fprintf('Numerador:   b1 = %.2f, b0 = %.2f\n', b1, b0);
fprintf('Denominador: a3 = %.2f, a2 = %.2f, a1 = %.2f, a0 = %.0f\n\n', a3, a2, a1, a0);

%% FUNCIÓN DE TRANSFERENCIA
num = [b1 b0];
den = [a3 a2 a1 a0];

% Crear objeto de función de transferencia
G = tf(num, den);

fprintf('=== FUNCIÓN DE TRANSFERENCIA G(s) = Y(s)/Fa(s) ===\n');
disp(G);

%% FORMA SIMPLIFICADA (dividiendo por a3)
num_simp = num/a3;
den_simp = den/a3;

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
        fprintf('  p%d = %.4f (real)\n', i, real(polos(i)));
    else
        fprintf('  p%d = %.4f %+.4fi\n', i, real(polos(i)), imag(polos(i)));
    end
end

fprintf('\nCeros del sistema:\n');
for i = 1:length(ceros)
    if imag(ceros(i)) == 0
        fprintf('  z%d = %.4f (real)\n', i, real(ceros(i)));
    else
        fprintf('  z%d = %.4f %+.4fi\n', i, real(ceros(i)), imag(ceros(i)));
    end
end

% Verificar estabilidad
if all(real(polos) < 0)
    fprintf('\n✓ SISTEMA ESTABLE (todos los polos tienen parte real negativa)\n');
else
    fprintf('\n✗ SISTEMA INESTABLE (existe al menos un polo con parte real positiva)\n');
end

%% CARACTERÍSTICAS DE LOS POLOS COMPLEJOS DOMINANTES
% Identificar polos complejos conjugados
polos_complejos = polos(abs(imag(polos)) > 1e-6);

if ~isempty(polos_complejos)
    % Tomar el par dominante (menor parte real negativa)
    [~, idx] = max(real(polos_complejos));
    polo_dom = polos_complejos(idx);
    
    wn = abs(polo_dom);
    zeta = -real(polo_dom)/wn;
    wd = abs(imag(polo_dom));
    
    fprintf('\n=== POLOS COMPLEJOS DOMINANTES ===\n');
    fprintf('Polo dominante: %.4f %+.4fi\n', real(polo_dom), imag(polo_dom));
    fprintf('Frecuencia natural (ωn): %.4f rad/s\n', wn);
    fprintf('Factor de amortiguamiento (ζ): %.4f\n', zeta);
    fprintf('Frecuencia amortiguada (ωd): %.4f rad/s\n', wd);
    
    % Predicciones teóricas
    ts_2 = 4/(zeta*wn);
    ts_5 = 3/(zeta*wn);
    Mp = exp(-pi*zeta/sqrt(1-zeta^2)) * 100;
    tp = pi/wd;
    
    fprintf('\n=== PREDICCIONES DE RESPUESTA (2do orden aproximado) ===\n');
    fprintf('Tiempo de asentamiento (2%%): %.4f s\n', ts_2);
    fprintf('Tiempo de asentamiento (5%%): %.4f s\n', ts_5);
    fprintf('Sobrepico máximo: %.2f%%\n', Mp);
    fprintf('Tiempo de pico: %.4f s\n', tp);
end

%% REPRESENTACIÓN EN ESPACIO DE ESTADOS
% Estados: x = [theta; theta_dot; x; x_dot]
% Entrada: u = fa
% Salida: y = L*theta

% Matriz A
A = [0, 1, 0, 0;
     -K1*L^2/J, -L^2*B1/(4*J), K1*L/J, 0;
     0, 0, 0, 1;
     K1*L/B2, 0, -(K1+K2)/B2, 0];

% Matriz B
B = [0; L/J; 0; 0];

% Matriz C
C = [L, 0, 0, 0];

% Matriz D
D = 0;

% Crear sistema en espacio de estados
sys_ss = ss(A, B, C, D);

fprintf('\n\n=== REPRESENTACIÓN EN ESPACIO DE ESTADOS ===\n');
fprintf('dx/dt = Ax + Bu\n');
fprintf('y = Cx + Du\n\n');

fprintf('Matriz A (4x4):\n');
disp(A);

fprintf('Matriz B (4x1):\n');
disp(B);

fprintf('Matriz C (1x4):\n');
disp(C);

fprintf('Matriz D:\n');
disp(D);

% Verificar equivalencia
fprintf('\n=== VERIFICACIÓN: Conversión SS -> TF ===\n');
G_from_ss = tf(sys_ss);
disp(G_from_ss);

%% GRÁFICAS DE CARACTERIZACIÓN
figure('Name', 'Análisis del Sistema en Lazo Abierto', 'Position', [100 100 1200 800]);

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
save('ejercicio_uno/datos_sistema.mat', 'G', 'sys_ss', 'L', 'J', 'K1', 'K2', 'B1', 'B2', ...
     'polos', 'ceros', 'A', 'B', 'C', 'D', 'num', 'den', 'b1', 'b0', 'a3', 'a2', 'a1', 'a0');

fprintf('\n✓ Datos del sistema guardados en "ejercicio_uno/datos_sistema.mat"\n');
fprintf('\n=== FIN DEL ANÁLISIS ===\n');
