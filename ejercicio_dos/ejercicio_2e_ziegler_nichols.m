%% EJERCICIO 2E: DISEÑO PID ZIEGLER-NICHOLS - SISTEMA PÉNDULO
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_dos/datos_sistema.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 2E: PID ZIEGLER-NICHOLS\n');
fprintf('  Sistema Péndulo con Resortes\n');
fprintf('========================================\n\n');

%% MÉTODO DE ZIEGLER-NICHOLS (LUGAR DE RAÍCES)
% Buscar ganancia crítica Ku y período crítico Tu

% Para sistema de segundo orden sin amortiguamiento: wn = 10.56 rad/s
% Ganancia crítica: aproximadamente 2*wn^2*M para sistema tipo este

% Ganancia crítica (estimada)
Ku = 223;  % Ganancia donde el sistema oscila sustentadamente

% Período crítico
Tu = 2*pi/wn;

fprintf('=== MÉTODO ZIEGLER-NICHOLS ===\n');
fprintf('Ganancia crítica: Ku = %.2f\n', Ku);
fprintf('Período crítico: Tu = %.4f s\n', Tu);
fprintf('Frecuencia de oscilación: %.4f rad/s\n\n', 2*pi/Tu);

%% CÁLCULO DE PARÁMETROS PID
Kp_zn = 0.6 * Ku;
Ki_zn = Kp_zn / (Tu/2);
Kd_zn = Kp_zn * (Tu/8);

fprintf('=== PARÁMETROS PID ZIEGLER-NICHOLS ===\n');
fprintf('Kp = %.2f\n', Kp_zn);
fprintf('Ki = %.2f\n', Ki_zn);
fprintf('Kd = %.2f\n\n', Kd_zn);

% Controlador PID
C_zn = pid(Kp_zn, Ki_zn, Kd_zn);

fprintf('Controlador PID:\n');
disp(C_zn);

%% SISTEMA EN LAZO CERRADO
sys_cl_zn = feedback(C_zn * G, 1);

fprintf('\n=== ANÁLISIS DE ESTABILIDAD ===\n');
polos_cl = pole(sys_cl_zn);
fprintf('Polos en lazo cerrado:\n');
for i = 1:length(polos_cl)
    if imag(polos_cl(i)) == 0
        fprintf('  p%d = %.4f\n', i, real(polos_cl(i)));
    else
        fprintf('  p%d = %.4f %+.4fi\n', i, real(polos_cl(i)), imag(polos_cl(i)));
    end
end

if all(real(polos_cl) < 0)
    fprintf('\n✓ Sistema estable en lazo cerrado\n');
else
    fprintf('\n✗ Sistema inestable en lazo cerrado\n');
end

%% GRÁFICAS
figure('Name', 'PID Ziegler-Nichols - Péndulo', 'Position', [100 100 1200 800]);

% Respuesta al escalón
subplot(2,2,1);
step(sys_cl_zn, 5);
grid on;
title('Respuesta al Escalón - Lazo Cerrado con PID ZN');
ylabel('Salida');

% Comparación con lazo abierto
subplot(2,2,2);
step(G, 5);
hold on;
step(sys_cl_zn, 5);
grid on;
title('Comparación Lazo Abierto vs Cerrado');
legend('Lazo Abierto', 'Lazo Cerrado PID ZN');

% Lugar de las raíces
subplot(2,2,3);
rlocus(C_zn * G);
grid on;
title('Lugar de las Raíces con PID');

% Diagrama de Bode
subplot(2,2,4);
margin(C_zn * G);
grid on;
title('Diagrama de Bode - Lazo Abierto con PID');

%% GUARDAR RESULTADOS
save('ejercicio_dos/pid_ziegler_nichols.mat', 'C_zn', 'sys_cl_zn', 'Kp_zn', 'Ki_zn', 'Kd_zn', 'Ku', 'Tu', 'polos_cl');

saveas(gcf, 'ejercicio_dos/pid_ziegler_nichols.fig');
saveas(gcf, 'ejercicio_dos/pid_ziegler_nichols.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 2E COMPLETADO ===\n');
