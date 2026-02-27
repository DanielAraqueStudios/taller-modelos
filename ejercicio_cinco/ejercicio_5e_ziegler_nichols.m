%% EJERCICIO 5E: DISEÑO PID ZIEGLER-NICHOLS - CIRCUITO RL
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_cinco/datos_sistema.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 5E: PID ZIEGLER-NICHOLS\n');
fprintf('  Circuito RL\n');
fprintf('========================================\n\n');

%% MÉTODO ZIEGLER-NICHOLS PARA SISTEMA DE PRIMER ORDEN
% Para sistema FOPDT: G(s) = K/(τs+1) con retardo θ≈0

% Como θ≈0 (sin retardo aparente), usamos aproximación
theta_aprox = tau * 0.1;  % Retardo pequeño estimado

fprintf('=== PARÁMETROS DEL SISTEMA ===\n');
fprintf('K = %.6e A/V\n', K);
fprintf('τ = %.4f s = %.2f ms\n', tau, tau*1000);
fprintf('θ (estimado) = %.4f s = %.2f ms\n\n', theta_aprox, theta_aprox*1000);

%% CÁLCULO PID ZIEGLER-NICHOLS
% Reglas ZN para PID controlando FOPDT

Kp_zn = 1.2 * tau / (K * theta_aprox);
Ki_zn = Kp_zn / (2 * theta_aprox);
Kd_zn = 0.5 * Kp_zn * theta_aprox;

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
    fprintf('\n✗ Sistema inestable\n');
end

%% GRÁFICAS
figure('Name', 'PID Ziegler-Nichols - Circuito RL', 'Position', [100 100 1200 800]);

% Respuesta al escalón
subplot(2,2,1);
step(sys_cl_zn * 1000, 0.02);  % Convertir a mA
grid on;
title('Respuesta al Escalón - Lazo Cerrado con PID ZN');
ylabel('Corriente [mA]');

% Comparación
subplot(2,2,2);
step(G * 1000, 0.02);
hold on;
step(sys_cl_zn * 1000, 0.02);
grid on;
title('Comparación Lazo Abierto vs Cerrado');
legend('Lazo Abierto', 'Lazo Cerrado PID ZN');
ylabel('Corriente [mA]');

% Lugar de raíces
subplot(2,2,3);
rlocus(C_zn * G);
grid on;
title('Lugar de las Raíces con PID');

% Bode
subplot(2,2,4);
margin(C_zn * G);
grid on;
title('Diagrama de Bode - Lazo Abierto con PID');

%% GUARDAR
save('ejercicio_cinco/pid_ziegler_nichols.mat', 'C_zn', 'sys_cl_zn', 'Kp_zn', 'Ki_zn', 'Kd_zn', 'polos_cl', 'theta_aprox');

saveas(gcf, 'ejercicio_cinco/pid_ziegler_nichols.fig');
saveas(gcf, 'ejercicio_cinco/pid_ziegler_nichols.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 5E COMPLETADO ===\n');
