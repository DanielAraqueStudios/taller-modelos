%% EJERCICIO 5G: SISTEMA CON RETARDO - CIRCUITO RL
% Retardo τ = 0.7 s = 700 ms
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_cinco/datos_sistema.mat');
load('ejercicio_cinco/pid_ziegler_nichols.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 5G: SISTEMA CON RETARDO\n');
fprintf('  Circuito RL\n');
fprintf('========================================\n\n');

%% PARÁMETROS DE RETARDO
tau_delay = 0.7;  % Retardo en segundos

fprintf('Retardo del sistema: τ_delay = %.2f s = %.0f ms\n', tau_delay, tau_delay*1000);
fprintf('Constante de tiempo del sistema: τ = %.2f ms\n\n', tau*1000);
fprintf('NOTA: Retardo >> τ del sistema (retardo muy grande)\n\n');

%% APROXIMACIÓN DE PADÉ
% Aproximación de primer orden
pade_num = [1, -tau_delay/2];
pade_den = [1, tau_delay/2];
H_delay = tf(pade_num, pade_den);

fprintf('Aproximación de Padé (1er orden):\n');
disp(H_delay);

% Sistema con retardo
G_delay = G * H_delay;

%% ESTRATEGIA 1: REDUCCIÓN CONSERVADORA
fprintf('\n=== ESTRATEGIA 1: REDUCCIÓN CONSERVADORA DE GANANCIAS ===\n');

% Reducción agresiva debido al gran retardo
factor_p = 0.5;
factor_i = 0.4;
factor_d = 0.6;

Kp_delay1 = factor_p * Kp_zn;
Ki_delay1 = factor_i * Ki_zn;
Kd_delay1 = factor_d * Kd_zn;

fprintf('Kp_delay = %.2f (%.0f%% de Kp original)\n', Kp_delay1, factor_p*100);
fprintf('Ki_delay = %.2f (%.0f%% de Ki original)\n', Ki_delay1, factor_i*100);
fprintf('Kd_delay = %.2f (%.0f%% de Kd original)\n\n', Kd_delay1, factor_d*100);

C_delay1 = pid(Kp_delay1, Ki_delay1, Kd_delay1);
sys_cl_delay1 = feedback(C_delay1 * G_delay, 1);

% Verificar estabilidad
polos_delay1 = pole(sys_cl_delay1);
estable1 = all(real(polos_delay1) < 0);
fprintf('Estabilidad: %s\n', char(estable1*'✓ ESTABLE' + ~estable1*'✗ INESTABLE'));

%% ESTRATEGIA 2: PREDICTOR DE SMITH
fprintf('\n=== ESTRATEGIA 2: PREDICTOR DE SMITH ===\n');
fprintf('El predictor de Smith es ideal para sistemas con retardo grande\n');
fprintf('Usa controlador sin modificación de ganancias\n');
fprintf('Kp = %.2f, Ki = %.2f, Kd = %.2f\n\n', Kp_zn, Ki_zn, Kd_zn);

C_smith = pid(Kp_zn, Ki_zn, Kd_zn);

%% ESTRATEGIA 3: MÉTODO IMC (Internal Model Control)
fprintf('\n=== ESTRATEGIA 3: IMC PARA SISTEMA CON RETARDO ===\n');

% Para FOPDT con retardo grande, usar IMC
lambda = τ_delay;  % Parámetro de sintonía IMC
tau_c = tau;

Kp_imc = tau_c / (K * (lambda + tau_delay));
Ki_imc = Kp_imc / tau_c;
Kd_imc = 0;  % IMC típicamente usa PI

fprintf('Parámetro λ (IMC): %.4f s\n', lambda);
fprintf('Kp_IMC = %.2f\n', Kp_imc);
fprintf('Ki_IMC = %.2f\n', Ki_imc);
fprintf('Kd_IMC = %.2f (sin derivativo)\n\n', Kd_imc);

C_imc = pid(Kp_imc, Ki_imc, Kd_imc);
sys_cl_imc = feedback(C_imc * G_delay, 1);

%% GRÁFICAS
figure('Name', 'Sistema con Retardo - Circuito RL', 'Position', [100 100 1400 900]);

% Comparación de respuestas
subplot(3,2,[1,2]);
step(sys_cl_zn * 1000, 1);
hold on;
step(sys_cl_delay1 * 1000, 1);
step(sys_cl_imc * 1000, 1);
grid on;
title('Comparación de Respuestas al Escalón');
legend('Sin retardo (ZN)', 'Con retardo (conservador)', 'Con retardo (IMC)');
ylabel('Corriente [mA]');

% Diagrama de Bode
subplot(3,2,3);
bode(G, G_delay);
grid on;
title('Bode: Sistema Original vs con Retardo');
legend('Sin retardo', 'Con retardo');

% Lugar de raíces - Conservador
subplot(3,2,4);
rlocus(C_delay1 * G_delay);
grid on;
title('Lugar Raíces - Controlador Conservador');

% Margen - Conservador
subplot(3,2,5);
margin(C_delay1 * G_delay);
grid on;
title('Margen - Controlador Conservador');

% Margen - IMC
subplot(3,2,6);
margin(C_imc * G_delay);
grid on;
title('Margen - Controlador IMC');

%% TABLA COMPARATIVA
fprintf('\n=== TABLA COMPARATIVA DE CONTROLADORES ===\n');
fprintf('%-20s | %10s | %10s | %10s\n', 'Controlador', 'Kp', 'Ki', 'Kd');
fprintf('------------------------------------------------------------\n');
fprintf('%-20s | %10.2f | %10.2f | %10.2f\n', 'ZN Original', Kp_zn, Ki_zn, Kd_zn);
fprintf('%-20s | %10.2f | %10.2f | %10.2f\n', 'Conservador', Kp_delay1, Ki_delay1, Kd_delay1);
fprintf('%-20s | %10.2f | %10.2f | %10.2f\n', 'Smith', Kp_zn, Ki_zn, Kd_zn);
fprintf('%-20s | %10.2f | %10.2f | %10.2f\n', 'IMC', Kp_imc, Ki_imc, Kd_imc);

%% GUARDAR
save('ejercicio_cinco/sistema_retardo.mat', 'tau_delay', 'G_delay', 'H_delay', ...
     'C_delay1', 'C_smith', 'C_imc', ...
     'Kp_delay1', 'Ki_delay1', 'Kd_delay1', ...
     'Kp_imc', 'Ki_imc', 'Kd_imc', ...
     'sys_cl_delay1', 'sys_cl_imc');

saveas(gcf, 'ejercicio_cinco/sistema_retardo.fig');
saveas(gcf, 'ejercicio_cinco/sistema_retardo.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 5G COMPLETADO ===\n');
