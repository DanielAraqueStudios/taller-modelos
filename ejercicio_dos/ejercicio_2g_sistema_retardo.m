%% EJERCICIO 2G: SISTEMA CON RETARDO - PÉNDULO
% Retardo τ = 0.1 s
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_dos/datos_sistema.mat');
load('ejercicio_dos/pid_ziegler_nichols.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 2G: SISTEMA CON RETARDO\n');
fprintf('  Sistema Péndulo con Resortes\n');
fprintf('========================================\n\n');

%% PARÁMETROS DE RETARDO
tau = 0.1;  % Retardo en segundos

fprintf('Retardo del sistema: τ = %.2f s\n\n', tau);

%% APROXIMACIÓN DE PADÉ PARA EL RETARDO
% Aproximación de primer orden de Padé: e^(-τs) ≈ (1 - τs/2)/(1 + τs/2)
pade_num = [1, -tau/2];
pade_den = [1, tau/2];
H_delay = tf(pade_num, pade_den);

fprintf('Aproximación de Padé (1er orden):\n');
disp(H_delay);

% Sistema con retardo
G_delay = G * H_delay;

fprintf('\nSistema con retardo G_delay(s):\n');
disp(G_delay);

%% ESTRATEGIA 1: REDUCCIÓN CONSERVADORA DE GANANCIAS
fprintf('\n=== ESTRATEGIA 1: REDUCCIÓN CONSERVADORA ===\n');

% Reducir ganancias proporcionalmente al retardo
factor_p = 0.8;
factor_i = 0.7;
factor_d = 0.9;

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
fprintf('El predictor de Smith compensa el retardo en el lazo de realimentación\n');
fprintf('Controlador: usa parámetros ZN originales\n');
fprintf('Kp = %.2f, Ki = %.2f, Kd = %.2f\n\n', Kp_zn, Ki_zn, Kd_zn);

% Implementación simplificada del predictor de Smith
C_smith = pid(Kp_zn, Ki_zn, Kd_zn);

%% ESTRATEGIA 3: MÉTODO CHR (Chien-Hrones-Reswick)
fprintf('\n=== ESTRATEGIA 3: MÉTODO CHR (RE-SINTONIZACIÓN) ===\n');

% Para sistema con retardo, usar método CHR con sobrepaso 0%
% Basado en modelo FOPDT aproximado

% Parámetros aproximados FOPDT
K_process = dcgain(G);
fprintf('Ganancia del proceso: K = %.6f\n', K_process);

% Re-sintonizar con CHR para 0% sobrepaso
Kp_chr = 0.6 * Kp_zn;
Ki_chr = Kp_chr / (4 * tau);
Kd_chr = 0.5 * Kp_zn * tau;

fprintf('Kp_CHR = %.2f\n', Kp_chr);
fprintf('Ki_CHR = %.2f\n', Ki_chr);
fprintf('Kd_CHR = %.2f\n\n', Kd_chr);

C_chr = pid(Kp_chr, Ki_chr, Kd_chr);
sys_cl_chr = feedback(C_chr * G_delay, 1);

%% GRÁFICAS COMPARATIVAS
figure('Name', 'Sistema con Retardo - Péndulo', 'Position', [100 100 1200 900]);

% Respuesta al escalón - Comparación
subplot(3,2,[1,2]);
step(sys_cl_zn, 5);
hold on;
step(sys_cl_delay1, 5);
step(sys_cl_chr, 5);
grid on;
title('Comparación de Respuestas al Escalón');
legend('Sin retardo (ZN)', 'Con retardo (conservador)', 'Con retardo (CHR)');
ylabel('Salida');

% Diagrama de Bode - Sistema con retardo
subplot(3,2,3);
bode(G, G_delay);
grid on;
title('Bode: Sistema Original vs con Retardo');
legend('Sin retardo', 'Con retardo');

% Lugar de raíces - Con controlador conservador
subplot(3,2,4);
rlocus(C_delay1 * G_delay);
grid on;
title('Lugar de Raíces - Controlador Conservador');

% Diagrama de Bode - Lazo abierto con controladores
subplot(3,2,5);
margin(C_delay1 * G_delay);
grid on;
title('Margen - Controlador Conservador');

% Diagrama de Bode - CHR
subplot(3,2,6);
margin(C_chr * G_delay);
grid on;
title('Margen - Controlador CHR');

%% TABLA COMPARATIVA
fprintf('\n=== TABLA COMPARATIVA DE CONTROLADORES ===\n');
fprintf('%-20s | %8s | %8s | %8s\n', 'Controlador', 'Kp', 'Ki', 'Kd');
fprintf('----------------------------------------------------------\n');
fprintf('%-20s | %8.2f | %8.2f | %8.2f\n', 'ZN Original', Kp_zn, Ki_zn, Kd_zn);
fprintf('%-20s | %8.2f | %8.2f | %8.2f\n', 'Conservador', Kp_delay1, Ki_delay1, Kd_delay1);
fprintf('%-20s | %8.2f | %8.2f | %8.2f\n', 'Smith', Kp_zn, Ki_zn, Kd_zn);
fprintf('%-20s | %8.2f | %8.2f | %8.2f\n', 'CHR', Kp_chr, Ki_chr, Kd_chr);

%% GUARDAR
save('ejercicio_dos/sistema_retardo.mat', 'tau', 'G_delay', 'H_delay', ...
     'C_delay1', 'C_smith', 'C_chr', ...
     'Kp_delay1', 'Ki_delay1', 'Kd_delay1', ...
     'Kp_chr', 'Ki_chr', 'Kd_chr', ...
     'sys_cl_delay1', 'sys_cl_chr');

saveas(gcf, 'ejercicio_dos/sistema_retardo.fig');
saveas(gcf, 'ejercicio_dos/sistema_retardo.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 2G COMPLETADO ===\n');
