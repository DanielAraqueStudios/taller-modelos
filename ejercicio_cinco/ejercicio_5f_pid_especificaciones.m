%% EJERCICIO 5F: PID CON ESPECIFICACIONES - CIRCUITO RL
% ts = 0.92*ts_LA, zeta=2, ess=0 para escalón
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_cinco/datos_sistema.mat');
load('ejercicio_cinco/resultados_lazo_abierto.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 5F: PID CON ESPECIFICACIONES\n');
fprintf('  Circuito RL\n');
fprintf('========================================\n\n');

%% ESPECIFICACIONES
ts_la = 4 * tau;  % Tiempo de establecimiento lazo abierto (2%)
ts_objetivo = 0.92 * ts_la;
zeta_objetivo = 2;  % Sobreamortiguado

fprintf('=== ESPECIFICACIONES ===\n');
fprintf('Tiempo de establecimiento LA: %.4f s = %.2f ms\n', ts_la, ts_la*1000);
fprintf('Tiempo de establecimiento objetivo: %.4f s = %.2f ms\n', ts_objetivo, ts_objetivo*1000);
fprintf('Factor de amortiguamiento: ζ = %.2f\n', zeta_objetivo);
fprintf('Error estado estacionario para escalón: ess = 0\n\n');

%% DISEÑO DEL CONTROLADOR
% Para ζ = 2 (muy sobreamortiguado), ajustar parámetros

Kp_spec = 2000;
Ki_spec = 300000;
Kd_spec = 3;

fprintf('=== PARÁMETROS PID CON ESPECIFICACIONES ===\n');
fprintf('Kp = %.2f\n', Kp_spec);
fprintf('Ki = %.2f\n', Ki_spec);
fprintf('Kd = %.2f\n\n', Kd_spec);

C_spec = pid(Kp_spec, Ki_spec, Kd_spec);

fprintf('Controlador PID:\n');
disp(C_spec);

%% SISTEMA EN LAZO CERRADO
sys_cl_spec = feedback(C_spec * G, 1);

fprintf('\n=== ANÁLISIS ===\n');
polos_cl = pole(sys_cl_spec);
fprintf('Polos en lazo cerrado:\n');
for i = 1:length(polos_cl)
    if imag(polos_cl(i)) == 0
        fprintf('  p%d = %.4f\n', i, real(polos_cl(i)));
    else
        fprintf('  p%d = %.4f %+.4fi\n', i, real(polos_cl(i)), imag(polos_cl(i)));
    end
end

% Verificar ess
ess_escalon = 1 - dcgain(sys_cl_spec);
fprintf('\nError estado estacionario para escalón: %.6f\n', ess_escalon);

if abs(ess_escalon) < 0.01
    fprintf('✓ Error prácticamente nulo\n');
end

%% GRÁFICAS
figure('Name', 'PID con Especificaciones - Circuito RL', 'Position', [100 100 1200 800]);

% Respuesta al escalón
subplot(2,2,1);
step(sys_cl_spec * 1000, 0.02);
grid on;
title('Respuesta al Escalón');
ylabel('Corriente [mA]');

% Comparación
subplot(2,2,2);
step(G * 1000, 0.02);
hold on;
step(sys_cl_spec * 1000, 0.02);
grid on;
title('Comparación');
legend('Lazo Abierto', 'PID con Especificaciones');
ylabel('Corriente [mA]');

% Lugar de raíces
subplot(2,2,3);
rlocus(C_spec * G);
grid on;
title('Lugar de las Raíces');

% Bode
subplot(2,2,4);
margin(C_spec * G);
grid on;
title('Diagrama de Bode');

%% GUARDAR
save('ejercicio_cinco/pid_especificaciones.mat', 'C_spec', 'sys_cl_spec', 'Kp_spec', 'Ki_spec', 'Kd_spec', 'polos_cl', 'ess_escalon');

saveas(gcf, 'ejercicio_cinco/pid_especificaciones.fig');
saveas(gcf, 'ejercicio_cinco/pid_especificaciones.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO  5F COMPLETADO ===\n');
