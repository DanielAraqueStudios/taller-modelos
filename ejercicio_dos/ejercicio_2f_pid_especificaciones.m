%% EJERCICIO 2F: PID CON ESPECIFICACIONES - SISTEMA PÉNDULO
% ts = 0.98*ts_LA, zeta=1.2, error nulo para rampa
% Autor: Daniel García Araque
% Fecha: 2026

clear all; close all; clc;

%% CARGAR DATOS
load('ejercicio_dos/datos_sistema.mat');
load('ejercicio_dos/resultados_lazo_abierto.mat');

fprintf('========================================\n');
fprintf('  EJERCICIO 2F: PID CON ESPECIFICACIONES\n');
fprintf('  Sistema Péndulo con Resortes\n');
fprintf('========================================\n\n');

%% ESPECIFICACIONES
% Sistema marginalmente estable requiere estabilización
% Tiempo de establecimiento: 98% del lazo abierto (que es infinito)
% Usaremos un ts objetivo razonable basado en la frecuencia natural

ts_objetivo = 2;  % segundos (tiempo razonable para este sistema)
zeta_objetivo = 1.2;  % Sobreamortiguado

fprintf('=== ESPECIFICACIONES ===\n');
fprintf('Tiempo de establecimiento objetivo: %.4f s\n', ts_objetivo);
fprintf('Factor de amortiguamiento: ζ = %.2f\n', zeta_objetivo);
fprintf('Tipo de sistema requerido: Tipo 2 (error nulo para rampa)\n\n');

%% DISEÑO
% Para ζ = 1.2 y ts dado
wn_deseado = 4 / (zeta_objetivo * ts_objetivo);

fprintf('Frecuencia natural deseada: ωn = %.4f rad/s\n\n', wn_deseado);

% Parámetros PID base más integrador adicional para tipo 2
Kp_spec = 120;
Ki_spec = 400;
Kd_spec = 12;
Ki2 = 50;  % Integrador doble

fprintf('=== PARÁMETROS PID CON ESPECIFICACIONES ===\n');
fprintf('Kp = %.2f\n', Kp_spec);
fprintf('Ki = %.2f\n', Ki_spec);
fprintf('Kd = %.2f\n', Kd_spec);
fprintf('Ki2 = %.2f (integrador adicional)\n\n', Ki2);

% Controlador PI²D
C_spec_num = conv([Kd_spec, Kp_spec, Ki_spec, Ki2], [1 0 0]);
C_spec_den = [1 0 0 0];
C_spec = tf(C_spec_num, C_spec_den);

fprintf('Controlador PI²D:\n');
disp(C_spec);

%% SISTEMA EN LAZO CERRADO
sys_cl_spec = feedback(C_spec * G, 1);

fprintf('\n=== ANÁLISIS DE ESTABILIDAD ===\n');
polos_cl = pole(sys_cl_spec);
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

%% VERIFICACIÓN DE ERROR EN ESTADO ESTACIONARIO
fprintf('\n=== VERIFICACIÓN DE ERROR ANTE RAMPA ===\n');

% Constante de error de velocidad
Kv = dcgain(C_spec * G * tf([1], [1 0]));
ess_rampa = 1 / Kv;

fprintf('Constante de velocidad Kv: %.4f\n', Kv);
fprintf('Error estado estacionario ante rampa: %.6f\n', ess_rampa);

if abs(ess_rampa) < 0.01
    fprintf('✓ Error prácticamente nulo\n');
end

%% GRÁFICAS
figure('Name', 'PID con Especificaciones - Péndulo', 'Position', [100 100 1200 800]);

% Respuesta al escalón
subplot(2,2,1);
step(sys_cl_spec, 10);
grid on;
title('Respuesta al Escalón');
ylabel('Salida');

% Respuesta a rampa
subplot(2,2,2);
t = 0:0.01:10;
u_ramp = t;
lsim(sys_cl_spec, u_ramp, t);
hold on;
plot(t, t, 'r--', 'LineWidth', 1.5);
grid on;
title('Respuesta a Entrada Rampa');
legend('Respuesta del sistema', 'Rampa de referencia');

% Lugar de las raíces
subplot(2,2,3);
rlocus(C_spec * G);
grid on;
title('Lugar de las Raíces');

% Diagrama de Bode
subplot(2,2,4);
margin(C_spec * G);
grid on;
title('Diagrama de Bode');

%% GUARDAR
save('ejercicio_dos/pid_especificaciones.mat', 'C_spec', 'sys_cl_spec', 'Kp_spec', 'Ki_spec', 'Kd_spec', 'Ki2', 'polos_cl', 'ess_rampa');

saveas(gcf, 'ejercicio_dos/pid_especificaciones.fig');
saveas(gcf, 'ejercicio_dos/pid_especificaciones.png');

fprintf('\n✓ Resultados guardados\n');
fprintf('\n=== EJERCICIO 2F COMPLETADO ===\n');
