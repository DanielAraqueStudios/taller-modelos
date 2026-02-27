%% EJERCICIO 3D: TIEMPO DE ESTABLECIMIENTO - SISTEMA MASA-CILINDRO
clear all; close all; clc;
load('ejercicio_tres/datos_sistema.mat');
fprintf('EJERCICIO 3D: Sistema Masa-Cilindro\n\n');
[y, t] = step(G, 10);
yss = dcgain(G);
info_step = stepinfo(G);
ts_lazo_abierto_2 = info_step.SettlingTime;
ts_lazo_abierto_5 = 3/abs(real(pole(G)));
figure; step(G); grid on; title('Respuesta al Escalón - Lazo Abierto');
save('ejercicio_tres/resultados_lazo_abierto.mat', 'ts_lazo_abierto_2', 'ts_lazo_abierto_5', 'yss', 'info_step', 't', 'y');
saveas(gcf, 'ejercicio_tres/respuesta_lazo_abierto.png');
fprintf('ts(2%%) = %.4f s\n', ts_lazo_abierto_2);
fprintf('Resultados guardados\n');
