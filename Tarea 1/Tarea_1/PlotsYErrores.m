


load("DatosEU.dat"); figure; plot(DatosEU(:,1), DatosEU(:,2)); title("Resultados - Método: Euler")

load("DatosRK2.dat"); figure; plot(DatosRK2(:,1), DatosRK2(:,2)); title("Resultados - Método: RK2")

load("DatosRK4.dat"); figure; plot(DatosRK4(:,1), DatosRK4(:,2)); title("Resultados - Método: RK4")

load("DatosRK5.dat"); figure; plot(DatosRK5(:,1), DatosRK5(:,2)); title("Resultados - Método: RK5")

load("DatosSRK4.dat"); figure; plot(DatosSRK4(:,1), DatosSRK4(:,2)); title("Resultados - Método: SRK4")

%%load("D:\Postgrado\EC7136 - Electrónica de Potencia II\Tareas\Tarea 1\Tarea_1\DatosRKF4.dat"); figure; plot(DatosRKF4(:,1), DatosRKF4(:,2)); title("Resultados - Método: RKF4")

%%load("D:\Postgrado\EC7136 - Electrónica de Potencia II\Tareas\Tarea 1\Tarea_1\DatosRKF5.dat"); figure; plot(DatosRKF5(:,1), DatosRKF5(:,2)); title("Resultados - Método: RKF5")

%% Errores
disp('error Euler')
errRMS_EU = sqrt(sum((out.simout.Data(:)-DatosEU(:,2)).^2))./length(out.tout(:))

disp('error RK2')
errRMS_RK2 = sqrt(sum((out.simout.Data(:)-DatosRK2(:,2)).^2))./length(out.tout(:))

disp('error RK4')
errRMS_RK4 = sqrt(sum((out.simout.Data(:)-DatosRK4(:,2)).^2))./length(out.tout(:))

disp('error RK5')
errRMS_RK5 = sqrt(sum((out.simout.Data(:)-DatosRK5(:,2)).^2))./length(out.tout(:))

disp('error SRK4')
errRMS_SRK4 = sqrt(sum((out.simout.Data(:)-DatosSRK4(:,2)).^2))./length(out.tout(:))

%%errRMS_RKF4 = sqrt(sum((out.simout.Data(:)-DatosRKF4(:,2)).^2))./length(out.tout(:))

%%errRMS_RKF5 = sqrt(sum((out.simout.Data(:)-DatosRKF5(:,2)).^2))./length(out.tout(:))

figure;
hold on

plot(DatosEU(:,1), out.simout.Data(:))
plot(DatosEU(:,1), DatosEU(:,2))
plot(DatosEU(:,1), DatosRK2(:,2))
plot(DatosEU(:,1), DatosRK4(:,2))
plot(DatosEU(:,1), DatosRK5(:,2))
plot(DatosEU(:,1), DatosSRK4(:,2))
%%plot(DatosEU(:,1), DatosRKF4(:,2))
%%plot(DatosEU(:,1), DatosRKF5(:,2))

hold off

legend({'Simulink', 'EU', 'RK2', 'RK4', 'RK5', 'SRK4'});
%%legend({'Simulink', 'EU', 'RK2', 'RK4', 'RK5', 'SRK4', 'RKF4', 'RKF5'});
title("todos los resultados en comparación con Simulink")





