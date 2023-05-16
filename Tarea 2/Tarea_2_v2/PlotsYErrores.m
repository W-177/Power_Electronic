


load("DatosEU.dat"); figure; hold on; plot(DatosEU(:,1), DatosEU(:,3)); plot(DatosEU(:,1), DatosEU(:,4)); hold off; title("Resultados - Método: Euler"); legend({'I','w'});

load("DatosRK2.dat"); figure; hold on; plot(DatosRK2(:,1), DatosRK2(:,3)); plot(DatosRK2(:,1), DatosRK2(:,4)); hold off; title("Resultados - Método: RK2"); legend({'I','w'});

load("DatosRK4.dat"); figure; hold on; plot(DatosRK4(:,1), DatosRK4(:,3)); plot(DatosRK4(:,1), DatosRK4(:,4)); hold off; title("Resultados - Método: RK4"); legend({'I','w'});

load("DatosRK5.dat"); figure; hold on; plot(DatosRK5(:,1), DatosRK5(:,3)); plot(DatosRK5(:,1), DatosRK5(:,4)); hold off; title("Resultados - Método: RK5"); legend({'I','w'});

load("DatosSRK4.dat"); figure; hold on; plot(DatosSRK4(:,1), DatosSRK4(:,3)); plot(DatosSRK4(:,1), DatosSRK4(:,4)); hold off; title("Resultados - Método: SRK4"); legend({'I','w'});

load("DatosSRK5.dat"); figure; hold on; plot(DatosSRK5(:,1), DatosSRK5(:,3)); plot(DatosSRK5(:,1), DatosSRK5(:,4)); hold off; title("Resultados - Método: SRK5"); legend({'I','w'});

load("DatosRKF4.dat"); figure; hold on; plot(DatosRKF4(:,1), DatosRKF4(:,3)); plot(DatosRKF4(:,1), DatosRKF4(:,4)); hold off; title("Resultados - Método: RKF4"); legend({'I','w'});

load("DatosRKF5.dat"); figure; hold on; plot(DatosRKF5(:,1), DatosRKF5(:,3)); plot(DatosRKF5(:,1), DatosRKF5(:,4)); hold off; title("Resultados - Método: RKF5"); legend({'I','w'});

load("DatosRK6.dat"); figure; hold on; plot(DatosRK6(:,1), DatosRK6(:,3)); plot(DatosRK6(:,1), DatosRK6(:,4)); hold off; title("Resultados - Método: RK6"); legend({'I','w'});

load("DatosRKF7.dat"); figure; hold on; plot(DatosRKF7(:,1), DatosRKF7(:,3)); plot(DatosRKF7(:,1), DatosRKF7(:,4)); hold off; title("Resultados - Método: RKF7"); legend({'I','w'});

load("DatosHIRK6.dat"); figure; hold on; plot(DatosHIRK6(:,1), DatosHIRK6(:,3)); plot(DatosHIRK6(:,1), DatosHIRK6(:,4)); hold off; title("Resultados - Método: HIRK6"); legend({'I','w'});

figure;
hold on

plot(DatosEU(:,1), DatosEU(:,4))
plot(DatosEU(:,1), DatosRK2(:,4))
plot(DatosEU(:,1), DatosRK4(:,4))
plot(DatosEU(:,1), DatosRK5(:,4))
plot(DatosEU(:,1), DatosSRK4(:,4))
plot(DatosEU(:,1), DatosSRK5(:,4))
plot(DatosEU(:,1), DatosRKF4(:,4))
plot(DatosEU(:,1), DatosRKF5(:,4))
plot(DatosEU(:,1), DatosRK6(:,4))
plot(DatosEU(:,1), DatosRKF7(:,4))
plot(DatosEU(:,1), DatosHIRK6(:,4))

hold off

legend({'Simulink', 'EU', 'RK2', 'RK4', 'RK5', 'SRK4', 'SRK5', 'RKF4', 'RKF5', 'RK6', 'RKF7', 'HIRK6'});
title("todos los resultados de velocidad angular en comparación")


figure;
hold on

plot(DatosEU(:,1), DatosEU(:,3))
plot(DatosEU(:,1), DatosRK2(:,3))
plot(DatosEU(:,1), DatosRK4(:,3))
plot(DatosEU(:,1), DatosRK5(:,3))
plot(DatosEU(:,1), DatosSRK4(:,3))
plot(DatosEU(:,1), DatosSRK5(:,3))
plot(DatosEU(:,1), DatosRKF4(:,3))
plot(DatosEU(:,1), DatosRKF5(:,3))
plot(DatosEU(:,1), DatosRK6(:,3))
plot(DatosEU(:,1), DatosRKF7(:,3))
plot(DatosEU(:,1), DatosHIRK6(:,3))

hold off

legend({'Simulink', 'EU', 'RK2', 'RK4', 'RK5', 'SRK4', 'SRK5', 'RKF4', 'RKF5', 'RK6', 'RKF7', 'HIRK6'});
title("todos los resultados de corriente en comparación")



