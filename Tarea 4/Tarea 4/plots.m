
close all;

t=0:1/10000:(25000-1)/10000;
load("Imr.dat"); figure; plot(t, Imr(:,6)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("angular speed [rad/s]");
load("Imr.dat"); figure; plot(t, Imr(:,7)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("Torque [Nm]");
load("Imr.dat"); figure; plot(t, Imr(:,11)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("Torque Referencia [Nm]");
load("Imr.dat"); figure; plot(t, Imr(:,10)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("N");
load("Imr.dat"); figure; plot(t, Imr(:,12)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("flux");
load("Imr.dat"); figure; plot(Imr(:,3), Imr(:,4)); title("Resultados - Máquina Kraus 3hp a "); xlabel("Rotor Magnetic Flow [Wb]"); ylabel("Rotor Magnetic Flow [Wb]");
load("Imr.dat"); figure; plot(t, Imr(:,1)-Imr(:,2)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("error de corriente de fase [A]");
%load("Imr.dat"); figure; plot(t, Imr(:,1)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("corriente Isa [A]");





