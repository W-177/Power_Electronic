
close all;

t=0:1/10000:(15000-1)/10000';
load("vflr1.dat"); figure; plot(t, vflr1(:,5)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("angular speed [rad/s]");
load("vflr1.dat"); figure; plot(t, vflr1(:,6)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("Torque [Nm]");
load("vflr1.dat"); figure; plot(vflr1(:,3), vflr1(:,4)); title("Resultados - Máquina Kraus 3hp a "); xlabel("Rotor Magnetic Flow [Wb]"); ylabel("Rotor Magnetic Flow [Wb]");
load("vflr1.dat"); figure; plot(t, vflr1(:,1)-vflr1(:,2)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("error de corriente de fase [A]");
load("vflr1.dat"); figure; plot(t, vflr1(:,1)); title("Resultados - Máquina Kraus 3hp a "); xlabel("time [s]"); ylabel("corriente Isa [A]");





