function [qi] = der2izq (qd)

qi=qd;
qi(1)=-qd(1);
qi(3)=-qd(3);
qi(5)=-qd(5);