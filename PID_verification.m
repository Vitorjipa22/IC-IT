%gama = 1/8;
%Temp = 3;
%num = [200 1200];
%den = [1 17 80 100];

%Kp = 0.49364697362150595;
%Ti = 0.655067793547871;
%Td = 0.07822875867824934;v

function y = PID_verification(gama, Temp, num, den, Kp, Ti, Td)
gama = gama;
Temp = Temp;
num = num;
den = den;

Kp = Kp;
Ti = Ti;
Td = Td;

sim('psopid');

y = ans.simout;
end