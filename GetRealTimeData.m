

%gama = 1/8;
%Temp = 2.625;
%num = 16;
%den = [1 4 16];
%Kp = 32025.71745690574;
%Ti = 33.076575618016115;
%Td = 0.001;

sim('psopid');

a = ans.simout;

getout(a);

function out = getout(a)
out = a

end
