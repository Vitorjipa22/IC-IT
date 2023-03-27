%gama = 1/8;
%Temp = 10;
%num = 16;
%den = [1 4 16];
%Kp = 100000;
%Ti = 2.10313104378828;
%Td = 0.0012020136813605598;

% Temp = 2;
% Ti = 2;
% Td = 0.05;
% Kp = 10.0;
% gama = 1/8;
% num = 16;
% den = [1 4 16];
sim('psopid');

out = ans.simout;

% out(:,2)
% out(end,2)
