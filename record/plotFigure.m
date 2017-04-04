clear
clc

SamplingTime = 0.1;
TaskSpaceDimension = 6;
fig_count = 1;

X_m = importdata('X_m.txt');
X_s = importdata('X_s.txt');
q_s = importdata('q_s.txt');
e_s = importdata('e_s.txt');
u_s = importdata('u_s.txt');
qDot_s = importdata('qDot_s.txt');

t = 0 : size(X_m, 1) - 1;
t = t * SamplingTime;

% 
% figure(fig_count)
% fig_count = fig_count + 1;
% plot(t,e_s)
% title('e_s')
% 
% figure(fig_count)
% fig_count = fig_count + 1;
% plot(t,q_s)
% title('q_s')
% 
% figure(fig_count)
% fig_count = fig_count + 1;
% plot(t,qDot_s)
% title('dq_s')
% 
% figure(fig_count)
% fig_count = fig_count + 1;
% plot(t,X_m)
% title('X_m')
% 
% figure(fig_count)
% fig_count = fig_count + 1;
% plot(t,X_s)
% title('X_s')
% 
% figure(fig_count)
% fig_count = fig_count + 1;
% plot(t,u_s)
% title('u_s')

for axis = 1:3
    figure(fig_count)
    fig_count = fig_count + 1;
    plot(t,e_s(:,axis),t,e_s(:,axis + TaskSpaceDimension/2))
    switch axis
        case 1
            title('e_s_y')
        case 2
            title('e_s_z')
        case 3
            title('e_s_x')
    end
end