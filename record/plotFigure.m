clear
clc

SamplingTime = 0.1;
TaskSpaceDimension = 6;
fig_count = 1;
DOF = 3;
StartTime = 20;
EndTime = 3700;

X_m = importdata('X_m.txt');
X_s = importdata('X_s.txt');
e_s = importdata('e_s.txt');
u_s = importdata('u_s.txt');
q_s = importdata('q_s.txt');
qDot_s = importdata('qDot_s.txt');

t = 0 : size(X_m, 1) - 1;
t = t * SamplingTime;

for axis = 1:3
    figure(fig_count)
    fig_count = fig_count + 1;
    subplot(3,1,1),plot(t(StartTime:EndTime),e_s(StartTime:EndTime,axis),t(StartTime:EndTime),e_s(StartTime:EndTime,axis + TaskSpaceDimension/2),'--')
    switch axis
        case 1
            title('e_s_y')
        case 2
            title('e_s_z')
        case 3
            title('e_s_x')
    end
    
    subplot(3,1,2),plot(t(StartTime:EndTime),X_m(StartTime:EndTime,axis),t(StartTime:EndTime),X_m(StartTime:EndTime,axis + TaskSpaceDimension/2),'--')
    switch axis
        case 1
            title('X_m_y')
        case 2
            title('X_m_z')
        case 3
            title('X_m_x')
    end
    
    subplot(3,1,3),plot(t(StartTime:EndTime),X_s(StartTime:EndTime,axis),t(StartTime:EndTime),X_s(StartTime:EndTime,axis + TaskSpaceDimension/2),'--')
    switch axis
        case 1
            title('X_s_y')
        case 2
            title('X_s_z')
        case 3
            title('X_s_x')
    end
end

for axis = 1:3
    figure(fig_count)
    fig_count = fig_count + 1;
    subplot(3,1,1),plot(t(StartTime:EndTime),u_s(StartTime:EndTime,axis),t(StartTime:EndTime),u_s(StartTime:EndTime,axis + DOF*1),'--',t(StartTime:EndTime),u_s(StartTime:EndTime,axis + DOF*2),t(StartTime:EndTime),u_s(StartTime:EndTime,axis + DOF*3),'-.')
    switch axis
        case 1
            title('u_s_y')
        case 2
            title('u_s_z')
        case 3
            title('u_s_x')
    end
    
    subplot(3,1,2),plot(t(StartTime:EndTime),q_s(StartTime:EndTime,axis),t(StartTime:EndTime),q_s(StartTime:EndTime,axis + DOF*1),'--',t(StartTime:EndTime),q_s(StartTime:EndTime,axis + DOF*2),t(StartTime:EndTime),q_s(StartTime:EndTime,axis + DOF*3),'-.')
    switch axis
        case 1
            title('q_s_y')
        case 2
            title('q_s_z')
        case 3
            title('q_s_x')
    end
    
    subplot(3,1,3),plot(t(StartTime:EndTime),qDot_s(StartTime:EndTime,axis),t(StartTime:EndTime),qDot_s(StartTime:EndTime,axis + DOF*1),'--',t(StartTime:EndTime),qDot_s(StartTime:EndTime,axis + DOF*2),t(StartTime:EndTime),qDot_s(StartTime:EndTime,axis + DOF*3),'-.')
    switch axis
        case 1
            title('dq_s_y')
        case 2
            title('dq_s_z')
        case 3
            title('dq_s_x')
    end
end

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

