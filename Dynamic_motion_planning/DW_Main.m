% main algorithm for Frenet path planning
%Editor:Robert  Time:2020.8.16
clc
clear
close all
disp('Starting')
show_animation=true;
if show_animation
    figure
end
% initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
% x=[0,0,pi/8,0, 0];
x = [9,3,0,0,0];
%   goal position [x(m), y(m)]
% goal = [10,10];
goal = [0,0];
% goal = goal;
% obstacles [x(m) y(m), ....]
% ob=[-1, -1 ;
%     0,  2 ;
%     4,  2 ;
%     5,  4 ;
%     5,  5 ;
%     5,  6 ;
%     5,  9 ;
%     8,  9 ;
%     7,  9 ;
%     8,  10 ;
%     12, 13 ;
%     12, 12 ;
%     15, 15;
%     13, 13] ;
ob = [-1.5,1;-1.5,-1.2;6,-1.2;6,1];
% [-5 -1.5 -1.5 6 6 15],[1 1 -1.2 -1.2 1 1]
% ob = ob;
%  input [forward speed, yaw_rate]
par=param;
% Num = config.predict_time/config.dt + 2;
% trajectory = zeros(Num,5);
% trajectory(1,:) = x;
% mm = 1;
trajectory=[];
% trajectory=[trajectory;x];
writerObj=VideoWriter('Dynamic Window Approach.avi'); %// 定义一个视频文件用来存动
writerObj.Quality = 100;
writerObj.FrameRate = 100;
open(writerObj); %// 打开该视频文件
while true
    [u,predicted_trajectory]= dwa_control(x, par, goal, ob);
    %     pause(1.0);
    x = motion(x, u, par.dt);  % simulate robot
    %     mm = mm+1;
    %     trajectory(mm,:)=x;  % store state history
    trajectory=[trajectory;x];  % store state history
    disp(x);

    if(show_animation)
        
        % for stopping simulation with the esc key.
        plot(predicted_trajectory(:, 1), predicted_trajectory(:, 2),'LineWidth',1,'Color','cyan');
        plot(trajectory(:, 1), trajectory(:, 2),'LineWidth',1,'Color','m');
        plot(x(1), x(2), "xr");
        PlotVehicle(x(1),x(2),x(3));
        plot(goal(1), goal(2), "xb");
        plot(ob(:,1), ob(:,2), "ok");
        AX = 3;AY = 2;
        axis([trajectory(end,1)-AX trajectory(1,1)+AX trajectory(end,2)-AY trajectory(1,2)+AY])
        %         cla()
        hold off;
        frame = getframe; %// 把图像存入视频文件中
        frame.cdata = imresize(frame.cdata, [653 514]); %重新定义帧大小
        writeVideo(writerObj,frame); %// 将帧写入视频
    end
    
    % check reaching goal
    dist_to_goal = hypot(x(1) - goal(1), x(2)- goal(2));
    if dist_to_goal <= par.robot_radius/5
        print("Goal!!")
        close(writerObj); %// 关闭视频文件句柄
        break
    end
end

function [u,predicted_trajectory]=dwa_control(x, par, goal, ob)

% Dynamic Window Approach control

dw = calc_dynamic_window(x, par); % 限值约束

[u,predicted_trajectory] = calc_control_and_trajectory(x, dw, par, goal, ob);

end

function dw=calc_dynamic_window(x, par)

%     calculation dynamic window based on current state x
%      Dynamic window from robot specification

Vs = [par.min_speed, par.max_speed,-par.max_yaw_rate, par.max_yaw_rate]; % 速度大小范围、转向速率大小范围

%      Dynamic window from motion model
Vd = [x(4) - par.max_accel * par.dt,x(4) + par.max_accel * par.dt,x(5) - par.max_delta_yaw_rate * par.dt,x(5) + par.max_delta_yaw_rate * par .dt]; % 下一时刻，速度大小范围、转向速率大小范围

%       [v_min, v_max, yaw_rate_min, yaw_rate_max]
dw = [max(Vs(1), Vd(1)), min(Vs(2), Vd(2)),max(Vs(3), Vd(3)), min(Vs(4), Vd(4))]; %限值约束，取最大的最小速率，最小的最大速率

end


function [best_u,best_trajectory]=calc_control_and_trajectory(x, dw, config, goal, ob)

%     calculation final input with dynamic window

x_init = x;
min_cost = inf;
best_u = [0.0, 0.0];
best_trajectory = NaN;

%      evaluate all trajectory with sampled input in dynamic window
for v = dw(1):config.v_resolution:dw(2) % 速度规划
    for y = dw(3):config.yaw_rate_resolution:dw(4)  % 转向速率规划
        
        trajectory = predict_trajectory(x_init, v, y, config);
        plot(trajectory(:,1),trajectory(:,2),'Color','r');hold on;
        %              calc cost
        to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal); %轨迹终点航向到目标点夹角代价
        speed_cost = config.speed_cost_gain * (config.max_speed + trajectory(end, 4)); %轨迹终点速度差代价,倒车优先
        ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config); %障碍物距所有轨迹点平方和开根号，障碍物代价
        %
        final_cost = to_goal_cost + speed_cost + ob_cost; % 计算的当前轨迹代价函数
        %
        %              search minimum trajectory
        if min_cost >= final_cost
            min_cost = final_cost;
            best_u = [v, y];
            best_trajectory = trajectory; % 找到最小的轨迹代价
            plot(best_trajectory(:,1),best_trajectory(:,2),'Color','b');
        end
        
    end
end
end

function trajectory= predict_trajectory(x_init, v, y, config) % x初始状态,速度、转向速率、参数param对象

%     predict trajectory with an input
Num = config.predict_time/config.dt + 1;
trajectory = zeros(Num,5);
x = x_init;
trajectory(1,:) = x;
m = 1;
% trajectory = x;
time = 0;
while time < config.predict_time % 预测3秒
    m = m+1;
    x = motion(x, [v, y], config.dt); % 运动规划
    trajectory(m,:) = x; % 算上初始状态，共31个状态点
    %     trajectory = [trajectory ;x];
    time =time+ config.dt; % 预测30步，单步0.1秒
    
end
end

function x= motion(x, u, dt)  % u(1)速度、u(2)转向速率

%     motion model

x(3) =x(3)+ u(2) * dt; % 运动规划、航向角
x(1) =x(1)+ u(1) * cos(x(3)) * dt; % X坐标
x(2) =x(2)+ u(1) * sin(x(3)) * dt; % Y坐标
x(4) = u(1);
x(5) = u(2);

end

function cost=calc_to_goal_cost(trajectory, goal) % 计算所有轨迹终点到目标点的代价

%      calc to goal cost with angle difference

dx = -goal(1) + trajectory(end,1); % 轨迹终点到目标坐标横纵坐标差
dy = -goal(2) + trajectory(end,2);
error_angle = atan2(dy, dx); %轨迹终点到目标坐标方向角度航向
cost_angle = error_angle - trajectory(end, 3); %轨迹航向与坐标差航向角度差值
cost = abs(atan2(sin(cost_angle), cos(cost_angle))); %轨迹航向与坐标差航向夹角 绝对值（锐角）
% cost = abs(error_angle);
end

function cost= calc_obstacle_cost(trajectory, ob, config)

%         calc obstacle cost inf: collision
Num = length(trajectory(:,1));
ox = ob(:, 1);
oy = ob(:, 2);
cost=0;
% r1=[];
r1=zeros(Num,length(ox));
m =1;
for i=1:1:length(ox) % 遍历所有障碍物
    dx = trajectory(:,1) - ox(i); % 遍历Num个轨迹点
    dy = trajectory(:,2) - oy(i);
    r = hypot(dx, dy); % 平方和的平方根（斜边）
    r1(:,m)=r;
    %             r = hypot(dx, dy);
    %         r1=[r1 r];
    m = m+1;
    if config.robot_type == 0
        if any(r <= config.robot_radius/5)
            return
        end
    end
end

min_r = min(r1(:));
cost= 1.0 / min_r;  % OK

end


