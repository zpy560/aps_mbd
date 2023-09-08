% 以 Frenet 坐标系转换做路径规划；
clc
clear
close all

%% 以起始点（0,0,0）到（9,3,0）泊出；
% 先考虑以最小转弯半径起始参考线为直线-圆弧-直线，选取计算生成的Num点
w_x = [0,0.1498,0.7030,1.2491,1.7827,2.2985,2.7015,3.7015,4.7015,5.7015,6.7015,6.8387,7.3599,7.8974,8.4459,9];
w_y = [0,0.0020,0.0446,0.1423,0.2940,0.4983,0.6816,1.1366,1.5916,2.0466,2.5016,2.5618,2.7521,2.8893,2.9722,3];
%% 设置车位角点障碍物坐标点
ob = [6,1.0];
ds=0.1;    %discrete step size
GenerateTargetCourse = @(wx, wy) calcSplineCourse(wx, wy, ds); % @：函数句柄，即是一种变量，也可用于传参和赋值；跟函数按相同规则使用
[RefX, RefY, RefYaw, RefCurvature, runningLength, referencePath]=...
    GenerateTargetCourse(w_x, w_y); % 根据离散点计算Frenet坐标，再由Frenet坐标计算cartisen坐标下轨迹坐标及曲率航向值

% Initial state
s0_d= 0 ; %0.1 / 3.6;          % current speed [m/s]
d0 = 0 ; %-0.01;                      % current lateral position [m]
d0_d=0;                       % current lateral speed [m/s]
d0_dd = 0;                    % current lateral acceleration [m/s^2]
s0= 0;                          % current course position[m]

area=20;                      % animation area length[m]

objFrenetPlanner = OptimalFrenetPlanner();
show_animation=true;
writerObj=VideoWriter('parking_out_Slot_7meters.avi'); %// 定义一个视频文件用来存动画
open(writerObj); %// 打开该视频文件
%start simulation
T=59;
for t = 1:T
    frenetTrajectories =objFrenetPlanner.CalcFrenetTrajectories(s0,s0_d, d0, d0_d, d0_dd);
    globalTrajectories = objFrenetPlanner.CalcGlobalTrajectories(frenetTrajectories,referencePath);
    okTrajectories = objFrenetPlanner.CheckTrajectories(globalTrajectories, ob);
    
    trajectory = objFrenetPlanner.FrenetOptimalPlanning (...
        referencePath, s0, s0_d, d0, d0_d, d0_dd, ob);
    
    %store the updated state of the planned trajectorut as initial
    %state of next iteration for the new trajectory
    s0 = trajectory.s(2);
    s0_d= trajectory.ds(2);
    d0 = trajectory.d(2);
    d0_d = trajectory.dd(2);
    d0_dd=trajectory.ddd(2);
    
    if(show_animation)
        cla;
        plot(RefX,RefY,'LineWidth',2,'Color','m');
        hold on
        PlotForward(trajectory.x(1),trajectory.y(1),trajectory.theta(1));
        PlotForward(0,0,0);
        axis ([-2 15 -2 6]);
        plot(ob(:,1),ob(:,2),'*k');
        plot([-5 -1.5 -1.5 6 6 15],[1 1 -1.2 -1.2 1 1],'LineWidth',2,'Color','k');
        for nn = 1:length(okTrajectories)
            frenet = okTrajectories{nn};
            plot(frenet.x, frenet.y, 'g');
        end
        plot(trajectory.x(1:end),trajectory.y(1:end), '-ob');
        grid on
        if(sqrt((trajectory.x(1)-RefX(end))^2+(trajectory.y(1)-RefY(end))^2)<0.4)
            close(writerObj); %// 关闭视频文件句柄
            PlotForward(trajectory.x(1),trajectory.y(1),0);
            disp(t);
            break
        end
        frame = getframe; %// 把图像存入视频文件中
        frame.cdata = imresize(frame.cdata, [653 514]); %重新定义帧大小
        writeVideo(writerObj,frame); %// 将帧写入视频
    end
end