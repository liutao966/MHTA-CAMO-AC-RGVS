clear;
clc;
% % 初始化三维坐标系
% 初始化默认海拔为0
map_x=0:0.1:100;
map_y=0:0.1:100;
map_z=0:0.1:100;
% 生成二维网格矩阵
[X,Y]=meshgrid(map_x,map_y);

% % 绘制小山
mountain_x=40;
mountain_y=60;
mountain_A_max=40;

mountain_x2=60;
mountain_y2=20;
mountain_A_max2=40;
% 设置小山宽度
sigma=40;
Z_guass=mountain_A_max*exp(-((X-mountain_x).^2+(Y-mountain_y).^2)/(2*sigma*sigma));
Z_guass2=mountain_A_max2*exp(-((X-mountain_x2).^2+(Y-mountain_y2).^2)/(2*sigma*sigma));
noise=randn(size(X));
Z_mountain=noise+Z_guass;
Z_mountain2=noise+Z_guass2;
Z_mountain(Z_mountain<0)=0;
Z_mountain2(Z_mountain<0)=0;

figure("Position",[100,100,800,800]);
% 'FaceColor'：固定关键字，意思是「三维表面的填充颜色模式」；
% 'EdgeColor'：固定关键字，意思是「三维表面网格边线的颜色」；
% 'Alpha', 0.9 —— 控制表面透明度（增强立体感，避免遮挡）
surf(X,Y,Z_mountain,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);
hold on;
surf(X,Y,Z_mountain2,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);
xlabel("x/米");
ylabel("y/米");
zlabel("高度/米");
zlim([0,100]);
title("障碍物地图");
grid on ;
axis square;
% 平滑着色，让障碍物更加清晰
shading interp;
hold on;

% % 绘制圆柱体障碍物
% 圆柱体半径
cylinder_r=5;
cylinder_r9=10;
cylinder_r10=7;
% 圆柱体侧面分段数
n=50;
[X_cylinder,Y_cylinder,Z_cylinder]=cylinder(cylinder_r,n);
number=11;
cylinder_position=cell(1,number);
cylinder_position{1}=[20,20,80];
cylinder_position{2}=[60,40,60];
cylinder_position{3}=[40,60,80];
cylinder_position{4}=[80,80,70];
cylinder_position{5}=[20,80,60];
cylinder_position{6}=[40,80,70];
cylinder_position{7}=[80,20,80];
cylinder_position{8}=[80,40,80];
% 
cylinder_position{9}=[65,65,80];
% 
cylinder_position{10}=[25,50,75];
cylinder_position{11}=[50,20,60];
for i=1:1:length(cylinder_position)
    if(i==9)
        cylinder_r=cylinder_r9;
    elseif(i==10)
        cylinder_r=cylinder_r10;
    else
        cylinder_r=5;
    end
    cylinder_x=cylinder_position{i}(1)+X_cylinder;
    cylinder_y=cylinder_position{i}(2)+Y_cylinder;
    cylinder_z=cylinder_position{i}(3)*Z_cylinder;
    surf(cylinder_x,cylinder_y,cylinder_z,"FaceColor",[1,0,0],"EdgeColor","none","FaceAlpha",1.0)
end

hold on;

% % 绘制动态障碍物
% 第一个
ball_r=2;
cx1=70;cy1=50;cz1=60;
n_ball=50;
[X_ball,Y_ball,Z_ball]=sphere(n_ball);
ball_x1=cx1+X_ball*ball_r;
ball_y1=cy1+Y_ball*ball_r;
ball_z1=cz1+Z_ball*ball_r;
h1=surf(ball_x1,ball_y1,ball_z1,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
hold on;
cball_x1=65;cball_y1=65;cball_z1=60;
r1=sqrt(abs(cball_x1-cx1)^2+abs(cball_y1-cy1)^2);
num_step=20000;
speed=1/(4*pi);
thate01=atan((cy1-cball_y1)/(cx1-cball_x1));
% 第二个
cx2=50;cy2=40;cz2=55;
n_ball=50;
[X_ball2,Y_ball2,Z_ball2]=sphere(n_ball);
ball_x2=cx2+X_ball2*ball_r;
ball_y2=cy2+Y_ball2*ball_r;
ball_z2=cz2+Z_ball2*ball_r;
h2=surf(ball_x2,ball_y2,ball_z2,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
hold on;
cball_x2=40;cball_y2=40;cball_z2=55;
r2=sqrt(abs(cball_x2-cx2)^2+abs(cball_y2-cy2)^2);
thate02=atan((cy2-cball_y2)/(cx2-cball_x2));

% 直线循环
start_point1=[10,60,40];
end_point1=[10,10,60];
speed_line=0.8;
distance_dir1=end_point1-start_point1;
% 计算距离
distance1=norm(distance_dir1);
% 单位化方向向量
unit_dir=distance_dir1/distance1;
current_dir=unit_dir;
min_distance=1;
[X_line1,Y_line1,Z_line1]=sphere(n_ball);
x_line1=start_point1(1)+X_line1*ball_r;
y_line1=start_point1(2)+Y_line1*ball_r;
Z_line1=start_point1(3)+Z_line1*ball_r;
h3=surf(x_line1,y_line1,Z_line1,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
current_pose=start_point1;

% 直线循环
start_point2=[10,10,40];
end_point2=[90,10,80];
distance_dir2=end_point2-start_point2;
% 计算距离
distance2=norm(distance_dir2);
% 单位化方向向量
unit_dir2=distance_dir2/distance2;
current_dir2=unit_dir2;
% min_distance=1;
[X_line2,Y_line2,Z_line2]=sphere(n_ball);
x_line2=start_point2(1)+X_line2*ball_r;
y_line2=start_point2(2)+Y_line2*ball_r;
Z_line2=start_point2(3)+Z_line2*ball_r;
h4=surf(x_line2,y_line2,Z_line2,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
current_pose2=start_point2;

% 直线循环
start_point3=[10,80,40];
end_point3=[70,10,70];
% speed_line=0.8;
distance_dir3=end_point3-start_point3;
% 计算距离
distance3=norm(distance_dir3);
% 单位化方向向量
unit_dir3=distance_dir3/distance3;
current_dir3=unit_dir3;
% min_distance=1;
[X_line3,Y_line3,Z_line3]=sphere(n_ball);
x_line3=start_point3(1)+X_line3*ball_r;
y_line3=start_point3(2)+Y_line3*ball_r;
Z_line3=start_point3(3)+Z_line3*ball_r;
h5=surf(x_line3,y_line3,Z_line3,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
current_pose3=start_point3;
for i=1:1:num_step
    % % 
    % 更新后的球心
    thate1=i*speed+thate01;
    update_x1=r1*sin(thate1)+cball_x1;
    update_y1=r1*cos(thate1)+cball_y1;
    update_z1=cball_z1;
    % 更新后的球矩阵
    [X_update,Y_update,Z_update]=sphere(n_ball);
    new_x1=update_x1+X_update*ball_r;
    new_y1=update_y1+Y_update*ball_r;
    new_z1=update_z1+Z_update*ball_r;
    % 删除上一帧
    delete(h1);
    h1=surf(new_x1,new_y1,new_z1,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
    % % 

    % % 
    % 更新后的球心
    thate2=i*speed+thate02;
    update_x2=r2*sin(thate2)+cball_x2;
    update_y2=r2*cos(thate2)+cball_y2;
    update_z2=cball_z2;
    % 更新后的球矩阵
    [X_update2,Y_update2,Z_update2]=sphere(n_ball);
    new_x2=update_x2+X_update2*ball_r;
    new_y2=update_y2+Y_update2*ball_r;
    new_z2=update_z2+Z_update2*ball_r;
    % 删除上一帧
    delete(h2);
    h2=surf(new_x2,new_y2,new_z2,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
    % % 

    % % 
    current_pose=current_pose+speed_line*current_dir;
    distance_to_end=norm(end_point1-current_pose);
    distance_to_start=norm(current_pose-start_point1);
    if(distance_to_start<min_distance)
        current_dir=unit_dir;
    elseif(distance_to_end<min_distance)
        current_dir=-unit_dir;
    end
    [X_update_line1,Y_update_line1,Z_update_line1]=sphere(n_ball);
    update_line_x1=X_update_line1*ball_r+current_pose(1);
    update_line_y1=Y_update_line1*ball_r+current_pose(2);
    update_line_z1=Z_update_line1*ball_r+current_pose(3);
    delete(h3);
    h3=surf(update_line_x1,update_line_y1,update_line_z1,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
    % % 

    % % 
    current_pose2=current_pose2+speed_line*current_dir2;
    distance_to_end2=norm(end_point2-current_pose2);
    distance_to_start2=norm(current_pose2-start_point2);
    if(distance_to_start2<min_distance)
        current_dir2=unit_dir2;
    elseif(distance_to_end2<min_distance)
        current_dir2=-unit_dir2;
    end
    [X_update_line2,Y_update_line2,Z_update_line2]=sphere(n_ball);
    update_line_x2=X_update_line2*ball_r+current_pose2(1);
    update_line_y2=Y_update_line2*ball_r+current_pose2(2);
    update_line_z2=Z_update_line2*ball_r+current_pose2(3);
    delete(h4);
    h4=surf(update_line_x2,update_line_y2,update_line_z2,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
    % % 

    % % 
    current_pose3=current_pose3+speed_line*current_dir3;
    distance_to_end3=norm(end_point3-current_pose3);
    distance_to_start3=norm(current_pose3-start_point3);
    if(distance_to_start3<min_distance)
        current_dir3=unit_dir3;
    elseif(distance_to_end3<min_distance)
        current_dir3=-unit_dir3;
    end
    [X_update_line3,Y_update_line3,Z_update_line3]=sphere(n_ball);
    update_line_x3=X_update_line3*ball_r+current_pose3(1);
    update_line_y3=Y_update_line3*ball_r+current_pose3(2);
    update_line_z3=Z_update_line3*ball_r+current_pose3(3);
    delete(h5);
    h5=surf(update_line_x3,update_line_y3,update_line_z3,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
    % % 


    % 强制刷新页面，避免卡顿=
    drawnow;
    % 控制频率每秒20次
    pause(0.05)
end
hold on;