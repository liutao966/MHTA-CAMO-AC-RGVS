clear;
clc;
% ===================== 字体大小一键调节区（直接修改数字即可）=====================
tick_font_size  = 20;   % 坐标轴刻度数字（0/50/100/150/200）的字号
label_font_size = 20;   % 坐标轴名称（X/m、Y/m、Z/m）的字号
title_font_size = 16;   % 图标题（障碍物地图）的字号
% =================================================================================
% 初始化三维坐标系
map_x=0:0.1:200;
map_y=0:0.1:200;
map_z=0:0.1:200;
% 生成二维网格矩阵
[X,Y]=meshgrid(map_x,map_y);
% 山的参数
mountain_A_max=90;
mountain_x=40;
mountain_y=140;
% 宽度
sigma=30;
z_guass=mountain_A_max*exp(-((X-mountain_x).^2+(Y-mountain_y).^2)/(2*sigma*sigma));
noise=randn(size(X));
z_mountain=noise+z_guass;
z_mountain(z_mountain<0)=0;
% 创建画布
figure("Position",[100,100,800,800],"Color","white");
surf(X,Y,z_mountain,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);
hold on;

% 第二座山体
mountain_x2=140;
mountain_y2=40;
mountain_A_max2=90;
sigma2=30;
z_guass2=mountain_A_max2*exp(-((X-mountain_x2).^2+(Y-mountain_y2).^2)/(2*sigma2*sigma2));
z_mountain2=noise+z_guass2;
z_mountain2(z_mountain2<0)=0;
surf(X,Y,z_mountain2,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);

% 第三座山体
mountain_x3=140;
mountain_y3=140;
sigma3=30;
mountain_A3=120;
z_guass3=mountain_A3*exp(-((X-mountain_x3).^2+(Y-mountain_y3).^2)/(2*sigma3*sigma3));
z_mountain3=z_guass3+noise;
z_mountain3(z_mountain3<0)=0;
surf(X,Y,z_mountain3,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);

% 禁飞区
r=50;
cx1=100;
cy1=100;
cz1=0;
n_ball=50;
[X_ball,Y_ball,Z_ball]=sphere(n_ball);
ball_x1=cx1+X_ball*r;
ball_y1=cy1+Y_ball*r;
ball_z1=cz1+Z_ball*r;
ball_z1(ball_z1<0)=0;
surf(ball_x1,ball_y1,ball_z1,"FaceColor",[1,0,1],"EdgeColor","none","FaceAlpha",1.0);

% 绘制圆柱体障碍物（带封顶）
cylinder_r=5;
n_cylinder=50;
[X_cylinder,Y_cylinder,Z_cylinder]=cylinder(cylinder_r,n_cylinder);
cylinder_position=cell(1,9);
cylinder_position{1}=[40,70,100];
cylinder_position{2}=[70,40,100];
cylinder_position{3}=[50,50,100];
cylinder_position{4}=[90,160,100];
cylinder_position{5}=[160,80,100];
cylinder_position{6}=[80,180,100];
cylinder_position{7}=[100,180,100];
cylinder_position{8}=[180,60,100];
cylinder_position{9}=[180,100,100];

% 循环绘制每个圆柱体+顶部封盖
for i=1:length(cylinder_position)
    x0 = cylinder_position{i}(1);
    y0 = cylinder_position{i}(2);
    h = cylinder_position{i}(3);
    cylinder_x = x0 + X_cylinder;
    cylinder_y = y0 + Y_cylinder;
    cylinder_z = h * Z_cylinder;
    surf(cylinder_x,cylinder_y,cylinder_z,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",1.0);
    theta = linspace(0, 2*pi, n_cylinder);
    top_x = x0 + cylinder_r * cos(theta);
    top_y = y0 + cylinder_r * sin(theta);
    top_z = repmat(h, size(top_x));
    fill3(top_x, top_y, top_z, [1,1,0], "EdgeColor","none","FaceAlpha",1.0);
end

% ===================== 【核心修改：所有轴设置统一放在绘图完成后】=====================
% 坐标轴名称+字号设置
xlabel("X/m","FontSize",label_font_size);
ylabel("Y/m","FontSize",label_font_size);
zlabel("Z/m","FontSize",label_font_size);

% 标题+字号设置
title("障碍物地图","FontSize",title_font_size);

% 【强制设置坐标范围，放在最后，不会被后续操作覆盖】
xlim([0,200]);
ylim([0,200]);
zlim([0,200]);

% 【三个坐标轴单位设置为20】
xticks(0:20:200);
yticks(0:20:200);
zticks(0:20:200);

% 【替代axis equal，保证X/Y/Z等比例，同时不覆盖zlim范围】
daspect([1 1 1]); 
% 补充：若觉得Z轴太空，可改为daspect([1 1 0.8])，微调Z轴比例，X/Y仍保持等比例

% 统一设置刻度字号
set(gca,"FontSize",tick_font_size);

% 基础样式设置
grid on;
% shading interp;
set(gca,"Color","white");

hold off;
