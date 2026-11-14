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

% 第四座山体
mountain_x3=90;
mountain_y3=90;
sigma3=30;
mountain_A3=120;
z_guass3=mountain_A3*exp(-((X-mountain_x3).^2+(Y-mountain_y3).^2)/(2*sigma3*sigma3));
z_mountain3=z_guass3+noise;
z_mountain3(z_mountain3<0)=0;
surf(X,Y,z_mountain3,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);

% 第五座山体
mountain_x3=40;
mountain_y3=40;
sigma3=30;
mountain_A3=50;
z_guass3=mountain_A3*exp(-((X-mountain_x3).^2+(Y-mountain_y3).^2)/(2*sigma3*sigma3));
z_mountain3=z_guass3+noise;
z_mountain3(z_mountain3<0)=0;
surf(X,Y,z_mountain3,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);

% 第6座山体
mountain_x3=180;
mountain_y3=90;
sigma3=30;
mountain_A3=100;
z_guass3=mountain_A3*exp(-((X-mountain_x3).^2+(Y-mountain_y3).^2)/(2*sigma3*sigma3));
z_mountain3=z_guass3+noise;
z_mountain3(z_mountain3<0)=0;
surf(X,Y,z_mountain3,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);

% 第7座山体
mountain_x3=90;
mountain_y3=180;
sigma3=30;
mountain_A3=100;
z_guass3=mountain_A3*exp(-((X-mountain_x3).^2+(Y-mountain_y3).^2)/(2*sigma3*sigma3));
z_mountain3=z_guass3+noise;
z_mountain3(z_mountain3<0)=0;
surf(X,Y,z_mountain3,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);

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
