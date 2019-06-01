%
%   @    spheroid fitting
%
%   @Author     Niu Hongfang
%   @Date       2019.05.24
%   @ref
%       https://blog.csdn.net/woniu199166/article/details/79108269
%       https://blog.csdn.net/hj199404182515/article/details/59480954
%
%       r^2 = (x-xo)^2 + (y-yo)^2 + (z-zo)^2;
%
%       e(xo,yo,zo,r) = (x-xo)^2 + (y-yo)^2 + (z-zo)^2 - r2;
%
%       E(xo,yo,zo,r) = e0(xo,yo,zo,r)^2 + e1(xo,yo,zo,r)^2 + ... + e(k-1)(xo,yo,zo,r)^2
%
%% %%

clear all;
close all
clc;
R = 2;         %球面半径
x0 = 100;      %球心x坐标
y0 = 1;        %球心y坐标
z0 = 76;       %球心z坐标
%********************************生成随机球面数据************************************
alfa = 0:pi/50:pi;
sita = 0:pi/50:2*pi;
num_alfa = length(alfa);
num_sita = length(sita);
x = zeros(num_alfa,num_sita);
y = zeros(num_alfa,num_sita);
z = zeros(num_alfa,num_sita);
for i = 1:num_alfa
    for j = 1:num_sita
        x(i,j) = x0+R*sin(alfa(i))*cos(sita(j));
        y(i,j) = y0+R*sin(alfa(i))*sin(sita(j));
        z(i,j) = z0+R*cos(alfa(i));
    end
end
 
x = reshape(x,num_alfa*num_sita,1);
y = reshape(y,num_alfa*num_sita,1);
z = reshape(z,num_alfa*num_sita,1);
figure;
plot3(x,y,z,'*');
title('生成的没有噪声的球面数据');
%加入均值为0的高斯分布噪声 
amp = 0.1;
x = x + amp*rand(num_alfa*num_sita,1);
y = y + amp*rand(num_alfa*num_sita,1);
z = z + amp*rand(num_alfa*num_sita,1);
figure;
plot3(x,y,z,'*');
title('加入噪声的球面数据');
%*******************************************************************************************
%球面拟合算法
num_points = length(x);
x_avr = sum(x)/num_points;
y_avr = sum(y)/num_points;
z_avr = sum(z)/num_points;
 
xx_avr = sum(x.*x)/num_points;
yy_avr = sum(y.*y)/num_points;
zz_avr = sum(z.*z)/num_points;
xy_avr = sum(x.*y)/num_points;
xz_avr = sum(x.*z)/num_points;
yz_avr = sum(y.*z)/num_points;
 
xxx_avr = sum(x.*x.*x)/num_points;
xxy_avr = sum(x.*x.*y)/num_points;
xxz_avr = sum(x.*x.*z)/num_points;
xyy_avr = sum(x.*y.*y)/num_points;
xzz_avr = sum(x.*z.*z)/num_points;
yyy_avr = sum(y.*y.*y)/num_points;
yyz_avr = sum(y.*y.*z)/num_points;
yzz_avr = sum(y.*z.*z)/num_points;
zzz_avr = sum(z.*z.*z)/num_points;
%计算求解线性方程的系数矩阵
A = [xx_avr - x_avr*x_avr,xy_avr - x_avr*y_avr,xz_avr - x_avr*z_avr;
     xy_avr - x_avr*y_avr,yy_avr - y_avr*y_avr,yz_avr - y_avr*z_avr;
     xz_avr - x_avr*z_avr,yz_avr - y_avr*z_avr,zz_avr - z_avr*z_avr];
b = [xxx_avr - x_avr*xx_avr + xyy_avr - x_avr*yy_avr + xzz_avr - x_avr*zz_avr;
     xxy_avr - y_avr*xx_avr + yyy_avr - y_avr*yy_avr + yzz_avr - y_avr*zz_avr;
     xxz_avr - z_avr*xx_avr + yyz_avr - z_avr*yy_avr + zzz_avr - z_avr*zz_avr];
b = b/2;
 
resoult = inv(A)*b;
 
x00 = resoult(1);     %拟合出的x坐标
y00 = resoult(2);     %拟合出的y坐标
z00 = resoult(3);     %拟合出的z坐标
r = sqrt(xx_avr-2*x00*x_avr+x00*x00 + yy_avr-2*y00*y_avr+y00*y00 + zz_avr-2*z00*z_avr+z00*z00);   %拟合出的球半径r

%%
fprintf("original param: \n\txo:%12f\t\tyo:%12f\t\tzo:%12f   R:%12f\n\n", 100, 1, 76, 2);
fprintf("fitting  param: \n\txo:%12f\t\tyo:%12f\t\tzo:%10f   R:%12f\n", x00, y00, z00, r);