%
%   @    circle fitting
%
%   @Author     Niu Hongfang
%   @Date       2019.05.24
%   @ref
%           https://blog.csdn.net/sinat_21107433/article/details/80877704
%           https://blog.csdn.net/Jacky_Ponder/article/details/70314919
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%;

clear all;
close all;
clc;


%% %%
% ===============属性===================%
Ro = 10;
xo = 5;
yo = 8;

% ================生成n个圆上的点========%
t=0:pi/50:2*pi;
N=length(t);

x = zeros(1,N);
y = zeros(1,N);
xn = zeros(1,N);
yn = zeros(1,N);
xf = zeros(1,N);
yf = zeros(1,N);

x = xo + Ro*sin(t);
y = yo + Ro*cos(t);

plot(x,y,'c',xo,yo,'co','LineWidth',2);
grid on;

% ================加入噪声========%
gama = 0.8;
xn = x + gama*(2*rand(1,N)-1);
yn = y + gama*(2*rand(1,N)-1);
hold on;
plot(xn,yn,'b*');

% ================拟合圆========%
sum_X_Raw = 0;
sum_Y_Raw = 0;
sum_XSquare_Raw = 0;
sum_YSquare_Raw = 0;
sum_XCube_Raw = 0;
sum_YCube_Raw = 0;
sum_XYY_Raw = 0;
sum_XY_Raw = 0;
sum_XXY_Raw = 0;

for i=1:N
    sum_X_Raw = sum_X_Raw+xn(i);
    sum_Y_Raw = sum_Y_Raw+yn(i);
    sum_XSquare_Raw = sum_XSquare_Raw+x(i)*xn(i);
    sum_YSquare_Raw = sum_YSquare_Raw+yn(i)*yn(i);
    sum_XCube_Raw = sum_XCube_Raw+xn(i)^3;
    sum_YCube_Raw = sum_YCube_Raw+yn(i)^3;
    sum_XY_Raw = sum_XY_Raw+xn(i)*yn(i);
    sum_XYY_Raw = sum_XYY_Raw+xn(i)*yn(i)^2;
    sum_XXY_Raw = sum_XXY_Raw+xn(i)^2*yn(i);
end
D = N*sum_XY_Raw-sum_X_Raw*sum_Y_Raw;
C = N*sum_XSquare_Raw-sum_X_Raw*sum_X_Raw;
E = N*sum_XCube_Raw+N*sum_XYY_Raw-(sum_XSquare_Raw+sum_YSquare_Raw)*sum_X_Raw;
G = N*sum_YSquare_Raw-sum_Y_Raw*sum_Y_Raw;
H = N*sum_YCube_Raw+N*sum_XXY_Raw-(sum_XSquare_Raw+sum_YSquare_Raw)*sum_Y_Raw;

a = (H*D-E*G)/(C*G-D*D);
b = (H*C-E*D)/(D*D-G*C);
c = -((sum_XSquare_Raw+sum_YSquare_Raw)+a*sum_X_Raw+b*sum_Y_Raw)/N;

p(1) = -0.5*a;
p(2) = -0.5*b;
p(3) = 0.5*sqrt(a*a+b*b-4*c);

% ==============拟合后的圆=======%
xf = p(1) + p(3)*sin(t);
yf = p(2) + p(3)*cos(t);

plot(p(1),p(2),'ro',xf,yf,'r');
