%
%   @    Stright line fitting
%
%   @Author     Niu Hongfang
%   @Date       2019.05.24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%;
clc;
close all;

%%  ������
x=1:50;
y=3*x+8*rand(1,50);

N = length(x);

z = zeros(1,N);

%% ����һ���ֶ��������ʽ

sum_x = 0;
sum_x_squre = 0;
sum_x_y = 0;
sum_y = 0;


for i=1:N
   
    sum_x = sum_x + x(i);
    sum_x_squre = sum_x_squre + x(i)^2;
    sum_x_y = sum_x_y + x(i)*y(i);
    sum_y = sum_y + y(i);
    
end

A = sum_x_squre;
B = sum_x;
C = sum_x_y;
D = sum_y;

%===============�����ϵ��a,b
a = (N*C-B*D)/(A*N-B^2);
b = (B*C-A*D)/(B^2-A*N);

%==============��Ϻ������z
z = a*x+b;

%% ��������matlab�⺯�����
p = polyfit(x,y,1);
z2 = polyval(p,x);

%% �Ա�����
plot(x,y,'r*',x,z,'k',x,z2,'g-');legend('y','z','z2');
