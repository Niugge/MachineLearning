%%==================================================================================
%       状态观测器：
%                   输入：sin(x)，加上高斯白噪声
%
%                   M=1
%                           使用二阶震荡环节   G(s) = wn^2/(s^2 + 2*ksi*wn*s + wn^2)
%
%                           dx1 = x2
%                           dx2 = -wn^2*(x1 - v) - 2*ksi*wn*x2
%
%                   M=2     e = v - x1
%                           dx1 = x2 + k1*e
%                           dx2 = k2*e
%                       误差传递函数：
%                           G(s) = xxx/(s^2 + k1*s + k2),  k1,k2>0
%
%                   M=3     
%                           e = v - x1
%                           dx1 = x2 + b1*e
%                           dx2 = b2*e
%
%       note:
%                   1、二阶震荡环节具有延迟
%                   2、状态观测器
%                   3、线性扩张状态观测器
%                   4、TD微分跟踪控制器
%%==================================================================================%%
clc;
clear;
close all;

h=0.001;
N=10000;

wn=10;
ksi=0.707;

x1 = zeros(1,N);
x2 = zeros(1,N);
y = zeros(1,N);

vo=zeros(1,N);
vn=zeros(1,N);

omg = 10;
beta1 = 2*omg;
beta2 = omg^2;

w = 1;

alpha = 0.08;
noise = alpha*(rand(1,N)-0.5);

M=4;

for i=1:N-1
   
    t(i) = h*i;
    vo(i) = sin(w*t(i));
    vn(i) = noise(i) + vo(i);
    
    if M==1
        x1(i+1) = x1(i) + h*x2(i);
        x2(i+1) = x2(i) + h*(-wn^2*(x1(i)-vn(i))-2*wn*ksi*x2(i));
    elseif M==2
        e = vn(i) - x1(i);
        
        x1(i+1) = x1(i) + h*(x2(i)+2*ksi*wn*e);
        x2(i+1) = x2(i) + h*wn^2*e;
    elseif M==3
        e = x1(i) - vn(i);
        
        x1(i+1) = x1(i) + h*(x2(i) - beta1*e);
        x2(i+1) = x2(i) + h*(-beta2*e);
    elseif M==4
    
        r=10;
        h0=50*h;
        
        x1(i+1) = x1(i) + h*x2(i);
        x2(i+1) = x2(i) + h*fhan(x1(i)-vn(i), x2(i), r, h0);
        
    end
    
    y(i)=x2(i);
end

t(N) = h*N;

figure(1);
plot(t,vo,'r',t,vn,'k');legend('vo','vn');
figure(2);
plot(t,vn,'k',t,x1,'r');legend('vn','x1');
figure(3);
plot(t,x1,'k',t,cos(t),'b',t,y,'r');legend('x1','cos','y');


function u = fhan(x1,x2,r,h)
%     d = r*h;
%     d0 = h*d;
%     y = x1+x2*h;
%     a0 = sqrt(d^2+8*r*sign(y));
% 
%     if abs(y)>d0
%         a= x2+(a0-d)*sign(y)/2;
%     else
%         a = x2+y/h;
%     end
%     
%     if abs(a)>d
%         u  = -r*sign(a);
%     else
%         u = -r*a/d;
%     end
    d = r*h^2;
    a0 = h*x2;
    y = x1+a0;
    a1 = sqrt(d*(d+8*abs(y)));
    a2 = a0 + sign(y)*(a1-d)/2;
    a = (a0+y)*fsg(y,d) + a2*(1-fsg(y,d));
    u = -r*(a/d)*fsg(a,d)-r*sign(a)*(1-fsg(a,d));
end

function u = fsg(x,d)
    u = (sign(x+d)-sign(x-d))/2;
end