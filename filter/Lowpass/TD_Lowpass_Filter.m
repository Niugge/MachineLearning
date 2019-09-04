clc;
clear;
close all;

wc=30;
r=(wc/1.14)^2;

w1 = 10;
w2 = 200;
w3 = 300;

N=1000;

x1 = zeros(1,N);
x2 = zeros(1,N);
v0 = zeros(1,N);

h=0.001;


for i=1:N-1
   
    t(i) = h*i;
    v0(i) = 4*sin(w1*t(i)) + sin(w2*t(i)) + sin(w3*t(i));
    
    fh = fhan(x1(i) - v0(i), x2(i), r, 2*h);
    
    x1(i+1) = x1(i) + h*x2(i);
    x2(i+1) = x2(i) + h*fh;
    
end

t(N) = h*N;

figure(1);
plot(t,v0,'k',t,x1,'b');legend('v0','x1');

vf=fft(v0,N);
vfab=abs(vf);
f=1:N/2;
figure(2);
plot(f*2*pi,vfab(1:N/2));

vf=fft(x1,N);
vfab=abs(vf);
f=1:N/2;
figure(3);
plot(f*2*pi,vfab(1:N/2));

%%
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