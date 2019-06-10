   %�ݹ���С�����㷨-----����˲ʱֵ�����ݶ�ʸ��
clc;
clear all;
close all;
%************************���ɷ����ź�**************************************
Fs = 10000;                                                     %���ò���Ƶ��
t = 0:1/Fs:7;  
t = t';
Size_t = size(t,1);
F1 = 2;
F2 = 10;
F3 = 20;
F4 = 500;
Signal = sin(2*pi*F1*t) + 0.5*sin(2*pi*F2*t) + 0.25*sin(2*pi*F3*t); %�����ź�
noise_amp = 1;                                           %���������ı�׼��
noise1 = noise_amp*randn(Size_t,1);                        %���ɸ�˹������
noise2 = noise_amp*randn(Size_t,1);
noise3 = 5*sin(2*pi*F4*t+pi/2);

noise = noise2;
Signal_noise = Signal + 0.2*noise;                           %�����˹������
Signal_noise(2:end) = Signal_noise(2:end) + 0.15*noise(1:end-1);
Signal_noise(3:end) = Signal_noise(3:end) + 0.1*noise(1:end-2);

figure("NAME","SIGNAL1");
subplot(2,1,1);
plot(t,Signal);
title('ԭʼ�ź�');
subplot(2,1,2);
plot(t,Signal_noise);
title('��������������ź�');
%*************************************************************************
M = 3;         %����FIR�˲�������
Signal_Len = Size_t - M -1;   %�����ź����ݵĸ���
niu = 0.00041;      %�㷨���ڲ�����������
y_out = zeros(Signal_Len,1);
error_out = zeros(Signal_Len,1);
Exp_out = zeros(Signal_Len,1);
w_out = zeros(Signal_Len,M);
for i=1:Signal_Len
    %��������
    if i == 1           %����ǵ�һ�ν���
        w = zeros(M,1); %��ʼ���˲�����ͷϵ��
    end
    d = Signal_noise(i+M-1);                 %�����µ������ź�
    x = noise((M + i -1):-1:i,1);           %�����µ��ź�ʸ��
    %�㷨����
    y = x' * w;                              %�����˲������
    error = d - y;                           %�������
    w_forward = w + niu * error * x;         %�����˲���ϵ������
    %��������
    w = w_forward;
    %�˲�����洢
    y_out(i) = y;
    error_out(i) = error;
    w_out(i,:) = w';
end
figure("NAME","SIGNAL2");
subplot(2,1,1);
plot(y_out);
title('�˲������');
subplot(2,1,2);
plot(error_out);
title('������');

figure("NAME","SIGNAL3");
plot(t(1:Signal_Len),w_out(:,1),'r',t(1:Signal_Len),w_out(:,fix(M/2)+1),'b',t(1:Signal_Len),w_out(:,M),'y');
title('����Ӧ�˲���ϵ��');

M = 3;                       %����FIR�˲�������
lamda = 1;                %������������
Signal_Len = Size_t - M -1;   %�����ź����ݵĸ���
I = eye(M);                   %���ɶ�Ӧ�ĵ�λ����
c = 1;                   %С���� ��֤����P������
y_out = zeros(Signal_Len,1);
Eta_out = zeros(Signal_Len,1);
w_out = zeros(Signal_Len,M);
for i=1:Signal_Len
    %��������
    if i == 1                 %����ǵ�һ�ν���
        P_last = I/c;
        w_last = zeros(M,1);  
    end
    d = Signal_noise(i+M-1);            %�����µ������ź�
    x = noise((M + i -1):-1:i,1);      %�����µ��ź�ʸ��
    %�㷨����
    K = (P_last * x)/(lamda + x'* P_last * x);   %��������ʸ��
    y = x'* w_last;                          %����FIR�˲������
    Eta = d - y;                             %������Ƶ����
    w = w_last + K * Eta;                    %�����˲���ϵ��ʸ��
    P = (I - K * x')* P_last/lamda;          %���������ؾ���
    %��������
    P_last = P;
    w_last = w;
    %�˲�����洢
    y_out(i) = y;
    Eta_out(i) = Eta;
    w_out(i,:) = w';
end
figure;
subplot(2,1,1);
plot(y_out);
title('�˲������');
subplot(2,1,2);
plot(Eta_out);
title('������');

figure;
plot(t(1:Signal_Len),w_out(:,1),'r',t(1:Signal_Len),w_out(:,fix(M/2)+1),'b',t(1:Signal_Len),w_out(:,M),'y');
title('����Ӧ�˲���ϵ��');
