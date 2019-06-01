
%
%   @    Accel raw data fitting
%
%   @Author     Niu Hongfang
%   @Date       2019.05.24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%;
clear all;
clc;
%close all;

earth_G = 9.80665;
Accel_ScaleFactor = 1/417.7;

accel_offset = [-0.023695834;-0.0131036071;-0.0022972305];
accel_scale = [1.00172770;1.00286853;0.987629831];

accel_mat = importdata("3_compressed.txt");

Accel_X = accel_mat.data(:,2);
Accel_Y = accel_mat.data(:,3);
Accel_Z = accel_mat.data(:,4);  

Accel_X_g = Accel_X*Accel_ScaleFactor/earth_G;
Accel_Y_g = Accel_Y*Accel_ScaleFactor/earth_G;
Accel_Z_g = Accel_Z*Accel_ScaleFactor/earth_G;

Accel_Z_g_mod = sqrt(Accel_X_g.*Accel_X_g + Accel_Y_g.*Accel_Y_g + Accel_Z_g.*Accel_Z_g) ;

Accel_X_g_cali = (Accel_X_g - accel_offset(1))*accel_scale(1);
Accel_Y_g_cali = (Accel_Y_g - accel_offset(2))*accel_scale(2);
Accel_Z_g_cali = (Accel_Z_g - accel_offset(3))*accel_scale(3);

Accel_Z_g_cali_mod = sqrt(Accel_X_g_cali.*Accel_X_g_cali + Accel_Y_g_cali.*Accel_Y_g_cali + Accel_Z_g_cali.*Accel_Z_g_cali);

N = length(Accel_X);
t = 1:N;

figure(1);
subplot(3,1,1);
plot(t,Accel_X_g,t,Accel_X_g_cali);grid on;;legend('Accel_X_g','Accel_X_g_cali');
subplot(3,1,2);
plot(t,Accel_Y_g,t,Accel_Y_g_cali);grid on;;
subplot(3,1,3);
plot(t,Accel_Z_g,t,Accel_Z_g_cali);grid on;;

figure(2);
plot(t,Accel_Z_g_mod, t, Accel_Z_g_cali_mod);legend('mod_a','mod_a_b');grid on;


fprintf("accel x mean value :%f \n", mean(Accel_X)*Accel_ScaleFactor/earth_G);
fprintf("accel y mean value :%f \n", mean(Accel_Y)*Accel_ScaleFactor/earth_G);
fprintf("accel z mean value :%f \n", mean(Accel_Z)*Accel_ScaleFactor/earth_G);

