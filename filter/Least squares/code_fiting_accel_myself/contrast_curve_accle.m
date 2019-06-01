function  contrast_curve_accle(param)

offset  = param(1:3);
diag    = param(4:6);
offdiag = param(7:9);

earth_G = 9.80665;
Accel_ScaleFactor = 1/417.7;

accel_mat = importdata("1_compressed.txt");

Accel_X = accel_mat.data(:,2);
Accel_Y = accel_mat.data(:,3);
Accel_Z = accel_mat.data(:,4);  

Accel_X_g = Accel_X*Accel_ScaleFactor/earth_G;
Accel_Y_g = Accel_Y*Accel_ScaleFactor/earth_G;
Accel_Z_g = Accel_Z*Accel_ScaleFactor/earth_G;

N = length(Accel_X);
t = 1:N;

data_raw = [Accel_X_g, Accel_Y_g, Accel_Z_g]';

[m,n] = size(data_raw);

data_filter = zeros(m,n);

M = [diag(1)        offdiag(1)      offdiag(2);...
     offdiag(1)     diag(2)         offdiag(3);...
     offdiag(2)     offdiag(3)      diag(3)];
 
for i=1:N
    data_filter(:,i) = M * (data_raw(:,i) + offset');%
end

data_raw_mod    = sqrt(data_raw(1,:).*data_raw(1,:) + data_raw(2,:).*data_raw(2,:) + data_raw(3,:).*data_raw(3,:));
data_filter_mod = sqrt(data_filter(1,:).*data_filter(1,:) + data_filter(2,:).*data_filter(2,:) + data_filter(3,:).*data_filter(3,:));

figure(3);
subplot(3,1,1);
plot(t,data_raw(1,:),t,data_filter(1,:));grid on;;legend('Accel_X_g','Accel_X_g_cali');
subplot(3,1,2);
plot(t,data_raw(2,:),t,data_filter(2,:));grid on;;
subplot(3,1,3);
plot(t,data_raw(3,:),t,data_filter(3,:));grid on;;

figure(4);
plot(t, data_raw_mod, t, data_filter_mod);grid on;

end