%clear all;
close all;
clc;
%[log_path] = uigetdir();
%log_mat = [log_path,'\log001_Mat.mat'];
%load(log_mat)

figure;
%subplot(3,1,1);plot(AtpLog.IMU0_Az);legend('z');
%subplot(3,1,2);plot(AtpLog.IMU0_Ay);legend('y');
%subplot(3,1,3);plot(AtpLog.IMU0_Ax);legend('x');

subplot(3,1,1);plot(ts_sensor_imu_0.ts_az.data);legend('z');
subplot(3,1,2);plot(ts_sensor_imu_0.ts_ay.data);legend('y');
subplot(3,1,3);plot(ts_sensor_imu_0.ts_ax.data);legend('x');

index_get = ginput(12);

for i = 1:6
    index(i,1) = floor(index_get(i*2 - 1));
    index(i,2) = floor(index_get(i*2));
end



fid = fopen('log_acc.txt', 'w');


for i = 1:length(index)
    %     for i = index(n,1):index(n,2)
    fprintf(fid, '%d %d %d\n', ts_sensor_imu_0.ts_ax.data(index(i)),ts_sensor_imu_0.ts_ay.data(index(i)),ts_sensor_imu_0.ts_az.data(index(i)));
    %     end
end

fclose(fid);
fid = fopen('log_acc.txt','r');
Data = fscanf(fid, '%f %f %f', [3 inf]);
Data = Data';
fclose(fid);
x = Data(:,1);
y = Data(:,2);
z = Data(:,3);

P = diag([10,10,10,0,0,0,1,1,1]);

v = zeros(9,1);

R = 0.001;

for i = 1:length(x)
    [v, P] = ellipsoid_fit_step(x(i),y(i),z(i),v,P,R);
end

[rotM, bias, u, radii] = ellipsoid_fit_solve(v);
disp('rotM is:');
%rot1 = [0 -1 0;-1 0 0;0 0 -1];
rot1 = [1 0 0;0 1 0;0 0 1];
disp(rot1*rotM);
disp('acc bias is');
disp(bias);




fid = fopen('log_gyr.txt', 'w');
for idx = index(1,1):index(1,2)
    fprintf(fid, '%d %d %d\n', ts_sensor_imu_0.ts_gx.data(idx),ts_sensor_imu_0.ts_gy.data(idx),ts_sensor_imu_0.ts_gz.data(idx));
end

clear x y z

fclose(fid);
fid = fopen('log_gyr.txt','r');

Data = fscanf(fid, '%f %f %f', [3 inf]);
Data = Data';
fclose(fid);
x = Data(:,1);
y = Data(:,2);
z = Data(:,3);
gyro_bias = mean([x y z]);
disp('gyro_bias = ');
disp(gyro_bias);


