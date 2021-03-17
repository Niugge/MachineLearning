
InputDir = uigetdir;
if InputDir == 0
    disp('请选择log路径!')
    return;
end
%RC_file_name = FindFiles(InputDir,'rc_channels'); 
PWM_file_name = FindFiles(InputDir,'actuator_controls');
float_log_name = FindFiles(InputDir,'control_system_y_float_log');
uint32_log_name = FindFiles(InputDir,'control_system_y_uint32_log');
states_estimator_name = FindFiles(InputDir,'states_estimator');
sensor_combined_name = FindFiles(InputDir,'sensor_combined');
mag_file_name = FindFiles(InputDir,'vehicle_magnetometer');
baro_file_name = FindFiles(InputDir,'vehicle_air_data');
gnss_file_name = FindFiles(InputDir,'vehicle_gps_position');
distance_file_name = FindFiles(InputDir,'distance_sensor');
command_mission_name = FindFiles(InputDir,'command_mission');
command_gs_name = FindFiles(InputDir,'command_gs');
command_cloud_name = FindFiles(InputDir,'command_cloud');
marker_file_name = FindFiles(InputDir,'visual_location');
mission_status_name = FindFiles(InputDir,'mission_status');
warning_file_name = FindFiles(InputDir,'warning_info');
ESC_file_name = FindFiles(InputDir,'esc_status1');
flight_status_file_name = FindFiles(InputDir,'flight_status');
CPU_file_name = FindFiles(InputDir,'cpuload');
soa_file_name = FindFiles(InputDir,'soa_info');
VehicleCmd_file_name = FindFiles(InputDir,'vehicle_command');
Trajectory_file_name = FindFiles(InputDir,'vehicle_trajectory_waypoint');
Battery_file_name = FindFiles(InputDir,'battery');
Safety_file_name = FindFiles(InputDir,'safety');
servo_file_name = FindFiles(InputDir,'servo_feedback');


if RC_file_name
    RC_log = readmatrix(RC_file_name);
else
    RC_log = false;
end
if PWM_file_name
    PWM_log = readmatrix(PWM_file_name);
else
    PWM_log = false;
end
if float_log_name
    float_log = readmatrix(float_log_name);
else
    float_log = false;
end
if uint32_log_name
    uint32_log = readmatrix(uint32_log_name);
else
    uint32_log = false;
end
if states_estimator_name
    states_estimator_log = readmatrix(states_estimator_name);
els
    states_estimator_log = false;
end
if sensor_combined_name
    sensor_combined_log = readmatrix(sensor_combined_name);
else
    sensor_combined_log = false;
end
if mag_file_name
    mag_log = readmatrix(mag_file_name);
else
    mag_log = false;
end

if baro_file_name
    baro_log = readmatrix(baro_file_name);
else
    baro_log = false;
end

if gnss_file_name
    gnss_log = readmatrix(gnss_file_name);
else
    gnss_log = false;
end

if distance_file_name
    distance_log = readmatrix(distance_file_name);
else
    distance_log = 0;
end

if command_mission_name
    command_mission_log = readmatrix(command_mission_name);
else
    command_mission_log = 0;
end

if command_gs_name
    command_gs_log = readmatrix(command_gs_name);
else
    command_gs_log = 0;
end

if command_cloud_name
    command_cloud_log = readmatrix(command_cloud_name);
else
    command_cloud_log = 0;
end

if mission_status_name
    mission_status_log = readmatrix(mission_status_name);
else
    mission_status_log = 0;
end

if warning_file_name
    warning_info_log = readmatrix(warning_file_name);
else
    warning_info_log = 0;
end

if ESC_file_name
    ESC_log = readmatrix(ESC_file_name);
else
    ESC_log = 0;
end

if flight_status_file_name
    flight_status_log = readmatrix(flight_status_file_name);
else
    flight_status_log = 0;
end

if CPU_file_name
    CPU_log = readmatrix(CPU_file_name);
else
    CPU_log = 0;
end

if VehicleCmd_file_name
    VehicleCmd_log = readmatrix(VehicleCmd_file_name);
else
    VehicleCmd_log = 0;
end

if Trajectory_file_name
    Trajectory_log = readmatrix(Trajectory_file_name);
else
    Trajectory_log = 0;
end

if soa_file_name
    soa_log = readmatrix(soa_file_name);
else
    soa_log = 0;
end

if Battery_file_name
    Battery_log = readmatrix(Battery_file_name);
else
    Battery_log = 0;
end

if Safety_file_name
    Safety_log = readmatrix(Safety_file_name);
else
    Safety_log = 0;
end

if servo_file_name
    servo_log = readmatrix(servo_file_name);
else
    servo_log = 0;
end


%% 时间戳处理
RC_timestamp = RC_log(:,1) * 1e-6;
PWM_timestamp = PWM_log(:,1) * 1e-6;
float_log_timestamp = float_log(:,1) * 1e-6;
uint32_log_timestamp = uint32_log(:,1) * 1e-6;
states_estimator_timestamp = states_estimator_log(:,1) * 1e-6;
sensor_combined_timestamp = sensor_combined_log(:,1) * 1e-6;
mag_timestamp = mag_log(:,1) * 1e-6;
baro_timestamp = baro_log(:,1) * 1e-6;
gnss_timestamp = gnss_log(:,1) * 1e-6;
distance_timestamp = distance_log(:,1) * 1e-6;
command_mission_timestamp = command_mission_log(:,1) * 1e-6;
command_gs_timestamp = command_gs_log(:,1) * 1e-6;
command_cloud_timestamp = command_cloud_log(:,1) * 1e-6;
marker_location_timestamp = marker_location_log(:,1) * 1e-6;
mission_status_timestamp = mission_status_log(:,1) * 1e-6;
warning_info_timestamp = warning_info_log(:,1) * 1e-6;
ESC_timestamp = ESC_log(:,1) * 1e-6;
flight_status_timestamp = flight_status_log(:,1) * 1e-6;
cpu_load_timestamp = CPU_log(:,1) * 1e-6;
soa_timestamp = soa_log(:,1)* 1e-6;
VehicleCmd_timestamp = VehicleCmd_log(:,1) * 1e-6;
Trajectory_timestamp = Trajectory_log(:,1) * 1e-6;
Battery_timestamp = Battery_log(:,1) * 1e-6;
Safety_timestamp = Safety_log(:,1) * 1e-6;
servo_timestamp = servo_log(:,1) * 1e-6;


% system_start_time = 0;
simu_start_time = min([RC_timestamp(1), PWM_timestamp(1), float_log_timestamp(1), uint32_log_timestamp(1), states_estimator_timestamp(1),...
                  sensor_combined_timestamp(1), mag_timestamp(1), baro_timestamp(1), gnss_timestamp(1), distance_timestamp(1),...
                  flight_status_timestamp(1), cpu_load_timestamp(1)]);
simu_stop_time = max([RC_timestamp(end), PWM_timestamp(end), float_log_timestamp(end), uint32_log_timestamp(end), states_estimator_timestamp(end),...
                 sensor_combined_timestamp(end), mag_timestamp(end), baro_timestamp(end), gnss_timestamp(end), distance_timestamp(end),...
                  flight_status_timestamp(end), cpu_load_timestamp(end)]);
system_start_time = simu_start_time;
simu_time = simu_stop_time - simu_start_time;

RC_timestamp = RC_timestamp - system_start_time;
PWM_timestamp = PWM_timestamp - system_start_time;
float_log_timestamp = float_log_timestamp - system_start_time;
uint32_log_timestamp = uint32_log_timestamp - system_start_time;
states_estimator_timestamp = states_estimator_timestamp - system_start_time;
% states_estimator_timestamp = states_estimator_timestamp;
sensor_combined_timestamp = sensor_combined_timestamp - system_start_time;
mag_timestamp = mag_timestamp - system_start_time;
baro_timestamp = baro_timestamp - system_start_time;
gnss_timestamp = gnss_timestamp - system_start_time;
distance_timestamp = distance_timestamp - system_start_time;
command_mission_timestamp = command_mission_timestamp - system_start_time;
command_gs_timestamp = command_gs_timestamp - system_start_time;
command_cloud_timestamp = command_cloud_timestamp - system_start_time;
marker_location_timestamp = marker_location_timestamp - system_start_time;
mission_status_timestamp = mission_status_timestamp - system_start_time;
warning_info_timestamp = warning_info_timestamp - system_start_time;
ESC_timestamp = ESC_timestamp - system_start_time;
flight_status_timestamp = flight_status_timestamp - system_start_time;
cpu_load_timestamp = cpu_load_timestamp - system_start_time;
VehicleCmd_timestamp = VehicleCmd_timestamp - system_start_time;
Trajectory_timestamp = Trajectory_timestamp - system_start_time;
Battery_timestamp = Battery_timestamp - system_start_time;
Safety_timestamp = Safety_timestamp - system_start_time;
soa_timestamp = soa_timestamp - system_start_time;
servo_timestamp = servo_timestamp - system_start_time;

%% RC input data
if RC_file_name
    ts_RC = [];
    
    kill_switch = [];
    control_mode = [];
    for i = 1:length(RC_log(:,10))
        if RC_log(i,10) >= 0.9 & RC_log(i,11) >= 0.9
            kill_switch(i) = 1;
        else
            kill_switch(i) = 0;
        end
        
        if RC_log(i,7) >= 0.1
            control_mode(i) = ControlMode.CONTROL_MODE_POSITION;
        elseif RC_log(i,7) <= -0.1
            control_mode(i) = ControlMode.CONTROL_MODE_MANUAL;
        else
            control_mode(i) = ControlMode.CONTROL_MODE_ATTITUDE;
        end
    end
    
    stick_in = [RC_log(:,2) RC_log(:,3) RC_log(:,4) RC_log(:,5)];
    
    ts_RC.stick_in = timeseries(single(stick_in),RC_timestamp);
    ts_RC.CH5 = timeseries(single(RC_log(:,6)),RC_timestamp);
    ts_RC.CH6 = timeseries(single(RC_log(:,7)),RC_timestamp);
    ts_RC.CH7 = timeseries(single(RC_log(:,8)),RC_timestamp);
    ts_RC.CH8 = timeseries(single(RC_log(:,9)),RC_timestamp);
    ts_RC.CH9 = timeseries(single(RC_log(:,10)),RC_timestamp);
    ts_RC.CH10 = timeseries(single(RC_log(:,11)),RC_timestamp);
    ts_RC.CH11 = timeseries(single(RC_log(:,12)),RC_timestamp);
%     ts_RC.CH12 = timeseries(single(RC_log(:,14)),RC_timestamp);
%     ts_RC.CH13 = timeseries(single(RC_log(:,15)),RC_timestamp);
%     ts_RC.CH14 = timeseries(single(RC_log(:,16)),RC_timestamp);
%     ts_RC.CH15 = timeseries(single(RC_log(:,17)),RC_timestamp);
%     ts_RC.CH16 = timeseries(single(RC_log(:,18)),RC_timestamp);
    ts_RC.kill_switch = timeseries(boolean(kill_switch),RC_timestamp);
    ts_RC.control_mode = timeseries(uint8(control_mode),RC_timestamp);
    ts_RC.timestamp = timeseries(single(RC_log(:,1)),RC_timestamp);
    clear kill_switch control_mode;
else
    ts_RC = [];
end

%% Raw IMU data input
if sensor_combined_name
    ts_gyroscope = [];
    ts_accelerator = [];
    ts_gyroscope.valid = timeseries(boolean(ones(length(sensor_combined_timestamp),1)),sensor_combined_timestamp);
    ts_gyroscope.x = timeseries(double(sensor_combined_log(:,2)),sensor_combined_timestamp);
    ts_gyroscope.y = timeseries(double(sensor_combined_log(:,3)),sensor_combined_timestamp);
    ts_gyroscope.z = timeseries(double(sensor_combined_log(:,4)),sensor_combined_timestamp);
    ts_gyroscope.temperature = timeseries(double(sensor_combined_log(:,11)),sensor_combined_timestamp);
    ts_gyroscope.timestamp = timeseries(double(sensor_combined_timestamp * 1e6),sensor_combined_timestamp);

    ts_accelerator.valid = timeseries(boolean(ones(length(sensor_combined_timestamp),1)),sensor_combined_timestamp);
    ts_accelerator.x = timeseries(double(sensor_combined_log(:,7)),sensor_combined_timestamp);
    ts_accelerator.y = timeseries(double(sensor_combined_log(:,8)),sensor_combined_timestamp);
    ts_accelerator.z = timeseries(double(sensor_combined_log(:,9)),sensor_combined_timestamp);
    ts_accelerator.temperature = timeseries(double(sensor_combined_log(:,11)),sensor_combined_timestamp);
    ts_accelerator.timestamp = timeseries(double(sensor_combined_timestamp * 1e6),sensor_combined_timestamp);
    clear sensor_combined_timestamp sensor_combined_log
else
    ts_gyroscope = [];
    ts_accelerator = [];
end

%% Magnetometor data
if mag_file_name
    ts_magnetometor = [];
    ts_magnetometor.mag_x = timeseries(single(mag_log(:,2)*1e5),mag_timestamp);
    ts_magnetometor.mag_y = timeseries(single(mag_log(:,3)*1e5),mag_timestamp);
    ts_magnetometor.mag_z = timeseries(single(mag_log(:,4)*1e5),mag_timestamp);
    ts_magnetometor.timestamp = timeseries(single(mag_timestamp*1e6),mag_timestamp);
    clear mag_log mag_timestamp
else
    ts_magnetometor = [];
end

%% GNSS data
if gnss_file_name
    ts_GNSS = [];
    ts_GNSS.timestamp_utc = timeseries(double(gnss_log(:,2)),gnss_timestamp);
    ts_GNSS.latitude = timeseries(double(gnss_log(:,3)/1e7),gnss_timestamp);
    ts_GNSS.longitude = timeseries(double(gnss_log(:,4)/1e7),gnss_timestamp);
    ts_GNSS.height = timeseries(double(gnss_log(:,6)/1e3),gnss_timestamp);
    ts_GNSS.sigma_h = timeseries(single(gnss_log(:,9)),gnss_timestamp);
    ts_GNSS.sigma_v = timeseries(single(gnss_log(:,10)),gnss_timestamp);
    ts_GNSS.V_N = timeseries(double(gnss_log(:,16)),gnss_timestamp);
    ts_GNSS.V_E = timeseries(double(gnss_log(:,17)),gnss_timestamp);
    ts_GNSS.V_D = timeseries(double(gnss_log(:,18)),gnss_timestamp);
    ts_GNSS.sigma_speed_h = timeseries(single(gnss_log(:,7)),gnss_timestamp);
    ts_GNSS.sigma_speed_v = timeseries(single(gnss_log(:,7)),gnss_timestamp);
    ts_GNSS.heading = timeseries(single(gnss_log(:,21)),gnss_timestamp);
    ts_GNSS.sigma_heading = timeseries(single(zeros(length(gnss_timestamp),1)),gnss_timestamp);
    ts_GNSS.fix_type_postion = timeseries(uint32(gnss_log(:,24)),gnss_timestamp);
    ts_GNSS.fix_type_heading = timeseries(uint32(zeros(length(gnss_timestamp),1)),gnss_timestamp);
    ts_GNSS.velocity_valid = timeseries(boolean(gnss_log(:,25)),gnss_timestamp);
    ts_GNSS.satellites_used = timeseries(uint8(gnss_log(:,26)),gnss_timestamp);
    ts_GNSS.differential_age = timeseries(single(gnss_log(:,23)),gnss_timestamp);
    ts_GNSS.timestamp = timeseries(double(gnss_timestamp*1e6),gnss_timestamp);
    clear gnss_log  gnss_timestamp
else
    ts_GNSS = [];
end

%% Barometor data
if baro_file_name
    ts_baro = [];
    ts_baro.pressure = timeseries(single(baro_log(:,4)),baro_timestamp);
    ts_baro.temperature = timeseries(single(baro_log(:,3)),baro_timestamp);
    ts_baro.timestamp = timeseries(double(baro_timestamp*1e6),baro_timestamp);
    P0 = 101325;
    ts_baro.height_sea=timeseries(44300*(1- (ts_baro.pressure.Data/P0).^(1/5.256)),baro_timestamp );
    clear baro_timestamp  baro_log
else
    ts_baro = [];
end


%% TOF data
if distance_file_name
    ts_TOF = [];
    ts_TOF.distance = timeseries(single(distance_log(:,4)),distance_timestamp);
    ts_TOF.tamplitude = timeseries(single(distance_log(:,3)),distance_timestamp);
    ts_TOF.percision = timeseries(single(distance_log(:,5)),distance_timestamp);
    ts_TOF.state = timeseries(uint16(distance_log(:,6)),distance_timestamp);
    ts_TOF.timestamp = timeseries(double(distance_timestamp*1e6),distance_timestamp);
    clear distance_log distance_timestamp
else
    ts_TOF = [];
end

%% PWM output data
if PWM_file_name
    ts_PWM = timeseries(single(PWM_log(:,3:8)),PWM_timestamp);
    clear PWM_timestamp PWM_log
else
    ts_PWM = [];
end

%% soa output data
if soa_file_name
    ts_soa.timestamp = timeseries(single(soa_log(:,1)),soa_timestamp);
    ts_soa.vx_positive_limit = timeseries(single(soa_log(:,2)),soa_timestamp);
    clear soa_timestamp soa_log
else
    ts_soa = [];
end

%% States Estimator Results
if states_estimator_name
    ts_StatesEstm =[];
    euler_ypr = states_estimator_log(:,9:11);
    q_b_e = [];
    R_b_e = [];
    for i = 1:length(euler_ypr(:,1))
        q_b_e(i,:) = angle2quat(euler_ypr(i,1), euler_ypr(i,2), euler_ypr(i,3), 'ZYX');
        R_b_e(:,:,i) = angle2dcm(euler_ypr(i,1), euler_ypr(i,2), euler_ypr(i,3), 'ZYX');
    end

    ts_StatesEstm.q_b_e = timeseries(double(q_b_e),states_estimator_timestamp);
    ts_StatesEstm.R_b_e = timeseries(double(R_b_e),states_estimator_timestamp);
    ts_StatesEstm.euler_ypr = timeseries(double(euler_ypr),states_estimator_timestamp);
    ts_StatesEstm.omega_b = timeseries(double(states_estimator_log(:,12:14)),states_estimator_timestamp);
    ts_StatesEstm.p_e = timeseries(double(states_estimator_log(:,15:17)),states_estimator_timestamp);
    ts_StatesEstm.p_WGS84 = timeseries(double(states_estimator_log(:,2:3)),states_estimator_timestamp);
    ts_StatesEstm.h_ground = timeseries(single(states_estimator_log(:,18)),states_estimator_timestamp);
    ts_StatesEstm.v_e = timeseries(double(states_estimator_log(:,19:21)),states_estimator_timestamp);
    ts_StatesEstm.a_e = timeseries(double(states_estimator_log(:,22:24)),states_estimator_timestamp);
    ts_StatesEstm.sigma = timeseries(single(states_estimator_log(:,27:32)),states_estimator_timestamp);
    ts_StatesEstm.reference_point = timeseries(double(states_estimator_log(:,4:8)),states_estimator_timestamp);
    ts_StatesEstm.status = timeseries(boolean(states_estimator_log(:,33:40)),states_estimator_timestamp);
    ts_StatesEstm.counters = timeseries(uint8(states_estimator_log(:,41:48)),states_estimator_timestamp);
    clear q_b_e  R_b_e  euler_ypr states_estimator_timestamp states_estimator_log
else
    ts_StatesEstm =[];
end


%% Controller Commands
if float_log_name
    ts_Cmds = [];
    ts_Float_logout = [];
    ts_Cmds.PxyCmd = timeseries(single(float_log(:,2:3)),float_log_timestamp);
    ts_Cmds.VxyCmd = timeseries(single(float_log(:,4:5)),float_log_timestamp);
    ts_Cmds.AxyESO = timeseries(single(float_log(:,6:7)),float_log_timestamp);
    ts_Cmds.PzCmd = timeseries(single(float_log(:,8)),float_log_timestamp);
    ts_Cmds.VzCmd = timeseries(single(float_log(:,9)),float_log_timestamp);
    ts_Cmds.AzESO = timeseries(single(float_log(:,10)),float_log_timestamp);
    ts_Cmds.RPYCmd = timeseries(single(float_log(:,11:13)),float_log_timestamp);
    ts_Cmds.OmgCmd = timeseries(single(float_log(:,14:16)),float_log_timestamp);
    ts_Cmds.dOmgESO = timeseries(single(float_log(:,17:19)),float_log_timestamp);
    ts_Cmds.TauT = timeseries(single(float_log(:,20)),float_log_timestamp);
    ts_Float_logout = timeseries(single(float_log(:,2:end)),float_log_timestamp);
    clear float_log_timestamp float_log
else
    ts_Cmds = [];
    ts_Float_logout = [];
end

%% 状态位log
if uint32_log_name
    ts_FlagStates =[];
    ts_Uint32_logout = [];
    state_1 = uint32_log(:,2);
    arm = [];
    on_grd = [];
    for i = 1:length(state_1)
        arm(i) = bitand(state_1(i),2^0);
        on_grd(i) = bitand(state_1(i),2^1);
    end
    ts_FlagStates.arm = timeseries(boolean(arm),uint32_log_timestamp);
    ts_FlagStates.on_grd = timeseries(boolean(on_grd),uint32_log_timestamp);
    ts_FlagStates.control_mode  = timeseries(uint32(uint32_log(:,3)),uint32_log_timestamp);
    ts_Uint32_logout = timeseries(uint32(uint32_log(:,2:end)),uint32_log_timestamp);
    clear state_1 arm  on_grd uint32_log_timestamp  uint32_log
else
    ts_FlagStates =[];
    ts_Uint32_logout = [];
end

%% command mission指令
if command_mission_name
    ts_command_mission = [];
    ts_command_mission.wp_prev = timeseries(double(command_mission_log(:,2:4)),command_mission_timestamp);
    ts_command_mission.wp_curr = timeseries(double(command_mission_log(:,5:7)),command_mission_timestamp);
    ts_command_mission.wp_next = timeseries(double(command_mission_log(:,8:10)),command_mission_timestamp);
    ts_command_mission.wp_num = timeseries(uint32(command_mission_log(:,11)),command_mission_timestamp);
    ts_command_mission.wp_idx = timeseries(uint32(command_mission_log(:,12)),command_mission_timestamp);
    ts_command_mission.speed_set = timeseries(single(command_mission_log(:,13)),command_mission_timestamp);
    ts_command_mission.wp_type = timeseries(uint8(command_mission_log(:,14)),command_mission_timestamp);
    ts_command_mission.timestamp = timeseries(uint64(command_mission_timestamp*1e6),command_mission_timestamp);
else
    ts_command_mission.wp_prev = timeseries(double(zeros(3,1)),command_mission_timestamp);
    ts_command_mission.wp_curr = timeseries(double(zeros(3,1)),command_mission_timestamp);
    ts_command_mission.wp_next = timeseries(double(zeros(3,1)),command_mission_timestamp);
    ts_command_mission.wp_num = timeseries(uint32(0),command_mission_timestamp);
    ts_command_mission.wp_idx = timeseries(uint32(0),command_mission_timestamp);
    ts_command_mission.speed_set = timeseries(single(0),command_mission_timestamp);
    ts_command_mission.wp_type = timeseries(uint8(0),command_mission_timestamp);
    ts_command_mission.timestamp = timeseries(uint64(command_mission_timestamp*1e6),command_mission_timestamp);
end
clear command_mission_log command_mission_timestamp

%% command ground station指令
if command_gs_name
    ts_command_gs = [];
    ts_command_gs.target = timeseries(double(command_gs_log(:,2:4)),command_gs_timestamp);
    ts_command_gs.command = timeseries(uint32(command_gs_log(:,5)),command_gs_timestamp);
    ts_command_gs.heading = timeseries(single(command_gs_log(:,6)),command_gs_timestamp);
    ts_command_gs.servo_cmd = timeseries(uint8(command_gs_log(:,7)),command_gs_timestamp);
    ts_command_gs.timestamp = timeseries(uint64(command_gs_timestamp*1e6),command_gs_timestamp);
else
    ts_command_gs.target = timeseries(double(zeros(3,1)),command_gs_timestamp);
    ts_command_gs.command = timeseries(uint32(0),command_gs_timestamp);
    ts_command_gs.heading = timeseries(single(0),command_gs_timestamp);
    ts_command_gs.servo_cmd = timeseries(uint8(0),command_gs_timestamp);
    ts_command_gs.timestamp = timeseries(uint64(0),command_gs_timestamp);
end
clear command_gs_log command_gs_timestamp

%% command cloud指令
if command_cloud_name
    ts_command_cloud = [];
    ts_command_cloud.target = timeseries(double(command_cloud_log(:,2:4)),command_cloud_timestamp);
    ts_command_cloud.command = timeseries(uint32(command_cloud_log(:,5)),command_cloud_timestamp);
    ts_command_cloud.heading = timeseries(single(command_cloud_log(:,6)),command_cloud_timestamp);
    ts_command_cloud.servo_cmd = timeseries(uint8(command_cloud_log(:,7)),command_cloud_timestamp);
    ts_command_cloud.timestamp = timeseries(uint64(command_cloud_timestamp*1e6),command_cloud_timestamp);
else
    ts_command_cloud.target = timeseries(double(zeros(3,1)),command_cloud_timestamp);
    ts_command_cloud.command = timeseries(uint32(0),command_cloud_timestamp);
    ts_command_cloud.heading = timeseries(single(0),command_cloud_timestamp);
    ts_command_cloud.servo_cmd = timeseries(uint8(0),command_cloud_timestamp);
    ts_command_cloud.timestamp = timeseries(uint64(0),command_cloud_timestamp);
end
    clear command_cloud_log command_cloud_timestamp

%% marker location指令
if marker_file_name
    ts_marker = [];
    ts_marker.x_rel = timeseries(single(marker_location_log(:,2)),marker_location_timestamp);
    ts_marker.y_rel = timeseries(single(marker_location_log(:,3)),marker_location_timestamp);
    ts_marker.z_rel = timeseries(single(marker_location_log(:,4)),marker_location_timestamp);
    ts_marker.yaw_rel = timeseries(single(marker_location_log(:,7)),marker_location_timestamp);
    ts_marker.timestamp = timeseries(double(marker_location_timestamp*1e6),marker_location_timestamp);
else
    ts_marker.x_rel = timeseries(single(0),marker_location_timestamp);
    ts_marker.y_rel = timeseries(single(0),marker_location_timestamp);
    ts_marker.z_rel = timeseries(single(0),marker_location_timestamp);
    ts_marker.yaw_rel = timeseries(single(0),marker_location_timestamp);
    ts_marker.timestamp = timeseries(double(0),marker_location_timestamp);
end

clear marker_location_log marker_location_timestamp

%% mission status指令
% warning_info_timestamp = warning_info_log(:,1) * 1e-6;
if mission_status_name
    ts_mission_status = [];
    ts_mission_status.wp_count = timeseries(uint32(mission_status_log(:,2)),mission_status_timestamp);
    ts_mission_status.servo_cmd = timeseries(single(mission_status_log(:,3)),mission_status_timestamp);
    ts_mission_status.action_id = timeseries(uint8(mission_status_log(:,4)),mission_status_timestamp);
    ts_mission_status.timestamp = timeseries(uint64(mission_status_timestamp*1e6),mission_status_timestamp);
    clear mission_status_log mission_status_timestamp
else
    ts_mission_status = [];
end

%% ESC 信息
if ESC_file_name
    ts_ESC = [];
    ts_ESC.rpm1 = timeseries(single(ESC_log(:,10)),ESC_timestamp);
    ts_ESC.status1 = timeseries(uint16(ESC_log(:,12)),ESC_timestamp);
    ts_ESC.rpm2 = timeseries(single(ESC_log(:,16)),ESC_timestamp);
    ts_ESC.status2 = timeseries(uint16(ESC_log(:,18)),ESC_timestamp);
    ts_ESC.rpm3 = timeseries(single(ESC_log(:,22)),ESC_timestamp);
    ts_ESC.status3 = timeseries(uint16(ESC_log(:,24)),ESC_timestamp);
    ts_ESC.rpm4 = timeseries(single(ESC_log(:,28)),ESC_timestamp);
    ts_ESC.status4 = timeseries(uint16(ESC_log(:,30)),ESC_timestamp);
    ts_ESC.timestamp = timeseries(uint64(ESC_timestamp*1e6),ESC_timestamp);
    clear ESC_log ESC_timestamp
else
    ts_ESC = [];
end

%% Warning Info
% if warning_file_name
%     ts_WarningInfo = [];
%     
%     ts_WarningInfo.arm_error.Gyro_invalid = timeseries(boolean(bitand(warning_info_log(:,2),2^0)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.Accel_invalid = timeseries(boolean(bitand(warning_info_log(:,2),2^1)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.Baro_invalid = timeseries(boolean(bitand(warning_info_log(:,2),2^2)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.TOF_invalid = timeseries(boolean(bitand(warning_info_log(:,2),2^3)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.GNSS_loss = timeseries(boolean(bitand(warning_info_log(:,2),2^4)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.mag_loss = timeseries(boolean(bitand(warning_info_log(:,2),2^5)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.VIO_loss = timeseries(boolean(bitand(warning_info_log(:,2),2^6)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.battery_loss = timeseries(boolean(bitand(warning_info_log(:,2),2^7)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.P_V_invalid = timeseries(boolean(bitand(warning_info_log(:,2),2^8)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.not_standstill = timeseries(boolean(bitand(warning_info_log(:,2),2^9)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.not_attitude_ready = timeseries(boolean(bitand(warning_info_log(:,2),2^10)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.not_esc_ready = timeseries(boolean(bitand(warning_info_log(:,2),2^11)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.critical_battery = timeseries(boolean(bitand(warning_info_log(:,2),2^12)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.tilt_20 = timeseries(boolean(bitand(warning_info_log(:,2),2^13)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.not_safety = timeseries(boolean(bitand(warning_info_log(:,2),2^14)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.MCU_lost = timeseries(boolean(bitand(warning_info_log(:,2),2^15)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.cpu_over_load = timeseries(boolean(bitand(warning_info_log(:,2),2^16)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.kill_switch = timeseries(boolean(bitand(warning_info_log(:,2),2^17)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.not_onground = timeseries(boolean(bitand(warning_info_log(:,2),2^18)),warning_info_timestamp);
%     ts_WarningInfo.arm_error.servo_not_ok = timeseries(boolean(bitand(warning_info_log(:,2),2^19)),warning_info_timestamp);
%     
%     ts_WarningInfo.NAV_error.Gyro_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^0)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.Accel_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^1)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.GNSS_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^2)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.TOF_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^3)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.baro_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^4)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.mag_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^5)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.VIO_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^6)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.xy_velocity_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^7)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.xy_position_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^8)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.LLA_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^9)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.z_position_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^10)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.battery_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^11)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.marker_loss = timeseries(boolean(bitand(warning_info_log(:,3),2^12)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.flag_TiltOver = timeseries(boolean(bitand(warning_info_log(:,3),2^13)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.reject_alt_control = timeseries(boolean(bitand(warning_info_log(:,3),2^14)),warning_info_timestamp);
%     ts_WarningInfo.NAV_error.reject_pos_control = timeseries(boolean(bitand(warning_info_log(:,3),2^15)),warning_info_timestamp);
%     
%     ts_WarningInfo.status_error.gyro_overheat = timeseries(boolean(bitand(warning_info_log(:,4),2^0)),warning_info_timestamp);
%     ts_WarningInfo.status_error.acc_overheat = timeseries(boolean(bitand(warning_info_log(:,4),2^1)),warning_info_timestamp);
%     ts_WarningInfo.status_error.baro_overheat = timeseries(boolean(bitand(warning_info_log(:,4),2^2)),warning_info_timestamp);
%     ts_WarningInfo.status_error.RC_Lost = timeseries(boolean(bitand(warning_info_log(:,4),2^3)),warning_info_timestamp);
%     ts_WarningInfo.status_error.data_link_lost = timeseries(boolean(bitand(warning_info_log(:,4),2^4)),warning_info_timestamp);
%     ts_WarningInfo.status_error.tx2_lost = timeseries(boolean(bitand(warning_info_log(:,4),2^5)),warning_info_timestamp);
%     ts_WarningInfo.status_error.cpu_over_load = timeseries(boolean(bitand(warning_info_log(:,4),2^6)),warning_info_timestamp);
%     ts_WarningInfo.status_error.flag_KillSwitch = timeseries(boolean(bitand(warning_info_log(:,4),2^7)),warning_info_timestamp);
%     ts_WarningInfo.status_error.large_wind = timeseries(boolean(bitand(warning_info_log(:,4),2^8)),warning_info_timestamp);
%     ts_WarningInfo.status_error.low_battery = timeseries(boolean(bitand(warning_info_log(:,4),2^9)),warning_info_timestamp);
%     ts_WarningInfo.status_error.critical_battery = timeseries(boolean(bitand(warning_info_log(:,4),2^10)),warning_info_timestamp);
%     ts_WarningInfo.status_error.track_error = timeseries(boolean(bitand(warning_info_log(:,4),2^11)),warning_info_timestamp);
%     ts_WarningInfo.status_error.MotorLowSaturation = timeseries(boolean(bitand(warning_info_log(:,4),2^12)),warning_info_timestamp);
%     ts_WarningInfo.status_error.MotorHighSaturation = timeseries(boolean(bitand(warning_info_log(:,4),2^13)),warning_info_timestamp);  
%     
%     ts_WarningInfo.timestamp = timeseries(uint64(warning_info_timestamp*1e6),warning_info_timestamp);
%     clear warning_info_timestamp
% else
%     ts_WarningInfo = [];
% end

%% Flight Status 信息
if flight_status_file_name
    ts_flight_status = [];
    ts_flight_status.home_lat = timeseries(double(flight_status_log(:,2)),flight_status_timestamp);
    ts_flight_status.home_long = timeseries(double(flight_status_log(:,3)),flight_status_timestamp);
    ts_flight_status.mode_mavlink = timeseries(uint32(flight_status_log(:,4)),flight_status_timestamp);
    ts_flight_status.arm_state = timeseries(boolean(flight_status_log(:,5)),flight_status_timestamp);
    ts_flight_status.arm_esc = timeseries(boolean(flight_status_log(:,6)),flight_status_timestamp);
    ts_flight_status.on_grd_state = timeseries(boolean(flight_status_log(:,7)),flight_status_timestamp);
    ts_flight_status.control_mode = timeseries(uint8(flight_status_log(:,8)),flight_status_timestamp);
    ts_flight_status.link_prio = timeseries(uint8(flight_status_log(:,9)),flight_status_timestamp);
    ts_flight_status.timestamp = timeseries(uint64(flight_status_timestamp*1e6),flight_status_timestamp);
    clear flight_status_log flight_status_timestamp
else
    ts_flight_status = [];
end

%% CPU load 信息
if CPU_file_name
    ts_cpu_load = [];
    ts_cpu_load.load = timeseries(double(CPU_log(:,2)),cpu_load_timestamp);
    ts_cpu_load.ram_usage = timeseries(double(CPU_log(:,3)),cpu_load_timestamp);
    clear CPU_log cpu_load_timestamp
else
    ts_cpu_load = [];
end

%% Vehicle Command指令
if VehicleCmd_file_name
    ts_VehicleCmd = [];
    ts_VehicleCmd.command = timeseries(uint16(VehicleCmd_log(:,9)),VehicleCmd_timestamp);
    ts_VehicleCmd.source_system = timeseries(uint8(VehicleCmd_log(:,12)),VehicleCmd_timestamp);
    ts_VehicleCmd.param1 = timeseries(single(VehicleCmd_log(:,4)),VehicleCmd_timestamp);
    ts_VehicleCmd.param2 = timeseries(single(VehicleCmd_log(:,5)),VehicleCmd_timestamp);
    ts_VehicleCmd.param3 = timeseries(single(VehicleCmd_log(:,6)),VehicleCmd_timestamp);
    ts_VehicleCmd.param4 = timeseries(single(VehicleCmd_log(:,7)),VehicleCmd_timestamp);
    ts_VehicleCmd.param5 = timeseries(double(VehicleCmd_log(:,2)),VehicleCmd_timestamp);
    ts_VehicleCmd.param6 = timeseries(double(VehicleCmd_log(:,3)),VehicleCmd_timestamp);
    ts_VehicleCmd.param7 = timeseries(single(VehicleCmd_log(:,8)),VehicleCmd_timestamp);
    ts_VehicleCmd.timestamp = timeseries(double(VehicleCmd_timestamp*1e6),VehicleCmd_timestamp);
else
    ts_VehicleCmd.command = timeseries(uint16(0),VehicleCmd_timestamp);
    ts_VehicleCmd.source_system = timeseries(uint8(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param1 = timeseries(single(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param2 = timeseries(single(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param3 = timeseries(single(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param4 = timeseries(single(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param5 = timeseries(double(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param6 = timeseries(double(0),VehicleCmd_timestamp);
    ts_VehicleCmd.param7 = timeseries(single(0),VehicleCmd_timestamp);
    ts_VehicleCmd.timestamp = timeseries(double(0),VehicleCmd_timestamp);
end

clear VehicleCmd_log VehicleCmd_timestamp

%% Trajectory指令
if Trajectory_file_name
    ts_Trajectory = [];
    ts_Trajectory.traj_p = timeseries(double(Trajectory_log(:,11:13)),Trajectory_timestamp);
    ts_Trajectory.traj_v = timeseries(single(Trajectory_log(:,14:16)),Trajectory_timestamp);
    ts_Trajectory.traj_a = timeseries(single(Trajectory_log(:,17:19)),Trajectory_timestamp);
    ts_Trajectory.traj_yaw = timeseries(single(Trajectory_log(:,20)),Trajectory_timestamp);
    ts_Trajectory.traj_yaw_dot = timeseries(single(Trajectory_log(:,21)),Trajectory_timestamp);
    ts_Trajectory.traj_index = timeseries(uint32(Trajectory_log(:,10)),Trajectory_timestamp);
    ts_Trajectory.global_traj_enable = timeseries(boolean(Trajectory_log(:,3)),Trajectory_timestamp);
    ts_Trajectory.timestamp = timeseries(double(Trajectory_timestamp*1e6),Trajectory_timestamp);
else
    ts_Trajectory.traj_p = timeseries(double(zeros(3,1)),Trajectory_timestamp);
    ts_Trajectory.traj_v = timeseries(single(zeros(3,1)),Trajectory_timestamp);
    ts_Trajectory.traj_a = timeseries(single(zeros(3,1)),Trajectory_timestamp);
    ts_Trajectory.traj_yaw = timeseries(single(0),Trajectory_timestamp);
    ts_Trajectory.traj_yaw_dot = timeseries(single(0),Trajectory_timestamp);
    ts_Trajectory.traj_index = timeseries(uint32(0),Trajectory_timestamp);
    ts_Trajectory.global_traj_enable = timeseries(boolean(0),Trajectory_timestamp);
    ts_Trajectory.timestamp = timeseries(double(0),Trajectory_timestamp);
end

clear Trajectory_log Trajectory_timestamp

%% Battery信息
if Battery_file_name
    ts_Battery = [];
    ts_Battery.voltagead = timeseries(single(Battery_log(:,2)),Battery_timestamp);
    ts_Battery.currentad = timeseries(single(Battery_log(:,3)),Battery_timestamp);
    ts_Battery.voltage = timeseries(single(Battery_log(:,4)),Battery_timestamp);
    ts_Battery.current = timeseries(single(Battery_log(:,5)),Battery_timestamp);
    ts_Battery.soc = timeseries(single(Battery_log(:,6)),Battery_timestamp);
    ts_Battery.bat_type = timeseries(uint8(Battery_log(:,8)),Battery_timestamp);
    ts_Battery.timestamp = timeseries(double(Battery_log(:,1)),Battery_timestamp);
else
    ts_Battery.voltage = timeseries(single(24),Battery_timestamp);
    ts_Battery.current = timeseries(single(0),Battery_timestamp);
    ts_Battery.bat_type = timeseries(uint8(0),Battery_timestamp);
    ts_Battery.timestamp = timeseries(double(0),Battery_timestamp);
end

clear Battery_log Battery_timestamp


%% Safety信息
if Safety_file_name
    ts_Safety = [];
    ts_Safety.safety_available = timeseries(boolean(Safety_log(:,2)),Safety_timestamp);
    ts_Safety.safety_off = timeseries(boolean(Safety_log(:,3)),Safety_timestamp);
    ts_Safety.timestamp = timeseries(double(Safety_log(:,1)),Safety_timestamp);
else
    ts_Safety.safety_available = timeseries(boolean(1),Safety_timestamp);
    ts_Safety.safety_off = timeseries(boolean(1),Safety_timestamp);
    ts_Safety.timestamp = timeseries(double(0),Safety_timestamp);
end

clear Safety_log Safety_timestamp

%% servo信息
if servo_file_name
    ts_servo = [];
    ts_servo.angle1 = timeseries(single(servo_log(:,2)),servo_timestamp);
    ts_servo.angle2 = timeseries(single(servo_log(:,3)),servo_timestamp);
    ts_servo.error1 = timeseries(uint8(servo_log(:,4)),servo_timestamp);
    ts_servo.error2 = timeseries(uint8(servo_log(:,5)),servo_timestamp);
    ts_servo.timestamp = timeseries(double(servo_log(:,1)),servo_timestamp);
else
    ts_servo = [];
end

clear servo_log servo_timestamp



%%
clear RC_file_name PWM_file_name float_log_name uint32_log_name states_estimator_name...
    sensor_combined_name mag_file_name baro_file_name gnss_file_name distance_file_name...
    command_mission_name command_gs_name command_cloud_name marker_file_name mission_status_name...
    ESC_file_name i warning_file_name warning_info_log warning_info_timestamp flight_status_file_name...
    CPU_file_name VehicleCmd_file_name Trajectory_file_name Battery_file_name Safety_file_name j
    
