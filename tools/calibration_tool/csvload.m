clear all;

%%
%通过界面选择文件
log_path = uigetdir('*.csv','please Select CSV file ');
log_pathcsv = [log_path,'/*.csv'];

file = dir(log_pathcsv);    % change the folder
%file1 = dir('C:/Users/wk/Desktop/test-logg/swift/swift-2-20-2/*.csv');    % change the folder

%%

len_file = length(file);

C = {};

for i = 1 : 8
    C{i} = file(i).name;
end

len_name = length(C{1});

for i = 1 : len_name
    S = strncmp(file(1).name, C, i);
    if ~all(S)
        index = i-1;
        break
    end
end

for i = 1:len_file
    oldname = file(i).name;
    file(i).oldname = oldname;
    newname = oldname(index + 1:end);
    file(i).name = newname;
end

clear C S index newname oldname;

%%
start_time = 1e12;
end_time = 1e12;
err_start_time = zeros(len_file, 1);
count = 1;
for i = 1 :len_file
    temp = importdata([file(i).folder, '/', file(i).oldname]);
    %     if strcmp(cell2mat(temp.textdata(1)), 'timestamp') && ~strcmp(file(i).name,...
    %     'commander_state_0.csv') && ~strcmp(file(i).name, 'mission_result_0.csv')
    %     if strcmp(cell2mat(temp.textdata(1)), 'timestamp') && isempty(strfind(file(i).name, 'mission')) ...
    %             && isempty(strfind(file(i).name, 'commander')) && length(temp.data(:,1)) > 200
    if strcmp(cell2mat(temp.textdata(1)), 'timestamp') && (length(temp.data(:,1)) > 1 &&temp.data(1,1) ~= temp.data(2,1)) ...
            && length(temp.data(:,1)) > 200
        temp_start_time = temp.data(1,1);
        temp_end_time = temp.data(end,1);
        if temp_start_time < start_time
            start_time = temp_start_time;
            end_time = temp_end_time;
            Var = file(i).name;
            max_time = (end_time - start_time) * 1e-6;
        end
        
        err_start_time(i) = (temp_start_time - start_time) * 1e-6;  % to be modified
        if err_start_time(i) > 0.5
            C{count} = file(i).name;
            count = count + 1;
        end
    end
end



clear temp_start_time temp_end_time end_time j;


%%

for i = 1 : len_file
    temp = importdata([file(i).folder, '/', file(i).oldname]);
    temp_name = file(i).name(1:end-4);
%     len_var = length(temp.textdata); % last column(s) with NaN data in .csv will cause l(textdata)>l(data) 
    len_var = length(temp.data(1,:));
    
    a = ['ts_', temp_name];
    
    eval([a, '= struct();']);
    
    if ~isempty(temp.data(:,1)) && strcmp(cell2mat(temp.textdata(1)), 'timestamp')
        
        timestamp = double(temp.data(:,1) - start_time) * 1e-6;
        timestamp = round(timestamp*1000)*1e-3;
        
        for i = 1 : len_var
            temp.textdata(i) = strrep(temp.textdata(i),'[','_');
            temp.textdata(i) = strrep(temp.textdata(i),']','');
            temp.textdata(i) = strrep(temp.textdata(i),'.','_');
            b = ['ts_', cell2mat(temp.textdata(i))];
            c = [temp_name, '_', cell2mat(temp.textdata(i))];
            %             eval([b, '= timeseries(temp.data(:,i),timestamp);']);
            eval([a, '.', b, '= timeseries(fillmissing(temp.data(:,i),"constant",0), timestamp, "Name", c);']);
        end
    end
    
end

clear a b c file i len_file len_var temp temp_name timestamp;

%%