
%% open log file
[file, folder] = uigetfile({'*.bin'; '*.log'});
logfile = strcat(folder, file);

[header, log_msg] = blog_parser(logfile);

clear file;
clear folder;
clear logfile;
