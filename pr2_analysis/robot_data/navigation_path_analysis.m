%scenario - interface - users
clc; clear all; close all;
tic
folder='./input/navigation/';
output='./output/';
resize_value=512;

scenario1 = xlsread('./input/scenario_1_ref_path.xlsx');
scenario1_path_x=scenario1(:,1)';
scenario1_path_y=scenario1(:,2)';
scenario2 = xlsread('./input/scenario_2_ref_path.xlsx');
scenario2_path_x=scenario2(:,1)';
scenario2_path_y=scenario2(:,2)';

file_list = dir(folder);
max_users=26;

s1_x=3;
s1_y=-4;
s2_x=3.7;
s2_y=0.14;

for n=1:length(file_list)
%for n=1:3
    if not( isempty(strfind(file_list(n).name,'.csv')) ) %not a csv file
        
        cvs_data = csvread([folder file_list(n).name],1,0);
        file_name = strsplit(file_list(n).name,'.');
        file_parameters = strsplit( file_name{1} ,'_'); %Ex: navigation_u02_s1_i2    {2}:u02 - {3}:s1 - {4}:i2 
        
        %s1.i1.max_length=
        try
            eval([ file_parameters{3} '.' file_parameters{4} '.max_length = max([' file_parameters{3} '.' file_parameters{4} '.max_length length(cvs_data(:,6))]);']);
        catch
            eval([ file_parameters{3} '.' file_parameters{4} '.max_length = length(cvs_data(:,6));']);
        end

        %s1.i1.u02.ABC = ... 
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.secs = cvs_data(:,1);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.secs = cvs_data(:,1);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.nsecs = cvs_data(:,2);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.user = cvs_data(:,3);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.scenario = cvs_data(:,4);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.interface = cvs_data(:,5);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.x = cvs_data(:,6);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.y = cvs_data(:,7);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.yaw = cvs_data(:,8);']);
        
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.vx = diff(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.x);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.vy = diff(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.y);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.ax = diff(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.vx);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.ay = diff(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.vy);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.d = sqrt( (cvs_data(:,6)-' file_parameters{3} '_x).^2 + (cvs_data(:,7)-' file_parameters{3} '_y).^2 );']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.davg = tsmovavg(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.d, ''s'', 150, 1);']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.dd = abs(diff( ' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.davg ));']);
        
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.resampled_x = resample(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.x,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.resampled_y = resample(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.y,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.resampled_d = resample(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.d,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.resampled_davg = resample(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.davg,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.resampled_dd = resample(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.dd,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.mean_dd = mean(' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.dd(150:end)) ;']);
                
        i=100;
        cumulative_error=0;
        scenario_path_x=[];
        scenario_path_y=[];
        eval([ 'ux = ' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.x;']);
        eval([ 'uy = ' file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.y;']);
        if not( isempty(strfind(file_list(n).name,'_s1_')) )
            scenario_path_x=scenario1_path_x;
            scenario_path_y=scenario1_path_y;
        else
            scenario_path_x=scenario2_path_x;
            scenario_path_y=scenario2_path_y;
        end
        
        for m=1:length(ux)
            current_x=ux(m);
            current_y=uy(m);
            path_x=scenario_path_x(i-99:i+99);
            path_y=scenario_path_y(i-99:i+99);
            error=sqrt((path_x-current_x).^2+(path_y-current_y).^2);
            [min_value,index] = min(error);
            cumulative_error = cumulative_error + min_value/length(ux);
            n_i = i + index - 100; %true value
            if(n_i < 100 || n_i > length(scenario_path_x) - 99 )
                n_i=i;
            end
            i=n_i;
        end
        eval([ file_parameters{3} '.' file_parameters{4} '.' file_parameters{2} '.cumulative_error = cumulative_error;']);
    end
end

clear scenario1 scenario2 cvs_data file_name file_parameters ux uy scenario_path_x scenario_path_y cumulative_error i m n n_i current_x current_y error path_x path_y min_value index; 

for s=1:2
    for i=1:3
        eval([ 's' num2str(s) '.i' num2str(i) '.all_x = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_y = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_d = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.err = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.mean_dd = zeros(max_users,1);']);
        for u=2:max_users
            user=num2str(u);
            if u<10
                user=['0' num2str(u)];
            end
            try
               eval([ 's' num2str(s) '.i' num2str(i) '.all_x(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_x;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_y(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_y;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_d(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_d;']);
               
               eval([ 's' num2str(s) '.i' num2str(i) '.err(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.cumulative_error;'])
               eval([ 's' num2str(s) '.i' num2str(i) '.mean_dd(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.mean_dd;'])
               
               eval([ 'norm_ax = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_ay = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_ax(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.ax)) = s' num2str(s) '.i' num2str(i) '.u' user '.ax;']);
               eval([ 'norm_ay(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.ay)) = s' num2str(s) '.i' num2str(i) '.u' user '.ay;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zax = norm_ax;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zay = norm_ay;']);

            catch
            end
        end
    end
end

xlswrite([output 'nav_err_ref_path_s1_i1'],s1.i1.err);
xlswrite([output 'nav_err_ref_path_s1_i2'],s1.i2.err);
xlswrite([output 'nav_err_ref_path_s1_i3'],s1.i3.err);
xlswrite([output 'nav_err_ref_path_s2_i1'],s2.i1.err);
xlswrite([output 'nav_err_ref_path_s2_i2'],s2.i2.err);
xlswrite([output 'nav_err_ref_path_s2_i3'],s2.i3.err);

xlswrite([output 'nav_residual_s1_i1'],s1.i1.mean_dd);
xlswrite([output 'nav_residual_s1_i2'],s1.i2.mean_dd);
xlswrite([output 'nav_residual_s1_i3'],s1.i3.mean_dd);
xlswrite([output 'nav_residual_s2_i1'],s2.i1.mean_dd);
xlswrite([output 'nav_residual_s2_i2'],s2.i2.mean_dd);
xlswrite([output 'nav_residual_s2_i3'],s2.i3.mean_dd);


toc;

%each column is an user.....standar graphic
Users=ones(resize_value,max_users);
for n=2:max_users
    Users(:,n)=n;
end
figure(1)
h=plot3(Users,s1.i1.all_x,s1.i1.all_y);
%h=surf(Users,s1.i1.all_x,s1.i1.all_y);
%set(h, 'linestyle', 'none');
title('On-Screen Joysticks')
xlabel('Users');
ylabel('X');
zlabel('Y');
figure(2)
h=plot3(Users,s1.i2.all_x,s1.i2.all_y);
title('Gestures')
xlabel('Users');
ylabel('X');
zlabel('Y');
figure(3)
h=plot3(Users,s1.i3.all_x,s1.i3.all_y);
title('IMU')
xlabel('Users');
ylabel('X');
zlabel('Y');
%s2
figure(4)
h=plot3(Users,s2.i1.all_x,s2.i1.all_y);
title('On-Screen Joysticks')
xlabel('Users');
ylabel('X');
zlabel('Y');
figure(5)
h=plot3(Users,s2.i2.all_x,s2.i2.all_y);
title('Gestures')
xlabel('Users');
ylabel('X');
zlabel('Y');
figure(6)
h=plot3(Users,s2.i3.all_x,s2.i3.all_y);
title('IMU')
xlabel('Users');
ylabel('X');
zlabel('Y');

%-------------
return;

%s1
figure(7)
plot(s1.i3.u19.d)
figure(8)
plot(s1.i3.u19.x,s1.i3.u19.y)
figure(9)
plot(s1.i3.u19.davg,'r')
hold on;
plot(s1.i3.u19.d,'b')
figure(10)
plot(s1.i3.u19.dd,'r')

figure(7)
plot(s1.i2.u08.d)
figure(8)
plot(s1.i2.u08.x,s1.i2.u08.y)
figure(9)
plot(s1.i2.u08.davg,'r')
hold on;
plot(s1.i2.u08.d,'b')
figure(10)
plot(s1.i2.u08.dd,'r')

%s2
figure(7)
plot(s2.i3.u21.d)
figure(8)
plot(s2.i3.u21.x,s2.i3.u21.y)
figure(9)
plot(s2.i3.u21.davg,'r')
hold on;
plot(s2.i3.u21.d,'b')
figure(10)
plot(s2.i3.u21.dd,'r')

figure(7)
plot(s2.i1.u15.d)
figure(8)
plot(s2.i1.u15.x,s2.i1.u15.y)
figure(9)
plot(s2.i1.u15.davg,'r')
hold on;
plot(s2.i1.u15.d,'b')
figure(10)
plot(s2.i1.u15.dd,'r')

return;
%S1-I1=21
%S1-I2=14
%S1-I3=27
%S2-I1=6
%S2-I2=10
%S2-I3=29
for n=1:26
    figure(n);
    user=num2str(n);
        if n<10
            user=['0' num2str(n)];
        end
    try
        eval([ 'plot(s1.i1.u' user '.x , s1.i1.u' user '.y)']);
    catch
    end
end
%check s2,i3,u24,


