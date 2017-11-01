%scenario - interface - users
clc; clear all; close all;
tic
folder='./input/manipulation/';
output='./output/';
resize_value=512;
Ts=1/25;

file_list = dir(folder);
max_users=26;

s1_x=0.5;
s1_y=0.0;
s1_z=0.6284;
s1_rr=0.508768;
s1_rp=-0.021325;
s1_ry=0.669494;

s3_x=0.7059;
s3_y=0.0;
s3_z=0.594858;
s3_rr=0.681;
s3_rp=-0.0289;
s3_ry=0.63239;

pr2_corr_x=0.049931;
pr2_corr_y=-0.00161;
pr2_corr_z=-0.764212;

for n=1:length(file_list)
%for n=1:3
    if not( isempty(strfind(file_list(n).name,'.csv')) ) %not a csv file
        
        cvs_data = csvread([folder file_list(n).name],1,0);
        file_name = strsplit(file_list(n).name,'.');
        file_parameters = strsplit( file_name{1} ,'_'); %Ex: navigation_u02_s1_i2   robot_navigation_u0#_s#_i# {2}:u02 - {3}:s1 - {4}:i2 
        
        %s1.i1.max_length=
        try
            eval([ file_parameters{4} '.' file_parameters{5} '.max_length = max([' file_parameters{4} '.' file_parameters{5} '.max_length length(cvs_data(:,6))]);']);
        catch
            eval([ file_parameters{4} '.' file_parameters{5} '.max_length = length(cvs_data(:,6));']);
        end

        %s1.i1.u02.ABC = ... 
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.secs = cvs_data(:,1);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.user = cvs_data(:,2);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.scenario = cvs_data(:,3);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.interface = cvs_data(:,4);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.x = cvs_data(:,5)-pr2_corr_x;']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.y = cvs_data(:,6)-pr2_corr_y;']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.z = cvs_data(:,7)-pr2_corr_z;']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.roll = cvs_data(:,8);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pitch = cvs_data(:,9);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.yaw = cvs_data(:,10);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vx = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.x);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vy = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.y);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vz = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.z);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vroll = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.roll);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vpitch = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pitch);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vyaw = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.yaw);']);
        
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.v = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vx).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vy).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vz).^2);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.spectralarc = SpectralArcLength( ' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.v, Ts );']);
        
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ax = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vx);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ay = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vy);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.az = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vz);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.aroll = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vroll);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.apitch = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vpitch);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ayaw = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.vyaw);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d = sqrt( (cvs_data(:,5)-pr2_corr_x-' file_parameters{4} '_x).^2 + (cvs_data(:,6)-pr2_corr_y-' file_parameters{4} '_y).^2 + (cvs_data(:,7)-pr2_corr_z-' file_parameters{4} '_z).^2 );']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dA = sqrt( (cvs_data(:,8)-' file_parameters{4} '_rr).^2 + (cvs_data(:,9)-' file_parameters{4} '_rp).^2 + (cvs_data(:,10)-' file_parameters{4} '_ry).^2 );']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.davg = tsmovavg(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d, ''s'', 150, 1);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dAavg = tsmovavg(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dA, ''s'', 150, 1);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd = abs(diff( ' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.davg ));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ddA = abs(diff( ' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dAavg ));']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_x = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.x,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_y = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.y,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_z = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.y,resize_value,length(cvs_data(:,6)));']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_roll = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.roll,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_pitch = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pitch,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_yaw = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.yaw,resize_value,length(cvs_data(:,6)));']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_d = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_davg = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.davg,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_dd = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd,resize_value,length(cvs_data(:,6)));']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_dA = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dA,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_dAavg = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dAavg,resize_value,length(cvs_data(:,6)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_ddA = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ddA,resize_value,length(cvs_data(:,6)));']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.mean_dd = mean(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd(150:end)) ;']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.mean_ddA = mean(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ddA(150:end)) ;']);
                
    end
end

clear scenario1 scenario2 cvs_data file_name file_parameters m n n_i current_x current_y error path_x path_y min_value index; 

for s=1:3
    if s==2
        continue;
    end
    for i=1:3
        eval([ 's' num2str(s) '.i' num2str(i) '.all_x = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_y = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_z = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_roll = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_pitch = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_yaw = zeros(resize_value,max_users);']);
        
        eval([ 's' num2str(s) '.i' num2str(i) '.all_d = zeros(resize_value,max_users);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.all_dA = zeros(resize_value,max_users);']);
        
        eval([ 's' num2str(s) '.i' num2str(i) '.spectralarc = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.mean_dd = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.mean_ddA = zeros(max_users,1);']);
        for u=2:max_users
            user=num2str(u);
            if u<10
                user=['0' num2str(u)];
            end
            try
               eval([ 's' num2str(s) '.i' num2str(i) '.all_x(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_x;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_y(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_y;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_z(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_z;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_d(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_d;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_roll(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_roll;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_pitch(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_pitch;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_yaw(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_yaw;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.all_dA(:,' num2str(u) ') = s' num2str(s) '.i' num2str(i) '.u' user '.resampled_d;']);
               
               eval([ 's' num2str(s) '.i' num2str(i) '.spectralarc(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.spectralarc;'])
               eval([ 's' num2str(s) '.i' num2str(i) '.mean_dd(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.mean_dd;'])
               eval([ 's' num2str(s) '.i' num2str(i) '.mean_ddA(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.mean_ddA;'])
               
               eval([ 'norm_ax = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_ay = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_az = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_roll = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_pitch = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               eval([ 'norm_yaw = zeros(s' num2str(s) '.i' num2str(i) '.max_length,1);']);
               
               eval([ 'norm_ax(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.ax)) = s' num2str(s) '.i' num2str(i) '.u' user '.ax;']);
               eval([ 'norm_ay(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.ay)) = s' num2str(s) '.i' num2str(i) '.u' user '.ay;']);
               eval([ 'norm_az(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.az)) = s' num2str(s) '.i' num2str(i) '.u' user '.az;']);
               
               eval([ 'norm_aroll(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.aroll)) = s' num2str(s) '.i' num2str(i) '.u' user '.aroll;']);
               eval([ 'norm_apitch(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.apitch)) = s' num2str(s) '.i' num2str(i) '.u' user '.apitch;']);
               eval([ 'norm_ayaw(1:length(s' num2str(s) '.i' num2str(i) '.u' user '.ayaw)) = s' num2str(s) '.i' num2str(i) '.u' user '.ayaw;']);
               
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zax = norm_ax;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zay = norm_ay;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zaz = norm_az;']);
               
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zaroll = norm_aroll;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zapitch = norm_apitch;']);
               eval([ 's' num2str(s) '.i' num2str(i) '.u' user '.zayaw = norm_ayaw;']);

            catch
            end
        end
    end
end

xlswrite([output 'man_residual_s1_i1'],s1.i1.mean_dd);
xlswrite([output 'man_residual_s1_i2'],s1.i2.mean_dd);
xlswrite([output 'man_residual_s1_i3'],s1.i3.mean_dd);
xlswrite([output 'man_residual_s3_i1'],s3.i1.mean_dd);
xlswrite([output 'man_residual_s3_i2'],s3.i2.mean_dd);
xlswrite([output 'man_residual_s3_i3'],s3.i3.mean_dd);

xlswrite([output 'man_angle_residual_s1_i1'],s1.i1.mean_ddA);
xlswrite([output 'man_angle_residual_s1_i2'],s1.i2.mean_ddA);
xlswrite([output 'man_angle_residual_s1_i3'],s1.i3.mean_ddA);
xlswrite([output 'man_angle_residual_s3_i1'],s3.i1.mean_ddA);
xlswrite([output 'man_angle_residual_s3_i2'],s3.i2.mean_ddA);
xlswrite([output 'man_angle_residual_s3_i3'],s3.i3.mean_ddA);

xlswrite([output 'man_spectral_s1_i1'],s1.i1.spectralarc);
xlswrite([output 'man_spectral_s1_i2'],s1.i2.spectralarc);
xlswrite([output 'man_spectral_s1_i3'],s1.i3.spectralarc);
xlswrite([output 'man_spectral_s3_i1'],s3.i1.spectralarc);
xlswrite([output 'man_spectral_s3_i2'],s3.i2.spectralarc);
xlswrite([output 'man_spectral_s3_i3'],s3.i3.spectralarc);

toc;

%-------------
return;


%s1
figure(7)
plot(s1.i3.u15.d)
figure(8)
plot3(s1.i3.u15.x,s1.i3.u15.y,s1.i3.u15.z)
figure(9)
plot(s1.i3.u15.davg,'r')
hold on;
plot(s1.i3.u15.d,'b')
figure(10)
plot(s1.i3.u15.dd,'r')

figure(7)
plot(s1.i3.u15.dA)
figure(8)
plot3(s1.i3.u15.roll,s1.i3.u15.pitch,s1.i3.u15.yaw)
figure(9)
plot(s1.i3.u15.dAavg,'r')
hold on;
plot(s1.i3.u15.dA,'b')
figure(10)
plot(s1.i3.u15.ddA,'r')


%%for angles
figure(7)
plot(s3.i2.u19.d)
figure(8)
plot3(s3.i2.u19.x,s3.i2.u19.y,s3.i2.u19.z)
figure(9)
plot(s3.i2.u19.davg,'r')
hold on;
plot(s3.i2.u19.d,'b')
figure(10)
plot(s3.i2.u19.dd,'r')

figure(7)
plot(s3.i2.u19.dA)
figure(8)
plot3(s3.i2.u19.roll,s3.i2.u19.pitch,s3.i2.u19.yaw)
figure(9)
plot(s3.i2.u19.dAavg,'r')
hold on;
plot(s3.i2.u19.dA,'b')
figure(10)
plot(s3.i2.u19.ddA,'r')



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



