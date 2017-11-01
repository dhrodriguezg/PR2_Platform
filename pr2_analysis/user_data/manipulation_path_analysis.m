%scenario - interface - users
clc; clear all; close all;
tic
folder='./input/manipulation/';
output='./output/';
resize_value=512;
Ts=1/100;

file_list = dir(folder);
max_users=27;

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
        file_parameters = strsplit( file_name{1} ,'_'); %Ex: nmanipulation_u0#_s#_i# {2}:u02 - {3}:s1 - {4}:i2 
        
        %s1.i1.max_length=
        try
            eval([ file_parameters{4} '.' file_parameters{5} '.max_length = max([' file_parameters{4} '.' file_parameters{5} '.max_length length(cvs_data(:,6))]);']);
        catch
            eval([ file_parameters{4} '.' file_parameters{5} '.max_length = length(cvs_data(:,6));']);
        end
        
        
        %s1.i1.u02.ABC = ... secs,camera,grasp,radius,sphere_x,sphere_y,pitch
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.secs = cvs_data(:,1);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.camera = cvs_data(:,2);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.grasp = cvs_data(:,3);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.radius = cvs_data(:,4);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.sphere_x = cvs_data(:,5);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.sphere_y = cvs_data(:,6);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pitch = cvs_data(:,7);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_grasp = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.grasp);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_radius = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.radius);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_sphere_x = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.sphere_x);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_sphere_y = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.sphere_y);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_pitch = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pitch);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_grasp = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_grasp);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_radius = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_radius);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_sphere_x = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_sphere_x);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_sphere_y = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_sphere_y);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_pitch = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_pitch);']);

        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.v = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_grasp).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_radius).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_sphere_x).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_sphere_y).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_pitch).^2 );']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.spectralarc = SpectralArcLength( ' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.v, Ts );']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.a = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_grasp).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_radius).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_sphere_x).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_sphere_y).^2 + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_pitch).^2 );']);        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_a = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.a,resize_value,length(cvs_data(:,1)));']);
    end
end

clear cvs_data file_name file_parameters;

for s=1:3
    for i=1:3
        eval([ 's' num2str(s) '.i' num2str(i) '.a = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.spectralarc = zeros(max_users,1);']);
        for u=2:max_users
            user=num2str(u);
            if u<10
                user=['0' num2str(u)];
            end
            try
                eval([ 's' num2str(s) '.i' num2str(i) '.a(' num2str(u) ') =  mean(s' num2str(s) '.i' num2str(i) '.u' user '.resampled_a);'])
               eval([ 's' num2str(s) '.i' num2str(i) '.spectralarc(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.spectralarc;'])
            catch
            end
        end
    end
end

xlswrite([output 'man_a_s1_i1'],s1.i1.a);
xlswrite([output 'man_a_s1_i2'],s1.i2.a);
xlswrite([output 'man_a_s1_i3'],s1.i3.a);
xlswrite([output 'man_a_s3_i1'],s3.i1.a);
xlswrite([output 'man_a_s3_i2'],s3.i2.a);
xlswrite([output 'man_a_s3_i3'],s3.i3.a);

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



