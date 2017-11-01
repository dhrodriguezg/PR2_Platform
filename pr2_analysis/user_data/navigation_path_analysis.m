%scenario - interface - users
clc; clear all; close all;
tic
folder='./input/navigation/';
output='./output/';
resize_value=512;
Ts=1/100;

file_list = dir(folder);
max_users=27;

for n=1:length(file_list)
%for n=1:3
    if not( isempty(strfind(file_list(n).name,'.csv')) ) %not a csv file
        
        cvs_data = csvread([folder file_list(n).name],1,0);
        file_name = strsplit(file_list(n).name,'.');
        file_parameters = strsplit( file_name{1} ,'_'); %Ex: robot_navigation_u0#_s#_i#   {2}:u02 - {3}:s1 - {4}:i2 
        
        %s1.i1.max_length=
        try
            eval([ file_parameters{4} '.' file_parameters{5} '.max_length = max([' file_parameters{4} '.' file_parameters{5} '.max_length length(cvs_data(:,1))]);']);
        catch
            eval([ file_parameters{4} '.' file_parameters{5} '.max_length = length(cvs_data(:,1));']);
        end

        

        %s1.i1.u02.ABC = ... secs,camera,head_ry,head_rz,pos_x,ang_z
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.secs = cvs_data(:,1);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.camera = cvs_data(:,2);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.head_ry = cvs_data(:,3);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.head_rz = cvs_data(:,4);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pos_x = cvs_data(:,5);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ang_z = cvs_data(:,6);']);

        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_camera = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.camera);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_head_ry = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.head_ry);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_head_rz = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.head_rz);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_pos_x = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.pos_x);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_ang_z = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ang_z);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_camera = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_camera);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_head_ry = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_head_ry);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_head_rz = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_head_rz);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_pos_x = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_pos_x);']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_ang_z = diff(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_ang_z);']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.v = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_pos_x).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_ang_z).^2 );']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.rv = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_head_ry).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.d_head_rz).^2 );']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.spectralarc = SpectralArcLength( ' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.v, Ts );']);
        %eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.rspectralarc = SpectralArcLength( ' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.rv, Ts );']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.a = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_pos_x).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_ang_z).^2 );']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ra = sqrt( (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_head_ry).^2  + (' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.dd_head_rz).^2 );']);
        
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_a = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.a,resize_value,length(cvs_data(:,1)));']);
        eval([ file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.resampled_ra = resample(' file_parameters{4} '.' file_parameters{5} '.' file_parameters{3} '.ra,resize_value,length(cvs_data(:,1)));']);
        
    end
end

clear cvs_data file_name file_parameters;

for s=1:2
    for i=1:3
        eval([ 's' num2str(s) '.i' num2str(i) '.a = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.ra = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.spectralarc = zeros(max_users,1);']);
        eval([ 's' num2str(s) '.i' num2str(i) '.rspectralarc = zeros(max_users,1);']);
        for u=2:max_users
            user=num2str(u);
            if u<10
                user=['0' num2str(u)];
            end
            try
                eval([ 's' num2str(s) '.i' num2str(i) '.a(' num2str(u) ') =  mean(s' num2str(s) '.i' num2str(i) '.u' user '.resampled_a);'])
                eval([ 's' num2str(s) '.i' num2str(i) '.ra(' num2str(u) ') =  mean(s' num2str(s) '.i' num2str(i) '.u' user '.resampled_ra);'])
               eval([ 's' num2str(s) '.i' num2str(i) '.spectralarc(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.spectralarc;'])
               %eval([ 's' num2str(s) '.i' num2str(i) '.rspectralarc(' num2str(u) ') =  s' num2str(s) '.i' num2str(i) '.u' user '.rspectralarc;'])
            catch
            end
        end
    end
end

xlswrite([output 'nav_a_s1_i1'],s1.i1.a);
xlswrite([output 'nav_a_s1_i2'],s1.i2.a);
xlswrite([output 'nav_a_s1_i3'],s1.i3.a);
xlswrite([output 'nav_a_s2_i1'],s2.i1.a);
xlswrite([output 'nav_a_s2_i2'],s2.i2.a);
xlswrite([output 'nav_a_s2_i3'],s2.i3.a);

xlswrite([output 'nav_ra_s1_i1'],s1.i1.ra);
xlswrite([output 'nav_ra_s1_i2'],s1.i2.ra);
xlswrite([output 'nav_ra_s1_i3'],s1.i3.ra);
xlswrite([output 'nav_ra_s2_i1'],s2.i1.ra);
xlswrite([output 'nav_ra_s2_i2'],s2.i2.ra);
xlswrite([output 'nav_ra_s2_i3'],s2.i3.ra);

xlswrite([output 'nav_spectral_s1_i1'],s1.i1.spectralarc);
xlswrite([output 'nav_spectral_s1_i2'],s1.i2.spectralarc);
xlswrite([output 'nav_spectral_s1_i3'],s1.i3.spectralarc);
xlswrite([output 'nav_spectral_s2_i1'],s2.i1.spectralarc);
xlswrite([output 'nav_spectral_s2_i2'],s2.i2.spectralarc);
xlswrite([output 'nav_spectral_s2_i3'],s2.i3.spectralarc);

%xlswrite([output 'nav_rspectral_s1_i1'],s1.i1.rspectralarc);
%xlswrite([output 'nav_rspectral_s1_i2'],s1.i2.rspectralarc);
%xlswrite([output 'nav_rspectral_s1_i3'],s1.i3.rspectralarc);
%xlswrite([output 'nav_rspectral_s3_i1'],s3.i1.rspectralarc);
%xlswrite([output 'nav_rspectral_s3_i2'],s3.i2.rspectralarc);
%xlswrite([output 'nav_rspectral_s3_i3'],s3.i3.rspectralarc);

toc;

return;

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


