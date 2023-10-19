%% DECLARACION DE SUBSCRIBERS %%
%  Odometria
sub_odom=rossubscriber('/robot0/odom', 'nav_msgs/Odometry');
%  Laser
sub_laser=rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');
%  Sonares
sub_sonar0=rossubscriber('/robot0/sonar_0', 'sensor_msgs/Range');
sub_sonar1=rossubscriber('/robot0/sonar_1', 'sensor_msgs/Range');
sub_sonar2=rossubscriber('/robot0/sonar_2', 'sensor_msgs/Range');
sub_sonar3=rossubscriber('/robot0/sonar_3', 'sensor_msgs/Range');
sub_sonar4=rossubscriber('/robot0/sonar_4', 'sensor_msgs/Range');
sub_sonar5=rossubscriber('/robot0/sonar_5', 'sensor_msgs/Range');
sub_sonar6=rossubscriber('/robot0/sonar_6', 'sensor_msgs/Range');
sub_sonar7=rossubscriber('/robot0/sonar_7', 'sensor_msgs/Range');

%% DECLARACION DE PUBLISHERS %%
%  Velocidad
pub_vel=rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

%% GENERACION DE MENSAJES %%
msg_vel=rosmessage(pub_vel);

%% Devinimos la periodicidad del bucle %%
r=rateControl(10);

%% Comprobamos que está todo OK
disp('-- Inicialización del simulador finalizada correctamente.');