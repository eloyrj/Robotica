%% LECTURA DE SENSORES
msg_sonar0 = sub_sonar0.LatestMessage;
msg_sonar1 = sub_sonar1.LatestMessage;
msg_sonar2 = sub_sonar2.LatestMessage;
msg_sonar3 = sub_sonar3.LatestMessage;
msg_sonar4 = sub_sonar4.LatestMessage;
msg_sonar5 = sub_sonar5.LatestMessage;
msg_sonar6 = sub_sonar6.LatestMessage;
msg_sonar7 = sub_sonar7.LatestMessage;
msg_laser = sub_laser.LatestMessage;

%% REPRESENTACIÓN DE LOS DATOS
% Representación gráfica de los datos del laser
plot(msg_laser, 'MaximumRange', 8);
% Mostramos las lecturas por pantalla
% disp(sprintf('\tSONARES 0-7: %f %f %f %f %f %f %f %f', ...
%     msg_sonar0.Range_, msg_sonar1.Range_, msg_sonar2.Range_, ...
%     msg_sonar3.Range_, msg_sonar4.Range_, msg_sonar5.Range_, ...
%     msg_sonar6.Range_, msg_sonar7.Range_));