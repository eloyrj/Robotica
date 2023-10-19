%% VARIABLES
% Establecemos el ángulo a girar y rellenamos los mensajes
finalang = 90;
msg_vel.Linear.X=0;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
if (finalang>0)
    msg_vel.Angular.Z=0.1;
else
    msg_vel.Angular.Z=-0.1;
end
% Leemos el ángulo inicial
initori = sub_odom.LatestMessage.Pose.Pose.Orientation;
ang_euler=quat2eul([initori.W initori.X initori.Y initori.Z]);
yawini=ang_euler(1);


%% BUCLE DE CONTROL INFINITO
while(1)
    %Obtenemos el ángulo actual
    initori = sub_odom.LatestMessage.Pose.Pose.Orientation;
    ang_euler=quat2eul([initori.W initori.X initori.Y initori.Z]);
    yaw=ang_euler(1);
    %disp(sprintf('\nactang_euler: %f   yawact: %f', actang_euler, yawact));
    %Comprobamos cuánto hemos girado
    ang=angdiff(yawini,yaw);
    ang = rad2deg(ang);
    %disp(sprintf('\nyaw: %f   ang: %f', yaw, ang));
    %Si hemos girado lo que queríamos detenemos el robot y salimos del bucle
    if (abs(ang)>abs(finalang))
        msg_vel.Angular.Z=0;
        send(pub_vel,msg_vel);
        break;
    else
        send(pub_vel,msg_vel);
    end
    lee_sensores;
    waitfor(r);
end