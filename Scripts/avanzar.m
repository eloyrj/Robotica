%% VARIABLES
% Establecemos la distancia a avanzar y rellenamos los mensajes
distancia = 0.5;
msg_vel.Linear.X=0.1;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;
% Leemos la posición inicial
initpos = sub_odom.LatestMessage.Pose.Pose.Position;

%% BUCLE DE CONTROL INFINITO
while(1)
    %Obtenemos la posición actual
    pos = sub_odom.LatestMessage.Pose.Pose.Position;
    disp(sprintf('\tPosición actual: X=%f  Y=%f  Z=%f', pos.X, pos.Y, pos.Z))
    %Calculamos la distancia euclídea que se ha desp??
    dist = sqrt((initpos.X-pos.X)^2+(initpos.Y-pos.Y)^2);
    disp(sprintf('. Se han avanzado %f metros.', dist));
    %Si hemos avanzado toda la distancia detenemos el robot y salimos del
    %bucle
    if (dist>distancia)
        msg_vel.Linear.X = 0;
        send(pub_vel,msg_vel);
        break;
    else
        send(pub_vel,msg_vel);
    end
    lee_sensores;
    waitfor(r);
end