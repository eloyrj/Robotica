%Definir la posicion de destino
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

endLocation = [6 0];

%Cargar el mapa
%%%%%%%%%%%%%%%

close all

load map_modified.mat
map=map_modified
show(map);
fig_laser=figure; title('LASER')
fig_vfh=figure; title('VFH')


%Crear el objeto VFH…y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VFH=controllerVFH;

VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.05 2];
VFH.RobotRadius=0.15;
VFH.SafetyDistance=0.1;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=5;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3 10];
VFH.UseLidarScan=true;

%Inicializar el localizador AMCL (práctica 1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;

tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'base_link','laser_frame');
sensorTransform = getTransform(tftree,'base_link', 'laser_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [500 10000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = true;      % global = true      local=false = sabemos posi inicial
amcl.InitialPose = [2 2 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([0.5 0.5 deg2rad(10)]*4); % Covariance of initial pose

visualizationHelper = ExampleHelperAMCLVisualization(map);

%Crear el objeto PurePursuit y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

controller=controllerPurePursuit;
controller.LookaheadDistance = 1;
controller.DesiredLinearVelocity= 0.25;
controller.MaxAngularVelocity = 1.0;

%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s

msg_vel.Linear.X = 0.1;

%Definimos los umbrales para la covarianza
umbralx=0.07;
umbraly=0.07;
umbralyaw=deg2rad(10);

%Bucle de control infinito
while(1)

     %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    
    figure(fig_laser);
    lee_sensores;
    scans = lidarScan(msg_laser);

    %Leer la odometría

    odompose = sub_odom.LatestMessage;

    %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior

    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    %Ejecutar amcl para obtener la posición estimada estimatedPose y la
    %covarianza estimatedCovariance (mostrar la última por pantalla para
    %facilitar la búsqueda de un umbral)

    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);

    %Si la covarianza está por debajo de un umbral, el robot está localizado y
    %finaliza el programa

    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp('Robot Localizado');
        break;
    end

    %Dibujar los resultados del localizador con el visualizationHelper
    
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura ‘fig_vfh’

    steeringDir = VFH(lidarScan(msg_laser),0);
    figure(fig_vfh);
    show(VFH)

    %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1)
    
    msg_vel.Angular.Z=steeringDir*0.3;
    
    %Publicar el mensaje de velocidad
    
    send(pub_vel,msg_vel);
    
    %Esperar al siguiente periodo de muestreo
    
    waitfor(r);
end
%%%%%%%%%%% AL SALIR DE ESTE BUCLE EL ROBOT YA SE HA LOCALIZADO %%%%%%%%%%
%%%%%%%%%%% COMIENZA LA PLANIFICACIÓN GLOBAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Paramos el robot, para que no avance mientras planificamos

msg_vel.Linear.X = 0;
msg_vel.Angular.Z=0;
send(pub_vel,msg_vel);

%Hacemos una copia del mapa, para “inflarlo” antes de planificar

cpMap= copy(map);
inflate(cpMap,0.25);

%Crear el objeto PRM y ajustar sus parámetros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

planner = mobileRobotPRM;
planner.Map=cpMap;
planner.NumNodes=500;
planner.ConnectionDistance = 3;

%Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figura

waypoints = findpath(planner, [estimatedPose(1) estimatedPose(2)], endLocation);
figure;
show(planner)

%%%%%%%%%%% COMIENZA EL BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Indicamos al controlador la lista de waypoints a recorrer (ruta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

controller.Waypoints=waypoints;

%Bucle de control
%%%%%%%%%%%%%%%%%
while(1)
    %Leer el láser y la odometría
    figure(fig_laser);
    lee_sensores;
    scans = lidarScan(msg_laser);
    odompose = sub_odom.LatestMessage;

    %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior

    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    %Ejecutar amcl para obtener la posición estimada estimatedPose

    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);

    %Dibujar los resultados del localizador con el visualizationHelper

    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    %Ejecutar el controlador PurePursuit para obtener las velocidades lineal
    %y angular

    [vel,angvel] = controller(estimatedPose);

    %Llamar a VFH pasándole como “targetDir” un valor proporcional a la
    %velocidad angular calculada por el PurePursuit

    steeringDir = VFH(lidarScan(msg_laser),angvel);
    figure(fig_vfh);
    show(VFH)

    %Calcular la velocidad angular final como una combinación lineal de la
    %generada por el controlador PurePursuit y la generada por VFH

    angvel = steeringDir*0.03;
    %Rellenar los campos del mensaje de velocidad
    
    msg_vel.Linear.X = vel;
    msg_vel.Angular.Z=angvel;

    %Publicar el mensaje de velocidad

    send(pub_vel,msg_vel);

    %Comprobar si hemos llegado al destino, calculando la distancia euclidea
    %y estableciendo un umbral
    if (sqrt((estimatedPose(1) - endLocation(1))^2 + (estimatedPose(2)-endLocation(2))^2) <= 0.5)
        %Parar el robot
        break;
    end
    %Esperar al siguiente periodo de muestreo

    waitfor(r);

end
msg_vel.Linear.X = 0;
msg_vel.Angular.Z=0;
send(pub_vel,msg_vel);