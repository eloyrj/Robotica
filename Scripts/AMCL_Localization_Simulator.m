load mi_mapa_slam_sim_sinRuido.mat
show(map);

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;
%% 
% Set |rangeFinderModel.SensorPose| to the coordinate transform of the fixed 
% laser sensor with respect to the robot base. This is used to transform the laser 
% readings from laser frame to the base frame of AmigoBot. Please refer to <docid:robotics_examples.example-ROSTransformationTreeExample 
% docid:robotics_examples.example-ROSTransformationTreeExample> for details on 
% coordinate transformations.
% 
% Note that currently |SensorModel| is only compatible with sensors that are 
% fixed on the robot's frame, which means the sensor transform is constant.

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'robot0','robot0_laser_1');
sensorTransform = getTransform(tftree,'robot0', 'robot0_laser_1');

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
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([0.5 0.5 deg2rad(10)]*1); % Covariance of initial pose

visualizationHelper = ExampleHelperAMCLVisualization(map);

i=1;
while (1)
    % Receive laser scan and odometry message.
    scan = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    
    %Crear objeto para almacenar el escaneo LiDAR 2-D
    scans = lidarScan(scan);
    
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    % Drive robot to next pose.
    %wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
end
%% Stop the AmigoBot and shutdown ROS in MATLAB

%stop(wanderHelper);
%rosshutdown
%% 
% _Copyright 2015-2016 The MathWorks, Inc._