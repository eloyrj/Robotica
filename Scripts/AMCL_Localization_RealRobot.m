

load map_modified.mat
map = map_modified;
show(map);

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
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];


amcl = monteCarloLocalization;
amcl.UseLidarScan = true;

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;


amcl.ParticleLimits = [1000 10000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = true;      % global = true      local=false = sabemos posi inicial
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([0.5 0.5 deg2rad(10)]*1); % Covariance of initial pose
%% Setup helper for visualization and driving AmigoBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's estimated 
% pose, particles, and laser scan readings on the map.

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