function [velocityMaximum, accelerationMaximum, timeEnd, timeArray, locationArray, velocityArray, accelerationArray, forceDriveArray, forceDragArray, forceNetArray,maximumDynamicPressure,decelerationDistance,finalLocation,currentRequirementArray, voltageRequirementArray, powerRequirementArray, powerLossArray, totalHeatGenerated, totalHeatGeneratedArray] = Numerical_Int_function(mass,radius,motorPowerKw,maxTorque,maxRPM,transmissionRatio,trialDistance,C_d,frontalArea,pressure,forceBrakePneumatic,Coeff_Friction,regen,kV,kI,lowerEfficencyBound,rotorInertia)
%% Variables
time = 0; %initial time (seconds)
torque = 0; %initial torque (N*m)
v=0.001; %intial velocity. Initialized above zero to avoid the break condition. 
a=0; %Initial acceleration (m/s^2)
s=0; %Initial location in the tube (m)
dt = 0.01; %timestep (seconds)
ds = 0; %initial spacestep
forceFrictionBrakeMaximum = forceBrakePneumatic * Coeff_Friction; % maximum force of the friction brakes (N)
velocityArray = []; %Historical Array for velocity (m/s)
locationArray = []; %Historical Array for location (m)
timeArray = [];
accelerationArray = [];
forceDriveArray = [];
forceDragArray = [];
forceNetArray = [];
state = 'acc'; %Initial state ('acc' for accelerate or 'dec' for decelerate)
currentRequirementArray = [];
voltageRequirementArray = [];
powerRequirementArray = [];
powerLossArray = []; %Historical Array for engine power losses (w)
totalHeatGenerated = 0; %Total heat generated (J)
totalHeatGeneratedArray = [];

mass = mass + EquivalentMassCalculator(rotorInertia,(radius*transmissionRatio));
%% Numerical Integration
while v > 0
    if strcmp(state,'acc') == 1
        RPM = v *60 /(2*pi*radius);
        %% Read torque from Torqe_curve_reader function
        torque = Torque_curve_reader(RPM,motorPowerKw,maxTorque,maxRPM,transmissionRatio);
        forceFrictionBrake = 0;
    elseif strcmp(state,'dec') == 1
        %torque = -Torque_curve_reader(RPM,motorPowerKw,maxTorque,maxRPM,transmissionRatio);
        forceFrictionBrake = forceFrictionBrakeMaximum;
        torque = -regen/transmissionRatio;
    end
    %% Update Kinematic Vectors
    time = time + dt;
    forceDrive = torque/(radius);
    forceDrag = DragCalc(v,C_d,frontalArea,pressure);
    forceNet = forceDrive - forceDrag - forceFrictionBrake;
    a = forceNet/mass;
    v = v + a*dt;
    s = s + v * dt;
    %% Electronic and Thermal Update
    % Engine Power Analysis
    [currentRequirement, voltageRequirement, powerRequirement, powerLoss] = EnginePowerAnalysis(kV, kI, torque, v, radius, lowerEfficencyBound);
    % Update Running total heat generation
    totalHeatGenerated = totalHeatGenerated + powerLoss * dt;
    %% Update History Arrays
    timeArray = [timeArray, time];
    locationArray = [locationArray, s];
    velocityArray = [velocityArray, v];
    accelerationArray = [accelerationArray, a];
    forceDriveArray = [forceDriveArray, forceDrive];
    forceDragArray = [forceDragArray, forceDrag];
    forceNetArray = [forceNetArray, forceNet];
    totalHeatGeneratedArray = [totalHeatGeneratedArray, totalHeatGenerated];
    currentRequirementArray = [currentRequirementArray, currentRequirement];
    voltageRequirementArray = [voltageRequirementArray, voltageRequirement];
    powerRequirementArray = [powerRequirementArray, powerRequirement];
    powerLossArray = [powerLossArray, powerLoss];
    %% Brake distance calculations
    if strcmp(state,'acc') == 1
        decelerationDistance = Stop_dist_calc(v,s,mass,radius,maxRPM,maxTorque, motorPowerKw,transmissionRatio,C_d,frontalArea, pressure,forceFrictionBrakeMaximum,trialDistance,regen);
    end
    %Determine acc/dec state
    if strcmp(state,'acc') == 1
        state = State_conditional(decelerationDistance,s,trialDistance);
    end
end
%% Define output variables
velocityMaximum = max(velocityArray);
accelerationMaximum = max(accelerationArray);
timeEnd = max(time);
maximumDynamicPressure = max(forceDragArray)/(C_d*frontalArea);
finalLocation = max(locationArray);
end
