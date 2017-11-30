function [velocityMaximum, accelerationMaximum, timeEnd, timeArray, locationArray, velocityArray, accelerationArray, forceDriveArray, forceDragArray, forceNetArray,maximumDynamicPressure,decelerationDistance,finalLocation] = Numerical_Int_function(mass,radius,motorPowerKw,maxTorque,maxRPM,transmissionRatio,trialDistance,C_d,frontalArea,pressure,forceBrakePneumatic,Coeff_Friction)
%% Variables
time = 0;
torque = 0;
v=0.001;
a=0;
s=0;
dt = 0.001;
ds = 0;
forceFrictionBrakeMaximum = forceBrakePneumatic * Coeff_Friction;
velocityArray = [];
locationArray = [];
timeArray = [];
accelerationArray = [];
forceDriveArray = [];
forceDragArray = [];
forceNetArray = [];
state = 'acc';
previousProgress = 0.001;
%% Numerical Integration
%while s <= trialDistance
while v > 0
    if strcmp(state,'acc') == 1
        RPM = v *60 /(2*pi*radius);
        %% Read torque from Torqe_curve_reader function
        torque = Torque_curve_reader(RPM,motorPowerKw,maxTorque,maxRPM,transmissionRatio);
        %torque_record = [torque_record, torque];
        forceFrictionBrake = 0;
    elseif strcmp(state,'dec') == 1
        torque = -Torque_curve_reader(RPM,motorPowerKw,maxTorque,maxRPM,transmissionRatio);
        forceFrictionBrake = forceFrictionBrakeMaximum;
    end
    forceDrag = DragCalc(v,C_d,frontalArea,pressure);
    forceDragArray = [forceDragArray forceDrag];
    %% Update Kinematic Vectors
    time = time + dt;
    timeArray = [timeArray, time];
    forceDrive = torque/(radius);
    forceDriveArray = [forceDriveArray, forceDrive];
    forceNet = forceDrive - forceDrag - forceFrictionBrake;
    forceNetArray = [forceNetArray, forceNet];
    %Calculate accleration 'a'
    a = forceNet/mass;
    %Update velocity
    v = v + a*dt;
    %Terminate loop when the pod stops before the end of the run distance
    %if v <= 0
    %    break
    %end
    %add velocity to velocity history vector and accleration to accleration history vector
    accelerationArray = [accelerationArray a];
    velocityArray = [velocityArray v];
    %Update position and add to location history
    ds = v * dt;
    s = s + ds;
    locationArray = [locationArray s];
    
    %Brake distance calculations
    if strcmp(state,'acc') == 1
        decelerationDistance = Stop_dist_calc(v,s,mass,radius,maxRPM,maxTorque, motorPowerKw,transmissionRatio,C_d,frontalArea, pressure,forceFrictionBrakeMaximum,Coeff_Friction,trialDistance);
        %decelerationDistance = (mass*v^2)/(2*forceFrictionBrakeMaximum);
    end
    %Determine acc/dec state
    if strcmp(state,'acc') == 1
        state = State_conditional(decelerationDistance,s,trialDistance);
    end
    
    %{
                if s + decelerationDistance > trialDistance+1 %&& s + decelerationDistance <= trialDistance + 1
                    state = 'dec';
                end
    %}
end
%% Define output variables
velocityMaximum = max(velocityArray);
accelerationMaximum = max(accelerationArray);
timeEnd = max(time);
maximumDynamicPressure = max(forceDragArray)/(C_d*frontalArea);
finalLocation = max(locationArray);

end
