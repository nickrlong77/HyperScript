%HyperScript

%inputs
default = input('Use defaults? (y/n): ','s');
if default == 'y'
    %default characteristics 
    mass = 125;
    radius = 0.33;
    motorPowerKw = 220;
    maxTorque = 500;
    maxRPM = 4500;
    transmissionRatio = 1;
    trialDistance = 1000;
    C_d = 0.3;
    frontalArea = 1;
    pressure = 900;
    Coeff_Friction = 0.4;
    forceBrakePneumatic = 5500;
    regen = 'on';
    kV = 18;
    kI = 0.5;
    lowerEfficencyBound = 0.92;
    rotorInertia = 0.0392;
    fprintf('\n-----\nWorking\n')
else
    mass = input('Pod mass (kg): ');
    radius = input('Wheel Radius (m): ');
    motorPowerKw = input('Motor power (Kw): ');
    maxTorque = input('Maximum torque (N*m): ');
    maxRPM = input('Maximum RPM: ');
    transmissionRatio = input('Transmission factor: ');
    trialDistance = input('Trial Distance (m): ');
    C_d = input('Coefficient of drag: ');
    frontalArea = input('Frontal area (m^2): ');
    pressure = input('Tube pressure (Pa): ');
    Coeff_Friction = input('Coefficient of friction (brake pads): ');
    forceBrakePneumatic = input('Pneumatic brake force (N): ');
    regen = input('Regenerative Braking? (y/n): ', 's');
    kV = input('Constant of velocity (RPM/V): ');
    kI = input('Constant of torque (Nm/A): ');
    lowerEfficencyBound = input('Lower bound of engine efficiency (decimal): ');
    rotorInertia = input('Rotor Inertia (Kg/m^2): ');
    fprintf('\n-----\nWorking\n')
end
%%%%%ADD BRAKE FORCE AND COEFF FRICTION TO FUNCTION INPUT
%Calculate:

%[velocityMaximum, accelerationMaximum, timeEnd, timeArray, locationArray, velocityArray, accelerationArray, forceDriveArray, forceDragArray, forceNetArray,maximumDynamicPressure,decelerationDistance,finalLocation] = Numerical_Int_function(mass,radius,motorPowerKw,maxTorque,maxRPM,transmissionRatio,trialDistance,C_d,frontalArea,pressure,forceBrakePneumatic,Coeff_Friction,regen);
[velocityMaximum, accelerationMaximum, timeEnd, timeArray, locationArray, velocityArray, accelerationArray, forceDriveArray, forceDragArray, forceNetArray,maximumDynamicPressure,decelerationDistance,finalLocation,currentRequirementArray, voltageRequirementArray, powerRequirementArray, powerLossArray, totalHeatGenerated, totalHeatGeneratedArray] = Numerical_Int_function(mass,radius,motorPowerKw,maxTorque,maxRPM,transmissionRatio,trialDistance,C_d,frontalArea,pressure,forceBrakePneumatic,Coeff_Friction,regen,kV,kI,lowerEfficencyBound,rotorInertia);

[validationTrialDistanceRange, validationVelocityArray] = ConstantTorque(trialDistance, mass,maxTorque,radius,maxRPM,forceBrakePneumatic,Coeff_Friction);

%Plotter
HyperPlotter(timeArray,locationArray,velocityArray,forceDragArray,totalHeatGeneratedArray,powerRequirementArray,validationTrialDistanceRange,validationVelocityArray);
%spacer
fprintf('-----\n\n')
%Outputs:
fprintf('-----\n')
fprintf('Maximum velocity: %.2f \n',velocityMaximum);
fprintf('Maximum acceleration = %.2f m/s^2 \n',accelerationMaximum);
fprintf('Maximum deceleration = %.2f m/s^2 \n',min(accelerationArray));
fprintf('Trial time: %.2f seconds\n',timeEnd);
fprintf('Maximum dynamic pressure: %.2f Pa\n',maximumDynamicPressure);
fprintf('Total heat generated: %.2f Joules\n',totalHeatGenerated)
fprintf('Stopping distance: %.2f m\n',decelerationDistance);
fprintf('Final location: %.2f m\n',finalLocation)
fprintf('-----\n')
%These variables were omited from outputting as they are arrays meant for ploting:
%{
timeArray
locationArray
velocityArray
accelerationArray
forceDriveArray
forceDragArray
forceNetArray
%}

