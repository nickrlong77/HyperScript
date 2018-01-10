function [validationTrialDistanceRange, validationVelocityArray] = ConstantTorque(trialDistance, mass,torque,radius,transmissionRatio,maxRPM,forceBrakePneumatic,Coeff_Friction,regen)
%Validation of the HyperScript methodology using assumptions of constant (trialDistance, mass,torque,radius,maxRPM,forceBrakePneumatic,Coeff_Friction)
%torque and constant brake force

%Depreciated I/O:
%{
trialDistance = input('Trial Distance (m): ');
mass = input('Mass (kg): ');
torque = input('Torque (N*m): ');
radius = input('Wheel Radius (m): ');
RPM_Max = input('Maximum RPM: ');
frictionBrakeForce = input('Friction brake force (N): ');
frictionCoeff = input('Coefficenct of Friction: ');
%}
stopForce = (forceBrakePneumatic*Coeff_Friction)+(regen/(transmissionRatio*radius));
velocityLimit = radius*maxRPM*transmissionRatio*2*pi/60;
validationVelocityArray = [];
validationTrialDistanceRange = 0:10:trialDistance;
stopLocation = 100; %This value is not used. Just needs to be initialized
deceleration = -stopForce/mass;
for d = validationTrialDistanceRange
    if d < stopLocation        
    v  = ((2*torque*d)/(mass*radius*transmissionRatio))^(.5);
    validationVelocityArray = [validationVelocityArray, v];
    stopDistance = (mass*v^2)/(2*stopForce);
    stopLocation = trialDistance - stopDistance;
    if v > velocityLimit
        v = velocityLimit;
    end
    velocityMaximum = max(validationVelocityArray);
    velocityMaximumLocation = d;
    elseif d >= stopLocation
        v = (((velocityMaximum)^2)+2*deceleration*(d-stopLocation))^(1/2);
        validationVelocityArray = [validationVelocityArray, v];
    end
    if v< 0
        break
    end
end
%{
plot(validationTrialDistanceRange,validationVelocityArray);
title('Constant Torque Validation')
axis([0 1200 0 200]);
grid on
ylabel('Velocity (m/s)');
xlabel('Location (m)');
%}
end