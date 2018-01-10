function [decelerationDistance] = Stop_dist_calc(v,s,mass,radius, maxRPM,maxTorque, motorPowerKw,transmissionRatio,C_d,frontalArea, pressure,forceFrictionBrakeMaximum, trialDistance,regen)
%Numerical Integration variables
dt = 0.001; %timesetp
decelerationDistance = 0; %initialize decelerationDistance
Stop_distance_history = []; %initialize history
F_braking_history = []; %intitialize another history
if s + ((mass*v^2)/(2*forceFrictionBrakeMaximum)) >= trialDistance %this is used to speed up the calculation. If the brakes could stop it before the limit, then skip this for now. Only calculate if decelerationDistance will be > decelerationDistance with brakes only.
    while v>0 % if the pod is moving...
        Given_RPM = v *60 /(2*pi*radius); %determine RPM for Torque curve reader
        % Full regen capability assumption %F_braking = -forceFrictionBrakeMaximum - (Torque_curve_reader(Given_RPM,motorPowerKw, maxTorque, maxRPM,transmissionRatio)/radius) - DragCalc(v,C_d,frontalArea,pressure); % force of "braking" / deceleration = pneumatic force * coeff. Friction + motor regen (same as torque curve, but negative sign) + aero drag
        F_braking = -forceFrictionBrakeMaximum - (regen/(radius*transmissionRatio)) - DragCalc(v,C_d,frontalArea,pressure); 
        a = F_braking/mass; %calculate decel
        v = v + a*dt; %time step to update velocity
        ds = v*dt; % update location based on velocity
        decelerationDistance = decelerationDistance + ds; %add ds to decelerationDistance. Cumulative sum = total Stop distance
        %Debugging things I tried for a bit. All below aren't part of the function
        F_braking_history = [F_braking_history, F_braking];
        %v_history = [v_history, v];
    end
end
Stop_distance_history = [Stop_distance_history, decelerationDistance];
Stop_distance_history;
F_braking_history;
assignin('base','FBH',F_braking_history);
assignin('base','Stop_distance_history',Stop_distance_history);
end
