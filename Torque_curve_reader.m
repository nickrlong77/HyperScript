function [torque] = Torque_curve_reader(Given_RPM,motorPowerKw, maxTorque, maxRPM,transmissionRatio)
maxTorque = maxTorque * (1/transmissionRatio);
maxRPM = maxRPM * transmissionRatio;
%% Conversions and Initilizations
Power_w = motorPowerKw*1000;
critical_w = (Power_w)/maxTorque;
torque = 0;
w = (Given_RPM.*2.*pi)./60;
w_max = (maxRPM.*2.*pi)./60;
%% Torque Curve Range Conditional
if w < critical_w
    torque = maxTorque;
elseif w > w_max
    torque = 0;
else
    torque = Power_w./w;
end
end