function [ currentRequirement, voltageRequirement, powerRequirement, powerLoss] = EnginePowerAnalysis(kV, kI, torque, v, radius, lowerEfficencyBound)
%EnginePowerAnalysis 
%   Takes in arguments of motor constants, torque, velocity, and lower
%   efficency and returns power draw and heat loss estimations

RPM = v *60 /(2*pi*radius);
currentRequirement = torque/kI;
voltageRequirement = RPM/kV;
powerRequirement = currentRequirement * voltageRequirement;
powerLoss = abs(powerRequirement) * (1-lowerEfficencyBound);

end
