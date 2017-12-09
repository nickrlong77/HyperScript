function [] = HyperPlotter(locationArray,velocityArray,forceDragArray,totalHeatGeneratedArray,powerRequirementArray)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

figure
subplot(4,1,1)
plot(locationArray,velocityArray)
title('Velocity Profile')
xlabel('Location (m)');
ylabel('Velocity (m/s)');
subplot(4,1,2)
plot(locationArray,forceDragArray)
title('Aerodynamic Profile')
xlabel('Location (m)');
ylabel('Aerodynamic Drag (N)');
subplot(4,1,3)
plot(locationArray,totalHeatGeneratedArray);
title('Thermal Profile')
xlabel('Location (m)');
ylabel('Total heat produced (J)');
subplot(4,1,4)
plot(locationArray,powerRequirementArray)
title('Electrical Profile')
xlabel('Location (m)')
ylabel('Power Requirement (w)')
end

