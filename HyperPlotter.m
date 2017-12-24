function [] = HyperPlotter(timeArray,locationArray,velocityArray,forceDragArray,totalHeatGeneratedArray,powerRequirementArray, validationTrialDistanceRange, validationVelocityArray)
%HyperPlotter
%   HyperScript Plotter Utility

figure
%% Velocity
subplot(2,2,1)
plot(locationArray,velocityArray,'-',validationTrialDistanceRange,validationVelocityArray,'r.')
title('Velocity Profile')
xlabel('Location (m)');
ylabel('Velocity (m/s)');
grid on
axis([locationArray(1), locationArray(end)*1.1, min(velocityArray), max(velocityArray)*1.25])

%% Drag
subplot(2,2,2)
plot(timeArray,forceDragArray)
title('Aerodynamic Profile')
xlabel('time (s)');
ylabel('Aerodynamic Drag (N)');
grid on
axis([timeArray(1), timeArray(end), min(forceDragArray), (max(forceDragArray)+1)*1.25])

%% Cumulative heat generated
subplot(2,2,3)
plot(timeArray,totalHeatGeneratedArray/1000);
title('Thermal Profile')
xlabel('Time (s)');
ylabel('Total heat produced (KJ)');
grid on
axis([timeArray(1), timeArray(end), min(totalHeatGeneratedArray/1000)*1.25, max(totalHeatGeneratedArray/1000)*1.25])

%% Power Requirement
subplot(2,2,4)
plot(timeArray,powerRequirementArray/1000)
title('Electrical Profile')
xlabel('Time (s)')
ylabel('Power Requirement (Kw)')
grid on
axis([timeArray(1), timeArray(end), min(powerRequirementArray/1000)*1.25, max(powerRequirementArray/1000)*1.25])
end

