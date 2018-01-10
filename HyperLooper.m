%% HyperLooper

%inputs

%default characteristics
%mass = 250;
%radius = 0.33;
motorPowerKw = 230;
maxTorque = 500;
maxRPM = 4500;
transmissionRatio = 1;
%trialDistance = 1180;
%C_d = 0.3;
%frontalArea = 1;
%pressure = 900;
%Coeff_Friction = .4;
%forceBrakePneumatic = 5500;
%regen = 0;
kV = 18;
kI = 0.5;
lowerEfficencyBound = 0.85;
rotorInertia = 0.0392;

fprintf('\n-----\nWorking\n')
%{
massRange = [100:25:300];
radiusRange = [.1:.01:.5];
trialDistanceRange = [800:10:1180];
C_dRange = [0.1:0.05:0.7];
frontalAreaRange = [.25:.25:2];
pressureRange = [1000:10000:101000];
Coeff_FrictionRange = [.3:0.025:.6];
forceBrakePneumaticRange = [5000:1000:15000];
regenRange = [0:50:500];
%}

massRange = [100:50:300];
radiusRange = [.3:.1:.5];
trialDistanceRange = [1000:100:1200];
C_dRange = [0.2:0.1:0.4];
frontalAreaRange = [.5:.5:1];
pressureRange = [1000:25000:101000];
Coeff_FrictionRange = [.4:0.1:.6];
forceBrakePneumaticRange = [5000:5000:15000];
regenRange = [0:250:500];

n = 1;
%resultArray = zeros(1,1000000);
conditionArray = []; %[0,0,0,0,0,0,0,0];


for imass = massRange
    for iradius = radiusRange
        for itrialDistance = trialDistanceRange
            for iC_d = C_dRange
                for ifrontalArea = frontalAreaRange
                    for ipressure = pressureRange
                        for iCoeff_Friction = Coeff_FrictionRange
                            for iforceBrakePneumatic = forceBrakePneumaticRange
                                for iregen = regenRange
                                    %[velocityMaximum, accelerationMaximum, timeEnd, timeArray, locationArray, velocityArray, accelerationArray, forceDriveArray, forceDragArray, forceNetArray,maximumDynamicPressure,decelerationDistance,finalLocation,currentRequirementArray, voltageRequirementArray, powerRequirementArray, powerLossArray, totalHeatGenerated, totalHeatGeneratedArray] = Numerical_Int_function(imass,iradius,motorPowerKw,maxTorque,maxRPM,transmissionRatio,itrialDistance,iC_d,ifrontalArea,ipressure,iforceBrakePneumatic,iCoeff_Friction,iregen,kV,kI,lowerEfficencyBound,rotorInertia);
                                    %resultArray(n) = velocityMaximum;
                                    conditionArray(n,:) = [imass,iradius,itrialDistance,iC_d,ifrontalArea,ipressure,iCoeff_Friction,iforceBrakePneumatic];
                                    n = n+1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

fprintf('Complete\n');
