%DragCalc

function [forceDrag] = DragCalc(velocity, C_d, frontalArea, pressure)

%DragCalc
%Function to calculate air drag on the pod. GUI takes input of Pascal and deg C -
%function converts deg C to deg K

R = 287.05; %specific gas constant for dry air
temp_air = 273.15; %+ app.TemperatureEditField.Value; %Kelvin - eqiv. to 77F
density = pressure/(R*temp_air);
forceDrag = 0.5 * C_d * density * velocity^2 * frontalArea;
end