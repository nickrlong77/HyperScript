%HyperScript - AeroEdit

velocityMaximumArray = [];
maxForceDragArray = [];
ipressureArray = [0:1000:100000];


for ipressure = ipressureArray

%inputs
mass = 125; %input('Pod mass (kg): ');
radius = .3; %input('Wheel Radius (m): ');
motorPowerKw = 230; %input('Motor power (Kw): ');
maxTorque = 500; %input('Maximum torque (N*m): ');
maxRPM = 5500; %input('Maximum RPM: ');
transmissionRatio = 1; %input('Transmission factor: ');
trialDistance = 1000; %input('Trial Distance (m): ');
C_d = 0.3; %input('Coefficient of drag: ');
frontalArea = 1; %input('Frontal area (m^2): ');
pressure = ipressure; %input('Tube pressure (Pa): ');
Coeff_Friction = .4; %input('Coefficient of friction (brake pads): ');
forceBrakePneumatic = 5500; %input('Pneumatic brake force (N): ');
%fprintf('\n-----\nWorking\n')
%%%%%ADD BRAKE FORCE AND COEFF FRICTION TO FUNCTION INPUT
%Calculate:

[velocityMaximum, accelerationMaximum, timeEnd, timeArray, locationArray, velocityArray, accelerationArray, forceDriveArray, forceDragArray, forceNetArray,maximumDynamicPressure,decelerationDistance,finalLocation] = Numerical_Int_function(mass,radius,motorPowerKw,maxTorque,maxRPM,transmissionRatio,trialDistance,C_d,frontalArea,pressure,forceBrakePneumatic,Coeff_Friction);

velocityMaximumArray = [velocityMaximumArray, velocityMaximum];
maxForceDragArray = [maxForceDragArray max(forceDragArray)];
end

plot(ipressureArray,velocityMaximumArray,ipressureArray,maxForceDragArray)

%{
%spacer
fprintf('-----\n\n')
%Outputs:
fprintf('Maximum velocity: %.2f \n',velocityMaximum);
fprintf('Maximum accleration = %.2f m/s^2 \n',accelerationMaximum);
fprintf('Trial time: %.2f seconds\n',timeEnd);
fprintf('Maximum dynamic pressure: %.2f Pa\n',maximumDynamicPressure);
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
%}

%Previous:
%{
        function [v_top, a_top, time_end, time_record, loc, vel, a_record, Tractive_record, drag_record, net_force_record,max_Q,Stop_distance,final_location] = Numerical_Int_function(mass,radius,Power_Kw,Torque_max,RPM_max,transmission_factor,track_dist,C_d,frontal_area,pressure,Coeff_Friction,Brake_force)
            %% Variables
            time = 0;
            torque = 0;
            v=0.001;
            a=0;
            s=0;
            dt = 0.001;
            ds = 0;
            %Brake_force = app.MaxBrakingForceEditField.Value;
            %Coeff_Friction = app.CoefficientofFrictionEditField.Value;
            F_brake_max = Brake_force * Coeff_Friction;
            vel = [];
            loc = [];
            time_record = [];
            a_record = [];
            Tractive_record = [];
            drag_record = [];
            net_force_record = [];
            state = 'acc';
            %% Numerical Integration
            %while s <= track_dist
            while v > 0
                if strcmp(state,'acc') == 1
                    RPM = v *60 /(2*pi*radius);
                    %% Read torque from Torqe_curve_reader function
                    torque = Torque_curve_reader(app,RPM,Power_Kw,Torque_max,RPM_max,transmission_factor);
                    %torque_record = [torque_record, torque];
                    F_brake = 0;
                elseif strcmp(state,'dec') == 1
                    torque = -Torque_curve_reader(app,RPM,Power_Kw,Torque_max,RPM_max,transmission_factor);
                    F_brake = F_brake_max;
                end
                F_drag = DragCalc(app,v,C_d,frontal_area,pressure);
                drag_record = [drag_record, F_drag];
                %% Update Kinematic Vectors
                time = time + dt;
                time_record = [time_record, time];
                F_Tractive = torque/(radius);
                Tractive_record = [Tractive_record, F_Tractive];
                F_net = F_Tractive - F_drag - F_brake;
                net_force_record = [net_force_record, F_net];
                %Calculate accleration 'a'
                a = F_net/mass;
                %Update velocity
                v = v + a*dt;
                %Terminate loop when the pod stops before the end of the run distance
                %if v <= 0
                %    break
                %end
                %add velocity to velocity history vector and accleration to accleration history vector
                a_record = [a_record, a];
                vel = [vel v];     
                %Update position and add to location history
                ds = v * dt;                                                                        
                s = s + ds;                                                                         
                loc = [loc s];
                %Brake distance calculations
%%%%%%%%%%%%%   %ADD MOTOR REGEN BRAKING TO THIS CALCULATION
                if strcmp(state,'acc') == 1
                    Stop_distance = Stop_dist_calc(app,v,s,mass,radius,RPM_max,Torque_max, Power_Kw,transmission_factor,C_d,frontal_area, pressure,F_brake_max,Coeff_Friction,track_dist);
                    %Stop_distance = (mass*v^2)/(2*F_brake_max);
                end
                %Determine acc/dec state                
                if strcmp(state,'acc') == 1
                    state = State_conditional(app,Stop_distance,s,track_dist);
                end
                %{
                if s + Stop_distance > track_dist+1 %&& s + Stop_distance <= track_dist + 1
                    state = 'dec';
                end       
                %}
            end
            %% Define output variables
            v_top = max(vel);
            a_top = max(a_record);
            time_end = max(time);
            max_Q = max(drag_record)/(C_d*frontal_area);
            final_location = max(loc);
            
        end
        
        function [state] = State_conditional(app,Stop_distance, s,track_dist)
            if s + Stop_distance > track_dist %&& s + Stop_distance <= track_dist + 1
                state = 'dec';
            else
                state = 'acc';
            end
        end
            
        function [Stop_distance] = Stop_dist_calc(app,v,s,mass,radius, RPM_max,Torque_max, Power_Kw,transmission_factor,C_d,frontal_area, pressure,F_brake_max,Coeff_Friction, track_dist)
                %Numerical Integration variables
                dt = 0.001; %timesetp
                Stop_distance = 0; %initialize Stop_distance
                Stop_distance_history = []; %initialize history
                F_braking_history = []; %intitialize another history
                if s + ((mass*v^2)/(2*F_brake_max)) >= track_dist %this is used to speed up the calculation. If the brakes could stop it before the limit, then skip this for now. Only calculate if Stop_distance will be > Stop_distance with brakes only.
                    while v>0 % if the pod is moving...
                        Given_RPM = v *60 /(2*pi*radius); %determine RPM for Torque curve reader
                        F_braking = -(F_brake_max)-(Torque_curve_reader(app,Given_RPM,Power_Kw, Torque_max, RPM_max,transmission_factor)/radius) - DragCalc(app,v,C_d,frontal_area,pressure); % force of "braking" / deceleration = pneumatic force * coeff. Friction + motor regen (same as torque curve, but negative sign) + aero drag
                        a = F_braking/mass; %calculate decel
                        v = v + a*dt; %time step to update velocity
                        ds = v*dt; % update location based on velocity
                        Stop_distance = Stop_distance + ds; %add ds to Stop_distance. Cumulative sum = total Stop distance
                        %Debugging things I tried for a bit. All below aren't part of the function
                        F_braking_history = [F_braking_history, F_braking];
                        %v_history = [v_history, v];
                    end
                end
            end
            
            function [torque] = Torque_curve_reader(app,Given_RPM,Power_Kw, Torque_max, RPM_max,transmission_factor)
                Torque_max = Torque_max * (1/transmission_factor);
                RPM_max = RPM_max * transmission_factor;
                %% Conversions and Initilizations
                Power_w = Power_Kw*1000;
                critical_w = (Power_w)/Torque_max;
                torque = 0;
                w = (Given_RPM.*2.*pi)./60;
                w_max = (RPM_max.*2.*pi)./60;
                %% Torque Curve Range Conditional
                if w<critical_w
                    torque = Torque_max;
                elseif w > w_max
                    torque = 0;
                else
                    torque = Power_w./w;
                end
            end
            
            function [F_Drag] = DragCalc(app,velocity, C_d, frontal_area, pressure)
                
                %DragCalc
                %Function to calculate air drag on the pod. GUI takes input of Pascal and deg C - 
                %function converts deg C to deg K
                
                R = 287.05; %specific gas constant for dry air
                temp_air = 273.15; %+ temp_input; %Kelvin - eqiv. to 77F
                density = pressure/(R*temp_air);
                F_Drag = 0.5 * C_d * density * velocity^2 * frontal_area;
            end               
%}
