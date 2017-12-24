mass = input('Mass (kg): ');
trialDist = input('Trial Distance (m): ');
torque = input('Torque (N*m): ');
radius = input('radius (m): ');
forceBrakePneumatic = input('Pneumatic Acutator Force (N): ');
Coeff_Friction = input('Brake Pad Coefficient of Friction: ');
F_acc = torque * radius;
F_dec = forceBrakePneumatic * Coeff_Friction;

dist_acc = (trialDist/((F_acc/F_dec)+1));
v_max = ((2*(dist_acc*F_acc))/mass)^(.5);

