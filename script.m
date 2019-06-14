% Modelling of Fluid Power Components


%4/3 valve

%Valve Dynamics
%2nd order transfer function
Valve.secondorder_delay = 0.03;
Valve_omega = 40.5;
Valve_eta = 0.989;

%RELATIVE OPENINGS
%Look-up Tabel PA
Valve.lookupopenPA = [5.86e-4 5.86e-4 5.86e-4 0.0329 0.1003 0.1600 0.6635 1];
Valve.lookupspoolPA = [-1 -0.5 0 0.2 0.4 0.5 0.8 1];

%Look-up Tabel PB
Valve.lookupopenPB = [1 0.4951 0.1599 0.0997 0.0324 5.86e-4 5.86e-4 5.86e-4];
Valve.lookupspoolPB = [-1 -0.7 -0.5 -0.4 -0.2 0 0.5 1];

%Look-up Tabel BT
Valve.lookupopenBT = [0.00115 0.00115 0.00115 0.0347 0.1102 0.2088 0.6821 1];
Valve.lookupspoolBT = [-1 -0.5 0 0.2 0.4 0.5 0.8 1];

%Look-up Tabel AT
Valve.lookupopenAT = [1 0.5189 0.1990 0.1118 0.0343 0.00115 0.00115 0.00115];
Valve.lookupspoolAT = [-1 -0.7 -0.5 -0.4 -0.2 0 0.5 1];


%Orifice/Flowpath Parameters
Valve.QN.PA = 0.001862;             % Nominal flow rate [m3/s]
Valve.QN.BT = 0.0007461;
Valve.QN.PB = 0.0007441;
Valve.QN.AT = 0.001864;
Valve.dpN = 0.5e6;                  % Nominal pressure difference [Pa]
Valve.ptr = 0.1e6;                  % Transition pressure [Pa]

%Supply Pressure
Supply_pr_Ps = 2.1e7;               % Pressure [Pa]

%Hose Block
Hose.B = 400e6;                             % Bulk Modulus [N/m2]
Hose.d = 25e-3;                             % Diameter of hose [m]
Hose.length_A = 5;                          % Hose lenghth connected to port A [m]
Hose.length_B = 6;                          % Hose lenghth connected to port B [m]
Hose.V_A = ((pi*Hose.length_A)/4)*Hose.d^2; % Volume at port A [m3]
Hose.V_B = ((pi*Hose.length_B)/4)*Hose.d^2; % Volume at port B [m3]
Hose.pa_init = 4450249.10234683;            % Initial pressure [Pa]
Hose.pb_init = 5202222.16271062;
Hose.dpN = 0.5e6;
Hose.mu = 0.7;
Hose.Area = (pi/4)*(Hose.d)^2;
Hose.Eta = 870;
Hose.QN = Hose.mu*Hose.Area*sqrt(2*Hose.dpN/Hose.Eta);
Hose.ptr = 0.1e6;                           % Transition pressure [Pa]


% Actuator

%Cylinder
Cyl.D = 145e-3;                             % Piston diameter A-side [m]
Cyl.d = 95e-3;                              % Piston diameter B-side [m]
Cyl.A_A = pi*(Cyl.D)^2/4;                   % Piston area A-side [m2]
Cyl.V0A = 0.2e-3;                           % Dead volume at A-side [m3]
Cyl.A_B = pi*((Cyl.D)^2-(Cyl.d)^2)/4;       % Piston area B-side area [m2]
Cyl.V0B = 0.2e-3;                           % Dead volume at B-side [m3]
Cyl.B = 1323e6;                             % Effective bulk modulus [Pa]
Cyl.xmax = 1.150;                           % Stroke Length [m]
Cyl.pa_init = 4450249.10234683;             % [Pa]
Cyl.pb_init = 5202222.16271062;             % [Pa]

%Friction Model
Load.m = 1650;                              % [kg]
Cyl.F_S =  4000;                            % Static friction force [N]
Cyl.F_C =  3850;                            % Coulombic friction force [N]
Cyl.b = 10000;                              % Viscous friction coefficient [Ns/m]
Cyl.v_s = 0.8;                              % Parameter related to minimum friction vel. [m/s]
zmax = 0.1e-3;       
Cyl.sigma_0 = Cyl.F_S/zmax;                 % (such that maximum seal deformation is 0.1 mm)
Cyl.sigma_1 = 0.5*sqrt(Cyl.sigma_0*Load.m); % Damping term of seal


% Mechanism
Cyl.x_min = 2;                              % Cylinder minimum length [m]

Boom.Mass = 1000;                           % [kg]
Boom.Inert = [0 0 0; 0 0 0; 0 0 0];         % Interia
Boom.CylJoint = [2.2841 1.8972 0];          % Co-ordinates [m]
Boom.ArmJoint = [4.6501 1.0896 0];

Arm.Mass = 850;                             % [kg]
Radius.arm = 0.21;                          % [m]
Arm.Length = 3.424;                         % [m]
Arm.CG = [0.8 0.21 0];                      % [m]
Arm.Orient = [0 0 -50];                     % [deg]
Arm.CylJoint = [-0.644 0.345 0];            % [m]
Arm.BucketJoint = [2.780 0.106 0];          % [m]
Arm.Inertia = [0, 0, 0; 0, (Arm.Mass*3.424^2)/12, 0; 0, 0, (Arm.Mass*3.424^2)/12];

Bucket.Mass = 800;                          % [kg]
Bucket.CG = [0.5 0 0];                      % [m]
Radius.bucket = 0.5;                        % [m]
Bucket.Interia = [(2/5)*Bucket.Mass*Radius.bucket^2, 0, 0; 0, (2/5)*Bucket.Mass*Radius.bucket^2, 0; 0, 0, (2/5)*Bucket.Mass*Radius.bucket^2];

%Controller
p_gain = -0.14025;
