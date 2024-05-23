%% power
power.voltage = 24;
%% dims
d_a = 80e-3;    % save diameter of motor
h_a = 27.5e-3;  % depth of motor
d_r = 0.5;      % arm length
%% kinematics
kinematics.l_1 = [     -d_a; 0;       0 ];
kinematics.l_2 = [      d_r; 0;       0 ];
kinematics.l_3 = [     -d_r; 0;       0 ];
kinematics.l_4 = [        0; 0; d_a+h_a ];
kinematics.l_5 = [ -d_a-h_a; 0;     d_a ];
kinematics.l_6 = [        0; 0;       0 ];
kinematics.l_all = {kinematics.l_1 kinematics.l_2 kinematics.l_3 kinematics.l_4 kinematics.l_5 kinematics.l_6};

kinematics.E_1 = roty(-90) * rotz(180);
kinematics.E_2 = roty(180);
kinematics.E_3 = roty(180);
kinematics.E_4 = rotx(180) * roty(-90);
kinematics.E_5 = roty(-90) * rotz(180);
kinematics.E_6 = eye(3);
kinematics.E_all = {kinematics.E_1 kinematics.E_2 kinematics.E_3 kinematics.E_4 kinematics.E_5 kinematics.E_6};

kinematics.limits = deg2rad([
    [-180, 180];
    [-90, 90];
    [-160, 160];
    [-140, 140];
    [-180, 180];
    [-360, 360]
    ]);

%% forward kinematics
forward_kinematics = ForwardKinematics(kinematics.l_all, kinematics.E_all, 6);

%% inverse kinematics
inverse_kinematics = InverseKinematics(forward_kinematics, kinematics.limits);
