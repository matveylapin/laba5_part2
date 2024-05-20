%% power
power.voltage = 24;
%% dims
d_a = 80e-3;    % save diameter of motor
h_a = 27.5e-3;  % depth of motor
d_r = 0.5;      % arm length
%% kinematics
kinematics.l_1 = [     -h_a; 0;       0 ];
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
%% forward kinematics
forward_kinematics.rot_01_fn = @(ang) rotz(ang) * kinematics.E_1;
forward_kinematics.rot_12_fn = @(ang) rotz(ang) * kinematics.E_2;
forward_kinematics.rot_23_fn = @(ang) rotz(ang) * kinematics.E_3;
forward_kinematics.rot_34_fn = @(ang) rotz(ang) * kinematics.E_4;
forward_kinematics.rot_45_fn = @(ang) rotz(ang) * kinematics.E_5;
forward_kinematics.rot_56_fn = @(ang) rotz(ang) * kinematics.E_6;

forward_kinematics.rot_1_zero = kinematics.E_1;
forward_kinematics.rot_2_zero = forward_kinematics.rot_1_zero * kinematics.E_2;
forward_kinematics.rot_3_zero = forward_kinematics.rot_2_zero * kinematics.E_3;
forward_kinematics.rot_4_zero = forward_kinematics.rot_3_zero * kinematics.E_4;
forward_kinematics.rot_5_zero = forward_kinematics.rot_4_zero * kinematics.E_5;
forward_kinematics.rot_6_zero = forward_kinematics.rot_5_zero * kinematics.E_6;

forward_kinematics.rot_1_fn = @(ang1) forward_kinematics.rot_01_fn(ang1);
forward_kinematics.rot_2_fn = @(ang1, ang2) forward_kinematics.rot_01_fn(ang1) * forward_kinematics.rot_12_fn(ang2);
forward_kinematics.rot_3_fn = @(ang1, ang2, ang3) forward_kinematics.rot_2_fn(ang1, ang2)* forward_kinematics.rot_23_fn(ang3);
forward_kinematics.rot_4_fn = @(ang1, ang2, ang3, ang4) forward_kinematics.rot_3_fn(ang1, ang2, ang3) * forward_kinematics.rot_34_fn(ang4);
forward_kinematics.rot_5_fn = @(ang1, ang2, ang3, ang4, ang5) forward_kinematics.rot_4_fn(ang1, ang2, ang3, ang4) * forward_kinematics.rot_45_fn(ang5);
forward_kinematics.rot_6_fn = @(ang1, ang2, ang3, ang4, ang5, ang6) forward_kinematics.rot_5_fn(ang1, ang2, ang3, ang4, ang5) * forward_kinematics.rot_56_fn(ang6);

forward_kinematics.shift_01_fn = @(ang) forward_kinematics.rot_01_fn(ang) * kinematics.l_1;
forward_kinematics.shift_12_fn = @(ang) forward_kinematics.rot_12_fn(ang) * kinematics.l_2;
forward_kinematics.shift_23_fn = @(ang) forward_kinematics.rot_23_fn(ang) * kinematics.l_3;
forward_kinematics.shift_34_fn = @(ang) forward_kinematics.rot_34_fn(ang) * kinematics.l_4;
forward_kinematics.shift_45_fn = @(ang) forward_kinematics.rot_45_fn(ang) * kinematics.l_5;
forward_kinematics.shift_56_fn = @(ang) forward_kinematics.rot_56_fn(ang) * kinematics.l_6;

forward_kinematics.shift_1_fn = @(ang1)                               forward_kinematics.rot_1_fn(ang1) * kinematics.l_1;
forward_kinematics.shift_2_fn = @(ang1, ang2)                         forward_kinematics.shift_1_fn(ang1) + forward_kinematics.rot_2_fn(ang1, ang2) * kinematics.l_2;
forward_kinematics.shift_3_fn = @(ang1, ang2, ang3)                   forward_kinematics.shift_2_fn(ang1, ang2) + forward_kinematics.rot_3_fn(ang1, ang2, ang3) * kinematics.l_3;
forward_kinematics.shift_4_fn = @(ang1, ang2, ang3, ang4)             forward_kinematics.shift_3_fn(ang1, ang2, ang3) + forward_kinematics.rot_4_fn(ang1, ang2, ang3, ang4) * kinematics.l_4;
forward_kinematics.shift_5_fn = @(ang1, ang2, ang3, ang4, ang5)       forward_kinematics.shift_4_fn(ang1, ang2, ang3, ang4) + forward_kinematics.rot_5_fn(ang1, ang2, ang3, ang4, ang5) * kinematics.l_5;
forward_kinematics.shift_6_fn = @(ang1, ang2, ang3, ang4, ang5, ang6) forward_kinematics.shift_5_fn(ang1, ang2, ang3, ang4, ang5) + forward_kinematics.rot_6_fn(ang1, ang2, ang3, ang4, ang5, ang6) * kinematics.l_6;

