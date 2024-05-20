classdef ForwardKinematics_test < matlab.unittest.TestCase

    properties
        d_a = 80e-3;
        h_a = 27.5e-3;
        d_r = 0.5;
        
        l_1;
        l_2;
        l_3;
        l_4;
        l_5;
        l_6;
        l_all;

        E_1;
        E_2;
        E_3;
        E_4;
        E_5;
        E_6;
        E_all;

        fk;

        rot_01_fn;
        rot_12_fn;
        rot_23_fn;
        rot_34_fn;
        rot_45_fn;
        rot_56_fn;

        rot_1_fn;
        rot_2_fn;
        rot_3_fn;
        rot_4_fn;
        rot_5_fn;
        rot_6_fn;

        shift_01_fn;
        shift_12_fn;
        shift_23_fn;
        shift_34_fn;
        shift_45_fn;
        shift_56_fn;
    end

    methods(TestClassSetup)

        function lengths_set(testCase)
            testCase.l_1 = [     -testCase.h_a; 0;       0 ];
            testCase.l_2 = [      testCase.d_r; 0;       0 ];
            testCase.l_3 = [     -testCase.d_r; 0;       0 ];
            testCase.l_4 = [        0; 0; testCase.d_a+testCase.h_a ];
            testCase.l_5 = [ -testCase.d_a-testCase.h_a; 0;     testCase.d_a ];
            testCase.l_6 = [        0; 0;       0 ];

            testCase.l_all = {testCase.l_1 testCase.l_2 testCase.l_3 testCase.l_4 testCase.l_5 testCase.l_6};
        end

        function rots_set(testCase)
            testCase.E_1 = roty(-90) * rotz(180);
            testCase.E_2 = roty(180);
            testCase.E_3 = roty(180);
            testCase.E_4 = rotx(180) * roty(-90);
            testCase.E_5 = roty(-90) * rotz(180);
            testCase.E_6 = eye(3);

            testCase.E_all = {testCase.E_1 testCase.E_2 testCase.E_3 testCase.E_4 testCase.E_5 testCase.E_6};
        end

        function functions_set(testCase)
            testCase.rot_01_fn = @(ang) rotz(ang) * testCase.E_1;
            testCase.rot_12_fn = @(ang) rotz(ang) * testCase.E_2;
            testCase.rot_23_fn = @(ang) rotz(ang) * testCase.E_3;
            testCase.rot_34_fn = @(ang) rotz(ang) * testCase.E_4;
            testCase.rot_45_fn = @(ang) rotz(ang) * testCase.E_5;
            testCase.rot_56_fn = @(ang) rotz(ang) * testCase.E_6;

            testCase.rot_1_fn = @(ang1) testCase.rot_01_fn(ang1);
            testCase.rot_2_fn = @(ang1, ang2) testCase.rot_01_fn(ang1) * testCase.rot_12_fn(ang2);
            testCase.rot_3_fn = @(ang1, ang2, ang3) testCase.rot_2_fn(ang1, ang2)* testCase.rot_23_fn(ang3);
            testCase.rot_4_fn = @(ang1, ang2, ang3, ang4) testCase.rot_3_fn(ang1, ang2, ang3) * testCase.rot_34_fn(ang4);
            testCase.rot_5_fn = @(ang1, ang2, ang3, ang4, ang5) testCase.rot_4_fn(ang1, ang2, ang3, ang4) * testCase.rot_45_fn(ang5);
            testCase.rot_6_fn = @(ang1, ang2, ang3, ang4, ang5, ang6) testCase.rot_5_fn(ang1, ang2, ang3, ang4, ang5) * testCase.rot_56_fn(ang6);
            
            testCase.shift_01_fn = @(ang) testCase.rot_01_fn(ang) * testCase.l_1;
            testCase.shift_12_fn = @(ang) testCase.rot_12_fn(ang) * testCase.l_2;
            testCase.shift_23_fn = @(ang) testCase.rot_23_fn(ang) * testCase.l_3;
            testCase.shift_34_fn = @(ang) testCase.rot_34_fn(ang) * testCase.l_4;
            testCase.shift_45_fn = @(ang) testCase.rot_45_fn(ang) * testCase.l_5;
            testCase.shift_56_fn = @(ang) testCase.rot_56_fn(ang) * testCase.l_6;
        end

        function class_constructor(testCase)
            testCase.fk = ForwardKinematics(testCase.l_all, testCase.E_all, 6);
        end
    end
    
    methods(TestMethodSetup)
        % Setup for each test
    end
    
    methods(Test)
        % Test methods
        
        function rot_between_frames_Test(testCase)
            testCase.assertEqual(testCase.fk.rot_between_frames(0, 1, zeros(1, 6)), testCase.rot_01_fn(0));
            testCase.assertEqual(testCase.fk.rot_between_frames(1, 2, zeros(1, 6)), testCase.rot_12_fn(0));
            testCase.assertEqual(testCase.fk.rot_between_frames(2, 3, zeros(1, 6)), testCase.rot_23_fn(0));
            testCase.assertEqual(testCase.fk.rot_between_frames(3, 4, zeros(1, 6)), testCase.rot_34_fn(0));
            testCase.assertEqual(testCase.fk.rot_between_frames(4, 5, zeros(1, 6)), testCase.rot_45_fn(0));
            testCase.assertEqual(testCase.fk.rot_between_frames(5, 6, zeros(1, 6)), testCase.rot_56_fn(0));
        end

        function zero_rot_to_frame_Test(testCase)
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(0), eye(3));
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(1), testCase.rot_1_fn(0));
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(2), testCase.rot_2_fn(0, 0));
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(3), testCase.rot_3_fn(0, 0, 0));
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(4), testCase.rot_4_fn(0, 0, 0, 0));
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(5), testCase.rot_5_fn(0, 0, 0, 0, 0));
            testCase.assertEqual(testCase.fk.zero_rot_to_frame(6), testCase.rot_6_fn(0, 0, 0, 0, 0, 0));
        end

        function rot_to_frame_Test(testCase)
            angles = (rand(1, 6)-0.5)*pi;
            testCase.assertEqual(testCase.fk.rot_to_frame(0, angles), eye(3));
            testCase.assertEqual(testCase.fk.rot_to_frame(1, angles), testCase.rot_1_fn(angles(1)));
            testCase.assertEqual(testCase.fk.rot_to_frame(2, angles), testCase.rot_2_fn(angles(1), angles(2)));
            testCase.assertEqual(testCase.fk.rot_to_frame(3, angles), testCase.rot_3_fn(angles(1), angles(2), angles(3)));
            testCase.assertEqual(testCase.fk.rot_to_frame(4, angles), testCase.rot_4_fn(angles(1), angles(2), angles(3), angles(4)));
            testCase.assertEqual(testCase.fk.rot_to_frame(5, angles), testCase.rot_5_fn(angles(1), angles(2), angles(3), angles(4), angles(5)));
            testCase.assertEqual(testCase.fk.rot_to_frame(6, angles), testCase.rot_6_fn(angles(1), angles(2), angles(3), angles(4), angles(5), angles(6)));
        end
        
        function link_shift_Test(testCase)
            angles = (rand(1, 6)-0.5)*pi;
            testCase.assertEqual(testCase.fk.one_link_shift(1, angles(1)), testCase.shift_01_fn(angles(1)));
            testCase.assertEqual(testCase.fk.one_link_shift(2, angles(2)), testCase.shift_12_fn(angles(2)));
            testCase.assertEqual(testCase.fk.one_link_shift(3, angles(3)), testCase.shift_23_fn(angles(3)));
            testCase.assertEqual(testCase.fk.one_link_shift(4, angles(4)), testCase.shift_34_fn(angles(4)));
            testCase.assertEqual(testCase.fk.one_link_shift(5, angles(5)), testCase.shift_45_fn(angles(5)));
            testCase.assertEqual(testCase.fk.one_link_shift(6, angles(6)), testCase.shift_56_fn(angles(6)));
        end
    end
    
end