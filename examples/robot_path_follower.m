%% user block
path_vatiant = 'example_path_1';

%% no user block

run(path_vatiant);

robot__.commands = inverse_kinematics.joint_sequence(command.user_poses, command.user_angles, command.user_points_count);
robot__.points_count = command.user_points_count;

sim('models/robot_main.slx')