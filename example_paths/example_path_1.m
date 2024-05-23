command.user_points_count = 20;

command.user_poses = [
 0.7*cos(linspace(-pi/2, pi/2, command.user_points_count));  % X
 0.7*sin(linspace(-pi, pi, command.user_points_count));  % Y
 linspace(0.5, 0.5, command.user_points_count)           % Z
 ];

command.user_angles = [
 zeros(1, command.user_points_count);
 linspace(90, 90, command.user_points_count);
 zeros(1, command.user_points_count);
 ];