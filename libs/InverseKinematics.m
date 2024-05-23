classdef InverseKinematics

    properties(Access = private)
        fk__ ForwardKinematics
        limits__
    end

    properties
        state_vector;
    end
    
    methods
        function obj = InverseKinematics(fk, limits)
           obj.fk__ = fk;
           obj.limits__ = limits;
        end
        
        function target_thetas = joint_goto(obj, pose, rotation)
            lsp_fn = @(theta) [
                (obj.fk__.rot_to_frame(obj.fk__.get_joins_count(), theta)) - (rotation), ...
                obj.fk__.shift_to_frame(obj.fk__.get_joins_count(), theta) - pose
            ];
            target_thetas = lsqnonlin(lsp_fn, obj.state_vector, obj.limits__(:, 1), obj.limits__(:, 2));
        end

        function sequence = joint_sequence(obj, poses, angles, points_count)

            thetas = zeros(points_count, obj.fk__.get_joins_count());
            obj.state_vector = zeros(1, obj.fk__.get_joins_count());

            for cmd_id = 1:points_count
                rot_matrix = rotx(angles(1, cmd_id)) * ...
                             roty(angles(2, cmd_id)) * ...
                             rotz(angles(3, cmd_id));
                thetas(cmd_id, :) = obj.joint_goto(poses(:, cmd_id), rot_matrix);
                obj.state_vector = thetas(cmd_id, :);
            end

            sequence = thetas;
        end

        function obj = set.state_vector(obj, current_state)
            obj.state_vector = current_state;
        end
    end
end

