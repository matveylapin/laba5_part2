classdef ForwardKinematics

    properties(Access = private)
        links_vectors__;
        joins_rots__;
        joins_count__ {mustBeNumeric};
        
        Rx__ = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
        Ry__ = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
        Rz__ = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];

    end
    
    methods
        function obj = ForwardKinematics(links_vectors, joins_rots, joins_count)

            obj.links_vectors__ = links_vectors;
            obj.joins_rots__ = joins_rots;
            obj.joins_count__ = joins_count;
        end

        function full_rot = rot_between_frames(obj, parent_frame, child_frame, angles)
            parent_frame_id = parent_frame + 1;
            child_frame_id = child_frame + 1;
            outputArg = eye(3);
            for id = parent_frame_id:(child_frame_id-1)
                outputArg = outputArg * obj.Rz__(angles(id)) * obj.joins_rots__{id}; % TODO: mabye not only z rotation
            end
            full_rot = outputArg;
        end

        function full_rot = rot_to_frame(obj, child_frame, angles)
            full_rot = obj.rot_between_frames(0, child_frame, angles);
        end

        function zero_full_rot = zero_rot_between_frames(obj, parent_frame, child_frame)
            zero_full_rot = obj.rot_between_frames(parent_frame, child_frame, zeros(1, obj.joins_count__));
        end

        function zero_full_rot = zero_rot_to_frame(obj, child_frame)
            zero_full_rot = obj.rot_between_frames(0, child_frame, zeros(1, obj.joins_count__));
        end

        function shift = one_link_shift(obj, link, angle)
            link_id = link;
            angles = zeros(1, obj.joins_count__);
            angles(link_id) = angle;
            shift = obj.rot_between_frames(link_id-1, link_id, angles) * obj.links_vectors__{link_id};
        end

        function shift = shift_between_frames(obj, parent_frame, child_frame, angles)
            first_link = parent_frame + 1;
            last_link = child_frame;
            outputArg = zeros(3,1);
            for link = first_link:(last_link)
                outputArg = outputArg + obj.rot_between_frames(parent_frame, link, angles) * obj.links_vectors__{link};
            end
            shift = outputArg;
        end

        function shift = shift_to_frame(obj, child_frame, angles)
            shift = obj.shift_between_frames(0, child_frame, angles);
        end

        function shift = zero_shift_between_frames(obj, parent_frame, child_frame)
            shift = obj.shift_between_frames(parent_frame, child_frame, zeros(1, obj.joins_count__));
        end

        function joints_count = get_joins_count(obj)
            joints_count = obj.joins_count__;
        end
    end
end

