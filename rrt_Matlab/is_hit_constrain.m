function [ result ] = is_hit_constrain( q_now, q_near, obs_size, obs_list )
    % Check if the line segment from q_now to q_near hit the obstacle
    % Return 1 if true, and 0 otherwise
    result = 0;
    counter = 1;
    while(counter <= obs_size)
        for i=1:obs_size
            bound = [obs_list(i, 1) - obs_list(i, 3)/ 2;    % x_l
                     obs_list(i, 1) + obs_list(i, 3)/ 2;    % x_r
                     obs_list(i, 2) - obs_list(i, 3)/ 2;    % y_d
                     obs_list(i, 2) + obs_list(i, 3)/ 2];   % y_u
            for bound_checker = 1:2
                t = (bound(bound_checker)-q_now(1))/(q_near(1)-q_now(1));
                s = (q_now(2)-bound(3) + (q_near(2)-q_now(2))*t) / (bound(4)-bound(3));
                if(t>=0 && t<=1 && s>=0 && s<=1)
                    result = 1;
%                     disp('x break');disp(bound_checker)
%                     disp('q_now: ');disp(q_now)
%                     disp('q_near: ');disp(q_near)
                    break;
                end
            end
            for bound_checker = 3:4
                t = (bound(bound_checker)-q_now(2))/(q_near(2)-q_now(2));
                s = (q_now(1)-bound(1) + (q_near(1)-q_now(1))*t) / (bound(2)-bound(1));
                if(t>=0 && t<=1 && s>=0 && s<=1)
                    result = 1;
%                     disp('y break '); disp(bound_checker)
%                     disp('q_now: ');disp(q_now)
%                     disp('q_near: ');disp(q_near)
                    break;
                end
            end
        counter = counter + 1;
        end
    end
end

