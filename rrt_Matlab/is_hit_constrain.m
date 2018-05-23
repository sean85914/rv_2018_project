function [ result ] = is_hit_constrain( q_now, q_near, l, obs_size, obs_list )
    % Check if the line segment from q_now to q_near hit the obstacle
    % Return 1 if true, and 0 otherwise
    result = 0;
    counter = 1;
    theta = atan2(q_near(2) - q_now(2), q_near(1) - q_now(1));
    t = zeros(16, 1);
    s = zeros(16, 1);
    while(counter <= obs_size)
        for obs_index=1:obs_size
            bound = [obs_list(obs_index, 1) - obs_list(obs_index, 3)/ 2;    % x_l
                     obs_list(obs_index, 1) + obs_list(obs_index, 3)/ 2;    % x_r
                     obs_list(obs_index, 2) - obs_list(obs_index, 3)/ 2;    % y_d
                     obs_list(obs_index, 2) + obs_list(obs_index, 3)/ 2];   % y_u
            
            t(1) = (q_near(1)-bound(1)+l/2*cos(theta))/sin(theta); s(1) = (q_near(2)-bound(3)+t(1)*cos(theta)+l/2*sin(theta))/(bound(4)-bound(3));
            t(2) = (q_near(1)-bound(2)+l/2*cos(theta))/sin(theta); s(2) = (q_near(2)-bound(3)+t(2)*cos(theta)+l/2*sin(theta))/(bound(4)-bound(3));
            t(3) = (bound(3)-q_near(2)-l/2*sin(theta))/cos(theta); s(3) = (q_near(1)-bound(1)-t(3)*sin(theta)+l/2*cos(theta))/(bound(2)-bound(1));
            t(4) = (bound(4)-q_near(2)-l/2*sin(theta))/cos(theta); s(4) = (q_near(1)-bound(1)-t(4)*sin(theta)+l/2*cos(theta))/(bound(2)-bound(1));
            
            t(5) = (q_near(1)-bound(1)-l/2*cos(theta))/sin(theta); s(5) = (q_near(2)-bound(3)+t(5)*cos(theta)-l/2*sin(theta))/(bound(4)-bound(3));
            t(6) = (q_near(1)-bound(2)-l/2*cos(theta))/sin(theta); s(6) = (q_near(2)-bound(3)+t(6)*cos(theta)-l/2*sin(theta))/(bound(4)-bound(3));
            t(7) = (bound(3)-q_near(2)+l/2*sin(theta))/cos(theta); s(7) = (q_near(1)-bound(1)-t(7)*sin(theta)-l/2*cos(theta))/(bound(2)-bound(1));
            t(8) = (bound(4)-q_near(2)+l/2*sin(theta))/cos(theta); s(8) = (q_near(1)-bound(1)-t(8)*sin(theta)-l/2*cos(theta))/(bound(2)-bound(1));
            
            t(9) = (bound(1)-q_near(1)+l/2*sin(theta))/cos(theta); s(9) = (q_near(2)-bound(3)+t(9)*sin(theta)+l/2*cos(theta))/(bound(4)-bound(3));
            t(10)= (bound(2)-q_near(1)+l/2*sin(theta))/cos(theta); s(10)= (q_near(2)-bound(3)+t(10)*sin(theta)+l/2*cos(theta))/(bound(4)-bound(3)); 
            t(11)= (bound(3)-q_near(2)-l/2*cos(theta))/sin(theta); s(11)= (q_near(1)-bound(1)+t(11)*cos(theta)-l/2*sin(theta))/(bound(2)-bound(1));
            t(12)= (bound(4)-q_near(2)-l/2*cos(theta))/sin(theta); s(12)= (q_near(1)-bound(1)+t(12)*cos(theta)-l/2*sin(theta))/(bound(2)-bound(1));
            
            t(13)= (bound(1)-q_near(1)-l/2*sin(theta))/cos(theta); s(13)= (q_near(2)-bound(3)+t(13)*sin(theta)-l/2*cos(theta))/(bound(4)-bound(3));
            t(14)= (bound(2)-q_near(1)-l/2*sin(theta))/cos(theta); s(14)= (q_near(2)-bound(3)+t(14)*sin(theta)-l/2*cos(theta))/(bound(4)-bound(3));
            t(15)= (bound(3)-q_near(2)+l/2*cos(theta))/sin(theta); s(15)= (q_near(1)-bound(1)+t(15)*cos(theta)+l/2*sin(theta))/(bound(2)-bound(1));
            t(16)= (bound(4)-q_near(2)+l/2*cos(theta))/sin(theta); s(16)= (q_near(1)-bound(1)+t(16)*cos(theta)+l/2*sin(theta))/(bound(2)-bound(1));
            
            for parameter_checker = 1:16
                if(t(parameter_checker)>= -l/2 && t(parameter_checker)<=l/2 && s(parameter_checker)>=0 && s(parameter_checker)<=1)
                    result = 1;
                end % end if
            end % end for
        counter = counter + 1;
        end % end obs_index
    end % end while
end % end function