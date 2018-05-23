function [ result ] = is_hit_constrain( q_now, q_near, obs_size, obs_list )
    result = 0;
    counter = 1;
    while(counter <= obs_size)
        for i=1:obs_size
            bound = [obs_list(i, 1) - obs_list(i, 3)/ 2;
                     obs_list(i, 1) + obs_list(i, 3)/ 2;
                     obs_list(i, 2) - obs_list(i, 3)/ 2;
                     obs_list(i, 2) + obs_list(i, 3)/ 2];
            for bound_checker = 1:2
                % Inter
                if(0<= (bound(bound_checker)-q_now(1))/(q_near(1)-q_now(1)) && (bound(bound_checker)-q_now(1))/(q_near(1)-q_now(1)) <= 1 && ...
                   bound(3)<= q_now(2)+(bound(bound_checker)-q_now(1))/(q_near(1)-q_now(1))*(q_near(2)-q_now(2)) && q_now(2)+(bound(bound_checker)-q_now(1))/(q_near(1)-q_now(1))*(q_near(2)-q_now(2))<= bound(4))
                    result = 1;
%                     disp('x break');disp(bound_checker)
%                     disp('q_now: ');disp(q_now)
%                     disp('q_near: ');disp(q_near)
                    break;
                end
            end
            for bound_checker = 3:4
                if(0< (bound(bound_checker)-q_now(2))/(q_near(2)-q_now(2)) && (bound(bound_checker)-q_now(2))/(q_near(2)-q_now(2)) < 1 && ...
                   bound(1)<= q_now(1)+(bound(bound_checker)-q_now(2))/(q_near(2)-q_now(2))*(q_near(1)-q_now(1)) && q_now(1)+(bound(bound_checker)-q_now(2))/(q_near(2)-q_now(2))*(q_near(1)-q_now(1)) <= bound(2))
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

