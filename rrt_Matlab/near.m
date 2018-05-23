function [ q_near, index] = near( q_list, q_rand, delta_d )
    % Set huge minimal and index
    min = 1e6;
    index = 0;
    [L, L_] = size(q_list);
    for i= 1: L
        if norm(q_rand - q_list(i, 1:2)) < min
            min = norm(q_rand - q_list(i, 1:2));
            index = i;
        end
    end
    q_near = q_list(index, 1:2) + (q_rand - q_list(index, 1:2))/ norm(q_rand - q_list(index, 1:2)) * delta_d;
end

