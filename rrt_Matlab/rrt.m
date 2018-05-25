clear all
clc
%% Constant definition
% Configuration space size
X_DIM = 20;
Y_DIM = 10;
% Vehicle size
l = 0.48;
% Initial point
q_init = [0, 0];
% Goal point
q_goal = [20, 10];
% Walk distance
delta_d = .5;
% Distance tolerance
epsilon = .1;
% Size of obstacle list
obs_size = 7;
% Obstacle length size list, 2 size
obs_size_list = [ 0.5, 2];
% Standard deviation for normal distribution
sigma = 10;
%% Obstacle list
% Model obstacle as a square
% format: [x, y, l]
%          x. y: center
%          l: side length 
obs_list = [ 3,   5, obs_size_list(2);
             6,   7, obs_size_list(1);
            11,   3, obs_size_list(2);
            10,   6, obs_size_list(1);
            14, 4.5, obs_size_list(1);
            16,   8, obs_size_list(2);
            18, 6.5, obs_size_list(1)];
        
%% Display
axis([q_init(1), X_DIM, q_init(2), Y_DIM]);
hold on
% Plot obstacle, color: black
for i= 1:obs_size
    plot([obs_list(i,1) - obs_list(i,3)/2, obs_list(i, 1) + obs_list(i,3)/2], [obs_list(i,2) - obs_list(i,3)/2, obs_list(i,2) - obs_list(i,3)/2], 'k');
    plot([obs_list(i,1) + obs_list(i,3)/2, obs_list(i, 1) + obs_list(i,3)/2], [obs_list(i,2) - obs_list(i,3)/2, obs_list(i,2) + obs_list(i,3)/2], 'k');
    plot([obs_list(i,1) + obs_list(i,3)/2, obs_list(i, 1) - obs_list(i,3)/2], [obs_list(i,2) + obs_list(i,3)/2, obs_list(i,2) + obs_list(i,3)/2], 'k');
    plot([obs_list(i,1) - obs_list(i,3)/2, obs_list(i, 1) - obs_list(i,3)/2], [obs_list(i,2) + obs_list(i,3)/2, obs_list(i,2) - obs_list(i,3)/2], 'k');
end
% Initial point, color: green
plot(q_init(1), q_init(2), '*g')
% Goal point, color: green
plot(q_goal(1), q_goal(2), '*g')

%% RRT Process
tic
% q_list: node list, 1:2 -> position, 3: parent
% q_init: origin position
% q_goal: goal position
% q_rand: position generated randomly in configuration space
% q_near: new q generated after obstacle checker
q_near = q_init;
% q list
q_size = 1;
q_list(1, 1:2) = q_init;
q_list(1, 3) = 1; % First node, with parent index 1
index = 1;
while (norm(q_goal - q_near) > epsilon)
    % Generate random position in configuration space
    % Using uniform distribution
    %q_rand = [rand* X_DIM, rand * Y_DIM];
    % Using normal distribution with goal as center
    q_rand = [normrnd(X_DIM, sigma), normrnd(Y_DIM, sigma)];
    % Plot q_rand, color: red
    %plot(q_rand(1), q_rand(2), '*r')
    [q_near, index] = near(q_list, q_rand, delta_d);
    % check bound, make sure node won't exceed the configuration space
    if(q_near(1) > X_DIM) 
        q_near(1) = X_DIM;
    end
    if(q_near(1) < 0)
        q_near(1) = 0;
    end
    if(q_near(2) > Y_DIM) 
        q_near(2) = Y_DIM;
    end
    if(q_near(2) < 0)
        q_near(2) = 0;
    end
    %plot([q_list(index, 1), q_rand(1)], [q_list(index, 2), q_rand(2)], '--k')
    % Add 0.2 for prevent collision in real world
    if(is_hit_constrain(q_list(index, 1:2), q_near, l+0.2, obs_size, obs_list) == 0) % Does not hit the constrain
        % q_size += 1
        q_size = q_size + 1;
        % Append new point to q_list
        q_list(q_size, 1:2) = q_near;
        q_list(q_size, 3) = index; % Parent node index
        % Update plot
        line([q_list(index, 1), q_near(1)], [q_list(index, 2), q_near(2)]);
        plot(q_near(1),q_near(2), '*b')
        drawnow
    end
    
end
%% Find path via inverse tracking
% Process pseudocode: 
% child <- q_size
% parent <- q_list(q_size,3)
% while (child != 1):
%   plot line between child node and parent node
%   waypoint_size += 1
%   waypoint_index_list.append(parent)
%   temp = parent
%   parent = q_list(temp, 3)
%   child = parent
waypoint_size = 1;
waypoint_index_list = [q_size]; % Last node as first element
child_index = q_size;
parent_index = q_list(q_size,3);
while(child_index ~= 1)
    plot([q_list(child_index, 1), q_list(parent_index, 1)],[q_list(child_index, 2), q_list(parent_index, 2)], 'r')
    theta = atan2(q_list(child_index, 2) - q_list(parent_index, 2), q_list(child_index, 1) - q_list(parent_index, 1));
    % Plot vehicle
    plot([l/2*cos(theta)+l/2*sin(theta)+q_list(parent_index, 1), l/2*cos(theta)-l/2*sin(theta)+q_list(parent_index, 1)],[l/2*sin(theta)-l/2*cos(theta)+q_list(parent_index, 2), l/2*sin(theta)+l/2*cos(theta)+q_list(parent_index, 2)], 'r')
    plot([-l/2*cos(theta)+l/2*sin(theta)+q_list(parent_index, 1), -l/2*cos(theta)-l/2*sin(theta)+q_list(parent_index, 1)],[-l/2*sin(theta)-l/2*cos(theta)+q_list(parent_index, 2), -l/2*sin(theta)+l/2*cos(theta)+q_list(parent_index, 2)], 'r')
    plot([l/2*cos(theta)-l/2*sin(theta)+q_list(parent_index, 1), -l/2*cos(theta)-l/2*sin(theta)+q_list(parent_index, 1)],[l/2*sin(theta)+l/2*cos(theta)+q_list(parent_index, 2), -l/2*sin(theta)+l/2*cos(theta)+q_list(parent_index, 2)], 'r')
    plot([l/2*cos(theta)+l/2*sin(theta)+q_list(parent_index, 1), -l/2*cos(theta)+l/2*sin(theta)+q_list(parent_index, 1)],[l/2*sin(theta)-l/2*cos(theta)+q_list(parent_index, 2), -l/2*sin(theta)-l/2*cos(theta)+q_list(parent_index, 2)], 'r')
    waypoint_size = waypoint_size + 1;
    waypoint_index_list(waypoint_size) = parent_index;
    temp = parent_index;
    parent_index = q_list(temp, 3);
    child_index = temp;
    drawnow
end

waypoint_list = zeros(waypoint_size, 2);
for i=fliplr(1:waypoint_size)
    waypoint_list(waypoint_size+1-i, :) = q_list(waypoint_index_list(i), 1:2)
end
toc
waypoint_size
waypoint_list
