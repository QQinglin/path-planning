
%% Initialize conditions
clc
clear all; close all;
x_I=1; y_I=1;           % initialize start point
x_G=1600; y_G=1600;       % initialize goal point
Thr=50;                 % initialize goal threshold value
Delta= 30;              % initialize expanding step size
R_rewire = 60;          % radius for rewiring
%% Initialize the tree
T.v(1).x = x_I;         % T is the tree, v is starting node. put the starting node in T
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % the father node of the starting node is self 
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % take the euclidean distance as the distance between father node to subnode 
T.v(1).indPrev = 0;     %
%% expanding tree
figure(1);
ImpRgb=imread('map.png');
% Imp=rgb2gray(ImpRgb);
imshow(ImpRgb)
xL=size(ImpRgb,2); % length the x axis of the map
yL=size(ImpRgb,1); % length the y axis of the map
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g'); % plot starting node and goal node in the map 
count=1;
bFind = false;

for iter = 1:1000
    x_rand=[];
    % Step 1: Sample a random point in the map
    x_rand(1) = randi([1, xL]); % Random x_coordinate within map bounds
    x_rand(2) = randi([1, yL]); % Random y_coordinate within map bounds  
    % plot the random point 
    plot(x_rand(1),x_rand(2), 'bo','MarkerSize',5,'MarkerFaceColor','b');
    
    x_near=[];
    % Step 2: Traversal the tree to find the nearest node to the sampled
    min_dist = inf;
    nearest_node = 1;
    for i = 1:length(T.v)
        dist = (T.v(i).x - x_rand(1))^2 + (T.v(i).y - x_rand(2))^2;
        if dist < min_dist
            min_dist = dist;
            nearest_node = i;
            x_near = [T.v(nearest_node).x,T.v(nearest_node).y];
        end
    end
    

    x_new = [];
    % Step 3: Expanding to get teh x_new node
    % Utilize the expanding step size Delta
    % direction = [x_rand(1) - T.v(nearest_node).x ; x_rand(2) - T.v(nearest_node).y]; % [6-3,8-4]
    % distance = norm(direction); % euclidean distance sqrt(3^2 + 4^2)
    % direction = direction / distance;
    % x_new = x_near + Delta * direction;
    
    direction = [x_rand(1) - T.v(nearest_node).x ; x_rand(2) - T.v(nearest_node).y];
    dist = norm(direction);
    x_new(1) = ((dist - Delta)*x_near(1) + Delta*x_rand(1)) / dist;
    x_new(2) = ((dist - Delta)*x_near(2) + Delta*x_rand(2)) / dist;
    
    % Ensure the new node is within map bounds
    if x_new(1) < 1 || x_new(1) > xL || x_new(2) < 1 || x_new(2) > yL
        continue;
    end

    % checking collision-free
    if ~collisionChecking(x_new,x_near,ImpRgb)
        continue;
    end

    count=count+1;
    
    %Step 4: expand the tree with the node x_new 
    count = count + 1;
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);
    T.v(count).xPrev = x_near(2);
    T.v(count).dist = Delta ;
    T.v(count).indPrev = nearest_node; 
    
    %Step 5: draw the path between n_near and x_new
    plot(x_new(1),x_new(2), 'bo','MarkerSize',5,'MarkerFaceColor','b');
     plot([x_new(1),x_near(1)], [x_new(2),x_near(2)], 'b-');
    
    %Step 6: Check if it is near to goal point with Thr 
    if norm(x_new(1) - x_G,x_new(2) - y_G) < Thr
        bFind = true;
        break
        % plot([x_new(1),x_G], [x_new(2),y_G], 'b-');
        % title('RRT');
        %continue;
    end
%% Rewiring step
    for j = 1:length(T.v)
        newdist = sqrt((T.v(i).x-x_new(1))^2 + (T.v(i).y - x_new(2))^2);
        if newdist < R_rewire
            if T.v(i).dist > newdist + Delta
                T.v(i).xPrev = x_new(1);
                T.v(i).yPrev = x_new(2);
                T.v(i).dist = newdist + Delta;
                T.v(i).indPrev = count;
                plot([T.v(i).x, x_new(1)], [T.v(i).y, x_new(2)], 'g-');
            end
        end
    end
    pause(0.05); % have a break
end
%% backtrack from the goal node to the start node
if bFind
    path.pos(1).x = x_G; % from the goal point
    path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; % fromt the goal point to the last x_new()
    path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % the last third point 
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev; % until the second point
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % backtrack from the goal node to the start node 
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % start point join
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
