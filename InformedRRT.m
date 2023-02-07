clc;clear;close all;
%% GLOBAL CONSTANTS: SET BEFORE RUNNING %%
x_max = 200; y_max = 200;
x_min = 0; y_min = 0;
x_start = 20; y_start = 50; theta_start = 0;
x_goal = 81; y_goal = 55; theta_goal = 90*pi/180;
l_car = 7.5; w_car = 3;
parking_spot_to_car_ratio = 2.4;
max_iter = 2000;
bias = 0.05;
angle_weighting = 5;

numofpathfound = 0;

obstacle1 = [35 45 10 35];
obstacle2 = [40 10 20 15];

%define parking spot based on above constants
parking_width = w_car*parking_spot_to_car_ratio;
parking_length = l_car*parking_spot_to_car_ratio;
thres_radius = parking_width*1.5;
X0 = x_goal - parking_width/2; 
X1 = x_goal + parking_width/2;
Y0 = y_goal - parking_length/2;
Y1 = y_goal + parking_length/2;

Y3 = y_goal - 1.5*parking_length;
obstacle3 = [X0 Y1 parking_width parking_length];  %parallel parking spot above
obstacle4 = [X0 Y3 parking_width parking_length];  %parallel parking spot below       
X2 = x_goal - 1.5*w_car*parking_spot_to_car_ratio;
obstacle5 = [X2 Y3 parking_width 3*parking_length];  %wall on left

th_max = 2*pi();
flag_success = 0;
flag_near_goal = 0;
goal = 0;

%non-holonomic values
dt = 0.1;
v_range_far = 1:1:10; 
v_range_near = thres_radius*(-1):0.1:thres_radius; 
steer_range_far = -0.8:0.2:0.8; %-45 to 45 deg
steer_range_near = -1:0.1:1;   %-60 to 60 deg
max_steps = 10; 

%% Plotting graphs
figure(1)
hold on
xlim([x_min x_max]); ylim([y_min y_max]); % Set bounds
axis square
plot(x_start, y_start, 'kx', 'MarkerSize', 15, 'LineWidth',3); % Start
%plot(x_goal, y_goal, 'g+', 'MarkerSize', 15); % goal
goal_circle = [x_goal-thres_radius, y_goal-thres_radius, 2*thres_radius, 2*thres_radius];
rectangle('Position', goal_circle, 'Curvature', [1,1], 'LineStyle', '--', 'EdgeColor', 'c'); % goal threshhold, plots circle
rectangle('Position', [X0 Y0 parking_width parking_length], 'EdgeColor', 'k', 'LineWidth', 2); % parking space
rectangle('Position', obstacle1, 'FaceColor', [.7 .7 .7]); % Obstacles
rectangle('Position', obstacle2, 'FaceColor', [.7 .7 .7]); % Obstacles
rectangle('Position', obstacle3, 'FaceColor', [.7 .7 .7]); % Obstacles
rectangle('Position', obstacle4, 'FaceColor', [.7 .7 .7]); % Obstacles
rectangle('Position', obstacle5, 'FaceColor', [.7 .7 .7]); % Obstacles

drawnow;

z = zeros(1,1);
node = struct('x',z,'y',z,'theta',z,'x_par',z,'y_par',z,'th_par',z,'pv',z,'pa',z,'pt',z,'dist',z,'index',z,'p_index',z,'path',z);
nodes_created = 1;
node(1).x = x_start; node(1).y = y_start; node(1).theta = theta_start;
node(1).x_par = x_start; node(1).y_par = y_start; node(1).th_par = theta_start;
node(1).pv = 0; node(1).pa = 0; node(1).pt = 0; 
node(1).dist = 0; node(1).index = 1; node(1).p_index = 0;

%% Tree generation
iter = 2;
while (iter <= max_iter)
    if goal
        %if goal is reached, have the newly generated node stay inside an eclipse
        cmin = sqrt( (x_goal - x_start)^2 + (y_goal - y_start)^2);
        if cbest > sqrt(cbest^2-cmin^2)
            a=cbest/2;
            b=sqrt(cbest^2-cmin^2)/2;
        else
            b=cbest/2;
            a=sqrt(cbest^2-cmin^2)/2; 
        end

        e_theta = -pi/2 + pi*rand;
        kk=(a*b)/(sqrt((b*cos(e_theta))^2 + (a*sin(e_theta))^2));
        max = round(kk);
        r = randi(max);
        inclination = atan((y_goal - y_start)/(x_goal - x_start));

        if mod(r,2) == 0
            xpoint = x_start + r*cos(e_theta + inclination);
            ypoint = y_start + r*sin(e_theta + inclination);
        else
            xpoint = x_goal - r*cos(e_theta + inclination);
            ypoint = y_goal - r*sin(e_theta + inclination);
        end
        x_rand = round(xpoint);
        y_rand = round(ypoint);
        th_rand = round(th_max*rand());
    else
    % Random Points
    x_rand = round(x_max*rand()); y_rand = round(y_max*rand()); th_rand = round(th_max*rand());

    % Biasing towards goal only when unsuccesful
    if flag_success == 0 && flag_near_goal && rand() < 0.5
        x_rand = x_goal - 2*thres_radius + 4*thres_radius*rand();
        y_rand = y_goal - 2*thres_radius + 4*thres_radius*rand();
        th_rand = mod(theta_goal - pi/180*15 + pi/180*30*rand(), 2*pi);
        if(rand()<0.5)
            th_rand = mod(-th_rand, 2*pi);
        end
    end
    end

    distance = zeros(nodes_created,1);
    for i = 1:nodes_created
        distance(i) = sqrt((x_rand-node(i).x)^2 + (y_rand-node(i).y)^2 + (parking_width/pi*angdiff(th_rand,node(i).theta))^2); %Euclidean distance + Angle * weighting
    end
    [~,parent_index] = min(distance);

    x_close = node(parent_index).x;
    y_close = node(parent_index).y;
    theta_close = node(parent_index).theta;

    % Motion Primitives
    path = zeros(max_steps, 3);
    path(1,:) = [x_close, y_close, theta_close];
    path_points = zeros(max_steps, 6); % x,y,theta,vel,angle,dt
    path_points(1,:) = [x_close, y_close, mod(theta_close,2*pi), 0, 0, 0]; 
    
    pp_iter = 2;
    
    %if near goal, negative velocity allowed, steer range more precise, set goal directly on target
    %if sqrt((x_close-x_goal)^2+(y_close-y_goal)^2) <= thres_radius
    
    %%x_rand = round(x_max*rand()); y_rand = round(y_max*rand()); th_rand = round(th_max*rand());

    if (sqrt((x_close-x_goal)^2+(y_close-y_goal)^2) <= thres_radius)
        v_range = v_range_near;
        steer_range = steer_range_near;
    else
        v_range = v_range_far;
        steer_range = steer_range_far;
    end
    
    %find best path from parent towards randomly sampled point
    for v_iter = v_range
        for st_iter = steer_range
            for i = 2:max_steps
                path(i,:) = [path(i-1,1)+v_iter*cos(path(i-1,3))*dt, path(i-1,2)+v_iter*sin(path(i-1,3))*dt, path(i-1,3)+v_iter*tan(st_iter)*dt/l_car];
                if IsObstacle(path(i,1),path(i,2), obstacle1,obstacle2,obstacle3,obstacle4,obstacle5, x_min,x_max,y_min,y_max) == 1
                    break
                end
                path_points(pp_iter,:)=[path(i,:),v_iter,st_iter,(i-1)*dt];
                pp_iter = pp_iter + 1;
            end
        end
    end

    %scatter(path_points(:,1),path_points(:,2))
    dist_pp = zeros(size(path_points,1),1);
    for i = 1:size(path_points,1)
        dist_pp(i) = sqrt( (x_rand - path_points(i,1))^2 + (y_rand - path_points(i,2))^2 + (parking_width/pi*angdiff(th_rand,path_points(i,3)))^2 ); %Euclidean distance + Angle * weighting
    end
    [dist_pp_value, pp_parent_index] = min(dist_pp(2:end));

    %check if node repeats
    if path_points(pp_parent_index,1)==path_points(1,1) && path_points(pp_parent_index,2)==path_points(1,2) && mod(path_points(pp_parent_index,3),2*pi)==mod(path_points(1,3),2*pi)
        continue;
    end

    % Create a new node with the closest point
    nodes_created = nodes_created + 1;
    node(nodes_created).x = path_points(pp_parent_index,1);
    node(nodes_created).y = path_points(pp_parent_index,2);
    node(nodes_created).theta = mod(path_points(pp_parent_index,3),2*pi);
    node(nodes_created).x_par = path_points(1,1);
    node(nodes_created).y_par = path_points(1,2);
    node(nodes_created).th_par = mod(path_points(1,3),2*pi);
    node(nodes_created).pv = path_points(pp_parent_index,4);
    node(nodes_created).pa = path_points(pp_parent_index,5);
    node(nodes_created).pt = path_points(pp_parent_index,6);
    node(nodes_created).index = nodes_created;
    node(nodes_created).p_index = parent_index;
    node(nodes_created).dist = node(parent_index).dist + dist_pp_value;
    node(nodes_created).path = path_points((path_points(:,4) == node(nodes_created).pv & path_points(:,5)==node(nodes_created).pa & path_points(:,6) <=node(nodes_created).pt), 1:2);
    node(nodes_created).path = flipud(node(nodes_created).path);
    
    %car position
    figure(1)
    ss=scatter(node(nodes_created).x,node(nodes_created).y,'filled');
    ss.SizeData = 20;
    %drawnow;

    car_angle = node(nodes_created).theta;
    car_center_x = node(nodes_created).x;
    car_center_y = node(nodes_created).y;

    car_x1 = car_center_x + 0.5*l_car*cos(car_angle) - 0.5*w_car*sin(car_angle);
    car_x2 = car_center_x + 0.5*l_car*cos(car_angle) + 0.5*w_car*sin(car_angle);
    car_x3 = car_center_x - 0.5*l_car*cos(car_angle) - 0.5*w_car*sin(car_angle);
    car_x4 = car_center_x - 0.5*l_car*cos(car_angle) + 0.5*w_car*sin(car_angle);

    car_y1 = car_center_y + 0.5*l_car*sin(car_angle) + 0.5*w_car*cos(car_angle);
    car_y2 = car_center_y + 0.5*l_car*sin(car_angle) - 0.5*w_car*cos(car_angle);
    car_y3 = car_center_y - 0.5*l_car*sin(car_angle) + 0.5*w_car*cos(car_angle);
    car_y4 = car_center_y - 0.5*l_car*sin(car_angle) - 0.5*w_car*cos(car_angle);     

    if IsInParking(car_x1,car_y1,car_x2,car_y2,car_x3,car_y3,car_x4,car_y1,X0,X1,Y0,Y1) == 1
       
        % plot car
        plot([car_x1 car_x2 car_x4 car_x3 car_x1], [car_y1 car_y2 car_y4 car_y3 car_y1], 'r','LineWidth',1);
        
        p_index = node(nodes_created).p_index;
        numofpathfound = numofpathfound + 1;
        allpath(numofpathfound) = node(nodes_created).index;
        line_array = [node(nodes_created).x, node(nodes_created).y];
        
        while p_index >= 2
            c = [node(p_index).path(:,:)];
            disp(c)
            line_array = [line_array;c];  %line([node(nodes_created).x node(p_index).x],[node(nodes_created).y,node(p_index).y])
            p_index = node(p_index).p_index;    
        end
        
        %plot(line_array(:,1),line_array(:,2), 'k','LineWidth',3);
        iter    %print iter when successful
        goal = 1;
        %plot(line_array(:,1),line_array(:,2))
        cbest = 0;
        for g = 1:1:size(line_array,1)-1
            cbest = cbest + pdist2(line_array(g,:),line_array(g+1,:));
        end
        clear line_array;
        fprintf("Route Found\n");
        flag_success = 1;     
        %break;

    %elseif car_center_x>X0 && car_center_x<X1 && car_center_y>Y0 && car_center_y<Y1
    elseif sqrt((car_center_x-x_goal)^2+(car_center_y-y_goal)^2) <= thres_radius
        %center is in parking, but car is not. for visualization
        %plot([car_x1 car_x2 car_x4 car_x3 car_x1], [car_y1 car_y2 car_y4 car_y3 car_y1], 'm')
        drawnow;
        flag_near_goal = 1;
    end
    iter = iter + 1;
end

%% find the best of all paths
totaldist = 0;
currentdist = 0;
bestpath = 0;
for z = 1:1:numofpathfound
    currentdist = node(allpath(z)).dist;
    if currentdist>totaldist
        totaldist=currentdist;
        bestpath = z;
    end
end
p_index = node(allpath(bestpath)).p_index;
line_array = [node(allpath(bestpath)).x, node(allpath(bestpath)).y];
        
        while p_index >= 2
            c = [node(p_index).path(:,:)];
            disp(c)
            line_array = [line_array;c];  %line([node(nodes_created).x node(p_index).x],[node(nodes_created).y,node(p_index).y])
            p_index = node(p_index).p_index;    
        end
plot(line_array(:,1),line_array(:,2), 'k','LineWidth',2);

%% Obstacle detection
function yn = IsObstacle(x,y,obstacle1,obstacle2,obstacle3,obstacle4,obstacle5,x_min,x_max,y_min,y_max)
%check if center point is in obstacle or outside of map
    yn = 0;
    if x>obstacle1(1) && x<(obstacle1(1)+obstacle1(3)) && y>obstacle1(2) && y<(obstacle1(2)+obstacle1(4))
        yn = 1;
        return;
    elseif x>obstacle2(1) && x<(obstacle2(1)+obstacle2(3)) && y>obstacle2(2) && y<(obstacle2(2)+obstacle2(4))
        yn = 1;
        return;
    elseif x>obstacle3(1) && x<(obstacle3(1)+obstacle3(3)) && y>obstacle3(2) && y<(obstacle3(2)+obstacle3(4))
        yn = 1;
        return;
    elseif x>obstacle4(1) && x<(obstacle4(1)+obstacle4(3)) && y>obstacle4(2) && y<(obstacle4(2)+obstacle4(4))
        yn = 1;
        return;
    elseif x>obstacle5(1) && x<(obstacle5(1)+obstacle5(3)) && y>obstacle5(2) && y<(obstacle5(2)+obstacle5(4))
        yn = 1;
        return;
    elseif x<x_min || x>x_max || y<y_min || y>y_max
        yn = 1;
        return;
    end
end

function wn = IsInParking(x1,y1,x2,y2,x3,y3,x4,y4,X0,X1,Y0,Y1)
%check if four corners of car are within parking space
    wn = 0;
    if (x1>X0 && x1<X1 && y1>Y0 && y1<Y1) && (x2>X0 && x2<X1 && y2>Y0 && y2<Y1) && (x3>X0 && x3<X1 && y3>Y0 && y3<Y1) && (x4>X0 && x4<X1 && y4>Y0 && y4<Y1)
        wn = 1;
    end
end