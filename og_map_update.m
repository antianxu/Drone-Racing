%Assume have laser readings

function [ogp, oglo] = og_update(ogp, oglo, y_laser, curr_robot_pose, M, N)
    ogres = 0.1; %or 0.05         % resolution of occ grid
    ogxmin = 0;                    % minimum x value
    ogymin = 0;                    % minimum y value

%     ognx = M/ogres;
%     ogny = N/ogres;

    r_min_laser = 0.3;
    r_max_laser = 10;

    phi_max_laser = 69/2/180*pi;
    phi_min_laser = -69/2/180*pi;

    % npoints = size(y_laser,2);
    angles = linspace(phi_min_laser, phi_max_laser,128);

    curr_x = curr_robot_pose(1);
    curr_y = curr_robot_pose(2);
    curr_theta = curr_robot_pose(3);

    % angle_width = (phi_max_laser - phi_min_laser)/127; %or 128?


    alpha = 1;
    beta = 1;
    
    for j = 1:size(y_laser,2)
        obs = y_laser(j);
        
        if (obs >= r_min_laser) && (obs <= r_max_laser)
            %Determine the field of view ie up to obs, with step size of
            %occ grid resolution
            FOV = 0:ogres:obs;
            %Determine field of view size
            FOV_size = size(FOV,2);
            
            %Loop through all indices in FOV
            for FOV_index = 1:FOV_size
                %Get x and y values - converting from laser frame to robot
                %frame, assuming robot's centre is 0.1m behind the laser
                %emitter (for laser's centre). These are the same equations
                %as in Assignment 1
                x = curr_x - 0.1*cos(curr_theta) + (FOV(FOV_index)*cos(curr_theta + angles(j)));
                y = curr_y - 0.1*sin(curr_theta) + (FOV(FOV_index)*sin(curr_theta + angles(j)));
                
                %Get indices and shift between occupancy grid map (reference
                %frame) and robot frame - note: ceil can also be used
                x_index = floor(x/ogres);
                y_index = floor(y/ogres);
                
                %Determine if occupied or free
                if FOV_index ~= FOV_size(end)
                    %If we're not at the end of the laser, it's a free
                    %space so subtract beta
                    oglo(x_index, y_index) = oglo(x_index, y_index) - beta;
                else
                    %If the last index ie last 0.1 section of laser, it's
                    %occupied so add alpha
                    if x_index < 1 || y_index > M || x_index < 1 || y_index > N
                        x_index = max(min(x_index, M), 1);
                        y_index = max(min(y_index, N), 1);
                    end
                    oglo( x_index, y_index) = oglo(x_index, y_index) + alpha;
                end
            end
        end
    end
    
    for ocy = 1:N
        for ocx = 1:M
            ogp(ocx, ocy) = exp(oglo(ocx, ocy)) / (1 + exp(oglo(ocx, ocy)));
        end
    end
end