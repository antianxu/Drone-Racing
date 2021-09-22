function path = A_star(pt_start, pt_end, map, goal_range)
    % pt_i: (1, 2)
    % pt_f: (1, 2)
    % map: (M, N)
    
    step = 5;   % move step
    M = size(map, 1);
    N = size(map, 2);
    
    openSet = zeros(1, 2);
    openSet(1, :) = pt_start;
    costs_to_come = [0];
    closeSet = [];
    came_from = containers.Map({get_pt_str(pt_start)},{[-1, -1]});
    ref_score = 10000;
    planned_pt_end = pt_end;
    while size(openSet, 1) > 0
        % pop the best point out of the openSet
        pt_i = get_best_pt(openSet, costs_to_come, pt_end);
        curr_pt = openSet(pt_i, :);
        curr_g = costs_to_come(pt_i);
        curr_score = curr_g + h(curr_pt, pt_end);
        openSet(pt_i, :) = [];
        costs_to_come(pt_i) = [];
        
        if curr_score > ref_score % if the best score is worse than the record, early stop and return record
            break;
        end
        
        if get_pts_distance(curr_pt, pt_end) < goal_range % check if the curr pt is close enough to the goal
            ref_score = curr_score;
            planned_pt_end = curr_pt;
            continue;
        end
        
        % loop through all possible next steps
        for move_x = [-step, 0, step]
            for move_y = [-step, 0, step]
                if move_x == 0 && move_y == 0   % this is the current point
                    continue;
                end
                next_pt = [curr_pt(1) + move_x, curr_pt(2) + move_y];
                if check_pt(next_pt, M, N, map, closeSet)
                    next_pt_cost_to_come = curr_g + get_pts_distance(curr_pt, next_pt);
                    next_pt_openSet_i = get_pt_in_set(next_pt, openSet);
                    if next_pt_openSet_i ~= 0   % if the next pt has been visited
                        if costs_to_come(next_pt_openSet_i) > next_pt_cost_to_come
                            costs_to_come(next_pt_openSet_i) = next_pt_cost_to_come;
                            came_from(get_pt_str(next_pt)) = curr_pt;
                        end
                    else    % if the next pt hasn't been visited
                        openSet(end+1, :) = next_pt;
                        costs_to_come(end+1) = next_pt_cost_to_come;
                        came_from(get_pt_str(next_pt)) = curr_pt;
                    end
                end
            end
        end
        
        closeSet(end+1, :) = curr_pt;
        
    end
    
    if ref_score == 10000
        path = 0;   % can't find a path!
    else
        path = retrace(came_from, planned_pt_end);
    end
end

function heuristic = h(curr_pt, goal_pt)
    heuristic = get_pts_distance(curr_pt, goal_pt);
end

function pt_i = get_best_pt(openSet, costs_to_come, goal_pt)
    min_score = 10000;
    min_score_pt_i = 0;
    for i = 1:size(openSet, 1)
        pt = openSet(i, :);
        g = costs_to_come(i);
        pt_score = g + h(pt, goal_pt);
        if pt_score < min_score
            min_score = pt_score;
            min_score_pt_i = i;
        end
    end
    pt_i = min_score_pt_i;
end

function dist = get_pts_distance(pt1, pt2)
    dist = sqrt((pt1(1) - pt2(1))^2 + (pt1(2) - pt2(2))^2);
end

% check if a point is in a set. If so return index, if not return 0
function pt_i = get_pt_in_set(pt, set)
    pt_i = 0;
    for i = 1:size(set,1)
        if isequal(pt, set(i, :))
            pt_i = i;
            break;
        end
    end
end

function path = retrace(came_from, pt)
    path = zeros(1, 2);
    path(1, :) = pt;
    curr_pt = pt;
    while 1
        prev_pt = came_from(get_pt_str(curr_pt));
        if isequal(prev_pt, [-1, -1]) % 0 indicate the pt is the starting point
            break;
        end
        path(end+1, :) = prev_pt;
        curr_pt = prev_pt;
    end
    path = flipud(path);    % reverse order to start from the starting pt
end

function pt_str = get_pt_str(pt)
    pt_str = [num2str(pt(1)), ',' num2str(pt(2))];
end

function is_valid = check_pt(pt, M, N, map, closeSet)    
    % Check if the point is in the boundary and if it's free from obstacle
    is_valid = true;
    if pt(1) < 1 || pt(1) > M || pt(2) < 1 || pt(2) > N
        is_valid = false;
    elseif get_pt_in_set(pt, closeSet) ~= 0 % if the cell is dead
        is_valid = false;
    else
        bubble_size = 5;   % let the robot path stay away from obstacles by some pixels
        x_min = round(max(1, pt(1)-bubble_size));
        x_max = round(min(M, pt(1)+bubble_size));
        y_min = round(max(1, pt(2)-bubble_size));
        y_max = round(min(N, pt(2)+bubble_size));
        if sum(map(x_min:x_max, y_min:y_max) > 0.1, 'all') > 0   % 1 means an obstacle cell
            is_valid = false;
        end
    end
end