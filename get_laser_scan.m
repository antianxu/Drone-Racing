function [laser_scan] = get_laser_scan(map, pos_rob, ang_row, r_max, inc)
    [M,N] = size(map);
    laser_scan = zeros(size(ang_row));

    for i=1:length(ang_row)
        c_ang = cos(pos_rob(3) + ang_row(i));
        s_ang = sin(pos_rob(3) + ang_row(i));
        dist = inc;
        
        while dist < r_max
            dd = round([(pos_rob(1) + dist*c_ang)/inc, (pos_rob(2) + dist*s_ang)/inc]);

            if dd(1)<1||dd(2)<1||dd(1)>M||dd(2)>N
                laser_scan(i) = dist;
                break;
            elseif map(dd(1),dd(2))
                laser_scan(i) = dist;
                break;
            end
            laser_scan(i) = dist;
            dist = dist + inc;
        end
    end
end