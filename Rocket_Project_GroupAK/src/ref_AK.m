%
% Trace out an TVC in ref_time seconds
%
function Ref4 = ref_AK(t, roll_max, tilt)

t = t(:);

ref_time = 30;

if nargin < 2
    roll_max = deg2rad(15);
end
if nargin < 3
    tilt = true;
end

% Coordinates (x, y, heading)

coords = [ ...
    % A
    0 0 0;
    0.33 1 0;
    0.66 2 0;
    0.8 1.5 0;
    1 1 0;
    0.66 1 0;
    0.33 1 0;
    0.66 1 0;
    1 1 0;
    1.16 0.5 0;
    1.33 0 0;
    1.66 0 0;
    % K
    2 0 0;
    2 1 0;
    2 2 0;
    2 1.5 0;
    2 1 0;
    2.33 1.5 0;
    2.66 2 0;
    2.33 1.5 0;
    2 1 0;
    2.33 0.5 0;
    2.66 0 0;
    ];

coords(:,1:2) = coords(:,1:2) - coords(1, 1:2);
coords(:,2) = coords(:,2) / 1.5;
coords(:,1:2) = coords(:,1:2) / 1;
coords(:,3) = coords(:,3) * rad2deg(roll_max);

nCoords = size(coords, 1);

% Break the path into legs, compute their end times such that
% the end of the path is reached at ref_time
legs = coords(2:end,1:2) - coords(1:end-1,1:2);
distances = vecnorm(legs, 2, 2);
leg_endtimes = [0; ref_time * cumsum(distances) / sum(distances)]';

% Find target index for each time point
target_id = sum(t(:) > leg_endtimes, 2) + 1;
% Limit target_id to final point
target_id = min(nCoords, target_id);

% Return target coordinates for each time point
XZ = [coords(target_id, 1:2)];
XYZ = [XZ(:,1), zeros(length(t),1), XZ(:,2)];
Roll = deg2rad(coords(target_id,3));

if tilt
    % Rotate
    alpha = deg2rad(-90); % -15
    beta = deg2rad(0); % 19
    gamma = deg2rad(0); %-24
    
    T = Rocket.eul2mat([alpha, beta, gamma]);
    XYZ = (T * XYZ')';
end
Ref4 = [XYZ, Roll]; % 4D X 0 Z roll

end