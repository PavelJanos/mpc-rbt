map_name = 'maps/indoor_3.txt';

% Default start position (x, y, theta)
start_position = [1, 1, 1.0*pi/2];

% Map-specific safe starts
if strcmp(map_name, 'maps/indoor_1.txt')
    start_position = [2, 8.5, -pi/2];
end







