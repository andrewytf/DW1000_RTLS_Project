
MEAS_INCOMING = hex2dec('55');

% create serial object if it doesnt exist yet
if ~exist('ser')
    ser = serial('COM8', 'BaudRate', 115200);
    fclose(ser);
    if strcmp(ser.Status, 'closed')
        fopen(ser);
    end
end

% set anchor positions (x,y) here
anchor_pos = [0,   0;
              627, -705;
              0, -705];

% get minimum and maximum x and y          
anchor_pos_min = min(anchor_pos,[], 1);          
anchor_pos_max = max(anchor_pos,[], 1); 

% compute required side length for ploting
plot_side_length = max(anchor_pos_max - anchor_pos_min);

% offset for drawing purposes
plot_lim_offset = 250;

figure(1);
while 1
    % flush input buffer, so we get the most recent data
    flushinput(ser);

    % read buffer until received byte is Measurement incoming alert
    byte = 0;
    while byte ~= MEAS_INCOMING
        if ser.BytesAvailable
            byte = fread(ser,1,'uint8');
        end
    end

    % read all three distances as 3 half words
    distances = fread(ser, 3, 'uint16');

    clf; % clear figure
    % plot three circles using measured distances
    for i = 1:3
        drawCircle(anchor_pos(i,:), distances(i)); 
        hold on
    end

    % plot circle centers
    plot(anchor_pos(:,1), anchor_pos(:,2), '^')
        
    % compute location of Tag and mark it on the map
    loc = getLocationPoint(anchor_pos, distances);
    plot(loc(1), loc(2), 'rx', 'MarkerSize', 20);
    axis equal
    xlim([anchor_pos_min(1) - plot_lim_offset, anchor_pos_min(1) + plot_side_length + plot_lim_offset]);
    ylim([anchor_pos_min(2) - plot_lim_offset, anchor_pos_min(2) + plot_side_length + plot_lim_offset]);
    grid minor

    % take time to draw everything
    drawnow;

end

