%% save measurement data to excel file

% conf_vec : vector of configuration (5 values)
%     value 1 : channel number
%     value 2 : PRF
%     value 3 : Data Rate
%     value 4 : PRM Length
%     value 5 : Payload size
C = num2cell([(1:size(ConfigCombinationsMat,2))', ConfigCombinationsMat', successful,  meas_time_avg, meas_time_std, baudrate, receiveTransmitRatio, RSSI_avg, RSSI_std, meas_trnarnd_time_avg, meas_trnarnd_time_std] );

% change PRF value
PRF_row = 3;
for i=1:size(ConfigCombinationsMat,2)
    if C{i,PRF_row} == 1
        C{i,PRF_row} = 16;
    elseif C{i,PRF_row} == 2
        C{i,PRF_row} = 64;
    end
end

% Change data rate value
data_rate_row = 4;
for i=1:size(ConfigCombinationsMat,2)
    cell_val = C{i,data_rate_row};
    switch (cell_val)
        case 0
            C{i,data_rate_row} = 110;
        case 1
            C{i,data_rate_row} = 850;
        case 2
            C{i,data_rate_row} = 6800;
    end

end


% Change Preamble size value
PRM_row = 5;
for i=1:size(ConfigCombinationsMat,2)
    cell_val = C{i,PRM_row};
    switch (cell_val)
        case hex2dec('04')
            C{i,PRM_row} = 64;
        case hex2dec('14')
            C{i,PRM_row} = 128;
        case hex2dec('24')
            C{i,PRM_row} = 256;
        case hex2dec('34')
            C{i,PRM_row} = 512;
        case hex2dec('08')
            C{i,PRM_row} = 1024;
        case hex2dec('18')
            C{i,PRM_row} = 1536;    
        case hex2dec('28')
            C{i,PRM_row} = 2048;
        case hex2dec('0C')
            C{i,PRM_row} = 4096;            
    end

end

% change payload size value
payload_row = 6;
for i=1:size(ConfigCombinationsMat,2)
    C{i,payload_row} = getPayloadSizeFromCode( C{i,payload_row} );
end

% add upper row with field titles
C(2:end+1,:) = C(1:end,:);
%C(1,:) = {};
C{1,1} = 'Measurement index';
C{1,2} = 'Channel number';
C{1,3} = 'PRF [MHz]';
C{1,4} = 'Data Rate [Mb/s]';
C{1,5} = 'Preamble length';
C{1,6} = 'Payload size [B]';
C{1,7} = 'Success';
C{1,8} = 'Average transmission time [us]';
C{1,9} = 'STD of transmission time [us]';
C{1,10} = 'Baudrate [kB/s]';
C{1,11} = 'Receive / Transmit [%]';
C{1,12} = 'Average RSSI [dBm]';
C{1,13} = 'STD RSSI [dBm]';
C{1,14} = 'Average turnaround time [us]';
C{1,15} = 'Standard deviation of turnaround time [us]';

%T = table(Channel number, PRF, DataRate, PRM, Payload size, );

xlswrite('DW_meritve_150m.xlsx', C);

%%
C_mat = C(2:end,2:end);

payload_vec = cell2mat(C_mat(:,5));
PRM_vec = cell2mat(C_mat(:,4));
data_rate = cell2mat(C_mat(:,3));
successful = cell2mat(C_mat(:,6));
tr_time_real = cell2mat(C_mat(:,7));

PAC_size_vec = zeros(size(C_mat,1), 1);
PAC_size_vec(PRM_vec == 64) = 8;
PAC_size_vec(PRM_vec == 128) = 8;
PAC_size_vec(PRM_vec == 256) = 16;
PAC_size_vec(PRM_vec == 512) = 16;
PAC_size_vec(PRM_vec == 1024) = 32;
PAC_size_vec(PRM_vec == 1536) = 64;
PAC_size_vec(PRM_vec == 2048) = 64;
PAC_size_vec(PRM_vec == 4096) = 64;

payload_vec = payload_vec(successful == 1);
PRM_vec = PRM_vec(successful == 1);
data_rate = data_rate(successful == 1);
tr_time_real = tr_time_real(successful == 1);
PAC_size_vec = PAC_size_vec(successful == 1);

koef = 150;

tr_time_ocena = (payload_vec * 8) ./ (data_rate * 1e3) * 1e6;

diff = tr_time_real - tr_time_ocena;

min(diff)
max(diff)


