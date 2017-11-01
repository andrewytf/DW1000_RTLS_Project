if exist('ser')
    fclose(ser)
    clear ser
end

clear; close; clc;

DWT_PLEN_4096 =  hex2dec('0C');   
DWT_PLEN_2048 =  hex2dec('28');
DWT_PLEN_1536 =  hex2dec('18');
DWT_PLEN_1024 =  hex2dec('08');
DWT_PLEN_512  =  hex2dec('34');
DWT_PLEN_256  =  hex2dec('24');
DWT_PLEN_128  =  hex2dec('14');
DWT_PLEN_64   =  hex2dec('04');

PAYLOAD_10B_CODE   = 1;
PAYLOAD_20B_CODE   = 2;
PAYLOAD_50B_CODE   = 3;
PAYLOAD_100B_CODE  = 4;
PAYLOAD_500B_CODE  = 5;
PAYLOAD_1000B_CODE = 6;

PRF_CODE_16 = 1;
PRF_CODE_64 = 2;

channel_number_vec = [1,2,3,4,5,7];
prf_vec = [PRF_CODE_16, PRF_CODE_64];  % 16M or 64M PRF
data_rate_vec = [0,1,2];               %110K, 850K and 6M8 
% prm_length_vec = [DWT_PLEN_64, DWT_PLEN_128, DWT_PLEN_256, DWT_PLEN_512,...
%               DWT_PLEN_1024, DWT_PLEN_1536, DWT_PLEN_2048, DWT_PLEN_4096  ];
prm_length_vec = [DWT_PLEN_64, DWT_PLEN_256, DWT_PLEN_1024, DWT_PLEN_4096  ];

payload = [PAYLOAD_10B_CODE,PAYLOAD_20B_CODE,PAYLOAD_50B_CODE,...
           PAYLOAD_100B_CODE,PAYLOAD_500B_CODE,PAYLOAD_1000B_CODE];
           
MEASUREMENT_COMPLETE     = hex2dec('55');           
MEASUREMENT_RES_INCOMING = hex2dec('44');
MEASUREMENT_RES_END      = hex2dec('33');
MEAS_ERROR_CONFIG        = hex2dec('22');
MEAS_ERROR_RESP          = hex2dec('11');
MEAS_ERROR_REC           = hex2dec('66');
MCU_RESET                = hex2dec('77');
MEAS_UNKNOWN_REC_ERROR   = hex2dec('68'); 

if ~exist('ser')
    ser = serial('COM6', 'BaudRate', 115200);
    fclose(ser);
    if strcmp(ser.Status, 'closed')
        fopen(ser);
    end
end

% create matrix with all configuration combinations. Each column represents
% different combination as 5×1 vector
ConfigCombinationsMat = combvec(channel_number_vec, prf_vec, data_rate_vec, prm_length_vec, payload);
%ConfigCombinationsMat = combvec(payload, prf_vec, data_rate_vec, prm_length_vec, payload);

% conf_vec : vector of configuration (5 values)
%     value 1 : channel number
%     value 2 : PRF
%     value 3 : Data Rate
%     value 4 : PRM Length
%     value 5 : Payload size

%ser.BytesAvailableFcn = @bytesAvailableFunctionCb;

cur_config = [2,2,0,DWT_PLEN_1024,PAYLOAD_20B_CODE];
cur_config = [1, cur_config];

num_of_combs = size(ConfigCombinationsMat, 2);

% uncomment this before new measurement
successful = zeros(num_of_combs, 1);

meas_time_avg = zeros(num_of_combs, 1);
meas_time_std = zeros(num_of_combs, 1);
baudrate = zeros(num_of_combs, 1);
receiveTransmitRatio = zeros(num_of_combs, 1);
RSSI_avg = zeros(num_of_combs, 1);
RSSI_std = zeros(num_of_combs, 1);
meas_trnarnd_time_avg = zeros(num_of_combs, 1);
meas_trnarnd_time_std = zeros(num_of_combs, 1);
error_code = zeros(num_of_combs, 1);

%%
start_ind = 1;
flushinput(ser);
meas_list = [31:35, 115:120, 223:228, 259:264, 266:269, 402:407, 511:514, 547:552, 559:562, 655:660, 691:696];
%for i = meas_list
for i =start_ind:num_of_combs
%for i =260

    cur_config = ConfigCombinationsMat(:,i);
    
%     if error_code(i) ~= MEAS_ERROR_CONFIG
%         continue
%     end
        
    for rep=1:2

        if successful(i)
            break;
        end

        % write current configuration to DW anchor
        fwrite(ser, [1; cur_config]);

        while 1 
            if ser.BytesAvailable
                byte = fread(ser,1,'uint8');
                if byte == MEASUREMENT_RES_INCOMING

                    % get raw data
                    measRawData = getRawData(ser);

                    % compute measurement data
                    successful(i) = 1;

                    % get number of measurement exchanges
                    num_of_messages = measRawData(1);

                    % divide by 2 because we want time of one sent end receive message, not entire exchange
                    meas_time_avg(i) = measRawData(2)./2; % in us
                    meas_time_std(i) = sqrt( measRawData(3) ./ 4 ./ measRawData(5) ); % in us

                    % speed of transmission
                    baudrate(i) = getPayloadSizeFromCode( cur_config(5) ) / meas_time_avg(i) * 1e3; % in B/s 
                    
                    % receive / transmit ratio
                    receiveTransmitRatio(i) = measRawData(5) / measRawData(4) * 100; % in %

                    % RSSI value in dBm
                    CIR_avg = measRawData(6);
                    CIR_std = sqrt(measRawData(7) / measRawData(5));
                    PRMcnt_avg = measRawData(8);
                    PRMcnt_std = sqrt(measRawData(9) / measRawData(5));
                    [RSSI_avg(i), RSSI_std(i)] = computeRSSI(CIR_avg, CIR_std, PRMcnt_avg, PRMcnt_std, cur_config(2));
                    
                    % Turnaround times
                    meas_trnarnd_time_avg(i) = measRawData(10); % in us
                    meas_trnarnd_time_std(i) = sqrt( measRawData(11) ./ measRawData(5) ); % in us

                    break;
                elseif byte == MEAS_ERROR_CONFIG
                     successful(i) = 0;
                     disp('Measure error with configuration, too many timeouts.')
                     error_code(i) = MEAS_ERROR_CONFIG;
                     break;
                elseif byte == MEAS_ERROR_RESP
                    successful(i) = 0;
                    disp('Error: Measure error with receiving response, too many timeouts.')
                    error_code(i) = MEAS_ERROR_RESP;
                    break;
                elseif byte == MEAS_ERROR_REC
                    successful(i) = 0;
                    disp('Error: Measure error with receiving.')
                    error_code(i) = MEAS_ERROR_REC;
                    break;
                elseif byte == MCU_RESET
                    successful(i) = 0;
                    disp('MCU reset occured.')
                    error_code(i) = MCU_RESET;
                    break;
                elseif byte == MEAS_UNKNOWN_REC_ERROR
                    successful(i) = 0;
                    disp('Error : Unknown message received.')
                    error_code(i) = MEAS_UNKNOWN_REC_ERROR;
                    break;
                end
            end
        end
    end
    disp(i);
end

save('meritve_temp.mat')

%%
successful1 = zeros(num_of_combs, 1);
meas_time_avg1 = zeros(num_of_combs, 1);
meas_time_std1 = zeros(num_of_combs, 1);
baudrate1 = zeros(num_of_combs, 1);
receiveTransmitRatio1 = zeros(num_of_combs, 1);
RSSI_avg1 = zeros(num_of_combs, 1);
RSSI_std1 = zeros(num_of_combs, 1);
meas_trnarnd_time_avg1 = zeros(num_of_combs, 1);
meas_trnarnd_time_std1 = zeros(num_of_combs, 1);
error_code1 = zeros(num_of_combs, 1);

load('meritve_100m_do_636.mat')

successful1(1:636) = successful(1:636);
meas_time_avg1(1:636) = meas_time_avg(1:636);
meas_time_std1(1:636) = meas_time_std(1:636);
baudrate1(1:636) = baudrate(1:636);
receiveTransmitRatio1(1:636) = receiveTransmitRatio(1:636);
RSSI_avg1(1:636) = RSSI_avg(1:636);
RSSI_std1(1:636) = RSSI_std(1:636);
meas_trnarnd_time_avg1(1:636) = meas_trnarnd_time_avg(1:636);
meas_trnarnd_time_std1(1:636) = meas_trnarnd_time_std(1:636);

load('meritve_100m_nadaljevanje.mat')

successful1(637:end) = successful(637:end);
meas_time_avg1(637:end) = meas_time_avg(637:end);
meas_time_std1(637:end) = meas_time_std(637:end);
baudrate1(637:end) = baudrate(637:end);
receiveTransmitRatio1(637:end) = receiveTransmitRatio(637:end);
RSSI_avg1(637:end) = RSSI_avg(637:end);
RSSI_std1(637:end) = RSSI_std(637:end);
meas_trnarnd_time_avg1(637:end) = meas_trnarnd_time_avg(637:end);
meas_trnarnd_time_std1(637:end) = meas_trnarnd_time_std(637:end);

successful = successful1;
meas_time_avg = meas_time_avg1;
meas_time_std = meas_time_std1;
baudrate = baudrate1;
receiveTransmitRatio = receiveTransmitRatio1;
RSSI_avg = RSSI_avg1;
RSSI_std = RSSI_std1;
meas_trnarnd_time_avg = meas_trnarnd_time_avg1;
meas_trnarnd_time_std = meas_trnarnd_time_std1;
error_code = error_code1;


