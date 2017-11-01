function [ ] = bytesAvailableFunctionCb()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    global ser
    global measure_data
    
    while 1
        byte = fread(ser,1);
        if byte == hex2dec('44') % measurement results incoming
            measure_data.num_of_meas = fread(ser,1,'precision','uint32');
            measure_data.meas_times = fread(ser, NUM_OF_MEASURES, 'precision', 'uint32');
            measure_data.trans_messages = fread(ser, 1, 'precision','uint32');
            measure_data.rec_messages = fread(ser, 1, 'precision','uint32')
            measure_data.num_of_timeouts = fread(ser, 1, 'precision','uint32');
            measure_data.successful = 1;
            break;
        elseif byte == hex2dec('22') % measure error with configuration
            measure_data.successful = 0;
            break;
        elseif byte == hex2dec('11') % measure error with measure exchange
            measure_data.successful = 
            break;
        elseif byte == hex2dec('66') % measure error when receiving
            measure_data.successful = 0;
            break;
        end
            
    end


end

