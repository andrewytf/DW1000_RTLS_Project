function [ measRawData ] = getRawData(ser_obj)
%GETRAWDATA Gets raw measurement data from Anchor DW.
%   Input  : Serial object
%   Output : measRawData : vector 9x1
%            1 - number of measurement exchanges
%            2 - average time of measurement excange
%            3 - variance of times of measurement excanges
%            4 - number of transmitted messages
%            5 - number of received messages
%            6 - average CIR of measurement excange
%            7 - variance of CIR
%            8 - average Preamble counter
%            9 - variance of Preamble counter
%           10 - average of turnaround times
%           11 - variance of turnaround times   

measRawData = fread(ser_obj, 11, 'uint32');

end

