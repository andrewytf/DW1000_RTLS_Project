function [ PayloadSize ] = getPayloadSizeFromCode( PayloadCode )
%getPayloadSizeFromCode gets actual payload from its used code
% PAYLOAD_10B_CODE   = 1;
% PAYLOAD_20B_CODE   = 2;
% PAYLOAD_50B_CODE   = 3;
% PAYLOAD_100B_CODE  = 4;
% PAYLOAD_500B_CODE  = 5;
% PAYLOAD_1000B_CODE = 6;

payload_vec = [10, 20, 50, 100, 500, 1000];

PayloadSize = payload_vec(PayloadCode);

end

