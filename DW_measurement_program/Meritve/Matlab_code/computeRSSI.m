function [ RSSI_avg, RSSI_std ] = computeRSSI( CIR_avg, CIR_std, PRMcnt_avg, PRMcnt_std , PRF_val_code )
%computeRSSI Computes average RSSI value from measurement and standard
%            deviation
%   Input : CIR_avg - average of measured CIR
%           CIR_std - standard deviation of measured CIR
%           PRMcnt_avg - average of measured Preamble counter
%           PRMcnt_std - standard deviation of measured Preamble counter
%           PRF_val - PRF value used in measurement, used for determining
%                     constant A in equation
% NOTE : equation is derived from DW1000 user manual, title 4.7.2

A_const_vec = [113.77, 121.74];
A_const = A_const_vec(PRF_val_code);

RSSI_avg = 10*log10(CIR_avg * 2^17 / PRMcnt_avg^2) - A_const;

% standard deviation koeficients are computed with derivations of RSSI
% equation with respect to CIR and PRM counter variables
CIR_koef = 10 / (CIR_avg * log(10));
PRMcnt_koef = -20 / (PRMcnt_avg * log(10));

RSSI_std = sqrt( CIR_koef^2 * CIR_std^2 + PRMcnt_koef^2 * PRMcnt_std^2 );

end

