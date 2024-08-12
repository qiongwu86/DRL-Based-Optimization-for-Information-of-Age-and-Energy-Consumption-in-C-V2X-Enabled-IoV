function [RBsBeacon,gammaMin_dB,NbitsHz] = findRBsBeaconSINRmin(MCS,BeaconSizeBits)
% This function calculates RBs per beacon and minimum required SINR

% Call function to find ITBS value from MCS调制编码方案
ITBS = findITBS(MCS);

% Call function to find the modulation format (number of bits per symbol)，即进制数M
Nbps = findModulation(MCS);

% Call function to find the number of RBs per beacon
[RBsBeacon,Nbits] = findRBsBeaconNbits(ITBS,BeaconSizeBits);

% Compute the effective code rate码率=传输的数据率（位宽）与信道理论最大传码率的比值
CR = Nbits/((RBsBeacon/2)*9*12*Nbps);   %2Blog2(1+SINR)    Nbit/bps = N Hz

% Compute spectral efficiency (bits/Hz)  =  CR/B  ,即为Rth
%每毫秒每赫兹的频谱带宽内可以传输的比特数
NbitsHz = (12*14*Nbps*CR)/(1e-3*180e3);      

% Compute the minimum required SINR
% (alfa is taken from 3GPP)
alfa = 0.4;
gammaMin_dB = 10*log10(2^(NbitsHz/alfa)-1);      
% NbitsHz = alfa*log2(1+SINR) → SINR=2^(NbitsHz/alfa)-1 ，再转换为dB

%%%%%%%%目前是先计算出1子帧可以传输的最大数据量，所对应的SINR，即功率大小不影响SINR阈值
end