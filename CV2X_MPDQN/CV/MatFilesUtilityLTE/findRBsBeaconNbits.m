function  [RBsBeacon,Nbits] = findRBsBeaconNbits(ITBS,BeaconSizeBits)
% This function looks at Table 7-1-7-2-1-1 of 3GPP TS 36.213 V14.0.0 and,
% based on ITBS (which is calculated before) and B (beacon size), finds the
% corresponding number of RBs in each slot of an LTE subframe that is
% needed to carry one beacon

X = load('TBL717211.txt');

RBsBeacon = 0;
%计算包所需的RBS和位宽
for i = 2 : 111
    if X(ITBS+1,i) > BeaconSizeBits        %表格X中的第8行第i列的值，X:https://www.docin.com/p-384635971.html
        Nbits = X(ITBS+1,i);
        RBsBeacon = 2*(i-1);
        break;
    end
end

if RBsBeacon == 0
    error('Beacon size is too large for the selected MCS');
end

end