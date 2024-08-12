function [appParams,phyParams] = calculateNB(appParams,phyParams)
% Compute NbeaconsF and NbeaconsT, subchannel sizes or multiples

% Depending on the subchannelization scheme: adjacent or non-adjacent PSCCH and PSSCH (TB + SCI)
if phyParams.ifAdjacent

    % Vector of supported subchannel sizes when adjacent (3GPP TS 36.331)
    phyParams.supportedSizeSubchannelAdjacent = [5 6 10 15 20 25 50 75 100];

    % Remove subchannel sizes exceeding the total number of RBs per Tslot
    phyParams.supportedSizeSubchannelAdjacent = phyParams.supportedSizeSubchannelAdjacent(phyParams.supportedSizeSubchannelAdjacent<=appParams.RBsFrequencyV2V);

    % Check whether the input sizeSubchannel is supported
    if isempty(find(phyParams.supportedSizeSubchannelAdjacent==phyParams.sizeSubchannel, 1)) && phyParams.sizeSubchannel~=-1
        error('Error: "phyParams.sizeSubchannel" must be -1 (best choice) or a supported value');
    end

    % Find RBs occupied by a beacon according to the subchannel size (adjacent case)
    % If a subchannel size is given by input
    if phyParams.sizeSubchannel ~= -1

        % Find number of subchannels in the frequency domain (Tslot)
        phyParams.NsubchannelsFrequency = floor(appParams.RBsFrequencyV2V/phyParams.sizeSubchannel);

        % Find number of subchannels in the frequency domain to carry a beacon + SCI (2 RBs)
        phyParams.NsubchannelsBeacon = -1;
        for i=1:phyParams.NsubchannelsFrequency
            multiple = i * phyParams.sizeSubchannel;
            while ~isValidForFFT(multiple - 2 * phyParams.ifAdjacent)
                multiple = multiple-1;
            end
            if multiple >= appParams.RBsBeacon/2+2
                phyParams.NsubchannelsBeacon = i;    %一个包占用的子信道数
                break;
            end
        end
        %phyParams.NsubchannelsBeacon = ceil((appParams.RBsBeacon/2+2)/phyParams.sizeSubchannel);

        % Find number of RBs per subchannel (or multiple)
        phyParams.RBsBeaconSubchannel = phyParams.NsubchannelsBeacon*phyParams.sizeSubchannel;
    end

    % Find number of beacons in the frequency domain (adjacent case)
    if phyParams.BRoverlapAllowed
        appParams.NbeaconsF = phyParams.NsubchannelsFrequency - phyParams.NsubchannelsBeacon + 1;
    else
        appParams.NbeaconsF = floor(appParams.RBsFrequencyV2V/phyParams.RBsBeaconSubchannel);
    end
    

end

% Find number of beacons in the time domain
appParams.NbeaconsT = floor(appParams.averageTbeacon./phyParams.Tsf);
% stationManagement.A(ceil(3*rand(length(Vehicles),1)))
end
