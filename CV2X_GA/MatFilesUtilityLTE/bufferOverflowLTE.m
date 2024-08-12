function [stationManagement,outputValues] = bufferOverflowLTE(idOverflow,positionManagement,stationManagement,phyParams,outputValues,outParams)

pckType = stationManagement.pckType(idOverflow);
iChannel = stationManagement.vehicleChannel(idOverflow);
for iPhyRaw=1:length(phyParams.Raw)
    % Count as a blocked transmission (previous packet is discarded)
    outputValues.NblockedLTE(iChannel,pckType,iPhyRaw) = outputValues.NblockedLTE(iChannel,pckType,iPhyRaw) + nnz(positionManagement.distanceReal(idOverflow,stationManagement.activeIDsLTE) < phyParams.Raw(iPhyRaw)) - 1; % -1 to remove self
    outputValues.NblockedTOT(iChannel,pckType,iPhyRaw) = outputValues.NblockedTOT(iChannel,pckType,iPhyRaw) + nnz(positionManagement.distanceReal(idOverflow,stationManagement.activeIDsLTE) < phyParams.Raw(iPhyRaw)) - 1;
end
if outParams.printPacketReceptionRatio
    for iRaw = 1:1:floor(phyParams.RawMaxLTE/outParams.prrResolution)
        distance = iRaw * outParams.prrResolution;
        outputValues.distanceDetailsCounterLTE(iChannel,pckType,iRaw,4) = outputValues.distanceDetailsCounterLTE(iChannel,pckType,iRaw,4) + nnz(positionManagement.distanceReal(idOverflow,stationManagement.activeIDsLTE)<distance) - 1;
    end
end

%% Print in command window
% if ~isfield(outParams,'nLTEoverflow')
%     outParams.nLTEoverflow=0;
% end
% outParams.nLTEoverflow=outParams.nLTEoverflow+1;
% fprintf('\nMore than one packet in the queue of an LTE node (counter=%d). Not expected.\n',outParams.nLTEoverflow);

% stationManagement.pckBuffer(idOverflow) = 1;
