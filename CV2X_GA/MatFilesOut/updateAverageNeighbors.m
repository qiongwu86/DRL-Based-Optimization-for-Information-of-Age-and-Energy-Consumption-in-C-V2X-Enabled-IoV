function [outputValues,NneighboursTot,NneighboursLTE,Nneighbours11p] = updateAverageNeighbors(simParams,stationManagement,outputValues,phyParams)
%观察周围有多少车辆？
NneighboursLTE = zeros(1,length(phyParams.Raw));
Nneighbours11p = zeros(1,length(phyParams.Raw));
NneighboursTot = zeros(1,length(phyParams.Raw));
StDevNeighboursLTE = zeros(1,length(phyParams.Raw));
StDevNeighbours11p = zeros(1,length(phyParams.Raw));

for iPhyRaw = 1:length(phyParams.Raw)
    % LTE
    %if simParams.technology ~= 2 % if not only 11p
    if sum(stationManagement.vehicleState(stationManagement.activeIDs)==100)>0    
        NneighborsRawLTE = zeros(length(stationManagement.activeIDsLTE),1);
        for i = 1:length(stationManagement.activeIDsLTE)
            if iPhyRaw==1
                NneighborsRawLTE(i) = nnz(stationManagement.awarenessIDLTE(i,:,iPhyRaw));
            else
                NneighborsRawLTE(i) = nnz(setdiff(stationManagement.awarenessIDLTE(i,:,iPhyRaw),stationManagement.awarenessIDLTE(i,:,iPhyRaw-1)));
            end
        end
        NneighboursLTE(iPhyRaw) = sum(NneighborsRawLTE);
        StDevNeighboursLTE(iPhyRaw) = std(NneighborsRawLTE);
    else
        NneighboursLTE(iPhyRaw) = 0;
        StDevNeighboursLTE(iPhyRaw) = 0;
    end

    % 11p
    if sum(stationManagement.vehicleState(stationManagement.activeIDs)~=100)>0
    %if simParams.technology ~= 1 % if not only LTE
        NneighborsRaw11p = zeros(length(stationManagement.activeIDs11p),1);
        for i = 1:length(stationManagement.activeIDs11p)
            if iPhyRaw==1
                NneighborsRaw11p(i) = nnz(stationManagement.awarenessID11p(i,:,iPhyRaw));
            else
                NneighborsRaw11p(i) = nnz(setdiff(stationManagement.awarenessID11p(i,:,iPhyRaw),stationManagement.awarenessID11p(i,:,iPhyRaw-1)));
            end
        end
        Nneighbours11p(iPhyRaw) = sum(NneighborsRaw11p);
        StDevNeighbours11p(iPhyRaw) = std(NneighborsRaw11p);
    else
        Nneighbours11p(iPhyRaw) = 0;
        StDevNeighbours11p(iPhyRaw) = 0;
    end

    NneighboursTot(iPhyRaw) = NneighboursLTE(iPhyRaw) + Nneighbours11p(iPhyRaw);
    outputValues.NneighborsLTE(iPhyRaw) = outputValues.NneighborsLTE(iPhyRaw) + NneighboursLTE(iPhyRaw);
    outputValues.Nneighbors11p(iPhyRaw) = outputValues.Nneighbors11p(iPhyRaw) + Nneighbours11p(iPhyRaw);
    outputValues.NneighborsTOT(iPhyRaw) = outputValues.NneighborsTOT(iPhyRaw) + NneighboursTot(iPhyRaw);
    outputValues.StDevNeighboursLTE(iPhyRaw) = outputValues.StDevNeighboursLTE(iPhyRaw) + StDevNeighboursLTE(iPhyRaw);
    outputValues.StDevNeighbours11p(iPhyRaw) = outputValues.StDevNeighbours11p(iPhyRaw) + StDevNeighbours11p(iPhyRaw);
    outputValues.StDevNeighboursTOT(iPhyRaw) = outputValues.StDevNeighboursTOT(iPhyRaw) + StDevNeighboursLTE(iPhyRaw) + StDevNeighbours11p(iPhyRaw);
end