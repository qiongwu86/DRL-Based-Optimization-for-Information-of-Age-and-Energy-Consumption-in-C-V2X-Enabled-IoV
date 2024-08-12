function [phyParams,simValues,outputValues,sinrManagement,stationManagement,timeManagement] = ...
            mainLTEsubframeEnds(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement)
% an LTE subframe ends
     
% local variables for simpler reading
awarenessID_LTE = stationManagement.awarenessIDLTE;
neighborsID_LTE = stationManagement.neighborsIDLTE;

% Compute elapsed time [the unit of measure is the subframe time i.e. phyParams.Tsf]
%elapsedTime_subframes = floor((timeManagement.timeNow-1e-9)/phyParams.Tsf)+1;

if ~isempty(stationManagement.transmittingIDsLTE)     
    
    % Find ID and index of vehicles that are currently transmitting in LTE
    activeIDsTXLTE = stationManagement.transmittingIDsLTE;
    indexInActiveIDsOnlyLTE = stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE;

    if simParams.technology == 4 % COEXISTENCE IN THE SAME BAND
        [timeManagement,stationManagement,sinrManagement,outputValues] = coexistenceAtLTEsubframeEnd(timeManagement,stationManagement,sinrManagement,simParams,simValues,phyParams,outParams,outputValues);
    end

    %% Start computing KPIs only after the first BR assignment (after the first cycle)

    % Compute SINR of received beacons
    sinrManagement = updateSINRLTE(timeManagement.timeNow,stationManagement,sinrManagement,phyParams.Pnoise_MHz*phyParams.BwMHz_lteBR,simParams,appParams);

    % Code for possible DEBUG
    % figure(100)
    % plot(elapsedTime_subframes*ones(1,length(neighborsSINRaverageLTE(:,:))),10*log10(neighborsSINRaverageLTE(:,:)),'*');
    % hold on

    % DEBUG TX
%     printDebugTx(timeManagement.timeNow,false,-1,stationManagement,positionManagement,sinrManagement,outParams,phyParams);

    % Code for possible DEBUG
    % figure(200)
    % plot(elapsedTime_subframes*ones(1,length(errorMatrixRawMax(:,4))),errorMatrixRawMax(:,4),'*');
    % hold on

    % Code for possible DEBUG
    % figure(300)
    % plot(elapsedTime_subframes*ones(1,length(errorMatrix(1,:))),10*log10(errorMatrix(1,:)),'*');
    % hold on

    % Check the correctness of SCI messages
    if simParams.BRAlgorithm==18
        % correctSCImatrix is nTXLTE x nNeighblors
        stationManagement.correctSCImatrixLTE = (sinrManagement.neighborsSINRsciAverageLTE > phyParams.minSCIsinr);
        
        %if simParams.technology==4 && simParams.coexMethod>1 && simParams.coex_slotManagement==2
        if simParams.technology==4 %&& simParams.coexMethod>1 && simParams.coex_slotManagement==2
            % In mitigation methods with dynamic slot duration, 
            % we need the calculation of the CBR_LTE
            % To this aim, the correct/wrong reception of SCI messages in this subframe is
            % stored in "stationManagement.correctSCImatrixLTE(,)"
            % A record with the SCI messages in the last 100 subframes is
            % required: "stationManagement.coex_correctSCIhistory(subframe,idVehicle)" is used 
            % Step 1: circular shift of the matrix and zeros to remove oldest record
            sinrManagement.coex_correctSCIhistory(:,:) = circshift(sinrManagement.coex_correctSCIhistory(:,:),appParams.NbeaconsF);
            sinrManagement.coex_correctSCIhistory(1:appParams.NbeaconsF,:) = 0;
            % Step 2: new record
            for i = 1:length(stationManagement.transmittingIDsLTE)
                indexVtxLte = stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(i);
                for indexNeighborsOfVtx = 1:length(stationManagement.neighborsIDLTE(indexVtxLte,:))
                   idVrx = stationManagement.neighborsIDLTE(indexVtxLte,indexNeighborsOfVtx);
                   if idVrx<=0
                       break;
                   end
                   if stationManagement.correctSCImatrixLTE(i,indexNeighborsOfVtx) == 1 % correct reception of the SCI
                       sinrManagement.coex_correctSCIhistory(mod(stationManagement.BRid(stationManagement.transmittingIDsLTE(i))-1,appParams.NbeaconsF)+1,idVrx) = 1;
                   end
                end
            end
        end
    end
    
    %% KPIs Computation (Snapshot)
    [stationManagement,outputValues,simValues] = updateKPILTE(activeIDsTXLTE,indexInActiveIDsOnlyLTE,awarenessID_LTE,neighborsID_LTE,timeManagement,stationManagement,positionManagement,sinrManagement,outputValues,outParams,simParams,appParams,phyParams,simValues);

else
% No LTE transmitting
    if simParams.technology == 4
        % IDvehicleTXLTE is empty
        % but I need to update the average interfering power from 11p nodes
        sinrManagement = updateSINRLTE(timeManagement.timeNow,stationManagement,sinrManagement,phyParams.Pnoise_MHz*phyParams.BwMHz_lteBR,simParams,appParams);
        % and in some cases (Method B) I need to reset Interf power from
        % LTE nodes
        %if simParams.coexMethod==2
        if sum(sinrManagement.coex_InterfFromLTEto11p)>0
            [timeManagement,stationManagement,sinrManagement,outputValues] = coexistenceAtLTEsubframeEnd(timeManagement,stationManagement,sinrManagement,simParams,simValues,phyParams,outParams,outputValues);
        end
    end
end 

Nreassign = 0;
if simParams.BRAlgorithm==18
    % BRs sensing procedure
    [timeManagement,stationManagement,sinrManagement] = ...
        LTEsensingProcedure(timeManagement,stationManagement,sinrManagement,simParams,phyParams,appParams,outParams);    
    
    % BRs reassignment (3GPP MODE 4)   
    [timeManagement,stationManagement,sinrManagement,Nreassign,~] = ...
        BRreassignment3GPPmode4(timeManagement,stationManagement,positionManagement,sinrManagement,simParams,phyParams,appParams,outParams); 
        % Code for possible DEBUG
        % figure(400)
        % plot(timeManagement.timeNow*ones(1,length(stationManagement.BRid)),stationManagement.BRid,'*');
        % hold on
        % figure(500)
        % plot(stationManagement.activeIDsLTE,stationManagement.BRid,'*');
        % hold on

elseif mod(timeManagement.elapsedTime_subframes,appParams.NbeaconsT)==0
    % All other algorithms except standard Mode 4
    % TODO not checked in version 5.X
    
    %% Radio Resources Reassignment
    if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==7 || simParams.BRAlgorithm==10
        
        if timeManagement.elapsedTime_subframes > 0
            % Current scheduled reassign period
            reassignPeriod = mod(round(timeManagement.elapsedTime_subframes/(appParams.NbeaconsT))-1,stationManagement.NScheduledReassignLTE)+1;

            % Find IDs of vehicles whose resource will be reassigned
            scheduledID = stationManagement.activeIDsLTE(stationManagement.scheduledReassignLTE(stationManagement.activeIDsLTE)==reassignPeriod);
        else
            % For the first allocation, all vehicles in the scenario
            % need to be scheduled
            scheduledID = stationManagement.activeIDsLTE;
        end
    end

    if simParams.BRAlgorithm==2

        % BRs reassignment (CONTROLLED with REUSE DISTANCE and scheduled vehicles)
        % Call function for BRs reassignment
        % Returns updated stationManagement.BRid vector and number of successful reassignments
        [stationManagement.BRid,Nreassign] = BRreassignmentControlled(stationManagement.activeIDsLTE,scheduledID,positionManagement.distanceEstimated,stationManagement.BRid,appParams.Nbeacons,phyParams.Rreuse);

    elseif simParams.BRAlgorithm==7

        % BRs reassignment (CONTROLLED with MAXIMUM REUSE DISTANCE)
        [stationManagement.BRid,Nreassign] = BRreassignmentControlledMaxReuse(stationManagement.activeIDsLTE,stationManagement.BRid,scheduledID,stationManagement.allNeighborsID,appParams.NbeaconsT,appParams.NbeaconsF);

%     elseif simParams.BRAlgorithm==9
% 
%         if mod(timeManagement.elapsedTime_subframes-appParams.NbeaconsT,simParams.Treassign)==0
%             % BRs reassignment (CONTROLLED with POWER CONTROL)
%             [stationManagement.BRid,phyParams.P_ERP_MHz_LTE,stationManagement.lambdaLTE,Nreassign] = BRreassignmentControlledPC(stationManagement.activeIDsLTE,stationManagement.BRid,phyParams.P_ERP_MHz_LTE,sinrManagement.CHgain,awarenessID_LTE,appParams.Nbeacons,stationManagement.lambdaLTE,phyParams.sinrThresholdLTE_LOS,phyParams.Pnoise_MHz,simParams.blockTarget,phyParams.maxERP_MHz);
%         else
%             Nreassign = 0;
%         end
% 
    elseif simParams.BRAlgorithm==10

        % BRs reassignment (CONTROLLED with MINIMUM POWER REUSE)
        [stationManagement.BRid,Nreassign] = BRreassignmentControlledMinPowerReuse(stationManagement.activeIDsLTE,stationManagement.BRid,scheduledID,sinrManagement.P_RX_MHz,sinrManagement.Shadowing_dB,simParams.knownShadowing,appParams.NbeaconsT,appParams.NbeaconsF);

    elseif simParams.BRAlgorithm==101  || (simParams.BRAlgorithm==9 && timeManagement.elapsedTime_subframes == 0) || (simParams.BRAlgorithm==10 && timeManagement.elapsedTime_subframes == 0)
        
        % Call Benchmark Algorithm 101 (RANDOM ALLOCATION)
        [stationManagement.BRid,Nreassign] = BRreassignmentRandom(simValues.IDvehicle,simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams);

    elseif simParams.BRAlgorithm==102

        % Call Benchmark Algorithm 102 (ORDERED ALLOCATION)
        [stationManagement.BRid,Nreassign] = BRreassignmentOrdered(positionManagement.XvehicleReal,stationManagement.activeIDsLTE,stationManagement.BRid,appParams.NbeaconsT,appParams.NbeaconsF);

    end

 end

% if simParams.BRAlgorithm == 18 && ~isfield(sinrManagement,'sensedPowerByLteNo11p')
%     sinrManagement.sensedPowerByLteNo11p = [];
% end

% Incremental sum of successfully reassigned and unlocked vehicles
outputValues.NreassignLTE = outputValues.NreassignLTE + Nreassign;

% Update KPIs for blocked vehicles
blockedIndex = find(stationManagement.BRid(stationManagement.transmittingIDsLTE)==-1);
Nblocked = length(blockedIndex);
for iBlocked = 1:Nblocked
    pckType = stationManagement.pckType(blockedIndex(iBlocked));
    iChannel = stationManagement.vehicleChannel(blockedIndex(iBlocked));
    for iPhyRaw=1:length(phyParams.Raw)
        % Count as a blocked transmission (previous packet is discarded)
        outputValues.NblockedLTE(iChannel,pckType,iPhyRaw) = outputValues.NblockedLTE(iChannel,pckType,iPhyRaw) + nnz(positionManagement.distanceReal(blockedIndex,stationManagement.activeIDsLTE) < phyParams.Raw(iPhyRaw)) - 1; % -1 to remove self
        outputValues.NblockedTOT(iChannel,pckType,iPhyRaw) = outputValues.NblockedTOT(iChannel,pckType,iPhyRaw) + nnz(positionManagement.distanceReal(blockedIndex,stationManagement.activeIDsLTE) < phyParams.Raw(iPhyRaw)) - 1; % -1 to remove self
    end
    if outParams.printPacketReceptionRatio
        for iRaw = 1:1:floor(phyParams.RawMaxLTE/outParams.prrResolution)
            distance = iRaw * outParams.prrResolution;
            outputValues.distanceDetailsCounterLTE(iChannel,pckType,iRaw,4) = outputValues.distanceDetailsCounterLTE(iChannel,pckType,iRaw,4) + nnz(positionManagement.distanceReal(blockedIndex,stationManagement.activeIDsLTE) < distance) - 1; % -1 to remove self
        end
    end
end


