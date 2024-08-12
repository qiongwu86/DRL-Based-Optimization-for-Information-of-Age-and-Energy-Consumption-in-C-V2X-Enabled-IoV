function [appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement] = ...
    mainPositionUpdate(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement)
% the position of all vehicles is updated
if simParams.typeOfScenario~=2 % Not traffic trace
    % Call function to update vehicles positions
    [indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,stationManagement.activeIDsExit,positionManagement] = updatePosition(timeManagement.timeNow/1000,stationManagement.activeIDs,simValues.v,simValues.direction,simParams.positionTimeResolution,simValues.Xmax,positionManagement,appParams,simValues,outParams);
else
    % Store IDs of vehicles at the previous beacon period and update positions
    [positionManagement.XvehicleReal,positionManagement.YvehicleReal,stationManagement.activeIDs,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,activeIDsExit,simValues.v] = updatePositionFile(round(timeManagement.timeNextPosUpdate*100)/100,simValues.dataTrace,stationManagement.activeIDs,positionManagement.XvehicleReal,positionManagement.YvehicleReal,round(timeManagement.timeNextPosUpdate*100)/100-simParams.positionTimeResolution,simValues,outParams);
    %% ONLY LTE
    if sum(stationManagement.vehicleState(stationManagement.activeIDs)==100)>0
    %if simParams.technology ~= 2 % not only 11p
        % Update stationManagement.BRid vector (variable number of vehicles in the scenario)
        [stationManagement.BRid] = updateBRidFile(stationManagement.BRid,stationManagement.activeIDs,indexNewVehicles);
    end
end
activeIDsExit = stationManagement.activeIDsExit;
% if ~isempty(activeIDsExit)
%     activeIDsExit
% end
% Vectors IDvehicleLTE and IDvehicle11p are updated
stationManagement.activeIDsLTE = stationManagement.activeIDs.*(stationManagement.vehicleState(stationManagement.activeIDs)==100);
stationManagement.activeIDsLTE = stationManagement.activeIDsLTE(stationManagement.activeIDsLTE>0);
stationManagement.activeIDs11p = stationManagement.activeIDs.*(stationManagement.vehicleState(stationManagement.activeIDs)~=100);
stationManagement.activeIDs11p = stationManagement.activeIDs11p(stationManagement.activeIDs11p>0);
stationManagement.indexInActiveIDs_ofLTEnodes = zeros(length(stationManagement.activeIDsLTE),1);
for i=1:length(stationManagement.activeIDsLTE)
    stationManagement.indexInActiveIDs_ofLTEnodes(i) = find(stationManagement.activeIDs==stationManagement.activeIDsLTE(i));
end
stationManagement.indexInActiveIDs_of11pnodes = zeros(length(stationManagement.activeIDs11p),1);
for i=1:length(stationManagement.activeIDs11p)
    stationManagement.indexInActiveIDs_of11pnodes(i) = find(stationManagement.activeIDs==stationManagement.activeIDs11p(i));
end
% % For possible DEBUG
% figure(300)
% plot(timeManagement.timeNextPosUpdate*100*ones(1,length(positionManagement.XvehicleReal)),positionManagement.XvehicleReal,'*');
% hold on
% Update variables for resource allocation in LTE-V2V% Update stationManagement.resReselectionCounterLTE for vehicles exiting the scenario
%if simParams.technology ~= 2 % not only 11p
if sum(stationManagement.vehicleState(stationManagement.activeIDs)==100)>0
    if simParams.BRAlgorithm==18 && timeManagement.timeNow/1000 > phyParams.Tsf
        % First random allocation 
        if ~isempty(indexNewVehicles)
            %[stationManagement.BRid,~] = BRreassignmentRandom(simValues.IDvehicle,stationManagement.BRid,simParams,sinrManagement,appParams);
%             stationManagement.BRid(indexNewVehicles) = (round(rand(length(indexNewVehicles),1,1).*appParams.Nbeacons(indexNewVehicles)));
%             [stationManagement.BRid(stationManagement.activeIDs(indexNewVehicles)),~] = BRreassignmentRandom(stationManagement.activeIDs(indexNewVehicles),simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams);
            if simParams.technology==4 && (simParams.coexMethod==1 || simParams.coexMethod==2 || simParams.coexMethod==6)
                timeManagement.coex_timeNextSuperframe(stationManagement.activeIDs(indexNewVehicles)) = timeManagement.timeNow/1000 + ...
                    (simParams.coex_endOfLTE + simParams.coex_guardTimeAfter) * ones(length(stationManagement.activeIDs(indexNewVehicles)),1);
                timeManagement.coex_timeNextSuperframe(stationManagement.activeIDs(indexNewVehicles)) = timeManagement.coex_timeNextSuperframe(stationManagement.activeIDs(indexNewVehicles)) +...
                    ( rand(length(stationManagement.activeIDs(indexNewVehicles)),1) * (2*simParams.coexA_desynchError) - simParams.coexA_desynchError);
            end
        end
        % Update stationManagement.resReselectionCounterLTE for vehicles exiting the scenario
        % stationManagement.resReselectionCounterLTE(activeIDsExit) = Inf;
        % Update stationManagement.resReselectionCounterLTE for vehicles entering the scenario
        % a) LTE vehicles that enter or are blocked start with a counter set to 0
        % b) 11p vehicles are set to Inf
        % stationManagement.resReselectionCounterLTE(logical((stationManagement.BRid==-1) .* (stationManagement.vehicleState==100))) = 0;
        % stationManagement.resReselectionCounterLTE(logical((stationManagement.BRid==-1) .* (stationManagement.vehicleState~=100))) = Inf;
        % Reset stationManagement.errorSCImatrixLTE for new computation of correctly received SCIs
        %stationManagement.correctSCImatrixLTE = zeros(length(stationManagement.activeIDsLTE),length(stationManagement.activeIDsLTE)-1);
    end
    
    % Add LTE positioning delay (if selected)
    [simValues.XvehicleEstimated,simValues.YvehicleEstimated,PosUpdateIndex] = addPosDelay(simValues.XvehicleEstimated,simValues.YvehicleEstimated,positionManagement.XvehicleReal,positionManagement.YvehicleReal,stationManagement.activeIDs,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,positionManagement.posUpdateAllVehicles,simParams.positionTimeResolution);
    % Add LTE positioning error (if selected)
    % (Xvehicle, Yvehicle): fictitious vehicles' position seen by the eNB
    [simValues.XvehicleEstimated(PosUpdateIndex),simValues.YvehicleEstimated(PosUpdateIndex)] = addPosError(positionManagement.XvehicleReal(PosUpdateIndex),positionManagement.YvehicleReal(PosUpdateIndex),simParams.sigmaPosError);
end
% Call function to compute the distances
[positionManagement,stationManagement] = computeDistance (simParams,simValues,stationManagement,positionManagement,phyParams);
% Call function to update positionManagement.distance matrix where D(i,j) is the
% change in positionManagement.distance of link i to j from time n-1 to time n and used
% for updating Shadowing matrix
[dUpdate,sinrManagement.Shadowing_dB,positionManagement.distanceRealOld] = updateDistanceChangeForShadowing(positionManagement.distanceReal,positionManagement.distanceRealOld,indexOldVehicles,indexOldVehiclesToOld,sinrManagement.Shadowing_dB,phyParams.stdDevShadowLOS_dB);
% Calculation of channel and then received power
% TODO in future version the two functions to compute the channel gain
% should be included in a single one
[CHgain,sinrManagement.Shadowing_dB,simValues.Xmap,simValues.Ymap,phyParams.LOS] = computeChannelGain(sinrManagement,positionManagement,phyParams,simParams,dUpdate);
% if ~phyParams.winnerModel   
%     [CHgain,sinrManagement.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeChGain(positionManagement.distanceReal,phyParams.L0,phyParams.beta,positionManagement.XvehicleReal,positionManagement.YvehicleReal,phyParams.Abuild,phyParams.Awall,positionManagement.XminMap,positionManagement.YmaxMap,positionManagement.StepMap,positionManagement.GridMap,simParams.fileObstaclesMap,sinrManagement.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
% else
%     [CHgain,sinrManagement.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeChGainWinner(positionManagement.distanceReal,positionManagement.XvehicleReal,positionManagement.YvehicleReal,positionManagement.XminMap,positionManagement.YmaxMap,positionManagement.StepMap,positionManagement.GridMap,simParams.fileObstaclesMap,sinrManagement.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
% end
% Compute RXpower
% NOTE: sinrManagement.P_RX_MHz( RECEIVER, TRANSMITTER) 
sinrManagement.P_RX_MHz = ( (phyParams.P_ERP_MHz_LTE(stationManagement.activeIDs).*(stationManagement.vehicleState(stationManagement.activeIDs)==100))' + ...
    (phyParams.P_ERP_MHz_11p(stationManagement.activeIDs).*(stationManagement.vehicleState(stationManagement.activeIDs)~=100) )' )...
    * phyParams.Gr .* min(1,CHgain) .* sinrManagement.mcoCoefficient(stationManagement.activeIDs,stationManagement.activeIDs);
% Temporary
% if simParams.mco_nVehInterf>0
%     [sinrManagement,positionManagement,outputValues] = mco_interfVehiclesCalculate(timeManagement,stationManagement,sinrManagement,positionManagement,phyParams,appParams,outputValues,simParams);
% end
% Floor coordinates for PRRmap creation (if enabled)
if simParams.typeOfScenario==2 && outParams.printPRRmap % only traffic trace 
    simValues.XmapFloor = floor(simValues.Xmap);
    simValues.YmapFloor = floor(simValues.Ymap);
end
% Call function to calculate effective neighbors (if enabled)
if simParams.neighborsSelection
    %% TODO - needs update
    error('Significant neighbors not updated in v5');
%     if simParams.technology ~= 2 % not only 11p
%         % LTE
%         [stationManagement.awarenessIDLTE,stationManagement.neighborsIDLTE,positionManagement.XvehicleRealOld,positionManagement.YvehicleRealOld,positionManagement.angleOld] = computeSignificantNeighbors(stationManagement.activeIDs,positionManagement.XvehicleReal,positionManagement.YvehicleReal,positionManagement.XvehicleRealOld,positionManagement.YvehicleRealOld,stationManagement.neighborsIDLTE,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,positionManagement.angleOld,simParams.Mvicinity,phyParams.RawLTE,phyParams.RawMaxLTE,stationManagement.neighborsDistance);
%     end
%     if simParams.technology ~= 1 % not only LTE
%         % 11p
%         [stationManagement.awarenessID11p,stationManagement.neighborsID11p,positionManagement.XvehicleRealOld,positionManagement.YvehicleRealOld,positionManagement.angleOld] = computeSignificantNeighbors(stationManagement.activeIDs,positionManagement.XvehicleReal,positionManagement.YvehicleReal,positionManagement.XvehicleRealOld,positionManagement.YvehicleRealOld,stationManagement.neighborsID11p,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,positionManagement.angleOld,simParams.Mvicinity,phyParams.Raw11p,phyParams.RawMax11p,stationManagement.neighborsDistance);
%     end
end
% Call function to compute hidden or non-hidden nodes (if enabled)
if outParams.printHiddenNodeProb
    %% TODO - needs update
    error('printHiddenNodeProb not updated in v5');
    %[outputValues.hiddenNodeSumProb,outputValues.hiddenNodeProbEvents] = computeHiddenNodeProb(stationManagement.activeIDs,positionManagement.distanceReal,sinrManagement.RXpower,phyParams.gammaMin,phyParams.PnRB,outParams.PthRB,outputValues.hiddenNodeSumProb,outputValues.hiddenNodeProbEvents);
end
% Number of vehicles in the scenario
outputValues.Nvehicles = length(stationManagement.activeIDs);
outputValues.NvehiclesTOT = outputValues.NvehiclesTOT + outputValues.Nvehicles;
outputValues.NvehiclesLTE = outputValues.NvehiclesLTE + length(stationManagement.activeIDsLTE);
outputValues.Nvehicles11p = outputValues.Nvehicles11p + length(stationManagement.activeIDs11p);
% Number of neighbors
[outputValues,~,NneighborsRawLTE,NneighborsRaw11p] = updateAverageNeighbors(simParams,stationManagement,outputValues,phyParams);
% Print number of neighbors per vehicle to file (if enabled)
if outParams.printNeighbors
    printNeighborsToFile(timeManagement.timeNow/1000,positionManagement,outputValues.Nvehicles,NneighborsRawLTE/length(stationManagement.activeIDsLTE),NneighborsRaw11p/length(stationManagement.activeIDs11p),outParams,phyParams);
end
% Prepare matrix for update delay computation (if enabled)
if outParams.printUpdateDelay
    % Reset update time of vehicles that are outside the scenario
    allIDOut = setdiff(1:simValues.maxID,stationManagement.activeIDs);
    simValues.updateTimeMatrix11p(allIDOut,:) = -1;
    simValues.updateTimeMatrix11p(:,allIDOut) = -1;
    simValues.updateTimeMatrixLTE(allIDOut,:) = -1;
    simValues.updateTimeMatrixLTE(:,allIDOut) = -1;
end
% Prepare matrix for update delay computation (if enabled)
if outParams.printDataAge
    % Reset update time of vehicles that are outside the scenario
    allIDOut = setdiff(1:simValues.maxID,stationManagement.activeIDs);
    simValues.dataAgeTimestampMatrix11p(allIDOut,:) = -1;
    simValues.dataAgeTimestampMatrix11p(:,allIDOut) = -1;
    simValues.dataAgeTimestampMatrixLTE(allIDOut,:) = -1;
    simValues.dataAgeTimestampMatrixLTE(:,allIDOut) = -1;
end
% Compute wireless blind spot probability (if enabled - update delay is required)
if outParams.printUpdateDelay && outParams.printWirelessBlindSpotProb
     error('Not updated in v. 5.X');
%         %% TODO with coexistence
%         if simParams.technology~=1 && simParams.technology~=2
%             error('Not implemented');
%         end
%         if simParams.technology==2 || elapsedTime_subframes>appParams.NbeacosT
%             if simParams.technology==1
%                 outputValues.wirelessBlindSpotCounter = countWirelessBlindSpotProb(simValues.updateTimeMatrixLTE,outputValues.wirelessBlindSpotCounter,timeManagement.timeNow);
%             else
%                 outputValues.wirelessBlindSpotCounter = countWirelessBlindSpotProb(simValues.updateTimeMatrix11p,outputValues.wirelessBlindSpotCounter,timeManagement.timeNow);
%             end
%         end        
end
% Update of parameters related to transmissions in IEEE 802.11p to cope
% with vehicles exiting the scenario
%if simParams.technology ~= 1 % not only LTE
if sum(stationManagement.vehicleState(stationManagement.activeIDs)~=100)>0    
    
    timeManagement.timeNextTxRx11p(activeIDsExit) = Inf;
    sinrManagement.idFromWhichRx11p(activeIDsExit) = activeIDsExit;
    sinrManagement.instantThisSINRavStarted11p(activeIDsExit) = Inf;
    stationManagement.vehicleState(activeIDsExit(stationManagement.vehicleState(activeIDsExit)~=100)) =  1;
    
    % The average SINR of all vehicles is then updated
    sinrManagement = updateSINR11p(timeManagement,sinrManagement,stationManagement,phyParams);
    % The nodes that may stop receiving must be checked
    [timeManagement,stationManagement,sinrManagement,outputValues] = checkVehiclesStopReceiving11p(timeManagement,stationManagement,sinrManagement,simParams,phyParams,outParams,outputValues);
    % The present overall/useful power received and the instant of calculation are updated
    % The power received must be calculated after
    % 'checkVehiclesStopReceiving11p', to have the correct idFromWhichtransmitting
    [sinrManagement] = updateLastPower11p(timeManagement,stationManagement,sinrManagement,phyParams,simValues);       
end
if ~isempty(activeIDsExit)
%     activeIDsExit
    stationManagement.BRid(activeIDsExit)=0;
    % Reset time next packet and tx-rx for vehicles that exit the scenario
    timeManagement.timeNextPacket(activeIDsExit,:) = Inf;
    % Reset time next packet and tx-rx for vehicles that exit the scenario
    stationManagement.pckBuffer(activeIDsExit,:,:) = zeros(length(activeIDsExit),10,4);
    stationManagement.PHIt(:,activeIDsExit)=0;
    stationManagement.lt(activeIDsExit,:)=0;                                           
    
end
% Generate time values of new vehicles entering the scenario
if ~isempty(stationManagement.activeIDs(indexNewVehicles))
    stationManagement.BRid(indexNewVehicles)=(ceil(rand(length(indexNewVehicles),1,1).*stationManagement.Nbeacons(indexNewVehicles)))';
    % stationManagement.resReselectionCounterLTE(indexNewVehicles) = 0.5./stationManagement.RRI(indexNewVehicles) + round(rand(length(indexNewVehicles),1,1)./stationManagement.RRI(indexNewVehicles));
    timeManagement.timeNextPacket(stationManagement.activeIDs(indexNewVehicles),:)=inf;    
    timeManagement.timeNextPacket(stationManagement.activeIDs(indexNewVehicles),3) = timeManagement.timeNow + round(stationManagement.B(indexNewVehicles).* rand(1,length(indexNewVehicles))*1000);

    stationManagement.lt(activeIDsExit,3) = round(100*rand(length(activeIDsExit),1)/10);
    a= stationManagement.lt(activeIDsExit,3);
    Nexit = length(a);
    b = round(100*rand(Nexit,1));
    for i = 1:Nexit
        stationManagement.pckBuffer(activeIDsExit(i),1:a(i),3) = b(i) + flip(100*((1:a(i))-1));
        stationManagement.PHIt(activeIDsExit(i),:) = repmat(stationManagement.pckBuffer((activeIDsExit(i)), 1, 3), 1, length(stationManagement.PHIt));%/*接受端信息年龄*/
        stationManagement.PHIt(activeIDsExit(i),:) = stationManagement.PHIt(activeIDsExit(i),:) + round(100*rand(1,length(stationManagement.PHIt)));
    end 

end
% if appParams.variabilityTbeacon==-1
%     timeManagement.generationInterval(stationManagement.activeIDs(indexNewVehicles)) = 1000*generationPeriodFromSpeed(simValues.v(indexNewVehicles),appParams);
% else
%     timeManagement.generationInterval(stationManagement.activeIDs(indexNewVehicles)) = 1000*(appParams.averageTbeaco - appParams.variabilityTbeacon/2 + appParams.variabilityTbeacon*rand(length(indexNewVehicles),1));
%     timeManagement.generationInterval(stationManagement.activeIDsLTE) = 1000*appParams.averageTbeaco;
% end
% CBR settings for the new vehicles
if simParams.cbrActive && (outParams.printCBR || (simParams.technology==4 && simParams.coexMethod~=0 && simParams.coex_slotManagement==2))
    timeManagement.cbr11p_timeStartMeasInterval(stationManagement.activeIDs(indexNewVehicles)) = timeManagement.timeNow;
    if simParams.technology==4 && simParams.coexMethod==1        
        timeManagement.cbr11p_timeStartBusy(stationManagement.activeIDs(indexNewVehicles) .* timeManagement.coex_superframeThisIsLTEPart(stationManagement.activeIDs(indexNewVehicles))) = timeManagement.timeNow;
    end
end