function [timeManagement,stationManagement,sinrManagement,Nreassign,appParams] = BRreassignment3GPPmode4(timeManagement,stationManagement,positionManagement,sinrManagement,simParams,phyParams,appParams,outParams)
% Sensing-based autonomous resource reselection algorithm (3GPP MODE 4)
% as from 3GPP TS 36.321 and TS 36.213
% Resources are allocated for a Resource Reselection Period (SPS)
% Sensing is performed in the last 1 second
% Map of the received power and selection of the best 20% transmission hypothesis
% Random selection of one of the M best candidates
% The selection is rescheduled after a random period, with random
% probability controlled by the input parameter 'probResKeep'

% Number of TTIs per beacon period，每周期资源数，要改
NbeaconsT = appParams.NbeaconsT;
% Number of possible beacon resources in one TTI,每子帧资源数，不需要改
NbeaconsF = appParams.NbeaconsF;
% Number of beacons per beacon period
% Nbeacons = NbeaconsT*NbeaconsF;

%确定当前的T2大小
% simParams.subframeT2Mode4 = stationManagement.RRI;
% appParams.averageTbeacon = stationManagement.RRI;


% Calculate current T within the NbeaconsT
% currentT = mod(timeManagement.elapsedTime_subframes-1,NbeaconsT)+1; 
currentT = mod(floor(timeManagement.timeNow)-1,NbeaconsT)+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FIRST PART is checking the stations that need reselection

% LTE vehicles that are active
activeIdsLTE = stationManagement.activeIDsLTE;
transmittedIDs = stationManagement.transmittedIDs;
% 1: check if a reselection is commanded by the PHY layer - i.e., in the case 
% the resource is not available in the interval T1-T2
% 2: check if (a) the reselection counter goes to zero and (b) reselection is
% commanded depending on p_keep

%% 1 - check reallocation commanded due to non available resource
%对于subframesToNextAlloc只能计算前或后半部分，所以如果下一资源大的超过T2或者小得在加上RRI后仍＜T1则确定...
% identify those vehicles having a new packet and no scheduled resource in the next T1-T2 interval
% subframeNextResource = ceil(stationManagement.BRid./appParams.NbeaconsF);
subframeNextResource = ceil(stationManagement.BRid/appParams.NbeaconsF);  %BRid变化引起下一传输子帧变化
subframesToNextAlloc = (subframeNextResource>currentT).*(subframeNextResource-currentT)+(subframeNextResource<=currentT).*(subframeNextResource+appParams.NbeaconsT-currentT);

% Following line for debug purposes - allows to remove PHY commanded reallocations-用于调试，可删
% scheduledID_PHY = activeIdsLTE(max(timeManagement.timeLastPacket,[],2)/1000 > timeManagement.timeNow/1000-phyParams.Tsf-1e-8 & (subframesToNextAlloc(activeIdsLTE) < simParams.subframeT1Mode4 | subframesToNextAlloc(activeIdsLTE)>simParams.subframeT2Mode4));
scheduledID_PHY = activeIdsLTE(max(timeManagement.timeLastPacket,[],2)/1000 > timeManagement.timeNow/1000 & (subframesToNextAlloc(activeIdsLTE) < simParams.subframeT1Mode4 | subframesToNextAlloc(activeIdsLTE)>simParams.subframeT2Mode4));

% if scheduledID_PHY>0
%     scheduledID_PHY
% end
scheduledID_PHY(max(timeManagement.timeLastPacket(scheduledID_PHY))<0) = [];
%scheduledID_PHY = [];

%% 2a - reselection counter to 0
% Evaluate which vehicles have the counter reaching zero

% LTE vehicles that have a resource allocated in this subframe
% haveResourceThisTbeacon = zeros(length(subframeNextResource),1);
haveResourceThisTbeacon = zeros(length(subframeNextResource),1);
haveResourceThisTbeacon(transmittedIDs) = 1;

% 必须发射才算一次占用  如果当前时刻没有包，计数器是不减小的
% % % haveResourceThisTbeacon(activeIdsLTE) = haveResourceThisTbeacon(activeIdsLTE).*(sum(stationManagement.pckBuffer(activeIdsLTE,1,:),3)>0);
% haveResourceThisTbeacon(activeIdsLTE) = haveResourceThisTbeacon(activeIdsLTE).*(stationManagement.pckBuffer1>0);
% Update of next allocation for the vehicles that have a resource allocated
% in this subframe
% TODO to modify 'appParams.averageTbeac
% on' into something like
% 'LTE_allocation_period'

% timeManagement.timeOfResourceAllocationLTE is for possible future use
%timeManagement.timeOfResourceAllocationLTE(haveResourceThisTbeacon>0) = timeManagement.timeOfResourceAllocationLTE(haveResourceThisTbeacon>0) + appParams.averageTbeacon;

% Update resReselectionCounter
% Reduce the counter by one to all those that have a packet generated in
% this subframe
stationManagement.resReselectionCounterLTE(activeIdsLTE) = stationManagement.resReselectionCounterLTE(activeIdsLTE)-haveResourceThisTbeacon(activeIdsLTE);
% stationManagement.resRCt = stationManagement.resReselectionCounterLTE==0;
% Among them, those that have reached 0 need to perform reselection
% Calculate IDs of vehicles which perform reselection
scheduledID_MAC = find (stationManagement.resReselectionCounterLTE==0);
% scheduledID_MAC = find (stationManagement.lt(scheduledID_MAC,:)>0);

% FOR DEBUG
% fid = fopen('temp.xls','a');
% for i=1:length(resReselectionCounter)
%     fprintf(fid,'%d\t',resReselectionCounter(i));
% end
% fprintf(fid,'\n');
% fclose(fid);

%% For the nodes with the counter reaching zero or with enforced reselction restart the reselection counter
% Calculate new resReselectionCounter for scheduledID
% needReselectionCounterRestart = union(scheduledID_PHY,scheduledID_MAC);
needReselectionCounterRestart = scheduledID_MAC;
% if ~isempty(needReselectionCounterRestart)
%     a=1;
% end

%%%%%%%%%%%替换station.A
% stationManagement.RRI(needReselectionCounterRestart)= (stationManagement.B(needReselectionCounterRestart))';
% % stationManagement.RRI(needReselectionCounterRestart)=(stationManagement.A(ceil(3*rand(length(needReselectionCounterRestart),1))))';
% simParams.subframeT2Mode4(needReselectionCounterRestart) = stationManagement.RRI(needReselectionCounterRestart)*1000;
% appParams.averageTbeacon(needReselectionCounterRestart) = stationManagement.RRI(needReselectionCounterRestart);
% appParams.NbeaconsT = floor(appParams.averageTbeacon./phyParams.Tsf);
% appParams.Nbeacons = appParams.NbeaconsF.*appParams.NbeaconsT;

% stationManagement.resReselectionCounterLTE(needReselectionCounterRestart) = (simParams.minRandValueMode4-1) + randi((simParams.maxRandValueMode4-simParams.minRandValueMode4)+1,1,length(needReselectionCounterRestart));
%minRandValueMode4 = 0.5./stationManagement.RRI;    maxRandValueMode4 = ./stationManagement.RRI
if any(scheduledID_MAC)
    stationManagement.subframeT2Mode4(scheduledID_MAC) = stationManagement.RRI(scheduledID_MAC)*1000;
    stationManagement.averageTbeacon(scheduledID_MAC) = stationManagement.RRI(scheduledID_MAC);
    stationManagement.NbeaconsT(scheduledID_MAC) = floor(stationManagement.averageTbeacon(scheduledID_MAC)./phyParams.Tsf);
    stationManagement.Nbeacons(scheduledID_MAC) = appParams.NbeaconsF.*stationManagement.NbeaconsT(scheduledID_MAC);
end
stationManagement.resReselectionCounterLTE(needReselectionCounterRestart) = 0.5./stationManagement.RRI(needReselectionCounterRestart) + round(rand(length(needReselectionCounterRestart),1,1)./stationManagement.RRI(needReselectionCounterRestart));
stationManagement.ReCounterLTE0(needReselectionCounterRestart) = stationManagement.resReselectionCounterLTE(needReselectionCounterRestart);
%以上为给RC为0的车辆分配资源占用次数，以下为需要重新选择资源的车辆执行重选过程

%% 2b - p_keep check
% For the nodes with the counter reaching zero, check if reselection should be performed based on p_keep
% For those that have the counter reaching 0, a random variable should be drawn
% to define if the resource is kept or not, based on the input parameter probResKeep

% if simParams.probResKeep>0
%     keepRand = rand(1,length(scheduledID_MAC));
%     % Update the vehicles which perform reselection
%     scheduledID_MAC = scheduledID_MAC(keepRand >= simParams.probResKeep);
% end

% else all vehicles with the counter reaching zero perform the reselection


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SECOND PART is performing the reselection
% Merge the scheduled IDs
scheduledID = union(scheduledID_PHY,scheduledID_MAC);
Nscheduled = length(scheduledID);

% Reset number of successfully reassigned vehicles
Nreassign = 0;
for indexSensingV = 1:Nscheduled
    inda=scheduledID(indexSensingV);
    subframeT2Mode4 = stationManagement.subframeT2Mode4(inda);
%     averageTbeacon = appParams.averageTbeacon(inda);
    NbeaconsT  = stationManagement.NbeaconsT(inda);
    Nbeacons = stationManagement.Nbeacons(inda);
    % Select the sensing matrix only for those vehicles that perform reallocation
    % and calculate the average of the measured power over the sensing window 
    %计算感知车辆在感知窗口中，子帧上平均功率，即RSSI -----除以各自的
    sensingMatrixScheduled = reshape(stationManagement.sensingMatrixLTE(:,:,inda),[],Nbeacons);
    sensingMatrixScheduled = sum(sensingMatrixScheduled,1)/length(sensingMatrixScheduled(:,1));
    % "sensingMatrixScheduled" is a '1 x NbeaconIntervals' vector
    
    % With intrafrequency coexistence, any coexistence method
    % (simParams.coexMethod==1,2,3,6) might forbid LTE using some subframes
    if simParams.technology==4 && simParams.coexMethod~=0
        %if simParams.coexMethod==1 || simParams.coexMethod==2 || simParams.coexMethod==3 || simParams.coexMethod==6
        %MBest = ceil(Nbeacons * (sinrManagement.coex_NtsLTE(activeIdsLTE(indexSensingV))/simParams.coex_superframeSF) * simParams.ratioSelectedMode4);
        for block = 1:ceil(NbeaconsT/simParams.coex_superframeSF)
            sensingMatrixScheduled(...
                (block-1)*simParams.coex_superframeSF*NbeaconsF + ...
                ((((sinrManagement.coex_NtsLTE(activeIdsLTE(indexSensingV)))*NbeaconsF)+1):(simParams.coex_superframeSF*NbeaconsF))...
                ) = inf;
        end            
    end
    
    % Check T1 and T2 and in case set the subframes that are not acceptable to
    % Since the currentT can be at any point of beacon resource matrix,
    % the calculations depend on where T1 and T2 are placed
    % Note: phyParams.TsfGap is needed in the calculation because this
    % function is performed before the gap and not at the end of the
    % subframeinda
    
    a= find(timeManagement.timeLastPacket(inda,:)>0);
    timeStartingT = inf;
    for i=1:length(a)
        if timeStartingT>timeManagement.timeLastPacket(inda,a(i))
            timeStartingT=timeManagement.timeLastPacket(inda,a(i));
        end
    end

    startingT = mod(floor((timeStartingT+phyParams.TsfGap+1e-7)/phyParams.Tsf),NbeaconsT)+1; 
    % IF Both T1 and T2 are within this beacon period
    if (startingT+subframeT2Mode4+1)<=NbeaconsT
        sensingMatrixScheduled([1:((startingT+simParams.subframeT1Mode4-1)*NbeaconsF),((startingT+subframeT2Mode4)*NbeaconsF+1):Nbeacons]) = inf;
    % IF Both are beyond this beacon period
    elseif (startingT+simParams.subframeT1Mode4-1)>NbeaconsT
        sensingMatrixScheduled([1:((startingT+simParams.subframeT1Mode4-1-NbeaconsT)*NbeaconsF),((startingT+subframeT2Mode4-NbeaconsT)*NbeaconsF+1):Nbeacons]) = inf;
    % IF T1 within, T2 beyond
    else
        sensingMatrixScheduled(((startingT+subframeT2Mode4-NbeaconsT)*NbeaconsF+1):((startingT+simParams.subframeT1Mode4-1)*NbeaconsF)) = inf;
    end 
    
%     figure(1)
%     hold off
%     bar(isinf(sensingMatrixScheduled))
%     hold on

    % The best 20% (parameter that can be changed) is selected inside the pool as in TS 36.213
    % The pool of available resources is obtained as those that are not set
    % to infty
%     nPossibleAllocations = sum(isfinite(sensingMatrixScheduled));  %判断是否inf
%     MBest = ceil(nPossibleAllocations * simParams.ratioSelectedMode4);     
    MBest = ceil(NbeaconsT * simParams.ratioSelectedMode4);        % SW的20%
%     if MBest<=0          
%         error('Mbest must be a positive scalar (it is %d)',MBest);
%     end


    % The knownUsedMatrix of the scheduled users is obtained,
    % 当前时刻哪些资源被占用,占用的要计算RSRP来排除%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    knownUsedMatrixScheduled = stationManagement.knownUsedMatrixLTE(:,inda)';

    % Create random permutation of the column indexes of sensingMatrix in
    % order to avoid the ascending order on the indexes of cells with the
    % same value (sort effect) -> minimize the probability of choosing the same
    % resource
    rpMatrix = randperm(Nbeacons);

    % Build matrix made of random permutations of the column indexes
    % Permute sensing matrix
    sensingMatrixPerm = sensingMatrixScheduled(rpMatrix);
    knownUsedMatrixPerm = knownUsedMatrixScheduled(rpMatrix);

    % Now perform sorting and relocation taking into account the threshold on RSRP
    % Please note that the sensed power is on a per MHz resource basis,
    % whereas simParams.powerThresholdMode4 is on a resource element (15 kHz) basis, 
    % The cycle is stopped internally; a max of 100 is used to avoid
    % infinite loops in case of bugs  RSRP
    powerThreshold = simParams.powerThresholdMode4;
    while powerThreshold < 100
        % If the number of acceptable BRs is lower than MBest,
        % powerThreshold is increased by 3 dB
        usableBRs = ((sensingMatrixPerm*0.015)<powerThreshold) | ((sensingMatrixPerm<inf) & (knownUsedMatrixPerm<1));
        if sum(usableBRs) < MBest
            powerThreshold = powerThreshold * 2;
        else
            break;
        end
    end        
    
    % To mark unacceptable RB as occupied, their power is set to
    % Inf，将RSSI中最高的赋值为忙？
    sensingMatrixPerm = sensingMatrixPerm + (1-usableBRs) * max(phyParams.P_ERP_MHz_LTE);
    
    % Sort sensingMatrix in ascending order
    [~, bestBRPerm] = sort(sensingMatrixPerm);

    % Reorder bestBRid matrix
    bestBR = rpMatrix(bestBRPerm);

    % Keep the best M canditates
    bestBR = bestBR(1:MBest);

    % Reassign, selecting a random BR among the bestBR
    BRindex = randi(MBest);
    BR = bestBR(BRindex);
	printDebugReallocation(timeManagement.timeNow,inda,positionManagement.XvehicleReal(stationManagement.activeIDs==inda),'reall',BR,outParams);
    
    stationManagement.BRid(inda)=BR;
    Nreassign = Nreassign + 1;
    
    printDebugBRofMode4(timeManagement,inda,BR,outParams);
end

% % % Optional: if it is assumed that the reselction counter is sent in the SCI          
if simParams.lteSCIinEmptyResource
    % % Reduce the knownUsedMatrix by 1 (not a problem if it goes below 0) for
    % % those vehicles that have checked in this subframe if it is time to change
    % % allocation
    % % NOTE: the function repmat(V,n,m) creates n copies in the 1st dimension 
    % % and m copies in the 2nd dimension of the vector V
    stationManagement.knownUsedMatrixLTE = stationManagement.knownUsedMatrixLTE - repmat(haveResourceThisTbeacon',length(stationManagement.knownUsedMatrixLTE(:,1)),1);
    % IF this is used - the knownUsedMatrix should not be reset in the sensing
    % procedure
end

end

