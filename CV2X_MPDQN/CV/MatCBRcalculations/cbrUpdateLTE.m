function [timeManagement,stationManagement,sinrManagement,CBRvalues,coex_cbrLTEonlyValues] = cbrUpdateLTE(timeManagement,vehiclesToConsider,stationManagement,sinrManagement,appParams,simParams,phyParams,outParams)

% ETSI TS 103 574 V1.1.1 (2018-11)
% 5.2 Calculation of CBR [...]
% For PSSCH, CBR is the fraction of "sub-channels" whose S-RSSI exceeds a threshold -94 dBm.
% Note: sensingMatrix is per-RB
% The threshold needs to be converted from sub-channel to RB
threshCBR_perSubchannel = 10^((-94-30)/10); % fixed in this version
threshCBR_perRB = threshCBR_perSubchannel/phyParams.sizeSubchannel;

% The sensingMatrix is a 3D matrix with
% 1st D -> Number of values to be stored in the time domain, corresponding
%          to the standard duration of 1 second, of size ceil(1/Tbeacon)
%          First is current subframe, then the second is teh previous one
%          and so on
% 2nd D -> BRid, of size Nbeacons
% 3rd D -> IDs of vehicles
sensingMatrix = stationManagement.sensingMatrixLTE;
% nAllocationPeriodsCBR counts the number of allocation periods used for the
% sensing
nAllocationPeriodsCBR = ceil(simParams.cbrSensingInterval./stationManagement.averageTbeacon);
% nAllocationPeriodsCBR = min(ceil(simParams.cbrSensingInterval./appParams.averageTbeacon),length(sensingMatrix(:,1,1)));

% MAYBE NOT USED ANYMORE
%subframeLastPacket = mod(ceil(timeManagement.timeLastPacket/phyParams.Tsf)-1,(appParams.NbeaconsT))+1;
%packetInTheLastSubframe(stationManagement.activeIdsLTE) = (subframeLastPacket(stationManagement.activeIdsLTE)==appParams.currentT);

coex_cbrLTEonlyValues = [];

% The list of vehicles that must be updated is provided in input

% CBR is calculated only if enough subframes have elapsed
% if ~isempty(vehiclesToConsider) && timeManagement.elapsedTime_subframes > max(nAllocationPeriodsCBR.*appParams.NbeaconsT)
if ~isempty(vehiclesToConsider) && timeManagement.timeNow > max(nAllocationPeriodsCBR.*appParams.NbeaconsT)
    % Print to output
    %if outParams.printCBR
    %    cbrToPrint = zeros(1,length(vehiclesToConsider));
    %    index = 1;
    %end
   
    % Cycle over the nodes to be updated
    for indexV = 1:length(vehiclesToConsider)
        iV = vehiclesToConsider(indexV);
        % sensingMatrix(-,-,-) > T returns a matrix (recall: comparisons
        % are considering the power per resource block)
        % then reshape converts into a vector and sum sums up
        % IF BR overlap is not allowed, all is simple
        % OTHERWISE the overlap should be taken into account        
        % FOR THE MOMENT IT IS APPROXIMATED
        if ~phyParams.BRoverlapAllowed
            sinrManagement.cbrLTE(iV) = sum(reshape(sensingMatrix(1:100*appParams.NbeaconsF,:,iV) > threshCBR_perRB, [], 1)) ...
                / (nAllocationPeriodsCBR(iV) .* stationManagement.NbeaconsT(iV) * appParams.NbeaconsF);
%              sinrManagement.cbrLTE(iV) = sum(reshape(sensingMatrix(1:1:nAllocationPeriodsCBR(iV)*appParams.averageTbeacon(iV),:,iV) > threshCBR_perRB, [], 1)) ...
%                 / (nAllocationPeriodsCBR .* appParams.NbeaconsT * appParams.NbeaconsF);        
        else
            % I store in BRoccupied the evaluation of the power sensed in
            % each beacon resource
            % The vector length is nBeaconsF x nBeaconsT x nAllocationPeriodsCBR            
            BRfree = reshape(sensingMatrix(1:nAllocationPeriodsCBR(iV),:,iV) <= threshCBR_perRB, 1, []);
            % Then I need to convert from beacon resources to subchannels
            % A matrix is used of size "nSubchannels" x "nBeaconsT x nAllocationPeriodsCBR"
            subchannelFree = true(phyParams.NsubchannelsFrequency,length(BRfree)/appParams.NbeaconsF);
            for i=1:appParams.NbeaconsF
                subchannelFree(i:(i+phyParams.NsubchannelsBeacon-1),:) = subchannelFree(i:(i+phyParams.NsubchannelsBeacon-1),:) & repmat(BRfree(i:appParams.NbeaconsF:end),phyParams.NsubchannelsBeacon,1);
            end    
            sinrManagement.cbrLTE(iV) = sum(sum(~subchannelFree)) ...
                / (nAllocationPeriodsCBR(iV) .* appParams.NbeaconsT(iV) * phyParams.NsubchannelsFrequency);                    
        end
                        
        % recall: phyParams.NsubchannelsBeacon indicates how many subchannel per
        % beacon and thus phyParams.NsubchannelsBeacon-1 is the overlapping
        % phyParams.NsubchannelsFrequency is the number of subchannels
        %if outParams.printCBR
        %    cbrToPrint(index) = sinrManagement.cbrLTE(iV);            
        %    index = index + 1;
        %end
    end
    %if outParams.printCBR
    %    printCBRToFile(cbrToPrint,outParams,false);
    %end
end

%if simParams.technology==4 && simParams.coexMethod>1 && simParams.coex_slotManagement==2
if simParams.technology==4
    % calculation of the CBR_LTE for the dynamic slot duration
    %
    %% Removed in v2.5.10_1
%     % The correct/wrong reception of SCI messages in this subframe is
%     % stored in "stationManagement.correctSCImatrixLTE(,)"
%     % A record with the SCI messages in the last 100 subframes is
%     % required: "stationManagement.coex_correctSCIhistory(subframe,idVehicle)" is used 
%     %
%     % Step 1: circular shift of the matrix and zeros to remove oldest
%     % record
%     sinrManagement.coex_correctSCIhistory(:,:) = circshift(sinrManagement.coex_correctSCIhistory(:,:),appParams.NbeaconsF);
%     sinrManagement.coex_correctSCIhistory(1:appParams.NbeaconsF,:) = 0;
%     % Step 2: new record
%     if  ~isempty(stationManagement.transmittingIDsLTE)
%         for i = 1:length(stationManagement.transmittingIDsLTE)
%             indexVtxLte = stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(i);
%             for indexNeighborsOfVtx = 1:length(stationManagement.neighborsIDLTE(indexVtxLte,:))
%                idVrx = stationManagement.neighborsIDLTE(indexVtxLte,indexNeighborsOfVtx);
%                if idVrx<=0
%                    break;
%                end
%                if stationManagement.correctSCImatrixLTE(i,indexNeighborsOfVtx) == 1 % correct reception of the SCI
%                    sinrManagement.coex_correctSCIhistory(mod(stationManagement.BRid(stationManagement.transmittingIDsLTE(i))-1,appParams.NbeaconsF)+1,idVrx) = 1;
%                end
%             end
%         end
%     end
    %% Removed from here since version 5.2.9 - now in main
    % % Possible step 3: update of the cbr_11p
    %if simParams.coex_cbrTotVariant==2
    %    for iV = vehiclesToConsider'
    %        [timeManagement,stationManagement,sinrManagement.cbrLTE_coex11ponly(iV)] = cbrUpdate11p(timeManagement,iV,stationManagement,simParams,outParams);
    %    end
    %end 
    %%

    % Step 3: update of the cbr
    if ~isempty(vehiclesToConsider) && timeManagement.elapsedTime_subframes > nAllocationPeriodsCBR * appParams.NbeaconsT
        if simParams.coexMethod>1 && simParams.coex_slotManagement==2 && simParams.coex_printTechPercentage
            filenameTechP = sprintf('%s/coex_TechPercStatistic_%.0f.xls',outParams.outputFolder,outParams.simID);
            fpTechP = fopen(filenameTechP,'at');
        end        
        % same timing as std CBR
        for iV = vehiclesToConsider'
            
            %% VARIANTS for CBR_LTE
            %
            % 1: number of received SCIs over number of beacon resources
            %    this is valid only if all packets use the same number of
            %    subchannels - cannot be the one in the standards
            if simParams.coex_cbrLteVariant==1
                sinrManagement.cbrLTE_coexLTEonly(iV) = (sum( sinrManagement.coex_correctSCIhistory(:,iV) )/((appParams.NbeaconsT * appParams.NbeaconsF)));
            %
            % 2: number of received SCIs over number of subchannels
            %    expected to understimate the use of the channel (saturates
            %    before 100%)
            elseif simParams.coex_cbrLteVariant==2
                sinrManagement.cbrLTE_coexLTEonly(iV) = (sum( sinrManagement.coex_correctSCIhistory(:,iV) )/((appParams.NbeaconsT * phyParams.NsubchannelsFrequency)));
            %
            % 3: CBR_LTE = sum( SCI*subchannels_per_SCI ) / (num_subchannels*num_subframes)
            %    NOTE: with this definition and overlapping packets, it migth
            %    happen that the CBR_LTE goes above 100%
            elseif simParams.coex_cbrLteVariant==3
                sinrManagement.cbrLTE_coexLTEonly(iV) = (sum( sinrManagement.coex_correctSCIhistory(:,iV)*phyParams.NsubchannelsBeacon)/((appParams.NbeaconsT * phyParams.NsubchannelsFrequency)));
            %
            % 4: CBR_LTE = sum( subchannels_used ) / (num_subchannels*num_subframes)
            elseif simParams.coex_cbrLteVariant==4
                if ~phyParams.BRoverlapAllowed
                     sinrManagement.cbrLTE_coexLTEonly(iV) = (sum( sinrManagement.coex_correctSCIhistory(:,iV)*phyParams.NsubchannelsBeacon)...
                         /((appParams.NbeaconsT * phyParams.NsubchannelsFrequency)));
                else
                    % I store in BRoccupied the evaluation of the power sensed in
                    % each beacon resource
                    % The vector length is nBeaconsF x nBeaconsT x nAllocationPeriodsCBR            
                    BRfree = ~sinrManagement.coex_correctSCIhistory(:,iV)';
                    % Then I need to convert from beacon resources to subchannels
                    % A matrix is used of size "nSubchannels" x "nBeaconsT x nAllocationPeriodsCBR"
                    subchannelFree = true(phyParams.NsubchannelsFrequency,length(BRfree)/appParams.NbeaconsF);
                    for i=1:appParams.NbeaconsF
                        subchannelFree(i:(i+phyParams.NsubchannelsBeacon-1),:) = subchannelFree(i:(i+phyParams.NsubchannelsBeacon-1),:) & repmat(BRfree(i:appParams.NbeaconsF:end),phyParams.NsubchannelsBeacon,1);
                    end    
                    sinrManagement.cbrLTE_coexLTEonly(iV) = sum(sum(~subchannelFree)) ...
                        / (appParams.NbeaconsT * phyParams.NsubchannelsFrequency);                    
                end
            elseif simParams.coex_cbrLteVariant==5
                BRfree = ~sinrManagement.coex_correctSCIhistory(:,iV)';
                subframeFree = true(1,length(BRfree)/appParams.NbeaconsF);
                for i=1:appParams.NbeaconsF
                    subframeFree = subframeFree & BRfree(i:appParams.NbeaconsF:end);
                end    
                sinrManagement.cbrLTE_coexLTEonly(iV) = sum(sum(~subframeFree)) ...
                    / (appParams.NbeaconsT);                    
            else
                error('Variant of CBR_LTE not implemented');
            end
            
            if simParams.coexMethod>1 
                % Update of the parameter "simParams.coex_NtsLTE"
                if simParams.coex_cbrTotVariant==1
                    Tech_percentage = sinrManagement.cbrLTE_coexLTEonly(iV)./sinrManagement.cbrLTE(iV);
                else %simParams.coex_cbrTotVariant==2
                    Tech_percentage = sinrManagement.cbrLTE_coexLTEonly(iV)./(sinrManagement.cbrLTE_coexLTEonly(iV)+sinrManagement.cbrLTE_coex11ponly(iV));
                end
                % The new number of subframes that can be used by node iV is
                % calculated
                % A factor '(simParams.coex_superframeSF/10)' is added to cope 
                % with possible value different to 10
                if simParams.coex_slotManagement==2 
                    sinrManagement.coex_NtsLTE(iV) = round((simParams.coex_superframeSF/10) * max(min(floor(Tech_percentage*10+0.5),9),1));
                end
                if simParams.coexMethod>1 && simParams.coex_slotManagement==2 && simParams.coex_printTechPercentage
                    %fprintf(fpTechP,'%f\t%d\t%f\t%d\n',timeManagement.timeNow,iV,Tech_percentage,sinrManagement.coex_NtsLTE(iV));
                    if simParams.coex_cbrTotVariant==1
                        fprintf(fpTechP,'%f\t%d\t%f\t%f\t%f\t%d\n',timeManagement.timeNow,iV,sinrManagement.cbrLTE_coexLTEonly(iV),sinrManagement.cbrLTE(iV),Tech_percentage,sinrManagement.coex_NtsLTE(iV));
                    else
                        fprintf(fpTechP,'%f\t%d\t%f\t%f\t%f\t%d\n',timeManagement.timeNow,iV,sinrManagement.cbrLTE_coexLTEonly(iV),(sinrManagement.cbrLTE_coexLTEonly(iV)+sinrManagement.cbrLTE_coex11ponly(iV)),Tech_percentage,sinrManagement.coex_NtsLTE(iV));
                    end
                end
            end
        end
%             figure(1)
%             bar([sinrManagement.cbrLTE sinrManagement.cbrLTE_coexLTEonly sinrManagement.cbrLTE_coex11ponly]);
% %             figure(2)
% %             bar(sinrManagement.cbrLTE_coexLTEonly./sinrManagement.cbrLTE);
%             figure(3)
%             bar(sinrManagement.coex_NtsLTE);
        if simParams.coexMethod>1 && simParams.coex_slotManagement==2 && simParams.coex_printTechPercentage
            fclose(fpTechP);
        end        
    end
    coex_cbrLTEonlyValues = sinrManagement.cbrLTE_coexLTEonly(vehiclesToConsider);
else
    coex_cbrLTEonlyValues = sinrManagement.cbrLTE(vehiclesToConsider);
end

CBRvalues = sinrManagement.cbrLTE(vehiclesToConsider);

if ~isempty(CBRvalues) && (min(CBRvalues)<-1e-6 || max(CBRvalues)>1+1e-6)
    error('Some CBRvalue is lower than 0 or higher than 1...');
end

if simParams.dcc_active
    % ETSI TS 103 574 V1.1.1, page 8, Table 1
    % ( CAM is PPP 5 )                
    CRlimit = 1*(CBRvalues<=0.3) + 0.03*(CBRvalues>0.3 & CBRvalues<=0.65) + 0.006*(CBRvalues>0.65 & CBRvalues<=0.8) + 0.003*(CBRvalues>0.8);
    timeManagement.dcc_minInterval(vehiclesToConsider) = (phyParams.NsubchannelsBeacon)/(phyParams.NsubchannelsFrequency) * phyParams.Tsf ./ CRlimit;          
    if timeManagement.dcc_minInterval(vehiclesToConsider)>timeManagement.generationInterval(vehiclesToConsider)
        stationManagement.dccLteTriggered(stationManagement.vehicleChannel(vehiclesToConsider)) = true;
    end
end
