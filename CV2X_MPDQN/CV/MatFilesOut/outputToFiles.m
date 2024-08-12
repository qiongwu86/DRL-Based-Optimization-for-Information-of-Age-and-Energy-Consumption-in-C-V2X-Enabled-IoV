function outputToFiles(simVersion,stationManagement,simParams,appParams,phyParams,sinrManagement,outParams,outputValues)
% This function writes one line in the main output file

outputFolder = outParams.outputFolder;
if isfolder(outputFolder) == false
    mkdir(outputFolder)
end

fileNameMain = sprintf('%s/%s',outputFolder,outParams.outMainFile);
fileMainID = fopen(fileNameMain,'at');

if fseek(fileMainID, 1, 'bof') == -1
    %1 Main settings
    fprintf(fileMainID,'SimID\tSim version\tWhen\tSeed\tSimulated duration\tComputation duration\t');
    fprintf(fileMainID,'Technology\t');
    fprintf(fileMainID,'File Cfg\t');
    %2 Scenario
    fprintf(fileMainID,'Vehicles position\tFile obstacles map\t');
    fprintf(fileMainID,'Sim (positionTimeResolution,Limits)\t');
    fprintf(fileMainID,'Sim (PosError,Tupdate,neighborsSelection,Mvicinity)\t');
    %3 App settings
    fprintf(fileMainID,'App (Tbeacon,Bsize,%%ofRes)\t');
    fprintf(fileMainID,'App (Available BRs,RBPairsBeacon,nSubcBeacon*sizeSubc,AdjSCI)\t');
    %4 Phy settings
    fprintf(fileMainID,'Phy (BW,MCS/Mode,Duplex)\t');
    fprintf(fileMainID,'Phy (Ptx,PtxMHz,PnMHz,Gt,Gr,L0,beta,sinrMin_dB)\t');
    fprintf(fileMainID,'Phy (MaxRange2Sigma,RawMaxLOS,RawMaxNLOS,StdDevShadowLOS,StdDevShadowNLOS)\t');
    fprintf(fileMainID,'Phy (Nchannels:proportion)\t');
    fprintf(fileMainID,'11p backoff (CW,AifsN,Snosy,Ssy)\t');
    %5 Algorithm
    fprintf(fileMainID,'LTE algorithm (Alg. params)\t');
    %6 Particularly relevant settings
    fprintf(fileMainID,'Raw\t');
    %7 Outputs - LTE
    fprintf(fileMainID,'Average vehicles in the scenario LTE\tAverage neighbors LTE\tVariance of neighbors LTE\t');
    fprintf(fileMainID,'Average reassignments per vehicle per second LTE\tTot tx LTE\tAv tx LTE\tMedian CBR\tBlocking rate LTE\tError rate LTE\tPacket reception ratio LTE\t');
    %8 Outputs - 11p
    fprintf(fileMainID,'Average vehicles in the scenario 11p\tAverage neighbors 11p\tVariance of neighbors 11p\t');
    fprintf(fileMainID,'Tot tx 11p\tAv tx 11p\tMedian CBR\tBlocking rate 11p\tError rate 11p\tPacket reception ratio 11p\t');
    %9 Outputs - TOT
    fprintf(fileMainID,'Average vehicles in the scenario TOT\tAverage neighbors TOT\tVariance of neighbors TOT\t');
    fprintf(fileMainID,'Tot tx TOT\tBlocking rate TOT\tError rate TOT\tPacket reception ratio TOT\n');
end

%1 Main settings
fprintf(fileMainID,'%.0f\t%s\t%s\t%.0f\t%f\t%f\t',outParams.simID,simVersion,datestr(now),simParams.seed,simParams.simulationTime,outputValues.computationTime);
if simParams.technology==1 || outputValues.AvgNvehicles11p==0
    fprintf(fileMainID,'LTE-V2X\t');
elseif simParams.technology==2 || outputValues.AvgNvehiclesLTE==0
    fprintf(fileMainID,'802.11p\t');
elseif simParams.technology==3
    fprintf(fileMainID,'LTE+802.11p, orthogonal\t');
elseif simParams.technology==4
    fprintf(fileMainID,'LTE+802.11p, coex');
    if simParams.coexMethod==0
        fprintf(fileMainID,', std');
    elseif simParams.coexMethod==1
        if simParams.coex_slotManagement==1
            fprintf(fileMainID,', A static (Tsf=%.3f,Tlte=%.3f,Tg=%.6f',simParams.coex_superFlength,simParams.coex_endOfLTE,simParams.coexA_guardTime);
            if simParams.coexA_improvements>0
                fprintf(fileMainID,',+%d',simParams.coexA_improvements);
            end
            if simParams.coexA_desynchError>0
                fprintf(fileMainID,',err%.6f',simParams.coexA_desynchError);
            end
                
            fprintf(fileMainID,')');
        else
            fprintf(fileMainID,', A non static - err');
        end
    elseif simParams.coexMethod==2
        if simParams.coex_slotManagement==1
            fprintf(fileMainID,', B static (Tsf=%.3f,Tlte=%.3f',simParams.coex_superFlength,simParams.coex_endOfLTE);
        else
            fprintf(fileMainID,', B dynamic (Tsf=%.3f',simParams.coex_superFlength);
        end
        %fprintf(fileMainID,',Tg=%.6f,dPw=%.2f',simParams.coexB_timeBeforeLTEstarts,simParams.coexB_portionOfPower);
        fprintf(fileMainID,',Tg=%.6f',simParams.coexB_timeBeforeLTEstarts);
        if simParams.coexB_allToTransmitInEmptySF
            fprintf(fileMainID,',allTx');
        else
            fprintf(fileMainID,',selTx');
        end
        fprintf(fileMainID,')');
    elseif simParams.coexMethod==3
        if simParams.coex_slotManagement==1
            % In Method C with the variant with a gap,
            % simParams.coex_endOfLTE is set to 0 to mamage the virtual
            % interference
            fprintf(fileMainID,', C static (Tsf=%.3f,Tlte=%.3f',simParams.coex_superFlength,sinrManagement.coex_NtsLTE(1)*phyParams.Tsf);
        else
            fprintf(fileMainID,', C dynamic (Tsf=%.3f',simParams.coex_superFlength);
        end
        if simParams.coexC_timegapVariant==2
            fprintf(fileMainID,',w.gap');
        end
        if simParams.coexC_11pDetection
            fprintf(fileMainID,',w.pdet');
        end
        fprintf(fileMainID,')');
    elseif simParams.coexMethod==6
        if simParams.coex_slotManagement==1
            fprintf(fileMainID,', F static (Tsf=%.3f,Tlte=%.3f)',simParams.coex_superFlength,simParams.coex_endOfLTE);  
        else
            fprintf(fileMainID,', F dynamic (Tsf=%.3f)',simParams.coex_superFlength);  
        end        
    else
        fprintf(fileMainID,', UNKNOWN');
    end
    if simParams.coexMethod>0 && simParams.coex_slotManagement==2
        fprintf(fileMainID,',dyn:vT%d',simParams.coex_cbrTotVariant);        
        %% Removed in v5.2.10
        %fprintf(fileMainID,',dyn:vT%d,vL%d',coex_cbrTotVariant,simParams.coex_cbrLteVariant);
        %if simParams.coex_cbrTotVariant==2 
        %    fprintf(fileMainID,',Pt%d',10*log10(simParams.coex_powerStopSensing11p));
        %end
        %%
    end
    fprintf(fileMainID,'\t');
else
    fprintf(fileMainID,'UNKNOWN\t');
end
fprintf(fileMainID,'%s\t',simParams.fileCfg);

%2 Scenario
if simParams.typeOfScenario == 2 % traffic trace
    fprintf(fileMainID,'TraceFile: %s\t',simParams.filenameTrace);
elseif simParams.typeOfScenario == 4
    fprintf(fileMainID,'ETSIU,');
    fprintf(fileMainID,'NBlocks=%.0f,rho=%.0f,vMean=%.2f,vStDev=%.2f\t',simParams.Nblocks,simParams.rho,simParams.vMean,simParams.vStDev);
else
    if simParams.typeOfScenario == 1 % Legacy PPP
        fprintf(fileMainID,'PPP,');
    
    else
        fprintf(fileMainID,'ETSIH,');
    end
    
    fprintf(fileMainID,'roadLength=%.0f,roadWidth=%.1f,NLanes=%.0f,rho=%.0f,vMean=%.2f,vStDev=%.2f\t',simParams.roadLength,simParams.roadWidth,simParams.NLanes,simParams.rho,simParams.vMean,simParams.vStDev);
end

if simParams.fileObstaclesMap == true
    fprintf(fileMainID,'%s\t',simParams.filenameObstaclesMap);
else
    fprintf(fileMainID,'-\t');
end

if simParams.typeOfScenario == 2 % Traffic trace
    fprintf(fileMainID,'%f',simParams.positionTimeResolution);
    if simParams.XminTrace~=-1 && simParams.XmaxTrace~=-1
        fprintf(fileMainID,',%.1f<X<%.1f',simParams.XminTrace,simParams.XmaxTrace);
    elseif simParams.XminTrace==-1 && simParams.XmaxTrace>=0
        fprintf(fileMainID,',X<%.1f',simParams.XmaxTrace);
    elseif simParams.XmaxTrace==-1 && simParams.XminTrace>=0
        fprintf(fileMainID,',X>%.1f',simParams.XminTrace);
    end
    if simParams.YminTrace~=-1 && simParams.YmaxTrace~=-1
        fprintf(fileMainID,',%.1f<Y<%.1f',simParams.YminTrace,simParams.YmaxTrace);
    elseif simParams.YminTrace==-1 && simParams.YmaxTrace>=0
        fprintf(fileMainID,',Y<%.1f',simParams.YmaxTrace);
    elseif simParams.YmaxTrace==-1 && simParams.YminTrace>=0
        fprintf(fileMainID,',Y>%.1f',simParams.YminTrace);
    end
    if simParams.XminTrace==-1 && simParams.XmaxTrace==-1 && simParams.YminTrace==-1 && simParams.YmaxTrace==-1
        fprintf(fileMainID,',-');
    end
else
    fprintf(fileMainID,'%f,-',simParams.positionTimeResolution);
end
if appParams.nRSUs>0
    fprintf(fileMainID,'+%dRSU',appParams.nRSUs);
end
fprintf(fileMainID,'\t');

if outputValues.AvgNvehiclesLTE>0
%if simParams.technology ~= 2 % not only 11p
    fprintf(fileMainID,'%.1f,',simParams.posError95);
    if simParams.Tupdate > simParams.simulationTime
        fprintf(fileMainID,'inf,');
    else
        fprintf(fileMainID,'%f,',simParams.Tupdate);
    end
else
    fprintf(fileMainID,'-,-,');
end

if simParams.neighborsSelection
    fprintf(fileMainID,'true,%0.f',simParams.Mvicinity);
else
    fprintf(fileMainID,'false,-');
end

fprintf(fileMainID,'\t');

%3 App settings
fprintf(fileMainID,'%.3f',appParams.averageTbeacon);
if appParams.variabilityTbeacon==-1
    fprintf(fileMainID,'(auto');
    if strcmp(appParams.camDiscretizationType,'allSteps') 
        fprintf(fileMainID,'-alls-%.1f',appParams.camDiscretizationIncrease);
    elseif strcmp(appParams.camDiscretizationType,'allocationAligned') 
        fprintf(fileMainID,'-algn-%.1f',appParams.camDiscretizationIncrease);
    end    
    fprintf(fileMainID,')');
elseif appParams.variabilityTbeacon>0
    fprintf(fileMainID,'(+/-%.3f,11p)',appParams.variabilityTbeacon/2);
end
fprintf(fileMainID,',');
if simParams.technology == 2 && appParams.variableBeaconSize  % only 11p with variable size
    fprintf(fileMainID,'%.0f(1)-%.0f(%d),',appParams.beaconSizeBytes,appParams.beaconSizeSmallBytes,appParams.NbeaconsSmall);
else
    fprintf(fileMainID,'%.0f,',appParams.beaconSizeBytes);
end
if outputValues.AvgNvehiclesLTE>0
%if simParams.technology~=2 % not only 11p
    fprintf(fileMainID,'%.0f\t',appParams.resourcesV2V);
else
    fprintf(fileMainID,'-\t');
end
if outputValues.AvgNvehiclesLTE>0
%if simParams.technology~=2 % not only 11p
    fprintf(fileMainID,'%.0f,%.1f,%.0fx%.0f,',appParams.Nbeacons,appParams.RBsBeacon/2,phyParams.NsubchannelsBeacon,phyParams.sizeSubchannel);
    if phyParams.ifAdjacent == true
        fprintf(fileMainID,'true\t');
    else
        fprintf(fileMainID,'false\t');
    end
else
    fprintf(fileMainID,'-,-,-,-\t');
end

%4 Phy settings
fprintf(fileMainID,'%.1f,',phyParams.BwMHz);
if outputValues.AvgNvehiclesLTE>0
%if simParams.technology ~= 2 % not only 11p
    fprintf(fileMainID,'%.0f',phyParams.MCS_LTE);
end
if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
%if simParams.technology > 2 % coexistence
    fprintf(fileMainID,'/');
end
if outputValues.AvgNvehicles11p>0
%if simParams.technology ~= 1 % not only LTE
    if ~phyParams.pWithLTEPHY
        fprintf(fileMainID,'%.0f',phyParams.MCS_11p);
    else
        fprintf(fileMainID,'%.0f',phyParams.MCS_pWithLTEphy);
    end
end
fprintf(fileMainID,',');
if outputValues.AvgNvehiclesLTE>0
%if simParams.technology ~= 2 % not only 11p
    fprintf(fileMainID,'%s',phyParams.duplexLTE);
    if strcmp(phyParams.duplexLTE,'FD')
        fprintf(fileMainID,'(Ksi=%.0fdB)',phyParams.Ksi_dB);
    end
else
    fprintf(fileMainID,'-');    
end
fprintf(fileMainID,'\t');

fprintf(fileMainID,'%.0f,',phyParams.Ptx_dBm);
%if simParams.technology ~= 2 % not only 11p
if outputValues.AvgNvehiclesLTE>0
    fprintf(fileMainID,'%.0f',phyParams.P_ERP_MHz_LTE_dBm);
end
%if simParams.technology > 2 % coexistence
if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'/');
end
%if simParams.technology ~= 1 % not only LTE
if outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'%.0f',phyParams.P_ERP_MHz_11p_dBm);
end
fprintf(fileMainID,',%.0f,',phyParams.Pnoise_MHz_dBm);
fprintf(fileMainID,'%.0f,%.0f,',phyParams.Gt_dB,phyParams.Gr_dB);

if phyParams.channelModel>0 %~phyParams.winnerModel
    fprintf(fileMainID,'%.0f,%.3f,%.2f',phyParams.L0_dB,phyParams.beta);
    if simParams.fileObstaclesMap == true
        fprintf(fileMainID,' (Abuild=%.2fdB,Awall=%.2fdB)',phyParams.Abuild_dB,phyParams.Awall_dB);
    end
else
    fprintf(fileMainID,'-,-(Winner+)');
end
fprintf(fileMainID,',');
if phyParams.PERcurves
    fprintf(fileMainID,'%s',phyParams.folderPERcurves);
else
    %if simParams.technology ~= 2 % not only 11p
    if outputValues.AvgNvehiclesLTE>0
        if length(phyParams.sinrThresholdLTE_LOS_dB)==1
            fprintf(fileMainID,'%.2f',phyParams.sinrThresholdLTE_LOS_dB);
        else
            error('Something wrong!');
            %fprintf(fileMainID,'BLERcurve(%s)',phyParams.filenameBLERLTE);
        end
        
        if simParams.typeOfScenario==4 %ETSI-Urban
            if length(phyParams.sinrThresholdLTE_NLOS_dB)==1
                fprintf(fileMainID,'%.2f',phyParams.sinrThresholdLTE_NLOS_dB);
            else
                error('Something wrong!');
                %fprintf(fileMainID,'BLERcurve(%s)',phyParams.filenameBLERLTE);
            end
        end
        
    end
    %if simParams.technology > 2 % coexistence
    if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
        fprintf(fileMainID,'/');
    end
    %if simParams.technology ~= 1 % not only LTE
    if outputValues.AvgNvehicles11p>0
        if length(phyParams.sinrThreshold11p_LOS_dB)==1
            fprintf(fileMainID,'%.2f',phyParams.sinrThreshold11p_LOS_dB);
        else
            error('Something wrong!');
            %fprintf(fileMainID,'BLERcurve(%s)',phyParams.filenameBLER11p);
        end
    end    
end
% Temporary
% if simParams.mco_nVehInterf>0
%     fprintf(fileMainID,'+%dMCO(Tx%ddBm,res%ddB)',10*log10(phyParams.mco_interfERP+30),10*log10(phyParams.mco_resPowerFromAdjacent));
% end
fprintf(fileMainID,'\t');

%if simParams.technology ~= 2 % not only 11p
if outputValues.AvgNvehiclesLTE>0
    fprintf(fileMainID,'%.0f',phyParams.RawMaxLTE);
end
%if simParams.technology > 2 % coexistence
if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'/');
end
%if simParams.technology ~= 1 % not only LTE
if outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'%.0f',phyParams.RawMax11p);
end    
fprintf(fileMainID,',');
%if simParams.technology ~= 2 % not only 11p
if outputValues.AvgNvehiclesLTE>0
    fprintf(fileMainID,'%.0f',phyParams.RawMaxLOSLTE);
end
%if simParams.technology > 2 % coexistence
if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'/');
end
%if simParams.technology ~= 1 % not only LTE
if outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'%.0f',phyParams.RawMaxLOS11p);
end 
fprintf(fileMainID,',');
if phyParams.channelModel==0 %phyParams.winnerModel
    if simParams.technology ~= 2 % not only 11p
        fprintf(fileMainID,'%.0f',phyParams.RawMaxNLOSLTE);
    end
    if simParams.technology > 2 % coexistence
        fprintf(fileMainID,'/');
    end
    if simParams.technology ~= 1 % not only LTE
        fprintf(fileMainID,'%.0f',phyParams.RawMaxNLOS11p);
    end
else
    fprintf(fileMainID,'-');
end
fprintf(fileMainID,',%.0f,%.0f\t',phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);

% Channels
fprintf(fileMainID,'%d',phyParams.nChannels);
if phyParams.nChannels>1
    fprintf(fileMainID,':');
    for iChannel=1:phyParams.nChannels
        if phyParams.propVehiclesChannels(iChannel)==0
            fprintf(fileMainID,'0');
        else
            fprintf(fileMainID,'%.2f',phyParams.propVehiclesChannels(iChannel));
        end
        if iChannel<phyParams.nChannels
            fprintf(fileMainID,',');
        end
    end
end
fprintf(fileMainID,'\t');

% Backoff settings of IEEE 802.11p
if outputValues.AvgNvehicles11p>0
    fprintf(fileMainID,'%d,%d,%ddB,%ddB',phyParams.CW,phyParams.AifsN,phyParams.CCAthr11p_notsync,phyParams.CCAthr11p_sync);
    if phyParams.rilModel11p
        fprintf(fileMainID,',rilM');
    end
    fprintf(fileMainID,'\t');
else
    fprintf(fileMainID,'-\t');
end

%5 Resource allocation algorithm
%if simParams.technology~=2 % not only 11p
if outputValues.AvgNvehiclesLTE>0
    fprintf(fileMainID,'%d ',simParams.BRAlgorithm);
    if simParams.BRAlgorithm==2
        fprintf(fileMainID,'Rreuse=%.0f (margin=%.0f)',phyParams.Rreuse,simParams.Mreuse);
    end
    if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==7 || simParams.BRAlgorithm==9 || simParams.BRAlgorithm==10
        fprintf(fileMainID,',Treassign=%.1f',simParams.Treassign);
    elseif simParams.BRAlgorithm==18
        fprintf(fileMainID,'TsensPer=%.2f,pKeep=%.2f,',simParams.TsensingPeriod,simParams.probResKeep);
        fprintf(fileMainID,'rRes=%.2f,minR=%d,maxR=%d,',simParams.ratioSelectedMode4,simParams.minRandValueMode4,simParams.maxRandValueMode4);
        fprintf(fileMainID,'T1=%d,T2=%d,',simParams.subframeT1Mode4,stationManagement.subframeT2Mode4);
        fprintf(fileMainID,'Pthr=%d,minSCIsinr=%.2f',10*log10(simParams.powerThresholdMode4)+30,10*log10(phyParams.minSCIsinr));
    end
    if simParams.BRAlgorithm==10
        if simParams.knownShadowing
            fprintf(fileMainID,',knownShadowing');
        else
            fprintf(fileMainID,',NOTknownShadowing');
        end
    end
    fprintf(fileMainID,'\t');
else
    fprintf(fileMainID,'-\t');
end

%6 Awareness range
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%.0f',phyParams.Raw(iPhyRaw));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\t');

%7 Outputs LTE
fprintf(fileMainID,'%.1f\t',outputValues.AvgNvehiclesLTE);
%if simParams.technology~=2 % not only 11p
if outputValues.AvgNvehiclesLTE>0
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.NneighborsLTE(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.StDevNeighboursLTE(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    fprintf(fileMainID,'%f\t',outputValues.NreassignLTE);
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%d',sum(sum(outputValues.NtxBeaconsLTE(:,:,iPhyRaw))));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    % Average tx per node per second
    avTx = sum(sum(outputValues.NtxBeaconsLTE(:,:,1))) / ...
        (outputValues.AvgNvehiclesLTE * outputValues.NneighborsLTE(1) * simParams.simulationTime);
    fprintf(fileMainID,'%.2f\t',avTx);
    if simParams.cbrActive && sum(stationManagement.vehicleState==100)>0
        if length(stationManagement.cbrLteValues(1,:))<11
            fprintf(fileMainID,'short_sim');
        else
            for iChannel=1:phyParams.nChannels
                cbrVector = stationManagement.cbrLteValues(stationManagement.vehicleChannel==iChannel,11:end);
                cbrVector = reshape(cbrVector,[],1);
                cbrVector = cbrVector(cbrVector>=0);
                if median(cbrVector)>0
                    fprintf(fileMainID,'%.3f',median(cbrVector));
                    if stationManagement.dccLteTriggered(iChannel)
                        fprintf(fileMainID,'*');
                    end
                else
                    fprintf(fileMainID,'0');
                end
                if iChannel<phyParams.nChannels
                    fprintf(fileMainID,',');
                end
            end
        end
        fprintf(fileMainID,'\t');
    else
        fprintf(fileMainID,'-\t');
    end    
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.blockingRateLTE(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.errorRateLTE(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.packetReceptionRatioLTE(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
else
    fprintf(fileMainID,'-\t-\t-\t-\t-\t-\t-\t-\t-\t');
end

%8 Outputs 11p
fprintf(fileMainID,'%.1f\t',outputValues.AvgNvehicles11p);
%if simParams.technology~=1 % not only LTE
if outputValues.AvgNvehicles11p>0
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.Nneighbors11p(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.StDevNeighbours11p(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%d',sum(sum(outputValues.NtxBeacons11p(:,:,iPhyRaw))));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    % Average tx per node per second
    avTx = sum(sum(outputValues.NtxBeacons11p(:,:,1))) / ...
        (outputValues.AvgNvehicles11p * outputValues.Nneighbors11p(1) * simParams.simulationTime);
    fprintf(fileMainID,'%.2f\t',avTx);    
    if simParams.cbrActive
        if length(stationManagement.cbr11pValues(1,:))<11
            fprintf(fileMainID,'short_sim');
        else
            for iChannel=1:phyParams.nChannels            
                cbrVector = stationManagement.cbr11pValues(stationManagement.vehicleChannel==iChannel,11:end);
                cbrVector = reshape(cbrVector,[],1);
                cbrVector = cbrVector(cbrVector>=0);
                if median(cbrVector)>0
                    fprintf(fileMainID,'%.3f',median(cbrVector));
                    if stationManagement.dcc11pTriggered(iChannel)
                        fprintf(fileMainID,'*');
                    end
                else
                    fprintf(fileMainID,'0');
                end
                if iChannel<phyParams.nChannels
                    fprintf(fileMainID,',');
                end
            end
        end
        fprintf(fileMainID,'\t');
    else
        fprintf(fileMainID,'-\t');
    end    
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.blockingRate11p(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.errorRate11p(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t');
    for iPhyRaw=1:length(phyParams.Raw)
        fprintf(fileMainID,'%f',outputValues.packetReceptionRatio11p(iPhyRaw));
        if iPhyRaw<length(phyParams.Raw)
            fprintf(fileMainID,',');
        end
    end
    fprintf(fileMainID,'\t'); 
else
    fprintf(fileMainID,'-\t-\t-\t-\t-\t-\t-\t-\t');
end

%9 Outputs TOT
fprintf(fileMainID,'%.1f\t',outputValues.AvgNvehiclesTOT);
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%f',outputValues.NneighborsTOT(iPhyRaw));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\t');
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%f',outputValues.StDevNeighboursTOT(iPhyRaw));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\t');
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%d',sum(sum(outputValues.NtxBeaconsTOT(:,:,iPhyRaw))));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\t');
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%f',outputValues.blockingRateTOT(iPhyRaw));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\t');
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%f',outputValues.errorRateTOT(iPhyRaw));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\t');
for iPhyRaw=1:length(phyParams.Raw)
    fprintf(fileMainID,'%f',outputValues.packetReceptionRatioTOT(iPhyRaw));
    if iPhyRaw<length(phyParams.Raw)
        fprintf(fileMainID,',');
    end
end
fprintf(fileMainID,'\n');