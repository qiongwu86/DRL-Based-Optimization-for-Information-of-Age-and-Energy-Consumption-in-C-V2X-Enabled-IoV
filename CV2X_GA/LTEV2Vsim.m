function LTEV2Vsim(varargin)
% The function LTEV2Vsim() is the main function of the simulator

% ==============
% Copyright (C) Alessandro Bazzi, University of Bologna, and Alberto Zanella, CNR
% 
% All rights reserved.
% 
% Permission to use, copy, modify, and distribute this software for any 
% purpose without fee is hereby granted, provided that this entire notice 
% is included in all copies of any software which is or includes a copy or 
% modification of this software and in all copies of the supporting 
% documentation for such software.
% 
% THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED 
% WARRANTY. IN PARTICULAR, NEITHER OF THE AUTHORS MAKES ANY REPRESENTATION 
% OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF THIS SOFTWARE 
% OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
% 
% Project: LTEV2Vsim
% ==============

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%               LTEV2Vsim              %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call it as
% LTEV2Vsim(fileCfg,paramName1,value1,...paramNameN,valueN)
%
% Parameters are optional.
% If one or more parameters are given in input, the first corresponds to the
% config file (a text file). Use 'default' or '0' to set the default config
% file (i.e., LTEV2Vsim.cfg). If a file that does not exist is set, the
% simulation continues without considering the settings from the config
% file.
% In the config file, write couples with i) the parameter name within squared
% brackets and ii) the value of the parameter.
%
% In the command line, couples of parameters follow the config file. Each couple
% must include i) the parameter name and ii) the value.
%
% All parameters can be set in the config file and/or in the command
% line; the priority is: 1) command line; 2) config file; 3) default value.
%
% Example call:
% LTEV2Vsim('default','seed',0,'MCS_LTE',2);
% In this example, the seed for random numbers is randomly selected and the
% MCS 2 is set. Then the other parameters take the value from the default
% config file if the file is present and the parameter is set; otherwise
% the default value is used.
%
% Write
% LTEV2Vsim('help')
% for a full list of the parameters with their default values.

%% Initialization

% Version of the simulator
simVersion = 'v5.4';
fprintf('LTEV2Vsim %s\n\n',simVersion);

% The path of the directory of the simulator is saved in 'fullPath'
fullPath = which('LTEV2Vsim');
[~,fullPathUnix] = strtok(fliplr(fullPath), '/');
[~,fullPathWindows] = strtok(fliplr(fullPath), '\');
if isempty(fullPathUnix)
    fullPath = fliplr(fullPathWindows);
else
    fullPath = fliplr(fullPathUnix);
end
addpath(genpath(fullPath));
chdir(fullPath);

% 'help' feature:
% "LTEV2Vsim('help')" allows to print the full list of parameters
% with default values
if nargin == 1 && strcmp(varargin{1},'help')
    fprintf('Help: list of the parameters with default values\n\n');
    initiateParameters({'help'});
    fprintf('End of the list.\n');
    return
end

% Simulator parameters and initial settings
[simParams,appParams,phyParams,outParams] = initiateParameters(varargin);

% Update PHY structure with the ranges
[phyParams] = deriveRanges(phyParams,simParams);

% Simulator output inizialization
outputValues = struct('computationTime',-1,...
    'blockingRateLTE',-1*ones(1,length(phyParams.Raw)),'blockingRate11p',-1*ones(1,length(phyParams.Raw)),'blockingRateTOT',-1*ones(1,length(phyParams.Raw)),...
    'errorRateLTE',-1*ones(1,length(phyParams.Raw)),'errorRate11p',-1*ones(1,length(phyParams.Raw)),'errorRateTOT',-1*ones(1,length(phyParams.Raw)),...
    'packetReceptionRatioLTE',-1*ones(1,length(phyParams.Raw)),'packetReceptionRatio11p',-1*ones(1,length(phyParams.Raw)),'packetReceptionRatioTOT',-1*ones(1,length(phyParams.Raw)),...
    'NvehiclesLTE',0,'Nvehicles11p',0,'NvehiclesTOT',0,...
    'NneighborsLTE',zeros(1,length(phyParams.Raw)),'Nneighbors11p',zeros(1,length(phyParams.Raw)),'NneighborsTOT',zeros(1,length(phyParams.Raw)),...
    'StDevNeighboursLTE',zeros(1,length(phyParams.Raw)),'StDevNeighbours11p',zeros(1,length(phyParams.Raw)),'StDevNeighboursTOT',zeros(1,length(phyParams.Raw)),...
    'NreassignLTE',0,...%%%%%
    'NblockedLTE',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'Nblocked11p',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'NblockedTOT',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),...
    'NtxBeaconsLTE',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'NtxBeacons11p',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'NtxBeaconsTOT',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),...
    'NerrorsLTE',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'Nerrors11p',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'NerrorsTOT',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),...    
    'NcorrectlyTxBeaconsLTE',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'NcorrectlyTxBeacons11p',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)),'NcorrectlyTxBeaconsTOT',zeros(phyParams.nChannels,appParams.nPckTypes,length(phyParams.Raw)));
%     'NblockedLTE',zeros(appParams.nPckTypes,length(phyParams.Raw)),'Nblocked11p',zeros(appParams.nPckTypes,length(phyParams.Raw)),'NblockedTOT',zeros(appParams.nPckTypes,length(phyParams.Raw)),...
%     'NtxBeaconsLTE',zeros(appParams.nPckTypes,length(phyParams.Raw)),'NtxBeacons11p',zeros(appParams.nPckTypes,length(phyParams.Raw)),'NtxBeaconsTOT',zeros(appParams.nPckTypes,length(phyParams.Raw)),...
%     'NerrorsLTE',zeros(appParams.nPckTypes,length(phyParams.Raw)),'Nerrors11p',zeros(appParams.nPckTypes,length(phyParams.Raw)),'NerrorsTOT',zeros(appParams.nPckTypes,length(phyParams.Raw)),...    
%     'NcorrectlyTxBeaconsLTE',zeros(appParams.nPckTypes,length(phyParams.Raw)),'NcorrectlyTxBeacons11p',zeros(appParams.nPckTypes,length(phyParams.Raw)),'NcorrectlyTxBeaconsTOT',zeros(appParams.nPckTypes,length(phyParams.Raw)));
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Scenario Description

% Load scenario from Trace File or generate initial positions of vehicles
[simParams,simValues,positionManagement,appParams] = initVehiclePositions(simParams,appParams);

% Load obstacles scenario from Obstacles Map File (if selected)
if simParams.typeOfScenario==2 && simParams.fileObstaclesMap % Only with traffic traces
    [simParams,positionManagement] = loadObstaclesMapFile(simParams,positionManagement);
else
    [positionManagement.XminMap,positionManagement.YmaxMap,positionManagement.StepMap,positionManagement.GridMap] = deal(-1);
end

% Initialization of matrices correctlyReceivedMap and neighborsMap (for PRRmap)
if simParams.typeOfScenario==2 && outParams.printPRRmap % Only traffic traces
    simValues.correctlyReceivedMap11p = zeros(size(positionManagement.GridMap));
    simValues.neighborsMap11p = zeros(size(positionManagement.GridMap));
    simValues.correctlyReceivedMapLTE = zeros(size(positionManagement.GridMap));
    simValues.neighborsMapLTE = zeros(size(positionManagement.GridMap));
end

if outParams.printUpdateDelay
    % Initialize matrix containing update time of the received beacons
    simValues.updateTimeMatrix11p = -1*ones(simValues.maxID,simValues.maxID,length(phyParams.Raw));
    simValues.updateTimeMatrixLTE = -1*ones(simValues.maxID,simValues.maxID,length(phyParams.Raw));
    
    % Initialize array with the counters of update delay events
    % (max 10 s + delayResolution -> delays larger than 10 s are
    % registered in the last element of the array)
    NupdateDelayEvents = round(10/outParams.delayResolution)+1;
%     outputValues.updateDelayCounter11p = zeros(appParams.nPckTypes,NupdateDelayEvents,length(phyParams.Raw));
%     outputValues.updateDelayCounterLTE = zeros(appParams.nPckTypes,NupdateDelayEvents,length(phyParams.Raw));
    outputValues.updateDelayCounter11p = zeros(phyParams.nChannels,appParams.nPckTypes,NupdateDelayEvents,length(phyParams.Raw));
    outputValues.updateDelayCounterLTE = zeros(phyParams.nChannels,appParams.nPckTypes,NupdateDelayEvents,length(phyParams.Raw));
    
%     if outParams.printWirelessBlindSpotProb
%         % Initialize matrix containing the counters needed for computation
%         % of wireless blind spot probability
%         % [Time interval - # delay events larger or equal than time interval - #
%         % delay events shorter than time interval - wireless blind spot probability]
%         delayValues = appParams.averageTbeaco:appParams.averageTbeaco:outParams.delayMax;
%         outputValues.wirelessBlindSpotCounter = zeros(length(delayValues),4);
%         outputValues.wirelessBlindSpotCounter(:,1) = delayValues;
%     end
end

if outParams.printDataAge
    % Initialize matrix containing update time of the received beacons
    simValues.dataAgeTimestampMatrix11p = -1*ones(simValues.maxID,simValues.maxID,length(phyParams.Raw));
    simValues.dataAgeTimestampMatrixLTE = -1*ones(simValues.maxID,simValues.maxID,length(phyParams.Raw));
    
    % Initialize array with the counters of update delay events
    % (max 10 s + delayResolution -> delays larger than 10 s are
    % registered in the last element of the array)
    NdataAgeEvents = round(10/outParams.delayResolution)+1;
%     outputValues.dataAgeCounter11p = zeros(appParams.nPckTypes,NdataAgeEvents,length(phyParams.Raw));
%     outputValues.dataAgeCounterLTE = zeros(appParams.nPckTypes,NdataAgeEvents,length(phyParams.Raw));
    outputValues.dataAgeCounter11p = zeros(phyParams.nChannels,appParams.nPckTypes,NdataAgeEvents,length(phyParams.Raw));
    outputValues.dataAgeCounterLTE = zeros(phyParams.nChannels,appParams.nPckTypes,NdataAgeEvents,length(phyParams.Raw));
end

if outParams.printPacketDelay
    % Initialize array with the counters of packet delay events
    % (max Tbeacon/delayResolution -> delays larger than Tbeaco are
    % registered in the last element of the array)       max Tbeaco == 0.1
%     NpacketDelayEvents = round((2*appParams.averageTbeaco)./outParams.delayResolution);
    NpacketDelayEvents = round((2*0.1)./outParams.delayResolution);
%     outputValues.packetDelayCounter11p = zeros(appParams.nPckTypes,NpacketDelayEvents,length(phyParams.Raw));
%     outputValues.packetDelayCounterLTE = zeros(appParams.nPckTypes,NpacketDelayEvents,length(phyParams.Raw));
    outputValues.packetDelayCounter11p = zeros(phyParams.nChannels,appParams.nPckTypes,NpacketDelayEvents,length(phyParams.Raw));
    outputValues.packetDelayCounterLTE = zeros(phyParams.nChannels,appParams.nPckTypes,NpacketDelayEvents,length(phyParams.Raw));
end

if outParams.printPacketReceptionRatio
    % If simulating variable beacon size (currently 802.11p only)
    if simParams.technology~=1 % not only LTE 
        if simParams.technology==2 && appParams.variableBeaconSize
            % Initialize 9 columns in distanceDetailsCounter (for smaller beacons)
            % The matrix becomes:
            % [distance, #Correctly decoded beacons (big), #Errors (big), #Blocked neighbors (big), #Neighbors (big),
            % #Correctly decoded beacons (small), #Errors (small), #Blocked neighbors (small), #Neighbors (small)]
%            outputValues.distanceDetailsCounter11p = zeros(appParams.nPckTypes,floor(phyParams.RawMax11p/outParams.prrResolution),9);
            outputValues.distanceDetailsCounter11p = zeros(phyParams.nChannels,appParams.nPckTypes,floor(phyParams.RawMax11p/outParams.prrResolution),9);
        else
%            outputValues.distanceDetailsCounter11p = zeros(appParams.nPckTypes,floor(phyParams.RawMax11p/outParams.prrResolution),5);
            outputValues.distanceDetailsCounter11p = zeros(phyParams.nChannels,appParams.nPckTypes,floor(phyParams.RawMax11p/outParams.prrResolution),5);
        end
        for iChannel = 1:phyParams.nChannels
            for pckType=1:appParams.nPckTypes
                outputValues.distanceDetailsCounter11p(iChannel,pckType,:,1) = (outParams.prrResolution:outParams.prrResolution:floor(phyParams.RawMax11p))';
            end
        end
    end
    
    if simParams.technology~=2 % not only 11p
        % Initialize array with the counters of Rx details vs. distance (up to RawMax)
        % [distance, #Correctly decoded beacons, #Errors, #Blocked neighbors, #Neighbors (computed in printDistanceDetailsCounter)]
        
%        outputValues.distanceDetailsCounterLTE = zeros(appParams.nPckTypes,floor(phyParams.RawMaxLTE/outParams.prrResolution),5);
        outputValues.distanceDetailsCounterLTE = zeros(phyParams.nChannels,appParams.nPckTypes,floor(phyParams.RawMaxLTE/outParams.prrResolution),5);
        for iChannel = 1:phyParams.nChannels
            for pckType=1:appParams.nPckTypes
%            outputValues.distanceDetailsCounterLTE(pckType,:,1) = (outParams.prrResolution:outParams.prrResolution:floor(phyParams.RawMaxLTE))';
                outputValues.distanceDetailsCounterLTE(iChannel,pckType,:,1) = (outParams.prrResolution:outParams.prrResolution:floor(phyParams.RawMaxLTE))';
            end
        end
    end
end

if outParams.printPowerControl
    %
    error('Power control output not updated in v5');
    % NOTE: needs check regarding the new power per MHz parameter
%     % Initialize array with the counters of power control events
%     % (max Ptx/powerResolution + 10dBm margin -> TX power higher than
%     % PtxMax + 10 dBm are registered in the last element of the array)
%     % (min -100 dBm -> TX power lower than -100dBm are registered in the
%     % first element of the array)
%     NpowerControlEvents = round(101/outParams.powerResolution) + round(phyParams.P_ERP_MHz_dBm/outParams.powerResolution);
%     outputValues.powerControlCounter = zeros(NpowerControlEvents,1);
end

if outParams.printHiddenNodeProb
    % TODO - not updated
    error('not supported in v5');
    % Initialize arrays for hidden node probability
    %outputValues.hiddenNodeSumProb = zeros(floor(phyParams.RawMax)+1,1);
    %outputValues.hiddenNodeProbEvents = zeros(floor(phyParams.RawMax)+1,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation
[simValues,outputValues,appParams,simParams,phyParams,sinrManagement,outParams,stationManagement] = mainV2X(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement);    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% KPIs Computation (Output)
fprintf('\nElaborating the outputs...\n');

% First of all convert from cumulative to groups
for pckType = 1:appParams.nPckTypes
    for iPhyRaw=length(phyParams.Raw):-1:2
        for idChannel = 1:phyParams.nChannels
%            outputValues.NblockedLTE(pckType,iPhyRaw) = outputValues.NblockedLTE(pckType,iPhyRaw)-outputValues.NblockedLTE(pckType,iPhyRaw-1);
%             outputValues.NcorrectlyTxBeaconsLTE(pckType,iPhyRaw) = outputValues.NcorrectlyTxBeaconsLTE(pckType,iPhyRaw)-outputValues.NcorrectlyTxBeaconsLTE(pckType,iPhyRaw-1);
%             outputValues.NerrorsLTE(pckType,iPhyRaw) = outputValues.NerrorsLTE(pckType,iPhyRaw)-outputValues.NerrorsLTE(pckType,iPhyRaw-1);
%             outputValues.NtxBeaconsLTE(pckType,iPhyRaw) = outputValues.NtxBeaconsLTE(pckType,iPhyRaw)-outputValues.NtxBeaconsLTE(pckType,iPhyRaw-1);
%             outputValues.Nblocked11p(pckType,iPhyRaw) = outputValues.Nblocked11p(pckType,iPhyRaw)-outputValues.Nblocked11p(pckType,iPhyRaw-1);
%             outputValues.NcorrectlyTxBeacons11p(pckType,iPhyRaw) = outputValues.NcorrectlyTxBeacons11p(pckType,iPhyRaw)-outputValues.NcorrectlyTxBeacons11p(pckType,iPhyRaw-1);
%             outputValues.Nerrors11p(pckType,iPhyRaw) = outputValues.Nerrors11p(pckType,iPhyRaw)-outputValues.Nerrors11p(pckType,iPhyRaw-1);
%             outputValues.NtxBeacons11p(pckType,iPhyRaw) = outputValues.NtxBeacons11p(pckType,iPhyRaw)-outputValues.NtxBeacons11p(pckType,iPhyRaw-1);
%             outputValues.NblockedTOT(pckType,iPhyRaw) = outputValues.NblockedTOT(pckType,iPhyRaw)-outputValues.NblockedTOT(pckType,iPhyRaw-1);
%             outputValues.NcorrectlyTxBeaconsTOT(pckType,iPhyRaw) = outputValues.NcorrectlyTxBeaconsTOT(pckType,iPhyRaw)-outputValues.NcorrectlyTxBeaconsTOT(pckType,iPhyRaw-1);
%             outputValues.NerrorsTOT(pckType,iPhyRaw) = outputValues.NerrorsTOT(pckType,iPhyRaw)-outputValues.NerrorsTOT(pckType,iPhyRaw-1);
%             outputValues.NtxBeaconsTOT(pckType,iPhyRaw) = outputValues.NtxBeaconsTOT(pckType,iPhyRaw)-outputValues.NtxBeaconsTOT(pckType,iPhyRaw-1);
            outputValues.NblockedLTE(idChannel,pckType,iPhyRaw) = outputValues.NblockedLTE(idChannel,pckType,iPhyRaw)-outputValues.NblockedLTE(idChannel,pckType,iPhyRaw-1);
            outputValues.NcorrectlyTxBeaconsLTE(idChannel,pckType,iPhyRaw) = outputValues.NcorrectlyTxBeaconsLTE(idChannel,pckType,iPhyRaw)-outputValues.NcorrectlyTxBeaconsLTE(idChannel,pckType,iPhyRaw-1);
            outputValues.NerrorsLTE(idChannel,pckType,iPhyRaw) = outputValues.NerrorsLTE(idChannel,pckType,iPhyRaw)-outputValues.NerrorsLTE(idChannel,pckType,iPhyRaw-1);
            outputValues.NtxBeaconsLTE(idChannel,pckType,iPhyRaw) = outputValues.NtxBeaconsLTE(idChannel,pckType,iPhyRaw)-outputValues.NtxBeaconsLTE(idChannel,pckType,iPhyRaw-1);
            outputValues.Nblocked11p(idChannel,pckType,iPhyRaw) = outputValues.Nblocked11p(idChannel,pckType,iPhyRaw)-outputValues.Nblocked11p(idChannel,pckType,iPhyRaw-1);
            outputValues.NcorrectlyTxBeacons11p(idChannel,pckType,iPhyRaw) = outputValues.NcorrectlyTxBeacons11p(idChannel,pckType,iPhyRaw)-outputValues.NcorrectlyTxBeacons11p(idChannel,pckType,iPhyRaw-1);
            outputValues.Nerrors11p(idChannel,pckType,iPhyRaw) = outputValues.Nerrors11p(idChannel,pckType,iPhyRaw)-outputValues.Nerrors11p(idChannel,pckType,iPhyRaw-1);
            outputValues.NtxBeacons11p(idChannel,pckType,iPhyRaw) = outputValues.NtxBeacons11p(idChannel,pckType,iPhyRaw)-outputValues.NtxBeacons11p(idChannel,pckType,iPhyRaw-1);
            outputValues.NblockedTOT(idChannel,pckType,iPhyRaw) = outputValues.NblockedTOT(idChannel,pckType,iPhyRaw)-outputValues.NblockedTOT(idChannel,pckType,iPhyRaw-1);
            outputValues.NcorrectlyTxBeaconsTOT(idChannel,pckType,iPhyRaw) = outputValues.NcorrectlyTxBeaconsTOT(idChannel,pckType,iPhyRaw)-outputValues.NcorrectlyTxBeaconsTOT(idChannel,pckType,iPhyRaw-1);
            outputValues.NerrorsTOT(idChannel,pckType,iPhyRaw) = outputValues.NerrorsTOT(idChannel,pckType,iPhyRaw)-outputValues.NerrorsTOT(idChannel,pckType,iPhyRaw-1);
            outputValues.NtxBeaconsTOT(idChannel,pckType,iPhyRaw) = outputValues.NtxBeaconsTOT(idChannel,pckType,iPhyRaw)-outputValues.NtxBeaconsTOT(idChannel,pckType,iPhyRaw-1);
        end
    end
end

% Temporary
% OUTPUT FOR MCO
% if simParams.mco_nVehInterf>0 && outParams.mco_printInterfStatistic
%     mco_printOutput(stationManagement,simParams,outParams,outputValues);
% end

% Average Blocking Rate
% outputValues.blockingRateLTE = sum(outputValues.NblockedLTE,1) ./ (sum(outputValues.NcorrectlyTxBeaconsLTE+outputValues.NerrorsLTE+outputValues.NblockedLTE,1));
% outputValues.blockingRate11p = sum(outputValues.Nblocked11p,1) ./ (sum(outputValues.NcorrectlyTxBeacons11p+outputValues.Nerrors11p+outputValues.Nblocked11p,1));
% outputValues.blockingRateTOT = sum(outputValues.NblockedTOT,1) ./ (sum(outputValues.NcorrectlyTxBeaconsTOT+outputValues.NerrorsTOT+outputValues.NblockedTOT,1));
outputValues.blockingRateLTE = sum(sum(outputValues.NblockedLTE,1),1) ./ (sum(sum(outputValues.NcorrectlyTxBeaconsLTE+outputValues.NerrorsLTE+outputValues.NblockedLTE,1),1));
outputValues.blockingRate11p = sum(sum(outputValues.Nblocked11p,1),1) ./ (sum(sum(outputValues.NcorrectlyTxBeacons11p+outputValues.Nerrors11p+outputValues.Nblocked11p,1),1));
outputValues.blockingRateTOT = sum(sum(outputValues.NblockedTOT,1),1) ./ (sum(sum(outputValues.NcorrectlyTxBeaconsTOT+outputValues.NerrorsTOT+outputValues.NblockedTOT,1),1));

% Average Error Rate
% outputValues.errorRateLTE = sum(outputValues.NerrorsLTE,1) ./ sum(outputValues.NtxBeaconsLTE,1);
% outputValues.errorRate11p = sum(outputValues.Nerrors11p,1) ./ sum(outputValues.NtxBeacons11p,1);
% outputValues.errorRateTOT = sum(outputValues.NerrorsTOT,1) ./ sum(outputValues.NtxBeaconsTOT,1);
outputValues.errorRateLTE = sum(sum(outputValues.NerrorsLTE,1),1) ./ sum(sum(outputValues.NtxBeaconsLTE,1),1);
outputValues.errorRate11p = sum(sum(outputValues.Nerrors11p,1),1) ./ sum(sum(outputValues.NtxBeacons11p,1),1);
outputValues.errorRateTOT = sum(sum(outputValues.NerrorsTOT,1),1) ./ sum(sum(outputValues.NtxBeaconsTOT,1),1);

% Average Packet Reception Ratio
% outputValues.packetReceptionRatioLTE = sum(outputValues.NcorrectlyTxBeaconsLTE,1) ./ sum(outputValues.NtxBeaconsLTE,1);
% outputValues.packetReceptionRatio11p = sum(outputValues.NcorrectlyTxBeacons11p,1) ./ sum(outputValues.NtxBeacons11p,1);
% outputValues.packetReceptionRatioTOT = sum(outputValues.NcorrectlyTxBeaconsTOT,1) ./ sum(outputValues.NtxBeaconsTOT,1);
outputValues.packetReceptionRatioLTE = sum(sum(outputValues.NcorrectlyTxBeaconsLTE,1),1) ./ sum(sum(outputValues.NtxBeaconsLTE,1),1);
outputValues.packetReceptionRatio11p = sum(sum(outputValues.NcorrectlyTxBeacons11p,1),1) ./ sum(sum(outputValues.NtxBeacons11p,1),1);
outputValues.packetReceptionRatioTOT = sum(sum(outputValues.NcorrectlyTxBeaconsTOT,1),1) ./ sum(sum(outputValues.NtxBeaconsTOT,1),1);

% Average number of neighbors per vehicle
outputValues.NneighborsLTE = outputValues.NneighborsLTE ./ outputValues.NvehiclesLTE;
outputValues.Nneighbors11p = outputValues.Nneighbors11p ./ outputValues.Nvehicles11p;
outputValues.NneighborsTOT = outputValues.NneighborsTOT ./ outputValues.NvehiclesTOT;
outputValues.StDevNeighboursLTE = outputValues.StDevNeighboursLTE / simValues.snapshots;
outputValues.StDevNeighbours11p = outputValues.StDevNeighbours11p / simValues.snapshots;
outputValues.StDevNeighboursTOT = outputValues.StDevNeighboursTOT / simValues.snapshots;

% Average number of vehicles in the scenario
outputValues.AvgNvehiclesLTE = outputValues.NvehiclesLTE / simValues.snapshots;
outputValues.AvgNvehicles11p = outputValues.Nvehicles11p / simValues.snapshots;
outputValues.AvgNvehiclesTOT = outputValues.NvehiclesTOT / simValues.snapshots;

% Average number of successful BR reassignment per vehicle per second
if outputValues.AvgNvehiclesLTE>0
    outputValues.NreassignLTE = (outputValues.NreassignLTE ./ outputValues.AvgNvehiclesLTE) / simParams.simulationTime;
else
    outputValues.NreassignLTE = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Print To Files

% Print to file of the CBR statistics
if outParams.printCBR
    if sum(stationManagement.vehicleState~=100)>0
        printCBRToFileITSG5(stationManagement,simParams,outParams,phyParams);
    end
    if sum(stationManagement.vehicleState==100)>0
        printCBRToFileLTE(stationManagement,simParams,outParams,phyParams);
    end
end

% Print update delay to file (if enabled)
if outParams.printUpdateDelay || outParams.printDataAge || outParams.printPacketDelay
    printDelay(stationManagement,outputValues,appParams,outParams,phyParams);
end

% Print details for distances up to the maximum awareness range (if enabled)
if outParams.printPacketReceptionRatio
    if sum(stationManagement.vehicleState==100)>0
    %if simParams.technology~=2 % LTE or coexistence, not 11p
        printPacketReceptionRatio('LTE',outputValues.distanceDetailsCounterLTE,outParams,appParams,simParams,phyParams);
    end
    if sum(stationManagement.vehicleState~=100)>0
    %if simParams.technology~=1 % 11p or coexistence, not LTE
        printPacketReceptionRatio('11p',outputValues.distanceDetailsCounter11p,outParams,appParams,simParams,phyParams);
    end
end

% Print PRRmap to file (if enabled)
if simParams.typeOfScenario==2 && outParams.printPRRmap && simParams.fileObstaclesMap
    printPRRmapToFile(simValues,simParams,outParams,positionManagement);
end

% Print power control allocation to file (if enabled)
if outParams.printPowerControl
    printPowerControl(outputValues,outParams);
end

% Print hidden node probability to file (if enabled)
if outParams.printHiddenNodeProb
    printHiddenNodeProb(outputValues,outParams);
end

% Print to XLS file
outputToFiles(simVersion,stationManagement,simParams,appParams,phyParams,sinrManagement,outParams,outputValues);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Print To Video
fprintf('\nAverage number of vehicles in the scenario = %.0f\n',outputValues.AvgNvehiclesTOT);
if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
    fprintf('Average %.0f LTE-V2X, ',outputValues.AvgNvehiclesLTE);        
    fprintf('average %.0f IEEE 802.11p\n',outputValues.AvgNvehicles11p);
end
for iPhyRaw=1:length(phyParams.Raw)
    if iPhyRaw==1
        fprintf('*** In the range 0-%d:\n',phyParams.Raw(iPhyRaw));
    else
        fprintf('*** In the range %d-%d:\n',phyParams.Raw(iPhyRaw-1),phyParams.Raw(iPhyRaw));
    end
    if outputValues.AvgNvehiclesLTE>0 && outputValues.AvgNvehicles11p>0
        fprintf('LTE: average neigbors %.2f +- %.2f, ',outputValues.NneighborsLTE(iPhyRaw),outputValues.StDevNeighboursLTE(iPhyRaw));
        fprintf('Blocking = %.5f\tError = %.5f\tCorrect = %.5f\n',outputValues.blockingRateLTE(iPhyRaw),outputValues.errorRateLTE(iPhyRaw),outputValues.packetReceptionRatioLTE(iPhyRaw));
%         fprintf('11p: average neighbors %.2f +- %.2f, ',outputValues.Nneighbors11p(iPhyRaw),outputValues.StDevNeighbours11p(iPhyRaw));
%         fprintf('Blocking = %.5f\tError = %.5f\tCorrect = %.5f\n',outputValues.blockingRate11p(iPhyRaw),outputValues.errorRate11p(iPhyRaw),outputValues.packetReceptionRatio11p(iPhyRaw));
    else
        fprintf('Average neighbors %.2f +- %.2f\n',outputValues.NneighborsTOT(iPhyRaw),outputValues.StDevNeighboursTOT(iPhyRaw));
        fprintf('Blocking = %.5f\tError = %.5f\tCorrect = %.5f\n',outputValues.blockingRateTOT(iPhyRaw),outputValues.errorRateTOT(iPhyRaw),outputValues.packetReceptionRatioTOT(iPhyRaw));
    end    
end

fclose('all');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end