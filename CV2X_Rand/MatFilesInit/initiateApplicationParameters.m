function [appParams,simParams,varargin] = initiateApplicationParameters(simParams,fileCfg,varargin)
% function [appParams,simParams,varargin]= initiateApplicationParameters(fileCfg,varargin)
%
% Settings of the application
% It takes in input the name of the (possible) file config and the inputs
% of the main function
% It returns the structure "appParams"

fprintf('Application settings\n');

% [averageTbeacon]
% Beacon period in seconds.
[appParams,varargin] = addNewParam([],'averageTbeacon',0.1,'Beacon period (s)','double',fileCfg,varargin{1});
if appParams.averageTbeacon<=0
    error('Error: "appParams.averageTbeacon" cannot be <= 0');
end

% [variabilityTbeacon]
% Interval of variability of Tbeacon from vehicle to vehicle
% Each (11p) vehicle will have a periodicity uniformly randomly chosen
% between "averageTbeacon-variabilityTbeacon/2" and "averageTbeacon+variabilityTbeacon/2"
% Note: it applies only to 11p nodes, in order to reproduce possible small variations in the
% speed of vehicles that cause small variation in periodicity
% In LTE this cannot apply, as in LTE the beacon interval is rigid and
% small variability to speed does not make the periodicity to vary
% ------11p中的指标------
[appParams,varargin]= addNewParam(appParams,'variabilityTbeacon',0,'Variability of beacon period per vehicle (s) (only 11p)','double',fileCfg,varargin{1});
if appParams.variabilityTbeacon~=-1 && (appParams.variabilityTbeacon<0 || appParams.variabilityTbeacon>=appParams.averageTbeacon)
    error('Error: "appParams.variabilityTbeacon" cannot be < 0 or >= "appParams.averageTbeacon" (except -1, automatic)');
end
if appParams.variabilityTbeacon==-1 % automatic
    % [camDiscretizationType]
    % Type of discretization: "allSteps" or "allocationAligned" [string]
    [appParams,varargin] = addNewParam(appParams,'camDiscretizationType','null','Type of discretization - it can be "allSteps" or "allocationAligned" if not "null" (continuous)','string',fileCfg,varargin{1});
    if ~strcmp(appParams.camDiscretizationType,'null') && ~strcmp(appParams.camDiscretizationType,'allSteps') && ~strcmp(appParams.camDiscretizationType,'allocationAligned') 
        error('Error in the setting of "appParams.camDiscretizationType".');
    end
    
    if ~strcmp(appParams.camDiscretizationType,'null')

        % [camDiscretizationIncrease]
        % Percentage of the admissibile increase of the generation interval [%, double]
        [appParams,varargin]= addNewParam(appParams,'camDiscretizationIncrease',20,'Percentage of the admissibile increase of the generation interval','double',fileCfg,varargin{1});
        if appParams.camDiscretizationIncrease<0 || appParams.camDiscretizationIncrease>100
            error('Error: appParams.camDiscretizationIncrease=%.1f! Cannot be < 0 or >100',appParams.camDiscretizationIncrease);
        end
    end
end    


% The beacon periodicity fB is derived - never used
%appParams.fB = 1/appParams.averageTbeaco;

% % Removed in version 5.2.10
% if simParams.typeOfScenario==2 % traffic trace
%     % if default value of time resolution is selected, update the value to the beacon period
%     if simParams.positionTimeResolution==-1
%         simParams.positionTimeResolution = appParams.averageTbeaco;
%     end
% end

% [beaconSizeBytes]
% Beacon size (Bytes)
[appParams,varargin]= addNewParam(appParams,'beaconSizeBytes',190,'Beacon size (Bytes)','integer',fileCfg,varargin{1});
if appParams.beaconSizeBytes<=0 || appParams.beaconSizeBytes>10000
    error('Error in the setting of "appParams.beaconSizeBytes".');
end

if simParams.technology ~= 2 % not only 11p
    % [resourcesV2V]
    % Resource allocated to V2V (LTE车辆占比%)
    [appParams,varargin]= addNewParam(appParams,'resourcesV2V',100,'Resource allocated to V2V (%)','integer',fileCfg,varargin{1});
    if appParams.resourcesV2V<=0 || appParams.resourcesV2V>100
        error('Error in the setting of "appParams.resourcesV2V". Not within 1-100%.');
    end
end
if simParams.technology == 2 % only 11p . variable size is not supported otherwise
    % [variableBeaconSize]
    % Enable to use variable beacon size
    [appParams,varargin]= addNewParam(appParams,'variableBeaconSize',false,'Varibale beacon size','bool',fileCfg,varargin{1});
    if appParams.variableBeaconSize~=false && appParams.variableBeaconSize~=true
        error('Error: "appParams.variableBeaconSize" must be equal to false or true');
    end
    
    if appParams.variableBeaconSize
        % [beaconSizeSmallBytes]
        % Beacon size small (Bytes)
        [appParams,varargin] = addNewParam(appParams,'beaconSizeSmallBytes',190,'Beacon size small (Bytes)','integer',fileCfg,varargin{1});
        if appParams.beaconSizeSmallBytes<=0 || appParams.beaconSizeSmallBytes>10000 || appParams.beaconSizeSmallBytes>appParams.beaconSizeBytes
            error('Error in the setting of "appParams.beaconSizeSmallBytes".');
        end
        
        % [NbeaconsSmall]
        % Number of small beacons between two large beacons
        [appParams,varargin]= addNewParam(appParams,'NbeaconsSmall',4,'Number of small beacons between two large beacons','integer',fileCfg,varargin{1});
        if appParams.NbeaconsSmall<=0
            error('Error in the setting of "appParams.beaconSizeSmallBytes".');
        end
    end
end

% [cbrActive]
% Duration of the interval for the CBR calculation [s]
[simParams,varargin] = addNewParam(simParams,'cbrActive',true,'If CBR calculation enabled','bool',fileCfg,varargin{1});
if simParams.cbrActive
    % [cbrSensingInterval]
    % Duration of the interval for the CBR calculation [s]
    [simParams,varargin] = addNewParam(simParams,'cbrSensingInterval',0.1,'Average duration of the interval for the CBR calculation (s)','double',fileCfg,varargin{1});
    if simParams.cbrSensingInterval<=0
        error('Error: "outParams.cbrSensingInterval" cannot be <= 0');
    end
    % [cbrSensingIntervalDesynchN]
    % The sensing interval is divided into cbrSensingIntervalDesynchN
    % subintervals to avoid synchronization among vehicles
    [simParams,varargin] = addNewParam(simParams,'cbrSensingIntervalDesynchN',100,'Number of subintervals for the CBR calculation desynch','integer',fileCfg,varargin{1});
    if simParams.cbrSensingIntervalDesynchN <= 0
        error('Error: "outParams.cbrSensingIntervalDesynchN" cannot be <= 0');
    end
    
end

% [dcc_active]
% Duration of the interval for the CBR calculation [s]
[simParams,varargin] = addNewParam(simParams,'dcc_active',true,'If DCC is enabled','bool',fileCfg,varargin{1});
if simParams.dcc_active && ~simParams.cbrActive
    error('Error: DCC requires "simParams.cbrActive" to be set to true');
end

% Temporary parameter
% % [mco_nVehInterf]
% % Number of vehicles that produce only an MCO interference
% [simParams,varargin]= addNewParam(simParams,'mco_nVehInterf',0,'Number of vehicles that produce only an MCO interference','integer',fileCfg,varargin{1});
% if simParams.mco_nVehInterf<0
%     error('Error: "simParams.mco_nVehInterf" cannot be < 0');
% end
    
fprintf('\n');
%
%%%%%%%%%
