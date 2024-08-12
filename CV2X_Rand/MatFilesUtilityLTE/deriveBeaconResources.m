function [appParams,phyParams,varargin] = deriveBeaconResources(appParams,phyParams,fileCfg,varargin)
% Function used to derive the beacon resources

% Call function to find the total number of RBs in the frequency domain per Tslot
phyParams.RBsFrequency = RBtable(phyParams.BwMHz);

% Number of RBs allocated to the Cooperative Awareness (V2V)
appParams.RBsFrequencyV2V = floor(phyParams.RBsFrequency*(appParams.resourcesV2V/100));

% % Find the total number of RBs per subframe
phyParams.RBsSubframe = phyParams.RBsFrequency*(phyParams.Tsf/phyParams.Tslot);

% % Number of RBs allocated to the Cooperative Awareness
appParams.RBsSubframeBeaconing = appParams.RBsFrequencyV2V*(phyParams.Tsf/phyParams.Tslot);

% Call function to find the number of RBs needed to carry a beacon and minimum SINR
appParams.beaconSizeBits = appParams.beaconSizeBytes*8;
[appParams.RBsBeacon, phyParams.sinrThresholdLTE_LOS_dB, phyParams.NbitsHz] = findRBsBeaconSINRmin(phyParams.MCS_LTE,appParams.beaconSizeBits);

%% SINR
% At this point, in phyParams.gammaMinLTE_dB / phyParams.gammaMinLTE the
% automatic value is stored - it might be overwritten in the following
% lines
if phyParams.PERcurves == true
    % If PER vs. SINR curves are used, the vector of SINR thresholds must be set
    % Set the file name
    % It includes the number of resource block pairs used by the packet, to
    % cope with adjacent/non-adjacent and various suchannel sizes
    fileName = sprintf('%s/PER_%s_MCS%d_%dB.txt',phyParams.folderPERcurves,'LTE',phyParams.MCS_LTE,appParams.beaconSizeBytes);
    [phyParams.sinrVectorLTE_LOS_dB,~,~,~,sinrInterp,perInterp] = readPERtable(fileName,10000);
    %[phyParams.sinrVectorLTE_LOS_dB,perVectorLTE,~,~,sinrInterp,perInterp] = readPERtable(fileName,10000);
    %% DEBUG PER vs. SINR curves
    %plot(phyParams.sinrVectorLTE_LOS_dB,perVectorLTE,'pc');
    %hold on
    %plot(10*log10(sinrInterp),perInterp,'.k');
    %
    % Calculate the sinrThreshold, corresponding to 90% PER
    phyParams.sinrThresholdLTE_LOS_dB = 10*log10( sinrInterp(find(perInterp>=0.9,1)) );
    
    if strcmp(fileCfg,'Urban3GPP.cfg')
        %NLOS
        fileName = sprintf('%s/PER_%s_MCS%d_%dB.txt',phyParams.folderPERcurvesNLOS,'LTE',phyParams.MCS_LTE,appParams.beaconSizeBytes);
        [phyParams.sinrVectorLTE_NLOS_dB,~,~,~,sinrInterp,perInterp] = readPERtable(fileName,10000);
        %[phyParams.sinrVectorLTE_LOS_dB,perVectorLTE,~,~,sinrInterp,perInterp] = readPERtable(fileName,10000);
        %% DEBUG PER vs. SINR curves
        %plot(phyParams.sinrVectorLTE_LOS_dB,perVectorLTE,'pc');
        %hold on
        %plot(10*log10(sinrInterp),perInterp,'.k');
        %
        % Calculate the sinrThreshold, corresponding to 90% PER
        phyParams.sinrThresholdLTE_NLOS_dB = 10*log10( sinrInterp(find(perInterp>=0.9,1)) );
    else
        phyParams.sinrVectorLTE_NLOS_dB=phyParams.sinrVectorLTE_LOS_dB;
        phyParams.sinrThresholdLTE_NLOS_dB=phyParams.sinrThresholdLTE_LOS_dB;
    end
    
else
    % If PER vs. SINR curves are not used, the SINR threshold must be set
    
    % [SINRthresholdLTE]
    % SINR threshold for IEEE 802.11p
    [phyParams,varargin] = addNewParam(phyParams,'sinrThresholdLTE_LOS',-1000,'SINR threshold for error assessment [dB]','double',fileCfg,varargin{1});
    
    % -1000 means automatic setting
    if phyParams.sinrThresholdLTE_LOS ~= -1000
        phyParams.sinrThresholdLTE_LOS_dB = phyParams.sinrThresholdLTE_LOS;
    end
    phyParams.sinrVectorLTE_LOS_dB = phyParams.sinrThresholdLTE_LOS_dB;
    
    phyParams.sinrVectorLTE_NLOS_dB = phyParams.sinrVectorLTE_LOS_dB;
    phyParams.sinrThresholdLTE_NLOS_dB = phyParams.sinrThresholdLTE_LOS_dB;
end
%
phyParams.sinrThresholdLTE_LOS = 10.^(phyParams.sinrThresholdLTE_LOS_dB/10);
phyParams.sinrVectorLTE_LOS = 10.^(phyParams.sinrVectorLTE_LOS_dB/10);
%
phyParams.sinrThresholdLTE_NLOS=10.^(phyParams.sinrThresholdLTE_NLOS_dB/10);
phyParams.sinrVectorLTE_NLOS = 10.^(phyParams.sinrVectorLTE_NLOS_dB/10);


% Check whether the beacon size (appParams.RBsBeacon) + SCI (2x 2 RBs) exceeds the number of available RBs per subframe
appParams.RBsSubframeV2V = appParams.RBsFrequencyV2V*(phyParams.Tsf/phyParams.Tslot);
%if ~phyParams.BLERcurveLTE
if (appParams.RBsBeacon+4)>appParams.RBsSubframeV2V
    error('Error: "appParams.beaconSizeBytes" is too large for the selected MCS (packet cannot fit in a subframe)');
end
%else
%    % If using BLER, control if RBPsBeacon can fit in the subframe
%    appParams.RBsBeacon = phyParams.RBPsBeacon*(phyParams.Tsf/phyParams.Tslot);
%     if appParams.RBsBeacon>appParams.RBsSubframeV2V
%        error('Error: "phyParams.RBPsBeacon" is too large');
%     end
%end

% Find NbeaconsF, subchannel sizes or multiples
[appParams,phyParams] = calculateNB(appParams,phyParams);

% Compute radiated power per RB
%phyParams.PtxERP_RB = phyParams.PtxERP/(appParams.RBsBeacon/2);


% Compute BW of a BR,一个RB的大小从Hz→MHz需要乘1e-6，再乘包占用的BR数量，得一个包所占带宽
phyParams.BwMHz_lteBR = (phyParams.RBbandwidth*1e-6) * (appParams.RBsBeacon/2) ;
% Compute power per MHz in LTE，计算每MHz的功率，指数的'减'是常数的'除'，所有功率除以带宽大小
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
phyParams.P_ERP_MHz_LTE = 10^((phyParams.P_ERP_MHz_LTE_dBm-30)/10);%RBdBm = 10*log10(RB)+30;30dbm为1w

% Compute In-Band Emission Matrix (following 3GPP TS 36.101 v15.0.0)计算发射矩阵
[phyParams.IBEmatrixData,phyParams.IBEmatrixControl] = IBEcalculation(phyParams,appParams);

% Check how many BRs to exploit in the frequency domain 检查频域中要利用的BR数量，NumBeaconsFrequency=-1没用上
if phyParams.NumBeaconsFrequency~=-1
    if phyParams.NumBeaconsFrequency > appParams.NbeaconsF
        fprintf('Number of beacons in frequency domain in input is larger than the maximum one: set to %.0f\n\n', NbeaconsF);
    else
        appParams.NbeaconsF = phyParams.NumBeaconsFrequency;
        phyParams.IBEmatrix = phyParams.IBEmatrix(1:appParams.NbeaconsF,1:appParams.NbeaconsF);
    end
end

% Total number of beacons per beacon period = Beacon Resources
% (BRs)，一周期子帧内的资源数
appParams.Nbeacons = appParams.NbeaconsF.*appParams.NbeaconsT;
%appParams.Nbeacons = appParams.NbeaconsF.*appParams.NbeaconsT;

% Error check
if appParams.Nbeacons<1
    fprintf('Number of beacons equal to %d. Error.', Nbeacons);
    error('');
end

end