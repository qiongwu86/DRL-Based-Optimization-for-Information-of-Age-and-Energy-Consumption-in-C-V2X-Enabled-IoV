function sinrManagement = updateSINRLTE(timeNow,stationManagement,sinrManagement,Pnoise,simParams,appParams)
% Calculates the average SINR to each receiving neighbor

if isempty(stationManagement.activeIDsLTE)
    return
else
    transmittingIDsLTE = stationManagement.transmittingIDsLTE;
end

%% FROM VERSION 5.3.0

% Given:
%     sinrManagement.neighPowerUsefulLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
%     sinrManagement.neighPowerInterfLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
%     sinrManagement.neighborsInterfFrom11pAverageLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
%     sinrManagement.neighborsSINRaverageLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
%     sinrManagement.neighborsSINRsciAverageLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
%     sinrManagement.instantThisPstartedLTE = timeManagement.timeNow;
%     sinrManagement.instantTheSINRaverageStartedLTE = timeManagement.timeNow;

% If coexistence, I have also to update the average interference from 11p nodes
% to LTE nodes
if simParams.technology == 4
    sinrManagement.coex_averageSFinterfFrom11pToLTE = (sinrManagement.coex_averageSFinterfFrom11pToLTE .* (sinrManagement.instantThisPstartedLTE-sinrManagement.instantTheSINRaverageStartedLTE) + ... 
        sinrManagement.coex_currentInterfFrom11pToLTE .* (timeNow-sinrManagement.instantThisPstartedLTE)) ./ (timeNow-sinrManagement.instantTheSINRaverageStartedLTE);
end

if ~isempty(transmittingIDsLTE)
    % Average interference from 11p nodes, if present (otherwise will emain zero)
    neighborsInterf11p = zeros(length(transmittingIDsLTE),length(sinrManagement.neighPowerUsefulLTE(1,:)));
    
    if simParams.technology == 4  %取值为1，不会进入if
        % This part converts the coex_averageSFinterfFrom11pToLTE
        % to stationManagement.neighborsIDLTE
        % TODO - could be optimized
        for iLTEtx = 1:length(transmittingIDsLTE)
            for iInterf = 1:length(stationManagement.neighborsIDLTE(1,:))     
                if stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(iLTEtx),iInterf)<=0
                    break;
                end
                neighborsInterf11p(iLTEtx,iInterf) = sinrManagement.coex_averageSFinterfFrom11pToLTE(stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(iLTEtx),iInterf));
            end
        end
    end
    
    %有data和sci两种信息的SINR
    % The average SINR for data is updateed
    sinrManagement.neighborsSINRaverageLTE = sinrManagement.neighPowerUsefulLTE ./ ...
        ( Pnoise + sinrManagement.neighPowerInterfDataLTE + neighborsInterf11p);

    % The average SINR for control is updateed
    sinrManagement.neighborsSINRsciAverageLTE = sinrManagement.neighPowerUsefulLTE ./ ...
        ( Pnoise + sinrManagement.neighPowerInterfControlLTE + (2/(appParams.RBsBeacon/2)) * neighborsInterf11p);
else
    sinrManagement.neighborsSINRaverageLTE = [];
    sinrManagement.neighborsSINRsciAverageLTE = [];
end

sinrManagement.instantThisPstartedLTE = timeNow;

% %% SINCE VERSION 5.2.10
% 
% % Given:
% % sinrManagement.neighPowerUsefulLastLTE
% % sinrManagement.neighPowerInterfLastLTE
% % sinrManagement.neighborsSINRaverageLTE
% % sinrManagement.instantThisPstartedLTE
% % sinrManagement.instantTheSINRaverageStartedLTE
% 
% % % This part was without interference from IEEE 802.11p
% % Without 11p interference
% % sinrLast = sinrManagement.neighPowerUsefulLastLTE ./ ( Pnoise + sinrManagement.neighPowerInterfLastLTE);
% % 
% % neighborsSINRaverageLTE = (sinrManagement.neighborsSINRaverageLTE .* (sinrManagement.instantThisPstartedLTE-sinrManagement.instantTheSINRaverageStartedLTE) + ... 
% %     sinrLast .* (timeNow-sinrManagement.instantThisPstartedLTE)) ./ (timeNow-sinrManagement.instantTheSINRaverageStartedLTE);
% % %
% 
% % % This is the new version to include interference form IEEE 802.11p nodes
% % TODO: is it possible to optimize?
% if ~isempty(transmittingIDsLTE)
%     sinrLast = zeros(length(transmittingIDsLTE),length(sinrManagement.neighPowerUsefulLastLTE(1,:)));
%     sinrSciLast = zeros(length(transmittingIDsLTE),length(sinrManagement.neighPowerUsefulLastLTE(1,:)));
%     for iLTEtx = 1:length(transmittingIDsLTE)
%         for iInterf = 1:length(stationManagement.neighborsIDLTE(1,:))     
% 
%             if stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(iLTEtx),iInterf)>0                
%                 % Data
%                 sinrLast(iLTEtx,iInterf) = sinrManagement.neighPowerUsefulLastLTE(iLTEtx,iInterf) ./ ( Pnoise + sinrManagement.neighPowerInterfLastLTE(iLTEtx,iInterf) + sinrManagement.coex_currentInterfFrom11pToLTE(stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(iLTEtx),iInterf)));
%                 % SCI - 11p interference must be scaled down
%                 sinrSciLast(iLTEtx,iInterf) = sinrManagement.neighPowerUsefulLastLTE(iLTEtx,iInterf) ./ ( Pnoise + sinrManagement.neighPowerInterfLastLTE(iLTEtx,iInterf) + (2/(appParams.RBsBeacon/2)) * sinrManagement.coex_currentInterfFrom11pToLTE(stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(iLTEtx),iInterf)));
% %fp = fopen('temp.xls','a');
% %fprintf(fp,'%f\n',10*log10(sinrLast(iLTEtx,iInterf)));
% %fclose(fp);
%             end
%         end
%     end
% 
%     sinrManagement.neighborsSINRaverageLTE = (sinrManagement.neighborsSINRaverageLTE .* (sinrManagement.instantThisPstartedLTE-sinrManagement.instantTheSINRaverageStartedLTE) + ... 
%         sinrLast .* (timeNow-sinrManagement.instantThisPstartedLTE)) ./ (timeNow-sinrManagement.instantTheSINRaverageStartedLTE);    
% 
%     sinrManagement.neighborsSINRsciAverageLTE = (sinrManagement.neighborsSINRsciAverageLTE .* (sinrManagement.instantThisPstartedLTE-sinrManagement.instantTheSINRaverageStartedLTE) + ... 
%         sinrSciLast .* (timeNow-sinrManagement.instantThisPstartedLTE)) ./ (timeNow-sinrManagement.instantTheSINRaverageStartedLTE);        
% else
%     sinrManagement.neighborsSINRaverageLTE = [];
%     sinrManagement.neighborsSINRsciAverageLTE = [];
% end
% 
% % If coexistence, I have also to update the average interference from 11p nodes
% % to LTE nodes
% if simParams.technology == 4
%     sinrManagement.coex_averageSFinterfFrom11pToLTE = (sinrManagement.coex_averageSFinterfFrom11pToLTE .* (sinrManagement.instantThisPstartedLTE-sinrManagement.instantTheSINRaverageStartedLTE) + ... 
%         sinrManagement.coex_currentInterfFrom11pToLTE .* (timeNow-sinrManagement.instantThisPstartedLTE)) ./ (timeNow-sinrManagement.instantTheSINRaverageStartedLTE);
% end
% 
% sinrManagement.instantThisPstartedLTE = timeNow;
