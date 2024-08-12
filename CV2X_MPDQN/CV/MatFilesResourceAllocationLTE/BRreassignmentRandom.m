function [BRid, Nreassign] = BRreassignmentRandom(IDvehicle,simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams)
% Benchmark Algorithm 101 (RANDOM ALLOCATION)

Nvehicles = length(IDvehicle(:,1));   % Number of vehicles      

% % Without limitations, this was simpler
% % Assign a random beacon resource to all vehicles
% BRid(IDvehicle) = randi(appParams.Nbeacons,Nvehicles,1);
BRid(IDvehicle) = (ceil(rand(length(IDvehicle),1,1).*stationManagement.Nbeacons(IDvehicle)));

% % This part considers various limitations
% BRid = zeros(length(IDvehicle(:,1)),1);
% % A cycle over the vehicles is needed
% for idV=1:Nvehicles
% %     if stationManagement.vehicleState(IDvehicle(idV))~=100
% %         continue;
% %     end
%     while BRid(idV)==0
%         % A random BR is selected
% %         BRid(idV) = randi(appParams.Nbeacons(idV),1,1);
%         BRid(idV) = randi(appParams.Nbeacons,1,1);
%         % If it is not acceptable, it is reset to zero and a new random
%         % value is obtained
%         
%         % Case coexistence with mitigation methods - limited by
%         % superframe - i.e., if in ITS-G5 slot must be changed
% %         if simParams.technology==4 && simParams.coexMethod>0
% %             if ((simParams.coex_slotManagement == 1) ...
% %                     && mod(BRid(idV)-1,simParams.coex_superframeSF*appParams.NbeaconsF)+1 > (sinrManagement.coex_NtsLTE(1)*appParams.NbeaconsF)) || ...
% %                 ((simParams.coex_slotManagement == 2) ...
% %                     && mod(BRid(idV)-1,simParams.coex_superframeSF*appParams.NbeaconsF)+1 > (ceil(simParams.coex_superframeSF/2)*appParams.NbeaconsF))
% %                 BRid(idV) = 0;
% %             end
% %         end        
%         
%         % Mode 4 in general
%         % If it is outside the interval given by T1 and T2 it is not
%         % acceptable
%         if simParams.BRAlgorithm == 18
%             subframeNextPacket = mod(ceil(timeManagement.timeNextPacket(idV,3)./(phyParams.Tsf*1000))-1,(appParams.NbeaconsT))+1;
%             Tselected = ceil(BRid(idV)/appParams.NbeaconsF); 
%             % IF Both T1 and T2 are within this beacon period
%             if (subframeNextPacket+simParams.subframeT2Mode4+1)<=appParams.NbeaconsT
%                 if Tselected<subframeNextPacket+simParams.subframeT1Mode4 || Tselected>subframeNextPacket+simParams.subframeT2Mode4
%                    BRid(idV) = 0;
%                 end
%             % IF Both are beyond this beacon period
%             elseif (subframeNextPacket+simParams.subframeT1Mode4-1)>appParams.NbeaconsT
%                 if Tselected<subframeNextPacket+simParams.subframeT1Mode4-appParams.NbeaconsT || Tselected>subframeNextPacket+simParams.subframeT2Mode4-appParams.NbeaconsT
%                    BRid(idV) = 0;
%                 end
%             % IF T1 within, T2 beyond
%             else
%                 if Tselected<subframeNextPacket+simParams.subframeT1Mode4 && Tselected>subframeNextPacket+simParams.subframeT2Mode4-appParams.NbeaconsT
%                    BRid(idV) = 0;
%                 end
%             end 
%         end
%     end
% end

Nreassign = Nvehicles;

end