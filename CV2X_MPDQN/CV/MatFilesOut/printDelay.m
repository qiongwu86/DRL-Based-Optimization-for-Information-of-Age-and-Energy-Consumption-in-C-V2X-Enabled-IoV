function printDelay(stationManagement,outputValues,appParams,outParams,phyParams)
% Print to file the delay occurrences
% [delay (s) - number of events - CDF]

% Update delay
if outParams.printUpdateDelay
    for iChannel = 1:phyParams.nChannels
        for pckType = 1:appParams.nPckTypes
            
            if sum(stationManagement.vehicleState==100)>0
            %if simParams.technology ~= 2 % not only 11p
                if sum(sum(outputValues.updateDelayCounterLTE(iChannel,pckType,:,:)))==0
                    continue;
                end
                % outputValues.updateDelayCounterLTE needs elaboration
                % Now contains values up to each awareness range value; must
                % instead include the values in each group
                for iPhyRaw=length(outputValues.updateDelayCounterLTE(iChannel,pckType,1,:)):-1:2
                    outputValues.updateDelayCounterLTE(iChannel,pckType,:,iPhyRaw) = outputValues.updateDelayCounterLTE(iChannel,pckType,:,iPhyRaw)-outputValues.updateDelayCounterLTE(iChannel,pckType,:,iPhyRaw-1);
                end
                % Now the values can be print
                filename_part1 = sprintf('%s/update_delay_%.0f_%s',outParams.outputFolder,outParams.simID,'LTE');
                if pckType==1
                    filename_part2 = '';
                elseif pckType==2
                    filename_part2 = '_DENM';
                else
                    error('Packet type not implemented');
                end  
                if phyParams.nChannels==1
                    filename_part3 = '';
                else
                    filename_part3 = sprintf('_C%d',iChannel);
                end
                filename = strcat(filename_part1,filename_part2,filename_part3,'.xls');                
%                 if pckType==1
%                     filename = sprintf('%s/update_delay_%.0f_%s.xls',outParams.outputFolder,outParams.simID,'LTE');
%                 elseif pckType==2
%                     filename = sprintf('%s/update_delay_%.0f_%s_DENM.xls',outParams.outputFolder,outParams.simID,'LTE');
%                 else
%                     error('Packet type not implemented');
%                 end                
                fileID = fopen(filename,'at');
                NeventsTOT = sum(squeeze(outputValues.updateDelayCounterLTE(iChannel,pckType,:,:)),1);
                for i = 1:length(outputValues.updateDelayCounterLTE(iChannel,pckType,:,1))
                    fprintf(fileID,'%.3f\t',i*outParams.delayResolution);
                    for iPhyRaw=1:length(NeventsTOT)   
                        fprintf(fileID,'%d\t%.6f',outputValues.updateDelayCounterLTE(iChannel,pckType,i,iPhyRaw),sum(squeeze(outputValues.updateDelayCounterLTE(iChannel,pckType,1:i,iPhyRaw)))/NeventsTOT(iPhyRaw));
                        if length(NeventsTOT)>1
                            fprintf(fileID,'\t');
                        else
                            fprintf(fileID,'\n');
                        end
                    end
                    if length(NeventsTOT)>1
                        fprintf(fileID,'%d\t%.6f\n',sum(squeeze(outputValues.updateDelayCounterLTE(iChannel,pckType,i,:))),sum(sum(squeeze(outputValues.updateDelayCounterLTE(iChannel,pckType,1:i,:)),1))/sum(NeventsTOT(:)));
                    end
                end
                fclose(fileID);
            end
            if sum(stationManagement.vehicleState~=100)>0
            %if simParams.technology ~= 1 % not only LTE
                if sum(sum(outputValues.updateDelayCounter11p(iChannel,pckType,:,:)))==0
                    continue;
                end
                % outputValues.updateDelayCounter11p needs elaboration
                % Now contains values up to each awareness range value; must
                % instead include teh values in each group
                for iPhyRaw=length(outputValues.updateDelayCounter11p(iChannel,pckType,1,:)):-1:2
                    outputValues.updateDelayCounter11p(iChannel,pckType,:,iPhyRaw) = outputValues.updateDelayCounter11p(iChannel,pckType,:,iPhyRaw)-outputValues.updateDelayCounter11p(iChannel,pckType,:,iPhyRaw-1);
                end
                % Now the values can be print
                filename_part1 = sprintf('%s/update_delay_%.0f_%s',outParams.outputFolder,outParams.simID,'11p');
                if pckType==1
                    filename_part2 = '';
                elseif pckType==2
                    filename_part2 = '_DENM';
                else
                    error('Packet type not implemented');
                end  
                if phyParams.nChannels==1
                    filename_part3 = '';
                else
                    filename_part3 = sprintf('_C%d',iChannel);
                end
                filename = strcat(filename_part1,filename_part2,filename_part3,'.xls');                
%                 if pckType==1
%                     filename = sprintf('%s/update_delay_%.0f_%s.xls',outParams.outputFolder,outParams.simID,'11p');
%                 elseif pckType==2
%                     filename = sprintf('%s/update_delay_%.0f_%s_DENM.xls',outParams.outputFolder,outParams.simID,'11p');
%                 else
%                     error('Packet type not implemented');
%                 end                
                fileID = fopen(filename,'at');
                NeventsTOT = sum(squeeze(outputValues.updateDelayCounter11p(iChannel,pckType,:,:)),1);
                for i = 1:length(outputValues.updateDelayCounter11p(iChannel,pckType,:,1))
                    fprintf(fileID,'%.3f\t',i*outParams.delayResolution);
                    for iPhyRaw=1:length(NeventsTOT)   
                        fprintf(fileID,'%d\t%.6f',outputValues.updateDelayCounter11p(iChannel,pckType,i,iPhyRaw),sum(squeeze(outputValues.updateDelayCounter11p(iChannel,pckType,1:i,iPhyRaw)))/NeventsTOT(iPhyRaw));
                        if length(NeventsTOT)>1
                            fprintf(fileID,'\t');
                        else
                            fprintf(fileID,'\n');
                        end
                    end
                    if length(NeventsTOT)>1
                        fprintf(fileID,'%d\t%.6f\n',sum(squeeze(outputValues.updateDelayCounter11p(iChannel,pckType,i,:))),sum(sum(squeeze(outputValues.updateDelayCounter11p(iChannel,pckType,1:i,:)),1))/sum(NeventsTOT(:)));
                    end
                end
                fclose(fileID);
            end
        end
    end

    % Wireless blind spot probability
    if outParams.printWirelessBlindSpotProb
        % Print to file the wireless blind spot probability
        % [Time interval - # delay events larger or equal than time interval - #
        % delay events shorter than time interval - wireless blind spot probability]
        filename = sprintf('%s/wireless_blind_spot_%.0f.xls',outParams.outputFolder,outParams.simID);
        fileID = fopen(filename,'at');
        for i = 1:length(outputValues.wirelessBlindSpotCounter)
            fprintf(fileID,'%.3f\t%d\t%d\t%.6f\n',outputValues.wirelessBlindSpotCounter(i,1),outputValues.wirelessBlindSpotCounter(i,2),outputValues.wirelessBlindSpotCounter(i,3),...
                outputValues.wirelessBlindSpotCounter(i,2)/(outputValues.wirelessBlindSpotCounter(i,2)+outputValues.wirelessBlindSpotCounter(i,3)));
        end
        fclose(fileID);  
    end
end

% Data Age
if outParams.printDataAge
    for iChannel = 1:phyParams.nChannels
        for pckType = 1:appParams.nPckTypes

            if sum(stationManagement.vehicleState==100)>0 % any LTE node
                if sum(sum(outputValues.dataAgeCounterLTE(iChannel,pckType,:,:)))==0
                    continue;
                end

                % outputValues.dataAgeCounterLTE needs elaboration
                % Now contains values up to each awareness range value; must
                % instead include teh values in each group
                for iPhyRaw=length(outputValues.dataAgeCounterLTE(iChannel,pckType,1,:)):-1:2
                    outputValues.dataAgeCounterLTE(iChannel,pckType,:,iPhyRaw) = outputValues.dataAgeCounterLTE(iChannel,pckType,:,iPhyRaw)-outputValues.dataAgeCounterLTE(iChannel,pckType,:,iPhyRaw-1);
                end
                % Now the values can be print
                filename_part1 = sprintf('%s/data_age_%.0f_%s',outParams.outputFolder,outParams.simID,'LTE');
                if pckType==1
                    filename_part2 = '';
                elseif pckType==2
                    filename_part2 = '_DENM';
                else
                    error('Packet type not implemented');
                end  
                if phyParams.nChannels==1
                    filename_part3 = '';
                else
                    filename_part3 = sprintf('_C%d',iChannel);
                end
                filename = strcat(filename_part1,filename_part2,filename_part3,'.xls');                
%                 if pckType==1
%                     filename = sprintf('%s/data_age_%.0f_%s.xls',outParams.outputFolder,outParams.simID,'LTE');
%                 elseif pckType==2
%                     filename = sprintf('%s/data_age_%.0f_%s_DENM.xls',outParams.outputFolder,outParams.simID,'LTE');
%                 else
%                     error('Packet type not implemented');
%                 end                            
                fileID = fopen(filename,'at');
                NeventsTOT = sum(squeeze(outputValues.dataAgeCounterLTE(iChannel,pckType,:,:)),1);
                for i = 1:length(outputValues.dataAgeCounterLTE(iChannel,pckType,:,1))
                    fprintf(fileID,'%.3f\t',i*outParams.delayResolution);
                    for iPhyRaw=1:length(NeventsTOT)   
                        fprintf(fileID,'%d\t%.6f',outputValues.dataAgeCounterLTE(iChannel,pckType,i,iPhyRaw),sum(squeeze(outputValues.dataAgeCounterLTE(iChannel,pckType,1:i,iPhyRaw)))/NeventsTOT(iPhyRaw));
                        if length(NeventsTOT)>1
                            fprintf(fileID,'\t');
                        else
                            fprintf(fileID,'\n');
                        end
                    end
                    if length(NeventsTOT)>1
                        fprintf(fileID,'%d\t%.6f\n',sum(squeeze(outputValues.dataAgeCounterLTE(iChannel,pckType,i,:))),sum(sum(squeeze(outputValues.dataAgeCounterLTE(iChannel,pckType,1:i,:)),1))/sum(NeventsTOT(:)));
                    end
                end
                fclose(fileID);
            end

            if sum(stationManagement.vehicleState~=100)>0 % If any 11p node
 
                if sum(sum(outputValues.dataAgeCounter11p(iChannel,pckType,:,:)))==0
                    continue;
                end

                % outputValues.dataAgeCounter11p needs elaboration
                % Now contains values up to each awareness range value; must
                % instead include teh values in each group
                for iPhyRaw=length(outputValues.dataAgeCounter11p(iChannel,pckType,1,:)):-1:2
                    outputValues.dataAgeCounter11p(iChannel,pckType,:,iPhyRaw) = outputValues.dataAgeCounter11p(iChannel,pckType,:,iPhyRaw)-outputValues.dataAgeCounter11p(iChannel,pckType,:,iPhyRaw-1);
                end
                % Now the values can be print
                filename_part1 = sprintf('%s/data_age_%.0f_%s',outParams.outputFolder,outParams.simID,'11p');
                if pckType==1
                    filename_part2 = '';
                elseif pckType==2
                    filename_part2 = '_DENM';
                else
                    error('Packet type not implemented');
                end  
                if phyParams.nChannels==1
                    filename_part3 = '';
                else
                    filename_part3 = sprintf('_C%d',iChannel);
                end
                filename = strcat(filename_part1,filename_part2,filename_part3,'.xls');                
%                 if pckType==1
%                     filename = sprintf('%s/data_age_%.0f_%s.xls',outParams.outputFolder,outParams.simID,'11p');
%                 elseif pckType==2
%                     filename = sprintf('%s/data_age_%.0f_%s_DENM.xls',outParams.outputFolder,outParams.simID,'11p');
%                 else
%                     error('Packet type not implemented');
%                 end                            
                fileID = fopen(filename,'at');
                NeventsTOT = sum(squeeze(outputValues.dataAgeCounter11p(iChannel,pckType,:,:)),1);
                for i = 1:length(outputValues.dataAgeCounter11p(iChannel,pckType,:,1))
                    fprintf(fileID,'%.3f\t',i*outParams.delayResolution);
                    for iPhyRaw=1:length(NeventsTOT)   
                        fprintf(fileID,'%d\t%.6f',outputValues.dataAgeCounter11p(iChannel,pckType,i,iPhyRaw),sum(squeeze(outputValues.dataAgeCounter11p(iChannel,pckType,1:i,iPhyRaw)))/NeventsTOT(iPhyRaw));
                        if length(NeventsTOT)>1
                            fprintf(fileID,'\t');
                        else
                            fprintf(fileID,'\n');
                        end
                    end
                    if length(NeventsTOT)>1
                        fprintf(fileID,'%d\t%.6f\n',sum(squeeze(outputValues.dataAgeCounter11p(iChannel,pckType,i,:))),sum(sum(squeeze(outputValues.dataAgeCounter11p(iChannel,pckType,1:i,:)),1))/sum(NeventsTOT(:)));
                    end
                end
                fclose(fileID);
            end
        end
    end
end

% Packet delay
if outParams.printPacketDelay
    for iChannel = 1:phyParams.nChannels
        for pckType = 1:appParams.nPckTypes
            if sum(stationManagement.vehicleState==100)>0
            %if simParams.technology ~= 2 % not only 11p
                if sum(sum(outputValues.packetDelayCounterLTE(iChannel,pckType,:,:)))==0
                    continue;
                end
                % outputValues.packetDelayCounterLTE needs elaboration
                % Now contains values up to each awareness range value; must
                % instead include teh values in each group
                for iPhyRaw=length(outputValues.packetDelayCounterLTE(iChannel,pckType,1,:)):-1:2
                    outputValues.packetDelayCounterLTE(iChannel,pckType,:,iPhyRaw) = outputValues.packetDelayCounterLTE(iChannel,pckType,:,iPhyRaw)-outputValues.packetDelayCounterLTE(iChannel,pckType,:,iPhyRaw-1);
                end
                % Now the values can be print
                filename_part1 = sprintf('%s/packet_delay_%.0f_%s',outParams.outputFolder,outParams.simID,'LTE');
                if pckType==1
                    filename_part2 = '';
                elseif pckType==2
                    filename_part2 = '_DENM';
                else
                    error('Packet type not implemented');
                end  
                if phyParams.nChannels==1
                    filename_part3 = '';
                else
                    filename_part3 = sprintf('_C%d',iChannel);
                end
                filename = strcat(filename_part1,filename_part2,filename_part3,'.xls');                
%                 if pckType==1
%                     filename = sprintf('%s/packet_delay_%.0f_%s.xls',outParams.outputFolder,outParams.simID,'LTE');
%                 elseif pckType==2
%                     filename = sprintf('%s/packet_delay_%.0f_%s_DENM.xls',outParams.outputFolder,outParams.simID,'LTE');
%                 else
%                     error('Packet type not implemented');
%                 end
                fileID = fopen(filename,'at');
                NeventsTOT = sum(squeeze(outputValues.packetDelayCounterLTE(iChannel,pckType,:,:)),1);
                for i = 1:length(outputValues.packetDelayCounterLTE(iChannel,pckType,:,1))
                    fprintf(fileID,'%.3f\t',i*outParams.delayResolution);
                    for iPhyRaw=1:length(NeventsTOT)   
                        fprintf(fileID,'%d\t%.6f',outputValues.packetDelayCounterLTE(iChannel,pckType,i,iPhyRaw),sum(squeeze(outputValues.packetDelayCounterLTE(iChannel,pckType,1:i,iPhyRaw)))/NeventsTOT(iPhyRaw));
                        if length(NeventsTOT)>1
                            fprintf(fileID,'\t');
                        else
                            fprintf(fileID,'\n');
                        end
                    end
                    if length(NeventsTOT)>1
                        fprintf(fileID,'%d\t%.6f\n',sum(squeeze(outputValues.packetDelayCounterLTE(iChannel,pckType,i,:))),sum(sum(squeeze(outputValues.packetDelayCounterLTE(iChannel,pckType,1:i,:)),1))/sum(NeventsTOT(:)));
                    end
                end
                fclose(fileID);    
            end
            if sum(stationManagement.vehicleState~=100)>0
            %if simParams.technology ~= 1 % not only LTE
                if sum(sum(outputValues.packetDelayCounter11p(iChannel,pckType,:,:)))==0
                    continue;
                end
                % outputValues.packetDelayCounter11p needs elaboration
                % Now contains values up to each awareness range value; must
                % instead include teh values in each group
                for iPhyRaw=length(outputValues.packetDelayCounter11p(iChannel,pckType,1,:)):-1:2
                    outputValues.packetDelayCounter11p(iChannel,pckType,:,iPhyRaw) = outputValues.packetDelayCounter11p(iChannel,pckType,:,iPhyRaw)-outputValues.packetDelayCounter11p(iChannel,pckType,:,iPhyRaw-1);
                end
                % Now the values can be print
                filename_part1 = sprintf('%s/packet_delay_%.0f_%s',outParams.outputFolder,outParams.simID,'11p');
                if pckType==1
                    filename_part2 = '';
                elseif pckType==2
                    filename_part2 = '_DENM';
                else
                    error('Packet type not implemented');
                end  
                if phyParams.nChannels==1
                    filename_part3 = '';
                else
                    filename_part3 = sprintf('_C%d',iChannel);
                end
                filename = strcat(filename_part1,filename_part2,filename_part3,'.xls');                
%                 if pckType==1
%                     filename = sprintf('%s/packet_delay_%.0f_%s.xls',outParams.outputFolder,outParams.simID,'11p');
%                 elseif pckType==2
%                     filename = sprintf('%s/packet_delay_%.0f_%s_DENM.xls',outParams.outputFolder,outParams.simID,'11p');
%                 else
%                     error('Packet type not implemented');
%                 end
                fileID = fopen(filename,'at');
                NeventsTOT = sum(squeeze(outputValues.packetDelayCounter11p(iChannel,pckType,:,:)),1);
                for i = 1:length(outputValues.packetDelayCounter11p(iChannel,pckType,:,1))
                    fprintf(fileID,'%.3f\t',i*outParams.delayResolution);
                    for iPhyRaw=1:length(NeventsTOT)   
                        fprintf(fileID,'%d\t%.6f',outputValues.packetDelayCounter11p(iChannel,pckType,i,iPhyRaw),sum(squeeze(outputValues.packetDelayCounter11p(iChannel,pckType,1:i,iPhyRaw)))/NeventsTOT(iPhyRaw));
                        if length(NeventsTOT)>1
                            fprintf(fileID,'\t');
                        else
                            fprintf(fileID,'\n');
                        end
                    end
                    if length(NeventsTOT)>1
                        fprintf(fileID,'%d\t%.6f\n',sum(squeeze(outputValues.packetDelayCounter11p(iChannel,pckType,i,:))),sum(sum(squeeze(outputValues.packetDelayCounter11p(iChannel,pckType,1:i,:)),1))/sum(NeventsTOT(:)));
                    end
                end
                fclose(fileID);    
            end
        end
    end
end

end

