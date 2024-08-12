function printCBRToFileLTE(stationManagement,simParams,outParams,phyParams)
% Print CBR statistics

%% CBR LTE statistic
% I remove the first 10 values of CBR to consider a minimum of transitory
if length(stationManagement.cbrLteValues(1,:))>10
    stationManagement.cbrLteValues(:,1:10) = [];
end

for iChannel=1:phyParams.nChannels
    
    if sum(stationManagement.vehicleChannel==iChannel)==0
        continue;
    end
    
    if phyParams.nChannels==1
        cbrOutputFileName = sprintf('%s/CBRstatistic_%.0f_LTE.xls',outParams.outputFolder,outParams.simID);
    else
        cbrOutputFileName = sprintf('%s/CBRstatistic_%.0f_LTE_C%d.xls',outParams.outputFolder,outParams.simID,iChannel);
    end

    cbrVector = stationManagement.cbrLteValues(stationManagement.vehicleChannel==iChannel,:);
    cbrVector = reshape(cbrVector,[],1);
    values = cbrVector(cbrVector>=0);
    [F,X] = ecdf(values);

    fileID = fopen(cbrOutputFileName,'w');
    for i=1:length(F)
        fprintf(fileID,'%f\t%f\n',X(i),F(i));
    end
    fclose(fileID);

    %% CBR_LTE_only
    if length(stationManagement.coex_cbrLteOnlyValues(1,:))>10
        stationManagement.coex_cbrLteOnlyValues(:,1:10) = [];
    end
    valuesCbrLteOnly = stationManagement.coex_cbrLteOnlyValues(stationManagement.coex_cbrLteOnlyValues~=-1);
    [F2,X2] = ecdf(valuesCbrLteOnly);
    cbrOutputFileName = sprintf('%s/coex_LteOnly_CBRstatistic_%.0f_LTE.xls',outParams.outputFolder,outParams.simID);
    fileID2 = fopen(cbrOutputFileName,'w');
    for i=1:length(F2)
        fprintf(fileID2,'%f\t%f\n',X2(i),F2(i));
    end
    fclose(fileID2);
end

%% Generic vehicle
if ~isempty(stationManagement.activeIDsLTE)    
    idTest = stationManagement.activeIDsLTE(1);
    
    cbrOutputFileName = sprintf('%s/CBRofGenericVehicle_%.0f_LTE.xls',outParams.outputFolder,outParams.simID);
    values = stationManagement.cbrLteValues(idTest,:);
    time = (10+(1:length(values))) * simParams.cbrSensingInterval;
    fileID = fopen(cbrOutputFileName,'w');
    for i=1:length(values)
        if values(i)>=0
            fprintf(fileID,'%f\t%f\n',time(i),values(i));
        end
    end
    fclose(fileID);
end

