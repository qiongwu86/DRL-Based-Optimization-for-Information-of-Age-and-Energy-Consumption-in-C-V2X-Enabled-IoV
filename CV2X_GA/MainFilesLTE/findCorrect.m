function [correctMatrix,stateavg] = findCorrect(IDvehicleTXLTE,indexVehicleTX,neighborsID,sinrManagement,stationManagement,positionManagement,phyParams)
% Detect wrongly decoded beacons and create Error Matrix
% [ID TX, ID RX, BRid, distance]

distance = positionManagement.distanceReal(stationManagement.activeIDs,stationManagement.activeIDs);
rawThreshold = phyParams.Raw;
BwMHz_lteBR = phyParams.BwMHz_lteBR;

N = length(stationManagement.activeIDs);
Ntx = length(IDvehicleTXLTE);                % Number of tx vehicles
correctMatrix = zeros(Ntx*Ntx-1,4);          % Initialize error matrix
Ncorrects = 0;                               % Initialize number of errors
stateavg = zeros(N,3);
stateavg(:,3) = ones(N,1);
delayt = zeros(N,1);


for i = 1:Ntx
    neighborsSINRaverageLTE = sinrManagement.neighborsSINRaverageLTE;
    Ni = 0;
    indexi = IDvehicleTXLTE(i);
    indexNeighborsRX = find(neighborsID(indexi,:));% Find indexes of receiving vehicles in neighborsID,距离排序
%     SINRaverageLTE = inf;
    for j = 1:length(indexNeighborsRX)   
        indexj = indexNeighborsRX(j);
        IDvehicleRX = neighborsID(indexi,indexj);
       %阈值是根据信道1子帧来计算的固定值，即1子帧时间段内，完成通信所需的最小SINR
        sinrThreshold=(phyParams.LOS(i,indexj)*phyParams.sinrVectorLTE_LOS(randi(length(phyParams.sinrVectorLTE_LOS)))+... %if LOS
            (1-phyParams.LOS(i,indexj))*phyParams.sinrVectorLTE_NLOS(randi(length(phyParams.sinrVectorLTE_NLOS))));  %if NLOS
        rateth = BwMHz_lteBR*1e6.*sinrThreshold*1e-3;%阈值即数据包大小，1e6是MHz→Hz，1e-3是bit/s→bit/ms
        ratet = BwMHz_lteBR*1e6.*neighborsSINRaverageLTE(i,indexj)*1e-3;  %对应小于阈值
        if neighborsSINRaverageLTE(i,indexj) >= sinrThreshold %非dB值     
            Ncorrects = Ncorrects + 1; 
            correctMatrix(Ncorrects,1) = indexi;
            correctMatrix(Ncorrects,2) = IDvehicleRX;  %neighborsID(indexVehicleTX(i),:),接收端的排序
            correctMatrix(Ncorrects,3) = stationManagement.BRid(indexi);
            correctMatrix(Ncorrects,4) = distance(indexi,IDvehicleRX);
%             ratet = BwMHz_lteBR*1e6.*neighborsSINRaverageLTE(i,indexj)*1e-3; %通信速率
        end

        if distance(indexi,IDvehicleRX) <= rawThreshold
            stateavg(indexi,1) = stateavg(indexi,1) + (neighborsSINRaverageLTE(i,indexj)>=sinrThreshold);  %大于阈值为1，小于阈值为0
            stateavg(indexi,2) = stateavg(indexi,2) + distance(indexi,IDvehicleRX);
            delayt(indexi) = delayt(indexi) + min(1,rateth./ratet);
            Ni = Ni +1;
        end      
    end
       
    if Ni >0 && nnz(neighborsSINRaverageLTE)>0
        stateavg(indexi,1) = stateavg(indexi,1)/Ni;
        stateavg(indexi,2) = stateavg(indexi,2)/Ni;
        stateavg(indexi,3) = delayt(indexi)/Ni;
    end
end


delIndex = correctMatrix(:,1)==0;
correctMatrix(delIndex,:) = [];
% SINRaverageLTE
end