function [sinrManagement,stationManagement] = initLastPowerLTE(timeManagement,stationManagement,sinrManagement,simParams,appParams,phyParams)

% If coexistence, I have to initialize the value of averageInterfFrom11pToLTE
if simParams.technology == 4
    sinrManagement.coex_averageSFinterfFrom11pToLTE = sinrManagement.coex_currentInterfFrom11pToLTE;
    % note: the following two parameters might be overwritten later, but with the same values 
    sinrManagement.instantThisPstartedLTE = timeManagement.timeNow;
    sinrManagement.instantTheSINRaverageStartedLTE = timeManagement.timeNow;
end

% If there is at least one LTE station transmitting, I have to initialize
% SINR values
if ~isempty(stationManagement.transmittingIDsLTE) 
    %当前时刻各车辆之间的接收功率
    RXpower_MHz_ofLTE = sinrManagement.P_RX_MHz(stationManagement.indexInActiveIDs_ofLTEnodes,stationManagement.indexInActiveIDs_ofLTEnodes);

    % Number of vehicles transmitting in the current subframe
    Ntx = length(stationManagement.transmittingIDsLTE);

    % Initialization of SINRmanagement
    sinrManagement.neighPowerUsefulLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
    sinrManagement.neighPowerInterfDataLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
    sinrManagement.neighPowerInterfControlLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
    sinrManagement.neighborsInterfFrom11pAverageLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
    sinrManagement.neighborsSINRaverageLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
    sinrManagement.neighborsSINRsciAverageLTE = zeros(Ntx,length(stationManagement.activeIDsLTE)-1);
    sinrManagement.instantThisPstartedLTE = timeManagement.timeNow;
    sinrManagement.instantTheSINRaverageStartedLTE = timeManagement.timeNow;

    % Find not assigned BRid
    indexNOT = (stationManagement.BRid<=0);

    % Calculate BRidT = vector of BRid in the time domain
    BRidT = ceil(stationManagement.BRid/appParams.NbeaconsF);
    BRidT(indexNOT) = -1;

    % Calculate BRidF = vector of BRid in the frequency domain
    BRidF = mod(stationManagement.BRid-1,appParams.NbeaconsF)+1;
    BRidF(indexNOT) = -1;


    
    for i_tx = 1:Ntx   %对于每一个发射车辆

        % Find BRT and BRF in use by tx vehicle i
        BRTtx = BRidT(stationManagement.transmittingIDsLTE(i_tx));
        BRFtx = BRidF(stationManagement.transmittingIDsLTE(i_tx));
%         i=0;
        % Find neighbors of vehicle i
        indexNeighborOfVehicleTX = find(stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(i_tx),:));

%         stationManagement.Ntxsum = stationManagement.Ntxsum + length(indexNeighborOfVehicleTX);  %对于每个数据包的接收车辆数为一个传输

        for j_neigh = indexNeighborOfVehicleTX          %发射车辆的相邻车辆才会有 接收功率

            % ID rx vehicle  %相邻矩阵的行 为发射端编号，列j_neigh为接收端编号，提取到IDrx
            IDrx = stationManagement.neighborsIDLTE(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(i_tx),j_neigh);  

            % Find BRT in use by rx vehicle j
            BRTrx = BRidT(IDrx);

            %i→j的有效功率 Useful received power by vehicle j
            C = RXpower_MHz_ofLTE(stationManagement.activeIDsLTE==IDrx,stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(i_tx));

            % Initialize interfering power sums vector
            Isums = zeros(appParams.NbeaconsF,1);

            % Interference computation
            % Find other vehicles transmitting in the same subframe of
            % tx vehicle i
            
            if Ntx > 1 % otherwise there is only one transmitter - no interference
                for k = 1:length(stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE)
                    % If interferer is different from tx vehicle i and
                    % different from receiving vehicle j
                    if k~=i_tx && stationManagement.transmittingIDsLTE(k)~=IDrx
                        % Find which BRF is used by the interferer k
                        BRFInt = BRidF(stationManagement.transmittingIDsLTE(k));
                        % Find power from interfering vehicle k receive by receiving vehicle j         
                        I = RXpower_MHz_ofLTE(stationManagement.activeIDsLTE==IDrx,stationManagement.indexInActiveIDsOnlyLTE_OfTxLTE(k));
                        % Isums(BRFInt,1) = Isums(BRFInt,1) + I; %OMA，接收到的功率都作为干扰，干扰大了就会接收失败
                        %根据NOMA -SIC，功率大的会被优先计算，不会成为干扰，所以干扰只能是小的功率之和
                        if I<C   
                            Isums(BRFInt,1) = Isums(BRFInt,1) + I;
                        end
                    end
                end
            end

%             if Isums>C
%                 i=i+1;
%             end
            % Find total interference using IBE
            ItotData = phyParams.IBEmatrixData(BRFtx,:)*Isums;
            ItotControl = phyParams.IBEmatrixControl(BRFtx,:)*Isums;

            % Check if the receiver j is transmitting on the same BRT
            % of transmitter i
            if BRTtx==BRTrx
                % Self-interference自干扰，计算为干扰抑制因子Ksi和发射器i的发射功率的乘积
%                 selfI = phyParams.Ksi*phyParams.P_ERP_MHz_LTE(stationManagement.transmittingIDsLTE(i_tx)); % does not include Gr
                selfI =Inf;       %同一时刻的话则不可以接收，所以将自干扰设置为最大
            else
                % No self-interference
                selfI = 0;
            end

            %% FROM VERSION 5.3.0
            % Interference from 11p, if present
            sinrManagement.neighborsInterfFrom11pAverageLTE(i_tx,j_neigh) = sinrManagement.coex_currentInterfFrom11pToLTE(IDrx);

            % SINR computation
            %SINR(i,j) = C / (PnRB + selfI + Itot);
            sinrManagement.neighPowerUsefulLTE(i_tx,j_neigh) = C * phyParams.BwMHz_lteBR;%有效信号功率，然后用在updateSINRLTE里
            sinrManagement.neighPowerInterfDataLTE(i_tx,j_neigh) = (selfI + ItotData) * phyParams.BwMHz_lteBR; %信号干扰功率
            sinrManagement.neighPowerInterfControlLTE(i_tx,j_neigh) = (selfI + ItotControl) * phyParams.BwMHz_lteBR;  %控制信息干扰功率
                       
%             %% UP TO VERSION 5.2.10
%             % Interference from 11p, if present
%             Ifrom11p = sinrManagement.coex_currentInterfFrom11pToLTE(IDrx);
% 
%             % SINR computation
%             %SINR(i,j) = C / (PnRB + selfI + Itot);
%             sinrManagement.neighPowerUsefulLastLTE(i_tx,j_neigh) = C * phyParams.BwMHz_lteBR;
%             sinrManagement.neighPowerInterfLastLTE(i_tx,j_neigh) = (selfI + Itot) * phyParams.BwMHz_lteBR + Ifrom11p;
        end
    end
stationManagement.Ntxall = 5;
% stationManagement.Blockall = 5;
end