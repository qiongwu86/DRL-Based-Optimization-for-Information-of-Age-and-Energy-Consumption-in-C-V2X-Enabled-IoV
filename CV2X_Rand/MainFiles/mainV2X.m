function [simValues,outputValues,appParams,simParams,phyParams,sinrManagement,outParams,stationManagement] = mainV2X(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement)
% Core function where events are sorted and executed

%% Initialization
[appParams,simParams,phyParams,outParams,simValues,outputValues,...
    sinrManagement,timeManagement,positionManagement,stationManagement] = mainInit(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement);

phyParams.NsubchannelsBeacon    %一个消息占用的子信道数
% The simulation starts at time '0'
timeManagement.timeNow = 0;
timeManagement.timeLast = 0;

% The variable 'timeNextPrint' is used only for printing purposes
timeNextPrint = 0;

% The variable minNextSuperframe is used in the case of coexistence
minNextSuperframe = min(timeManagement.coex_timeNextSuperframe);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Cycle
% The simulation ends when the time exceeds the duration of the simulation
% (not really used, since a break inside the cycle will stop the simulation
% earlier)
% Start stopwatch
tic
fprintf('Simulation Time: ');
reverseStr = '';
fileID = fopen('Rho60-R20-AoI-PHI.txt', 'w+');
powdb = 23;  R = 1;
figure
xdata = [];  % 记录历史x数据的数组
ydata = [];  % 记录历史y数据的数组

L=10;
N = simValues.maxID; 
activeIDs = stationManagement.activeIDs;
H=1;
D=2;
C=3;
M=4;
TH=100;
KH=8;
TD=500;
KD=5;
TC =100;
% customer=zeros(N,100000,2);                              %activeID代表车辆数，1000代表是1000个包，2代表如下2个信息
NO=1;                                                           %/*包编号*/  也是产包的总数
gtime=2;                                                        %/*包的产生时刻*/
repeat=3;                                                       %重复总次数
rept=4;                                                         %重复进队的时刻
lambda = 0.0001;          %0.0001=随机消息在1ms内产生一个包的可能性,平均0.1包/秒
k = 1;
pg1 = exp(-lambda) * lambda^k / factorial(k);
wa=0.6;wb=1-wa;

stationManagement.lt=zeros(N,4);                                                  %/*用一个循环函数选择队列的0<b<lt(i,state)
stationManagement.lt(:,3) = round(100*rand(N,1)/10);
a= stationManagement.lt(:,3);
b = round(TC*rand(N,1));
stationManagement.pckBuffer = zeros(N,L,4);
for i = 1:length(a)
    stationManagement.pckBuffer(i,1:a(i),3) = b(i) + flip(TC*((1:a(i))-1));
end

stationManagement.PHIt = repmat(stationManagement.pckBuffer(:, 1, 3), 1, N);%/*接受端信息年龄*/
stationManagement.PHIt = stationManagement.PHIt + round(TC*rand(N,N));
stationManagement.PHIt(1:N+1:end) = 0;
stationManagement.transmittedIDs = [];

Hnum=zeros(N,1);                                         %给H类型计数
Dnum=zeros(N,1);
Hcustomer=zeros(N,1000,4);
Dcustomer=zeros(N,1000,4);
atnum=zeros(1,N);
st=zeros(N,4);

aoi_consum = zeros(N,1);
ploc = zeros(1,N);
% acon = zeros(1,N);
% Numjsum = zeros(N,1);

stept = zeros(N,1);
steptx = zeros(N,1);
ifreselect = zeros(N,1);
reward = -1*ones(1,N);
episode_reward=0;
episode = 0;
step = 0;

actiondis = R*ones(1,N); %randi([1, 3], 1, N)
actioncon = ones(1,N) ; %rand(1,N);
stationManagement.B = stationManagement.A(actiondis);
stationManagement.RRI= stationManagement.B';

powermax = 10.^((23-30)/10);    %最大功率
phyParams.Ptx_dBm = (powdb+10*log10(actioncon))';      %计算LTE 每MHz的等效辐射功率
phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
phyParams.P_ERP_MHz_LTE = 10.^((phyParams.Ptx_dBm-30)/10);  % 转换为线性单位
power = (10.^((phyParams.Ptx_dBm - 30)/10))/powermax; %*1000就转换为mW,*5就归一化
energy = zeros(1,N);

while timeManagement.timeNow < simParams.simulationTime*1000+1
    % 接收数据 
    if any(ifreselect)  %做出动作
        idx = find(ifreselect==1);
        actiondis = randi([1, 3], 1, N); %R*ones(1,N)
        actioncon = rand(1,N);   % ones(1,N)
        stationManagement.B(idx) = stationManagement.A(actiondis(idx));
        phyParams.Ptx_dBm(idx) = (powdb+10*log10(actioncon(idx)))';      %计算LTE 每MHz的等效辐射功率
        phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
        phyParams.P_ERP_MHz_LTE = 10.^((phyParams.P_ERP_MHz_LTE_dBm-30)/10);  % 转换为线性单位
        power = 10.^((phyParams.Ptx_dBm - 30)/10)/powermax;
        ifreselect = zeros(N,1);
    end

    % If the time instant exceeds or is equal to the duration of the simulation, the simulation is ended
    if round(timeManagement.timeNow*1e10)/1e13>=round((simParams.simulationTime+1)*1e10)/1e10
        break;
    end


    [minTimeValue,indexRow] = min(timeManagement.timeNextEvent);
    [timeEvent,indexCol] = min(minTimeValue,[],2);                    %时间换算，除以一千
    indexEvent = indexRow(indexCol);
    idEvent = activeIDs(indexEvent);
    timeEvent = timeEvent/1000;

    % If the next LTE event is earlier than timeEvent, set the time to the
    % LTE event       占用资源时刻timeManagement.timeNextLTE
    if timeEvent >= timeManagement.timeNextLTE
        timeEvent = timeManagement.timeNextLTE;
    end

    % If the next superframe event (coexistence, method A) is earlier than timeEvent, set the time to the this event
    if timeEvent >= minNextSuperframe
        timeEvent = minNextSuperframe;
    end
        
   % If timeEvent is later than the next CBR update, set the time
    % to the CBR update
%     if timeEvent >= (timeManagement.timeNextCBRupdate-1e-12)
%         timeEvent = timeManagement.timeNextCBRupdate;
%         %fprintf('CBR update%.6f\n',timeEvent);
%     end
        
    % If timeEvent is later than the next position update, set the time to the position update
    % 并且不出现以上三种情况之一则timeEvent被赋值timeManagement.timeNextPosUpdate
    if timeEvent >= (timeManagement.timeNextPosUpdate-1e-9) && ...
        (isempty(stationManagement.activeIDsLTE) || timeEvent > timeManagement.timeNextLTE || timeManagement.subframeLTEstarts==true)
        timeEvent = timeManagement.timeNextPosUpdate;
       
    end
   

    %%
    % Print time to video
    while timeManagement.timeNow/1000>timeNextPrint
        reverseStr = printUpdateToVideo(timeManagement.timeNow/1000,simParams.simulationTime,reverseStr);
        timeNextPrint = timeNextPrint + simParams.positionTimeResolution;
    end

    %% Action
    % The action at timeManagement.timeNow depends on the selected event
    % POSITION UPDATE: positions of vehicles are updated
    if timeEvent==timeManagement.timeNextPosUpdate        
        % DEBUG EVENTS
        %printDebugEvents(timeEvent,'position update',-1);
        if isfield(timeManagement,'subframeLTEstarts') && timeManagement.subframeLTEstarts==false
            % During a position update, some vehicles can enter or exit the scenario; 
            % this is not managed if it happens during one subframe
            error('A position update is occurring during the subframe; not allowed by implementation.');
        end
            
        [appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement] = ...
              mainPositionUpdate(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);
        
        % DEBUG IMAGE
        % printDebugImage('position update',timeManagement,stationManagement,positionManagement,simParams,simValues);

        % Set value of next position update
        timeManagement.timeNextPosUpdate = timeManagement.timeNextPosUpdate+ simParams.positionTimeResolution;
        positionManagement.NposUpdates = positionManagement.NposUpdates+1;

%     elseif timeEvent == timeManagement.timeNextCBRupdate
%         % Part dealing with the channel busy ratio calculation
%         % Done for every station in the system, if the option is active
%         thisSubInterval = mod(ceil((timeEvent-1e-9)/(simParams.cbrSensingInterval/simParams.cbrSensingIntervalDesynchN))-1,simParams.cbrSensingIntervalDesynchN)+1;
%         %
%         % ITS-G5
%         % CBR and DCC (if active)
%         if ~isempty(stationManagement.activeIDs11p)
%             vehiclesToConsider = stationManagement.activeIDs11p(stationManagement.cbr_subinterval(stationManagement.activeIDs11p)==thisSubInterval);        
%             [timeManagement,stationManagement,stationManagement.cbr11pValues(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval))] = ...
%                 cbrUpdate11p(timeManagement,vehiclesToConsider,stationManagement,simParams,phyParams);
%         end
%         % In case of Mitigation method with dynamic slots, also in LTE nodes
%         if simParams.technology==4 && simParams.coexMethod>0 && simParams.coex_slotManagement==2 && simParams.coex_cbrTotVariant==2
%             vehiclesToConsider = stationManagement.activeIDsLTE(stationManagement.cbr_subinterval(stationManagement.activeIDsLTE)==thisSubInterval);
%             [timeManagement,stationManagement,sinrManagement.cbrLTE_coex11ponly(vehiclesToConsider)] = ...
%                 cbrUpdate11p(timeManagement,vehiclesToConsider,stationManagement,simParams,phyParams);
%         end
        
        % LTE-V2X
        % CBR and DCC (if active)
%         if ~isempty(stationManagement.activeIDsLTE)
%             vehiclesToConsider = stationManagement.activeIDsLTE(stationManagement.cbr_subinterval(stationManagement.activeIDsLTE)==thisSubInterval);
%             [timeManagement,stationManagement,sinrManagement,stationManagement.cbrLteValues(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval)),stationManagement.coex_cbrLteOnlyValues(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval))] = ...
%                 cbrUpdateLTE(timeManagement,vehiclesToConsider,stationManagement,sinrManagement,appParams,simParams,phyParams,outParams);
%         end

%         timeManagement.timeNextCBRupdate = timeManagement.timeNextCBRupdate + (simParams.cbrSensingInterval/simParams.cbrSensingIntervalDesynchN);
     
        % CASE LTE ,占用资源时刻
    elseif timeEvent == timeManagement.timeNextLTE
       %即占用资源时刻jt，start代码中的stationManagement.transmittingIDsLTE表示此刻占用资源的车辆编号
        if timeManagement.subframeLTEstarts
            t=timeManagement.timeNow;
%             t3=t3+1;
            for j=1:N
               
                %然后决定at  =======产包间隔=======
%                 pg1=poisspdf(1,0.0001); %0.0001                                           %0.0001=随机消息在1ms内产生一个包的可能性,平均0.1包/秒
                pH=rand(1);
                if pH<pg1
                    atnum(j) = atnum(j) +1;
                    % at(j,H)=1;
%                     totnum(j)=totnum(j)+1;
                    Hnum(j)=Hnum(j)+1;              %车辆j第Hnum个包产生，共Hnum包
%                     customer(j,totnum(j),NO)=totnum(j);
%                     customer(j,totnum(j),gtime)=t;
                    Hcustomer(j,Hnum(j),NO)=Hnum(j);
                    Hcustomer(j,Hnum(j),gtime)=t;
                    Hcustomer(j,Hnum(j),repeat)=KH;
                    Hcustomer(j,Hnum(j),rept)=t+TH;
                    %当前新包产生的情况下去计算下一时刻产包时刻，然后将当前时刻的包看作last包，其中addedToGenerationTime在maininit中赋值为0
                    timeManagement.timeNextPacket(j,H) = Hcustomer(j,Hnum(j),rept);
                    timeManagement.timeLastPacket(j,H) = t;
        %             -timeManagement.addedToGenerationTime(j);
                    if stationManagement.lt(j,H)<L
%                         que_totnum(j)=que_totnum(j)+1;                             %成功进入队列的包+1
                        stationManagement.lt(j,H)=stationManagement.lt(j,H)+1;
                    else
                        [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
        %                                                %bufferOverflowLTE(idEvent,idPacType... 
                    end
%                 else
%                     at(j,H)=0;
                end
        
                logicalIndex = Hcustomer(j,:,repeat) > 0 & t == Hcustomer(j,:,rept);
                repeatIndex = find(logicalIndex);
%                 repeatIndex = find(Hcustomer(j,:,repeat) > 0 & t == Hcustomer(j,:,rept));
                if ~isempty(repeatIndex)
                    atnum(j) = atnum(j) +1;
%                     at(j,H) = 1;
    %                 timeManagement.timeLastPacket(j,H) = t - timeManagement.addedToGenerationTime(j);
                    timeManagement.timeLastPacket(j,H) = t;
                    Hcustomer(j,repeatIndex,repeat) = Hcustomer(j,repeatIndex,repeat) - 1;
                    repeatMask = Hcustomer(j,repeatIndex,repeat) > 0;
                    Hcustomer(j,repeatIndex(repeatMask),rept) = Hcustomer(j,repeatIndex(repeatMask),rept) + TH;
                
%                     totnum(j) = totnum(j) + length(repeatIndex);
%                     customer(j,totnum(j),NO) = totnum(j);
%                     customer(j,totnum(j),gtime) = t;
%                     
%                     que_totnum(j)=que_totnum(j)+sum(stationManagement.lt(j,H) < L);                     %成功进入队列的包+sum(stationManagement.lt(j,H) < L)
                    stationManagement.lt(j,H) = stationManagement.lt(j,H) + sum(stationManagement.lt(j,H) < L);
                
                    bufferOverflowIndex = find(stationManagement.lt(j,H) >= L);
                    if ~isempty(bufferOverflowIndex)
                        [stationManagement, outputValues] = bufferOverflowLTE(idEvent(bufferOverflowIndex), positionManagement(bufferOverflowIndex), stationManagement(bufferOverflowIndex), phyParams(bufferOverflowIndex), outputValues(bufferOverflowIndex), outParams(bufferOverflowIndex));
                    end
                
                    nextPacketIndex = find(Hcustomer(j,:,repeat) > 0);
                    if ~isempty(nextPacketIndex)
                        timeManagement.timeNextPacket(j,H) = min(Hcustomer(j,nextPacketIndex,rept));
                    else
                        timeManagement.timeNextPacket(j,H) = inf;
                    end
                end
                                
        
                pD=rand(1);
                if pD<pg1
                    atnum(j) = atnum(j) +1;
%                     at(j,D)=1;
%                     totnum(j)=totnum(j)+1;
                    Dnum(j)=Dnum(j)+1;
%                     customer(j,totnum(j),NO)=totnum(j);
%                     customer(j,totnum(j),gtime)=t;
                    Dcustomer(j,Dnum(j),NO)=Dnum(j);
                    Dcustomer(j,Dnum(j),gtime)=t;
                    Dcustomer(j,Dnum(j),repeat)=KD;
                    Dcustomer(j,Dnum(j),rept)=t+TD;
                    timeManagement.timeNextPacket(j,D) = Dcustomer(j,Dnum(j),rept);
                    timeManagement.timeLastPacket(j,D) = t;
                    if stationManagement.lt(j,D)<L
%                         que_totnum(j)=que_totnum(j)+1;                             %成功进入队列的包+1
                        stationManagement.lt(j,D)=stationManagement.lt(j,D)+1;
                    else
                        [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
            %                                                  %bufferOverflowLTE(idEvent,idPacType... 
                    end
%                 else
%                     at(j,D)=0;
                end
    
                repeatIndex = find(Dcustomer(j,:,repeat) > 0 & t == Dcustomer(j,:,rept));
                if ~isempty(repeatIndex)
                    atnum(j) = atnum(j) +1;      %   at(j,D) = 1;
    %                 timeManagement.timeLastPacket(j,D) = t - timeManagement.addedToGenerationTime(j);
                    timeManagement.timeLastPacket(j,D) = t;
                    Dcustomer(j,repeatIndex,repeat) = Dcustomer(j,repeatIndex,repeat) - 1;
                    repeatMask = Dcustomer(j,repeatIndex,repeat) > 0;
                    Dcustomer(j,repeatIndex(repeatMask),rept) = Dcustomer(j,repeatIndex(repeatMask),rept) + TD;
                
%                     totnum(j) = totnum(j) + length(repeatIndex);
%                     customer(j,totnum(j),NO) = totnum(j);
%                     customer(j,totnum(j),gtime) = t;
%     
%                     que_totnum(j)=que_totnum(j)+sum(stationManagement.lt(j,D) < L);                     %成功进入队列的包+sum(lt(j,D) < L)
                    stationManagement.lt(j,D) = stationManagement.lt(j,D) + sum(stationManagement.lt(j,D) < L);
                
                    bufferOverflowIndex = find(stationManagement.lt(j,D) >= L);
                    if ~isempty(bufferOverflowIndex)
                        [stationManagement, outputValues] = bufferOverflowLTE(idEvent(bufferOverflowIndex), positionManagement(bufferOverflowIndex), stationManagement(bufferOverflowIndex), phyParams(bufferOverflowIndex), outputValues(bufferOverflowIndex), outParams(bufferOverflowIndex));
                    end
                
                    nextPacketIndex = find(Dcustomer(j,:,repeat) > 0);
                    if ~isempty(nextPacketIndex)
                        timeManagement.timeNextPacket(j,D) = min(Dcustomer(j,nextPacketIndex,rept));
                    else
                        timeManagement.timeNextPacket(j,D) = inf;
                    end
                end
    
            
                if t == timeManagement.timeNextPacket(j,C) 
                    atnum(j) = atnum(j) +1;
%                     at(j,C)=1;
%                     totnum(j)=totnum(j)+1;
%                     customer(j,totnum(j),NO)=totnum(j);
%                     customer(j,totnum(j),gtime)=t;
    %                 timeManagement.timeNextPacket(j,C) = t+max(timeManagement.generationInterval(idEvent),timeManagement.dcc_minInterval(idEvent));
                    timeManagement.timeNextPacket(j,C) = t+TC;
        %             timeManagement.timeLastPacket(j,C) = t-timeManagement.addedToGenerationTime(j);
                    timeManagement.timeLastPacket(j,C) = t;
                    if stationManagement.lt(j,C)<L
%                         que_totnum(j)=que_totnum(j)+1;                                        
                        stationManagement.lt(j,C)=stationManagement.lt(j,C)+1;
                    else
                        [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
                    end
%                 else
%                     at(j,C)=0;
                end
            
                pM=rand(1);
                if pM<pg1
                    atnum(j) = atnum(j) +1;
%                     at(j,M)=1;
%                     totnum(j)=totnum(j)+1;
%                     customer(j,totnum(j),NO)=totnum(j);
%                     customer(j,totnum(j),gtime)=t;
                    timeManagement.timeNextPacket(j,M) =  inf;
                    timeManagement.timeLastPacket(j,M) = t;
                    if stationManagement.lt(j,M)<L
%                         que_totnum(j)=que_totnum(j)+1;                             
                        stationManagement.lt(j,M)=stationManagement.lt(j,M)+1;
                    else
                        [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
                    end
%                 else
%                     at(j,M)=0;
                end      
            end%for j

            qt = stationManagement.lt > 0;
        
            [sinrManagement,stationManagement,timeManagement,outputValues] = ...
                mainLTEsubframeStarts(appParams,phyParams,timeManagement,sinrManagement,stationManagement,simParams,simValues,outParams,outputValues);

            transmittingIDs = stationManagement.transmittingIDsLTE;
            for j = 1:length(transmittingIDs)
                id = transmittingIDs(j);
                st(id,find(qt(id,:) == 1, 1))=1; 
%                 s(id) = s(id)+1;%在start中已经排除队列为空的情况，所以s1<=t/RRI          
            end

            timeManagement.subframeLTEstarts = false;%传输完成后变成子帧结束subframeLTEend
            timeManagement.timeNextLTE = timeManagement.timeNextLTE + (phyParams.Tsf - phyParams.TsfGap);  %进入下一子帧
%             t1=t1+1;
             

        else    
            [phyParams,simValues,outputValues,sinrManagement,stationManagement,timeManagement] = ...
                mainLTEsubframeEnds(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);
            
            stationManagement.transmittedIDs = stationManagement.transmittingIDsLTE;
            timeManagement.subframeLTEstarts = true;
            timeManagement.timeNextLTE = timeManagement.timeNextLTE + phyParams.TsfGap;  %加一个小差值
            timeManagement.timeNextLTE = round(timeManagement.timeNextLTE*1e10)/1e10; %去抖动
            timeManagement.timeNow = round(timeManagement.timeNextLTE*1000);
 
%             t2=t2+1;

            %start(18)设置队列非空才算发射
%             transmittingIDs = stationManagement.transmittingIDsLTE;
            [correctMatrix,stateavg] = findCorrect(transmittingIDs,transmittingIDs,stationManagement.neighborsIDLTE,sinrManagement,stationManagement,positionManagement,phyParams);
            CorrectMatrixRawMax=correctMatrix;
            
            ut = zeros(N,N);    %ut=1代表当前发射且通信成功的情况，st代表当前发射的是哪一个队列
%             Powerate = zeros(N,1);
            for j = 1:length(CorrectMatrixRawMax(:,1))   
                ut(CorrectMatrixRawMax(j,1),CorrectMatrixRawMax(j,2))=1;
            end

            distanceReal = positionManagement.distanceReal;
            rawThreshold = phyParams.Raw;%范围内车辆才能算aoi，可注释phyParams.RawMaxLTE
            noAboveThreshold = distanceReal <= rawThreshold;
            Numj = sum(noAboveThreshold, 2)-1;
%             cont = ut.*noAboveThreshold;

            for j = 1:N               %计算接收端aoi
                ut_j = ut(j,:);
                st_j = st(j,:);
                isUt1 = ut_j == 1;
                isSt1 = any(st_j == 1);
                stationManagement.PHIt(j, :) = stationManagement.PHIt(j, :) + 1;
                stationManagement.PHIt(j, isUt1 & isSt1) = stationManagement.pckBuffer(j, 1, find(st_j == 1, 1)) + stateavg(j,3);
                stationManagement.PHIt(j, j) = 0;
            end
            noAboveThreshold = distanceReal <= phyParams.Raw;
            stationManagement.PHIt = stationManagement.PHIt.*noAboveThreshold;
            %ut = ut.*noAboveThreshold;
            %nutrate = 1-(sum(ut,2) ./ Numj);
            %nutrate(isnan(nutrate)) = 0;

           %% 时隙变化时向Python发送数据，用于训练
            % resReselectionCounterLTE = stationManagement.resReselectionCounterLTE;    
            %steput < ReCounterLTE0 ,因为队列中没有包的时候是不会发射的，即不会产生功耗
            stept=stept+1;
%             aoi_loct = (sum(sum(stationManagement.pckBuffer(:,:,:), 3), 2) / 40)';
            aoi_cont = sum(stationManagement.PHIt,2)./Numj/1000;
            aoi_cont(isnan(aoi_cont)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
            aoi_consum = aoi_consum + aoi_cont; % /N
            if ~isempty(transmittingIDs)
                ifreselect(transmittingIDs) = (stationManagement.resReselectionCounterLTE(transmittingIDs) == 1);
                steptx(transmittingIDs) = steptx(transmittingIDs) + 1;
            end

            %acon = acon + nutrate';
            if any(ifreselect)
                idx = find(ifreselect==1);
                RCrate = (steptx /75)';   %  归一化./stept*1000后1s内最多传输50次： 总功耗为pt*RCt0，但是RCt0经历的时间不同(RRI*λ)，所以需要归一化
                aoi_conavg = aoi_consum(idx)./stept(idx); % /N
                aoi_conavg(isnan(aoi_conavg)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
                reward(idx) = - (wa*power(idx)' .* RCrate(idx) + wb*aoi_conavg') ;
                energy(idx) = energy(idx) + power(idx)' .* RCrate(idx);
                reward_nan = any(isnan(reward));
                if sum(reward_nan)>0
                    disp('The reward contains NaN values.');
                end   

                stept(idx) = 0;
                steptx(idx) = 0; 
                ploc(idx) = 0; 
                aoi_consum(idx) = 0;
                atnum(idx) = 0;

                episode_reward = episode_reward +sum(reward(idx));
                step = step +length(idx);
                if step > 9
                    xdata = [xdata,episode];
                    ydata = [ydata,episode_reward/step];    
                    episode_reward = 0;
                    step =0;
                    episode = episode + 1;
                    if mod(episode,5) == 0 
                        plot(xdata, ydata); % 绘制曲线图     
                        drawnow;% 让图形刷新
                    end
                end
            end
            
            lt = stationManagement.lt;
            for j = 1:N    %将队首传输以后更新包的位置和队列长度 
                for k = 1:4
                    stjk = st(j,k);
                    if stjk == 0
                        ltvalue = lt(j,k);
                        stationManagement.pckBuffer(j,1:ltvalue,k) = stationManagement.pckBuffer(j,1:ltvalue,k) + 1;  
                    elseif stjk == 1
                        ltvalue = lt(j,k);
                        stationManagement.pckBuffer(j,1:ltvalue-1,k) = stationManagement.pckBuffer(j,2:ltvalue,k) + 1;
                        if lt(j,k) > 0
                            stationManagement.pckBuffer(j,ltvalue,k) = 0;
                        end
                        stationManagement.lt(j,k) = ltvalue - stjk;
                    end
                end
            end

             %% 测试
            if any(ifreselect)
                totPHItavg = mean(stationManagement.PHIt(:))/1000; % 计算 接收端totPHItavg        
                locsum = nnz(stationManagement.pckBuffer);
                if locsum > 0
                    AoIloc = mean(stationManagement.pckBuffer(:))/1000;        %计算当前时刻系统所有车本地 队列中 数据包的AoI之和
                else
                    AoIloc = 0;
                end
                pow = mean(energy(idx));
                pow(isnan(pow)) = 0;
%                 if ~mod(t,fs) && t>0
                    fprintf(fileID, '%f\n', totPHItavg);
                    fprintf(fileID, '%f\n', AoIloc);
                    fprintf(fileID, '%f\n', pow);
%                 end
                energy(idx) = 0;
            end
            st=zeros(N,4);
        end     
        
        printDebugGeneration(timeManagement,idEvent,positionManagement,outParams);     
        if simParams.technology==4 && simParams.coexMethod==1 && simParams.coexA_improvements>0
            timeManagement = coexistenceImprovements(timeManagement,idEvent,stationManagement,simParams,phyParams);
        end

    end     % The next event is selected as the minimum of all values in 'timeNextPacket'
       
    


    timeManagement.timeNextEvent = timeManagement.timeNextPacket;
    timeNextEvent = min(timeManagement.timeNextEvent(:));
    if timeNextEvent < timeManagement.timeNow-1e-8 % error check
        format long
        fprintf('next=%f, now=%f\n',min(timeManagement.timeNextEvent(:)),timeManagement.timeNow);
        error('An event is schedule in the past...');
    end  
end %While

fclose(fileID);
% Print end of simulation
msg = sprintf('%.1f / %.1fs',simParams.simulationTime,simParams.simulationTime);
fprintf([reverseStr, msg]);

% Number of position updates
simValues.snapshots = positionManagement.NposUpdates;

% Stop stopwatch
outputValues.computationTime = toc;

end
