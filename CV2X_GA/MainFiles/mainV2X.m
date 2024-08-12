function [simValues,outputValues,appParams,simParams,phyParams,sinrManagement,outParams,stationManagement] = mainV2X(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement)
% Core function where events are sorted and executed

%% Initialization
[appParams,simParams,phyParams,outParams,simValues,outputValues,...
    sinrManagement,timeManagement,positionManagement,stationManagement] = mainInit(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement);


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
fileID = fopen('GALTE100-4000.txt', 'w+');%20：0.9-0.4 21：1-0.5
L=10;
N = simValues.maxID; 
activeIDs = stationManagement.activeIDs;

% 遗传算法的参数设置
populationSize = N; % 种群大小
chromosomeLength = 10; % 染色体长度
mutationRate = 0.1; % 变异率
crossoverRate = 0.8; % 交叉率
% maxGenerations = 20; % 最大迭代次数
elitismSize = N/10;
tournamentSize = 3;
Generation = 1;
population = randi([0, 1], populationSize, chromosomeLength);%ones(populationSize, chromosomeLength);
for i = 1:populationSize
    if population(i, 1:2) == [1, 1]
        while isequal(population(i, 1:2), [1, 1])  % 确保不是11
            population(i, 1:2) = [0,0];
        end
    end
end
X = decodeChromosome(population);
% fileIDGA = fopen('GAsolution-NR40.txt', 'w+');
% fprintf(fileIDGA, '%d %d %d\n', solution1(1), solution1(2), objectiveValue1);
% fclose(fileIDGA);

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
wa=0.5;wb=1-wa;
energy = zeros(1,N);
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
Hcustomer=zeros(N,100,4);
Dcustomer=zeros(N,100,4);
atnum=zeros(1,N);
st=zeros(N,4);
aoi_consum = zeros(N,1);
stept = zeros(N,1);
steptx = zeros(N,1);  consum = zeros(N,1);  aoiloc = zeros(N,1);
ifreselect = zeros(N,1); 
reward = -1*ones(1,N);
Fval1 = zeros(1,N);
Fvalnum = zeros(1,N);
fitness =-1*ones(1,N);
gene=zeros(N,1);

actiondis = X(:,1)+1;
actioncon = X(:,2)';
stationManagement.B = stationManagement.A(actiondis);
stationManagement.RRI= stationManagement.B';
stationManagement.subframeT2Mode4 = stationManagement.RRI*1000;
stationManagement.averageTbeacon = stationManagement.RRI;
stationManagement.NbeaconsT = floor(stationManagement.averageTbeacon./phyParams.Tsf);
stationManagement.Nbeacons = appParams.NbeaconsF.*stationManagement.NbeaconsT;
[stationManagement.BRid,~] = BRreassignmentRandom(activeIDs,simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams);
stationManagement.BRid = stationManagement.BRid(:);

powermax = 10.^((23-30)/10);    %最大功率
phyParams.Ptx_dBm = (23+10*log10(actioncon))';      %计算LTE 每MHz的等效辐射功率
phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
phyParams.P_ERP_MHz_LTE = 10.^((phyParams.Ptx_dBm-30)/10);  % 转换为线性单位
power = (10.^((phyParams.Ptx_dBm - 30)/10))/powermax; %*1000就转换为mW,*5就归一化
% while Generation <= maxGenerations 
while timeManagement.timeNow < simParams.simulationTime*1000+1
    % 接收数据 
    if any(ifreselect)  %做出动作
%         decodedPopulation = decodeChromosome(mutatedPopulation); %bestpopall            
        idx = find(ifreselect==1);
        actiondis = X(:,1)+1;
        actioncon = X(:,2)';
        stationManagement.B(idx) = stationManagement.A(actiondis(idx));%
        phyParams.Ptx_dBm(idx) = (23+10*log10(actioncon(idx)))';      %计算LTE 每MHz的等效辐射功率
        phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
        phyParams.P_ERP_MHz_LTE = 10.^((phyParams.P_ERP_MHz_LTE_dBm-30)/10);  % 转换为线性单位
        power = 10.^((phyParams.Ptx_dBm - 30)/10)/powermax;
        ifreselect = zeros(N,1);
        gene(idx) = 1;
    end

    if gene==ones(N,1)
        fitness = Fval1./Fvalnum;
        [bestFitness,~] = max(fitness);
        disp(['Generation: ', num2str(Generation), ', Best Fitness: ', num2str(bestFitness)]);
        [~, idx] = sort(fitness, 'descend');
        tops_idx = idx(1:elitismSize);
        lastpops =  population(tops_idx,:);
        % 选择
        % selectedPopulation = selection(population, fitness);   
        selectedPopulation = tournamentSelection(population, fitness,tournamentSize);
        % 交叉
        offspringPopulation = crossover(selectedPopulation, crossoverRate,elitismSize);    
        % 变异
        mutatedPopulation = mutation(offspringPopulation, mutationRate);
        % 替换当前种群
        population = mutatedPopulation;
        replace_idx = randperm(N, elitismSize);
        population(replace_idx, :) = lastpops(1:elitismSize, :);
        X = decodeChromosome(population);
        Generation = Generation + 1;
        gene=zeros(N,1);
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
            at = zeros(N,4);
            for j=1:N
               
                %然后决定at  =======产包间隔=======
%                 pg1=poisspdf(1,0.0001); %0.0001                                           %0.0001=随机消息在1ms内产生一个包的可能性,平均0.1包/秒
                pH=rand(1);
                if pH<pg1
                    atnum(j) = atnum(j) +1;
                    at(j,H)=1;
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
                    at(j,H) = 1;
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
                    at(j,D)=1;
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
                    atnum(j) = atnum(j) +1;      
                    at(j,D) = 1;
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
                    at(j,C)=1;
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
                    at(j,M)=1;
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
%             noAboveThreshold = distanceReal <= phyParams.Raw;
            stationManagement.PHIt = stationManagement.PHIt.*noAboveThreshold;
  
            %% 时隙变化时向Python发送数据，用于训练
            % resReselectionCounterLTE = stationManagement.resReselectionCounterLTE;    
            %steput < ReCounterLTE0 ,因为队列中没有包的时候是不会发射的，即不会产生功耗
            stept = stept + 1;
            aoi_cont = sum(stationManagement.PHIt,2)./Numj/1000;
            aoi_cont(isnan(aoi_cont)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
            aoi_consum = aoi_consum + aoi_cont; % /N

            if ~isempty(transmittingIDs)
                ifreselect(transmittingIDs) = (stationManagement.resReselectionCounterLTE(transmittingIDs) == 1);
                steptx(transmittingIDs) = steptx(transmittingIDs) + 1;

                for i = 1:length(transmittingIDs)
                    id = transmittingIDs(i);
                    comp = sum(ut(id,:)>0)/Numj(id);
                    consum(id) = consum(id) + comp; 
                    locsum = nnz(stationManagement.pckBuffer(id,:,:));
                    if locsum > 0
                        aoiloc(id) = aoiloc(id) + mean(mean(stationManagement.pckBuffer(id,:,:)))/1000;        %计算当前时刻系统所有车本地 队列中 数据包的AoI之和
                    else
                        aoiloc(id) = aoiloc(id) + 0;
                    end
                end
            end

            if any(ifreselect)
                idx = find(ifreselect==1);
                RCrate = (steptx /75)';  
                Powrate = power(idx)' .* RCrate(idx); 
                energy(idx) = energy(idx) + Powrate;
                aoi_conavg = aoi_consum(idx)./stept(idx); % /N
                aoi_conavg(isnan(aoi_conavg)) = 0;
%                 aoi_conavg = 1-consum(idx)./steptx(idx); % % /N
%                 aoilocavg = aoiloc(idx)./steptx(idx);
%                 reward(idx) = - (Powrate + aoi_conavg' + aoilocavg')/3 ;
                reward(idx) = - (wa*Powrate + wb*aoi_conavg') ;
                reward(power==0)=-1;

                steptx(idx) = 0;
                stept(idx) = 0;
                aoi_consum(idx) = 0;
                consum(idx) = 0;
                aoiloc(idx) = 0;
                atnum(idx) = 0;
                Fval1(idx) = reward(idx);
                Fvalnum(idx) = 1;
            end

            lt = stationManagement.lt;
            for j = 1:N    %将队首传输以后更新包的位置和队列长度 
                for k = 1:4
                    stjk = st(j,k);
                    ltvalue = lt(j,k);
                    if stjk == 0  
%                         stationManagement.pckBuffer(j,1:ltvalue,k) = stationManagement.pckBuffer(j,1:ltvalue,k) + 1;  
                        for i = 1:ltvalue
                            stationManagement.pckBuffer(j,i,k) = stationManagement.pckBuffer(j,i,k) + 1;
                        end
                    elseif stjk == 1    
                        if ltvalue > 1
                            stationManagement.pckBuffer(j,1:ltvalue-1,k) = stationManagement.pckBuffer(j,2:ltvalue,k) + 1;    
                        end
                        stationManagement.pckBuffer(j,ltvalue,k) = 0;
                        stationManagement.lt(j,k) = ltvalue - stjk;
                    end
                end
            end

            %%测试
            if any(ifreselect)
                idx = find(ifreselect == 1);
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


% 关闭与python的连接
%clear t;

% Print end of simulation
msg = sprintf('%.1f / %.1fs',simParams.simulationTime,simParams.simulationTime);
fprintf([reverseStr, msg]);

% Number of position updates
simValues.snapshots = positionManagement.NposUpdates;

% Stop stopwatch
outputValues.computationTime = toc;

stationManagement.pckBuffer
end


% 解码染色体，将二进制编码转换为实际值
function decodedPopulation = decodeChromosome(population)
    decodedPopulation = zeros(size(population, 1), 2);
    decodedPopulation(:, 1) = round(population(:, 1:2) * [2; 1]);
    decodedPopulation(:, 2) = binaryToDecimal(population(:, 3:end)) / (2^(size(population, 2)-2)-1);
end

% 二进制编码转十进制
function decimalValue = binaryToDecimal(binaryValue)
    [~, n] = size(binaryValue);
    binaryValue = fliplr(binaryValue);
    powersOfTwo = 2.^(0:n-1);
    decimalValue = sum(binaryValue .* powersOfTwo, 2);
end

% % 计算适应度的函数
% function fitness = calculateFitness(population)
%     % TODO: 根据问题定义计算适应度
%     % 这里假设适应度是染色体中所有基因值的总和
%     fitness = sum(population, 2);
% end

% 选择操作
function selectedPopulation = selection(population, fitness)
    % 根据适应度进行轮盘赌选择
    fitnessSum = sum(fitness);
    if fitnessSum == 0
        selectionProbabilities = ones(size(fitness)) / numel(fitness);
    else
        selectionProbabilities = fitness / fitnessSum;
    end
%     selectionProbabilities = fitness / fitnessSum;
    selectedIndices = rouletteWheelSelection(selectionProbabilities, numel(fitness));
    selectedPopulation = population(selectedIndices, :);
end

% 轮盘赌选择的一个简化和优化版本
function selectedIndices = rouletteWheelSelection(selectionProbabilities, numSelections)
    cumulativeProbabilities = cumsum(selectionProbabilities);
    randNums = rand(numSelections, 1);
    selectedIndices = zeros(numSelections, 1);
    for i = 1:numSelections
        % 对于每个随机数，找到对应的区间
        index = find(randNums(i) <= cumulativeProbabilities, 1, 'first');
        selectedIndices(i) = index;
    end
end

function selectedPopulation = tournamentSelection(population, fitness, tournamentSize)
    populationSize = size(population, 1);
    selectedPopulation = zeros(populationSize, size(population, 2));
    
    for i = 1:populationSize
        % 随机选择锦标赛参与者
        participants = randperm(populationSize, tournamentSize);
        % 从参与者中选择适应度最高的个体进入下一代种群
        [~, winnerIndex] = max(fitness(participants));
        selectedPopulation(i, :) = population(participants(winnerIndex), :);
    end
end



% 交叉操作
function offspringPopulation = crossover(selectedPopulation, crossoverRate, elitismSize)
    offspringPopulation = selectedPopulation;
    numPairs = size(selectedPopulation, 1) / 2;
    
    % 染色体拆分为前两位和后八位
    firstPartLength = 2; % 前两位长度    
    for pair = 1:numPairs  
        % 对前两位染色体确定交叉点并进行交叉操作
        if rand() < crossoverRate
            crossoverPoint = randi([1, firstPartLength]);
            temp = offspringPopulation(pair*2-1, 1:crossoverPoint);
            offspringPopulation(pair*2-1, 1:crossoverPoint) = offspringPopulation(pair*2, 1:crossoverPoint);
            offspringPopulation(pair*2, 1:crossoverPoint) = temp;
        end
        
        % 对后八位染色体确定交叉点并进行交叉操作
        if rand() < crossoverRate
            crossoverPoint = randi([firstPartLength+1, size(selectedPopulation, 2)]);
            temp = offspringPopulation(pair*2-1, crossoverPoint:end);
            offspringPopulation(pair*2-1, crossoverPoint:end) = selectedPopulation(pair*2, crossoverPoint:end);
            offspringPopulation(pair*2, crossoverPoint:end) = temp;
        end
    end
end

% 变异操作
function mutatedPopulation = mutation(offspringPopulation, mutationRate)
    % TODO: 根据变异率进行变异操作，可以使用位翻转、插入删除等
    % 这里使用位翻转
    mutatedPopulation = offspringPopulation;
    mutationMask = rand(size(offspringPopulation)) < mutationRate;
    mutatedPopulation(mutationMask) = ~offspringPopulation(mutationMask);
    %剔除11情况
    for i = 1:size(mutatedPopulation, 1)
        if isequal(mutatedPopulation(i, 1:2), [1, 1])
            % 随机生成00、01或者10
            newPrefix = randi([0, 1], 1, 2);
            while isequal(newPrefix, [1, 1])  % 确保不是11
                newPrefix = [0,1];
            end
            mutatedPopulation(i, 1:2) = newPrefix;
%             mutatedPopulation(i, 1:2) = [0,0];
        end
    end
end