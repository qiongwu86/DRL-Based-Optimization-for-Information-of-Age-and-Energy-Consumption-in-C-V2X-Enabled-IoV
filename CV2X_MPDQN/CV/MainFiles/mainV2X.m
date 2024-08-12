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
fileID = fopen('LTE100test-2000.txt', 'a'); wa=0.6;wb=1-wa;
L=10;
% PHIt = stationManagement.PHIt;
t=0;
N = simValues.maxID; 

st=zeros(N,4);
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


fs=1;                                                       %采样频率
plottime=simParams.simulationTime*1000/fs;
% x=zeros(1,plottime);
% y=zeros(N,plottime);
z=zeros(1,plottime);
w=zeros(1,plottime);
% e = 1e-12;
dataReceived = false;
activeIDs = stationManagement.activeIDs;

% 创建TCP/IP对象  
try
    tcp = tcpclient('localhost', 8889);  % 连接到Python程序的TCP服务器
    disp('成功建立连接');
catch ME
    disp(['连接异常: ', ME.message]);
    return;
end

% P_ERP_MHz_LTE_dBm_Max = (23 + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
powermax = 10.^((23-30)/10);    %最大功率

while timeManagement.timeNow < simParams.simulationTime*1000+1
    % 接收数据
    while ~dataReceived
        receivedData = read(tcp);        % 从 Python 接收数据
        if ~isempty(receivedData)
            decoded_str = native2unicode(receivedData);  %在这里处理初始化
            if strcmp(decoded_str, 'reset')     %会导致未来的包早于当前时刻  
                % distanceReal = positionManagement.distanceReal;
                stationManagement.lt=zeros(N,4);                                                  %/*用一个循环函数选择队列的0<b<lt(i,state)
                stationManagement.lt(:,3) = 10*ones(N,1) ;  % round(100*rand(N,1)/10)
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
% 
%                 Poweratet = zeros(1,N);
%                 distanceratet = zeros(1,N);
                ploc = zeros(1,N);
                aloc = zeros(1,N);
                acon = zeros(1,N);
                suscon = zeros(N,1);
                Numjsum = zeros(N,1);
                aoi_consum = zeros(N,1);
%                 conratet = zeros(N,1);
%                 lts = zeros(N,1);
                statu1 = zeros(N,1);
                statu2 = zeros(N,1);
                statu3 = zeros(N,1);
                statu4 = zeros(N,1);
                stept = zeros(N,1);
                steptx = zeros(N,1);
                ifreselect = zeros(N,1);

                statu = zeros(N,6);
                distanceReal = positionManagement.distanceReal;
                rawThreshold = phyParams.Raw;%范围内车辆才能算aoi，可注释phyParams.RawMaxLTE
                noAboveThreshold = distanceReal <= rawThreshold;
                Numj = sum(noAboveThreshold, 2)-1;
                statu(:, 3) = Numj/N ;
                num_above_zero = sum(stationManagement.lt,2); 
                statu(:, 4) = num_above_zero.*ifreselect/40;      %队列长度比
                % statu = round(100*rand(N,5))/100;
                state = reshape(statu.', 1, []);
                initialState = mat2str(state);  % 然后返回初始状态
                %发射功率都为0，aoi也都为0，根据算式，reward=-（0.5*1+0.5*1）
                energy = zeros(1,N);
                reward = -1*ones(1,N);
%                 rewardavg = reward;
                initialreward = mat2str(reward);
%                 rewardt = zeros(1,N);
                % write(tcp,initialState)
                % write(tcp,initialreward)

                write(tcp, [initialState, ',', initialreward]);
                reset = 1;
                continue;  % 跳过当前迭代，从头开始下一次循环 
            end
            str = strrep(decoded_str, '[', '');% 转换动作为数值类型
            str = strrep(str, ']', '');
            action = str2double(strsplit(str, ', '));

            act_nan = any(isnan(action));
            if act_nan
                disp('The data contains NaN values.');
            end

            %action数据类型转换，将第一个位置的整数取出作为RRI和action后三个位置的索引，从而选择RRI和Ptx
            actiondis = uint8(action(1: N)+1); %matlab索引从1开始
            actioncon = action(N+1:N*2);
            actioncon(actioncon<0) = 0;
            stationManagement.B = stationManagement.A(actiondis);
            stationManagement.RRI= stationManagement.B';

%             phyParams.Ptx_dBm = 23*actioncon;
            phyParams.Ptx_dBm = (23+10*log10(actioncon))';      %计算LTE 每MHz的等效辐射功率
            phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_lteBR);
            phyParams.P_ERP_MHz_LTE = 10.^((phyParams.P_ERP_MHz_LTE_dBm-30)/10);  % 转换为线性单位
            %1solt内mW数，*1000mW/1000ms
            power = (10.^((phyParams.Ptx_dBm - 30)/10))/powermax; %功率W,*1000就转换为mW,/3600/1000是1ms内的功耗mWh

            if reset == 1
                stationManagement.subframeT2Mode4 = stationManagement.RRI*1000;
                stationManagement.averageTbeacon = stationManagement.RRI;
                stationManagement.NbeaconsT = floor(stationManagement.averageTbeacon./phyParams.Tsf);
                stationManagement.Nbeacons = appParams.NbeaconsF.*stationManagement.NbeaconsT;
                [stationManagement.BRid,~] = BRreassignmentRandom(activeIDs,simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams);
                stationManagement.BRid = stationManagement.BRid(:);
                reset = 0;
            end
    
            %ifreselect =0...
            ifreselect = zeros(N,1);
            dataReceived = true;              % 设置数据接收完成标志
        end
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
%             ut = ut.*noAboveThreshold;
%             nutrate = 1-(sum(ut,2) ./ Numj);   %未通信率   stateavg(j,3)
%             nutrate(isnan(nutrate)) = 0;

            %% 时隙变化时向Python发送数据，用于训练
            % resReselectionCounterLTE = stationManagement.resReselectionCounterLTE;    
            %steput < ReCounterLTE0 ,因为队列中没有包的时候是不会发射的，即不会产生功耗
            stept=stept+1;
            Numjsum = Numjsum + Numj;
            statu4 = statu4 + sum(qt>0,2)/4;

%             aoi_con = aoi_con + stateavg(:,3);
%             aoi_con = aoi_con + sum(stationManagement.PHIt,2)./Numj/1000;
%             aoi_con(isnan(aoi_con)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
            powert = zeros(1,N);
%             aoi_loct = (sum(sum(stationManagement.pckBuffer(:,:,:), 3), 2) / 40)';
            aoi_cont = sum(stationManagement.PHIt,2)./Numj/1000;
            aoi_cont(isnan(aoi_cont)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
            aoi_consum = aoi_consum + aoi_cont; % /N
            
            if ~isempty(transmittingIDs)
                ifreselect(transmittingIDs) = (stationManagement.resReselectionCounterLTE(transmittingIDs) == 1);
                steptx(transmittingIDs) = steptx(transmittingIDs) + 1;
                statu1 = statu1 + stateavg(:,1);
                statu2 = statu2 + stateavg(:,2);
                powert(transmittingIDs) = power(transmittingIDs);
                ploc = ploc + powert;
%                 distancein = noAboveThreshold.*distanceReal;
%                 disrate = 1-max(distancein(transmittingIDs,:),[],2)/phyParams.Raw;
%                 conratet(transmittingIDs) = conratet(transmittingIDs) + disrate.*nutrate(transmittingIDs);  
            end
           
%             acon = acon + (stateavg(:,3))';
%             acon = acon + nutrate';
%             ploc = ploc + powert;
%             aloc = aloc + aoi_loct;

            if any(ifreselect)
%                 ReCounterLTE0 = stationManagement.ReCounterLTE0;
                idx = find(ifreselect==1);
                for i =1:length(idx)
                    if steptx(idx(i))>0
                        statu(idx(i),1) = min(1,statu1(idx(i))./ steptx(idx(i)));    %每次传输的成功通信比                        
                        statu(idx(i),2) = statu2(idx(i))./phyParams.RawMaxLTE/steptx(idx(i));   
                        statu(idx(i),3) = Numjsum(idx(i))/N ./stept(idx(i));             %范围内车辆数
                    else
                        statu(idx(i),1) = 0;            
                        statu(idx(i),2) = 0;
                        statu(idx(i),3) = 0;
                    end
                end
%                 statu(:, 3) = stationManagement.RRI;
                statu(:, 4) = statu3/40./stept;      %队列长度比
%                 statu(:, 4) = statu4./stept;      %队列有包比
%                 statu(:, 4) = aloc ./ stept';
                statu(:, 5) = steptx ./ 75;  %100
                statu(:, 6) = ifreselect;
                state_nan = any(isnan(statu));
                if sum(state_nan)>0
                    disp('The statu contains NaN values.');
                end

%                 aconrate = 1-((stept - acon') .* (stept ./ steptx) )'; %归一化到每次传输，和power同步，最大值为T（RCt）*（RCt→0的时间/RCt），约为1000
%                 aconrate(isnan(aconrate)) = 0;
%                 ploct = ploc ./ 75; %75=powermax*RCt0max，即最大功耗
%                 ploct = ploc ./ steptx';  % /0.2
%                 ploct(isnan(ploct)) = 0;
                RCrate = (steptx /75)';   %  归一化./stept*1000后1s内最多传输50次： 总功耗为pt*RCt0，但是RCt0经历的时间不同(RRI*λ)，所以需要归一化
                
%                 rou = atnum ./ steptx';   %rou = λ/μ=（到达数/stept）/（服务数/stept）
%                 rou(isnan(rou)) = 0;
%                 rouabs = abs(rou - 1); 
%                 aconrate = 1-((stept - acon') ./ steptx)'; %未传输率
%                 aconrate(isnan(aconrate)) = 0;

                aoi_conavg = aoi_consum(idx)./stept(idx); % /N
                aoi_conavg(isnan(aoi_conavg)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
%                 reward(idx) = - (wa*power(idx) .* RCrate(idx) + wb*aconrate(idx) + wb*rouabs(idx) ) ;
                reward(idx) = - (wa*power(idx)' .* RCrate(idx) + wb*aoi_conavg') ;
                energy(idx) = energy(idx) + power(idx)' .* RCrate(idx); 
                
                reward_nan = any(isnan(reward));
                if sum(reward_nan)>0
                    disp('The reward contains NaN values.');
                end

%                 rewardavg(idx) = mean(reward);
                state = reshape(statu.', 1, []);
                nextState = mat2str(state);  % 计算下一个状态
                rewardstr = mat2str(reward);   % 接收端总aoi（可以作为奖励的一部分） 
                write(tcp,nextState)
                write(tcp,rewardstr)
                % write(tcp, [nextState, ',', reward]);

                 %下一时刻要重新选择资源，所以该时刻将状态和奖励传输后就可以不保留      
%                 PHIrateavgt(idx) = 0;
%                 Poweratet(idx) = 0;
                statu1(idx) = 0;
                statu2(idx) = 0;
                statu3(idx) = 0;
                statu4(idx) = 0;
                stept(idx) = 0;
                steptx(idx) = 0; 
                suscon(idx) = 0;
                aoi_consum(idx) = 0;
                Numjsum(idx) = 0;
                ploc(idx) = 0; 
                acon(idx) = 0;
                aloc(idx) = 0;
                atnum(idx) = 0;
%                 conratet(idx) = 0;
%                 lts(idx) = 0;
%                 ifreselect = zeros(N,1);
                dataReceived = false;
                
            end
            
            lt = stationManagement.lt;
            for j = 1:N    %将队首传输以后更新包的位置和队列长度 
                for k = 1:4
                    stjk = st(j,k);
                    ltvalue = lt(j,k);
                    if stjk == 0
                        for i = 1:ltvalue
                            stationManagement.pckBuffer(j,i,k) = stationManagement.pckBuffer(j,i,k) + 1;  
                        end
                    elseif stjk == 1                       
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


% 关闭与python的连接
clear t;

% Print end of simulation
msg = sprintf('%.1f / %.1fs',simParams.simulationTime,simParams.simulationTime);
fprintf([reverseStr, msg]);

% Number of position updates
simValues.snapshots = positionManagement.NposUpdates;

% Stop stopwatch
outputValues.computationTime = toc;


% %保存数据
% fid=fopen('E:\Code\Mat\LTEV2Vsim_v5.4\V70-Tc100-0.1-OMA.txt','wt');    %写入文件路径
% matrix=z;
% %input_matrix为待输出矩阵
% [m,n]=size(matrix);
% for i=1:1:m
%     for j=1:1:n
%         if j==n
%             fprintf(fid, '%g\n',matrix(i,j));
%         else
%             fprintf(fid,' %g\t',matrix(i,j));
%         end
%     end
% end
% fclose(fid);
% 
% 
% fid=fopen('E:\Code\Mat\LTEV2Vsim_v5.4\V70-Tc100-0.1.txt','wt');    %写入文件路径
% matrix=w;
% %input_matrix为待输出矩阵
% [m,n]=size(matrix);
% for i=1:1:m
%     for j=1:1:n
%         if j==n
%             fprintf(fid, '%g\n',matrix(i,j));
%         else
%             fprintf(fid,' %g\t',matrix(i,j));
%         end
%     end
% end
% fclose(fid);

% x
% figure(1)
% plot(x);
% figure(2)
% plot(z); 
% figure(3)
% plot(w); 
stationManagement.pckBuffer
% s1
% s2
% s
% que_totnum
% t1
% t2
% t3
end
