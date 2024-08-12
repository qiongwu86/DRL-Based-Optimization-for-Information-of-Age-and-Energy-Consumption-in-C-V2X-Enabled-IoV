% 1. 加载txt文件数据
data = load('AoI-Pow0.6-100-test.txt');
totPHItavg_ind= 1:3:length(data);
totPHItavg = data(totPHItavg_ind);

AoIloc_ind= 2:3:length(data);
AoIloc = data(AoIloc_ind);

Energy_ind= 3:3:length(data);300
Energy = data(Energy_ind);

a=mean(totPHItavg)
b=mean(AoIloc)
c=mean(Energy) %传输次数被除以过75.由此此处可以乘回来
% 2. 绘制折线图
figure(1)
plot(totPHItavg);
title('totPHItavg');
xlabel('X');
ylabel('Y');

figure(2)
plot(AoIloc);
title('AoIloc');
xlabel('X');
ylabel('Y');

figure(3)
plot(Energy);
title('Energy');
xlabel('X');
ylabel('Y');
