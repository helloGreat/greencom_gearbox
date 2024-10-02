clear;

% 按照功率分配 张克勤：43; 张思乐：46； 付：49；贾：52；陈：55 

%测试用
% tx_power=49; %10W, 20W, 40W, 80W, 160W, 320W;
% BW = [20, 40];
% SCS = 30;
% DAC_resolution = 12;
% numRFs = [32,64];
% numAntennas=192;
% numStream=16;

%分布式
tx_power= 46;% 20W, 40W, 80W, 160W, 320W;
% BW = [20, 40, 60, 80, 100];
BW = 100;
SCS = 30;
% DAC_resolution = [4,6,8,10,12,14];
DAC_resolution = 10;
% numRFs = 16:8:64;
numRFs =64;
numAntennas=192;
numStream=16;

[Tx, Bw, Scs, Dac, Rf] = ndgrid(tx_power, BW, SCS, DAC_resolution, numRFs);
[data_rate, total_power,aclr_concated] = arrayfun(@(tx, bw, scs, dac, rf) ...
    gearbox(tx, bw, scs, dac, rf, numAntennas, numStream), ...
    Tx, Bw, Scs, Dac, Rf);

% 将结果整理为表格
results = table(Tx(:), Bw(:), Scs(:), Dac(:), Rf(:), aclr_concated(:),data_rate(:), total_power(:), ...
    'VariableNames', {'TxPower', 'Bandwidth', 'SCS', 'DACResolution', 'NumRFs','ACLR' ,'DataRate', 'TotalPower'});

% % 绘制散点图
% scatter(results.DataRate, results.TotalPower, 36, 'b', 'filled');  % 36 是点的大小，'b' 是蓝色，'filled' 表示实心
% 
% % 设置坐标轴标签
% xlabel('Data Rate (bps)');
% ylabel('Total Power (W)');
% 
% % 设置标题
% title('Data Rate vs Total Power');
% 
% % 显示网格
% grid on;



