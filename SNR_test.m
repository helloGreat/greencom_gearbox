% MATLAB Script: Calculate SNR and plot user distribution with varying distances
clc;
clear;
close all;

% 参数设置
numUE = 12;              % 用户数
radius_BS = 900;         % 基站覆盖半径 (米)
NRB = 51;                % 资源块数量
SCS = 30;                % 子载波间隔 (kHz)
txpower = 43;            % 基站发射功率 (dBm)

% 调用函数
[SNR, user_angle] = cal_SNR(numUE, radius_BS, NRB, SCS, txpower);

% 打印SNR
disp('信噪比 (SNR)：');
disp(SNR);

% 画出用户分布图
figure;
hold on;

% 基站位置
scatter3(0, 0, 35, 'r', 'filled'); % 基站坐标 (0, 0, 35)
text(0, 0, 35, ' BS', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% 画用户的分布
for i = 1:numUE
    angle = user_angle(i);
    % 用户的X, Y, Z坐标
    radius = radius_BS; % 假设用户距离基站的距离接近基站覆盖半径
    x = radius * cos(angle);
    y = radius * sin(angle);
    z = 1.5; % 用户高度
    
    % 绘制用户点
    scatter3(x, y, z, 'filled');
    text(x, y, z, [' UE' num2str(i)], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('User 3D Distribution Around the Base Station');
grid on;
axis equal;
hold off;

function [SNR,user_angle] = cal_SNR(numUE, radius_BS, NRB, SCS, txpower)
    %所需参数设置
    simParameters.Carrier = nrCarrierConfig;
    simParameters.Carrier.NSizeGrid = NRB;
    simParameters.Carrier.SubcarrierSpacing = SCS;
    simParameters.Carrier.CyclicPrefix = 'Normal';
    simParameters.CarrierFrequency = 3.5e9;
    simParameters.TxHeight = 35;
    simParameters.TxPower = txpower;
    simParameters.RxHeight = 1.5;
    simParameters.RxNoiseFigure = 6;
    simParameters.RxAntTemperature = 290;
    simParameters.PathLossModel = '5G-NR';
    simParameters.PathLoss = nrPathLossConfig;
    simParameters.PathLoss.Scenario = 'UMa';
    simParameters.PathLoss.EnvironmentHeight = 1;
    simParameters.DelayProfile = 'CDL-B';

    if contains(simParameters.DelayProfile,'CDL','IgnoreCase',true)
        channel = nrCDLChannel;
        channel.DelayProfile = simParameters.DelayProfile;
        chInfo = info(channel);
        kFactor = chInfo.KFactorFirstCluster; % dB
    else
        channel = nrTDLChannel;
        channel.DelayProfile = simParameters.DelayProfile;
        chInfo = info(channel);
        kFactor = chInfo.KFactorFirstTap; % dB
    end
    
    simParameters.LOS = kFactor>-Inf;
    waveformInfo = nrOFDMInfo(simParameters.Carrier);
    
    D = radius_BS;
    user_num = numUE;

    % 定义每组的角度范围
    angle_ranges = [0 15; 30 45; 60 75; 90 105]; 
    
    % 为每组设置不同的基准距离
    base_radii = [0.4 * D, 0.6 * D, 0.8 * D, D]; % 不同组的用户参考距离

    group_size = user_num / 4;  % 每组用户的数量
    user_angle = zeros(user_num, 1);
    x_position = zeros(2, user_num);
    rx_position = [];
    
    % 撒点：分组生成用户位置
    for g = 1:4
        % 使用组对应的基准距离
        base_radius = base_radii(g);
        for i = 1:group_size
            idx = (g-1)*group_size + i;
            
            % 在给定角度范围内生成随机角度
            angle = (angle_ranges(g, 1) + (angle_ranges(g, 2) - angle_ranges(g, 1)) * rand()) * pi / 180;
            % 在组内基准距离的基础上生成相近的距离
            radius = base_radius + (rand() - 0.5) * 0.1 * D;  % 距离基准点波动范围设置为基站覆盖范围的 10%
            
            % 生成用户的 x, y 坐标
            x = [radius * cos(angle), radius * sin(angle)];
            x_2 = [x(1); x(2)];
            
            % 更新用户位置和角度
            x_position(:, idx) = x_2;
            user_angle(idx) = angle;
            % 生成 3D 坐标并存储
            x_3 = [abs(x(1)); abs(x(2)); 1.5];
            rx_position = [rx_position, x_3];
        end
    end

    % 计算基站与用户之间的3D距离
    txPosition = [0; 0; simParameters.TxHeight];
    rxPosition = rx_position;
    d3D0 = c_d3D(txPosition, rxPosition);
    simParameters.TxRxDistance = d3D0;

    % 计算路径损耗
    if contains(simParameters.PathLossModel,'5G','IgnoreCase',true)
        pathLoss = nrPathLoss(simParameters.PathLoss, simParameters.CarrierFrequency, simParameters.LOS, txPosition, rxPosition);
    else
        lambda = physconst('LightSpeed')/simParameters.CarrierFrequency;
        pathLoss = fspl(simParameters.TxRxDistance, lambda);
    end

    % 计算信噪比
    kBoltz = physconst('Boltzmann');
    NF = 10^(simParameters.RxNoiseFigure/10);
    Teq = simParameters.RxAntTemperature + 290*(NF-1); % K
    N0 = sqrt(kBoltz*waveformInfo.SampleRate*Teq/2.0);

    fftOccupancy = 12*simParameters.Carrier.NSizeGrid/waveformInfo.Nfft;
    simParameters.SNRIn = (simParameters.TxPower-30) - pathLoss - 10*log10(fftOccupancy) - 10*log10(2*N0^2);
    SNR = simParameters.SNRIn;

    function [d3D] = c_d3D(posBS, posUE)
        persistent c0;
        if isempty(c0)
            c0 = physconst('lightspeed');
        end
        nBS = size(posBS,2);
        nUE = size(posUE,2);
        bs = permute(repmat(posBS,1,1,nUE),[2 3 1]);
        ue = permute(repmat(posUE,1,1,nBS),[3 2 1]);
        d3D  = sqrt((ue(:,:,1)-bs(:,:,1)).^2 + ...
                    (ue(:,:,2)-bs(:,:,2)).^2 + ...
                    (ue(:,:,3)-bs(:,:,3)).^2);
    end
end
