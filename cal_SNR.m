function [SNR,user_angle] = cal_SNR(numUE, radius_BS, NRB, SCS, txpower)
    %所需参数设置
    % Configure the carrier frequency, transmitter (BS), receiver (UE), and
    % distance between the BS and UE. Specify this distance as a vector for
    % multiple SNR points.
    simParameters.Carrier = nrCarrierConfig;
    simParameters.Carrier.NSizeGrid = NRB;            % Bandwidth in number of resource blocks (51 RBs at 30 kHz SCS for 20 MHz BW)
    simParameters.Carrier.SubcarrierSpacing = SCS;    % 15, 30, 60, 120, 240 (kHz)
    simParameters.Carrier.CyclicPrefix = 'Normal';   % 'Normal' or 'Extended' (Extended CP is relevant for 60 kHz SCS only)
    simParameters.CarrierFrequency = 3.5e9;   % Carrier frequency (Hz)
    simParameters.TxHeight = 35;              % Height of the BS antenna (m)
    simParameters.TxPower = txpower;               % Power delivered to all antennas of the BS on a fully allocated grid (dBm)
    simParameters.RxHeight = 1.5;             % Height of UE antenna (m)
    simParameters.RxNoiseFigure = 6;          % Noise figure of the UE (dB)
    simParameters.RxAntTemperature = 290;     % Antenna temperature of the UE (K)
    simParameters.PathLossModel = '5G-NR';        % '5G-NR' or 'fspl'
    simParameters.PathLoss = nrPathLossConfig;
    simParameters.PathLoss.Scenario = 'UMa';      % Urban macrocell
    simParameters.PathLoss.EnvironmentHeight = 1; % Average height of the environment in UMa/UMi
    simParameters.DelayProfile = 'CDL-B'; % A, B, and C profiles are NLOS channels. D and E profiles are LOS channels.

    %Configure Fading Channel and LOS
    if contains(simParameters.DelayProfile,'CDL','IgnoreCase',true) %CDL
        channel = nrCDLChannel;
        channel.DelayProfile = simParameters.DelayProfile;
        chInfo = info(channel);
        kFactor = chInfo.KFactorFirstCluster; % dB
    else % TDL
        channel = nrTDLChannel;
        channel.DelayProfile = simParameters.DelayProfile;
        chInfo = info(channel);
        kFactor = chInfo.KFactorFirstTap; % dB
    end
    
    % Determine LOS between Tx and Rx based on Rician factor K.
    simParameters.LOS = kFactor>-Inf;

    % Determine the sample rate and FFT size that are required for this carrier.
    waveformInfo = nrOFDMInfo(simParameters.Carrier);
    
    %在基站覆盖范围（六边形）中进行用户撒点
    D = radius_BS; % 基站覆盖半径/m，可设置(900)
    user_num = numUE;    % 用户数，可设置(12)

    i = 0;
    rx_position = [];
    x_position = zeros(2,user_num);
    user_angle = zeros(user_num,1);
    while i < user_num
        % 生成随机角度和半径
        angle = rand()*2*pi/3; % 在0到120度之间生成随机角度 random_value = (0.75 + (1-0.75) * rand()
        radius = (3/7 + (1-3/7) * rand())*D; % 在0到D之间生成随机半径
        x = [radius * cos(angle), radius * sin(angle)];
        x_2 = [x(1);x(2)];
        % 将点添加到图中
        i = i + 1;
        x_position(:,i) = x_2;
        user_angle(i) = angle;
        x_3 = [abs(x(1)); abs(x(2)); 1.5];
        rx_position = [rx_position, x_3];
    end

    % Calculate the path loss.
    %计算基站与用户之间的3d距离
    txPosition = [0;0; simParameters.TxHeight];
    rxPosition = rx_position;
    d3D0 = c_d3D(txPosition,rxPosition);
    simParameters.TxRxDistance = d3D0;

    %Calculate Propagation Path Loss
    %计算基站与用户之间的传播路径损耗
    if contains(simParameters.PathLossModel,'5G','IgnoreCase',true)
        pathLoss = nrPathLoss(simParameters.PathLoss,simParameters.CarrierFrequency,simParameters.LOS,txPosition,rxPosition);
    else % Free-space path loss
        lambda = physconst('LightSpeed')/simParameters.CarrierFrequency;
        pathLoss = fspl(simParameters.TxRxDistance,lambda);
    end

    %Calculate Antenna and Receiver Noise
    kBoltz = physconst('Boltzmann');
    NF = 10^(simParameters.RxNoiseFigure/10);
    Teq = simParameters.RxAntTemperature + 290*(NF-1); % K
    N0 = sqrt(kBoltz*waveformInfo.SampleRate*Teq/2.0);

    fftOccupancy = 12*simParameters.Carrier.NSizeGrid/waveformInfo.Nfft;
    simParameters.SNRIn = (simParameters.TxPower-30) - pathLoss - 10*log10(fftOccupancy) - 10*log10(2*N0^2);
    SNR = simParameters.SNRIn;

    function [d3D] = c_d3D(posBS,posUE)
    
        persistent c0;
        if isempty(c0)
            c0 = physconst('lightspeed');
        end
   
        % Number of BS and UE
        nBS = size(posBS,2);
        nUE = size(posUE,2);
    
        % Expansion of BS and UE positions for matrix operations
        bs = permute(repmat(posBS,1,1,nUE),[2 3 1]);
        ue = permute(repmat(posUE,1,1,nBS),[3 2 1]);
    
        % 3D distance between BS and UE
        d3D  = sqrt((ue(:,:,1)-bs(:,:,1)).^2 + ...
                    (ue(:,:,2)-bs(:,:,2)).^2 + ...
                    (ue(:,:,3)-bs(:,:,3)).^2);
    end

end