%hMultiDLReceive Downlink receiver for MU-MIMO
%   [TBS,CRC,EQSYMBOLS] = hMultiDLReceive(CARRIER,PDSCHs,RX,ALG) performs
%   downlink reception for a multi-user set of received waveforms,
%   returning multi-user sets of transport block sizes, TBS, cyclic
%   redundancy check values, CRC, and equalized symbols EQSYMBOLS. CARRIER
%   is the carrier configuration. PDSCHs is a multi-user set of PDSCH
%   configurations. RX is a multi-user set of received waveforms. ALG is a
%   structure containing algorithmic options.

% Copyright 2021-2022 The MathWorks, Inc.

function [rxblk_ue,TBS,CRC,eqSymbols,rxSymbols] = hMultiDLReceive(carrier,PDSCHs,rx,alg)

    numUEs = numel(rx);
    TBS = cell(1,numUEs);
    CRC = cell(1,numUEs);
    eqSymbols = cell(1,numUEs);
    rxSymbols = cell(1,numUEs);
    rxblk_ue = cell(1,numUEs);

    % For each UE
    for ue = 1:numUEs

        % Extract the configuration for this UE
        pdsch = PDSCHs(ue).Config;
        pdschExt = PDSCHs(ue).Extension;

        % Perform OFDM demodulation (for ChannelFiltering = true)
        rxGrid = rx(ue).rxGrid;
        offset = rx(ue).ChannelFilterDelay;
        if (isempty(rxGrid))
            rxWaveform = rx(ue).rxWaveform;
            rxWaveform = rxWaveform(1+offset:end,:);
            rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
        end

        % Perform channel and noise estimation
        if (alg.PerfectChannelEstimator)

            H = nrPerfectChannelEstimate(carrier,rx(ue).pathGains,rx(ue).pathFilters,offset,rx(ue).sampleTimes);
            noiseGrid = nrOFDMDemodulate(carrier,rx(ue).noise(1+offset:end,:));
            nVar = var(noiseGrid(:));

        else

            % Create DM-RS symbols and indices
            dmrsIndices = nrPDSCHDMRSIndices(carrier,pdsch);
            dmrsSymbols = nrPDSCHDMRS(carrier,pdsch);
            
            [H,nVar] = hSubbandChannelEstimate(carrier,rxGrid,dmrsIndices,dmrsSymbols,pdschExt.PRGBundleSize,'CDMLengths',[2 2]);

            % Average noise estimate across PRGs and layers
            nVar = mean(nVar,'all');

        end

        % Create PDSCH indices and extract allocated PDSCH REs in the
        % received grid and channel estimation
        [pdschIndices,indicesInfo] = nrPDSCHIndices(carrier,pdsch);
        [pdschRx,pdschH,~,pdschHIndices] = nrExtractResources(pdschIndices,rxGrid,H);

        % If perfect channel estimation is configured, the channel
        % estimates must be precoded so that they are w.r.t. layers rather
        % than transmit antennas
        if (alg.PerfectChannelEstimator)
            pdschH = hPRGPrecode(size(H),carrier.NStartGrid,pdschH,pdschHIndices,permute(pdschExt.W,[2 1 3]));
        end

        % Perform equalization
        [eqSymbols{ue},csi] = nrEqualizeMMSE(pdschRx,pdschH,nVar);

        % Perform PDSCH demodulation
        [cws,rxSymbols_value] = nrPDSCHDecode(carrier,pdsch,eqSymbols{ue},nVar);
        rxSymbols{ue}=rxSymbols_value;

        % Apply CSI to demodulated codewords
        csi = nrLayerDemap(csi);
        for c = 1:pdsch.NumCodewords
            Qm = length(cws{c}) / length(rxSymbols_value{c});
            csi{c} = repmat(csi{c}.',Qm,1);
            cws{c} = cws{c} .* csi{c}(:);
        end

        % Perform DL-SCH decoding
        decodeDLSCH = nrDLSCHDecoder();
        decodeDLSCH.TargetCodeRate = pdschExt.TargetCodeRate;
        decodeDLSCH.LDPCDecodingAlgorithm = 'Normalized min-sum';
        decodeDLSCH.MaximumLDPCIterationCount = 6;
        TBS{ue} = nrTBS(pdsch.Modulation,pdsch.NumLayers,numel(pdsch.PRBSet),indicesInfo.NREPerPRB,pdschExt.TargetCodeRate,pdschExt.XOverhead);
        decodeDLSCH.TransportBlockLength = TBS{ue};
        RV = 0;
        [rxblk,CRC{ue}] = decodeDLSCH(cws,pdsch.Modulation,pdsch.NumLayers,RV);
        rxblk_ue{ue} = rxblk; 

    end

end