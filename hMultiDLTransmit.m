%hMultiDLTransmit Downlink transmitter for MU-MIMO
%   [TXDL,TXSYMBOLS,SINGLELAYERTBS] =
%   hMultiDLTransmit(CARRIER,PDSCHs,BSANTSIZE,B) performs downlink
%   transmission for multiple users, returning the transmitted downlink
%   waveform, TXDL, multi-user set of transmitted PDSCH symbols (used for
%   EVM calculation), TXSYMBOLS, and the transport block size for a
%   single-layer full allocation transmission (used for capacity
%   calculation), SINGLELAYERTBS. CARRIER is the carrier configuration.
%   PDSCHs is a multi-user set of PDSCH configurations. BSANTSIZE is a
%   vector specifying the base station antenna array size. B is an
%   optional array of RF weights applied prior to the antenna array.

% Copyright 2021-2023 The MathWorks, Inc.

function [trblk_ue,txDL,txSymbols,singleLayerTBS] = hMultiDLTransmit(carrier,PDSCHs,bsAntSize,B)

    % Create transmit resource grid
    nTxAnts = prod(bsAntSize);
    dlGrid = nrResourceGrid(carrier,nTxAnts);

    numUEs = numel(PDSCHs);
    txSymbols = cell(1,numUEs);
    singleLayerTBS = NaN;
    trblk_ue = cell(1,numUEs);

    % For each UE
    for ue = 1:numUEs

        % Extract the configuration for this UE
        pdsch = PDSCHs(ue).Config;
        pdschExt = PDSCHs(ue).Extension;

        % Create the PDSCH indices
        [indices,indicesInfo] = nrPDSCHIndices(carrier,pdsch);

        % Calculate transport block size and create transport block
        TBS = nrTBS(pdsch.Modulation,pdsch.NumLayers,numel(pdsch.PRBSet),indicesInfo.NREPerPRB,pdschExt.TargetCodeRate,pdschExt.XOverhead);
        trblk = randi([0 1],TBS,1);
        trblk_ue{ue}=trblk;


        % For the first UE, calculate the single-layer transport block size
        % for a full RB allocation, used for capacity calculation
        if (ue==1)
            singleLayerTBS = nrTBS(pdsch.Modulation,1,carrier.NSizeGrid,indicesInfo.NREPerPRB,pdschExt.TargetCodeRate,pdschExt.XOverhead);
        end

        % Perform DL-SCH encoding
        encodeDLSCH = nrDLSCH;
        encodeDLSCH.TargetCodeRate = pdschExt.TargetCodeRate;
        setTransportBlock(encodeDLSCH,trblk);
        RV = 0;
        cws = encodeDLSCH(pdsch.Modulation,pdsch.NumLayers,indicesInfo.G,RV);

        % Perform PDSCH modulation
        symbols = nrPDSCH(carrier,pdsch,cws);
        txSymbols{ue} = symbols;

        % Apply beamforming and map to the transmit grid
        [antSymbols,antIndices] = hPRGPrecode(size(dlGrid),carrier.NStartGrid,symbols,indices,pdschExt.W);
        dlGrid(antIndices) = dlGrid(antIndices) + antSymbols;

        % Create the PDSCH DM-RS
        dmrsIndices = nrPDSCHDMRSIndices(carrier,pdsch);
        dmrsSymbols = nrPDSCHDMRS(carrier,pdsch);

        % Apply beamforming and map to the transmit grid
        [dmrsAntSymbols,dmrsAntIndices] = hPRGPrecode(size(dlGrid),carrier.NStartGrid,dmrsSymbols,dmrsIndices,pdschExt.W);
        dlGrid(dmrsAntIndices) = dlGrid(dmrsAntIndices) + dmrsAntSymbols;

    end

    % Create the output structure containing the waveform, resource grid,
    % and OFDM information
    txDL = struct();
    txDL.dlWaveform = nrOFDMModulate(carrier,dlGrid); 
    if exist('B','var') && ~isempty(B)
        txDL.dlWaveform = txDL.dlWaveform * B; % apply RF beamforming
    end

    txDL.dlGrid = dlGrid;
    txDL.ofdmInfo = nrOFDMInfo(carrier);

end
