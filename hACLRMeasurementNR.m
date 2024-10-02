%hACLRMeasurementNR NR ACLR measurement
%   [ACLRdB,SIGNALPOWERdBm] = hACLRMeasurementNR(ACLRCFG,WAVEFORM) measures
%   the NR ACLR of a WAVEFORM given the parameters in the structure
%   ACLRCFG. The result ACLRdB is a vector of ACLR values in dBs in
%   neighboring channels. SIGNALPOWERdBm is the main channel signal power in
%   dBm.

%   Copyright 2019-2022 The MathWorks, Inc.

function [aclrdB,signalPowerdBm] = hACLRMeasurementNR(aclrCfg,waveform)

    % Measurement bandwidth is BWConfig for downlink ACLR (Section 6.6.3.2
    % in TS 38.104), BWConfig = NRB*SCS*12 (Section 3.2 TS 38.104).
    % BWConfig is divisible by 15 kHz, so maximum delta_f = 15 kHz
    delta_f = 15e3;
    Ndft = aclrCfg.SamplingRate/delta_f;

    % Calculate multNbinsChannelInt, the number of DFT bins between adjacent
    % channel center frequencies, increasing the number of DFT bins if
    % necessary to make multNbinsChannelInt an integer.
    gcdBW = gcd(aclrCfg.SamplingRate,aclrCfg.Bandwidth);
    NdftTemp = aclrCfg.SamplingRate/gcdBW;
    multNbinsChannelInt = NdftTemp/gcd(Ndft,NdftTemp);
    Ndft = Ndft*multNbinsChannelInt;

    % Comm ACPR object
    acpr = comm.ACPR;
    acpr.SampleRate = aclrCfg.SamplingRate;
    acpr.MainChannelFrequency = 0;
    acpr.MainMeasurementBandwidth = aclrCfg.MeasurementBandwidth;
    acpr.AdjacentChannelOffset = aclrCfg.AdjacentChannelOffset;
    acpr.AdjacentMeasurementBandwidth = aclrCfg.MeasurementBandwidth;
    acpr.SpectralEstimation = "Specify window parameters";

    acpr.SegmentLength = Ndft;
    acpr.Window = "Blackman-Harris";
    acpr.FFTLength = "Same as segment length";
    acpr.MainChannelPowerOutputPort = true;

    [acprOut,signalPowerdBm] = acpr(waveform);
    aclrdB = -acprOut;

end