%hMultiUserChannels CDL channel configurations for MU-MIMO
%   CHANNELS =
%   hMultiUserChannels(PROFILE,SPREAD,DOPPLER,BSANTSIZE,UEANTSIZES,GROUPS)
%   creates a multi-user set of CDL channel configurations, CHANNELS.
%   PROFILE is the delay profile (CDL-A...CDL-E). SPREAD is the channel
%   delay spread in seconds. DOPPLER is the maximum Doppler shift in Hz.
%   BSANTSIZE is a vector specifying the base station antenna array size.
%   UEANTSIZES is a matrix specifying the UE antenna array sizes. GROUPS is
%   an optional vector of group assignments for each UE.

% Copyright 2021-2023 The MathWorks, Inc.

function channels = hMultiUserChannels(delayProfile,delaySpread,maximumDopplerShift,bsAntSize,ueAntSizes,groups)

    % Create empty output structures
    numUEs = size(ueAntSizes,1);
    channels = repmat(struct('channel',[],'chInfo',[],'pathFilters',[]),1,numUEs);

    % Create a CDL channel model object configured with the desired delay
    % profile, delay spread and Doppler frequency
    channel = nrCDLChannel;
    channel.DelayProfile = delayProfile;
    channel.DelaySpread = delaySpread;
    channel.MaximumDopplerShift = maximumDopplerShift;

    % Set the base station antenna array size. Initially, the channel
    % operates in the DL direction, therefore the transmit antenna array
    % corresponds to the base station, while the receive antenna array
    % corresponds to the UE
    channel.TransmitAntennaArray.Size = [bsAntSize 1 1];

    % Configure channel filtering:
    % * For ChannelFiltering = true, the transmit resource grid will be
    %   OFDM modulated, filtered by the channel impulse repsonse, and
    %   OFDM demodulated to produce the receive resource grid
    % * For ChannelFiltering = false, the channel will be applied to the
    %   transmit resource grid in the frequency domain to produce the
    %   receive resource grid
    % ChannelFiltering = false is faster, at the expense of not modelling
    % channel variation (and loss of orthogonality) due to Doppler across
    % the duration of each OFDM symbol
    channel.ChannelFiltering = true;
    
    channel.OutputDataType = "single";
    % Configure a set of azimuth and zenith angle offsets, used to adjust
    % the angles of departure of the channel for each UE. This simulates
    % the effect of different UEs being in different locations in the
    % environment around the base station
    if ~exist('groups','var')
        groups = ones(1,numUEs);
    end
    G = max(groups);
    if G == 1
        % For a group size of one, scatter UEs throughout the 120 degree
        % wide sector. Azimuth offsets: offset values are spread between
        % (-60,60)
        offsetsAoD = (rand([1 numUEs])-0.5) * 120;
    
        % Elevation offsets: assume a tower height of 30m above the UE and
        % cell radius of 600m, with UEs between 150m and 600m from the cell
        % centre
        range = 150 + (rand([1 numUEs]) * 450);
        offsetsZoD = atand(30 ./ range);
    else
        % If more than one group is specified, arrange groups to be
        % equidistantly separated in azimuth within a 120 degree span, then
        % scatter users within their respective groups
        groupsAoD = -60 + 60/(G+1) + (120-(120/(G+1)))/(G-1) * (0:G-1);
        groupRange = 300 * ones(1,G);
        offsetsAoD = zeros(1,numUEs);
        offsetsZoD = zeros(1,numUEs);
        for g = 1:G
            offsetsAoD(groups==g) = ...
                groupsAoD(g) + 8*rand(sum(groups==g),1);
            offsetsZoD(groups==g) = ...
                atand(30 ./ (groupRange(g) + 30*rand(sum(groups==g),1)));
        end
    end

    % For each UE
    for ue = 1:numUEs

        % Create a copy of the original channel
        cdl = hMakeCustomCDL(channel);

        % Set the UE antenna array size
        cdl.ReceiveAntennaArray.Size = [ueAntSizes(ue,:) 1 1];

        % Configure the channel seed based on the UE number
        % (results in independent fading for each UE)
        cdl.Seed = 73 + (ue - 1);

        % Configure the azimuth and zenith angle offsets for this UE
        cdl.AnglesAoD(:) = cdl.AnglesAoD(:) + offsetsAoD(ue);
        cdl.AnglesZoD(:) = cdl.AnglesZoD(:) + offsetsZoD(ue);

        % Record the channel object and channel information in the output
        channels(ue).channel = cdl;
        channels(ue).chInfo = info(cdl);

    end

end
