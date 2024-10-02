function Rg = helperGetCovariance(hDp,prm)
% Create spatial covariance matrices Rg for all groups from hDp
% Rg = helperGetCovariance(hDp,prm)
% Inputs: hDp{} - channel matrices per user
%         prm   - parameter structure
% The spatial covariance statistics assumes ergodicity in space and
% frequency, so this will accumulate over all receiver antennas and over
% all subcarriers. There is no temporal averaging.

% Copyright 2023 The MathWorks, Inc.

G = prm.numGroups;
Rg = cell(1,G);
Nt = size(hDp{1},2);

% Get spatial covariance for each group
for g = 1:G
    R = zeros(Nt,Nt); % define the covariance matrix R

    % Loop through all users within the group and calculate R from the
    % channel estimates of all the antennas across all subcarriers for all
    % users in group
    usersInGroup = find(prm.groups == g);
    for u = usersInGroup
        Hu = hDp{u};    % get channel estimates for this user
        Nrx = size(Hu,3);
        activeCarriers = find(all(~isnan(Hu(:,:,1)),2));

        % Accumulate estimates over all antennas for this user
        for r = 1:Nrx
            % Accumulate estimates over all subcarriers
            for sc = activeCarriers
                h = Hu(sc,:,r).';
                R = R + h*h';
            end
        end
    end
    Rg{g} = R / max(R,[],'all'); % normalize to get good rank measurement
end

% [EOF]