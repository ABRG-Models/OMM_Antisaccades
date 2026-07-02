% Get response times from a path which should be a directory containing the results of many simulation runs
function [ AS_inds PS_inds ] = getRTs( path )

    mypwd = pwd; % Save initial directory location for a return at the end
    cd( path )
    listdir = dir                      % gets all files & folders, inc hidden
    fnames = { listdir.name };          % Cell array with names of all
    include_these = '^[^.]+';           % set criterion for inclusion: doesn't begin with '.'
    keep = regexp( fnames, include_these ); % binary cell array indexing files meeting inclusion criteria
    lose = cellfun( 'isempty', keep );  % array indexing files to exclude
    listdir.name( ( lose == 1 ), : ) = [];   % remove excluded files from listdir

    stim_on = 350;
    min_outlier = 180;
    storePS_RTs = ones( size( listdir, 1 ), 1 ) .* NaN;
    storeAS_RTs = ones( size( listdir, 1 ), 1 ) .* NaN;
    anomalies = [];

    for i = 1:size( listdir, 1 )

        % get eye movement data
        current_folder = listdir( i ).name;
        %disp(current_folder)
        cd( current_folder )
        SS = csvread ( 'saccsim_side.log', 1 , 0);
        eyeRx = SS(:,8);
        eyeRy = SS(:,9);
        eyeRz = SS(:,10);
        eyeTime = SS(:,1);
        clear SS;

        PS_RT = min( find( eyeRy > 0 ) );
        AS_RT = min( find( eyeRy < 0 ) );
        if ~isempty( PS_RT )
	    storePS_RTs( i ) = PS_RT - stim_on;
        end
        if ~isempty( AS_RT )
	    storeAS_RTs( i ) = AS_RT - stim_on;
        end

        if ( PS_RT - stim_on ) < min_outlier || ( AS_RT - stim_on ) < min_outlier
	    anomalies = [ anomalies; current_folder ];
        end

        cd ..

    end

    PS_trials = find( ~isnan( storePS_RTs ) );
    PS_trial_locs = [];
    if ~isempty( PS_trials )
        for i = 1:length( PS_trials )
            PS_trial_locs = [ PS_trial_locs; listdir( PS_trials( i ) ).name ];
        end
        %    disp( 'PS trials:' )
        %    disp( PS_trial_locs )
    else disp( 'No PS trials' )
    end

    AS_trials = find( ~isnan( storeAS_RTs ) );
    AS_trial_locs = [];
    if ~isempty( AS_trials )
        for i = 1:length( AS_trials )
            AS_trial_locs = [ AS_trial_locs; listdir( AS_trials( i ) ).name ];
        end
        %    disp( 'AS trials:' )
        %    disp( AS_trial_locs )
    else disp( 'No AS trials' )
    end

    storePS_RTs( isnan( storePS_RTs ) ) = [];
    storeAS_RTs( isnan( storeAS_RTs ) ) = [];
    if ~isempty( storePS_RTs )
        totPS = length( storePS_RTs );
        meanPS_RT = mean( storePS_RTs );
        medPS_RT = median( storePS_RTs );
        PSsd = std( storePS_RTs );
        PSrange = max( storePS_RTs ) - min( storePS_RTs );
        disp( [ 'mean PS RT = ', num2str( meanPS_RT ) ] )
        disp( [ 'median PS RT = ', num2str( medPS_RT ) ] )
        disp( [ 'PS SD = ', num2str( PSsd ) ] )
        disp( [ 'PS range = ', num2str( PSrange ) ] )
        disp( [ 'total PS responses = ' num2str( totPS ) ] )
    end
    if ~isempty( storeAS_RTs )
        totAS = length( storeAS_RTs );
        meanAS_RT = mean( storeAS_RTs );
        medAS_RT = median( storeAS_RTs );
        ASsd = std( storeAS_RTs );
        ASrange = max( storeAS_RTs ) - min( storeAS_RTs );
        disp( [ 'mean AS RT = ', num2str( meanAS_RT ) ] )
        disp( [ 'median AS RT = ', num2str( medAS_RT ) ] )
        disp( [ 'AS SD = ', num2str( ASsd ) ] )
        disp( [ 'AS range = ', num2str( ASrange ) ] )
        disp( [ 'total AS responses = ' num2str( totAS ) ] )
    end

    if sum( ~isnan( anomalies ) ) > 0
        #    disp( [ 'Anomalous responses on trial/s: ' ] )
        #    disp( anomalies )
    end


    % get arrays of trial numbers:
    % Antisaccades
    AS_inds = ones( 1, size( AS_trial_locs, 1 ) ) .* NaN;
    if ~isempty( AS_trial_locs )
	for i = 1:length( AS_inds )
	        if strcmp( AS_trial_locs( i, 5 ), '_' )
		        trial_ind = str2num( AS_trial_locs( i, 4 ) );
			AS_inds( i ) = trial_ind;
		elseif strcmp( AS_trial_locs( i, 6 ), '_' )
		        trial_ind = str2num( AS_trial_locs( i, 4:5 ) );
			AS_inds( i ) = trial_ind;
		elseif strcmp( AS_trial_locs( i, 7 ), '_' )
		        trial_ind = str2num( AS_trial_locs( i, 4:6 ) );
			AS_inds( i ) = trial_ind;
		end
	end
	AS_inds = sort( AS_inds );
    end

    % Prosaccades
    PS_inds = ones( 1, size( PS_trial_locs, 1 ) ) .* NaN;
    if ~isempty( PS_trial_locs )
	for i = 1:length( PS_inds )
	        if strcmp( PS_trial_locs( i, 5 ), '_' )
		        trial_ind = str2num( PS_trial_locs( i, 4 ) );
			PS_inds( i ) = trial_ind;
		elseif strcmp( PS_trial_locs( i, 6 ), '_' )
		        trial_ind = str2num( PS_trial_locs( i, 4:5 ) );
			PS_inds( i ) = trial_ind;
		elseif strcmp( PS_trial_locs( i, 7 ), '_' )
		        trial_ind = str2num( PS_trial_locs( i, 4:6 ) );
			PS_inds( i ) = trial_ind;
		end
	end
	PS_inds = sort( PS_inds );
    end

    figure( 20 )
    clf
    subplot( 2, 1, 1 )
    if ~isempty( storePS_RTs )
        hist( storePS_RTs, 20 )
        title( 'Prosaccade RTs' )
    end
    subplot( 2, 1, 2 )
    if ~isempty( storeAS_RTs )
        hist( storeAS_RTs, 20 )
        title( 'Antisaccade RTs' )
    end

    % 'Return home' (Needed?)
    cd (mypwd);
end
