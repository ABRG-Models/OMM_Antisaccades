% Here's example code to show when the pro- or anti-saccade occurred

args=argv();
if (length(args) > 0)
  model_log_path = args{1};
else
  model_log_path = '/home/seb/SpineML_2_BRAHMS/temp/Oculomotor_Model_lumcomp_arith_e15';
end

stim_on = 350;
min_outlier = 180;
SS = csvread ( [model_log_path '/run/saccsim_side.log'], 1 , 0);
eyeRx = SS(:,8);
eyeRy = SS(:,9);
eyeRz = SS(:,10);
eyeTime = SS(:,1);
clear SS;

PS_RT_absolute = min( find( eyeRy > 0 ) );
AS_RT_absolute = min( find( eyeRy < 0 ) );
if ~isempty( PS_RT_absolute )
    PS_RT = PS_RT_absolute - stim_on;
    printf ('Prosaccade response after %d ms\n', PS_RT)
    if PS_RT < min_outlier
        % Anomaly
        display ('That was a fast response...')
    end
end
if ~isempty( AS_RT_absolute )
    AS_RT = AS_RT_absolute - stim_on;
    printf ('Antisaccade response after %d ms\n', AS_RT)
    if AS_RT < min_outlier
        % Anomaly
        display ('That was a fast response...')
    end
end
