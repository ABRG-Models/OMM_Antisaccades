% Example code to load and plot a surface from a 2500 element
% population in the OMM_Antisaccade model.

% Set the path to the log directory

model_log_path = '/home/seb/SpineML_2_BRAHMS/temp/Oculomotor_Model_lumcomp_arith_e6/log/';

% Use the load_sc_data function to extract data from the logs. To
% access the function, which is provided as part of the SpineCreator
% source code, add /path/to/scsrc/SpineCreator/analysis_utils/matlab
% to your matlab or octave path. If you are using GNU Octave, add
% something like this to your .octaverc:
% addpath('/home/seb/src/SpineCreator/analysis_utils/matlab');

[fefout, count] = load_sc_data ([model_log_path 'FEF_out_log.bin'], 2500);

% fefout is 2500x1000 - 2500 in a 50x50 sheet and 1000 time steps. We
% need to reshape it:

fefout = reshape (fefout, 50, 50, []);

% Now let's plot one timepoint as a sheet

t = 450;

surf (fefout(:,:,t));
