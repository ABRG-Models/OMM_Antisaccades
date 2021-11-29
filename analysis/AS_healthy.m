% Antisaccdes healthy is e5
model_log_path = '/home/seb/SpineML_2_BRAHMS/temp/Oculomotor_Model_lumcomp_arith_e15/log/';

[worldout, count] = load_sc_data ([model_log_path 'World_out_log.bin'], 2500);
worldout = reshape (worldout, 50, 50, []);
[fefin, count] = load_sc_data ([model_log_path 'FEF_in_log.bin'], 2500);
fefin = reshape (fefin, 50, 50, []);
[fefout, count] = load_sc_data ([model_log_path 'FEF_out_log.bin'], 2500);
fefout = reshape (fefout, 50, 50, []);

% You an alternatively specify the logrep.xml file instead of the bin file; load_sc_data doesn't mind which:
[lipfef, count] = load_sc_data ([model_log_path 'LIP_to_FEF_Synapse_0_postsynapse_out_logrep.xml'], 2500);
lipfef = reshape (lipfef, 50, 50, []);

[pfcstn, count] = load_sc_data ([model_log_path 'PFC_to_STN_Synapse_0_postsynapse_out_log.bin'], 2500);
pfcstn = reshape (pfcstn, 50, 50, []);

[snr, count] = load_sc_data ([model_log_path 'SNr_out_log.bin'], 2500);
snr = reshape (snr, 50, 50, []);

[pfcfef, count] = load_sc_data ([model_log_path 'PFC_to_FEF_Synapse_0_postsynapse_out_log.bin'], 2500);
pfcfef = reshape (pfcfef, 50, 50, []);

% The view aximuth and elevation
% Flat view:
%v=[180,90];
% A 3D view
v=[30,35];

t = t+5

figure(1);
surf (worldout(:,:,t));
view(v); title('World')

figure(2);
surf (fefin(:,:,t));
view(v); title('FEF in')

figure(3);
surf (fefout(:,:,t));
view(v); title('FEF out')

figure(4);
surf (lipfef(:,:,t));
view(v); title('LIP to FEF')

figure(5);
surf (pfcstn(:,:,t));
view(v); title('PFC to STN')

figure(6)
surf (pfcfef(:,:,t));
view(v); title('PFC to FEF')

figure(7);
surf (snr(:,:,t));
view(v); title('SNr')
