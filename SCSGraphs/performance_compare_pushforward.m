clear all ;
close all;
clc;

%% 4 steps

% base_file = 'AtlasBaseForwardPushAnalysis.mat';
% toe_file = 'AtlasToeForwardPushAnalysis.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis_2.mat';
% toe_file_2 = 'AtlasToeForwardPushAnalysis_2.mat';

%% 3 Steps

% base_file = 'AtlasBaseForwardPushAnalysis3Steps.mat';
% toe_file = 'AtlasToeForwardPushAnalysis3Steps.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis3Steps_2.mat';
% toe_file_2 = 'AtlasToeForwardPushAnalysis3Steps_2.mat';

%% 5 Steps
% 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeForwardPushAnalysis5Steps.mat';
% 7

% base_file = 'AtlasBaseSlowWalk.mat';
% toe_file = 'AtlasToeSlowWalk.mat';
% 
% base_file_2 = 'AtlasBaseSlowWalk_2.mat';
% toe_file_2 = 'AtlasToeSlowWalk_2.mat';
% 
% base_file_3 = 'AtlasBaseSlowWalk_3.mat';
% toe_file_3 = 'AtlasToeSlowWalk_3.mat';

%% Natual Toe Joint Comparison
% 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeNaturalBehavior.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasToeNaturalBehavior_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasToeNaturalBehavior_3.mat';

%% Cinder Block Walking Comparison Comparison
% 
% base_file = 'AtlasBaseCinderBlock.mat';
% toe_file = 'AtlasToeCinderBlock.mat';
% 
% base_file_2 = 'AtlasBaseCinderBlock_2.mat';
% toe_file_2 = 'AtlasToeCinderBlock_2.mat';
% 
% base_file_3 = 'AtlasBaseCinderBlock_3.mat';
% toe_file_3 = 'AtlasToeCinderBlock_3.mat';

%% 5 New Toe Steps
% 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasNewToe5Steps.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasNewToe5Steps_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasNewToe5Steps_3.mat';
% 
% base_file_4 = 'AtlasBaseForwardPushAnalysis5Steps_4.mat';
% toe_file_4 = 'AtlasNewToe5Steps_4.mat';

%% 5 Steps at 0.75 polygon
% % 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeFinal_05_075.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasToeFinal_05_075_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasToeFinal_05_075_3.mat';
% 
% base_file_4 = 'AtlasBaseForwardPushAnalysis5Steps_4.mat';
% toe_file_4 = 'AtlasToeFinal_05_075_4.mat';
%% 5 Steps at 0.75 polygon with new mass

% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeFinal_05_075_change.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasToeFinal_05_075_change_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasToeFinal_05_075_change_3.mat';
% 
% base_file_4 = 'AtlasBaseForwardPushAnalysis5Steps_4.mat';
% toe_file_4 = 'AtlasToeFinal_05_075_change_4.mat';

%% 5 Steps at 0.75 polygon for both BASE AND TOE with new mass

base_file = 'AtlasBaseFinal_05_075_change.mat';
toe_file = 'AtlasToeFinal_05_075_change.mat';

base_file_2 = 'AtlasBaseFinal_05_075_change_2.mat';
toe_file_2 = 'AtlasToeFinal_05_075_change_2.mat';

base_file_3 = 'AtlasBaseFinal_05_075_change_3.mat';
toe_file_3 = 'AtlasToeFinal_05_075_change_3.mat';

base_file_4 = 'AtlasBaseFinal_05_075_change_4.mat';
toe_file_4 = 'AtlasToeFinal_05_075_change_4.mat';
%% 5 Steps at 0.80 polygon
% 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeFinal_05_080.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasToeFinal_05_080_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasToeFinal_05_080_3.mat';
% 
% base_file_4 = 'AtlasBaseForwardPushAnalysis5Steps_4.mat';
% toe_file_4 = 'AtlasToeFinal_05_080_4.mat';

%% 5 Steps at 0.85 polygon
% 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeFinal_05_085.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasToeFinal_05_085_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasToeFinal_05_085_3.mat';
% 
% base_file_4 = 'AtlasBaseForwardPushAnalysis5Steps_4.mat';
% toe_file_4 = 'AtlasToeFinal_05_085_4.mat';

%% 5 Steps at 0.90 polygon
% 
% base_file = 'AtlasBaseForwardPushAnalysis5Steps.mat';
% toe_file = 'AtlasToeFinal_05_090.mat';
% 
% base_file_2 = 'AtlasBaseForwardPushAnalysis5Steps_2.mat';
% toe_file_2 = 'AtlasToeFinal_05_090_2.mat';
% 
% base_file_3 = 'AtlasBaseForwardPushAnalysis5Steps_3.mat';
% toe_file_3 = 'AtlasToeFinal_05_090_3.mat';
% 
% base_file_4 = 'AtlasBaseForwardPushAnalysis5Steps_4.mat';
% toe_file_4 = 'AtlasToeFinal_05_090_4.mat';

%% 5 Steps at 0.6 footstep
% 
% base_file = 'AtlasBaseFinal_06.mat';
% toe_file = 'AtlasToeFinal_06_075.mat';
% 
% base_file_2 = 'AtlasBaseFinal_06_2.mat';
% toe_file_2 = 'AtlasToeFinal_06_075_2.mat';
% 
% base_file_3 = 'AtlasBaseFinal_06_3.mat';
% toe_file_3 = 'AtlasToeFinal_06_075_3.mat';
% 
% base_file_4 = 'AtlasBaseFinal_06_4.mat';
% toe_file_4 = 'AtlasToeFinal_06_075_4.mat';

%% 5 Steps at 0.7 footstep
% 
% base_file = 'AtlasBaseFinal_07.mat';
% toe_file = 'AtlasToeFinal_07_075.mat';
% 
% base_file_2 = 'AtlasBaseFinal_07_2.mat';
% toe_file_2 = 'AtlasToeFinal_07_075_2.mat';
% 
% base_file_3 = 'AtlasBaseFinal_07_3.mat';
% toe_file_3 = 'AtlasToeFinal_07_075_3.mat';
% 
% base_file_4 = 'AtlasBaseFinal_07_4.mat';
% toe_file_4 = 'AtlasToeFinal_07_075_4.mat';
%% Base Variables

base_l_knee_angle = load(base_file,'q_l_leg_kny');
base_l_knee_torque = load(base_file,'tau_l_leg_kny');

base_r_knee_angle = load(base_file,'q_r_leg_kny');
base_r_knee_torque = load(base_file,'tau_r_leg_kny');

base_l_leg_state = load(base_file_2,'leftLegState');
base_l_foot_state = load(base_file_2,'leftFootState');

base_l_knee_vel = load(base_file_2, 'qd_l_leg_kny');
base_r_knee_vel = load(base_file_2, 'qd_r_leg_kny');

base_r_leg_state = load(base_file_2,'rightLegState');
base_r_foot_state = load(base_file_2,'rightFootState');

base_time = load(base_file_2, 't');

base_com_x = load(base_file_2, 'q_x');
base_com_y = load(base_file_2, 'q_y');
base_com_xd = load(base_file_2, 'qd_x');
base_com_xdd = load(base_file_2, 'qdd_x');
base_com_yd = load(base_file_2, 'qd_y');
base_com_zd = load(base_file_2, 'qd_z');
base_com_z = load(base_file_2, 'q_z');
base_cmp_x = load(base_file_2, 'actualCMPX');
base_cmp_y = load(base_file_2, 'actualCMPY');

base_l_foot_force_x = load(base_file, 'l_footStateEstimatorForceX');
base_l_foot_force_y = load(base_file, 'l_footStateEstimatorForceY');
base_l_foot_force_z = load(base_file, 'l_footStateEstimatorForceZ');

base_l_foot_torque_x = load(base_file, 'l_footStateEstimatorTorqueX');
base_l_foot_torque_y = load(base_file, 'l_footStateEstimatorTorqueY');
base_l_foot_torque_z = load(base_file, 'l_footStateEstimatorTorqueZ');
base_l_foot_mag = load(base_file , 'l_footStateEstimatorFootForceMag');

base_r_foot_force_x = load(base_file, 'r_footStateEstimatorForceX');
base_r_foot_force_y = load(base_file, 'r_footStateEstimatorForceY');
base_r_foot_force_z = load(base_file, 'r_footStateEstimatorForceZ');

base_r_foot_torque_x = load(base_file, 'r_footStateEstimatorTorqueX');
base_r_foot_torque_y = load(base_file, 'r_footStateEstimatorTorqueY');
base_r_foot_torque_z = load(base_file, 'r_footStateEstimatorTorqueZ');
base_r_foot_mag = load(base_file , 'r_footStateEstimatorFootForceMag');

base_r_hipZ_torque = load(base_file_3,'tau_r_leg_hpz');
base_r_hipX_torque = load(base_file_3,'tau_r_leg_hpx');
base_r_hipY_torque = load(base_file_3,'tau_r_leg_hpy');
base_r_ankleX_torque = load(base_file_3,'tau_r_leg_akx');
base_r_ankleY_torque = load(base_file_3,'tau_r_leg_aky');

base_r_hipZ_vel = load(base_file_3,'qd_r_leg_hpz');
base_r_hipX_vel = load(base_file_3,'qd_r_leg_hpx');
base_r_hipY_vel = load(base_file_3,'qd_r_leg_hpy');
base_r_ankleX_vel = load(base_file_3,'qd_r_leg_akx');
base_r_ankleY_vel = load(base_file_3,'qd_r_leg_aky');

base_l_hipZ_torque = load(base_file_3,'tau_l_leg_hpz');
base_l_hipX_torque = load(base_file_3,'tau_l_leg_hpx');
base_l_hipY_torque = load(base_file_3,'tau_l_leg_hpy');
base_l_ankleX_torque = load(base_file_3,'tau_l_leg_akx');
base_l_ankleY_torque = load(base_file_3,'tau_l_leg_aky');

base_l_hipZ_vel = load(base_file_3,'qd_l_leg_hpz');
base_l_hipX_vel = load(base_file_3,'qd_l_leg_hpx');
base_l_hipY_vel = load(base_file_3,'qd_l_leg_hpy');
base_l_ankleX_vel = load(base_file_3,'qd_l_leg_akx');
base_l_ankleY_vel = load(base_file_3,'qd_l_leg_aky');

base_backZ_torque = load(base_file_4, 'tau_back_bkz');
base_backX_torque = load(base_file_4, 'tau_back_bkx');
base_backY_torque = load(base_file_4, 'tau_back_bky');

base_backZ_vel = load(base_file_4, 'qd_back_bkz');
base_backX_vel = load(base_file_4, 'qd_back_bkx');
base_backY_vel = load(base_file_4, 'qd_back_bky');

base_avg_speed = base_com_x.q_x(end) / base_time.t(end);

base_r_hipZPower = base_r_hipZ_torque.tau_r_leg_hpz.*base_r_hipZ_vel.qd_r_leg_hpz;
base_r_hipXPower = base_r_hipX_torque.tau_r_leg_hpx.*base_r_hipX_vel.qd_r_leg_hpx;
base_r_hipYPower = base_r_hipY_torque.tau_r_leg_hpy.*base_r_hipY_vel.qd_r_leg_hpy;
base_r_ankleXPower = base_r_ankleX_torque.tau_r_leg_akx.*base_r_ankleX_vel.qd_r_leg_akx;
base_r_ankleYPower = base_r_ankleY_torque.tau_r_leg_aky.*base_r_ankleY_vel.qd_r_leg_aky;
base_r_kneePower = base_r_knee_torque.tau_r_leg_kny.*base_r_knee_vel.qd_r_leg_kny;
base_l_hipZPower = base_l_hipZ_torque.tau_l_leg_hpz.*base_l_hipZ_vel.qd_l_leg_hpz;
base_l_hipXPower = base_l_hipX_torque.tau_l_leg_hpx.*base_l_hipX_vel.qd_l_leg_hpx;
base_l_hipYPower = base_l_hipY_torque.tau_l_leg_hpy.*base_l_hipY_vel.qd_l_leg_hpy;
base_l_ankleXPower = base_l_ankleX_torque.tau_l_leg_akx.*base_l_ankleX_vel.qd_l_leg_akx;
base_l_ankleYPower = base_l_ankleY_torque.tau_l_leg_aky.*base_l_ankleY_vel.qd_l_leg_aky;
base_l_kneePower =  base_l_knee_torque.tau_l_leg_kny.*base_l_knee_vel.qd_l_leg_kny;
base_back_bkzPower = base_backZ_torque.tau_back_bkz.*base_backZ_vel.qd_back_bkz;
base_back_bkxPower = base_backX_torque.tau_back_bkx.*base_backX_vel.qd_back_bkx;
base_back_bkyPower = base_backY_torque.tau_back_bky.*base_backY_vel.qd_back_bky;

% base_energy_all_chest = base_r_hipZPower + base_r_hipXPower + base_r_hipYPower ...
%     + base_r_ankleXPower + base_r_ankleYPower + base_l_hipZPower + base_l_hipXPower ...
%     + base_l_hipYPower + base_l_ankleXPower + base_l_ankleYPower + ...
%     base_l_kneePower + base_r_kneePower + base_back_bkzPower + base_back_bkxPower...
%     + base_back_bkyPower;
% 
% base_energy_all = base_r_hipZPower + base_r_hipXPower + base_r_hipYPower ...
%     + base_r_ankleXPower + base_r_ankleYPower + base_l_hipZPower + base_l_hipXPower ...
%     + base_l_hipYPower + base_l_ankleXPower + base_l_ankleYPower + ...
%     base_l_kneePower + base_r_kneePower;
% 
% base_energy_sag_chest = base_r_hipYPower ...
%     + base_r_ankleYPower ...
%     + base_l_hipYPower + base_l_ankleYPower + ...
%     base_l_kneePower + base_r_kneePower + base_back_bkyPower;
% 
% base_energy_sag = base_r_hipYPower ...
%     + base_r_ankleYPower ...
%     + base_l_hipYPower + base_l_ankleYPower + ...
%     base_l_kneePower + base_r_kneePower;

%% Both Legs
% base_energy_all_chest = [base_r_hipZPower ; base_r_hipXPower ; base_r_hipYPower ...
%     ; base_r_ankleXPower ; base_r_ankleYPower ; base_l_hipZPower ; base_l_hipXPower ...
%     ; base_l_hipYPower ; base_l_ankleXPower ; base_l_ankleYPower ; ...
%     base_l_kneePower ; base_r_kneePower ; base_back_bkzPower ; base_back_bkxPower...
%     ; base_back_bkyPower];
% 
% base_energy_all = [base_r_hipZPower ; base_r_hipXPower ; base_r_hipYPower ...
%     ; base_r_ankleXPower ; base_r_ankleYPower ; base_l_hipZPower ; base_l_hipXPower ...
%     ; base_l_hipYPower ; base_l_ankleXPower ; base_l_ankleYPower ; ...
%     base_l_kneePower ; base_r_kneePower];
% 
% base_energy_sag_chest = [base_r_hipYPower ...
%     ; base_r_ankleYPower ...
%     ; base_l_hipYPower ; base_l_ankleYPower ; ...
%     base_l_kneePower ; base_r_kneePower ; base_back_bkyPower];
% 
% base_energy_sag = [base_r_hipYPower ...
%     ; base_r_ankleYPower ...
%     ; base_l_hipYPower ; base_l_ankleYPower ; ...
%     base_l_kneePower ; base_r_kneePower];

%% Left Foot
% base_energy_all_chest = [ base_l_hipZPower ; base_l_hipXPower ...
%     ; base_l_hipYPower ; base_l_ankleXPower ; base_l_ankleYPower ; ...
%     base_l_kneePower ; base_back_bkzPower ; base_back_bkxPower...
%     ; base_back_bkyPower];
% 
% base_energy_all = [ base_l_hipZPower ; base_l_hipXPower ...
%     ; base_l_hipYPower ; base_l_ankleXPower ; base_l_ankleYPower ; ...
%     base_l_kneePower];
% 
% base_energy_sag_chest = [ base_l_hipYPower ; base_l_ankleYPower ; ...
%     base_l_kneePower ;  base_back_bkyPower];
% 
% base_energy_sag = [ base_l_hipYPower ; base_l_ankleYPower ; ...
%     base_l_kneePower];

%% Right Foot
% base_energy_all_chest = [ base_r_hipZPower ; base_r_hipXPower ...
%     ; base_r_hipYPower ; base_r_ankleXPower ; base_r_ankleYPower ; ...
%     base_r_kneePower ; base_back_bkzPower ; base_back_bkxPower...
%     ; base_back_bkyPower];
% 
% base_energy_all = [ base_r_hipZPower ; base_r_hipXPower ...
%     ; base_r_hipYPower ; base_r_ankleXPower ; base_r_ankleYPower ; ...
%     base_r_kneePower];
% 
% base_energy_sag_chest = [ base_r_hipYPower ; base_r_ankleYPower ; ...
%     base_r_kneePower ;  base_back_bkyPower];
% 
% base_energy_sag = [ base_r_hipYPower ; base_r_ankleYPower ; ...
%     base_r_kneePower];
%% Knee

base_energy_all_chest = [base_r_kneePower];

base_energy_all = [ base_r_kneePower ; base_l_kneePower];

base_energy_sag_chest = [ base_l_kneePower];

base_energy_sag = [ base_r_ankleYPower];







% P = T*OMEGA
% base_energy_conventional = base_l_knee_torque.tau_l_leg_kny.*base_l_knee_vel.qd_l_leg_kny ...
% + base_r_knee_torque.tau_r_leg_kny.*base_r_knee_vel.qd_r_leg_kny ...
% + base_r_hipZ_torque.tau_r_leg_hpz.*base_r_hipZ_vel.qd_r_leg_hpz ...
% + base_r_hipX_torque.tau_r_leg_hpx.*base_r_hipX_vel.qd_r_leg_hpx ...
% + base_r_hipY_torque.tau_r_leg_hpy.*base_r_hipY_vel.qd_r_leg_hpy ...
% + base_l_hipZ_torque.tau_l_leg_hpz.*base_l_hipZ_vel.qd_l_leg_hpz ...
% + base_l_hipX_torque.tau_l_leg_hpx.*base_l_hipX_vel.qd_l_leg_hpx ...
% + base_l_hipY_torque.tau_l_leg_hpy.*base_l_hipY_vel.qd_l_leg_hpy ...
% + base_r_ankleX_torque.tau_r_leg_akx.*base_r_ankleX_vel.qd_r_leg_akx ...
% + base_r_ankleY_torque.tau_r_leg_aky.*base_r_ankleY_vel.qd_r_leg_aky ...
% + base_l_ankleX_torque.tau_l_leg_akx.*base_l_ankleX_vel.qd_l_leg_akx ...
% + base_l_ankleY_torque.tau_l_leg_aky.*base_l_ankleY_vel.qd_l_leg_aky ;


base_omega_square = (9.81./base_com_z.q_z);
base_mass = 175.4; % kgs
base_accel_x = base_omega_square.*(base_com_x.q_x - base_cmp_x.actualCMPX);
base_accel_y = base_omega_square.*(base_com_y.q_y - base_cmp_y.actualCMPY);
base_force_x = base_mass.*base_accel_x;
base_force_y = base_mass.*base_accel_y;
base_energy_x = base_force_x.*base_com_xd.qd_x;
base_energy_y = base_force_y.*base_com_yd.qd_y;

base_sum_energy_x = zeros(1, length(base_energy_x));
base_sum_energy_y = zeros(1, length(base_energy_y));

base_sum_energy_all_chest = zeros(1, length(base_r_hipZPower));
base_sum_energy_all = zeros(1, length(base_r_hipZPower));
base_sum_energy_sag = zeros(1, length(base_r_hipZPower));
base_sum_energy_sag_chest = zeros(1, length(base_r_hipZPower));

base_power_direct_all_chest = zeros(1, length(base_r_hipZPower));
base_power_positive_all_chest = zeros(1, length(base_r_hipZPower));
base_power_mod_all_chest = zeros(1, length(base_r_hipZPower));
base_sum_power_direct_all_chest = zeros(1, length(base_r_hipZPower));
base_sum_power_positive_all_chest = zeros(1, length(base_r_hipZPower));
base_sum_power_mod_all_chest = zeros(1, length(base_r_hipZPower));

base_power_direct_all = zeros(1, length(base_r_hipZPower));
base_power_positive_all = zeros(1, length(base_r_hipZPower));
base_power_mod_all = zeros(1, length(base_r_hipZPower));
base_sum_power_direct_all = zeros(1, length(base_r_hipZPower));
base_sum_power_positive_all = zeros(1, length(base_r_hipZPower));
base_sum_power_mod_all = zeros(1, length(base_r_hipZPower));

base_power_direct_sag = zeros(1, length(base_r_hipZPower));
base_power_positive_sag = zeros(1, length(base_r_hipZPower));
base_power_mod_sag = zeros(1, length(base_r_hipZPower));
base_sum_power_direct_sag = zeros(1, length(base_r_hipZPower));
base_sum_power_positive_sag = zeros(1, length(base_r_hipZPower));
base_sum_power_mod_sag = zeros(1, length(base_r_hipZPower));

base_power_direct_sag_chest = zeros(1, length(base_r_hipZPower));
base_power_positive_sag_chest = zeros(1, length(base_r_hipZPower));
base_power_mod_sag_chest = zeros(1, length(base_r_hipZPower));
base_sum_power_direct_sag_chest = zeros(1, length(base_r_hipZPower));
base_sum_power_positive_sag_chest = zeros(1, length(base_r_hipZPower));
base_sum_power_mod_sag_chest = zeros(1, length(base_r_hipZPower));




for i = 1:length(base_energy_x)

            %% 
               % ----1
        base_power_direct_all_chest(i) = sum(base_energy_all_chest(:,i));
        base_sum_power_direct_all_chest(i) = sum(base_power_direct_all_chest);
        
        temp = base_energy_all_chest(:,i);
        base_power_positive_all_chest(i) = sum(temp(temp>0));
        base_sum_power_positive_all_chest(i) = sum(base_power_positive_all_chest); 
        
        base_power_mod_all_chest(i) = sum(abs(base_energy_all_chest(:,i)));
        base_sum_power_mod_all_chest(i) = sum(base_power_mod_all_chest);
        
                % ----2
        base_power_direct_all(i) = sum(base_energy_all(:,i));
        base_sum_power_direct_all(i) = sum(base_power_direct_all);
        
        temp = base_energy_all(:,i);
        base_power_positive_all(i) = sum(temp(temp>0));
        base_sum_power_positive_all(i) = sum(base_power_positive_all);
        
        
        base_power_mod_all(i) = sum(abs(base_energy_all(:,i)));
        base_sum_power_mod_all(i) = sum(base_power_mod_all);
        
                % ----3
        base_power_direct_sag(i) = sum(base_energy_sag(:,i));
        base_sum_power_direct_sag(i) = sum(base_power_direct_sag);
        
        temp = base_energy_sag(:,i);
        base_power_positive_sag(i) = sum(temp(temp>0));
        base_sum_power_positive_sag(i) = sum(base_power_positive_sag);
        
        base_power_mod_sag(i) = sum(abs(base_energy_sag(:,i)));
        base_sum_power_mod_sag(i) = sum(base_power_mod_sag);
        
                % ----4
        base_power_direct_sag_chest(i) = sum(base_energy_sag_chest(:,i));
        base_sum_power_direct_sag_chest(i) = sum(base_power_direct_sag_chest);
        
        temp = base_energy_sag_chest(:,i);
        base_power_positive_sag_chest(i) = sum(temp(temp>0));
        base_sum_power_positive_sag_chest(i) = sum(base_power_positive_sag_chest);
        
        base_power_mod_sag_chest(i) = sum(abs(base_energy_sag_chest(:,i)));
        base_sum_power_mod_sag_chest(i) = sum(base_power_mod_sag_chest);
        
    if(i~=1)
        base_sum_energy_x(i) =base_sum_energy_x(i-1) + base_energy_x(i);
        base_sum_energy_y(i) =base_sum_energy_y(i-1) + base_energy_y(i);
        base_sum_energy_all_chest(i) = base_sum_energy_all_chest(i-1) + base_sum_energy_all_chest(i);
        base_sum_energy_all(i) = base_sum_energy_all(i-1) + base_sum_energy_all(i);
        base_energy_sag(i) = base_energy_sag(i-1) + base_energy_sag(i);
        base_energy_sag_chest(i) = base_energy_sag_chest(i-1) + base_energy_sag_chest(i);
        

        
    else
        base_sum_energy_x(i) = base_energy_x(i);
        base_sum_energy_y(i) = base_energy_y(i);
        base_sum_energy_all_chest(i) = base_sum_energy_all_chest(i);
        base_sum_energy_all(i) = base_sum_energy_all(i);
        base_sum_energy_sag(i) = base_sum_energy_sag(i);
        base_sum_energy_sag_chest(i) = base_sum_energy_sag_chest(i);
        
        
    end
end

% Cost of Transport
% base_cost = base_sum_energy_conventional(end) / (base_mass*9.81*base_avg_speed);

base_l_foot_force_x.l_footStateEstimatorForceX(883) = -800;
base_l_foot_force_x.l_footStateEstimatorForceX(884) = -400;
%% Toe Variables

toe_l_knee_angle = load(toe_file,'q_l_leg_kny');
toe_l_toe_angle = load(toe_file,'q_l_leg_toe');

toe_l_knee_torque = load(toe_file,'tau_l_leg_kny');
toe_l_toe_torque = load(toe_file,'raw_tau_l_leg_toe');

toe_l_knee_vel = load(toe_file_2, 'qd_l_leg_kny');
toe_r_knee_vel = load(toe_file_2, 'qd_r_leg_kny');

toe_r_knee_angle = load(toe_file,'q_r_leg_kny');
toe_r_toe_angle = load(toe_file,'q_r_leg_toe');

toe_r_knee_torque = load(toe_file,'tau_r_leg_kny');
toe_r_toe_torque = load(toe_file,'raw_tau_r_leg_toe');

toe_l_leg_state = load(toe_file_2,'leftLegState');
toe_l_foot_state = load(toe_file_2,'leftFootState');

toe_r_leg_state = load(toe_file_2,'rightLegState');
toe_r_foot_state = load(toe_file_2,'rightFootState');

toe_com_x = load(toe_file_2, 'q_x');
toe_com_y = load(toe_file_2, 'q_y');
toe_com_xd = load(toe_file_2, 'qd_x');
toe_com_yd = load(toe_file_2, 'qd_y');
toe_com_xdd = load(toe_file_2, 'qdd_x');
toe_com_z = load(toe_file_2, 'q_z');
toe_cmp_x = load(toe_file_2, 'actualCMPX');
toe_cmp_y = load(toe_file_2, 'actualCMPY');

toe_time = load(toe_file_2, 't');

toe_avg_speed = toe_com_x.q_x(end) / toe_time.t(end);

toe_l_foot_force_x = load(toe_file, 'l_footStateEstimatorForceX');
toe_l_foot_force_y = load(toe_file, 'l_footStateEstimatorForceY');
toe_l_foot_force_z = load(toe_file, 'l_footStateEstimatorForceZ');

toe_l_foot_torque_x = load(toe_file, 'l_footStateEstimatorTorqueX');
toe_l_foot_torque_y = load(toe_file, 'l_footStateEstimatorTorqueY');
toe_l_foot_torque_z = load(toe_file, 'l_footStateEstimatorTorqueZ');
toe_l_foot_mag = load(toe_file , 'l_footStateEstimatorFootForceMag');

toe_r_foot_force_x = load(toe_file, 'r_footStateEstimatorForceX');
toe_r_foot_force_y = load(toe_file, 'r_footStateEstimatorForceY');
toe_r_foot_force_z = load(toe_file, 'r_footStateEstimatorForceZ');

toe_r_foot_torque_x = load(toe_file, 'r_footStateEstimatorTorqueX');
toe_r_foot_torque_y = load(toe_file, 'r_footStateEstimatorTorqueY');
toe_r_foot_torque_z = load(toe_file, 'r_footStateEstimatorTorqueZ');
toe_r_foot_mag = load(toe_file , 'r_footStateEstimatorFootForceMag');

toe_r_hipZ_torque = load(toe_file_3,'tau_r_leg_hpz');
toe_r_hipX_torque = load(toe_file_3,'tau_r_leg_hpx');
toe_r_hipY_torque = load(toe_file_3,'tau_r_leg_hpy');
toe_r_ankleX_torque = load(toe_file_3,'tau_r_leg_akx');
toe_r_ankleY_torque = load(toe_file_3,'tau_r_leg_aky');

toe_r_hipZ_vel = load(toe_file_3,'qd_r_leg_hpz');
toe_r_hipX_vel = load(toe_file_3,'qd_r_leg_hpx');
toe_r_hipY_vel = load(toe_file_3,'qd_r_leg_hpy');
toe_r_ankleX_vel = load(toe_file_3,'qd_r_leg_akx');
toe_r_ankleY_vel = load(toe_file_3,'qd_r_leg_aky');
toe_r_toe_vel = load(toe_file_3,'qd_r_leg_toe');

toe_l_hipZ_torque = load(toe_file_3,'tau_l_leg_hpz');
toe_l_hipX_torque = load(toe_file_3,'tau_l_leg_hpx');
toe_l_hipY_torque = load(toe_file_3,'tau_l_leg_hpy');
toe_l_ankleX_torque = load(toe_file_3,'tau_l_leg_akx');
toe_l_ankleY_torque = load(toe_file_3,'tau_l_leg_aky');

toe_l_hipZ_vel = load(toe_file_3,'qd_l_leg_hpz');
toe_l_hipX_vel = load(toe_file_3,'qd_l_leg_hpx');
toe_l_hipY_vel = load(toe_file_3,'qd_l_leg_hpy');
toe_l_ankleX_vel = load(toe_file_3,'qd_l_leg_akx');
toe_l_ankleY_vel = load(toe_file_3,'qd_l_leg_aky');
toe_l_toe_vel = load(toe_file_3,'qd_l_leg_toe');

toe_backZ_torque = load(toe_file_4, 'tau_back_bkz');
toe_backX_torque = load(toe_file_4, 'tau_back_bkx');
toe_backY_torque = load(toe_file_4, 'tau_back_bky');

toe_backZ_vel = load(toe_file_4, 'qd_back_bkz');
toe_backX_vel = load(toe_file_4, 'qd_back_bkx');
toe_backY_vel = load(toe_file_4, 'qd_back_bky');

toe_r_hipZPower = toe_r_hipZ_torque.tau_r_leg_hpz.*toe_r_hipZ_vel.qd_r_leg_hpz;
toe_r_hipXPower = toe_r_hipX_torque.tau_r_leg_hpx.*toe_r_hipX_vel.qd_r_leg_hpx;
toe_r_hipYPower = toe_r_hipY_torque.tau_r_leg_hpy.*toe_r_hipY_vel.qd_r_leg_hpy;
toe_r_ankleXPower = toe_r_ankleX_torque.tau_r_leg_akx.*toe_r_ankleX_vel.qd_r_leg_akx;
toe_r_ankleYPower = toe_r_ankleY_torque.tau_r_leg_aky.*toe_r_ankleY_vel.qd_r_leg_aky;
toe_r_toePower = toe_r_toe_torque.raw_tau_r_leg_toe.*toe_r_toe_vel.qd_r_leg_toe;

toe_l_hipZPower = toe_l_hipZ_torque.tau_l_leg_hpz.*toe_l_hipZ_vel.qd_l_leg_hpz;
toe_l_hipXPower = toe_l_hipX_torque.tau_l_leg_hpx.*toe_l_hipX_vel.qd_l_leg_hpx;
toe_l_hipYPower = toe_l_hipY_torque.tau_l_leg_hpy.*toe_l_hipY_vel.qd_l_leg_hpy;
toe_l_ankleXPower = toe_l_ankleX_torque.tau_l_leg_akx.*toe_l_ankleX_vel.qd_l_leg_akx;
toe_l_ankleYPower = toe_l_ankleY_torque.tau_l_leg_aky.*toe_l_ankleY_vel.qd_l_leg_aky;
toe_l_toePower = toe_l_toe_torque.raw_tau_l_leg_toe.*toe_l_toe_vel.qd_l_leg_toe;
toe_l_kneePower = toe_l_knee_torque.tau_l_leg_kny.*toe_l_knee_vel.qd_l_leg_kny ;
toe_r_kneePower = toe_r_knee_torque.tau_r_leg_kny.*toe_r_knee_vel.qd_r_leg_kny ;
toe_back_bkzPower = toe_backZ_torque.tau_back_bkz.*toe_backZ_vel.qd_back_bkz;
toe_back_bkxPower = toe_backX_torque.tau_back_bkx.*toe_backX_vel.qd_back_bkx;
toe_back_bkyPower = toe_backY_torque.tau_back_bky.*toe_backY_vel.qd_back_bky;

% toe_energy_conventional = toe_r_hipZPower + toe_r_hipXPower + toe_r_hipYPower ...
%     + toe_r_ankleXPower + toe_r_ankleYPower + toe_l_hipZPower + toe_l_toePower ...
%     +toe_l_hipXPower + toe_l_hipYPower + toe_l_ankleXPower + toe_l_ankleYPower ...
%     + toe_r_toePower + toe_l_kneePower + toe_r_kneePower  + toe_back_bkzPower ...
%     + toe_back_bkxPower +toe_back_bkyPower;

% toe_energy_conventional = toe_r_hipYPower + toe_r_ankleYPower + toe_l_toePower ...
%     + toe_l_hipYPower + toe_l_ankleYPower ...
%     + toe_r_toePower + toe_l_kneePower + toe_r_kneePower ;

% P = T*OMEGA
% toe_energy_conventional = toe_l_knee_torque.tau_l_leg_kny.*toe_l_knee_vel.qd_l_leg_kny ...
% + toe_r_knee_torque.tau_r_leg_kny.*toe_r_knee_vel.qd_r_leg_kny ...
% + toe_r_hipZ_torque.tau_r_leg_hpz.*toe_r_hipZ_vel.qd_r_leg_hpz ...
% + toe_r_hipX_torque.tau_r_leg_hpx.*toe_r_hipX_vel.qd_r_leg_hpx ...
% + toe_r_hipY_torque.tau_r_leg_hpy.*toe_r_hipY_vel.qd_r_leg_hpy ...
% + toe_l_hipZ_torque.tau_l_leg_hpz.*toe_l_hipZ_vel.qd_l_leg_hpz ...
% + toe_l_hipX_torque.tau_l_leg_hpx.*toe_l_hipX_vel.qd_l_leg_hpx ...
% + toe_l_hipY_torque.tau_l_leg_hpy.*toe_l_hipY_vel.qd_l_leg_hpy ...
% + toe_r_ankleX_torque.tau_r_leg_akx.*toe_r_ankleX_vel.qd_r_leg_akx ...
% + toe_r_ankleY_torque.tau_r_leg_aky.*toe_r_ankleY_vel.qd_r_leg_aky ...
% + toe_l_ankleX_torque.tau_l_leg_akx.*toe_l_ankleX_vel.qd_l_leg_akx ...
% + toe_l_ankleY_torque.tau_l_leg_aky.*toe_l_ankleY_vel.qd_l_leg_aky ...
% + toe_r_toe_torque.raw_tau_r_leg_toe.*toe_r_toe_vel.qd_r_leg_toe ...
% + toe_l_toe_torque.raw_tau_l_leg_toe.*toe_l_toe_vel.qd_l_leg_toe ;

%% Both Legs
% toe_energy_all_chest = [toe_r_hipZPower ; toe_r_hipXPower ; toe_r_hipYPower ...
%     ; toe_r_ankleXPower ; toe_r_ankleYPower ; toe_l_hipZPower ; toe_l_hipXPower ...
%     ; toe_l_hipYPower ; toe_l_ankleXPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_r_kneePower ; toe_back_bkzPower ; toe_back_bkxPower...
%     ; toe_back_bkyPower; toe_r_toePower; toe_l_toePower];
% 
% toe_energy_all = [toe_r_hipZPower ; toe_r_hipXPower ; toe_r_hipYPower ...
%     ; toe_r_ankleXPower ; toe_r_ankleYPower ; toe_l_hipZPower ; toe_l_hipXPower ...
%     ; toe_l_hipYPower ; toe_l_ankleXPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_r_kneePower; toe_r_toePower; toe_l_toePower];
% 
% toe_energy_sag_chest = [toe_r_hipYPower ...
%     ; toe_r_ankleYPower ...
%     ; toe_l_hipYPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_r_kneePower ; toe_back_bkyPower; toe_r_toePower; toe_l_toePower];
% 
% toe_energy_sag = [toe_r_hipYPower ...
%     ; toe_r_ankleYPower ...
%     ; toe_l_hipYPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_r_kneePower; toe_r_toePower; toe_l_toePower];

%% Left Foot
% toe_energy_all_chest = [toe_l_hipZPower ; toe_l_hipXPower ...
%     ; toe_l_hipYPower ; toe_l_ankleXPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_back_bkzPower ; toe_back_bkxPower...
%     ; toe_back_bkyPower; toe_l_toePower];
% 
% toe_energy_all = [toe_l_hipZPower ; toe_l_hipXPower ...
%     ; toe_l_hipYPower ; toe_l_ankleXPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_l_toePower];
% 
% toe_energy_sag_chest = [toe_l_hipYPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower ; toe_back_bkyPower; toe_l_toePower];
% 
% toe_energy_sag = [ toe_l_hipYPower ; toe_l_ankleYPower ; ...
%     toe_l_kneePower; toe_l_toePower];

%% Right Foot

% toe_energy_all_chest = [toe_r_hipZPower ; toe_r_hipXPower ...
%     ; toe_r_hipYPower ; toe_r_ankleXPower ; toe_r_ankleYPower ; ...
%     toe_r_kneePower ; toe_back_bkzPower ; toe_back_bkxPower...
%     ; toe_back_bkyPower; toe_r_toePower];
% 
% toe_energy_all = [toe_r_hipZPower ; toe_r_hipXPower ...
%     ; toe_r_hipYPower ; toe_r_ankleXPower ; toe_r_ankleYPower ; ...
%     toe_r_kneePower ; toe_r_toePower];
% 
% toe_energy_sag_chest = [toe_r_hipYPower ; toe_r_ankleYPower ; ...
%     toe_r_kneePower ; toe_back_bkyPower; toe_r_toePower];
% 
% toe_energy_sag = [ toe_r_hipYPower ; toe_r_ankleYPower ; ...
%     toe_r_kneePower; toe_r_toePower];

%% Knee
toe_energy_all_chest = [toe_r_kneePower];

toe_energy_all = [ toe_r_kneePower ; toe_l_kneePower];

toe_energy_sag_chest = [ toe_l_kneePower];

toe_energy_sag = [ toe_r_ankleYPower];


toe_power_direct_all_chest = zeros(1, length(toe_r_hipZPower));
toe_power_positive_all_chest = zeros(1, length(toe_r_hipZPower));
toe_power_mod_all_chest = zeros(1, length(toe_r_hipZPower));
toe_sum_power_direct_all_chest = zeros(1, length(toe_r_hipZPower));
toe_sum_power_positive_all_chest = zeros(1, length(toe_r_hipZPower));
toe_sum_power_mod_all_chest = zeros(1, length(toe_r_hipZPower));

toe_power_direct_all = zeros(1, length(toe_r_hipZPower));
toe_power_positive_all = zeros(1, length(toe_r_hipZPower));
toe_power_mod_all = zeros(1, length(toe_r_hipZPower));
toe_sum_power_direct_all = zeros(1, length(toe_r_hipZPower));
toe_sum_power_positive_all = zeros(1, length(toe_r_hipZPower));
toe_sum_power_mod_all = zeros(1, length(toe_r_hipZPower));

toe_power_direct_sag = zeros(1, length(toe_r_hipZPower));
toe_power_positive_sag = zeros(1, length(toe_r_hipZPower));
toe_power_mod_sag = zeros(1, length(toe_r_hipZPower));
toe_sum_power_direct_sag = zeros(1, length(toe_r_hipZPower));
toe_sum_power_positive_sag = zeros(1, length(toe_r_hipZPower));
toe_sum_power_mod_sag = zeros(1, length(toe_r_hipZPower));

toe_power_direct_sag_chest = zeros(1, length(toe_r_hipZPower));
toe_power_positive_sag_chest = zeros(1, length(toe_r_hipZPower));
toe_power_mod_sag_chest = zeros(1, length(toe_r_hipZPower));
toe_sum_power_direct_sag_chest = zeros(1, length(toe_r_hipZPower));
toe_sum_power_positive_sag_chest = zeros(1, length(toe_r_hipZPower));
toe_sum_power_mod_sag_chest = zeros(1, length(toe_r_hipZPower));


toe_omega_square = (9.81./toe_com_z.q_z);
toe_mass = 175.4+4.8; % kgs
toe_accel_x = toe_omega_square.*(toe_com_x.q_x - toe_cmp_x.actualCMPX);
toe_force_x = toe_mass.*toe_accel_x;
toe_accel_y = toe_omega_square.*(toe_com_y.q_y - toe_cmp_y.actualCMPY);
toe_force_y = toe_mass.*toe_accel_y;
toe_energy_x = toe_force_x.*toe_com_xd.qd_x;
toe_energy_y = toe_force_y.*toe_com_yd.qd_y;

toe_accel_x(895) = 3.0;

toe_sum_energy_x = zeros(1, length(toe_energy_x));
toe_sum_energy_y = zeros(1, length(toe_energy_y));


for i = 1:length(toe_energy_x)
    
        %% 
                % ----1
        toe_power_direct_all_chest(i) = sum(toe_energy_all_chest(:,i));
        toe_sum_power_direct_all_chest(i) = sum(toe_power_direct_all_chest);
        
        temp = toe_energy_all_chest(:,i);
        toe_power_positive_all_chest(i) = sum(temp(temp>0));
        toe_sum_power_positive_all_chest(i) = sum(toe_power_positive_all_chest); 
        
        toe_power_mod_all_chest(i) = sum(abs(toe_energy_all_chest(:,i)));
        toe_sum_power_mod_all_chest(i) = sum(toe_power_mod_all_chest);
        
                % ----2
        toe_power_direct_all(i) = sum(toe_energy_all(:,i));
        toe_sum_power_direct_all(i) = sum(toe_power_direct_all);
        
        temp = toe_energy_all(:,i);
        toe_power_positive_all(i) = sum(temp(temp>0));
        toe_sum_power_positive_all(i) = sum(toe_power_positive_all);
        
        
        toe_power_mod_all(i) = sum(abs(toe_energy_all(:,i)));
        toe_sum_power_mod_all(i) = sum(toe_power_mod_all);
        
                % ----3
        toe_power_direct_sag(i) = sum(toe_energy_sag(:,i));
        toe_sum_power_direct_sag(i) = sum(toe_power_direct_sag);
        
        temp = toe_energy_sag(:,i);
        toe_power_positive_sag(i) = sum(temp(temp>0));
        toe_sum_power_positive_sag(i) = sum(toe_power_positive_sag);
        
        toe_power_mod_sag(i) = sum(abs(toe_energy_sag(:,i)));
        toe_sum_power_mod_sag(i) = sum(toe_power_mod_sag);
        
                % ----4
        toe_power_direct_sag_chest(i) = sum(toe_energy_sag_chest(:,i));
        toe_sum_power_direct_sag_chest(i) = sum(toe_power_direct_sag_chest);
        
        temp = toe_energy_sag_chest(:,i);
        toe_power_positive_sag_chest(i) = sum(temp(temp>0));
        toe_sum_power_positive_sag_chest(i) = sum(toe_power_positive_sag_chest);
        
        toe_power_mod_sag_chest(i) = sum(abs(toe_energy_sag_chest(:,i)));
        toe_sum_power_mod_sag_chest(i) = sum(toe_power_mod_sag_chest);
    
    if(i~=1)
        toe_sum_energy_x(i) =toe_sum_energy_x(i-1) + toe_energy_x(i);
        toe_sum_energy_y(i) =toe_sum_energy_y(i-1) + toe_energy_y(i);
        
    else
        toe_sum_energy_x(i) = toe_energy_x(i);
        toe_sum_energy_y(i) = toe_energy_y(i);
        
    end
end

% Cost of Transport
% toe_cost = toe_sum_energy_conventional(end) / (toe_mass*9.81*toe_avg_speed);

%% Figures
%% Angles and Torques

figure (1)
subplot(6,2,1)
plot(base_time.t , base_l_knee_angle.q_l_leg_kny, 'b');
hold on;
plot(toe_time.t , toe_l_knee_angle.q_l_leg_kny, 'r');
hold off;
title('Left Knee Angle');

subplot(6,2,2)
plot(base_time.t ,base_r_knee_angle.q_r_leg_kny, 'b');
hold on;
plot(toe_time.t ,toe_r_knee_angle.q_r_leg_kny, 'r');
hold off;
title('Right Knee Angle');

subplot(6,2,3)
plot(base_time.t ,base_l_knee_torque.tau_l_leg_kny, 'b');
hold on;
plot(toe_time.t ,toe_l_knee_torque.tau_l_leg_kny, 'r');
hold off;
title('Left Knee Torque');

subplot(6,2,4)
plot(base_time.t , base_r_knee_torque.tau_r_leg_kny, 'b');
hold on;
plot(toe_time.t ,toe_r_knee_torque.tau_r_leg_kny, 'r');
hold off;
title('Right Knee Torque');

subplot(6,2,5)
plot(toe_time.t ,toe_l_toe_torque.raw_tau_l_leg_toe./toe_l_toe_angle.q_l_leg_toe);
title('Left Toe Spring Constant');

subplot(6,2,6)
plot(toe_time.t ,toe_r_toe_torque.raw_tau_r_leg_toe./toe_r_toe_angle.q_r_leg_toe);
title('Right Toe Spring Constant');

subplot(6,2,7)
plot(base_time.t , base_l_knee_torque.tau_l_leg_kny.*base_l_knee_vel.qd_l_leg_kny, 'b');
hold on;
plot(toe_time.t ,toe_l_knee_torque.tau_l_leg_kny.*toe_l_knee_vel.qd_l_leg_kny, 'r');
hold off;
ylim([-500.0 500.0])
title('Left Knee Power');

subplot(6,2,8)
plot(base_time.t , base_r_knee_torque.tau_r_leg_kny.*base_r_knee_vel.qd_r_leg_kny, 'b');
hold on;
plot(toe_time.t ,toe_r_knee_torque.tau_r_leg_kny.*toe_r_knee_vel.qd_r_leg_kny, 'r');
hold off;
title('Right Knee Power');

subplot(6,2,9)
plot(toe_time.t ,toe_l_toe_angle.q_l_leg_toe, 'r');
title('Left Toe Angle');

subplot(6,2,10)
plot(toe_time.t ,toe_r_toe_angle.q_r_leg_toe, 'r');
title('Right Toe Angle');

subplot(6,2,11)
plot(toe_time.t ,toe_l_toe_torque.raw_tau_l_leg_toe, 'r');
title('Left Toe Torque');

subplot(6,2,12)
plot(toe_time.t ,toe_r_toe_torque.raw_tau_r_leg_toe, 'r');
title('Right Toe Torque');

% subplot(8,2,13)
% plot(base_l_leg_state.leftLegState, 'b');
% hold on;
% plot(toe_l_leg_state.leftLegState, 'r');
% hold off;
% title('Left Leg State');
% 
% subplot(8,2,14)
% plot(base_r_leg_state.rightLegState, 'b');
% hold on;
% plot(toe_r_leg_state.rightLegState, 'r');
% hold off;
% title('Right Leg State');
% 
% subplot(8,2,15)
% plot(base_l_foot_state.leftFootState, 'b');
% hold on;
% plot(toe_l_foot_state.leftFootState, 'r');
% hold off;
% title('Left Foot State');
% 
% subplot(8,2,16)
% plot(base_r_foot_state.rightFootState, 'b');
% hold on;
% plot(toe_r_foot_state.rightFootState, 'r');
% hold off;
% title('Right Foot State');

%% Center of Mass Analysis

figure (2)

subplot(8,2,1)
plot(base_time.t , base_com_x.q_x, 'b');
hold on;
plot(toe_time.t ,toe_com_x.q_x, 'r');
hold off;
title('COM X direction');

subplot(8,2,2)
plot(base_time.t , base_com_y.q_y, 'b');
hold on;
plot(toe_time.t ,toe_com_y.q_y, 'r');
hold off;
title('COM Y direction');

subplot(8,2,3)
plot(base_time.t , base_com_xd.qd_x, 'b');
hold on;
plot(toe_time.t ,toe_com_xd.qd_x, 'r');
hold off;
title('COM X Velocity');

subplot(8,2,4)
plot(base_time.t , base_com_yd.qd_y, 'b');
hold on;
plot(toe_time.t ,toe_com_yd.qd_y, 'r');
hold off;
title('COM Y Velocity');

subplot(8,2,5)
plot(base_time.t , base_cmp_x.actualCMPX, 'b');
hold on;
plot(toe_time.t ,toe_cmp_x.actualCMPX, 'r');
hold off;
title('CMP X direction');

subplot(8,2,6)
plot(base_time.t , base_cmp_y.actualCMPY, 'b');
hold on;
plot(toe_time.t ,toe_cmp_y.actualCMPY, 'r');
hold off;
title('CMP Y direction');

subplot(8,2,[7,8])
plot(base_time.t , base_com_z.q_z, 'b');
hold on;
plot(toe_time.t ,toe_com_z.q_z, 'r');
hold off;
title('COM Height');

subplot(8,2,[9,10])
% plot(base_time.t , base_sum_energy_conventional, 'b');
% hold on;
% plot(toe_time.t ,toe_sum_energy_conventional, 'r');
% hold off;
% ylim([-10000, 40000]);
title('Sagittal Energy');

subplot(8,2,11)
plot(base_time.t , base_sum_energy_x , 'b');
hold on;
plot(toe_time.t ,toe_sum_energy_x , 'r');
hold off;
title('Weighted Energy Norm X');

subplot(8,2,12)
plot(base_time.t , base_sum_energy_y , 'b');
hold on;
plot(toe_time.t ,toe_sum_energy_y , 'r');
hold off;
title('Weighted Energy Norm Y');

subplot(8,2,13)
plot(base_time.t , base_l_leg_state.leftLegState, 'b');
hold on;
plot(toe_time.t ,toe_l_leg_state.leftLegState, 'r');
hold off;
title('Left Leg State');

subplot(8,2,14)
plot(base_time.t , base_r_leg_state.rightLegState, 'b');
hold on;
plot(toe_time.t ,toe_r_leg_state.rightLegState, 'r');
hold off;
title('Right Leg State');

subplot(8,2,15)
plot(base_time.t , base_l_foot_state.leftFootState, 'b');
hold on;
plot(toe_time.t ,toe_l_foot_state.leftFootState, 'r');
hold off;
title('Left Foot State');

subplot(8,2,16)
plot(base_time.t , base_r_foot_state.rightFootState, 'b');
hold on;
plot(toe_time.t ,toe_r_foot_state.rightFootState, 'r');
hold off;
title('Right Foot State');

%% Force and Torque Analysis
figure (3)

subplot(4,2,1)
plot(base_l_foot_force_x.l_footStateEstimatorForceX, 'b');
hold on;
plot(toe_l_foot_force_x.l_footStateEstimatorForceX, 'r');
hold off;
title('Left Force X');

subplot(4,2,2)
plot(base_time.t , base_r_foot_force_x.r_footStateEstimatorForceX, 'b');
hold on;
plot(toe_time.t ,toe_r_foot_force_x.r_footStateEstimatorForceX, 'r');
hold off;
title('Right Force X');

subplot(4,2,3)
plot(base_time.t , base_l_foot_force_z.l_footStateEstimatorForceZ, 'b');
hold on;
plot(toe_time.t ,toe_l_foot_force_z.l_footStateEstimatorForceZ, 'r');
hold off;
title('Left Force Z');

subplot(4,2,4)
plot(base_time.t , base_r_foot_force_z.r_footStateEstimatorForceZ, 'b');
hold on;
plot(toe_time.t ,toe_r_foot_force_z.r_footStateEstimatorForceZ, 'r');
hold off;
title('Right Force Z');

subplot(4,2,5)
plot(base_time.t , base_l_foot_torque_y.l_footStateEstimatorTorqueY, 'b');
hold on;
plot(toe_time.t ,toe_l_foot_torque_y.l_footStateEstimatorTorqueY, 'r');
hold off;
title('Left Torque Y');

subplot(4,2,6)
plot(base_time.t , base_r_foot_torque_y.r_footStateEstimatorTorqueY, 'b');
hold on;
plot(toe_time.t ,toe_r_foot_torque_y.r_footStateEstimatorTorqueY, 'r');
hold off;
title('Right Torque Y');

subplot(4,2,7)
plot(base_time.t , base_l_foot_mag.l_footStateEstimatorFootForceMag, 'b');
hold on;
plot(toe_time.t ,toe_l_foot_mag.l_footStateEstimatorFootForceMag, 'r');
hold off;
title('Left Force Magnitude');

subplot(4,2,8)
plot(base_time.t , base_r_foot_mag.r_footStateEstimatorFootForceMag, 'b');
hold on;
plot(toe_time.t ,toe_r_foot_mag.r_footStateEstimatorFootForceMag, 'r');
hold off;
title('Right Force Magnitude');


%% Power of Lower Body

figure (4)

subplot(6,2,1)
plot(base_time.t , base_l_hipZPower, 'b');
hold on;
plot(toe_time.t ,toe_l_hipZPower, 'r');
title('Left Hip Z Joint Power');

subplot(6,2,2)
plot(base_time.t , base_r_hipZPower, 'b');
hold on;
plot(toe_time.t ,toe_r_hipZPower, 'r');
title('Right Hip Z Joint Power');

subplot(6,2,3)
plot(base_time.t , base_l_hipXPower, 'b');
hold on;
plot(toe_time.t ,toe_l_hipXPower, 'r');
title('Left Hip X Joint Power');

subplot(6,2,4)
plot(base_time.t , base_r_hipXPower, 'b');
hold on;
plot(toe_time.t ,toe_r_hipXPower, 'r');
title('Right Hip X Joint Power');

subplot(6,2,5)
plot(base_time.t , base_l_hipYPower, 'b');
hold on;
plot(toe_time.t ,toe_l_hipYPower, 'r');
title('Left Hip Y Joint Power');

subplot(6,2,6)
plot(base_time.t , base_r_hipYPower, 'b');
hold on;
plot(toe_time.t ,toe_r_hipYPower, 'r');
title('Right Hip Y Joint Power');

subplot(6,2,7)
plot(base_time.t , base_l_ankleXPower, 'b');
hold on;
plot(toe_time.t ,toe_l_ankleXPower, 'r');
title('Left Ankle X Joint Power');

subplot(6,2,8)
plot(base_time.t , base_r_ankleXPower, 'b');
hold on;
plot(toe_time.t, toe_r_ankleXPower, 'r');
title('Right Ankle X Joint Power');

subplot(6,2,9)
plot(base_time.t ,base_l_ankleYPower, 'b');
hold on;
plot(toe_time.t ,toe_l_ankleYPower, 'r');
title('Left Ankle Y Joint Power');

subplot(6,2,10)
plot(base_time.t , base_r_ankleYPower, 'b');
hold on;
plot(toe_time.t ,toe_r_ankleYPower, 'r');
title('Right Ankle Y Joint Power');

subplot(6,2,11)
plot(toe_time.t ,toe_l_toePower, 'r');
title('Left Toe Joint Power');

subplot(6,2,12)
plot(toe_time.t ,toe_r_toePower, 'r');
title('Right Toe Joint Power');

%% Energetics

% figure (5)
% 
% subplot(2,1,1)
% plot(base_time.t , base_energy_conventional, 'b', 'LineWidth',2);
% hold on;
% plot(toe_time.t ,toe_energy_conventional, 'r', 'LineWidth',2);
% hold off;

%% Paper Plots

figure (7)
subplot(4,1,1)
plot(base_time.t , base_l_knee_angle.q_l_leg_kny, 'b',  'LineWidth',2);
hold on;
plot(toe_time.t , toe_l_knee_angle.q_l_leg_kny, 'r',  'LineWidth',2);
hold off;
% title('Left Knee Angle [rad]');

subplot(4,1,2)
plot(toe_time.t ,toe_l_toe_angle.q_l_leg_toe, 'r',  'LineWidth',2);
% title('Left Toe Angle [rad]');
subplot(4,1,3)
plot(base_time.t , base_com_z.q_z, 'b',  'LineWidth',2);
hold on;
plot(toe_time.t ,toe_com_z.q_z, 'r',  'LineWidth',2);
hold off;
% title('Center of Mass Height [m]');

subplot(4,1,4)
plot(toe_time.t ,toe_l_leg_state.leftLegState, 'r',  'LineWidth',2);
% title('Left Leg State');

figure (8)

subplot(3,1,1)
plot(base_time.t , base_l_foot_force_x.l_footStateEstimatorForceX, 'b',  'LineWidth',2);
hold on;
plot(toe_time.t ,toe_l_foot_force_x.l_footStateEstimatorForceX, 'r', 'LineWidth',2);
line([1.75 1.75], [-1000 1000],'Color','blue','LineStyle','--','LineWidth',1)
line([2.71 2.71], [-1000 1000],'Color','red','LineStyle','--','LineWidth',1.0)
line([3.54 3.54], [-1000 1000],'Color','blue','LineStyle','--','LineWidth',1.0)
line([4.5 4.5], [-1000 1000],'Color','red','LineStyle','--','LineWidth',1.0)
line([5.3 5.3], [-1000 1000],'Color','blue','LineStyle','--','LineWidth',1.0)
ylim([-1000,1000]);
hold off;
% title('Left Force X Direction [N]');

subplot(3,1,2)
plot(base_time.t , base_l_foot_force_z.l_footStateEstimatorForceZ, 'b',  'LineWidth',2);
hold on;
plot(toe_time.t ,toe_l_foot_force_z.l_footStateEstimatorForceZ, 'r',  'LineWidth',2);
line([1.75 1.75], [-1000 4000],'Color','blue','LineStyle','--','LineWidth',1.0)
line([2.71 2.71], [-1000 4000],'Color','red','LineStyle','--','LineWidth',1.0)
line([3.54 3.54], [-1000 4000],'Color','blue','LineStyle','--','LineWidth',1.0)
line([4.5 4.5], [-1000 4000],'Color','red','LineStyle','--','LineWidth',1.0)
line([5.3 5.3], [-1000 4000],'Color','blue','LineStyle','--','LineWidth',1.0)
hold off;
ylim([0,4000]);
% title('Left Force Z Direction [N]');

subplot(3,1,3)
plot(base_time.t , base_l_foot_mag.l_footStateEstimatorFootForceMag, 'b', 'LineWidth',2);
hold on;
plot(toe_time.t ,toe_l_foot_mag.l_footStateEstimatorFootForceMag, 'r', 'LineWidth',2);
line([1.75 1.75], [0 4000],'Color','blue','LineStyle','--','LineWidth',1.0)
line([2.71 2.71], [0 4000],'Color','red','LineStyle','--','LineWidth',1.0)
line([3.54 3.54], [0 4000],'Color','blue','LineStyle','--','LineWidth',1.0)
line([4.5 4.5], [0 4000],'Color','red','LineStyle','--','LineWidth',1.0)
line([5.3 5.3], [0 4000],'Color','blue','LineStyle','--','LineWidth',1.0)
hold off;
ylim([0,4000]);
% title('Left Force Magnitude [N]');

figure (9)

% plot(base_time.t , base_sum_energy_conventional, 'b', 'LineWidth',2);
% hold on;
% plot(toe_time.t ,toe_sum_energy_conventional, 'r', 'LineWidth',2);
% hold off;
% ylim([-10000, 40000]);
% title('Total Sagittal Energy [Joules]');

%% All Energies including chest
figure (9)

subplot(6,1,1)
plot(base_time.t, base_power_direct_all_chest,'b');
hold on;
plot(toe_time.t, toe_power_direct_all_chest,'r');
hold off;
title('Power Direct Including Chest');

subplot(6,1,2)
plot(base_time.t, base_power_positive_all_chest,'b');
hold on;
plot(toe_time.t, toe_power_positive_all_chest,'r');
hold off;
title('Power Positive Including Chest');

subplot(6,1,3)
plot(base_time.t, base_power_mod_all_chest,'b');
hold on;
plot(toe_time.t, toe_power_mod_all_chest,'r');
hold off;
title('Power Mod Including Chest');

subplot(6,1,4)
plot(base_time.t, base_sum_power_direct_all_chest,'b');
hold on;
plot(toe_time.t, toe_sum_power_direct_all_chest,'r');
hold off;
title('Sum Power Direct Including Chest');

subplot(6,1,5)
plot(base_time.t, base_sum_power_positive_all_chest,'b');
hold on;
plot(toe_time.t, toe_sum_power_positive_all_chest,'r');
hold off;
title('Sum Power Positive Including Chest');

subplot(6,1,6)
plot(base_time.t, base_sum_power_mod_all_chest,'b');
hold on;
plot(toe_time.t, toe_sum_power_mod_all_chest,'r');
hold off;
title('Sum Power Mod Including Chest');

%% All Energies excluding chest
figure (10)

subplot(6,1,1)
plot(base_time.t, base_power_direct_all,'b');
hold on;
plot(toe_time.t, toe_power_direct_all,'r');
hold off;
title('Power Direct Excluding Chest');

subplot(6,1,2)
plot(base_time.t, base_power_positive_all,'b');
hold on;
plot(toe_time.t, toe_power_positive_all,'r');
hold off;
title('Power Positive Excluding Chest');

subplot(6,1,3)
plot(base_time.t, base_power_mod_all,'b');
hold on;
plot(toe_time.t, toe_power_mod_all,'r');
hold off;
title('Power Mod Excluding Chest');

subplot(6,1,4)
plot(base_time.t, base_sum_power_direct_all,'b');
hold on;
plot(toe_time.t, toe_sum_power_direct_all,'r');
hold off;
title('Sum Power Direct Excluding Chest');

subplot(6,1,5)
plot(base_time.t, base_sum_power_positive_all,'b');
hold on;
plot(toe_time.t, toe_sum_power_positive_all,'r');
hold off;
title('Sum Power Positive Excluding Chest');

subplot(6,1,6)
plot(base_time.t, base_sum_power_mod_all,'b');
hold on;
plot(toe_time.t, toe_sum_power_mod_all,'r');
hold off;
title('Sum Power Mod Excluding Chest');


%% dummy graphs for knee

figure (20)

subplot(2,1,1)
plot(base_time.t, base_sum_power_direct_all,'b');
hold on;
plot(toe_time.t, toe_sum_power_direct_all,'r');
hold off;


subplot(2,1,2)
plot(base_time.t, base_sum_power_positive_all,'b');
hold on;
plot(toe_time.t, toe_sum_power_positive_all,'r');
hold off;


%% Sagittal Energies excluding chest
figure (11)

subplot(6,1,1)
plot(base_time.t, base_power_direct_sag,'b');
hold on;
plot(toe_time.t, toe_power_direct_sag,'r');
hold off;
title('Power Direct Sagittal Excluding Chest');

subplot(6,1,2)
plot(base_time.t, base_power_positive_sag,'b');
hold on;
plot(toe_time.t, toe_power_positive_sag,'r');
hold off;
title('Power Positive Sagittal Excluding Chest');

subplot(6,1,3)
plot(base_time.t, base_power_mod_sag,'b');
hold on;
plot(toe_time.t, toe_power_mod_sag,'r');
hold off;
title('Power Mod Sagittal Excluding Chest');

subplot(6,1,4)
plot(base_time.t, base_sum_power_direct_sag,'b');
hold on;
plot(toe_time.t, toe_sum_power_direct_sag,'r');
hold off;
title('Sum Power Direct Sagittal Excluding Chest');

subplot(6,1,5)
plot(base_time.t, base_sum_power_positive_sag,'b');
hold on;
plot(toe_time.t, toe_sum_power_positive_sag,'r');
hold off;
title('Sum Power Positive Sagittal Excluding Chest');

subplot(6,1,6)
plot(base_time.t, base_sum_power_mod_sag,'b');
hold on;
plot(toe_time.t, toe_sum_power_mod_sag,'r');
hold off;
title('Sum Power Mod Sagittal Excluding Chest');

%% Sagittal Energies including chest
figure (12)

subplot(6,1,1)
plot(base_time.t, base_power_direct_sag_chest,'b');
hold on;
plot(toe_time.t, toe_power_direct_sag_chest,'r');
hold off;
title('Power Direct Sagittal Including Chest');

subplot(6,1,2)
plot(base_time.t, base_power_positive_sag_chest,'b');
hold on;
plot(toe_time.t, toe_power_positive_sag_chest,'r');
hold off;
title('Power Positive Sagittal Including Chest');

subplot(6,1,3)
plot(base_time.t, base_power_mod_sag_chest,'b');
hold on;
plot(toe_time.t, toe_power_mod_sag_chest,'r');
hold off;
title('Power Mod Sagittal Including Chest');

subplot(6,1,4)
plot(base_time.t, base_sum_power_direct_sag_chest,'b');
hold on;
plot(toe_time.t, toe_sum_power_direct_sag_chest,'r');
hold off;
title('Sum Power Direct Sagittal Including Chest');

subplot(6,1,5)
plot(base_time.t, base_sum_power_positive_sag_chest,'b');
hold on;
plot(toe_time.t, toe_sum_power_positive_sag_chest,'r');
hold off;
title('Sum Power Positive Sagittal Including Chest');

subplot(6,1,6)
plot(base_time.t, base_sum_power_mod_sag_chest,'b');
hold on;
plot(toe_time.t, toe_sum_power_mod_sag_chest,'r');
hold off;
title('Sum Power Mod Sagittal Including Chest');


%% Spring Constant

% figure (5)
% subplot(1,2,1)
% plot(toe_r_toe_torque.raw_tau_r_leg_toe./toe_r_toe_angle.q_r_leg_toe);
% 
% subplot(1,2,2);
% plot(toe_l_toe_torque.raw_tau_l_leg_toe./toe_l_toe_angle.q_l_leg_toe);


%%
% figure (4)
% plot(base_com_xdd.qdd_x,'b');
% hold on;
% plot(base_accel_x,'r');
% title('Compare Base Accel X');
% 
% figure (5)
% plot(toe_com_xdd.qdd_x,'b');
% hold on;
% plot(toe_accel_x,'r');
% title('Compare Toe Accel X');
% 
% figure (6)
% plot(base_accel_x,'b');
% hold on;
% plot(toe_accel_x,'r');
% hold off;
% 
% figure (7)
% plot(base_accel_y,'b');
% hold on;
% plot(toe_accel_y,'r');
% hold off;
