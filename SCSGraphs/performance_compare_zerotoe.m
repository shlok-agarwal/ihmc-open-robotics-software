clear all ;
clc;
close all;

base_file = 'AtlasStraightZeroPrivilegedAnalysis.mat';
toe_file = 'AtlasToeZeroPrivilegedAnalysis.mat';

base_l_ankle_angle = load(base_file,'q_l_leg_akx');
base_l_knee_angle = load(base_file,'q_l_leg_kny');
base_l_ankle_torque = load(base_file,'tau_l_leg_akx');
base_l_knee_torque = load(base_file,'tau_l_leg_kny');

base_r_ankle_angle = load(base_file,'q_r_leg_akx');
base_r_knee_angle = load(base_file,'q_r_leg_kny');
base_r_ankle_torque = load(base_file,'tau_r_leg_akx');
base_r_knee_torque = load(base_file,'tau_r_leg_kny');

base_leftFootHitGround = load(base_file,'l_footStateEstimatorFootHitGround');
base_rightFootHitGround = load(base_file,'r_footStateEstimatorFootHitGround');


toe_l_ankle_angle = load(toe_file,'q_l_leg_akx');
toe_l_knee_angle = load(toe_file,'q_l_leg_kny');
toe_l_toe_angle = load(toe_file,'q_l_leg_toe');
toe_l_ankle_torque = load(toe_file,'tau_l_leg_akx');
toe_l_knee_torque = load(toe_file,'tau_l_leg_kny');
toe_l_toe_torque = load(toe_file,'tau_l_leg_toe');

toe_r_ankle_angle = load(toe_file,'q_r_leg_akx');
toe_r_knee_angle = load(toe_file,'q_r_leg_kny');
toe_r_toe_angle = load(toe_file,'q_r_leg_toe');

toe_r_ankle_torque = load(toe_file,'tau_r_leg_akx');
toe_r_knee_torque = load(toe_file,'tau_r_leg_kny');
toe_r_toe_torque = load(toe_file,'tau_r_leg_toe');

toe_leftFootHitGround = load(toe_file,'l_footStateEstimatorFootHitGround');
toe_rightFootHitGround = load(toe_file,'r_footStateEstimatorFootHitGround');

% filer 

subplot(4,4,1)
plot(base_l_ankle_angle.q_l_leg_akx, 'b');
hold on;
plot(toe_l_ankle_angle.q_l_leg_akx, 'r');
hold off;
title('Left Ankle Angle');

subplot(4,4,2)
plot(base_l_ankle_torque.tau_l_leg_akx, 'b');
hold on;
plot(toe_l_ankle_torque.tau_l_leg_akx, 'r');
hold off;
title('Left Ankle Torque');

subplot(4,4,3)
plot(base_r_ankle_angle.q_r_leg_akx, 'b');
hold on;
plot(toe_r_ankle_angle.q_r_leg_akx, 'r');
hold off;
title('Right Ankle Angle');

subplot(4,4,4)
plot(base_r_ankle_torque.tau_r_leg_akx, 'b');
hold on;
plot(toe_r_ankle_torque.tau_r_leg_akx, 'r');
hold off;
title('Right Ankle Torque');

%%
subplot(4,4,5)
plot(base_l_knee_angle.q_l_leg_kny, 'b');
hold on;
plot(toe_l_knee_angle.q_l_leg_kny, 'r');
hold off;
title('Left Knee Angle');

subplot(4,4,6)
plot(base_l_knee_torque.tau_l_leg_kny, 'b');
hold on;
plot(toe_l_knee_torque.tau_l_leg_kny, 'r');
hold off;
title('Left Knee Torque');

subplot(4,4,7)
plot(base_r_knee_angle.q_r_leg_kny, 'b');
hold on;
plot(toe_r_knee_angle.q_r_leg_kny, 'r');
hold off;
title('Right Knee Angle');

subplot(4,4,8)
plot(base_r_knee_torque.tau_r_leg_kny, 'b');
hold on;
plot(toe_r_knee_torque.tau_r_leg_kny, 'r');
hold off;
title('Right Knee Torque');

%%

subplot(4,4,9)
plot(toe_l_toe_angle.q_l_leg_toe, 'r');
title('Left Toe Angle');

subplot(4,4,10)
plot(toe_l_toe_torque.tau_l_leg_toe, 'r');
title('Left Toe Torque');

subplot(4,4,11)
plot(toe_r_toe_angle.q_r_leg_toe, 'r');
title('Right Toe Angle');

subplot(4,4,12)
plot(toe_r_toe_torque.tau_r_leg_toe, 'r');
hold off;
title('Right Toe Torque');

%%

subplot(4,4,13)
plot(base_leftFootHitGround.l_footStateEstimatorFootHitGround, 'b');
hold on;
plot(toe_leftFootHitGround.l_footStateEstimatorFootHitGround, 'r');
hold off;
title('Left Foot Hit Ground');

subplot(4,4,14)
plot(base_rightFootHitGround.r_footStateEstimatorFootHitGround, 'b');
hold on;
plot(toe_rightFootHitGround.r_footStateEstimatorFootHitGround, 'r');
hold off;
title('Right Foot Hit Ground');
