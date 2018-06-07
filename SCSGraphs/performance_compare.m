clear all ;
clc;
close all;

base_l_ankle_angle = load('AtlasBaseSim.mat','q_l_leg_akx');
base_l_knee_angle = load('AtlasBaseSim.mat','q_l_leg_kny');
base_l_ankle_torque = load('AtlasBaseSim.mat','tau_l_leg_akx');
base_l_knee_torque = load('AtlasBaseSim.mat','tau_l_leg_kny');

base_r_ankle_angle = load('AtlasBaseSim.mat','q_r_leg_akx');
base_r_knee_angle = load('AtlasBaseSim.mat','q_r_leg_kny');
base_r_ankle_torque = load('AtlasBaseSim.mat','tau_r_leg_akx');
base_r_knee_torque = load('AtlasBaseSim.mat','tau_r_leg_kny');

toe_l_ankle_angle = load('AtlasToeEmergentAnalysis.mat','q_l_leg_akx');
toe_l_knee_angle = load('AtlasToeEmergentAnalysis.mat','q_l_leg_kny');
toe_l_toe_angle = load('AtlasToeEmergentAnalysis.mat','q_l_leg_toe');
toe_l_ankle_torque = load('AtlasToeEmergentAnalysis.mat','tau_l_leg_akx');
toe_l_knee_torque = load('AtlasToeEmergentAnalysis.mat','tau_l_leg_kny');
toe_l_toe_torque = load('AtlasToeEmergentAnalysis.mat','tau_l_leg_toe');

toe_r_ankle_angle = load('AtlasToeEmergentAnalysis.mat','q_r_leg_akx');
toe_r_knee_angle = load('AtlasToeEmergentAnalysis.mat','q_r_leg_kny');
toe_r_toe_angle = load('AtlasToeEmergentAnalysis.mat','q_r_leg_toe');

toe_r_ankle_torque = load('AtlasToeEmergentAnalysis.mat','tau_r_leg_akx');
toe_r_knee_torque = load('AtlasToeEmergentAnalysis.mat','tau_r_leg_kny');
toe_r_toe_torque = load('AtlasToeEmergentAnalysis.mat','tau_r_leg_toe');

subplot(3,4,1)
plot(base_l_ankle_angle.q_l_leg_akx, 'b');
hold on;
plot(toe_l_ankle_angle.q_l_leg_akx, 'r');
hold off;
title('Left Ankle Angle');

subplot(3,4,2)
plot(base_l_ankle_torque.tau_l_leg_akx, 'b');
hold on;
plot(toe_l_ankle_torque.tau_l_leg_akx, 'r');
hold off;
title('Left Ankle Torque');

subplot(3,4,3)
plot(base_r_ankle_angle.q_r_leg_akx, 'b');
hold on;
plot(toe_r_ankle_angle.q_r_leg_akx, 'r');
hold off;
title('Right Ankle Angle');

subplot(3,4,4)
plot(base_r_ankle_torque.tau_r_leg_akx, 'b');
hold on;
plot(toe_r_ankle_torque.tau_r_leg_akx, 'r');
hold off;
title('Right Ankle Torque');

%%
subplot(3,4,5)
plot(base_l_knee_angle.q_l_leg_kny, 'b');
hold on;
plot(toe_l_knee_angle.q_l_leg_kny, 'r');
hold off;
title('Left Knee Angle');

subplot(3,4,6)
plot(base_l_knee_torque.tau_l_leg_kny, 'b');
hold on;
plot(toe_l_knee_torque.tau_l_leg_kny, 'r');
hold off;
title('Left Knee Torque');

subplot(3,4,7)
plot(base_r_knee_angle.q_r_leg_kny, 'b');
hold on;
plot(toe_r_knee_angle.q_r_leg_kny, 'r');
hold off;
title('Right Knee Angle');

subplot(3,4,8)
plot(base_r_knee_torque.tau_r_leg_kny, 'b');
hold on;
plot(toe_r_knee_torque.tau_r_leg_kny, 'r');
hold off;
title('Right Knee Torque');

%%

subplot(3,4,9)
plot(toe_l_toe_angle.q_l_leg_toe, 'r');
title('Left Toe Angle');

subplot(3,4,10)
plot(toe_l_toe_torque.tau_l_leg_toe, 'r');
title('Left Toe Torque');

subplot(3,4,11)
plot(toe_r_toe_angle.q_r_leg_toe, 'r');
title('Right Toe Angle');

subplot(3,4,12)
plot(toe_r_toe_torque.tau_r_leg_toe, 'r');
hold off;
title('Right Toe Torque');
