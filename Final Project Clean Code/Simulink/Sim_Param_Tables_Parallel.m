% Parallel motion Simulink

function [leg1_alpha,leg2_alpha,leg3_alpha,leg4_alpha,...
    leg1_beta,leg2_beta,leg3_beta,leg4_beta,...
    leg1_gamma,leg2_gamma,leg3_gamma,leg4_gamma,hip_length,shin_length,thigh_length] = Sim_Param_Tables_Parallel(allAlphai,allBetai,allGammai,tibia,femur,coxa)

shin_length = tibia;
thigh_length = femur;
hip_length = coxa;

timerange = seconds(linspace(0,4,20));

allAlphai = allAlphai';
allBetai = allBetai';
allGammai = allGammai';

leg1_alpha = timetable(-[allAlphai(1,:)].', 'rowTimes', timerange.');
leg2_alpha = timetable(-[allAlphai(2,:)].', 'rowTimes', timerange.');
leg3_alpha = timetable(-[allAlphai(3,:)].', 'rowTimes', timerange.');
leg4_alpha = timetable(-[allAlphai(4,:)].', 'rowTimes', timerange.');

leg1_beta = timetable(allBetai(1,:).', 'rowTimes', timerange.');
leg2_beta = timetable(allBetai(2,:).', 'rowTimes', timerange.');
leg3_beta = timetable(allBetai(3,:).', 'rowTimes', timerange.');
leg4_beta = timetable(allBetai(4,:).', 'rowTimes', timerange.');

leg1_gamma = timetable(allGammai(1,:).'-pi/2, 'rowTimes', timerange.');
leg2_gamma = timetable(allGammai(2,:).'-pi/2, 'rowTimes', timerange.');
leg3_gamma = timetable(allGammai(3,:).'-pi/2, 'rowTimes', timerange.');
leg4_gamma = timetable(allGammai(4,:).'-pi/2, 'rowTimes', timerange.');

end