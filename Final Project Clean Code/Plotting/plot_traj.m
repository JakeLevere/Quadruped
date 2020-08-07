%% Plotting Graphs Function

function plot_traj(posF_g,timeMat,velF_g,Alpha,Beta,Gamma,alphaVels,betaVels,gammaVels)

% Foot Trajectory
trajFig = figure('Name','Foot Trajectory Fwd and Back');
plot(posF_g(1,:),posF_g(2,:),'k-')
grid on
xlabel('y pos (in)')
ylabel('z pos (in)')
title('Foot Trajectory Fwd and Back')
saveas(trajFig,'Fwd and BackTrajectories.png')

%%
% Foot Positions and velocities
foot6Fig = figure('Name','Foot Positions and Velocities wrt Gnd Fwd and Back');
subplot(2,2,1)
plot(timeMat,posF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('y pos (in)')
title('Y pos vs time wrt gnd')

subplot(2,2,2)
plot(timeMat,velF_g(1,:),'r-')
grid on
xlabel('time (s)')
ylabel('dy vel (in/s)')
title('DY vel vs time wrt gnd')

subplot(2,2,3)
plot(timeMat,posF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('Z pos (in)')
title('Z pos vs time wrt gnd')

subplot(2,2,4)
plot(timeMat,velF_g(2,:),'r-')
grid on
xlabel('time (s)')
ylabel('dz vel (in/s)')
title('DZ vel vs time wrt gnd')
saveas(foot6Fig,'Fwd and Back Foot Pos and Vel.png')

%%
% Joint Positions and Velocities
jntPosFig = figure('Name','Joint Positions vs time(fwd_back)');
subplot(3,1,1)
plot(timeMat,Alpha(1,:),'r-')
hold on
plot(timeMat,Alpha(2,:),'g-')
hold on
plot(timeMat,Alpha(3,:),'b-')
hold on
plot(timeMat,Alpha(4,:),'c-')
hold on
grid on
legend('Leg 1','Leg 2','Leg 3','Leg 4')
xlabel('time (s)')
ylabel('Alpha pos (rad)')
title('Alpha Positions vs time')

subplot(3,1,2)
plot(timeMat,Beta(1,:),'r-')
hold on
plot(timeMat,Beta(2,:),'g-')
hold on
plot(timeMat,Beta(3,:),'b-')
hold on
plot(timeMat,Beta(4,:),'c-')
hold on
grid on
legend('Leg 1','Leg 2','Leg 3','Leg 4')
grid on
xlabel('time (s)')
ylabel('Beta pos (rad)')
title('Beta Positions vs time')

subplot(3,1,3)
plot(timeMat,Gamma(1,:),'r-')
hold on
plot(timeMat,Gamma(2,:),'g-')
hold on
plot(timeMat,Gamma(3,:),'b-')
hold on
plot(timeMat,Gamma(4,:),'c-')
hold on
grid on
legend('Leg 1','Leg 2','Leg 3','Leg 4')
grid on
xlabel('time (s)')
ylabel('Gamma pos (rad/s)')
title('Gamma Positions vs time')
saveas(jntPosFig,'Joint Positions vs time (fwd_back).png')

jntVelFig = figure('Name','Joint Velocities vs time(fwd_back)');
subplot(3,1,1)
plot(timeMat,alphaVels(1,:),'r-')
hold on
plot(timeMat,alphaVels(2,:),'g-')
hold on
plot(timeMat,alphaVels(3,:),'b-')
hold on
plot(timeMat,alphaVels(4,:),'c-')
hold on
grid on
legend('Leg 1','Leg 2','Leg 3','Leg 4')
xlabel('time (s)')
ylabel('Alpha vel (rad/s)')
title('Alpha Velocities vs time')

subplot(3,1,2)
plot(timeMat,betaVels(1,:),'r-')
hold on
plot(timeMat,betaVels(2,:),'g-')
hold on
plot(timeMat,betaVels(3,:),'b-')
hold on
plot(timeMat,betaVels(4,:),'c-')
hold on
grid on
legend('Leg 1','Leg 2','Leg 3','Leg 4')
grid on
xlabel('time (s)')
ylabel('Beta vel (rad/s)')
title('Beta Velocities vs time')

subplot(3,1,3)
plot(timeMat,gammaVels(1,:),'r-')
hold on
plot(timeMat,gammaVels(2,:),'g-')
hold on
plot(timeMat,gammaVels(3,:),'b-')
hold on
plot(timeMat,gammaVels(4,:),'c-')
hold on
grid on
legend('Leg 1','Leg 2','Leg 3','Leg 4')
grid on
xlabel('time (s)')
ylabel('Gamma vel (rad/s)')
title('Gamma Velocities vs time')
saveas(jntVelFig,'Joint Velocities vs time (fwd_back).png')


end
