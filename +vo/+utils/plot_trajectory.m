function plot_trajectory(traj, voLog, cfg)
t = voLog.t;
p = traj.pWvo;

if cfg.plot.autoGroundPlane
    ax = traj.meta.groundAxes;
else
    % fallback: X-Z plane by default (common for camera frame)
    ax = [1 3];
end

figure('Position',[50,100,1000,400]); 
subplot(1,2,1); hold on;
plot(p(:,ax(1)), p(:,ax(2)),'r--','LineWidth',1.5);
scatter( p(1,ax(1)),p(1,ax(2)),'ro');
axis equal; grid on;
labels = ["X","Y","Z"];
xlabel(labels(ax(1)));
ylabel(labels(ax(2)));
title("VO-only trajectory (ground plane, arbitrary scale)");
subtitle(traj.meta.Wvo_def);

subplot(1,2,2); hold on;
plot(t, 180/pi*voLog.dyaw,'b-','LineWidth',1.5);
xlabel("time(s)"); ylabel("\psi (deg)");
grid on;
end
