%% two_link_planar_no_spline.m
% 2-link planar manipulator following a real square in Cartesian space
clear; close all; clc;

%% robot parameters
a1 = 1.0;    % link1 length (m)
a2 = 0.8;    % link2 length (m)

%% define square (10cm x 10cm) in task space
center = [1.0, 0.0];    % (x,y) center in meters
halfSide = 0.05;        % half side = 5 cm

% corners clockwise and close loop
corners = [ center + [-halfSide, -halfSide];   % bottom-left
            center + [ halfSide, -halfSide];   % bottom-right
            center + [ halfSide,  halfSide];   % top-right
            center + [-halfSide,  halfSide];   % top-left
            center + [-halfSide, -halfSide] ]; % back to start

% sample points along edges (linear interpolation in XY)
pointsPerEdge = 150; % increase for smoother Cartesian path
qx = []; qy = [];
for i=1:(size(corners,1)-1)
    p1 = corners(i,:);
    p2 = corners(i+1,:);
    xs = linspace(p1(1), p2(1), pointsPerEdge+1);
    ys = linspace(p1(2), p2(2), pointsPerEdge+1);
    if i< (size(corners,1)-1)
        % omit the last point to avoid duplicating corner with next edge
        xs = xs(1:end-1); ys = ys(1:end-1);
    end
    qx = [qx, xs];
    qy = [qy, ys];
end
numSamples = numel(qx);

%% time vector (for plotting/animation)
T_total = 8; % seconds
t = linspace(0, T_total, numSamples);

%% compute IK for each sampled Cartesian point (choose continuous branch)
q1 = zeros(1, numSamples);
q2 = zeros(1, numSamples);
prev_q = [0; 0]; % initial previous joint angle guess (rad)

for k = 1:numSamples
    xk = qx(k);
    yk = qy(k);
    [qsol, valid] = ikine2R(xk, yk, a1, a2); % qsol is 2x2: cols are two solutions
    if ~valid
        error('Point (%.3f,%.3f) is out of workspace or no real IK solution.', xk, yk);
    end
    sol1 = qsol(:,1);
    sol2 = qsol(:,2);
    % choose the solution closest to previous (continuous)
    if norm(sol1 - prev_q) <= norm(sol2 - prev_q)
        chosen = sol1;
    else
        chosen = sol2;
    end
    q1(k) = chosen(1);
    q2(k) = chosen(2);
    prev_q = chosen;
end

% unwrap to avoid artificial jumps in plotting
q1_unwrap = unwrap(q1);
q2_unwrap = unwrap(q2);

%% forward kinematics reconstruction (sanity check)
EE = zeros(2, numSamples);
for k=1:numSamples
    T01 = [ cos(q1_unwrap(k)) -sin(q1_unwrap(k)) 0 a1*cos(q1_unwrap(k));
            sin(q1_unwrap(k))  cos(q1_unwrap(k)) 0 a1*sin(q1_unwrap(k));
            0                  0                 1 0;
            0                  0                 0 1];
    T12 = [ cos(q2_unwrap(k)) -sin(q2_unwrap(k)) 0 a2*cos(q2_unwrap(k));
            sin(q2_unwrap(k))  cos(q2_unwrap(k)) 0 a2*sin(q2_unwrap(k));
            0                  0                 1 0;
            0                  0                 0 1];
    T02 = T01 * T12;
    EE(:,k) = T02(1:2,4);
end

%% FIGURE: End-effector trajectory vs desired square
figure('Name','End-effector trajectory (Cartesian path)');
plot(qx, qy, 'k--', 'LineWidth', 1.2); hold on;        % desired square sample points (exact)
plot(EE(1,:), EE(2,:), 'b-', 'LineWidth', 1.5);         % reconstructed EE via FK of chosen IK
plot(corners(:,1), corners(:,2), 'ro', 'MarkerFaceColor','r'); % corners
legend('Desired Cartesian path (sampled edges)','Executed EE path (FK of IK)', 'Corners','Location','best');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('End-effector trajectory (real square followed in Cartesian space)');

%% FIGURE: Joint angle trajectories
figure('Name','Joint angle trajectories');
subplot(2,1,1);
plot(t, q1_unwrap, 'LineWidth', 1.2); grid on;
ylabel('\theta_1 (rad)'); title('Joint 1 trajectory (from IK)');
subplot(2,1,2);
plot(t, q2_unwrap, 'LineWidth', 1.2); grid on;
ylabel('\theta_2 (rad)'); xlabel('Time (s)'); title('Joint 2 trajectory (from IK)');

%% FIGURE: Workspace (sampled) with executed path overlay
theta1vec = linspace(-pi, pi, 301);
theta2vec = linspace(-pi, pi, 301);
[TH1, TH2] = meshgrid(theta1vec, theta2vec);
Xpts = a1*cos(TH1) + a2*cos(TH1 + TH2);
Ypts = a1*sin(TH1) + a2*sin(TH1 + TH2);

figure('Name','Workspace with executed path');
scatter(Xpts(:), Ypts(:), 1, '.'); hold on; % sampled workspace
plot(EE(1,:), EE(2,:), 'r-', 'LineWidth', 1.5); % executed path
plot(corners(:,1), corners(:,2), 'ko', 'MarkerFaceColor','k');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Workspace of 2R manipulator (sampled) with executed path overlay');

%% ANIMATION: manipulator moving along trajectory
figure('Name','Manipulator animation');
lim = a1 + a2 + 0.1;
axis([-lim lim -lim lim]); axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); title('Manipulator animation following square (Cartesian sampling)');
hold on;
h_link1 = plot([0,0],[0,0],'-o','LineWidth',3,'MarkerSize',6,'MarkerFaceColor','b');
h_link2 = plot([0,0],[0,0],'-o','LineWidth',3,'MarkerSize',6,'MarkerFaceColor','g');
h_path = plot(nan,nan,'r.','MarkerSize',6);

% animate (skip some frames for speed if desired)
step = 4;
for k = 1:step:numSamples
    q1k = q1_unwrap(k); q2k = q2_unwrap(k);
    p0 = [0;0];
    p1 = [a1*cos(q1k); a1*sin(q1k)];
    p2 = [a1*cos(q1k) + a2*cos(q1k+q2k); a1*sin(q1k) + a2*sin(q1k+q2k)];
    set(h_link1, 'XData', [p0(1), p1(1)], 'YData', [p0(2), p1(2)]);
    set(h_link2, 'XData', [p1(1), p2(1)], 'YData', [p1(2), p2(2)]);
    % append executed path
    oldX = get(h_path,'XData'); oldY = get(h_path,'YData');
    set(h_path,'XData',[oldX p2(1)], 'YData',[oldY p2(2)]);
    drawnow;
    pause(0.01);
end

%% Additional figure: Joint trajectories (angles vs sample index)
figure('Name','Joint angle trajectories (index)');
plot(1:numSamples, q1_unwrap, 'b-', 'LineWidth', 1.2); hold on;
plot(1:numSamples, q2_unwrap, 'r-', 'LineWidth', 1.2);
xlabel('Sample index'); ylabel('Angle (rad)'); grid on;
legend('\theta_1','\theta_2'); title('Joint angle trajectories (by sample index)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local functions (IK and FK) - included for completeness
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [qsol, valid] = ikine2R(x, y, a1, a2)
% Closed-form inverse kinematics for 2R planar manipulator
% qsol is 2x2 matrix: [q1_sol1, q1_sol2; q2_sol1, q2_sol2]
    r2 = x^2 + y^2;
    c2 = (r2 - a1^2 - a2^2) / (2*a1*a2);
    % numeric safety clamp
    if abs(c2) > 1 + 1e-9
        valid = false;
        qsol = NaN(2,2);
        return;
    end
    valid = true;
    c2 = max(min(c2,1), -1);
    s2 = sqrt(max(0,1 - c2^2));
    q2a = atan2( s2, c2 );   % elbow "one" (positive sin)
    q2b = atan2(-s2, c2 );   % elbow "other" (negative sin)

    k1a = a1 + a2*cos(q2a); k2a = a2*sin(q2a);
    q1a = atan2(y, x) - atan2(k2a, k1a);

    k1b = a1 + a2*cos(q2b); k2b = a2*sin(q2b);
    q1b = atan2(y, x) - atan2(k2b, k1b);

    qsol = [q1a, q1b; q2a, q2b];
end
