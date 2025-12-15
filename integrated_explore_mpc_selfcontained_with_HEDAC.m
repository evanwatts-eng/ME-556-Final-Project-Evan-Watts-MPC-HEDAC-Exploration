% integrated_explore_mpc_selfcontained_with_HEDAC.m
% Single-file runnable script: integrates local RRT exploration + MPC tracking
% Adds a simple heat field + diffusion and biases RRT samples toward cooler areas.
clear; clc; close all;
rng(195212115)
%% ========== Environment & Robot Parameters ==========
gridSize = 20;
numShapes = 12;
cellsPerRow = ceil(sqrt(numShapes));
cellSize = gridSize / cellsPerRow;
shapeVertices = 6;
minScale = 0.3; maxScale = 0.9;
bufferDist = 0.5;            % obstacle buffer (m)
senseRadius = 5;             % sensing / local planning radius (m)

% Start and global goal
start = [0,0];
goal = [rand*gridSize, rand*gridSize];

% Robot dynamic + MPC parameters
dt_ctrl = 0.05;              % control timestep used by MPC integrator
N_mpc = 6;                   % MPC horizon
rho = 0.01;                  % input cost weight
Q = diag([20, 20]);          % state tracking cost
R = rho * eye(2);            % control cost
u_min = [-1; -1]; u_max = [1; 1];  % input bounds (m/s)
tol_waypoint = 0.08;         % waypoint arrival tolerance for MPC (m)
max_global_iters = 600;      % main loop limit

%% ========== HEDAC (heat) parameters ==========
heatRes = 0.5;               % spatial resolution of heat grid (m)
xHeat = 0:heatRes:gridSize;
yHeat = 0:heatRes:gridSize;
[HX, HY] = meshgrid(xHeat, yHeat);  % HX: columns = x positions, HY: rows = y positions
heatT = zeros(size(HX));     % temperature field
diffusion_alpha = 1.6;       % diffusion coefficient (tunable; larger -> faster spreading)
dt_heat = 0.1;               % time step for heat integration (s)
heat_source_strength = 3.0;  % temperature increase per second while heating (continuous)
heat_cap = 300;              % cap maximum temperature
bias_beta = 1.8;             % sampling bias factor
max_rrt_bias_trials = 200;   % not used in new approach, kept for compatibility
heat_avoid_threshold = 4.0;  % temperatures above this are treated as 'too hot' for MPC to step into


%% ========== Create Figure ==========
figure('Name','Exploration + MPC (Self-contained)'); hold on; axis equal;
xlim([0 gridSize]); ylim([0 gridSize]);
title('Integrated Exploration (Local RRT) + MPC Tracking');

% precompute circle param (for plotting sensing radius)
theta = linspace(0,2*pi,120);

% heat map visualization (put under obstacles so heat appears behind)
hHeat = imagesc(xHeat, yHeat, heatT); set(gca,'YDir','normal');
colormap(gca, hot);
alpha(hHeat, 0.6);        % semi-transparent heat overlay
colorbar;
caxis([0 10]);            % visual scaling (tune to expected temps)

%% ========== Generate Random Convex Obstacles ==========
hObstacles = gobjects(0); hBuffers = gobjects(0);
bufX_all = {}; bufY_all = {};
usedCells = zeros(0,2);

for i = 1:numShapes
    while true
        cx = randi([0 cellsPerRow-1]);
        cy = randi([0 cellsPerRow-1]);
        if ~ismember([cx cy], usedCells, 'rows')
            usedCells = [usedCells; cx cy];
            break;
        end
    end
    originX = cx * cellSize; originY = cy * cellSize;
    scale = (minScale + (maxScale-minScale)*rand) * cellSize/2;
    angles = rand(shapeVertices,1)*2*pi;
    radius = scale*(0.5 + rand(shapeVertices,1));
    x = cos(angles).*radius; y = sin(angles).*radius;
    K = convhull(x,y);

    centerX = originX + cellSize/2;
    centerY = originY + cellSize/2;
    shapeX = x(K) + centerX;
    shapeY = y(K) + centerY;

    hObs = fill(shapeX, shapeY, rand(1,3), 'FaceAlpha',0.5, 'EdgeColor','k');
    hObstacles(end+1) = hObs; %#ok<SAGROW>

    % Buffer
    cX = mean(shapeX); cY = mean(shapeY);
    vecX = shapeX - cX; vecY = shapeY - cY;
    normF = sqrt(vecX.^2 + vecY.^2); normF(normF==0)=1;
    bufX = shapeX + bufferDist*(vecX./normF);
    bufY = shapeY + bufferDist*(vecY./normF);
    Kbuf = convhull(bufX, bufY);
    hBuf = plot(bufX(Kbuf), bufY(Kbuf), 'r--','LineWidth',1.2);
    hBuffers(end+1) = hBuf; 

    bufX_all{i} = bufX(Kbuf); bufY_all{i} = bufY(Kbuf);
end

% Plot start and goal (keep handles)
hStart = plot(start(1),start(2),'go','MarkerFaceColor','g','MarkerSize',8);
hGoal  = plot(goal(1),goal(2),'ro','MarkerFaceColor','r','MarkerSize',8);
text(goal(1)+0.3, goal(2)+0.3, 'Goal');

% Dummy handles for legend items that will be plotted later or transient
hRRTNodes = plot(NaN, NaN, 'b.', 'MarkerSize', 4);  % will update when nodes exist
hCorrDummy = patch([-1 -1 -1],[-1 -1 -1],'c','FaceAlpha',0.25,'EdgeColor','c'); % corridor symbol
hTraj = plot(NaN, NaN, 'k-', 'LineWidth', 1); % trajectory line to update
hRobot = plot(start(1), start(2), 'ko', 'MarkerFaceColor','k'); % robot marker
hSensingCircle = plot(NaN, NaN, 'b--', 'LineWidth', 1.2);   % sensing circle handle

legend([hObstacles(1), hBuffers(1), hStart, hGoal, hRRTNodes, hCorrDummy, hTraj, hSensingCircle], ...
       {'Obstacles','Buffers','Start','Goal','RRT nodes','Corridor','Trajectory','Sensing radius'}, ...
       'Location','bestoutside');

drawnow;

%% ========== Helper function handles (collision checks) ==========
in_collision_segment = @(p1,p2) any(arrayfun(@(j) ...
    any(inpolygon(linspace(p1(1),p2(1),12), ...
                  linspace(p1(2),p2(2),12), ...
                  bufX_all{j}, bufY_all{j})), ...
    1:numShapes));

%% ========== Robot initialization ==========
x_robot = start(:);          % column vector [x;y]
traj_all = x_robot.';        % record of executed robot states (for plotting)

% initialize trajectory/robot visuals
set(hTraj, 'XData', traj_all(:,1), 'YData', traj_all(:,2));
set(hRobot, 'XData', x_robot(1), 'YData', x_robot(2));
% initialize sensing circle
circX = x_robot(1) + senseRadius * cos(theta);
circY = x_robot(2) + senseRadius * sin(theta);
set(hSensingCircle, 'XData', circX, 'YData', circY);

%% ===== Logging for report / numerical results =====
robot_positions = x_robot.';         % store every robot position
heat_exposure = interp2(HX, HY, heatT, x_robot(1), x_robot(2), 'linear', 0); % initial
mpc_scaling_count = 0;               % count of scaled steps due to heat

%% ========== Precompute MPC matrices (integrator) ==========
nx = 2; nu = 2;
S_singlerow = cell(N_mpc,1);
for i = 1:N_mpc
    Srow = zeros(nx, nu*N_mpc);
    for j = 1:i
        Srow(:, (j-1)*nu+1:j*nu) = dt_ctrl*eye(nx);
    end
    S_singlerow{i} = Srow;
end
S_big = cell2mat(S_singlerow);           % (nx*N) x (nu*N)
Phi_big = kron(ones(N_mpc,1), eye(nx));  % (nx*N) x nx

Qbar = kron(eye(N_mpc), Q);
Rbar = kron(eye(N_mpc), R);
H_quad_base = 2*(S_big'*Qbar*S_big + Rbar) + 1e-6*eye(nu*N_mpc);

optsQP = optimoptions('quadprog','Display','off');

%% ========== Main exploration loop ==========
fprintf('Global goal: [%.2f, %.2f]\n', goal(1), goal(2));
iter = 0;
goal_reached = false;

% movement threshold for detecting "stop"
move_threshold = 1e-3;

while ~goal_reached && iter < max_global_iters
    iter = iter + 1;
    prev_pos = x_robot;
    fprintf('Main iter %d, robot at [%.2f, %.2f]\n', iter, x_robot(1), x_robot(2));

    % Determine local planning target:
    if norm(goal - x_robot') <= senseRadius
        local_target = goal;
    else
        dir_to_goal = (goal - x_robot')';
        dir_to_goal = dir_to_goal / norm(dir_to_goal);
        local_target = x_robot' + 0.98*senseRadius*dir_to_goal;
        % clamp
        local_target(1) = min(max(local_target(1),0),gridSize);
        local_target(2) = min(max(local_target(2),0),gridSize);
    end

    % Generate local RRT path from robot to local_target within sensing radius
    [local_path, nodes] = local_rrt(x_robot', local_target, senseRadius, 500, 0.7, in_collision_segment, gridSize, HX, HY, heatT, bias_beta, max_rrt_bias_trials);
    if isempty(local_path)
        % try a few random alternative local goals
        success = false;
        for trial = 1:6
            ang = 2*pi*rand; candidate = (x_robot' + 0.9*senseRadius*[cos(ang),sin(ang)]);
            candidate(1) = min(max(candidate(1),0),gridSize);
            candidate(2) = min(max(candidate(2),0),gridSize);
            [local_path, nodes] = local_rrt(x_robot', candidate, senseRadius, 500, 0.7, in_collision_segment, gridSize, HX, HY, heatT, bias_beta, max_rrt_bias_trials);
            if ~isempty(local_path), success = true; break; end
        end
        if ~success
            % small random nudge to escape tight spot
            rnddir = randn(2,1); rnddir = rnddir / norm(rnddir);
            x_robot = x_robot + min(0.5, 0.2*senseRadius)*rnddir;
            x_robot(1) = min(max(x_robot(1),0), gridSize);
            x_robot(2) = min(max(x_robot(2),0), gridSize);
            traj_all = [traj_all; x_robot.'];
            % update visuals
            set(hTraj, 'XData', traj_all(:,1), 'YData', traj_all(:,2));
            set(hRobot, 'XData', x_robot(1), 'YData', x_robot(2));
            set(hSensingCircle, 'XData', x_robot(1) + senseRadius * cos(theta), ...
                                'YData', x_robot(2) + senseRadius * sin(theta));
            drawnow;
            % apply heating because robot got nudged/stuck
            heatT = apply_heat_source_continuous(heatT, HX, HY, x_robot, senseRadius, heat_source_strength, heat_cap);
            set(hHeat, 'CData', heatT);
            drawnow;
            continue;
        end
    end

    % Plot RRT nodes (light) -- update RRT nodes handle once per iteration
    if ~isempty(nodes)
        set(hRRTNodes, 'XData', nodes(:,1), 'YData', nodes(:,2));
    else
        set(hRRTNodes, 'XData', NaN, 'YData', NaN);
    end

    % Convert local_path to corridor (simple rectangle pieces)
    corridorPolys = path_to_corridor_simple(local_path, 0.4);
    hCorr = gobjects(numel(corridorPolys),1);
    for cidx = 1:numel(corridorPolys)
        poly = corridorPolys{cidx};
        hCorr(cidx) = fill(poly(:,1), poly(:,2), [0.6 0.9 1], 'FaceAlpha',0.25, 'EdgeColor','c');
    end
    drawnow;

    % Use MPC to drive along the local_path until the end of that path
    fprintf('  Driving along local path with MPC (segments: %d)\n', size(local_path,1)-1);
    success_drive = true;
    for segIdx = 1:size(local_path,1)-1
        % target waypoint for this segment
        wp_start = local_path(segIdx,:)';
        wp_goal  = local_path(segIdx+1,:)';
        [x_robot, ok] = mpc_drive_to_waypoint(x_robot, wp_goal, S_big, Phi_big, Qbar, Rbar, H_quad_base, ...
         N_mpc, dt_ctrl, tol_waypoint, u_min, u_max, corridorPolys, segIdx, optsQP, HX, HY, heatT, heat_avoid_threshold);
        % After moving robot along MPC
        heatT = apply_heat_source_continuous(heatT, HX, HY, x_robot, senseRadius, heat_source_strength, dt_heat, heat_cap);



        traj_all = [traj_all; x_robot.' ];
        % update visuals incrementally
        set(hTraj, 'XData', traj_all(:,1), 'YData', traj_all(:,2));
        set(hRobot, 'XData', x_robot(1), 'YData', x_robot(2));
        set(hSensingCircle, 'XData', x_robot(1) + senseRadius * cos(theta), ...
                            'YData', x_robot(2) + senseRadius * sin(theta));
        drawnow;

        if ~ok
            success_drive = false;
            warning('  MPC failed to reach waypoint %d of local path. Replan next iteration.', segIdx+1);
            % apply heating because MPC got stuck
            heatT = apply_heat_source_continuous(heatT, HX, HY, x_robot, senseRadius, heat_source_strength, heat_cap);
            set(hHeat, 'CData', heatT);
            drawnow;
            break;
        end
    end

        % if robot barely moved during this main iteration, consider it stopped -> heat (continuous)
    if norm(x_robot - prev_pos) < move_threshold
        heatT = apply_heat_source_continuous(heatT, HX, HY, x_robot, senseRadius, heat_source_strength, dt_heat, heat_cap);
    end

    % also heat if MPC failed earlier in the path (this code already sets heatT when MPC failed)
    % Diffuse heat (explicit stable scheme) with a few sub-steps
    numSubsteps = 4;
    for s = 1:numSubsteps
        heatT = diffuse_heat_fast(heatT, heatRes, diffusion_alpha, dt_heat/numSubsteps);
    end

    % update heat visualization (must update CData)
    set(hHeat, 'CData', heatT);
    % dynamic color scaling so changes are visible
    clim_low = 0;
    clim_high = max(6, max(heatT(:)));
    caxis([clim_low, clim_high]);
    drawnow;


    % cleanup corridor visuals
    for h = 1:numel(hCorr)
        if isvalid(hCorr(h)), delete(hCorr(h)); end
    end

    % check global goal reached
    if norm(goal - x_robot') <= 0.3
        goal_reached = true;
        fprintf('Reached goal vicinity at [%.2f, %.2f]\n', x_robot(1), x_robot(2));
        break;
    end

    pause(0.01);
    % --- log robot position ---
    robot_positions = [robot_positions; x_robot.'];
    
    % --- log heat exposure at robot location ---
    Tcand_current = interp2(HX, HY, heatT, x_robot(1), x_robot(2), 'linear', 0);
    heat_exposure = [heat_exposure; Tcand_current];
    
    % --- count if MPC had to scale control due to heat ---
    if Tcand_current > heat_avoid_threshold
        mpc_scaling_count = mpc_scaling_count + 1;
    end

end

if ~goal_reached
    fprintf('Stopped after %d main iterations. Current pos [%.2f, %.2f]\n', iter, x_robot(1), x_robot(2));
end

% final plot: ensure trajectories and markers are current
set(hTraj, 'XData', traj_all(:,1), 'YData', traj_all(:,2));
set(hRobot, 'XData', x_robot(1), 'YData', x_robot(2));
set(hSensingCircle, 'XData', x_robot(1) + senseRadius * cos(theta), ...
                    'YData', x_robot(2) + senseRadius * sin(theta));
drawnow;

%% ===== Compute numerical results =====
total_path_length = sum(vecnorm(diff(robot_positions),2,2));
max_heat_encountered = max(heat_exposure);
avg_heat_encountered = mean(heat_exposure);
fprintf('Total path length: %.2f\n', total_path_length);
fprintf('Maximum heat encountered along path: %.2f\n', max_heat_encountered);
fprintf('Average heat encountered along path: %.2f\n', avg_heat_encountered);
fprintf('Number of MPC control scalings due to heat: %d\n', mpc_scaling_count);


%% ===========================
%% Local functions
%% ===========================
function [path, nodes] = local_rrt(p_start, p_goal, r_s_local, rrt_max_nodes, rrt_step, in_collision_segment, gridSize, HX, HY, heatT, beta, ~)
    % local_rrt: grid-based importance sampling inside sensing disk + standard RRT connect
    % p_start, p_goal: 1x2
    nodes = p_start;
    parents = 1;
    success = false;

    % Precompute candidate grid indices inside sensing disk (same for all iterations)
    dx = HX - p_start(1);
    dy = HY - p_start(2);
    dist2 = dx.^2 + dy.^2;
    diskMask = dist2 <= (r_s_local + 1e-9)^2;
    if ~any(diskMask(:))
        path = []; nodes = []; return;
    end

    % Pre-flatten candidate positions and their base coords
    [rows, cols] = find(diskMask);
    candX = HX(diskMask);
    candY = HY(diskMask);
    numCand = numel(candX);

    for ii = 1:rrt_max_nodes
        % compute sampling weights from heat map (lower T -> higher weight)
        temps = heatT(diskMask);
        weights = exp(-beta * temps);
        if sum(weights) <= 0
            weights = ones(size(weights));
        end
        weights = weights / sum(weights);

        % sample one candidate grid cell index
        % cumulative distribution
        cumw = cumsum(weights);
        r = rand();
        idx = find(cumw >= r, 1, 'first');
        sample = [candX(idx), candY(idx)];

        % jitter sample slightly within the cell to avoid exact grid centers
        jitter = (rand(1,2)-0.5) * min(r_s_local/10, 0.3);
        sample = sample + jitter;
        sample(1) = min(max(sample(1),0), gridSize);
        sample(2) = min(max(sample(2),0), gridSize);

        % steer from nearest node toward sample
        dists = vecnorm(nodes - sample, 2, 2);
        [~, nid] = min(dists);
        nearest = nodes(nid,:);
        dir = sample - nearest;
        if norm(dir) < 1e-6, continue; end
        dir = dir / norm(dir);
        newNode = nearest + min(rrt_step, norm(sample - nearest)) * dir;
        newNode(1) = min(max(newNode(1),0), gridSize);
        newNode(2) = min(max(newNode(2),0), gridSize);

        % keep inside sensing disk
        if norm(newNode - p_start) > r_s_local + 1e-6, continue; end
        if in_collision_segment(nearest, newNode), continue; end

        nodes = [nodes; newNode]; 
        parents = [parents; nid]; 

        % try connect to goal
        if norm(newNode - p_goal) < 0.8 && ~in_collision_segment(newNode, p_goal)
            nodes = [nodes; p_goal];
            parents = [parents; size(nodes,1)-1];
            success = true;
            break;
        end
    end

    if ~success
        path = [];
        return;
    end

    % Reconstruct path
    path = nodes(end,:);
    current = size(nodes,1);
    while current ~= 1
        current = parents(current);
        path = [nodes(current,:); path]; %#ok<AGROW>
    end
end


function corridorPolys = path_to_corridor_simple(path, halfwidth)
    % builds rectangle around each path segment (returns cell array of Nx2 polygons)
    if nargin < 2, halfwidth = 0.5; end
    corridorPolys = {};
    for i = 1:size(path,1)-1
        p1 = path(i,:); p2 = path(i+1,:);
        dir = p2 - p1; len = norm(dir);
        if len < 1e-6
            continue;
        end
        dir = dir/len;
        perp = [-dir(2), dir(1)]*halfwidth;
        poly = [p1+perp; p2+perp; p2-perp; p1-perp];
        corridorPolys{end+1} = poly;
    end
end

function [x_new, ok] = mpc_drive_to_waypoint(x0, wp_goal, S_big, Phi_big, Qbar, Rbar, H_quad_base, ...
                                             N_mpc, dt_ctrl, tol_waypoint, u_min, u_max, corridorPolys, segIdx, optsQP, HX, HY, heatT, heat_thres)
    ok = false;
    x_robot = x0;
    max_local_iters = 200;
    iter_local = 0;
    nx = 2; nu = 2;

    % Precompute corridor constraints (same as before)
    if segIdx <= numel(corridorPolys)
        poly = corridorPolys{segIdx};
        V = poly;
        if ~isequal(V(1,:), V(end,:)), V = [V; V(1,:)]; end
        cen = mean(V(1:end-1,:),1);
        A_corr = []; b_corr = [];
        for jj = 1:size(V,1)-1
            edge = V(jj+1,:) - V(jj,:);
            n = [-edge(2); edge(1)];
            if n'*cen' <= n'*V(jj,:)'
                A_corr = [A_corr; n'];
                b_corr = [b_corr; n'*V(jj,:)'];
            else
                A_corr = [A_corr; -n'];
                b_corr = [b_corr; -n'*V(jj,:)'];
            end
        end
    else
        A_corr = []; b_corr = [];
    end

    while iter_local < max_local_iters
        iter_local = iter_local + 1;

        % stacked reference
        x_ref_stacked = repmat(wp_goal, N_mpc, 1);

        z0 = Phi_big * x_robot - x_ref_stacked;
        f_quad = 2*(S_big'*Qbar*z0);

        if ~isempty(A_corr)
            n_edges = size(A_corr,1);
            numSlack = n_edges * N_mpc;
            Aineq = zeros(N_mpc*n_edges, nu*N_mpc + numSlack);
            bineq = zeros(N_mpc*n_edges, 1);
            for i = 1:N_mpc
                Srow = S_big( (i-1)*nx+1:i*nx, : );
                row_idx = (i-1)*n_edges + 1 : i*n_edges;
                slack_idx = (i-1)*n_edges + 1 : i*n_edges;
                Aineq(row_idx, 1:nu*N_mpc) = A_corr * Srow;
                Aineq(row_idx, nu*N_mpc + slack_idx) = -eye(n_edges);
                bineq(row_idx) = b_corr - A_corr * x_robot;
            end
            Aineq = [Aineq;
                     [eye(nu*N_mpc), zeros(nu*N_mpc, numSlack)];
                     [-eye(nu*N_mpc), zeros(nu*N_mpc, numSlack)] ];
            bineq = [bineq;
                     repmat(u_max, N_mpc, 1);
                     -repmat(u_min, N_mpc, 1)];
            Hslack = blkdiag(H_quad_base, 2*100*eye(numSlack));
            f_slack = [f_quad; zeros(numSlack,1)];
            [zopt,~,ef] = quadprog(Hslack, f_slack, Aineq, bineq, [], [], [], [], [], optsQP);
            if isempty(zopt) || ef ~= 1
                Aineq2 = [ [eye(nu*N_mpc)]; [-eye(nu*N_mpc)] ];
                bineq2 = [ repmat(u_max, N_mpc, 1); -repmat(u_min, N_mpc, 1) ];
                [zopt,~,ef2] = quadprog(H_quad_base, f_quad, Aineq2, bineq2, [], [], [], [], [], optsQP);
                if isempty(zopt) || ef2~=1
                    ok = false;
                    x_new = x_robot;
                    return;
                end
            end
        else
            Aineq2 = [ [eye(nu*N_mpc)]; [-eye(nu*N_mpc)] ];
            bineq2 = [ repmat(u_max, N_mpc, 1); -repmat(u_min, N_mpc, 1) ];
            [zopt,~,ef2] = quadprog(H_quad_base, f_quad, Aineq2, bineq2, [], [], [], [], [], optsQP);
            if isempty(zopt) || ef2~=1
                ok = false; x_new = x_robot; return;
            end
        end

        % apply first control, but check resulting temperature
        u0 = zopt(1:2);
        x_candidate = x_robot + dt_ctrl * u0;

        % interpolate temperature at candidate
        Tcand = interp2(HX, HY, heatT, x_candidate(1), x_candidate(2), 'linear', 0);
        if Tcand > heat_thres
            % try scaling control down to step away from hot zone
            scales = [0.5, 0.25, 0.1];
            scaled_ok = false;
            for s = 1:numel(scales)
                xc = x_robot + dt_ctrl * (scales(s) * u0);
                Tc = interp2(HX, HY, heatT, xc(1), xc(2), 'linear', 0);
                if Tc <= heat_thres
                    x_robot = xc;
                    scaled_ok = true;
                    break;
                end
            end
            if ~scaled_ok
                % cannot step without entering hot area: declare failure to force replan
                ok = false;
                x_new = x_robot;
                return;
            end
        else
            % safe to accept control
            x_robot = x_candidate;
        end

        % check arrival
        if norm(wp_goal - x_robot) < tol_waypoint
            ok = true;
            x_new = x_robot;
            return;
        end
        
    end

    ok = false;
    x_new = x_robot;
end

%% ===== heat helper functions =====
function heatT = apply_heat_source_continuous(heatT, HX, HY, robotPos, r_source, strength_per_sec, dt, cap)
    % Adds heat proportional to exposure time (dt). Strength is in "temp units per second".
    d2 = (HX - robotPos(1)).^2 + (HY - robotPos(2)).^2;
    mask = d2 <= r_source^2;
    if ~any(mask(:)), return; end
    % Optionally weight by distance (closer -> hotter)
    dist = sqrt(d2(mask));
    weight = (1 - dist / r_source);  % linear falloff
    weight(weight<0)=0;
    delta = strength_per_sec * dt .* weight;
    heatT(mask) = heatT(mask) + delta;
    heatT(heatT > cap) = cap;
end

function Tnew = diffuse_heat_fast(T, dx, alpha, dt)
    % Stable explicit diffusion using 5-point stencil implemented with circshift
    lap = (circshift(T, [1,0]) + circshift(T, [-1,0]) + ...
           circshift(T, [0,1]) + circshift(T, [0,-1]) - 4*T) / (dx^2);
    Tnew = T + alpha * dt * lap; 
    % simple Neumann BC behavior is automatically implied by circshift wrapping,
    % so reduce wrap effect by copying edges after update:
    Tnew(1,:) = Tnew(2,:); Tnew(end,:) = Tnew(end-1,:);
    Tnew(:,1) = Tnew(:,2); Tnew(:,end) = Tnew(:,end-1);
    Tnew(Tnew < 0) = 0;
end
