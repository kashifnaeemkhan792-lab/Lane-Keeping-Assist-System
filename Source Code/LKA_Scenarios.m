%% LKA System - Three Scenario Comparison
rng(42);

%% Common Parameters
L = 2.64; m = 1525; T = 0.1; N = 10; Nc = 5;
delta_max = deg2rad(35); delta_rate_max = deg2rad(5);
w1 = 30; w2 = 10; w3 = 3;
F_wind = 50; theta = deg2rad(1.72); g = 9.81;
noise_std = 0.03;
y0 = 0.6; psi0 = deg2rad(3);
t_total = 10; steps = t_total / T;
t_hist = 0:T:t_total;

%% Define 3 Scenarios
% Scenario 1: Straight road, 100 km/h
% Scenario 2: Curved road, 100 km/h  
% Scenario 3: Straight road, 60 km/h vs 120 km/h

speeds = [27.78, 27.78, 16.67, 33.33];  % m/s: 100, 100, 60, 120 km/h
labels = {'Straight 100km/h', 'Curved 100km/h', 'Straight 60km/h', 'Straight 120km/h'};
colors = {'b', 'r', 'g', 'm'};
use_curve = [false, true, false, false];  % which scenarios have road curvature

% Storage for all scenarios
all_y = zeros(4, steps+1);
all_psi = zeros(4, steps+1);
all_delta = zeros(4, steps);

%% Run Each Scenario
for sc = 1:4
    rng(42);  % Reset noise for fair comparison
    v = speeds(sc);
    a_dist = (F_wind/m) + g*sin(theta);
    
    y_hist = zeros(1, steps+1);
    psi_hist = zeros(1, steps+1);
    delta_hist = zeros(1, steps);
    y_hist(1) = y0;
    psi_hist(1) = psi0;
    delta_prev = 0;
    
    for k = 1:steps
        % Current time
        t_now = (k-1) * T;
        
        % Road curvature (only for scenario 2)
        if use_curve(sc)
            % Gentle curve: curvature changes like a winding road
            kappa = 0.001 * sin(0.5 * t_now);  % road curvature (1/m)
            psi_road = kappa * v * T;  % heading change from road curve
        else
            psi_road = 0;
        end
        
        % LIDAR measurement with noise
        y_meas = y_hist(k) + noise_std * randn();
        psi_meas = psi_hist(k) + deg2rad(0.2) * randn();
        
        % MPC Optimization
        delta_seq = ones(1, Nc) * delta_prev;
        
        for iter = 1:20
            for s = 1:Nc
                best_J_local = inf;
                best_val = delta_seq(s);
                
                test_vals = [delta_seq(s) - deg2rad(2), ...
                             delta_seq(s) - deg2rad(0.5), ...
                             delta_seq(s), ...
                             delta_seq(s) + deg2rad(0.5), ...
                             delta_seq(s) + deg2rad(2)];
                
                for t_idx = 1:length(test_vals)
                    trial = delta_seq;
                    trial(s) = test_vals(t_idx);
                    trial(s) = max(-delta_max, min(delta_max, trial(s)));
                    
                    if s == 1 && abs(trial(1) - delta_prev) > delta_rate_max
                        continue;
                    end
                    if s > 1 && abs(trial(s) - trial(s-1)) > delta_rate_max
                        continue;
                    end
                    
                    y_p = y_meas; psi_p = psi_meas; J = 0;
                    
                    for i = 1:N
                        if i <= Nc
                            d_i = trial(i);
                        else
                            d_i = trial(Nc);
                        end
                        psi_p = psi_p + T * (v / L) * tan(d_i) - psi_road;
                        y_p = y_p + T * v * sin(psi_p) + 0.5 * T^2 * a_dist;
                        J = J + w1 * y_p^2 + w2 * psi_p^2;
                    end
                    
                    J = J + w3 * (trial(1) - delta_prev)^2;
                    for s2 = 2:Nc
                        J = J + w3 * (trial(s2) - trial(s2-1))^2;
                    end
                    
                    if J < best_J_local
                        best_J_local = J;
                        best_val = test_vals(t_idx);
                    end
                end
                delta_seq(s) = max(-delta_max, min(delta_max, best_val));
            end
        end
        
        best_delta = delta_seq(1);
        delta_hist(k) = best_delta;
        
        psi_hist(k+1) = psi_hist(k) + T * (v / L) * tan(best_delta) - psi_road;
        y_hist(k+1) = y_hist(k) + T * v * sin(psi_hist(k)) + 0.5 * T^2 * a_dist;
        delta_prev = best_delta;
    end
    
    all_y(sc,:) = y_hist;
    all_psi(sc,:) = psi_hist;
    all_delta(sc,:) = delta_hist;
    
    disp([labels{sc}, ' --> Lateral: ', num2str(y_hist(end)*100,'%.1f'), ...
          'cm, Heading: ', num2str(rad2deg(psi_hist(end)),'%.2f'), ' deg'])
end

%% PLOTTING - Scenario 1: Straight Road 100 km/h (Baseline)
figure('Name', 'Scenario 1: Straight Road 100km/h', 'Position', [50 50 900 600])

subplot(3,1,1)
plot(t_hist, all_y(1,:)*100, 'b', 'LineWidth', 1.5); hold on
yline(0, 'g--', 'Lane Center', 'LineWidth', 1)
yline(91.25, 'r--', 'Lane Edge', 'LineWidth', 1)
yline(-91.25, 'r--', 'Lane Edge', 'LineWidth', 1)
xlabel('Time (s)'); ylabel('Lateral Position (cm)')
title('Scenario 1: Straight Road at 100 km/h - Lateral Position')
grid on

subplot(3,1,2)
plot(t_hist, rad2deg(all_psi(1,:)), 'b', 'LineWidth', 1.5); hold on
yline(0, 'g--', 'Desired', 'LineWidth', 1)
xlabel('Time (s)'); ylabel('Heading (deg)')
title('Heading Angle'); grid on

subplot(3,1,3)
stairs(t_hist(1:end-1), rad2deg(all_delta(1,:)), 'b', 'LineWidth', 1.2); hold on
yline(35, 'r--', 'LineWidth', 1); yline(-35, 'r--', 'LineWidth', 1)
xlabel('Time (s)'); ylabel('Steering (deg)')
title('MPC Steering Command'); grid on

%% PLOTTING - Scenario 2: Curved vs Straight
figure('Name', 'Scenario 2: Straight vs Curved Road', 'Position', [100 100 900 600])

subplot(3,1,1)
plot(t_hist, all_y(1,:)*100, 'b', 'LineWidth', 1.5); hold on
plot(t_hist, all_y(2,:)*100, 'r', 'LineWidth', 1.5)
yline(0, 'g--', 'LineWidth', 1)
yline(91.25, 'k--', 'LineWidth', 0.5); yline(-91.25, 'k--', 'LineWidth', 0.5)
xlabel('Time (s)'); ylabel('Lateral Position (cm)')
title('Lateral Position: Straight vs Curved Road at 100 km/h')
legend('Straight', 'Curved', 'Location', 'northeast'); grid on

subplot(3,1,2)
plot(t_hist, rad2deg(all_psi(1,:)), 'b', 'LineWidth', 1.5); hold on
plot(t_hist, rad2deg(all_psi(2,:)), 'r', 'LineWidth', 1.5)
yline(0, 'g--', 'LineWidth', 1)
xlabel('Time (s)'); ylabel('Heading (deg)')
title('Heading Angle: Straight vs Curved')
legend('Straight', 'Curved', 'Location', 'northeast'); grid on

subplot(3,1,3)
stairs(t_hist(1:end-1), rad2deg(all_delta(1,:)), 'b', 'LineWidth', 1.2); hold on
stairs(t_hist(1:end-1), rad2deg(all_delta(2,:)), 'r', 'LineWidth', 1.2)
yline(35, 'k--', 'LineWidth', 0.5); yline(-35, 'k--', 'LineWidth', 0.5)
xlabel('Time (s)'); ylabel('Steering (deg)')
title('Steering Command: Straight vs Curved')
legend('Straight', 'Curved', 'Location', 'northeast'); grid on

%% PLOTTING - Scenario 3: Speed Comparison (separate subplots)
figure('Name', 'Scenario 3: Speed Comparison', 'Position', [150 150 900 700])

subplot(3,1,1)
plot(t_hist, all_y(3,:)*100, 'g', 'LineWidth', 1.5); hold on
yline(0, 'k--', 'LineWidth', 0.5)
yline(91.25, 'r--', 'LineWidth', 0.5); yline(-91.25, 'r--', 'LineWidth', 0.5)
xlabel('Time (s)'); ylabel('Lateral Position (cm)')
title('60 km/h — Lateral: -4.5cm, Heading: 0.12°')
grid on

subplot(3,1,2)
plot(t_hist, all_y(1,:)*100, 'b', 'LineWidth', 1.5); hold on
yline(0, 'k--', 'LineWidth', 0.5)
yline(91.25, 'r--', 'LineWidth', 0.5); yline(-91.25, 'r--', 'LineWidth', 0.5)
xlabel('Time (s)'); ylabel('Lateral Position (cm)')
title('100 km/h — Lateral: 8.9cm, Heading: 1.42°')
grid on

subplot(3,1,3)
plot(t_hist, all_y(4,:)*100, 'm', 'LineWidth', 1.5); hold on
yline(0, 'k--', 'LineWidth', 0.5)
yline(91.25, 'r--', 'LineWidth', 0.5); yline(-91.25, 'r--', 'LineWidth', 0.5)
xlabel('Time (s)'); ylabel('Lateral Position (cm)')
title('120 km/h — Lateral: -15.2cm, Heading: 0.47°')
grid on