function plotQuad(R, Q, Est)
% plot_estimation_results : Generates standard 3-sigma bounded plots for trajectory, 
%                           attitude, and estimation errors. Supports single runs
%                           as well as arrays of structs for Monte Carlo plotting.
%
% INPUTS:
%   R   - Command structure (or array of structures)
%   Q   - True simulated state structure (or array)
%   Est - Estimated state structure (or array)

    numRuns = length(Est);
    isMC = numRuns > 1;

    %% Adaptive Styling for Single vs. Monte Carlo modes
    if isMC
        % Scale transparency based on number of runs to keep density readable
        alpha_mc = min(1.0, max(0.05, 2/numRuns)); 
        lw_est = 0.5;
        lw_q = 0.5;
        lw_err = 0.5;
        ls_est = '-'; % Solid lines look cleaner than dotted when layered 100x
        
        c_est = [186/255, 153/255, 4/255, alpha_mc]; % RGBA Gold
        c_q   = [0, 0, 1, alpha_mc];                 % RGBA Blue
        c_r   = [1, 0, 0, alpha_mc];                 % RGBA Red
        c_g   = [0, 1, 0, alpha_mc];                 % RGBA Green
        c_b   = [0, 0, 1, alpha_mc];                 % RGBA Blue
    else
        lw_est = 2;
        lw_q = 3;
        lw_err = 2;
        ls_est = ':';
        
        c_est = '#ba9904';
        c_q   = 'b';
        c_r   = 'r';
        c_g   = 'g';
        c_b   = 'b';
    end

    %% Pre-process time vectors and theoretical bounds from the FIRST run
    t_patch = [Est(1).tVec(:); flipud(Est(1).tVec(:))];
    
    sigx_std = sqrt(squeeze(Est(1).PMat(1, 1, :))); 
    sigy_std = sqrt(squeeze(Est(1).PMat(2, 2, :))); 
    sigz_std = sqrt(squeeze(Est(1).PMat(3, 3, :))); 

    sigphi_std = sqrt(squeeze(Est(1).PMat(7, 7, :)));
    sigtheta_std = sqrt(squeeze(Est(1).PMat(8, 8, :)));
    sigpsi_std = sqrt(squeeze(Est(1).PMat(9, 9, :)));

    %% Setup Figures and Patches
    figure(1); clf; hold on; title('Vertical position of CM', 'FontSize', 20);
    patch(t_patch, [(Est(1).state.rMat(:, 3) + 3*sigz_std); flipud(Est(1).state.rMat(:, 3) - 3*sigz_std)], ...
        [186, 153, 4]/255, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    figure(2); clf; hold on; title('Horizontal position of CM', 'FontSize', 20);
    patch([(Est(1).state.rMat(:, 1) + 3*sigx_std); flipud(Est(1).state.rMat(:, 1) - 3*sigx_std)], ...
          [(Est(1).state.rMat(:, 2) + 3*sigy_std); flipud(Est(1).state.rMat(:, 2) - 3*sigy_std)], ...
          [186, 153, 4]/255, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
      
    figure(3); clf; hold on; title('Attitude (Euler Angles)', 'FontSize', 20);
    patch(t_patch, [(Est(1).state.eMat(:, 1) + 3*sigphi_std); flipud(Est(1).state.eMat(:, 1) - 3*sigphi_std)], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [(Est(1).state.eMat(:, 2) + 3*sigtheta_std); flipud(Est(1).state.eMat(:, 2) - 3*sigtheta_std)], 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [(Est(1).state.eMat(:, 3) + 3*sigpsi_std); flipud(Est(1).state.eMat(:, 3) - 3*sigpsi_std)], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    figure(4); clf; hold on; title('Position Estimation Error', 'FontSize', 20);
    patch(t_patch, [3*sigx_std(:); flipud(-3*sigx_std(:))], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigy_std(:); flipud(-3*sigy_std(:))], 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigz_std(:); flipud(-3*sigz_std(:))], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    figure(5); clf; hold on; title('Attitude Estimation Error', 'FontSize', 20);
    patch(t_patch, [3*sigphi_std(:); flipud(-3*sigphi_std(:))], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigtheta_std(:); flipud(-3*sigtheta_std(:))], 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigpsi_std(:); flipud(-3*sigpsi_std(:))], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    %% Plot Planned Trajectory (R) on Figure 2 first to keep legend order intact
    figure(2);
    plot(R(1).rIstar(:,1), R(1).rIstar(:,2), 'k-.', 'LineWidth', 3, 'HandleVisibility', 'on');

    %% Loop Over All Runs
    for i = 1:numRuns
        % Control legend visibility (only show the first run's lines in the legend)
        if i == 1
            hv = 'on';
            hv_est23 = 'off'; % Special handle for attitude figure estimates
        else
            hv = 'off';
            hv_est23 = 'off';
        end

        % Align Truth Data to Estimator Rate for Error Calculations
        rMat_true_aligned = interp1(Q(i).tVec, Q(i).state.rMat, Est(i).tVec, 'linear', 'extrap');
        eMat_true_aligned = interp1(Q(i).tVec, Q(i).state.eMat, Est(i).tVec, 'linear', 'extrap');

        err_r = rMat_true_aligned - Est(i).state.rMat;
        err_e = eMat_true_aligned - Est(i).state.eMat;
        err_e = mod(err_e + pi, 2*pi) - pi; % Wrap to [-pi, pi]

        % 1. Height Plot
        figure(1);
        plot(Q(i).tVec, Q(i).state.rMat(:,3), 'Color', c_q, 'LineStyle', '-', 'LineWidth', lw_q, 'HandleVisibility', hv);
        plot(Est(i).tVec, Est(i).state.rMat(:, 3), 'Color', c_est, 'LineStyle', ls_est, 'LineWidth', lw_est, 'HandleVisibility', hv);

        % 2. X-Y Plot
        figure(2);
        plot(Q(i).state.rMat(:,1), Q(i).state.rMat(:,2), 'Color', c_q, 'LineStyle', '-', 'LineWidth', lw_q, 'HandleVisibility', hv);
        plot(Est(i).state.rMat(:, 1), Est(i).state.rMat(:, 2), 'Color', c_est, 'LineStyle', ls_est, 'LineWidth', lw_est, 'HandleVisibility', hv);

        % 3. Attitude Plot
        figure(3);
        plot(Q(i).tVec, Q(i).state.eMat(:,1), 'Color', c_r, 'LineStyle', '-', 'LineWidth', lw_q, 'HandleVisibility', hv);
        plot(Q(i).tVec, Q(i).state.eMat(:,2), 'Color', c_g, 'LineStyle', '-', 'LineWidth', lw_q, 'HandleVisibility', hv);
        plot(Q(i).tVec, Q(i).state.eMat(:,3), 'Color', c_b, 'LineStyle', '-', 'LineWidth', lw_q, 'HandleVisibility', hv); 
        
        plot(Est(i).tVec, Est(i).state.eMat(:, 1), 'Color', c_est, 'LineStyle', ls_est, 'LineWidth', lw_est, 'HandleVisibility', hv);
        plot(Est(i).tVec, Est(i).state.eMat(:, 2), 'Color', c_est, 'LineStyle', ls_est, 'LineWidth', lw_est, 'HandleVisibility', hv_est23);
        plot(Est(i).tVec, Est(i).state.eMat(:, 3), 'Color', c_est, 'LineStyle', ls_est, 'LineWidth', lw_est, 'HandleVisibility', hv_est23);

        % 4. Position Error
        figure(4);
        plot(Est(i).tVec, err_r(:,1), 'Color', c_r, 'LineStyle', '-', 'LineWidth', lw_err, 'HandleVisibility', hv);
        plot(Est(i).tVec, err_r(:,2), 'Color', c_g, 'LineStyle', '-', 'LineWidth', lw_err, 'HandleVisibility', hv);
        plot(Est(i).tVec, err_r(:,3), 'Color', c_b, 'LineStyle', '-', 'LineWidth', lw_err, 'HandleVisibility', hv);

        % 5. Attitude Error
        figure(5);
        plot(Est(i).tVec, err_e(:,1), 'Color', c_r, 'LineStyle', '-', 'LineWidth', lw_err, 'HandleVisibility', hv);
        plot(Est(i).tVec, err_e(:,2), 'Color', c_g, 'LineStyle', '-', 'LineWidth', lw_err, 'HandleVisibility', hv);
        plot(Est(i).tVec, err_e(:,3), 'Color', c_b, 'LineStyle', '-', 'LineWidth', lw_err, 'HandleVisibility', hv);
        
        % Plot the body x-axis direction for Figure 2 ONLY on the first run to prevent clutter
        if i == 1
            figure(2);
            for j = 1:length(Q(1).tVec)
                if mod(j, 500) == 0
                    phi   = Q(1).state.eMat(j, 1);
                    theta = Q(1).state.eMat(j, 2);
                    psi   = Q(1).state.eMat(j, 3);
                    x_dir = cos(psi)*cos(theta) - sin(psi)*sin(phi)*sin(theta);
                    y_dir = sin(psi)*cos(theta) + cos(psi)*sin(phi)*sin(theta);
                    plot([Q(1).state.rMat(j, 1), Q(1).state.rMat(j, 1) + x_dir], ...
                         [Q(1).state.rMat(j, 2), Q(1).state.rMat(j, 2) + y_dir], 'r-', 'LineWidth', 2, 'HandleVisibility', 'off');
                end
            end
        end
    end

    %% Apply Final Formatting and Legends
    figure(1);
    plot(R(1).tVec, R(1).rIstar(:,3), 'k-.', 'LineWidth', 2, 'HandleVisibility', 'on'); 
    yline(0, 'k-.', 'HandleVisibility', 'off');
    xlabel('Time (sec)', 'FontSize', 20); ylabel('Vertical (m)', 'FontSize', 20); set(gca, 'FontSize', 14);
    legend('Flown Height', 'Estimated Height', 'Planned Height', 'FontSize', 12);

    figure(2);
    plot(0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 5, 'HandleVisibility', 'on');
    % Create dummy line for x_b direction legend handle
    plot(NaN, NaN, 'r-', 'LineWidth', 2, 'HandleVisibility', 'on');
    axis equal; grid on; xlabel('X (m)', 'FontSize', 20); ylabel('Y (m)', 'FontSize', 20); set(gca, 'FontSize', 14);
    legend('Planned Trajectory', 'Flown Trajectory', 'Estimated Trajectory', 'Circle Center', 'x_b Direction', 'FontSize', 12);

    figure(3);
    grid on; yline(0, 'k-.', 'HandleVisibility', 'off');
    xlabel('Time (sec)', 'FontSize', 20); ylabel('Angle (rad)', 'FontSize', 20); set(gca, 'FontSize', 14);
    legend('Phi', 'Theta', 'Psi', 'Estimates', 'FontSize', 12);

    figure(4);
    grid on; yline(0, 'k-.', 'HandleVisibility', 'off');
    xlabel('Time (sec)', 'FontSize', 20); ylabel('Error (m)', 'FontSize', 20); set(gca, 'FontSize', 14);
    legend('Error X', 'Error Y', 'Error Z', 'FontSize', 12);

    figure(5);
    grid on; yline(0, 'k-.', 'HandleVisibility', 'off');
    xlabel('Time (sec)', 'FontSize', 20); ylabel('Error (rad)', 'FontSize', 20); set(gca, 'FontSize', 14);
    legend('Error Phi', 'Error Theta', 'Error Psi', 'FontSize', 12);

end