function plotQuad(R, Q, Est)
% plot_estimation_results : Generates standard 3-sigma bounded plots for trajectory, 
%                           attitude, and estimation errors.
%
% INPUTS:
%   R   - Command structure (contains tVec, rIstar)
%   Q   - True simulated state structure (contains tVec, state.rMat, state.eMat)
%   Est - Estimated state structure (contains tVec, state.rMat, state.eMat, PMat)

    %% Pre-process time vectors and bounds
    t_patch = [Est.tVec(:); flipud(Est.tVec(:))];
    
    % Standard deviations for position
    sigx_std = sqrt(squeeze(Est.PMat(1, 1, :))); 
    sigy_std = sqrt(squeeze(Est.PMat(2, 2, :))); 
    sigz_std = sqrt(squeeze(Est.PMat(3, 3, :))); 

    % Standard deviations for attitude
    sigphi_std = sqrt(squeeze(Est.PMat(7, 7, :)));
    sigtheta_std = sqrt(squeeze(Est.PMat(8, 8, :)));
    sigpsi_std = sqrt(squeeze(Est.PMat(9, 9, :)));

    %% Align Truth Data to Estimator Rate for Error Calculations
    % Interpolates the M-length true states at the N-length estimator times
    rMat_true_aligned = interp1(Q.tVec, Q.state.rMat, Est.tVec, 'linear', 'extrap');
    eMat_true_aligned = interp1(Q.tVec, Q.state.eMat, Est.tVec, 'linear', 'extrap');

    % Calculate Errors (Truth - Estimate)
    err_r = rMat_true_aligned - Est.state.rMat;
    err_e = eMat_true_aligned - Est.state.eMat;
    
    % Wrap attitude errors to [-pi, pi] to prevent artificial spikes on crossovers
    err_e = mod(err_e + pi, 2*pi) - pi;

    %% 1. Height Plot
    figure(1); clf; hold on;
    
    z_upper = Est.state.rMat(:, 3) + 3*sigz_std;
    z_lower = Est.state.rMat(:, 3) - 3*sigz_std;
    patch(t_patch, [z_upper(:); flipud(z_lower(:))], [186, 153, 4]/255, ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    plot(Q.tVec, Q.state.rMat(:,3), 'b-', 'LineWidth', 3); grid on;
    plot(Est.tVec, Est.state.rMat(:, 3), 'Color', '#ba9904', 'LineStyle', ':', 'LineWidth', 2);
    plot(R.tVec, R.rIstar(:,3), 'k-.', 'LineWidth', 2); 
    
    xlabel('Time (sec)', 'FontSize', 20);
    ylabel('Vertical (m)', 'FontSize', 20);
    set(gca, 'FontSize', 14);
    title('Vertical position of CM', 'FontSize', 20);
    legend('Flown Height', 'Estimated Height', 'Planned Height', 'FontSize', 12);

    %% 2. X-Y Plot
    figure(2); clf; hold on;
    
    x_upper = Est.state.rMat(:, 1) + 3*sigx_std;
    x_lower = Est.state.rMat(:, 1) - 3*sigx_std;
    y_upper = Est.state.rMat(:, 2) + 3*sigy_std;
    y_lower = Est.state.rMat(:, 2) - 3*sigy_std;
    patch([x_upper(:); flipud(x_lower(:))], [y_upper(:); flipud(y_lower(:))], [186, 153, 4]/255, ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    plot(R.rIstar(:,1), R.rIstar(:,2), 'k-.', 'LineWidth', 3);
    plot(Q.state.rMat(:,1), Q.state.rMat(:,2), 'b-', 'LineWidth', 3);
    plot(Est.state.rMat(:, 1), Est.state.rMat(:, 2), 'Color', '#ba9904', 'LineStyle', ':', 'LineWidth', 2);

    plot(0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 5);
    
    % Calculate true body x-axis direction assuming a 3-1-2 Euler sequence (Yaw, Roll, Pitch)
    for i = 1:length(Q.tVec)
        if mod(i, 500) == 0
            phi   = Q.state.eMat(i, 1);
            theta = Q.state.eMat(i, 2);
            psi   = Q.state.eMat(i, 3);
            
            % First column of R_B^I for 3-1-2 sequence
            x_dir = cos(psi)*cos(theta) - sin(psi)*sin(phi)*sin(theta);
            y_dir = sin(psi)*cos(theta) + cos(psi)*sin(phi)*sin(theta);
            
            plot([Q.state.rMat(i, 1), Q.state.rMat(i, 1) + x_dir], ...
                 [Q.state.rMat(i, 2), Q.state.rMat(i, 2) + y_dir], 'r-', 'LineWidth', 2);
        end
    end
    
    axis equal; grid on;
    xlabel('X (m)', 'FontSize', 20);
    ylabel('Y (m)', 'FontSize', 20);
    set(gca, 'FontSize', 14);
    title('Horizontal position of CM', 'FontSize', 20);
    legend('Planned Trajectory', 'Flown Trajectory', 'Estimated Trajectory', 'Circle Center', 'x_b Direction', 'FontSize', 12);

    %% 3. Attitude Plot
    figure(3); clf; hold on;

    phi_u = Est.state.eMat(:, 1) + 3*sigphi_std;
    phi_l = Est.state.eMat(:, 1) - 3*sigphi_std;
    patch(t_patch, [phi_u(:); flipud(phi_l(:))], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    theta_u = Est.state.eMat(:, 2) + 3*sigtheta_std;
    theta_l = Est.state.eMat(:, 2) - 3*sigtheta_std;
    patch(t_patch, [theta_u(:); flipud(theta_l(:))], 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    psi_u = Est.state.eMat(:, 3) + 3*sigpsi_std;
    psi_l = Est.state.eMat(:, 3) - 3*sigpsi_std;
    patch(t_patch, [psi_u(:); flipud(psi_l(:))], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    plot(Q.tVec, Q.state.eMat(:,1), 'r-', 'LineWidth', 3);
    plot(Q.tVec, Q.state.eMat(:,2), 'g-', 'LineWidth', 3);
    plot(Q.tVec, Q.state.eMat(:,3), 'b-', 'LineWidth', 3); 
    
    plot(Est.tVec, Est.state.eMat(:, 1), 'Color', '#ba9904', 'LineStyle', ':', 'LineWidth', 2);
    plot(Est.tVec, Est.state.eMat(:, 2), 'Color', '#ba9904', 'LineStyle', ':', 'LineWidth', 2);
    plot(Est.tVec, Est.state.eMat(:, 3), 'Color', '#ba9904', 'LineStyle', ':', 'LineWidth', 2);

    grid on;
    yline(0, 'k-.');
    xlabel('Time (sec)', 'FontSize', 20);
    ylabel('Angle (rad)', 'FontSize', 20);
    set(gca, 'FontSize', 14);
    title('Attitude (Euler Angles)', 'FontSize', 20);
    legend('Phi', 'Theta', 'Psi', 'Estimates', 'FontSize', 12);

    %% 4. Position Error Plot
    figure(4); clf; hold on;
    
    % Zero-centered 3-sigma bounds
    patch(t_patch, [3*sigx_std(:); flipud(-3*sigx_std(:))], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigy_std(:); flipud(-3*sigy_std(:))], 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigz_std(:); flipud(-3*sigz_std(:))], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    plot(Est.tVec, err_r(:,1), 'r-', 'LineWidth', 2);
    plot(Est.tVec, err_r(:,2), 'g-', 'LineWidth', 2);
    plot(Est.tVec, err_r(:,3), 'b-', 'LineWidth', 2);

    grid on;
    yline(0, 'k-.');
    xlabel('Time (sec)', 'FontSize', 20);
    ylabel('Error (m)', 'FontSize', 20);
    set(gca, 'FontSize', 14);
    title('Position Estimation Error', 'FontSize', 20);
    legend('Error X', 'Error Y', 'Error Z', 'FontSize', 12);

    %% 5. Attitude Error Plot
    figure(5); clf; hold on;

    % Zero-centered 3-sigma bounds
    patch(t_patch, [3*sigphi_std(:); flipud(-3*sigphi_std(:))], 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigtheta_std(:); flipud(-3*sigtheta_std(:))], 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    patch(t_patch, [3*sigpsi_std(:); flipud(-3*sigpsi_std(:))], 'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');

    plot(Est.tVec, err_e(:,1), 'r-', 'LineWidth', 2);
    plot(Est.tVec, err_e(:,2), 'g-', 'LineWidth', 2);
    plot(Est.tVec, err_e(:,3), 'b-', 'LineWidth', 2);

    grid on;
    yline(0, 'k-.');
    xlabel('Time (sec)', 'FontSize', 20);
    ylabel('Error (rad)', 'FontSize', 20);
    set(gca, 'FontSize', 14);
    title('Attitude Estimation Error', 'FontSize', 20);
    legend('Error Phi', 'Error Theta', 'Error Psi', 'FontSize', 12);

end