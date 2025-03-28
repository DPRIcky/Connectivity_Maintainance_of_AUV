function u_opt = solve_qp(nominal_control, u_sbc, u_cbc, u_los_cbc, umax)
    % SOLVE_QP: Solves the Quadratic Programming (QP) problem for optimal control
    % nominal_control -> Nx2 matrix (task-related control)
    % u_sbc, u_cbc, u_los_cbc -> Nx2 matrices (CBF constraints)
    % umax -> Maximum control input
    % Returns:
    %   u_opt -> Nx2 matrix of optimal control inputs

    N = size(nominal_control, 1);
    d = size(nominal_control, 2); % 2D control input

    % Define QP cost function: Minimize || u - nominal_control ||²
    H = 2 * eye(N * d); % Quadratic term (identity matrix)
    f = -2 * reshape(nominal_control, [], 1); % Linear term

    % Constraints: Combine CBF constraints
    A = [reshape(u_sbc, [], 1), reshape(u_cbc, [], 1), reshape(u_los_cbc, [], 1)]';
    b = zeros(size(A, 1), 1); % Constraints must be non-negative

    % Control bounds
    lb = -umax * ones(N * d, 1);
    ub = umax * ones(N * d, 1);

    % Solve QP using MATLAB's quadprog solver
    options = optimset('Display', 'off');
    u_opt_vec = quadprog(H, f, A, b, [], [], lb, ub, [], options);

    % Reshape result back to Nx2 format
    if isempty(u_opt_vec)
        u_opt = nominal_control; % Fallback to nominal control if solver fails
    else
        u_opt = reshape(u_opt_vec, N, d);
    end
end
