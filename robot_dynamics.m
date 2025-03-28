function x_dot = robot_dynamics(x, u, f, g)
    % ROBOT_DYNAMICS: Defines the dynamics of a mobile robot
    % x = Current state [position, velocity]
    % u = Control input
    % f = Drift function f(x) (Lipschitz continuous)
    % g = Control effectiveness function g(x)
    
    % Compute the state derivative
    x_dot = f(x) + g(x) * u;
end
