function [c_ineq, c_eq] = constraint(u, v0, N, T_MPC, theta)
    v = zeros(N+1, 1);
    v(1) = v0;

    % Constants
    mu = 0.7;
    m = 2500;
    g = 9.8;
    rho = 1.225;
    Cd = 0.28;
    Aref = 2.5;

    a_min = -3.5;  
    a_max = 3.5;   
    c_ineq = zeros(2*N, 1);  % [a - a_max; a_min - a]

    for i = 1:N
        drag_force = 0.5 * rho * Cd * Aref * v(i)^2;
        gravity_force = g * sin(theta);
        rolling_resistance = mu * g * cos(theta);

        acc = (u(i)/m - drag_force/m - gravity_force - rolling_resistance);
        v(i+1) = v(i) + acc * T_MPC;

        a = acc;  
        
        % Inequality constraints: enforce a_min ≤ a ≤ a_max
        c_ineq(i)     = a - a_max;  
        c_ineq(i+N)   = a_min - a;  
    end

    c_eq = [];  % No equality constraints
end
