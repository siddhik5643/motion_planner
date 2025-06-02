function J = objective(u, v0, v_ref, N, T_MPC, theta)
    v = zeros(N+1,1);
    v(1) = v0;

    % Constants
    mu = 0.7;
    m = 2500;
    g = 9.8;
    rho = 1.225;
    Cd = 0.28;
    Aref = 2.5;

    
    v_ref_vec = v_ref * ones(N+1, 1);

    for i = 1:N
        drag_force = 0.5 * rho * Cd * Aref * v(i)^2;
        gravity_force = g * sin(theta);
        rolling_resistance = mu * g * cos(theta);
        v(i+1) = v(i) + ((u(i)/m - drag_force/m - gravity_force - rolling_resistance) * T_MPC);
    end

    %entire prediction horizon
    J = sum((v_ref_vec - v).^2)*10000;
end
