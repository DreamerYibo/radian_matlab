function T = DH_T_2(theta_i, di, ai, alpha_i)
    %DH-method, get T. unit of theta and alpha:: rad
    %Note that there is different input order!!!
    % d_r = pi/180;
    xi = theta_i;
    yi = alpha_i;

    if yi == pi / 2 % SYMS simplify has a bug as cos(pi/2) != 0. Add this to fix that bug.
        T = [cos(xi), 0, sin(xi), ai * cos(xi); ...
                sin(xi), 0, -cos(xi), ai * sin(xi); ...
                0, 1, 0, di; ...
                0, 0, 0, 1];
    elseif yi == -pi / 2
        T = [cos(xi), 0, -sin(xi), ai * cos(xi); ...
                sin(xi), 0, cos(xi), ai * sin(xi); ...
                0, -1, 0, di; ...
                0, 0, 0, 1];
    else
        T = [cos(xi), -sin(xi) * cos(yi), sin(xi) * sin(yi), ai * cos(xi); ...
                sin(xi), cos(xi) * cos(yi), -cos(xi) * sin(yi), ai * sin(xi); ...
                0, sin(yi), cos(yi), di; ...
                0, 0, 0, 1];
    end

end