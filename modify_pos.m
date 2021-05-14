function X_mod = modify_pos(X) % modify the 6 joint positions in [-pi, pi]

    X_mod = X;

    for i = 1:length(X_mod)

        if X_mod(i) > pi

            while X_mod(i) > pi
                X_mod(i) = X_mod(i) - 2 * pi;
            end

        elseif X_mod(i) < -pi

            while X_mod(i) < -pi
                X_mod(i) = X_mod(i) + 2 * pi;
            end

        end

    end

end
