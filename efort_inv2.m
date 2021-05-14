function X = efort_inv2(kn_T0_6, L1, L2, L3, L4, L5, L8, fcn_T0_3, fcn_T0_4) %Fix some problems of version1

    p = kn_T0_6(1:3, 4);
    n = kn_T0_6(1:3, 1);
    o = kn_T0_6(1:3, 2);
    a = kn_T0_6(1:3, 3);

    x123_true = zeros(4, 3);
    x123_guess = zeros(8, 3);

    A = p(1) - L5 * a(1);
    B = p(2) - L5 * a(2);
    C = p(3) - L5 * a(3);

    x123_guess(1:4, 1) = atan2(B, A);
    x123_guess(5:end, 1) = atan2(-B, -A);

    for i = 1:4:8 % Guess x3
        x1 = x123_guess(i, 1);
        D = (A - cos(x1) * L8)^2 + (B - sin(x1) * L8)^2 + (C - L1)^2 - L2^2 - L3^2 - L4^2;
        phi1 = atan2(L3, L4);
        x3_guess = asin(D / (2 * L2 * (L3^2 + L4^2)^0.5)) - phi1;
        x3_guess2 = pi - asin(D / (2 * L2 * (L3^2 + L4^2)^0.5)) - phi1;

        if (imag(x3_guess) ~= 0)
            x123_guess(i:i + 1, 3) = nan;
            x123_guess(i + 2:i + 3, 3) = nan;
        else
            x123_guess(i:i + 1, 3) = x3_guess;
            x123_guess(i + 2:i + 3, 3) = x3_guess2;
        end

    end

    for i = 1:2:8 % Guess x2
        x1 = x123_guess(i, 1);
        x3 = x123_guess(i, 3);

        if (isnan(x3))
            x123_guess(i, 2) = nan;
            x123_guess(i + 1, 2) = nan;
        else
            A_ = L3 * cos(x3) + L4 * sin(x3) + L2;
            B_ = -L3 * sin(x3) + L4 * cos(x3);
            phi2 = atan2(A_, B_);

            temp = (C - L1) / (A_^2 + B_^2)^0.5;
            x123_guess(i, 2) = asin(temp) - phi2;
            x123_guess(i + 1, 2) = pi - asin(temp) - phi2;
        end

    end

    count = 0;

    for i = 1:8
        x2 = x123_guess(i, 2);
        x3 = x123_guess(i, 3);

        if (isnan(x3))
            continue
        else
            op = sign(L4 * cos(x2 + x3) - L3 * sin(x2 + x3) + L8 - L2 * sin(x2));
            x1_true = atan2(B * op, A * op);

            if x123_guess(i, 1) == x1_true
                count = count + 1;
                x123_true(count, :) = [x1_true, x2, x3];
            end

        end

    end

    % x123_true * 180 / pi
    % x123_guess * 180 / pi %debug
    size_x123_true = count;

    ind_to_delete = 0;

    pos_value_fake = zeros(3, 1); %calculate pos value if size_x123_true > 4
    times_pos_value = []; % How many times occurrs for each pos value.
    pos_container = zeros(3, size_x123_true);

    if (size_x123_true < 4)
        x123_true(size_x123_true + 1:4, :) = [];
    elseif (size_x123_true > 4) % some solutions are fake

        for i = 1:size_x123_true
            x1 = x123_true(i, 1);
            x2 = x123_true(i, 2);
            x3 = x123_true(i, 3);
            x4 = 0;
            temp = fcn_T0_4(L1, L2, L3, L4, L8, x1, x2, x3, x4);
            pos_temp = temp(1:3, 4);
            pos_container(:, i) = pos_temp;

            if (i == 1)
                pos_value_fake = pos_temp;
                times_pos_value = 1;
            else
                flag_has_same = 1;

                for j = 1:size(times_pos_value,2) %Update the information

                    if any(abs(pos_temp - pos_value_fake(:, j)) > 1e-3)
                        flag_has_same = 0;
                    else
                        times_pos_value(j) = times_pos_value(j) + 1;
                        flag_has_same = 1;
                        break
                    end

                end

                if flag_has_same == 0 % no value is the same with current pos_temp
                    pos_value_fake(:, end + 1) = pos_temp;
                    times_pos_value(end + 1) = 1;
                end

            end
            
        end
        pos_container;
        pos_value_fake ;%debug
        times_pos_value; %debug
        [~, true_value_ind] = max(times_pos_value);

        for i = 1:size_x123_true

            if any(abs(pos_container(:, i) - pos_value_fake(:, true_value_ind)) > 1e-3)
                ind_to_delete(end + 1) = i;
            end

        end

        x123_true(ind_to_delete(2:end), :) = [];
        size_x123_true = size(x123_true,1); %update the size value

    end

    x123_true * 180 / pi;
    % for i = 1:count
    %     x1 = x123_true(i, 1);
    %     x2 = x123_true(i, 2);
    %     x3 = x123_true(i, 3);
    %     x4 = 0;
    %     fcn_T0_4(L1, L2, L3, L4, L8, x1, x2, x3, x4)
    % end

    % for i = 1:8
    %     x1 = x123_guess(i, 1);
    %     x2 = x123_guess(i, 2);
    %     x3 = x123_guess(i, 3);
    %     x4 = 0;
    %     fcn_T0_4(L1, L2, L3,L4, L8, x1, x2, x3, x4)
    % end

    R3_6 = zeros(3, 3);
    X = zeros(2 * size_x123_true, 6); %Each row corresponds with a unique set of solution(x1, x2,...,x6)

    for i = 1:2:2 * size_x123_true

        ind = (i - mod(i, 2)) / 2 + 1;
        x1 = x123_true(ind, 1);
        x2 = x123_true(ind, 2);
        x3 = x123_true(ind, 3);
        X(i:i + 1, 1) = x1;
        X(i:i + 1, 2) = x2;
        X(i:i + 1, 3) = x3;

        temp = fcn_T0_3(L1, L2, L3, L8, x1, x2, x3);
        R0_3 = temp(1:3, 1:3);
        R3_6 = inv(R0_3) * kn_T0_6(1:3, 1:3);
        n_ = R3_6(1:3, 1);
        o_ = R3_6(1:3, 2);
        a_ = R3_6(1:3, 3);

        X(i, 5) = acos(a_(3));
        X(i + 1, 5) = -acos(a_(3));

        op = sign(sin(X(i, 5)));
        X(i, 4) = atan2(a_(2) * op, a_(1) * op);
        X(i, 6) = atan2(o_(3) * op, -n_(3) * op);
        op = -op;
        X(i + 1, 4) = atan2(a_(2) * op, a_(1) * op);
        X(i + 1, 6) = atan2(o_(3) * op, -n_(3) * op);

    end

end
