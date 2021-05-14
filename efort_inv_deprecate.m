function X = efort_inv_deprecate(kn_T0_6, L1, L2, L3, L4, L5, L8, fcn_T0_3)
    %fcn_T0_3 is a function handle type
    X = zeros(8, 6); %Each row corresponds with a unique set of solution(x1, x2,...,x6)
    p = kn_T0_6(1:3, 4);
    n = kn_T0_6(1:3, 1);
    o = kn_T0_6(1:3, 2);
    a = kn_T0_6(1:3, 3);

    % Rule for index: i stands for x3's choice,
    %j stands for x2's choice, k stands for x5's choice

    x1 = zeros(2, 2, 2);
    x2 = zeros(2, 2, 2);
    x3 = zeros(2, 2, 2);
    x4 = zeros(2, 2, 2);
    x5 = zeros(2, 2, 2);
    x6 = zeros(2, 2, 2);

    A = p(1) - L5 * a(1)
    B = p(2) - L5 * a(2)
    C = p(3) - L5 * a(3)

    D = (A - cos(x1(1)) * L8)^2 + (B - sin(x1(1)) * L8)^2 + (C - L1)^2 - L2^2 - L3^2 - L4^2;
    phi1 = atan2(L3, L4);
    x3(1, :, :) = asin(D / (2 * L2 * (L3^2 + L4^2)^0.5)) - phi1;
    x3(2, :, :) = pi - asin(D / (2 * L2 * (L3^2 + L4^2)^0.5)) - phi1;

    A_ = A * cos(x1(1)) + B * sin(x1(1));
    C_ = C - L1;
    phi2 = atan2(C_, -A_);

    for i = 1:2
        temp = (L4 * sin(x3(i, 1, 1)) + L3 * cos(x3(i, 1, 1)) + L2) / (A_^2 + C_^2)^0.5;
        x2(i, 1, :) = asin(temp) - phi2;
        x2(i, 2, :) = pi - asin(temp) - phi2;
    end

    for i = 1:2
        for j = 1:2
            op = sign(L4 * cos(x2(i, j, 1) + x3(i, j, 1)) - L3 * sin(x2(i, j, 1) + x3(i, j, 1)) + L8 - L2 * sin(x2(i, j, 1)));
            x1(i, j, :) = atan2(B*op, A*op);
        end
    end

    R3_6 = zeros(3, 3);

    for i = 1:2

        for j = 1:2
            temp = fcn_T0_3(L1, L2, L3, L8, x1(i, j, 1), x2(i, j, 1), x3(i, j, 1));
            R0_3 = temp(1:3, 1:3);
            R3_6 = inv(R0_3) * kn_T0_6(1:3, 1:3);
            n_ = R3_6(1:3, 1);
            o_ = R3_6(1:3, 2);
            a_ = R3_6(1:3, 3);

            x5(i, j, 1) = acos(a_(3));
            x5(i, j, 2) = -acos(a_(3));

            op = sign(sin(x5(i, j, 1)));
            x4(i, j, 1) = atan2(a_(2) * op, a_(1) * op);
            x6(i, j, 1) = atan2(o_(3) * op, -n_(3) * op);
            op = -op;
            x4(i, j, 2) = atan2(a_(2) * op, a_(1) * op);
            x6(i, j, 2) = atan2(o_(3) * op, -n_(3) * op);
        end

    end

    X = zeros(8, 6);
    count = 1;

    for i = 1:2

        for j = 1:2

            for k = 1:2
                X(count, :) = [x1(i, j, k), x2(i, j, k), x3(i, j, k), x4(i, j, k), x5(i, j, k), x6(i, j, k)];
                count = count + 1;
            end

        end

    end

end
