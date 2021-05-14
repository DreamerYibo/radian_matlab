function [dist, intersect_point1, intersect_point2] = find_dist_between_3d_lines(p1, v1, p2, v2)

    %p is the start point (3d col vector), v is the normal orientation vector (3d col vector)
    %intersect_point1: where common orthogonal line intersects with line 1(or 2)
    if (norm(cross(v1, v2)) ~= 0) % lin1 and lin2 are not parralell
        ortho_v = cross(v1, v2);
        ortho_v = ortho_v ./ sqrt(sum(ortho_v.^2));
        dist = abs(dot((p2 - p1), ortho_v));

        p1_p2 = p2 - p1;
        % dot(p1_p2, ortho_v) ;
        o1_o2 = dot(p1_p2, ortho_v) * ortho_v;

        t = zeros(2,1); 
        V = [-v1, v2];
        B = p1-p2+o1_o2;
        t = linsolve(V,B) % solve the 3X2 2X1 = 3X1 linear systems. The algorithm is QR factorization.
        error1 = B - V*t;

        % if (any(abs(diff(t)) > 1e-8))
        %     error("wrong algorithms")
        % end

        intersect_point1 = p1 + t(1)*v1;
        intersect_point2 = p2 + t(2)*v2;
    else
        dist = nan;
        intersect_point1 = nan;
        intersect_point2 = nan;
    end

end
