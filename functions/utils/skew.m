function S = skew(v)
    % SKEW  Computes the skew symmetric matrix from a given vector v.
    %   SKEW(v) v is a vector in R^3.
    S = [0, -v(3), v(2);
        v(3), 0, -v(1);
        -v(2), v(1), 0];
end