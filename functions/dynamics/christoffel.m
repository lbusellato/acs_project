function C = christoffel(B)
    % CHRISTOFFEL Computes the Christoffel symbols of the first type and
    % constructs the C matrix.
    %   C = CHRISTOFFEL(B) where B is the inertial matrix.
    
    % Set up symbols for the joint variables
    syms d0 a1 a3 d2 q1(t) q2(t) q3(t) dq1(t) dq2(t) dq3(t) h1 a2 h3 c2 b2 r1 r3
    dq = [dq1(t), dq2(t), dq3(t)];
    
    % Compute the inertial matrix derivatives
    dB1 = diff(B, q1(t));
    dB2 = diff(B, q2(t));
    dB3 = diff(B, q3(t));
    dB = cat(3, dB1, dB2, dB3);
    % Construct the C matrix
    C = sym(zeros(3));
    for i = 1:size(B,1)
        for j = i:size(B,1)
            for k = 1:size(B,1)
                % Apply the formula
                C(i,j) = C(i,j) + 0.5*(dB(i,j,k) + dB(i,k,j) - dB(j,k,i))*dq(k);
                C(j,i) = C(i,j); % C is symmetric!
            end
        end
    end
    % Simplify if possible
    C = simplify(C);
end