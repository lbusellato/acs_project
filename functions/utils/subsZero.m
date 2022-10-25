function A = subsZero(A, threshold)
    % SUBSZERO Removes very small coefficients from a symbolic matrix.
    %   A = SUBSZERO(A) where A is a symbolic matrix
    
    % Check if a threshold was provided, if not set it to a default
    if nargin == 1
        threshold = 1e-4;
    end
    % Cycle over the matrix components
    for i = 1:size(A,1)
        for j = 1:size(A,2)
            % Extract the coefficients C from the matrix
            [C, T] = coeffs(A(i,j));
            % Eliminate small coefficients
            C(C<threshold) = 0;
            % Put the result back into the matrix
            A(i,j) = dot(C, T);
        end
    end
end