function v = skew2vec(S)
    % skew2vec: Converts a skew-symmetric matrix to a vector.
    %
    % Input:
    %   S - 3x3 skew-symmetric matrix
    % Output:
    %   v - 3x1 vector [x; y; z]
    %
    % Example:
    %   S = [  0  -z   y;
    %           z   0  -x;
    %          -y   x   0];
    %   v = skew2vec(S); % [x; y; z]

    % Validate input
    if ~isequal(size(S), [3, 3])
        error('Input must be a 3x3 matrix.');
    end
    % if ~isequal(S, -S')
    %     error('Input must be a skew-symmetric matrix.');
    % end

    % Extract the vector from the skew-symmetric matrix
    v = [S(3, 2); S(1, 3); S(2, 1)];
end

