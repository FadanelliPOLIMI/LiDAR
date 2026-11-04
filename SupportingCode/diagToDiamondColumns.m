function result = diagToDiamondColumns(A)
    % Input: A is an MxN matrix (possibly non-square)
    % Output: RxC matrix with diagonals of A as columns, main diagonal in center column,
    % skipping diagonals with >50% empty values, forming a diamond pattern

    % Get input dimensions
    [M, N] = size(A);

    % Maximum dimension for diagonal length
    max_dim = max(M, N);
    min_dim = min(M, N);

    % Define all possible diagonal offsets
    diags = -(M-1):N-1;

    % Filter diagonals with at least 50% non-empty values
    valid_diags = [];

    for d = diags
        len = min(M - max(0, d), N - max(0, -d)); % Length of diagonal
        if len >= max_dim / 2 % Keep if at least 50% non-empty
            valid_diags = [valid_diags, d];
        end
    end

    % Number of valid diagonals (columns in output)
    num_valid_diags = length(valid_diags);

    if num_valid_diags == 0
        result = zeros(min_dim, 0); % Empty output if no valid diagonals
        return;
    end

    % Output dimensions: rows = longest diagonal length, cols = valid diagonals
    R = min_dim; % Rows equal to main diagonal length
    C = num_valid_diags; % Columns equal to number of valid diagonals

    % Initialize output matrix
    result = zeros(R, C);

    % Find index of main diagonal (d=0) in valid_diags
    main_diag_idx = find(valid_diags == 0);

    if isempty(main_diag_idx)
        warning('Main diagonal skipped due to emptiness criterion.');
        main_diag_idx = ceil(C/2); % Fallback to approximate center
    end

    % Center column in output
    center_col = main_diag_idx;

    % Extract all valid diagonals using spdiags
    B = spdiags(A, valid_diags);

    % Place each diagonal as a column
    for k = 1:num_valid_diags
        d = valid_diags(k);
        diag_vals = B(:, k); % Get k-th diagonal values
        len = min(M - max(0, d), N - max(0, -d)); % Length of this diagonal

        % Trim padding zeros
        if d >= 0
            diag_vals = diag_vals(1:len);
        else
            diag_vals = diag_vals(end-len+1:end);
        end

        % Calculate row indices to center the diagonal vertically
        row_start = floor((R - len + 1) / 2) + 1;
        row_indices = row_start:(row_start + len - 1);

        % Column index in output (relative to center)
        col = center_col + (k - main_diag_idx);

        % Place diagonal values using vectorized indexing
        if col > 0 && col <= C
            result(row_indices, col) = diag_vals;
        end
    end
end