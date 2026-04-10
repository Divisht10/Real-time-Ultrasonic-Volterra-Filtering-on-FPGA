function B = custom_flipud(A)

    % Get the dimensions of the input matrix.
    [num_rows, num_cols] = size(A);

    % Pre-allocate an output matrix of the same size for efficiency.
    B = zeros(num_rows, num_cols);

    % Iterate through each row of the input matrix A.
    for i = 1:num_rows
        destination_row = num_rows - i + 1;
        
        % Copy the current row from A to its new position in B.
        B(destination_row, :) = A(i, :);
    end

end