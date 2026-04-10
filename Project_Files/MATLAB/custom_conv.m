function output_matrix = custom_conv(input1, input2)
%function to get a custom convolution 
% getting the dimensions of the input arrays
[rows1, cols1] = size(input1);
[rows2, cols2] = size(input2);

%padding 
padrows = floor(rows2/2);
padcols = floor(cols2/2);

padded_rows_total = rows1 + 2 * padrows;
padded_cols_total = cols1 + 2 * padcols;
paddedInput = zeros(padded_rows_total, padded_cols_total);

%Copy the original input matrix into the center of the new padded matrix
start_row = padrows + 1;
end_row = start_row + rows1 - 1;
start_col = padcols + 1;
end_col = start_col + cols1 - 1;

paddedInput(start_row:end_row, start_col:end_col) = input1;

%initializing the output matrix
output_matrix = zeros(rows1,cols1);

for i = 1:rows1
    for j = 1:cols1

        region = paddedInput(i:(i+rows2 - 1), j:(j+cols2 -1));

        conv_sum = sum(region .* input2,'all');

        output_matrix(i,j) = conv_sum;
    end
end
end
