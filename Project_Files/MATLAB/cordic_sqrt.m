function root = cordic_sqrt(n, num_iterations)
% calculates the square root using a CORDIC like algorithm
%setting the number of iterations as 50 if the second function argument is
%not given
if nargin<2
    num_iterations = 50;
end

if ~isnumeric(n) ||~isreal(n) || n<0 ||numel(n)~=1
    error('Input must be a non-negative real scalar');
end
%if the number is 0, return root as 0
if n==0
    root =0 ;
    return;
end
%for cordic algorithm to work best, we need to define the numbers in a
%fixed range, so we will do some normalization
%here we normalize the input 'n' to the range [0.25,1] by scaling it by
%powers of 4
%sqrt(n* 4^m) = sqrt(n) * 2^m
m = 0;
n_norm = n;
while n_norm>= 1.0
    n_norm = n_norm/4.0;
    m = m+1;
end
while n_norm <0.25
    n_norm = n_norm * 4.0;
    m = m-1;
end

%building the root bit by bit from msb to lsb
y=0.0; %initialize the root estimate 'y'
r = n_norm; %initializing the remainder why

for i = 1:num_iterations
    test_bit = 2^(-i); % equivalent to bit shift
    % The term to subtract is derived from (y + test_bit)^2
    % (y+b)^2 = y^2 + 2yb + b^2. The change is 2yb + b^2.
    % The remainder 'r' is currently n_norm - y^2.
    % We check if r >= 2yb + b^2
    trial_subtrahend = (2 * y + test_bit) * test_bit;
        
    if r>= trial_subtrahend
       r= r - trial_subtrahend;
       y =y + test_bit;
    end
end

%De_normalization
root = y*(2^m);
end

