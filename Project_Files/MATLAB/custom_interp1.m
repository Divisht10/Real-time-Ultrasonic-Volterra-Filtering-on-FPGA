function vq = custom_interp1(x,y,xq)
%x is the input original grid
%y is the output original grid
%xq is the new x grid, where we want the finer points to be placed
if length(x)~=length(y)
    error('Sample point and vectors should be of equal length');
end

%make all the vectors as column vectors
x=x(:);
y=y(:);
xq=xq(:);

%preallocating vector 
vq = NaN(size(xq));
parfor i = 1:length(xq)
    current_xq = xq(i);

    %handling the edge cases
    if current_xq < x(1) || current_xq > x(end)
        continue;
    end
    idx = 1;
    while idx <length(x) && x(idx+1) <current_xq
        idx = idx +1;
    end

    %if query point is exactly one of the sample points no interpolation
    %required
    if x(idx) == current_xq
        vq(i) = y(idx);
        continue;
    elseif idx<length(x) && x(idx+1)==current_xq
            vq(i) = y(idx+1);
            continue;
    end

    %if none of the above conditions are true then apply the linear
    %interpolation
    x1 = x(idx);
    x2 = x(idx+1);
    y1 = y(idx);
    y2 = y(idx+1);

    %calculating the fractional distance 't' of every query point
    t = (current_xq - x1)/(x2-x1);

    vq(i) = y1 * (1 - t) + y2 * t;
end
end