function result = cross_product(v, A)
% vector cross matrix
    result = convert_skew(v) * A;
end