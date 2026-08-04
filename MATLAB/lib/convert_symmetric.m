function result = convert_symmetric(A)
    result = tril(A) + tril(A, -1).';
end