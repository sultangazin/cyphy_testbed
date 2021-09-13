function y = selIndex(i, numCoeff)
    %{
    Generate the indexes to select a subvector made of pieces of
    length numCoeff
    %}
    y = (numCoeff * (i - 1) + 1 : numCoeff * i);
end