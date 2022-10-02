using StaticArrays
using ArraysOfArrays


function split_vec_of_arrays(u)
    vec.(u) |>
    x ->
    VectorOfSimilarVectors(x).data |>
    transpose |>
    VectorOfSimilarVectors
end

"""辞書のキーを文字列からシンボルに変換"""
function keytosymbol(x)
    Dict(Symbol(k) => v for (k, v) in pairs(x))
end




# hoge = Dict(
#     "x" => 1.0,
#     "y" => 2.0
# )

# f(;x, y) = x + y


# p = keytosymbol(hoge)

# f(;p...)