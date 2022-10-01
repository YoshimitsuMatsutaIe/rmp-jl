"""環境いろいろ"""



using Random
using Parameters

using StaticArrays
using ArraysOfArrays


function split_vec_of_arrays(u)
    vec.(u) |>
    x ->
    VectorOfSimilarVectors(x).data |>
    transpose |>
    VectorOfSimilarVectors
end

function rotate(a::T, b::T, c::T) where T
    a = deg2rad(a)
    b = deg2rad(b)
    c = deg2rad(c)
    return [
        1 0 0
        0 cos(a) -sin(a)
        0 sin(a) cos(a)
    ] * [
        cos(b) 0 sin(b)
        0 1 0
        -sin(b) 0 cos(b)
    ] * [
        cos(c) -sin(c) 0
        sin(c) cos(c) 0
        0 0 1
    ]
end

function rotate(a::T) where T
    a = deg2rad(a)
    return [
        cos(a) -sin(a)
        sin(a) cos(a)
    ]
end





sd = Random.seed!(123)


function _set_point(x::T, y::T, z::T) where T
    [x, y, z]
end

function _set_point(x::T, y::T) where T
    [x, y]
end


function _set_sphere(n::Int64, r::T, x::T, y::T) where T



end