using LinearAlgebra

struct id_map
end

"""何もしない"""
function (p::id_map)(
    x::Vector{T}, x_dot::Vector{T},
    out_x::Vector{T}, out_x_dot::Vector{T}, out_J::Matrix{T}, out_J_dot::Matrix{T}
    ) where T
    out_x .= x
    out_x_dot .= x_dot
    out_J .= Matrix(I, length(x), length(x))
    out_J_dot .= zero(out_J_dot)
end