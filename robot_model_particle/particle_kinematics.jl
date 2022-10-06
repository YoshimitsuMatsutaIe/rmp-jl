


module ParticleKinematics
using LinearAlgebra

const c_dim = 2
const t_dim = 2

const q_min = [
    -1e10
    -1e10
]
const q_max = -q_min

const q_neutral = [
    0.0
    0.0
]

const ee_id = (1, 1)

const rss = (
    ((0., 0.),),
)


"""point mapp"""
struct Mapping{T}
    frame::Int64
    index::Int64
    r::Tuple{T, T}
end

function Mapping(frame::Int64, index::Int64)
    println(typeof(rss))
    Mapping(
        frame, index, rss[frame][index]
    )
end

function (p::Mapping)(
    q::Vector{T}, q_dot::Vector{T},
    out_x::Vector{T}, out_x_dot::Vector{T}, out_J::Matrix{T}, out_J_dot::Matrix{T}
) where T
    out_x .= q
    out_J .= Matrix{T}(I, length(q), length(q))
    out_x_dot .= q_dot
    out_J_dot .= zero(out_J)
end


end