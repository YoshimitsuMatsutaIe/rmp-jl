
module SiceKinematics

include("wrapper.jl")


const q_min = [
    (-3/4)π
    (-3/4)π
    (-3/4)π
    (-3/4)π
]
const q_max = [
    (3/4)π
    (3/4)π
    (3/4)π
    (3/4)π
]
const q_neutral = [
    0.0
    0.0
    0.0
    0.0
]

const ee_id = (5, 1)

const rss = (
    ((0., 0.),),
    ((0., 0.),),
    ((0., 0.),),
    ((0., 0.),),
    ((0., 0.),),
)


"""sinicose mapp"""
struct Mapping{T}
    frame::Int64
    index::Int64
    r::Tuple{T, T}
end

function Mapping(frame::Int64, index::Int64)
    #println(typeof(rss))
    Mapping(
        frame, index, rss[frame][index]
    )
end

function (p::Mapping)(
    q::Vector{T}, q_dot::Vector{T},
    out_x::Vector{T}, out_x_dot::Vector{T}, out_J::Matrix{T}, out_J_dot::Matrix{T}
) where T
    out_x .= rx(p.frame, q) .* p.r[1] .+ ry(p.frame, q) .* p.r[2] .+ o(p.frame, q)
    out_J .= jrx(p.frame, q) .* p.r[1] .+ jry(p.frame, q) .* p.r[2] .+ jo(p.frame, q)
    out_x_dot .= out_J * q_dot
    out_J_dot .= jrx_dot(p.frame, q, q_dot) .* p.r[1] .+ jry_dot(p.frame, q, q_dot) .* p.r[2] .+ jo_dot(p.frame, q, q_dot)
end


end

