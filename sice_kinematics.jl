
"""
sice用のモジュール
"""
module SiceKinematics

export Mapping
export q_min, q_max, q_neutral
export  c_dim, t_dim

const c_dim = 4
const t_dim = 2

const l1 = 1.0
const l2 = 1.0
const l3 = 1.0
const l4 = 1.0

const q_min = [(-3/4)π+(1/2)π, (-3/4)π, (-3/4)π, (-3/4)π]
const q_max = [(3/4)π+(1/2)π, (3/4)π, (3/4)π, (3/4)π]
const q_neutral = [(1/2)π, 0.0, 0.0, 0.0]

"""sice mapp"""
struct Mapping
    n::Int64
end

function (p::Mapping)(
    q::Vector{T}, q_dot::Vector{T},
    out_x::Vector{T}, out_x_dot::Vector{T}, out_J::Matrix{T}, out_J_dot::Matrix{T}
) where T
    out_x .= phi(p.n, q)
    out_J .= J(p.n, q)
    out_J_dot .= J_dot(p.n, q, q_dot)
    out_x_dot .= out_J * q_dot
end


function phi(n::Int64, q::Vector{T}) where T
    if n == 1
        return [
            l1*cos(q[1])
            l1*sin(q[1])
        ]
    elseif  n == 2
        return [
            l1*cos(q[1]) + l2*cos(q[1] + q[2])
            l1*sin(q[1]) + l2*sin(q[1] + q[2])
        ]
    elseif  n == 3
        return [
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3])
            l1*sin(q[1]) + l2*sin(q[1] + q[2]) + l3*sin(q[1] + q[2] + q[3])
        ]
    elseif n == 4
        return [
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4])
            l1*sin(q[1]) + l2*sin(q[1] + q[2]) + l3*sin(q[1] + q[2] + q[3]) + l4*sin(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end


function J(n::Int64, q::Vector{T}) where T
    if n == 1
        return [
            -l1*sin(q[1]) 0 0 0
            l1*cos(q[1]) 0 0 0
        ]
    elseif n == 2
        return [
            -l1*sin(q[1]) - l2*sin(q[1] + q[2]) -l2*sin(q[1] + q[2]) 0 0
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) l2*cos(q[1] + q[2]) 0 0
        ]
    elseif n == 3
        return [
            -l1*sin(q[1]) - l2*sin(q[1] + q[2]) - l3*sin(q[1] + q[2] + q[3]) -l2*sin(q[1] + q[2]) - l3*sin(q[1] + q[2] + q[3]) -l3*sin(q[1] + q[2] + q[3]) 0
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) l3*cos(q[1] + q[2] + q[3]) 0
        ]
    elseif n == 4
        return [
            -l1*sin(q[1]) - l2*sin(q[1] + q[2]) - l3*sin(q[1] + q[2] + q[3]) - l4*sin(q[1] + q[2] + q[3] + q[4]) -l2*sin(q[1] + q[2]) - l3*sin(q[1] + q[2] + q[3]) - l4*sin(q[1] + q[2] + q[3] + q[4]) -l3*sin(q[1] + q[2] + q[3]) - l4*sin(q[1] + q[2] + q[3] + q[4]) -l4*sin(q[1] + q[2] + q[3] + q[4])
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4]) l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4]) l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4]) l4*cos(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end


function J_dot(n::Int64, q::Vector{T}, dq::Vector{T}) where T
    if n == 1
        return [
            -dq[1]*l1*cos(q[1]) 0 0 0
            -dq[1]*l1*sin(q[1]) 0 0 0
        ]
    elseif n == 2
        return [
            -dq[1]*l1*cos(q[1]) - l2*(dq[1] + dq[2])*cos(q[1] + q[2]) -l2*(dq[1] + dq[2])*cos(q[1] + q[2]) 0 0
            -dq[1]*l1*sin(q[1]) - l2*(dq[1] + dq[2])*sin(q[1] + q[2]) -l2*(dq[1] + dq[2])*sin(q[1] + q[2]) 0 0
        ]
    elseif n == 3
        return [
            -dq[1]*l1*cos(q[1]) - l2*(dq[1] + dq[2])*cos(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) -l2*(dq[1] + dq[2])*cos(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) -l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) 0
            -dq[1]*l1*sin(q[1]) - l2*(dq[1] + dq[2])*sin(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) -l2*(dq[1] + dq[2])*sin(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) -l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) 0
        ]
    elseif n == 4
        return [
            -dq[1]*l1*cos(q[1]) - l2*(dq[1] + dq[2])*cos(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4]) -l2*(dq[1] + dq[2])*cos(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4]) -l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4]) -l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4])
            -dq[1]*l1*sin(q[1]) - l2*(dq[1] + dq[2])*sin(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -l2*(dq[1] + dq[2])*sin(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end



end