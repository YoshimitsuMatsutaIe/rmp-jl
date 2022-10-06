
"""
sinicose用のモジュール
"""
module SiceKinematics
using Parameters
# export Mapping
# export q_min q_max q_neutral
# export  cos_dim t_dim

const c_dim = 4
const t_dim = 2

const l1 = 1.0
const l2 = 1.0
const l3 = 1.0
const l4 = 1.0

const q_min = [
    (-3/4)π+(1/2)π
    (-3/4)π
    (-3/4)π
    (-3/4)π
]
const q_max = [
    (3/4)π+(1/2)π
    (3/4)π
    (3/4)π
    (3/4)π
]
const q_neutral = [
    (1/2)π
    0.0
    0.0
    0.0
]

const ee_id = (5, 1)


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
    elseif n == 5
        return [
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4])
            l1*sin(q[1]) + l2*sin(q[1] + q[2]) + l3*sin(q[1] + q[2] + q[3]) + l4*sin(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end



function rx(n::Int64, q::Vector{T}) where T
    if n == 1
        return [
            cos(q[1])
            sin(q[1])
        ]
    elseif n == 2
        return [
            -sin(q[1])*sin(q[2]) + cos(q[1])*cos(q[2])
            sin(q[1])*cos(q[2]) + sin(q[2])*cos(q[1])
        ]
    elseif n == 3
        return [
            -sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[1])*sin(q[3])*cos(q[2]) - sin(q[2])*sin(q[3])*cos(q[1]) + cos(q[1])*cos(q[2])*cos(q[3])
            -sin(q[1])*sin(q[2])*sin(q[3]) + sin(q[1])*cos(q[2])*cos(q[3]) + sin(q[2])*cos(q[1])*cos(q[3]) + sin(q[3])*cos(q[1])*cos(q[2])
        ]
    elseif n == 4
        return [
            sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) - sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) - sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) - sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) - sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
            -sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) - sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) + sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) - sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) + sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) + sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) + sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
        ]
    elseif n == 5
        return [
            sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) - sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) - sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) - sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) - sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
            -sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) - sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) + sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) - sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) + sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) + sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) + sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
        ]
    else
        AssertionError
    end
end


function ry(n::Int64, q::Vector{T}) where T
    if  n == 1
        return [
            -sin(q[1])
            cos(q[1])
        ]
    elseif  n == 2
        return [
            -sin(q[1])*cos(q[2]) - sin(q[2])*cos(q[1])
            -sin(q[1])*sin(q[2]) + cos(q[1])*cos(q[2]) 
        ]
    elseif  n == 3
        return [
            sin(q[1])*sin(q[2])*sin(q[3]) - sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]) - sin(q[3])*cos(q[1])*cos(q[2])
            -sin(q[1])*sin(q[2])*cos(q[3]) - sin(q[1])*sin(q[3])*cos(q[2]) - sin(q[2])*sin(q[3])*cos(q[1]) + cos(q[1])*cos(q[2])*cos(q[3])
        ]
    elseif  n == 4
        return [
            sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
            sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) - sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) - sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) - sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) - sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
        ]
    elseif  n == 5
        return [
            sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
            sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) - sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) - sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) - sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) - sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) - sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
        ]
    else
        AssertionError
    end
end


function jo(n::Int64, q::Vector{T}) where T
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
    elseif n == 5
        return [
            -l1*sin(q[1]) - l2*sin(q[1] + q[2]) - l3*sin(q[1] + q[2] + q[3]) - l4*sin(q[1] + q[2] + q[3] + q[4]) -l2*sin(q[1] + q[2]) - l3*sin(q[1] + q[2] + q[3]) - l4*sin(q[1] + q[2] + q[3] + q[4]) -l3*sin(q[1] + q[2] + q[3]) - l4*sin(q[1] + q[2] + q[3] + q[4]) -l4*sin(q[1] + q[2] + q[3] + q[4])
            l1*cos(q[1]) + l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4]) l2*cos(q[1] + q[2]) + l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4]) l3*cos(q[1] + q[2] + q[3]) + l4*cos(q[1] + q[2] + q[3] + q[4]) l4*cos(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end


function jo_dot(n::Int64, q::Vector{T}, dq::Vector{T}) where T
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
    elseif n == 5
        return [
            -dq[1]*l1*cos(q[1]) - l2*(dq[1] + dq[2])*cos(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4]) -l2*(dq[1] + dq[2])*cos(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4]) -l3*(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4]) -l4*(dq[1] + dq[2] + dq[3] + dq[4])*cos(q[1] + q[2] + q[3] + q[4])
            -dq[1]*l1*sin(q[1]) - l2*(dq[1] + dq[2])*sin(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -l2*(dq[1] + dq[2])*sin(q[1] + q[2]) - l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -l3*(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) - l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -l4*(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end


function jrx(n::Int64, q::Vector{T}) where T
    if n == 1
        return [
            - sin(q[1]) 0. 0. 0.
            cos(q[1]) 0. 0. 0.
        ]
    elseif n == 2
        return [
            - sin(q[1])*cos(q[2]) -  sin(q[2])*cos(q[1]) -sin(q[1])*cos(q[2]) -  sin(q[2])*cos(q[1]) 0. 0.
            - sin(q[1])* sin(q[2]) + cos(q[1])*cos(q[2]) -sin(q[1])* sin(q[2]) + cos(q[1])*cos(q[2]) 0. 0.
        ]
    elseif n == 3
        return [
            sin(q[1])* sin(q[2])* sin(q[3]) -  sin(q[1])*cos(q[2])*cos(q[3]) -  sin(q[2])*cos(q[1])*cos(q[3]) -  sin(q[3])*cos(q[1])*cos(q[2])  sin(q[1])* sin(q[2])* sin(q[3]) -  sin(q[1])*cos(q[2])*cos(q[3]) -  sin(q[2])*cos(q[1])*cos(q[3]) -  sin(q[3])*cos(q[1])*cos(q[2])  sin(q[1])* sin(q[2])* sin(q[3]) -  sin(q[1])*cos(q[2])*cos(q[3]) -  sin(q[2])*cos(q[1])*cos(q[3]) -  sin(q[3])*cos(q[1])*cos(q[2]) 0.
            - sin(q[1])* sin(q[2])*cos(q[3]) -  sin(q[1])* sin(q[3])*cos(q[2]) -  sin(q[2])* sin(q[3])*cos(q[1]) + cos(q[1])*cos(q[2])*cos(q[3]) - sin(q[1])* sin(q[2])*cos(q[3]) -  sin(q[1])* sin(q[3])*cos(q[2]) -  sin(q[2])* sin(q[3])*cos(q[1]) + cos(q[1])*cos(q[2])*cos(q[3]) - sin(q[1])* sin(q[2])*cos(q[3]) -  sin(q[1])* sin(q[3])*cos(q[2]) -  sin(q[2])* sin(q[3])*cos(q[1]) + cos(q[1])*cos(q[2])*cos(q[3]) 0.
        ]
    elseif n == 4
        return [
            sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])  sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])  sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])  sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
            sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])  sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])  sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])  sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
        ]
    elseif n == 5
        return  [
            sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])  sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])  sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])  sin(q[1])* sin(q[2])* sin(q[3])*cos(q[4]) +  sin(q[1])* sin(q[2])* sin(q[4])*cos(q[3]) +  sin(q[1])* sin(q[3])* sin(q[4])*cos(q[2]) -  sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) +  sin(q[2])* sin(q[3])* sin(q[4])*cos(q[1]) -  sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) -  sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) -  sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
            sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])  sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])  sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])  sin(q[1])* sin(q[2])* sin(q[3])* sin(q[4]) -  sin(q[1])* sin(q[2])*cos(q[3])*cos(q[4]) -  sin(q[1])* sin(q[3])*cos(q[2])*cos(q[4]) -  sin(q[1])* sin(q[4])*cos(q[2])*cos(q[3]) -  sin(q[2])* sin(q[3])*cos(q[1])*cos(q[4]) -  sin(q[2])* sin(q[4])*cos(q[1])*cos(q[3]) -  sin(q[3])* sin(q[4])*cos(q[1])*cos(q[2]) + cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
        ]
    else
        AssertionError
    end
end



function jrx_dot(n::Int64, q::Vector{T}, dq::Vector{T}) where T
    if  n == 1
        return [
            -cos(q[1])*dq[1] 0. 0. 0.
            -sin(q[1])*dq[1] 0. 0. 0.
        ]
    elseif  n == 2
        return [
            -(dq[1] + dq[2])*cos(q[1] + q[2]) -(dq[1] + dq[2])*cos(q[1] + q[2]) 0. 0.
            -(dq[1] + dq[2])*sin(q[1] + q[2]) -(dq[1] + dq[2])*sin(q[1] + q[2]) 0. 0.
        ]
    elseif  n == 3
        return [
            -(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) -(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) -(dq[1] + dq[2] + dq[3])*cos(q[1] + q[2] + q[3]) 0
            -(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) -(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) -(dq[1] + dq[2] + dq[3])*sin(q[1] + q[2] + q[3]) 0
        ]
    elseif  n == 4
        return [
            sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] 
            -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4])
        ]
    elseif  n == 5
        return [
            sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4] sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[1] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[2] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[3] + sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[4] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[1] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[2] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[3] + sin(q[2] + q[3])*sin(q[1])*cos(q[4])*dq[4] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[1] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[2] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[3] + sin(q[2] + q[4])*sin(q[3])*cos(q[1])*dq[4] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[1] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[2] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[3] - sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[4] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[1] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[2] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[3] - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[4]
            -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4]) -(dq[1] + dq[2] + dq[3] + dq[4])*sin(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end


function jry(n::Int64, q::Vector{T}) where T
    if  n == 1
        return [
            -cos(q[1]) 0. 0. 0.
            -sin(q[1]) 0. 0. 0.
        ]
    elseif  n == 2
        return [
            sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2]) sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2]) 0. 0.
            -sin(q[1])*cos(q[2]) - sin(q[2])*cos(q[1]) -sin(q[1])*cos(q[2]) - sin(q[2])*cos(q[1]) 0. 0.
        ]
    elseif  n == 3
        return [
            sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[2]) + sin(q[2])*sin(q[3])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[2]) + sin(q[2])*sin(q[3])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[2]) + sin(q[2])*sin(q[3])*cos(q[1]) - cos(q[1])*cos(q[2])*cos(q[3]) 0.
            sin(q[1])*sin(q[2])*sin(q[3]) - sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]) - sin(q[3])*cos(q[1])*cos(q[2]) sin(q[1])*sin(q[2])*sin(q[3]) - sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]) - sin(q[3])*cos(q[1])*cos(q[2]) sin(q[1])*sin(q[2])*sin(q[3]) - sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[2])*cos(q[1])*cos(q[3]) - sin(q[3])*cos(q[1])*cos(q[2]) 0.
        ]
    elseif  n == 4
        return [
            -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
            sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
        ]
    elseif  n == 5
        return [
            -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) -sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[3])*cos(q[4]) + sin(q[1])*sin(q[3])*cos(q[2])*cos(q[4]) + sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3]) + sin(q[2])*sin(q[3])*cos(q[1])*cos(q[4]) + sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3]) + sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2]) - cos(q[1])*cos(q[2])*cos(q[3])*cos(q[4])
            sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3]) sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4]) + sin(q[1])*sin(q[2])*sin(q[4])*cos(q[3]) + sin(q[1])*sin(q[3])*sin(q[4])*cos(q[2]) - sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4]) + sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1]) - sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4]) - sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4]) - sin(q[4])*cos(q[1])*cos(q[2])*cos(q[3])
        ]
    else
        AssertionError
    end
end


function jry_dot(n::Int64, q::Vector{T}) where T
    if  n == 1
        return [
            -cos(q[1]) 0. 0. 0.
            -sin(q[1]) 0. 0. 0.
        ]
    elseif  n == 2
        return [
            -cos(q[1] + q[2]) -cos(q[1] + q[2]) 0. 0.
            -sin(q[1] + q[2]) -sin(q[1] + q[2]) 0. 0.
        ]
    elseif  n == 3
        return [
            -cos(q[1] + q[2] + q[3]) -cos(q[1] + q[2] + q[3]) -cos(q[1] + q[2] + q[3]) 0.
            -sin(q[1] + q[2] + q[3]) -sin(q[1] + q[2] + q[3]) -sin(q[1] + q[2] + q[3]) 0.
        ]
    elseif  n == 4
        return [
            -cos(q[1] + q[2] + q[3] + q[4]) -cos(q[1] + q[2] + q[3] + q[4]) -cos(q[1] + q[2] + q[3] + q[4]) -cos(q[1] + q[2] + q[3] + q[4])
            -sin(q[1] + q[2] + q[3] + q[4]) -sin(q[1] + q[2] + q[3] + q[4]) -sin(q[1] + q[2] + q[3] + q[4]) -sin(q[1] + q[2] + q[3] + q[4])
        ]
    elseif  n == 5
        return [
            -cos(q[1] + q[2] + q[3] + q[4]) -cos(q[1] + q[2] + q[3] + q[4]) -cos(q[1] + q[2] + q[3] + q[4]) -cos(q[1] + q[2] + q[3] + q[4])
            -sin(q[1] + q[2] + q[3] + q[4]) -sin(q[1] + q[2] + q[3] + q[4]) -sin(q[1] + q[2] + q[3] + q[4]) -sin(q[1] + q[2] + q[3] + q[4])
        ]
    else
        AssertionError
    end
end

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
    println(typeof(rss))
    Mapping(
        frame, index, rss[frame][index]
    )
end

function (p::Mapping)(
    q::Vector{T}, q_dot::Vector{T},
    out_x::Vector{T}, out_x_dot::Vector{T}, out_J::Matrix{T}, out_J_dot::Matrix{T}
) where T
    x, y = p.r

    @show rx(p.frame, q) |> size
    @show ry(p.frame, q) |> size
    @show phi(p.frame, q) |> size
    @show jrx(p.frame, q) |> size
    @show jry(p.frame, q) |> size
    @show jo(p.frame, q) |> size
    @show jrx_dot(p.frame, q, q_dot) |> size
    @show jry_dot(p.frame, q) |> size
    @show jo_dot(p.frame, q, q_dot) |> size

    out_x .= rx(p.frame, q) .* x.+ ry(p.frame, q) .* y .+ phi(p.frame, q)
    
    out_J .= jrx(p.frame, q) .* x .+ jry(p.frame, q) .* y .+ jo(p.frame, q)
    out_x_dot .= out_J * q_dot
    out_J_dot .= jrx_dot(p.frame, q, q_dot) .* x .+ jry_dot(p.frame, q) .* y .+ jo_dot(p.frame, q, q_dot)
    
end

end