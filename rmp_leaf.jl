using LinearAlgebra
using Parameters

include("attractor_xi_2d.jl")



"""param of goal attractor"""
@with_kw mutable struct GoalAttractor{T}
    g::Vector{T}
    max_speed::T
    gain::T
    sigma_alpha::T
    sigma_gamma::T
    wu::T
    wl::T
    alpha::T
    epsilon::T
end


function (p::GoalAttractor)(x::Vector{T}, x_dot::Vector{T}, out_M::Matrix{T}, out_f::Vector{T}) where {T}

    dim = length(x)
    z = x - p.g
    z_norm = norm(z)
    grad_phi = (1-exp(-2*p.alpha*z_norm)) / (1+exp(-2*p.alpha*z_norm)) * z / z_norm
    alpha_x = exp(-z_norm^2 / (2 * p.sigma_alpha^2))
    gamma_x = exp(-z_norm^2 / (2 * p.sigma_gamma^2))
    wx = gamma_x*p.wu + (1 - gamma_x)*p.wl
    

    if dim == 2
        xi = xi_2d(z, x_dot, p.sigma_alpha, p.sigma_gamma, p.wu, p.wl, p.alpha, p.epsilon)
    elseif dim == 3
        AssertionError
    else
        AssertionError
    end

    damp = p.max_speed / p.gain

    out_M .= wx .* ((1-alpha_x) .* grad_phi * grad_phi' .+ (alpha_x+p.epsilon) .* Matrix{T}(I, dim, dim))
    out_f .= out_M * (-p.gain .* grad_phi .- damp .* x_dot) .- xi
end


"""param of obstacle avoidance"""
@with_kw mutable struct ObstacleAvoidnce{T}
    o::Vector{T}
    scale_rep::T
    scale_damp::T
    gain::T
    sigma::T
    rw::T
end

function (p::ObstacleAvoidnce{T})(x::Vector{T}, x_dot::Vector{T}, out_M::Matrix{T}, out_f::Vector{T}) where {T}
    s = norm(x - p.o)

    if p.rw - s > 0
        w2 = (p.rw - s)^2 / s
        w2_dot = (-2*(p.rw-s)*s + (p.rw-s)) / s^2
    else
        out_M .= zero(out_M)
        out_f .= zero(out_f)
        return
    end

    J = -(x - p.o)' ./ s
    s_dot = (J * (x_dot))[1]
    J_dot = -(x_dot' - (x-p.o)' * (1/s * ((x-p.o)' * x_dot)[1])) ./ s^2
    if s_dot < 0
        u2 = 1 - exp(-s_dot^2 / (2*p.sigma^2))
        u2_dot = -exp(s_dot^2 / (2*p.sigma^2)) * (-s_dot/p.sigma^3)
    else
        u2 = 0
        u2_dot = 0
    end

    delta = u2 + 1/2 * s_dot * u2_dot
    xi = 1/2 * u2 * w2_dot * s_dot^2
    grad_phi = p.gain * w2 * w2_dot
    
    m = w2 * delta
    f = -grad_phi - xi

    out_M .= m .* J' * J
    out_f .= (f - m * (J_dot * x_dot)[1]) .* J'

end



"""RMPfromGDSのジョイント制限回避のパラメータ"""
@with_kw struct JointLimitAvoidance{T}
    q_neutral::Vector{T}
    q_max::Vector{T}
    q_min::Vector{T}
    gamma_p::T
    gamma_d::T
    lambda::T
    sigma::T
end


function (p::JointLimitAvoidance)(q::Vector{T}, q_dot::Vector{T}, out_M::Matrix{T}, out_f::Vector{T}) where {T}
    dim = length(q)
    xi = zeros(T, dim)

    for i in 1:dim
        alpha_upper = 1 - exp(-max(q_dot[i], 0)^2 / (2*p.sigma^2))
        alpha_lower = 1 - exp(-min(q_dot[i], 0)^2 / (2*p.sigma^2))
        s = (q[i] - p.q_min[i]) / (p.q_max[i] - p.q_min[i])
        s_dot = 1 / (p.q_max[i] - p.q_min[i])
        d = 4*s*(1-s)
        d_dot = (4 - 8*s) * s_dot
        b =  s*(alpha_upper*d + (1-alpha_upper)) + (1-s)*(alpha_lower*d + (1-alpha_lower))
        b_dot = (s_dot*(alpha_upper*d + (1-alpha_upper)) + s*d_dot) \
            + -s_dot*(alpha_lower * d + (1-alpha_lower)) + (1-s) * d_dot
        a = b^(-2)
        a_dot = -2*b^(-3) * b_dot
        

        xi[i] = 1/2 * a_dot * q_dot[i]^2
        out_M[i,i] = p.lambda * a
    end

    out_f .= out_M * (p.gamma_p*(p.q_neutral - q) - p.gamma_d*q_dot) - xi
end