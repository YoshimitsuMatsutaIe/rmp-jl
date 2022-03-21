"""RMP制御器"""


# """
# RMPいろいろ
# """
# module RMP



using LinearAlgebra
using ForwardDiff  # 自動微分パッケージ

using Parameters

# export pullbacked_rmp
# export  natural

# export OriginalRMPAttractor
# export OriginalRMPCollisionAvoidance
# export OriginalJointLimitAvoidance
# export RMPfromGDSAttractor
# export RMPfromGDSCollisionAvoidance



"""pullback演算"""
function pullbacked_rmp(f, M, J, dJ, dx)
    pulled_f = J' * (f .- M * dJ * dx)
    pulled_M = J' * M * J
    return pulled_f, pulled_M
end

"""pullback演算"""
function pullbacked_rmp(f, M, J)
    pulled_f = J' * f
    pulled_M = J' * M * J
    return pulled_f, pulled_M
end


"""リーフノード"""
@with_kw mutable struct Leaf{T}
    parent::Int64
    ϕ::Vector{T}
    J::Matrix{T}  # parent -> childへのタスク写像のヤコビアン
    J_dot::Union{Matrix{T}, Nothing}
    x::Vector{T}
    x_dot::Vector{T}
    rmp_param
    f::Vector{T}
    M::Matrix{T}
end


"""ノード"""
@with_kw mutable struct Node{T}
    parent
    children::Vector{Leaf{T}}
    ϕ::Vector{T}
    J::Matrix{T}  # parent -> childへのタスク写像のヤコビアン
    J_dot::Union{Matrix{T}, Nothing}
    x::Vector{T}
    x_dot::Vector{T}
    f::Vector{T}
    M::Matrix{T}
end


"""ルートノード"""
@with_kw mutable struct Root{T}
    children
    q::Vector{T}
    q_dot::Vector{T}
    f::Vector{T}
    M::Matrix{T}
end


"""push  
root -> leaf
"""
function pushforward(leaf::Leaf{T}, q::Vector{T}, q_dot::Vector{T}) where T
    leaf.x = leaf.ϕ(q)
    leaf.x_dot = leaf.J(q) * q_dot
end


"""push  
node -> leaf
"""
function pushforward(node::Node{T}, q::Vector{T}, q_dot::Vector{T}) where T
    node.x = node.ϕ(q)
    node.x_dot = node.J(q) * q_dot
    node.f = zero(node.f)
    node.M = zero(node.M)
    for child in node.children
        pushforward(child, node.x, node.x_dot)
    end
end


"""push  
root -> node
"""
function pushforward(root::Root{T}, q_ddot::Vector{T}, Δt::T) where {T}
    root.q_dot = root.q_dot .+ Δt .* q_ddot
    root.q = root.q .+ .+ Δt .* q_dot
    root.f = zero(root.f)
    root.M = zero(root.M)
    for child in root.children
        pushforward(child, root.q, root.q_dot)
    end
end


function pullback(leaf::Leaf{T}) where {T}
    leaf.f, leaf.M = natural(leaf.rmp_param, leaf.x, leaf.x_dot, leaf.x)
    
end



### 以下RMP

"""ソフトマックス関数"""
function soft_max(s::T, α::T) where T
    s + 1/α * log(1 + exp(-2 * α * s))
end

"""fromGDSのアトラクターパラメータ"""
@with_kw struct RMPfromGDSAttractor{T}
    max_speed::T
    gain::T
    f_alpha::T
    sigma_alpha::T
    sigma_gamma::T
    wu::T
    wl::T
    alpha::T
    epsilon::T
end

"""目標吸引ポテンシャル2"""
potential_2 = soft_max

"""ポテンシャルの勾配"""
function ∇potential_2(x, η)
    x_norm = norm(x)
    return (1-exp(-2*η*x_norm)) / (1+exp(-2*η*x_norm)) .* x ./ x_norm
end

"""?"""
α_or_γ(x, σ) = exp(-(norm(x))^2 / (2*σ^2))

"""重み行列（fromGDSのアトラクターで使用）"""
w(x, sigma_gamma, wu, wl) = (wu-wl) * α_or_γ(x, sigma_gamma) + wl

"""fromGDSのアトラクター慣性行列"""
function inertia_matrix(x, p::RMPfromGDSAttractor{T}, x₀) where T
    dim = length(x)
    z = x .- x₀
    ∇pot = ∇potential_2(z, p.alpha)
    α = α_or_γ(z, p.sigma_alpha)
    return w(z, p.sigma_gamma, p.wu, p.wl) .* ((1-α) .* ∇pot * ∇pot' .+ (α + p.epsilon).*Matrix{T}(I, dim, dim))
end

"""力用"""
function xMx(x, p::RMPfromGDSAttractor{T}, dx, x₀) where T
    dx' * inertia_matrix(x, p, x₀) * dx
end

"""曲率項"""
function ξ(p::RMPfromGDSAttractor{T}, x, dx, x₀) where T
    # 第一項を計算
    dim = length(x)  # タスク空間の次元
    #A = Matrix{T}(undef, dim, dim)
    A = zeros(T, dim, dim)
    for i in 1:dim
        _M(x) = inertia_matrix(x, p, x₀)[:, i]
        _jacobian_M = ForwardDiff.jacobian(_M, x)
        #println(_jacobian_M)
        A[:, i] = _jacobian_M * dx
    end
    A *= dx

    # 第二項を計算
    _xMx(x) = xMx(x, p, dx, x₀)
    B = 1/2 .* ForwardDiff.gradient(_xMx, x)  # 便利
    
    return A .- B
end

"""fromGDSのアトラクター力"""
function f(p::RMPfromGDSAttractor{T}, x, dx, x₀, M) where T
    z = x .- x₀
    damp = p.gain / p.max_speed
    #return M * (-p.gain .* soft_normal(z, p.f_alpha) .- damp .* dx) .- ξ(p, x, dx, x₀)
    return M * (-p.gain .* ∇potential_2(z, p.alpha) .- damp .* dx) .- ξ(p, x, dx, x₀)
end

"""fromGDSのアトラクタの自然形式RMPを取得"""
function natural(p::RMPfromGDSAttractor{T}, x, dx, x₀) where T
    M = inertia_matrix(x, p, x₀)
    return f(p, x, dx, x₀, M), M
end




### 障害物回避 ###
"""fromGDSの障害物回避rmpのパラメータ"""
@with_kw struct RMPfromGDSCollisionAvoidance{T}
    rw::T
    sigma::T
    alpha::T
end

# 
# w(s) = s^(-4)



# """重み関数の微分"""
# dwds(s) = -4 * s^(-5)







"""距離に関する重み関数"""
function w2(s, rw=1.0)
    if rw - s > 0.0
        return (rw - s)^2 / s
    else
        return zero(s)
    end
    
end

"""重み関数のs微分"""
function dw2(s, rw)
    if rw - s > 0.0
        return (-2*(rw - s) * s + (rw - s)) / s^2
    else
        return zero(s)
    end
end

"""速度に関する重み関数"""
function u2(ds, σ)
    if ds < 0.0
        return 1 - exp(-ds^2 / (2*σ^2))
    else
        return zero(ds)
    end
end

"""重み関数のds微分"""
function du2(ds, σ)
    if ds < 0.0
        return -exp(-ds^2 / (2*σ^2)) * (-ds / σ^2)
    else
        return zero(ds)
    end
end

function δ(s, ds, σ)
    u2(ds, σ) + 1/2 * ds * du2(ds, σ)
end

"""曲率項"""
function ξ(s, ds, σ, rw)
    1/2 * u2(ds, σ) * dw2(s, rw) * ds^2
end

"""障害物回避ポテンシャル"""
function Φ1(s, α, rw)
    1/2 * α * w2(s, rw)^2
end

"""障害物回避ポテンシャルの勾配"""
function ∇Φ1(s, α, rw)
    α * w2(s, rw) * dw2(s, rw)
end


"""fromGDSの障害物回避力"""
function f(p::RMPfromGDSCollisionAvoidance{T}, s, ds) where T
    return -∇Φ1(s, p.alpha, p.rw) - ξ(s, ds, p.sigma, p.rw)
end

"""fromGDSの障害物回避慣性行列"""
function inertia_matrix(p::RMPfromGDSCollisionAvoidance{T}, s, ds) where T
    return w2(s, p.rw) * δ(s, ds, p.sigma)
end

"""fromGDSの障害物回避rmpの自然形式を取得"""
function natural(p::RMPfromGDSCollisionAvoidance{T}, x, dx, x₀, dx₀=zero(x₀)) where T
    
    s_vec = x .- x₀
    ds_vec = dx .- dx₀
    s = norm(s_vec)
    ds = (1/s .* dot(s_vec, ds_vec))[1]


    m = inertia_matrix(p, s, ds)
    _f = f(p, s, ds)

    J = -s_vec' ./ s
    J̇ = -s^(-2) .* (ds_vec' .- s_vec' .* ds)

    _f, M = pullbacked_rmp(_f, m, J, J̇, dx)

    return _f, M
end



"""RMPfromGDSのジョイント制限回避のパラメータ"""
@with_kw struct RMPfromGDSJointLimitAvoidance{T}
    gamma_p::T
    gamma_d::T
    lambda::T
    sigma::T
end

function α_upper(dq::T, sigma::T) where T
    1.0 - exp(-max(dq, 0)^2 / (2 * sigma^2))
end

function α_lower(dq::T, sigma::T) where T
    1.0 - exp(-min(dq, 0)^2 / (2 * sigma^2))
end

function _s(q::T, q_u::T, q_l::T) where T
    (q - q_l) / (q_u - q_l)
end

function _dsdq(q::T, q_u::T, q_l::T) where T
    1 / (q_u - q_l)
end

function _d(s::T) where T
    4 * s * (1-s)
end

function _dddq(s::T, dsdq::T) where T
    (4.0 - 8 * s) * dsdq
end

function _b(q::T, dq::T, q_u::T, q_l::T, sigma::T) where T
    s = _s(q, q_u, q_l)
    d = _d(s)
    α_u = α_upper(dq, sigma)
    α_l = α_lower(dq, sigma)
    return s*(α_u * d + (1-α_u)) + (1-s)*(α_l * d + (1-α_l))
end

function _dbdq(q::T, dq::T, q_u::T, q_l::T, sigma::T) where T
    s = _s(q, q_u, q_l)
    d = _d(s)
    α_u = α_upper(dq, sigma)
    α_l = α_lower(dq, sigma)
    dsdq = _dsdq(q, q_u, q_l)
    dddq = _dddq(s, dsdq)
    return (dsdq*(α_u * d + (1-α_u)) + s * dddq) + -dsdq*(α_l * d + (1-α_l)) + (1-s) * dddq
end

"""慣性行列の対角要素"""
function _a(q::T, dq::T, q_u::T, q_l::T, sigma::T) where T
    b = _b(q, dq, q_u, q_l, sigma)
    return b^(-2)
end

function _dadq(q::T, dq::T, q_u::T, q_l::T, sigma::T) where T
    -2 * (_b(q, dq, q_u, q_l, sigma))^(-3) * _dbdq(q, dq, q_u, q_l, sigma)
end


"""fromGDSのジョイント制限回避慣性行列"""
function inertia_matrix(
    p::RMPfromGDSJointLimitAvoidance{T}, q::Vector{T}, dq::Vector{T},
    q_max::Vector{T}, q_min::Vector{T}
    ) where T
    dim = length(q)
    A = zeros(T, dim, dim)

    for i in 1:dim
        A[i, i] = _a(q[i], dq[i], q_max[i], q_min[i], p.sigma)
    end
    #println(A)
    return p.lambda * A
end

"""ジョイント制限回避の曲率項"""
function ξ(q::Vector{T}, dq::Vector{T}, q_max::Vector{T}, q_min::Vector{T}, sigma::T) where T
    
    #z = Vector{T}(undef, 7)
    dim = length(q)
    z = zero(q)

    for i in 1:dim
        z[i] = 1/2 * _dadq(q[i], dq[i], q_max[i], q_min[i], sigma) * dq[i]^2
    end
    return z
end


"""fromGDSのジョイント制限回避慣性力"""
function f(
    p::RMPfromGDSJointLimitAvoidance{T}, q::Vector{T}, dq::Vector{T},
    q_neutral::Vector{T}, A::Matrix{T},
    ) where T
    ξ_A = ξ(q, dq, q_max, q_min, p.sigma)
    #ξ_A = zeros(T, 7)
    return A * (p.gamma_p * (q_neutral .- q) - p.gamma_d * dq) - ξ_A
end


"""fromGDSのジョイント制限回避RMP"""
function natural(
    p::RMPfromGDSJointLimitAvoidance{T}, q::Vector{T}, dq::Vector{T},
    q_max::Vector{T}, q_min::Vector{T}, q_neutral::Vector{T}
    ) where T
    A = inertia_matrix(p, q, dq, q_max, q_min)
    _f = f(p, q, dq, q_neutral, A)
    return _f, A
end





#end