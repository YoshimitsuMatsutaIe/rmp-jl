"""RMP制御器"""


"""
RMPいろいろ
"""
module RMP



using LinearAlgebra
using ForwardDiff  # 自動微分パッケージ

using Parameters

export pullbacked_rmp
export  get_natural

export OriginalRMPAttractor
export OriginalRMPCollisionAvoidance
export OriginalJointLimitAvoidance
export RMPfromGDSAttractor
export RMPfromGDSCollisionAvoidance



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




#### オリジナルのRMP ###
"""ソフトマックス関数"""
function soft_max(s::T, α::T) where T
    s + 1/α * log(1 + exp(-2 * α * s))
end

"""ソフト正規化関数"""
function soft_normal(v, alpha)
    return v ./ soft_max(norm(v), alpha)
end

"""空間を一方向に伸ばす計量"""
function metric_stretch(v, alpha)
    xi = soft_normal(v, alpha)
    return xi * xi'
end

"""基本の計量"""
function basic_metric_H(f::Vector{T}, alpha::T, beta::T) where T
    dim = length(f)
    return beta .* metric_stretch(f, alpha) + (1 - beta) .* Matrix{T}(I, dim, dim)
end



"""OriginalRMPの目標到達制御器のパラメータ"""
@with_kw struct OriginalRMPAttractor{T}
    max_speed::T
    gain::T
    ddq_damp_r::T
    sigma_W::T
    sigma_H::T
    metric_damp_r::T
end



"""目標加速度 from OirginalRMP"""
function ddz(p::OriginalRMPAttractor{T}, z::Vector{T}, dz::Vector{T}, z0::Vector{T}) where T
    damp = p.gain / p.max_speed
    a = p.gain .* soft_normal(z0.-z, p.metric_damp_r) .- damp*dz
    return a
end

"""目標計量  from OirginalRMP"""
function inertia_matrix(
    p::OriginalRMPAttractor{T}, z::Vector{T}, dz::Vector{T}, z0::Vector{T}, ddq::Vector{T}
) where T
    dis = norm(z0 .- z)
    weight = exp(-dis ./ p.sigma_W)
    beta = 1.0 - exp(-1/2 * (dis / p.sigma_H)^2)
    return weight .* basic_metric_H(ddq, p.ddq_damp_r, beta)
end

"""canonical form []"""
function get_canonical(p::OriginalRMPAttractor{T}, z, dz, z0) where T
    a = ddz(p, z, dz, z0)
    M = inertia_matrix(p, z, dz, z0, a)
    return a, M
end


"""natural form ()"""
function get_natural(p::OriginalRMPAttractor{T}, z, dz, z0) where T
    a, M = get_canonical(p, z, dz, z0)
    f = M * a
    return f, M
end


@with_kw struct OriginalRMPCollisionAvoidance{T}
    scale_rep::T
    scale_damp::T
    ratio::T
    gain::T
    r::T
end

"""障害物回避加速度  from OirginalRMP"""
function ddz(p::OriginalRMPCollisionAvoidance{T}, z, dz, z0) where T
    
    x = z .- z0
    d = norm(x)
    ∇d = x ./ d  # 勾配

    # 斥力項
    α_rep = p.gain .* exp(-d / p.scale_rep)
    ddq_rep = α_rep .* ∇d

    # ダンピング項
    P_obs = max(0.0, dot(-dz, ∇d)) * ∇d * ∇d' * dz
    damp_gain = p.gain * p.ratio
    α_damp = damp_gain / (d / p.scale_damp + 1e-7)
    ddq_damp = α_damp .* P_obs

    return ddq_rep .+ ddq_damp
end

"""障害物計量 from OriginalRMP"""
function inertia_matrix(p::OriginalRMPCollisionAvoidance{T}, z, dz, z0, ddq) where T
    dim = length(z)
    d = norm(z .- z0)
    weight = (d / p.r)^2 - 2 * d / p.r + 1
    return weight .* Matrix{T}(I, dim, dim)
end

"""canonical form ()"""
function get_canonical(p::OriginalRMPCollisionAvoidance{T}, z, dz, z0) where T
    a = ddz(p, z, dz, z0)
    M = inertia_matrix(p, z, dz, z0, a)
    return a, M
end

"""natural form []"""
function get_natural(p::OriginalRMPCollisionAvoidance{T}, z, dz, z0) where T
    a, M = get_canonical(p, z, dz, z0)
    f = M * a
    return f, M
end


@with_kw struct OriginalJointLimitAvoidance{T}
    gamma_p::T
    gamma_d::T
    lambda::T
end

# """アフィン変換されたシグモイド写像"""
# sigma_L(q, q_min, q_max) = (q_max - q_min) * (1 / (1 + exp.(-q))) + q_min

"""ジョイント制限に関する対角ヤコビ行列"""
function D_sigma(q::Vector{T}, q_min::Vector{T}, q_max::Vector{T}) where T
    diags = (q_max .- q_min) .* (exp.(-q) ./ (1.0 .+ exp.(-q)).^2)
    return diagm(diags)
end

"""ジョイント制限回避加速度 from OriginalRMP"""
function ddz(
    p::OriginalJointLimitAvoidance{T},
    q::Vector{T}, dq::Vector{T}, q_max::Vector{T}, q_min::Vector{T}
) where T
    z = p.gamma_p .* (-q) .- p.gamma_d .* dq
    a = inv(D_sigma(q, q_min, q_max)) * z
    return a
end

"""ジョイント制限回避計量 from OriginalRMP"""
function inertia_matrix(p::OriginalJointLimitAvoidance{T}, q, dq) where T
    n = length(q)
    return p.lambda .* Matrix{T}(I, n, n)
end

"""canonical form ()"""
function get_canonical(p::OriginalJointLimitAvoidance{T}, q, dq, q_max, q_min) where T
    return ddz(p, q, dq, q_max, q_min), inertia_matrix(p, q, dq)
end


"""natural form []"""
function get_natural(p::OriginalJointLimitAvoidance{T}, q, dq, q_max, q_min, q_neutral) where T
    a, M = get_canonical(p, q, dq, q_max, q_min)
    f = M * a
    return f, M
end




### fromGDS ###


"""fromGDSのアトラクター　パラメータ"""
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
function get_natural(p::RMPfromGDSAttractor{T}, x, dx, x₀) where T
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
        return 0.0
    end
    
end

"""重み関数のs微分"""
function dw2(s, rw)
    if rw - s > 0.0
        return (-2*(rw - s) * s + (rw - s)) / s^2
    else
        return 0.0
    end
end

"""速度に関する重み関数"""
function u2(ds, σ)
    if ds < 0.0
        return 1 - exp(-ds^2 / (2*σ^2))
    else
        return 0.0
    end
end

"""重み関数のds微分"""
function du2(ds, σ)
    if ds < 0.0
        return -exp(-ds^2 / (2*σ^2)) * (-ds / σ^2)
    else
        return 0.0
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
function get_natural(p::RMPfromGDSCollisionAvoidance{T}, x, dx, x₀, dx₀=zero(x₀)) where T
    
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
function get_natural(
    p::RMPfromGDSJointLimitAvoidance{T}, q::Vector{T}, dq::Vector{T},
    q_max::Vector{T}, q_min::Vector{T}, q_neutral::Vector{T}
    ) where T
    A = inertia_matrix(p, q, dq, q_max, q_min)
    _f = f(p, q, dq, q_neutral, A)
    return _f, A
end




#### sice2021 ####
"""インピーダンス制御RMP（自作）
M_d : 所望のインピーダンス慣性行列  
D_d : 所望のインピーダンス減衰行列  
P_d : 所望のインピーダンス剛性行列  
D_e : 環境減衰行列  
P_e : 環境剛性行列  
eta_d : 一般化位置に関するスケール係数  
eta_e : 外力の平衡位置に関するスケール係数  
"""
@with_kw struct RMPfromGDSImpedance{T}
    M_d::Matrix{T}
    D_d::Matrix{T}
    P_d::Matrix{T}
    D_e::Matrix{T}
    P_e::Matrix{T}
    a::T  # シグモイドのパラメータ
    eta_d::T
    eta_e::T
    f_alpha::T
    sigma_alpha::T
    sigma_gamma::T
    wu::T
    wl::T
    alpha::T
    epsilon::T
end

"""シグモイド関数"""
sigmoid(x, a) = 1/(1+exp(-a*x))


function s_alpha(x, η)
    x_norm = norm(x)
    (1-exp(-2*η*x_norm)) / (1+exp(-2*η*x_norm))
end


hat(x) = x / norm(x)


# # """インピーダンスポテンシャル

# # ・計算には使いません
# # """
# # function potential_I(e_d, eta_d, e_e, eta_e, a)
# #     alpha = sigmoid(norm(e_d), a)
# #     return (1-alpha)*soft_normal(e_d, eta_d) + alpha*soft_normal(e_e, eta_e)
# # end



"""インピーダンスポテンシャルの勾配"""
function ∇potential_I(p::RMPfromGDSImpedance{T}, y, yd, ye, alpha) where T
    
    # 古い方

    e_d = y - yd
    e_e = y - ye

    inv_M_d = inv(p.M_d)
    P_tilde_e = inv_M_d * p.P_e
    P_tilde_d = inv_M_d * p.P_d


    t1 = -(1-alpha)*alpha .* hat(e_d) .* soft_max(norm(e_d), p.eta_d)
    t2 = (1-alpha) * s_alpha(e_d, p.eta_d) .* hat(e_d)
    t3 = (1-alpha)*alpha .* hat(e_e) .* soft_max(norm(e_e), p.eta_e)
    t4 = alpha * s_alpha(e_e, p.eta_e) .* hat(e_e)

    #t1 = t3 = zero(t2)

    t1 .+ 
    P_tilde_d * t2 .+ 
    t3 .+ 
    P_tilde_e * t4


    # # 新しい方
    # inv_M_d = inv(p.M_d)
    # P_tilde_e = inv_M_d * p.P_e
    # P_tilde_d = inv_M_d * p.P_d

    # e_d = y - yd
    # e_e = y - ye

    # x2 = norm(yd - ye)

    # y = y - ye

    # a = 0.5
    # b = x2^2 / 4

    # function df(x)
    #     return a*((x - x2/2)^2-b)^2
    # end

    # println(df(norm(y)))
    # return P_tilde_e .* df(norm(y)) * hat(y)

end






"""インピーダンスRMPの慣性行列"""
function inertia_matrix(p::RMPfromGDSImpedance{T}, y, yd, ye, dy, alpha) where T
    e_d = y - yd
    e_e = y - ye
    dim = length(y)
    ∇pot = ∇potential_I(p, y, yd, ye, alpha)
    α = α_or_γ(e_d, p.sigma_alpha)
    return w(e_d, p.sigma_gamma, p.wu, p.wl) .* ((1-α) .* ∇pot * ∇pot' .+ (α + p.epsilon).*Matrix{T}(I, dim, dim))
end

"""力用"""
function xMx(p::RMPfromGDSImpedance{T}, y, yd, ye, dy, alpha) where T
    dy' * inertia_matrix(p, y, yd, ye, dy, alpha) * dy
end

"""曲率項"""
function ξ(p::RMPfromGDSImpedance{T}, y, yd, ye, dy, alpha) where T
    # 第一項を計算
    #A = Matrix{T}(undef, 3, 3)
    e_d = y - yd
    e_e = y - ye

    dim = length(y)
    A = zeros(T, dim, dim)
    for i in 1:length(e_d)
        println("kyokuritu i = ", i)
        _M(y) = inertia_matrix(p, y, yd, ye, dy, alpha)[:, i]
        _jacobian_M = ForwardDiff.jacobian(_M, y)
        #println(_jacobian_M)
        A[:, i] = _jacobian_M * dy
    end
    A *= dy

    # 第二項を計算
    _xMx(y) = xMx(p, y, yd, ye, dy, alpha)
    B = 1/2 .* ForwardDiff.gradient(_xMx, y)  # 便利
    
    return A .- B
end

"""fromGDSのアトラクター力"""
function f(p::RMPfromGDSImpedance{T}, y, yd, ye, dy, M, alpha) where T
    e_d = y - yd
    e_e = y - ye
    inv_M_d = inv(p.M_d)
    D_tilde_d = inv_M_d * p.D_d
    D_tilde_e = inv_M_d * p.D_e
    

    # P_tilde_d =  inv_M_d * p.P_d
    # P_tilde_e =  inv_M_d * p.P_e

    #return M * (-p.gain .* soft_normal(z, p.f_alpha) .- damp .* dx) .- ξ(p, x, dx, x₀)


    z = M * (-∇potential_I(p, y, yd, ye, alpha) .- (D_tilde_d .+ alpha .* D_tilde_e) * dy)*1.5 #.-
    #ξ(p, y, yd, ye, dy)

    
    return z
end

""""""
function get_natural(p::RMPfromGDSImpedance{T}, y, yd, ye, dy, center, r) where T
    #println("呼ばれた！")
    s = r - norm(y-center)
    alpha = sigmoid(s, p.a)
    #println(alpha)
    M = inertia_matrix(p, y, yd, ye, dy, alpha)
    return f(p, y, yd, ye, dy, M, alpha), M
end





end