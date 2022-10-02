"""環境いろいろ"""

#using Random
#sd = Random.seed!(123)
#using Parameters


"""3d rotation matrix"""
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

"""2d rotation matrix"""
function rotate(a::T) where T
    a = deg2rad(a)
    return [
        cos(a) -sin(a)
        sin(a) cos(a)
    ]
end



function set_point(x::T, y::T, z::T) where T
    [x, y, z]
end

function set_point(x::T, y::T) where T
    [x, y]
end



function set_sphere(d::T, r::T, x::T, y::T) where T
    betas = 0:d/r:2π
    obs = Vector{Vector{T}}(undef, length(betas))
    for (i, β) in enumerate(betas)
        obs[i] = [
                r*cos(β) + x
                r*sin(β) + y
            ]
    end
    obs
end


function set_sphere(d::T, r::T, x::T, y::T, z::T) where T
    N = round(Int, 4 * 4^2/d^2)
    obs = Vector{Vector{T}}(undef, N)
    θ = π
    ϕ = 0
    for i in 1:N
        if i == 1
            # pass
        elseif i == N
            θ = 0
            ϕ = π
        else
            h = 2*(i-1) / (N-1) - 1
            θ = acos(h)
            ϕ += 3.6/√N * 1/√(1-h^2)
        end
        obs[i] = [
                r*sin(θ)*cos(ϕ) + x
                r*sin(θ)*sin(ϕ) + y
                r*cos(θ) + z
            ]
    end
    obs
end


function set_cylinder(
    d::T, r::T, L::T, x::T, y::T, z::T, alpha::T, beta::T, gamma::T
) where T
    R = rotate(alpha, beta, gamma)
    center = [x, y, z]
    thetas = 0:d/r:2π
    ss = -L/2:d:L/2
    obs = Vector{Vector{T}}(undef, length(thetas)*length(ss))

    i = 0
    for θ in thetas
        for s in ss
            obs[i] = R * [
                r * cos(θ)
                r * sin(θ)
                s
            ] + center
            i += 1
        end
    end

    obs
end


function set_field(
    d::T, lx::T, ly::T, x::T, y::T, z::T, alpha::T, beta::T, gamma::T
) where T
    R = rotate(alpha, beta, gamma)
    center = [x, y, z]
    ss = -lx/2:d:lx/2
    ts = -ly/2:d:ly/2
    obs = Vector{Vector{T}}(undef, length(ss)*length(ts))
    i = 1
    for s in ss
        for t in ts
            obs[i] = R * [s, t, 0] + center
            i += 1
        end
    end
    obs
end


function set_box(
    d::T, lx::T, ly::T, lz::T, x::T, y::T, z::T, alpha::T, beta::T, gamma::T
) where T
    obs = Vector{Vector{T}}[]
    append!(obs, set_field(d, lx, ly, x, y, z+lz/2, alpha, beta, gamma))
    append!(obs, set_field(d, lx, ly, x, y, z-lz/2, alpha, beta, gamma))
    append!(obs, set_field(d, lx, lz, x, y+ly/2, z, alpha+90, beta, gamma))
    append!(obs, set_field(d, lx, lz, x, -y+ly/2, z, alpha+90, beta, gamma))
    append!(obs, set_field(d, lx, ly, x+lx/2, y, z, alpha, beta+90, gamma))
    append!(obs, set_field(d, lx, ly, x+-lx/2, y, z, alpha, beta+90, gamma))
    obs
end

function set_cubbie(
    d::T, lx::T, ly::T, lz::T, x::T, y::T, z::T, alpha::T, beta::T, gamma::T
) where T
    obs = Vector{Vector{T}}[]
    append!(obs, set_field(d, lx, ly, x, y, z+lz/2, alpha, beta, gamma))
    append!(obs, set_field(d, lx, ly, x, y, z-lz/2, alpha, beta, gamma))
    append!(obs, set_field(d, lx, lz, x, y+ly/2, z, alpha+90, beta, gamma))
    #append!(obs, set_field(d, lx, lz, x, -y+ly/2, z, alpha+90, beta, gamma))
    append!(obs, set_field(d, lx, ly, x+lx/2, y, z, alpha, beta+90, gamma))
    append!(obs, set_field(d, lx, ly, x+-lx/2, y, z, alpha, beta+90, gamma))
    obs
end


function set_object(type::String, p::Dict)
    if type == "cylinder"
        return set_cylinder(p["d"], p["r"], p["L"], p["x"], p["y"], p["z"], p["alpha"], p["beta"], p["gamma"])
    elseif  type == "point"
        if !haskey(p, "z")
            return set_point(p["x"], p["y"])
        else
            return set_point(p["x"], p["y"], p["z"])
        end
    elseif type == "sphere"
        if !haskey(p, "z")
            return set_sphere(p["d"], p["r"], p["x"], p["y"])
        else
            return set_sphere(p["d"], p["r"], p["x"], p["y"], p["z"])
        end
    elseif type == "field"
        return set_field(p["d"], p["lx"], p["ly"], p["x"], p["y"], p["z"], p["alpha"], p["beta"], p["gamma"])
    elseif type == "box"
        return set_box(p["d"], p["lx"], p["ly"], p["lz"], p["x"], p["y"], p["z"], p["alpha"], p["beta"], p["gamma"])
    elseif type == "cubie"
        return set_cubie(p["d"], p["lx"], p["ly"], p["lz"], p["x"], p["y"], p["z"], p["alpha"], p["beta"], p["gamma"])
    else
        AssertionError
    end
end