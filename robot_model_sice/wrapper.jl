"""ラッパー"""
#const mylib = joinpath(pwd(), "/robot_model_sice/build/lib_sice_kinematics.so")

const mylib = "/home/matsuta_conda/src/rmp-jl/robot_model_sice/build/lib_sice_kinematics.so"


#const mylib = "build/lib_sice_kinematics.so"

const c_dim = 4
const t_dim = 2

const l1 = 1.0
const l2 = 1.0
const l3 = 1.0
const l4 = 1.0

function o(n::Int64, q::Vector{Float64})
    out = Vector{Float64}(undef, t_dim)
    ccall(
        (:o, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, l1, l2, l3, l4, out
    )
    out
end

function rx(n::Int64, q::Vector{Float64})
    out = Vector{Float64}(undef, t_dim)
    ccall(
        (:rx, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, l1, l2, l3, l4, out
    )
    out
end

function ry(n::Int64, q::Vector{Float64})
    out = Vector{Float64}(undef, t_dim)
    ccall(
        (:ry, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, l1, l2, l3, l4, out
    )
    out
end


function jo(n::Int64, q::Vector{Float64})
    out = Matrix{Float64}(undef, c_dim, t_dim)
    ccall(
        (:jo, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, l1, l2, l3, l4, out
    )
    out'
end

function jrx(n::Int64, q::Vector{Float64})
    out = Matrix{Float64}(undef, c_dim, t_dim)
    ccall(
        (:jrx, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, l1, l2, l3, l4, out
    )
    out'
end

function jry(n::Int64, q::Vector{Float64})
    out = Matrix{Float64}(undef, c_dim, t_dim)
    ccall(
        (:jry, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, l1, l2, l3, l4, out
    )
    out'
end



function jo_dot(n::Int64, q::Vector{Float64}, q_dot::Vector{Float64})
    out = Matrix{Float64}(undef, c_dim, t_dim)
    ccall(
        (:jo_dot, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, q_dot, l1, l2, l3, l4, out
    )
    out'
end

function jrx_dot(n::Int64, q::Vector{Float64}, q_dot::Vector{Float64})
    out = Matrix{Float64}(undef, c_dim, t_dim)
    ccall(
        (:jrx_dot, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, q_dot, l1, l2, l3, l4, out
    )
    out'
end

function jry_dot(n::Int64, q::Vector{Float64}, q_dot::Vector{Float64})
    out = Matrix{Float64}(undef, c_dim, t_dim)
    ccall(
        (:jry_dot, mylib),
        Cvoid,
        (Cint, Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}),
        n, q, q_dot, l1, l2, l3, l4, out
    )
    out'
end


#@show jo(2, [pi/3, pi/3, pi/3, pi/3])