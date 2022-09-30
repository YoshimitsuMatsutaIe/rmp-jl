using LinearAlgebra
using Parameters

include("rmp_leaf.jl")

@with_kw mutable struct Node{T}
    dim::Int64
    x::Vector{T}
    x_dot::Vector{T}
    J::Matrix{T}
    J_dot::Matrix{T}
    mapping
    rmp_functor::Union{Nothing, GoalAttractor{T}, ObstacleAvoidnce{T}, JointLimitAvoidance{T}}
    f::Vector{T}
    M::Matrix{T}
    parent#::Union{Node{T}, Root{T}, Nothing}
    children::Vector
end

# function Node()
# end

function Node(
    dim::Int64, parent_dim::Int64,
    mapping,
    rmp_functor::Union{Nothing, GoalAttractor{T}, ObstacleAvoidnce{T}, JointLimitAvoidance{T}}
) where T
    println("hogeoh")
    Node(
        dim,
        zeros(T, dim),
        zeros(T, dim),
        zeros(T, dim, dim),
        zeros(T, dim, dim),
        mapping,
        rmp_functor,
        zeros(T, dim),
        zeros(T, dim, parent_dim),
        nothing,
        Node[]
    )
end


@with_kw mutable struct Root{T}
    dim::Int64
    x::Vector{T}
    x_dot::Vector{T}
    x_ddot::Vector{T}
    f::Vector{T}
    M::Matrix{T}
    children::Vector
end

function Root(dim::Int64)
    Root(
        dim,
        zeros(Float64, dim),
        zeros(Float64, dim),
        zeros(Float64, dim),
        zeros(Float64, dim),
        zeros(Float64, dim, dim),
        Node[]
    )
end


function add_child!(root::Root, node::Node)
    push!(root.children, node)
    node.parent = root
end

function add_child!(parent::Node, child::Node)
    push!(parent.children, child)
    child.parent = parent
end



function pushforward!(root::Root{T}, q::Vector{T}, q_dot::Vector{T}) where {T}
    root.x = q
    root.x_dot = q_dot
    for child in root.children
        child.mapping(root.x, root.x_dot, child.x, child.x_dot, child.J, child.J_dot)
        pushforward!(child)
    end
end


function pushforward!(node::Node)
    for child in node.children
        child.mapping(node.x, node.x_dot, child.x, child.x_dot, child.J, child.J_dot)
        if length(child.children) != 0
            pushforward!(child)
        end
    end
end


function pullback!(root::Root)
    root.M = zero(root.M)
    root.f = zero(root.f)
    for child in root.children
        pullback!(child)
    end
end


function pullback!(node::Node)
    node.M = zero(node.M)
    node.f = zero(node.f)
    if isnothing(node.rmp_functor)
        for child in node.children
            pullback!(child)
        end
    else
        node.rmp_functor(node.x, node.x_dot, node.f, node.M)
    end
    node.parent.M += node.J' * node.M * node.J
    node.parent.f += node.J' * (node.f - node.M * node.J_dot * node.parent.x_dot)
end


function resolve!(root::Root)
    root.x_ddot = pinv(root.M) * root.f
end


function solve!(root::Root{T}, q::Vector{T}, q_dot::Vector{T}) where T
    pushforward!(root, q, q_dot)
    pullback!(root)
    resolve!(root)

    root.x_ddot
end

