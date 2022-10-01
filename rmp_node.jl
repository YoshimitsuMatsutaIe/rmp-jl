using LinearAlgebra
#using Parameters

include("rmp_leaf.jl")

mutable struct Node{T}
    name::String
    dim::Int64
    x::Vector{T}
    x_dot::Vector{T}
    x_ddot::Union{Vector{T}, Nothing}
    J::Union{Matrix{T}, Nothing}
    J_dot::Union{Matrix{T}, Nothing}
    mapping::Union{Any, Nothing}
    rmp_functor::Union{Nothing, GoalAttractor{T}, ObstacleAvoidnce{T}, JointLimitAvoidance{T}}
    f::Vector{T}
    M::Matrix{T}
    parent::Union{Node{T}, Nothing}
    children::Vector#{Node{T}}
end

function Node()
end

function Node(
    dim::Int64, parent_dim::Int64,
    mapping::Union{Any, Nothing};
    name::String="node"
)
    println("node")
    Node(
        name,
        dim,
        zeros(Float64, dim),
        zeros(Float64, dim),
        nothing,
        zeros(Float64, dim, parent_dim),
        zeros(Float64, dim, parent_dim),
        mapping,
        nothing,
        zeros(Float64, dim),
        zeros(Float64, dim, dim),
        nothing,
        []
    )
end


function Node(
    dim::Int64, parent_dim::Int64,
    mapping::Union{Any, Nothing},
    rmp_functor::Union{GoalAttractor{T}, ObstacleAvoidnce{T}, JointLimitAvoidance{T}};
    name::String="leaf"
    ) where T
    println("leaf")
    Node(
        name,
        dim,
        zeros(T, dim),
        zeros(T, dim),
        nothing,
        zeros(T, dim, parent_dim),
        zeros(T, dim, parent_dim),
        mapping,
        rmp_functor,
        zeros(T, dim),
        zeros(T, dim, dim),
        nothing,
        []
    )
end

function Node(dim::Int64)
    println("root node")
    Node(
        "root",
        dim,
        zeros(Float64, dim),
        zeros(Float64, dim),
        zeros(Float64, dim),
        nothing,
        nothing,
        nothing,
        nothing,
        zeros(Float64, dim),
        zeros(Float64, dim, dim),
        nothing,
        []
    )
end


function print_state(node::Node)
    if isnothing(node.parent)
        println("\nprint state ...")
    end
    println("name = ", node.name)
    println("x =\n", node.x)
    println("x_dot =\n", node.x)
    if !isnothing(node.parent)
        println("J =\n", node.J)
        println("\nJ_dot = \n", node.J_dot)
    end
    println("f = \n", node.f)
    println("M = \n", node.M)
    if length(node.children) == 0
        println("child = none\n")
        return
    else
        "child = " |> print
        for child in node.children
            child.name * ", " |> print
        end
        println("\n")
        for child in node.children
            print_state(child)
        end
    end
end


function add_child!(parent::Node, child::Node)
    push!(parent.children, child)
    child.parent = parent
end



function pushforward!(root::Node{T}, q::Vector{T}, q_dot::Vector{T}) where {T}
    #println("push at ", root.name)
    root.x = q
    root.x_dot = q_dot
    for child in root.children
        child.mapping(root.x, root.x_dot, child.x, child.x_dot, child.J, child.J_dot)
        pushforward!(child)
    end
end


function pushforward!(node::Node)
    #println("push at ", node.name)
    for child in node.children
        child.mapping(node.x, node.x_dot, child.x, child.x_dot, child.J, child.J_dot)
        if length(child.children) != 0
            pushforward!(child)
        end
    end
end



function pullback!(node::Node)
    #println("pull at ", node.name)
    node.M = zero(node.M)
    node.f = zero(node.f)
    if isnothing(node.rmp_functor)
        for child in node.children
            pullback!(child)
        end
    else
        node.rmp_functor(node.x, node.x_dot, node.M, node.f)
    end
    if !isnothing(node.parent)
        node.parent.M += node.J' * node.M * node.J
        node.parent.f += node.J' * (node.f - node.M * node.J_dot * node.parent.x_dot)
    else
        return
    end
end


function resolve!(root::Node)
    @assert !isnothing(root.x_ddot)
    root.x_ddot = pinv(root.M) * root.f
end


function solve!(root::Node{T}, q::Vector{T}, q_dot::Vector{T}) where T
    pushforward!(root, q, q_dot)
    pullback!(root)
    resolve!(root)
    root.x_ddot
end

