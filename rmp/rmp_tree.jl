using Parameters


mutable struct Root{T}
    name::String
    self_dim::Int64
    children::Vector{String}
    q::Vector{T}
    q̇::Vector{T}
    q̈::Vector{T}
    f::Vector{T}
    M::Matrix{T}
end

function Root(self_dim::Int64)
    name = "root"
    children = String[]
    q = zeros(Float64, self_dim)
    q̇ = zero(q)
    q̈ = zero(q)
    f = zero(q)
    M = zeros(Float64, self_dim, self_dim)
    Root(name, children, self_dim, q, q̇, q̈, f, M)
end


"""ノード"""
mutable struct Node{T}
    name::String
    parent::String
    self_dim::Int64
    parent_dim::Int64
    children::Vector{String}
    x::Vector{T}
    ẋ::Vector{T}
    J::Matrix{T}
    J̇::Matrix{T}
    f::Vector{T}
    M::Matrix{T}
end


function Node(name::String, parent::String, self_dim::Int64, parent_dim::Int64)
    children = String[]
    x = zeros(Float64, self_dim)
    ẋ = zero(x)
    J = zeros(Float64, self_dim, parent_dim)
    J̇ = zero(J)
    f = zero(x)
    M = zeros(Float64, self_dim, parent_dim)
    Node(name, parent, self_dim, parent_dim, children, x, ẋ, J, J̇, f, M)
end


mutable struct Leaf{T}
    name::String
    self_dim::Int64
    parent_dim::Int64
    parent::String
    x::Vector{T}
    ẋ::Vector{T}
    J::Matrix{T}
    J̇::Matrix{T}
    f::Vector{T}
    M::Matrix{T}
end


function Leaf(name::String, parent::String, self_dim::Int64, parent_dim::Int64)
    x = zeros(Float64, self_dim)
    ẋ = zero(x)
    J = zeros(Float64, self_dim, parent_dim)
    J̇ = zero(J)
    f = zero(x)
    M = zeros(Float64, self_dim, parent_dim)
    Leaf(name, parent, self_dim, parent_dim, x, ẋ, J, J̇, f, M)
end



function pushforward!(root::Root{T}, rmp_tree::Dict{String, Any}) where {T}
    for child in root.children
        update!(root.x, root.ẋ, rmp_tree[child].x, rmp_tree[child].ẋ, rmp_tree[child].J, rmp_tree[child].J̇)
        pushforward!(rmp_tree[child], rmp_tree)
    end
end

function pushforward!(node::Node{T}, rmp_tree::Dict{String, Any}) where {T}
    for child in node.children
        pushforward!(rmp_tree[child], rmp_tree)
    end
end

function pushforward!(leaf::Leaf{T}, rmp_tree::Dict{String, Any}) where {T}
    println("this is leaf")
end