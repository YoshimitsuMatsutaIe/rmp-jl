using Plots
using DifferentialEquations
using JSON

include("rmp_leaf.jl")
include("rmp_node.jl")


function solve(X_dot, X, param, t)
    
    q = X[]



end