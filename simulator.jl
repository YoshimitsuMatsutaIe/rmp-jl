using Plots
using DifferentialEquations
using JSON
using JSON3

include("environment.jl")
include("utils.jl")
# include("rmp_leaf.jl")
# include("rmp_node.jl")

include("tree_constructor.jl")





function set_obstacle(params)
    obs = Vector{Float64}[]
    for p in params
        append!(obs, set_object(p["type"], p["param"]))
    end
    obs
end

function set_goal(param)
    @assert param["type"] != "point"
    set_object("point", param["param"])
end




function dX(dX::Vector{T}, X::Vector{T}, root::Node{T}, t::T) where T
    dim = root.dim
    X[1:dim] .= X[dim+1:end]
    dX[dim+1:end] .= solve!(root, X[1:dim], X[dim+1:end])
end



function run(json_path)

    param = JSON.parsefile(json_path)

    # f = open(json_path, "r")
    # u = read(f, String)

    # close(f)
    # param = JSON3.read_json_str(u)
    # @show param |> typeof
    if param["robot_name"] == "sice"
        rm = SiceKinematics
    else
        AssertionError
    end


    obs = set_obstacle(param["env_param"]["obstacle"])
    goal = set_goal(param["env_param"]["goal"])



    println(typeof(obs))
    
    #@show param["robot_name"] |> typeof
    root = make_tree(goal, obs, param["rmp_param"], param["robot_name"])


    t_span = (0.0, param["time_span"])
    X₀ = vcat(rm.q_neutral, zero(rm.q_neutral))
    prob = ODEProblem(dX, X₀, t_span, root)
    sol = solve(prob)

    plot(sol, label="sol")

end





p = run("config/test_sice.json")