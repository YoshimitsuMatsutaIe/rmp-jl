using Plots
using DifferentialEquations
using JSON

include("environment.jl")
include("utils.jl")
include("rmp_leaf.jl")
include("rmp_node.jl")






function solve(X_dot, X, param, t)
    
    q = X[]



end


function set_obstacle(params)
    obs = Vector[]
    for p in params
        append!(obs, set_object(p["type"], p["param"]))
    end
    obs
end

function set_goal(param)
    @assert param["type"] != "point"
    set_object("point", param["param"])
end


function run(json_path)

    param = JSON.parsefile(json_path)

    obs = set_obstacle(param["env_param"]["obstacle"])
    goal = set_goal(param["env_param"]["goal"])


    


end





p = run("config/test_sice.json")