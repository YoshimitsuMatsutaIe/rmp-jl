using Plots
using DifferentialEquations
using JSON
#using JSON3
using DataFrames
using CSV

include("environment.jl")
include("utils.jl")
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
    #@show t
    #@show X
    dim = root.dim
    dX[1:dim] .= X[dim+1:end]
    dX[dim+1:end] .= solve!(root, X[1:dim], X[dim+1:end])
end


function run(json_path)

    param = JSON.parsefile(json_path)

    # f = open(json_path, "r")
    # u = read(f, String)

    # close(f)
    # param = JSON3.read_json_str(u)
    # @show param |> typeof

    robot_name = param["robot_name"]
    if robot_name == "sice"
        rm = SiceKinematics
    elseif robot_name == "particle"
        rm = ParticleKinematics
    else
        AssertionError
    end

    obs = set_obstacle(param["env_param"]["obstacle"])
    goal = set_goal(param["env_param"]["goal"])

    root = make_tree2(goal, obs, param["rmp_param"], param["robot_name"])

    dt = param["time_interval"]
    tend = param["time_span"]
    tstops = 0.0:dt:tend
    t_span = (0.0, param["time_span"])
    X₀ = vcat(rm.q_neutral, zero(rm.q_neutral))
    prob = ODEProblem(dX, X₀, t_span, root)
    @time sol = solve(prob; tstops)

    #X = hcat(sol.u...)'
    #display(X)

    #@show sol(2.0)


    # plot(sol.t, X[:, 1], label="q1")
    # plot!(sol.t, X[:, 2], label="q2")
    # plot!(sol.t, X[:, 3], label="q3")
    # plot!(sol.t, X[:, 4], label="q4")
    # savefig("tmp/configration.png")
    df = DataFrame(sol)
    CSV.write("tmp/configration.csv", df, header=[:t, :x1, :x2, :x3, :x4, :dx1, :dx2, :dx3, :dx4])
    plot(sol.t)
end




#@time run("config/test_particle.json")

#@time run("../rmp-py/config/test_sice.json")
run("../rmp-py/config/sice_non_obs.json")