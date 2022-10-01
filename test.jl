include("rmp_node.jl")
include("sice_kinematics.jl")
include("mapping.jl")

using .SiceKinematics


using Plots
using DifferentialEquations


const param = Dict(
    "goal_attractor" => Dict(
        "max_speed" => 5.0,
        "gain" => 3.0,
        "sigma_alpha" => 1.0,
        "sigma_gamma" => 1.0,
        "wu" => 10.0,
        "wl" => 0.1,
        "alpha" => 0.15,
        "epsilon" => 0.5
    ),
    "obstacle_avoidance" => Dict(
        "scale_rep" => 1.0,
        "scale_damp" => 1.0,
        "gain" => 5.0,
        "sigma" => 1.0,
        "rw" => 0.8
    ),
    "joint_limit_avoidance" => Dict(
        "gamma_p" => 0.05,
        "gamma_d" => 0.05,
        "lam" => 1.0,
        "sigma" => 0.1
    )
)


function main()
    goal = [0.5, 2.5]
    obs = [0.5, 1.0]

    root = Node(c_dim)


    # control point
    for i in 1:4
        add_child!(root, Node(t_dim, c_dim, Mapping(i), name="cpoint_"*string(i)))
    end

    # obs avoidance
    p = param["obstacle_avoidance"]
    obs_avo = ObstacleAvoidnce(
        o = obs,
        scale_rep = p["scale_rep"],
        scale_damp = p["scale_damp"],
        gain = p["gain"],
        sigma = p["sigma"],
        rw = p["rw"]
    )


    p = param["goal_attractor"]
    g_at = GoalAttractor(
        g = goal,
        max_speed = p["max_speed"],
        gain = p["gain"],
        sigma_alpha = p["sigma_alpha"],
        sigma_gamma = p["sigma_gamma"],
        wu = p["wu"],
        wl = p["wl"],
        alpha = p["alpha"],
        epsilon = p["epsilon"]
    )



    for (i, cp) in enumerate(root.children)
        add_child!(
            cp,
            Node(
                t_dim,
                t_dim,
                id_map(),
                obs_avo,
                name=string(i) * "_to_obs"
            )
        )
        if i == 4
            add_child!(
                cp,
                Node(
                    t_dim,
                    t_dim,
                    id_map(),
                    g_at,
                    name = "goal"
                )
            )
        end
    end

    p = param["joint_limit_avoidance"]
    jl_avo = JointLimitAvoidance(
        q_neutral = q_neutral,
        q_max = q_max,
        q_min = q_min,
        gamma_p = p["gamma_p"],
        gamma_d = p["gamma_d"],
        lambda = p["lam"],
        sigma = p["sigma"]
    )
    jl_node = Node(
        c_dim,
        c_dim,
        id_map(),
        jl_avo,
        name="jl"
    )
    add_child!(root, jl_node)



    # print_state(root)

    # pushforward!(root, q_neutral, zero(q_neutral))

    # print_state(root)

    # pullback!(root)

    # print_state(root)



    t_span = (0.0, 10.0)
    X₀ = vcat(q_neutral, zero(q_neutral))

    function dX(dX::Vector{T}, X::Vector{T}, root::Node{T}, t::T) where T
        dX[1:c_dim] .= X[c_dim+1:end]
        dX[c_dim+1:end] .= solve!(root, X[1:c_dim], X[c_dim+1:end])
    end

    prob = ODEProblem(dX, X₀, t_span, root)
    sol = solve(prob)

    plot(sol, label="sol")

end




@time main()