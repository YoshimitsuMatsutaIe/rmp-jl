include("rmp_node.jl")
include("sice_kinematics.jl")
include("mapping.jl")

using .SiceKinematics

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


goal = [0.5, 2.5]
obs = [0.5, 1.0]

root = Root(c_dim)

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
    jl_avo
)
add_child!(root, jl_node)

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


p = param["obstacle_avoidance"]
obs_avo = ObstacleAvoidnce(
    o = obs,
    scale_rep = p["scale_rep"],
    scale_damp = p["scale_damp"],
    gain = p["gain"],
    sigma = p["sigma"],
    rw = p["rw"]
)


