

include("rmp_node.jl")
include("sice_kinematics.jl")
include("mapping.jl")
include("utils.jl")




using .SiceKinematics



function make_tree(
    goal::Vector{T}, obs::Vector{Vector{T}},
    rmp_param::Dict, robot_name::String
) where T

    if robot_name == "sice"
        rm = SiceKinematics
    else
        AssertionError
    end

    c_dim = rm.c_dim
    t_dim = rm.t_dim

    root = Node(c_dim)

    jl_avo = JointLimitAvoidance(
        q_neutral = rm.q_neutral,
        q_max = rm.q_max,
        q_min = rm.q_min;
        rmp_param["joint_limit_avoidance"]...
    )
    jl_node = Node(
        c_dim,
        c_dim,
        id_map(),
        jl_avo,
        name="jl"
    )
    add_child!(root, jl_node)


    


    root
end


g = [1., 2.]
obs = [[3., 4.]]

make_tree(g, obs, Dict(), "sice")