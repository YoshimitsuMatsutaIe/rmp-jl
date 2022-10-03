

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

    

    p = rmp_param["joint_limit_avoidance"] |> keytosymbol
    jl_avo = JointLimitAvoidance(
        q_neutral = rm.q_neutral,
        q_max = rm.q_max,
        q_min = rm.q_min;
        p...
    )
    jl_node = Node(
        c_dim,
        c_dim,
        id_map(),
        jl_avo,
        name="jl"
    )
    add_child!(root, jl_node)

    for (i, rs) in enumerate(rm.rss)
        for (j, r) in enumerate(rs)
            cp_map_ = rm.Mapping(i, j)
            cp_node_ = Node(
                t_dim,
                c_dim,
                cp_map_,
                name="cpoint" *string(i) * ", " * string(j)
            )
            
            for (k, o) in enumerate(obs)
                p = rmp_param["obstacle_avoidance"] |> keytosymbol
                obs_avo = ObstacleAvoidnce(
                    o = o; p...
                )
                obs_node_ = Node(
                    t_dim,
                    t_dim,
                    id_map(),
                    obs_avo,
                    name = "obs_" * string(k) * "_at_" * string(i)*","*string(j)
                )
                add_child!(cp_node_, obs_node_)
            end

            if (i, j) == rm.ee_id
                p = rmp_param["goal_attractor"] |> keytosymbol
                goal_at = GoalAttractor(
                    g = goal; p...
                )
                goal_node = Node(
                    t_dim,
                    t_dim,
                    id_map(),
                    goal_at,
                    name = "goal"
                )
                add_child!(cp_node_, goal_node)
            end

            add_child!(root, cp_node_)

        end
    end


    root
end

