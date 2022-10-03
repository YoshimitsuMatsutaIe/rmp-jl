

function get_robot_struct(rss::Vector{Vector})
    s = Int[]
    for rs in rss
        push!(s, length(rs))
    end
    s
end



function get_cpoint_id(rss::Vector{Vector})
    ids = Vector[Tuple{Int, Int}][]
    for (i, rs) in enumerate(rss)
        for (j, r) in enumerate(rs)
            push!(ids, (i, j))
        end
    end
    ids
end