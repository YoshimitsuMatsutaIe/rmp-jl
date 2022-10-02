

function get_robot_struct(rss::Vector{Vector})
    s = Int[]
    for rs in rss
        push!(s, length(rs))
    end
    s
end