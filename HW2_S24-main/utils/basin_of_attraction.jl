function plot_basin_of_attraction(initial_conditions, convergence_list, ps, thetas)
    function nearest_neighbor(x, y)
        distances = [norm([xic[1], rad2deg(xic[2]),0, 0] .- [x,y,0,0]) for xic in initial_conditions]
        min_distance, nearest_index = findmin(distances)
        return convergence_list[nearest_index]
    end
    
    contourf(ps, thetas, nearest_neighbor, f=true, levels = 1, c=[:red, :green], cbar = false)
    for (initial_condition, convergence) in zip(initial_conditions, convergence_list)
        if convergence
            color = :green
        else
            color = :red
        end
        plot!([initial_condition[1]], [rad2deg(initial_condition[2])],
              seriestype=:scatter, color=color, label = false)
    end
        
    plot!([],[],seriestype=:scatter, color=:red, label = "failure")
    plot!([],[],seriestype=:scatter, color=:green, label = "success")
    display(plot!(xlabel = "p_0 (meters)", ylabel = "θ_0 (degrees)", legend = true))
end