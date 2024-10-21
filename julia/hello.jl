module Boids
using Plots

mutable struct WorldState
    boids::Vector{Tuple{Float64, Float64}}
    velocities::Vector{Tuple{Float64, Float64}}
    height::Float64
    width::Float64
    function WorldState(n_boids, h, w)
        boids = [(rand(0:w), rand(0:h)) for _ in 1:n_boids]
        speed = 2
        velocities = [(rand(-1.0:0.1:1.0) * speed, rand(-1.0:0.1:1.0) * speed) for _ in 1:n_boids]
        new(boids, velocities, h, w)
    end
end

function update!(state::WorldState)
    max_radius = 20
    min_radius = 5
    for i in 1:length(state.boids)
        boid = state.boids[i]
        velocity = state.velocities[i]
        
        # Находим боидов в радиусе 
        neighbors = []
        sum_rast = (0.0, 0.0)
        for j in 1:length(state.boids)
            if i != j
                neighbor = state.boids[j]
                neighbor_velocity= state.velocities[j]
                rast= (-boid[1] + neighbor[1], -boid[2] + neighbor[2])
                dist = sqrt(rast[1]^2 + rast[2]^2)
                if dist < max_radius
                    sum_rast = (sum_rast[1] + rast[1], sum_rast[2] + rast[2])
                    push!(neighbors, neighbor)
                end
                if dist < min_radius 
                    sum_rast = (sum_rast[1] - rast[1]*2,
                     sum_rast[2] - rast[2]*2)
                end
                sum_rast= (neighbor_velocity[1]/length(neighbor)+sum_rast[1],
                neighbor_velocity[2]/length(neighbor)+sum_rast[2])
              end
        position = state.boids[i]
        pos_x = position[1]
        pos_y = position[2]
        if pos_x < 5
          sum_rast = (sum_rast[1] + (30 - pos_x)/3, sum_rast[2])
        end    
        if pos_x > 25
          sum_rast = (sum_rast[1] - (30 - pos_x)/3, sum_rast[2])
        end   
        if pos_y < 5
          sum_rast = (sum_rast[1], sum_rast[2] + (30 - pos_x)/3)
        end    
        if pos_y > 25
          sum_rast = (sum_rast[1], sum_rast[2] - (30 - pos_x)/3)
        end   
        end
         # Если найдены соседи, то направляем боид в направлении суммарного rast
         if length(neighbors) > 0
            avg_rast = (sum_rast[1] / length(neighbors), sum_rast[2] / length(neighbors))
            new_velocity = (velocity[1] * 0.9 + avg_rast[1] * 0.1, velocity[2] * 0.9 + avg_rast[2] * 0.1)
        else
            new_velocity = velocity
        end

        # Обновляем позиции
        new_position = (boid[1] + new_velocity[1], boid[2] + new_velocity[2])
           


      
        # Обновление позиции и скорости
        state.boids[i] = (mod(new_position[1], state.width), mod(new_position[2], state.height))
        state.velocities[i] = new_velocity

      
    end
    return nothing
end

function (@main)(ARGS)
    h = 30
    w = 30
    n_boids = 10
    state = WorldState(n_boids, h, w)
    anim = @animate for time = 1:1000
        update!(state)
        boids = state.boids
        scatter(boids, xlim = (0, state.width), ylim = (0, state.height), label=false)
    end
    gif(anim, "boids.gif", fps = 10)
end

end

using .Boids
Boids.main("")
