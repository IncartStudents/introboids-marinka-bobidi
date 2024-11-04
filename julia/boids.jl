module Boids
using Plots

mutable struct WorldState
    boids::Vector{Tuple{Float64, Float64}}
    velocities::Vector{Tuple{Float64, Float64}}
    height::Float64
    width::Float64
    radius::Float64
    function WorldState(n_boids, h, w)
        boids = [(rand(0:w), rand(0:h)) for _ in 1:n_boids]
        speed = 7
        velocities = [(rand(-1.0:0.1:1.0) * speed, rand(-1.0:0.1:1.0) * speed) for _ in 1:n_boids]
        new(boids, velocities, h, w)
    end
end

function boid_movement(state::WorldState, i::Int, max_radius::Int64, min_radius::Int, factor_center::Float64, factor_separation::Float64) 
    sum_velocity = (0, 0)
    neighbor_count_max = 0
    neighbor_count_min = 0
    sum_neigh_cord_max = (0,0)
    sum_neigh_cord_min = (0,0)

    for j in 1:length(state.boids)
        if i != j
            distance = sqrt((state.boids[j][1] - state.boids[i][1])^2 + (state.boids[j][2] - state.boids[i][2])^2)
            if distance < max_radius
                sum_velocity = (sum_velocity[1] + state.velocities[j][1], sum_velocity[2] + state.velocities[j][2])
                sum_neigh_cord_max = (sum_neigh_cord_max[1]+ state.boids[j][1], sum_neigh_cord_max[2]+ state.boids[j][2])
                neighbor_count_max += 1
            end
            if distance < min_radius
                sum_neigh_cord_min = (sum_neigh_cord_min[1]+ state.boids[j][1], sum_neigh_cord_min[2]+ state.boids[j][2])
                neighbor_count_min += 1
            end
        end
    end

    if neighbor_count_max > 0
        centermax = (sum_neigh_cord_max[1]/neighbor_count_max,
        sum_neigh_cord_max[2]/neighbor_count_max)
       
        if neighbor_count_min <= 0
            separation = (0,0)
        else
            centermin = (sum_neigh_cord_min[1]/neighbor_count_min,
            sum_neigh_cord_min[2]/neighbor_count_min)
            separation = ((centermin[1]-state.boids[i][1]) *  factor_separation,
            (centermin[2]-state.boids[i][2]) *  factor_separation)
        end

        return (sum_velocity[1] / neighbor_count_max
        + (centermax[1]-state.boids[i][1]) *  factor_center
        - separation[1], 
        sum_velocity[2] / neighbor_count_max
        + (centermax[2]-state.boids[i][2]) * factor_center
        - separation[2])
    else
        return state.velocities[i]
    end
end

function keepWithinBounds(state::WorldState, i::Int, margin::Int)
    if (state.boids[i][1] > state.width - margin)
        state.velocities[i] = (-abs(state.velocities[i][1]+0.1), 
        state.velocities[i][2])
    end
    if (state.boids[i][1] <  margin)
        state.velocities[i] = (abs(state.velocities[i][1]+0.1), 
        state.velocities[i][2])
    end
    if (state.boids[i][2] > state.height - margin)
        state.velocities[i] = (state.velocities[i][1],
        -abs(state.velocities[i][2]+0.1))
    end
    if (state.boids[i][2] < margin)
        state.velocities[i] = (state.velocities[i][1],
        abs(state.velocities[i][2]+0.1))
    end
end

# Функция для обновления состояния
function update!(state::WorldState) 
    max_radius = 20
    min_radius =3
    factor_center = 0.01
    factor_separation= 0.5
    margin = 5
    for i in 1:length(state.boids) 
        new_velocity = boid_movement(state, i, max_radius, min_radius, factor_center, factor_separation) 
    # Обновление позиции 
        new_position = (state.boids[i][1] + state.velocities[i][1],
        state.boids[i][2] + state.velocities[i][2]) 
    # Проверка границ
        keepWithinBounds(state, i, margin)
        state.boids[i] = new_position 
        state.boids[i] = (mod(new_position[1], state.width), mod(new_position[2], state.height))
        state.velocities[i] = new_velocity
    end 
end

  function (@main)(ARGS)
    h = 80
    w = 80
    n_boids = 150
    state = WorldState(n_boids, h, w)
    anim = @animate for time = 1:200
        update!(state)
        boids = state.boids
        scatter(boids, xlim = (0, state.width), ylim = (0, state.height), label=false)
    end
    gif(anim, "boids.gif", fps = 10)
end

end

using .Boids
Boids.main("")
