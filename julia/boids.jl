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
        speed = 2
        velocities = [(rand(-1.0:0.1:1.0) * speed, rand(-1.0:0.1:1.0) * speed) for _ in 1:n_boids]
        new(boids, velocities, h, w)
    end
end

# Функция движения в одном направлении
function alignment(state::WorldState, i::Int, max_radius::Int)
    sum_velocity_x = 0.0
    sum_velocity_y = 0.0
    neighbor_count = 0

    position = state.boids
    velocities = state.velocities
    boids_velocity = velocities[i]

    for j in 1:length(position)
        if i != j
            pos_i_x, pos_i_y = position[i]
            pos_j_x, pos_j_y = position[j]
            distance = sqrt((pos_j_x - pos_i_x)^2 + (pos_j_y - pos_i_y)^2)
            if distance < max_radius
                sum_velocity_x += velocities[j][1]
                sum_velocity_y += velocities[j][2]
                neighbor_count += 1
            end
        end
    end

    if neighbor_count > 0
        return (sum_velocity_x / neighbor_count, sum_velocity_y / neighbor_count)
    else
        return boids_velocity 
    end
end

# Функция для расчета центра масс
function cohesion(state::WorldState, i::Int, max_radius::Int, factor_center::Float64)
    sum_neigh_cord_x = 0.0
    sum_neigh_cord_y = 0.0
    neighbor_count = 0

    position = state.boids

    for j in 1:length(position)
        if i != j
            pos_i_x, pos_i_y = position[i]
            pos_j_x, pos_j_y = position[j]
            distance = sqrt((pos_j_x - pos_i_x)^2 + (pos_j_y - pos_i_y)^2)
            if distance < max_radius
                sum_neigh_cord_x += pos_j_x
                sum_neigh_cord_y += pos_j_y
                neighbor_count += 1
            end
        end
    end

    if neighbor_count > 0
        center_x = sum_neigh_cord_x / neighbor_count
        center_y = sum_neigh_cord_y / neighbor_count
        return ((center_x - position[i][1]) * factor_center, (center_y - position[i][2]) * factor_center)
    else
        return state.velocities[i]
    end
end


# Функция отталкивания
function separation(state::WorldState, i::Int, min_radius::Int, factor_separation::Float64)
    sum_neigh_cord_x = 0.0
    sum_neigh_cord_y = 0.0
    neighbor_count = 0

    position = state.boids

    for j in 1:length(position)
        if i != j
            pos_i_x, pos_i_y = position[i]
            pos_j_x, pos_j_y = position[j]
            distance = sqrt((pos_j_x - pos_i_x)^2 + (pos_j_y - pos_i_y)^2)
            if distance < min_radius
                sum_neigh_cord_x += pos_j_x
                sum_neigh_cord_y += pos_j_y
                neighbor_count += 1
            end
        end
    end

    if neighbor_count > 0
        center_x = sum_neigh_cord_x / neighbor_count
        center_y = sum_neigh_cord_y / neighbor_count
        return (-(center_x - position[i][1]) * factor_separation, -(center_y - position[i][2]) * factor_separation)
    else
        return state.velocities[i]
    end
end

# # Функция ограничения зоны движения
# function keepWithinBounds(state::WorldState, i::Int, margin::Int)
#     if (state.boids[i][1] > state.width - margin)
#         state.velocities[i] = (-abs(state.velocities[i][1]+0.1), 
#         state.velocities[i][2])
#     end
#     if (state.boids[i][1] <  margin)
#         state.velocities[i] = (abs(state.velocities[i][1]+0.1), 
#         state.velocities[i][2])
#     end
#     if (state.boids[i][2] > state.height - margin)
#         state.velocities[i] = (state.velocities[i][1],
#         -abs(state.velocities[i][2]+0.1))
#     end
#     if (state.boids[i][2] < margin)
#         state.velocities[i] = (state.velocities[i][1],
#         abs(state.velocities[i][2]+0.1))
#     end
# end

# Функция для обновления состояния
function update!(state::WorldState)
    max_radius = 10
    min_radius = 3
    factor_center = 0.1
    factor_separation = 0.5
    margin = 5

    for i in 1:length(state.boids)
        # Вычисляем компоненты новой скорости по каждому правилу
        sep_x, sep_y = separation(state, i, min_radius, factor_separation)
        align_x, align_y = alignment(state, i, max_radius)
        coh_x, coh_y = cohesion(state, i, max_radius, factor_center)

        # Суммируем компоненты для получения новой скорости
        new_velocity_x = sep_x + align_x + coh_x
        new_velocity_y = sep_y + align_y + coh_y
        new_velocity = (new_velocity_x, new_velocity_y)

        # Обновление позиции
        boid_x, boid_y = state.boids[i]
        vel_x, vel_y = state.velocities[i]
        new_position = (boid_x + vel_x, boid_y + vel_y)

        # Проверка границ
        # keepWithinBounds(state, i, margin)

        # Обновление состояния
        state.boids[i] = (mod(new_position[1], state.width), mod(new_position[2], state.height))
        state.velocities[i] = new_velocity
    end
end



  function (@main)(ARGS)
    h = 80
    w = 80
    n_boids = 10
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
