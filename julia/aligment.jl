module Boids
using Plots

mutable struct WorldState
    boids::Vector{Tuple{Float64, Float64}}
    velocities::Vector{Tuple{Float64, Float64}}
    height::Float64
    width::Float64
    function WorldState(n_boids, h, w)
        boids = [(rand() * w, rand() * h) for _ in 1:n_boids]
        speed = 2
        velocities = [(rand(-1.0:0.1:1.0) * speed, rand(-1.0:0.1:1.0) * speed) for _ in 1:n_boids]
        new(boids, velocities, h, w)
    end
end

# Функция для расчета новой скорости на основе соседей
function alignment(state::WorldState, i::Int, max_radius::Int)
    sum_velocity = (0, 0)
    neighbor_count = 0

    for j in 1:length(state.boids)
        if i != j
            distance = sqrt((state.boids[j][1] - state.boids[i][1])^2 + (state.boids[j][2] - state.boids[i][2])^2)
            if distance < max_radius
                sum_velocity = (sum_velocity[1] + state.velocities[j][1], sum_velocity[2] + state.velocities[j][2])
                neighbor_count += 1
            end
        end
    end

    if neighbor_count > 0
        return (sum_velocity[1] / neighbor_count, sum_velocity[2] / neighbor_count)
    else
        return state.velocities[i]
    end
end

# Функция для обновления состояния boids
function update!(state::WorldState)
    max_radius = 5

    for i in 1:length(state.boids)
        # Вызов функции alignment для расчета новой скорости
        new_velocity = alignment(state, i, max_radius)

        # Обновление позиции и проверка границ
        new_position = (state.boids[i][1] + new_velocity[1], state.boids[i][2] + new_velocity[2])
        state.boids[i] = (mod(new_position[1], state.width), mod(new_position[2], state.height))
        state.velocities[i] = new_velocity
    end
end

# Функция для запуска анимации
function main(ARGS)
    h = 80.0
    w = 80.0
    n_boids = 10
    state = WorldState(n_boids, h, w)
    anim = @animate for time in 1:200
        update!(state)
        boids = state.boids
        scatter(boids, xlim=(0, state.width), ylim=(0, state.height), label=false)
    end
    gif(anim, "boids_aligment.gif", fps=10)
end

end

# Запуск модуля
using .Boids
Boids.main("")