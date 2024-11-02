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

function boid_movement(state::WorldState, i::Int, max_radius::Int64, centering_factor::Float64) 
  sum_velocity = (0.0, 0.0) 
  centerX = 0.0 
  centerY = 0.0 
  neighbor_count = 0 
  for j in 1:length(state.boids) 
    if i != j
       distance = sqrt((state.boids[j][1] - state.boids[i][1])^2 + (state.boids[j][2] - state.boids[i][2])^2) 
       if distance < max_radius 
        sum_velocity = (sum_velocity[1] + state.velocities[j][1], sum_velocity[2] + state.velocities[j][2]) 
        centerX += state.boids[j][1] 
        centerY += state.boids[j][2] 
        neighbor_count += 1 
      end 
    end 
  end 
  
  if neighbor_count > 0 
    # Выравнивание скоростей 
    sum_velocity = (sum_velocity[1] / neighbor_count, sum_velocity[2] / neighbor_count)
    state.velocities[i] = sum_velocity 
    # Движение к центру масс 
    centerX /= neighbor_count 
    centerY /= neighbor_count 
    state.velocities[i] = (state.velocities[i][1] + (centerX - state.boids[i][1]) * centering_factor,
                          state.velocities[i][2] + (centerY - state.boids[i][2]) * centering_factor) 
  end 
end

# Функция для обновления состояния
function update!(state::WorldState) 
  max_radius = 10
  centering_factor = 0.005
  for i in 1:length(state.boids) 
    boid_movement(state, i, max_radius, centering_factor) 
    # Обновление позиции 
    new_position = (state.boids[i][1] + state.velocities[i][1], state.boids[i][2] + state.velocities[i][2]) 
    state.boids[i] = (mod(new_position[1], state.width), mod(new_position[2], state.height)) 
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
