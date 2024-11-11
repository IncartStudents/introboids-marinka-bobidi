module Boids
using Plots

mutable struct WorldState
    boids::Vector{Tuple{Float64, Float64}}
    vec_vel::Vector{Tuple{Float64, Float64}}
    vec_accel::Vector{Tuple{Float64, Float64}}
    height::Float64
    width::Float64
    max_vel::Float64
    max_r::Float64
    factor_center::Float64
    function WorldState(n_boids, h, w, max_vel, max_r, factor_center)
        boids = [(rand() * w, rand() * h) for _ in 1:n_boids]
        vec_vel = [(rand(-1.0:0.1:1.0) * max_vel * 2, rand(-1.0:0.1:1.0) * max_vel * 2) for _ in 1:n_boids]
        vec_accel = [(0.0, 0.0) for _ in 1:n_boids]
        new(boids, vec_vel, vec_accel, h, w, max_vel, max_r, factor_center)
    end
end

# Функция для расчета движения к центру
function cohesion(state::WorldState, i::Int)
  sum_neigh_coords=(0,0)
  neighbor_count = 0
  boid_x, boid_y = state.boids[i]
  
  for j in 1:length(state.boids)
      if i != j
          neigh_x, neigh_y = state.boids[j]
          distance = hypot(neigh_x - boid_x, neigh_y - boid_y)
          if distance < state.max_r
              sum_neigh_coords = (sum_neigh_coords[1] + neigh_x, sum_neigh_coords[2] + neigh_y)
              neighbor_count += 1
          end
      end
  end

  if neighbor_count > 0
       center_x = sum_neigh_coords[1] / neighbor_count 
       center_y = sum_neigh_coords[2] / neighbor_count 
       accel_x = (center_x - boid_x) * state.factor_center
       accel_y = (center_y - boid_y) * state.factor_center
      return (accel_x, accel_y)
  else
      return (0.0, 0.0)
  end
end

# Функция для обновления состояния boids
function update!(state::WorldState)

  for i in 1:length(state.boids)
      new_vel = state.vec_vel[i]
      boid_x, boid_y = state.boids[i]

      coh_x, coh_y = cohesion(state, i) 
      accel_x= coh_x
      accel_y= coh_y 

      # Обновление скорости с учетом ускорения
      new_vel = (new_vel[1] + accel_x, new_vel[2] + accel_y)

      # Ограничение скорости
      vel_x, vel_y = new_vel
      speed = hypot(vel_x, vel_y)
      if speed > state.max_vel
         scale = state.max_vel / speed
         new_vel = (vel_x * scale, vel_y * scale)
      end

      # Обновление позиции и проверка границ
      new_position = (boid_x + new_vel[1], boid_y + new_vel[2])
      state.boids[i] = (mod(new_position[1], state.width), mod(new_position[2], state.height))
      state.vec_vel[i] = new_vel
  end
end

# Функция для запуска анимации
function main(ARGS)
  h = 80.0
  w = 80.0
  n_boids = 20
  max_r = 20
  max_vel = 1.0
  factor_center = 0.15
  state = WorldState(n_boids, h, w, max_vel, max_r, factor_center )
  anim = @animate for time in 1:200
      update!(state)
      boids = state.boids
      scatter(boids, xlim=(0, state.width), ylim=(0, state.height), label=false)
  end
  gif(anim,"cohesion.gif", fps=10)
end

end

# Запуск модуля
using .Boids
Boids.main("")
