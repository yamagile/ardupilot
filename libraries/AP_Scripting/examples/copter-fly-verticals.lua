-- command a Copter to takeoff to 5m and fly a vertical circle in the clockwise direction
--
-- CAUTION: This script only works for Copter
-- this script waits for the vehicle to be armed and RC6 input > 1800 and then:
--    a) switches to Guided mode
--    b) takeoff to 5m
--    c) flies a vertical circle using the velocity controller
--    d) switches to RTL mode

local takeoff_alt_above_home = 5
local copter_guided_mode_num = 4
local copter_rtl_mode_num = 6
local stage = 0
local circle_angle = 0
local circle_angle_increment = 1    -- increment the target angle by 1 deg every 0.1 sec (i.e. 10deg/sec)
local circle_speed = 1              -- velocity is always 1m/s
local yaw_cos = 0                   -- cosine of yaw at takeoff
local yaw_sin = 0                   -- sine of yaw at takeoff
local count = 0
local count_end = 0
local index = 0
local vertical_points = {{x=0, y=0}, {x=-0.74, y=5.02}, {x=-0.86, y=5.02}, {x=-0.98, y=5.02}, {x=-1.02, y=4.9}, {x=-1.08, y=4.78}, {x=-1.14, y=4.66}, {x=-1.24, y=4.6}, {x=-1.36, y=4.56}, {x=-1.48, y=4.52}, {x=-1.58, y=4.48}, {x=-1.7, y=4.46}, {x=-1.8, y=4.44}, {x=-1.92, y=4.44}, {x=-2.02, y=4.44}, {x=-2.12, y=4.44}, {x=-2.22, y=4.44}, {x=-2.34, y=4.44}, {x=-2.44, y=4.46}, {x=-2.56, y=4.52}, {x=-2.66, y=4.56}, {x=-2.78, y=4.58}, {x=-2.88, y=4.62}, {x=-2.98, y=4.62}, {x=-3.08, y=4.62}, {x=-3.2, y=4.64}, {x=-3.32, y=4.72}, {x=-3.42, y=4.76}, {x=-3.52, y=4.82}, {x=-3.62, y=4.84}, {x=-3.74, y=4.9}, {x=-3.86, y=4.96}, {x=-3.9, y=4.84}, {x=-3.82, y=4.72}, {x=-3.7, y=4.62}, {x=-3.64, y=4.5}, {x=-3.58, y=4.38}, {x=-3.48, y=4.3}, {x=-3.36, y=4.22}, {x=-3.26, y=4.12}, {x=-3.16, y=4.02}, {x=-3.04, y=3.98}, {x=-2.92, y=3.94}, {x=-2.8, y=3.88}, {x=-2.68, y=3.82}, {x=-2.58, y=3.78}, {x=-2.48, y=3.72}, {x=-2.38, y=3.68}, {x=-2.28, y=3.66}, {x=-2.16, y=3.62}, {x=-2.02, y=3.54}, {x=-1.92, y=3.52}, {x=-1.8, y=3.48}, {x=-1.7, y=3.46}, {x=-1.58, y=3.44}, {x=-1.48, y=3.42}, {x=-1.36, y=3.4}, {x=-1.24, y=3.4}, {x=-1.14, y=3.4}, {x=-1.02, y=3.36}, {x=-0.9, y=3.36}, {x=-0.76, y=3.36}, {x=-0.64, y=3.36}, {x=-0.52, y=3.36}, {x=-0.4, y=3.36}, {x=-0.3, y=3.36}, {x=-0.16, y=3.38}, {x=-0.02, y=3.4}, {x=0.1, y=3.44}, {x=0.22, y=3.48}, {x=0.36, y=3.5}, {x=0.5, y=3.5}, {x=0.62, y=3.5}, {x=0.74, y=3.52}, {x=0.86, y=3.56}, {x=0.98, y=3.6}, {x=1.08, y=3.64}, {x=1.2, y=3.68}, {x=1.3, y=3.74}, {x=1.42, y=3.76}, {x=1.52, y=3.78}, {x=1.52, y=3.6}, {x=1.44, y=3.48}, {x=1.32, y=3.38}, {x=1.22, y=3.3}, {x=1.1, y=3.24}, {x=0.98, y=3.16}, {x=0.84, y=3.04}, {x=0.7, y=2.92}, {x=0.56, y=2.86}, {x=0.46, y=2.8}, {x=0.36, y=2.72}, {x=0.24, y=2.62}, {x=0.12, y=2.56}, {x=-0.02, y=2.48}, {x=-0.16, y=2.42}, {x=-0.28, y=2.42}, {x=-0.42, y=2.4}, {x=-0.52, y=2.4}, {x=-0.64, y=2.38}, {x=-0.76, y=2.38}, {x=-0.9, y=2.38}, {x=-1.02, y=2.38}, {x=-1.12, y=2.36}, {x=-1.26, y=2.36}, {x=-1.16, y=2.28}, {x=-1.02, y=2.24}, {x=-0.88, y=2.24}, {x=-0.76, y=2.24}, {x=-0.64, y=2.22}, {x=-0.52, y=2.2}, {x=-0.42, y=2.2}, {x=-0.3, y=2.18}, {x=-0.18, y=2.18}, {x=-0.04, y=2.18}, {x=0.08, y=2.18}, {x=0.22, y=2.18}, {x=0.34, y=2.18}, {x=0.46, y=2.18}, {x=0.58, y=2.18}, {x=0.68, y=2.18}, {x=0.82, y=2.18}, {x=0.94, y=2.24}, {x=1.04, y=2.26}, {x=1.16, y=2.28}, {x=1.26, y=2.3}, {x=1.36, y=2.3}, {x=1.48, y=2.3}, {x=1.58, y=2.3}, {x=1.7, y=2.32}, {x=1.8, y=2.36}, {x=1.92, y=2.42}, {x=2.04, y=2.48}, {x=2.14, y=2.56}, {x=2.26, y=2.6}, {x=2.36, y=2.66}, {x=2.46, y=2.72}, {x=2.56, y=2.76}, {x=2.68, y=2.86}, {x=2.74, y=2.96}, {x=2.82, y=3.06}, {x=2.86, y=3.18}, {x=2.88, y=3.38}, {x=2.92, y=3.5}, {x=2.92, y=3.6}, {x=3, y=3.72}, {x=3.1, y=3.8}, {x=3.2, y=3.88}, {x=3.3, y=3.98}, {x=3.4, y=4.12}, {x=3.48, y=4.26}, {x=3.58, y=4.38}, {x=3.66, y=4.48}, {x=3.74, y=4.6}, {x=3.76, y=4.7}, {x=3.8, y=4.82}, {x=3.88, y=5.02}, {x=3.94, y=5.14}, {x=4, y=5.28}, {x=4, y=5.4}, {x=4, y=5.52}, {x=4, y=5.62}, {x=4, y=5.76}, {x=4, y=5.86}, {x=3.96, y=5.98}, {x=3.86, y=6.04}, {x=3.78, y=6.18}, {x=3.72, y=6.28}, {x=3.62, y=6.38}, {x=3.52, y=6.42}, {x=3.42, y=6.42}, {x=3.32, y=6.42}, {x=3.2, y=6.44}, {x=3.1, y=6.44}, {x=3, y=6.34}, {x=2.9, y=6.28}, {x=2.86, y=6.18}, {x=2.78, y=6.06}, {x=2.68, y=5.94}, {x=2.64, y=5.84}, {x=2.6, y=5.7}, {x=2.56, y=5.84}, {x=2.52, y=5.94}, {x=2.46, y=6.1}, {x=2.44, y=6.22}, {x=2.4, y=6.32}, {x=2.38, y=6.46}, {x=2.34, y=6.58}, {x=2.32, y=6.7}, {x=2.3, y=6.82}, {x=2.18, y=6.86}, {x=2.08, y=6.9}, {x=1.98, y=6.9}, {x=1.88, y=6.9}, {x=1.76, y=6.92}, {x=1.66, y=6.92}, {x=1.5, y=6.92}, {x=1.4, y=6.92}, {x=1.28, y=6.92}, {x=1.18, y=6.94}, {x=1.06, y=6.94}, {x=0.96, y=6.94}, {x=0.84, y=6.94}, {x=0.72, y=6.9}, {x=0.6, y=6.86}, {x=0.48, y=6.84}, {x=0.36, y=6.72}, {x=0.22, y=6.62}, {x=0.08, y=6.52}, {x=-0.04, y=6.42}, {x=-0.16, y=6.36}, {x=-0.28, y=6.28}, {x=-0.4, y=6.16}, {x=-0.48, y=6.04}, {x=-0.6, y=5.94}, {x=-0.68, y=5.84}, {x=-0.7, y=5.94}, {x=-0.7, y=6.06}, {x=-0.7, y=6.16}, {x=-0.66, y=6.28}, {x=-0.64, y=6.4}, {x=-0.6, y=6.52}, {x=-0.54, y=6.64}, {x=-0.5, y=6.78}, {x=-0.46, y=6.94}, {x=-0.38, y=7.08}, {x=-0.28, y=7.22}, {x=-0.22, y=7.32}, {x=-0.1, y=7.5}, {x=-0.02, y=7.62}, {x=0.02, y=7.74}, {x=0.16, y=7.84}, {x=0.28, y=7.92}, {x=0.4, y=7.96}, {x=0.54, y=7.98}, {x=0.66, y=7.98}, {x=0.78, y=7.9}, {x=0.92, y=7.82}, {x=1.04, y=7.72}, {x=1.14, y=7.6}, {x=1.18, y=7.48}, {x=1.2, y=7.36}, {x=1.24, y=7.26}, {x=1.24, y=7.14}, {x=1.28, y=7.02}, {x=1.28, y=6.9}, {x=1.28, y=6.78}, {x=1.28, y=6.68}, {x=1.28, y=6.54}, {x=1.22, y=6.42}, {x=1.18, y=6.3}, {x=1.12, y=6.18}, {x=1.04, y=6.06}, {x=1, y=5.92}, {x=0.96, y=5.8}, {x=0.88, y=5.68}, {x=0.8, y=5.56}, {x=0.7, y=5.44}, {x=0.58, y=5.34}, {x=0.46, y=5.28}, {x=0.34, y=5.22}, {x=0.24, y=5.18}, {x=0.12, y=5.18}, {x=-0.02, y=5.18}, {x=-0.14, y=5.18}, {x=-0.26, y=5.2}, {x=-0.38, y=5.22}, {x=-0.5, y=5.24}, {x=-0.62, y=5.3}, {x=-0.7, y=5.42}, {x=-0.74, y=5.54}, {x=-0.82, y=5.66}, {x=-0.88, y=5.82}, {x=-0.92, y=5.98}, {x=-0.96, y=6.1}, {x=-1, y=6.22}, {x=-1.04, y=6.32}, {x=-1.1, y=6.44}, {x=-1.1, y=6.56}, {x=-1.12, y=6.66}, {x=-1.14, y=6.82}, {x=-1.16, y=7}, {x=-1.2, y=7.12}, {x=-1.22, y=7.24}, {x=-1.28, y=7.44}, {x=-1.28, y=7.56}, {x=-1.34, y=7.66}, {x=-1.4, y=7.78}, {x=-1.5, y=7.88}, {x=-1.6, y=7.96}, {x=-1.72, y=8.04}, {x=-1.82, y=8.04}, {x=-1.96, y=8.04}, {x=-2.06, y=8.04}, {x=-2.18, y=8}, {x=-2.3, y=7.9}, {x=-2.34, y=7.8}, {x=-2.4, y=7.66}, {x=-2.46, y=7.56}, {x=-2.48, y=7.44}, {x=-2.5, y=7.34}, {x=-2.52, y=7.22}, {x=-2.56, y=7.1}, {x=-2.6, y=6.94}, {x=-2.6, y=6.84}, {x=-2.64, y=6.72}, {x=-2.68, y=6.58}, {x=-2.68, y=6.46}, {x=-2.68, y=6.32}, {x=-2.68, y=6.22}, {x=-2.68, y=6.1}, {x=-2.68, y=5.96}, {x=-2.64, y=5.84}, {x=-2.62, y=5.7}, {x=-2.6, y=5.6}, {x=-2.54, y=5.48}, {x=-2.46, y=5.38}, {x=-2.38, y=5.26}, {x=-2.28, y=5.18}, {x=-2.16, y=5.14}, {x=-2.06, y=5.12}, {x=-1.96, y=5.12}, {x=-1.84, y=5.12}, {x=-1.74, y=5.12}, {x=-1.64, y=5.18}, {x=-1.52, y=5.24}, {x=-1.42, y=5.28}, {x=-1.3, y=5.4}, {x=-1.22, y=5.52}, {x=-1.16, y=5.62}, {x=-1.08, y=5.74}, {x=-1.06, y=5.86}, {x=-1.14, y=5.98}, {x=-1.24, y=6.08}, {x=-1.32, y=6.2}, {x=-1.42, y=6.32}, {x=-1.52, y=6.4}, {x=-1.62, y=6.5}, {x=-1.72, y=6.62}, {x=-1.82, y=6.72}, {x=-1.92, y=6.82}, {x=-2.04, y=6.96}, {x=-2.14, y=7.02}, {x=-2.24, y=7.1}, {x=-2.36, y=7.18}, {x=-2.46, y=7.2}, {x=-2.56, y=7.2}, {x=-2.66, y=7.22}, {x=-2.78, y=7.26}, {x=-2.88, y=7.34}, {x=-3, y=7.36}, {x=-3.1, y=7.36}, {x=-3.2, y=7.4}, {x=-3.32, y=7.42}, {x=-3.42, y=7.48}, {x=-3.52, y=7.5}, {x=-3.62, y=7.54}, {x=-3.72, y=7.58}, {x=-3.84, y=7.66}, {x=-3.94, y=7.66}, {x=-4.04, y=7.68}, {x=-4.16, y=7.72}, {x=-4.28, y=7.78}, {x=-4.4, y=7.8}, {x=-4.28, y=7.9}, {x=-4.18, y=7.94}, {x=-4.06, y=8.02}, {x=-3.94, y=8.1}, {x=-3.84, y=8.16}, {x=-3.72, y=8.2}, {x=-3.62, y=8.22}, {x=-3.52, y=8.26}, {x=-3.42, y=8.28}, {x=-3.32, y=8.34}, {x=-3.2, y=8.38}, {x=-3.1, y=8.46}, {x=-3, y=8.5}, {x=-3.1, y=8.6}, {x=-3.22, y=8.66}, {x=-3.34, y=8.7}, {x=-3.44, y=8.7}, {x=-3.56, y=8.7}, {x=-3.66, y=8.72}, {x=-3.78, y=8.76}, {x=-3.88, y=8.78}, {x=-3.98, y=8.8}, {x=-4.12, y=8.88}, {x=-4.24, y=8.92}, {x=-4.22, y=9.04}, {x=-4.08, y=9.1}, {x=-3.98, y=9.14}, {x=-3.88, y=9.14}, {x=-3.78, y=9.16}, {x=-3.64, y=9.2}, {x=-3.52, y=9.2}, {x=-3.42, y=9.2}, {x=-3.32, y=9.2}, {x=-3.2, y=9.24}, {x=-3.1, y=9.26}, {x=-3, y=9.26}, {x=-2.9, y=9.28}, {x=-2.8, y=9.3}, {x=-2.9, y=9.36}, {x=-3, y=9.4}, {x=-3.1, y=9.46}, {x=-3.2, y=9.52}, {x=-3.32, y=9.58}, {x=-3.42, y=9.62}, {x=-3.54, y=9.68}, {x=-3.64, y=9.7}, {x=-3.74, y=9.74}, {x=-3.76, y=9.86}, {x=-3.64, y=9.88}, {x=-3.54, y=9.9}, {x=-3.44, y=9.9}, {x=-3.32, y=9.9}, {x=-3.18, y=9.9}, {x=-3.08, y=9.9}, {x=-2.98, y=9.92}, {x=-2.88, y=9.94}, {x=-2.76, y=9.94}, {x=-2.64, y=9.94}, {x=-2.54, y=9.94}, {x=-2.44, y=9.94}, {x=-2.34, y=9.94}, {x=-2.2, y=9.94}, {x=-2.1, y=9.94}, {x=-2, y=9.94}, {x=-1.9, y=9.94}, {x=-1.76, y=9.94}, {x=-1.62, y=9.94}, {x=-1.52, y=9.94}, {x=-1.42, y=9.94}, {x=-1.28, y=9.92}, {x=-1.18, y=9.9}, {x=-1.06, y=9.9}, {x=-0.96, y=9.88}, {x=-0.82, y=9.86}, {x=-0.7, y=9.84}, {x=-0.58, y=9.82}, {x=-0.46, y=9.8}, {x=-0.36, y=9.74}, {x=-0.24, y=9.7}, {x=-0.1, y=9.64}, {x=0.08, y=9.6}, {x=0.2, y=9.58}, {x=0.32, y=9.54}, {x=0.46, y=9.52}, {x=0.56, y=9.48}, {x=0.78, y=9.44}, {x=0.9, y=9.42}, {x=1.08, y=9.4}, {x=1.2, y=9.38}, {x=1.3, y=9.36}, {x=1.44, y=9.3}, {x=1.54, y=9.28}, {x=1.66, y=9.26}, {x=1.78, y=9.2}, {x=1.94, y=9.12}, {x=2.04, y=9.1}, {x=2.14, y=9.04}, {x=2.24, y=8.98}, {x=2.36, y=8.88}, {x=2.46, y=8.86}, {x=2.56, y=8.84}, {x=2.66, y=8.8}, {x=2.78, y=8.76}, {x=2.9, y=8.72}, {x=3, y=8.7}, {x=3.1, y=8.62}, {x=3.2, y=8.56}, {x=3.32, y=8.5}, {x=3.42, y=8.4}, {x=3.52, y=8.34}, {x=3.62, y=8.28}, {x=3.74, y=8.22}, {x=3.86, y=8.14}, {x=3.94, y=8.02}, {x=4, y=7.9}, {x=4, y=7.78}, {x=4, y=7.68}, {x=4, y=7.52}, {x=3.98, y=7.36}, {x=3.96, y=7.26}, {x=3.92, y=7.12}, {x=3.86, y=6.98}, {x=3.84, y=6.88}, {x=3.82, y=6.72}, {x=3.8, y=6.6}, {x=3.78, y=6.48}, {x=3.76, y=6.38}, {x=3.72, y=6.26}, {x=3.66, y=6.14}, {x=3.62, y=6.02}, {x=3.58, y=5.88}, {x=3.48, y=5.8}, {x=3.38, y=5.8}, {x=3.28, y=5.8}, {x=3.16, y=5.78}, {x=3.06, y=5.7}, {x=2.94, y=5.58}, {x=2.88, y=5.44}, {x=2.8, y=5.3}, {x=2.76, y=5.18}, {x=2.76, y=5.06}, {x=2.88, y=5.18}, {x=2.98, y=5.26}, {x=3.08, y=5.32}, {x=3.18, y=5.34}, {x=3.3, y=5.34}, {x=3.4, y=5.34}, {x=3.48, y=5.2}, {x=3.48, y=5.1}, {x=3.46, y=4.98}, {x=3.4, y=4.86}, {x=3.34, y=4.64}, {x=3.22, y=4.54}, {x=3.12, y=4.48}, {x=3.02, y=4.44}, {x=2.92, y=4.42}, {x=2.8, y=4.46}, {x=2.66, y=4.72}}

-- the main update function that uses the takeoff and velocity controllers to fly a rough square pattern
function update()
  if not arming:is_armed() then -- reset state when disarmed
    gcs:send_text(0, "disarmed")
    stage = 0
    circle_angle = 0
    count = 0
    count_end = 0
    index = 0
  else
    pwm6 = rc:get_pwm(6)
    if pwm6 and pwm6 > 1800 then    -- check if RC6 input has moved high
      if (stage == 0) then          -- change to guided mode
        if (vehicle:set_mode(copter_guided_mode_num)) then  -- change to Guided mode
          local yaw_rad = ahrs:get_yaw()
          yaw_cos = math.cos(yaw_rad)
          yaw_sin = math.sin(yaw_rad)
          stage = stage + 1
        end
      elseif (stage == 1) then      -- Stage1: takeoff
        if (vehicle:start_takeoff(takeoff_alt_above_home)) then
          stage = stage + 1
        end
      elseif (stage == 2) then      -- Stage2: check if vehicle has reached target altitude
        local home = ahrs:get_home()
        local curr_loc = ahrs:get_position()
        if home and curr_loc then
          local vec_from_home = home:get_distance_NED(curr_loc)
          gcs:send_text(0, "alt above home: " .. tostring(math.floor(-vec_from_home:z())) .. " " .. tostring(takeoff_alt_above_home) .. " " .. tostring(vec_from_home:z()) .. " " .. tostring(math.abs(takeoff_alt_above_home + vec_from_home:z())))
          -- if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
          if (math.abs(vec_from_home:z()) >= takeoff_alt_above_home) then
            stage = stage + 1
            bottom_left_loc = curr_loc          -- record location when starting square
          end
        end
      elseif (stage == 3 ) then   -- Stage3: fly a vertical circle

        -- calculate velocity vector
        -- circle_angle = circle_angle + circle_angle_increment
        -- if (circle_angle >= 360) then
        --   stage = stage + 1
        -- end
        if (index + 1 >= #vertical_points) then
          stage = stage + 1
        else
          if (count == 0) then
            local x = vertical_points[index+2].x - vertical_points[index+1].x
            local y = vertical_points[index+2].y - vertical_points[index+1].y
            local rad
            local deg
            if (x > 0) then
              rad = math.atan(y/x)
              deg = math.deg(rad)
              deg = 180 - deg
            elseif (x < 0) then
              rad = math.atan(y/x)
              deg = math.deg(rad)
              if (y > 0) then
                deg = 0 - deg
              elseif (y < 0) then
                deg = 360 - deg
              else
                deg = 0
              end
            else
              if (y > 0) then
                deg = 90
              elseif (y < 0) then
                deg = 270
              else
                deg = 0 -- unexpected
              end
              rad = math.rad(deg)
            end
            circle_angle = deg
            count_end = math.sqrt(x*x + y*y) * 10
          end
          if (count < count_end) then
            gcs:send_text(0, "stage 3: " .. tostring(circle_angle) .. " " .. tostring(count) .. " " .. tostring(index))
            local target_vel = Vector3f()
            local vel_xy = math.cos(math.rad(circle_angle)) * circle_speed
            target_vel:x(yaw_sin * vel_xy)
            target_vel:y(yaw_cos * -vel_xy)
            target_vel:z(-math.sin(math.rad(circle_angle)) * circle_speed)

            -- send velocity request
            if not (vehicle:set_target_velocity_NED(target_vel)) then
              gcs:send_text(0, "failed to execute velocity command")
            end
            count = count + 1
          else
            index = index + 1
            count = 0
          end
        end
      elseif (stage == 4) then  -- Stage4: change to RTL mode
        vehicle:set_mode(copter_rtl_mode_num)
        stage = stage + 1
        gcs:send_text(0, "finished square, switching to RTL")
      end
    end
  end

  return update, 100
end

return update()
