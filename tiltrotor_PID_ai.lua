--[[ x

Tiltrotor AI.

Goals:

    For large airships to provide a stable gun platform.  Controls dediblades mounted on spinblocks.
    Altitude set by pilot.  Can stop and land on water.
    Roll and pitch stability.
    Yaw via differential tilt of starboard / port nacelles.
    Forward / backward thrust via tilt fore/aft of all nacelles.
    Don't use any grossly unrealistic physics settings like the 'always
        up' thrust setting on dediblades.
    Automatically compensate for vehicle damage.

To be added:
    Terrain avoidance.
    Dodging.


This code is inspired by CP75's sigma delta stabilization code:
http://www.fromthedepthsgame.com/forum/showthread.php?tid=10680&page=3
But has been almost completely re-written.

--]]

--[[

Requirements:

Complex controller block.

Manual controls require 3 hydrofoils.  These should all be in a row, and from
left to right are:
1. Altitude Setpoint (-45 degrees is 0 altitude, +45 degrees is 360m)
2. Forward Thrust Setpoint (-45 is -90 m/s backwards, 0 degrees is 0 m/s, and +45 is 90 m/s forward)
3. Yaw. (negative degrees yaw the vehicle to the left, positive to the right, yaw rate proportional angle TBD).
    Note that without continual user input, this will reset to 0 degrees.

--]]

--
--
--
-- Tunable parameters.
--
--
--
local max_v = 100 -- Max velocity. Should be set a little lower than what the aircraft is actually cable of.
local max_t = 84  -- Maximum tilt for nacelles to use for forward thrust.
local thrustMP = (max_v / 45) * 1.5
local max_alt_error = 10 -- Artifically increase / decrease velocity by up to this much to back off the thrust angle a little.
local pitchAngleCompensation = 0.1 -- Differential tilt of fore/aft nacelles to compenstate for center of mass
                                   -- offset while travelling at high foward speeds.
                                   -- It is always safe to set this to 0.0 if you are seeing pitch oscillations
                                   -- first, and if you are still having problems, adjust the PID.
local altMP = 6 -- hydro angle multiplier to get desired altitude

local max_yaw = 20 -- Max angle offset for yaw, don't set this so high you are losing altitude while turning.

local vehicleName = "tiltrotor"

--[[

A nacelle (tiltrotor assembly) is one or two (up and down
facing) dediblades mounted on a spinblock subobject.

View from front of vehicle:

    vehicle    -------d--------
      ========s========
               -------d--------

    s is a normal spin block
    d is a dediblade
    - is a blade block
    = are 4m beams
--]]

-- This is the approximate distance from the dediblade block to the
-- spinblock in meters.
-- Dediblades found within this distance of a spinblock are assumed to be part
-- of that nacelle.
local nacelle_radius = 9
local nacelle_radius_fudge = 2 -- nacelle radius can be plus or minus this.
-- An exclusion zone near the CoM for pitch and roll considerations.  Nacelles
-- close enough to the center of mass will not be used for pitch / roll correction.
local nacelle_pitch_exclusion = 10 -- Airships tend to be longer than they are wide.
local nacelle_roll_exclusion = 5

function NewPID(Kp,Ki,Kd,IMax,IMin,OutMax,OutMin)
   local PID = {}
   PID.Kp = Kp        -- Proportional constant
   PID.Ki = Ki        -- Integral constant
   PID.Kd = Kd        -- Derivative constant
   PID.IMax = IMax
   PID.IMin = IMin
   PID.OutMax = OutMax
   PID.OutMin = OutMin
   PID.integral = 0
   PID.pre_error = 0
   return PID
end

--          set PIDs      (  P,   I,     D,   IMax,  IMin,    OutMax, OutMin)
local RollPID     = NewPID(0.4, 0.0100, 0.08, 180.0, -180.0,   1.0,   -1.0)
local PitchPID    = NewPID(0.4, 0.0100, 0.08, 180.0, -180.0,   1.0,   -1.0)
local ALTPID      = NewPID(0.2, 0.0010, 0.12,   1.0, -1.0,     1.0,   -0.5)
local FWThrustPID = NewPID(1.0, 0.01,   0.01, max_v,  0.0,     0.0,   -1*max_t)
local YawPID      = NewPID(0.5, 0.01,   0.01,   1.0, -1.0,     1.0,   -1) -- not currently used.

--
--
--
-- Internal variables.  Don't modify these.
--
--
--

local dt = 0.025 -- 25mS == 40Hz game tick.

-- Maps the names of the manual control hydrofoils to component indexes.
local manual_controls = {}
local manual_controls_valid = false

-- Tiltrotor assemblies may be in up to two groups
-- (bow/stern, port/starboard) depending upon placement on the vehicle.
local tiltrotor_list   = {}
local last_spinner_count = 0

local tick_counter = 0
-- #############################################################################


-- Check all regular and dediblade spinblocks on the vehicle.
-- Assemble tiltrotor nacelle objects from these components.
function Check_Spinners(I)
    local spinners = I:GetSpinnerCount()
    if spinners == last_spinner_count then return end
    I:ClearLogs()
    I:Log(string.format("old spinners %d  new spinners %d", last_spinner_count, spinners))
    last_spinner_count = spinners
    tiltrotor_list = {}

    -- Now find spinblock closest to each dediblade.
    --
    -- Ideally, we would just use the local position of each dediblade block relative
    -- to the center of mass to determine which quadrant it is in.
    -- Unfortunately, this is not so simple.  The LocalPosition value for the dediblade
    -- block on a spin block assembly reports its position relative to the origin of
    -- that subassembly, not the vehicle itself.
    -- So instead we look at the world coordinates for the dediblade
    -- blocks and the spinblocks.  Since the vehicle will be in some random orientation,
    -- we must calculate the distance.

    -- Iternate through all spinblocks (dediblades and regular blocks), recording
    -- all of them in separate lists.

    for spinner = 0, spinners - 1 do
        if not I:IsSpinnerDedicatedHelispinner(spinner) then
            -- Check that zero degrees is pointed up, and the rotation axis is left-right on the vehicle.
            --
            local new_tiltrotor = {}
            new_tiltrotor.spinblock_id = spinner
            new_tiltrotor.up_dediblade   = -1    -- spinner id
            new_tiltrotor.down_dediblade = -1    -- spinner id
            new_tiltrotor.bow       = false
            new_tiltrotor.stern     = false
            new_tiltrotor.port      = false
            new_tiltrotor.starboard = false
            local spi = I:GetSpinnerInfo(spinner)
            -- I:Log(string.format("regular spinner %d,   world  x %d, y %d, z %d", spinner,
            --    spi.Position.x, spi.Position.y, spi.Position.z))

            for dediblade = 0, spinners - 1 do
                if I:IsSpinnerDedicatedHelispinner(dediblade) then
                    local di = I:GetSpinnerInfo(dediblade)
                    local rel_pos = di.Position - spi.Position -- relative position
                    local distance = math.sqrt( rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y + rel_pos.z * rel_pos.z)
                    if ( distance < nacelle_radius + nacelle_radius_fudge and distance > nacelle_radius - nacelle_radius_fudge) then
                        -- I:Log(string.format("dediblade %d,   world  x %d, y %d, z %d", dediblade,
                        --     di.Position.x, di.Position.y, di.Position.z))
                        -- I:Log(string.format("dediblade %d,   local  x %d, y %d, z %d", dediblade,
                        --    di.LocalPosition.x, di.LocalPosition.y, di.LocalPosition.z))
                        if di.LocalPosition.z >= 0 then
                            new_tiltrotor.up_dediblade = dediblade
                        else
                            new_tiltrotor.down_dediblade = dediblade
                        end
                    end
                end
            end -- for

            -- For actually deciding where on the vehicle the nacelle is, we now
            -- can use the local coordinates.
            if new_tiltrotor.down_dediblade > -1 or new_tiltrotor.up_dediblade > -1 then
                table.insert(tiltrotor_list, new_tiltrotor)
                I:SetSpinnerRotationAngle(new_tiltrotor.spinblock_id, 0)
                local spi_local_pos = spi.LocalPositionRelativeToCom

                local bs = ""
                local ps = ""
                if spi_local_pos.x > nacelle_roll_exclusion then
                    new_tiltrotor.starboard = true
                    ps = "starboard"
                elseif spi_local_pos.x < -1 * nacelle_roll_exclusion then
                    new_tiltrotor.port = true
                    ps = "port"
                end

                if spi_local_pos.z > nacelle_pitch_exclusion then
                    new_tiltrotor.bow = true
                    bs = "bow"
                elseif spi_local_pos.z < -1 * nacelle_pitch_exclusion then
                    new_tiltrotor.stern = true
                    bs = "stern"
                end
                I:Log(string.format("tiltrotor %d  position x%d y%d z%d   up: %d  down: %d   %s %s",
                   new_tiltrotor.spinblock_id,
                   spi_local_pos.x, spi_local_pos.y, spi_local_pos.z,
                   new_tiltrotor.up_dediblade, new_tiltrotor.down_dediblade,
                   ps, bs))

            end -- if
        end -- if
    end -- for
end -- Check_Spinners()


-- Hardcoded for 3 manual controls (see above).
function Check_Manual_Controls(I)
    if I:Component_GetCount(8) == 3 then
        if manual_controls_valid then
            return
        else
            -- Scan for and assign manual control hydrofoils based on
            -- the location on the vehicle.
            local max_x = -10000000
            local min_x =  10000000
            for c = 0, 2 do
                local hf_info = I:Component_GetBlockInfo(8, c)
                max_x = math.max(max_x, hf_info.LocalPosition.x)
                min_x = math.min(min_x, hf_info.LocalPosition.x)
            end
            for c = 0, 2 do
                local hf_info = I:Component_GetBlockInfo(8, c)
                if hf_info.LocalPosition.x == min_x then
                    manual_controls.altitude_set_point = c
                elseif hf_info.LocalPosition.x == max_x then
                    manual_controls.yaw_request = c
                else
                    manual_controls.forward_thrust_set_point = c
                end
            end
            I:Log("manual controls OK")
            manual_controls_valid = true
        end
    else
        I:Log("manual controls invalid")
        manual_controls_valid = false
        -- Can't do anything else at this point until
        -- the controls have been repaired.
    end
end


function PIDcal(SetPoint,ProcessVariable,PID)
   -- P
   local err = SetPoint - ProcessVariable

   -- I
   PID.integral = PID.integral + (err * dt)
   if (PID.integral > PID.IMax) then PID.integral = PID.IMax end
   if (PID.integral < PID.IMin) then PID.integral = PID.IMin end

   -- D
   local derivative = (err - PID.pre_error)/dt

   -- combine and clamp
   local output = (PID.Kp*err) + (PID.Ki*PID.integral) + (PID.Kd*derivative)
   if (output > PID.OutMax) then output = PID.OutMax end
   if (output < PID.OutMin) then output = PID.OutMin end

   --Update err
   PID.pre_error = err

   -- done
   return output,PID
end


-- Set thrust for tiltrotor nacelles.
function MixAndSetDediblades(I, roll_thr, pitch_thr, alt_thr)
    for i, tiltrotor in ipairs(tiltrotor_list) do
        local thrust = alt_thr
        if     tiltrotor.bow       then thrust = thrust - pitch_thr
        elseif tiltrotor.stern     then thrust = thrust + pitch_thr end
        if     tiltrotor.starboard then thrust = thrust + roll_thr
        elseif tiltrotor.port      then thrust = thrust - roll_thr  end
        -- clamp output
        if thrust >  1 then
            thrust =  1
        elseif thrust < -1 then
            thrust = -1
        end
        if tiltrotor.up_dediblade   > -1 then
            I:SetSpinnerContinuousSpeed(tiltrotor.up_dediblade,    30 * thrust)
        end
        if tiltrotor.down_dediblade > -1 then
            I:SetSpinnerContinuousSpeed(tiltrotor.down_dediblade, -30 * thrust)
        end
   end
end

-- Set angle for tiltrotor nacelles.
function MixAndSetTiltrotors(I, thrust_angle, yaw_angle, outputPitch)
    -- The problem with the pitch angle compensation is that the
    -- effect is highly non-linear.  It makes a big difference at high (close to 90)
    -- tiltrotor angles, but has little effect at modest angles.
    local pitch_compensation = outputPitch * pitchAngleCompensation
    for i, tiltrotor in ipairs(tiltrotor_list) do
        local angle = -1 * yaw_angle
        if tiltrotor.starboard then
            angle = angle + thrust_angle
            if tiltrotor.bow then
                angle = angle - pitch_compensation
            elseif tiltrotor.stern then
                angle = angle + pitch_compensation
            end
        elseif tiltrotor.port then
            angle = angle - thrust_angle
            if tiltrotor.bow then
                angle = angle + pitch_compensation
            elseif tiltrotor.stern then
                angle = angle - pitch_compensation
            end
        end
        -- Ensure that even with pitch compensation and yaw that we don't exceed max tilt.
        if angle < -1 * max_t then
            angle = -1 * max_t
        elseif angle > max_t then
            angle = max_t
        end
        I:SetSpinnerRotationAngle(tiltrotor.spinblock_id, angle)
    end
end


function GetManualAltInput(I)
    return (I:Component_GetFloatLogic(8, manual_controls.altitude_set_point) + 45) * altMP
end

function GetManualThrustInput(I)
    return (I:Component_GetFloatLogic(8, manual_controls.forward_thrust_set_point)) * thrustMP
end

function GetManualYawInput(I)
    local curr_hydro_angle = I:Component_GetFloatLogic(8, manual_controls.yaw_request)
    if curr_hydro_angle >= 0.9 then
        curr_hydro_angle = curr_hydro_angle - 0.9
        I:Component_SetFloatLogic(8, manual_controls.yaw_request, curr_hydro_angle)
        return 1
    elseif curr_hydro_angle <= -0.9 then
        curr_hydro_angle = curr_hydro_angle + 0.9
        I:Component_SetFloatLogic(8, manual_controls.yaw_request, curr_hydro_angle)
        return -1
    else
        I:Component_SetFloatLogic(8, manual_controls.yaw_request, 0)
        return 0
    end
end

function Shutdown(I)
    for i, tiltrotor in ipairs(tiltrotor_list) do
        if tiltrotor.down_dediblade > -1 then
            I:SetSpinnerContinuousSpeed(tiltrotor.down_dediblade, 0)
        end
        if tiltrotor.up_dediblade > -1 then
            I:SetSpinnerContinuousSpeed(tiltrotor.up_dediblade, 0)
        end
        I:SetSpinnerRotationAngle(tiltrotor.spinblock_id, 0)
    end
end

function Update(I)

    if I:IsDocked() then
        Shutdown(I)
        return
    end

    Check_Manual_Controls(I)
    Check_Spinners(I)
    local pos = I:GetConstructPosition()

    -- tick pids
    -- local alt_target = AvoidTerrain(I, pos) + GetManualAltInput(I)
    local alt_target = GetManualAltInput(I)
    local curr_altitude = pos.y
    local alt_error = math.max(alt_target - curr_altitude, 10)
    if alt_error > 0 then
        alt_error = math.min(alt_error, max_alt_error)
    else
        alt_error = math.max(alt_error, -1 * max_alt_error)
    end
    outputAlt,ALTPID = PIDcal(alt_target, curr_altitude, ALTPID)

    local pitch = I:GetConstructPitch()
    if pitch > 180 then pitch = -360 + pitch end -- set pitch range to +/- 180 degrees
    outputPitch, PitchPID = PIDcal(0, pitch, PitchPID)

    local roll = I:GetConstructRoll()
    if roll > 180 then roll = -360 + roll end    -- set roll range to +/- 180 degrees
    outputRoll,RollPID = PIDcal(0,roll,RollPID)

    -- yaw
    local yaw_request = GetManualYawInput(I)
    local outputYawAngle = max_yaw  * yaw_request

    -- forward thrust
    local thrust_target = GetManualThrustInput(I)
    local fw_velocity = I:GetForwardsVelocityMagnitude()
    local outputThrustAngle = 0

    if (thrust_target <= thrustMP and thrust_target >= -1 * thrustMP) then
        outputThrustAngle = 0 -- dead zone
    else
        if curr_altitude < alt_target then
            -- Exaggerate the velocity by the altitude error.
            if thrust_target > 0 and fw_velocity > 0 then
                fw_velocity = fw_velocity + alt_error
            elseif thrust_target < 0 and fw_velocity < 0 then
                fw_velocity = fw_velocity - alt_error
            end
        end
        -- thrust_target is signed
        outputThrustAngle, FWThrustPID = PIDcal(math.abs(thrust_target), math.abs(fw_velocity), FWThrustPID)
        if thrust_target < 0 then
            outputThrustAngle = outputThrustAngle * -1
        end
    end

    -- send commands
    MixAndSetDediblades(I, outputRoll, outputPitch, outputAlt)
    MixAndSetTiltrotors(I, outputThrustAngle, outputYawAngle, outputPitch)

    tick_counter = tick_counter + 1
    if tick_counter % 40 == 0 then
        I:LogToHud(string.format("tt %.0f  vel + err %.1f  OTA %.1f   OYA %.1f   tar alt %d",
            thrust_target, fw_velocity, outputThrustAngle, outputYawAngle, alt_target))
        tick_counter = 0
    end
end
