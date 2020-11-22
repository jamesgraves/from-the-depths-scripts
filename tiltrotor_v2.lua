-- See comments at end.
-- Tunable parameters.
local max_v = 200 -- Max velocity.
-- Should be set a little lower than what the aircraft is actually cable of.
local max_t = 84  -- Maximum tilt for nacelles to use for forward thrust.
local max_alt_error = 10 -- Artifically increase / decrease velocity by
-- up to this much to back off the thrust angle a little.
local pitchAngleCompensation = 0.1 -- Differential tilt of fore/aft nacelles
-- to compenstate for center of mass offset while travelling at high
-- foward speeds.  It is always safe to set this to 0.0 if you are
-- seeing pitch oscillations first, and if you are still having
-- problems, adjust the PID.
local altMP = 6 -- hydro angle multiplier to get desired altitude

local max_yaw = 20 -- Max angle offset for yaw, don't set this so high
-- you are losing altitude while turning.

-- This is the approximate distance from the dediblade block to the
-- spinblock in meters.  Dediblades found within this distance of a
-- spinblock are assumed to be part of that nacelle.
local nacelle_radius = 9
local nacelle_radius_fudge = 2 -- nacelle radius can be plus or minus this.

-- An exclusion zone near the CoM for pitch and roll considerations. 
-- Nacelles close enough to the center of mass will not be used for
-- pitch / roll correction.
local nacelle_pitch_exclusion = 10
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
local FWThrustPID = NewPID(1.0, 0.01,   0.01, max_v, -1*max_v, max_t, -1*max_t)

-- not currently used.
local YawPID      = NewPID(0.5, 0.01,   0.01,   1.0, -1.0,     1.0,   -1)

-- Internal variables.
local dt = 0.025 -- 25mS == 40Hz game tick.

-- Maps the names of the manual control hydrofoils to component indexes.
local manual_controls = {}
local manual_controls_valid = false

-- Tiltrotor assemblies may be in up to two groups
-- (bow/stern, port/starboard) depending upon placement on the vehicle.
local tiltrotor_list   = {}
local last_spinner_count = 0
local tick_counter = 0
local target_altitude = 50
local target_thrust = 0

--------------------------- Program Code -------------------------------
-- Check all regular and dediblade spinblocks on the vehicle.
-- Assemble tiltrotor nacelle objects from these components.
function Check_Spinners(I)
    local spinners = I:GetSpinnerCount()
    if spinners == last_spinner_count then return end
    I:ClearLogs()
    I:Log(string.format("old spinners %d  new spinners %d",
                        last_spinner_count, spinners))
    last_spinner_count = spinners
    tiltrotor_list = {}

    -- Now find spinblock closest to each dediblade.
    --
    -- Ideally, we would just use the local position of each dediblade
    -- block relative to the center of mass to determine which quadrant
    -- it is in.  Unfortunately, this is not so simple.  The
    -- LocalPosition value for the dediblade block on a spin block
    -- assembly reports its position relative to the origin of
    -- that subassembly, not the vehicle itself.
    -- So instead we look at the world coordinates for the dediblade
    -- blocks and the spinblocks.  Since the vehicle will be in some
    -- random orientation, we must calculate the distance.
    -- Iternate through all spinblocks (dediblades and regular blocks),
    -- recording all of them in separate lists.

    for spinner = 0, spinners - 1 do
        if not I:IsSpinnerDedicatedHelispinner(spinner) then
            -- Check that zero degrees is pointed up, and the rotation
            -- axis is left-right on the vehicle.
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

    Check_Spinners(I)
    local pos = I:GetConstructPosition()

    -- tick pids
    -- local alt_target = AvoidTerrain(I, pos) + GetManualAltInput(I)
    if I:GetInput(0,5) == 1 then
        target_altitude = target_altitude + 1
    elseif I:GetInput(0,4) == 1 then
        target_altitude = target_altitude - 1
    end
    local curr_altitude = pos.y
    local alt_error = math.max(target_altitude - curr_altitude, 10)
    if alt_error > 0 then
        alt_error = math.min(alt_error, max_alt_error)
    else
        alt_error = math.max(alt_error, -1 * max_alt_error)
    end
    outputAlt,ALTPID = PIDcal(target_altitude, curr_altitude, ALTPID)

    local pitch = I:GetConstructPitch()
    if pitch > 180 then pitch = -360 + pitch end -- normalize pitch range
    outputPitch, PitchPID = PIDcal(0, pitch, PitchPID)

    local roll = I:GetConstructRoll()
    if roll > 180 then roll = -360 + roll end    -- normalize roll range
    outputRoll,RollPID = PIDcal(0,roll,RollPID)

    -- yaw
    local yaw_request = 0
    if I:GetInput(0,0) == 1 then
        yaw_request = -1
    elseif I:GetInput(0,1) == 1 then
        yaw_request = 1
    end
    local outputYawAngle = max_yaw  * yaw_request

    -- forward thrust
    target_thrust = I:GetDrive(0) * 200
    local fw_velocity = I:GetForwardsVelocityMagnitude()
    local outputThrustAngle = 0

    if curr_altitude < target_altitude then
        -- Exaggerate the velocity by the altitude error.
        if target_thrust > 0 and fw_velocity > 0 then
            fw_velocity = fw_velocity + alt_error
        elseif target_thrust < 0 and fw_velocity < 0 then
            fw_velocity = fw_velocity - alt_error
        end
    end
    -- target_thrust is signed
    outputThrustAngle, FWThrustPID = PIDcal(target_thrust, fw_velocity, FWThrustPID)
    tick_counter = tick_counter + 1
    if tick_counter % 160 == 0 then
        I:LogToHud(string.format("velocity  %.1f  thrust angle %.1f",
            fw_velocity, outputThrustAngle))
        tick_counter = 0
    end

    -- send commands
    MixAndSetDediblades(I, outputRoll, outputPitch, outputAlt)
    MixAndSetTiltrotors(I, outputThrustAngle, outputYawAngle, outputPitch)

end

--[[ x

Tiltrotor AI.

Goals:

    For large airships to provide a stable gun platform.
    Controls dediblades mounted on spinblocks.
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

New design philosophy:

Use PIDs for altitude, pitch and yaw.  Calculate the actual thrust from
each nacelle based on the current angle.

Forward thrust control input from user is just mapped to forward tilt angle.
So forward speed is just whatever you get from that tilt angle, it is
not something actively controlled for.

Don't bother with reverse, how often do you need that anyway, use the
additional range for forward tilt control.

Zero requested altitude and zero requested tilt will turn off the
dediblades, so the craft can rest on the water without using power.

Use non-linear mapping for forward tilt, to increase precision at top
speed. Not worried about going 10 m/s vs. 20 m/s.

Nominal hydro angle is -45 to +45 degrees.

    -45 to -40 degrees:  tilt = 0
    -40 to -25 degrees:  tilt = (hydro + 40) * 2 +  5   ( 5 to 35)
    -25 to 10  degrees:  tilt = (hydro + 25) * 1 + 35   (35 to 70)
    10  to 30  degrees:  tilt = (hydro - 10) / 2 + 70   (70 to 80)
    30  to 45  degrees:  tilt = (hydro - 30) / 3 + 80   (80 to 85)

When the nacelles are at a high angle, that obviously reduces downward
thrust considerably (the craft nominally does not have wings that provide
lift, but maybe I should reconsider that).

Another issue is pitch and roll authority when the nacelles are at a
high angle.  This drastically reduces the downward thrust, so it becomes
harder to maintain stable flight.

Other option:

Adjust tilt angle via PID to control pitch.
    nose down: bow nacelles tilt up, stern down
    nose up: bow nacelles tilt down, stern up
Adjust left/right dediblade power to control roll.
Adjust base dediblade power to control altitude.


--]]

--[[

Requirements:

Complex controller block.

Manual controls require 3 hydrofoils.
These should all be in a row, and from left to right are:

1. Altitude Setpoint (-45 degrees is 0 altitude, +45 degrees is max)
2. Forward Thrust Setpoint (-45 is 0 m/s, and +45 is max tilt forward,
   non-linear).
3. Yaw. (negative degrees yaw the vehicle to the left, positive to the
   right, yaw rate proportional angle TBD).
   Note that without continual user input, this will reset to 0 degrees.

--]]

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

