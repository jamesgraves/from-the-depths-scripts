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

local max_yaw = 30 -- Max angle offset for yaw, don't set this so high
-- you are losing a lot of altitude while turning.

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
local RollPID     = NewPID(0.1, 0.0100, 0.05, 180.0, -180.0,   1.0,   -1.0)
local PitchPID    = NewPID(0.4, 0.0100, 0.08, 180.0, -180.0,   1.0,   -1.0)
local AltThrPID   = NewPID(0.2, 0.0010, 0.12,   1.0, -1.0,     2.0,   -0.5)
local YawPID      = NewPID(0.1, 0.0100, 0.05, 180.0, -180.0,   1.0,   -1.0)

-- Internal variables.
local dt = 0.025 -- 25mS == 40Hz game tick.

-- Tiltrotor assemblies may be in up to two groups
-- (bow/stern, port/starboard) depending upon placement on the vehicle.
local tiltrotor_list   = {}
local last_spinner_count = 0
local tick_counter = 0
local target_altitude = 50
local target_heading = 0.0
local turning = false

function normalize_heading(heading)
    if heading > 360  then heading = heading - 360 end
    if heading < -360 then heading = heading + 360 end
    return heading
end

--------------------------- Program Code -------------------------------
-- Check all regular and dediblade spinblocks on the vehicle.
-- Assemble tiltrotor nacelle objects from these components.
function Check_Spinners(I)
    local spinners = I:GetSpinnerCount()
    if spinners == last_spinner_count then return end
    target_heading = I:GetConstructYaw() -- maintain current heading
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

-- Set angle for all tiltrotor nacelles.
function MixAndSetNacelles(I, thrust_angle, yaw_angle, outputPitch)
    -- The problem with the pitch angle compensation is that the
    -- effect is highly non-linear.  It makes a big difference at high
    -- (close to 90) tiltrotor angles, but has little effect at modest angles.
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
        -- Ensure that even with pitch compensation and yaw that we
        -- don't exceed max tilt.
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

    -- local alt_target = AvoidTerrain(I, pos) + GetManualAltInput(I)

    -- altitude
    if I:GetInput(0,4) == 1 then
        target_altitude = target_altitude + 1
    elseif I:GetInput(0,5) == 1 then
        target_altitude = target_altitude - 1
    end
    local pos = I:GetConstructPosition()
    local curr_altitude = pos.y
    local alt_error = math.max(target_altitude - curr_altitude, 10)
    if alt_error > 0 then
        alt_error = math.min(alt_error, max_alt_error)
    else
        alt_error = math.max(alt_error, -1 * max_alt_error)
    end
    local throttle = 0.0
    local tilt_reduction = 0.0
    throttle, AltThrPID = PIDcal(target_altitude, curr_altitude, AltThrPID)
    if throttle > 1.0 then
        tilt_reduction = throttle - 1.0
        throttle = 1.0
    end

    -- Pitch & Roll Stabilization
    local pitch = I:GetConstructPitch()
    if pitch > 180 then pitch = pitch - 360 end -- normalize pitch range
    outputPitch, PitchPID = PIDcal(0, pitch, PitchPID)

    local roll = I:GetConstructRoll()
    if roll > 180 then roll = roll - 360 end    -- normalize roll range
    local outputRollYaw = 0.0
    outputRollYaw, RollPID = PIDcal(0,roll,RollPID)

    -- forward / backward thrust level
    local nacelle_angle = I:GetDrive(0) * 90.0
    nacelle_angle = nacelle_angle * (1.0 - tilt_reduction)

    -- yaw
    local heading = I:GetConstructYaw()
    if I:GetInput(0,0) == 1 then
        target_heading = normalize_heading(target_heading - 0.5)
        turning = true
    elseif I:GetInput(0,1) == 1 then
        target_heading = normalize_heading(target_heading + 0.5)
        turning = true
    elseif turning == true then
        turning = false
        target_heading = heading -- snap to the heading when the user lets go the key
    end
    local heading_diff = heading - target_heading
    if heading_diff < -180 then heading_diff = heading_diff + 360 end
    if heading_diff >  180 then heading_diff = heading_diff - 360 end
    local outputYaw, YawPID = PIDcal(0, heading_diff, YawPID)
    local differentialYawAngle = outputYaw * max_yaw

    if I:GetDrive(0) == 0 and target_altitude < 0.0 then
        throttle = 0.0
    end

    tick_counter = tick_counter + 1
    if tick_counter % 160 == 0 then
        I:LogToHud(string.format("heading   %.1f  target heading %.1f", heading, target_heading))
        I:LogToHud(string.format("heading diff   %.1f  outputYaw %.1f", heading_diff, outputYaw))
        tick_counter = 0
    end



    -- send commands
    MixAndSetDediblades(I, outputRollYaw, outputPitch, throttle)
    MixAndSetNacelles(I, nacelle_angle, differentialYawAngle, outputPitch)

end

--[[

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

Zero requested altitude and zero requested tilt will turn off the
dediblades, so the craft can rest on the water without using power.

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

Settings:
    Spinner Data Tab:
        Continuous
        Motor Drive:                10
        Upwards Force Fraction:      0
        Advanced Controller Insta-spin factor: 0
    Spin rate control Tab:
        Traditional Movement:
            Forward/Backward:       off
            Yaw:                    off
            Pitch:                  off
            Roll:                   off
        Drive Factors
            Main Positive:           0

--]]

