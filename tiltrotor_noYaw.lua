-- Tiltrotor without yaw control.  See comments at end.

-- Tunable parameters.

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
local RollPID     = NewPID(0.5, 0.0100, 0.08, 180.0, -180.0,   4.0,   -4.0)
local PitchPID    = NewPID(0.4, 0.0100, 0.08, 180.0, -180.0,   4.0,   -4.0)
local AltThrPID   = NewPID(0.2, 0.0010, 0.12,   1.0, -1.0,     2.0,   -0.5)

-- Internal variables.
local dt = 0.025 -- 25mS == 40Hz game tick.

-- Tiltrotor assemblies may be in up to two groups
-- (fore/aft, port/starboard) depending upon placement on the vehicle.
local tiltrotor_list   = {}
local last_spinner_count = 0
local tick_counter = 0
local target_altitude = 50
local target_heading = 0.0
local turning = false
local altitude_change = false

--------------------------- Program Code -------------------------------

local max=math.max
local min=math.min

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

    -- Vars to choose aft-most starboard / port pair for pitch / roll control
    local aftmost_port_index = -1
    local aftmost_port_z = 0
    local aftmost_starboard_index = -1
    local aftmost_starboard_z = 0

    -- fore-most pair
    local foremost_port_index = -1
    local foremost_port_z = 0
    local foremost_starboard_index = -1
    local foremost_starboard_z = 0

    for spinner = 0, spinners - 1 do
        if not I:IsSpinnerDedicatedHelispinner(spinner) then
            -- Check that zero degrees is pointed up, and the rotation
            -- axis is left-right on the vehicle.
            local new_tiltrotor = {}
            new_tiltrotor.spinblock_id   = spinner
            new_tiltrotor.up_dediblade   = -1    -- spinner id
            new_tiltrotor.down_dediblade = -1    -- spinner id
            new_tiltrotor.fore           = false
            new_tiltrotor.aft            = false
            new_tiltrotor.port           = false
            new_tiltrotor.starboard      = false
            local spi = I:GetSpinnerInfo(spinner)

            for dediblade = 0, spinners - 1 do
                if I:IsSpinnerDedicatedHelispinner(dediblade) then
                    local di = I:GetSpinnerInfo(dediblade)
                    local rel_pos = di.Position - spi.Position -- relative pos
                    local distance = math.sqrt( rel_pos.x * rel_pos.x
                                              + rel_pos.y * rel_pos.y
                                              + rel_pos.z * rel_pos.z )
                    if ( distance < nacelle_radius + nacelle_radius_fudge
                     and distance > nacelle_radius - nacelle_radius_fudge ) then
                        if di.LocalPosition.z >= 0 then
                            new_tiltrotor.up_dediblade = dediblade
                        else
                            new_tiltrotor.down_dediblade = dediblade
                        end
                    end
                end
            end -- for

            -- For actually deciding where on the vehicle the nacelle is,
            -- we now can use the local coordinates.
            if new_tiltrotor.down_dediblade > -1
            or new_tiltrotor.up_dediblade   > -1 then
                table.insert(tiltrotor_list, new_tiltrotor)
                I:SetSpinnerRotationAngle(new_tiltrotor.spinblock_id, 0)
                local spi_local_pos = spi.LocalPositionRelativeToCom

                local bs = "" -- fore or aft
                local ps = "" -- port or starboard
                if spi_local_pos.x > nacelle_roll_exclusion then
                    new_tiltrotor.starboard = true
                    ps = "starboard"
                    if spi_local_pos.z < aftmost_starboard_z then
                        aftmost_starboard_z     = spi_local_pos.z
                        aftmost_starboard_index = #tiltrotor_list
                    end
                    if spi_local_pos.z > foremost_starboard_z then
                        foremost_starboard_z     = spi_local_pos.z
                        foremost_starboard_index = #tiltrotor_list
                    end
                elseif spi_local_pos.x < -1 * nacelle_roll_exclusion then
                    new_tiltrotor.port = true
                    ps = "port"
                    if spi_local_pos.z < aftmost_port_z then
                        aftmost_port_z     = spi_local_pos.z
                        aftmost_port_index = #tiltrotor_list
                    end
                    if spi_local_pos.z > foremost_port_z then
                        foremost_port_z     = spi_local_pos.z
                        foremost_port_index = #tiltrotor_list
                    end
                end

                if spi_local_pos.z > nacelle_pitch_exclusion then
                    new_tiltrotor.fore = true
                    bs = "fore"
                elseif spi_local_pos.z < -1 * nacelle_pitch_exclusion then
                    new_tiltrotor.aft = true
                    bs = "aft"
                end
                I:Log(string.format("nacelle %d  pos x%d y%d z%d  up: %d down: %d   %s %s",
                   new_tiltrotor.spinblock_id,
                   spi_local_pos.x, spi_local_pos.y, spi_local_pos.z,
                   new_tiltrotor.up_dediblade, new_tiltrotor.down_dediblade,
                   ps, bs))

            end -- if
        end -- if
    end -- for

    for i, tiltrotor in ipairs(tiltrotor_list) do
        if i == aftmost_starboard_index or i == aftmost_port_index then
            tiltrotor.rollPitch = true
        else
            tiltrotor.rollPitch = false
        end
    end

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
function MixAndSetDediblades(I, main_throttle, pitch_comp, roll_comp)
    for i, tiltrotor in ipairs(tiltrotor_list) do
        local thrust = main_throttle
        if tiltrotor.fore then
            if tiltrotor.rollPitch then
                thrust = thrust - pitch_comp.mainThrottle
            else
                thrust = thrust - pitch_comp.allThrottle
            end
        elseif tiltrotor.aft then
            if tiltrotor.rollPitch then
                thrust = thrust + pitch_comp.mainThrottle
            else
                thrust = thrust + pitch_comp.allThrottle
            end
        end

        if tiltrotor.starboard then
            if tiltrotor.rollPitch then
                thrust = thrust + roll_comp.mainThrottle
            else
                thrust = thrust + roll_comp.allThrottle
            end
        elseif tiltrotor.port then
            if tiltrotor.rollPitch then
                thrust = thrust - roll_comp.mainThrottle
            else
                thrust = thrust - roll_comp.allThrottle
            end
        end

        -- Ideally, if we're clamped on one side, we should reduce
        -- the opposite side even more.
        thrust = max(-1, min(1, thrust)) -- Clamp output to [-1..1].

        if tiltrotor.up_dediblade   > -1 then
            I:SetSpinnerContinuousSpeed(tiltrotor.up_dediblade,    30 * thrust)
        end
        if tiltrotor.down_dediblade > -1 then
            I:SetSpinnerContinuousSpeed(tiltrotor.down_dediblade, -30 * thrust)
        end
   end
end

-- Set angle for all tiltrotor nacelles.
function MixAndSetNacelles(I, all_angle, altAngReduce, pitch_comp, roll_comp)
    for i, tiltrotor in ipairs(tiltrotor_list) do
        local reduce_pct = 0
        if tiltrotor.rollPitch then
            reduce_pct = max(pitch_comp.mainAngReduce, roll_comp.mainAngReduce)
        else
            reduce_pct = max(pitch_comp.allAngReduce,  roll_comp.allAngReduce)
        end
        reduce_pct = max(reduce_pct, altAngReduce)
        local angle = all_angle * (1.0 - reduce_pct)
        if tiltrotor.port then
            angle = -1 * angle
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

    -- altitude
    local pos = I:GetConstructPosition()
    local curr_altitude = pos.y
    if I:GetInput(0,4) == 1 then
        target_altitude = target_altitude + 1
        altitude_change = true
    elseif I:GetInput(0,5) == 1 then
        target_altitude = target_altitude - 1
        altitude_change = true
    elseif altitude_change == true then
        -- snap to the current altitude when the user lets go the key
        altitude_change = false
        target_altitude = curr_altitude
    end
    local throttle = 0.0
    local altAngReduce = 0.0
    throttle, AltThrPID = PIDcal(target_altitude, curr_altitude, AltThrPID)
    if throttle > 1.0 then
        altAngReduce = throttle - 1.0
        throttle = 1.0
    end

    -- Pitch Stabilization
    local pitch = I:GetConstructPitch()
    if pitch > 180 then pitch = pitch - 360 end -- normalize pitch range
    outputPitch, PitchPID = PIDcal(0, pitch, PitchPID)

    local pitchCompensation = { mainThrottle = 0.0, mainAngReduce = 0.0,
                                allThrottle  = 0.0, allAngReduce  = 0.0 }
    if outputPitch < 1.0 then
        pitchCompensation.mainThrottle = outputPitch
    else
        pitchCompensation.mainThrottle = 1.0
        if outputPitch < 2.0 then
            pitchCompensation.mainAngReduce = outputPitch - 1.0
        else
            pitchCompensation.mainAngReduce = 1.0
            if outputPitch < 3.0 then
                pitchCompensation.allThrottle = outputPitch - 2.0
            else
                pitchCompensation.allThrottle = 1.0
                pitchCompensation.allAngReduce = outputPitch - 3.0
            end
        end
    end

    -- Pitch Stabilization
    local roll = I:GetConstructRoll()
    if roll > 180 then roll = roll - 360 end    -- normalize roll range
    local outputRoll = 0.0
    outputRoll, RollPID = PIDcal(0,roll,RollPID)

    local rollCompensation = { mainThrottle = 0.0, mainAngReduce = 0.0,
                               allThrottle  = 0.0, allAngReduce  = 0.0 }
    if outputRoll < 1.0 then
        rollCompensation.mainThrottle = outputRoll
    else
        rollCompensation.mainThrottle = 1.0
        if outputRoll < 2.0 then
            rollCompensation.mainAngReduce = outputRoll - 1.0
        else
            rollCompensation.mainAngReduce = 1.0
            if outputRoll < 3.0 then
                rollCompensation.allThrottle = outputRoll - 2.0
            else
                rollCompensation.allThrottle = 1.0
                rollCompensation.allAngReduce = outputRoll - 3.0
            end
        end
    end


    -- forward / backward thrust level
    local nacelle_angle = I:GetDrive(0) * 90.0

    -- yaw not handled, need manual thrusters

    tick_counter = tick_counter + 1
    if tick_counter % 160 == 0 then
    --[[
        I:LogToHud(string.format("heading   %.1f  target heading %.1f", heading, target_heading))
    --]]
        tick_counter = 0
    end

    MixAndSetDediblades(I, throttle, pitchCompensation, rollCompensation)
    MixAndSetNacelles(I, nacelle_angle, altAngReduce,
                         pitchCompensation, rollCompensation)

end

--[[

Tiltrotor AI.

Goals:

    For large airships to provide a stable gun platform.
    Controls dediblades mounted on spinblocks.
    Altitude set by pilot.  Can stop and land on water.
    Roll and pitch stability.
    
    Forward / backward thrust via tilt fore/aft of all nacelles.
    Don't use any grossly unrealistic physics settings like the 'always
        up' thrust setting on dediblades.
    Automatically compensate for vehicle damage.

To be added:
    Terrain avoidance.
    Dodging.
    Yaw via differential tilt of starboard / port nacelles.

This code is inspired by CP75's sigma delta stabilization code:
http://www.fromthedepthsgame.com/forum/showthread.php?tid=10680&page=3
But has been almost completely re-written.

--]]

--[[

New design philosophy:

Use PIDs for altitude, pitch and yaw (future).  Calculate the actual
thrust from each nacelle based on the current angle.

Forward thrust control input from user is just mapped to forward tilt angle.
So forward speed is just whatever you get from that tilt angle, it is
not something actively controlled for.

Zero requested altitude and zero requested tilt will turn off the
dediblades, so the craft can rest on the water without using power.

When the nacelles are at a high angle, that obviously reduces downward
thrust considerably.  It is beneficial to add wings for lift for high
forward speed flight.  When thrust is at 100%, the altitude PID will
reduce tilt angle to maintain altitude.  Values less than 1.0 represent
just the thrust of the dediblades.  The value exceeding 1.0 (up to 2.0)
is the percentage reduction of the nacelle tilt.  So when the PID outputs
2.0, this means 100% thrust up, and 0% nacelle tilt.

Another issue is pitch and roll authority when the nacelles are at a
high angle.  This drastically reduces the downward thrust, so it becomes
harder to maintain stable flight.

Tiltrotor craft (with four or more nacelles) tend to be longer than they
are wide, so they are correspondingly more sensitive to roll oscillation
than to pitch oscillation.  The PIDs can be easily tuned to not oscillate,
but this may reduce the ability to do gross corrections quickly (or at all).

Nominally use just one pair of nacelles (nearest the center of mass) to
maintain roll control.  If the requested thrust exceeds 1.0 (100%) thrust,
start using all the nacelles on each side.  The value above 1.0 (to 2.0)
represents the additional thrust from the not-normally-used nacelles for
roll control, so 2.0 (200%) means 100% for the normal roll control nacelle
pair, and 100% for all the other nacelles as well.

Roll and pitch control can reduce fore/back tilt of nacelles up to 100%
as needed to increase control authority.  Ideally, these functions would
use different pairs of nacelles.

4 stages of roll control:

1. throttle on roll pair
2. nacelle angle on roll pair
3. throttle on all port/starboard
4. nacelle angle on all port / starboard

Ditto for pitch control.

We don't want the various PIDs fighting for control of the vehicle.  The
roll and pitch controls can share the same pair of nacelles. Should yaw 
be separated?

Goal is at full throttle (from user), we maintain best forward speed
while maintaining altitude.  All dediblades should be at 100% power.

Yaw control (future work).

Yaw control is... complex.  If thrust is not at maximum on outward turning
side, increase nacelle forward angle, and correspondingly increase thrust
to avoid inducing roll or loss of altitude.  On inward turning side, decrease
nacelle angle (turning more vertical), and correspondingly decrease thrust
to avoid inducing roll or loss of altitude.

Need to work out the formula to do this, so that the PIDs for roll and
altitude don't need to do a lot of work.

The user sets the heading via the yaw control.  A PID tries to maintain the
heading.

At 0-degree forward tilt, nacelles on both sides tilt, thrust is increased
to maintain altitude (so vertical thrust component remains the same), 
corresponding to actual nacelle tilt.

At 90-degree nacelle tilt (directly forwards or backwards), yaw is
accomplished soley by differntial thrust, no adjustment needed for nacelle
tilt, and assuming the nacelles are near the center of mass, there should be
minimal effect on pitch / roll.

Unlike roll / pitch, only one stage of yaw control?

Differential thrust (starboard / port) is math.sin() of nacelle tilt.
Differential tilt   (starboard / port) is math.cos() of nacelle tilt.

Use only one pair of nacelles for nominal yaw control?

At max thrust, all that can be done for yaw is to reduce tilt on
the side the turn is going towards, and reduce thrust on that side to
maintain vertical thrust and not incude roll.


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

