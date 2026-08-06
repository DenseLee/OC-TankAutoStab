-- Complete remote-aim tank stabilizer for CC:VS + VS:Clockwork.
--
-- Put this computer on the separate turret ship.
-- It holds the turret's world-space orientation, while remote analogue
-- controller signals move the held target.
--
-- OUTPUT WIRING
--   computer left  -> horizontal Redstone Resistor (speed)
--   computer right -> vertical Redstone Resistor (speed)
--   relay left     -> horizontal gearshift (direction)
--   relay right    -> vertical gearshift (direction)
--
-- REMOTE AIM INPUT WIRING
--   computer top   <- horizontal +X signal
--   computer back  <- horizontal -X signal
--   relay top      <- elevation +Y signal
--   relay bottom   <- elevation -Y signal
-- Each directional source is analogue redstone 0..15. Idle must be 0.

local CONFIG = {
  horizontalOutputSide = "left",
  verticalOutputSide = "right",
  horizontalGearshiftSide = "left",
  verticalGearshiftSide = "right",
  horizontalReverseOnPositiveError = true,
  verticalReverseOnPositiveError = true,

  fullSpeedAtDegrees = 45,
  deadzoneDegrees = 0.2,
  proportionalGain = 1,
  derivativeGainSeconds = 0.0005,
  resistorCooldownTicks = 1,
  directionSettleTicks = 2,
  aimReleaseSettleTicks = 2,
  minimumRedstoneResistorLevel = 0,

  -- Remote controller target movement. 90 means full stick moves the saved
  -- aim point by 90 degrees per second.
  aimDegreesPerSecond = 36,
  -- Continuous remote aim curve. Input 1 starts at the first usable speed
  -- step; input 15 is full speed. Exponent > 1 makes fine input slow and
  -- progressively accelerates toward full stick.
  manualAimCurveExponent = 2,
  manualMinimumSpeedFraction = 1 / 15,
  aimInputDeadzone = 0.05,
  invertHorizontalAim = true,
  invertVerticalAim = true,
  showDebug = true,
}

local TICK_SECONDS = 0.05
local function clamp(v, low, high) return math.max(low, math.min(high, v)) end
local function degrees(v) return v * 180 / math.pi end

local function multiply(a, b)
  return {
    x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  }
end

local function normalize(q)
  local length = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
  assert(length > 0, "Invalid zero-length quaternion")
  return { x=q.x/length, y=q.y/length, z=q.z/length, w=q.w/length }
end

local function inverse(q)
  local lengthSquared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
  assert(lengthSquared > 0, "Invalid zero-length quaternion")
  return { x=-q.x/lengthSquared, y=-q.y/lengthSquared,
    z=-q.z/lengthSquared, w=q.w/lengthSquared }
end

local function axisAngle(x, y, z, angle)
  local half = angle / 2
  local sine = math.sin(half)
  return { x=x*sine, y=y*sine, z=z*sine, w=math.cos(half) }
end

local function asQuaternion(value)
  assert(type(value) == "table", "ship.getQuaternion() did not return a table")
  local x, y, z, w = value.x or value[1], value.y or value[2],
    value.z or value[3], value.w or value[4]
  if value.v then
    x, y, z, w = value.v.x or value.v[1], value.v.y or value.v[2],
      value.v.z or value.v[3], value.a
  end
  assert(type(x)=="number" and type(y)=="number" and type(z)=="number"
    and type(w)=="number", "Unknown quaternion format")
  return { x=x, y=y, z=z, w=w }
end

local function getYawPitchError(target, current)
  current = current or asQuaternion(ship.getQuaternion())
  local q = normalize(multiply(inverse(current), target))
  local pitch = math.asin(clamp(2 * (q.x*q.w - q.y*q.z), -1, 1))
  local yaw = math.atan(2 * (q.x*q.z + q.y*q.w),
    1 - 2 * (q.x*q.x + q.y*q.y))
  return yaw, pitch
end

local function manualLeadRadians(rawLevel)
  if rawLevel <= 0 then return 0 end
  local input = clamp(rawLevel / 15, 0, 1)
  local curved = input ^ CONFIG.manualAimCurveExponent
  local fraction = CONFIG.manualMinimumSpeedFraction
    + (1 - CONFIG.manualMinimumSpeedFraction) * curved
  return math.rad(CONFIG.fullSpeedAtDegrees * fraction)
end

local function readSignedAxis(positiveSide, negativeSide, reader)
  -- Separate wires encode direction. If both are present, they cancel.
  local positive = reader(positiveSide)
  local negative = reader(negativeSide)
  return clamp((positive - negative) / 15, -1, 1), positive, negative
end

local function applyAimDeadzone(value, inverted)
  if inverted then value = -value end
  if math.abs(value) <= CONFIG.aimInputDeadzone then return 0 end
  local sign = value < 0 and -1 or 1
  return sign * (math.abs(value) - CONFIG.aimInputDeadzone)
    / (1 - CONFIG.aimInputDeadzone)
end

local function updateTarget(target, relay)
  local horizontal, right, left = readSignedAxis("top", "back",
    redstone.getAnalogInput)
  local vertical, up, down = readSignedAxis("top", "bottom", function(side)
      return relay.getAnalogInput(side)
    end)
  horizontal = applyAimDeadzone(horizontal, CONFIG.invertHorizontalAim)
  vertical = applyAimDeadzone(vertical, CONFIG.invertVerticalAim)

  if horizontal ~= 0 or vertical ~= 0 then
    local amount = math.rad(CONFIG.aimDegreesPerSecond * TICK_SECONDS)
    -- Traverse around the world Y axis. Elevation is local to the aimed
    -- turret orientation, so it remains intuitive after traversing.
    local yaw = axisAngle(0, 1, 0, horizontal * amount)
    local pitch = axisAngle(1, 0, 0, vertical * amount)
    target = normalize(multiply(yaw, multiply(target, pitch)))

    -- Keep a virtual point ahead of the turret in the direction the player
    -- is commanding. This preserves world-space stabilization, but prevents
    -- the turret from catching a slow moving target and being told to reverse.
    local current = asQuaternion(ship.getQuaternion())
    local yawError, pitchError = getYawPitchError(target, current)
    if horizontal ~= 0 then
      local lead = manualLeadRadians(math.max(right, left))
      if horizontal * yawError < lead then
        local wantedError = (horizontal < 0 and -lead or lead)
        target = normalize(multiply(axisAngle(0, 1, 0, wantedError - yawError), target))
        yawError, pitchError = getYawPitchError(target, current)
      end
    end
    if vertical ~= 0 then
      local lead = manualLeadRadians(math.max(up, down))
      if vertical * pitchError < lead then
        local wantedError = (vertical < 0 and -lead or lead)
        local localTarget = multiply(inverse(current), target)
        local adjustment = axisAngle(1, 0, 0, wantedError - pitchError)
        target = normalize(multiply(current, multiply(adjustment, localTarget)))
      end
    end
  end
  return target, horizontal, vertical, right, left, up, down,
    horizontal ~= 0 or vertical ~= 0
end

local function resistorLevel(command)
  if command <= math.rad(CONFIG.deadzoneDegrees) then return 15 end
  local fraction = clamp(command / math.rad(CONFIG.fullSpeedAtDegrees), 0, 1)
  local level = 15 - math.floor(fraction * 15 + 0.5)
  return math.max(CONFIG.minimumRedstoneResistorLevel, level)
end

local function pdCommand(error, previous)
  if math.abs(error) <= math.rad(CONFIG.deadzoneDegrees) then return 0 end
  if not previous then return CONFIG.proportionalGain * math.abs(error) end
  local derivative = (error - previous) / TICK_SECONDS
  local directional = (error < 0 and -1 or 1) * derivative
  return math.max(0, CONFIG.proportionalGain * math.abs(error)
    + CONFIG.derivativeGainSeconds * directional)
end

local function setResistor(state, side, level)
  if state.cooldown > 0 then
    state.cooldown = state.cooldown - 1
  elseif state.level ~= level then
    redstone.setAnalogOutput(side, level)
    state.level, state.cooldown = level, CONFIG.resistorCooldownTicks
  end
  return state.level or 15
end

local function forceResistorStop(state, side)
  -- A reversal must never wait for the normal output cooldown. Stop the
  -- transmitted rotation immediately, then allow the gearshift to change.
  if state.level ~= 15 then redstone.setAnalogOutput(side, 15) end
  state.level = 15
  state.cooldown = CONFIG.resistorCooldownTicks
  return 15
end

local function writeAxis(relay, state, resistorSide, gearshiftSide, error,
    command, reverseOnPositive)
  local inDeadzone = math.abs(error) <= math.rad(CONFIG.deadzoneDegrees)
  local desiredReverse = error > 0
  if not reverseOnPositive then desiredReverse = not desiredReverse end
  if inDeadzone then desiredReverse = false end

  if state.switching then
    forceResistorStop(state, resistorSide)
    if state.phase == "change" then
      -- The immediately preceding control tick had the resistor at 15.
      relay.setOutput(gearshiftSide, state.nextReverse)
      state.reverse = state.nextReverse
      state.phase = "settle"
      state.settleTicks = CONFIG.directionSettleTicks
    else
      state.settleTicks = state.settleTicks - 1
      if state.settleTicks <= 0 then state.switching = false end
    end
    return 15, state.reverse
  end

  if desiredReverse ~= state.reverse then
    -- Step 1: stop transmission now. Step 2 happens next tick; it changes
    -- the gearshift. Step 3 holds stopped for directionSettleTicks ticks.
    state.switching = true
    state.phase = "change"
    state.nextReverse = desiredReverse
    forceResistorStop(state, resistorSide)
    return 15, state.reverse
  end

  return setResistor(state, resistorSide, resistorLevel(command)), state.reverse
end

local function stop(relay)
  redstone.setAnalogOutput(CONFIG.horizontalOutputSide, 15)
  redstone.setAnalogOutput(CONFIG.verticalOutputSide, 15)
  if relay then
    relay.setOutput(CONFIG.horizontalGearshiftSide, false)
    relay.setOutput(CONFIG.verticalGearshiftSide, false)
  end
end

local function draw(yaw, pitch, hLevel, vLevel, hReverse, vReverse,
    hAim, vAim, right, left, up, down)
  if not CONFIG.showDebug then return end
  term.clear(); term.setCursorPos(1, 1)
  print("CompleteStab - remote world hold")
  print(("Aim X:%+.2f Y:%+.2f"):format(hAim, vAim))
  print(("Raw X +%2d -%2d | Y +%2d -%2d"):format(right, left, up, down))
  print(("Yaw   error: %7.2f deg"):format(degrees(yaw)))
  print(("Pitch error: %7.2f deg"):format(degrees(pitch)))
  print("Horiz resistor: "..hLevel.."/15  gear: "..(hReverse and "reverse" or "normal"))
  print("Vert  resistor: "..vLevel.."/15  gear: "..(vReverse and "reverse" or "normal"))
  print("Ctrl+T = stop")
end

local function run()
  assert(type(ship)=="table" and type(ship.getQuaternion)=="function",
    "No CC:VS ship API. Put this computer on the turret ship.")
  local relay = peripheral.find("redstone_relay")
  assert(relay, "No redstone_relay on the wired modem network.")
  -- Establish a known, unloaded direction state before control begins.
  stop(relay)

  local target = asQuaternion(ship.getQuaternion())
  local previousYaw, previousPitch = nil, nil
  local wasAiming = false
  local releaseSettleTicks = 0
  local horizontalState = { level=15, cooldown=0, reverse=false, switching=false }
  local verticalState = { level=15, cooldown=0, reverse=false, switching=false }
  while true do
    local hAim, vAim, right, left, up, down, aiming
    target, hAim, vAim, right, left, up, down, aiming = updateTarget(target, relay)
    if aiming and releaseSettleTicks > 0 then
      -- Player resumed input before the release settle completed.
      releaseSettleTicks = 0
      target = asQuaternion(ship.getQuaternion())
      previousYaw, previousPitch = nil, nil
    end
    if wasAiming and not aiming then
      -- Stop immediately and let any last mechanical movement settle before
      -- recording the final held direction.
      releaseSettleTicks = CONFIG.aimReleaseSettleTicks
      horizontalState.switching = false
      verticalState.switching = false
      previousYaw, previousPitch = nil, nil
    end

    if releaseSettleTicks > 0 then
      forceResistorStop(horizontalState, CONFIG.horizontalOutputSide)
      forceResistorStop(verticalState, CONFIG.verticalOutputSide)
      releaseSettleTicks = releaseSettleTicks - 1
      if releaseSettleTicks == 0 then
        target = asQuaternion(ship.getQuaternion())
      end
      draw(0, 0, 15, 15, horizontalState.reverse, verticalState.reverse,
        hAim, vAim, right, left, up, down)
    else
      local yaw, pitch = getYawPitchError(target)
      local hLevel, hReverse = writeAxis(relay, horizontalState,
        CONFIG.horizontalOutputSide, CONFIG.horizontalGearshiftSide, yaw,
        pdCommand(yaw, previousYaw), CONFIG.horizontalReverseOnPositiveError)
      local vLevel, vReverse = writeAxis(relay, verticalState,
        CONFIG.verticalOutputSide, CONFIG.verticalGearshiftSide, pitch,
        pdCommand(pitch, previousPitch), CONFIG.verticalReverseOnPositiveError)
      draw(yaw, pitch, hLevel, vLevel, hReverse, vReverse,
        hAim, vAim, right, left, up, down)
      previousYaw, previousPitch = yaw, pitch
    end
    wasAiming = aiming
    sleep(TICK_SECONDS)
  end
end

local ok, message = xpcall(run, function(e)
  if tostring(e) == "Terminated" then return "__terminated__" end
  return tostring(e).."\n"..debug.traceback()
end)
stop(peripheral.find("redstone_relay"))
if not ok and message ~= "__terminated__" then error(message, 0) end
