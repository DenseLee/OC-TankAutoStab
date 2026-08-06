-- World-space turret hold for CC:VS + VS: Clockwork.
--
-- Put this computer ON THE SEPARATE TURRET SHIP. On boot it records that
-- ship's quaternion, then tries to keep the turret at that exact world-space
-- yaw/pitch while the hull moves.
--
-- Requires the CC:VS `ship` API (and its bundled `quaternion` library).

local CONFIG = {
  -- Computer outputs connected to the two Redstone Resistors.
  horizontalOutputSide = "left",
  verticalOutputSide = "right",

  -- Redstone Relay sides that touch the two gearshifts. These sides are
  -- relative to the relay itself, not the computer.
  horizontalGearshiftSide = "left",
  verticalGearshiftSide = "right",

  -- A Create gearshift reverses its output while redstone-powered. Flip one
  -- of these if that axis initially corrects away from the saved heading.
  horizontalReverseOnPositiveError = true,
  verticalReverseOnPositiveError = true,

  -- A proportional controller: errors at or above this angle use full speed.
  -- Increase it for gentler correction; decrease it for more aggressive hold.
  fullSpeedAtDegrees = 45,
  deadzoneDegrees = 0.2,

  -- PD tuning. Keep proportionalGain at 1.0 to preserve the original speed
  -- response. The derivative term slows an axis already moving toward its
  -- target and boosts it slightly when moving farther away. Do not add an
  -- integral term: it would wind up while the gearshift is changing state.
  proportionalGain = 1,
  derivativeGainSeconds = 0.0005,

  -- Minimum number of control ticks between analogue resistor changes.
  -- The main loop still runs every 0.05 s; this only prevents rapid redstone
  -- level churn on the Redstone Resistors. 3 ticks = 0.15 seconds.
  resistorCooldownTicks = 1,

  -- Safety cap for the Clockwork Redstone Resistors. The program will never
  -- send a value lower than this, so it never releases the full input RPM.
  -- Start at 10 (roughly a gentle third-speed cap) and only lower it after
  -- confirming the kinetic network is stable and below Create's speed limit.
  minimumRedstoneResistorLevel = 0,

  -- Tweaked Lectern Controller aiming. Axis 3/4 are the right-stick X/Y
  -- inputs. While a player is using the lectern, these change the saved
  -- world-space target. Releasing the controller holds the new target.
  horizontalAimAxis = 3,
  verticalAimAxis = 4,
  aimDegreesPerSecond = 90,
  aimDeadzone = 0.08,
  invertHorizontalAim = false,
  invertVerticalAim = true,
  showDebug = true,
}

local function clamp(value, low, high)
  return math.max(low, math.min(high, value))
end

local function radiansToDegrees(value)
  return value * 180 / math.pi
end

local function resistorLevel(errorRadians)
  -- Clockwork Redstone Resistor: 0 = full speed, 15 = stopped.
  local deadzone = math.rad(CONFIG.deadzoneDegrees)
  if math.abs(errorRadians) <= deadzone then return 15 end

  local fraction = clamp(math.abs(errorRadians) /
    math.rad(CONFIG.fullSpeedAtDegrees), 0, 1)
  local requestedLevel = 15 - math.floor(fraction * 15 + 0.5)
  return math.max(CONFIG.minimumRedstoneResistorLevel, requestedLevel)
end

local function setResistor(state, side, desiredLevel)
  if state.cooldown > 0 then
    state.cooldown = state.cooldown - 1
    return state.level
  end

  if state.level ~= desiredLevel then
    redstone.setAnalogOutput(side, desiredLevel)
    state.level = desiredLevel
    state.cooldown = CONFIG.resistorCooldownTicks
  end
  return state.level
end

local function writeAxis(relay, resistorState, resistorSide, gearshiftSide,
    directionError, speedCommand, reverseOnPositive)
  -- The Redstone Resistor receives the analogue speed value. The linked
  -- gearshift is separately powered through the Redstone Relay to choose the
  -- correction direction.
  local inDeadzone = math.abs(directionError) <= math.rad(CONFIG.deadzoneDegrees)
  local reversed = directionError > 0
  if not reverseOnPositive then reversed = not reversed end
  if inDeadzone then reversed = false end

  local appliedLevel = setResistor(resistorState, resistorSide,
    resistorLevel(speedCommand))
  relay.setOutput(gearshiftSide, reversed)
  return appliedLevel, reversed
end

local function stop(relay)
  redstone.setAnalogOutput(CONFIG.horizontalOutputSide, 15)
  redstone.setAnalogOutput(CONFIG.verticalOutputSide, 15)
  if relay then
    relay.setOutput(CONFIG.horizontalGearshiftSide, false)
    relay.setOutput(CONFIG.verticalGearshiftSide, false)
  end
end

-- CC:VS versions serialise ship quaternions differently. Newer versions may
-- return a quaternion object, while older/modpack builds return a plain Lua
-- table. Convert both forms to { x, y, z, w } and do the small amount of
-- quaternion maths here, avoiding a version-specific dependency on :copy().
local function asQuaternion(value)
  assert(type(value) == "table", "ship.getQuaternion() did not return a table")

  local x = value.x or value[1]
  local y = value.y or value[2]
  local z = value.z or value[3]
  local w = value.w or value[4]

  -- CC:VS Advanced Math quaternion format: { v = vector(x,y,z), a = w }.
  if value.v then
    x = value.v.x or value.v[1]
    y = value.v.y or value.v[2]
    z = value.v.z or value.v[3]
    w = value.a
  end

  assert(type(x) == "number" and type(y) == "number"
      and type(z) == "number" and type(w) == "number",
    "Unknown quaternion format; run: print(textutils.serialize(ship.getQuaternion()))")
  return { x = x, y = y, z = z, w = w }
end

local function normalize(q)
  local length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
  assert(length > 0, "Invalid zero-length quaternion")
  return { x = q.x / length, y = q.y / length, z = q.z / length, w = q.w / length }
end

local function fromAxisAngle(x, y, z, angle)
  local halfAngle = angle / 2
  local sine = math.sin(halfAngle)
  return { x = x * sine, y = y * sine, z = z * sine, w = math.cos(halfAngle) }
end

local function inverse(q)
  local lengthSquared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
  assert(lengthSquared > 0, "Invalid zero-length quaternion")
  return {
    x = -q.x / lengthSquared, y = -q.y / lengthSquared,
    z = -q.z / lengthSquared, w = q.w / lengthSquared,
  }
end

local function multiply(a, b)
  return {
    x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  }
end

local function toPitchYaw(q)
  -- Euler extraction in the YXZ convention used by CC:VS Advanced Math.
  local pitch = math.asin(clamp(2 * (q.x * q.w - q.y * q.z), -1, 1))
  local yaw = math.atan(2 * (q.x * q.z + q.y * q.w),
    1 - 2 * (q.x * q.x + q.y * q.y))
  return pitch, yaw
end

local function getYawPitchError(target)
  local current = asQuaternion(ship.getQuaternion())

  -- Convert the desired world rotation into the turret's current local frame.
  local localError = normalize(multiply(inverse(current), target))
  local pitch, yaw = toPitchYaw(localError)
  return yaw, pitch, current
end

local function pdCommand(errorRadians, previousError)
  if math.abs(errorRadians) <= math.rad(CONFIG.deadzoneDegrees) then return 0 end
  if previousError == nil then return CONFIG.proportionalGain * math.abs(errorRadians) end

  local derivative = (errorRadians - previousError) / 0.05
  -- Convert the derivative to the direction of the current error. A negative
  -- result means the turret is already closing the error, so reduce speed.
  local directionalDerivative = (errorRadians < 0 and -1 or 1) * derivative
  return math.max(0, CONFIG.proportionalGain * math.abs(errorRadians)
    + CONFIG.derivativeGainSeconds * directionalDerivative)
end

local function findController()
  local controller = peripheral.find("tweaked_controller")
  if controller then return controller end

  for _, name in ipairs(peripheral.getNames()) do
    local candidate = peripheral.wrap(name)
    if candidate and type(candidate.hasUser) == "function"
        and type(candidate.getAxis) == "function" then
      return candidate
    end
  end
end

local function controllerAxis(controller, axis, inverted)
  local value = controller.getAxis(axis)
  if type(value) ~= "number" then return 0 end
  value = clamp(value, -1, 1)
  if inverted then value = -value end
  if math.abs(value) <= CONFIG.aimDeadzone then return 0 end

  local sign = value < 0 and -1 or 1
  return sign * (math.abs(value) - CONFIG.aimDeadzone) / (1 - CONFIG.aimDeadzone)
end

local function updateTargetFromController(target, controller)
  if not controller or not controller.hasUser() then return target, false end

  local horizontal = controllerAxis(controller, CONFIG.horizontalAimAxis,
    CONFIG.invertHorizontalAim)
  local vertical = controllerAxis(controller, CONFIG.verticalAimAxis,
    CONFIG.invertVerticalAim)
  if horizontal == 0 and vertical == 0 then return target, true end

  local degreesPerTick = CONFIG.aimDegreesPerSecond * 0.05
  -- Yaw is world-space (global Y). Pitch is target-local X so elevation stays
  -- intuitive after the turret has been traversed.
  local yawAdjustment = fromAxisAngle(0, 1, 0,
    math.rad(horizontal * degreesPerTick))
  local pitchAdjustment = fromAxisAngle(1, 0, 0,
    math.rad(vertical * degreesPerTick))
  return normalize(multiply(yawAdjustment, multiply(target, pitchAdjustment))), true
end

local function draw(target, yaw, pitch, horizontalLevel, verticalLevel, horizontalReverse,
    verticalReverse, playerAiming)
  if not CONFIG.showDebug then return end
  term.clear()
  term.setCursorPos(1, 1)
  print("VS turret world hold")
  print("Player aim: " .. (playerAiming and "active" or "holding target"))
  print(("Yaw error  : %7.2f deg"):format(radiansToDegrees(yaw)))
  print(("Pitch error: %7.2f deg"):format(radiansToDegrees(pitch)))
  print("Horiz resistor: " .. horizontalLevel .. "/15")
  print("Vert  resistor: " .. verticalLevel .. "/15")
  print("Horiz gearshift: " .. (horizontalReverse and "reverse" or "normal"))
  print("Vert  gearshift: " .. (verticalReverse and "reverse" or "normal"))
  print("Ctrl+T = stop")
end

local function run()
  assert(type(ship) == "table" and type(ship.getQuaternion) == "function",
    "CC:VS ship API was not found. Put this computer on the turret ship.")
  local relay = peripheral.find("redstone_relay")
  assert(relay, "No redstone_relay found on the wired-modem network.")
  local controller = findController()
  assert(controller, "No tweaked_controller found. Put the controller on a lectern.")
  pcall(controller.setFullPrecision, true)

  local target = asQuaternion(ship.getQuaternion())
  local previousYaw, previousPitch = nil, nil
  local horizontalResistor = { level = nil, cooldown = 0 }
  local verticalResistor = { level = nil, cooldown = 0 }
  while true do
    local playerAiming
    target, playerAiming = updateTargetFromController(target, controller)
    local yaw, pitch = getYawPitchError(target)
    local horizontalCommand = pdCommand(yaw, previousYaw)
    local verticalCommand = pdCommand(pitch, previousPitch)
    local horizontalLevel, horizontalReverse = writeAxis(relay,
      horizontalResistor,
      CONFIG.horizontalOutputSide, CONFIG.horizontalGearshiftSide, yaw,
      horizontalCommand,
      CONFIG.horizontalReverseOnPositiveError)
    local verticalLevel, verticalReverse = writeAxis(relay,
      verticalResistor,
      CONFIG.verticalOutputSide, CONFIG.verticalGearshiftSide, pitch,
      verticalCommand,
      CONFIG.verticalReverseOnPositiveError)
    draw(target, yaw, pitch, horizontalLevel, verticalLevel, horizontalReverse,
      verticalReverse, playerAiming)
    previousYaw, previousPitch = yaw, pitch
    -- CC:Tweaked's sleep yields execution. 0.05 seconds is one Minecraft
    -- tick at 20 TPS, so this samples and corrects once per game tick.
    sleep(0.05)
  end
end

local ok, message = xpcall(run, function(errorObject)
  if tostring(errorObject) == "Terminated" then return "__terminated__" end
  return tostring(errorObject) .. "\n" .. debug.traceback()
end)

-- The relay is discovered inside run(), so locate it again for a safe stop
-- after either Ctrl+T or a runtime error.
stop(peripheral.find("redstone_relay"))
if not ok and message ~= "__terminated__" then
  term.clear()
  term.setCursorPos(1, 1)
  print("Turret stabilizer stopped:")
  print(message)
end
