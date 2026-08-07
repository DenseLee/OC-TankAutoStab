-- Turret-side world-space stabilizer for Create Rotation Speed Controllers.
--
-- Computer layout:
--   front = wireless modem to the hull computer
--   wired modem network = two CC:Tweaked Redstone Relays
--
-- Relay wiring (faces are relative to the relay blocks):
--   horizontal/yaw relay: top = aim right (+X), bottom = aim left (-X)
--   vertical/pitch relay: top = aim up (+Y), bottom = aim down (-Y)
--   horizontal/yaw relay: front = steering right (+X)
--   vertical/pitch relay: front = steering left (-X)
--
-- The turret computer owns the target because it is on the turret ship and
-- can therefore read that ship's quaternion. It sends signed RPM commands to
-- the hull computer; it does not drive any Create parts directly.

local CONFIG = {
  wirelessModemSide = "front",
  protocol = "tank_speed_stab",
  -- Set this to the hull computer ID for one-to-one messages. Leave nil to
  -- broadcast; the hull script ignores all other protocols.
  hullComputerId = nil,

  -- Run peripheral_check.lua on the turret computer and put the exact two
  -- relay names here. They must be different.
  yawRelayName = "SET_YAW_RELAY_NAME",
  pitchRelayName = "SET_PITCH_RELAY_NAME",
  yawPositiveSide = "top",
  yawNegativeSide = "bottom",
  pitchPositiveSide = "top",
  pitchNegativeSide = "bottom",
  steeringPositiveSide = "front",
  steeringNegativeSide = "front",

  -- Maximum signed RPM sent to the hull controllers.
  maxYawRPM = 256,
  maxPitchRPM = 256,
  yawPositiveRPMReversed = false,
  pitchPositiveRPMReversed = false,

  fullSpeedAtDegrees = 45,
  deadzoneDegrees = 0.2,
  proportionalGain = 1,
  derivativeGainSeconds = 0.0005,

  aimDegreesPerSecond = 36,
  aimInputDeadzone = 0.05,
  aimInputCurveExponent = 1.5,
  invertHorizontalAim = true,
  invertVerticalAim = true,
  loopSeconds = 0.05,
  showDebug = true,
}

local function clamp(value, low, high)
  return math.max(low, math.min(high, value))
end

local function degrees(value) return value * 180 / math.pi end

local function multiply(a, b)
  return {
    x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
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
  assert(type(x) == "number" and type(y) == "number" and type(z) == "number"
    and type(w) == "number", "Unknown quaternion format")
  return { x=x, y=y, z=z, w=w }
end

local function getYawPitchError(target, current)
  local relative = normalize(multiply(inverse(current), target))
  local pitch = math.asin(clamp(2 * (relative.x*relative.w - relative.y*relative.z), -1, 1))
  local yaw = math.atan(2 * (relative.x*relative.z + relative.y*relative.w),
    1 - 2 * (relative.x*relative.x + relative.y*relative.y))
  return yaw, pitch
end

local function getRelay(name, label)
  if name:sub(1, 4) == "SET_" then
    error("Set " .. label .. " in CONFIG using peripheral_check.lua output.")
  end
  local relay = peripheral.wrap(name)
  if not relay or type(relay.getAnalogInput) ~= "function" then
    error("Cannot find Redstone Relay '" .. name .. "'.")
  end
  return relay
end

local function signedInput(relay, positiveSide, negativeSide)
  local positive = relay.getAnalogInput(positiveSide)
  local negative = relay.getAnalogInput(negativeSide)
  return clamp((positive - negative) / 15, -1, 1), positive, negative
end

local function applyInputCurve(value, inverted)
  if inverted then value = -value end
  local magnitude = math.abs(value)
  if magnitude <= CONFIG.aimInputDeadzone then return 0 end
  magnitude = (magnitude - CONFIG.aimInputDeadzone) / (1 - CONFIG.aimInputDeadzone)
  magnitude = magnitude ^ CONFIG.aimInputCurveExponent
  return value < 0 and -magnitude or magnitude
end

local function rpmForError(errorRadians, previousError, maximumRPM, reversed)
  local errorDegrees = degrees(errorRadians)
  if math.abs(errorDegrees) <= CONFIG.deadzoneDegrees then return 0, errorDegrees end

  local rateDegrees = (errorDegrees - previousError) / CONFIG.loopSeconds
  local correction = CONFIG.proportionalGain * errorDegrees
    + CONFIG.derivativeGainSeconds * rateDegrees
  local rpm = clamp(correction / CONFIG.fullSpeedAtDegrees * maximumRPM,
    -maximumRPM, maximumRPM)
  if reversed then rpm = -rpm end
  if rpm < 0 then rpm = math.ceil(rpm - 0.5) else rpm = math.floor(rpm + 0.5) end
  return rpm, errorDegrees
end

local function send(command)
  if CONFIG.hullComputerId then
    rednet.send(CONFIG.hullComputerId, command, CONFIG.protocol)
  else
    rednet.broadcast(command, CONFIG.protocol)
  end
end

local function draw(yawInput, pitchInput, steeringInput, yawError, pitchError, yawRPM, pitchRPM)
  if not CONFIG.showDebug then return end
  term.setCursorPos(1, 1)
  term.clear()
  print("Turret speed-controller stabilizer")
  print("Ctrl+T stops hull aim motors")
  print("")
  print(string.format("Aim X/Y:       %+.2f / %+.2f", yawInput, pitchInput))
  print(string.format("Steering X:    %+.2f", steeringInput))
  print(string.format("Yaw error:     %+.2f deg", yawError))
  print(string.format("Pitch error:   %+.2f deg", pitchError))
  print(string.format("Yaw RPM:       %+d", yawRPM))
  print(string.format("Pitch RPM:     %+d", pitchRPM))
end

local function run()
  rednet.open(CONFIG.wirelessModemSide)
  local yawRelay = getRelay(CONFIG.yawRelayName, "yawRelayName")
  local pitchRelay = getRelay(CONFIG.pitchRelayName, "pitchRelayName")
  local target = normalize(asQuaternion(ship.getQuaternion()))
  local lastYawError, lastPitchError = 0, 0

  while true do
    local rawYaw = signedInput(yawRelay, CONFIG.yawPositiveSide, CONFIG.yawNegativeSide)
    local rawPitch = signedInput(pitchRelay, CONFIG.pitchPositiveSide, CONFIG.pitchNegativeSide)
    -- Steering sides live on separate relays, so read and combine them here.
    local steeringPositive = yawRelay.getAnalogInput(CONFIG.steeringPositiveSide)
    local steeringNegative = pitchRelay.getAnalogInput(CONFIG.steeringNegativeSide)
    local rawSteering = clamp((steeringPositive - steeringNegative) / 15, -1, 1)
    local yawInput = applyInputCurve(rawYaw, CONFIG.invertHorizontalAim)
    local pitchInput = applyInputCurve(rawPitch, CONFIG.invertVerticalAim)
    local steeringInput = applyInputCurve(rawSteering, false)

    if yawInput ~= 0 or pitchInput ~= 0 then
      local amount = math.rad(CONFIG.aimDegreesPerSecond * CONFIG.loopSeconds)
      target = normalize(multiply(axisAngle(0, 1, 0, yawInput * amount),
        multiply(target, axisAngle(1, 0, 0, pitchInput * amount))))
    end

    local current = normalize(asQuaternion(ship.getQuaternion()))
    local yawError, pitchError = getYawPitchError(target, current)
    local yawRPM, yawDegrees = rpmForError(yawError, lastYawError,
      CONFIG.maxYawRPM, CONFIG.yawPositiveRPMReversed)
    local pitchRPM, pitchDegrees = rpmForError(pitchError, lastPitchError,
      CONFIG.maxPitchRPM, CONFIG.pitchPositiveRPMReversed)
    lastYawError, lastPitchError = yawDegrees, pitchDegrees

    send({ kind="aim_rpm", yawRPM=yawRPM, pitchRPM=pitchRPM,
      steering=steeringInput })
    draw(yawInput, pitchInput, steeringInput, yawDegrees, pitchDegrees, yawRPM, pitchRPM)
    sleep(CONFIG.loopSeconds)
  end
end

local ok, err = pcall(run)
-- One final stop command lets the hull stop immediately on Ctrl+T.
pcall(function() send({ kind="aim_rpm", yawRPM=0, pitchRPM=0 }) end)
if not ok and err ~= "Terminated" then error(err, 0) end
