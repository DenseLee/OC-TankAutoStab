-- Turret-side world-space stabilizer for Create Rotation Speed Controllers.
--
-- Computer layout:
--   front = wireless modem to the hull computer
--   wired modem network = two CC:Tweaked Redstone Relays
--
-- Relay wiring (faces are relative to the relay blocks):
--   horizontal/yaw relay: top = aim right (+X), bottom = aim left (-X)
--   vertical/pitch relay: top = aim up (+Y), bottom = aim down (-Y)
--   steering relay (redstone_relay_5): top = steering right (+X),
--                                      bottom = steering left (-X)
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
  yawRelayName = "redstone_relay_4",
  pitchRelayName = "redstone_relay_2",
  yawPositiveSide = "top",
  yawNegativeSide = "bottom",
  pitchPositiveSide = "top",
  pitchNegativeSide = "bottom",
  steeringRelayName = "redstone_relay_5",
  steeringPositiveSide = "top",
  steeringNegativeSide = "bottom",

  -- Maximum signed RPM sent to the hull controllers.
  maxYawRPM = 40,
  maxPitchRPM = 40,
  yawPositiveRPMReversed = false,
  pitchPositiveRPMReversed = false,

  deadzoneDegrees = 0.2,
  -- Temporary settling test: immediately command 0 RPM after manual input
  -- is released and the axis is inside the deadzone.
  hardStopInsideDeadzone = true,
  -- Outer loop: the desired angular speed is error / response time.
  -- Smaller values correct faster; output still cannot exceed max axis RPM.
  targetResponseSeconds = 1.0,
  -- Inner rate correction. It compares desired and measured turret rate.
  rateFeedbackGain = 0.20,

  -- Learned plant response, in turret degrees/second per controller RPM.
  -- 3.3 is the measured initial estimate for the current turret setup.
  -- The estimate adapts only during steady manual movement.
  initialYawDegreesPerSecondPerRPM = 3.3,
  initialPitchDegreesPerSecondPerRPM = 3.3,
  plantLearningRate = 0.02,
  plantLearningMinimumRPM = 30,
  plantLearningMaximumAccelerationDegrees = 3,
  minimumPlantGain = 0.0005,
  maximumPlantGain = 10,
  yawOmegaSign = 1,
  pitchOmegaSign = 1,
  -- Largest command change each second. This is a command smoother, not a
  -- mechanical speed limit: 1600 allows 80 RPM change per game tick.
  maxRPMChangePerSecond = 1600,

  aimDegreesPerSecond = 36,
  -- Direct RPM contribution while the player is aiming. Tune these so a full
  -- stick roughly matches aimDegreesPerSecond through your turret gearing.
  -- The PD correction remains active to hold the world-space target.
  manualAimMaxYawRPM = 40,
  manualAimMaxPitchRPM = 40,
  -- Keep a held stick's requested RPM direct and predictable. The adaptive
  -- rate loop resumes as soon as that axis is released.
  manualAimBypassesRateFeedback = true,
  aimInputDeadzone = 0.05,
  -- Higher exponent makes small analogue inputs much slower and more precise.
  aimInputCurveExponent = 2.5,
  invertHorizontalAim = true,
  invertVerticalAim = true,
  -- On releasing both aim axes, hold the actual turret orientation instead
  -- of returning to a virtual target that may lag behind feed-forward motion.
  captureTargetOnAimRelease = true,
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

local function rotateVector(q, vector)
  local rotated = multiply(multiply(q, { x=vector.x, y=vector.y, z=vector.z, w=0 }), inverse(q))
  return { x=rotated.x, y=rotated.y, z=rotated.z }
end

local function dot(a, b) return a.x*b.x + a.y*b.y + a.z*b.z end

local function asVector(value, methodName)
  assert(type(value) == "table", methodName .. "() did not return a table")
  local x, y, z = value.x or value[1], value.y or value[2], value.z or value[3]
  if value.v then x, y, z = value.v.x or value.v[1], value.v.y or value.v[2], value.v.z or value.v[3] end
  assert(type(x) == "number" and type(y) == "number" and type(z) == "number",
    "Unknown vector format from " .. methodName .. "()")
  return { x=x, y=y, z=z }
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

local function approach(current, target, maximumChange)
  if target > current then return math.min(current + maximumChange, target) end
  return math.max(current - maximumChange, target)
end

local function newAxisState(initialPlantGain)
  return { lastOmega = 0, commandRPM = 0, plantGain = initialPlantGain }
end

local function rpmForError(errorRadians, state, maximumRPM, reversed, feedforwardRPM,
  angularVelocity, isManualAiming)
  local errorDegrees = degrees(errorRadians)
  local angularVelocityDegrees = degrees(angularVelocity)
  local angularAcceleration = (angularVelocityDegrees - state.lastOmega) / CONFIG.loopSeconds
  state.lastOmega = angularVelocityDegrees

  -- Learn the actual turret rate produced by a controller RPM only while a
  -- manual command is steady. This avoids learning hull shocks as gearing.
  local learningRPM = state.commandRPM
  if isManualAiming and math.abs(learningRPM) >= CONFIG.plantLearningMinimumRPM
    and math.abs(angularAcceleration) <= CONFIG.plantLearningMaximumAccelerationDegrees
    and learningRPM * angularVelocityDegrees > 0 then
    local sample = clamp(math.abs(angularVelocityDegrees / learningRPM),
      CONFIG.minimumPlantGain, CONFIG.maximumPlantGain)
    state.plantGain = state.plantGain + CONFIG.plantLearningRate
      * (sample - state.plantGain)
  end

  -- Desired rate comes from position error, plus direct manual aim rate.
  -- The measured rate closes the inner loop without requiring an error
  -- history or a fixed degrees-to-RPM proportional scale.
  local hardStop = CONFIG.hardStopInsideDeadzone and feedforwardRPM == 0
    and math.abs(errorDegrees) <= CONFIG.deadzoneDegrees
  local desiredRPM
  local desiredRate = 0
  if hardStop then
    -- Bypass rate braking for this test. With the configured RPM slew this
    -- reaches 0 in one tick at the current 40 RPM output limit.
    desiredRPM = 0
  elseif CONFIG.manualAimBypassesRateFeedback and feedforwardRPM ~= 0 then
    -- Do not let a noisy/incorrect omega estimate slow a held manual input.
    -- Target re-capture on release still returns control to stabilization.
    desiredRPM = feedforwardRPM
    desiredRate = feedforwardRPM * state.plantGain
  else
    local positionRate = 0
    if math.abs(errorDegrees) > CONFIG.deadzoneDegrees then
      positionRate = errorDegrees / CONFIG.targetResponseSeconds
    end
    desiredRate = feedforwardRPM * state.plantGain + positionRate
    local maximumRate = maximumRPM * state.plantGain
    desiredRate = clamp(desiredRate, -maximumRate, maximumRate)
    local rateError = desiredRate - angularVelocityDegrees
    desiredRPM = desiredRate / state.plantGain
      + CONFIG.rateFeedbackGain * rateError / state.plantGain
  end
  local rpm = clamp(desiredRPM,
    -maximumRPM, maximumRPM)
  if reversed then rpm = -rpm end
  state.commandRPM = approach(state.commandRPM, rpm,
    CONFIG.maxRPMChangePerSecond * CONFIG.loopSeconds)
  -- A stopped target should not retain a fractional-RPM command forever.
  if desiredRPM == 0 and math.abs(state.commandRPM) < 0.5 then state.commandRPM = 0 end
  rpm = state.commandRPM
  if rpm < 0 then rpm = math.ceil(rpm - 0.5) else rpm = math.floor(rpm + 0.5) end
  return rpm, errorDegrees, state.plantGain, desiredRate
end

local function send(command)
  if CONFIG.hullComputerId then
    rednet.send(CONFIG.hullComputerId, command, CONFIG.protocol)
  else
    rednet.broadcast(command, CONFIG.protocol)
  end
end

local function draw(yawInput, pitchInput, steeringInput, yawFeedforward, pitchFeedforward,
  yawError, pitchError, yawOmega, pitchOmega, yawGain, pitchGain, yawRPM, pitchRPM)
  if not CONFIG.showDebug then return end
  term.setCursorPos(1, 1)
  term.clear()
  print("Turret speed-controller stabilizer")
  print("Ctrl+T stops hull aim motors")
  print("")
  print(string.format("Aim X/Y:       %+.2f / %+.2f", yawInput, pitchInput))
  print(string.format("Steering X:    %+.2f", steeringInput))
  print(string.format("Aim FF RPM:    %+.0f / %+.0f", yawFeedforward, pitchFeedforward))
  print(string.format("Yaw error:     %+.2f deg", yawError))
  print(string.format("Pitch error:   %+.2f deg", pitchError))
  print(string.format("Omega Y/P:     %+.2f / %+.2f rad/s", yawOmega, pitchOmega))
  print(string.format("Gain Y/P:      %.5f / %.5f deg/s/RPM", yawGain, pitchGain))
  print(string.format("Yaw RPM:       %+d", yawRPM))
  print(string.format("Pitch RPM:     %+d", pitchRPM))
end

local function run()
  rednet.open(CONFIG.wirelessModemSide)
  local yawRelay = getRelay(CONFIG.yawRelayName, "yawRelayName")
  local pitchRelay = getRelay(CONFIG.pitchRelayName, "pitchRelayName")
  local steeringRelay = getRelay(CONFIG.steeringRelayName, "steeringRelayName")
  local target = normalize(asQuaternion(ship.getQuaternion()))
  local yawAxis = newAxisState(CONFIG.initialYawDegreesPerSecondPerRPM)
  local pitchAxis = newAxisState(CONFIG.initialPitchDegreesPerSecondPerRPM)
  local wasAiming = false

  while true do
    local rawYaw = signedInput(yawRelay, CONFIG.yawPositiveSide, CONFIG.yawNegativeSide)
    local rawPitch = signedInput(pitchRelay, CONFIG.pitchPositiveSide, CONFIG.pitchNegativeSide)
    local rawSteering = signedInput(steeringRelay,
      CONFIG.steeringPositiveSide, CONFIG.steeringNegativeSide)
    local yawInput = applyInputCurve(rawYaw, CONFIG.invertHorizontalAim)
    local pitchInput = applyInputCurve(rawPitch, CONFIG.invertVerticalAim)
    local steeringInput = applyInputCurve(rawSteering, false)
    local current = normalize(asQuaternion(ship.getQuaternion()))
    local isAiming = yawInput ~= 0 or pitchInput ~= 0

    if isAiming then
      local amount = math.rad(CONFIG.aimDegreesPerSecond * CONFIG.loopSeconds)
      target = normalize(multiply(axisAngle(0, 1, 0, yawInput * amount),
        multiply(target, axisAngle(1, 0, 0, pitchInput * amount))))
    elseif wasAiming and CONFIG.captureTargetOnAimRelease then
      -- Feed-forward can move the physical turret faster than the virtual
      -- target. Re-anchor at release so it holds this exact orientation.
      target = current
    end
    wasAiming = isAiming

    local yawError, pitchError = getYawPitchError(target, current)
    local omega = asVector(ship.getOmega(), "ship.getOmega")
    -- Yaw is around world Y. Pitch is around the turret's current local X
    -- axis, rotated into world space before projecting omega onto it.
    local yawOmega = omega.y * CONFIG.yawOmegaSign
    local pitchOmega = dot(omega, rotateVector(current, { x=1, y=0, z=0 }))
      * CONFIG.pitchOmegaSign
    local yawFeedforward = yawInput * CONFIG.manualAimMaxYawRPM
    local pitchFeedforward = pitchInput * CONFIG.manualAimMaxPitchRPM
    local yawRPM, yawDegrees, yawGain = rpmForError(yawError, yawAxis,
      CONFIG.maxYawRPM, CONFIG.yawPositiveRPMReversed, yawFeedforward,
      yawOmega, isAiming)
    local pitchRPM, pitchDegrees, pitchGain = rpmForError(pitchError, pitchAxis,
      CONFIG.maxPitchRPM, CONFIG.pitchPositiveRPMReversed, pitchFeedforward,
      pitchOmega, isAiming)

    send({ kind="aim_rpm", yawRPM=yawRPM, pitchRPM=pitchRPM,
      steering=steeringInput })
    draw(yawInput, pitchInput, steeringInput, yawFeedforward, pitchFeedforward,
      yawDegrees, pitchDegrees, yawOmega, pitchOmega, yawGain, pitchGain, yawRPM, pitchRPM)
    sleep(CONFIG.loopSeconds)
  end
end

local ok, err = pcall(run)
-- One final stop command lets the hull stop immediately on Ctrl+T.
pcall(function() send({ kind="aim_rpm", yawRPM=0, pitchRPM=0 }) end)
if not ok and err ~= "Terminated" then error(err, 0) end
