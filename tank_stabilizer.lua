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

  -- A proportional controller: errors at or above this angle use full speed.
  -- Increase it for gentler correction; decrease it for more aggressive hold.
  fullSpeedAtDegrees = 12,
  deadzoneDegrees = 0.35,
  updateSeconds = 0.05,
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
  return 15 - math.floor(fraction * 15 + 0.5)
end

local function writeAxis(side, errorRadians)
  -- The Redstone Resistor receives the analogue speed value. The linked
  -- gearshift assembly is responsible for applying the sign of this error
  -- (left/right or up/down) to the mechanical rotation path.
  redstone.setAnalogOutput(side, resistorLevel(errorRadians))
  return errorRadians
end

local function stop()
  redstone.setAnalogOutput(CONFIG.horizontalOutputSide, 15)
  redstone.setAnalogOutput(CONFIG.verticalOutputSide, 15)
end

local function getYawPitchError(target)
  local current = ship.getQuaternion()

  -- Convert the desired world rotation into the turret's current local frame.
  -- CC:VS quaternion:toEuler returns pitch, yaw, roll in a YXZ frame.
  local localError = current:inverse():mul(target):normalize()
  local pitch, yaw = localError:toEuler()
  return yaw, pitch, current
end

local function draw(target, yaw, pitch, horizontal, vertical)
  if not CONFIG.showDebug then return end
  term.clear()
  term.setCursorPos(1, 1)
  print("VS turret world hold")
  print("Target captured on boot")
  print(("Yaw error  : %7.2f deg"):format(radiansToDegrees(yaw)))
  print(("Pitch error: %7.2f deg"):format(radiansToDegrees(pitch)))
  print("Horiz resistor: " .. resistorLevel(horizontal) .. "/15")
  print("Vert  resistor: " .. resistorLevel(vertical) .. "/15")
  print("Ctrl+T = stop")
end

local function run()
  assert(type(ship) == "table" and type(ship.getQuaternion) == "function",
    "CC:VS ship API was not found. Put this computer on the turret ship.")

  local target = ship.getQuaternion():copy()
  while true do
    local yaw, pitch = getYawPitchError(target)
    local horizontal = writeAxis(CONFIG.horizontalOutputSide, yaw)
    local vertical = writeAxis(CONFIG.verticalOutputSide, pitch)
    draw(target, yaw, pitch, horizontal, vertical)
    sleep(CONFIG.updateSeconds)
  end
end

local ok, message = xpcall(run, function(errorObject)
  return tostring(errorObject) .. "\n" .. debug.traceback()
end)

stop()
if not ok then
  term.clear()
  term.setCursorPos(1, 1)
  print("Turret stabilizer stopped:")
  print(message)
end
