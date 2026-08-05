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
  fullSpeedAtDegrees = 12,
  deadzoneDegrees = 0.35,
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

local function writeAxis(relay, resistorSide, gearshiftSide, errorRadians,
    reverseOnPositive)
  -- The Redstone Resistor receives the analogue speed value. The linked
  -- gearshift is separately powered through the Redstone Relay to choose the
  -- correction direction.
  local inDeadzone = math.abs(errorRadians) <= math.rad(CONFIG.deadzoneDegrees)
  local reversed = errorRadians > 0
  if not reverseOnPositive then reversed = not reversed end
  if inDeadzone then reversed = false end

  redstone.setAnalogOutput(resistorSide, resistorLevel(errorRadians))
  relay.setOutput(gearshiftSide, reversed)
  return errorRadians, reversed
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

local function draw(target, yaw, pitch, horizontal, vertical, horizontalReverse,
    verticalReverse)
  if not CONFIG.showDebug then return end
  term.clear()
  term.setCursorPos(1, 1)
  print("VS turret world hold")
  print("Target captured on boot")
  print(("Yaw error  : %7.2f deg"):format(radiansToDegrees(yaw)))
  print(("Pitch error: %7.2f deg"):format(radiansToDegrees(pitch)))
  print("Horiz resistor: " .. resistorLevel(horizontal) .. "/15")
  print("Vert  resistor: " .. resistorLevel(vertical) .. "/15")
  print("Horiz gearshift: " .. (horizontalReverse and "reverse" or "normal"))
  print("Vert  gearshift: " .. (verticalReverse and "reverse" or "normal"))
  print("Ctrl+T = stop")
end

local function run()
  assert(type(ship) == "table" and type(ship.getQuaternion) == "function",
    "CC:VS ship API was not found. Put this computer on the turret ship.")
  local relay = peripheral.find("redstone_relay")
  assert(relay, "No redstone_relay found on the wired-modem network.")

  local target = asQuaternion(ship.getQuaternion())
  while true do
    local yaw, pitch = getYawPitchError(target)
    local horizontal, horizontalReverse = writeAxis(relay,
      CONFIG.horizontalOutputSide, CONFIG.horizontalGearshiftSide, yaw,
      CONFIG.horizontalReverseOnPositiveError)
    local vertical, verticalReverse = writeAxis(relay,
      CONFIG.verticalOutputSide, CONFIG.verticalGearshiftSide, pitch,
      CONFIG.verticalReverseOnPositiveError)
    draw(target, yaw, pitch, horizontal, vertical, horizontalReverse,
      verticalReverse)
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
