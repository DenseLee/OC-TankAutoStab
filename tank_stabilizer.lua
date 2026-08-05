-- Tank Tussle / CC:Tweaked tank controller
--
-- Expected wiring:
--   left side  -> left redstone resistor / left gearshift circuit
--   right side -> right redstone resistor / right gearshift circuit
--   front      -> wired or wireless modem (optional; used to find the lectern)
--   behind     -> Tweaked Lectern Controller, directly or through the modem network
--
-- The resistors are normal redstone targets, so this program writes analog
-- redstone levels (0..15) to the computer's left and right sides.

local CONFIG = {
  -- "differential" maps the left stick to tank drive:
  --   forward/back = throttle, left/right = steering.
  -- "single_axis" is useful when the two gearshifts are being used as a
  -- stabilizer pair: one side is driven for left correction and the other
  -- for right correction.
  mode = "differential",

  leftOutputSide = "left",
  rightOutputSide = "right",

  -- Tweaked Controller axis IDs:
  -- 1 = left X, 2 = left Y, 3 = right X, 4 = right Y,
  -- 5 = left trigger, 6 = right trigger.
  throttleAxis = 2,
  steeringAxis = 1,
  stabilizerAxis = 3,

  invertThrottle = true, -- gamepads normally report forward as negative Y
  invertSteering = false,
  invertStabilizer = false,

  deadzone = 0.08,
  responseCurve = 1.0, -- >1 gives finer control near center
  updateSeconds = 0.05,
  showDebug = true,
}

local function clamp(value, low, high)
  if value < low then return low end
  if value > high then return high end
  return value
end

local function applyDeadzone(value)
  if math.abs(value) <= CONFIG.deadzone then return 0 end
  local sign = value < 0 and -1 or 1
  local scaled = (math.abs(value) - CONFIG.deadzone) / (1 - CONFIG.deadzone)
  return sign * (scaled ^ CONFIG.responseCurve)
end

local function readAxis(controller, axis, invert)
  local value = controller.getAxis(axis)
  if type(value) ~= "number" then value = 0 end
  value = clamp(value, -1, 1)
  if invert then value = -value end
  return applyDeadzone(value)
end

local function toRedstone(value)
  -- A signed command is represented by magnitude. Direction is supplied by
  -- the tank's gearshift/link arrangement; a resistor receives 0..15.
  return math.floor(clamp(math.abs(value), 0, 1) * 15 + 0.5)
end

local function setOutputs(left, right)
  redstone.setAnalogOutput(CONFIG.leftOutputSide, toRedstone(left))
  redstone.setAnalogOutput(CONFIG.rightOutputSide, toRedstone(right))
end

local function stop()
  redstone.setAnalogOutput(CONFIG.leftOutputSide, 0)
  redstone.setAnalogOutput(CONFIG.rightOutputSide, 0)
end

local function findController()
  local controller = peripheral.find("tweaked_controller")
  if controller then return controller end

  -- Some older builds expose the peripheral through a modem with a name.
  -- Accept it when its methods match the documented API.
  for _, name in ipairs(peripheral.getNames()) do
    local candidate = peripheral.wrap(name)
    if candidate and type(candidate.getAxis) == "function"
        and type(candidate.hasUser) == "function" then
      return candidate
    end
  end
end

local function draw(controller, left, right)
  if not CONFIG.showDebug then return end
  term.clear()
  term.setCursorPos(1, 1)
  print("Tank stabilizer")
  print("Mode: " .. CONFIG.mode)
  print("Controller: " .. (controller and "connected" or "not found"))
  if controller then
    print("User: " .. (controller.hasUser() and "yes" or "no"))
  end
  print("Left output : " .. toRedstone(left) .. "/15")
  print("Right output: " .. toRedstone(right) .. "/15")
  print("Press Ctrl+T to stop")
end

local function run()
  while true do
    local controller = findController()
    local left, right = 0, 0

    if controller and controller.hasUser() then
      if CONFIG.mode == "single_axis" then
        local correction = readAxis(controller, CONFIG.stabilizerAxis,
          CONFIG.invertStabilizer)
        left = math.max(0, correction)
        right = math.max(0, -correction)
      else
        local throttle = readAxis(controller, CONFIG.throttleAxis,
          CONFIG.invertThrottle)
        local steering = readAxis(controller, CONFIG.steeringAxis,
          CONFIG.invertSteering)
        left = clamp(throttle + steering, -1, 1)
        right = clamp(throttle - steering, -1, 1)
      end
    end

    setOutputs(left, right)
    draw(controller, left, right)
    sleep(CONFIG.updateSeconds)
  end
end

local ok, errorMessage = xpcall(run, function(errorObject)
  return tostring(errorObject) .. "\n" .. debug.traceback()
end)

stop()
if not ok then
  term.clear()
  term.setCursorPos(1, 1)
  print("Tank stabilizer stopped:")
  print(errorMessage)
end
