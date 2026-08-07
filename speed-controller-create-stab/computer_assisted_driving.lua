-- Computer-assisted analogue driving for the hull computer.
--
-- Hardware layout:
--   computer front = controller left-stick +Y (forward)
--   computer top   = controller left-stick -Y (reverse), through gearbox
--   computer back  = Create Rotation Speed Controller for transmission
--   computer bottom = wireless modem (unused by this standalone drive test)
--
-- Redstone levels are 0..15. The two directions are subtracted, so if both
-- are active at once they cancel instead of commanding both directions.

local CONFIG = {
  transmissionControllerSide = "back",
  forwardInputSide = "front",
  reverseInputSide = "top",

  maxDriveRPM = 256,
  -- Used only while an input is held. Time from 0 to full speed is
  -- maxDriveRPM / this value; 128 is about two seconds to 256 RPM.
  -- Releasing both directions always commands 0 RPM immediately.
  accelerationRPMPerSecond = 128,
  inputDeadzone = 0.03,
  inputCurveExponent = 1.5,
  loopSeconds = 0.05,
  showDebug = true,
}

local transmission = peripheral.wrap(CONFIG.transmissionControllerSide)
if not transmission or type(transmission.setTargetSpeed) ~= "function" then
  error("No Create Rotation Speed Controller on " .. CONFIG.transmissionControllerSide
    .. ". Run peripheral_check.lua on the hull computer.")
end

local commandedRPM = 0

local function clamp(value, low, high)
  return math.max(low, math.min(high, value))
end

local function round(value)
  return math.floor(value + (value >= 0 and 0.5 or -0.5))
end

local function curveInput(value)
  local magnitude = math.abs(value)
  if magnitude <= CONFIG.inputDeadzone then return 0 end

  -- Rescale after the deadzone so the remaining stick range still reaches 1.
  magnitude = (magnitude - CONFIG.inputDeadzone) / (1 - CONFIG.inputDeadzone)
  magnitude = magnitude ^ CONFIG.inputCurveExponent
  return value < 0 and -magnitude or magnitude
end

local function setRPM(rpm)
  rpm = round(rpm)
  if rpm ~= commandedRPM then
    transmission.setTargetSpeed(rpm)
    commandedRPM = rpm
  end
end

local function approach(current, target, maximumChange)
  if target > current then
    return math.min(current + maximumChange, target)
  end
  return math.max(current - maximumChange, target)
end

local function draw(forward, reverse, input, targetRPM)
  if not CONFIG.showDebug then return end
  term.setCursorPos(1, 1)
  term.clear()
  print("Computer-assisted driving")
  print("Ctrl+T stops the transmission")
  print("")
  print(string.format("Forward (front): %2d", forward))
  print(string.format("Reverse (top):   %2d", reverse))
  print(string.format("Drive input:     %+.2f", input))
  print(string.format("Requested:       %+d RPM", round(targetRPM)))
  print(string.format("Commanded:       %+d RPM", commandedRPM))
  print(string.format("Acceleration:    %d RPM/s", CONFIG.accelerationRPMPerSecond))
  print("")
  print("Both inputs cancel each other.")
end

local function run()
  -- Synchronize the controller with our cached value before reading input.
  -- This prevents an old command from a previous program continuing at boot.
  transmission.setTargetSpeed(0)
  commandedRPM = 0

  while true do
    local forward = redstone.getAnalogInput(CONFIG.forwardInputSide)
    local reverse = redstone.getAnalogInput(CONFIG.reverseInputSide)

    local rawInput = clamp((forward - reverse) / 15, -1, 1)
    local driveInput = curveInput(rawInput)
    local targetRPM = driveInput * CONFIG.maxDriveRPM
    local nextRPM
    if driveInput == 0 then
      -- Do not let the acceleration ramp leave a stale movement command after
      -- the player releases the stick. This also prevents turn-only inputs
      -- from causing a short forward creep.
      nextRPM = 0
    else
      local maximumChange = CONFIG.accelerationRPMPerSecond * CONFIG.loopSeconds
      nextRPM = approach(commandedRPM, targetRPM, maximumChange)
    end

    setRPM(nextRPM)
    draw(forward, reverse, driveInput, targetRPM)
    sleep(CONFIG.loopSeconds)
  end
end

local ok, err = pcall(run)
-- Never leave the drive transmission spinning if this program ends.
pcall(function() transmission.setTargetSpeed(0) end)

if not ok then
  term.setCursorPos(1, 10)
  term.clearLine()
  if err == "Terminated" then
    print("Driving stopped: transmission set to 0 RPM.")
  else
    error(err, 0)
  end
end
