-- Create Rotation Speed Controller smooth-direction test.
--
-- This drives ONE Create Rotation Speed Controller through:
--   0 RPM -> +256 RPM -> 0 RPM -> -256 RPM -> 0 RPM
--
-- It changes target speed every game tick, so direction reverses only after
-- reaching 0. Set MAX_RPM lower first if your drivetrain should be tested
-- more gently. Ctrl+T always commands 0 RPM before exiting.

local CONFIG = {
  -- Leave nil to automatically find the first speed-controller peripheral.
  -- Set this to the exact name printed by peripheral_check.lua to choose one.
  peripheralName = nil,

  maxRPM = 256,
  secondsPerRamp = 4,
  loopSeconds = 0.05,
}

local function isSpeedController(name)
  for _, peripheralType in ipairs({ peripheral.getType(name) }) do
    if peripheralType == "Create_RotationSpeedController"
      or peripheralType == "rotation_speed_controller" then
      return true
    end
  end
  return false
end

local function findController()
  if CONFIG.peripheralName then
    if not peripheral.isPresent(CONFIG.peripheralName) then
      error("Configured peripheral is not present: " .. CONFIG.peripheralName)
    end
    return CONFIG.peripheralName, peripheral.wrap(CONFIG.peripheralName)
  end

  for _, name in ipairs(peripheral.getNames()) do
    if isSpeedController(name) then
      return name, peripheral.wrap(name)
    end
  end

  error("No Create Rotation Speed Controller found. Run peripheral_check.lua first.")
end

local controllerName, controller = findController()
if not controller or type(controller.setTargetSpeed) ~= "function" then
  error("'" .. controllerName .. "' does not provide setTargetSpeed(number).")
end

local currentRPM = 0
local function setRPM(rpm)
  -- Round to whole RPM: Create's peripheral accepts its target in integer RPM.
  currentRPM = math.floor(rpm + (rpm >= 0 and 0.5 or -0.5))
  controller.setTargetSpeed(currentRPM)
end

local function status(label)
  term.setCursorPos(1, 1)
  term.clear()
  print("Create speed-controller sweep")
  print("Peripheral: " .. controllerName)
  print("Phase: " .. label)
  print(string.format("Command: %+d RPM", currentRPM))
  print("")
  print("Ctrl+T = immediately command 0 RPM")
end

local function ramp(fromRPM, toRPM, label)
  local ticks = math.max(1, math.floor(CONFIG.secondsPerRamp / CONFIG.loopSeconds + 0.5))
  for tick = 1, ticks do
    local fraction = tick / ticks
    setRPM(fromRPM + (toRPM - fromRPM) * fraction)
    status(label)
    sleep(CONFIG.loopSeconds)
  end
end

local function run()
  setRPM(0)
  status("starting at zero")
  sleep(1)

  ramp(0, CONFIG.maxRPM, "clockwise ramp up")
  ramp(CONFIG.maxRPM, 0, "clockwise ramp down")
  ramp(0, -CONFIG.maxRPM, "counter-clockwise ramp up")
  ramp(-CONFIG.maxRPM, 0, "counter-clockwise ramp down")

  setRPM(0)
  status("finished at zero")
  print("Test complete.")
end

local ok, err = pcall(run)
-- Do not leave the drivetrain commanded to move after Ctrl+T or an error.
pcall(setRPM, 0)

if not ok then
  term.setCursorPos(1, 8)
  term.clearLine()
  if err == "Terminated" then
    print("Stopped: target speed set to 0 RPM.")
  else
    error(err, 0)
  end
end
