-- Computer-assisted analogue driving for the hull computer.
--
-- Hardware layout:
--   computer front = controller left-stick +Y (forward)
--   computer top   = controller left-stick -Y (reverse), through gearbox
--   computer back  = Create Rotation Speed Controller for transmission
--   computer left  = Create Rotation Speed Controller for turret yaw
--   computer right = Create Rotation Speed Controller for gun elevation
--   computer bottom = wireless modem from the turret computer
--
-- Redstone levels are 0..15. The two directions are subtracted, so if both
-- are active at once they cancel instead of commanding both directions.

local CONFIG = {
  transmissionControllerSide = "back",
  yawControllerSide = "left",
  pitchControllerSide = "right",
  forwardInputSide = "front",
  reverseInputSide = "top",
  wirelessModemSide = "bottom",
  aimProtocol = "tank_speed_stab",
  aimTimeoutTicks = 4,

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

local yawController = peripheral.wrap(CONFIG.yawControllerSide)
local pitchController = peripheral.wrap(CONFIG.pitchControllerSide)
if not yawController or type(yawController.setTargetSpeed) ~= "function"
  or not pitchController or type(pitchController.setTargetSpeed) ~= "function" then
  error("Missing yaw or pitch Create Rotation Speed Controller. Check left/right sides.")
end

local commandedRPM = 0
local commandedYawRPM, commandedPitchRPM = 0, 0

local function clamp(value, low, high)
  return math.max(low, math.min(high, value))
end

local function round(value)
  if value < 0 then return math.ceil(value - 0.5) end
  return math.floor(value + 0.5)
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

local function setAimRPM(controller, rpm, previous)
  rpm = round(rpm)
  if rpm ~= previous then controller.setTargetSpeed(rpm) end
  return rpm
end

local function approach(current, target, maximumChange)
  if target > current then
    return math.min(current + maximumChange, target)
  end
  return math.max(current - maximumChange, target)
end

local function draw(forward, reverse, input, targetRPM, timeoutTicks)
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
  print(string.format("Yaw / pitch:     %+d / %+d RPM", commandedYawRPM, commandedPitchRPM))
  print(string.format("Aim link ticks:  %d", timeoutTicks))
  print("")
  print("Both inputs cancel each other.")
end

local function updateDrive()
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
  return forward, reverse, driveInput, targetRPM
end

local function run()
  -- Synchronize the controller with our cached value before reading input.
  -- This prevents an old command from a previous program continuing at boot.
  transmission.setTargetSpeed(0)
  yawController.setTargetSpeed(0)
  pitchController.setTargetSpeed(0)
  commandedRPM = 0
  commandedYawRPM, commandedPitchRPM = 0, 0
  rednet.open(CONFIG.wirelessModemSide)

  local timeoutTicks = CONFIG.aimTimeoutTicks + 1

  -- Keep reception separate from the fixed-tick drive loop. A continuous
  -- stream of turret packets must never postpone local throttle updates.
  local targetYawRPM, targetPitchRPM = 0, 0
  local function receiveAimCommands()
    while true do
      local _, command = rednet.receive(CONFIG.aimProtocol)
      if type(command) == "table" and command.kind == "aim_rpm" then
        targetYawRPM = command.yawRPM or 0
        targetPitchRPM = command.pitchRPM or 0
        timeoutTicks = 0
      end
    end
  end

  local function fixedTickLoop()
    while true do
      local forward, reverse, driveInput, targetRPM = updateDrive()
      if timeoutTicks > CONFIG.aimTimeoutTicks then
        commandedYawRPM = setAimRPM(yawController, 0, commandedYawRPM)
        commandedPitchRPM = setAimRPM(pitchController, 0, commandedPitchRPM)
      else
        commandedYawRPM = setAimRPM(yawController, targetYawRPM, commandedYawRPM)
        commandedPitchRPM = setAimRPM(pitchController, targetPitchRPM, commandedPitchRPM)
      end
      timeoutTicks = timeoutTicks + 1
      draw(forward, reverse, driveInput, targetRPM, timeoutTicks)
      sleep(CONFIG.loopSeconds)
    end
  end

  parallel.waitForAny(receiveAimCommands, fixedTickLoop)
end

local ok, err = pcall(run)
-- Never leave the drive transmission spinning if this program ends.
pcall(function() transmission.setTargetSpeed(0) end)
pcall(function() yawController.setTargetSpeed(0) end)
pcall(function() pitchController.setTargetSpeed(0) end)

if not ok then
  term.setCursorPos(1, 10)
  term.clearLine()
  if err == "Terminated" then
    print("Driving stopped: transmission set to 0 RPM.")
  else
    error(err, 0)
  end
end
