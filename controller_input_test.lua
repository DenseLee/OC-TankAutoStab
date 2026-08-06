-- Controller / Drive-By-Wire input diagnostic.
--
-- This DOES NOT move the tank. It only shows the analogue redstone levels
-- arriving at the computer and at the Redstone Relay.
--
-- Wiring being tested:
--   computer top  = horizontal +X input
--   computer back = horizontal -X input
--   relay top     = elevation +Y input
--   relay bottom  = elevation -Y input
--
-- Each source must be an ordinary analogue redstone output facing the listed
-- face. A redstone level has no sign itself: the two faces per axis provide
-- the positive and negative directions.

local LOOP_SECONDS = 0.05

local function bar(level)
  return string.rep("#", level) .. string.rep(".", 15 - level)
end

local function signedAxis(positive, negative)
  -- Returns -1..1. If both inputs are present, they cancel proportionally.
  return (positive - negative) / 15
end

local function axisDirection(value, negativeName, positiveName)
  if math.abs(value) < 0.05 then return "center" end
  if value > 0 then return positiveName end
  return negativeName
end

local function drawLine(y, label, level)
  term.setCursorPos(1, y)
  term.clearLine()
  write(string.format("%-18s %2d [%s]", label, level, bar(level)))
end

local function run()
  local relay = peripheral.find("redstone_relay")
  if not relay then
    error("No CC:Tweaked Redstone Relay found on the wired modem network.")
  end

  while true do
    -- Inputs directly touching the computer.
    local horizontalPositive = redstone.getAnalogInput("top")
    local horizontalNegative = redstone.getAnalogInput("back")

    -- Inputs touching the Redstone Relay. These names are relative to the
    -- relay block, not relative to the computer.
    local verticalPositive = relay.getAnalogInput("top")
    local verticalNegative = relay.getAnalogInput("bottom")

    local horizontal = signedAxis(horizontalPositive, horizontalNegative)
    local vertical = signedAxis(verticalPositive, verticalNegative)

    term.setCursorPos(1, 1)
    term.clear()
    print("Controller input test - Ctrl+T to stop")
    print("Move one controller direction at a time.")
    print("")
    drawLine(4, "Computer top (+X)", horizontalPositive)
    drawLine(5, "Computer back (-X)", horizontalNegative)
    drawLine(6, "Relay top (+Y)", verticalPositive)
    drawLine(7, "Relay bottom (-Y)", verticalNegative)
    print("")
    print(string.format("Horizontal: %+.2f (%s)", horizontal,
      axisDirection(horizontal, "-X", "+X")))
    print(string.format("Elevation:  %+.2f (%s)", vertical,
      axisDirection(vertical, "-Y", "+Y")))
    print("")
    print("Expected: idle is 0/0. One direction rises toward 15.")
    print("Both directions nonzero means two outputs are active together.")

    sleep(LOOP_SECONDS)
  end
end

local ok, err = pcall(run)
term.setCursorPos(1, 14)
term.clearLine()
if err == "Terminated" then
  print("Input test stopped.")
else
  error(err, 0)
end
