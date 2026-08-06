-- Controlled fault-isolation test for the Clockwork turret transmission.
--
-- Run only after replacing any broken parts. This program deliberately makes
-- rapid changes, so begin with MODE = "resistor" and TEST_SECONDS = 5.
-- Press Ctrl+T at any time: both resistors stop and both gearshifts unpower.

local CONFIG = {
  -- "resistor": rapidly changes only the resistor level; gearshift stays put.
  -- "gearshift": rapidly reverses only the gearshift; resistor speed is fixed.
  -- "both": changes both, reproducing the most demanding controller case.
  mode = "resistor",

  -- Test a single axis first, so a failure identifies the responsible path.
  axis = "horizontal", -- "horizontal" or "vertical"

  TEST_SECONDS = 5,
  LOOP_SECONDS = 0.05,      -- one game tick
  CHANGE_EVERY_TICKS = 1,   -- 1 = change every game tick

  -- Resistor level to use for maximum transmission in gearshift-only mode.
  -- Clockwork resistor: 0 = full transmission, 15 = stopped.
  FIXED_SPEED_LEVEL = 0,

  -- Levels cycled in resistor/both mode. This intentionally includes the two
  -- extremes; remove 0 if you want a less severe resistor-only test.
  RESISTOR_LEVELS = { 15, 10, 5, 0, 5, 10 },

  horizontalResistorSide = "left",
  verticalResistorSide = "right",
  horizontalGearshiftSide = "left", -- side of the Redstone Relay
  verticalGearshiftSide = "right",  -- side of the Redstone Relay
}

local function stop(relay)
  redstone.setAnalogOutput(CONFIG.horizontalResistorSide, 15)
  redstone.setAnalogOutput(CONFIG.verticalResistorSide, 15)
  if relay then
    relay.setOutput(CONFIG.horizontalGearshiftSide, false)
    relay.setOutput(CONFIG.verticalGearshiftSide, false)
  end
end

local function selectedSides()
  if CONFIG.axis == "vertical" then
    return CONFIG.verticalResistorSide, CONFIG.verticalGearshiftSide
  end
  return CONFIG.horizontalResistorSide, CONFIG.horizontalGearshiftSide
end

local function draw(relay, tick, resistorLevel, reversed)
  term.clear()
  term.setCursorPos(1, 1)
  print("Turret transmission test")
  print("Mode: " .. CONFIG.mode .. "  Axis: " .. CONFIG.axis)
  print("Tick: " .. tick)
  print("Resistor: " .. resistorLevel .. "/15")
  print("Gearshift: " .. (reversed and "reverse" or "normal"))
  print("Ctrl+T stops safely")
end

local function run()
  assert(CONFIG.mode == "resistor" or CONFIG.mode == "gearshift"
      or CONFIG.mode == "both", "Invalid mode")
  local relay = assert(peripheral.find("redstone_relay"),
    "No redstone_relay found on the wired-modem network.")
  local resistorSide, gearshiftSide = selectedSides()
  local totalTicks = math.max(1, math.floor(CONFIG.TEST_SECONDS / CONFIG.LOOP_SECONDS))
  local sequenceIndex, reversed = 1, false

  -- Keep the untested axis stopped and in its normal gearshift direction.
  stop(relay)

  for tick = 1, totalTicks do
    local shouldChange = tick == 1
      or (tick - 1) % CONFIG.CHANGE_EVERY_TICKS == 0
    if shouldChange then
      if CONFIG.mode == "resistor" or CONFIG.mode == "both" then
        sequenceIndex = sequenceIndex % #CONFIG.RESISTOR_LEVELS + 1
      end
      if CONFIG.mode == "gearshift" or CONFIG.mode == "both" then
        reversed = not reversed
      end
    end

    local level = CONFIG.FIXED_SPEED_LEVEL
    if CONFIG.mode == "resistor" or CONFIG.mode == "both" then
      level = CONFIG.RESISTOR_LEVELS[sequenceIndex]
    end

    -- Do not rewrite an unchanged signal: each mode should test actual state
    -- changes, not the cost of repeatedly calling the CC redstone API.
    if shouldChange then
      redstone.setAnalogOutput(resistorSide, level)
      relay.setOutput(gearshiftSide, reversed)
    end
    draw(relay, tick, level, reversed)
    sleep(CONFIG.LOOP_SECONDS)
  end
end

local ok, message = xpcall(run, function(err)
  if tostring(err) == "Terminated" then return "__terminated__" end
  return tostring(err) .. "\n" .. debug.traceback()
end)

stop(peripheral.find("redstone_relay"))
if not ok and message ~= "__terminated__" then
  term.clear()
  term.setCursorPos(1, 1)
  print("Test stopped with an error:")
  print(message)
end
