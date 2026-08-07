-- Create Speed Controller peripheral diagnostic.
--
-- Run this before the motion test. It only lists peripherals; it does not
-- change any mechanical speed.

local function join(values)
  if not values or #values == 0 then return "(none)" end
  return table.concat(values, ", ")
end

term.clear()
term.setCursorPos(1, 1)
print("Peripheral check")
print("Ctrl+T to stop after reading")
print("")

local names = peripheral.getNames()
if #names == 0 then
  print("No peripherals found.")
  print("Check the wired modem, cable, and the controller's chunk.")
  return
end

for _, name in ipairs(names) do
  local types = { peripheral.getType(name) }
  local methods = peripheral.getMethods(name) or {}

  print(name)
  print("  type:    " .. join(types))
  print("  methods: " .. join(methods))
  print("")
end

print("Look for Create_RotationSpeedController.")
