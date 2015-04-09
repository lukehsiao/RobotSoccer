from motor_control import roboclaw

reading = roboclaw.readmainbattery()
voltage = (1.0 * reading)/10.0
print "Battery Level: %.1f V" %voltage
if voltage < 10.0:
  print "WARNING: Battery low, change soon!"
