< envPaths

# For use with https://github.com/epics-motor/motorMotorSim.

dbLoadDatabase("$(TOP)/dbd/motorSim.dbd")
motorSim_registerRecordDeviceDriver(pdbbase)

motorSimCreateController("motorSim1", 2)
# motorSimConfigAxis(port, axis, lowLimit, highLimit, home, start)
motorSimConfigAxis("motorSim1", 0, 20000, -20000, 0, 0)
motorSimConfigAxis("motorSim1", 1, 20000, -20000, 0, 0)
dbLoadTemplate("dither-demo.substitutions")

# Load the `:DITHER` record for each axis.
dbLoadRecords("$(MOTORDITHER)/db/motorDither.db", "P=DEMO:m1,PREC=3")
dbLoadRecords("$(MOTORDITHER)/db/motorDither.db", "P=DEMO:m2,PREC=3")

iocInit

date
