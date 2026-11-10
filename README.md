# EPICS motor dither record

EPICS IOC support for oscillating an existing motor record around its saved
user-coordinate setpoint. Each motor needs one instance of the database.
The motor and the dither records **must be in the same IOC**.

## Build and integrate

1) Create a `RELEASE.local` with `EPICS_BASE` and `MOTORDITHER` set to this directory.

2) Add the dbd and libs to your IOC:

```make
myIoc_DBD += motorDither.dbd
myIoc_LIBS += motorDither
```

3) Before `iocInit`, after loading the motor record, load:

```iocsh
dbLoadRecords("$(MOTORDITHER)/db/motorDither.db", "P=IOC:m1")
```

Replace `IOC:m1` with your EPICS motor record.

## Operation

One `dither` record per motor, named `$(P):DITHER`.

| PV | Meaning |
| --- | --- |
| `$(P):DITHER` | `"Enabled"` starts dithering; `"Disabled"` stops and returns to the saved center. |
| `$(P):DITHER.PLUS` | Distance above center in motor user units, at least 0 (negative writes clamp to 0). |
| `$(P):DITHER.MINUS` | Distance below center in motor user units, at least 0 (negative writes clamp to 0). |
| `$(P):DITHER.VELO` | Positive dither velocity in motor user units per second. |
| `$(P):DITHER.CEN` | Saved motor `.VAL` at enable time (read-only). |
| `$(P):DITHER.STATE` | Menu: Idle, Dithering, Returning, Fault (read-only). |
| `$(P):DITHER.ERR` | Fault description, empty when acknowledged (read-only). |
| `$(P):DITHER.ACK` | Write 1 to acknowledge a fault; the record clears it again. |

## Important Notes

* Acceleration is the same as the motor acceleration, e.g `.ACCS` of the motor.
* Once dithering, the motor will always move back to the center and dither about it. If a different `.VAL` is put to during dithering, the motor will move towards it, but then move back to `:DITHER.CEN` and continue dithering as usual. To move to a new position dithering must be stopped.
* **Use soft limits!** Soft limits are respected by the dither record. If the dither is on the boundary then the movement will take place up until the soft limit, but not move over. If the soft limits change to inside the dither radius during dithering then a fault will be made and dithering will stop.

## Simulator example

The example directory contains:

* [st.cmd`](sim_motor_example/st.cmd) for starting two simulation axes.
* [`plot_trajectory.py`](sim_motor_example/plot_trajectory.py) for live plotting the simulation axes trajectory.
  - [`requirements.txt`](sim_motor_example/requirements.txt) for live plotting.
* [`top.bob`](sim_motor_example/top.bob) for setting values through phoebus.


Simply run:

``` bash
cd sim_motor_example/
pip -r requirements.txt
$MOTOR_SIM_DIRECTORY/iocs/motorSimIOC/bin/linux-x86_64/motorSim st.cmd
python plot_trajectory.py
```
