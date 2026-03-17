import rtde_control
import rtde_receive
import rtde_io
import socket

print(dir(rtde_control))

print(dir(rtde_control.RTDEControlInterface))

# hostip = "192.168.0.39"
# toolioport = 54321
# rtde_frequency = 500.0
# rtde_c = rtde_control.RTDEControlInterface(hostip,)
# rtde_r = rtde_receive.RTDEReceiveInterface(hostip)
# rtde_i = rtde_io.RTDEIOInterface(hostip)


# q = rtde_r.getActualQ()
# print(q)
# t = rtde_c.getJointTorques()
# print(t)
# J = rtde_c.getJacobian()
# print(J)

# import minimalmodbus

# # Use the virtual port created by socat
# instrument = minimalmodbus.Instrument(
#     "/tmp/ttyUR ", 1
# )  # port name, slave address (1)
# instrument.serial.baudrate = 115200

# # Example: Read a holding register (address 200)
# data = instrument.read_register(200, 1)
# print(f"Device Data: {data}")
