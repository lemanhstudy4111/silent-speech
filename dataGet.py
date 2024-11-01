# code for getting data from IMU to computer 


import serial

# Open the serial port
port = serial.Serial('COM8', 115200, timeout=1)


with open('test_resultWord/r.txt', 'w') as file:

    # Function to read from serial port
    def read_ser(num_char=1):
        string = port.read(num_char)
        return string.decode()

    # Function to write to serial port
    def write_ser(cmd):
        port.write(cmd)

    # Write a command to the serial port
    write_ser('a'.encode())

    # Read and save serial data into file
    x = read_ser(1000)
    while x:
        file.write(x)  # Save data to the file
        x = read_ser(1000)