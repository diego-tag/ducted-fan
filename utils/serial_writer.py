import serial

def main():
    try:
        # Configure serial port
        port_name = 'COM9' # For Windows use COM ports (listed in device manager)
        baud_rate = 115200         # Ensure this value matches trasmitter baud rate

        ser = serial.Serial(port_name, baud_rate, timeout=1)
        ser.dtr = False
        ser.rts = False
        print(f"Listening on {port_name} at {baud_rate} bps...")


        while True:
            # START SENDING SECTION - comment/delete it if you want
            data_to_send = input("> Send to serial: ")
            data_with_newline = data_to_send + "\n"


            ser.write(data_with_newline.encode())  # Send the string with the newline

            print(f"Sent: {data_with_newline.encode()}")
            # END SENDING PART

            # START READING SECTION - comment/delete it if you want
            # line = ser.readline()
            #
            # if line:
            #     print(line)
            # END READING SECTION

    except serial.SerialException as e:
        print(f"Serial error (check port name is correct and cable is connected): {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if "ser" in locals() and ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == '__main__': main()