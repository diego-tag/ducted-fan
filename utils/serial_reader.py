import serial

def main():
    try:
        port_name = 'COM9' # For Windows use COM ports (listed in device manager)
        baud_rate = 115200         # Ensure this value matches trasmitter baud rate

        ser = serial.Serial(port_name, baud_rate, timeout=1)
        print(f"Listening on {port_name} at {baud_rate} bps...")

        while True:
            line = ser.readline()

            if line:
                print(line.decode('utf-8', errors='replace').strip())
    except serial.SerialException as e:
        print(f"Serial error (check port name is correct and cable is connected): {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if "ser" in locals() and ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == '__main__': main()