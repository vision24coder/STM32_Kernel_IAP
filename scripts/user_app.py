import fcntl
import array
import os
import time

IOC_MAGIC = ord('k')
def _IO(magic, nr):
    return (magic << 8) | nr
def _IOW(magic, nr, size):
    return (0x40000000 | (size << 16) | (magic << 8) | nr)
def _IOR(magic, nr, size):
    return (0x80000000 | (size << 16) | (magic << 8) | nr)

# --- IOCTL COMMANDS ---
STM32_IOC_SET_BAUD   = _IOW(IOC_MAGIC, 1, 4)
STM32_IOC_CLR_BUF    = _IO(IOC_MAGIC, 2)
STM32_IOC_GET_STAT   = _IOR(IOC_MAGIC, 3, 4)
STM32_IOC_GET_LEVEL  = _IOR(IOC_MAGIC, 4, 4)
STM32_IOC_RESET_PTR  = _IO(IOC_MAGIC, 5)
STM32_IOC_ENTER_BOOT = _IO(IOC_MAGIC, 6)

ACK = b'\x06'

def upload_firmware(fd, bin_file_path):
    if not os.path.exists(bin_file_path):
        print(f"Error: File {bin_file_path} not found.")
        return

    file_size = os.path.getsize(bin_file_path)
    print(f"Binary size: {file_size} bytes")
    CHUNK_SIZE = 256
    count = 0
    try:
        with open(bin_file_path, "rb") as f:
            sent_bytes = 0
            start_time = time.time()

            print("Starting Upload... Press Reset on STM32 if it hangs.")

            while sent_bytes < file_size:
                
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                
                if len(chunk) < CHUNK_SIZE:
                    chunk += b'\x00' * (CHUNK_SIZE - len(chunk))
                    

                # Use the existing fd passed from main
                os.write(fd, chunk)
                # Wait for ACK from STM32
                print('sent byte : ',chunk,'\n\n')
                response = os.read(fd, 1)
                
                
                if response == ACK:
                    print(count,'\n')
                    count+=1
                    sent_bytes += len(chunk)
                    #print(sent_bytes,"sent: \n")
                    if (sent_bytes % 100) == 0:
                        print(f"Progress: {sent_bytes}/{file_size} bytes", end='\r')
                else:
                    print(f"\nError: Received {response.hex()} instead of ACK.")
                    break

            duration = time.time() - start_time
            print(f"\nUpload Complete! Sent {sent_bytes} bytes in {duration:.2f} seconds.")
            # REMOVED os.close(fd) from here so the menu keeps working

    except Exception as e:
        print(f"\nFailed to upload: {e}")

def main():
    DEVICE_PATH = "/dev/stm32_ctrl"
    BIN_FILE = "Project_application.bin"
    
    try:
        fd = os.open(DEVICE_PATH, os.O_RDWR)
        print(f"Successfully opened {DEVICE_PATH}")
        
        while True:
            print("\n--- STM32 Python Controller ---")
            print("1. Send String")
            print("2. Read Buffer")
            print("3. Get Buffer Fill Level (IOCTL)")
            print("4. Set Baud Rate (IOCTL)")
            print("5. Enter Bootloader (IOCTL)")
            print("6. Get Status")
            print("7. Update")
            print("8. Exit")
               
            choice = input("Select an option: ")

            if choice == '1':
                msg = input("Enter message: ") + '\n'
                os.write(fd, msg.encode())
                print("Data sent.")                
                
            elif choice == '2':
                data = os.read(fd, 1024)
                if data:
                    print(f"Received from STM32: {data.decode(errors='ignore')}")
                else:
                    print("No data available.")

            elif choice == '3':
                arg = array.array('i', [0])
                fcntl.ioctl(fd, STM32_IOC_GET_LEVEL, arg, True)
                print(f"Bytes waiting in Kernel Buffer: {arg[0]}")

            elif choice == '4':
                baud = int(input("Enter Baud (9600 or 115200): "))
                arg = array.array('i', [baud])
                fcntl.ioctl(fd, STM32_IOC_SET_BAUD, arg)
                print(f"Baud rate command sent for {baud}")

            elif choice == '5':
                fcntl.ioctl(fd, STM32_IOC_ENTER_BOOT, 1)
                print("Sent ENTER_BOOT command.")
                #upload_firmware(fd, BIN_FILE)

            elif choice == '6':
                msg = "STATUS" + '\n'
                os.write(fd, msg.encode())
                data = os.read(fd, 1024)
                print(f"Received from STM32: {data.decode(errors='ignore')}")

            elif choice == '7':
                # First trigger the bootloader jump
                print("Triggering Bootloader...")
                #fcntl.ioctl(fd, STM32_IOC_ENTER_BOOT)
                print("Sent ENTER_BOOT command.")
                time.sleep(0.5)
                upload_firmware(fd, BIN_FILE)

            elif choice == '8':
                os.close(fd)
                print("Closed. Goodbye!")
                break
                
    except PermissionError:
        print("Error: Permission denied. Try 'sudo'.")
    except FileNotFoundError:
        print(f"Error: Device {DEVICE_PATH} not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()