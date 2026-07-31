from machine import Pin, UART
import time

uart = UART(0, baudrate=19200, tx=Pin(0), rx=Pin(1))

def compute_crc8_atm(datagram):
    crc = 0
    for byte in datagram:
        for _ in range(8):
            if (crc >> 7) ^ (byte & 0x01):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            byte = byte >> 1
    return crc

print("====================================")
print("  TMC2209 UART ADDRESS SCANNER")
print("====================================\n")

for addr in range(4):
    print(f"Testing Address {addr}...")
    
    # Build request for GCONF register (0x00)
    req = bytearray([0x55, addr, 0x00, 0])
    req[3] = compute_crc8_atm(req[:-1])
    
    # Clear buffer
    while uart.any():
        uart.read()
        
    # Send request
    uart.write(req)
    time.sleep(0.1) # Wait for reply
    
    if uart.any():
        resp = uart.read()
        if len(resp) == 4:
            print(f"   ❌ Address {addr}: Echo only (No reply)")
        elif len(resp) >= 12:
            print(f"   ✅ SUCCESS! Driver found at Address {addr}!")
            print(f"   Raw Data: {[hex(b) for b in resp]}\n")
        else:
            print(f"   ⚠️ Address {addr}: Corrupted data ({len(resp)} bytes)")
    else:
        print(f"   ❌ Address {addr}: Absolute Silence (Wire broken or grounded)")

print("\nScan Complete.")