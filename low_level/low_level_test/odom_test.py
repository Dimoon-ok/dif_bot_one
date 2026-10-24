import serial
import serial.tools.list_ports
import re
import time

# Путь к порту (ваш путь может отличаться)
SERIAL_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0"
BAUD_RATE = 115200

# Регулярное выражение для поиска значений
pattern = re.compile(r'POS X=([-\d.]+) Y=([-\d.]+) Th=([-\d.]+)')

def open_serial(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    # Ожидание инициализации Serial
    time.sleep(2)
    ser.flushInput()
    ser.write(b"PING\n")
    print(ser.readline().decode())
    return ser

def read_robot_position(ser):
    while True:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("POS"):
                match = pattern.search(line)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    theta = float(match.group(3))
                    print(f"Положение робота → X: {x:.2f} мм | Y: {y:.2f} мм | Угол: {theta:.2f} рад")
        except KeyboardInterrupt:
            print("\nОстановка чтения данных.")
            break
        except Exception as e:
            print(f"Ошибка чтения данных: {e}")

if __name__ == "__main__":
    print(f"Подключаемся к роботу через порт {SERIAL_PORT}")
    ser = open_serial(SERIAL_PORT, BAUD_RATE)
    print("Чтение положения робота начато. Для выхода нажмите Ctrl+C\n")
    read_robot_position(ser)
    ser.close()
