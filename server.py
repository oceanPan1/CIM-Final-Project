import socket

HOST = "0.0.0.0"
PORT = 8080

# Create socket
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("Listening on port 8080...")

# Accept one client
conn, addr = server.accept()
print("Connected from", addr)

buffer = ""
values = []

try:
    while True:
        data = conn.recv(1024).decode()

        if not data:
            print("Client disconnected")
            break

        buffer += data

        # Process complete lines
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()

            if line:
                try:
                    values.append(float(line))

                    # Every 3 values = x, y, z
                    if len(values) == 3:
                        x, y, z = values
                        print(f"Position -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
                        values = []

                except ValueError:
                    print("Invalid data:", line)

except Exception as e:
    print("Error:", e)

finally:
    conn.close()
    server.close()
