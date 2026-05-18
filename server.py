import socket
import threading

HOST = "0.0.0.0"
PORT = 8080

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen()

print("Server listening on port 8080...")

clients = []  # list of all connected clients


# Handle each client connection
def handle_client(conn, addr):
    print("New connection:", addr)
    buffer = ""
    values = []

    try:
        while True:
            data = conn.recv(1024).decode()

            if not data:
                break

            buffer += data

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if line:
                    try:
                        values.append(float(line))

                        if len(values) == 3:
                            x, y, z = values
                            msg = f"Position -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
                            print(msg)

                            # send to ALL connected clients
                            broadcast(msg + "\n")

                            values = []

                    except ValueError:
                        print("Bad data:", line)

    except:
        pass

    print("Disconnected:", addr)
    clients.remove(conn)
    conn.close()


# Send message to all clients
def broadcast(message):
    for c in clients:
        try:
            c.send(message.encode())
        except:
            pass


# Accept multiple clients
while True:
    conn, addr = server.accept()
    clients.append(conn)

    thread = threading.Thread(target=handle_client, args=(conn, addr))
    thread.start()