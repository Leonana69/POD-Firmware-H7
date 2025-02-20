import socket
import pickle
from PIL import Image
import io

# Client setup
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_name = "10.0.0.177"
port = 80

client_socket.connect((host_name, port))
print('Connected to the server...')

try:
    # Receive image data
    data = b''
    while True:
        packet = client_socket.recv(8192)
        if not packet: 
            break
        data += packet

    print(len(data))
    image = Image.frombytes('RGB', (640, 480), data)  # 'L' for grayscale, adjust mode and size as needed
    image.show()
except Exception as e:
    print(f'Error receiving data: {e}')

client_socket.close()