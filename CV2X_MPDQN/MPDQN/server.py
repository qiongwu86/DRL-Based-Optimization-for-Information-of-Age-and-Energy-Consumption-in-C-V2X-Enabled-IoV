import socket

# 设置服务器IP地址和端口
ip = '127.0.0.1'
port = 1234

# 创建socket对象
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定IP地址和端口
server_socket.bind((ip, port))

# 开始监听
server_socket.listen(1)

print('等待客户端连接...')

# 接受连接
client_socket, addr = server_socket.accept()
print('客户端已连接:', addr)

try:
    while True:
        # 接收客户端消息
        data = client_socket.recv(1024)
        if not data:
            break
        print('接收到的数据:', data.decode())

        # 处理数据
        processed_data = "Hello from Python!"

        # 发送回传消息给客户端
        client_socket.send(processed_data.encode())

except Exception as e:
    print(str(e))

# 关闭连接
client_socket.close()
server_socket.close()

