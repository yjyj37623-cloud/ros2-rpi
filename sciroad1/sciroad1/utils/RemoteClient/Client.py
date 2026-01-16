import asyncio
import websockets
from RemoteDevice import RemoteDevice

Aliyun_ip = '47.49.168.126'


def on_message(ws, message):
    print(message)

def on_error(ws, error):
    print(error)

def on_close(ws):
    print("WebSocket connection closed")

def on_open(ws):
    def run(*args):
        while True:
            text = input('Enter message: ')
            ws.send(text)

    run()

if __name__ == "__main__":
    websockets.enableTrace(True)
    ws = websockets.WebSocketApp("ws://<SERVER_ADDRESS>:<PORT>/chat",
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
    ws.on_open = on_open
    ws.run_forever()

