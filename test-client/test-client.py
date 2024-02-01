import websocket
import time

def on_message(ws, message):
    print(f"Received: {message}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    print("Closed")

def on_open(ws):
    print("Connected")
    # You can send a message if needed

if __name__ == "__main__":
    ws = websocket.WebSocketApp("ws://localhost:1234",
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open

    # Run the WebSocket client in a separate thread
    ws.run_forever()
