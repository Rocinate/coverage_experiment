import asyncio
import websockets
async def hello(uri):
    async with websockets.connect(uri) as websocket:
        await websocket.send("hello world")
        print("< HELLO WORLD")
        while True:
            recv_text = await websocket.recv()
            print("> {}".format(recv_text))
asyncio.get_event_loop().run_until_complete(hello('ws://106.12.220.82:8088/ws'))
