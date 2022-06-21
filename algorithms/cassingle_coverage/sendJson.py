import asyncio
import websockets
uri = 'ws://106.12.220.82:8088/ws'

async def Send(uri, message):
    async with websockets.connect(uri) as websocket:
        await websocket.send(message)

def SendJson(json):
    asyncio.get_event_loop().run_until_complete(Send(uri, json))