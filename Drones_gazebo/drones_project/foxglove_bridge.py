import asyncio
import json
import math
from pymavlink import mavutil
from foxglove_websocket.server import FoxgloveServer

async def main():
    # Listen on dedicated port 14540 to avoid QGC (14550) collisions
    mav = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
    print("Bridge connected to MAVLink stream on port 14540...")

    async with FoxgloveServer("0.0.0.0", 8765, "PX4-Bridge") as server:
        channel_id = await server.add_channel({
            "topic": "/mavlink/attitude",
            "encoding": "json",
            "schemaName": "Attitude",
            "schema": json.dumps({
                "type": "object",
                "properties": {
                    "roll": {"type": "number"},
                    "pitch": {"type": "number"},
                    "yaw": {"type": "number"}
                }
            })
        })

        print("Foxglove server running on ws://0.0.0.0:8765")
        
        while True:
            msg = mav.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                data = {
                    "roll": math.degrees(msg.roll),
                    "pitch": math.degrees(msg.pitch),
                    "yaw": math.degrees(msg.yaw)
                }
                timestamp_ns = int(asyncio.get_event_loop().time() * 1e9)
                await server.send_message(
                    channel_id, 
                    timestamp_ns, 
                    json.dumps(data).encode('utf-8')
                )
            await asyncio.sleep(0.01)

if __name__ == "__main__":
    asyncio.run(main())
