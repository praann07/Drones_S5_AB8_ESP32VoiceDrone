import asyncio
import json
from mavsdk import System
import websockets

STREAM_PORT = 14535
INTERNAL_GRPC_PORT = 50053
CONNECTED_CLIENTS = set()


async def register_client(websocket):
  CONNECTED_CLIENTS.add(websocket)
  try:
    await websocket.wait_closed()
  finally:
    CONNECTED_CLIENTS.remove(websocket)


async def telemetry_streamer():
  drone = System(port=INTERNAL_GRPC_PORT)
  await drone.connect(system_address=f"udpin://0.0.0.0:{STREAM_PORT}")

  async def stream_attitude():
    async for att in drone.telemetry.attitude_euler():
      data = json.dumps({
          "type": "attitude",
          "roll": att.roll_deg,
          "pitch": att.pitch_deg,
          "yaw": att.yaw_deg,
      })
      if CONNECTED_CLIENTS:
        await asyncio.gather(
            *[client.send(data) for client in CONNECTED_CLIENTS]
        )

  async def stream_position():
    async for pos in drone.telemetry.position_velocity_ned():
      data = json.dumps({
          "type": "position",
          "north": pos.position.north_m,
          "east": pos.position.east_m,
          "down": pos.position.down_m,
      })
      if CONNECTED_CLIENTS:
        await asyncio.gather(
            *[client.send(data) for client in CONNECTED_CLIENTS]
        )

  await asyncio.gather(stream_attitude(), stream_position())


async def main():
  server = await websockets.serve(register_client, "0.0.0.0", 8765)
  print("Foxglove WebSocket Live Streamer running on ws://localhost:8765")
  await asyncio.gather(server.wait_closed(), telemetry_streamer())


if __name__ == "__main__":
  asyncio.run(main())
