#!/usr/bin/env python3
"""
Simplified voice connection test
"""

import asyncio
from mavsdk import System

async def test_connection():
    print("Testing connection to PX4...")
    
    drone = System()
    
    # Use port 14531 (PX4's remote port)
    port = 14531
    print(f"Connecting to 127.0.0.1:{port}...")
    
    try:
        await drone.connect(system_address=f"udpout://127.0.0.1:{port}")
        print("✓ Connection initiated")
        
        # Wait for connection with timeout
        print("Waiting for connection state...")
        async for state in drone.core.connection_state():
            print(f"  State: connected={state.is_connected}")
            if state.is_connected:
                print("✓ Connected to PX4!")
                
                # Try to get some telemetry
                try:
                    print("Requesting position...")
                    async for pos in drone.telemetry.position():
                        print(f"Position: {pos}")
                        break
                except Exception as e:
                    print(f"Could not get position: {e}")
                break
                
    except asyncio.TimeoutError:
        print("✗ Connection timeout")
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    asyncio.run(test_connection())
