#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Connected")
# Run the asyncio loop
asyncio.run(run())