#!/usr/bin python3
# Copyright (c) 2025, ANTOBOT LTD.
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description: Provides a class to handle streaming a preivew of the Pi camera
# # # Interfaces:       Imported by camera_record.py

# Contacts: Authors:    james.bennett@antobot.ai
#           Owner:      james.bennett@antobot.ai


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import asyncio
import json
import uuid

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
import aiohttp_cors


class PreviewStreamer:
    """
    Provides a web server that will setup and run the preview stream.
    Provides an `/offer` route on port 8080.
    """
    def __init__(self, track_reference):
        """
        Initialise the PreviewStreamer class.

        Args:
            track_reference (CameraStreamTrack): must be a reference to an object that inherits from aiortc MediaStreamTrack
            
        """
        self.pcs = set()
        
        # Reference to CameraStreamTrack provided by camera
        self.track_reference = track_reference


    async def offer(self, request):
        params = await request.json()
        cam_id = request.match_info.get("cam_id")
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        self.pcs.add(pc)

        def log_info(msg):
            print(pc_id + " " + msg)

        log_info(f"Created connection for {request.remote}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            log_info("Connection state is {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
                self.pcs.discard(pc)
        track_reference = self.track_reference.get(cam_id)
        if track_reference:
            track_reference.set_size(params["width"], params["height"])
            pc.addTrack(track_reference)
            log_info(f"Added track for {cam_id}")
        else:
            log_info(f"No track found for {cam_id}")
            
        # handle offer
        await pc.setRemoteDescription(offer)

        # send answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps(
                {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
            ),
            headers={
                "X-Custom-Server-Header": "Custom data",
            }
        )


    async def on_shutdown(self, app):
        # close peer connections
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()


    def run(self):
        """
        Run the server to provide the /cam_id resource and handle requests.
        """
               
        app = web.Application()
        app.on_shutdown.append(self.on_shutdown)

        # use "*" to allow all (we don't know their ip address)
        cors = aiohttp_cors.setup(app, defaults={
            "*": aiohttp_cors.ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                )
        })
        resource = cors.add(app.router.add_resource("/{cam_id}"))
        cors.add(resource.add_route("POST", self.offer))

        host = '0.0.0.0'
        port = 8080

        # app event loop
        web.run_app(
            app, access_log=None, host=host, port=port,
        )
