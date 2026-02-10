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
# import logging

# # Set logging to debug to see internal library errors
# logging.basicConfig(level=logging.DEBUG)

import asyncio
import json
import uuid
import math
import numpy as np

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCRtpSender
import aiohttp_cors
from av import VideoFrame

class CameraStreamTrack(VideoStreamTrack):
    """
    A video track that captures frames from a callback to the latest pi camera request
    """
    def __init__(self, read_array_callback, frame_dims):
        """
        Initialise a CameraStreamTrack

        Args:
            read_array_callback (callback): callback to read frame from the camera
            frame_dims (tuple): dimensions of camera frame (height, width)   
        """
        super().__init__()
        
        # Callback to read frame from the camera
        self.read_frame = read_array_callback
        
        # Dimensions that the camera records at
        # (height, width) (assuming portrait after a 90 deg rotation)
        self.camera_dims = frame_dims
        
        # Placeholder for stream dimensions, calculated and updated when stream is requested
        # (height, width)
        self.stream_dims = self.camera_dims
        

    def set_size(self, width, height):
        """
        Calculates and saves the appropriate stream size given the dimensions of the container on the webpage.

        Args:
            width (int): maximum width in px permitted for the stream 
            height (int): maximum height in px permitted for the stream       
        """
        # container dims on webpage
        hc = height
        wc = width

        # camera frame dims
        hf = self.camera_dims[0]
        wf = self.camera_dims[1]

        # set stream size to limiting height/width
        if hf/hc > wf/wc:
            height = hc
            width = math.floor(wf/hf * height)
        else:
            width = wc
            height = math.floor(hf/wf * width)

        self.stream_dims = (height, width)

    async def recv(self):
        """
        Returns frame for webRTC stream when called.

        Reads frame from camera using provided callback, rotates and resizes. 
        Returns green frame if the callback doesn't yield a frame.

        Returns:
            video_frame (VideoFrame): encoded frame   
        """
        pts, time_base = await self.next_timestamp()

        # Read latest frame from RPiInsightCam
        frame = self.read_frame()  

        # If frame is none, return green
        if frame is not None:
            video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
            video_frame = video_frame.reformat(self.stream_dims[1], self.stream_dims[0])
        else:
            video_frame = VideoFrame(width=self.stream_dims[1], height=self.stream_dims[0])

        video_frame.pts = pts
        video_frame.time_base = time_base
        
        return video_frame

    def stop(self):
        # catch stop from receiver and keep alive
        print('CameraStreamTrack stop caught. Keeping alive.')

    def close(self):
        print('Stopping CameraStreamTrack.')
        super().stop()


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
            log_info(f"Connection state is {pc.connectionState}")
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
            app, access_log=None, host=host, port=port, handle_signals=False
        )
