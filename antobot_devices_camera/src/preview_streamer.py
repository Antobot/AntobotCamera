#!/usr/bin/env python3
# Copyright (c) 2024, ANTOBOT LTD.
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

# # # Code Description: 
# # # Interfaces:       

# Contacts: Authors:    james.bennett@antobot.ai
#           Owner:      james.bennett@antobot.ai


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import argparse
import asyncio
import json
import logging
import os
import ssl
import uuid
import threading

import cv2
from aiohttp import web
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder, MediaRelay
from av import VideoFrame
from picamera2 import Picamera2
import aiohttp_cors


class PreviewStreamer:
    def __init__(self, track_reference):
        self.logger = logging.getLogger("pc")
        self.pcs = set()
        
        self.track_reference = track_reference

        # thread = threading.Thread(target=self.run)
        # thread.start()


    async def offer(self, request):
        print("in offer function")
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        print("offer request received")

        pc = RTCPeerConnection()
        pc_id = "PeerConnection(%s)" % uuid.uuid4()
        self.pcs.add(pc)

        def log_info(msg, *args):
            self.logger.info(pc_id + " " + msg, *args)

        log_info("Created for %s", request.remote)

        
        @pc.on("datachannel")
        def on_datachannel(channel):
            @channel.on("message")
            def on_message(message):
                if isinstance(message, str) and message.startswith("ping"):
                    channel.send("pong" + message[4:])

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            log_info("Connection state is %s", pc.connectionState)
            if pc.connectionState == "failed":
                print('connection state change failed')
                await pc.close()
                self.pcs.discard(pc)

        @pc.on("track")
        def on_track(track):
            log_info("Track %s received", track.kind)
            print('TRACK CALLBACK')
            
            # pc.addTrack(VideoStreamTrack())
            if self.track_reference is not None:
                self.track_reference.set_size(params["width"], params["height"])

                pc.addTrack(self.track_reference)
            else:
                print("Track is None")
            
            
            
            @track.on("ended")
            async def on_ended():
                log_info("Track %s ended", track.kind)
                print("ENDED")
                

        # handle offer
        await pc.setRemoteDescription(offer)
        # await recorder.start()

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
        print('in run')
        
        verbose = False
        if verbose:
            logging.basicConfig(level=logging.DEBUG)
        else:
            logging.basicConfig(level=logging.INFO)

        app = web.Application()
                    
        cors = aiohttp_cors.setup(app)

        resource = cors.add(app.router.add_resource("/offer"))
        route = cors.add(
        resource.add_route("POST", self.offer), {
            "http://localhost:5173": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers=("X-Custom-Server-Header",),
                allow_headers=("X-Requested-With", "Content-Type"),
                max_age=3600,
            )
        })

        host = '0.0.0.0'
        port = 8080

        app.on_shutdown.append(self.on_shutdown)
        web.run_app(
            app, access_log=None, host=host, port=port,
        )