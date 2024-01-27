import cv2
import json
import time

import asyncio
from aiohttp import web, helpers, web_runner
from av import VideoFrame
from av.frame import Frame
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.rtcrtpsender import RTCRtpSender
from typing import Tuple
import fractions

FPS = 30
VIDEO_CLOCK_RATE = 90000
VIDEO_PTIME = 1 / FPS
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)

class MediaStreamError(Exception): pass

class OpenCVStreamTrack(MediaStreamTrack):
  kind = "video"
  _start: float
  _timestamp: int

  def __init__(self, path="/dev/video0"):
    super().__init__()
    self.camera = cv2.VideoCapture(path)
    self.camera.read()

  async def next_timestamp(self) -> Tuple[int, fractions.Fraction]:
    if self.readyState != "live":
      raise MediaStreamError

    if hasattr(self, "_timestamp"):
      self._timestamp += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
      wait = self._start + (self._timestamp / VIDEO_CLOCK_RATE) - time.time()
      await asyncio.sleep(wait)
    else:
      self._start = time.time()
      self._timestamp = 0
    return self._timestamp, VIDEO_TIME_BASE

  async def recv(self) -> Frame:
    frame = None
    _, img = self.camera.read()
    if img is not None:
      frame = VideoFrame.from_ndarray(img, format="bgr24")
      pts, time_base = await self.next_timestamp()
      frame.pts = pts
      frame.time_base = time_base
    return frame

async def index(request):
    content = open("index.html", "r").read()
    return web.Response(content_type="text/html", text=content)

async def js(request):
    content = open("client.js", "r").read()
    return web.Response(content_type="application/javascript", text=content)

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

class RTCStreamEntity:
  pcs = set()
  rpc_calls = {}

  def __init__(self, camera_path="/dev/video0", host="0.0.0.0", stream_port=8080, loop=None):
    self.app = web.Application()
    self.app.on_shutdown.append(self.on_shutdown)
    self.app.router.add_get("/", index)
    self.app.router.add_get("/client.js", js)
    self.app.router.add_post("/offer", self.offer)
    
    self.track = OpenCVStreamTrack(camera_path)
    self.host = host
    self.stream_port = stream_port
    self.loop = None
    self.stream_task = None

  async def offer(self, req):
    params = await req.json()

    pc = RTCPeerConnection()
    RTCStreamEntity.pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
      print("Connection state change is %s" % pc.connectionState)
      if pc.connectionState == "failed":
        await pc.close()
        RTCStreamEntity.pcs.discard(pc)

    desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    video_sender = pc.addTrack(self.track)
    force_codec(pc, video_sender, "video/H264")

    await pc.setRemoteDescription(desc)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
      content_type="application/json",
      text=json.dumps({
          "sdp":  pc.localDescription.sdp,
          "type": pc.localDescription.type
        })
    )

  async def on_shutdown(self, app):
    coros = [pc.close() for pc in RTCStreamEntity.pcs]
    await asyncio.gather(*coros)
    RTCStreamEntity.pcs.clear()

  def run(self):
    self.loop = asyncio.new_event_loop()
    self.stream_task = self.loop.create_task(
      web._run_app(
        self.app,
        host=self.host,
        port=self.stream_port,
        ssl_context=None,
        access_log=None
      )
    )

    try:
      asyncio.set_event_loop(self.loop)
      self.loop.run_until_complete(self.stream_task)
    except (web_runner.GracefulExit, KeyboardInterrupt):
      pass
    finally:
      web._cancel_tasks((self.stream_task,), self.loop)
      web._cancel_tasks(helpers.all_tasks(self.loop), self.loop)
      self.loop.run_until_complete(self.loop.shutdown_asyncgens())
      self.loop.close()

if __name__ == "__main__":
  streamer = RTCStreamEntity("/dev/video0")
  streamer.run()