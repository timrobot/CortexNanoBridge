import requests
import socket

endpoint = "http://192.168.1.34:3000"
device_name = "test-robot"

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    s.connect(('10.254.254.254', 1))
    IP = s.getsockname()[0]
    return IP

def set_name(name=""):
  global device_name
  device_name = name

def heartbeat():
  name = device_name
  ip = get_ip()
  if name is None:
     name = ip
  requests.post(f"{endpoint}/heartbeat",
    json={"name": device_name, "ipv4": ip})