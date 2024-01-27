let pc = null;

async function negotiate() {
  pc.addTransceiver('video', { direction: 'recvonly' });
  try {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    if (pc.iceGatheringState !== 'complete') {
      await new Promise(function(resolve) {
        function checkState() {
          if (pc.iceGatheringState === 'complete') { // finished negotiations, found ip
            pc.removeEventListener('icegatheringstatechange', checkState);
            resolve();
          }
        }
        pc.addEventListener('icegatheringstatechange', checkState);
      });
    }

    const desc = pc.localDescription;
    const response = await fetch('/offer', {
      body: JSON.stringify({
        sdp: desc.sdp,
        type: desc.type,
      }),
      headers: {
        'Content-Type': 'application/json'
      },
      method: 'POST'
    });
    const answer = await response.json();
    await pc.setRemoteDescription(answer);
  } catch (e) {
    console.log(e);
  }
}

window.startVideoStream = function() {
  pc = new RTCPeerConnection({
    sdpSemantics: 'unified-plan',
    iceServers: [{ urls: ['stun:stun.l.google.com:19302'] }]
  });
  
  let remoteStream = new MediaStream();
  pc.ontrack = event => {
    if (event.streams[0]) {
      event.streams[0].getTracks().forEach(track => {
        remoteStream.addTrack(track);
      });
    }
  };
  document.getElementById('video').srcObject = remoteStream;
  document.getElementById('start').style.display = 'none';
  negotiate().catch(()=>{});
};