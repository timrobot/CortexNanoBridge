import {
  Scene,
  Color,
  PerspectiveCamera,
  HemisphereLight,
  DirectionalLight,
  BoxBufferGeometry,
  SphereBufferGeometry,
  MeshStandardMaterial,
  Mesh,
  Matrix4,
  WebGLRenderer
} from "https://cdn.skypack.dev/three";

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

            const sdp = pc.localDescription.sdp;
            const ip_begin = sdp.indexOf("IP4 ") + 4;
            let ip_end = ip_begin;
            while (sdp.charAt(ip_end) === '.' || !isNaN(sdp.charAt(ip_end))) {
              ip_end++;
            }
            constructSocket(sdp.substring(ip_begin, ip_end));
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
  negotiate().then(()=>{});
};

// gamepad stuff
// let gamepadIndex;
// window.addEventListener('gamepadconnected', (event) => {
//   const gamepad = navigator.getGamepads()[event.gamepad.index];
//   if (gamepad.id.toLowerCase().indexOf("xbox") !== -1 ||
//   gamepad.id.toLowerCase().indexOf("ps") !== -1) { // supported devices
//     gamepadIndex = event.gamepad.index;
//   }
// });

// window.addEventListener('gamepaddisconnected', (event) => {
//   if (event.gamepad.index === gamepadIndex) {
//     gamepadIndex = undefined;
//   }
// });

let scene, camera, renderer;
const mate_geometry = new SphereBufferGeometry(0.5, 12, 8);
const mate_material = new MeshStandardMaterial({ color: 0xff0000 });
let objects = {};
let rendered = [];

function createNewObject(desc) { // factory
  if (desc.model) {
    return new Mesh(mate_geometry, mate_material);
  } else { // mate
    return new Mesh(mate_geometry, mate_material);
  }
}

function render() {
  let all_names = [];
  Object.keys(objects).forEach(function(name) {
    all_names.push(name);
    const desc = objects[name];
    let object;
    if (rendered.indexOf(name) === -1) {
      object = createNewObject(desc);
      object.name = name;
      scene.add(object);
      rendered.push(name);
    } else {
      object = scene.getObjectByName(name);
    }
    const xyz = desc.xyz;
    object.position.set(xyz[0], xyz[1], xyz[2]);
    object.setRotationFromMatrix(new Matrix4().fromArray(desc.rotation));
  });
  for (let i = rendered.length-1; i >= 0; i--) {
    const name = rendered[i];
    if (all_names.indexOf(name) === -1) {
      let object = scene.getObjectByName(name);
      scene.remove(object);
      rendered = rendered.filter((e) => {
        return e !== name;
      });
    }
  }
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}

function constructViewer() {
  const container = document.getElementById("viewer");
  const width = 640;
  const height = 480;
  scene = new Scene();
  scene.background = new Color(0xB3E5FC);

  const hemilight = new HemisphereLight();
  scene.add(hemilight);

  const dirlight = new DirectionalLight('white', 8);
  scene.add(dirlight);
  
  const fov = 75;
  const aspect = width / height;
  const near = 0.1;
  const far = 100;
  camera = new PerspectiveCamera(fov, aspect, near, far);
  camera.position.set(0, -8, 4);
  camera.lookAt(0, 0, 0);

  renderer = new WebGLRenderer();
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);
  container.append(renderer.domElement);

  const ground_geometry = new BoxBufferGeometry(1000, 1000, 0.01);
  const ground_material = new MeshStandardMaterial({ color: 0xFFE0B2 });
  const ground = new Mesh(ground_geometry, ground_material);
  ground.position.set(0, 0, -9);
  scene.add(ground);

  renderer.render(scene, camera);
  requestAnimationFrame(render);
}

// websockets
let sock = null;
let sock_connected = false;
let keysdown = [];

function renderList(id, pfx, values) {
  let ul = document.getElementById(id);
  ul.innerHTML = '';
  for (let i = 0; i < values.length; i++) {
    let li = document.createElement('li');
    li.innerHTML = pfx + "[" + i + "]: " + values[i];
    ul.appendChild(li);
  }
}

function renderBoundingBoxes(boxes) {
  let ul = document.getElementById("bboxes");
  ul.innerHTML = '';
  boxes.forEach((item) => {
    let li = document.createElement('li');
    li.innerHTML = item[0] + ': ' + item[1];
    ul.append(li);
  });
}

function onMessage(req) {
  const j = JSON.parse(req.data);
  if (j.logs) {
    for (let i = 0; i < j.logs.length; i++) {
      console.log("[robot]", j.logs[i]);
    }
  }
  if (j.robot_values) {
    renderList("motors", "motor", j.robot_values.motors);
    renderList("sensors", "sensor", j.robot_values.sensors);
  }
  if (j.model) {
    objects = j.model;
  }
  if (j.bounding_boxes) {
    renderBoundingBoxes(j.bounding_boxes);
  }
}

function sendMessage() {
  if (sock_connected) {
    let msg = {
      "logs": null,
      "robot_values": null,
      "model": null,
      "bounding_boxes": null
    };
    // if (gamepadIndex !== undefined) {
    //   const gamepad = navigator.getGamepads()[gamepadIndex];
    //   const buttons = gamepad.buttons.map(x => x.value);

    //   renderList("gamepad_buttons", "button", buttons);
    //   renderList("gamepad_axes", "axes", gamepad.axes);

    //   msg["gamepad"] = {
    //     "buttons": buttons,
    //     "axes": gamepad.axes,
    //     "timestamp": gamepad.timestamp
    //   };
    // }

    msg["keydown"] = keysdown;
    sock.send(JSON.stringify(msg));
  }
}

function constructSocket(ip) {
  sock = new WebSocket('ws://' + ip + ':9876');
  sock.onopen = function() {
    console.log("[open] Connection established");
    sock_connected = true;
  };
  sock.onmessage = onMessage;
  sock.onclose = function(e) {
    if (e.wasClean) {
      console.log(`[close] Connection closed cleanly, code=${e.code} reason=${e.reason}`);
    } else {
      // e.g. server process killed or network down
      // event.code is usually 1006 in this case
      console.log('[close] Connection died');
    }
  };
  sock.onerror = function(e) {
    console.log(`[error] ${e.message}`);
  };
  setInterval(sendMessage, 10); // 100Hz
}

window.addEventListener('load', function() {
  constructViewer();
  // setTimeout(window.startVideoStream, 5000);
});

document.addEventListener('keydown', (event) => {
  const name = event.key;
  const code = event.code;
  keysdown.push(code);
});
document.addEventListener('keyup', (event) => {
  const name = event.key;
  const code = event.code;
  keysdown = keysdown.filter((e) => {
    return e !== code;
  });
});