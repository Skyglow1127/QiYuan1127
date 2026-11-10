import * as THREE from 'three';
import {STLLoader} from './vendor/STLLoader.js';

export async function mountWristCamera(links) {
  const response = await fetch('/config/wrist_camera.json', {cache: 'no-store'});
  if (!response.ok) throw new Error('腕部相机外观配置尚未就绪');
  const config = await response.json(), parent = links[config.parent];
  if (!parent) throw new Error('未找到腕部相机挂载关节');
  const mount = new THREE.Group();
  mount.name = 'wrist_camera_display';
  mount.position.fromArray(config.mount.xyz);
  mount.rotation.set(...config.mount.rpy, 'ZYX');
  mount.userData.previewView = config.view;
  const stl = new STLLoader();
  for (const shape of config.shapes || []) {
    let geometry;
    if (shape.geometry === 'mesh') {
      geometry = await stl.loadAsync(shape.mesh);
      geometry.computeVertexNormals();
    } else if (shape.geometry === 'box') geometry = new THREE.BoxGeometry(...shape.size);
    else if (shape.geometry === 'cylinder') {
      geometry = new THREE.CylinderGeometry(shape.radius, shape.radius, shape.length, 20);
      geometry.rotateX(Math.PI / 2);
    } else if (shape.geometry === 'sphere') geometry = new THREE.SphereGeometry(shape.radius, 16, 10);
    else continue;
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
      color: shape.color, roughness: shape.roughness ?? .6, metalness: shape.metalness ?? .15,
    }));
    mesh.name = shape.name;
    mesh.position.fromArray(shape.position);
    mesh.rotation.set(...(shape.rpy || [0, 0, 0]), 'ZYX');
    if (shape.scale) mesh.scale.fromArray(shape.scale);
    mesh.castShadow = mesh.receiveShadow = true;
    mount.add(mesh);
  }
  parent.add(mount);
  return mount;
}

// Both image elements display the same decoded, actual Gazebo JPEG. Freshness
// comes from the sensor timestamp reported by the server, never a local timer
// that could accidentally label a repeatedly downloaded old image as live.
export function createWristCameraFeed(container) {
  const images = ['wrist-image', 'wrist-image-large'].map(id => document.getElementById(id));
  const messages = ['wrist-empty', 'wrist-empty-large'].map(id => document.getElementById(id));
  const status = document.getElementById('wrist-status');
  const details = document.getElementById('wrist-details');
  const rgbButton = document.getElementById('rgb-view');
  let metadata = null, stateAt = 0, imageAt = 0, imageURL = null, requestOK = false;
  const fresh = () => metadata?.connected && typeof metadata.age === 'number' &&
    Number.isFinite(metadata.age) && metadata.age + (performance.now() - stateAt) / 1000 < 2;
  function refreshStatus() {
    const live = fresh() && requestOK && imageURL && performance.now() - imageAt < 2000;
    const text = live ? '● 实时' : metadata?.connected ? '等待新画面' : '仿真相机未连接';
    status.textContent = text;
    status.classList.toggle('live', !!live);
    const empty = metadata?.connected && !fresh() ? '画面已过期，等待新帧' : text;
    for (const element of messages) { element.hidden = !!live; element.textContent = empty; }
    for (const image of images) image.hidden = !live;
    details.textContent = live ? `${metadata.width} × ${metadata.height} · Gazebo RGB · 约 5 Hz` : '由 Gazebo 腕部传感器生成';
  }
  function showCamera() {
    container.classList.add('camera-mode');
    rgbButton.classList.add('active');
    document.querySelectorAll('[data-view]').forEach(button => button.classList.remove('active'));
  }
  rgbButton.onclick = showCamera;
  document.getElementById('wrist-expand').onclick = showCamera;
  document.getElementById('wrist-back').onclick = () => document.getElementById('resetview').click();
  async function pump() {
    const start = performance.now();
    if (fresh()) {
      let pendingURL = null;
      try {
        const response = await fetch(`/api/wrist_camera.jpg?t=${Date.now()}`, {cache: 'no-store', signal: AbortSignal.timeout(1800)});
        if (!response.ok || !response.headers.get('content-type')?.startsWith('image/')) throw new Error('相机画面暂不可用');
        pendingURL = URL.createObjectURL(await response.blob());
        const decoded = new Image(); decoded.src = pendingURL; await decoded.decode();
        if (fresh()) {
          const previousURL = imageURL;
          imageURL = pendingURL; pendingURL = null;
          for (const image of images) image.src = imageURL;
          imageAt = performance.now(); requestOK = true;
          if (previousURL) URL.revokeObjectURL(previousURL);
        }
      } catch (error) { requestOK = false; }
      finally { if (pendingURL) URL.revokeObjectURL(pendingURL); }
    } else requestOK = false;
    refreshStatus();
    setTimeout(pump, Math.max(30, 200 - (performance.now() - start)));
  }
  pump();
  return {
    update(value) { metadata = value || null; stateAt = performance.now(); refreshStatus(); },
    showScene() { container.classList.remove('camera-mode'); rgbButton.classList.remove('active'); },
  };
}
