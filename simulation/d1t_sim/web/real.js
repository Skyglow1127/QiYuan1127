import * as THREE from 'three';
import {STLLoader} from './vendor/STLLoader.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {SoftwareRenderer} from './software-renderer.js';

const el = id => document.getElementById(id);
const canvas = el('canvas'), view = el('view');
const scene = new THREE.Scene();
scene.up.set(0, 0, 1);
const camera = new THREE.PerspectiveCamera(36, 1, .01, 20);
camera.up.set(0, 0, 1);
let renderer, software = false;
try {
  renderer = new THREE.WebGLRenderer({canvas, antialias: true, alpha: true});
} catch {
  software = true;
  renderer = new SoftwareRenderer(canvas);
  el('renderer-mode').textContent = '兼容视图 · 只读反馈';
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
const orbit = new OrbitControls(camera, canvas);
orbit.enableDamping = true;
orbit.minDistance = .35;
orbit.maxDistance = 4;
function resetView() {
  camera.position.set(.85, -1, .75);
  orbit.target.set(.18, 0, .24);
  orbit.update();
}
resetView();
el('resetview').addEventListener('click', resetView);
scene.add(new THREE.HemisphereLight(0xc7deff, 0x324456, 2));
const light = new THREE.DirectionalLight(0xf1f6ff, 3.2);
light.position.set(1, -1, 3);
scene.add(light);
const rim = new THREE.DirectionalLight(0x65dfce, 1.5);
rim.position.set(-1, 1, 1);
scene.add(rim);
const grid = new THREE.GridHelper(4, 40, 0x38546b, 0x24374a);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
const mount = new THREE.Mesh(new THREE.BoxGeometry(.15, .23, .02), new THREE.MeshStandardMaterial({color: 0x354e62, roughness: .7}));
mount.position.set(0, 0, .01);
scene.add(mount);
scene.add(new THREE.AxesHelper(.12));

const pivots = {};
let robotRoot = null, robotShown = false, lastPose = null, lastState = null;
let receivedAt = 0, apiFailed = true;
for (let i = 0; i < 6; i++) {
  const row = document.createElement('tr');
  row.innerHTML = `<td>J${i}</td><td id="raw-${i}">—</td><td id="mapped-${i}">J${i + 1} / —</td>`;
  el('joint-values').append(row);
}
const finite = value => typeof value === 'number' && Number.isFinite(value);
const angleNames = Array.from({length: 6}, (_, i) => `arm_Joint${i + 1}`);
const validPose = state => Array.isArray(state.angles_deg) && state.angles_deg.length >= 7 &&
  state.angles_deg.every(finite) && angleNames.every(name => finite(state.positions?.[name]));
const showRaw = value => value === undefined || value === null ? '—' :
  typeof value === 'object' ? JSON.stringify(value) : String(value);
const ageText = age => finite(age) && age >= 0 ? `${age < 100 ? age.toFixed(1) : Math.round(age)} s` : '—';
const switchText = value => value === 0 ? '关闭 (0)' : value === 1 ? '开启 (1)' : showRaw(value);

function applyPose(state) {
  lastPose = state;
  for (let i = 0; i < 6; i++) {
    const name = angleNames[i], value = state.positions[name];
    if (pivots[name]) pivots[name].pivot.quaternion.setFromAxisAngle(pivots[name].axis, value);
    el(`raw-${i}`).textContent = `${state.angles_deg[i].toFixed(2)}°`;
    el(`mapped-${i}`).textContent = `J${i + 1} / ${(value * 180 / Math.PI).toFixed(2)}°`;
  }
  el('gripper-value').textContent = state.angles_deg[6].toFixed(3);
  // The software renderer traverses hidden meshes too. Do not add the robot
  // until a valid feedback pose exists, rather than relying on visible=false.
  if (robotRoot && !robotShown) {
    scene.add(robotRoot);
    robotShown = true;
  }
}

function updateFreshness() {
  const elapsed = receivedAt ? (performance.now() - receivedAt) / 1000 : 0;
  const age = finite(lastState?.age) ? lastState.age + elapsed : null;
  const statusAge = finite(lastState?.status_age) ? lastState.status_age + elapsed : null;
  el('angle-age').textContent = ageText(age);
  el('status-age').textContent = ageText(statusAge);
  const staleSeconds = finite(lastState?.max_age_s) && lastState.max_age_s > 0 ? lastState.max_age_s : .6;
  const fresh = !apiFailed && lastState?.connected === true && lastState.reader_alive === true && validPose(lastState) &&
    finite(age) && age >= 0 && age <= staleSeconds &&
    finite(statusAge) && statusAge >= 0 && statusAge <= staleSeconds;
  el('status').classList.toggle('live', fresh);
  view.classList.toggle('stale', !fresh);
  el('pose-overlay').hidden = fresh && robotShown;
  if (fresh) {
    el('status').textContent = '● 实机反馈在线';
    el('pose-state').textContent = '候选映射 · 实时反馈';
    el('reading-state').textContent = '当前反馈读数';
    el('connection-note').textContent = '角度与状态反馈均在线。本页只读取反馈，不发送运动命令。';
    if (!robotShown) el('pose-message').textContent = '已收到实机反馈，正在加载机械臂模型…';
  } else {
    el('status').textContent = apiFailed ? '监视服务未连接' : '反馈未就绪或已过期';
    el('pose-state').textContent = lastPose ? '旧姿态 · 非当前实机位置' : '等待首帧';
    el('reading-state').textContent = lastPose ? '以下为旧读数，不能确认当前实机状态' : '尚无有效反馈读数';
    el('pose-message').textContent = lastPose ?
      '反馈中断或过期：保留最后一次姿态，仅供参考，不能视为当前实机位置。' :
      '尚无有效角度反馈，机械臂模型暂不显示。';
    el('connection-note').textContent = apiFailed ?
      '无法读取监视服务。已显示的数据属于先前收到的反馈，不能确认当前上电、使能或位置。' :
      '角度或状态反馈未就绪／已过期，不能确认实机当前状态。保留的数值只供排查。';
  }
}

async function fetchJSON(url, timeout = 3000) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch(url, {cache: 'no-store', signal: controller.signal});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return await response.json();
  } finally {
    clearTimeout(timer);
  }
}

async function loadModel() {
  try {
    const model = await fetchJSON('/generated/model.json');
    const display = software ? await fetchJSON('/generated/display_meshes.json', 10000) : null;
    const links = {}, loader = new STLLoader();
    for (const link of model.links) {
      const group = new THREE.Group();
      links[link.name] = group;
      // J6 has no calibrated opening conversion. Never draw either finger.
      if (!link.mesh || /^arm_Link7_/.test(link.name)) continue;
      const add = geometry => {
        geometry.computeVertexNormals();
        const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({color: 0xb9c9d5, metalness: .65, roughness: .34}));
        group.add(mesh);
      };
      if (software) {
        const data = display[link.mesh.split('/').pop()];
        if (!data) throw new Error(`缺少显示网格 ${link.name}`);
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(data.vertices.flat(), 3));
        geometry.setIndex(data.faces.flat());
        add(geometry);
      } else {
        loader.load(link.mesh, add, undefined, () => {
          el('load-error').textContent = '部分机械臂网格加载失败，请刷新页面。角度反馈仍可查看。';
        });
      }
    }
    for (const joint of model.joints) {
      const origin = new THREE.Group();
      origin.position.fromArray(joint.xyz);
      origin.rotation.set(...joint.rpy, 'ZYX');
      links[joint.parent].add(origin);
      const pivot = new THREE.Group();
      origin.add(pivot);
      pivot.add(links[joint.child]);
      pivots[joint.name] = {pivot, axis: new THREE.Vector3(...joint.axis)};
    }
    robotRoot = links.world;
    if (lastPose) applyPose(lastPose);
    updateFreshness();
  } catch (error) {
    el('load-error').textContent = `模型加载失败：${error.message}。数值反馈仍可查看。`;
  }
}

async function poll() {
  try {
    const state = await fetchJSON('/api/state', 1800);
    if (state.source !== 'hardware_readonly') throw new Error('数据源不是只读实机服务');
    lastState = state;
    receivedAt = performance.now();
    apiFailed = false;
    if (validPose(state)) applyPose(state);
    el('power').textContent = switchText(state.status?.power_status);
    el('enable').textContent = switchText(state.status?.enable_status);
    const errorStatus = state.status?.error_status;
    el('error').textContent = errorStatus === 0 ? '无错误报告 (0)' : finite(errorStatus) ? `故障 (${errorStatus})` : '—';
    el('error').classList.toggle('warning', finite(errorStatus) && errorStatus !== 0);
    el('reader').textContent = state.reader_alive ? '运行中' : '已停止';
    el('mapping').textContent = `候选映射，仅用于预览：${state.mapping?.description || '方向与零位尚未完成全行程校准。'}`;
    el('limits-warning').textContent = Array.isArray(state.limits_warning) ? state.limits_warning.join('\n') : '';
    el('counters').textContent = `有效帧 ${showRaw(state.frames)} · 无效帧 ${showRaw(state.invalid_frames)} · 运行 ${finite(state.uptime_s) ? Math.round(state.uptime_s) + ' s' : '—'}`;
  } catch {
    apiFailed = true;
  } finally {
    updateFreshness();
    setTimeout(poll, 250);
  }
}

new ResizeObserver(() => {
  const width = view.clientWidth, height = view.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}).observe(view);
renderer.setAnimationLoop(() => {
  orbit.update();
  renderer.render(scene, camera);
});
setInterval(updateFreshness, 250);
loadModel();
poll();
