import * as THREE from 'three';
import {STLLoader} from './vendor/STLLoader.js';
import {SoftwareRenderer} from './software-renderer.js?v=photo-environment-3';
import {OrbitControls} from './vendor/OrbitControls.js';
import {loadEnvironment} from './environment-view.js?v=piper-labels-1';
import {mountWristCamera,createWristCameraFeed} from './wrist-camera-view.js?v=stl-1';
const el=id=>document.getElementById(id),canvas=el('canvas'),container=el('view');
const scene=new THREE.Scene();scene.up.set(0,0,1);
const camera=new THREE.PerspectiveCamera(36,1,.01,20);camera.up.set(0,0,1);
let renderer,software=false;try{renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true})}catch(e){software=true;renderer=new SoftwareRenderer(canvas);document.querySelector(".legend").textContent="Canvas 兼容视图 · Gazebo 反馈"};renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;renderer.outputColorSpace=THREE.SRGBColorSpace;
const orbit=new OrbitControls(camera,canvas);orbit.enableDamping=true;orbit.minDistance=.35;orbit.maxDistance=6;
const views={overview:{position:[-1.6,.92,1.48],target:[.24,-.28,.19]},grasp:{position:[-.65,.74,.62],target:[.25,0,.17]},top:{position:[.2,-.27,2.3],target:[.2,-.27,0]}};
views.aipc={position:[-.34,-.12,.22],target:[-.10,-.31,.024]};
views.kr260={position:[-.34,-.37,.23],target:[-.10,-.60,.018]};
let activeView='overview',wristCameraMount=null,wristStream=null;
function setView(name='overview'){let v=views[name];if(name==='wrist'){if(!wristCameraMount)return;scene.updateMatrixWorld(true);const local=wristCameraMount.userData.previewView||{position:[.25,-.28,.18],target:[0,0,0]};v={position:wristCameraMount.localToWorld(new THREE.Vector3(...local.position)).toArray(),target:wristCameraMount.localToWorld(new THREE.Vector3(...local.target)).toArray()};v.position[2]=Math.max(v.position[2],v.target[2]+.12,.15)}activeView=name;wristStream?.showScene();orbit.target.fromArray(v.target);const fit=Math.max(1,1.1*container.clientHeight/Math.max(1,container.clientWidth));camera.position.fromArray(v.position).sub(orbit.target).multiplyScalar(fit).add(orbit.target);orbit.update();document.querySelectorAll('[data-view]').forEach(button=>button.classList.toggle('active',button.dataset.view===name))};
function resetView(){setView('overview')};resetView();el('resetview').onclick=resetView;document.querySelectorAll('[data-view]').forEach(button=>button.onclick=()=>setView(button.dataset.view));
scene.add(new THREE.HemisphereLight(0xf0f6ff,0x73848b,2.2));const key=new THREE.DirectionalLight(0xfff5e8,3.2);key.position.set(-1,1,3);key.castShadow=true;key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-1.2;key.shadow.camera.right=1.2;key.shadow.camera.top=1.2;key.shadow.camera.bottom=-1.2;scene.add(key);const rim=new THREE.DirectionalLight(0xbddde8,1.1);rim.position.set(1,-1,2);scene.add(rim);
const robotResponse=await fetch('/config/robot.json',{cache:'no-store'});if(!robotResponse.ok)throw Error('机器人配置加载失败');const robot=await robotResponse.json();
const environment=await loadEnvironment(scene,container,{labelOverrides:{'宇树 D1-T':robot.name}});
if(environment.config.camera){views.overview=environment.config.camera;resetView()}
el('labels-toggle').onchange=e=>environment.setLabels(e.target.checked);
const grid=new THREE.GridHelper(2,20,0x53778c,0x6b8a9a);grid.rotation.x=Math.PI/2;grid.position.set(.21,-.29,.002);grid.visible=false;scene.add(grid);scene.userData.showGrid=false;el('grid-toggle').onchange=e=>{grid.visible=e.target.checked;scene.userData.showGrid=e.target.checked};
const graspScene=await(await fetch('/config/grasp_scene.json')).json();
const mount=new THREE.Mesh(new THREE.BoxGeometry(...graspScene.mount.size),new THREE.MeshStandardMaterial({color:0x687077,roughness:.7}));mount.position.fromArray(graspScene.mount.position);mount.castShadow=true;scene.add(mount);
const cube=new THREE.Mesh(new THREE.BoxGeometry(...graspScene.cube.size),new THREE.MeshStandardMaterial({color:0xee152b,roughness:.65}));cube.position.fromArray(graspScene.cube.position);cube.castShadow=true;scene.add(cube);
const modelResponse=await fetch('/generated/model.json',{cache:'no-store'});if(!modelResponse.ok)throw Error('机器人模型加载失败');const model=await modelResponse.json();
const armJoints=robot.arm_joints,fingerJoints=robot.finger_joints,fingerOpen=robot.finger_open;
if(!Array.isArray(armJoints)||armJoints.length!==6||!Array.isArray(fingerJoints)||!Array.isArray(fingerOpen)||fingerJoints.length!==fingerOpen.length||!fingerOpen.length||fingerOpen.some(value=>!Number.isFinite(value)||value===0))throw Error('机器人关节配置无效');
const links={},pivots={},slider={};const stl=new STLLoader();
el('robot-dof').textContent=armJoints.length+' + 1';
el('gripper-travel').textContent=(fingerOpen.reduce((sum,value)=>sum+Math.abs(value),0)*1000).toFixed(0);
const closure=values=>Math.min(1,Math.max(0,1-fingerJoints.reduce((sum,name,index)=>sum+(Number.isFinite(values?.[name])?values[name]/fingerOpen[index]:0),0)/fingerJoints.length));
const displayMeshes=software?await(await fetch('/generated/display_meshes.json')).json():null;
for(const l of model.links){const group=new THREE.Group();links[l.name]=group;if(l.mesh){const add=g=>{g.computeVertexNormals();const rgba=Array.isArray(l.color)?l.color:[.25,.28,.31,1],material=new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(...rgba.slice(0,3)),metalness:.35,roughness:.52,opacity:rgba[3]??1,transparent:(rgba[3]??1)<1});const m=new THREE.Mesh(g,material);m.castShadow=true;m.receiveShadow=true;group.add(m)};if(software){const d=displayMeshes[l.mesh.split('/').pop()],g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(d.vertices.flat(),3));g.setIndex(d.faces.flat());add(g)}else stl.load(l.mesh,add,undefined,e=>{el('message').textContent='STL 模型加载失败';console.error(e)})}}
for(const j of model.joints){const origin=new THREE.Group();origin.position.fromArray(j.xyz);origin.rotation.set(...j.rpy,'ZYX');links[j.parent].add(origin);const pivot=new THREE.Group();origin.add(pivot);pivot.add(links[j.child]);pivots[j.name]={pivot,axis:new THREE.Vector3(...j.axis),type:j.type}}
const childLinks=new Set(model.joints.map(joint=>joint.child));for(const [name,group]of Object.entries(links))if(!childLinks.has(name))scene.add(group);
wristStream=createWristCameraFeed(container);
try{wristCameraMount=await mountWristCamera(links);el('wrist-view').disabled=false}catch(error){console.warn(error.message)}
const names=['底座旋转','肩关节','肘关节','腕部旋转','腕部俯仰','末端旋转'];
for(let i=1;i<=6;i++){let name=armJoints[i-1],j=model.joints.find(x=>x.name===name),row=document.createElement('div');row.className='joint';let lo=j.limits.lower*180/Math.PI,hi=j.limits.upper*180/Math.PI;row.innerHTML=`<div class="label"><span>J${i} · ${names[i-1]}</span><output id="actual${i}">0.0°</output></div><input aria-label="J${i} ${names[i-1]}" type="range" min="${lo}" max="${hi}" step="0.1" value="0" disabled><div class="ends"><span>${lo.toFixed(1)}°</span><span id="target${i}">目标 0°</span><span>${hi.toFixed(1)}°</span></div>`;el('joints').append(row);const input=row.querySelector('input');slider[name]=input;input.addEventListener('input',()=>{el('target'+i).textContent='目标 '+Number(input.value).toFixed(1)+'°';queue[name]=Number(input.value)*Math.PI/180})}
let connected=false,holding=false,queue={},busy=false,gripDragging=false,transientError="",transientUntil=0;
async function command(data){try{const r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const d=await r.json();if(!r.ok){const st=await(await fetch('/api/state')).json();syncTargets(st.targets);throw Error(d.error);}el('message').textContent='';return d}catch(e){transientError=e.message;transientUntil=Date.now()+5000;el('message').textContent=e.message;return null}}
setInterval(async()=>{if(busy||!Object.keys(queue).length)return;busy=true;const values=queue;queue={};await command({values});busy=false},80);
el('gripper').oninput=()=>{queue.gripper=Number(el('gripper').value)/100};
for(const action of ['home','ready'])el(action).onclick=async()=>{queue={};const d=await command({action});if(d)syncTargets(d.targets)};
el('hold').onclick=async()=>{queue={};const d=await command({action:holding?'resume':'hold'});if(d)syncTargets(d.targets)};
function syncTargets(t){for(let i=1;i<=armJoints.length;i++){let n=armJoints[i-1];if(!Number.isFinite(t?.[n]))continue;slider[n].value=t[n]*180/Math.PI;el('target'+i).textContent='目标 '+(t[n]*180/Math.PI).toFixed(1)+'°'}el('gripper').value=closure(t)*100}
for(const action of ['grasp','place','reset_cube'])el(action).onclick=async()=>{queue={};const d=await command({action});if(d)syncTargets(d.targets)};
let first=true;
async function poll(){try{const s=await(await fetch('/api/state',{cache:'no-store'})).json();wristStream.update(s.wrist_camera);connected=s.connected;holding=s.hold;el('message').textContent=s.collision_warning||(Date.now()<transientUntil?transientError:'');el('time').textContent=s.sim_time.toFixed(2)+' s';el('feedback').textContent=connected?'● Gazebo 反馈在线':'反馈中断';el('status').textContent=connected?(holding?'● 已保持当前位置':'● Gazebo 已连接'):'Gazebo 未连接';el('status').classList.toggle('live',connected);el('hold').textContent=holding?'恢复控制':'保持当前位置';el('hold').disabled=!connected;const auto=s.grasp?.active;for(const a of ['home','ready','gripper','grasp','place','reset_cube'])el(a).disabled=!connected||holding||auto;for(const input of Object.values(slider))input.disabled=!connected||holding||auto;
if(s.cube?.pose){cube.position.fromArray(s.cube.pose.position);cube.quaternion.fromArray(s.cube.pose.quaternion);el('cubeheight').textContent=(s.cube.pose.position[2]*100).toFixed(1)+' cm';el('contacts').textContent=s.cube.finger_contacts.length+' / 2';}el('graspstatus').textContent=s.grasp?.phase||'等待状态';if(auto)syncTargets(s.targets);
for(const [n,v]of Object.entries(s.positions)){let p=pivots[n];if(!p)continue;if(p.type==='revolute'||p.type==='continuous')p.pivot.quaternion.setFromAxisAngle(p.axis,v);else if(p.type==='prismatic')p.pivot.position.copy(p.axis).multiplyScalar(v)}
for(let i=1;i<=armJoints.length;i++)el('actual'+i).textContent=((s.positions[armJoints[i-1]]||0)*180/Math.PI).toFixed(1)+'°';el('gripvalue').textContent=Math.round(closure(s.positions)*100)+'%';if(first&&connected){syncTargets(s.targets);first=false}}
catch(e){wristStream.update(null);connected=false;el('status').textContent='控制服务已断开';el('status').classList.remove('live');document.querySelectorAll('aside input,aside button:not(#resetview):not(#wrist-expand)').forEach(e=>e.disabled=true)}finally{setTimeout(poll,100)}}poll();
new ResizeObserver(()=>{const w=container.clientWidth,h=container.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();setView(activeView)}).observe(container);
let renderFrames=0,renderStart=performance.now();
renderer.setAnimationLoop(()=>{orbit.update();renderer.render(scene,camera);environment.projectLabels(camera);renderFrames++;const now=performance.now();if(now-renderStart>=1000){document.querySelector('.legend').textContent=(software?'Canvas 兼容视图':'WebGL 视图')+' · '+Math.round(renderFrames*1000/(now-renderStart))+' fps';renderFrames=0;renderStart=now}});
