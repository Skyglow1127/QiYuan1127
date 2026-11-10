import * as THREE from 'three';

// The same metre-based primitives are used by Gazebo and by this display.
// No photograph texture is used: personal information in the source photos
// therefore never becomes part of the simulator or its exported assets.
export async function loadEnvironment(scene, container, {labelOverrides = {}} = {}) {
  const response = await fetch('/config/environment.json', {cache: 'no-store'});
  if (!response.ok) throw new Error('照片环境配置尚未就绪');
  const config = await response.json();
  const root = new THREE.Group();
  root.name = 'photo_environment';
  const materials = new Map();
  for (const shape of config.shapes) {
    let geometry;
    if (shape.geometry === 'box') {
      geometry = new THREE.BoxGeometry(...shape.size);
    } else if (shape.geometry === 'cylinder') {
      geometry = new THREE.CylinderGeometry(shape.radius, shape.radius, shape.length, 16);
      // SDF cylinders point along local Z; Three cylinders point along Y.
      geometry.rotateX(Math.PI / 2);
    } else if (shape.geometry === 'sphere') {
      geometry = new THREE.SphereGeometry(shape.radius, 12, 8);
    } else continue;
    const key = [shape.color, shape.roughness ?? .7, shape.metalness ?? .15].join('|');
    if (!materials.has(key)) materials.set(key, new THREE.MeshStandardMaterial({
      color: shape.color, roughness: shape.roughness ?? .7, metalness: shape.metalness ?? .15,
    }));
    const mesh = new THREE.Mesh(geometry, materials.get(key));
    mesh.name = shape.name;
    mesh.position.fromArray(shape.position);
    mesh.rotation.set(...(shape.rpy || [0, 0, 0]), 'ZYX');
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    root.add(mesh);
  }
  // Shared vector markings also appear in native Gazebo; no photo textures.
  const markingResponse = await fetch('/models/photo_workspace/markings.json', {cache:'no-store'});
  if (markingResponse.ok) {
    for (const marking of await markingResponse.json()) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.Float32BufferAttribute(marking.vertices.flat(),3));
      geometry.setIndex(marking.faces.flat());geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:marking.color,roughness:.85}));
      mesh.name=marking.name+'_lettering';mesh.position.fromArray(marking.position);mesh.rotation.set(...(marking.rpy||[0,0,0]),'ZYX');root.add(mesh);
    }
  }
  scene.add(root);
  const overlay = document.createElement('div');
  overlay.className = 'environment-labels';
  overlay.setAttribute('aria-hidden', 'true');
  container.append(overlay);
  const labels = (config.labels || []).map(label => {
    const element = document.createElement('span');
    element.className = 'environment-label';
    element.textContent = labelOverrides[label.text] ?? label.text;
    if (label.color) element.style.borderColor = label.color;
    overlay.append(element);
    return {element, point: new THREE.Vector3(...label.position)};
  });
  const projected = new THREE.Vector3();
  return {
    config,
    setLabels(visible) { overlay.hidden = !visible; },
    projectLabels(camera) {
      if (overlay.hidden) return;
      const width = container.clientWidth, height = container.clientHeight;
      for (const label of labels) {
        projected.copy(label.point).project(camera);
        const visible = projected.z >= -1 && projected.z <= 1 && Math.abs(projected.x) < .98 && Math.abs(projected.y) < .95;
        label.element.hidden = !visible;
        if (visible) label.element.style.transform = `translate(${(projected.x + 1) * width / 2}px, ${(1 - projected.y) * height / 2}px) translate(-50%, -100%)`;
      }
    },
  };
}
