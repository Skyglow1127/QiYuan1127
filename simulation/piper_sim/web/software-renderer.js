import * as THREE from 'three';

// Canvas fallback for embedded browsers without WebGL. Geometry normals are
// cached, but all transforms still come from the live Gazebo joint feedback.
export class SoftwareRenderer {
  constructor(canvas) {
    this.canvas = canvas; this.ctx = canvas.getContext('2d');
    this.shadowMap = {}; this.w = 1; this.h = 1;
    this.cache = new WeakMap();
  }
  setPixelRatio() {}
  setSize(w, h) {
    this.w = Math.round(w); this.h = Math.round(h);
    this.canvas.width = this.w; this.canvas.height = this.h;
    this.frame = this.ctx.createImageData(this.w, this.h);
    this.depth = new Float32Array(this.w * this.h);
  }
  setAnimationLoop(fn) {
    let last = 0;
    const tick = time => { requestAnimationFrame(tick); if (time - last > 66) { last = time; fn(); } };
    requestAnimationFrame(tick);
  }
  geometryData(geometry) {
    if (this.cache.has(geometry)) return this.cache.get(geometry);
    const position = geometry.attributes.position, index = geometry.index;
    const vertices = Array.from({length: position.count}, (_, i) => new THREE.Vector3().fromBufferAttribute(position, i));
    const faces = [];
    const count = index ? index.count : position.count;
    for (let i = 0; i < count; i += 3) {
      const ids = [0, 1, 2].map(n => index ? index.getX(i + n) : i + n);
      const [a, b, c] = ids.map(n => vertices[n]);
      const normal = b.clone().sub(a).cross(c.clone().sub(a)).normalize();
      faces.push({ids, normal, point: a});
    }
    const data = {vertices, faces}; this.cache.set(geometry, data); return data;
  }
  render(scene, camera) {
    const c = this.ctx, w = this.w, h = this.h;
    if (!this.frame) this.setSize(w, h);
    const pixels = this.frame.data, depth = this.depth;
    pixels.fill(0); depth.fill(Infinity);
    scene.updateMatrixWorld(true); camera.updateMatrixWorld(true);
    const vp = new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
    const frustum = new THREE.Frustum().setFromProjectionMatrix(vp);
    const light = new THREE.Vector3(-1, 1, 3).normalize();
    const normalMatrix = new THREE.Matrix3(), inverse = new THREE.Matrix4(), localCamera = new THREE.Vector3();
    // Clip at the near plane before projection, avoiding huge inverted faces
    // when the viewer orbits through the enclosure or zooms into the arm.
    const clipNear = polygon => {
      const output = [];
      for (let i = 0; i < polygon.length; i++) {
        const a = polygon[i], b = polygon[(i + 1) % polygon.length];
        const da = a.z + a.w, db = b.z + b.w;
        if (da >= 0) output.push(a);
        if ((da >= 0) !== (db >= 0)) output.push(a.clone().lerp(b, da / (da - db)));
      }
      return output;
    };
    // A pixel depth buffer matters for the thin PCB components and checkerboard
    // tiles. Sorting whole triangles can incorrectly paint their support panel
    // over them even when the geometry has the correct physical separation.
    const rasterize = (a, b, d, rgb) => {
      const area = (b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0]);
      if (Math.abs(area) < .001) return;
      const minX = Math.max(0, Math.floor(Math.min(a[0], b[0], d[0])));
      const maxX = Math.min(w - 1, Math.ceil(Math.max(a[0], b[0], d[0])));
      const minY = Math.max(0, Math.floor(Math.min(a[1], b[1], d[1])));
      const maxY = Math.min(h - 1, Math.ceil(Math.max(a[1], b[1], d[1])));
      const bx = (d[1] - a[1]) / area, by = -(d[0] - a[0]) / area;
      const dx = -(b[1] - a[1]) / area, dy = (b[0] - a[0]) / area;
      const bz = b[2] - a[2], dz = d[2] - a[2];
      let rowB = ((minX + .5 - a[0]) * (d[1] - a[1]) - (minY + .5 - a[1]) * (d[0] - a[0])) / area;
      let rowD = ((b[0] - a[0]) * (minY + .5 - a[1]) - (b[1] - a[1]) * (minX + .5 - a[0])) / area;
      for (let y = minY; y <= maxY; y++, rowB += by, rowD += dy) {
        let wb = rowB, wd = rowD;
        for (let x = minX; x <= maxX; x++, wb += bx, wd += dx) {
          if (wb < -1e-7 || wd < -1e-7 || wb + wd > 1.0000001) continue;
          const z = a[2] + wb * bz + wd * dz, index = y * w + x;
          if (z >= depth[index]) continue;
          depth[index] = z;
          const p = index * 4;
          pixels[p] = rgb[0]; pixels[p + 1] = rgb[1]; pixels[p + 2] = rgb[2]; pixels[p + 3] = 255;
        }
      }
    };
    scene.traverseVisible(object => {
      if (!object.isMesh || !frustum.intersectsObject(object)) return;
      const {vertices, faces} = this.geometryData(object.geometry);
      const transform = new THREE.Matrix4().multiplyMatrices(vp, object.matrixWorld);
      const projected = vertices.map(p => new THREE.Vector4(p.x, p.y, p.z, 1).applyMatrix4(transform));
      inverse.copy(object.matrixWorld).invert(); localCamera.copy(camera.position).applyMatrix4(inverse);
      normalMatrix.getNormalMatrix(object.matrixWorld);
      const base = object.material.color.clone().convertLinearToSRGB();
      for (const face of faces) {
        if (face.normal.dot(localCamera.clone().sub(face.point)) <= 0) continue;
        let polygon = face.ids.map(i => projected[i]);
        if (polygon.every(p => p.z < -p.w) || polygon.every(p => p.z > p.w)) continue;
        if (polygon.some(p => p.z < -p.w)) polygon = clipNear(polygon);
        if (polygon.length < 3) continue;
        const points = polygon.map(p => [(p.x / p.w + 1) * w / 2, (1 - p.y / p.w) * h / 2, p.z / p.w]);
        if (points.every(p => p[0] < 0) || points.every(p => p[0] > w) || points.every(p => p[1] < 0) || points.every(p => p[1] > h)) continue;
        const normal = face.normal.clone().applyMatrix3(normalMatrix).normalize();
        const intensity = .62 + .38 * Math.max(0, normal.dot(light));
        const rgb = [base.r, base.g, base.b].map(value => Math.round(value * 255 * intensity));
        for (let i = 1; i < points.length - 1; i++) rasterize(points[0], points[i], points[i + 1], rgb);
      }
    });
    c.putImageData(this.frame, 0, 0);
    if (scene.userData.showGrid) {
      c.strokeStyle = '#395b7666'; c.lineWidth = .7;
      for (let i = 0; i <= 12; i++) {
        for (let axis = 0; axis < 2; axis++) {
          const a = new THREE.Vector3(axis ? -.4 + i * .1 : -.4, axis ? -.88 : -.88 + i * .1, .003).project(camera);
          const b = new THREE.Vector3(axis ? -.4 + i * .1 : .82, axis ? .30 : -.88 + i * .1, .003).project(camera);
          if (a.z < -1 || a.z > 1 || b.z < -1 || b.z > 1) continue;
          c.beginPath(); c.moveTo((a.x + 1) * w / 2, (1 - a.y) * h / 2); c.lineTo((b.x + 1) * w / 2, (1 - b.y) * h / 2); c.stroke();
        }
      }
    }
  }
}
