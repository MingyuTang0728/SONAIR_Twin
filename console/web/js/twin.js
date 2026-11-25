/* The lab twin: the Unity cell and room rebuilt in three.js from the Unity project's own data (scene.json, lab.json,
   sequences.json, SNM1 meshes, PNG textures), served read-only by the agent under /lab/.

   Unity is left-handed (x right, y up, z forward). Everything Unity describes is built in Unity numbers under one
   root group whose matrix is P (x_B = z_u, y_B = -x_u, z_B = y_u), so the world of this view is the robot base frame
   B with z up, the same frame as every log. The arm follows the rig of SonairRig.cs exactly:
     localRotation = rest * AngleAxis(q, axis), localPosition = restPos + rest * (pivot - AngleAxis(q, axis) * pivot).

   Modes: LIVE (measured joints, the commanded arm as a translucent ghost), PREVIEW (a kit sequence; the ghost is
   then the live arm) and REPLAY (a recorded run). The mode is always written on the view: a preview must never be
   taken for the arm. */
import * as THREE from "three";
import { OrbitControls } from "../vendor/three/OrbitControls.js";

const P = new THREE.Matrix4().set(0, 0, 1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);
const SKIP_LAB = /^(ceiling|led)/;

function parseSNM(buf) {
  const dv = new DataView(buf);
  if (dv.getUint8(0) !== 83 || dv.getUint8(1) !== 78 || dv.getUint8(2) !== 77 || dv.getUint8(3) !== 49) throw new Error("not an SNM1 mesh");
  const nv = dv.getInt32(4, true), ni = dv.getInt32(8, true), flags = dv.getInt32(12, true);
  let o = 16;
  const pos = new Float32Array(buf, o, nv * 3); o += nv * 12;
  const nrm = new Float32Array(buf, o, nv * 3); o += nv * 12;
  let uv = null; if (flags & 2) { uv = new Float32Array(buf, o, nv * 2); o += nv * 8; }
  const idx = new Uint32Array(buf.slice(o, o + ni * 4));
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  g.setAttribute("normal", new THREE.BufferAttribute(nrm, 3));
  if (uv) g.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  g.setIndex(new THREE.BufferAttribute(idx, 1));
  g.computeBoundingSphere();
  return g;
}
const V3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
const Q4 = (a) => new THREE.Quaternion(a[0], a[1], a[2], a[3]);

export class Twin {
  constructor() {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.02, 60);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.75, -1.35, 1.25);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.15, 0, 0.3);
    this.controls.maxDistance = 3.6; this.controls.minDistance = 0.15; this.controls.enableDamping = false;
    this.controls.addEventListener("change", () => (this.dirty = true));
    this.root = new THREE.Group(); this.root.matrixAutoUpdate = false; this.root.matrix.copy(P); this.scene.add(this.root);
    this.labRoot = new THREE.Group(); this.root.add(this.labRoot);
    this.setBackground();
    const hemi = new THREE.HemisphereLight(0xf2f0ea, 0x4a4844, 1.15); hemi.position.set(0, 0, 1); this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.6); key.position.set(1.8, -1.2, 3.0); this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xdde6f0, 0.55); fill.position.set(-2, 1.5, 1.2); this.scene.add(fill);
    this.geo = new Map(); this.tex = new Map(); this.mats = {};
    this.bodies = []; this.joints = []; this.ghost = null; this.objects = {}; this.sensors = {}; this.frames = [];
    this.q = [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]; this.qShown = this.q.slice(); this.qGhost = null; this.closure = 0;
    this.mode = "live"; this.opts = { lab: true, ghost: true, frames: false, frustum: true };
    this.dirty = true; this.ready = false; this.onProgress = null; this.overlay = null; this.eye = null;
    this.loop = this.loop.bind(this); this.last = performance.now(); requestAnimationFrame(this.loop);
    this.ls = { seek: new Set(), mode: new Set() };
  }
  on(ev, fn) { this.ls[ev].add(fn); }
  emit(ev, x) { this.ls[ev].forEach((f) => { try { f(x); } catch (e) { console.error(e); } }); }

  setBackground() {
    const light = document.documentElement.dataset.theme === "light";
    this.scene.background = new THREE.Color(light ? 0xcfd2d4 : 0x0c0d0e);
    this.dirty = true;
  }

  /* ---------------------------------------------------------------- mounting (one renderer, moved between slots) */
  mount(container) {
    if (!container) return;
    this.container = container;
    container.classList.add("viewport");
    container.innerHTML = `<div class="vtools">
        <button data-v="fit">Fit</button><button data-v="top">Top</button><button data-v="eyeG">D435i eye</button><button data-v="eyeA">D405 eye</button>
        <button data-v="lab" aria-pressed="true">Lab</button><button data-v="ghost" aria-pressed="true">Ghost</button><button data-v="frames" aria-pressed="false">Frames</button>
        <button data-v="live" hidden>Back to live</button></div>
      <div class="vempty">Loading the cell from the Unity scene...</div>
      <div class="vlabel">Twin view &middot; not a measurement</div><div class="vmode">LIVE</div><div class="vprog"></div>`;
    container.prepend(this.renderer.domElement);
    this.overlay = { empty: container.querySelector(".vempty"), mode: container.querySelector(".vmode"), prog: container.querySelector(".vprog"), tools: container.querySelector(".vtools") };
    this.overlay.tools.onclick = (e) => { const b = e.target.closest("button"); if (b) this.tool(b.dataset.v, b); };
    this.paintTools(); this.paintMode();
    if (this.ready) this.overlay.empty.hidden = true;
    if (!this.ro) { this.ro = new ResizeObserver(() => this.resize()); }
    this.ro.disconnect(); this.ro.observe(container); this.resize();
  }
  resize() {
    if (!this.container) return;
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix(); this.dirty = true;
  }
  tool(v, b) {
    if (v === "fit") this.view("fit");
    else if (v === "top") this.view("top");
    else if (v === "eyeG") this.view("eye", "G_D435i");
    else if (v === "eyeA") this.view("eye", "A_D405");
    else if (v === "live") this.setMode("live");
    else { this.opts[v] = !this.opts[v]; this.apply(); }
    this.paintTools();
  }
  paintTools() {
    if (!this.overlay) return;
    this.overlay.tools.querySelectorAll("button[aria-pressed]").forEach((b) => b.setAttribute("aria-pressed", String(!!this.opts[b.dataset.v])));
    const lb = this.overlay.tools.querySelector("[data-v=live]"); if (lb) lb.hidden = this.mode === "live";
  }
  paintMode() {
    if (!this.overlay) return;
    const m = this.overlay.mode;
    m.className = "vmode " + this.mode;
    m.textContent = this.mode === "live" ? (this.eye ? `LIVE · VIEW FROM ${this.eye}` : "LIVE") : this.mode === "preview" ? "PREVIEW · ghost = live arm" : "REPLAY · ghost = live arm";
  }
  eyeFollow() {
    const s = this.sensors[this.eyeSensor]; if (!s) return;
    s.node.updateWorldMatrix(true, false);
    const m = s.node.matrixWorld, p = new THREE.Vector3().setFromMatrixPosition(m);
    this.camera.position.copy(p); this.camera.up.copy(new THREE.Vector3(0, 1, 0).transformDirection(m));
    this.camera.lookAt(p.clone().add(new THREE.Vector3(0, 0, 1).transformDirection(m)));
  }
  view(kind, sensor) {
    this.eye = null; this.controls.enabled = true; this.camera.fov = 38;
    if (kind === "fit") { this.camera.position.set(1.75, -1.35, 1.25); this.controls.target.set(0.15, 0, 0.3); }
    if (kind === "top") { this.camera.position.set(0.2, -0.001, 2.6); this.controls.target.set(0.2, 0, 0); }
    if (kind === "eye" && this.sensors[sensor]) {
      const s = this.sensors[sensor]; s.node.updateWorldMatrix(true, false);
      const m = s.node.matrixWorld, p = new THREE.Vector3().setFromMatrixPosition(m);
      const fwd = new THREE.Vector3(0, 0, 1).transformDirection(m), up = new THREE.Vector3(0, 1, 0).transformDirection(m);
      this.camera.position.copy(p); this.camera.up.copy(up); this.camera.lookAt(p.clone().add(fwd)); this.camera.fov = s.vfov;
      this.camera.updateProjectionMatrix(); this.controls.enabled = false; this.eye = sensor === "G_D435i" ? "D435i" : "D405"; this.eyeSensor = sensor;
      this.paintMode(); this.dirty = true; return;
    }
    this.camera.up.set(0, 0, 1); this.camera.updateProjectionMatrix(); this.controls.update(); this.paintMode(); this.dirty = true;
  }

  /* ---------------------------------------------------------------- loading */
  async fetchJSON(p) { const r = await fetch("lab/" + p); if (!r.ok) throw new Error(p + ": HTTP " + r.status); return r.json(); }
  async mesh(name) {
    if (this.geo.has(name)) return this.geo.get(name);
    const pr = fetch(`lab/Meshes/${name}.bytes`).then((r) => { if (!r.ok) throw new Error(name); return r.arrayBuffer(); }).then(parseSNM);
    this.geo.set(name, pr); return pr;
  }
  texture(key, repeat) {
    const id = key + (repeat ? repeat.join("x") : "");
    if (this.tex.has(id)) return this.tex.get(id);
    const t = new THREE.TextureLoader().load(`lab/Textures/${key}.bytes`, () => (this.dirty = true));
    t.colorSpace = THREE.SRGBColorSpace;
    if (repeat) { t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(repeat[0], repeat[1]); t.anisotropy = 4; }
    this.tex.set(id, t); return t;
  }
  material(m, opts = {}) {
    const { repeat, ...extra } = opts;
    const c = m.color || [0.7, 0.7, 0.7, 1];
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color(c[0], c[1], c[2]).convertSRGBToLinear(), metalness: Math.min(0.9, m.metallic || 0),
      roughness: Math.max(0.08, 1 - (m.smoothness ?? 0.5) * 0.92), transparent: c[3] < 0.999, opacity: c[3], side: THREE.FrontSide, ...extra });
    if (m.texture) { mat.map = this.texture(m.texture, repeat); mat.color = new THREE.Color(1, 1, 1); }
    if (m.emission && m.emission.length >= 3 && m.emission[0] + m.emission[1] + m.emission[2] > 0) { mat.emissive = new THREE.Color(m.emission[0], m.emission[1], m.emission[2]); if (mat.map) mat.emissiveMap = mat.map; }
    return mat;
  }

  async load() {
    try {
      const sc = await this.fetchJSON("scene.json");
      this.sc = sc;
      sc.materials.forEach((m) => (this.mats[m.name] = this.material(m)));
      this.ghostMat = new THREE.MeshStandardMaterial({ color: 0x59c2ff, transparent: true, opacity: 0.26, depthWrite: false, roughness: 0.6 });
      // robot first: it is what the operator needs to see
      this.rig = this.buildRig(sc, false); this.root.add(this.rig.group);
      this.ghost = this.buildRig(sc, true); this.root.add(this.ghost.group);
      this.tool0 = this.rig.bodies[sc.tool0];
      let n = 0; const all = [];
      const add = (node, vis, mat) => (vis || []).forEach((v) => all.push([node, v, mat]));
      sc.bodies.forEach((b, i) => { add(this.rig.bodies[i], b.visuals); add(this.ghost.bodies[i], b.visuals, this.ghostMat); });
      sc.objects.forEach((o) => {
        const g = new THREE.Group(); g.name = o.name; g.position.copy(V3(o.pos)); g.quaternion.copy(Q4(o.rot));
        (o.parent === "tool0" ? this.tool0 : this.root).add(g);
        this.objects[o.name] = { node: g, home: { parent: o.parent || "world", pos: o.pos, rot: o.rot }, label: o.label, dynamic: o.dynamic };
        add(g, o.visuals);
      });
      sc.sensors.forEach((s) => this.buildSensor(s));
      sc.frames.forEach((f) => this.buildFrame(f));
      const robotFirst = all.filter((a) => /^(ur_|rq_)/.test(a[1].mesh)).concat(all.filter((a) => !/^(ur_|rq_)/.test(a[1].mesh)));
      await Promise.all(robotFirst.map(async ([node, v, mat], k) => {
        try {
          const g = await this.mesh(v.mesh);
          const m = new THREE.Mesh(g, mat || this.mats[v.mat] || this.mats.ur_link);
          m.name = v.mesh; node.add(m);
        } catch (e) { console.warn("mesh", v.mesh, e); }
        n++; this.progress(n / robotFirst.length * 0.8); this.dirty = true;
        if (n === 40 && this.overlay) this.overlay.empty.hidden = true;
      }));
      this.ready = true;
      if (this.overlay) this.overlay.empty.hidden = true;
      this.setArm(this.q); this.apply();
      await this.loadLab();
      this.progress(1);
    } catch (e) {
      console.error(e);
      if (this.overlay) { this.overlay.empty.hidden = false; this.overlay.empty.textContent = "Could not build the cell from the Unity scene: " + e.message; }
    }
  }
  progress(f) { if (this.overlay) { this.overlay.prog.style.width = (f * 100).toFixed(0) + "%"; if (f >= 1) setTimeout(() => (this.overlay.prog.style.width = "0"), 600); } }

  buildRig(sc, ghost) {
    const group = new THREE.Group(); group.name = ghost ? "ghost" : "robot";
    const bodies = sc.bodies.map((b) => { const o = new THREE.Group(); o.name = b.name; o.position.copy(V3(b.pos)); o.quaternion.copy(Q4(b.rot)); return o; });
    sc.bodies.forEach((b, i) => (b.parent < 0 ? group : bodies[b.parent]).add(bodies[i]));
    const joints = sc.joints.map((J) => { const b = sc.bodies[J.body]; return { t: bodies[J.body], rest: Q4(b.rot), restPos: V3(b.pos), axis: V3(b.axis).normalize(), pivot: V3(b.pivot) }; });
    return { group, bodies, joints };
  }
  applyJoint(rig, i, rad) {
    const j = rig.joints[i]; if (!j) return;
    const r = new THREE.Quaternion().setFromAxisAngle(j.axis, rad);
    j.t.quaternion.copy(j.rest).multiply(r);
    const p = j.pivot.clone().sub(j.pivot.clone().applyQuaternion(r)).applyQuaternion(j.rest);
    j.t.position.copy(j.restPos).add(p);
  }
  gripperLut(c) {
    const g = this.sc.gripper, n = g.closure.length, x = Math.max(0, Math.min(1, c)) * (n - 1);
    const i0 = Math.min(n - 1, Math.floor(x)), i1 = Math.min(n - 1, i0 + 1), f = x - i0;
    return Array.from({ length: g.nq }, (_, k) => g.q[i0 * g.nq + k] + (g.q[i1 * g.nq + k] - g.q[i0 * g.nq + k]) * f);
  }
  setArm(q, rig = this.rig) { if (!rig) return; for (let i = 0; i < 6; i++) this.applyJoint(rig, i, q[i]); this.dirty = true; }
  setGripper(c, rig = this.rig) { if (!rig || !this.sc) return; this.gripperLut(c).forEach((v, k) => this.applyJoint(rig, 6 + k, v)); this.dirty = true; }

  buildSensor(s) {
    const parent = s.parent === "tool0" ? this.tool0 : this.objects[s.parent] && this.objects[s.parent].node;
    if (!parent) return;
    const node = new THREE.Group(); node.position.copy(V3(s.pos)); node.quaternion.copy(Q4(s.rot)); parent.add(node);
    const d = s.frustum || 0.1, hx = d * Math.tan((s.hfov * Math.PI) / 360), hy = d * Math.tan((s.vfov * Math.PI) / 360);
    const c = [[-hx, -hy, d], [hx, -hy, d], [hx, hy, d], [-hx, hy, d]], pts = [];
    c.forEach((p, k) => { pts.push(0, 0, 0, ...p); const q = c[(k + 1) % 4]; pts.push(...p, ...q); });
    const geo = new THREE.BufferGeometry(); geo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    const lines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: s.name.startsWith("G") ? 0x23a7a1 : 0xec7a2c, transparent: true, opacity: 0.85 }));
    node.add(lines);
    this.sensors[s.name] = { node, lines, vfov: s.vfov, label: s.label };
  }
  buildFrame(f) {
    let parent = this.root;
    if (f.parent === "tool0") parent = this.tool0;
    else if (this.objects[f.parent]) parent = this.objects[f.parent].node;
    else if (this.sensors[f.parent]) parent = this.sensors[f.parent].node;
    const g = new THREE.Group(); g.position.copy(V3(f.pos)); g.quaternion.copy(Q4(f.rot)); parent.add(g);
    const s = f.size || 0.05;
    [[0, 0, 1, 0xe5484d], [-1, 0, 0, 0x57b36b], [0, 1, 0, 0x5aa9e6]].forEach(([x, y, z, col]) => {     // robot x, y, z in Unity axes
      const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(x * s, y * s, z * s)]);
      g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: col, depthTest: false })));
    });
    g.renderOrder = 5; g.visible = false; this.frames.push(g);
  }

  async loadLab() {
    try {
      const lab = await this.fetchJSON("lab.json");
      const mats = {}; lab.materials.forEach((m) => (mats[m.name] = m));
      for (const p of lab.prims) {
        if (SKIP_LAB.test(p.name)) continue;
        const m = mats[p.mat] || {}; const s = p.size;
        let geo;
        if (p.type === "cyl") geo = new THREE.CylinderGeometry(s[0] / 2, s[0] / 2, s[1], 24);
        else geo = new THREE.BoxGeometry(s[0], s[1], s[2]);
        const big = [...s].sort((a, b) => b - a);
        const rep = m.texture && !p.fit ? [Math.max(1, big[0] / (p.uv ? p.uv[0] : 1)), Math.max(1, big[1] / (p.uv ? p.uv[1] : 1))] : null;
        const mat = this.material(m, rep ? { repeat: rep } : {});
        const mesh = new THREE.Mesh(geo, mat); mesh.position.copy(V3(p.pos)); mesh.quaternion.copy(Q4(p.rot)); mesh.name = p.name;
        // three.js boxes wind for a right-handed frame; under the mirroring root they are seen from inside: draw both sides
        mat.side = THREE.DoubleSide;
        this.labRoot.add(mesh);
      }
      this.labHide = lab.hide || [];
      this.apply();
    } catch (e) { console.warn("lab", e); }
  }

  apply() {
    const vis = (o, v) => { if (o && o.visible !== v) { o.visible = v; this.dirty = true; } };
    vis(this.labRoot, !!this.opts.lab);
    (this.labHide || []).forEach((n) => this.objects[n] && vis(this.objects[n].node, !this.opts.lab));
    if (this.ghost) vis(this.ghost.group, !!this.opts.ghost && (this.mode !== "live" || !!this.qGhost));
    this.frames.forEach((f) => vis(f, !!this.opts.frames));
    Object.values(this.sensors).forEach((s) => vis(s.lines, !!this.opts.frustum));
  }

  /* ---------------------------------------------------------------- live, preview, replay */
  live(q, tq, closure) {
    const same = (a, b) => a && b && a.every((v, i) => Math.abs(v - b[i]) < 2e-5);
    let ghostQ;
    if (this.mode === "live") {
      if (q) this.q = q.slice();
      ghostQ = tq ? tq.slice() : null;
      if (closure !== null && closure !== undefined) this.closure = closure;
    } else {
      ghostQ = q ? q.slice() : null;                                 // in a preview the ghost is the real arm
    }
    const gc = closure !== null && closure !== undefined ? closure : null;
    if (this.ghost && ghostQ && (!same(ghostQ, this.qGhost) || gc !== this._gcl)) {
      this.setArm(ghostQ, this.ghost); if (gc !== null) this.setGripper(gc, this.ghost); this._gcl = gc;
    }
    this.qGhost = ghostQ;
    this.apply();
  }
  setMode(m) {
    this.mode = m;
    if (m === "live") { this.stopPlayer(); this.resetObjects(); this.qShown = this.q.slice(); }
    this.paintMode(); this.paintTools(); this.apply();
    this.emit("mode", m);
  }
  resetObjects() {
    Object.values(this.objects).forEach((o) => { if (o.dynamic) this.attach(o, o.home); });
  }
  attach(o, tr) {
    const parent = tr.parent === "tool0" ? this.tool0 : this.root;
    if (o.node.parent !== parent) parent.add(o.node);
    o.node.position.copy(V3(tr.pos)); o.node.quaternion.copy(Q4(tr.rot)); this.dirty = true;
  }
  async sequences() {
    if (!this.seqs) this.seqs = this.fetchJSON("sequences.json").then((d) => d.sequences);
    return this.seqs;
  }
  async preview(key) {
    const seqs = await this.sequences(); const s = seqs.find((x) => x.key === key); if (!s) return null;
    this.stopPlayer();
    this.player = { kind: "seq", s, t: 0, dur: s.duration, fps: s.fps, n: s.grip.length, playing: false, rate: 1 };
    this.mode = "preview"; this.paintMode(); this.paintTools(); this.seek(0); this.apply();
    this.emit("mode", "preview");
    return s;
  }
  replay(run) {
    // run: {t: [...], q: [[...6], ...] one row per sample, name}
    this.stopPlayer();
    this.player = { kind: "run", run, t: 0, dur: run.t[run.t.length - 1] || 0, playing: false, rate: 1 };
    this.mode = "replay"; this.paintMode(); this.paintTools(); this.seek(0); this.apply();
    this.emit("mode", "replay");
  }
  seek(t) {
    const p = this.player; if (!p) return;
    p.t = Math.max(0, Math.min(p.dur, t));
    if (p.kind === "seq") {
      const k = Math.min(p.n - 1, Math.floor(p.t * p.fps)), q = p.s.q.slice(k * 6, k * 6 + 6);
      this.qShown = q; this.setArm(q); this.setGripper(p.s.grip[k]);
      const last = {};
      p.s.tracks.forEach((tr) => { if (tr.t <= p.t + 1e-6) last[tr.obj] = tr; });
      Object.entries(last).forEach(([obj, tr]) => this.objects[obj] && this.attach(this.objects[obj], tr));
    } else {
      const T = p.run.t; let lo = 0, hi = T.length - 1;
      while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (T[mid] < p.t) lo = mid; else hi = mid; }
      const q = p.run.q[lo]; if (q && q.every((v) => v !== null)) { this.qShown = q; this.setArm(q); }
    }
    this.emit("seek", p);
    this.dirty = true;
  }
  play(on = true) { if (this.player) { this.player.playing = on; if (on && this.player.t >= this.player.dur) this.player.t = 0; } }
  stopPlayer() { if (this.player) this.player.playing = false; this.player = null; }

  /* ---------------------------------------------------------------- render loop: on demand */
  loop(now) {
    const dt = Math.min(0.1, (now - this.last) / 1000); this.last = now;
    if (this.player && this.player.playing) { this.seek(this.player.t + dt * this.player.rate); if (this.player.t >= this.player.dur) this.player.playing = false; }
    if (this.mode === "live" && this.rig) {
      let d = 0; const k = Math.min(1, dt * 14);
      for (let i = 0; i < 6; i++) { const e = this.q[i] - this.qShown[i]; d = Math.max(d, Math.abs(e)); this.qShown[i] += e * k; }
      if (d > 1e-5) this.setArm(this.qShown);
      if (this.closure !== this._cl) { this.setGripper(this.closure); this._cl = this.closure; }
    }
    if (this.eye && this.dirty) this.eyeFollow();
    if (this.dirty && this.container && this.renderer.domElement.offsetParent) { this.dirty = false; this.renderer.render(this.scene, this.camera); }
    requestAnimationFrame(this.loop);
  }
}

export const twin = new Twin();
