// Builds the whole cell (UR5e + 2F-85 rig, printed parts, sensors, frames) from Resources/SONAIR/scene.json,
// and the translucent "ghost" arm that shows the commanded joints in Live mode.
// Works in Play mode and in Edit mode (menu SONAIR > 2 Bake Cell Into Scene).
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Sonair
{
    public static class SonairBuilder
    {
        public const int GizmoLayer = 1;   // TransparentFX: frames and frustums, hidden from the sensor cameras

        public static void Build(SonairWorld w, SScene sc)
        {
            if (w.builtRoot != null) SafeDestroy(w.builtRoot.gameObject);
            SonairData.ClearCaches();
            var root = new GameObject("SONAIR_Built").transform;
            root.SetParent(w.transform, false);

            var mats = new Dictionary<string, Material>();
            foreach (var m in sc.materials) mats[m.name] = SonairData.MakeMaterial(m);

            // ---------------- robot
            var robot = new GameObject("UR5e_2F85").transform;
            robot.SetParent(root, false);
            var rig = robot.gameObject.AddComponent<SonairRig>();
            var bodyT = new Transform[sc.bodies.Length];
            for (int i = 0; i < sc.bodies.Length; i++)
            {
                var b = sc.bodies[i];
                var t = new GameObject(b.name).transform;
                t.SetParent(b.parent < 0 ? robot : bodyT[b.parent], false);
                t.localPosition = SonairData.V3(b.pos);
                t.localRotation = SonairData.Q(b.rot);
                AddVisuals(t, b.visuals, mats);
                bodyT[i] = t;
            }
            rig.joints = new SonairJoint[sc.joints.Length];
            for (int j = 0; j < sc.joints.Length; j++)
            {
                var J = sc.joints[j]; var b = sc.bodies[J.body];
                rig.joints[j] = new SonairJoint
                {
                    name = J.name, t = bodyT[J.body], rest = SonairData.Q(b.rot), restPos = SonairData.V3(b.pos),
                    axis = SonairData.V3(b.axis).normalized, pivot = SonairData.V3(b.pivot), lo = J.lo, hi = J.hi, arm = J.group == "arm"
                };
            }
            rig.tool0 = bodyT[sc.tool0];
            rig.SetupLut(sc.gripper);

            // ---------------- objects (printed parts, specimens, cell)
            var names = new List<string>(); var ts = new List<Transform>();
            foreach (var o in sc.objects)
            {
                Transform parent = o.parent == "tool0" ? rig.tool0 : root;
                var t = new GameObject(o.name).transform;
                t.SetParent(parent, false);
                t.localPosition = SonairData.V3(o.pos);
                t.localRotation = SonairData.Q(o.rot);
                AddVisuals(t, o.visuals, mats);
                names.Add(o.name); ts.Add(t);
            }

            // ---------------- sensor optical frames (cameras are added at run time)
            var sens = new Transform[sc.sensors.Length];
            for (int i = 0; i < sc.sensors.Length; i++)
            {
                var s = sc.sensors[i];
                Transform parent = s.parent == "tool0" ? rig.tool0 : ts[names.IndexOf(s.parent)];
                var t = new GameObject(s.name).transform;
                t.SetParent(parent, false);
                t.localPosition = SonairData.V3(s.pos);
                t.localRotation = SonairData.Q(s.rot);
                sens[i] = t;
            }

            // ---------------- coordinate frames (robot convention: x red, y green, z blue)
            var framesRoot = new GameObject("Frames").transform;
            framesRoot.SetParent(root, false);
            var fT = new Transform[sc.frames.Length];
            var red = SonairData.MakeColor("axis_x", new Color(0.90f, 0.12f, 0.10f));
            var green = SonairData.MakeColor("axis_y", new Color(0.15f, 0.75f, 0.20f));
            var blue = SonairData.MakeColor("axis_z", new Color(0.15f, 0.35f, 0.95f));
            for (int i = 0; i < sc.frames.Length; i++)
            {
                var f = sc.frames[i];
                Transform parent = root;
                if (f.parent == "tool0") parent = rig.tool0;
                else if (names.Contains(f.parent)) parent = ts[names.IndexOf(f.parent)];
                else for (int k = 0; k < sc.sensors.Length; k++) if (sc.sensors[k].name == f.parent) parent = sens[k];
                var t = new GameObject("Frame_" + f.name).transform;
                t.SetParent(parent, false);
                t.localPosition = SonairData.V3(f.pos);
                t.localRotation = SonairData.Q(f.rot);
                AxisBar(t, new Vector3(0, 0, 1), f.size, red);     // x_robot -> +z_unity
                AxisBar(t, new Vector3(-1, 0, 0), f.size, green);  // y_robot -> -x_unity
                AxisBar(t, new Vector3(0, 1, 0), f.size, blue);    // z_robot -> +y_unity
                fT[i] = t;
            }

            w.builtRoot = root; w.rig = rig;
            w.objectNames = names.ToArray(); w.objectTransforms = ts.ToArray();
            w.sensorFrames = sens; w.frameTransforms = fT;
        }

        /// <summary>
        /// A copy of the robot (arm, gripper and what is rigidly on tool0) with one translucent material, no shadows,
        /// no frames. Call it before the sensor cameras are created so the copy carries no cameras.
        /// </summary>
        public static SonairRig BuildGhost(SonairWorld w)
        {
            if (w.rig == null) return null;
            var go = Object.Instantiate(w.rig.gameObject, w.rig.transform.parent);
            go.name = "Ghost_commanded";
            var g = go.GetComponent<SonairRig>();
            foreach (var t in go.GetComponentsInChildren<Transform>(true))
                if (t != null && t.name.StartsWith("Frame_")) SafeDestroy(t.gameObject);
            var gm = SonairData.MakeMaterial(new SMaterial { name = "ghost_commanded", color = new[] { 0.35f, 0.75f, 1.0f, 0.28f }, metallic = 0f, smoothness = 0.6f, texture = "" });
            foreach (var r in go.GetComponentsInChildren<MeshRenderer>(true))
            {
                var ms = new Material[r.sharedMaterials.Length];
                for (int i = 0; i < ms.Length; i++) ms[i] = gm;
                r.sharedMaterials = ms;
                r.shadowCastingMode = ShadowCastingMode.Off; r.receiveShadows = false;
            }
            go.SetActive(false);
            return g;
        }

        static void AddVisuals(Transform t, SVisual[] vis, Dictionary<string, Material> mats)
        {
            if (vis == null) return;
            foreach (var v in vis)
            {
                var mesh = SonairData.LoadMesh(v.mesh);
                if (mesh == null) continue;
                var go = new GameObject(v.mesh);
                go.transform.SetParent(t, false);
                go.AddComponent<MeshFilter>().sharedMesh = mesh;
                var mr = go.AddComponent<MeshRenderer>();
                Material m;
                mr.sharedMaterial = mats.TryGetValue(v.mat, out m) ? m : null;
                if (v.mesh.Contains("_tag_")) mr.shadowCastingMode = ShadowCastingMode.Off;
            }
        }

        static Mesh cube;
        public static Mesh Cube()
        {
            if (cube != null) return cube;
            var V = new List<Vector3>(); var N = new List<Vector3>(); var I = new List<int>();
            Vector3[] dirs = { Vector3.right, Vector3.left, Vector3.up, Vector3.down, Vector3.forward, Vector3.back };
            foreach (var n in dirs)
            {
                Vector3 a = Vector3.Cross(n, Mathf.Abs(n.y) > 0.5f ? Vector3.forward : Vector3.up);
                Vector3 b = Vector3.Cross(n, a);
                int o = V.Count;
                V.Add(0.5f * (n - a - b)); V.Add(0.5f * (n + a - b)); V.Add(0.5f * (n + a + b)); V.Add(0.5f * (n - a + b));
                for (int k = 0; k < 4; k++) N.Add(n);
                // front face = clockwise seen from outside in Unity: test with the cross product and flip if needed
                Vector3 c = Vector3.Cross(V[o + 1] - V[o], V[o + 2] - V[o]);
                if (Vector3.Dot(c, n) > 0) { I.Add(o); I.Add(o + 1); I.Add(o + 2); I.Add(o); I.Add(o + 2); I.Add(o + 3); }
                else { I.Add(o); I.Add(o + 2); I.Add(o + 1); I.Add(o); I.Add(o + 3); I.Add(o + 2); }
            }
            cube = new Mesh { name = "SONAIR_cube" };
            cube.SetVertices(V); cube.SetNormals(N); cube.SetTriangles(I, 0); cube.RecalculateBounds();
            return cube;
        }

        static void AxisBar(Transform parent, Vector3 dir, float len, Material m)
        {
            var go = new GameObject("axis");
            go.layer = GizmoLayer;
            go.transform.SetParent(parent, false);
            float w = Mathf.Max(0.0015f, len * 0.06f);
            go.transform.localPosition = dir * (len * 0.5f);
            go.transform.localScale = new Vector3(Mathf.Abs(dir.x) > 0.5f ? len : w, Mathf.Abs(dir.y) > 0.5f ? len : w, Mathf.Abs(dir.z) > 0.5f ? len : w);
            go.AddComponent<MeshFilter>().sharedMesh = Cube();
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = m; mr.shadowCastingMode = ShadowCastingMode.Off; mr.receiveShadows = false;
        }

        /// <summary>Translucent pyramid in camera space (x right, y up, z forward).</summary>
        public static Mesh Frustum(float hfovDeg, float vfovDeg, float d)
        {
            float hx = d * Mathf.Tan(0.5f * hfovDeg * Mathf.Deg2Rad), hy = d * Mathf.Tan(0.5f * vfovDeg * Mathf.Deg2Rad);
            var v = new[] { Vector3.zero, new Vector3(-hx, -hy, d), new Vector3(hx, -hy, d), new Vector3(hx, hy, d), new Vector3(-hx, hy, d) };
            var V = new List<Vector3>(); var N = new List<Vector3>(); var tri = new List<int>();
            int[,] sides = { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 } };
            for (int k = 0; k < 4; k++)
            {
                for (int s = 0; s < 2; s++)   // both windings, separate vertices -> visible from inside and outside
                {
                    Vector3 a = v[sides[k, 0]], b = v[sides[k, s == 0 ? 1 : 2]], c = v[sides[k, s == 0 ? 2 : 1]];
                    Vector3 n = Vector3.Cross(b - a, c - a).normalized;
                    int o = V.Count; V.Add(a); V.Add(b); V.Add(c); N.Add(n); N.Add(n); N.Add(n);
                    tri.Add(o); tri.Add(o + 1); tri.Add(o + 2);
                }
            }
            var m = new Mesh { name = "SONAIR_frustum" };
            m.SetVertices(V); m.SetNormals(N); m.SetTriangles(tri, 0); m.RecalculateBounds();
            return m;
        }

        public static void SafeDestroy(Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) Object.Destroy(o); else Object.DestroyImmediate(o);
        }
    }
}
