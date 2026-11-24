// D405 (A, on the camera tool) and D435i (G, on the gripper collar) as Unity cameras rendering into RenderTextures.
// The optical frame follows the RealSense / ROS convention (x right, y down, z forward); the Unity Camera is a child
// rotated by LookRotation(up, right) so that it looks along optical +z with optical -y as "up".
// Capture writes PNG + JSON (pose of the optical frame and of tool0 in robot base frame B, pinhole intrinsics, joint state).
using System;
using System.IO;
using UnityEngine;

namespace Sonair
{
    [Serializable]
    public class SonairCaptureMeta
    {
        public string image; public string sensor; public string sequence; public string phase; public float time_s;
        public int width; public int height; public float fx, fy, cx, cy; public float hfov_deg, vfov_deg;
        public float[] p_B_optical_m; public float[] R_B_optical_rowmajor;
        public float[] p_B_tool0_m; public float[] R_B_tool0_rowmajor;
        public float[] q_rad; public float gripper_closure; public float gripper_width_mm; public string held_object;
        public string note;
    }

    public class SonairSensors : IDisposable
    {
        public class Sensor
        {
            public SSensor def; public Transform frame; public Camera cam; public RenderTexture rt; public GameObject frustum;
        }

        public readonly Sensor[] items;
        readonly SonairWorld w;
        bool pipVisible = true, frustumVisible = true;
        int counter;

        public SonairSensors(SonairWorld world)
        {
            w = world;
            var defs = w.scene.sensors;
            items = new Sensor[defs.Length];
            var fm = SonairData.MakeMaterial(new SMaterial { name = "frustum_fx", color = new[] { 0.95f, 0.70f, 0.25f, 0.16f }, metallic = 0, smoothness = 0.2f, texture = "" });
            for (int i = 0; i < defs.Length; i++)
            {
                var d = defs[i];
                var s = new Sensor { def = d, frame = w.sensorFrames[i] };
                var go = new GameObject("Cam_" + d.name);
                go.transform.SetParent(s.frame, false);
                go.transform.localRotation = Quaternion.LookRotation(Vector3.up, Vector3.right);
                s.cam = go.AddComponent<Camera>();
                s.cam.fieldOfView = d.vfov; s.cam.nearClipPlane = d.near; s.cam.farClipPlane = d.far;
                s.cam.cullingMask = ~(1 << SonairBuilder.GizmoLayer);
                s.cam.clearFlags = CameraClearFlags.SolidColor; s.cam.backgroundColor = new Color(0.12f, 0.12f, 0.13f);
                s.rt = new RenderTexture(d.width, d.height, 24) { name = "RT_" + d.name, antiAliasing = 1 };
                s.rt.Create();
                s.cam.targetTexture = s.rt;
                s.cam.aspect = (float)d.width / d.height;
                s.cam.depth = -10;
                var fr = new GameObject("Frustum_" + d.name);
                fr.layer = SonairBuilder.GizmoLayer;
                fr.transform.SetParent(go.transform, false);
                fr.AddComponent<MeshFilter>().sharedMesh = SonairBuilder.Frustum(d.hfov, d.vfov, d.frustum);
                var mr = fr.AddComponent<MeshRenderer>();
                mr.sharedMaterial = fm; mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off; mr.receiveShadows = false;
                s.frustum = fr;
                items[i] = s;
            }
        }

        public void SetVisible(bool pip, bool frustum)
        {
            if (pip != pipVisible) { pipVisible = pip; foreach (var s in items) s.cam.enabled = pip; }
            if (frustum != frustumVisible) { frustumVisible = frustum; foreach (var s in items) s.frustum.SetActive(frustum); }
        }

        public int CaptureAll(string tag)
        {
            int n = 0;
            for (int i = 0; i < items.Length; i++) if (Capture(i, tag) != null) n++;
            return n;
        }

        public string Capture(int i, string tag)
        {
            var s = items[i];
            try
            {
                string dir = SonairData.CaptureDir();
                Directory.CreateDirectory(dir);
                var seq = w.mode == SonairMode.Sequence && w.Current != null ? w.Current.key : w.mode.ToString();
                string stem = string.Format("{0}_{1}_{2:000.00}s_{3}_{4:0000}", seq, s.def.name, w.mode == SonairMode.Sequence ? w.t : Time.time, tag, counter++);

                s.cam.Render();
                var prev = RenderTexture.active;
                RenderTexture.active = s.rt;
                var tex = new Texture2D(s.rt.width, s.rt.height, TextureFormat.RGB24, false);
                tex.ReadPixels(new Rect(0, 0, s.rt.width, s.rt.height), 0, 0);
                tex.Apply();
                RenderTexture.active = prev;
                File.WriteAllBytes(Path.Combine(dir, stem + ".png"), tex.EncodeToPNG());
                UnityEngine.Object.Destroy(tex);

                Transform root = w.builtRoot;
                float fy = 0.5f * s.def.height / Mathf.Tan(0.5f * s.def.vfov * Mathf.Deg2Rad);
                var pO = SonairData.ToB(root.InverseTransformPoint(s.frame.position));
                var pT = SonairData.ToB(root.InverseTransformPoint(w.rig.tool0.position));
                var meta = new SonairCaptureMeta
                {
                    image = stem + ".png", sensor = s.def.name, sequence = seq, phase = w.mode == SonairMode.Sequence ? w.PhaseAt(w.t) : "",
                    time_s = w.mode == SonairMode.Sequence ? w.t : Time.time, width = s.def.width, height = s.def.height,
                    fx = fy, fy = fy, cx = 0.5f * s.def.width, cy = 0.5f * s.def.height, hfov_deg = s.def.hfov, vfov_deg = s.def.vfov,
                    p_B_optical_m = new[] { pO.x, pO.y, pO.z },
                    R_B_optical_rowmajor = SonairData.RotToB(Quaternion.Inverse(root.rotation) * s.frame.rotation),
                    p_B_tool0_m = new[] { pT.x, pT.y, pT.z },
                    R_B_tool0_rowmajor = SonairData.RotToB(Quaternion.Inverse(root.rotation) * w.rig.tool0.rotation),
                    q_rad = (float[])w.rig.q.Clone(), gripper_closure = w.rig.closure, gripper_width_mm = w.rig.WidthMm(),
                    held_object = w.mode == SonairMode.Sequence ? w.HeldAt(w.t) : w.heldObject,
                    note = "Simulated RGB, ideal pinhole, no distortion. Optical frame: x right, y down, z forward. B = UR base frame."
                };
                File.WriteAllText(Path.Combine(dir, stem + ".json"), JsonUtility.ToJson(meta, true));
                return stem;
            }
            catch (Exception e)
            {
                Debug.LogWarning("[SONAIR] capture failed: " + e.Message);
                return null;
            }
        }

        public void Dispose()
        {
            foreach (var s in items)
            {
                if (s.cam != null) s.cam.targetTexture = null;
                if (s.rt != null) { s.rt.Release(); UnityEngine.Object.Destroy(s.rt); }
            }
        }
    }
}
