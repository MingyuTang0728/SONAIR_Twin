// SONAIR cell twin: main controller.
// Put this component on an empty GameObject (menu SONAIR > 1 Create and Open Demo Scene does it) and press Play.
//   Sequence : replays the pre-computed trajectories (Mode A/B on S2 and S4, two hand-eye calibrations)
//   Manual   : joint and gripper sliders, key poses, which object is in the hand
//   Live     : mirrors the real UR5e + gripper from UDP; if the sender includes target_q, a translucent ghost shows
//              the commanded arm, so the controller's tracking error is visible while the cell runs
// Unity is a viewer and an authoring tool here: nothing in this package commands the robot.
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sonair
{
    public enum SonairMode { Sequence = 0, Manual = 1, Live = 2 }

    [DisallowMultipleComponent]
    public class SonairWorld : MonoBehaviour
    {
        // ---- filled by SonairBuilder (serialised, so a baked scene keeps them)
        [HideInInspector] public Transform builtRoot;
        [HideInInspector] public SonairRig rig;
        [HideInInspector] public string[] objectNames = new string[0];
        [HideInInspector] public Transform[] objectTransforms = new Transform[0];
        [HideInInspector] public Transform[] sensorFrames = new Transform[0];
        [HideInInspector] public Transform[] frameTransforms = new Transform[0];

        [Header("Start-up")]
        public SonairMode startMode = SonairMode.Sequence;
        public int startSequence = 0;
        public bool autoPlay = true;
        [Header("Environment")]
        public bool buildLab = true;
        [Header("Live mirror (UDP)")]
        public int udpPort = 5005;
        [Header("Capture")]
        public float autoCaptureHz = 2f;

        // ---- runtime state
        [NonSerialized] public SScene scene;
        [NonSerialized] public SSequence[] sequences = new SSequence[0];
        [NonSerialized] public SonairMode mode;
        [NonSerialized] public int seqIndex;
        [NonSerialized] public float t;
        [NonSerialized] public bool playing;
        [NonSerialized] public float speed = 1f;
        [NonSerialized] public bool loop = true;
        [NonSerialized] public float[] manualQ = new float[6];
        [NonSerialized] public float manualClosure;
        [NonSerialized] public string heldObject = "";
        [NonSerialized] public bool showFrames = true, showFrustums = true, showPip = true, showGhost = true, autoCapture, sensorFullscreen;
        [NonSerialized] public SonairSensors sensors;
        [NonSerialized] public SonairUdp udp;
        [NonSerialized] public Camera mainCam;
        [NonSerialized] public bool scanning;
        [NonSerialized] public string scanningSensor = "";
        [NonSerialized] public SonairRig ghost;
        [NonSerialized] public bool ghostActive;
        [NonSerialized] public float trackErrDeg, trackErrMm;
        [NonSerialized] public Transform labRoot;

        readonly Dictionary<string, Transform> objByName = new Dictionary<string, Transform>();
        readonly Dictionary<string, Pose> defaultPose = new Dictionary<string, Pose>();
        readonly Dictionary<string, Pose> grasp = new Dictionary<string, Pose>();
        readonly List<string> dynamicObjects = new List<string>();
        readonly List<GameObject> hiddenByLab = new List<GameObject>();
        Dictionary<string, List<STrack>> tracks = new Dictionary<string, List<STrack>>();
        readonly float[] qa = new float[6];
        float captureClock;
        Light keyLight;

        public SSequence Current { get { return sequences.Length > 0 ? sequences[Mathf.Clamp(seqIndex, 0, sequences.Length - 1)] : null; } }
        public bool LabVisible { get { return labRoot != null && labRoot.gameObject.activeSelf; } }

        void Awake()
        {
            scene = SonairData.LoadScene();
            if (scene == null) { enabled = false; return; }
            sequences = SonairData.LoadSequences();
            if (builtRoot == null || rig == null || rig.joints == null || rig.joints.Length == 0) SonairBuilder.Build(this, scene);
            if (rig.lutClosure == null || rig.lutClosure.Length == 0) rig.SetupLut(scene.gripper);

            objByName.Clear();
            for (int i = 0; i < objectNames.Length; i++) objByName[objectNames[i]] = objectTransforms[i];
            foreach (var o in scene.objects)
            {
                if (!o.dynamic) continue;
                dynamicObjects.Add(o.name);
                defaultPose[o.name] = new Pose(SonairData.V3(o.pos), SonairData.Q(o.rot));
            }
            foreach (var g in scene.grasps) grasp[g.obj] = new Pose(SonairData.V3(g.pos), SonairData.Q(g.rot));
            for (int k = 0; k < 6; k++) manualQ[k] = scene.home[k] * Mathf.Rad2Deg;

            ghost = SonairBuilder.BuildGhost(this);          // before the sensor cameras exist, so the ghost carries none
            SetupCameraAndLight();
            if (buildLab) BuildLab();
            sensors = new SonairSensors(this);
            udp = new SonairUdp();
            if (GetComponent<SonairUI>() == null) gameObject.AddComponent<SonairUI>();

            mode = startMode;
            SelectSequence(Mathf.Clamp(startSequence, 0, Mathf.Max(0, sequences.Length - 1)));
            playing = autoPlay;
            if (mode == SonairMode.Live) StartLive();
        }

        void OnDestroy()
        {
            if (udp != null) udp.Stop();
            if (sensors != null) sensors.Dispose();
        }

        void SetupCameraAndLight()
        {
            mainCam = Camera.main;
            if (mainCam == null)
            {
                var go = new GameObject("Main Camera") { tag = "MainCamera" };
                mainCam = go.AddComponent<Camera>();
            }
            mainCam.nearClipPlane = 0.01f; mainCam.farClipPlane = 60f; mainCam.fieldOfView = 40f;
            var lgo = GameObject.Find("Directional Light");
            if (lgo == null)
            {
                lgo = new GameObject("Directional Light");
                var l = lgo.AddComponent<Light>();
                l.type = LightType.Directional; l.intensity = 1.1f; l.shadows = LightShadows.Soft;
                lgo.transform.rotation = Quaternion.Euler(52f, -35f, 0f);
            }
            keyLight = lgo.GetComponent<Light>();
        }

        void BuildLab()
        {
            var lab = SonairLab.Load();
            if (lab == null) return;
            labRoot = SonairLab.Build(builtRoot, lab);
            if (lab.hide != null) foreach (var n in lab.hide) { var tr = FindObject(n); if (tr != null) hiddenByLab.Add(tr.gameObject); }
            foreach (var go in hiddenByLab) go.SetActive(false);
            if (keyLight != null && lab.keyLightEuler != null && lab.keyLightEuler.Length == 3)
            {
                keyLight.transform.rotation = Quaternion.Euler(lab.keyLightEuler[0], lab.keyLightEuler[1], lab.keyLightEuler[2]);
                keyLight.intensity = lab.keyLightIntensity;
            }
            if (mainCam != null) { mainCam.clearFlags = CameraClearFlags.SolidColor; mainCam.backgroundColor = new Color(0.2f, 0.21f, 0.22f); }
        }

        public void SetLabVisible(bool on)
        {
            if (labRoot == null) return;
            labRoot.gameObject.SetActive(on);
            foreach (var go in hiddenByLab) go.SetActive(!on);
        }

        // ------------------------------------------------------------------ modes
        public void SetMode(SonairMode m)
        {
            if (m == mode) return;
            if (mode == SonairMode.Live) udp.Stop();
            if (m == SonairMode.Manual || m == SonairMode.Live)
            {
                for (int k = 0; k < 6; k++) manualQ[k] = rig.q[k] * Mathf.Rad2Deg;
                manualClosure = rig.closure;
                heldObject = HeldAt(t);
            }
            mode = m;
            ghostActive = false;
            if (m == SonairMode.Live) StartLive();
        }

        public void StartLive() { udp.Start(udpPort); }

        public void SelectSequence(int i)
        {
            if (sequences.Length == 0) return;
            seqIndex = Mathf.Clamp(i, 0, sequences.Length - 1);
            t = 0f;
            tracks = new Dictionary<string, List<STrack>>();
            foreach (var e in Current.tracks)
            {
                List<STrack> l;
                if (!tracks.TryGetValue(e.obj, out l)) { l = new List<STrack>(); tracks[e.obj] = l; }
                l.Add(e);
            }
            foreach (var l in tracks.Values) l.Sort((a, b) => a.t.CompareTo(b.t));
            ApplySequence(0f);
        }

        void Update()
        {
            if (rig == null || scene == null) return;
            switch (mode)
            {
                case SonairMode.Sequence:
                    var s = Current;
                    if (s == null) break;
                    if (playing)
                    {
                        t += Time.deltaTime * speed;
                        if (t > s.duration) { if (loop) t = 0f; else { t = s.duration; playing = false; } }
                    }
                    ApplySequence(t);
                    break;
                case SonairMode.Manual:
                    for (int k = 0; k < 6; k++) qa[k] = manualQ[k] * Mathf.Deg2Rad;
                    rig.SetArm(qa); rig.SetGripper(manualClosure); ApplyHeld();
                    scanning = false;
                    break;
                case SonairMode.Live:
                    float[] lq, lt; float lc; bool hasC, hasT;
                    if (udp.TryGet(out lq, out lc, out hasC, out lt, out hasT))
                    {
                        rig.SetArm(lq);
                        for (int k = 0; k < 6; k++) manualQ[k] = lq[k] * Mathf.Rad2Deg;
                        if (hasC) manualClosure = lc;
                        ghostActive = hasT;
                        if (hasT && ghost != null)
                        {
                            ghost.SetArm(lt);
                            float e = 0f;
                            for (int k = 0; k < 6; k++) e = Mathf.Max(e, Mathf.Abs(lt[k] - lq[k]));
                            trackErrDeg = e * Mathf.Rad2Deg;
                        }
                    }
                    rig.SetGripper(manualClosure); ApplyHeld();
                    if (ghost != null)
                    {
                        ghost.SetGripper(manualClosure);
                        trackErrMm = ghostActive ? 1000f * Vector3.Distance(ghost.tool0.position, rig.tool0.position) : 0f;
                    }
                    scanning = false;
                    break;
            }
            if (ghost != null)
            {
                bool on = mode == SonairMode.Live && ghostActive && showGhost;
                if (ghost.gameObject.activeSelf != on) ghost.gameObject.SetActive(on);
            }
            sensors.SetVisible(showPip || sensorFullscreen, showFrustums);
            foreach (var f in frameTransforms) if (f != null && f.gameObject.activeSelf != showFrames) f.gameObject.SetActive(showFrames);
            if (autoCapture && scanning && mode == SonairMode.Sequence && playing)
            {
                captureClock += Time.deltaTime;
                if (captureClock >= 1f / Mathf.Max(0.1f, autoCaptureHz)) { captureClock = 0f; sensors.CaptureAll("auto"); }
            }
        }

        // ------------------------------------------------------------------ sequence playback
        public void ApplySequence(float time)
        {
            var s = Current;
            if (s == null) return;
            int n = s.grip.Length;
            float x = Mathf.Clamp(time * s.fps, 0f, n - 1);
            int i = Mathf.FloorToInt(x), j = Mathf.Min(i + 1, n - 1);
            float f = x - i;
            for (int k = 0; k < 6; k++) qa[k] = Mathf.Lerp(s.q[i * 6 + k], s.q[j * 6 + k], f);
            rig.SetArm(qa);
            rig.SetGripper(Mathf.Lerp(s.grip[i], s.grip[j], f));
            foreach (var name in dynamicObjects)
            {
                Transform tr;
                if (!objByName.TryGetValue(name, out tr)) continue;
                List<STrack> l;
                STrack e = null;
                if (tracks.TryGetValue(name, out l)) foreach (var c in l) { if (c.t <= time + 1e-4f) e = c; else break; }
                if (e == null) { SetWorldPose(tr, defaultPose[name]); continue; }
                var p = new Pose(SonairData.V3(e.pos), SonairData.Q(e.rot));
                if (e.parent == "tool0") SetToolPose(tr, p); else SetWorldPose(tr, p);
            }
            scanning = false; scanningSensor = "";
            if (s.scans != null) foreach (var sc in s.scans) if (time >= sc.t0 && time <= sc.t1) { scanning = true; scanningSensor = sc.sensor; }
        }

        public string PhaseAt(float time)
        {
            var s = Current;
            if (s == null || s.phaseT == null) return "";
            string lab = "";
            for (int i = 0; i < s.phaseT.Length; i++) { if (s.phaseT[i] <= time + 1e-4f) lab = s.phaseLabel[i]; else break; }
            return lab;
        }

        public string HeldAt(float time)
        {
            string held = "";
            foreach (var kv in tracks)
            {
                STrack e = null;
                foreach (var c in kv.Value) { if (c.t <= time + 1e-4f) e = c; else break; }
                if (e != null && e.parent == "tool0") held = kv.Key;
            }
            return held;
        }

        // ------------------------------------------------------------------ manual / live: objects in their default places, one in the hand
        void ApplyHeld()
        {
            foreach (var name in dynamicObjects)
            {
                Transform tr;
                if (!objByName.TryGetValue(name, out tr)) continue;
                Pose g;
                if (name == heldObject && grasp.TryGetValue(name, out g)) SetToolPose(tr, g);
                else SetWorldPose(tr, DefaultPoseFor(name));
            }
        }

        Pose DefaultPoseFor(string name)
        {
            List<STrack> l;      // the specimen on the riser is whichever the current sequence starts with there; others stay parked
            if (tracks.TryGetValue(name, out l) && l.Count > 0 && l[0].parent == "world")
                return new Pose(SonairData.V3(l[0].pos), SonairData.Q(l[0].rot));
            return defaultPose[name];
        }

        void SetWorldPose(Transform tr, Pose p)
        {
            Transform r = builtRoot;
            tr.SetPositionAndRotation(r.TransformPoint(p.position), r.rotation * p.rotation);
        }

        void SetToolPose(Transform tr, Pose p)
        {
            Transform tool = rig.tool0;
            tr.SetPositionAndRotation(tool.TransformPoint(p.position), tool.rotation * p.rotation);
        }

        public void GoToKeyPose(SKeyPose k)
        {
            for (int i = 0; i < 6; i++) manualQ[i] = k.q[i] * Mathf.Rad2Deg;
        }

        // ------------------------------------------------------------------ readouts in the robot base frame B
        public Vector3 ToolPosB()
        {
            return SonairData.ToB(builtRoot.InverseTransformPoint(rig.tool0.position));
        }

        public Vector3 ToolRotVecB()
        {
            var q = Quaternion.Inverse(builtRoot.rotation) * rig.tool0.rotation;
            return SonairData.RotVec(SonairData.RotToB(q));
        }

        public Transform FindObject(string name)
        {
            Transform tr;
            if (objByName.TryGetValue(name, out tr)) return tr;
            for (int i = 0; i < objectNames.Length; i++) if (objectNames[i] == name) return objectTransforms[i];
            return null;
        }

        public string LabelOf(string name)
        {
            foreach (var o in scene.objects) if (o.name == name) return o.label;
            return name;
        }
    }
}
