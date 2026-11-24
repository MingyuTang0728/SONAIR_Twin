// IMGUI control panel + orbit camera. Uses only OnGUI events, so it works with the old Input Manager
// and with the new Input System package alike.
//   Left drag: orbit | Right drag or Shift+drag: pan | Wheel: zoom
//   Space: play/pause | R: restart | F: frames | P: picture-in-picture | L: lab | G: ghost | C: capture | V: view
//   Tab: hide panel | 1..6: pick a sequence
using UnityEngine;

namespace Sonair
{
    [DisallowMultipleComponent]
    public class SonairUI : MonoBehaviour
    {
        enum View { Orbit = 0, FollowTool = 1, D405 = 2, D435i = 3 }

        SonairWorld w;
        View view = View.Orbit;
        Vector3 target; float yaw, pitch, dist;
        bool dragging; int dragButton;
        Rect panelRect, pipRect;
        Vector2 scroll;
        bool panelOpen = true;
        float uiScale = 1f;
        string portText = "5005";
        string lastCapture = "";
        GUIStyle title, label, small, wrap, hot, pill;
        bool stylesReady;

        static readonly string[] JointNames = { "q1 base", "q2 shoulder", "q3 elbow", "q4 wrist 1", "q5 wrist 2", "q6 wrist 3" };
        static readonly string[] HeldKeys = { "", "camtool", "s2", "s4", "paddle" };
        static readonly string[] HeldLabels = { "None", "Cam tool", "S2", "S4", "Paddle" };

        void Start()
        {
            w = GetComponent<SonairWorld>();
            if (w == null || w.builtRoot == null) { enabled = false; return; }
            portText = w.udpPort.ToString();
            ResetView();
        }

        void ResetView()
        {
            var r = w.builtRoot;
            target = r.TransformPoint(SonairData.FromB(new Vector3(0.42f, -0.02f, 0.16f)));
            Vector3 cam = r.TransformPoint(SonairData.FromB(new Vector3(1.55f, -1.30f, 1.00f)));
            Vector3 d = cam - target;
            dist = d.magnitude;
            pitch = Mathf.Asin(d.y / dist) * Mathf.Rad2Deg;
            yaw = Mathf.Atan2(-d.x, -d.z) * Mathf.Rad2Deg;
        }

        void LateUpdate()
        {
            if (w == null || w.mainCam == null) return;
            if (view == View.FollowTool) target = Vector3.Lerp(target, w.rig.tool0.position, 0.15f);
            var rot = Quaternion.Euler(pitch, yaw, 0f);
            w.mainCam.transform.SetPositionAndRotation(target + rot * new Vector3(0f, 0f, -dist), rot);
            w.sensorFullscreen = view == View.D405 || view == View.D435i;
        }

        void Styles()
        {
            if (stylesReady) return;
            title = new GUIStyle(GUI.skin.label) { fontSize = 15, fontStyle = FontStyle.Bold, wordWrap = true };
            label = new GUIStyle(GUI.skin.label) { fontSize = 12 };
            small = new GUIStyle(GUI.skin.label) { fontSize = 11, wordWrap = true };
            small.normal.textColor = new Color(0.82f, 0.84f, 0.88f);
            wrap = new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = true };
            hot = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, wordWrap = true };
            hot.normal.textColor = new Color(1f, 0.55f, 0.2f);
            pill = new GUIStyle(GUI.skin.box) { fontSize = 11, alignment = TextAnchor.MiddleCenter };
            pill.normal.textColor = Color.white;
            stylesReady = true;
        }

        void OnGUI()
        {
            if (w == null || w.scene == null || w.rig == null) return;
            Styles();
            uiScale = Mathf.Clamp(Screen.height / 900f, 0.8f, 2.5f);
            GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(uiScale, uiScale, 1f));
            float sw = Screen.width / uiScale, sh = Screen.height / uiScale;

            if (view == View.D405 || view == View.D435i)
            {
                var s = w.sensors.items[view == View.D405 ? 0 : 1];
                GUI.DrawTexture(new Rect(0, 0, sw, sh), s.rt, ScaleMode.ScaleToFit, false);
                GUI.Label(new Rect(sw * 0.5f - 160, 8, 320, 24), s.def.label + "  (V: next view)", pill);
            }
            else if (w.showFrames) FrameLabels();

            pipRect = new Rect(0, 0, 0, 0);
            if (w.showPip && view != View.D405 && view != View.D435i) Pip(sw);

            panelRect = panelOpen ? new Rect(8, 8, 360, sh - 16) : new Rect(8, 8, 130, 30);
            if (panelOpen)
            {
                GUILayout.BeginArea(panelRect, GUI.skin.box);
                scroll = GUILayout.BeginScrollView(scroll);
                Panel();
                GUILayout.EndScrollView();
                GUILayout.EndArea();
            }
            else if (GUI.Button(panelRect, "Show panel (Tab)")) panelOpen = true;

            GUI.Label(new Rect(panelOpen ? 376 : 146, sh - 26, sw - 390, 22),
                "Drag: orbit | Right drag: pan | Wheel: zoom | Space: play | R: restart | F: frames | P: PiP | L: lab | G: ghost | C: capture | V: view | Tab: panel", small);
            HandleInput();
        }

        // ------------------------------------------------------------------ panel
        void Panel()
        {
            GUILayout.Label("SONAIR cell twin  |  UR5e + 2F-85", title);
            int m = GUILayout.Toolbar((int)w.mode, new[] { "Sequence", "Manual", "Live" });
            if (m != (int)w.mode) w.SetMode((SonairMode)m);
            GUILayout.Space(4);
            if (w.mode == SonairMode.Sequence) SequenceSection();
            else if (w.mode == SonairMode.Manual) ManualSection();
            else LiveSection();

            GUILayout.Space(6);
            GUILayout.Label("View", title);
            int v = GUILayout.Toolbar((int)view, new[] { "Orbit", "Tool", "D405", "D435i" });
            if (v != (int)view) { view = (View)v; if (view == View.Orbit) ResetView(); }
            w.showFrames = GUILayout.Toggle(w.showFrames, " Frames (x red, y green, z blue, robot convention)");
            w.showFrustums = GUILayout.Toggle(w.showFrustums, " Camera frusta");
            w.showPip = GUILayout.Toggle(w.showPip, " Picture-in-picture (D405, D435i)");
            if (w.labRoot != null)
            {
                bool lab = GUILayout.Toggle(w.LabVisible, " Lab environment");
                if (lab != w.LabVisible) w.SetLabVisible(lab);
            }

            GUILayout.Space(6);
            GUILayout.Label("Capture", title);
            if (GUILayout.Button("Save D405 + D435i images with poses (C)"))
                lastCapture = string.Format("Saved {0} images at {1:HH:mm:ss}", w.sensors.CaptureAll("manual"), System.DateTime.Now);
            w.autoCapture = GUILayout.Toggle(w.autoCapture, string.Format(" Auto-save during scan phases ({0:0.#} Hz)", w.autoCaptureHz));
            if (lastCapture.Length > 0) GUILayout.Label(lastCapture, small);
            GUILayout.Label("Folder: " + SonairData.CaptureDir(), small);

            GUILayout.Space(6);
            GUILayout.Label("Readout (robot base frame B)", title);
            var q = w.rig.q;
            GUILayout.Label(string.Format("Joints (deg)  {0:0.0}  {1:0.0}  {2:0.0}  {3:0.0}  {4:0.0}  {5:0.0}",
                q[0] * Mathf.Rad2Deg, q[1] * Mathf.Rad2Deg, q[2] * Mathf.Rad2Deg, q[3] * Mathf.Rad2Deg, q[4] * Mathf.Rad2Deg, q[5] * Mathf.Rad2Deg), wrap);
            var p = w.ToolPosB() * 1000f; var rv = w.ToolRotVecB();
            GUILayout.Label(string.Format("tool0  X {0:0.0}  Y {1:0.0}  Z {2:0.0} mm", p.x, p.y, p.z), wrap);
            GUILayout.Label(string.Format("Rotation vector  RX {0:0.000}  RY {1:0.000}  RZ {2:0.000} rad", rv.x, rv.y, rv.z), wrap);
            string held = w.mode == SonairMode.Sequence ? w.HeldAt(w.t) : w.heldObject;
            GUILayout.Label(string.Format("Gripper opening {0:0.0} mm | In hand: {1}", w.rig.WidthMm(), held.Length > 0 ? w.LabelOf(held) : "nothing"), wrap);
            GUILayout.Label("With the TCP offset set to zero on the pendant, tool0 above matches the pendant readout.", small);
        }

        void SequenceSection()
        {
            var seqs = w.sequences;
            for (int i = 0; i < seqs.Length; i++)
            {
                bool sel = i == w.seqIndex;
                bool now = GUILayout.Toggle(sel, string.Format("{0}. {1}", i + 1, seqs[i].title), GUI.skin.button);
                if (now && !sel) { w.SelectSequence(i); w.playing = true; }
            }
            var s = w.Current;
            if (s == null) return;
            GUILayout.Label(s.desc, small);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(w.playing ? "Pause" : "Play")) w.playing = !w.playing;
            if (GUILayout.Button("Restart")) { w.t = 0f; w.playing = true; }
            w.loop = GUILayout.Toggle(w.loop, " Loop", GUILayout.Width(60));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("Speed x{0:0.00}", w.speed), label, GUILayout.Width(80));
            w.speed = GUILayout.HorizontalSlider(w.speed, 0.1f, 4f);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("{0:0.0} / {1:0.0} s", w.t, s.duration), label, GUILayout.Width(100));
            float nt = GUILayout.HorizontalSlider(w.t, 0f, s.duration);
            if (Mathf.Abs(nt - w.t) > 1e-4f) w.t = nt;
            GUILayout.EndHorizontal();
            GUILayout.Label("Phase: " + w.PhaseAt(w.t), wrap);
            if (w.scanning) GUILayout.Label("Scanning with " + (w.scanningSensor == "A_D405" ? "D405" : w.scanningSensor), hot);
        }

        void ManualSection()
        {
            for (int k = 0; k < 6; k++)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(JointNames[k], label, GUILayout.Width(80));
                float lim = k == 2 ? 180f : 360f;
                w.manualQ[k] = GUILayout.HorizontalSlider(w.manualQ[k], -lim, lim);
                GUILayout.Label(w.manualQ[k].ToString("0.0") + " deg", label, GUILayout.Width(64));
                GUILayout.EndHorizontal();
            }
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("Gripper {0:0.0} mm", w.rig.WidthMm()), label, GUILayout.Width(110));
            w.manualClosure = GUILayout.HorizontalSlider(w.manualClosure, 0f, 1f);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Open 85")) w.manualClosure = 0f;
            if (GUILayout.Button("Handle 27")) w.manualClosure = w.rig.c27;
            if (GUILayout.Button("Closed")) w.manualClosure = 1f;
            GUILayout.EndHorizontal();
            GUILayout.Label("Key poses", label);
            foreach (var k in w.scene.keyPoses) if (GUILayout.Button(k.label)) w.GoToKeyPose(k);
            HeldPicker();
        }

        void LiveSection()
        {
            GUILayout.Label("UDP: q1..q6[,closure[,t1..t6]] in rad; t = commanded target_q, drawn as the ghost", small);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Port", label, GUILayout.Width(40));
            portText = GUILayout.TextField(portText, GUILayout.Width(70));
            if (GUILayout.Button(w.udp.Running ? "Stop listening" : "Start listening"))
            {
                if (w.udp.Running) w.udp.Stop();
                else { int pp; if (int.TryParse(portText, out pp)) w.udpPort = pp; w.StartLive(); }
            }
            GUILayout.EndHorizontal();
            double age = w.udp.SecondsSinceLast();
            GUILayout.Label(string.Format("{0} | {1} packets ({2:0} Hz) | {3} bad | last {4}",
                w.udp.Running ? "Listening on " + w.udp.Port : "Not listening", w.udp.Packets, w.udp.Rate(), w.udp.BadPackets,
                double.IsInfinity(age) ? "never" : age.ToString("0.0") + " s ago"), wrap);
            if (!string.IsNullOrEmpty(w.udp.Error)) GUILayout.Label("Error: " + w.udp.Error, hot);
            if (!string.IsNullOrEmpty(w.udp.Status)) GUILayout.Label("Sender: " + w.udp.Status, wrap);
            if (w.ghostActive)
            {
                w.showGhost = GUILayout.Toggle(w.showGhost, " Show commanded arm (ghost)");
                GUILayout.Label(string.Format("Tracking: max joint error {0:0.000} deg | tool0 offset {1:0.00} mm", w.trackErrDeg, w.trackErrMm), wrap);
            }
            if (!double.IsInfinity(age) && age > 1.0) GUILayout.Label("No packet for more than 1 s: the arm shown is stale.", hot);
            GUILayout.Label("Sender: python -m sonair jetson-record ... --unity HOST:5005, or Tools/sonair_live_bridge.py (--demo B_S2.csv without a robot)", small);
            GUILayout.BeginHorizontal();
            GUILayout.Label(string.Format("Gripper {0:0.0} mm", w.rig.WidthMm()), label, GUILayout.Width(110));
            w.manualClosure = GUILayout.HorizontalSlider(w.manualClosure, 0f, 1f);
            GUILayout.EndHorizontal();
            HeldPicker();
        }

        void HeldPicker()
        {
            GUILayout.Label("In the hand (hung on tool0 at its standard grasp)", label);
            int cur = System.Array.IndexOf(HeldKeys, w.heldObject);
            if (cur < 0) cur = 0;
            int sel = GUILayout.Toolbar(cur, HeldLabels);
            if (sel != cur) w.heldObject = HeldKeys[sel];
        }

        // ------------------------------------------------------------------ overlays
        void Pip(float sw)
        {
            float pw = 300f, y = 8f;
            float x = sw - pw - 8f;
            foreach (var s in w.sensors.items)
            {
                float ph = pw * s.def.height / s.def.width;
                var r = new Rect(x, y + 20, pw, ph);
                GUI.Box(new Rect(x - 2, y, pw + 4, ph + 24), "");
                GUI.Label(new Rect(x + 4, y + 1, pw, 20), s.def.label, label);
                GUI.DrawTexture(r, s.rt, ScaleMode.StretchToFill, false);
                y += ph + 30f;
            }
            pipRect = new Rect(x - 2, 8, pw + 4, y - 8);
        }

        void FrameLabels()
        {
            var cam = w.mainCam;
            for (int i = 0; i < w.frameTransforms.Length && i < w.scene.frames.Length; i++)
            {
                var f = w.frameTransforms[i];
                if (f == null) continue;
                Vector3 sp = cam.WorldToScreenPoint(f.position);
                if (sp.z <= 0) continue;
                var r = new Rect(sp.x / uiScale + 6, (Screen.height - sp.y) / uiScale - 10, 140, 20);
                GUI.Label(r, w.scene.frames[i].label, small);
            }
        }

        // ------------------------------------------------------------------ input (IMGUI events only)
        void HandleInput()
        {
            Event e = Event.current;
            Vector2 mp = e.mousePosition;
            bool overUI = panelRect.Contains(mp) || pipRect.Contains(mp);
            switch (e.type)
            {
                case EventType.MouseDown:
                    if (!overUI) { dragging = true; dragButton = e.button; e.Use(); }
                    break;
                case EventType.MouseUp:
                    dragging = false;
                    break;
                case EventType.MouseDrag:
                    if (!dragging) break;
                    if (dragButton == 0 && !e.shift)
                    {
                        yaw += e.delta.x * 0.3f;
                        pitch = Mathf.Clamp(pitch + e.delta.y * 0.3f, -15f, 89f);
                    }
                    else
                    {
                        var ct = w.mainCam.transform;
                        float k = dist * 0.0016f;
                        target += -ct.right * e.delta.x * k + ct.up * e.delta.y * k;
                        if (view == View.FollowTool) view = View.Orbit;
                    }
                    e.Use();
                    break;
                case EventType.ScrollWheel:
                    if (!overUI) { dist = Mathf.Clamp(dist * (1f + e.delta.y * 0.04f), 0.08f, 12f); e.Use(); }
                    break;
                case EventType.KeyDown:
                    if (GUIUtility.keyboardControl != 0) break;   // typing in a text field
                    bool used = true;
                    switch (e.keyCode)
                    {
                        case KeyCode.Space: w.playing = !w.playing; break;
                        case KeyCode.R: w.t = 0f; w.playing = true; break;
                        case KeyCode.F: w.showFrames = !w.showFrames; break;
                        case KeyCode.P: w.showPip = !w.showPip; break;
                        case KeyCode.L: w.SetLabVisible(!w.LabVisible); break;
                        case KeyCode.G: w.showGhost = !w.showGhost; break;
                        case KeyCode.Tab: panelOpen = !panelOpen; break;
                        case KeyCode.C: lastCapture = string.Format("Saved {0} images", w.sensors.CaptureAll("manual")); break;
                        case KeyCode.V: view = (View)(((int)view + 1) % 4); if (view == View.Orbit) ResetView(); break;
                        default:
                            int n = (int)e.keyCode - (int)KeyCode.Alpha1;
                            if (n >= 0 && n < w.sequences.Length) { w.SetMode(SonairMode.Sequence); w.SelectSequence(n); w.playing = true; }
                            else used = false;
                            break;
                    }
                    if (used) e.Use();
                    break;
            }
        }
    }
}
