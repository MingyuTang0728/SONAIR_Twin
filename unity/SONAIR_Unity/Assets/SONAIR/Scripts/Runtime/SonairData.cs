// SONAIR two-mode inspection kit: data classes (JsonUtility) and loaders.
// All poses are already in Unity coordinates (metres, left-handed, y up).
// Robot base frame B (UR controller base) <-> Unity:  x_B = z_u,  y_B = -x_u,  z_B = y_u.
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

namespace Sonair
{
    [Serializable] public class SVisual { public string mesh; public string mat; }
    [Serializable] public class SBody { public string name; public int parent; public float[] pos; public float[] rot; public int joint; public float[] axis; public float[] pivot; public SVisual[] visuals; }
    [Serializable] public class SJoint { public string name; public int body; public float lo; public float hi; public string group; }
    [Serializable] public class SGripper { public int nq; public float[] closure; public float[] q; public float[] width; public float c27; }
    [Serializable] public class SMaterial { public string name; public float[] color; public float metallic; public float smoothness; public string texture; public float[] emission; }
    [Serializable] public class SObject { public string name; public string label; public bool dynamic; public string parent; public float[] pos; public float[] rot; public SVisual[] visuals; }
    [Serializable] public class SSensor { public string name; public string label; public string parent; public float[] pos; public float[] rot; public float hfov; public float vfov; public float near; public float far; public int width; public int height; public float frustum; }
    [Serializable] public class SFrame { public string name; public string label; public string parent; public float[] pos; public float[] rot; public float size; }
    [Serializable] public class SGrasp { public string obj; public float[] pos; public float[] rot; }
    [Serializable] public class SKeyPose { public string name; public string label; public float[] q; }
    [Serializable]
    public class SScene
    {
        public string version; public string note; public SBody[] bodies; public SJoint[] joints; public int tool0; public SGripper gripper;
        public SMaterial[] materials; public SObject[] objects; public SSensor[] sensors; public SFrame[] frames; public SGrasp[] grasps;
        public SKeyPose[] keyPoses; public float[] home;
    }
    [Serializable] public class SScan { public float t0; public float t1; public string sensor; }
    [Serializable] public class STrack { public string obj; public float t; public string parent; public float[] pos; public float[] rot; }
    [Serializable]
    public class SSequence
    {
        public string key; public string title; public string mode; public string desc; public float duration; public int fps;
        public float[] q; public float[] grip; public float[] phaseT; public string[] phaseLabel; public SScan[] scans; public STrack[] tracks;
    }
    [Serializable] public class SSequenceFile { public SSequence[] sequences; }

    public static class SonairData
    {
        public const string ResRoot = "SONAIR/";

        public static Vector3 V3(float[] a) { return new Vector3(a[0], a[1], a[2]); }
        public static Quaternion Q(float[] a) { return new Quaternion(a[0], a[1], a[2], a[3]); }

        public static SScene LoadScene()
        {
            var ta = Resources.Load<TextAsset>(ResRoot + "scene");
            if (ta == null) { Debug.LogError("[SONAIR] Resources/SONAIR/scene.json missing"); return null; }
            return JsonUtility.FromJson<SScene>(ta.text);
        }

        public static SSequence[] LoadSequences()
        {
            var ta = Resources.Load<TextAsset>(ResRoot + "sequences");
            if (ta == null) { Debug.LogError("[SONAIR] Resources/SONAIR/sequences.json missing"); return new SSequence[0]; }
            return JsonUtility.FromJson<SSequenceFile>(ta.text).sequences;
        }

        // ---------------------------------------------------------------- meshes (binary "SNM1", already Unity-handed)
        static readonly Dictionary<string, Mesh> meshCache = new Dictionary<string, Mesh>();
        public static Mesh LoadMesh(string name)
        {
            Mesh cached;
            if (meshCache.TryGetValue(name, out cached) && cached != null) return cached;
            var ta = Resources.Load<TextAsset>(ResRoot + "Meshes/" + name);
            if (ta == null) { Debug.LogError("[SONAIR] mesh not found: " + name); return null; }
            byte[] b = ta.bytes;
            if (b.Length < 16 || b[0] != 'S' || b[1] != 'N' || b[2] != 'M' || b[3] != '1') { Debug.LogError("[SONAIR] bad mesh file: " + name); return null; }
            int nv = BitConverter.ToInt32(b, 4), ni = BitConverter.ToInt32(b, 8), flags = BitConverter.ToInt32(b, 12);
            int o = 16;
            var fp = new float[nv * 3]; Buffer.BlockCopy(b, o, fp, 0, nv * 12); o += nv * 12;
            var fn = new float[nv * 3]; Buffer.BlockCopy(b, o, fn, 0, nv * 12); o += nv * 12;
            float[] fuv = null;
            if ((flags & 2) != 0) { fuv = new float[nv * 2]; Buffer.BlockCopy(b, o, fuv, 0, nv * 8); o += nv * 8; }
            var idx = new int[ni]; Buffer.BlockCopy(b, o, idx, 0, ni * 4);
            var v = new Vector3[nv]; var n = new Vector3[nv];
            for (int i = 0; i < nv; i++)
            {
                v[i] = new Vector3(fp[3 * i], fp[3 * i + 1], fp[3 * i + 2]);
                n[i] = new Vector3(fn[3 * i], fn[3 * i + 1], fn[3 * i + 2]);
            }
            var m = new Mesh { name = name };
            if (nv > 65000) m.indexFormat = IndexFormat.UInt32;
            m.vertices = v; m.normals = n;
            if (fuv != null)
            {
                var uv = new Vector2[nv];
                for (int i = 0; i < nv; i++) uv[i] = new Vector2(fuv[2 * i], fuv[2 * i + 1]);
                m.uv = uv;
            }
            m.triangles = idx;
            m.RecalculateBounds();
            meshCache[name] = m;
            return m;
        }

        // ---------------------------------------------------------------- textures (PNG bytes)
        static readonly Dictionary<string, Texture2D> texCache = new Dictionary<string, Texture2D>();
        public static Texture2D LoadTexture(string key)
        {
            Texture2D t;
            if (texCache.TryGetValue(key, out t) && t != null) return t;
            var ta = Resources.Load<TextAsset>(ResRoot + "Textures/" + key);
            if (ta == null) { Debug.LogError("[SONAIR] texture not found: " + key); return null; }
            t = new Texture2D(2, 2, TextureFormat.RGBA32, false) { name = key };
            t.LoadImage(ta.bytes);
            t.filterMode = FilterMode.Point; t.wrapMode = TextureWrapMode.Clamp; t.anisoLevel = 1;
            texCache[key] = t;
            return t;
        }

        // ---------------------------------------------------------------- materials (Built-in, URP or HDRP)
        public static Shader LitShader()
        {
            Shader s = null;
            var rp = GraphicsSettings.currentRenderPipeline;
            if (rp != null)
            {
                string n = rp.GetType().Name;
                if (n.Contains("Universal")) s = Shader.Find("Universal Render Pipeline/Lit");
                else if (n.Contains("HD")) s = Shader.Find("HDRP/Lit");
                if (s == null) s = rp.defaultShader;
            }
            if (s == null) s = Shader.Find("Standard");
            if (s == null) s = Shader.Find("Diffuse");
            return s;
        }

        static readonly Dictionary<string, Material> matCache = new Dictionary<string, Material>();
        public static void ClearCaches() { meshCache.Clear(); texCache.Clear(); matCache.Clear(); }

        public static Material MakeMaterial(SMaterial s)
        {
            Material m;
            if (matCache.TryGetValue(s.name, out m) && m != null) return m;
            m = new Material(LitShader()) { name = "SONAIR_" + s.name };
            var c = new Color(s.color[0], s.color[1], s.color[2], s.color[3]);
            m.color = c;
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", s.metallic);
            if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", s.smoothness);
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", s.smoothness);
            if (!string.IsNullOrEmpty(s.texture))
            {
                var tex = LoadTexture(s.texture);
                if (tex != null)
                {
                    m.mainTexture = tex;
                    if (m.HasProperty("_BaseMap")) m.SetTexture("_BaseMap", tex);
                    if (m.HasProperty("_BaseColorMap")) m.SetTexture("_BaseColorMap", tex);
                }
            }
            if (c.a < 0.999f) MakeTransparent(m);
            matCache[s.name] = m;
            return m;
        }

        public static Material MakeColor(string name, Color c, float metallic = 0f, float smooth = 0.4f)
        {
            return MakeMaterial(new SMaterial { name = name, color = new[] { c.r, c.g, c.b, c.a }, metallic = metallic, smoothness = smooth, texture = "" });
        }

        public static void MakeTransparent(Material m)
        {
            // Built-in Standard (Fade)
            if (m.HasProperty("_Mode")) m.SetFloat("_Mode", 2f);
            m.SetOverrideTag("RenderType", "Transparent");
            if (m.HasProperty("_SrcBlend")) m.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
            if (m.HasProperty("_DstBlend")) m.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
            if (m.HasProperty("_ZWrite")) m.SetFloat("_ZWrite", 0f);
            m.DisableKeyword("_ALPHATEST_ON"); m.EnableKeyword("_ALPHABLEND_ON"); m.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            // URP / HDRP
            if (m.HasProperty("_Surface")) m.SetFloat("_Surface", 1f);
            if (m.HasProperty("_SurfaceType")) m.SetFloat("_SurfaceType", 1f);
            if (m.HasProperty("_Blend")) m.SetFloat("_Blend", 0f);
            if (m.HasProperty("_Cull")) m.SetFloat("_Cull", (float)CullMode.Off);
            m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            m.renderQueue = (int)RenderQueue.Transparent;
        }

        // ---------------------------------------------------------------- robot frame B <-> Unity
        public static Vector3 ToB(Vector3 u) { return new Vector3(u.z, -u.x, u.y); }
        public static Vector3 FromB(Vector3 b) { return new Vector3(-b.y, b.z, b.x); }
        /// <summary>Rotation matrix (row-major 3x3) in frame B of a Unity rotation whose local axes follow the robot convention.</summary>
        public static float[] RotToB(Quaternion q)
        {
            // R_B = M^T R_u M with M e_x = +z_u, M e_y = -x_u, M e_z = +y_u
            Vector3 cx = ToB(q * new Vector3(0, 0, 1));     // image of B-local x axis
            Vector3 cy = ToB(q * new Vector3(-1, 0, 0));    // image of B-local y axis
            Vector3 cz = ToB(q * new Vector3(0, 1, 0));     // image of B-local z axis
            return new[] { cx.x, cy.x, cz.x, cx.y, cy.y, cz.y, cx.z, cy.z, cz.z };
        }
        /// <summary>UR-style rotation vector (rx, ry, rz) in radians from a row-major 3x3 matrix.</summary>
        public static Vector3 RotVec(float[] R)
        {
            double tr = R[0] + R[4] + R[8];
            double c = Math.Max(-1.0, Math.Min(1.0, (tr - 1.0) / 2.0));
            double ang = Math.Acos(c);
            if (ang < 1e-6) return Vector3.zero;
            if (Math.PI - ang < 1e-4)
            {
                // 180 deg: axis from the largest diagonal term
                double xx = Math.Sqrt(Math.Max(0, (R[0] + 1) / 2)), yy = Math.Sqrt(Math.Max(0, (R[4] + 1) / 2)), zz = Math.Sqrt(Math.Max(0, (R[8] + 1) / 2));
                if (R[1] < 0) yy = -yy;
                if (R[2] < 0) zz = -zz;
                return new Vector3((float)(xx * ang), (float)(yy * ang), (float)(zz * ang));
            }
            double s = 2 * Math.Sin(ang);
            return new Vector3((float)((R[7] - R[5]) / s * ang), (float)((R[2] - R[6]) / s * ang), (float)((R[3] - R[1]) / s * ang));
        }

        public static string CaptureDir()
        {
            return Path.Combine(Application.persistentDataPath, "SONAIR_captures");
        }
    }
}
