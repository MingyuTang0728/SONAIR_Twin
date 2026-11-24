// Editor helpers: menu "SONAIR"
//   1 Create and Open Demo Scene  -> Assets/SONAIR/SONAIR_Demo.unity with one "SONAIR" object (SonairWorld). Press Play.
//                                    Also switches the project to linear colour space and a 15 m shadow distance,
//                                    which the lab lighting is tuned for.
//   2 Bake Cell Into Scene        -> builds the cell in Edit mode and stores meshes and materials as assets
//                                    (the lab is always built at Play)
//   3 Clear Bake                  -> removes the baked objects (the scene then builds at Play again)
//   4 Open Capture Folder
// On first import the demo scene is created and opened automatically (once per project).
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sonair.EditorTools
{
    [InitializeOnLoad]
    public static class SonairEditor
    {
        const string ScenePath = "Assets/SONAIR/SONAIR_Demo.unity";
        const string BakeDir = "Assets/SONAIR/Baked";

        static SonairEditor()
        {
            EditorApplication.delayCall += AutoSetup;
        }

        static string PrefKey { get { return "SONAIR_autosetup_" + Application.dataPath.GetHashCode(); } }

        static void AutoSetup()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode || EditorPrefs.GetBool(PrefKey, false)) return;
            EditorPrefs.SetBool(PrefKey, true);
            if (File.Exists(ScenePath)) return;
            if (SceneManager.GetActiveScene().isDirty)
            {
                Debug.Log("[SONAIR] The open scene has unsaved changes, so the demo scene was not created. Use SONAIR > 1 Create and Open Demo Scene.");
                return;
            }
            CreateScene();
        }

        [MenuItem("SONAIR/1 Create and Open Demo Scene", false, 1)]
        public static void CreateSceneMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            CreateScene();
        }

        static void CreateScene()
        {
            if (PlayerSettings.colorSpace != ColorSpace.Linear)
            {
                PlayerSettings.colorSpace = ColorSpace.Linear;
                Debug.Log("[SONAIR] Colour space set to Linear (Project Settings > Player) for physically based lighting.");
            }
            QualitySettings.shadowDistance = 15f;
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var go = new GameObject("SONAIR");
            go.AddComponent<SonairWorld>();
            go.AddComponent<SonairUI>();
            var cam = Camera.main;
            if (cam != null)
            {
                cam.transform.position = SonairData.FromB(new Vector3(1.55f, -1.30f, 1.00f));
                cam.transform.LookAt(SonairData.FromB(new Vector3(0.42f, -0.02f, 0.16f)));
                cam.nearClipPlane = 0.01f;
            }
            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.Refresh();
            Selection.activeGameObject = go;
            Debug.Log("[SONAIR] Created " + ScenePath + ". Press Play.");
        }

        static SonairWorld FindWorld(bool create)
        {
            var go = GameObject.Find("SONAIR");
            var w = go != null ? go.GetComponent<SonairWorld>() : null;
            if (w == null && create)
            {
                go = new GameObject("SONAIR");
                w = go.AddComponent<SonairWorld>();
                go.AddComponent<SonairUI>();
            }
            return w;
        }

        [MenuItem("SONAIR/2 Bake Cell Into Scene", false, 2)]
        public static void Bake()
        {
            if (Application.isPlaying) { Debug.LogWarning("[SONAIR] Leave Play mode first."); return; }
            var w = FindWorld(true);
            var sc = SonairData.LoadScene();
            if (sc == null) return;
            ClearBakeAssets();
            SonairBuilder.Build(w, sc);
            Directory.CreateDirectory(BakeDir);
            int nm = 0, nt = 0, nx = 0;
            var renderers = w.builtRoot.GetComponentsInChildren<MeshRenderer>(true);
            foreach (var r in renderers)   // textures first, then materials, then meshes
            {
                var m = r.sharedMaterial;
                if (m == null || AssetDatabase.Contains(m)) continue;
                var tex = m.mainTexture as Texture2D;
                if (tex != null && !AssetDatabase.Contains(tex)) { AssetDatabase.CreateAsset(tex, Unique(tex.name, "asset")); nx++; }
            }
            foreach (var r in renderers)
            {
                var m = r.sharedMaterial;
                if (m != null && !AssetDatabase.Contains(m)) { AssetDatabase.CreateAsset(m, Unique(m.name, "mat")); nt++; }
            }
            foreach (var f in w.builtRoot.GetComponentsInChildren<MeshFilter>(true))
            {
                var mesh = f.sharedMesh;
                if (mesh != null && !AssetDatabase.Contains(mesh)) { AssetDatabase.CreateAsset(mesh, Unique(mesh.name, "asset")); nm++; }
            }
            AssetDatabase.SaveAssets();
            EditorUtility.SetDirty(w);
            EditorSceneManager.MarkSceneDirty(w.gameObject.scene);
            EditorSceneManager.SaveScene(w.gameObject.scene);
            Debug.Log(string.Format("[SONAIR] Baked {0} meshes, {1} materials, {2} textures into {3}", nm, nt, nx, BakeDir));
        }

        [MenuItem("SONAIR/3 Clear Bake", false, 3)]
        public static void ClearBake()
        {
            if (Application.isPlaying) return;
            var w = FindWorld(false);
            if (w != null && w.builtRoot != null)
            {
                Object.DestroyImmediate(w.builtRoot.gameObject);
                w.builtRoot = null; w.rig = null;
                EditorUtility.SetDirty(w);
                EditorSceneManager.MarkSceneDirty(w.gameObject.scene);
            }
            ClearBakeAssets();
        }

        [MenuItem("SONAIR/4 Open Capture Folder", false, 20)]
        public static void OpenCaptures()
        {
            Directory.CreateDirectory(SonairData.CaptureDir());
            EditorUtility.RevealInFinder(SonairData.CaptureDir());
        }

        static void ClearBakeAssets()
        {
            if (AssetDatabase.IsValidFolder(BakeDir)) AssetDatabase.DeleteAsset(BakeDir);
        }

        static string Unique(string name, string ext)
        {
            foreach (char c in Path.GetInvalidFileNameChars()) name = name.Replace(c, '_');
            return AssetDatabase.GenerateUniqueAssetPath(BakeDir + "/" + name + "." + ext);
        }
    }
}
