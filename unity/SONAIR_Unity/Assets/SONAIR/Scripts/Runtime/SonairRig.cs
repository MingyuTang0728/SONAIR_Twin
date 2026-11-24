// Kinematic UR5e + Robotiq 2F-85 rig.
// Each hinge: localRotation = rest * AngleAxis(q, axis);  localPosition = restPos + rest * (pivot - AngleAxis(q, axis) * pivot)
// (axis and pivot are in the body's own frame, already converted from MuJoCo Menagerie; verified against MuJoCo FK to < 0.01 mm).
// The 2F-85 four-bar linkage is driven from a lookup table computed in MuJoCo: closure 0 (85 mm open) .. 1 (closed).
using System;
using UnityEngine;

namespace Sonair
{
    [Serializable]
    public class SonairJoint
    {
        public string name;
        public Transform t;
        public Quaternion rest;
        public Vector3 restPos;
        public Vector3 axis;
        public Vector3 pivot;
        public float lo, hi;
        public bool arm;
    }

    public class SonairRig : MonoBehaviour
    {
        public SonairJoint[] joints = new SonairJoint[0];   // 0..5 = UR5e, 6.. = gripper
        public Transform tool0;
        public float[] lutClosure = new float[0];
        public float[] lutQ = new float[0];
        public float[] lutWidth = new float[0];
        public int lutNq = 8;
        public float c27 = 0.68f;

        [NonSerialized] public float[] q = new float[6];
        [NonSerialized] public float closure;

        public int ArmCount { get { return 6; } }

        public void SetupLut(SGripper g)
        {
            lutClosure = g.closure; lutQ = g.q; lutWidth = g.width; lutNq = g.nq; c27 = g.c27;
        }

        public void SetArm(float[] qa)
        {
            for (int i = 0; i < 6 && i < qa.Length; i++) { q[i] = qa[i]; ApplyJoint(i, qa[i]); }
        }

        public void SetGripper(float c)
        {
            closure = Mathf.Clamp01(c);
            int n = lutClosure.Length;
            if (n < 2) return;
            float x = closure * (n - 1);
            int i0 = Mathf.Clamp(Mathf.FloorToInt(x), 0, n - 1), i1 = Mathf.Min(i0 + 1, n - 1);
            float f = x - i0;
            for (int k = 0; k < lutNq && 6 + k < joints.Length; k++)
            {
                float a = lutQ[i0 * lutNq + k], b = lutQ[i1 * lutNq + k];
                ApplyJoint(6 + k, a + (b - a) * f);
            }
        }

        /// <summary>Opening between the silicone pads (mm) for the current closure.</summary>
        public float WidthMm()
        {
            int n = lutWidth.Length;
            if (n < 2) return 0;
            float x = closure * (n - 1);
            int i0 = Mathf.Clamp(Mathf.FloorToInt(x), 0, n - 1), i1 = Mathf.Min(i0 + 1, n - 1);
            return Mathf.Lerp(lutWidth[i0], lutWidth[i1], x - i0);
        }

        /// <summary>Closure that gives a pad opening of width mm (free closing, no object).</summary>
        public float ClosureForWidth(float mm)
        {
            int n = lutWidth.Length;
            for (int i = 0; i + 1 < n; i++)
            {
                float a = lutWidth[i], b = lutWidth[i + 1];
                if ((mm <= a && mm >= b) || (mm >= a && mm <= b))
                {
                    float f = Mathf.Approximately(a, b) ? 0f : (mm - a) / (b - a);
                    return Mathf.Lerp(lutClosure[i], lutClosure[i + 1], f);
                }
            }
            return mm > 40 ? 0f : 1f;
        }

        void ApplyJoint(int i, float rad)
        {
            if (i >= joints.Length) return;
            var j = joints[i];
            if (j == null || j.t == null) return;
            Quaternion r = Quaternion.AngleAxis(rad * Mathf.Rad2Deg, j.axis);
            j.t.localRotation = j.rest * r;
            j.t.localPosition = j.restPos + j.rest * (j.pivot - r * j.pivot);
        }
    }
}
