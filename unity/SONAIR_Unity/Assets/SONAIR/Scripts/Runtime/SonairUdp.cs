// Live mirror: background UDP listener.
// Packet (ASCII, comma or space separated), one per datagram:
//   q1..q6                          actual joints (rad, UR order: base, shoulder, elbow, wrist 1, wrist 2, wrist 3)
//   q1..q6,closure                  + gripper closure (0 = open 85 mm .. 1 = closed)
//   q1..q6,closure,t1..t6           + commanded joints target_q (rad): drawn as the ghost arm (closure may be NaN)
//   #free text                      status line from the sender (shown in the panel)
// Senders: the Jetson recorder (python -m sonair jetson-record ... --unity HOST:PORT) or Tools/sonair_live_bridge.py.
using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace Sonair
{
    public class SonairUdp
    {
        UdpClient client;
        Thread thread;
        volatile bool running;
        readonly object lk = new object();
        readonly float[] q = new float[6];
        readonly float[] qt = new float[6];
        float closure;
        bool hasClosure, hasTarget, fresh;
        long lastTicks, firstTicks;
        string status = "";
        public int Port { get; private set; }
        public int Packets { get; private set; }
        public int BadPackets { get; private set; }
        public string Error { get; private set; }
        public bool Running { get { return running; } }

        public void Start(int port)
        {
            Stop();
            Port = port; Error = ""; Packets = 0; BadPackets = 0; firstTicks = 0; status = "";
            try
            {
                client = new UdpClient(port);
                running = true;
                thread = new Thread(Loop) { IsBackground = true, Name = "SONAIR UDP" };
                thread.Start();
            }
            catch (Exception e) { Error = e.Message; running = false; }
        }

        public void Stop()
        {
            running = false;
            try { if (client != null) client.Close(); } catch (Exception) { }
            client = null;
            if (thread != null && thread.IsAlive) thread.Join(200);
            thread = null;
        }

        void Loop()
        {
            var ep = new IPEndPoint(IPAddress.Any, 0);
            while (running)
            {
                try
                {
                    byte[] data = client.Receive(ref ep);
                    Parse(Encoding.ASCII.GetString(data));
                }
                catch (SocketException) { if (!running) break; }
                catch (ObjectDisposedException) { break; }
                catch (Exception e) { Error = e.Message; }
            }
        }

        void Parse(string s)
        {
            s = s.Trim();
            if (s.StartsWith("#")) { lock (lk) status = s.Substring(1).Trim(); return; }
            var parts = s.Split(new[] { ',', ' ', ';', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 6) { BadPackets++; return; }
            var v = new float[parts.Length];
            for (int i = 0; i < parts.Length; i++)
                if (!float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out v[i])) { BadPackets++; return; }
            lock (lk)
            {
                for (int i = 0; i < 6; i++) q[i] = v[i];
                hasClosure = parts.Length >= 7 && !float.IsNaN(v[6]);      // "NaN" = closure unknown
                if (hasClosure) closure = Math.Max(0f, Math.Min(1f, v[6]));
                hasTarget = parts.Length >= 13;
                if (hasTarget) for (int i = 0; i < 6; i++) qt[i] = v[7 + i];
                fresh = true; lastTicks = DateTime.UtcNow.Ticks; if (firstTicks == 0) firstTicks = lastTicks; Packets++;
            }
        }

        /// <summary>Latest state (copies). Returns false if nothing new arrived since the last call.</summary>
        public bool TryGet(out float[] qOut, out float c, out bool hasC, out float[] qtOut, out bool hasT)
        {
            lock (lk)
            {
                qOut = (float[])q.Clone(); c = closure; hasC = hasClosure; qtOut = (float[])qt.Clone(); hasT = hasTarget;
                bool f = fresh; fresh = false; return f;
            }
        }

        public string Status { get { lock (lk) return status; } }

        public double SecondsSinceLast()
        {
            long t;
            lock (lk) t = lastTicks;
            return t == 0 ? double.PositiveInfinity : (DateTime.UtcNow.Ticks - t) / 1e7;
        }

        /// <summary>Mean packet rate since listening started (Hz).</summary>
        public double Rate()
        {
            long a, b; int n;
            lock (lk) { a = firstTicks; b = lastTicks; n = Packets; }
            return (a == 0 || b <= a) ? 0.0 : (n - 1) / ((b - a) / 1e7);
        }
    }
}
