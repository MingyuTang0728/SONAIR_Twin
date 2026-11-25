"""python -m console [--mock] [--robot IP] [--bind 127.0.0.1] [--port 8765]"""
import argparse
import asyncio
import logging

from .agent import main_async


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m console", description="SONAIR cell console (browser UI on the same port)")
    ap.add_argument("--mock", action="store_true", help="no robot: stand-in controller on 127.0.0.1 and a simulated IMU")
    ap.add_argument("--robot", help="controller IP to connect at start (URSim first)")
    ap.add_argument("--robot-hz", type=float, default=500.0, help="robot link rate; sessions need 500 (default)")
    ap.add_argument("--bind", default="127.0.0.1", help="address to serve on; anything but loopback requires a token")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--token", help="access token for the WebSocket (generated when --bind is not loopback)")
    ap.add_argument("--runs", help="session folder (default runs_jetson/)")
    ap.add_argument("--unity", help="host:port of the Unity live mirror (display only), e.g. 127.0.0.1:9870")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
                        datefmt="%H:%M:%S")
    try:
        asyncio.run(main_async(a))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
