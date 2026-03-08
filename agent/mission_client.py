"""
MissionClient — fetches mission JSON from the ARST server at startup.

Endpoint: GET http://localhost:7400/mission
Expected response:
  {
    "targets": ["fire extinguisher", "hazard sign", ...],
    "time_limit": 300
  }
"""

import time
import urllib.request
import urllib.error
import json
import logging

logger = logging.getLogger(__name__)

MISSION_URL = "http://localhost:7400/mission"
RETRY_DELAY = 2.0   # seconds between retries
MAX_RETRIES = 30    # 60 s total before giving up


class MissionData:
    def __init__(self, targets: list[str], time_limit: int):
        self.targets = targets
        self.time_limit = time_limit

    def __repr__(self):
        return f"MissionData(targets={self.targets}, time_limit={self.time_limit}s)"


def fetch_mission(url: str = MISSION_URL) -> MissionData:
    """
    Block until the mission endpoint is reachable, then return parsed mission data.
    Retries every RETRY_DELAY seconds up to MAX_RETRIES times.
    """
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                raw = resp.read().decode("utf-8")
            data = json.loads(raw)
            raw_targets = data.get("targets", [])
            # Targets may be plain strings or dicts like {"type": "fire_extinguisher", ...}
            targets = [
                t["type"] if isinstance(t, dict) else str(t)
                for t in raw_targets
            ]
            time_limit = int(data.get("time_limit", 300))
            mission = MissionData(targets=targets, time_limit=time_limit)
            logger.info("Mission fetched: %s", mission)
            return mission
        except (urllib.error.URLError, ConnectionRefusedError, OSError) as exc:
            logger.warning(
                "Mission server not ready (attempt %d/%d): %s — retrying in %.1fs",
                attempt, MAX_RETRIES, exc, RETRY_DELAY,
            )
            time.sleep(RETRY_DELAY)
        except (json.JSONDecodeError, KeyError, ValueError) as exc:
            logger.error("Malformed mission response: %s", exc)
            raise

    raise RuntimeError(
        f"Mission endpoint unreachable after {MAX_RETRIES} attempts ({url})"
    )
