#!/usr/bin/env python3

# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Symovo contract probe (manual tool).

Goal (todo: collect_runtime_contract):
- Print what Symovo currently returns for map list and AMR pose, including
  map ids / last_changed / offsetX/Y / resolution.
- This is the minimal evidence needed to select correct map update strategy
  and pose-to-map conversion policy.

Usage (on robot):
  ros2 run aehub_symovo_map_mirror symovo_map_contract_probe -- \
    --endpoint https://192.168.1.100 --amr-id 15 --tls-verify false
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

from aehub_symovo_map_mirror.symovo_api_client import SymovoApiClient


def _pp(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, indent=2, sort_keys=True)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--endpoint", required=True)
    p.add_argument("--amr-id", type=int, required=True)
    p.add_argument("--tls-verify", type=str, default="true")
    p.add_argument("--try-wait-for-changes", action="store_true")
    p.add_argument("--wait-timeout-sec", type=float, default=5.0)
    args = p.parse_args()

    tls_verify = str(args.tls_verify).lower() in ("1", "true", "yes", "on")
    client = SymovoApiClient(args.endpoint, tls_verify=tls_verify, timeout_sec=10.0)

    try:
        pose = client.get_amr_pose(amr_id=args.amr_id)
        print("AMR pose (/v0/amr/{id}/pose):")
        print(_pp(pose.__dict__))
    except Exception as e:
        print(f"ERROR: failed to get AMR pose: {e}", file=sys.stderr)

    try:
        maps = client.list_maps()
        print(f"\nMaps (/v0/map): count={len(maps)}")
        for m in maps[:20]:
            print(_pp(m.__dict__))
    except Exception as e:
        print(f"ERROR: failed to list maps: {e}", file=sys.stderr)
        return 2

    if args.try_wait_for_changes and maps:
        try:
            m0 = maps[0]
            print(f"\nTrying wait_for_changes on map_id={m0.map_id} last_changed={m0.last_changed} ...")
            changed, new_last_changed = client.wait_for_map_changes(
                map_id=m0.map_id,
                last_changed=m0.last_changed,
                timeout_sec=args.wait_timeout_sec,
            )
            print(_pp({"changed": changed, "new_last_changed": new_last_changed}))
        except Exception as e:
            print(f"ERROR: wait_for_changes failed: {e}", file=sys.stderr)
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

