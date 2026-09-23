"""Cache behaviour of the validation endpoints, with Firestore stubbed out.

No network and no Firestore reads -- the stub counts every document a request
would have billed, which is what the cache exists to keep at zero.

    .venv/bin/python checks/cache_check.py
"""
import os, sys, shutil, tempfile

CACHE = tempfile.mkdtemp(prefix="valcache-")
os.environ["MEP_DASHBOARD_CACHE_DIR"] = CACHE
os.environ["MEP_DASHBOARD_RUNS_TTL_S"] = "60"
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))

from fastapi import HTTPException
from backend import app as A

reads = {"n": 0}

class Doc:
    def __init__(self, id, data): self.id, self._d = id, data
    def to_dict(self): return dict(self._d)

class Query:
    def __init__(self, docs): self._docs = docs
    def order_by(self, *_): return self
    def stream(self):
        for d in self._docs:
            reads["n"] += 1
            yield d

class DocRef:
    def __init__(self, db, sid): self.db, self.sid = db, sid
    def collection(self, _): return Query(self.db.telemetry.get(self.sid, []))
    def get(self):
        reads["n"] += 1
        return Doc(self.sid, self.db.sessions.get(self.sid, {}))

class Coll:
    def __init__(self, db): self.db = db
    def stream(self):
        for sid, data in self.db.sessions.items():
            reads["n"] += 1
            yield Doc(sid, data)
    def document(self, sid): return DocRef(self.db, sid)

class DB:
    def __init__(self):
        self.sessions = {
            "run_done":    {"session_id": "run_done", "is_running": False, "duration_sec": 10.0},
            "run_live":    {"session_id": "run_live", "is_running": True},
            "run_legacy":  {"session_id": "run_legacy", "duration_sec": 5.0},
        }
        self.telemetry = {
            "run_done":   [Doc(f"{i:06d}", {"sim_time": i * .2, "distance_m": 1.0}) for i in range(500)],
            "run_live":   [Doc(f"{i:06d}", {"sim_time": i * .2}) for i in range(20)],
            "run_legacy": [Doc(f"{i:06d}", {"sim_time": i * .2}) for i in range(30)],
        }
    def collection(self, _): return Coll(self)

db = DB()
A.firestore_client = lambda: db

def fail():
    raise HTTPException(status_code=503, detail="Firebase Connection Error: ResourceExhausted: 429 Quota exceeded.")

ok = True
def check(label, cond, extra=""):
    global ok
    ok = ok and cond
    print(("PASS " if cond else "FAIL ") + label + ((" -> " + extra) if extra else ""))

# 1. run list: first call hits Firestore, second is served from cache
reads["n"] = 0
r1 = A.validation_runs()
check("runs first call is live", r1["cache"] == "live" and len(r1["runs"]) == 3, f'{r1["cache"]}, {len(r1["runs"])} runs, {reads["n"]} reads')
n_first = reads["n"]
reads["n"] = 0
r2 = A.validation_runs()
check("runs second call costs 0 reads", r2["cache"] == "fresh" and reads["n"] == 0, f'{r2["cache"]}, {reads["n"]} reads (was {n_first})')

# 2. telemetry of a finished run: cached, then 0 reads
reads["n"] = 0
t1 = A.validation_telemetry("run_done")
check("finished run first call is live", t1["cache"] == "live" and len(t1["telemetry"]) == 500, f'{t1["cache"]}, {reads["n"]} reads')
n_tel = reads["n"]
reads["n"] = 0
t2 = A.validation_telemetry("run_done")
check("finished run second call costs 0 reads", t2["cache"] == "complete" and reads["n"] == 0 and len(t2["telemetry"]) == 500,
      f'{t2["cache"]}, {reads["n"]} reads (was {n_tel})')

# 3. a run still recording must be re-read every time
reads["n"] = 0
l1 = A.validation_telemetry("run_live")
l2 = A.validation_telemetry("run_live")
check("running run is never served as final", l1["cache"] == "live-running" and l2["cache"] == "live-running" and reads["n"] > 0,
      f'{l1["cache"]}/{l2["cache"]}, {reads["n"]} reads')

# 4. pre-`is_running` run counts as finished
A.validation_telemetry("run_legacy")
reads["n"] = 0
g2 = A.validation_telemetry("run_legacy")
check("legacy run without is_running is cached", g2["cache"] == "complete" and reads["n"] == 0, f'{g2["cache"]}, {reads["n"]} reads')

# 5. quota exhausted -> stale cache instead of 503
A.firestore_client = lambda: fail()
os.utime(os.path.join(CACHE, "runs.json"), (0, 0))   # force the runs TTL to expire
import json as _j
p = os.path.join(CACHE, "runs.json")
d = _j.load(open(p)); d["cached_at"] = 0.0; _j.dump(d, open(p, "w"))
s1 = A.validation_runs()
check("runs fall back to stale on quota error", s1["cache"] == "stale" and len(s1["runs"]) == 3, f'{s1["cache"]}, {len(s1["runs"])} runs')
s2 = A.validation_telemetry("run_live")   # running -> no complete cache, but a stale copy exists
check("running run falls back to stale", s2["cache"] == "stale" and len(s2["telemetry"]) == 20, f'{s2["cache"]}, {len(s2["telemetry"])} rows')
s3 = A.validation_telemetry("run_done")   # complete cache -> never even calls Firestore
check("finished run unaffected by quota error", s3["cache"] == "complete" and len(s3["telemetry"]) == 500, s3["cache"])

# 6. no cache at all -> still a real 503 with the cause
shutil.rmtree(CACHE); os.makedirs(CACHE)
try:
    A.validation_telemetry("run_done"); check("uncached error still raises 503", False)
except HTTPException as e:
    check("uncached error still raises 503", e.status_code == 503 and "Quota exceeded" in e.detail, e.detail)

shutil.rmtree(CACHE, ignore_errors=True)
print("\nRESULT:", "ALL PASS" if ok else "FAILURES")
sys.exit(0 if ok else 1)
