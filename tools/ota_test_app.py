"""linka-ota-server plus a fake measurements API, for tools/ota-test.sh.

Serves everything linka-ota-server does, and accepts the sensor's measurement
reports on /api (and any path below it, e.g. /api/v1/measurements) so test
reports never reach the real backend. Each report is checked to be valid JSON
in the format the firmware sends, and printed with the API key redacted.

Responds 200 to valid reports, and 400 to invalid JSON or reports with missing
or non-numeric fields, like a real backend would reject them.
"""
import json
import sys
import time

from flask import request

from linka_ota_server import create_app

app = create_app()

EXPECTED_FIELDS = ("sensor", "source", "version", "pm1dot0", "pm2dot5", "pm10",
                   "latitude", "longitude", "recorded")


def log(message):
    print(f"[api {time.strftime('%H:%M:%S')}] {message}", file=sys.stderr, flush=True)


@app.post("/api")
@app.post("/api/<path:subpath>")
def measurements(subpath=None):
    body = request.get_data(as_text=True)
    api_key = "set" if request.headers.get("x-api-key") else "MISSING"
    source = request.remote_addr

    try:
        reports = json.loads(body)
    except ValueError as error:
        log(f"INVALID JSON from {source} ({error}), api key {api_key}:\n    {body}")
        return {"error": f"invalid JSON: {error}"}, 400

    if not isinstance(reports, list):
        reports = [reports]
    valid = True
    for report in reports:
        missing = [f for f in EXPECTED_FIELDS if f not in report] if isinstance(report, dict) else EXPECTED_FIELDS
        bad_coords = [f for f in ("latitude", "longitude")
                      if isinstance(report, dict) and not isinstance(report.get(f), (int, float))]
        problems = []
        if missing:
            problems.append(f"missing {', '.join(missing)}")
        if bad_coords:
            problems.append(f"non-numeric {', '.join(bad_coords)}")
        summary = ", ".join(f"{k}={report.get(k)}" for k in EXPECTED_FIELDS) if isinstance(report, dict) else repr(report)
        log(f"report from {source}, api key {api_key}: {summary}"
            + (f"  PROBLEMS: {'; '.join(problems)}" if problems else "  OK"))
        valid = valid and not problems

    if not valid:
        return {"error": "invalid report, see the server log"}, 400
    return {"status": "ok", "received": len(reports)}, 200
