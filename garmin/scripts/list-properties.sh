#!/usr/bin/env bash
# Print Arduino Cloud property names and ids for the tank Thing.
# Reads garmin/source/Secrets.mc, unless CLIENT_ID, CLIENT_SECRET, and
# THING_ID are already set in the environment.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SECRETS="$ROOT/source/Secrets.mc"

extract() {
    local key="$1"
    sed -n "s/.*${key}[[:space:]]*=[[:space:]]*\"\\([^\"]*\\)\".*/\\1/p" "$SECRETS" | head -n 1
}

if [[ -z "${CLIENT_ID:-}" || -z "${CLIENT_SECRET:-}" || -z "${THING_ID:-}" ]]; then
    if [[ ! -f "$SECRETS" ]]; then
        echo "Copy source/Secrets.mc.example to source/Secrets.mc and fill in the Arduino Cloud credentials." >&2
        exit 1
    fi
    CLIENT_ID="${CLIENT_ID:-$(extract CLIENT_ID)}"
    CLIENT_SECRET="${CLIENT_SECRET:-$(extract CLIENT_SECRET)}"
    THING_ID="${THING_ID:-$(extract THING_ID)}"
fi

if [[ -z "$CLIENT_ID" || -z "$CLIENT_SECRET" || -z "$THING_ID" || "$CLIENT_ID" == YOUR_* || "$CLIENT_SECRET" == YOUR_* || "$THING_ID" == YOUR_* ]]; then
    echo "Fill CLIENT_ID, CLIENT_SECRET, and THING_ID in source/Secrets.mc first." >&2
    exit 1
fi

token_file="$(mktemp)"
props_file="$(mktemp)"
trap 'rm -f "$token_file" "$props_file"' EXIT

token_code="$(curl -sS -o "$token_file" -w '%{http_code}' \
    -X POST 'https://api2.arduino.cc/iot/v1/clients/token' \
    -H 'Content-Type: application/x-www-form-urlencoded' \
    -H 'Accept: application/json' \
    --data-urlencode 'grant_type=client_credentials' \
    --data-urlencode "client_id=${CLIENT_ID}" \
    --data-urlencode "client_secret=${CLIENT_SECRET}" \
    --data-urlencode 'audience=https://api2.arduino.cc/iot')"

if [[ "$token_code" != "200" ]]; then
    echo "Token request failed with HTTP ${token_code}." >&2
    exit 1
fi

access_token="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["access_token"])' "$token_file")"

props_code="$(curl -sS -o "$props_file" -w '%{http_code}' \
    -H "Authorization: Bearer ${access_token}" \
    -H 'Accept: application/json' \
    "https://api2.arduino.cc/iot/v2/things/${THING_ID}/properties")"

if [[ "$props_code" != "200" ]]; then
    echo "Property list failed with HTTP ${props_code}." >&2
    exit 1
fi

python3 - "$props_file" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1]))
if not isinstance(data, list):
    print("Unexpected property response.", file=sys.stderr)
    sys.exit(1)

print(f"{'variable_name':<24} {'name':<24} {'id':<38} last_value")
found = set()
for prop in data:
    variable = str(prop.get("variable_name") or "")
    name = str(prop.get("name") or "")
    print(f"{variable:<24} {name:<24} {prop.get('id',''):<38} {prop.get('last_value')}")
    found.add(variable)
    found.add(name)

missing = [key for key in ("nivel_percentual", "abrir_valvula") if key not in found]
if missing:
    print("Missing: " + ", ".join(missing), file=sys.stderr)
    sys.exit(2)
PY
