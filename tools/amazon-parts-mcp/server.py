"""
amazon-parts-mcp: an MCP server for finding and ordering drone parts on Amazon
from a chat session.

Layers (each works without the next):
  1. Parts catalog   - drones, their parts (ASINs), and an order log, stored in a JSON file.
  2. Cart links      - Amazon's documented "Add to Cart" form URL. No credentials needed.
                       Claude builds the link, you click it, review the cart, and check out.
  3. Live lookups    - Product Advertising API 5.0 (search + prices). Needs an Amazon
                       Associates account and PA-API keys.
  4. Direct ordering - Amazon Business Ordering API placeOrder. Needs an approved Amazon
                       Business developer app and is gated behind an explicit env flag.

Run:  python3 server.py                     (stdio, for Claude Code / Claude Desktop)
      python3 server.py --http --port 8765  (streamable HTTP, for claude.ai remote connectors)
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import hmac
import json
import os
import re
import secrets
import urllib.parse
from pathlib import Path
from typing import Any

import httpx
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError

# ----------------------------------------------------------------------------- config

DATA_PATH = Path(os.environ.get("AMAZON_PARTS_DATA") or Path(__file__).resolve().parent / "catalog.json")
AMAZON_DOMAIN = os.environ.get("AMAZON_DOMAIN", "www.amazon.com")
ASSOCIATE_TAG = os.environ.get("AMAZON_ASSOCIATE_TAG", "")

# Product Advertising API 5.0 (optional)
PAAPI_ACCESS_KEY = os.environ.get("PAAPI_ACCESS_KEY", "")
PAAPI_SECRET_KEY = os.environ.get("PAAPI_SECRET_KEY", "")
PAAPI_HOST = os.environ.get("PAAPI_HOST", "webservices.amazon.com")
PAAPI_REGION = os.environ.get("PAAPI_REGION", "us-east-1")

# Amazon Business Ordering API (optional, gated)
AB_CLIENT_ID = os.environ.get("AB_CLIENT_ID", "")
AB_CLIENT_SECRET = os.environ.get("AB_CLIENT_SECRET", "")
AB_REFRESH_TOKEN = os.environ.get("AB_REFRESH_TOKEN", "")
AB_HOST = os.environ.get("AB_HOST", "na.business-api.amazon.com")
ALLOW_ORDERS = os.environ.get("AMAZON_PARTS_ALLOW_ORDERS", "") == "1"

ASIN_RE = re.compile(r"^[A-Z0-9]{10}$")

mcp = MCPServer(
    "amazon-parts",
    version="0.1.0",
    instructions=(
        "Helps the user keep a parts list for each of their drones and buy those parts on Amazon. "
        "Typical flow: look up the drone's parts with list_parts, then build_restock_cart or build_cart "
        "to get a one-click Amazon cart link, then log_order once the user has checked out. "
        "New parts are added with add_part using the Amazon ASIN (the 10-character code in a product URL). "
        "search_products only returns live results when PA-API keys are configured; otherwise use web search "
        "to find the ASIN and then add_part. place_business_order places a real order and must only be called "
        "after the user has confirmed the items and total in the conversation."
    ),
)

# ----------------------------------------------------------------------------- storage


def _now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")


def _load() -> dict[str, Any]:
    if DATA_PATH.exists():
        return json.loads(DATA_PATH.read_text())
    return {"drones": {}, "parts": {}, "orders": []}


def _save(db: dict[str, Any]) -> None:
    DATA_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = DATA_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(db, indent=2, sort_keys=True))
    tmp.replace(DATA_PATH)


def _norm(name: str) -> str:
    return re.sub(r"\s+", " ", name.strip()).lower()


def _asin(value: str) -> str:
    """Accept a bare ASIN or any Amazon product URL and return the ASIN."""
    v = value.strip()
    m = re.search(r"/(?:dp|gp/product|gp/aws/cart/add\.html\?ASIN\.1=)/?([A-Z0-9]{10})(?:[/?&]|$)", v)
    if m:
        return m.group(1)
    v = v.upper()
    if not ASIN_RE.match(v):
        raise ToolError(f"'{value}' is not a valid ASIN (10 letters/digits) or Amazon product URL")
    return v


def product_url(asin: str) -> str:
    url = f"https://{AMAZON_DOMAIN}/dp/{asin}"
    return f"{url}?tag={ASSOCIATE_TAG}" if ASSOCIATE_TAG else url


def cart_url(items: list[tuple[str, int]]) -> str:
    params: dict[str, str] = {}
    if ASSOCIATE_TAG:
        params["AssociateTag"] = ASSOCIATE_TAG
    for i, (asin, qty) in enumerate(items, start=1):
        params[f"ASIN.{i}"] = asin
        params[f"Quantity.{i}"] = str(qty)
    return f"https://{AMAZON_DOMAIN}/gp/aws/cart/add.html?{urllib.parse.urlencode(params)}"


# ----------------------------------------------------------------------------- catalog tools


@mcp.tool()
def add_drone(name: str, notes: str = "") -> dict:
    """Register a drone (or any build) that parts will be tracked for. Idempotent on name."""
    db = _load()
    key = _norm(name)
    if key in db["drones"]:
        if notes:
            db["drones"][key]["notes"] = notes
    else:
        db["drones"][key] = {"name": name.strip(), "notes": notes, "created": _now()}
    _save(db)
    return db["drones"][key]


@mcp.tool()
def list_drones() -> list[dict]:
    """List registered drones with how many parts each has in the catalog."""
    db = _load()
    out = []
    for key, d in db["drones"].items():
        n = sum(1 for p in db["parts"].values() if p["drone"] == key)
        out.append({**d, "part_count": n})
    return out


@mcp.tool()
def add_part(
    drone: str,
    name: str,
    asin_or_url: str,
    qty_per_set: int = 1,
    category: str = "",
    last_price_usd: float | None = None,
    notes: str = "",
) -> dict:
    """Add an Amazon part to a drone's catalog.

    asin_or_url: the 10-character ASIN or a full Amazon product URL.
    qty_per_set: how many of this part one full restock (one "set") needs, e.g. 4 motors.
    category: free text such as motor, prop, battery, frame, fc, esc, camera, vtx, hardware.
    """
    asin = _asin(asin_or_url)
    db = _load()
    dkey = _norm(drone)
    if dkey not in db["drones"]:
        db["drones"][dkey] = {"name": drone.strip(), "notes": "", "created": _now()}
    part_id = f"{dkey}:{asin}"
    part = db["parts"].get(part_id, {"id": part_id, "created": _now()})
    part.update(
        {
            "drone": dkey,
            "name": name.strip(),
            "asin": asin,
            "url": product_url(asin),
            "qty_per_set": max(1, int(qty_per_set)),
            "category": category.strip().lower(),
            "notes": notes,
            "updated": _now(),
        }
    )
    if last_price_usd is not None:
        part["last_price_usd"] = round(float(last_price_usd), 2)
        part["price_checked"] = _now()
    db["parts"][part_id] = part
    _save(db)
    return part


@mcp.tool()
def list_parts(drone: str = "", category: str = "") -> list[dict]:
    """List catalogued parts, optionally filtered by drone and/or category."""
    db = _load()
    dkey = _norm(drone) if drone else ""
    cat = category.strip().lower()
    return [
        p
        for p in sorted(db["parts"].values(), key=lambda p: (p["drone"], p["category"], p["name"]))
        if (not dkey or p["drone"] == dkey) and (not cat or p["category"] == cat)
    ]


@mcp.tool()
def update_part(
    part_id: str,
    name: str | None = None,
    asin_or_url: str | None = None,
    qty_per_set: int | None = None,
    category: str | None = None,
    last_price_usd: float | None = None,
    notes: str | None = None,
) -> dict:
    """Change fields on a catalogued part. part_id comes from list_parts (format drone:ASIN)."""
    db = _load()
    part = db["parts"].get(part_id)
    if not part:
        raise ToolError(f"No part with id {part_id}")
    if name is not None:
        part["name"] = name.strip()
    if asin_or_url is not None:
        part["asin"] = _asin(asin_or_url)
        part["url"] = product_url(part["asin"])
    if qty_per_set is not None:
        part["qty_per_set"] = max(1, int(qty_per_set))
    if category is not None:
        part["category"] = category.strip().lower()
    if notes is not None:
        part["notes"] = notes
    if last_price_usd is not None:
        part["last_price_usd"] = round(float(last_price_usd), 2)
        part["price_checked"] = _now()
    part["updated"] = _now()
    new_id = f"{part['drone']}:{part['asin']}"
    if new_id != part_id:
        del db["parts"][part_id]
        part["id"] = new_id
    db["parts"][new_id] = part
    _save(db)
    return part


@mcp.tool()
def remove_part(part_id: str) -> dict:
    """Remove a part from the catalog."""
    db = _load()
    part = db["parts"].pop(part_id, None)
    if not part:
        raise ToolError(f"No part with id {part_id}")
    _save(db)
    return {"removed": part_id}


# ----------------------------------------------------------------------------- cart tools


@mcp.tool()
def build_cart(items: list[dict]) -> dict:
    """Build a one-click Amazon 'Add to Cart' link for arbitrary items.

    items: list of {"asin_or_url": "...", "quantity": n}. The user opens the link, Amazon adds
    everything to their cart, and they check out normally in their own account.
    """
    if not items:
        raise ToolError("items is empty")
    resolved = [(_asin(it["asin_or_url"]), max(1, int(it.get("quantity", 1)))) for it in items]
    return {
        "cart_url": cart_url(resolved),
        "items": [{"asin": a, "quantity": q, "url": product_url(a)} for a, q in resolved],
        "next_step": "Open cart_url, review quantities and prices, check out, then call log_order.",
    }


@mcp.tool()
def build_restock_cart(drone: str, sets: int = 1, part_names: list[str] | None = None, categories: list[str] | None = None) -> dict:
    """Build an Amazon cart link for a drone's catalogued parts.

    sets: how many complete sets to buy (quantity = qty_per_set x sets).
    part_names / categories: optional filters (case-insensitive substring match on name; exact on category).
    """
    db = _load()
    dkey = _norm(drone)
    if dkey not in db["drones"]:
        raise ToolError(f"Unknown drone '{drone}'. Known: {[d['name'] for d in db['drones'].values()]}")
    names = [_norm(n) for n in (part_names or [])]
    cats = [c.strip().lower() for c in (categories or [])]
    chosen = []
    for p in db["parts"].values():
        if p["drone"] != dkey:
            continue
        if names and not any(n in _norm(p["name"]) for n in names):
            continue
        if cats and p["category"] not in cats:
            continue
        chosen.append(p)
    if not chosen:
        raise ToolError("No parts matched. Call list_parts to see what is catalogued.")
    sets = max(1, int(sets))
    lines = [(p["asin"], p["qty_per_set"] * sets) for p in chosen]
    est = sum(p.get("last_price_usd", 0) * p["qty_per_set"] * sets for p in chosen)
    priced = all("last_price_usd" in p for p in chosen)
    return {
        "drone": db["drones"][dkey]["name"],
        "sets": sets,
        "cart_url": cart_url(lines),
        "items": [
            {"name": p["name"], "asin": p["asin"], "quantity": p["qty_per_set"] * sets, "last_price_usd": p.get("last_price_usd")}
            for p in chosen
        ],
        "estimated_total_usd": round(est, 2) if priced else None,
        "estimate_note": "Based on last recorded prices; Amazon shows live prices in the cart." if priced else "Some parts have no recorded price.",
    }


# ----------------------------------------------------------------------------- order log


@mcp.tool()
def log_order(drone: str, items: list[dict], total_usd: float | None = None, amazon_order_id: str = "", notes: str = "") -> dict:
    """Record an order after checkout so future sessions know what was bought and when.

    items: list of {"asin_or_url": "...", "quantity": n, "unit_price_usd": optional}.
    Also refreshes last_price_usd on matching catalog parts when unit_price_usd is given.
    """
    db = _load()
    dkey = _norm(drone)
    lines = []
    for it in items:
        asin = _asin(it["asin_or_url"])
        line = {"asin": asin, "quantity": max(1, int(it.get("quantity", 1)))}
        if it.get("unit_price_usd") is not None:
            line["unit_price_usd"] = round(float(it["unit_price_usd"]), 2)
            pid = f"{dkey}:{asin}"
            if pid in db["parts"]:
                db["parts"][pid]["last_price_usd"] = line["unit_price_usd"]
                db["parts"][pid]["price_checked"] = _now()
        lines.append(line)
    order = {
        "id": f"ord_{secrets.token_hex(4)}",
        "drone": dkey,
        "placed": _now(),
        "items": lines,
        "total_usd": round(float(total_usd), 2) if total_usd is not None else None,
        "amazon_order_id": amazon_order_id,
        "notes": notes,
    }
    db["orders"].append(order)
    _save(db)
    return order


@mcp.tool()
def order_history(drone: str = "", limit: int = 20) -> list[dict]:
    """Most recent logged orders, newest first, optionally filtered by drone."""
    db = _load()
    dkey = _norm(drone) if drone else ""
    orders = [o for o in db["orders"] if not dkey or o["drone"] == dkey]
    return list(reversed(orders))[: max(1, int(limit))]


# ----------------------------------------------------------------------------- PA-API (optional)


def _sigv4_headers(method: str, host: str, path: str, region: str, service: str, target: str, body: str) -> dict[str, str]:
    t = dt.datetime.now(dt.timezone.utc)
    amz_date = t.strftime("%Y%m%dT%H%M%SZ")
    date_stamp = t.strftime("%Y%m%d")
    headers = {
        "content-encoding": "amz-1.0",
        "content-type": "application/json; charset=utf-8",
        "host": host,
        "x-amz-date": amz_date,
        "x-amz-target": target,
    }
    signed = ";".join(sorted(headers))
    canonical_headers = "".join(f"{k}:{headers[k]}\n" for k in sorted(headers))
    payload_hash = hashlib.sha256(body.encode()).hexdigest()
    canonical_request = f"{method}\n{path}\n\n{canonical_headers}\n{signed}\n{payload_hash}"
    scope = f"{date_stamp}/{region}/{service}/aws4_request"
    string_to_sign = f"AWS4-HMAC-SHA256\n{amz_date}\n{scope}\n{hashlib.sha256(canonical_request.encode()).hexdigest()}"

    def _sign(key: bytes, msg: str) -> bytes:
        return hmac.new(key, msg.encode(), hashlib.sha256).digest()

    k = _sign(("AWS4" + PAAPI_SECRET_KEY).encode(), date_stamp)
    k = _sign(k, region)
    k = _sign(k, service)
    k = _sign(k, "aws4_request")
    signature = hmac.new(k, string_to_sign.encode(), hashlib.sha256).hexdigest()
    headers["authorization"] = (
        f"AWS4-HMAC-SHA256 Credential={PAAPI_ACCESS_KEY}/{scope}, SignedHeaders={signed}, Signature={signature}"
    )
    return headers


def _paapi(operation: str, payload: dict) -> dict:
    payload = {**payload, "PartnerTag": ASSOCIATE_TAG, "PartnerType": "Associates", "Marketplace": AMAZON_DOMAIN}
    body = json.dumps(payload)
    path = f"/paapi5/{operation.lower()}"
    target = f"com.amazon.paapi5.v1.ProductAdvertisingAPIv1.{operation}"
    headers = _sigv4_headers("POST", PAAPI_HOST, path, PAAPI_REGION, "ProductAdvertisingAPI", target, body)
    r = httpx.post(f"https://{PAAPI_HOST}{path}", content=body, headers=headers, timeout=20)
    data = r.json()
    if r.status_code >= 400 or "Errors" in data:
        raise ToolError(f"PA-API {operation} failed ({r.status_code}): {data.get('Errors', data)}")
    return data


def _paapi_item(item: dict) -> dict:
    info = item.get("ItemInfo", {})
    listing = (item.get("Offers", {}).get("Listings") or [{}])[0]
    price = listing.get("Price", {})
    return {
        "asin": item.get("ASIN"),
        "title": info.get("Title", {}).get("DisplayValue"),
        "brand": info.get("ByLineInfo", {}).get("Brand", {}).get("DisplayValue"),
        "price_usd": price.get("Amount"),
        "price_display": price.get("DisplayAmount"),
        "prime": listing.get("DeliveryInfo", {}).get("IsPrimeEligible"),
        "in_stock": listing.get("Availability", {}).get("Message"),
        "url": item.get("DetailPageURL"),
    }


PAAPI_RESOURCES = [
    "ItemInfo.Title",
    "ItemInfo.ByLineInfo",
    "Offers.Listings.Price",
    "Offers.Listings.Availability.Message",
    "Offers.Listings.DeliveryInfo.IsPrimeEligible",
]


def _paapi_ready() -> bool:
    return bool(PAAPI_ACCESS_KEY and PAAPI_SECRET_KEY and ASSOCIATE_TAG)


@mcp.tool()
def search_products(keywords: str, max_results: int = 5) -> dict:
    """Search Amazon for parts.

    With PA-API keys configured this returns live titles, prices and ASINs. Without them it returns
    an Amazon search URL and tells the assistant to use web search to find the ASIN instead.
    """
    if not _paapi_ready():
        return {
            "live_results": False,
            "search_url": f"https://{AMAZON_DOMAIN}/s?{urllib.parse.urlencode({'k': keywords})}",
            "how_to": (
                "PA-API keys are not configured. Use web search (query: 'amazon ' + keywords) to find the product, "
                "take the 10-character ASIN from its URL (/dp/XXXXXXXXXX), then call add_part or build_cart with it."
            ),
        }
    data = _paapi("SearchItems", {"Keywords": keywords, "ItemCount": max(1, min(10, int(max_results))), "Resources": PAAPI_RESOURCES})
    items = data.get("SearchResult", {}).get("Items", [])
    return {"live_results": True, "results": [_paapi_item(i) for i in items]}


@mcp.tool()
def get_product(asin_or_url: str) -> dict:
    """Current title, price and availability for one ASIN (live only with PA-API keys)."""
    asin = _asin(asin_or_url)
    if not _paapi_ready():
        return {"live_results": False, "asin": asin, "url": product_url(asin), "how_to": "Open the URL (or web-fetch it) to read the current price."}
    data = _paapi("GetItems", {"ItemIds": [asin], "Resources": PAAPI_RESOURCES})
    items = data.get("ItemsResult", {}).get("Items", [])
    if not items:
        raise ToolError(f"ASIN {asin} not found")
    return {"live_results": True, **_paapi_item(items[0])}


@mcp.tool()
def refresh_prices(drone: str = "") -> dict:
    """Update last_price_usd for catalogued parts from PA-API (requires keys)."""
    if not _paapi_ready():
        raise ToolError("PA-API keys are not configured; see README.")
    db = _load()
    dkey = _norm(drone) if drone else ""
    parts = [p for p in db["parts"].values() if not dkey or p["drone"] == dkey]
    updated = []
    for i in range(0, len(parts), 10):  # GetItems takes up to 10 ASINs
        chunk = parts[i : i + 10]
        data = _paapi("GetItems", {"ItemIds": [p["asin"] for p in chunk], "Resources": PAAPI_RESOURCES})
        by_asin = {it["ASIN"]: _paapi_item(it) for it in data.get("ItemsResult", {}).get("Items", [])}
        for p in chunk:
            live = by_asin.get(p["asin"])
            if live and live.get("price_usd") is not None:
                p["last_price_usd"] = live["price_usd"]
                p["price_checked"] = _now()
                updated.append({"id": p["id"], "price_usd": live["price_usd"], "in_stock": live.get("in_stock")})
    _save(db)
    return {"updated": updated, "count": len(updated)}


# ----------------------------------------------------------------------------- Amazon Business ordering (optional, gated)


def _ab_access_token() -> str:
    r = httpx.post(
        "https://api.amazon.com/auth/o2/token",
        data={
            "grant_type": "refresh_token",
            "refresh_token": AB_REFRESH_TOKEN,
            "client_id": AB_CLIENT_ID,
            "client_secret": AB_CLIENT_SECRET,
        },
        timeout=20,
    )
    r.raise_for_status()
    return r.json()["access_token"]


@mcp.tool()
def ordering_status() -> dict:
    """Report which purchasing paths are available with the current configuration."""
    return {
        "cart_links": True,
        "paapi_live_prices": _paapi_ready(),
        "business_ordering_configured": bool(AB_CLIENT_ID and AB_CLIENT_SECRET and AB_REFRESH_TOKEN),
        "business_ordering_enabled": ALLOW_ORDERS,
        "associate_tag_set": bool(ASSOCIATE_TAG),
        "data_file": str(DATA_PATH),
        "amazon_domain": AMAZON_DOMAIN,
    }


@mcp.tool()
def place_business_order(order: dict, confirm_total_usd: float, user_confirmed: bool = False) -> dict:
    """Place a REAL order through the Amazon Business Ordering API (placeOrder).

    Only works when AB_* credentials are set AND AMAZON_PARTS_ALLOW_ORDERS=1. `order` is the request
    body documented by Amazon Business (externalId, lineItems with ASIN product references and expected
    unit prices, shipping address, payment reference). The assistant must show the user the items and
    confirm_total_usd and get an explicit yes in the conversation before setting user_confirmed=true.
    """
    if not (AB_CLIENT_ID and AB_CLIENT_SECRET and AB_REFRESH_TOKEN):
        raise ToolError("Amazon Business credentials (AB_CLIENT_ID/AB_CLIENT_SECRET/AB_REFRESH_TOKEN) are not set. Use build_cart instead.")
    if not ALLOW_ORDERS:
        raise ToolError("Direct ordering is disabled. Set AMAZON_PARTS_ALLOW_ORDERS=1 to enable it.")
    if not user_confirmed:
        raise ToolError("user_confirmed must be true. Show the user the items and total first and get an explicit yes.")
    if not order.get("lineItems"):
        raise ToolError("order.lineItems is empty")
    order.setdefault("externalId", f"drone-parts-{secrets.token_hex(6)}")
    token = _ab_access_token()
    r = httpx.post(
        f"https://{AB_HOST}/ordering/2022-10-30/orders",
        json=order,
        headers={
            "x-amz-access-token": token,
            "content-type": "application/json",
            "user-agent": "amazon-parts-mcp/0.1 (Language=Python)",
        },
        timeout=30,
    )
    body = r.json() if r.content else {}
    if r.status_code >= 400:
        raise ToolError(f"placeOrder failed ({r.status_code}): {body}")
    db = _load()
    db["orders"].append(
        {
            "id": f"ord_{secrets.token_hex(4)}",
            "drone": "",
            "placed": _now(),
            "items": order["lineItems"],
            "total_usd": round(float(confirm_total_usd), 2),
            "amazon_order_id": body.get("orderId") or body.get("externalId", ""),
            "notes": "placed via Amazon Business Ordering API",
        }
    )
    _save(db)
    return body


# ----------------------------------------------------------------------------- entrypoint


def main() -> None:
    ap = argparse.ArgumentParser(description="amazon-parts MCP server")
    ap.add_argument("--http", action="store_true", help="serve streamable HTTP instead of stdio")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    if args.http:
        mcp.run("streamable-http", host=args.host, port=args.port)
    else:
        mcp.run("stdio")


if __name__ == "__main__":
    main()
