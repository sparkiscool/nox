"""Start the server over stdio and exercise the no-credential flow end to end."""

import asyncio
import json
import os
import sys
import tempfile
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]


def _data(result):
    assert not result.is_error, f"tool error: {result.content[0].text}"
    if result.structured_content is not None:
        sc = result.structured_content
        return sc["result"] if isinstance(sc, dict) and set(sc) == {"result"} else sc
    return json.loads(result.content[0].text)


async def main() -> None:
    tmp = tempfile.mkdtemp()
    env = {**os.environ, "AMAZON_PARTS_DATA": f"{tmp}/catalog.json", "AMAZON_ASSOCIATE_TAG": "testtag-20"}
    params = StdioServerParameters(command=sys.executable, args=[str(ROOT / "server.py")], env=env)
    async with stdio_client(params) as (r, w), ClientSession(r, w) as s:
        await s.initialize()
        tools = {t.name for t in (await s.list_tools()).tools}
        expected = {
            "add_drone", "list_drones", "add_part", "list_parts", "update_part", "remove_part",
            "build_cart", "build_restock_cart", "log_order", "order_history", "search_products",
            "get_product", "refresh_prices", "ordering_status", "place_business_order",
        }
        assert expected <= tools, expected - tools
        print(f"tools: {len(tools)} ok")

        d = _data(await s.call_tool("add_drone", {"name": "NOX", "notes": "5in freestyle"}))
        assert d["name"] == "NOX"

        p1 = _data(await s.call_tool("add_part", {
            "drone": "nox", "name": "2207 motor", "asin_or_url": "https://www.amazon.com/dp/B0CTESTMTR/ref=x",
            "qty_per_set": 4, "category": "motor", "last_price_usd": 18.99,
        }))
        assert p1["asin"] == "B0CTESTMTR" and p1["qty_per_set"] == 4, p1
        p2 = _data(await s.call_tool("add_part", {
            "drone": "NOX", "name": "5.1in props 4-pack", "asin_or_url": "b0ctestprp",
            "qty_per_set": 2, "category": "prop", "last_price_usd": 12.50,
        }))
        assert p2["asin"] == "B0CTESTPRP"
        parts = _data(await s.call_tool("list_parts", {"drone": "NOX"}))
        assert len(parts) == 2, parts
        print("catalog ok")

        cart = _data(await s.call_tool("build_restock_cart", {"drone": "NOX", "sets": 2}))
        url = cart["cart_url"]
        assert url.startswith("https://www.amazon.com/gp/aws/cart/add.html?"), url
        assert "AssociateTag=testtag-20" in url and "ASIN.1=" in url and "Quantity.2=" in url, url
        assert cart["estimated_total_usd"] == round((18.99 * 4 + 12.50 * 2) * 2, 2), cart
        print("restock cart ok:", url)

        only_props = _data(await s.call_tool("build_restock_cart", {"drone": "NOX", "categories": ["prop"]}))
        assert len(only_props["items"]) == 1 and only_props["items"][0]["quantity"] == 2
        adhoc = _data(await s.call_tool("build_cart", {"items": [{"asin_or_url": "B000000001", "quantity": 3}]}))
        assert "Quantity.1=3" in adhoc["cart_url"]
        print("filters + adhoc cart ok")

        o = _data(await s.call_tool("log_order", {
            "drone": "NOX", "total_usd": 90.46,
            "items": [{"asin_or_url": "B0CTESTMTR", "quantity": 4, "unit_price_usd": 17.49}],
        }))
        hist = _data(await s.call_tool("order_history", {"drone": "NOX"}))
        assert hist[0]["id"] == o["id"]
        parts = _data(await s.call_tool("list_parts", {"drone": "NOX", "category": "motor"}))
        assert parts[0]["last_price_usd"] == 17.49, parts
        print("order log + price refresh ok")

        sp = _data(await s.call_tool("search_products", {"keywords": "2207 brushless motor"}))
        assert sp["live_results"] is False and "amazon.com/s?" in sp["search_url"]
        st = _data(await s.call_tool("ordering_status", {}))
        assert st["cart_links"] and not st["business_ordering_enabled"]
        res = await s.call_tool("place_business_order", {"order": {"lineItems": [{}]}, "confirm_total_usd": 1, "user_confirmed": True})
        assert res.is_error and "not set" in res.content[0].text, res
        print("no-credential fallbacks + order gate ok")

        bad = await s.call_tool("add_part", {"drone": "NOX", "name": "x", "asin_or_url": "not-an-asin"})
        assert bad.is_error
        print("validation ok")
    print("ALL OK")


if __name__ == "__main__":
    asyncio.run(main())
