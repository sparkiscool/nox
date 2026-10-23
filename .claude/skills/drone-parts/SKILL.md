---
name: drone-parts
description: Buy or restock parts for the Nox builds (Scout, Overseer, box) on Amazon through the amazon-parts MCP server. Use when Spark wants to order, reorder, price or list bought parts (e.g. "order props for the scout", "what is still to buy for the box", "add this motor to the overseer's parts").
---

# Buying parts for Nox

The `amazon-parts` MCP server (`tools/amazon-parts-mcp/`) keeps `tools/amazon-parts-mcp/catalog.json`: one entry per build (scout, overseer, box), each part by Amazon ASIN with quantity per set and last price, plus an order log. `nox/parts-list.md` stays the human list of every part and its status; the catalog holds the Amazon side (ASINs, prices, carts).

## Flow

1. `list_drones` (builds: scout, overseer, box). `list_parts(drone)` for what is catalogued.
2. New part: get the ASIN (10 characters after `/dp/` in the Amazon URL). From a description, call `search_products`; when it returns `live_results: false`, web-search "amazon " + description, pick the listing, confirm with Spark, then `add_part`.
3. Order: `build_restock_cart(drone, sets, categories or part_names)` or `build_cart(items)`. Reply with the item list, quantities, estimated total and the `cart_url`. Spark clicks it and checks out in his Amazon account.
4. After Spark says it is ordered: `log_order` with the total and unit prices, set the part's row in `nox/parts-list.md` to **ordered**, add one line at the top of `nox/log.md`, commit and push (`catalog.json` is tracked).

## Rules

- Never call `place_business_order` unless `ordering_status` shows it enabled and Spark has confirmed the exact items and total in this conversation.
- No browser automation of amazon.com; the cart link is the path.
- `qty_per_set` is per complete build or rebuild (4 motors, 2 prop packs); `sets` multiplies it.
