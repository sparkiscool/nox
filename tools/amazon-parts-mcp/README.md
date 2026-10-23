# amazon-parts-mcp

An MCP server so you can say, in any Claude chat, "order a fresh set of props and two batteries for NOX" and get it done.

Amazon does not offer a public API for consumer purchases, so the server works in layers. Each layer works without the ones below it.

| Layer | What you get | Needs |
| --- | --- | --- |
| Parts catalog | Each drone's parts with ASINs, quantities per set, last prices, order log | Nothing |
| Cart links | A one-click Amazon "Add to Cart" link with every part and quantity pre-filled. You review and check out in your own Amazon account | Nothing |
| Live search and prices | `search_products`, `get_product`, `refresh_prices` | Amazon Associates account plus Product Advertising API keys |
| Direct ordering | `place_business_order` places the order with no browser step | Amazon Business account, approved developer app, and `AMAZON_PARTS_ALLOW_ORDERS=1` |

## Install

```bash
cd amazon-parts-mcp
uv sync
uv run server.py --help
```

### Claude Code (this machine)

Add to `~/.claude.json` under `mcpServers`, or to a project `.mcp.json`:

```json
{
  "mcpServers": {
    "amazon-parts": {
      "command": "uv",
      "args": ["run", "--directory", "/ABSOLUTE/PATH/amazon-parts-mcp", "server.py"],
      "env": {
        "AMAZON_PARTS_DATA": "/ABSOLUTE/PATH/amazon-parts-mcp/data/catalog.json"
      }
    }
  }
}
```

Or from the terminal:

```bash
claude mcp add amazon-parts -- uv run --directory /ABSOLUTE/PATH/amazon-parts-mcp server.py
```

### Claude Desktop

Same JSON block in `claude_desktop_config.json`.

### claude.ai web and mobile (remote connector)

claude.ai connectors need an HTTPS URL. Run the HTTP transport behind a reverse proxy or tunnel, then add the URL under Settings, Connectors, Add custom connector:

```bash
uv run server.py --http --host 0.0.0.0 --port 8765
# e.g. expose with: cloudflared tunnel --url http://localhost:8765
```

Put authentication in front of it (the server itself has none). Keep the catalog file on a persistent disk.

## Configuration

Copy `.env.example` and set what you have. Everything is optional.

| Variable | Purpose |
| --- | --- |
| `AMAZON_PARTS_DATA` | Path to the JSON catalog. Default `catalog.json` next to `server.py` (tracked in this repo so every session shares it) |
| `AMAZON_DOMAIN` | `www.amazon.com` by default; set `www.amazon.co.uk` etc. for other stores |
| `AMAZON_ASSOCIATE_TAG` | Associates tag added to links. Required for PA-API |
| `PAAPI_ACCESS_KEY`, `PAAPI_SECRET_KEY` | Product Advertising API 5.0 keys |
| `PAAPI_HOST`, `PAAPI_REGION` | Default `webservices.amazon.com`, `us-east-1` |
| `AB_CLIENT_ID`, `AB_CLIENT_SECRET`, `AB_REFRESH_TOKEN` | Login with Amazon credentials for an approved Amazon Business app |
| `AB_HOST` | Default `na.business-api.amazon.com` |
| `AMAZON_PARTS_ALLOW_ORDERS` | Must be `1` before `place_business_order` will send anything |

## Tools

| Tool | Use |
| --- | --- |
| `add_drone`, `list_drones` | Register builds (e.g. "NOX") |
| `add_part`, `list_parts`, `update_part`, `remove_part` | Maintain each drone's parts list by ASIN |
| `search_products`, `get_product`, `refresh_prices` | Live Amazon data when PA-API keys exist; otherwise returns a search URL and tells Claude to web-search for the ASIN |
| `build_restock_cart` | Cart link for a drone's parts, with `sets`, `part_names`, `categories` filters |
| `build_cart` | Cart link for any ad-hoc list of ASINs and quantities |
| `log_order`, `order_history` | Remember what was bought, when, and at what price |
| `ordering_status` | Which layers are active in this configuration |
| `place_business_order` | Real purchase through Amazon Business, triple-gated (credentials, env flag, explicit user confirmation) |

## Example conversation

> **You:** add my drone NOX, it's a 5-inch freestyle quad
> **Claude:** calls `add_drone("NOX", ...)`
>
> **You:** the motors are https://www.amazon.com/dp/B0XXXXXXXX, it takes 4
> **Claude:** calls `add_part("NOX", "motor", "B0XXXXXXXX", qty_per_set=4, category="motor")`
>
> **You (next month):** I crashed NOX, need all four motors and a set of props
> **Claude:** calls `build_restock_cart("NOX", categories=["motor", "prop"])`, returns the cart link and the estimated total
>
> **You:** ordered, came to $86.40
> **Claude:** calls `log_order(...)` so prices and history stay current

## Honest limits

- The Amazon cart link uses Amazon's documented Add to Cart form. It pre-fills the cart; you still click Place Order.
- Browser automation of amazon.com was deliberately left out. Amazon's agent policy prohibits scraping and undocumented endpoints, and it can get an account locked.
- `place_business_order` implements the documented `POST /ordering/2022-10-30/orders` call with Login with Amazon token refresh, but it has not been exercised against a live Amazon Business account. Amazon now requires approval before developer registration. Expect to adjust the request body to the current Ordering API schema.
- PA-API access requires an Associates account that has made qualifying sales; new accounts get keys but they are revoked without sales within 180 days.

## Development

```bash
uv run python tests/smoke_test.py
```

The smoke test starts the server over stdio, lists its tools, and runs the catalog and cart flow against a temporary data file.
