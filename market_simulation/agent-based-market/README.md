# Agent-Based Market

A simple Mesa-based agent market with a limit order book, three trader types, and basic market statistics/plots.

## Run

```bash
python agent_based_market.py
```

## Output

The script prints:
- Last 5 steps of price/volume/spread.
- Summary stats (price range, average volume/spread, return mean/volatility).
- Wealth distribution stats at the final step (min/percentiles/mean/gini).

It also shows a multi-panel chart with:
- Price series
- Volume series
- Bid-ask spread series
- Wealth distribution histogram (final step)

## Notes

- The plot uses the `TkAgg` backend to open a GUI window from the terminal.
- If you run in a headless environment, remove the backend override and use `plt.savefig(...)` instead.
