from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from mesa import Agent, Model
from mesa.datacollection import DataCollector

# ==========================================================
# Limit Order Book
# ==========================================================


@dataclass
class Order:
    trader_id: int
    side: str  # "buy" or "sell"
    price: float
    qty: int
    time: int
    ttl: int


class LimitOrderBook:
    def __init__(self):
        self.bids: List[Order] = []
        self.asks: List[Order] = []
        self.last_trade_price: Optional[float] = None

    def add_order(self, order: Order):
        if order.side == "buy":
            self.bids.append(order)
            self.bids.sort(key=lambda o: (-o.price, o.time))
        else:
            self.asks.append(order)
            self.asks.sort(key=lambda o: (o.price, o.time))

    def decrement_ttl(self):
        for o in self.bids + self.asks:
            o.ttl -= 1

    def cancel_expired(self):
        self.bids = [o for o in self.bids if o.ttl > 0 and o.qty > 0]
        self.asks = [o for o in self.asks if o.ttl > 0 and o.qty > 0]

    def spread(self):
        if self.bids and self.asks:
            return self.asks[0].price - self.bids[0].price
        return None

    def match(self, model) -> int:
        volume = 0

        while self.bids and self.asks and self.bids[0].price >= self.asks[0].price:
            bid = self.bids[0]
            ask = self.asks[0]

            price = ask.price
            qty = min(bid.qty, ask.qty)

            buyer = model.agent_by_id(bid.trader_id)
            seller = model.agent_by_id(ask.trader_id)

            max_affordable = int(buyer.cash // price)
            qty = min(qty, max_affordable, seller.inventory)

            if qty <= 0:
                break

            buyer.cash -= price * qty
            buyer.inventory += qty
            seller.cash += price * qty
            seller.inventory -= qty

            bid.qty -= qty
            ask.qty -= qty

            volume += qty
            self.last_trade_price = price

            if bid.qty == 0:
                self.bids.pop(0)
            if ask.qty == 0:
                self.asks.pop(0)

        return volume


# ==========================================================
# Agents
# ==========================================================


class Trader(Agent):
    def __init__(self, model, cash, inventory):
        super().__init__(model)
        self.cash = cash
        self.inventory = inventory

    def place_order(self, side, price, qty, ttl):
        if side == "buy" and self.cash < price:
            return
        if side == "sell" and self.inventory < qty:
            return

        self.model.lob.add_order(
            Order(
                trader_id=self.unique_id,
                side=side,
                price=max(0.01, price),
                qty=qty,
                time=self.model.step_count,
                ttl=ttl,
            )
        )


class Fundamentalist(Trader):
    def step(self):
        p = self.model.current_price()
        f = self.model.fundamental_value(self.model.step_count)

        if f > p:
            self.place_order("buy", p + 0.5 * (f - p), 1, ttl=10)
        else:
            self.place_order("sell", p - 0.5 * (p - f), 1, ttl=10)


class Chartist(Trader):
    def step(self):
        trend = self.model.price_trend(10)
        p = self.model.current_price()

        if trend > 0:
            self.place_order("buy", p * 1.01, 1, ttl=5)
        elif trend < 0:
            self.place_order("sell", p * 0.99, 1, ttl=5)


class NoiseTrader(Trader):
    def step(self):
        p = self.model.current_price()
        side = "buy" if random.random() < 0.5 else "sell"
        price = p * (1 + random.gauss(0, 0.01))
        self.place_order(side, price, 1, ttl=3)


# ==========================================================
# Market Model (Mesa 3.x)
# ==========================================================


class MarketModel(Model):
    def __init__(
        self,
        n_fundamentalists=50,
        n_chartists=10,
        n_noise=200,
        initial_price=100.0,
        seed=1,
    ):
        super().__init__(seed=seed)

        self.step_count = 0
        self.lob = LimitOrderBook()
        self._price_history = [initial_price]
        self._volume_history = [0]

        self.traders: List[Trader] = []
        self.trader_by_id = {}

        for _ in range(n_fundamentalists):
            trader = Fundamentalist(self, 10_000, 10)
            self.traders.append(trader)
            self.trader_by_id[trader.unique_id] = trader
        for _ in range(n_chartists):
            trader = Chartist(self, 10_000, 10)
            self.traders.append(trader)
            self.trader_by_id[trader.unique_id] = trader
        for _ in range(n_noise):
            trader = NoiseTrader(self, 10_000, 10)
            self.traders.append(trader)
            self.trader_by_id[trader.unique_id] = trader

        self.datacollector = DataCollector(
            model_reporters={
                "price": lambda m: m.current_price(),
                "volume": lambda m: m._volume_history[-1],
                "spread": lambda m: m.lob.spread(),
            }
        )

    def agent_by_id(self, uid):
        return self.trader_by_id[uid]

    def fundamental_value(self, t):
        return 100 + random.gauss(0, 0.5)

    def current_price(self):
        return self.lob.last_trade_price or self._price_history[-1]

    def price_trend(self, lookback):
        if len(self._price_history) < lookback + 1:
            return 0.0
        return (self._price_history[-1] / self._price_history[-lookback - 1]) - 1.0

    def step(self):
        self.step_count += 1

        self.lob.decrement_ttl()
        self.lob.cancel_expired()

        traders = self.traders[:]
        random.shuffle(traders)
        for trader in traders:
            trader.step()

        volume = self.lob.match(self)
        price = self.current_price()

        self._price_history.append(price)
        self._volume_history.append(volume)

        self.datacollector.collect(self)


# ==========================================================
# Run
# ==========================================================

N_STEPS = 5000
if __name__ == "__main__":
    model = MarketModel()

    for _ in range(N_STEPS):
        model.step()

    df = model.datacollector.get_model_vars_dataframe()
    print("Last 5 steps:")
    print(df.tail())

    price = df["price"]
    volume = df["volume"]
    spread = df["spread"]

    returns = price.pct_change().dropna()
    summary = {
        "price_min": price.min(),
        "price_max": price.max(),
        "price_last": price.iloc[-1],
        "avg_volume": volume.mean(),
        "total_volume": volume.sum(),
        "avg_spread": spread.mean(skipna=True),
        "volatility": returns.std(),
        "return_mean": returns.mean(),
    }

    final_price = price.iloc[-1]
    wealth = np.array(
        [t.cash + t.inventory * final_price for t in model.traders], dtype=float
    )
    wealth_by_trader = list(zip(model.traders, wealth, strict=True))
    poorest_trader, poorest_wealth = min(wealth_by_trader, key=lambda x: x[1])
    richest_trader, richest_wealth = max(wealth_by_trader, key=lambda x: x[1])
    wealth_sorted = np.sort(wealth)
    n = len(wealth_sorted)
    if wealth_sorted.sum() > 0:
        gini = ((2 * np.arange(1, n + 1) - n - 1) @ wealth_sorted) / (
            n * wealth_sorted.sum()
        )
    else:
        gini = 0.0

    wealth_summary = {
        "wealth_min": wealth_sorted[0],
        "wealth_p10": np.percentile(wealth_sorted, 10),
        "wealth_median": np.median(wealth_sorted),
        "wealth_p90": np.percentile(wealth_sorted, 90),
        "wealth_max": wealth_sorted[-1],
        "wealth_mean": wealth_sorted.mean(),
        "wealth_gini": gini,
    }

    print("\nSummary:")
    for k, v in summary.items():
        print(f"{k:>12}: {v:.4f}")

    print("\nWealth distribution (final step):")
    for k, v in wealth_summary.items():
        print(f"{k:>12}: {v:.4f}")
    print(
        f"     poorest: id={poorest_trader.unique_id} "
        f"type={poorest_trader.__class__.__name__} wealth={poorest_wealth:.4f}"
    )
    print(
        f"     richest: id={richest_trader.unique_id} "
        f"type={richest_trader.__class__.__name__} wealth={richest_wealth:.4f}"
    )

    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(price, color="tab:blue", label="Price")
    axes[0].set_title("Agent-Based Market Simulation")
    axes[0].set_ylabel("Price")
    axes[0].legend(loc="upper left")

    axes[1].plot(volume, color="tab:green", label="Volume")
    axes[1].set_ylabel("Volume")
    axes[1].legend(loc="upper left")

    axes[2].plot(spread, color="tab:orange", label="Spread")
    axes[2].set_ylabel("Spread")
    axes[2].set_xlabel("Step")
    axes[2].legend(loc="upper left")

    fig.tight_layout()
    plt.show()

    fig2, ax2 = plt.subplots(figsize=(8, 5))
    ax2.hist(wealth, bins=20, color="tab:purple", alpha=0.7)
    ax2.set_title("Wealth Distribution (Final Step)")
    ax2.set_xlabel("Wealth")
    ax2.set_ylabel("Count")
    fig2.tight_layout()
    plt.show()
