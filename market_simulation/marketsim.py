import random

import matplotlib.pyplot as plt

class Buyer:
    def __init__(self, id, max_price):
        self.id = id
        self.max_price = max_price  # Maximum price willing to pay

    def decide_to_buy(self, price):
        return price <= self.max_price

class Seller:
    def __init__(self, id, min_price):
        self.id = id
        self.min_price = min_price  # Minimum price willing to accept

    def decide_to_sell(self, price):
        return price >= self.min_price

class Market:
    def __init__(self, num_buyers, num_sellers, initial_price):
        self.buyers = [Buyer(i, random.uniform(8, 15)) for i in range(num_buyers)]
        self.sellers = [Seller(i, random.uniform(5, 12)) for i in range(num_sellers)]
        self.price = initial_price
        self.price_history = [initial_price]
        self.demand_history = []
        self.supply_history = []

    def simulate_step(self, demand_shock=0):
        # Buyers decide to buy
        buyers_willing = [b for b in self.buyers if b.decide_to_buy(self.price)]
        # Sellers decide to sell
        sellers_willing = [s for s in self.sellers if s.decide_to_sell(self.price)]

        demand = len(buyers_willing) + demand_shock
        supply = len(sellers_willing)

        # Simple price adjustment mechanism
        if demand > supply:
            self.price *= 1 + 0.01 * (demand - supply) / max(1, supply)
        elif supply > demand:
            self.price *= 1 - 0.01 * (supply - demand) / max(1, demand)
        self.price = max(1, self.price)  # Prevent negative or zero price

        self.price_history.append(self.price)
        self.demand_history.append(demand)
        self.supply_history.append(supply)

    def run(self, steps=100):
        for t in range(steps):
            # Simulate demand shocks (e.g., random events)
            demand_shock = random.choice([0, 0, 1, -1])  # Mostly 0, sometimes +1 or -1
            self.simulate_step(demand_shock)

    def plot(self):
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(self.price_history, label='Price')
        plt.ylabel('Price')
        plt.legend()
        plt.subplot(2, 1, 2)
        plt.plot(self.demand_history, label='Demand')
        plt.plot(self.supply_history, label='Supply')
        plt.ylabel('Count')
        plt.xlabel('Time Step')
        plt.legend()
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    num_buyers = 50
    num_sellers = 40
    initial_price = 10

    market = Market(num_buyers, num_sellers, initial_price)
    market.run(steps=100)
    market.plot()