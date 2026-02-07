import multiprocessing as mp
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

# =========================
# GLOBAL CONFIG
# =========================

N_SIM = 100000  # increase freely; parallelized
START_AGE = 22
END_AGE = 65
YEARS = END_AGE - START_AGE

INITIAL_WEALTH = 0
BASE_SALARY = 50_000
EDUCATED_SALARY = 70_000

STUDENT_LOAN = 80_000
LOAN_RATE = 0.05
LOAN_TERM = 10

OUTPUT_DIR = "outputs"
os.makedirs(OUTPUT_DIR, exist_ok=True)

np.random.seed()  # allow independent worker seeds

# =========================
# MODEL FUNCTIONS
# =========================


def savings_rate(age):
    if age <= 30:
        return 0.10
    elif age <= 45:
        return 0.20
    elif age <= 60:
        return 0.25
    else:
        return 0.30


def equity_return():
    return np.random.normal(0.065, 0.15)


def real_estate_return():
    base = np.random.normal(0.045, 0.08)
    leverage = 1.8
    return base * leverage


def annual_loan_payment(principal, rate, years):
    return principal * (rate * (1 + rate) ** years) / ((1 + rate) ** years - 1)


ANNUAL_LOAN_PAYMENT = annual_loan_payment(STUDENT_LOAN, LOAN_RATE, LOAN_TERM)

# =========================
# SINGLE LIFE SIMULATION
# =========================


def simulate_life(args):
    investment_type, education_debt = args

    wealth = INITIAL_WEALTH

    if education_debt:
        salary = EDUCATED_SALARY
        loan_balance = STUDENT_LOAN
    else:
        salary = BASE_SALARY
        loan_balance = 0

    for year in range(YEARS):
        age = START_AGE + year

        salary *= 1 + np.random.normal(0.03, 0.02)
        save = salary * savings_rate(age)

        if loan_balance > 0:
            payment = min(ANNUAL_LOAN_PAYMENT, loan_balance)
            loan_balance = loan_balance * (1 + LOAN_RATE) - payment
            save -= payment

        save = max(save, 0)

        if investment_type == "equity":
            r = equity_return()
        else:
            r = real_estate_return()

        wealth = (wealth + save) * (1 + r)
        wealth = max(wealth, 0)

    return wealth


# =========================
# PARALLEL MONTE CARLO
# =========================


def run_parallel(sim_args, label):
    cpu_count = mp.cpu_count() - 2
    print(f"Running {label} on {cpu_count} cores")

    with mp.Pool(cpu_count) as pool:
        results = pool.map(simulate_life, sim_args)

    return np.array(results)


# =========================
# MAIN
# =========================

if __name__ == "__main__":
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    scenarios = {
        "equity_debt": ("equity", True),
        "equity_no_debt": ("equity", False),
        "realestate_debt": ("real_estate", True),
        "realestate_no_debt": ("real_estate", False),
    }

    results = {}

    for name, params in scenarios.items():
        args = [params] * N_SIM
        results[name] = run_parallel(args, name)

    # =========================
    # SAVE HISTOGRAMS
    # =========================

    for name, data in results.items():
        plt.figure(figsize=(10, 6))
        plt.hist(data / 1e6, bins=80)
        plt.xlabel("Final Wealth ($ Millions)")
        plt.ylabel("Frequency")
        plt.title(name.replace("_", " ").title())

        filename = f"{OUTPUT_DIR}/{name}_hist_{timestamp}.png"
        plt.savefig(filename, dpi=150)
        plt.close()

    # =========================
    # COMPARISON PLOT
    # =========================

    plt.figure(figsize=(12, 7))
    for name, data in results.items():
        plt.hist(data / 1e6, bins=80, alpha=0.35, label=name)

    plt.xlabel("Final Wealth ($ Millions)")
    plt.ylabel("Frequency")
    plt.title("Monte Carlo Wealth Comparison")
    plt.legend()

    filename = f"{OUTPUT_DIR}/comparison_{timestamp}.png"
    plt.savefig(filename, dpi=150)
    plt.close()

    # =========================
    # SAVE SUMMARY STATS
    # =========================

    stats_file = f"{OUTPUT_DIR}/summary_{timestamp}.csv"
    with open(stats_file, "w") as f:
        f.write("scenario,median,p10,p90,mean\n")
        for name, data in results.items():
            f.write(
                f"{name},"
                f"{np.median(data):.2f},"
                f"{np.percentile(data, 10):.2f},"
                f"{np.percentile(data, 90):.2f},"
                f"{np.mean(data):.2f}\n"
            )

    print(f"\nAll outputs saved in: {OUTPUT_DIR}/")
