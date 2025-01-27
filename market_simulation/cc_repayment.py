import numpy as np
import matplotlib.pyplot as plt

# Parameters
balance = 5000  # Initial balance in dollars
monthly_interest_rate = 0.02  # 2% monthly interest
monthly_payment = 250  # Minimum payment in dollars

# Function to calculate balance after n months
def calculate_balance(b, r, p, n):
    return b * (1 + r)**n - (p / r) * ((1 + r)**n - 1)

# Function to calculate months required to pay off debt
def months_to_pay_off(b, r, p):
    if p <= b * r:
        return float('inf')  # Infinite time if payment is too low
    return np.log(p / (p - b * r)) / np.log(1 + r)

# Visualization
months = np.arange(1, 61)  # 5 years
balances = [calculate_balance(balance, monthly_interest_rate, monthly_payment, n) for n in months]

plt.figure(figsize=(10, 6))
plt.plot(months, balances, label=f"Monthly Payment = ${monthly_payment}", color='blue')
plt.axhline(0, color='black', linestyle='--', linewidth=0.8)  # Zero balance line
plt.title("Credit Card Balance Over Time")
plt.xlabel("Months")
plt.ylabel("Balance ($)")
plt.legend()
plt.grid(True)
plt.show()

# Time to pay off the debt
time_to_pay = months_to_pay_off(balance, monthly_interest_rate, monthly_payment)
print(f"Time to pay off the debt: {time_to_pay:.2f} months")
