# Monte Carlo Simulations

## Wealth Monte Carlo

Simulates lifetime wealth outcomes from age 22 to 65 across four scenarios:
- Equity vs. real estate returns
- With vs. without student loan debt

### Requirements

- Python 3
- numpy
- matplotlib

Install dependencies (if needed):

```bash
pip install numpy matplotlib
```

### Run

```bash
python wealth_monte_carlo.py
```

### Outputs

The script writes artifacts into `outputs/` with a timestamped filename:
- Histograms per scenario
- A combined comparison histogram
- Summary stats CSV (median, p10, p90, mean)

### Notes

- Adjust `N_SIM` at the top of `wealth_monte_carlo.py` to change the number of simulations.
- Uses multiprocessing and reserves two CPU cores by default.
