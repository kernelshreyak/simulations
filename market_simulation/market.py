'''

	Main simulation program which simulates a market with items, sellers and consumers
	Items come and go out of stock, sellers gain or lose money and demand.supply of items
	
	Each of them can run in separate threads if intended
'''

from models import *
import random
import matplotlib.pyplot as plt
import numpy as np




# -------------------------------Utility Functions------------------------------------
def get_random(min = 1,max = 1000):
	return random.random()*max + min

def calculate_initial_demand(price = 0.1):
	global Ck

	demand = Ck/price*10
	return demand

# ------------------------------------------------------------------------------------


def produce_items(count = 10,rate = 1,maxtime = 10):
	items = []
	for i in range(count):
		price = get_random(1,1000)
		items.append(Item(i + 1,price,calculate_initial_demand(price)))


	return items



# initial parameters
sellercount = 100
consumercount = 20
Ck = consumercount/sellercount


# Create some items for the market
items = produce_items(5) #main items stock
# sort items by price
items.sort(key = lambda x:x.demand,reverse = True)

c = np.array([])
d = np.array([])
for item in items:
	print(item)
	c = np.append(c,item.price)
	d = np.append(d,item.demand)

# print(c)
# print(d)

# # Plot initial cost and demand
plt.plot(c,d)
plt.show()



# Add sellers to buy items and put into their stock




