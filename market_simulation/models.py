
'''
	Main classes for Market Simulation.

	Note:- producers are defined directly by generating items at a fixed or variable rate
'''

class Item:
	"""
		A commodity which can be sold and purchased in the market.
		Demand of an item specifies how much consumers want it. It is usually set randomly
	"""

	def __init__(self,id,price = 1,demand = 1):
		self.id = id
		self.price = round(price,2)
		self.demand = round(demand,3)

	def __str__(self):
		return "Item(%d): [price = %2f, demand = %3f]" % (self.id,self.price,self.demand)



class Seller:
	"""
		Buys items from producer for selling in market
	"""

	def __init__(self,items = [],sellingprices = [],stock = []):

		if len(items) != len(sellingprices) != len():
			raise

		self.items = items
		self.sellingprices = sellingprices
		self.stock = stock


class Consumer:
	"""
		Purchases items from sellers in the market
	""" 

	def __init(self,purchasingpower = 1):
		self.purchasingpower = purchasingpower
