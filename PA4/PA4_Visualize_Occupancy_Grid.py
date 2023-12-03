# Author: Carter Kruse
# Date: May 10, 2023

# Import Relevant Libraries (Python Modules)
from numpy import genfromtxt
from matplotlib import pyplot

# Determine what the data is, according to the CSV file.
data = genfromtxt('data.csv', delimiter = ',')

# Create the figure, which displays the grid/map.
pyplot.figure(figsize = (100, 100))
pyplot.imshow(data, origin = 'lower')
pyplot.show()
