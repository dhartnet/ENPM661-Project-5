import numpy as np

map = np.zeros((600, 200))

i = 0
j = 0

# obs 1
x11 = 150
x12 = 175
y11 = 0
y12 = 100

# obs 2
x21 = 250
x22 = 275
y21 = 100
y22 = 200

# obs 3

x31 = 382
x32 = 458
y31 = 55
y32 = 105

case = False

while case == False:

  # obs 1
  if i >= x11 and i <= x12 and j >= y11 and j <= y12:
    map[i,j] = 1

  # obs 2
  elif i >= x21 and i <= x22 and j >= y21 and j <= y22:
    map[i,j] = 1

  # obs 3
  elif i >= x31 and i <= x32 and j >= y31 and j <= y32:
    map[i,j] = 1

  i = i + 1

  if i == 600 and j == 199:

    case = True

  if i == 600:
    i = 0
    j = j + 1

print(map[420,75])