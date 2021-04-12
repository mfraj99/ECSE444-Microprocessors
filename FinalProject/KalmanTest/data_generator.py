#data.txt is meant to reflect noise of the accelerometer difference array of the idle board

import random

file = open("data.txt", "w")
n = 200
for i in range(n):
    number = random.randint(-30,30)
    chance = random.randint(0,10)
    #if chance <= 8:
    #    number = random.randint(-2,2)
    file.write(str(number)+",")
