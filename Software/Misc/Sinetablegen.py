from matplotlib import pyplot as plt
import numpy as np


t = np.linspace(start=0, stop=2*np.pi*7, num=3600)

sintab = np.sin(t)
#sintab += 1
sintab = sintab * 1024
sintab = sintab.astype('int')

print("max:", max(sintab))
print("min:", min(sintab))
print("n:", len(sintab))


print("uint16_t sintab* = {")
count = 0
for x in sintab:
    print(str(x) + ", ", end='')
    count += 1
    if x < 1000:
        print(" ", end="")
        if x < 100:
            print(" ", end="")
            if x < 10:
                print(" ", end="")
    if count > 16:
        print("")
        count = 0
    

plt.plot(t, sintab)
plt.grid(visible=True, which='both')
plt.show()
