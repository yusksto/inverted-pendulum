import time
from time import sleep
import numpy as np

moter_sleep_min = 0.001
moter_sleep_max = 0.05

def MOTER_SLEEP(dt_moter):
	n = int(dt_moter / moter_sleep_min)
	for i in range(n):
		time.sleep(moter_sleep_min)

if __name__ == '__main__':
	try:
		f = open('data.dat', 'w')
		t_0 = moter_sleep_min
		t_1 = 5
		n = 0
		t = t_0
		while True:
			n += 1
			t = moter_sleep_min * np.exp(n)
			t_start = time.time()
			MOTER_SLEEP(t)
			print(str(t) + " " + str(time.time() - t_start))
			f.write(str(t) + " " + str(time.time() - t_start))
			if t > t_1:
				break
		
	except KeyboardInterrupt:
		print("\nCtl+C")
	except Exception as e:
		print(str(e))
	finally:
		print("\nexit program")