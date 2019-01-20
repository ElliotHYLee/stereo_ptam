from threading import Thread
import time

def func1():
    for i in range(0, 5):
        print("func1: %d" %(i))
        time.sleep(1)

def func2():
    print("func2")


if __name__ == '__main__':
    t = Thread(target=func1)
    t.start()
    func2()
    t.join()