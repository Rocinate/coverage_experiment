# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Pool

def sqr(x):

    return x*x

if __name__ == '__main__':

    po = Pool(processes = 10)

    resultslist = []

    for i in range(1, 10):

        arg = [i]

        result = po.apply_async(sqr, arg)

        resultslist.append(result)

    for res in resultslist:

        print(res.get())
