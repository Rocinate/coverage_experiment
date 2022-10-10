import enum
import numpy as np
from tables import EnumAtom

if __name__ == '__main__':
    data = np.loadtxt("dataLT.csv", delimiter=",")
    data *= 1e7
    result = []

    for indexX, posX in enumerate(np.linspace(-10, 10, 30)):
        for indexY, posY in enumerate(np.linspace(-10, 10, 30)):
            result.append(",".join([str(round(posX, 2)), str(round(posY, 2)), str(data[indexX*33, indexY*33])]) + "\n")

    with open("field_strength.csv", 'w') as f:
        f.writelines(result)

