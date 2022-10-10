import enum
import numpy as np
from tables import EnumAtom

if __name__ == '__main__':
    data = np.loadtxt("dataLT.csv", delimiter=",")
    data *= 1e7
    result = []

    for indexX, posX in enumerate(np.linspace(-3.2, -3.2 + 15, 100)):
        for indexY, posY in enumerate(np.linspace(-3.2, -3.2 + 15, 100)):
            result.append(",".join([str(round(posX, 2)), str(round(posY, 2)), str(data[indexX*10, indexY*10])]) + "\n")

    with open("field_strength.csv", 'w') as f:
        f.writelines(result)