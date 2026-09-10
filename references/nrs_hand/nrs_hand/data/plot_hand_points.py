import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D projection 활성화

def main():
    # hand_pointed_position.txt 가 같은 디렉토리에 있어야 합니다.
    data = np.loadtxt('hand_pointed_position.txt')
    # shape: (N, 3)
    x, y, z = data[:,0], data[:,1], data[:,2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, linestyle='-', marker='o', markersize=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Hand Point Trajectory')

    plt.show()

if __name__ == '__main__':
    main()
