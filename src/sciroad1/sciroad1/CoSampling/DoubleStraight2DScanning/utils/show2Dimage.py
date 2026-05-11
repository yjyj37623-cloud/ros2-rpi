import pandas as pd
import matplotlib.pyplot as plt
import PIL


def show_image(file_path):
    df = pd.read_excel(file_path)
    image = []
    image_dict = {}
    maxX = 0
    maxY = 0
    for i in range(len(df)):
        point = df.iloc[i]
        image_dict[(int(point['posX']), int(point['posY']))] = point['value']
        maxX = max(maxX, point['posX'])
        maxY = max(maxY, point['posY'])

    for i in range(maxX + 1):
        row = []
        if i % 5 != 0:
            continue
        for j in range(maxY + 1):
            if j % 5 != 0:
                continue
            if (i, j) in image_dict:
                row.append(image_dict[(i, j)])
            else:
                print("Missing point at", i, j)
                row.append(0)
        image.append(row)

    plt.imshow(image)

    plt.show()


if __name__ == '__main__':
    show_image("data/2024-02-28-22-02-44_test.xlsx")