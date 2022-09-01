#!/usr/bin/env python3
import argparse
import sys, os
from pathlib import Path
import pickle
from turtle import st

import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

X = 0
Y = 1
Z = 2


class DataGenerator:
    def __init__(self):
        pass

    def load_data(self, file_path):
        with open(file_path, "rb") as handle:
            self.init_data = pickle.load(handle)
        self.init_data = np.array(self.init_data)
        self.normalize_data()

    def generate_one_sample(self, start_id=0, end_id=-1, steps=50):
        start_hand = self.init_data[start_id]
        end_hand = self.init_data[end_id]

        num = len(self.init_data) if end_id == -1 else end_id - start_id

        random_id_1, random_id_2 = np.random.choice(num, size=2, replace=False)

        if random_id_2 > random_id_1:
            random_id_1, random_id_2 = random_id_2, random_id_1

        random_hand_one = self.init_data[random_id_1]
        random_hand_two = self.init_data[random_id_2]

        step = 1.0 / steps
        arange = np.arange(0.0, 1.0 + step, step)

        generated_move = []

        cum_distortion_step = np.random.randint(0, 100, 3) / 200000.0
        cum_distortion = 0

        sign = np.random.randint(0, 2, 3) % 2
        sign = np.where(sign == 0, -1, sign)

        cum_distortion_step *= sign

        for t in arange:
            distortion = np.random.randint(0, 100, 3) / 10000.0
            sign = np.random.randint(0, 2, 3) % 2
            sign = np.where(sign == 0, -1, sign)
            distortion *= sign

            bezier_hand = (
                bezier_curve(start_hand, random_hand_one, random_hand_two, end_hand, t)
                + distortion
                + cum_distortion
            )
            generated_move.append(bezier_hand.tolist())
            cum_distortion += cum_distortion_step

        return generated_move

    def generate_samples(self, samples_num=300, start_id=0, end_id=-1, steps=50):
        gen_data = []
        for i in range(samples_num):
            move = self.generate_one_sample(
                start_id=start_id, end_id=end_id, steps=steps
            )
            gen_data.append(move)
        return gen_data

    def normalize_data(self):
        landmark = self.init_data[0]

        min_x = 2.0
        min_y = 2.0
        min_z = 2.0

        for point in landmark:
            if point[X] < min_x:
                min_x = point[X]
            if point[Y] < min_y:
                min_y = point[Y]
            if point[Z] < min_z:
                min_z = point[Z]

        offset = [min_x, min_y, min_z]
        self.init_data += offset

    @staticmethod
    def visualize_hand(hand, ax, point_color="red", plot_color="#18f500"):
        finger1 = hand[:5]
        finger1 += finger1[::-1]
        finger2 = hand[5:9]
        finger2 += finger2[::-1]
        finger3 = hand[9:13]
        finger3 += finger3[::-1]
        finger4 = hand[13:17]
        finger4 += finger4[::-1]
        finger5 = hand[17:]
        finger5 += finger5[::-1]
        visualization_points = (
            finger1 + finger2 + finger3 + finger4 + finger5 + [hand[0]]
        )

        ax.set_xlabel("X")

        ax.set_ylabel("Y")

        ax.set_zlabel("Z")

        for point in hand:
            ax.scatter(*point, c=[point_color])

        visualization_points = np.array(visualization_points)
        ax.plot(
            visualization_points[:, 0],
            visualization_points[:, 1],
            visualization_points[:, 2],
            color=plot_color,
        )


def bezier_curve(A, B, C, D, t):
    coef_one = 1 - t
    coef_two = coef_one**2
    coef_three = coef_one**3
    t2 = t**2
    t3 = t**3

    first_p = coef_three
    second_p = 3.0 * t * coef_two
    third_p = 3.0 * t2 * coef_one
    fourth_p = t3

    point = A * first_p + B * second_p + C * third_p + D * fourth_p
    return point


def add_arguments(parser: argparse.ArgumentParser):
    parser.add_argument(
        "-T",
        "--train",
        nargs="?",
        default=500,
        type=int,
        required=False,
        help="Number of train set samples to generate",
        metavar="N",
        dest="train_samples",
    )

    parser.add_argument(
        "-t",
        "--test",
        nargs="?",
        default=100,
        type=int,
        required=False,
        help="Number of test set samples to generate",
        metavar="N",
        dest="test_samples",
    )

    parser.add_argument(
        "-v",
        "--valid",
        nargs="?",
        default=250,
        type=int,
        required=False,
        help="Number of validation set samples to generate",
        metavar="N",
        dest="valid_samples",
    )

    parser.add_argument(
        "-p",
        "--paths",
        nargs="+",
        default="dataset.pickle",
        type=str,
        required=True,
        help="Paths to files with recorded data",
        metavar="[paths]",
        dest="paths",
    )


def make_dataset(valid, test, train, paths, steps=50):
    try:
        Path("movement_dataset").mkdir(parents=True)
    except FileExistsError as e:
        print(
            "'movement_dataset' directory exist in the working directory already. Can't process images."
        )
        exit()
    Path("movement_dataset/valid").mkdir(parents=True, exist_ok=True)
    Path("movement_dataset/train").mkdir(parents=True, exist_ok=True)
    Path("movement_dataset/test").mkdir(parents=True, exist_ok=True)

    data_gen = DataGenerator()

    out_names = ["valid", "test", "train"]

    for path in paths:
        label = path.split("/")[-2]
        data_gen.load_data(path)
        for partition, num in zip(out_names, [valid, test, train]):
            generated_data = data_gen.generate_samples(num, steps=steps)

            with open(
                os.path.join("movement_dataset", partition, label + ".pickle"), "wb+"
            ) as handle:
                pickle.dump(generated_data, handle, protocol=pickle.DEFAULT_PROTOCOL)

    for partition, num in zip(out_names, [valid, test, train]):
        data = []
        for path in args.paths:
            label = path.split("/")[-2]
            data_gen.load_data(path)

            data += data_gen.generate_samples(num // len(paths), 0, 10, steps=steps)

        with open(
            os.path.join("movement_dataset", partition, "nothing.pickle"), "wb+"
        ) as handle:
            pickle.dump(data, handle, protocol=pickle.DEFAULT_PROTOCOL)


def get_color(start_color, end_color, value):
    red = start_color[0] + (end_color[0] - start_color[0]) * value
    green = start_color[1] + (end_color[1] - start_color[1]) * value
    blue = start_color[2] + (end_color[2] - start_color[2]) * value

    return (red, green, blue)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate data based by gathered manually data"
    )

    add_arguments(parser)
    args = parser.parse_args(sys.argv[1:])
    valid, test, train = args.valid_samples, args.test_samples, args.train_samples

    # make_dataset(valid, test, train, args.paths, 25)

    data_gen = DataGenerator()
    data_gen.load_data("/home/olo/Desktop/movement_recorded/front/1.pickle")

    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection="3d")
    s = data_gen.generate_one_sample(0, steps=25)

    i = 0
    step = 1.0 / float(len(s))
    for hand in s:
        color = get_color((0.94901961, 0.06666667, 0.68627451), (0.43137255, 0.00392157, 0.30196078), i)
        data_gen.visualize_hand(hand, ax, point_color=color, plot_color=color)
        i += step

    i = 0
    step = 1.0 / float(len(data_gen.init_data.tolist()))
    print(len(data_gen.init_data.tolist()))
    for hand in data_gen.init_data.tolist():
        color = get_color((0.10, 0.75, 0.99), (0.00392157, 0.31764706, 0.43137255), i)
        data_gen.visualize_hand(hand, ax, point_color=color, plot_color=color)
        i += step

    plt.show()
