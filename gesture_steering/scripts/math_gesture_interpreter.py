import cv2
import hand_tracking as ht

X = 0
Y = 1
Z = 2


class Interpreter:
    finger_points = [
        (ht.PINKY_PIP, ht.PINKY_TIP, ht.PINKY_MCP),
        (ht.RING_FINGER_PIP, ht.RING_FINGER_TIP, ht.RING_FINGER_MCP),
        (ht.MIDDLE_FINGER_PIP, ht.MIDDLE_FINGER_TIP, ht.MIDDLE_FINGER_MCP),
        (ht.INDEX_FINGER_PIP, ht.INDEX_FINGER_TIP, ht.INDEX_FINGER_MCP),
    ]

    def __init__(self):
        self.detected_gestures = []

    def get_fingers_open(self, landmarks):
        status = []

        # 4 fingers except of thumb
        for pip, tip, mcp in self.finger_points:
            if landmarks[tip][Y] < landmarks[pip][Y]:
                status.append(1)
            else:
                status.append(0)

        # thumb
        if landmarks[ht.THUMB_TIP][X] < landmarks[ht.THUMB_IP][X]:
            status.append(1)
        else:
            status.append(0)

        return status

    # separate function for each gesture

    def gesture_up(self, landmarks):
        # ring, pinky, index and middle fingers must be closed
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][X] > landmarks[pip][X]
                or landmarks[tip][X] < landmarks[mcp][X]
            ):
                return False

        # thumb must be pointing up
        if (
            landmarks[ht.THUMB_TIP][Y] > landmarks[ht.THUMB_MCP][Y]
            or landmarks[ht.THUMB_IP][Y] > landmarks[ht.INDEX_FINGER_MCP][Y]
        ):
            return False

        return True

    def gesture_down(self, landmarks):
        # pinky, ring, index and middel finger closed
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][X] < landmarks[pip][X]
                or landmarks[tip][X] > landmarks[mcp][X]
            ):
                return False

        # thumb must be pointing down
        if (
            landmarks[ht.THUMB_TIP][Y] < landmarks[ht.THUMB_MCP][Y]
            or landmarks[ht.THUMB_IP][Y] < landmarks[ht.INDEX_FINGER_MCP][Y]
        ):
            return False

        return True

    def gesture_left(self, landmarks):
        # ring, pinky, index and middle finger must be closed
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][Y] > landmarks[pip][Y]
                or landmarks[tip][Y] < landmarks[mcp][Y]
            ):
                return False

        # thumb must be pointing left
        if (
            landmarks[ht.THUMB_TIP][X] < landmarks[ht.THUMB_MCP][X]
            or landmarks[ht.THUMB_IP][X] < landmarks[ht.INDEX_FINGER_MCP][X]
        ):
            return False

        return True

    def gesture_right(self, landmarks):
        # ring, pinky and middle finger must be closed
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][Y] < landmarks[pip][Y]
                or landmarks[tip][Y] > landmarks[mcp][Y]
            ):
                return False

        # thumb must be pointing right
        if (
            landmarks[ht.THUMB_TIP][X] > landmarks[ht.THUMB_MCP][X]
            or landmarks[ht.THUMB_IP][X] > landmarks[ht.INDEX_FINGER_MCP][X]
        ):
            return False

        return True

    def flip_back(self, landmarks):
        # all fingers pointing up with palm towards camera
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][Y] > landmarks[pip][Y]
                or landmarks[mcp][Y] > landmarks[ht.WRIST][Y]
            ):
                return False

        # palm twoards camera
        if landmarks[ht.THUMB_TIP][X] < landmarks[ht.WRIST][X]:
            return False

        # thumb pointing up
        if landmarks[ht.THUMB_TIP][Y] > landmarks[ht.THUMB_IP][Y]:
            return False

        return True

    def flip_front(self, landmarks):
        # all fingers pointig down with back of the hand towards camera
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][Y] < landmarks[pip][Y]
                or landmarks[mcp][Y] < landmarks[ht.WRIST][Y]
            ):
                return False

        # back fo the hand towards camera
        if landmarks[ht.THUMB_TIP][X] < landmarks[ht.WRIST][X]:
            return False

        # thumb pointing down
        if landmarks[ht.THUMB_TIP][Y] < landmarks[ht.THUMB_IP][Y]:
            return False

        return True

    def flip_left(self, landmarks):
        # all fingers pointing left with back of the hand towards camera
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][X] < landmarks[pip][X]
                or landmarks[mcp][X] < landmarks[ht.WRIST][X]
            ):
                return False

        # back of the hand towards camera
        if landmarks[ht.THUMB_TIP][Y] > landmarks[ht.WRIST][Y]:
            return False

        # thumb pointing left
        if landmarks[ht.THUMB_TIP][X] < landmarks[ht.THUMB_IP][X]:
            return False

        return True

    def flip_right(self, landmarks):
        # all fingers pointing right with palm towards camera
        for pip, tip, mcp in self.finger_points:
            if (
                landmarks[tip][X] > landmarks[pip][X]
                or landmarks[mcp][X] > landmarks[ht.WRIST][X]
            ):
                return False

        # palm towards the camera
        if landmarks[ht.THUMB_TIP][Y] > landmarks[ht.WRIST][Y]:
            return False

        # thumb pointing right
        if landmarks[ht.THUMB_TIP][X] > landmarks[ht.THUMB_IP][X]:
            return False

        return True


def get_binary_value(fingers):
    power = 1
    value = 0
    for finger in fingers:
        value += finger * power
        power *= 2

    return value


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    tracker = ht.HandTracker(640, 480, draw=True)
    interpreter = Interpreter()

    while True:
        succes, img = cap.read()

        hand_landmarks = tracker.get_landmarks(img)

        if hand_landmarks:
            status = interpreter.get_fingers_open(hand_landmarks)

            if interpreter.gesture_up(hand_landmarks):
                print("up")
            elif interpreter.gesture_down(hand_landmarks):
                print("down")
            elif interpreter.gesture_left(hand_landmarks):
                print("left")
            elif interpreter.gesture_right(hand_landmarks):
                print("right")
            elif interpreter.flip_back(hand_landmarks):
                print("flip_back")
            elif interpreter.flip_front(hand_landmarks):
                print("flip_front")
            elif interpreter.flip_left(hand_landmarks):
                print("flip_left")
            elif interpreter.flip_right(hand_landmarks):
                print("flip_right")
        cv2.imshow("test", img)
        cv2.waitKey(1)
