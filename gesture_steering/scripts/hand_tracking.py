import cv2

import mediapipe as mp
import time


# mpHands.HandLandmark
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_FINGER_MCP = 5
INDEX_FINGER_PIP = 6
INDEX_FINGER_DIP = 7
INDEX_FINGER_TIP = 8
MIDDLE_FINGER_MCP = 9
MIDDLE_FINGER_PIP = 10
MIDDLE_FINGER_DIP = 11
MIDDLE_FINGER_TIP = 12
RING_FINGER_MCP = 13
RING_FINGER_PIP = 14
RING_FINGER_DIP = 15
RING_FINGER_TIP = 16
PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20


class HandTracker:
    def __init__(
        self,
        width,
        height,
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        draw=False,
    ):
        self.mp_solutions_hands = mp.solutions.hands
        self.hands = self.mp_solutions_hands.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

        self.drawing_utils = mp.solutions.drawing_utils
        self.drawing_styles = mp.solutions.drawing_styles
        self.width = width
        self.height = height
        self.draw = draw

    def get_normalized_landmarks(self, img, hand_num=0):
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_img)

        if (
            results.multi_hand_landmarks
            and len(results.multi_hand_landmarks) > hand_num
        ):
            return results.multi_hand_landmarks[hand_num]
        else:
            return None

    def get_landmarks(self, img, hand_num=0, normalized=False):
        normalized_landmarks = self.get_normalized_landmarks(img, hand_num)

        landmark_pos = None

        if normalized_landmarks:
            landmark_pos = []

            if self.draw:
                self.drawing_utils.draw_landmarks(
                    img,
                    normalized_landmarks,
                    self.mp_solutions_hands.HAND_CONNECTIONS,
                    connection_drawing_spec=self.drawing_utils.DrawingSpec(
                        color=(0, 255, 0)
                    ),
                )

            for id, landmark in enumerate(normalized_landmarks.landmark):
                x_pos, y_pos = int(self.width * landmark.x), int(
                    self.height * landmark.y
                )
                landmark_pos.append((x_pos, y_pos))

        return landmark_pos if not normalized else normalized_landmarks


if __name__ == "__main__":
    tracker = HandTracker(640, 480, draw=True, max_num_hands=3)

    cap = cv2.VideoCapture(0)

    pTime = 0
    cTime = 0
    # h,w,c
    while True:
        success, img = cap.read()

        for i in range(3):
            p = tracker.get_landmarks(img, hand_num=i)

        # , mpDraw.GREEN_COLOR)#mp_drawing_styles.get_default_hand_landmarks_style(),
        # mp_drawing_styles.get_default_hand_connections_style())

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(
           img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_COMPLEX, 3, (255, 0, 255), 3
        )

        cv2.imshow("image", img)
        cv2.waitKey(1)
