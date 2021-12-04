import cv2
import mediapipe as mp
import SharedData
from Yamanami.myutil import *

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


def getTrackingFlag(hand_landmark):
    thumb_tip = hand_landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_finger_tip = hand_landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]

    if distance2D(thumb_tip.x, thumb_tip.y, index_finger_tip.x, index_finger_tip.y) < 0.03:
        return True

    return False


def main(sdict: dict = SharedData.SharedDict):


    cap = cv2.VideoCapture(1)
    with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
        try:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style(),
                        )
                        pass

                    wrist_pos = results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.WRIST]
                    sdict["tracking_flag"] = getTrackingFlag(results.multi_hand_landmarks[0].landmark)
                    sdict["hand_isDetected"] = True
                    sdict["hand_pos_x"] = wrist_pos.x
                    sdict["hand_pos_y"] = wrist_pos.y

                    pass
                else:
                    sdict["hand_isDetected"] = False

                cv2.imshow("MediaPipe Hands", image)
                if cv2.waitKey(5) & 0xFF == 27:
                    print("Break")
                    sdict["terminate_flag"] = True
                    break
                pass
        except KeyboardInterrupt:
            print("Break Process")

    cap.release()
    return


if __name__ == "__main__":
    main()
    pass


