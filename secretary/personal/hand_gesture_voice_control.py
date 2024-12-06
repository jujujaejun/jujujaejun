import cv2
import mediapipe as mp
import speech_recognition as sr
import os
import webbrowser
import pyautogui
import threading

# 손 인식을 위한 MediaPipe 초기화
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# 음성 인식 초기화
recognizer = sr.Recognizer()

# 비동기로 음성 인식을 수행할 함수
def recognize_speech_async():
    global listening, speech_active
    with sr.Microphone() as source:
        print("음성 인식을 시작합니다. 말을 해주세요.")
        audio = recognizer.listen(source)

        try:
            command = recognizer.recognize_google(audio, language='ko-KR')
            print(f"인식된 명령어: {command}")
            execute_command(command)
        except sr.UnknownValueError:
            print("음성을 인식하지 못했습니다.")
        except sr.RequestError:
            print("Google Speech Recognition 서비스에 접근할 수 없습니다.")

        # 음성 인식이 끝나면 listening 상태 종료
        listening = False
        speech_active = False

# 명령어 실행 함수
def execute_command(command):
    if "메모장" in command:
        print("메모장을 실행합니다.")
        os.system("notepad")
    elif "계산기" in command:
        print("계산기를 실행합니다.")
        os.system("calc")
    elif "검색" in command:
        search_query = command.replace("검색", "").strip()
        if search_query:
            print(f"'{search_query}'에 대해 검색합니다.")
            webbrowser.open(f"https://www.google.com/search?q={search_query}")
        else:
            print("검색할 내용을 말해주세요.")
    else:
        print(f"'{command}' 명령어를 이해하지 못했습니다.")

# 각 손가락이 펴졌는지 확인하는 함수
def is_finger_open(hand_landmarks, finger_tip_index, finger_pip_index):
    return hand_landmarks.landmark[finger_tip_index].y < hand_landmarks.landmark[finger_pip_index].y

# 펴진 손가락의 개수를 계산하는 함수
def count_fingers(hand_landmarks):
    # 각 손가락의 랜드마크 인덱스
    fingers = [(8, 6),  # 검지 (index finger)
               (12, 10),  # 중지 (middle finger)
               (16, 14),  # 약지 (ring finger)
               (20, 18)]  # 새끼 (pinky finger)

    count = 0
    for tip, pip in fingers:
        if is_finger_open(hand_landmarks, tip, pip):
            count += 1
    return count

# 주먹 상태 확인하는 함수 (모든 손가락이 접혀있는지 확인)
def is_fist(hand_landmarks):
    # 검지, 중지, 약지, 새끼가 모두 접혀있는지 확인
    return count_fingers(hand_landmarks) == 0

# 웹캠으로 손 동작을 인식하는 함수
def main():
    global listening, speech_active
    cap = cv2.VideoCapture(0)
    listening = False
    speech_active = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            continue

        # 이미지 처리
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # 손이 인식되면 처리
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # 펴진 손가락 개수 세기
                num_fingers = count_fingers(hand_landmarks)

                # 손가락 하나가 펴졌을 때 "Enter" 입력
                if num_fingers == 1:
                    print("손가락 하나가 펴졌습니다. Enter 키를 입력합니다.")
                    pyautogui.press('enter')

                # 손가락 두 개가 펴졌을 때 "ㅋ" 입력
                elif num_fingers == 2:
                    print("손가락 두 개가 펴졌습니다. 'ㅋ'를 입력합니다.")
                    pyautogui.typewrite('lol', interval=0.05)

                # 손가락이 모두 펴졌을 때 음성 인식 시작 (비동기로 처리)
                elif num_fingers >= 3 and not speech_active:
                    print("손이 펴졌습니다. 음성 인식을 비동기로 시작합니다.")
                    listening = True
                    speech_active = True
                    threading.Thread(target=recognize_speech_async).start()

                # 주먹을 쥐면 음성 인식 종료
                if is_fist(hand_landmarks) and listening:
                    print("주먹이 감지되었습니다. 음성 인식을 종료합니다.")
                    listening = False
                    speech_active = False

        # 화면에 출력
        cv2.imshow('Hand Detection', frame)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    