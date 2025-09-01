import threading
import time

# ---- 전역/공유 자원 ----
search_thread = None
search_stop = threading.Event()
serial_lock = threading.Lock()

def safe_send_servo_angle(pan=None, tilt=None):
    """시리얼 충돌 방지를 위한 락 래퍼"""
    with serial_lock:
        p = "0" if pan is None else str(int(pan))
        t = "0" if tilt is None else str(int(tilt))
        nano.write(f"P:{p},T:{t}\n".encode())
        nano.flush()

class SearchThread(threading.Thread):
    """
    얼굴이 안 보일 때 수행되는 '탐색 패턴'을 담당.
    Event로 중단 신호를 받고 스스로 종료한다.
    """
    def __init__(self, stop_event, current_pan_getter, current_tilt_getter, step_delay=0.8):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.get_pan = current_pan_getter
        self.get_tilt = current_tilt_getter
        self.step_delay = step_delay  # 각 스텝 간 대기(초)

    def run(self):
        try:
            # 간단한 스윕 패턴: 90 -> 0 -> 180 -> 90 반복
            sequence = [90, 0, 180, 90]
            idx = 0
            while not self.stop_event.is_set():
                pan_target = sequence[idx % len(sequence)]
                idx += 1

                # 현재 틸트는 그대로 유지
                tilt_now = self.get_tilt()
                safe_send_servo_angle(pan_target, tilt_now)

                # step_delay 동안 중단 신호 폴링
                end_time = time.monotonic() + self.step_delay
                while time.monotonic() < end_time:
                    if self.stop_event.is_set():
                        break
                    time.sleep(0.02)  # 폴링 주기
        except Exception as e:
            if Debug:
                print(f"[SearchThread] error: {e}")

# ---- 기존 변수들 가정 ----
currentPan = 90
currentTilt = 90

def get_current_pan():
    return currentPan

def get_current_tilt():
    return currentTilt

# ---- 메인 루프 안의 제어 흐름 예시 ----
# faceFound가 True인 순간: 탐색 스레드가 있으면 중단
def stop_search_thread_if_running():
    global search_thread, search_stop
    if search_thread is not None and search_thread.is_alive():
        search_stop.set()
        # 너무 오래 기다리지 않도록 타임아웃 join
        search_thread.join(timeout=1.0)
    search_thread = None
    search_stop = threading.Event()  # 다음 시작을 위해 이벤트 새로 만듦

# 얼굴을 못 찾은 상태가 일정 시간 지속되면 서치 시작
def ensure_search_thread_running():
    global search_thread, search_stop
    if search_thread is None or not search_thread.is_alive():
        search_stop.clear()
        search_thread = SearchThread(
            stop_event=search_stop,
            current_pan_getter=get_current_pan,
            current_tilt_getter=get_current_tilt,
            step_delay=0.8,  # 탐색 속도
        )
        search_thread.start()
        if Debug:
            print("[Search] started")

# ---- 메인 처리 곳에서의 사용 예 ----
# (얼굴 검출 분기와 연동)
if faceFound:
    # 추적 제어(미세 보정) 수행…
    stop_search_thread_if_running()
else:
    # 일정 시간/조건이 되면 탐색 시작
    ensure_search_thread_running()

# ---- 종료 처리 (KeyboardInterrupt/ finally 등) ----
stop_search_thread_if_running()