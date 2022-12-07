import time
import threading
import numpy as np
class Joint_Realtime():
    def __init__(self,vel_ctrl):
        self.stop_event = threading.Event()
        self.vel_ctrl = vel_ctrl
        self._lock = threading.RLock()
        self.joint_position = []
        self.clock = []

    def start(self):
        self._checker = threading.Thread(target=self.get_states, args = (self.stop_event,))
        self._checker.daemon = True
        self._checker.start()

    def stop(self):
        self.stop_event.set()
        self._checker.join()

    def get_states(self,event):
        start_time = time.time()
        while not self.stop_event.is_set():
            while self._lock:
                time.sleep(0.01) ## 100 Hz
                self.joint_position.append(self.vel_ctrl.joint_position())
                self.clock.append(time.time())
                # check for stop
                if event.is_set():
                    self.joint_position = np.array(self.joint_position)
                    self.clock = np.array(self.clock)-start_time
                    return

if __name__ == "__main__":
    rt_state = Joint_Realtime(vel_ctrl)
    rt_state.start()
    time.sleep(2)
    rt_state.stop()
    print(rt_state.clock,rt_state.joint_position*180/np.pi)