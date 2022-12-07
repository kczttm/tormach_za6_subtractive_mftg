import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import argparse
import sys
import platform
import threading
import numpy as np
import time
sys.path.append('toolbox')
import rpi_ati_net_ft

class ATIDriver(object):
    def __init__(self, host):

        self._lock = threading.RLock()
        self._recv_lock=threading.RLock()
        self._streaming = False
        
        self.host = host
        self.ati_obj = rpi_ati_net_ft.NET_FT(host)

        self.ati_obj.set_tare_from_ft()
        print(self.ati_obj.read_ft_http())
        print(self.ati_obj.try_read_ft_http())
        
        self.ati_obj.start_streaming()

        # publish rate
        self.rate = 1e-3

        ## for testing ##
        self.last_ft = np.zeros(6)
    
    def srv_start_streaming(self):
        
        with self._lock:
            if (self._streaming):
                raise Exception("Already Streaming")
            
            # start data streaming
            self._streaming = True
            self.t = threading.Thread(target=self.stream_loop)
            self.t.start()
    def srv_stop_streaming(self):
        self._streaming = False
        self.t.join()

    def stream_loop(self):
        
        while self._streaming:
            st = time.time()
            if(not self._streaming): return
            res, ft, status = self.ati_obj.try_read_ft_streaming(.1)
            self.send_sensor_val(ft)
            elapse = time.time()-st

            if self.rate > elapse:
                time.sleep(self.rate - elapse)

    def send_sensor_val(self, ft):
        
        # create the new namedarray
        arraytype = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Wrench')
        # print(arraytype)
        msg_arr = np.zeros((1,),dtype=arraytype)
        # print(msg_arr)
        # pack the data to the structure to send to the client
        msg_arr[0]['torque']['x'] = ft[0]
        msg_arr[0]['torque']['y'] = ft[1]
        msg_arr[0]['torque']['z'] = ft[2]
        msg_arr[0]['force']['x'] = ft[3]
        msg_arr[0]['force']['y'] = ft[4]
        msg_arr[0]['force']['z'] = ft[5]
        # print(msg_arr)

        self.wrench_sensor_value.OutValue=msg_arr

def main():
    parser = argparse.ArgumentParser(description="ATI force torque sensor driver service for Robot Raconteur")
    parser.add_argument("--sensor-ip", type=str, default="192.168.50.65", help="the ip address of the ati sensor")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args,_ = parser.parse_known_args()

    # not yet know what this do
    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv
    RRC.RegisterStdRobDefServiceTypes(RRN)

    ati_obj = ATIDriver(args.sensor_ip)

    with RR.ServerNodeSetup("com.robotraconteur.sensor.wrenchsensor",59823,argv=rr_args):
        
        service_ctx = RRN.RegisterService("ati_sensor","com.robotraconteur.sensor.WrenchSensor",ati_obj)
        ati_obj.srv_start_streaming()

        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        
        else:    
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")

        ati_obj.srv_stop_streaming()


if __name__ == "__main__":
    main()