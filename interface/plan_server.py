import socket
import pickle
import threading
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from pub_recv import PathMonitor  # Assuming PathMonitor class is in path_monitor.py file

class Server:

    def __init__(self, host='localhost', port=12346):
        self.path_monitor = PathMonitor()
        
        # Create a socket object
        self.s = socket.socket()
        self.s.settimeout(300)
        # Bind to the port
        self.s.bind((host, port))

        # Now wait for client connection
        self.s.listen(5)
        self.stop_event = threading.Event()

    def handle_client(self, c):
        # self.path_monitor.pub_goal(0, 0, 0)
        while not self.stop_event.is_set():
            try:
                data = c.recv(1024)
                if not data:
                    break

                # unpickle the received data
                received_data = pickle.loads(data)

                init = received_data.get('init')

                goal = received_data.get('goal')


                print(init, goal)
                # Publish init and goal positions
                
                
                self.path_monitor.path_ready = False
                self.path_monitor.pub_init_pos(*init)
                rospy.sleep(0.2)
                self.path_monitor.pub_goal(*goal)
                count = 10
                while (not self.path_monitor.path_ready) and count > 0:
                    rospy.sleep(0.1)
                    count -= 1

                # Send path back to the client
                path = self.path_monitor.get_path()
                print(len(path))
                c.send(pickle.dumps(path))

            except Exception as e:
                print(e)
                exit()
                break

        # Close the connection
        c.close()

    def start(self):
        while not self.stop_event.is_set():
            print('Waiting for a connection')

            # Establish connection with client
            c, addr = self.s.accept()
            c.settimeout(300)
            print('Got a connection from', addr)
            self.s.close()
            threading.Thread(target=self.handle_client, args=(c,)).start()

    def stop(self):
        self.stop_event.set()
        self.s.close()

if __name__ == '__main__':
    server = Server()
    try:
        server.start()
    except KeyboardInterrupt:
        print("Interrupt received, stopping server...")
        server.stop()
