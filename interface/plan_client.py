import socket
import pickle

class Client:
    def __init__(self, host='localhost', port=12346):
        # Create a socket object
        self.s = socket.socket()

        # Define the port on which you want to connect
        self.port = port
        self.host = host
# Connect to the server
        self.s.connect((self.host, self.port))
    def send_data(self, init_pose, goal_pose):
        

        # Send the initial and goal poses
        self.s.send(pickle.dumps({'init': init_pose, 'goal': goal_pose}))

        # Receive data from the server
        data = self.s.recv(102400)

        # Unpickle the received data to get the path
        path = pickle.loads(data)
        
        # Close the connection
        

        return path
    def close(self):
        self.s.close()

if __name__ == '__main__':
    client = Client()
    
    while True:
        try:
            init_pose = tuple(map(float, input("Enter initial pose (x, y, heading): ").split(',')))
            goal_pose = tuple(map(float, input("Enter goal pose (x, y, heading): ").split(',')))
        except Exception as e:
            print("Invalid input. Please enter three comma-separated floats.")
            continue

        path = client.send_data(init_pose, goal_pose)
        print(path)
    client.close()