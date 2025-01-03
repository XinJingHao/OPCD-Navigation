from Env_Color import Gazebo_fml_robot_env
from pynput.keyboard import Key,Listener
import rospy


'''############################### Keyboard Listening ##############################'''
ACTION = 7  # Stop
def on_press(key):
    global ACTION

    #key control
    if key == Key.up: ACTION = 2
    elif key == Key.down: ACTION = 5
    elif key == Key.left: ACTION = 0
    elif key == Key.right: ACTION = 4
    else: ACTION = 7  # Stop

def on_release(key):
    global ACTION
    ACTION = 7  # Stop

listener = Listener(on_press=on_press, on_release=on_release)
listener.start()
'''############################### Keyboard Listening ##############################'''

if __name__ == '__main__':
    rospy.init_node('Agent', anonymous=True)
    env = Gazebo_fml_robot_env()
    env.reset()
    print('\nUse Up/Down/Left/Right to control the agent.')
    while True:
        s, win, collide = env.step(ACTION)
        # print(f'win:{win}, collide:{collide}')









