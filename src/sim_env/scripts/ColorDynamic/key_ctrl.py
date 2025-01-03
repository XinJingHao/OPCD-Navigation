from Sparrow_Gazebo_single import Gazebo_fml_robot_env
import rospy, pygame


if __name__ == '__main__':
    rospy.init_node('Agent', anonymous=True)
    env = Gazebo_fml_robot_env()
    env.reset()
    while True:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]: a = 0
        elif keys[pygame.K_UP]: a = 2
        elif keys[pygame.K_RIGHT]: a = 4
        elif keys[pygame.K_DOWN]: a = 5
        else: a = 7 # Stop
        s = env.step(a)


