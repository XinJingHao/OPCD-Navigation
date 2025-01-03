from Env_for_Evaluation import Gazebo_fml_robot_env
import rospy



# 19 editable configurations:
default_cfg = dict(
    frequency=10, # record frequency
    ld_range=300, # max scanning distance; cm
    ld_num=240, # number of lidar streams in each world
    ld_GN=10, # how many lidar streams are grouped for each group
    startpose = (9,-9,2.35), # x,y,theta of fml_robot when reset
    target_point = (-9,9,2.35) # target point of navigation
    # startpose = (9,-9,1.57), # x,y,theta of fml_robot when reset
    # target_point = (0,0,3.14) # target point of navigation
    )


def play_with_rviz():
    # Env init:
    env = Gazebo_fml_robot_env(**default_cfg)


    travel_distance = travel_time = avg_v = win_cnt = 0
    for i in range(1,100+1):
        env.reset()

        t_start = rospy.Time.now()
        while True:
            win, collide = env.step()

            if win or collide:
                if win: win_cnt += 1

                '''当前回合数据:'''
                temp_travel_distance = round(env.get_traj_length(), 2)
                temp_travel_time = round((rospy.Time.now() - t_start).to_sec(), 2)
                temp_avg_v = round(temp_travel_distance / temp_travel_time, 2)
                print(f'Evaluation {i} >> Travel distance:{temp_travel_distance}m, Travel time:{temp_travel_time}s, Average velocity:{temp_avg_v}m/s, Arrived:{win}')

                # '''所有回合平均数据:'''
                # if win:
                #     travel_distance += temp_travel_distance
                #     travel_time += temp_travel_time
                #     avg_v += temp_avg_v
                #     print(f'Averaged Result >> Travel distance:{round(travel_distance/win_cnt, 2)}m, '
                #           f'Travel time:{round(travel_time/win_cnt, 2)}s, '
                #           f'Average velocity:{round(avg_v/win_cnt, 2)}m/s, '
                #           f'Success rate:{round(win_cnt/i, 2)} \n')

                break

if __name__ == '__main__':
    rospy.init_node('Agent', anonymous=True)
    play_with_rviz()