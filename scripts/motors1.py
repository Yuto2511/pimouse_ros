#!/usr/bin/env python
#encoding: utf8
import sys, rospy, math
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist

class Motor():
    #
    def __init__(self):
        #起動していなかった場合　終了ステータスを[0]でプログラムを終了する
        if not self.set_power(True): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        #last_timeが最後にcmd_velのコールバック関数が呼ばれた時の時刻
        #rospy.Time.nowは現在の時刻
        self.last_time = rospy.Time.now()
        #using_cmd_velはコールバック関数が呼ばれてcmd_velの値がモータに反映されてるフラグ
        self.using_cmd_vel = False

    #/dev/motor0に値を書き込んでモーターの電源を操作する
    #デフォルトでonoffをFalseに指定
    def set_power(self,onoff=False):
        en = "/dev/rtmotoren0"
        #モーターの電源を入れる
        try:
            with open(en,'w') as f:
                #「A if 条件式 else B」 条件式がTrueならA FalseならB
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self,left_hz,right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_10",'w') as lf,\
                 open("/dev/rtmotor_raw_10",'w') as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    #motor_rawのコールバック関数
    def callback_raw_freq(self,message):
        self.set_raw_freq(message.left_hz,message.right_hz)

    #cmd_velのコールバック関数
    def callback_cmd_vel(self,message):
        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
        self.using_cmd_vel = True
        self.last_time = rospy.Time.now()

if __name__ == '__main__':
    #ノードの初期化 インスタンス作成
    rospy.init_node('motors')
    m = Motor()

    rate = rospy.Rate(10)
    #motor.pyが立ち下がるまで実行
    while not rospy.is_shutdown():
        #メッセージが1秒以上昔(last_time)であれば周波数を0におとす
        if m.using_cmd_vel and rospy.Time.now().to_sec() - m.last_time.to_sec() >= 1.0:
            m.set_raw_freq(0,0)
            m.using_cmd_vel = False
        rate.sleep()
