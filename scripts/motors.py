#!/usr/bin/env python
#encoding: utf8
import sys, rospy, math
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion

class Motor():
    #インスタンス作成時に呼び出される
    def __init__(self):
        #モータに電源が入っていない場合プログラム終了
        if not self.set_power(True): sys.exit(1)

        #ノード終了時に行う。
        rospy.on_shutdown(self.set_power)
        #以下の2行で、外からのメッセージをコールバック関数を通して受け取る
        #トピック名、トピックの型、起動するコールバック関数
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        #サービス名、型、コールバック関数を引数とする
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        #
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)
        #last_timeにcmd_velのコールバック関数が呼ばれた時の時刻
        self.last_time = rospy.Time.now()
        #cmd_velの値が反映されているかのフラグ
        self.using_cmd_vel = False

    #モータの電源操作
    #onoffはFalseで初期化
    def set_power(self,onoff=False):
        #rtmotoren0はモータを動かすときの電源スイッチ
        en = "/dev/rtmotoren0"
        try:
            #'w'は上書き保存
            with open(en,'w') as f:
                #onoffがTrueなら1、Falseなら0
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False
    
    def set_raw_freq(self,left_hz,right_hz):
        #is_on <= onoff is_onがTrueでなければ
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_l0",'w') as lf,\
                 open("/dev/rtmotor_raw_r0",'w') as rf:
                #roundは四捨五入、○○_hzを四捨五入して整数にして文字列にするo
                #文字列にしたものをデバイスファイルに書き込む
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    #以下のcallback関数は受け取ったメッセージをset_raw_freqsに渡すもの
    #raw_freqは周波数を受け取り渡す
    def callback_raw_freq(self,message):
        self.set_raw_freq(message.left_hz,message.right_hz)
    
    #cmd_velは速度と加速度を受け取り渡す
    def callback_cmd_vel(self,message):
        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        #set_raw_freqに値を渡す
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
        #値の反映でフラグをたてる
        self.using_cmd_vel = True
        #時刻の更新
        self.last_time = rospy.Time.now()

    def onoff_response(self,onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def callback_on(self,message):  return self.onoff_response(True)
    def callback_off(self,message): return self.onoff_response(False)

    def callback_tm(self,message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev,'w') as f:
                f.write("%d %d %d\n" %
                    (message.left_hz,message.right_hz,message.duration_ms))
        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True


if __name__ == '__main__':
    #ノードの初期化、インスタンス作成
    rospy.init_node('motors')
    m = Motor()
    
    #10hz
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #cmd_velのメッセージ受信から１秒以上経過したら
        if m.using_cmd_vel and rospy.Time.now().to_sec() - m.last_time.to_sec() >= 1.0:
            #モータ周波数を0に落とす
            m.set_raw_freq(0,0)
            #反映をきるためフラグをおとす
            m.using_cmd_vel = False
        rate.sleep()
