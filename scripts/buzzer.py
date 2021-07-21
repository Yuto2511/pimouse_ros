#!/usr/bin/env python
#encoding: utf8
#ラズパイマウス側のブザーは鳴らなかったが一応動く
import rospy
import rospy, actionlib
from std_msgs.msg import UInt16
from pimouse_ros.msg import MusicAction, MusicResult, MusicFeedback

#引数のデフォルトが0であり、引数なしで呼び出した場合hzに0が入った状態で処理される。
def write_freq( hz=0 ):
    bfile = "/dev/rtbuzzer0"
    try:
        #withを用いることで後処理を省いている 今回はclose
        #openでファイルを開く処理 今回は/dev/rtbuzzer0を開く "w"は上書き保存
        #asで読み込んだファイルをfという変数で処理が出来るようになる
        #f = open("/dev/buzzer0", "w")とも書き換えられる
        with open(bfile,"w") as f:
            f.write(str(hz) + "\n")
    #エラーをログに吐く
    except IOError:
        rospy.logerr("can't write to " + bfile)

#コールバック関数
def recv_buzzer(data):
    write_freq(data.data)

def exec_music(goal):
    #結果として返すデータ
    r = MusicResult()
    #途中経過として返すデータ
    fb = MusicFeadback()

    for i, f in enumerate(goal.freqs):
        fb.remaining_steps = len(goal.freqs) - i
        music.publish_feedback(fb)

        #中断があった時の処理
        if music.is_preempt_requested():
            #周波数を0
            write_freq(0)
            r.finished = False
            #set_preemptedを通してexec_musicを終了
            music.set_preempted(r)
            return
        
        write_freq(f)
        rospy.sleep(1.0 if i >= len(goal.durations) else goal.durations[i])

    #アクション終了時の処理
    r.finished = True
    music.set_succeeded(r)

if __name__ == '__main__':
    #初期化しROSにノードを追加する
    rospy.init_node('buzzer')
    #Subscriberで定期読み込みをする("トピック名",トピックの型,コールバック関数)
    rospy.Subscriber("buzzer", UInt16, recv_buzzer)
    music = actionlib.SimpleActionServer('music', MusicAction, exec_music, False)
    music.start()
    rospy.on_shutdown(write_freq)
    rospy.spin()
