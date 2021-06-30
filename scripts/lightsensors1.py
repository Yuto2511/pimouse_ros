#!/usr/bin/env python
#encoding: utf8
import sys, rospy
from pimouse_ros.msg import LightSensorValues

if __name__ == '__main__':
    devfile = '/dev/rtlightsensor0'
    #初期化しROSにノードを追加する
    rospy.init_node('lightsensors')
    #lightsensorというトピックにLughtSensorValuesというメッセージのタイプ
    #で送る宣言。queue_sizeでメッセージを十分な速度で受け取れていない場合
    #メッセージの量を制限している。
    pub = rospy.Publisher('lightsensors', LightSensorValues, queue_size=1)
   
    #sleep()用いて設定した速度でループさせる。今回は10Hz 1秒間に10回処理できる
    rate = rospy.Rate(10)
    #rospy.is_shutdownは例えばCtr+Cなどのことで無限ループ
    while not rospy.is_shutdown():
        try:
            #withでclose処理も行てくれる
            with open(devfile,'r') as f:
                #デバイスファイルはスペーサー区切りでセンサの値を送る
                #readline()でファイルを一行読む
                #split()は空白文字で区切りをつけるのでデータを4つに区切る
                data = f.readline().split()
                #dataの要素を一つづつeという変数に取り出しint関数で整数に戻す
                data = [ int(e) for e in data ]
                d = LightSensorValues()
                #LightSensorValueにそれぞれデータを入れている
                d.right_forward = data[0]
                d.right_side = data[1]
                d.left_side = data[2]
                d.left_forward = data[3]
                d.sum_all = sum(data)
                d.sum_forward = data[0] + data[3]
                #oublishを用いてデータをメッセージとして送信している
                pub.publish(d)
        except IOError:
            rospy.logerr("cannot write to " + devfile)

        rate.sleep()
