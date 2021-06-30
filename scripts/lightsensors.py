#!/usr/bin/env python
#encoding: utf8
import sys, rospy
from pimouse_ros.msg import LightSensorValues

#外部からの入力でセンサの読むスピード(rate)を変化させる
def get_freq():
    #rospy.get_paramはROSパラメータサーバから周波数を受け取って返す関数
    #lightsensors_freqはパラメータ名 10はパラメータの初期値
    f = rospy.get_param('lightsensors_freq', 10)
    try:
        #fが0以下の場合例外を投げる。
        if f <= 0.0:
            raise Exception()
        #例外やfが数字でなかった場合はエラーを吐く。 
    except:
        rospy.logerr("value error: lightsensors_freq")
        #sys.exit()でプログラム終了させる。また終了ステータスは1
        #終了ステータスは「0」が正常、「1」が異常
        sys.exit(1)

    return f

if __name__ == '__main__':
    devfile = '/dev/rtlightsensor0'
    #初期化しROSにノードを追加する
    rospy.init_node('lightsensors')
    #lightsensorというトピックにLughtSensorValuesというメッセージのタイプ
    #で送る宣言。queue_sizeでメッセージを十分な速度で受け取れていない場合
    #メッセージの量を制限している。
    pub = rospy.Publisher('lightsensors', LightSensorValues, queue_size=1)
   
    #sleep()用いて設定した速度でループさせる。
    freq = get_freq()
    rate = rospy.Rate(freq)
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

        f = get_freq()
        if f != freq:
            freq = f
            rate = rospy.Rate(freq)

        rate.sleep()
