#!/usr/bin/env python
#encoding: utf8

#importはc言語でいうinclude モジュール(.py)やパッケージをプログラム内に持ち込める
import rospy, unittest, rostest
import rosnode
import time

#classを作る クラスは関数や変数をまとめたもの(設計図)でこれ自体で実行されない
#BuzzerTestというクラスの作成 unittestというテストフレームワークを使用
#selfはクラスで作成されたインスタンスのことを指す
#aaa.bbbは「aaa」の「bbb」を指す
class BuzzerTest(unittest.TestCase):
    #defでは関数を定義する test_node_exist関数
    def test_node_exist(self):
        #rosnode.get_node_namesで今立ち上がっているノードを取得
        #取得した値をnodesに代入
        nodes = rosnode.get_node_names()
        #assertInはunittest内にあるモジュール
        #nodesにbuzzerが存在しているかを確認 確認できなかった場合「node does not exist」と表示
        self.assertIn('/buzzer',nodes,"node does not exist")

#このif文がないとimportしたモジュールやパッケージが勝手に実行されてしまう
#仮にimport time とした場合 timeの__name__は__time__になる
#このtravis_test_buzzr.pyの__name__は__main__である(importされない場合は__name__は__main__になる)
#__name__が__main__だと実行つまりtravis_test_buzzer.pyのみが実行される
if __name__ == '__main__':
    #テスト対象のノードが実行されるまで待つ
    time.sleep(3)
    #このプログラムにノード名をつける
    rospy.init_node('travis_test_buzzer')
    #このテストプログラムを走らせる
    #パッケージ名、ノード名、走らせるクラス名の順に記述
    rostest.rosrun('pimouse_ros','travis_test_buzzer',BuzzerTest)
