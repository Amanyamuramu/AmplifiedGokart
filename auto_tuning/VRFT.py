# データ駆動制御プログラム：PID調整
# 作成日2022/11/19(自称AI・制御エンジニア)
# 更新日2022/11/19(自称AI・制御エンジニア)

import numpy as np
from control.matlab import *
from matplotlib import pyplot as plt
import pandas as pd
import sys
from scipy.optimize.minpack import transpose

df = pd.read_csv('VRFT_data_angular.csv') # 入出力応答の読み込み

u=df['u'].values #入力データ列読み込み
t=df['t'].values #時間データ列読み込み
y=df['y'].values #出力データ列読み込み
ref=df['ref'].values #出力データ列読み込み

# 以下PID制御器更新
Ts=0.5 #規範モデルの時定数
Td=tf([1],[Ts,1]) 
L=Td #プレフィルタ(カスタマイズしたい場合はここを任意のプレフィルタに差し替えてください)

intg=tf([1],[1,0]) # 積分
dif=tf([1,0],[0.1,1])# 微分

(y1p, T1p, x1p )=lsim(L*(1-Td)/Td,y,t)# P制御擬似誤差
(y1i, T1i, x1i )=lsim(L*intg*(1-Td)/Td,y,t) # I制御擬似誤差 
(y1d, T1d, x1d )=lsim(L*dif*(1-Td)/Td,y,t) # D制御擬似誤差
(y2a, T2a, x2a )=lsim(L,u,t) #

valb=input("PI制御なら1、PID制御なら2を入力してください:")
if valb != '1' and valb !='2':
  sys.exit('エラーが発生しました。もう一度プログラムを実行してください')
else:
  if valb=='1':
    A=[y1p,y1i] # PI制御の場合
  if valb=='2':  
    A=[y1p,y1i,y1d] # PID制御の場合   

invA=np.linalg.pinv(A) #擬似逆行列
rho=y2a@invA #最適パラメータの求解

if valb=='1':
  print('PIゲイン更新結果:','kp=',rho[0],':ki=',rho[1]) #PIパラメータ確認
if valb=='2':
  print('PIDゲイン更新結果:','kp=',rho[0],':ki=',rho[1],':kd=',rho[2]) #PIDパラメータ確認

