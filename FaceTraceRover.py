'''
FaceTraceRover.py

Copyright (c) 2021 toshibox

'''

from __future__ import print_function
import serial
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import time
import os
import argparse
from logging import getLogger, StreamHandler, DEBUG, basicConfig, NullHandler, INFO

"""
motorは2byteでモーター制御値、1byteが右のモーター(負で前回転)、2byte目が左のモーター(正で前回転)
right_int,left_intをmotorにセットする
"""
class cWheel:
    right_int = 0
    left_int = 0
    last_right_int = 0
    last_left_int = 0
    
class Mode:
    Normal = 0
    Up = 1
    Stop = 2
    Back = 3
    Last = 0

class Face_Dis:
    CLOSE = 250 #検出した顔のサイズ大きい→顔が近い閾値
    APPROPRIATE = 200 #検出した顔のサイズが適度である閾値
    DISTANT = 110 #検出した顔のサイズが小さい→遠い閾値

#引数初期化、引数を格納するargsオブジェクトを返す
def args_init():
    parser = argparse.ArgumentParser() #parserをつくる
    parser.add_argument('-p','--port')
    parser.add_argument('-d','--debugLevel')
    args = parser.parse_args()  
    return args

#ログ初期化、ロギングオブジェクトを返す
def logger_init(args):
    basicConfig(level=INFO, format='%(relativeCreated)6d %(threadName)s %(message)s')
    logger = getLogger(__name__)
    handler = StreamHandler()
    handler.setLevel(DEBUG)
    logger.setLevel(DEBUG)
    logger.addHandler(handler)
    logger.propagate = False
    return logger

def Camera_init():
    #フレームサイズ
    FRAME_W =800
    FRAME_H =600
    cam = PiCamera()
    cam.resolution = (FRAME_W, FRAME_H) #画素数設定
    cam.framerate = 15 #フレームレート設定
    rawCap = PiRGBArray(cam, size=(FRAME_W,FRAME_H))
    return cam, rawCap

#シリアル通信の初期化
def SerialInit(port):
    serialPort = serial.Serial(port, 19200, timeout = 0)
    serialPort.flushOutput()
    serialPort.flushInput()
    return serialPort

#左右のモーターに値をセット
def Setint(p,right,left):
    p.right_int = right
    p.left_int = left
    
#左右のモーターの値を入れ替える
def SwitchingintValue(p):
    tmp = p.left_int
    p.left_int = p.right_int
    p.right_int = tmp
    tmp = 0
    
#送信用モーター制御値を作成
def SetMotor(p):
    p.right_motor = ((p.right_int) & 0x00ff)
    p.left_motor = ((p.left_int) & 0x00ff) << 8

def ChangeToGray(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #顔検出用にグレースケール化
    gray = cv2.equalizeHist(gray)
    return gray
    
def SlowDown(p):
    if not ((p.right_int == 0) and (p.left_int == 0)):
        if((abs(p.right_int)<=6) or (abs(p.left_int)<=6)):
            p.right_int = 0
            p.left_int = 0
            
        p.right_int = p.right_int // 2
        p.left_int = p.left_int // 2

def SpeedUp(p):
    right_left_dif = 0 #速度調整用の左右モーターの差分変数
    right_cut=abs(p.right_int)//5
    right_left_dif=abs(abs(p.right_int)-abs(p.left_int))
    p.right_int-=right_cut     

    if abs(p.right_int) > abs(p.left_int):
        p.left_int=abs(p.right_int)-right_left_dif
    else:
        p.left_int=abs(p.right_int)+right_left_dif


def make_cs(p):
    cs = (0- (p.right_int & 0xff) - (p.left_int & 0xff)) & 0xff
    return cs

def SendMotorCommand(p,serial):
    SetMotor(p) #モーターの値をセット
    cs = make_cs(p)
    bytes_sent = serial.write(bytes([p.right_int & 0xff , p.left_int & 0xff, cs & 0xff, 0xff])) #シリアルでモーターを2Byte送信
    return bytes_sent
"""
顔を検出した座標を元に、モーターの制御値を設定
ラズパイカメラから取得した画像を4x4のグリットに区切り、どのグリットで顔を認識したか（顔の座標を元に）
によって、左右のモーター値を設定する（値は実際動かしてみて程良かったもの）
4x4は暫定的なもので、試作として最低限にしてある
"""
def MotorControl(p,_x,_y):
    
    if ((_x>=0) and (_x<=100)) and ((_y>=0) and (_y<=100)):
        Setint(p,-17,14)
    elif ((_x>100) and (_x<=200)) and ((_y>=0) and (_y<=100)):
        Setint(p,-16,14)
    elif ((_x>200) and (_x<=300)) and ((_y>=0) and (_y<=100)):
        Setint(p,-14,16)
    elif ((_x>300) and (_x<=400)) and ((_y>=0) and (_y<=100)):
        Setint(p,-14,17)
    elif ((_x>=0) and (_x<=100)) and ((_y>100) and (_y<=200)):
        Setint(p,-19,16)
    elif ((_x>100) and (_x<=200)) and ((_y>100) and (_y<=200)):
        Setint(p,-18,16)
    elif ((_x>200) and (_x<=300)) and ((_y>100) and (_y<=200)):
        Setint(p,-16,18)
    elif ((_x>300) and (_x<=400)) and ((_y>100) and (_y<=200)):
        Setint(p,-16,19)
    elif ((_x>=0) and (_x<=100)) and ((_y>200) and (_y<=300)):
        Setint(p,-21,18)
    elif ((_x>100) and (_x<=200)) and ((_y>200) and (_y<=300)):
        Setint(p,-20,18)
    elif ((_x>200) and (_x<=300)) and ((_y>200) and (_y<=300)):
        Setint(p,-18,20)
    elif ((_x>300) and (_x<=400)) and ((_y>200) and (_y<=300)):
        Setint(p,-18,21)
"""
画面に顔を認識したかしていないかわかる情報を表示 
具体的には、画面左上に、Labels:認識した顔のラベル（今回は自分のみ）、cofidence:学習モデルと検出した顔の一致度（以下、認識度）（低い方が良い）
"""   
'''#通信相手からシリアルで受信データを受け取る際のエラーカウント
def setInfo(img, msg, num):
    #cv2.rectangle(img, (50, 50), (300, 100), (0, 0, 0), -1)
    cv2.putText(img=img, text="{0} error={1}".format(msg, str(num)), org=(50, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.5, color=(255, 255, 255), lineType=cv2.LINE_AA)
'''
def setInfo(img, msg):
    #cv2.rectangle(img, (50, 50), (300, 100), (0, 0, 0), -1)
    cv2.putText(img=img, text="{0}".format(msg), org=(50, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.5, color=(255, 255, 255), lineType=cv2.LINE_AA)

#指定のディレクトリから認識させたい顔のモデルを生成
def loadFace(logger):
    labels = []
    faceImages = list()
    fileCount = 0
    baseImg = []
    path = ('7') #複数のモデルを毎回検出した顔と比較すると時間がかかるため、自分のモデルのみを学習させる
    # files = os.listdir(os.getcwd())
    # dir = [f for f in files if os.path.isdir(os.path.join(os.getcwd(), f))]
    #指定したフォルダ内には、自分の写真が100毎程度あるため、全部読み込んで学習させる
    for i, folder in enumerate(path):
        for fileName in os.listdir(folder):
            logger.debug(fileName)
            face = cv2.imread(folder + '/' + fileName)
            if fileCount == 0:
                baseImg = face
            face = resizeAuthImg(face, baseImg)
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
            faceImages.append(face)
            labels.append(int(folder))
            fileCount += 1
    return faceImages, labels

#顔認識するために指定の画像サイズにする
def resizeAuthImg(src, baseImg):

    height = baseImg.shape[0]
    width = baseImg.shape[1]
    dst = cv2.resize(src, (width, height))

    return dst
#顔の検出サイズによって認識度(score)が違ったため（実際動かしてみた）、検出サイズと認識度の組み合わせで分岐させる
#顔の検出サイズw<80のときだけとりあえずちかづく？もしくは、wに比例してscoreの値を決めていく？
def PatternClose(w,score):
    if ((w > 280 or (w <= 120 and w >= 80)) and score <= 60):
        return True
    else:
        return False
    
def PatterMiddle(w,score):
    if (((w <= 280 and w > 200) or (w <= 200 and w > 120)) and score <= 50):
        return True
    else:
        return False
    
def PatternDistant(w,score):
    if (w < 80 and score <= 80):
        return True
    else:
        return False
    
def CheckDataScore(w,score):
    if (PatternClose(w,score)) or (PatterMiddle(w,score)) or (PatternDistant(w,score)):
        return True
    else:
        return False

def GetScore(recognizer, detected_face, compaired_faces):
    detected_face=resizeAuthImg(detected_face,compaired_faces[0])
    detected_face = cv2.cvtColor(detected_face, cv2.COLOR_BGR2GRAY)
    labelNum, score = recognizer.predict(detected_face) #学習データとカメラ画像を照合し、最も近いラベル番号と信頼度値を受け取る
    return labelNum, score

def FaceTrace(logger, args):
    
    compaired_faces, labels = loadFace(logger) #自分の顔のモデル読み込む

    recognizer = cv2.face.LBPHFaceRecognizer_create() #顔認識用オブジェクト作成
    recognizer.train(compaired_faces, np.array(labels)) #自分の顔のモデルをlabeltp紐付ける

    #正面の顔を検出するためのモデル
    cascade = ('/home/pi/rpicamera/data/haarcascades/haarcascade_frontalface_alt2.xml')
    
    #検出用の顔のモデルを画像データと比較できるよう変換（顔分類器）
    face_cascade = cv2.CascadeClassifier(cascade)
    #各種パラメータの初期化
    mode = Mode.Normal #検出した顔の距離によって速度調整するためのモード
    mode_last = Mode.Normal
    wheel = cWheel() #左右のモータークラスのインスタンス
    counter = 0
    
    camera, rawCapture = Camera_init() #カメラ初期化
    
    time.sleep(0.1)

    #シリアル通信の初期化
    serialPort = SerialInit(args.port)
    #画像処理開始
    for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        detected_facelocation =[0,0]
        detected_face = [0,0]
        starttime = time.time()
        #ある時点の写真を切り取り、前処理をする
        frame = image.array
        #logger.debug("frame:" + str(type(frame)))
        gray = ChangeToGray(frame)
        #顔分類器で顔検出する
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3, minSize=(70,70))
        #顔を検出したら、検出した座標と検出範囲（検出範囲が大きい→検出した顔が大きい→顔が近い）を取得
        for (x, y, w, h) in faces: #検出した顔の座標データが入る
            cv2.rectangle(frame,(x,y),(x+w, y+h),(0,255,0),2) #検出範囲を矩形でかこう
            detected_facelocation = np.array([int((x+w)/2), int((y+h)/2)], dtype=int) #検出した顔の中心の座標をとる
            logger.debug("yoko:"+ str(w) + "  tate:" + str(h))

            if w > Face_Dis.CLOSE: #検出サイズ大きい→近い→後退モード
                mode=Mode.Back

            elif w<=Face_Dis.CLOSE and Face_Dis.APPROPRIATE<w:#検出サイズ適度→停止モード
                mode=Mode.Stop

            elif w<=Face_Dis.DISTANT: #検出サイズ小さい→遠い→速度UP
                mode=Mode.Up
                
            detected_face = frame[y:y+h, x:x+w] #検出した顔の部分だけ取得（認識用）
            logger.debug(type(frame))
                #ここで顔周辺を切り取ってリサイズしないと、サイズの関係で顔認識できない←解決？
                #x,y,w,hを使って、顔周辺 を切り出す処理が必要（現状これしかわからん）

        #取得した顔情報を認識したい顔モデルと比較
        if len(detected_face) is not 2: #顔を検出していない場合、detected_face = 2
            #logger.debug("detected_face check : " + str(len(detected_face)))
            labelNum, score = GetScore(recognizer, detected_face, compaired_faces)
            #検出サイズ(w)と認識スコア(cofidence)の組み合わせにより、認識したい顔か判断し分岐
            if (CheckDataScore(w,score)):
                message = 'Label={0} level={1}'.format(labelNum, int(score))
                    
                if mode_last==Mode.Stop: #ちょうどいい距離で検出したから、3ループ目で減速しつつ止まる（機体への負荷軽減のため）
                    SlowDown(wheel)
                #検出した顔の座標からモーターの制御値を決定
                else:
                    _x = detected_facelocation[0]
                    _y = detected_facelocation[1]
                            
                    MotorControl(wheel,_x,_y)

                    #検出範囲を元に動作を決定
                    if mode==Mode.Back: #検出した顔が近すぎる→後退する
                        SwitchingintValue(wheel) #左右のモーターの値を入れ替える
                           
                    elif mode==Mode.Stop: #ちょうどいい距離で検出したから、減速しつつ止まる（機体への負荷軽減のため）                        
                        SlowDown(wheel)
                        
                    elif mode==Mode.Up: #検出した顔が遠いから、1/5速度アップ
                        SpeedUp(wheel)
                    #mode=Mode.Normalの場合、検出した顔は適度な距離なので速度はそのまま
                        
                logger.debug('mode=' + str(mode))
                mode_last = mode
                mode = Mode.Normal #最後にノーマルモードに戻す
                    
            else:
                SlowDown(wheel)
                message = 'During verification'#顔は検出したけど、自分の顔と認識しない場合
                logger.debug("Not recognized")
        
        else: #顔が検出されなければ、減速する（機体への負荷軽減のため）
            score = 0
            message = 'None'
            SlowDown(wheel)
            #logger.debug("(right_int : "+ str(wheel.right_int)+ "left_int :" + str(wheel.left_int) + ")")
            
        bytes_sent = SendMotorCommand(wheel,serialPort)
        line = serialPort.readline()
        '''#コメントアウトしているsetInfoと共に使用
        if not line == (1 & 0xff):
            counter += 1
        '''
        endtime = time.time()
        interval = endtime - starttime
        
        logger.debug("H8 : " + str(line))
        logger.debug("time : " + str(interval))
        logger.debug("1cycle end : cofidence" + str(score))
        logger.debug("sent" + str(bytes_sent) + "bytes") 

        #setInfo(frame, message, counter) #顔認識の結果を表示
        setInfo(frame, message)
        
        frame = cv2.resize(frame, (800,600))
            
        cv2.imshow('Video',frame) #画面に検出した顔を矩形で囲った画像を表示
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
            
        if key == ord("q"): #qを押したらループ終了
            break

    cv2.destroyAllWindows() #画面閉じる


def __main():
    
    args = args_init()
    logger = logger_init(args)
    FaceTrace(logger, args)

if __name__=='__main__':
    __main()