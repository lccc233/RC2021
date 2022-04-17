import sensor
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)

sensor.set_auto_exposure(False, \
    exposure_us = 20000)
sensor.skip_frames(20)
#sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(1, 115200)
mode=0
kind=0
aim_threshold = (10, 85, 3, 68, 6, 59)
#blue (0,100,-128,127,-92,22)
#red  (22, 100, 2, 73, -14, 43)
yellow=(18, 83, -37, 11, 22, 77)
white=()
aim_QRcode='R'
test_roi=(10,10,60,40)
x_p=0
y_p=0
x_mid=40

while True:
    if uart.any():
        mode_read=uart.readline()
        print(mode_read)
        if(mode_read[0]==0):mode=0
        if(mode_read[0]==1):mode=1
        if(mode_read[0]==2):mode=2
    img = sensor.snapshot()
    #img.draw_rectangle(test_roi,(100,255,50))#
    mode=1;
    if mode==1:
        print("mode: 1")
        kind=0
        aim_blobs=img.find_blobs([aim_threshold],area_threshold=100)
        yellow_blobs=img.find_blobs([yellow],x_stride=2,y_stride=2,
                               area_threshold=100,pixel_threshold=50,
                               merge=True,margin=2)
        for blob in aim_blobs:
            kind=1
            img.draw_rectangle(blob.rect(),(255,0,0))#
            x_p=blob.cx()
            y_p=blob.cy()
        for blob in yellow_blobs:
            kind=2
            img.draw_rectangle(blob.rect(),(255,255,0))#
            x_p=blob.cx()
            y_p=blob.cy()
    if mode==2:
        kind=0;
        aim_blobs=img.find_blobs([aim_threshold],x_stride=2,y_stride=2,
                             area_threshold=10,pixel_threshold=10,
                             merge=True,margin=2)
        white_blobs=img.find_blobs([white],x_stride=2,y_stride=2,
                                    area_threshold=6000,pixel_threshold=100,
                                    merge=True,margin=2)
        for blob in aim_blobs:
            kind=1
            img.binary([aim_threshold])
            img.dilate(2)
            img.draw_rectangle(blob.rect(),(255,0,0))#
            detect_area=(int((blob.x()+blob.cx())/2),
                         int((blob.y()+blob.cy())/2),
                         int(blob.w()/2+1),int(blob.h()/2+1))
            statis=img.get_statistics(roi=detect_area)
            img.draw_rectangle(detect_area,(0,255,0))#
            img.draw_cross(blob.cx(),blob.cy(),size=5, color=(0,255,0))#
            if statis.l_mean()<60:
                kind=2
        for blob in white_blobs:
            if abs(blob.cx()-x_mid)<10:
                kind=3
                for code in img.find_qrcodes():
                    if(code[4]==aim_QRCode):
                        kind=1;
    data=bytearray([0xff,mode,kind,x_p,y_p])
    uart.write(data)
    print(data)
