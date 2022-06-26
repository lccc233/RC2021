import sensor
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_auto_exposure(False,exposure_us = 118000)
sensor.skip_frames(20)
#sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(1, 115200)
mode=0
kind=0
aim_threshold = (3, 74, -9, 18, -52, -16)
aim_threshold2 = (47, 100, -38, -13, -33, -8)
#blue (0,100,-128,127,-92,22)
#red  (22, 100, 2, 73, -14, 43)
yellow=(38, 100, -20, 0, 25, 80)
white=(84, 100, -17, 1, -18, 5)
aim_QRCode='B'
test_roi=(10,10,60,40)
x_p=0
y_p=0
x_mid=160
dis=160

while True:
    if uart.any():
        mode_read=uart.readline()
        print(mode_read)
        if(mode_read[0]==0):mode=0
        if(mode_read[0]==1):mode=1
        if(mode_read[0]==2):mode=2
    img = sensor.snapshot()
    #img.draw_rectangle(test_roi,(100,255,50))#
    #mode=2;
    dis=160
    if mode==1:
        print("mode: 1")
        kind=0
        aim_blobs=img.find_blobs([aim_threshold],roi=(0,60,320,180),area_threshold=1800)
        yellow_blobs=img.find_blobs([yellow],roi=(0,60,320,180),x_stride=2,y_stride=2,
                               area_threshold=1400,pixel_threshold=900,
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
        aim_blobs=img.find_blobs([aim_threshold2],roi=(0,60,320,180),x_stride=2,y_stride=2,
                             area_threshold=1400,pixel_threshold=10,
                             merge=True,margin=2)
        white_blobs=img.find_blobs([white],roi=(30,62,260,160),x_stride=2,y_stride=2,
                                    area_threshold=1800,pixel_threshold=1200,
                                    merge=True,margin=2)
        for blob in aim_blobs:
            if(abs(blob.cx()-160)<dis):
                dis=abs(blob.cx()-160)
                kind=1
                x_p=blob.cx()
                y_p=blob.cy()
                img.binary([aim_threshold2])
                img.dilate(2)
                img.draw_rectangle(blob.rect(),(255,0,0))#
                detect_area=(int((blob.x()+blob.cx())/2),
                            int((blob.y()+blob.cy())/2),
                            int(blob.w()/2+1),int(blob.h()/2+1))
                statis=img.get_statistics(roi=detect_area)
                img.draw_rectangle(detect_area,(0,255,0))#
                img.draw_cross(blob.cx(),blob.cy(),size=5, color=(0,255,0))#
                if statis.l_mean()<80:
                    kind=2
                    x_p=blob.cx()
                    y_p=blob.cy()
        for blob in white_blobs:
            if(abs(blob.cx()-160)<dis):
                dis=abs(blob.cx()-160)
                kind=3
                #img.draw_rectangle(blob.rect(),(255,255,0))#
                for code in img.find_qrcodes():
                    if(code[4]==aim_QRCode):
                        kind=1;
    if(x_p>160):
        x_p=x_p-160;
    else:
        x_p=160-x_p;
    data=bytearray([0xff,mode,kind,x_p,y_p])
    uart.write(data)
    print(data)
