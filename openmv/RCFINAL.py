import sensor
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(20)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(1, 115200)
mode=0
kind=0
aim_threshold = (0, 100, 12, 105, -6, 60)
#blue (0,100,-128,127,-92,22)
#red  (22, 100, 2, 73, -14, 43)
yellow=(0, 100, -24, 39, 32, 76)
white=()
x_mid=160
y_mid=120
aim_QRcode='R'

while True:
    if uart.any():
        mode_read=uart.readline()
        print(mode_read)
        if(mode_read[0]==1):mode=1
    img = sensor.snapshot()
    if mode==1:
        kind=0
        aim_blobs=img.find_blobs([aim_threshold],x_stride=2,y_stride=2,
                             area_threshold=6000,pixel_threshold=100,
                             merge=True,margin=2)
        yellow_blobs=img.find_blobs([yellow],x_stride=2,y_stride=2,
                               area_threshold=600,pixel_threshold=100,
                               merge=True,margin=2)
        for blob in aim_blobs:
            if abs( blob.cx() - x_mid ) <10:
                kind=1
                img.draw_rectangle(blob.rect(),(255,0,0))#
        for blob in yellow_blobs:
            if abs(blob.cx()-x_mid)<10:
                kind=2
                img.draw_rectangle(blob.rect(),(255,255,0))#
    if mode==2:
        kind=0;
        aim_blobs=img.find_blobs([aim_threshold],x_stride=2,y_stride=2,
                             area_threshold=6000,pixel_threshold=100,
                             merge=True,margin=2)
        white_blobs=img.find_blobs([white],x_stride=2,y_stride=2,
                                    area_threshold=6000,pixel_threshold=100,
                                    merge=True,margin=2)
        for blob in aim_blobs:
            if abs(blob.cx()-x_mid)<10:
                kind=1
                img.binary([self.aim_threshold])
                img.dilate(2)
                img.draw_rectangle(blob.rect(),(255,0,0))#
                detect_area=(int((blob.x()+blob.cx())/2),
                             int((blob.y()+blob.cy())/2),
                             int(blob.w()/2),int(blob.h()/2))
                statis=img.get_statistics(roi=detect_area)
                img.draw_rectangle(detect_area,(0,255,0))#
                img.draw_cross(blob.cx(),blob.cy(),size=5, color=(0,255,0))#
                if statis.l_mean()>60:
                    kind=2
        for blob in white_blobs:
            if abs(blob.cx()-x_mid)<10:
                kind=3
                for code in img.find_qrcodes():
                    if(code[4]==aim_QRCode):
                        kind=1;
    data=bytearray([0xff,mode,kind])
    uart.write(data)
