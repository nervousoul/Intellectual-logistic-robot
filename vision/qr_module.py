import sensor, image

def qr():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA) # can be QVGA on M7...
    sensor.skip_frames(30)
    sensor.set_auto_gain(False) # must turn this off to prevent image washout...
    while(True):
        img = sensor.snapshot()
        img.lens_corr(1.8) # strength of 1.8 is good for the 2.8mm lens.
        for code in img.find_qrcodes():
            print(code)
            if code==11:
                case=1
            elif code==12:
                case=2
            elif code==21:
                case=3
            elif code==22:
                case=4
            return case
        ''''''
