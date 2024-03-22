import lt_module
from main import cr_status

threshold = lt_module.cr()


def cr(threshold):
    cr_status = 0
    bool_cr=False
    while (True):
        if bool_cr:
            break
        else:
            clock.tick()
            img = sensor.snapshot()
            for blob in img.find_blobs([threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10):
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
            print(clock.fps())
            bool_cr=True
    '''if 20 <= threshold[0] <= threshold[1] <= 80 and -50 <= threshold[2] <= threshold[3] <= -20 and 10 <= threshold[4] <= threshold[5] <= 40:
        cr_status = 1'''  # 紫色
    if 40 <= threshold[0] <= threshold[1] <= 90 and 20 <= threshold[2] <= threshold[3] <= 100 and -10 <= threshold[4] <= threshold[5] <= 10:
        cr_status = 1  # 红色
    elif 40 <= threshold[0] <= threshold[1] <= 80 and -80 <= threshold[2] <= threshold[3] <= -20 and -40 <= threshold[4] <= threshold[5] <= 10:
        cr_status = 2  # 绿色
    '''elif 20 <= threshold[0] <= threshold[1] <= 80 and -20 <= threshold[2] <= threshold[3] <= 20 and -40 <= threshold[4] <= threshold[5] <= 30:
        cr_status = 4''' # 棕色
    return cr_status
''''''
