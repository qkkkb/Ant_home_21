import sensor, image, time
import tf#运行.tfile模型
import pyb#openMV的底层硬件库
from pyb import LED
from machine import UART#导入串口通信摸块
import ustruct#把串口通信数据转化为字节
roi_count=0
# ================= 摄像头初始化 =================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)#关闭自动增益
sensor.set_auto_whitebal(False)#关闭白平衡
sensor.set_auto_exposure(True,exposure_us=150)#控制高光
# ================= 模型路径 =================
detect_model_path = "detect.tflite"      # 目标检测模型
classify_model_path = "classify.tflite"  # 分类模型

# ================= 加载模型 =================
net_detect = tf.load(detect_model_path)#加载检测模型
net_classify = tf.load(classify_model_path, load_to_fb=True)#加载分类模型

# ================= 类别标签 =================
labels = [
    'ball',
    'redbag',
    'bluebag',
    'brownbear',
    'whitebear'
]


# ================= YOLO检测 =================
def detect_object():
    img = sensor.snapshot()

    objects=[]

    for obj in tf.detect(net_detect, img.crop(roi=(40,0,240,240))):#图片裁剪到1:1
        x1,y1,x2,y2,label,score = obj

        if score > 0.6:

            w = int((x2-x1)*240)
            h = int((y2-y1)*240)

            x1 = int(x1*240)#裁剪后实际像素的坐标值
            y1 = int(y1*240)
            #print(f"x1={x1},y1={y1},w={w},h={h}")
            roi_img = img.copy(roi=(x1,y1,w,h)) # 裁剪出目标区域ROI(region of interest)

            img.draw_rectangle(x1,y1,w,h,color=(0,255,0))#在终端画出绿色框

            objects.append((roi_img,x1,y1))

    return objects if objects else None


# ================= 分类 =================
def classify_object(img_roi,x1,y1):
    #print(f"img_roi.width={img_roi.width()},img_roi.height={img_roi.height()},x1={x1},y1={y1}")
    '''global roi_count  # 引用全局计数器


    roi_count += 1
    save_path = r"/picture/roi_%d.bmp" % roi_count
    img_roi.save(save_path)  # 保存图片到OpenMV的Flash/SD卡
    print("已保存ROI图片:", save_path)  # 串口打印确认'''

    for obj in tf.classify(net_classify, img_roi,
                           min_scale=1,  #不修改传入图片
                           scale_mul=0.2, x_overlap=0.2, y_overlap=0.2,
                           scale=1,offset=1):

        sorted_list = sorted(zip(labels, obj.output()),  #输出[('target1', 0.1),('target2', 0.7),('target3', 0.1),('target4', 0.05),('target5', 0.05)]
                             key=lambda x:x[1],
                             reverse=True)
        #print(sorted_list)

        if sorted_list[0][0]>=0.4:
            img.draw_string(x1,y1,labels,color=(255,0,0),scale=1)



    return False


# ================= 主循环 =================
clock=time.clock()
while True:
    clock.tick()
    objects=detect_object()
    if objects is not None:
        for object in objects:
            classify_object(*object)
    #print(clock.fps())
