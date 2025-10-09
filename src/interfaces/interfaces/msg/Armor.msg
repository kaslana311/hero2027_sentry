float64 time     # 图像的ros时间戳
int32 nums                    # 灯条个数，一定是偶数                  2n
int32[] center_xs             # 表示灯条的旋转矩形的中心点坐标之x       2n
int32[] center_ys             # 表示灯条的旋转矩形的中心点坐标之y       2n
int32[] heights               # 表示灯条的旋转矩形的高度               2n
int32[] widths                # 表示灯条的旋转矩形的宽度               2n
float64[] angles             # 表示灯条的旋转矩形的角度               2n
float64[] distances          # 表示距离装甲板的距离，每两个灯条对应一个  n
bool[] isbigarmors           # 表示是否为大装甲板                     n
int32[] class_nums           # 找到的装甲板对应的数字 
bool net                     # 是否使用神经网络
int32[] cord_x               # 四个一组，为角点x坐标
int32[] cord_y               # 四个一组，为角点y坐标
float64[] x                    # 尼采
float64[] y
float64[] z