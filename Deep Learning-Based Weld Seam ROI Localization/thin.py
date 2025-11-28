import cv2
import numpy as np
from skimage.morphology import skeletonize
import os

def overlay_skeleton_on_image(gray_path, output_path):
    """
    将骨架化后的中心线以红色叠加到原图上并保存。

    参数：
        gray_path (str): 输入灰度图像路径。
        output_path (str): 输出图像保存路径。
    """
    # 确保输出目录存在
    output_dir = os.path.dirname(output_path)
    os.makedirs(output_dir, exist_ok=True)

    # 1. 使用支持中文路径的方式读取灰度图
    gray = cv2.imdecode(np.fromfile(gray_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(f"❌ 无法读取图像文件，请检查路径或文件完整性: {gray_path}")
        return

    # 转换为 BGR 彩色图
    color_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # 2. 二值化并骨架化
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    skeleton = skeletonize(binary > 0)

    # 3. 叠加红色骨架
    color_img[skeleton] = (0, 0, 255)  # BGR 格式中的红色

    # 4. 使用支持中文路径的方式保存结果图
    ext = os.path.splitext(output_path)[1]
    if not ext:
        ext = '.png'  # 如果没有扩展名，默认为.png
    cv2.imencode(ext, color_img)[1].tofile(output_path)

if __name__ == '__main__':
    gray_path = r"C:\Users\admin\Desktop\小论文\蒙版图片\false\黑白蒙版\1.png"
    output_path = r"C:\Users\admin\Desktop\小论文\蒙版图片\false\黑白蒙版\新建文件夹\1.png"
    overlay_skeleton_on_image(gray_path, output_path)
    print(f"输出图已保存至 {output_path}")
