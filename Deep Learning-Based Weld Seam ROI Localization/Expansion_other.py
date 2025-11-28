import cv2
import numpy as np

# 膨胀区域并相减
def process_image(image_path):
    # 读取图像（灰度模式）
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("Image not found or path is incorrect")

    # 1. 提取亮度在200-255的区域（二值化）
    high_brightness = (img >= 200) & (img <= 255)
    high_brightness_region = np.zeros_like(img, dtype=np.uint8)
    high_brightness_region[high_brightness] = 255

    # 2. 定义圆形膨胀核（直径=5x5，可调整大小）
    kernel_size = 45  # 必须是奇数（3,5,7,...）
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    # 3. 膨胀操作（使用圆形核）
    dilated = cv2.dilate(high_brightness_region, kernel, iterations=1)

    # 4. 检查膨胀后的区域是否位于原图亮度<20的区域
    mask = (img < 20)  # 原图中亮度<20的区域
    dilated_masked = dilated.copy()
    dilated_masked[mask] = 0  # 如果膨胀后的像素在原图亮度<20，则强制设为0

    # 5. 生成新图像
    new_img = img.copy()
    new_img[dilated_masked == 255] = 255  # 应用膨胀后的高亮度区域
    new_img[mask] = 0  # 确保原图亮度<20的区域保持为0（可选）

    return new_img


# 示例使用
# input_image_path = 'input.jpg'
# output_image = process_image(input_image_path)
# cv2.imwrite('output.jpg', output_image)
# 示例使用
input_image_path = r"C:\Users\admin\Desktop\小论文\蒙版图片\false\黑白蒙版\1.png"
output_image = process_image(input_image_path)
cv2.imwrite('C:/Users/admin/Desktop/144224.png', output_image)