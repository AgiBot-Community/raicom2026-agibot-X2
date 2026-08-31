import cv2
import numpy as np

def analyze_grayscale(image_path):
    # 1. 读取图片
    img = cv2.imread(image_path)
    if img is None:
        print(f"❌ 错误: 无法读取图片 {image_path}")
        return

    # 2. 转为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    print(f"✅ 成功读取图片，分辨率: {w}x{h}")

    # 3. 统计基础灰度数据
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray)
    mean_val = np.mean(gray)
    median_val = np.median(gray)
    
    print(f"📊 灰度统计结果:")
    print(f"   - 最低灰度值 (最黑): {min_val} (位置: {min_loc})")
    print(f"   - 最高灰度值 (最白): {max_val} (位置: {max_loc})")
    print(f"   - 平均灰度值 (Mean): {mean_val:.2f}")
    print(f"   - 中位灰度值 (Median): {median_val}")

    # 4. 计算灰度直方图 (0-255 每个灰度级有多少个像素)
    hist = cv2.calcHist([gray], [0], None, [256], [0, 256])

    # 打印部分区间的像素占比，帮你看清白纸在哪里集中
    print("\n📈 灰度区间像素分布简报:")
    print(f"   - 极暗区 (0-50   | 阴影/黑椅/深色地毯): {np.sum(hist[0:51]) / (h*w) * 100:.2f}%")
    print(f"   - 中间区 (51-180 | 灰色地面/杂物/椅腿)  : {np.sum(hist[51:181]) / (h*w) * 100:.2f}%")
    print(f"   - 高亮区 (181-255| A4白纸的主体范围)    : {np.sum(hist[181:256]) / (h*w) * 100:.2f}%")

    # 5. 如果你的环境支持 matplotlib，顺便保存一张直方图曲线图
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        plt.plot(hist, color='black')
        plt.title('Grayscale Histogram')
        plt.xlabel('Gray Level (0-255)')
        plt.ylabel('Pixel Count')
        plt.grid(True)
        plt.savefig('/var/tmp/grayscale_histogram_plot.png')
        print("💾 直方图曲线图已保存至: /var/tmp/grayscale_histogram_plot.png")
    except ImportError:
        print("💡 提示: 未安装 matplotlib，跳过曲线图生成（但文字统计数据已完整输出）。")

if __name__ == "__main__":
    # 默认读取你刚刚保存的这张调试原图
    analyze_grayscale("/var/tmp/task3_single_digit.jpg")