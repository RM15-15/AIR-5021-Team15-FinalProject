import cv2
import numpy as np
from sklearn.cluster import DBSCAN
import heapq
import math


def is_same_point(pt1, pt2, threshold=5):
    """
    判断两个点是否相同，允许一定的误差
    :param pt1: 第一个点 (x, y)
    :param pt2: 第二个点 (x, y)
    :param threshold: 允许的误差范围
    :return: bool
    """
    return abs(pt1[0] - pt2[0]) <= threshold and abs(pt1[1] - pt2[1]) <= threshold

def is_connected(ptr1, ptr2, binary_image):
    x1, y1 = ptr1[0], ptr1[1]
    x2, y2 = ptr1[0], ptr1[1]
    if x1 > x2:
        x1, x2 = x2, x1
    if y1 > y2:
        y1, y2 = y2, y1

    bias_x = x2 - x1
    bias_y = y2 - y1
    if bias_x < 10 or bias_y < 10:
        if bias_x > bias_y:
            for i in range(x1, x2):
                if binary_image[i, y1] == 0:
                    return False
            return True
        else:
            for i in range(y1, y2):
                if binary_image[x1, i] == 0:
                    return False
            return True


def heuristic(node, goal):
    # 这里使用曼哈顿距离作为启发式函数
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])


def astar_search(graph, start, goal, corner_ptr):
    # 将start和goal转换为图中的索引
    start_idx = corner_ptr.index(start)
    goal_idx = corner_ptr.index(goal)

    # 优先队列，元素为 (f_score, g_score, current_node, path)
    open_set = []
    heapq.heappush(open_set, (0, 0, start_idx, [start_idx]))

    # 已访问的节点及其g_score
    g_scores = {start_idx: 0}

    while open_set:
        _, g_score, current, path = heapq.heappop(open_set)

        if current == goal_idx:
            # 返回路径中的实际节点
            return [corner_ptr[i] for i in path]

        for neighbor in range(len(graph)):
            if graph[current][neighbor] == 1:  # 有连接
                tentative_g_score = g_score + 1  # 假设每步代价为1

                if neighbor not in g_scores or tentative_g_score < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(corner_ptr[neighbor], corner_ptr[goal_idx])
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor, path + [neighbor]))

    return None  # 没有找到路径


def find_maze_entry_exit(binary_img):
    h, w = binary_img.shape
    entry_exit = []

    # 定义四个边的坐标访问方式
    borders = {
        "top":    [(0, x) for x in range(w)],
        "bottom": [(h-1, x) for x in range(w)],
        "left":   [(y, 0) for y in range(h)],
        "right":  [(y, w-1) for y in range(h)],
    }

    for side, pixels in borders.items():
        white_coords = [pt for pt in pixels if binary_img[pt] == 255]
        if white_coords:
            # 找中点坐标
            coords_np = np.array(white_coords)
            y_mean = int(np.mean(coords_np[:, 0]))
            x_mean = int(np.mean(coords_np[:, 1]))
            entry_exit.append((x_mean, y_mean))  # 注意：OpenCV中坐标是(x, y)

    return entry_exit  # 最多返回两个点：入口和出口
def show_image(title, img, scale=1.0):
    h, w = img.shape[:2]
    resized = cv2.resize(img, (int(w*scale), int(h*scale)))
    cv2.imshow(title, resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def cluster_points(points, eps=10, min_samples=1):
    """
    使用DBSCAN聚类点集
    :param points: 输入点集，形状为 (N,2)
    :param eps: 邻域半径（像素）
    :param min_samples: 最小簇大小
    :return: 聚类中心点
    """
    points = np.squeeze(points, axis=1) if points.ndim == 3 else points
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)

    # 提取每个簇的中心点
    clustered_points = []
    for label in np.unique(clustering.labels_):
        if label == -1:
            continue  # 忽略噪声点
        cluster = points[clustering.labels_ == label]
        clustered_points.append(cluster.mean(axis=0))  # 取簇内均值

    return np.array(clustered_points).reshape(-1, 1, 2)

def extract_dots(thresh, width):
    kernel = np.ones((width, width), np.uint8)
    # 3. 细化前：适当腐蚀使路径变细
    thresh_eroded = cv2.erode(thresh, kernel, iterations=1)
    output_path = "output_image.jpg"
    cv2.imwrite(output_path, thresh_eroded)
    # show_image('Cleaned + Eroded', thresh_eroded, 0.3)
    contours, _ = cv2.findContours(thresh_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    epsilon = 10  # 调节精度，值越小拐点越多
    approx = cv2.approxPolyDP(contours[0], epsilon, closed=False)
    clustered_approx = cluster_points(approx, eps=20, min_samples=1)
    return clustered_approx

def coordinate_transform(point, h, w):
    """
    将坐标转换为 (x, y) 形式
    :param points: 输入点集，形状为 (N,2)
    :return: 转换后的点集
    """
    x = point[1] / h * 0.35
    y = point[0] / w * 0.5 - 0.25
    return (x, y)

if __name__ == "__main__":
    img = cv2.imread('maze.png')
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    exit_ptrs = find_maze_entry_exit(thresh)
    start_ptr = exit_ptrs[0]
    end_ptr = exit_ptrs[1]

    print("exit_ptrs:", exit_ptrs)
    width = (exit_ptrs[0][0] - exit_ptrs[0][1]) // 3 * 2
    print("width:", width)
    clustered_approx = extract_dots(thresh, width)
    corner_ptr = [tuple(x[0]) for x in clustered_approx]
    for ptr in corner_ptr:
        if is_same_point(ptr, start_ptr):
            start_ptr = ptr
        if is_same_point(ptr, end_ptr):
            end_ptr = ptr
    corner_ptr.remove(start_ptr)
    corner_ptr.remove(end_ptr)
    graph = [[0] for _ in range(len(corner_ptr))]

    for i in range(len(corner_ptr)):
        for j in range(i, len(corner_ptr)):
            if i != j and is_connected(corner_ptr[i], corner_ptr[j], thresh):
                graph[i][j] = 1

    path = astar_search(graph,start_ptr, end_ptr, corner_ptr)
    path = [coordinate_transform(ptr, h, w) for ptr in path]
    print(path)
    with open("point.txt", "w") as f:
        for pt in path:
            f.write(f"{pt[0]} {pt[1]}\n")
    print("路径点已写入 point.txt")

