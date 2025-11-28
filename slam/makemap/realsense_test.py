#!/usr/bin/env python3
"""
Quick non-ROS test for Intel RealSense on the robot.
Prints whether color/depth frames are present, points.size(), and the first few vertices.
Run with: python3 realsense_test.py
"""

import pyrealsense2 as rs
import time

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)

    try:
        profile = pipeline.start(config)
        print('管道启动成功')
        time.sleep(0.2)

        # 获取一帧
        frames = pipeline.wait_for_frames(timeout_ms=2000)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()

        print('color_frame:', 'OK' if color else 'None')
        print('depth_frame:', 'OK' if depth else 'None')

        if not color or not depth:
            print('未获取到 color 或 depth 帧，检查相机连接和分辨率/帧率设置。')
            return

        # 计算点云
        pc = rs.pointcloud()
        try:
            pc.map_to(color)
        except Exception as e:
            print('pc.map_to(color) 发生异常:', e)

        points = pc.calculate(depth)

        # 尝试获取 size
        size_attr = getattr(points, 'size', None)
        pts_count = None
        try:
            if callable(size_attr):
                pts_count = int(size_attr())
            elif size_attr is not None:
                pts_count = int(size_attr)
        except Exception as e:
            print('读取 points.size 时异常:', e)

        # 备用：尝试通过遍历顶点来计数与采样
        vertices = None
        try:
            vertices = points.get_vertices()
            # 仅采样前几个顶点作为示例（不要展开整个迭代器以节省内存）
            sample = []
            count = 0
            for v in vertices:
                if count < 8:
                    sample.append((float(v.x), float(v.y), float(v.z)))
                count += 1
                # 如果点很多，限制遍历长度以便快速返回
                if count > 100000:
                    break
            if pts_count is None:
                pts_count = count
            print('通过遍历得到的顶点数(上限100000):', count)
            print('顶点样例(最多8个):')
            for s in sample:
                print('  ', s)
        except Exception as e:
            print('points.get_vertices() 发生异常:', e)

        print('最终推断的 pts_count =', pts_count)

    except Exception as e:
        print('运行时出错:', e)
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass

if __name__ == '__main__':
    main()
