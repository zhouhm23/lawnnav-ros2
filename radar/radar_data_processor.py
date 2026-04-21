"""
RadarDataProcessor v10.2
功能：聚合雷达数据帧为数据立方体，支持动态chirp索引
修复：适配实际chirp发送模式、添加show参数、优化日志
修复：补充缺失的time模块导入（2025-11-10）
"""

# ==================== 标准库导入 ====================
import logging
import struct
import time
from collections import deque
from enum import IntEnum
from typing import Optional, Tuple, Dict, Any, List, Set

# ==================== 第三方库导入 ====================
import numpy as np


# ==================== 数据类型枚举 ====================
class RadarDataType(IntEnum):
    """雷达数据类型枚举 - 与 SerialPortConnector 保持一致"""
    UNKNOWN = 0
    DS_RAW = 0b010
    RANGE_FFT = 0b011
    DOPPLER_FFT = 0b100


# ==================== 主处理器类 ====================
# ==================== 主处理器类 ====================
class RadarDataProcessor:
    """
    雷达数据处理器 - 与 SerialPortConnector v9.3+ 完全兼容
    - 聚合多chirp数据为立方体 (samples, chirps, rx)
    - 支持动态chirp索引（适配固件实际发送模式）
    - 自动锁定数据格式，防止运行时切换
    - 传递并记录校验和状态
    - 跨chirp慢时间IIR滤波（针对串扰优化）
    """
    
    def __init__(self, num_chirps_to_aggregate: int, output_mode: str = "1dfft", 
                #  show: bool = False, alpha: float = 5e-1,alpha_2: float = 5e-1):
                 show: bool = False, alpha: float = 0.95, alpha_2: float = 0.95):
        """
        初始化处理器
        
        Args:
            num_chirps_to_aggregate: 每个数据立方体包含的chirp数量（必须与雷达配置一致）
            output_mode: 
                - "original": 输出原始解析数据（复数IQ）
                - "raw": 输出原始采样格式（保留时域特征）
                - "1dfft": 输出距离FFT结果，并应用慢时间IIR滤波
            show: 是否显示详细接收进度和警告信息（调试用）
            alpha: 慢时间IIR滤波器系数（0.9-0.98，值越大抑制越强）
        """
        if num_chirps_to_aggregate <= 0:
            raise ValueError("num_chirps_to_aggregate 必须大于 0")
            
        self.num_chirps_to_aggregate = num_chirps_to_aggregate
        self.output_mode = output_mode
        self.show = show
        self.iir_alpha = alpha
        self.alpha_2 = alpha_2
        
        # 状态锁
        self.is_locked = False
        self.locked_num_samples = None
        self.locked_data_type = None
        self.expected_channels = 1
        
        # 统计数据
        self.stats = {
            'total_frames': 0,
            'valid_frames': 0,
            'checksum_failures_in': 0,
            'checksum_failures_out': 0,
            'type_mismatches': 0,
            'size_mismatches': 0,
            'cubes_generated': 0,
            'chirp_missing_count': 0,
            'channel_sync_errors': 0,
        }
        
        # 数据缓冲区: {rx_channel: {chirp_index: {'data': np.ndarray, 'valid': bool}}}
        self.data_buffers = {}
        
        # 活跃通道集合
        self.active_channels = set()
        
        # 动态记录的chirp索引集合
        self.received_chirp_indices: Set[int] = set()
        
        # 日志
        self.logger = logging.getLogger(__name__)
        
        # 慢时间IIR状态（延迟初始化）
        self.Bn_slow = None  # 形状: (num_samples, num_channels)
        
    # ==================== 核心处理接口 ====================
    

    def process_frame(self, radar_seg: bytes, info: Dict[str, Any]) -> Optional[Tuple[np.ndarray, Dict]]:
        """
        处理单帧雷达数据 - 与 SerialPortConnector read_data_frame() 输出完全兼容
        
        Args:
            radar_seg: 雷达数据段（bytes），包含 Header + Data + Tail
            info: 帧信息字典，必须包含以下字段：
                - type: RadarDataType
                - rx_channel: int (0=RX1, 1=RX2)
                - chirp_index: int (Chirp序号)
                - frame_index: int (帧序号)
                - data_points: int (数据点数)
                - packet_length: int (帧总长度)
                - checksum_valid: bool (连接器已验证的校验和状态)
                - timestamp: float (接收时间戳)
        
        Returns:
            成功聚合时返回 (data_cube, metadata)，否则返回 None
            data_cube: np.ndarray，形状为 (num_samples, num_chirps, num_rx)
        """
        try:
            self.stats['total_frames'] += 1
            
            # 1. 基础验证
            if len(radar_seg) < 8:
                self.logger.warning(f"帧过短: {len(radar_seg)} bytes < 8")
                self.stats['size_mismatches'] += 1
                return None
                
            # 2. 提取并验证info必需字段
            required_keys = ['type', 'rx_channel', 'chirp_index', 'data_points', 'checksum_valid']
            for key in required_keys:
                if key not in info:
                    self.logger.error(f"info缺少必需字段: {key}")
                    return None
            
            # 3. 使用info中的元数据
            rx_channel = info['rx_channel']
            data_type = info['type']
            chirp_index = info['chirp_index']
            data_points = info['data_points']
            
            # 4. 记录校验和状态
            if not info.get('checksum_valid', True):
                self.stats['checksum_failures_in'] += 1
                #if self.show:
                    #self.logger.warning(f"收到校验和失败帧 - RX{rx_channel+1}, Chirp{chirp_index+1}")
            
            # 5. 精确计算实际数据点数
            payload = radar_seg[4:-4]  # 排除Header和Tail
            actual_points = len(payload) // 4
            
            # 6. 动态记录chirp索引模式
            if not self.received_chirp_indices:
                if self.show:
                    self.logger.info(f"首次chirp索引: {chirp_index}，开始记录模式")
            self.received_chirp_indices.add(chirp_index)
            # 防止索引集合过大 (假设正常索引在0-511之间)
            if len(self.received_chirp_indices) > 1000:
                 self.received_chirp_indices.clear()
                 self.received_chirp_indices.add(chirp_index)
            
            # 7. 首次锁定数据格式
            if not self.is_locked:
                self.locked_num_samples = self._lock_format(data_type, actual_points, rx_channel, info)
            
            # 8. 一致性检查
            if not self._check_consistency(data_type, actual_points, rx_channel):
                self._reset_buffers()
                return None
            
            # 9. 解析复数数据（严格大端序）
            try:
                iq = np.frombuffer(payload, dtype='>i2')  # 大端序有符号16位
                complex_data = iq[::2] + 1j * iq[1::2]   # Real + j*Imag
                
                if len(complex_data) != actual_points:
                    self.logger.error(f"复数解析点数不匹配: {len(complex_data)} != {actual_points}")
                    return None
                    
            except Exception as e:
                self.logger.error(f"复数解析失败: {e}")
                return None
            
            # 10. 输出模式处理
            processed_data = self._apply_output_mode(complex_data, data_type, rx_channel)

            # 如果处理后数据全为零，则视为无效（可能为设备端采样异常）
            try:
                if np.all(np.abs(processed_data) == 0):
                    self.stats['zero_payloads'] = self.stats.get('zero_payloads', 0) + 1
                    self.logger.warning(f"检测到全零有效载荷 (RX{rx_channel+1}, Chirp{chirp_index}), 丢弃此帧")
                    return None
            except Exception:
                pass

            # 11. 存入缓冲区
            self._store_to_buffer(rx_channel, chirp_index, processed_data, 
                                checksum_valid=info.get('checksum_valid', False))
            
            # 12. 检查聚合条件
            if not self._is_aggregation_ready():
                return None
            
            # 13. 构建数据立方体
            data_cube = self._build_cube()
            
            # 14. 生成元数据
            metadata = self._generate_metadata(info)
            
            # 15. 重置缓冲区（保留IIR状态）
            self._reset_buffers()
            
            self.stats['cubes_generated'] += 1
            self.logger.info(f"✓ 生成数据立方体 #{self.stats['cubes_generated']:04d}: {data_cube.shape}")
            
            # if input is raw and output mode is frequency domain notch, use apply_frequency_domain_notch
            # data_cube = self.apply_frequency_domain_notch(data_cube, axis=0, width_ratio=0.25)
            if self.output_mode == "1dfft" and data_type == RadarDataType.DS_RAW:
                data_cube = self.apply_frequency_domain_notch(data_cube, axis=1, width_ratio=0.25)
            

            
            
            return data_cube, metadata
            
        except Exception as e:
            self.logger.error(f"处理帧时发生未预期错误: {e}", exc_info=True)
            self._reset_buffers()
            return None
        
        
    
    def apply_frequency_domain_notch(self, cube, axis=1, width_ratio=0.25):
        """
        [重写逻辑] 频域硬阈值陷波 (Frequency Domain Hard Notch).
        
        逻辑:
        1. 对 Chirp 维做 FFT。
        2. 找到能量最强的频点 (杂波中心)。
        3. 将该中心周围一定宽度 (num_samples * width_ratio) 的频点直接置零。
        4. IFFT 恢复时域。
        
        参数:
        width_ratio: 切除带宽度的比例。
                     例如 0.125 (1/8) 表示如果 N=128，则切除中心周围约 16 个频点。
                     这是一种非常激进的去杂波方式，适合杂波泄露严重的移动雷达。
        """
        # 0. 数据清洗
        if np.any(np.isnan(cube)):
            cube = np.nan_to_num(cube, nan=0.0)

        n_samples = cube.shape[axis]
        
        # ---------------------------------------------------------
        # 1. FFT 进入频域
        # ---------------------------------------------------------
        spectrum = np.fft.fft(cube, axis=axis)
        
        # ---------------------------------------------------------
        # 2. 寻找杂波中心 (Global Peak Detection)
        # ---------------------------------------------------------
        # 使用全局平均能量 (L2 Norm) 锁定最强干扰源
        sum_axes = tuple(i for i in range(cube.ndim) if i != axis)
        # Mean(|X|^2)
        power_spectrum = np.mean(np.abs(spectrum)**2, axis=sum_axes)
        
        center_idx = np.argmax(power_spectrum)
        
        # ---------------------------------------------------------
        # 3. 构建陷波掩膜 (Notch Mask)
        # ---------------------------------------------------------
        # 计算需要切除的半宽度 (Radius)
        # 如果 width_ratio = 1/8, total_width = N/8, radius = N/16
        notch_width_total = int(n_samples * width_ratio)
        notch_radius = max(1, notch_width_total // 2) 
        
        # 创建掩膜，默认全为 1 (保留)
        mask = np.ones(n_samples, dtype=np.float32)
        
        # 计算需要置零的索引范围 (处理循环边界)
        # 比如 center=0, radius=2 -> indices=[-2, -1, 0, 1, 2] -> [126, 127, 0, 1, 2]
        indices = np.arange(center_idx - notch_radius, center_idx + notch_radius + 1)
        indices = indices % n_samples # 循环取模，防止越界
        
        # 将陷波区域置零
        mask[indices] = 0.0
        
        # ---------------------------------------------------------
        # 4. 应用掩膜并 IFFT
        # ---------------------------------------------------------
        # 调整 mask 形状以进行广播 (Broadcasting)
        # mask shape: (n_samples,) -> (1, n_samples, 1)
        reshape_dims = [1] * cube.ndim
        reshape_dims[axis] = n_samples
        mask_reshaped = mask.reshape(reshape_dims)
        
        # 频域相乘
        spectrum_filtered = spectrum * mask_reshaped
        
        # IFFT 恢复时域
        filtered_cube = np.fft.ifft(spectrum_filtered, axis=axis)
        
        return filtered_cube
    
    
    # ==================== 内部辅助方法 ====================
    def _lock_format(self, data_type: RadarDataType, num_samples: int, rx_channel: int, 
                    info: Dict[str, Any]):
        """首次锁定数据格式，初始化处理链"""
        if self.is_locked:
            return
            
        self.locked_data_type = data_type
        # self.locked_num_samples = num_samples
        self.is_locked = True
        
        # 动态通道数：RX2时启用双通道
        self.expected_channels = 2 
        
        self.logger.info("="*60)
        self.logger.info(f"【格式锁定】数据类型: {data_type.name}, 采样点: {num_samples}, 通道数: {self.expected_channels}")
        
        # DS_RAW模式专用初始化
        if data_type == RadarDataType.DS_RAW:
            # 快时间汉明窗
            self.hamming_window = np.hamming(num_samples).astype(np.float32)
            
            # 慢时间IIR状态初始化 (距离门 × 通道)
            self.Bn_slow = np.zeros((num_samples, self.expected_channels), dtype=np.complex64)
            
            self.logger.info(f"  IIR Alpha: {self.iir_alpha}, 窗长: {len(self.hamming_window)}")
            self.logger.info(f"  慢时间IIR状态: {self.Bn_slow.shape}")
        
        self.logger.info("="*60)
        return num_samples

    def _apply_output_mode(self, complex_data: np.ndarray, data_type: RadarDataType, 
                          channel: int) -> np.ndarray:
        """应用输出模式转换（核心：慢时间IIR）"""
        
        if self.output_mode != "1dfft":
            return complex_data
            
        # 硬件1DFFT直通
        if data_type == RadarDataType.RANGE_FFT:
            return complex_data
        
        # DS_RAW处理（慢时间IIR流程）
        if data_type == RadarDataType.DS_RAW:
            # 1. 快时间DC消除（逐Chirp，抑制静态偏置）
            dc_mean = np.mean(complex_data)
            data_no_dc = complex_data# - dc_mean
            
            # 2. 快时间加窗（降低旁瓣）
            windowed = data_no_dc * self.hamming_window
            
            # 3. 距离FFT
            range_fft = np.fft.fft(windowed)
            
            # 4. 慢时间IIR滤波（跨Chirp，针对串扰优化）
            # filtered = self._iir_filter_slow_time(range_fft, channel-1)
            
            # filtered = self.apply_frequency_domain_notch(range_fft, axis=1, width_ratio=0.25)
            
            
            return range_fft
        
        return complex_data

    # def _iir_filter_slow_time(self, data: np.ndarray, channel: int) -> np.ndarray:
    #     """
    #     慢时间IIR高通滤波器（跨chirp，串扰抑制核心）
    #     公式: Bn[n] = α*Bn[n-1] + (1-α)*X[n]
    #          Y[n] = X[n] - Bn[n]
        
    #     串扰抑制原理：
    #     - 串扰是跨chirp相干的低频分量（多普勒≈0Hz）
    #     - 慢时间IIR构建自适应背景估计，有效抑制静态/慢速干扰
    #     - 真实目标因多普勒频率被保留
    #     """
    #     if self.Bn_slow is None:
    #         self.logger.warning("IIR状态未初始化，跳过滤波")
    #         return data
            
    #     # 更新背景估计（指数加权累积）
    #     n = data.shape[0]
    #     mid = n // 2
        
    #     y = data.copy()
    #     # end = int(0.9*n)
    #     # 仅对正频率部分 (索引 0 .. mid-1) 更新慢时间背景估计，
    #     # 保留负频率 (mid .. n-1) 不滤波（即不更新 Bn_slow 的对应段）
        
    #     # if mid > 0:
    #     #     self.Bn_slow[:mid, channel] = (
    #     #     self.iir_alpha * self.Bn_slow[:mid, channel] +
    #     #     (1.0 - self.iir_alpha) * data[:mid]
    #     #     )
    #     #     y[:mid] = (data[:mid] - self.Bn_slow[:mid, channel]) / self.iir_alpha**1
            
    #     #     self.Bn_slow[mid:, channel] = (
    #     #     self.alpha_2 * self.Bn_slow[mid:, channel] +    
    #     #     (1.0 - self.alpha_2) * data[mid:]
    #     #     )
    #     #     y[mid:] = (data[mid:] - self.Bn_slow[mid:, channel]) / self.alpha_2**1
            

    #     # # 对于偶数长度，索引 mid 为奈奎斯特点，如需单独处理可在此处添加逻辑（当前保持不更新）
        
    #     # 返回高通结果（抑制串扰和静态杂波）
    #     return y
    #     return data
    #     return (data - self.Bn_slow[:, channel])#/self.iir_alpha
                
    def _check_consistency(self, data_type: RadarDataType, num_samples: int, 
                          rx_channel: int) -> bool:
        """全面一致性检查"""
        if data_type != self.locked_data_type:
            self.logger.error(f"数据类型改变: {self.locked_data_type} -> {data_type}")
            self.stats['type_mismatches'] += 1
            return False
        
        if num_samples != self.locked_num_samples:
            self.logger.error(f"采样点数改变: {self.locked_num_samples} -> {num_samples}")
            self.stats['size_mismatches'] += 1
            return False
        
        return True
    
    def _store_to_buffer(self, rx_channel: int, chirp_index: int, data: np.ndarray,
                        checksum_valid: bool):
        """存储数据到缓冲区"""
        if rx_channel not in self.data_buffers:
            self.data_buffers[rx_channel] = {}
        
        # 防止缓冲区无限增长（应对丢包或不同步导致的累积）
        # 允许一定余量（例如2倍帧大小），超过则清理最旧数据
        if len(self.data_buffers[rx_channel]) > self.num_chirps_to_aggregate * 2:
            # 移除最旧的键（假设chirp_index是循环递增的，这里简单移除最早加入的）
            # 由于Python字典3.7+保持插入顺序，iter(dict)返回的是插入顺序
            oldest_key = next(iter(self.data_buffers[rx_channel]))
            del self.data_buffers[rx_channel][oldest_key]
            
            if self.show and len(self.data_buffers[rx_channel]) % 10 == 0:
               self.logger.warning(f"RX{rx_channel+1} 缓冲区过大，丢弃旧chirp {oldest_key}")

        self.data_buffers[rx_channel][chirp_index] = {
            'data': data,
            'valid': checksum_valid,
            'chirp_index': chirp_index
        }
        self.active_channels.add(rx_channel)
        
        # 调试用：显示每个chirp存储状态
        if self.show:
            self.logger.debug(f"存储: RX{rx_channel+1}, Chirp{chirp_index+1:02d}, 长度{len(data)}, 校验和: {checksum_valid}")
    
    def _is_aggregation_ready(self) -> bool:
        """检查是否满足聚合条件"""
        if not self.active_channels:
            return False
        
        # 检查每个通道的chirp数是否达到要求
        for ch in self.active_channels:
            stored_count = len(self.data_buffers[ch])
            if stored_count < self.num_chirps_to_aggregate:
                if self.show and stored_count > 0:
                    stored_indices = sorted(self.data_buffers[ch].keys())
                    self.logger.info(f"RX{ch+1} 进度: {stored_count}/{self.num_chirps_to_aggregate} chirps, 索引: {stored_indices}")
                return False
        
        # 检查跨通道chirp索引同步
        if len(self.active_channels) > 1:
            ref_channel = min(self.active_channels)
            ref_indices = set(self.data_buffers[ref_channel].keys())
            
            for ch in self.active_channels:
                if ch == ref_channel:
                    continue
                current_indices = set(self.data_buffers[ch].keys())
                if current_indices != ref_indices:
                    self.stats['channel_sync_errors'] += 1
                    self.logger.error(f"通道chirp索引不匹配!")
                    self.logger.error(f"  RX{ref_channel+1} 索引: {sorted(ref_indices)}")
                    self.logger.error(f"  RX{ch+1} 索引: {sorted(current_indices)}")
                    self.logger.error(f"  缺失: {ref_indices - current_indices}")
                    return False
        
        # 达到聚合数量要求
        if self.show:
            for ch in self.active_channels:
                indices = sorted(self.data_buffers[ch].keys())
                self.logger.info(f"RX{ch+1} 准备聚合: {len(indices)} chirps, 索引: {indices}")
        
        return True
    
    def _build_cube(self) -> np.ndarray:
        """构建数据立方体 (samples, chirps, rx)"""
        try:
            rx_channels = sorted(self.active_channels)
            
            # 动态获取当前chirp索引
            ref_channel = rx_channels[0]
            chirp_indices = sorted(self.data_buffers[ref_channel].keys())
            
            # 验证是否为期望的chirp数量
            if len(chirp_indices) != self.num_chirps_to_aggregate:
                self.logger.warning(f"Chirp数量不匹配: {len(chirp_indices)} != {self.num_chirps_to_aggregate}")
                raise ValueError("Chirp数量不足")
            
            # 构建立方体
            cube_list = []
            for ch in rx_channels:
                # 按相同索引顺序提取数据
                chirp_data_list = []
                for idx in chirp_indices:
                    if idx not in self.data_buffers[ch]:
                        raise ValueError(f"RX{ch+1} 缺少chirp索引 {idx}")
                    
                    data_entry = self.data_buffers[ch][idx]
                    #if not data_entry['valid']:
                        #self.logger.warning(f"使用校验和失败帧: RX{ch+1}, Chirp{idx+1}")
                    
                    chirp_data_list.append(data_entry['data'])
                
                # 堆叠为 (samples, chirps)
                channel_cube = np.stack(chirp_data_list, axis=1)
                cube_list.append(channel_cube)
            
            # 最终形状: (samples, chirps, rx)
            data_cube = np.stack(cube_list, axis=2)
            
        except Exception as e:
            #self.logger.error(f"构建立方体失败: {e}")
            raise
        
        return data_cube
    
    def _generate_metadata(self, info: Dict[str, Any]) -> Dict[str, Any]:
        """生成完整元数据"""
        # 动态获取实际chirp索引
        if self.active_channels:
            ref_channel = min(self.active_channels)
            actual_chirp_indices = sorted(self.data_buffers[ref_channel].keys())
        else:
            actual_chirp_indices = []
        
        return {
            'data_type': self.locked_data_type,
            'rx_channels': sorted(self.active_channels),
            'num_samples': self.locked_num_samples,
            'num_chirps': self.num_chirps_to_aggregate,
            'actual_chirp_indices': actual_chirp_indices,
            'shape': (
                self.locked_num_samples, 
                self.num_chirps_to_aggregate, 
                len(self.active_channels)
            ),
            'output_mode': self.output_mode,
            'iir_alpha': self.iir_alpha,  # 新增：记录滤波器参数
            'timestamp': info.get('timestamp', time.time()),
            'frames_processed': self.stats['total_frames'],
            'checksum_failures_in': self.stats['checksum_failures_in'],
            'data_quality': {
                'total_frames': self.stats['total_frames'],
                'valid_frames': self.stats['total_frames'] - self.stats['checksum_failures_in'],
                'checksum_error_rate': self.stats['checksum_failures_in'] / max(self.stats['total_frames'], 1),
                'channel_sync_errors': self.stats['channel_sync_errors'],
            }
        }
    
    def _reset_buffers(self):
        """重置数据缓冲区（保持锁定状态和IIR状态）"""
        self.data_buffers.clear()
        self.active_channels.clear()
        self.received_chirp_indices.clear()
    
    # ==================== 控制接口 ====================
    def set_output_mode(self, mode: str):
        """设置输出模式"""
        if mode not in ["original", "raw", "1dfft"]:
            raise ValueError("output_mode 必须是 'original', 'raw' 或 '1dfft'")
        self.output_mode = mode
        self.logger.info(f"输出模式切换为: {mode}")
    
    def reset(self):
        """完全重置处理器状态（包括锁定和IIR状态）"""
        self.is_locked = False
        self.locked_num_samples = None
        self.locked_data_type = None
        
        # 重置IIR状态
        self.Bn_slow = None
        
        # 重置缓冲区
        self._reset_buffers()
        
        # 重置统计
        for k in self.stats:
            self.stats[k] = 0
        
        self.logger.info("处理器已完全重置")
    
    def reset_iir_state(self):
        """仅重置慢时间IIR滤波器状态（保留锁定和统计）"""
        if self.Bn_slow is not None:
            self.Bn_slow.fill(0)
            self.logger.info("慢时间IIR滤波器状态已重置（用于抑制发散或配置变更）")
    
    def get_stats(self) -> Dict[str, int]:
        """获取处理统计（线程安全）"""
        return self.stats.copy()

    def get_status(self) -> Dict[str, Any]:
        """获取完整状态（含数据质量和IIR状态）"""
        return {
            'is_locked': self.is_locked,
            'locked_type': self.locked_data_type.name if self.locked_data_type else None,
            'locked_samples': self.locked_num_samples,
            'iir_alpha': self.iir_alpha,
            'received_chirp_indices': sorted(self.received_chirp_indices),
            'stats': self.get_stats(),
            'active_channels': sorted(self.active_channels),
            'buffer_status': {
                ch: sorted(self.data_buffers[ch].keys()) if ch in self.data_buffers else []
                for ch in sorted(self.active_channels)
            },
            'iir_state_shape': self.Bn_slow.shape if self.Bn_slow is not None else None,
            'iir_state_mean': np.mean(np.abs(self.Bn_slow)) if self.Bn_slow is not None else 0.0
        }

# ==================== 异常定义 ====================
class DataIntegrityError(Exception):
    """数据完整性错误"""
    pass


# ==================== 实验程序 ====================
if __name__ == "__main__":
    # 注意：运行实验程序需要serial_port_connector模块
    import sys
    sys.path.append('.')  # 确保当前目录在搜索路径中
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 降低处理器日志级别（减少噪声）
    logging.getLogger('__main__').setLevel(logging.WARNING)
    
    from serial_port_connector import SerialPortConnector, RadarConfig
    
    # ==================== 用户配置区 ====================
    # 硬件配置
    PORT = '/dev/ttyACM2'  # Linux 
    BAUD = 921600
    
    # 处理配置（必须与雷达固件配置一致）
    # 重要：如果GUI中"每帧Chirp数"设为16，这里应为16而不是32
    NUM_CHIRPS = 32  # 修正：根据实际收到的chirp数量设置
    OUTPUT_MODE = "1dfft"  # "original", "raw", "1dfft"
    SHOW_PROGRESS = True  # 调试用：显示每个chirp接收状态
    
    # ==================== 主程序 ====================
    def main():
        print("="*80)
        print("EVB1122 雷达数据立方体生成测试 - v10.2")
        print("="*80)
        
        # 创建处理器（关键：添加show参数）
        processor = RadarDataProcessor(
            num_chirps_to_aggregate=NUM_CHIRPS,
            output_mode=OUTPUT_MODE,
            show=SHOW_PROGRESS  # 调试用：显示详细进度
        )
        
        # 创建连接器配置
        config = RadarConfig(data_type=RadarDataType.RANGE_FFT)
        
        cube_count = 0
        start_time = time.time()
        
        try:
            with SerialPortConnector(PORT, BAUD, config=config, skip_checksum=True) as conn:
                print(f"已连接至 {PORT}")
                print(f"配置: {NUM_CHIRPS} chirps/帧, 输出模式: {OUTPUT_MODE}")
                print(f"调试模式: {'开启' if SHOW_PROGRESS else '关闭'}")
                print("按 Ctrl+C 停止\n")
                
                while True:
                    # 1. 读取数据帧
                    result = conn.read_data_frame(timeout=30.0, show=False)
                    
                    if result is None:
                        print("读取超时，退出")
                        break
                    
                    radar_seg, info = result
                    
                    # 2. 处理数据
                    process_result = processor.process_frame(radar_seg, info)
                    
                    # 3. 检查是否生成立方体
                    if process_result is not None:
                        data_cube, metadata = process_result
                        cube_count += 1
                        
                        # 4. 显示结果
                        elapsed = time.time() - start_time
                        print(f"\n{'='*80}")
                        print(f"【数据立方体 #{cube_count:04d}】")
                        print(f"生成时间: {elapsed:.3f} 秒")
                        print(f"元数据:")
                        for k, v in metadata.items():
                            if k == 'data_quality':
                                print(f"  {k}:")
                                for qk, qv in v.items():
                                    print(f"    {qk}: {qv}")
                            else:
                                print(f"  {k}: {v}")
                        print(f"数据形状: {data_cube.shape}")
                        print(f"数据类型: {data_cube.dtype}")
                        print(f"数据范围: Real[{data_cube.real.min():.1f}, {data_cube.real.max():.1f}]")
                        print(f"          Imag[{data_cube.imag.min():.1f}, {data_cube.imag.max():.1f}]")
                        print(f"第一个样本: {data_cube[0, 0, 0]}")
                        print(f"{'='*80}\n")
                        
                        # 5. 重置计时
                        start_time = time.time()
        
        except KeyboardInterrupt:
            print("\n\n用户中断")
        except Exception as e:
            print(f"\n发生错误: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            print("\n" + "="*80)
            print("测试结束 - 最终统计:")
            print("-" * 40)
            
            print("【处理器统计】:")
            stats = processor.get_stats()
            for k, v in stats.items():
                print(f"  {k}: {v}")
            
            print(f"\n  数据立方体总数: {cube_count}")
            
            print("\n【处理器最终状态】:")
            status = processor.get_status()
            for k, v in status.items():
                if isinstance(v, dict):
                    print(f"  {k}:")
                    for vk, vv in v.items():
                        print(f"    {vk}: {vv}")
                else:
                    print(f"  {k}: {v}")
            
            # 显示chirp接收模式分析
            if status.get('received_chirp_indices'):
                indices = status['received_chirp_indices']
                print(f"\n【Chirp索引模式分析】:")
                print(f"  收到不同索引数: {len(indices)}")
                print(f"  索引列表: {indices}")
                print(f"  是否为偶数序列: {all(i % 2 == 0 for i in indices)}")
                print(f"  建议NUM_CHIRPS设置: {len(indices)}")
            
            print("="*80)
    
    if __name__ == "__main__":
        main()