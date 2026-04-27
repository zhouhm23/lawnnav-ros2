import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import threading
import queue
import time
import logging
import serial.tools.list_ports
from scipy import signal
from math import ceil, floor

# 引入本地模块
try:
    from radar_data_processor import RadarDataProcessor, RadarDataType
    from serial_port_connector import SerialPortConnector, RadarConfig
except ImportError:
    # print("缺少必要的本地模块: radar_data_processor 或 serial_port_connector")
    # 为了保证代码在无硬件环境下不报错，保留此结构，实际使用请确保文件存在
    pass

# ==================== 全局日志配置 ====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# 字体配置 — 自动回退到可用的 CJK 字体，避免缺字警告
import matplotlib.font_manager as _fm
preferred_fonts = [
    'Microsoft YaHei',
    'WenQuanYi Micro Hei',
    'WenQuanYi Zen Hei',
    'Noto Sans CJK SC',
    'Noto Sans CJK JP',
    'DejaVu Sans'
]
available_fonts = set(f.name for f in _fm.fontManager.ttflist)
for _f in preferred_fonts:
    if _f in available_fonts:
        font_name = _f
        break
else:
    font_name = 'DejaVu Sans'
plt.rcParams['font.family'] = font_name
plt.rcParams['font.sans-serif'] = [font_name, 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 9

# ==================== 配置类定义 ====================
class RadarSignalConfig:
    """
    雷达信号处理配置类
    """
    def __init__(self):
        # --- 硬件与串口 ---
        self.PORT = '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_336235763332-if00'
        self.BAUD = 921600
        
        # --- 系统控制 ---
        self.REFRESH_INTERVAL = 100  # 动画刷新间隔 (ms)
        self.ENABLE_ANCHOR_CANCELLATION = True # 是否开启锚点消除
        
        # --- M-Sequence 参数 ---
        self.M_ORDER = 5
        self.CHIP_RATE_RATIO = 0.25  # fs/4
        self.FN2_RATIO = 0.5         # fs/2
        
        # --- 通信解码参数 (Updated from test.m) ---
        self.FFT_BIN_SEARCH = 50.0   # 消除时的搜索范围
        self.FILTER_WIDTH_RATIO = 0.05 # 滤波器宽度占总带宽的比例
        self.BIT_THRESHOLD_BINS = 1  # 判决阈值 (Freq Offset Bins)
        
        # --- 成像参数 ---
        self.ANGLE_RANGE = 60
        self.ANGLE_BINS = 61 
        
        # --- 迭代探测参数 ---
        self.MAX_ANCHOR_SEARCH_LIMIT = 2 # 最大迭代消除次数
        self.STOP_ENERGY_PCT = 0.01      # 能量变化停止阈值 (0.01%)
        
        self.f1_bin_idx = 2  # (Legacy)

# ==================== 核心处理类：锚点处理器 (基于 test.m 重构) ====================
class AnchorProcessor:
    def __init__(self, config: RadarSignalConfig):
        self.config = config
        self.m_seq_raw = self._generate_m_sequence(config.M_ORDER)
        self.cache = {} # 缓存预计算的Code Spectrum
        self.f1_bin_idx = config.f1_bin_idx

    def process(self, cube_raw):
        """
        基于 test.m 的迭代消除处理管道
        Input: cube_raw (n_samples, n_chirps, n_rx) (Frequency Domain)
        Returns: cube_clean, anchor_map_display, comm_results, anchor_info
        """
        n_samples, n_chirps, n_rx = cube_raw.shape
        
        # 1. 预计算/获取参考码 (Prepare Reference Code)
        code_seq, code_spec_conj = self._get_reference_code(n_samples)
        
        # 初始化状态变量
        cube_current = cube_raw.copy()
        energy0 = np.sum(np.abs(cube_raw)**2)
        
        # 用于UI显示的结果容器
        first_iter_anchor_map = None
        first_iter_comm_results = {'bits': np.zeros(n_chirps), 'offsets': np.zeros(n_chirps)}
        first_iter_info = {'freq_bin': 0, 'angle_deg': 0, 'amp': 0}
        
        # 迭代循环
        for iteration in range(self.config.MAX_ANCHOR_SEARCH_LIMIT):
            # [Step 1] 锚点定位 (R-H Map)
            rH_2d_map, current_center, _, mask_minus = self._solve_iterative_anchor(cube_current)
            
            # 分析峰值
            f_est_idx, peak_angle, peak_mag_lin = self._analyze_anchor_peak(rH_2d_map)
            
            # 如果是第一次迭代，保存用于显示的信息
            if iteration == 0:
                first_iter_anchor_map = rH_2d_map 
                
                # 计算显示的频率 Bin (Centered)
                freq_display = f_est_idx if f_est_idx <= n_samples//2 else f_est_idx - n_samples//2
                
                first_iter_info = {
                    'freq_bin': freq_display,
                    'angle_deg': peak_angle,
                    'amp': peak_mag_lin
                }

            # [Step 2] 延迟估计 (Lag Estimation)
            pilot_idx = int(round((f_est_idx + n_samples * self.config.FN2_RATIO) % n_samples))
            
            code_cube, lags = self._step_lag_estimation(
                cube_current, mask_minus, pilot_idx, code_seq, code_spec_conj
            )
            
            # [Optimization] 预计算解扩后的频谱 (Reuse Data for Step 3 & 4)
            # 避免在 Decoding 和 Cancellation 中重复计算 IFFT/FFT
            cube_t = np.fft.ifft(cube_current, axis=0)
            signal_decoded_time = cube_t * code_cube
            signal_decoded_spec = np.fft.fft(signal_decoded_time, axis=0)

            # [Step 3] 通信解码 (Communication Decoding - Using Cached Data)
            comm_bits, freq_offsets = self._step_comm_decoding(
                signal_decoded_spec, f_est_idx, n_samples
            )
            
            if iteration == 0:
                first_iter_comm_results['bits'] = comm_bits
                first_iter_comm_results['offsets'] = freq_offsets

            # [Step 4] 信号消除 (Cancellation - Using Cached Data)
            if self.config.ENABLE_ANCHOR_CANCELLATION:
                cube_current, reduce_pct = self._step_cancellation(
                    cube_current, code_cube, pilot_idx, f_est_idx, n_samples, energy0,
                    signal_decoded_spec=signal_decoded_spec # 传入预计算数据
                )
                
                # 终止条件: 能量变化极小
                if reduce_pct < self.config.STOP_ENERGY_PCT:
                    break
            else:
                break
                
        # 异常保护
        if first_iter_anchor_map is None:
             first_iter_anchor_map = np.zeros((n_samples, self.config.ANGLE_BINS))
             
        return cube_current, first_iter_anchor_map, first_iter_comm_results, first_iter_info, comm_bits

    # ================= 核心算法函数 =================

    def _solve_iterative_anchor(self, cube):
        """
        Refactored to match test.m logic strictly
        包含 N/4 模糊度判断 (Aliasing Check)
        """
        n_samples = cube.shape[0]
        n_half = n_samples // 2
        chip_band = int(round(n_samples * self.config.CHIP_RATE_RATIO))
        
        # 1. 初始全范围搜索
        mask_plus = np.zeros(n_samples)
        mask_plus[:n_half] = 1
        mask_minus = np.zeros(n_samples)
        mask_minus[n_half:] = 1
        
        # 计算初始 rH
        _, rH_freq_init = self._compute_rH_map_masked(cube, mask_plus, mask_minus)
        
        # 寻找频谱峰值确定粗略中心
        spec_mag = np.sum(np.abs(rH_freq_init), axis=(1, 2))
        idx_highest = np.argmax(spec_mag) # 0-based index
        
        if idx_highest < n_half:
            new_center = idx_highest / 2.0
        else:
            new_center = (idx_highest - n_half) / 2.0
            
        # 2. 准备 N/4 模糊度测试 (Hypothesis Testing)
        center1 = int(round(max(0, new_center)))
        center2 = int(round(center1 + n_samples / 4.0)) 
        
        chip_band_comp = int(round(chip_band / 2.0))
        
        # --- Hypothesis 1 ---
        rp1 = [center1 - chip_band_comp, center1 + chip_band_comp]
        rm1 = [rp1[0] + n_half, rp1[1] + n_half]
        mp1 = self._create_circular_mask(n_samples, rp1)
        mm1 = self._create_circular_mask(n_samples, rm1)
        rH_map1, _ = self._compute_rH_map_masked(cube, mp1, mm1)
        
        # --- Hypothesis 2 ---
        rp2 = [center2 - chip_band_comp, center2 + chip_band_comp]
        rm2 = [rp2[0] + n_half, rp2[1] + n_half]
        mp2 = self._create_circular_mask(n_samples, rp2)
        mm2 = self._create_circular_mask(n_samples, rm2)
        rH_map2, _ = self._compute_rH_map_masked(cube, mp2, mm2)
        
        # --- 决策 ---
        max1 = np.max(np.abs(rH_map1))
        max2 = np.max(np.abs(rH_map2))
        
        if max1 >= max2:
            final_center= center1
            final_rp = [center1 - chip_band, center1 + chip_band]
            final_rm = [final_rp[0] + n_half, final_rp[1] + n_half]
        else:
            final_center = center2
            final_rp = [center2 - chip_band, center2 + chip_band]
            final_rm = [final_rp[0] + n_half, final_rp[1] + n_half]
            
        final_mp = self._create_circular_mask(n_samples, final_rp)
        final_mm = self._create_circular_mask(n_samples, final_rm)
        
        # 3. 最终计算
        final_map, rH_freq_final = self._compute_rH_map_masked(cube, final_mp, final_mm)
        
        # 更新精确中心
        # spec_mag_final = np.sum(np.abs(rH_freq_final), axis=(1, 2))
        # idx_highest_final = np.argmax(spec_mag_final)
        
        # if idx_highest_final < n_half:
        #     refined_center = idx_highest_final / 2.0
        # else:
        #     refined_center = (idx_highest_final - n_half) / 2.0
        # print(f"final center{final_center}")
        return final_map, final_center, rH_freq_final, final_mm

    def _step_lag_estimation(self, cube_in, mask_minus, pilot_idx, code_seq, code_spec_conj):
        """
        Lag 估计
        """
        n_samples, n_chirps, _ = cube_in.shape
        
        # --- 1. 快速相关 ---
        # 提取负频带 (Only Rx0 used for estimation)
        # r_minus_spec = cube_in[:, :, 0] * mask_minus[:, None]
        r_minus_spec = cube_in[:, :, 0] # --- IGNORE ---
        
        # Frequency Domain Shift
        X_sig = np.roll(r_minus_spec, -pilot_idx, axis=0)
        
        # Freq Domain Correlation
        corr_circ = np.fft.ifft(X_sig * code_spec_conj[:, None], axis=0)
        
        # Find peaks
        chirp_lags = np.argmax(np.abs(corr_circ), axis=0)
        
        # --- 2. 向量化生成 Code Cube ---
        raw_indices = np.arange(n_samples)[:, None] - chirp_lags[None, :]
        code_indices = np.mod(raw_indices, n_samples)
        
        code_mat = code_seq[code_indices]
        code_cube = code_mat[:, :, None] # (N, C, 1)
            
        return code_cube, chirp_lags

    def _step_comm_decoding(self, signal_decoded_spec, f_est_idx, n_samples):
        """
        通信解码 - 使用预计算的 Decoded Spectrum 进行复用
        Input: signal_decoded_spec (n_samples, n_chirps, n_rx)
        """
        n_chirps = signal_decoded_spec.shape[1]
        
        # 初始化结果
        bits = np.zeros(n_chirps, dtype=int)
        freq_offsets = np.zeros(n_chirps, dtype=float)
        
        # 参数计算
        delta_k = int(round(self.config.FN2_RATIO * n_samples))
        filter_width = int(round(n_samples * self.config.FILTER_WIDTH_RATIO))
        if filter_width < 3: filter_width = 3
        
        # 搜索范围定义 (用于 Product 谱的 DC 附近搜索)
        search_radius = 8
        dc_indices_pos = np.arange(0, search_radius + 1)
        dc_indices_neg = np.arange(n_samples - search_radius, n_samples)
        dc_search_indices = np.concatenate((dc_indices_pos, dc_indices_neg)).astype(int)
        
        # 载波搜索范围 (在估计的 f_est_idx 附近搜索精确峰值)
        search_width = 10
        center_idx_approx = int(round(f_est_idx))
        carrier_search_range = np.arange(center_idx_approx - search_width, center_idx_approx + search_width + 1)
        carrier_search_range = np.mod(carrier_search_range, n_samples).astype(int)

        for c in range(n_chirps):
            # 1. 直接使用预计算的 Rx0 频谱 (Resource Reuse)
            r_spec = signal_decoded_spec[:, c, 0]
            
            # 2. 动态寻找 Carrier 峰值 (在此 Chirp 内最强点)
            spec_in_range = np.abs(r_spec[carrier_search_range])
            local_peak_idx = np.argmax(spec_in_range)
            carrier_idx = carrier_search_range[local_peak_idx]
            
            # 3. 计算 Sideband 索引 (Carrier - Delta_K)
            sideband_idx = (carrier_idx - delta_k) % n_samples
            
            # 4. 提取 Carrier 和 Sideband 分量 (Bandpass Masking)
            mask_c = self._create_bandpass_mask(n_samples, carrier_idx, filter_width)
            mask_s = self._create_bandpass_mask(n_samples, sideband_idx, filter_width)
            
            spec_c = r_spec * mask_c
            spec_s = r_spec * mask_s
            
            # 5. 移至基带 (Baseband Shift)
            spec_c_bb = np.roll(spec_c, -carrier_idx)
            spec_s_bb = np.roll(spec_s, -sideband_idx)
            
            # 6. 时域共轭乘积 (Dual Channel Product)
            time_c = np.fft.ifft(spec_c_bb)
            time_s = np.fft.ifft(spec_s_bb)
            
            phasor_prod = time_s * np.conj(time_c)
            spec_prod = np.fft.fft(phasor_prod)
            
            # 7. 检测 Product 谱中的峰值
            spec_prod_search = np.abs(spec_prod[dc_search_indices])
            peak_local_idx = np.argmax(spec_prod_search)
            peak_global_idx = dc_search_indices[peak_local_idx]
            
            # 8. 计算频偏 (Signed Bins)
            if peak_global_idx <= n_samples // 2:
                current_offset = float(peak_global_idx)
            else:
                current_offset = float(peak_global_idx - n_samples)
            
            freq_offsets[c] = current_offset
            
            # 9. 通信判决
            if abs(current_offset) > self.config.BIT_THRESHOLD_BINS:
                bits[c] = 1
            else:
                bits[c] = 0
                
        return bits, freq_offsets

    def _step_cancellation(self, cube_in, code_cube, pilot_idx, f_est_idx, n_samples, energy0, signal_decoded_spec=None):
        """
        信号消除 (根据用户要求采用特定逻辑)
        """
        fft_bin_search = int(self.config.FFT_BIN_SEARCH)
        
        # 如果未传入预计算的 Spectrum，则在此计算 (Fallback)
        # 如果传入了 signal_decoded_spec，则复用，不再重复计算
        if signal_decoded_spec is None:
            cube_t = np.fft.ifft(cube_in, axis=0)
            signal_decoded_time = cube_t * code_cube
            signal_decoded_spec = np.fft.fft(signal_decoded_time, axis=0)

        # # 1. 创建 Mask (Pilot + Data)
        # # 优化: 使用 bool 数组操作代替 set 循环，提升速度
        # mask_all = np.zeros(n_samples, dtype=float)
        
        # offsets = np.arange(-fft_bin_search, fft_bin_search + 1)
        # pilot_indices = np.mod(np.round(pilot_idx) + offsets, n_samples).astype(int)
        # est_indices = np.mod(np.round(f_est_idx) + offsets, n_samples).astype(int)
        
        # mask_all[pilot_indices] = 1.0
        # mask_all[est_indices] = 1.0
        # mask_all = mask_all.reshape(-1, 1, 1)
        
        
                
        # 1. 创建 Mask (Pilot + Data) — 使用带通掩码函数 _create_bandpass_mask
        # 基于 fft_bin_search 定义掩码宽度，合并 pilot 与估计频点的带通
        bin_width = int(round(2 * fft_bin_search + 1))
        if bin_width < 3:
            bin_width = 3
        mask_p = self._create_bandpass_mask(n_samples, int(round(pilot_idx)), bin_width)
        mask_e = self._create_bandpass_mask(n_samples, int(round(f_est_idx)), bin_width)
        # 合并并扩展为 (N,1,1) 以便与频谱广播
        mask_all = ((mask_p + mask_e) > 0).astype(float).reshape(-1, 1, 1)
        
        # 2. 提取并重构 (Filtering in Decoded Domain)
        # 直接使用 Decoded Spectrum 进行 Mask 操作
        signal_filtered_spec = signal_decoded_spec * mask_all
        
        signal_reconstructed_time = np.fft.ifft(signal_filtered_spec, axis=0)
        signal_reconstructed_final = signal_reconstructed_time * code_cube
        
        # 3. 消除
        signal_reconstructed_freq = np.fft.fft(signal_reconstructed_final, axis=0)
        cube_out = cube_in - signal_reconstructed_freq
        
        # 计算能量移除比
        energy_removed = np.sum(np.abs(signal_reconstructed_final)**2)
        reduce_pct = (energy_removed / energy0) * 100
        
        return cube_out, reduce_pct

    # ================= 辅助函数 =================
    
    def _create_bandpass_mask(self, n_samples, center_idx, width):
        """
        创建带通Mask (复刻 test.m: create_bandpass_mask)
        """
        mask = np.zeros(n_samples, dtype=float)
        half_w = width // 2
        # indices = (center_idx - half_w) : (center_idx + half_w)
        indices = np.arange(center_idx - half_w, center_idx + half_w + 1)
        indices = np.mod(indices, n_samples).astype(int)
        mask[indices] = 1.0
        return mask

    def _generate_m_sequence(self, order):
        taps_map = {5: [2, 5], 6: [1, 6], 7: [3, 7]}
        taps = taps_map.get(order, [2, 5])
        n_len = 2**order - 1
        state = np.ones(order, dtype=int)
        seq = []
        for _ in range(n_len):
            seq.append(state[-1])
            feedback = np.sum(state[[t-1 for t in taps]]) % 2
            state = np.roll(state, 1)
            state[0] = feedback
        return np.array(seq) * 2 - 1

    def _get_reference_code(self, n_samples):
        if n_samples in self.cache:
            return self.cache[n_samples]
            
        samples_per_chip = 1 / self.config.CHIP_RATE_RATIO
        m_seq_shaped = np.kron(self.m_seq_raw, np.ones(int(samples_per_chip)))
        repeats = int(np.ceil(n_samples / len(m_seq_shaped)))
        temp_seq = np.tile(m_seq_shaped, repeats)
        code_seq = temp_seq[:n_samples]
        
        # 预计算频谱共轭
        code_spec_conj = np.conj(np.fft.fft(code_seq))
        
        self.cache[n_samples] = (code_seq, code_spec_conj)
        return code_seq, code_spec_conj

    def _create_circular_mask(self, n_samples, range_limits):
        mask = np.zeros(n_samples)
        start = int(round(range_limits[0]))
        stop = int(round(range_limits[1]))
        length = stop - start + 1
        indices = (np.arange(length) + start) % n_samples
        mask[indices.astype(int)] = 1
        return mask

    def _compute_rH_freq_only(self, cube, mask_plus, mask_minus):
        # Broadcast masks
        mask_plus_bc = mask_plus.reshape(-1, 1, 1)
        mask_minus_bc = mask_minus.reshape(-1, 1, 1)
        
        r_plus_spec = cube * mask_plus_bc
        r_minus_spec = cube * mask_minus_bc
        
        r_plus_time = np.fft.ifft(r_plus_spec, axis=0)
        r_minus_time = np.fft.ifft(r_minus_spec, axis=0)
        
        r_H_time = r_minus_time * r_plus_time
        r_H_freq = np.fft.fft(r_H_time, axis=0)
        return r_H_freq

    def _compute_rH_map_masked(self, cube, mask_plus, mask_minus):
        r_H_freq = self._compute_rH_freq_only(cube, mask_plus, mask_minus)
        
        n_samples, n_chirps, n_rx = r_H_freq.shape
        n_angle_bins = self.config.ANGLE_BINS
        
        if n_rx < 2:
            angle_map = np.zeros((n_samples, n_angle_bins))
            mag_sum = np.mean(np.abs(r_H_freq), axis=(1,2))
            center = n_angle_bins // 2
            angle_map[:, center] = mag_sum
        else:
            angle_bins = np.linspace(-self.config.ANGLE_RANGE, self.config.ANGLE_RANGE, n_angle_bins)
            angle_map = np.zeros((n_samples, n_angle_bins))
            
            ch1 = r_H_freq[:, :, 0]
            ch2 = r_H_freq[:, :, 1]
            phase_diff = np.angle(ch1 * np.conj(ch2))
            
            ratio = phase_diff / (2 * np.pi)
            ratio = np.clip(ratio, -1, 1)
            angle_deg = np.degrees(np.arcsin(ratio))
            
            idx_float = (angle_deg - angle_bins[0]) / (angle_bins[-1] - angle_bins[0]) * (n_angle_bins - 1)
            idx_vec = np.round(idx_float).astype(int)
            idx_vec = np.clip(idx_vec, 0, n_angle_bins - 1)
            
            mag = np.abs(ch1)
            
            rows = np.arange(n_samples)[:, None]
            flat_indices = rows * n_angle_bins + idx_vec
            
            angle_map_flat = angle_map.ravel()
            np.add.at(angle_map_flat, flat_indices.ravel(), mag.ravel())
            angle_map = angle_map_flat.reshape(n_samples, n_angle_bins)
            
            if n_chirps > 1:
                angle_map /= n_chirps
            
        return np.fft.fftshift(angle_map, axes=0), r_H_freq

    def _analyze_anchor_peak(self, rH_2d_map):
        idx_linear = np.argmax(rH_2d_map)
        row, col = np.unravel_index(idx_linear, rH_2d_map.shape)
        
        angle_bins = np.linspace(-self.config.ANGLE_RANGE, self.config.ANGLE_RANGE, self.config.ANGLE_BINS)
        angle_deg = angle_bins[col]
        
        n_samples = rH_2d_map.shape[0]
        if row < n_samples // 2:
            f_est_idx = row 
        else:
            f_est_idx = row - n_samples // 2
            
        peak_mag = rH_2d_map[row, col]
        return f_est_idx, angle_deg, peak_mag


# ==================== 前端显示类 ====================
class RadarSystem:
    def __init__(self, config: RadarSignalConfig):
        self.config = config
        self.processor_anchor = AnchorProcessor(config)
        self.processor_raw = RadarDataProcessor(
            num_chirps_to_aggregate=32,
            output_mode="1dfft", 
            show=False
        )
        self.data_queue = queue.Queue(maxsize=2)
        self.running = False
        self._setup_gui()
        
    def _setup_gui(self):
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(18, 10), facecolor='#1C1C1C')
        self.fig.canvas.manager.set_window_title('Advanced Radar System: 地物、锚点与通信解码')
        
        gs = GridSpec(2, 3, figure=self.fig, wspace=0.3, hspace=0.3)
        
        # 1. 原始地物成像
        self.ax_env_raw = self.fig.add_subplot(gs[0, 0])
        self.im_env_raw = self.ax_env_raw.imshow(
            [[0]], aspect='auto', cmap='plasma', interpolation='bilinear', origin='lower'
        )
        self.ax_env_raw.set_title('1. 原始地物成像 (Range-Angle)', color='white')
        self.ax_env_raw.set_xlabel('Angle (deg)')
        self.ax_env_raw.set_ylabel('Range Bin')
        plt.colorbar(self.im_env_raw, ax=self.ax_env_raw)
        
        # 2. 锚点成像 (r_H)
        self.ax_anchor = self.fig.add_subplot(gs[0, 1])
        self.im_anchor = self.ax_anchor.imshow(
            [[0]], aspect='auto', cmap='plasma', interpolation='bilinear', origin='lower'
        )
        self.ax_anchor.set_title('2. 锚点成像 (Iter 1)', color='white')
        self.ax_anchor.set_xlabel('Angle (deg)')
        self.ax_anchor.set_ylabel('Freq Bin')
        plt.colorbar(self.im_anchor, ax=self.ax_anchor)
        
        # 3. 净化后地物
        self.ax_env_clean = self.fig.add_subplot(gs[0, 2])
        self.im_env_clean = self.ax_env_clean.imshow(
            [[0]], aspect='auto', cmap='plasma', interpolation='bilinear', origin='lower'
        )
        self.ax_env_clean.set_title('3. 净化后地物成像', color='lime')
        self.ax_env_clean.set_xlabel('Angle (deg)')
        plt.colorbar(self.im_env_clean, ax=self.ax_env_clean)
        
        # 4. 1D FFT 频谱
        self.ax_fft = self.fig.add_subplot(gs[1, 0])
        self.line_fft, = self.ax_fft.plot([], [], color='cyan', lw=1)
        self.ax_fft.set_title('4. 1D FFT Spectrum', color='white')
        self.ax_fft.set_xlabel('Frequency')
        self.ax_fft.grid(True, alpha=0.3)
        
        # 5. Chirp 比特识别结果 (Visual Update: Show Offsets)
        self.ax_bits = self.fig.add_subplot(gs[1, 1:])
        self.ax_bits.set_title('5. Chirp Frequency Offset & Bit Decision', color='white')
        self.ax_bits.set_xlabel('Chirp Index')
        self.ax_bits.set_ylabel('Freq Offset (Bins)')
        # set ylim
        
        # self.ax_bits.set_ylim(-6, 6) # 动态范围可能更好，或者固定大一点
        
        self.status_text = self.fig.text(0.01, 0.01, "System Ready", color='gray', fontsize=8)

    def _calculate_range_angle_map(self, data_cube):
        n_samples, n_chirps, n_rx = data_cube.shape
        n_angle_bins = self.config.ANGLE_BINS
        n_samples_half = n_samples // 2
        cube_valid = data_cube[:n_samples_half, :, :]
        range_angle_map = np.zeros((n_samples_half, n_angle_bins))
        
        if n_rx < 2:
            center_angle_idx = n_angle_bins // 2
            mag = np.abs(cube_valid[:, :, 0])
            range_angle_map[:, center_angle_idx] = np.mean(mag, axis=1)
            return range_angle_map
        
        angle_bins = np.linspace(-self.config.ANGLE_RANGE, self.config.ANGLE_RANGE, n_angle_bins)
        
        ch1 = cube_valid[:, :, 0]
        ch2 = cube_valid[:, :, 1]
        
        phase_diff = np.angle(ch1 * np.conj(ch2))
        ratio = np.clip(phase_diff / np.pi, -1, 1)
        angle_deg = np.degrees(np.arcsin(ratio))
        
        idx_float = (angle_deg - angle_bins[0]) / (angle_bins[-1] - angle_bins[0]) * (n_angle_bins - 1)
        idx_vec = np.clip(np.round(idx_float), 0, n_angle_bins - 1).astype(int)
        
        mag = np.abs(ch1)
        
        rows = np.arange(n_samples_half)[:, None]
        flat_indices = rows * n_angle_bins + idx_vec
        
        map_flat = range_angle_map.ravel()
        np.add.at(map_flat, flat_indices.ravel(), mag.ravel())
        range_angle_map = map_flat.reshape(range_angle_map.shape)
                    
        if n_chirps > 1:
            range_angle_map /= n_chirps
        return range_angle_map

    def acquisition_thread(self):
        port = self.config.PORT
        baud = self.config.BAUD
        ports = serial.tools.list_ports.comports()
        
        if len(ports) > 0 and port not in [p.device for p in ports]:
            logger.info(f"指定的端口 {port} 不存在，尝试使用第一个可用端口: {ports[0].device}")
            port = ports[0].device
            
        radar_conf = RadarConfig(data_type=RadarDataType.RANGE_FFT)
        try:
            with SerialPortConnector(port, baud, config=radar_conf) as conn:
                while self.running:
                    data = conn.read_data_frame()
                    if data:
                        seg, info = data
                        processed = self.processor_raw.process_frame(seg, info)
                        if processed:
                            cube, meta = processed
                            if not self.data_queue.full():
                                self.data_queue.put(cube)
                    else:
                        time.sleep(0.001)
        except Exception as e:
            logger.error(f"Serial Error: {e}")
    
    def update(self, frame):
        if self.data_queue.empty():
            return
            
        cube_raw = self.data_queue.get()
        
        # 1. 原始地物成像
        ra_map_raw = self._calculate_range_angle_map(cube_raw)
        self.im_env_raw.set_data(ra_map_raw)
        self.im_env_raw.set_clim(0, np.max(ra_map_raw) * 0.9)
        self.im_env_raw.set_extent([-self.config.ANGLE_RANGE, self.config.ANGLE_RANGE, 0, ra_map_raw.shape[0]])
        
        # 2. 锚点处理核心 (Process)
        cube_clean, anchor_map, comm_res, anchor_info, comm_bits = self.processor_anchor.process(cube_raw)
        
        # 3. 锚点成像更新
        if anchor_map is not None:
            n_freq_bins = anchor_map.shape[0]
            self.im_anchor.set_data(np.abs(anchor_map))
            self.im_anchor.set_clim(0, np.max(np.abs(anchor_map)) * 0.9)
            self.im_anchor.set_extent([
                -self.config.ANGLE_RANGE, self.config.ANGLE_RANGE, 
                -n_freq_bins // 2, n_freq_bins // 2
            ])
            
            title_str = f"2. 锚点成像 (Iter 1) (Freq: {anchor_info['freq_bin']:.1f}, Ang: {anchor_info['angle_deg']:.1f}°)"
            self.ax_anchor.set_title(title_str, color='white')
        
        # 4. 净化后地物更新
        ra_map_clean = self._calculate_range_angle_map(cube_clean)
        self.im_env_clean.set_data(ra_map_clean)
        self.im_env_clean.set_clim(0, np.max(ra_map_clean) * 0.9)
        self.im_env_clean.set_extent([-self.config.ANGLE_RANGE, self.config.ANGLE_RANGE, 0, ra_map_clean.shape[0]])
        
        # 5. FFT 更新
        # spec_1d = cube_raw[:, 0, 0]
        # spec_shifted = np.fft.fftshift(spec_1d)
        # n_s = len(spec_shifted)
        # freq_axis = np.linspace(-n_s/2, n_s/2, n_s)
        # self.line_fft.set_data(freq_axis, np.abs(spec_shifted))
        # self.ax_fft.set_xlim(freq_axis[0], freq_axis[-1])
        # self.ax_fft.set_ylim(0, np.max(np.abs(spec_shifted)) * 1.1)
        
        # 6. Bits 更新 (使用 Offsets 可视化)
        offsets = comm_res['offsets']
        bits = comm_bits
        
        self.ax_bits.cla()
        self.ax_bits.set_title('5. Chirp Frequency Offset & Bit Decision', color='white')
        self.ax_bits.set_ylabel('Freq Offset (Bins)')
        self.ax_bits.set_xlabel('Chirp Index')
        
        # 绘制阈值线
        threshold = self.config.BIT_THRESHOLD_BINS
        self.ax_bits.axhline(threshold, color='lime', ls='--', alpha=0.6, lw=1)
        self.ax_bits.axhline(-threshold, color='lime', ls='--', alpha=0.6, lw=1)
        self.ax_bits.axhline(0, color='gray', ls='-', alpha=0.3, lw=1)
        
        # 设置Y轴范围 (保证能看到数据和阈值)
        max_offset = np.max(np.abs(offsets)) if len(offsets) > 0 else 1.0
        limit = max(max_offset * 1.2, threshold * 2.0)
        # self.ax_bits.set_ylim(-limit, limit)
        self.ax_bits.set_ylim(-8, 8)
        
        # 颜色区分: Bit 1 用红色, Bit 0 用蓝色
        colors = ['red' if b == 1 else 'cyan' for b in bits]
        
        # 绘制 Stem 图
        markerline, stemlines, baseline = self.ax_bits.stem(
            range(len(offsets)), offsets, basefmt=' '
        )
        ylim = [-5,5]
        # 手动设置颜色
        plt.setp(stemlines, 'color', 'gray', 'linewidth', 1, 'alpha', 0.5)
        
        # 分别绘制 marker 以实现不同颜色
        # 也可以通过 set_color 逐个设置，但重画 scatter 更方便控制颜色
        # self.ax_bits.scatter(range(len(offsets)), offsets, c=colors, s=20, zorder=3)
        
        # 标注文本
        # for i, b in enumerate(bits):
        #     if b == 1:
        #         # 在点上方标注 "1"
        #         self.ax_bits.text(i, offsets[i] + limit*0.05, "1", color='yellow', fontsize=7, ha='center')
        
        status_str = "Yes" if self.config.ENABLE_ANCHOR_CANCELLATION else "No"
        self.status_text.set_text(f"Processed. Cancellation: {status_str}")
        
        return self.im_env_raw, self.im_anchor, self.im_env_clean, self.line_fft

    def run(self):
        self.running = True
        t = threading.Thread(target=self.acquisition_thread)
        t.daemon = True
        t.start()
        ani = animation.FuncAnimation(
            self.fig, self.update, 
            interval=self.config.REFRESH_INTERVAL, 
            blit=False
        )
        plt.show()
        self.running = False

if __name__ == "__main__":
    config = RadarSignalConfig()
    
    # ========== 参数配置 ==========
    config.PORT = '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_336235763332-if00'
    config.BAUD = 921600
    config.REFRESH_INTERVAL = 200
    config.MAX_ANCHOR_SEARCH_LIMIT = 2 # 实时建议2-3次
    # ============================

    app = RadarSystem(config)
    try:
        app.run()
    except KeyboardInterrupt:
        print("程序停止")