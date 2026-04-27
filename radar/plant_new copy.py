import numpy as np
import time
import logging
import serial.tools.list_ports
from scipy import signal
from math import ceil, floor
import sys

# 引入本地模块 (确保这些文件在小车的文件系统中)
try:
    from radar_data_processor import RadarDataProcessor, RadarDataType
    from serial_port_connector import SerialPortConnector, RadarConfig
except ImportError:
    # 模拟环境或缺少驱动时的提示，实际部署时请忽略或确保文件存在
    pass

# ==================== 全局日志配置 ====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# ==================== 配置类定义 ====================
class RadarSignalConfig:
    """
    雷达信号处理配置类 - 针对嵌入式环境优化
    """
    def __init__(self):
        # --- 硬件与串口 ---
        # 注意：在Linux/树莓派上，端口通常是 /dev/ttyUSB0 或 /dev/ttyACM2
        self.PORT = '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_336235763332-if00'
        self.BAUD = 921600
        
        # --- 系统控制 ---
        self.ENABLE_ANCHOR_CANCELLATION = True # 是否开启锚点消除
        
        # --- M-Sequence 参数 ---
        self.M_ORDER = 5
        self.CHIP_RATE_RATIO = 0.25  # fs/4
        self.FN2_RATIO = 0.5         # fs/2
        
        # --- 通信解码参数 ---
        self.FFT_BIN_SEARCH = 50.0   # 消除时的搜索范围
        self.FILTER_WIDTH_RATIO = 0.05 # 滤波器宽度占总带宽的比例
        self.BIT_THRESHOLD_BINS = 1  # 判决阈值 (Freq Offset Bins)
        
        # --- 成像参数 (仅用于计算逻辑，不显示) ---
        self.ANGLE_RANGE = 60
        self.ANGLE_BINS = 61 
        
        # --- 迭代探测参数 ---
        self.MAX_ANCHOR_SEARCH_LIMIT = 2 # 最大迭代消除次数
        self.STOP_ENERGY_PCT = 0.01      # 能量变化停止阈值 (0.01%)
        
        self.f1_bin_idx = 2  # (Legacy)

# ==================== 核心处理类：锚点处理器 ====================
class AnchorProcessor:
    """
    核心算法类 - 保持原逻辑不变以确保结果正确性
    """
    def __init__(self, config: RadarSignalConfig):
        self.config = config
        self.m_seq_raw = self._generate_m_sequence(config.M_ORDER)
        self.cache = {} # 缓存预计算的Code Spectrum
        self.f1_bin_idx = config.f1_bin_idx

    def process(self, cube_raw):
        """
        处理管道
        Returns: anchor_info (dict), comm_bits (array)
        """
        n_samples, n_chirps, n_rx = cube_raw.shape
        
        # 1. 预计算/获取参考码
        code_seq, code_spec_conj = self._get_reference_code(n_samples)
        
        cube_current = cube_raw.copy()
        energy0 = np.sum(np.abs(cube_raw)**2)
        
        first_iter_info = None
        comm_bits_result = None
        
        # 迭代循环
        for iteration in range(self.config.MAX_ANCHOR_SEARCH_LIMIT):
            # [Step 1] 锚点定位
            rH_2d_map, current_center, _, mask_minus = self._solve_iterative_anchor(cube_current)
            
            # 分析峰值
            f_est_idx, peak_angle, peak_mag_lin = self._analyze_anchor_peak(rH_2d_map)
            
            # 仅保存第一次迭代的信息用于输出（通常是最强的锚点）
            if iteration == 0:
                freq_display = f_est_idx if f_est_idx <= n_samples//2 else f_est_idx - n_samples//2
                first_iter_info = {
                    'freq_bin': freq_display,
                    'angle_deg': peak_angle,
                    'amp': peak_mag_lin
                }

            # [Step 2] 延迟估计
            pilot_idx = int(round((f_est_idx + n_samples * self.config.FN2_RATIO) % n_samples))
            
            code_cube, lags = self._step_lag_estimation(
                cube_current, mask_minus, pilot_idx, code_seq, code_spec_conj
            )
            
            # [Step 3] 预计算用于解码和消除
            cube_t = np.fft.ifft(cube_current, axis=0)
            signal_decoded_time = cube_t * code_cube
            signal_decoded_spec = np.fft.fft(signal_decoded_time, axis=0)

            # [Step 4] 通信解码
            comm_bits, freq_offsets = self._step_comm_decoding(
                signal_decoded_spec, f_est_idx, n_samples
            )
            
            if iteration == 0:
                comm_bits_result = comm_bits

            # [Step 5] 信号消除
            if self.config.ENABLE_ANCHOR_CANCELLATION:
                cube_current, reduce_pct = self._step_cancellation(
                    cube_current, code_cube, pilot_idx, f_est_idx, n_samples, energy0,
                    signal_decoded_spec=signal_decoded_spec
                )
                if reduce_pct < self.config.STOP_ENERGY_PCT:
                    break
            else:
                break
                
        return first_iter_info, comm_bits_result

    # ... (核心算法部分保持不变，确保数学结果正确) ...

    def _solve_iterative_anchor(self, cube):
        n_samples = cube.shape[0]
        n_half = n_samples // 2
        chip_band = int(round(n_samples * self.config.CHIP_RATE_RATIO))
        
        mask_plus = np.zeros(n_samples); mask_plus[:n_half] = 1
        mask_minus = np.zeros(n_samples); mask_minus[n_half:] = 1
        
        _, rH_freq_init = self._compute_rH_map_masked(cube, mask_plus, mask_minus)
        
        spec_mag = np.sum(np.abs(rH_freq_init), axis=(1, 2))
        idx_highest = np.argmax(spec_mag)
        
        if idx_highest < n_half: new_center = idx_highest / 2.0
        else: new_center = (idx_highest - n_half) / 2.0
            
        center1 = int(round(max(0, new_center)))
        center2 = int(round(center1 + n_samples / 4.0)) 
        
        chip_band_comp = int(round(chip_band / 2.0))
        
        # Hypothesis 1
        rp1 = [center1 - chip_band_comp, center1 + chip_band_comp]
        rm1 = [rp1[0] + n_half, rp1[1] + n_half]
        mp1 = self._create_circular_mask(n_samples, rp1)
        mm1 = self._create_circular_mask(n_samples, rm1)
        rH_map1, _ = self._compute_rH_map_masked(cube, mp1, mm1)
        
        # Hypothesis 2
        rp2 = [center2 - chip_band_comp, center2 + chip_band_comp]
        rm2 = [rp2[0] + n_half, rp2[1] + n_half]
        mp2 = self._create_circular_mask(n_samples, rp2)
        mm2 = self._create_circular_mask(n_samples, rm2)
        rH_map2, _ = self._compute_rH_map_masked(cube, mp2, mm2)
        
        if np.max(np.abs(rH_map1)) >= np.max(np.abs(rH_map2)):
            final_center = center1
            final_rp = [center1 - chip_band, center1 + chip_band]
        else:
            final_center = center2
            final_rp = [center2 - chip_band, center2 + chip_band]
            
        final_rm = [final_rp[0] + n_half, final_rp[1] + n_half]
        final_mp = self._create_circular_mask(n_samples, final_rp)
        final_mm = self._create_circular_mask(n_samples, final_rm)
        
        final_map, rH_freq_final = self._compute_rH_map_masked(cube, final_mp, final_mm)
        return final_map, final_center, rH_freq_final, final_mm

    def _step_lag_estimation(self, cube_in, mask_minus, pilot_idx, code_seq, code_spec_conj):
        n_samples, n_chirps, _ = cube_in.shape
        r_minus_spec = cube_in[:, :, 0] # Use Rx0
        X_sig = np.roll(r_minus_spec, -pilot_idx, axis=0)
        corr_circ = np.fft.ifft(X_sig * code_spec_conj[:, None], axis=0)
        chirp_lags = np.argmax(np.abs(corr_circ), axis=0)
        
        raw_indices = np.arange(n_samples)[:, None] - chirp_lags[None, :]
        code_indices = np.mod(raw_indices, n_samples)
        code_mat = code_seq[code_indices]
        code_cube = code_mat[:, :, None]
        return code_cube, chirp_lags

    def _step_comm_decoding(self, signal_decoded_spec, f_est_idx, n_samples):
        n_chirps = signal_decoded_spec.shape[1]
        bits = np.zeros(n_chirps, dtype=int)
        freq_offsets = np.zeros(n_chirps, dtype=float)
        
        delta_k = int(round(self.config.FN2_RATIO * n_samples))
        filter_width = int(round(n_samples * self.config.FILTER_WIDTH_RATIO))
        if filter_width < 3: filter_width = 3
        
        search_radius = 8
        dc_indices_pos = np.arange(0, search_radius + 1)
        dc_indices_neg = np.arange(n_samples - search_radius, n_samples)
        dc_search_indices = np.concatenate((dc_indices_pos, dc_indices_neg)).astype(int)
        
        search_width = 10
        center_idx_approx = int(round(f_est_idx))
        carrier_search_range = np.arange(center_idx_approx - search_width, center_idx_approx + search_width + 1)
        carrier_search_range = np.mod(carrier_search_range, n_samples).astype(int)

        for c in range(n_chirps):
            r_spec = signal_decoded_spec[:, c, 0]
            spec_in_range = np.abs(r_spec[carrier_search_range])
            local_peak_idx = np.argmax(spec_in_range)
            carrier_idx = carrier_search_range[local_peak_idx]
            
            sideband_idx = (carrier_idx - delta_k) % n_samples
            
            mask_c = self._create_bandpass_mask(n_samples, carrier_idx, filter_width)
            mask_s = self._create_bandpass_mask(n_samples, sideband_idx, filter_width)
            
            spec_c = r_spec * mask_c
            spec_s = r_spec * mask_s
            
            spec_c_bb = np.roll(spec_c, -carrier_idx)
            spec_s_bb = np.roll(spec_s, -sideband_idx)
            
            time_c = np.fft.ifft(spec_c_bb)
            time_s = np.fft.ifft(spec_s_bb)
            
            phasor_prod = time_s * np.conj(time_c)
            spec_prod = np.fft.fft(phasor_prod)
            
            spec_prod_search = np.abs(spec_prod[dc_search_indices])
            peak_local_idx = np.argmax(spec_prod_search)
            peak_global_idx = dc_search_indices[peak_local_idx]
            
            if peak_global_idx <= n_samples // 2:
                current_offset = float(peak_global_idx)
            else:
                current_offset = float(peak_global_idx - n_samples)
            
            freq_offsets[c] = current_offset
            if abs(current_offset) > self.config.BIT_THRESHOLD_BINS:
                bits[c] = 1
            else:
                bits[c] = 0
                
        return bits, freq_offsets

    def _step_cancellation(self, cube_in, code_cube, pilot_idx, f_est_idx, n_samples, energy0, signal_decoded_spec=None):
        fft_bin_search = int(self.config.FFT_BIN_SEARCH)
        
        if signal_decoded_spec is None:
            cube_t = np.fft.ifft(cube_in, axis=0)
            signal_decoded_time = cube_t * code_cube
            signal_decoded_spec = np.fft.fft(signal_decoded_time, axis=0)

        bin_width = int(round(2 * fft_bin_search + 1))
        if bin_width < 3: bin_width = 3
        mask_p = self._create_bandpass_mask(n_samples, int(round(pilot_idx)), bin_width)
        mask_e = self._create_bandpass_mask(n_samples, int(round(f_est_idx)), bin_width)
        mask_all = ((mask_p + mask_e) > 0).astype(float).reshape(-1, 1, 1)
        
        signal_filtered_spec = signal_decoded_spec * mask_all
        signal_reconstructed_time = np.fft.ifft(signal_filtered_spec, axis=0)
        signal_reconstructed_final = signal_reconstructed_time * code_cube
        
        signal_reconstructed_freq = np.fft.fft(signal_reconstructed_final, axis=0)
        cube_out = cube_in - signal_reconstructed_freq
        
        energy_removed = np.sum(np.abs(signal_reconstructed_final)**2)
        reduce_pct = (energy_removed / energy0) * 100
        
        return cube_out, reduce_pct

    def _create_bandpass_mask(self, n_samples, center_idx, width):
        mask = np.zeros(n_samples, dtype=float)
        half_w = width // 2
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
            if n_chirps > 1: angle_map /= n_chirps
            
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


# ==================== 嵌入式运行系统 (单线程版) ====================
class HeadlessRadarSystem:
    def __init__(self, config: RadarSignalConfig):
        self.config = config
        self.processor_anchor = AnchorProcessor(config)
        # 初始化原始数据处理器 (假设只用于解包数据，不做绘图)
        self.processor_raw = RadarDataProcessor(
            num_chirps_to_aggregate=32,
            output_mode="1dfft", 
            show=False
        )
        self.running = False
        
    def run(self):
        """主处理循环 (单线程顺序执行)"""
        port = self.config.PORT
        baud = self.config.BAUD
        ports = serial.tools.list_ports.comports()
        
        # 自动搜寻串口逻辑
        target_port_exists = any(p.device == port for p in ports)
        if not target_port_exists and len(ports) > 0:
            logger.info(f"警告：端口 {port} 未找到，自动切换至: {ports[0].device}")
            port = ports[0].device
            
        radar_conf = RadarConfig(data_type=RadarDataType.RANGE_FFT)
        
        logger.info(f"正在尝试连接雷达串口: {port} @ {baud}...")
        
        try:
            with SerialPortConnector(port, baud, config=radar_conf) as conn:
                logger.info("串口连接成功，开始主循环...")
                logger.info("系统已启动 (按 Ctrl+C 退出)")
                logger.info("-" * 40)
                
                self.running = True
                
                while self.running:
                    try:
                        # 1. 阻塞式或轮询式读取数据帧
                        data = conn.read_data_frame()
                        
                        if data:
                            seg, info = data
                            # 2. 预处理 (解包)
                            processed = self.processor_raw.process_frame(seg, info)
                            
                            if processed:
                                cube_raw, meta = processed
                                
                                # 3. 核心算法处理 (同步执行)
                                start_time = time.time()
                                anchor_info, comm_bits = self.processor_anchor.process(cube_raw)
                                process_time = (time.time() - start_time) * 1000
                                
                                # 4. 打印结果
                                self._print_results(anchor_info, comm_bits, process_time)
                        else:
                            # 如果没有数据，短暂休眠避免CPU 100%
                            time.sleep(0.01)
                            
                    except KeyboardInterrupt:
                        raise # 抛出给外层捕获
                    except Exception as e_inner:
                        logger.error(f"处理循环错误: {e_inner}")
                        time.sleep(0.1) # 出错后稍作等待
                        
        except KeyboardInterrupt:
            print("\n用户终止，正在停止...")
        except Exception as e:
            logger.error(f"串口严重错误/无法打开: {e}")
        finally:
            self.running = False
            print("系统已安全退出。")

    def _print_results(self, anchor_info, comm_bits, process_time):
        if anchor_info:
            # 格式化比特串
            bits_str = "".join(str(b) for b in comm_bits)
            
            # 打印信息
            print(f"\n[处理耗时: {process_time:.1f}ms]")
            print(f"锚点 | FreqBin: {anchor_info['freq_bin']:.2f} | Ang: {anchor_info['angle_deg']:.1f}° | Amp: {anchor_info['amp']:.2e}")
            print(f"数据 | {bits_str}")
            print("-" * 30)
            time.sleep(0.5)
        else:
            print(".", end="", flush=True) # 无锚点时打印点号表示存活

if __name__ == "__main__":
    # 配置
    config = RadarSignalConfig()
    
    # 根据实际小车情况修改串口号
    # config.PORT = '/dev/ttyUSB0' 
    config.MAX_ANCHOR_SEARCH_LIMIT = 2 
    
    app = HeadlessRadarSystem(config)
    app.run()