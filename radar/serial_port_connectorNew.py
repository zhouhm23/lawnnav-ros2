import serial
import serial.tools.list_ports
import time
import struct
import logging
import threading
from enum import IntEnum
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass

# ==================== 日志配置 ====================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ==================== 数据类型枚举 ====================
class RadarDataType(IntEnum):
    """雷达数据类型枚举"""
    UNKNOWN = 0
    DS_RAW = 0b010
    RANGE_FFT = 0b011
    DOPPLER_FFT = 0b100

@dataclass
class RadarConfig:
    """雷达配置参数"""
    data_type: RadarDataType = RadarDataType.RANGE_FFT
    range_nfft: int = 256
    doppler_nfft: int = 32
    rx_gain: float = 28.5
    tx_power: float = 9.4

# ==================== 主连接器类 ====================
class SerialPortConnector:
    """串口连接类 - 支持EVB1122毫米波雷达数据解析"""
    
    # MCU帧界定符
    FRAME_HEADER = b'ICLH'
    FRAME_TAIL = b'ICLT'
    FRAME_HEADER_LEN = len(FRAME_HEADER)  # 4
    FRAME_TAIL_LEN = len(FRAME_TAIL)      # 4
    
    # 雷达帧头魔数（高8位）- 大端序模式
    RADAR_HDR_MAGIC = 0xAA
    
    # 缓冲区限制
    MAX_DATA_LEN = 2 * 1024 * 1024  # 2MB
    
    # 同步参数
    SYNC_ATTEMPTS = 10
    
    def __init__(self, port_name: str = 'COM13', baud_rate: int = 921600, 
                 logger: logging.Logger = None, config: RadarConfig = None,
                 skip_checksum: bool = False):
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.serial_obj: Optional[serial.Serial] = None
        self.is_connected = False
        self.skip_checksum = skip_checksum
        
        # 线程安全锁
        self._lock = threading.RLock()
        
        # 内部缓冲区
        self.buffer = bytearray()
        
        # 自定义日志
        self.logger = logger or logging.getLogger(__name__)
        
        # 雷达配置
        self.config = config or RadarConfig()
        
        # 统计信息
        self.stats = {
            'total_packets': 0,
            'valid_packets': 0,
            'checksum_errors': 0,
            'type_errors': 0,
            'magic_errors': 0,
            'bytes_processed': 0,
            'sync_attempts': 0
        }
        
        # MCU元数据长度 = 帧头(4) + 长度(2) + 类型(1) + 通道(1) + 帧尾(4) = 12
        self.mcu_meta_len = self.FRAME_HEADER_LEN + 2 + 1 + 1 + self.FRAME_TAIL_LEN
        
        # 重连配置
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 1.0
        
        # 同步状态
        self._synced = False
    
    # ==================== 上下文管理器 ====================
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()
    
    # ==================== 连接管理 ====================
    def connect(self) -> bool:
        """建立串口连接"""
        with self._lock:
            try:
                if self.serial_obj and self.serial_obj.is_open:
                    self.serial_obj.close()
                
                self.serial_obj = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baud_rate,
                    timeout=0.1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                self.serial_obj.reset_input_buffer()
                self.serial_obj.reset_output_buffer()
                self.buffer.clear()
                self.is_connected = True
                self._synced = False
                
                self.logger.info(f'成功连接到串口: {self.port_name} (波特率: {self.baud_rate})')
                return True
                
            except Exception as e:
                self.is_connected = False
                #self.logger.error(f'连接串口失败: {e}')
                return False
    
    def disconnect(self):
        """断开串口连接"""
        with self._lock:
            try:
                if self.is_connected and self.serial_obj:
                    if self.serial_obj.is_open:
                        self.serial_obj.close()
                    self.serial_obj = None
                
                self.is_connected = False
                self.buffer.clear()
                self._synced = False
                self.logger.info('已断开串口连接')
                
            except Exception as e:
                self.logger.error(f'断开连接时出错: {e}')
    
    def reconnect(self) -> bool:
        """尝试重连串口"""
        self.logger.warning('尝试重连串口...')
        self.disconnect()
        
        for attempt in range(1, self.max_reconnect_attempts + 1):
            self.logger.info(f'重连尝试 {attempt}/{self.max_reconnect_attempts}')
            if self.connect():
                self.logger.info('重连成功')
                return True
            time.sleep(self.reconnect_delay)
        
        self.logger.error('重连失败，已达到最大尝试次数')
        return False
    
    # ==================== 核心读取方法 ====================
    def read_data_frame(
        self, 
        timeout: Optional[float] = None, 
        show: bool = False,
        max_retries: int = 3
    ) -> Optional[Tuple[bytes, Dict[str, Any]]]:
        """
        读取并解析雷达数据帧
        
        Args:
            timeout: 超时时间（秒），None表示无限等待
            show: 控制打印行为
                  - False: 不打印（默认）
                  - True: 每次打印
                  - int: 每show次打印一次
            max_retries: 最大重试次数
        
        Returns:
            成功时返回(radar_seg, info_dict)，失败或超时时返回None
        """
        start_time = time.time() if timeout else None
        loop_counter = 0
        consecutive_magic_errors = 0
        
        while True:
            loop_counter += 1
            
            # 超时检查
            if timeout and (time.time() - start_time) > timeout:
                self.logger.warning("读取数据帧超时")
                return None
            
            # 连接状态检查
            if not self.is_connected:
                self.logger.warning("未连接，尝试重连")
                if not self.reconnect():
                    return None
                continue
            
            try:
                with self._lock:
                    # 1. 读取串口数据
                    self._read_serial_data()
                    
                    # 2. 缓冲区溢出保护
                    if self._check_buffer_overflow():
                        continue
                    
                    # 3. 查找MCU帧头
                    header_idx = self.buffer.find(self.FRAME_HEADER)
                    if header_idx == -1:
                        time.sleep(0.001)
                        continue
                    
                    # 4. 丢弃帧头前的数据
                    if header_idx > 0:
                        if show == 1:
                            self.logger.debug(f'丢弃帧头前 {header_idx} bytes')
                        self._slide_window(header_idx)
                    
                    # 5. 检查是否可读取MCU元数据
                    if len(self.buffer) < self.FRAME_HEADER_LEN + 2:
                        time.sleep(0.001)
                        continue
                    
                    # 6. 解析MCU数据长度
                    mcu_data_len = struct.unpack('<H', self.buffer[self.FRAME_HEADER_LEN:self.FRAME_HEADER_LEN+2])[0]
                    total_len = self.mcu_meta_len + mcu_data_len
                    
                    # 调试模式打印
                    if show == 1 and loop_counter % 10 == 0:
                        self.logger.info(f'[调试] MCU数据长度: {mcu_data_len}, 总长度: {total_len}, 缓冲区: {len(self.buffer)}')
                    
                    # 7. 长度合理性检查
                    if mcu_data_len > self.MAX_DATA_LEN:
                        self.logger.warning(f'MCU数据长度({mcu_data_len})超过最大值')
                        self._slide_window(1)
                        continue
                    
                    # 8. 检查完整数据包
                    if len(self.buffer) < total_len:
                        time.sleep(0.001)
                        continue
                    
                    # 9. 验证MCU帧尾
                    if not self._validate_mcu_tail(total_len):
                        # self._slide_window(1)
                        self._skip_garbage()   # <--- 修改为：直接跳到下一个帧头
                        continue
                    
                    # 10. 提取雷达数据段
                    radar_seg = self._extract_radar_segment(mcu_data_len, total_len)
                    if len(radar_seg) != mcu_data_len:
                        self.logger.warning(f'雷达数据段长度不匹配')
                        self._slide_window(1)
                        continue
                    
                    # 11. 验证雷达帧头（大端序）
                    if not self._validate_radar_header(radar_seg, show=(show==1)):
                        consecutive_magic_errors += 1
                        self.stats['magic_errors'] += 1
                        
                        # 连续错误过多时重新同步
                        if consecutive_magic_errors > self.SYNC_ATTEMPTS:
                            self.logger.error(f"连续{consecutive_magic_errors}次魔数错误，重新同步")
                            self._synced = False
                            self.buffer.clear()
                            consecutive_magic_errors = 0
                            time.sleep(0.1)
                            continue
                        
                        self._slide_window(1)
                        continue
                    
                    # 魔数验证通过，说明已经同步成功
                    if not self._synced:
                        self.logger.info(f"同步成功！收到第一个有效帧")
                    consecutive_magic_errors = 0
                    self._synced = True
                    
                    # 12. 验证校验和（开发模式可跳过）
                    is_checksum_valid = True
                    if not self.skip_checksum:
                        is_checksum_valid = self._verify_checksum(radar_seg, show=(show==1))
                        if not is_checksum_valid:
                            self.stats['checksum_errors'] += 1
                            #self.logger.warning('雷达数据校验和验证失败')
                            # 开发模式：校验和失败仍继续处理
                            # 生产模式：应滑窗1字节继续
                    
                    # 13. 数据包验证通过
                    self.stats['total_packets'] += 1
                    
                    # 14. 解析雷达数据信息
                    info = self._parse_radar_info(radar_seg)
                    info['checksum_valid'] = is_checksum_valid  # <-- 添加校验和状态
                    
                    # 15. 从缓冲区移除已处理数据
                    self.buffer = self.buffer[total_len:]
                    self.stats['valid_packets'] += 1
                    
                    # 16. 显示处理
                    self._handle_show(show, loop_counter, radar_seg, info)
                    
                    return radar_seg, info
            
            except serial.SerialException as e:
                self.logger.error(f'串口异常: {e}')
                self.is_connected = False
                if not self.reconnect():
                    return None
            except Exception as e:
                self.logger.error(f'未知错误: {e}', exc_info=True)
                self.is_connected = False
                if not self.reconnect():
                    return None
    
    # ==================== 内部辅助方法 ====================
    
    def _skip_garbage(self):
        """跳过当前错误数据，直接搜索缓冲区中下一个帧头位置"""
        # 从当前位置+1开始搜寻 'ICLH'
        next_header = self.buffer.find(self.FRAME_HEADER, 1)
        if next_header != -1:
            # 找到了，直接切掉前面的垃圾数据
            self.buffer = self.buffer[next_header:]
        else:
            # 没找到，说明缓冲区全是垃圾，清空
            self.buffer.clear()
            
            
    def _read_serial_data(self):
        """读取串口数据"""
        if self.serial_obj.in_waiting:
            raw_data = self.serial_obj.read(self.serial_obj.in_waiting)
            self.buffer.extend(raw_data)
            self.stats['bytes_processed'] += len(raw_data)
            self.logger.debug(f'收到 {len(raw_data)} bytes，缓冲区总长度: {len(self.buffer)}')
    
    def _check_buffer_overflow(self) -> bool:
        """检查缓冲区溢出"""
        if len(self.buffer) > self.MAX_DATA_LEN:
            self.logger.warning(f'缓冲区溢出({len(self.buffer)} > {self.MAX_DATA_LEN})，清空缓冲区')
            self.buffer.clear()
            time.sleep(0.1)
            return True
        return False
    
    def _slide_window(self, n: int):
        """滑窗n字节"""
        if len(self.buffer) > n:
            self.buffer = self.buffer[n:]
        else:
            self.buffer.clear()
    
    def _validate_mcu_tail(self, total_len: int) -> bool:
        """验证MCU帧尾"""
        if self.buffer[total_len-self.FRAME_TAIL_LEN:total_len] != self.FRAME_TAIL:
            #self.logger.warning('MCU帧尾验证失败')
            return False
        return True
    
    def _extract_radar_segment(self, mcu_data_len: int, total_len: int) -> bytes:
        """提取雷达数据段"""
        return bytes(self.buffer[self.FRAME_HEADER_LEN + 4 : total_len - self.FRAME_TAIL_LEN])
    
    def _validate_radar_header(self, radar_seg: bytes, show: bool = False) -> bool:
        """验证雷达帧头（大端序）"""
        if len(radar_seg) < 4:
            return False
        
        # 大端序解析
        radar_hdr = struct.unpack('>I', radar_seg[0:4])[0]
        
        # 调试模式打印详细信息
        if show:
            self.logger.info(f'[调试] 雷达帧头: 0x{radar_hdr:08X}')
            self.logger.info(f'[调试] 原始数据前16字节: {radar_seg[:16].hex()}')
        
        # 检查高8位
        magic = (radar_hdr >> 24) & 0xFF
        if magic != self.RADAR_HDR_MAGIC:
            self.logger.warning(f'雷达帧头魔数错误(0x{magic:02X} != 0x{self.RADAR_HDR_MAGIC:02X})')
            return False
        return True
    
    def _parse_radar_info(self, radar_seg: bytes) -> Dict[str, Any]:
        """解析雷达数据信息"""
        # 大端序解析
        radar_hdr = struct.unpack('>I', radar_seg[0:4])[0]
        
        # 提取各字段
        rx_channel = (radar_hdr >> 23) & 0x01
        data_type = self._identify_radar_type(radar_hdr)
        chirp_index = (radar_hdr >> 11) & 0x1FF
        data_count = radar_hdr & 0x7FF
        
        # 解析帧尾信息
        frame_index, cfg_msg = self._extract_tail_info(radar_seg)
        
        return {
            'type': data_type,
            'rx_channel': rx_channel,
            'chirp_index': chirp_index,
            'frame_index': frame_index,
            'cfg_msg': cfg_msg,
            'data_points': data_count,
            'packet_length': len(radar_seg),
            'timestamp': time.time()
        }
    
    def _identify_radar_type(self, radar_hdr: int) -> RadarDataType:
        """识别雷达数据类型"""
        type_bits = (radar_hdr >> 20) & 0b111
        try:
            return RadarDataType(type_bits)
        except ValueError:
            self.stats['type_errors'] += 1
            self.logger.warning(f'未知的数据类型: {type_bits:03b}')
            return RadarDataType.UNKNOWN
    
    def _extract_tail_info(self, radar_seg: bytes) -> Tuple[int, int]:
        """从帧尾提取帧索引和CFG_MSG"""
        if len(radar_seg) < 8:
            return 0, 0
        
        # 倒数第2个dword（大端序解析）
        tail_dword = struct.unpack('>I', radar_seg[-8:-4])[0]
        
        # 提取帧索引[15:12]
        frame_index = (tail_dword >> 12) & 0xF
        
        # 提取CFG_MSG[9:8]
        cfg_msg = (tail_dword >> 8) & 0b11
        
        return frame_index, cfg_msg
    
    def _verify_checksum(self, radar_seg: bytes, show: bool = False) -> bool:
        """验证雷达数据校验和"""
        if len(radar_seg) < 8:
            return False
        
        # 提取校验和（倒数第2个dword的高16位）
        tail_dword = struct.unpack('>I', radar_seg[-8:-4])[0]
        checksum = (tail_dword >> 16) & 0xFFFF
        
        # 计算所有16位数据的和（排除校验和自身）
        calculated_sum = 0
        
        # 累加所有dword的低16位和高16位
        for i in range(0, len(radar_seg) - 4, 4):
            dword = struct.unpack('>I', radar_seg[i:i+4])[0]
            calculated_sum += (dword & 0xFFFF) + ((dword >> 16) & 0xFFFF)
        
        # 只比较低16位
        result = (calculated_sum & 0xFFFF) == checksum
        
        # 调试模式打印详细信息
        if show and not result:
            self.logger.error(f'[调试] 校验和验证失败!')
            self.logger.error(f'[调试] 期望值: 0x{checksum:04X}')
            self.logger.error(f'[调试] 计算值: 0x{calculated_sum & 0xFFFF:04X}')
            self.logger.error(f'[调试] 帧长度: {len(radar_seg)} bytes')
            self.logger.error(f'[调试] 原始数据尾16字节: {radar_seg[-16:].hex()}')
        
        return result
    
    def _handle_show(self, show: Any, loop_counter: int, radar_seg: bytes, info: Dict[str, Any]):
        """处理show参数"""
        show_now = False
        if isinstance(show, int):
            if show > 0 and loop_counter % show == 0:
                show_now = True
        elif show is True:
            show_now = True
        
        if show_now:
            self._print_radar_info(radar_seg, info, loop_counter)
    
    def _print_radar_info(self, radar_seg: bytes, info: Dict[str, Any], loop_counter: int):
        """打印雷达数据详细信息（开发调试用）"""
        print("\n" + "="*100)
        print(f"【数据包 #{loop_counter:06d} | 时间戳: {info['timestamp']:.6f}】")
        print("="*100)
        
        # 基础信息
        sync_status = "✓ 已同步" if self._synced else "✗ 未同步"
        print(f"同步状态: {sync_status}")
        print(f"数据类型: {info['type'].name} ({info['type'].value:#05b})")
        print(f"RX通道: RX{info['rx_channel']+1}")
        print(f"Chirp索引: {info['chirp_index']}")
        print(f"帧索引: {info['frame_index']}")
        print(f"CFG_MSG: {info['cfg_msg']:#04b} ({info['cfg_msg']})")
        print(f"数据点数: {info['data_points']}")
        print(f"数据段长度: {info['packet_length']} bytes")
        if 'checksum_valid' in info:
            cs_status = "✓ 通过" if info['checksum_valid'] else "✗ 失败"
            print(f"校验和状态: {cs_status}")
        
        # 十六进制转储（完整数据包）
        print("\n【完整数据包十六进制转储】")
        self._hexdump(radar_seg, width=16)
        
        # 数据样本（前8个复数）
        if info['packet_length'] >= 20:
            print("\n【数据样本（前8个复数）】")
            for i in range(min(8, info['data_points'])):
                idx = 4 + i * 4
                if idx + 4 <= len(radar_seg):
                    real = struct.unpack('>h', radar_seg[idx:idx+2])[0]
                    imag = struct.unpack('>h', radar_seg[idx+2:idx+4])[0]
                    print(f"  [{i:2d}] 0x{radar_seg[idx:idx+4].hex()}: {real:7d} + {imag:7d}j")
        
        print("="*100 + "\n")
    
    def _hexdump(self, data: bytes, width: int = 16):
        """十六进制转储"""
        for i in range(0, len(data), width):
            chunk = data[i:i+width]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            print(f"  {i:04X}: {hex_str:<{width*3}} |{ascii_str:<{width}}|")
    
    def get_stats(self) -> Dict[str, int]:
        """获取统计信息（线程安全）"""
        with self._lock:
            return self.stats.copy()
    
    def reset_stats(self):
        """重置统计信息（线程安全）"""
        with self._lock:
            for key in self.stats:
                self.stats[key] = 0
    
    def is_synced(self) -> bool:
        """获取同步状态"""
        with self._lock:
            return self._synced

# ==================== 实验性主程序 ====================
if __name__ == "__main__":
    print("="*80)
    print("EVB1122 毫米波雷达串口连接测试程序")
    print("="*80)
    
    # ==================== 用户配置区 ====================
    PORT = '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_336235763332-if00'  # Linux (stable by-id)
    BAUD = 921600                # 波特率
    TIMEOUT = 30.0               # 读取超时（秒），设为None为无限等待
    SHOW_INTERVAL = 1            # 每1帧打印一次
    MAX_PACKETS = 100            # 最大接收帧数（None表示无限）
    STATS_INTERVAL = 10          # 统计信息显示间隔
    
    # 雷达配置
    RADAR_CONFIG = RadarConfig(
        data_type=RadarDataType.RANGE_FFT,
        range_nfft=256,
        doppler_nfft=32,
        rx_gain=28.5,
        tx_power=9.4
    )
    
    # 开发模式：跳过校验和验证（建议先设为True）
    SKIP_CHECKSUM = True
    
    # 日志级别
    logging.getLogger().setLevel(logging.INFO)
    
    # ===================================================
    
    try:
        with SerialPortConnector(PORT, BAUD, config=RADAR_CONFIG, skip_checksum=SKIP_CHECKSUM) as conn:
            print(f"已连接至 {PORT}，开始接收数据...")
            print(f"配置: 波特率={BAUD}, 超时={TIMEOUT}s, 显示间隔={SHOW_INTERVAL}帧")
            print(f"雷达配置: {RADAR_CONFIG}")
            print(f"跳过校验和: {SKIP_CHECKSUM}")
            print("按 Ctrl+C 停止\n")
            
            packet_count = 0
            start_time = time.time()
            last_stats_time = start_time
            
            while True:
                # 检查最大帧数限制
                if MAX_PACKETS and packet_count >= MAX_PACKETS:
                    print(f"\n已达到最大帧数限制 ({MAX_PACKETS})，退出")
                    break
                
                # 检查同步状态
                if not conn.is_synced():
                    print(f"\r[等待同步... 魔数错误: {conn.stats['magic_errors']:04d}]", end='', flush=True)
                
                # 读取并解析数据帧
                result = conn.read_data_frame(timeout=TIMEOUT, show=SHOW_INTERVAL)
                
                if result is None:
                    print("\n读取超时或连接失败，退出")
                    break
                
                radar_data, info = result
                packet_count += 1
                
                # 显示统计信息
                current_time = time.time()
                if current_time - last_stats_time >= 2.0:  # 每2秒显示一次
                    elapsed = current_time - start_time
                    avg_fps = packet_count / elapsed if elapsed > 0 else 0
                    stats = conn.get_stats()
                    
                    print(f"\r[运行状态] 帧数: {packet_count:05d} | "
                          f"有效: {stats['valid_packets']:05d} | "
                          f"同步: {'✓' if conn.is_synced() else '✗'} | "
                          f"魔数错误: {stats['magic_errors']:04d} | "
                          f"校验错误: {stats['checksum_errors']:03d} | "
                          f"类型错误: {stats['type_errors']:03d} | "
                          f"平均帧率: {avg_fps:.2f} fps", 
                          end='', flush=True)
                    last_stats_time = current_time
    
    except KeyboardInterrupt:
        print("\n\n用户中断，程序退出")
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n" + "="*80)
        print("测试结束 - 最终统计:")
        if 'conn' in locals():
            stats = conn.get_stats()
            elapsed = time.time() - start_time
            
            print(f"  运行时间: {elapsed:.2f} 秒")
            print(f"  总接收帧: {packet_count}")
            print(f"  有效帧: {stats['valid_packets']}")
            print(f"  同步成功: {'是' if conn.is_synced() else '否'}")
            print(f"  魔数错误: {stats['magic_errors']} (持续>100次说明未同步)")
            print(f"  校验错误: {stats['checksum_errors']}")
            print(f"  类型错误: {stats['type_errors']}")
            print(f"  平均帧率: {packet_count/elapsed:.2f} fps")
            print(f"  处理字节: {stats['bytes_processed']}")
        
        print("\n开发调试建议:")
        print("  1. 数据已正常接收，可开始后续处理")
        print("  2. 校验和算法需手动验证，目前设置为跳过")
        print("  3. 如需长期测试，增大MAX_PACKETS或设为None")
        print("  4. 性能优化：将logging级别设为WARNING减少输出")
        print("="*80)