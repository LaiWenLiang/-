# -*- coding: utf-8 -*-
"""
OTA 固件打包脚本（RS485 通道）
把 app.bin 打包成带帧头/校验的 .ota 文件，之后可以用串口助手直接"发文件"。

帧格式（小端）：
  [55 AA] [CMD] [LEN_L LEN_H] [DATA...] [CRC16_L CRC16_H]
  CMD: 01=START(数据=总大小4B)  02=DATA(<=255B)  03=END(数据=整包CRC32 4B)
  CRC16: Modbus(0xA001, 初值0xFFFF)，对 CMD+LEN+DATA 计算

用法：
  python pack_ota.py app.bin              -> 生成 app.ota
  python pack_ota.py app.bin out.ota      -> 指定输出文件名
"""

import sys
import zlib

HEAD = b"\x55\xAA"
CMD_START = 0x01
CMD_DATA  = 0x02
CMD_END   = 0x03
DATA_CHUNK = 255          # 每帧数据区最大字节数


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def pack_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([cmd, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    crc = crc16_modbus(body)
    return HEAD + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    in_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else in_path.rsplit(".", 1)[0] + ".ota"

    with open(in_path, "rb") as f:
        fw = f.read()

    size = len(fw)
    if size < 512 or size > 256 * 1024:
        print("固件大小 %d 字节，超出允许范围（512B ~ 256KB）" % size)
        return

    crc32 = zlib.crc32(fw) & 0xFFFFFFFF

    out = bytearray()
    out += pack_frame(CMD_START, size.to_bytes(4, "little"))          # START 帧
    for i in range(0, size, DATA_CHUNK):                              # DATA 帧
        out += pack_frame(CMD_DATA, fw[i:i + DATA_CHUNK])
    out += pack_frame(CMD_END, crc32.to_bytes(4, "little"))           # END 帧

    with open(out_path, "wb") as f:
        f.write(out)

    print("打包完成：")
    print("  输入：%s（%d 字节）" % (in_path, size))
    print("  输出：%s（%d 字节，%d 个数据帧）" % (out_path, len(out), (size + DATA_CHUNK - 1) // DATA_CHUNK))
    print("  整包 CRC32：0x%08X" % crc32)
    print("")
    print("串口助手发送注意：")
    print("  1. 车上先按 KEY3 切到 MODE: RS485(OTA)")
    print("  2. 115200 8N1，十六进制发送关闭（按原始字节发文件）")
    print("  3. 必须开启\"分包发送/帧间隔\"，建议每 255~300 字节延时 20ms 以上")
    print("     （车端每帧要写 SPI Flash，连续猛发会丢帧）")


if __name__ == "__main__":
    main()
