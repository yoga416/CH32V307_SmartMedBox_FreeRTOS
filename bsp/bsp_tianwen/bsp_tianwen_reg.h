#ifndef __BSP_TIANWEN_REG_H
#define __BSP_TIANWEN_REG_H

/* 通用帧头帧尾定义 */
#define TIANWEN_FRAME_HEAD      0x5A
#define TIANWEN_FRAME_TAIL      0xFF
#define TIANWEN_FIXED_DATA1     0xD5
#define TIANWEN_FIXED_DATA2     0x5A

/* ================= 主控发送给天问的指令 (TX) ================= */
#define CMD_TX_ALARM_TIME_UP    0x01  // 闹钟时间到
#define CMD_TX_UNREGISTERED     0x03  // 未注册
#define CMD_TX_REGISTERED_USER  0x04  // 已注册，用户X
#define CMD_TX_NOT_TIME_TO_EAT  0x05  // 未到吃药时间
#define CMD_TX_TIME_TO_EAT      0x06  // 到了吃药时间
#define CMD_TX_TEMP_RESULT      0x08  // 发送体温测量结果 (带数据)
#define CMD_TX_HR_RESULT        0x0F  // 发送心率测量结果 (带数据)
#define CMD_TX_SPO2_RESULT      0x0A  // 发送血氧测量结果 (带数据)
#define CMD_TX_ADD_SUCCESS      0x0E  // 添加用户成功
#define CMD_TX_BOOT_VOICE       0x11  // 开机语音播报
/* ================= 天问发送给主控的指令 (RX) ================= */
#define CMD_RX_START_FACE_ID    0x02  // 进行人脸识别
#define CMD_RX_HEALTH_DETECT    0x07  // 进行健康检测
#define CMD_RX_TEMP_DETECT      0x0B  // 进行体温检测
#define CMD_RX_HR_SPO2_DETECT   0x0C  // 进行心率血氧检测
#define CMD_RX_ADD_USER         0x0D  // 进行人脸添加

//数据包结构定义
typedef struct {
    uint8_t id;          // 传感器ID
    uint8_t data_len;    // 数据长度
    uint8_t data[64];    // 数据内容 (最大64字节)
} Tianwen_Packet_t;

#endif /* __BSP_TIANWEN_REG_H */