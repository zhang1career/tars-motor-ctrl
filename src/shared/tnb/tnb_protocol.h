#ifndef TNB_PROTOCOL_H
#define TNB_PROTOCOL_H

/*
 * TARS Node Bus — motor-ctrl 从机副本。
 * 产品号 / 电机寄存器与 tars/App/node_bus/tnb_protocol.h 对齐。
 * 不修改 tars-io-mux。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TNB_PROTO_VER            0x01U

#define TNB_VENDOR_TARS          0x5441U
#define TNB_PRODUCT_IO_MUX       0x0001U
#define TNB_PRODUCT_IO_MUX_HV    0x0002U
#define TNB_PRODUCT_IO_CROSSPOINT 0x0004U
#define TNB_PRODUCT_MOTOR_CTRL   0x0005U

#define TNB_PROFILE_FULL         0x00U
#define TNB_PROFILE_LITE         0x01U

#define TNB_ADDR_GENERAL_CALL    0x00U
#define TNB_ADDR_ARP             0x61U
#define TNB_ADDR_ALERT_RESPONSE  0x0CU

#define TNB_ADDR_FULL_BASE       0x40U
#define TNB_ADDR_FULL_MAX        0x5FU

#define TNB_ADDR_IS_FULL_POOL(a)                                               \
  ((((uint8_t)(a)) >= TNB_ADDR_FULL_BASE) && (((uint8_t)(a)) <= TNB_ADDR_FULL_MAX))

#define TNB_ADDR_FULL_FROM_BOARD_ID(id)                                        \
  ((uint8_t)(TNB_ADDR_FULL_BASE + ((uint8_t)(id) & 0x1FU)))

#define TNB_GC_ARP_PREPARE       0x01U
#define TNB_GC_RESET             0x06U

#define TNB_REG_PROTO_VER        0x00U
#define TNB_REG_STATUS           0x01U
#define TNB_REG_VENDOR_ID        0x02U
#define TNB_REG_PRODUCT_ID       0x04U
#define TNB_REG_FW_VER           0x06U
#define TNB_REG_BOARD_ID         0x08U
#define TNB_REG_ADDR             0x09U
#define TNB_REG_CAP_COUNT        0x0AU
#define TNB_REG_PROFILE          0x0BU
#define TNB_REG_UID              0x0CU

#define TNB_REG_FAULT_FLAGS      0x20U
#define TNB_REG_UPTIME_S         0x22U
#define TNB_REG_TEMP_C10         0x26U
#define TNB_REG_VDDA_MV          0x28U
#define TNB_REG_HEARTBEAT        0x2AU
#define TNB_REG_LAST_ERR         0x2BU
#define TNB_REG_I2C_OK           0x2CU
#define TNB_REG_MAX_WRITE_PAYLOAD 0x2DU
#define TNB_REG_MAX_READ_BURST    0x2EU
#define TNB_REG_XFER_FLAGS        0x2FU

#define TNB_XFER_NO_SR            (1U << 0)
#define TNB_XFER_STOP_FLUSH_WRITE (1U << 1)
#define TNB_XFER_STATIC_ADDR_ONLY (1U << 2)
#define TNB_XFER_BYTE_READ_PREF   (1U << 3)
#define TNB_XFER_PEC              (1U << 4) /* SMBus CRC-8 on write payload / read burst */

#define TNB_MAX_WRITE_PAYLOAD_FULL  16U
#define TNB_MAX_READ_BURST_FULL     32U
#define TNB_XFER_FLAGS_FULL         (TNB_XFER_STOP_FLUSH_WRITE)

#define TNB_REG_ALERT_CTRL       0x5EU
#define TNB_REG_ALERT_CLEAR      0x5FU
#define TNB_ALERT_CTRL_EN        0x01U
#define TNB_ALERT_CTRL_FORCE     0x80U
#define TNB_REG_CAP_INDEX        0x60U
#define TNB_REG_CAP_DESC         0x61U
#define TNB_REG_CMD              0x80U
#define TNB_REG_RESP             0x81U

#define TNB_REG_MOT_MODE         0x90U
#define TNB_REG_MOT_CMD          0x91U
#define TNB_REG_MOT_DIR          0x92U
#define TNB_REG_MOT_IQ_REF_MA    0x94U
#define TNB_REG_MOT_W_REF_EPS    0x96U
#define TNB_REG_MOT_THETA_Q16    0xA0U
#define TNB_REG_MOT_W_MEAS_EPS   0xA2U
#define TNB_REG_MOT_IQ_MA        0xA4U
#define TNB_REG_MOT_ID_MA        0xA6U
#define TNB_REG_MOT_VBUS_MV      0xA8U
#define TNB_REG_MOT_FAULT        0xAAU
#define TNB_REG_MOT_HALL         0xACU
#define TNB_REG_MOT_STATE        0xADU
#define TNB_REG_MOT_THETA_ACC    0xAEU /* i32 RO 展开电角度 Q16，wrap 用 Δacc/dt */
#define TNB_REG_MOT_IQ_MAX_MA    0xB2U /* i16 RO 电流环 |iq| 顶，mA */
#define TNB_REG_MOT_VBOOST_MV    0xB4U /* u16 RO 电荷泵，mV */
#define TNB_REG_MOT_T_BOARD_C10  0xB6U /* i16 RO 主板 RT1，0.1 °C；TNB_TEMP_NONE=无效 */
#define TNB_REG_MOT_T_CASE_C10   0xB8U /* i16 RO J15 壳温。v0.1 无 ADC，恒为 TNB_TEMP_NONE */
#define TNB_REG_MOT_T_MCU_C10    0xBAU /* i16 RO 片内 TS，与 0x26 同一数 */
#define TNB_REG_MOT_CLOCK_KHZ    0xBCU /* u16 RO SystemCoreClock/1000 */
#define TNB_REG_MOT_SENSE        0xBEU /* u16 RO TNB_MOT_SNS_* */
#define TNB_REG_MOT_VD_MV        0xC0U /* i16 RO 电压指令 d，mV */
#define TNB_REG_MOT_VQ_MV        0xC2U /* i16 RO 电压指令 q，mV */
#define TNB_TEMP_NONE            ((int16_t)0x8000)
#define TNB_MOT_SNS_NFAULT       (1U << 0) /* PA6 高=1 正常 */
#define TNB_MOT_SNS_NOTEMP       (1U << 1) /* PA12 高=1 未过温（J14/J15 比较器） */
#define TNB_MOT_SNS_CLK_OK       (1U << 2) /* SYSCLK ≥ 40 MHz */
#define TNB_MOT_SNS_TBOARD       (1U << 3) /* t_board 有效 */
#define TNB_MOT_SNS_TCASE        (1U << 4) /* t_case 模拟有效（v0.1 没有） */
#define TNB_MOT_SNS_REV          (1U << 5) /* 正在换向 */
#define TNB_MOT_SNS_SAT          (1U << 6) /* 电压顶 */
#define TNB_MOT_SNS_TMCU         (1U << 7) /* t_mcu 有效 */

#define TNB_MOT_MODE_STOP        0U
#define TNB_MOT_MODE_TORQUE      1U
#define TNB_MOT_MODE_SPEED       2U
#define TNB_MOT_CMD_RUN          (1U << 0)
#define TNB_MOT_CMD_FAULT_CLR    (1U << 1)
#define TNB_MOT_ST_IDLE          0U
#define TNB_MOT_ST_HALL6         1U
#define TNB_MOT_ST_TORQUE        2U
#define TNB_MOT_ST_SPEED         3U
#define TNB_MOT_ST_FAULT         4U
#define TNB_MOT_FLT_OTEMP        (1U << 0)
#define TNB_MOT_FLT_NFAULT       (1U << 1)
#define TNB_MOT_FLT_STALL        (1U << 2)
#define TNB_MOT_FLT_I2C          (1U << 3)
#define TNB_MOT_FLT_PWM          (1U << 4)

#define TNB_REG_SPACE_SIZE       0x100U

#define TNB_ST_READY             (1U << 0)
#define TNB_ST_FAULT             (1U << 1)
#define TNB_ST_OVERTEMP          (1U << 2)
#define TNB_ST_ALERT             (1U << 5)

#define TNB_FLT_OVERTEMP         (1U << 0)
#define TNB_FLT_PWM              (1U << 3) /* hardware nFAULT */
#define TNB_FLT_I2C              (1U << 4)

#define TNB_CAP_ALERT            0x04U
#define TNB_CAP_MOTOR            0x10U

#define TNB_ACC_R                (1U << 0)
#define TNB_ACC_W                (1U << 1)
#define TNB_ACC_RW               (TNB_ACC_R | TNB_ACC_W)

#define TNB_UNIT_NONE            0x00U
#define TNB_UNIT_RAW             0x02U

#define TNB_OK                   0x00U
#define TNB_ERR_PARAM            0x01U
#define TNB_ERR_CRC              0x02U
#define TNB_ERR_STATE            0x03U
#define TNB_ERR_UNSUPPORTED      0x04U
#define TNB_ERR_BUSY             0x05U

#define TNB_CMD_PING             0x01U

#define TNB_UDID_LEN             16U
#define TNB_CAP_NAME_MAX         16U

#define TNB_DEVCAP_PEC           (1U << 0)
#define TNB_DEVCAP_ALERT         (1U << 1)

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  id;
  uint8_t  access;
  uint8_t  count;
  uint16_t reg_base;
  uint16_t range_min;
  uint16_t range_max;
  uint8_t  unit;
  uint8_t  name_len;
} tnb_cap_desc_t;

#ifdef __cplusplus
}
#endif

#endif /* TNB_PROTOCOL_H */
