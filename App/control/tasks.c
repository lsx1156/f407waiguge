#include "tasks.h"
#include "main.h"
#include "udp_protocol.h"
#include "control_isr.h"
#include "safety.h"
#include "eeprom.h"
#include "can_motor.h"
#include "bsp_config.h"
#include "contract.h"
#include "joint_unit.h"
#include "global_coordinator.h"
#include "global_phase.h"
#include "global_pose.h"
#include "lwip_comm.h"
#include "udp_net.h"
#include <string.h>

void network_task_run(void)
{
    static uint8_t comm_lost_flag = 0;
    static uint8_t comm_miss_cnt = 0;      /* 连续超时计数，达到 COMM_HEARTBEAT_MISS_CNT 才判故障 */

    lwip_pkt_handle();
    lwip_periodic_handle(10);

    can_tx_task_drain();
    
    ReportFrame_t frame;
    static uint32_t report_cnt = 0;
    while (report_fifo_read(&frame)) {
        udp_net_send((uint8_t*)&frame, sizeof(ReportFrame_t));
        report_cnt++;
    }
    
    /* 每秒打印一次调试信息 */
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print >= 1000) {
        last_print = HAL_GetTick();
        extern volatile uint32_t g_dbg_isr_calls;
        extern volatile uint32_t g_dbg_fifo_write_ok;
        extern volatile uint32_t g_dbg_fifo_write_fail;
        extern volatile uint32_t g_dbg_eth_rx_frames;
        extern volatile uint32_t g_dbg_eth_input_calls;
        extern volatile uint32_t g_dbg_eth_getframe_fail;
        extern volatile uint32_t g_dbg_eth_dmasr;
        extern volatile uint32_t g_dbg_eth_desc0_status;
        extern volatile uint32_t g_dbg_eth_irq_cnt;
        /* v1.6.6: 暂时关闭 DBG 打印, 减少串口噪音, 便于按键调试 */
#if 0
        printf("[DBG] isr=%lu wr=%lu/%lu sent=%lu eth_in=%lu rx=%lu rx_fail=%lu irq=%lu\r\n",
               (unsigned long)g_dbg_isr_calls,
               (unsigned long)g_dbg_fifo_write_ok,
               (unsigned long)g_dbg_fifo_write_fail,
               (unsigned long)report_cnt,
               (unsigned long)g_dbg_eth_input_calls,
               (unsigned long)g_dbg_eth_rx_frames,
               (unsigned long)g_dbg_eth_getframe_fail,
               (unsigned long)g_dbg_eth_irq_cnt);
        printf("[DBG] DMASR=0x%08lX Desc0=0x%08lX\r\n",
               (unsigned long)g_dbg_eth_dmasr,
               (unsigned long)g_dbg_eth_desc0_status);
#endif
        g_dbg_isr_calls = 0;
        g_dbg_fifo_write_ok = 0;
        g_dbg_fifo_write_fail = 0;
        g_dbg_eth_rx_frames = 0;
        g_dbg_eth_input_calls = 0;
        g_dbg_eth_getframe_fail = 0;
        g_dbg_eth_irq_cnt = 0;
        report_cnt = 0;
    }
    
    uint16_t fault_code;
    while (fault_fifo_read(&fault_code)) {
        HeartbeatFrame_t hb_frame;
        hb_frame.header.frame_type = FRAME_TYPE_HEARTBEAT;
        hb_frame.header.reserved = 0;
        hb_frame.header.seq_num = 0;
        hb_frame.header.timestamp = HAL_GetTick();
        hb_frame.data = fault_code;
        hb_frame.crc = crc16((uint8_t*)&hb_frame, sizeof(HeartbeatFrame_t) - 2);
        udp_net_send((uint8_t*)&hb_frame, sizeof(HeartbeatFrame_t));
    }
    
    JointCommand_t new_cmd[6];
    while (command_fifo_read(new_cmd)) {
        safe_cmd_write(new_cmd);
    }

    /* 心跳超时检测：连续 COMM_HEARTBEAT_MISS_CNT 次超时才判故障，避免偶发丢包误触发 */
    /* STANDALONE 模式下: 完全忽略心跳超时，不触发 COMM_LOST 故障 */
    if (g_comm_mode == COMM_MODE_HOST &&
        g_last_comm_ts != 0 && (HAL_GetTick() - g_last_comm_ts > COMM_HEARTBEAT_TIMEOUT_MS)) {
        if (comm_miss_cnt < COMM_HEARTBEAT_MISS_CNT) {
            comm_miss_cnt++;
        }
        if (comm_miss_cnt >= COMM_HEARTBEAT_MISS_CNT && !comm_lost_flag) {
            comm_lost_flag = 1;
            fault_fifo_write(FAULT_COMM_LOST);
        }
    } else {
        comm_miss_cnt = 0;
        comm_lost_flag = 0;
    }

    static uint32_t last_hb_send = 0;
    static uint16_t hb_seq = 0;
    if (HAL_GetTick() - last_hb_send >= 200) {
        last_hb_send = HAL_GetTick();
        HeartbeatFrame_t hb;
        memset(&hb, 0, sizeof(hb));
        hb.header.frame_type = FRAME_TYPE_HEARTBEAT;
        hb.header.seq_num = hb_seq++;
        hb.header.timestamp = HAL_GetTick();
        hb.data = g_safety_state.fault_code;
        hb.crc = crc16((uint8_t*)&hb, sizeof(HeartbeatFrame_t) - 2);
        udp_net_send((uint8_t*)&hb, sizeof(HeartbeatFrame_t));
    }
}

/* ★ v1.7: 工业负载估计器 — 主循环 100Hz 调用
 * 低通滤波力矩 → 准静态负载 (去除人力高频)
 * 用于 ABO 增益调度: 轻载高增益, 重载低增益 */
void industrial_load_estimator(void)
{
    static int32_t load_lpf[6] = {0};

    for (int i = 0; i < JOINT_COUNT; i++) {
        if (!g_abo_state[i].industrial_mode) continue;

        /* 架构评审 #2: 经 JointUnit 统一取力矩, 去除 i<2?leg:arm 分支 */
        int32_t tau = joint_status_ptr(i)->torque;

        /* 一阶 LPF: alpha=50/1024 ≈ 0.049, fc ≈ 0.8 Hz @ 100Hz 采样 */
        load_lpf[i] += (50 * (tau - load_lpf[i])) >> 10;
        g_abo_state[i].load_est_q10 = load_lpf[i];
    }
}

void control_task_run(void)
{
    /* v1.7 legacy: 单关节低通负载估计 → ABO 增益调度 (保持 100Hz, legacy 路径) */
    industrial_load_estimator();

    /* v1.8.1 P1-1b: §C/§D/§E 降频到 GLOBAL_TASK_RATE_HZ (默认 50Hz, 每 2 拍调一次)
     *   节省 CPU; safety_task_run/safety_handle_fault 保持 100Hz 不变.
     *   100/GLOBAL_TASK_RATE_HZ 为编译期常量, 无运行时除法. */
    {
        static uint8_t global_div = 0;
        if (++global_div >= (uint8_t)(100u / GLOBAL_TASK_RATE_HZ)) {
            global_div = 0;

            /* §C 全局协调器: 全身运动学逆动力学 → 末端外力 F_ext
             * → 回映射 per-joint payload_tau → 下发给 JointUnit.eso3.payload_est
             *    供 §B ESO3 模型前馈剥离 "手里的 20kg" 这类全局负载
             * → 负载突变时触发 §C-boost (ESO β3×3 + 遗忘旧 z3) */
            Global_LoadEstimator_Update100Hz();

            /* §D 相位调度器: 模式识别 (GAIT/SQUAT/CARRY/FREE) + 相位/导纳映射
             * → 下发 adm_scale → JointUnitState.global_adm_scale */
            Global_PhaseScheduler_Update100Hz();

            /* §E 位姿估计器: 接触检测 + ZUPT + 基座位姿 → 重力向量 (基座系)
             *    供 §B Gravity_Comp 修正 (数据流接入待定) */
            Global_Pose_Estimator_Update100Hz();
        }
    }

    safety_task_run();
    safety_handle_fault();
}

void eeprom_task_run(void)
{
    eeprom_process_write_buffer();
}

void tasks_init(void)
{
    eeprom_init();
    udp_net_init();
    global_coordinator_init();   /* §C 全局协调器 (100Hz 负载估计器) */
    global_phase_init();         /* §D 相位调度器 (GAIT/SQUAT/CARRY/FREE) */
    global_pose_init();          /* §E 位姿估计器 (ZUPT + 重力向量) */

    /* §E → §B 数据流闭环: 绑定基座系重力向量指针 (上电一次)
     *   指向 g_global_pose.gravity_base.gx (GravityVec_t 三个连续 float)
     *   1kHz ISR 只读, Cortex-M4 单 float 原子, 无撕裂
     * P0-1: 绑定期间关全局中断, 避免 ISR 在指针半写入时读到野指针.
     *   __disable_irq/__enable_irq 由 core_cm4.h (stm32f4xx_hal.h 间接 include) 提供. */
    __disable_irq();
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        joint_bind_gravity_vector(i, &g_global_pose.gravity_base.gx);
    }
    __enable_irq();
}
