/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "conf_general.h"
#include "commands.h"
#include "app_origin.h"
#include "app.h"
#include "mcpwm_foc.h"
#include "datatypes.h"
#include <string.h>
#include "utils.h"
#include "ch.h"

#ifdef APP_CUSTOM_TO_USE
#include APP_CUSTOM_TO_USE
#endif

#define ADDR_CONF_1 0
#define ADDR_CONF_2 12
#define ADDR_CONF_3 24
#define ADDR_CONF_FLAG 37

#define CONF_SET   22
#define CONF_UN_SET 0XFFFFFFFF

#define RATIO_MIN 0
#define RATIO_MAX 3600
#define MODE_ARM 1
#define MODE_WHEEL 2

#define CMD_READ 0
#define CMD_WRITE 1

extern volatile motor_all_state_t m_motor_1;
eeprom_var RATIO;

_TPara TPara;

static void CTRL_Mode_te_send(uint8_t ax, uint8_t code, uint8_t *data, uint8_t NUM);
static void origin_default_config(void);

static void origin_comm_process(unsigned char *data, unsigned int len)
{
    cmd_org_t ax_cmd;
    ax_cmd.cmd_var = data[0];
    CTRL_Mode_te cmd_buff = ax_cmd.bit.cm;
    app_configuration *CON_ADDR = app_get_configuration();
    if (ax_cmd.bit.id == CON_ADDR->controller_id)
    {
        switch (ax_cmd.bit.cm)
        {
        case CMD_MTR_BET: //2020.11.20上午完成测试
        {
            SHUTDOWN_RESET();
            timeout_reset();
            if (TPara.mode & 0X01)
            {
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }  
        }
        break;

        case CMD_MTR_STA: //2020.11.20上午完成测试
        {
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                if (((*(data + 1)) & 0XF0) == 0XF0)
                {
                    mc_interface_set_current(0.0);
                }

                if (((*(data + 1)) & 0X0F) == 0X0F)
                {
                    m_motor_1.m_pos_pid_now = 0.0;
                    mc_interface_set_pid_pos(0.0); /* code */
                }

                if (TPara.mode & 0X01)
                {
                    mc_fault_code fault;
                    fault = mc_interface_get_fault();
                    *(data + 2) = (uint8_t)fault;
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 2);                    /* code */
                }
            }

            if (ax_cmd.bit.rw == CMD_READ)
            {
                mc_fault_code fault;
                fault = mc_interface_get_fault();
                *(data + 2) = (uint8_t)fault;
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 2);
            }
            timeout_reset();
        }
        break;

        case CMD_WHELL_POS: //2020.11.21 10点18分
        {
            double pos;
            float curr_scale;
            format_bytes_t rx_pos;

            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                mc_configuration *mcconf;
                mcconf = mc_interface_get_configuration();
                memcpy((unsigned char *)(&curr_scale), data + 1 + 8, 4); //取出电流参数
                memcpy((unsigned char *)&pos, data + 1, 8);              //取出目标位置              
                utils_truncate_number(&curr_scale, 0.0, 1.0);            //电流参数范围处理
                mcconf->l_current_max_scale = curr_scale;                //电流参数赋值到系统变量中
                mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
                if ((TPara.Ratio > RATIO_MIN) && (TPara.Ratio < RATIO_MAX))
                {              
                    if (TPara.dir > 0)
                    {
                        mc_interface_set_pid_pos(pos * TPara.Ratio + TPara.pZero);
                    }
                    else
                    {
                        mc_interface_set_pid_pos(-pos * TPara.Ratio - TPara.pZero);
                    }
                }

                if (TPara.mode & 0X01)
                {
                    pos = mc_interface_get_pid_pos_now();
                    if (TPara.dir > 0)
                    {
                        rx_pos.float64[0] =  pos/ TPara.Ratio + TPara.pZero;                  
                        rx_pos.float32[2] = mcpwm_foc_get_tot_current();
                    }
                    else
                    {
                        rx_pos.float64[0] =  -pos/ TPara.Ratio - TPara.pZero;
                        rx_pos.float32[2] = -mcpwm_foc_get_tot_current();
                    }
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, rx_pos.ch, sizeof(double) + sizeof(float));                    /* code */
                }
            }
            if (ax_cmd.bit.rw == CMD_READ)
            {
                pos = mc_interface_get_pid_pos_now();

                if (TPara.dir > 0)
                {
                    rx_pos.float64[0] =  pos/ TPara.Ratio + TPara.pZero;                  
                    rx_pos.float32[2] = mcpwm_foc_get_tot_current();
                }
                else
                {
                    rx_pos.float64[0] =  -pos/ TPara.Ratio - TPara.pZero;
                    rx_pos.float32[2] = -mcpwm_foc_get_tot_current();
                }
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, rx_pos.ch, sizeof(double) + sizeof(float));
            }
            timeout_reset();
        }
        break;

        case CMD_JOINT_POS:
        {
            float curr_scale,pos;
            format_bytes_t rx_pos;

            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                mc_configuration *mcconf;
                mcconf = mc_interface_get_configuration();
                memcpy((unsigned char *)(&curr_scale), data + 1 + 8, 4); //取出电流参数
                memcpy((unsigned char *)&pos, data + 1, 4);              //取出目标位置
                utils_truncate_number(&curr_scale, 0.0, 1.0);            //电流参数范围处理
                mcconf->l_current_max_scale = curr_scale;                //电流参数赋值到系统变量中
                mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
                if ((TPara.Ratio > RATIO_MIN) && (TPara.Ratio < RATIO_MAX) && ((pos < TPara.pMax) && (pos > TPara.pMin)))
                {                  
                    if (TPara.dir > 0)
                    {
                        mc_interface_set_pid_pos(pos * TPara.Ratio + TPara.pZero);
                    }
                    else
                    {
                        mc_interface_set_pid_pos(-pos * TPara.Ratio - TPara.pZero);
                    }
                }

                if (TPara.mode & 0X01)
                {
                    pos = (float)mc_interface_get_pid_pos_now();
                    if (TPara.dir > 0)
                    {
                        rx_pos.float32[0] =  (pos/ TPara.Ratio + TPara.pZero);                     
                        rx_pos.float32[2] = mcpwm_foc_get_tot_current();
                    }
                    else
                    {
                        rx_pos.float32[0] =  (-pos/ TPara.Ratio - TPara.pZero);
                        rx_pos.float32[2] = -mcpwm_foc_get_tot_current();
                    }
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, rx_pos.ch, sizeof(double) + sizeof(float));                    /* code */
                }
            }
            if (ax_cmd.bit.rw == CMD_READ)
            {
                
                pos = (float)mc_interface_get_pid_pos_now();

                if (TPara.dir > 0)
                {
                    rx_pos.float32[0] =  (pos/ TPara.Ratio + TPara.pZero);                     
                    rx_pos.float32[2] = mcpwm_foc_get_tot_current();
                }
                else
                {
                    rx_pos.float32[0] =  (-pos/ TPara.Ratio - TPara.pZero);
                    rx_pos.float32[2] = -mcpwm_foc_get_tot_current();
                }
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, rx_pos.ch, sizeof(double) + sizeof(float));
            }
            timeout_reset();
        }
        break;

        case CMD_CUR: //2020.11.20上午完成测试
        {
            float curr = 2.0;
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                memcpy((uint8_t *)&curr, data + 9, sizeof(float));
                if (TPara.dir > 0)
                {
                    mc_interface_set_current(curr);
                }
                else
                {
                    mc_interface_set_current(-curr);
                }

                if (TPara.mode & 0x01)
                {
                    curr = mc_interface_get_tot_current();
                    if (TPara.dir < 0)
                    {
                        curr = -curr;
                    }
                    memset(data + 1, 0X55, 12);
                    memcpy(data + 9, (uint8_t *)&curr, sizeof(float));
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);                    /* code */
                }
            }

            if (ax_cmd.bit.rw == CMD_READ)
            {
                curr = mc_interface_get_tot_current();
                if (TPara.dir < 0)
                {
                    curr = -curr;
                }
                memset(data + 1, 0X55, 12);
                memcpy(data + 9, (uint8_t *)&curr, sizeof(float));
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
            timeout_reset();
        }
        break;

        case CMD_PID_POS: //2020.11.21 10点24分
        {
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                float p, i, d;
                mc_configuration *mcconf;
                memcpy((unsigned char *)&(p), data + 1, 4);
                memcpy((unsigned char *)&(i), data + 1 + 4, 4);
                memcpy((unsigned char *)&(d), data + 1 + 8, 4);
                mcconf = mc_interface_get_configuration();
                mcconf->p_pid_kp = p;
                mcconf->p_pid_ki = i;
                mcconf->p_pid_kd = d; /* code */

                if (TPara.mode &0X01)
                {
                    mcconf = mc_interface_get_configuration();
                    memcpy(data + 1, &(mcconf->p_pid_kp), 4);
                    memcpy(data + 1 + 4, &(mcconf->p_pid_ki), 4);
                    memcpy(data + 1 + 8, &(mcconf->p_pid_kd), 4);
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);                     /* code */
                }              
            }

            if (ax_cmd.bit.rw == CMD_READ)
            {
                mc_configuration *mcconf;
                mcconf = mc_interface_get_configuration();
                memcpy(data + 1, &(mcconf->p_pid_kp), 4);
                memcpy(data + 1 + 4, &(mcconf->p_pid_ki), 4);
                memcpy(data + 1 + 8, &(mcconf->p_pid_kd), 4);
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
        }
        break;

        case CMD_PID_SPD: //2020.11.21 10点31分
        {
            mc_configuration *mcconf;
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                float p, i, d;
                memcpy((unsigned char *)&(p), data + 1, 4);
                memcpy((unsigned char *)&(i), data + 1 + 4, 4);
                memcpy((unsigned char *)&(d), data + 1 + 8, 4);
                mcconf = mc_interface_get_configuration();
                mcconf->s_pid_kp = p;
                mcconf->s_pid_ki = i;
                mcconf->s_pid_kd = d; /* code */

                if (TPara.mode & 0X01)
                {
                    memcpy(data + 1, &(mcconf->s_pid_kp), 4);
                    memcpy(data + 1 + 4, &(mcconf->s_pid_ki), 4);
                    memcpy(data + 1 + 8, &(mcconf->s_pid_kd), 4);
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);                    /* code */
                }
            }

            if (ax_cmd.bit.rw == CMD_READ)
            {            
                mcconf = mc_interface_get_configuration();
                memcpy(data + 1, &(mcconf->s_pid_kp), 4);
                memcpy(data + 1 + 4, &(mcconf->s_pid_ki), 4);
                memcpy(data + 1 + 8, &(mcconf->s_pid_kd), 4);
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
        }
        break;

        case CMD_RPM_CUR: //2020.11.21 10点46分
        {
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                float e, curr_scale;
                mc_configuration *mcconf = mc_interface_get_configuration();
                memcpy((unsigned char *)&(curr_scale), data + 1 + 8, 4); //取出电流参数
                utils_truncate_number(&curr_scale, 0.0, 1.0);            //电流参数范围处理
                mcconf->l_current_max_scale = curr_scale;                //电流参数赋值到系统变量中
                mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
                memcpy((unsigned char *)&(e), data + 1 + 4, 4);
                if (e < TPara.vMax)
                {
                    if (TPara.Ratio > RATIO_MIN && TPara.Ratio < RATIO_MAX)
                    {
                        if (TPara.dir > 0)
                        {
                            mc_interface_set_pid_speed(e * TPara.Ratio);
                        }
                        else
                        {
                            mc_interface_set_pid_speed(-e * TPara.Ratio);
                        }
                    }
                } /* code */

                if (TPara.mode & 0X01)
                {
                    float s;
                    memset(data + 1, 0x55, 12);
                    s = mc_interface_get_rpm()* TPara.Ratio;
                    if (TPara.dir < 0)
                        s = -s;
                    memcpy(data + 1 + 4, &s, 4);
                    s = mcpwm_foc_get_tot_current();
                    if (TPara.dir < 0)
                        s = -s;
                    memcpy(data + 1 + 8, &s, 4);
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
                }
            }

            if (ax_cmd.bit.rw == CMD_READ)
            {
                float s;
                memset(data + 1, 0x55, 12);
                s = mc_interface_get_rpm()* TPara.Ratio;
                if (TPara.dir < 0)
                    s = -s;
                memcpy(data + 1 + 4, &s, 4);
                s = mcpwm_foc_get_tot_current();
                if (TPara.dir < 0)
                    s = -s;
                memcpy(data + 1 + 8, &s, 4);
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
            timeout_reset();
        }
        break;

        case CMD_MTR_CFG: //2020.11.20.20点55分
        {
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                (CON_ADDR->controller_id) = *(data + 1);
                TPara.ID = *(data + 1);
                TPara.dir = *(data + 2);
                TPara.mode = *(data + 3);
                TPara.Tao = *(data + 4);
                memcpy((unsigned char *)&(TPara.I2R), data + 5, 4);
                memcpy((unsigned char *)&(TPara.Ratio), data + 9, 4);
                for (size_t i = 0; i < 3; i++)
                {
                    conf_general_store_eeprom_var_custom((eeprom_var *)(&TPara) + i, ADDR_CONF_1 + i * 4); /* code */
                }


                if (TPara.mode & 0X01)
                {
                    memcpy(data + 1, &TPara, 12);
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);                   /* code */
                }
            }

            if (ax_cmd.bit.rw == CMD_READ)
            {
                memcpy(data + 1, &TPara, 12);
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
        }
        break;

        case CMD_MEC_POS: //2020.11.20.20点55分
        {
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                memcpy((unsigned char *)&(TPara.pZero), data + 1, 4);
                memcpy((unsigned char *)&(TPara.pMin), data + 5, 4);
                memcpy((unsigned char *)&(TPara.pMax), data + 9, 4);
                for (size_t i = 0; i < 3; i++)
                {
                    conf_general_store_eeprom_var_custom((eeprom_var *)(&TPara) + 3 + i, ADDR_CONF_2 + i * 4);
                } /* code */

                if (TPara.mode & 0X01)
                {
                    memcpy(data + 1, (uint8_t *)(&TPara) + 12, 12);
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);                    /* code */
                }

            }

            if (ax_cmd.bit.rw == CMD_READ)
            {
                memcpy(data + 1, (uint8_t *)(&TPara) + 12, 12);
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
        }
        break;

        case CMD_TRA_CFG: //2020.11.20。20点54分
        {
            if (ax_cmd.bit.rw == CMD_WRITE)
            {
                memcpy((unsigned char *)&(TPara.vMax), data + 1, 4);
                memcpy((unsigned char *)&(TPara.aMax), data + 5, 4);
                memcpy((unsigned char *)&(TPara.jMax), data + 9, 4);
                for (size_t i = 0; i < 3; i++)
                {
                    conf_general_store_eeprom_var_custom((eeprom_var *)(&TPara) + 6 + i, ADDR_CONF_3 + i * 4);
                }

                if (TPara.mode & 0X01)
                {
                    memcpy(data + 1, (uint8_t *)(&TPara) + 24, 12);
                    CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
                }
            }
            if (ax_cmd.bit.rw == CMD_READ)
            {
                memcpy(data + 1, (uint8_t *)(&TPara) + 24, 12);
                CTRL_Mode_te_send(ax_cmd.bit.id, ax_cmd.bit.cm, data + 1, 12);
            }
        }
        break;

        default:
            break;
        }
    }
}

void origin_init(void)
{
    eeprom_var CON_FLAG; 
    app_configuration *CON_ADDR = app_get_configuration();
    CON_FLAG.as_u32=CONF_UN_SET;
    conf_general_read_eeprom_var_custom(&CON_FLAG, ADDR_CONF_FLAG);
    if(CON_FLAG.as_u32 != CONF_SET)
    {
        origin_default_config(); 
    }
    for (size_t i = 0; i < sizeof(TPara) / 4; i++)
    {
       conf_general_read_eeprom_var_custom((eeprom_var *)(&TPara) + i, ADDR_CONF_1 + 4 * i);
    }
    CON_ADDR->controller_id = TPara.ID;

    commands_set_app_data_handler(origin_comm_process);
}

uint8_t CheckSum(uint8_t *Ptr, uint8_t Num)
{
    uint8_t Sum = 0;
    while (Num--)
    {
        Sum += *Ptr;
        Ptr++;
    }
    return Sum;
}

static void CTRL_Mode_te_send(uint8_t ax, uint8_t code, uint8_t *data, uint8_t NUM)
{
    cmd_org_t ax_cmd;
    uint8_t buff[ORIGIN_FRAME_LENTH];
    memset(buff, ORIGIN_FILL_VALUE, ORIGIN_FRAME_LENTH);
    ax_cmd.bit.id = ax;
    ax_cmd.bit.cm = code;
    buff[0] = ORIGIN_FRAME_START;
    buff[ORIGIN_FRAME_LENTH - 1] = ORIGIN_FRAME_END;
    buff[1] = ax_cmd.cmd_var;
    memcpy(buff + 2, data, NUM);
    buff[ORIGIN_FRAME_LENTH - 2] = CheckSum(buff + ORIGIN_FRAME_CHECK_BEGIN, ORIGIN_FRAME_CHECK_NUM);

    commands_send_packet(buff, 0);
}

static void origin_default_config(void)
{
    eeprom_var stor_f;
    TPara.aMax = 1;
    TPara.dir = 1;
    TPara.I2R = 10;
    TPara.ID = 0;
    TPara.jMax = 1;
    TPara.mode = 0;
    TPara.pMax = 180;
    TPara.pMin = 0;
    TPara.pZero = 0;
    TPara.Ratio = 1;
    TPara.Tao = 1;
    TPara.vMax = 200;
    for (size_t i = 0; i < 9; i++)
    {
        conf_general_store_eeprom_var_custom((eeprom_var *)(&TPara) + i, ADDR_CONF_1 + i * 4);
    }
    stor_f.as_u32 = CONF_SET;
    conf_general_store_eeprom_var_custom(&stor_f, ADDR_CONF_FLAG);
}