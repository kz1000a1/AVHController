//
// AVH(Auto Vehicle Hold) auto introduce and remove system firmware for SUBARU Levorg VN5
//

#include <stdbool.h>
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "led.h"
#include "system.h"
#include "error.h"
#include "printf.h"
#include "subaru_levorg_vnx.h"

#ifdef DEBUG_MODE
    enum debug_mode DebugMode = DEBUG;
#else
    enum debug_mode DebugMode = NORMAL;
#endif

/*
uint16_t bytesToUint(uint8_t raw[], int shift, int size){
    uint16_t result = 0;

    for (int i = 0; i < size; i++){
        result = result << (sizeof raw[0] * 8);
        for (int j = 0; j < sizeof raw[0] * 8; j++){
            result += raw[i + shift] & (1 << j);
        }
    }

    return result;
}
*/

void print_rx_frame(CAN_RxHeaderTypeDef* rx_msg_header, uint8_t* rx_msg_data){
    uint32_t CurrentTime;

    CurrentTime = HAL_GetTick();

    // Output all received message(s) to CDC port as candump -L
    if(rx_msg_header->RTR == CAN_RTR_DATA){ // Data Frame
        printf_("(%d.%03d000) can0 %03X#", CurrentTime / 1000,
                                           CurrentTime % 1000,
                                           rx_msg_header->StdId);
        for (uint8_t i=0; i < rx_msg_header->DLC; i++){
            printf_("%02X", rx_msg_data[i]);
        }
        printf_("\n");
    } else { // Remote Frame
        printf_("(%d.%03d000) can0 %03X#R%d\n", CurrentTime / 1000,
                                                CurrentTime % 1000,
                                                rx_msg_header->StdId,ｍｋ
                                                rx_msg_header->DLC);ｍｋ
    }
}


void print_tx_frame(CAN_TxHeaderTypeDef* tx_msg_header, uint8_t* tx_msg_data){
    uint32_t CurrentTime;

    CurrentTime = HAL_GetTick();

    // Output all received message(s) to CDC port as candump -L
    printf_("# (%d.%03d000) can0 %03X#%02X%02X%02X%02X%02X%02X%02X%02X\n",
                                CurrentTime / 1000,
                                CurrentTime % 1000,
                                tx_msg_header->StdId,
                                tx_msg_data[0],
                                tx_msg_data[1],
                                tx_msg_data[2],
                                tx_msg_data[3],
                                tx_msg_data[4],
                                tx_msg_data[5],
                                tx_msg_data[6],
                                tx_msg_data[7]);
}

void transmit_can_frame(uint8_t* rx_msg_data, bool avh){
    // Storage for transmit message buffer
    CAN_TxHeaderTypeDef tx_msg_header;
    tx_msg_header.IDE = CAN_ID_STD;
    tx_msg_header.StdId = CAN_ID_AVH_CONTROL;
    tx_msg_header.ExtId = 0;
    tx_msg_header.RTR = CAN_RTR_DATA;
    tx_msg_header.DLC = 8;
    uint8_t tx_msg_data[8] = {0};

    if((rx_msg_data[1] & 0x0f) == 0x0f){
        tx_msg_data[1] = rx_msg_data[1] &= 0xf0;
    } else {
        tx_msg_data[1] = rx_msg_data[1] += 0x01;
    }

    if(avh){
        tx_msg_data[2] = rx_msg_data[2] | 0x02; // Introduce auto behicle hold bit on
    } else {
        tx_msg_data[2] = rx_msg_data[2] | 0x01; // Remove auto behicle hold bit on
    }
        
    tx_msg_data[3] = rx_msg_data[3];
    tx_msg_data[4] = rx_msg_data[4];
    tx_msg_data[5] = rx_msg_data[5];
    tx_msg_data[6] = rx_msg_data[6];
    tx_msg_data[7] = rx_msg_data[7];
    // Calculate checksum
    tx_msg_data[0] = (tx_msg_data[1] +
                      tx_msg_data[2] +
                      tx_msg_data[3] +
                      tx_msg_data[4] +
                      tx_msg_data[5] +
                      tx_msg_data[6] +
                      tx_msg_data[7]) + SUM_CHECK_ADDER;
    can_tx(&tx_msg_header, tx_msg_data); // Queueing message
    can_process(); // Transmit message
    if(DebugMode == DEBUG){
        print_tx_frame(&tx_msg_header, tx_msg_data);
    }
}

void led_blink(enum status Status){
    if(Status & 1){
        led_orange_on();
    } else {
        led_orange_off();
    }
    if(Status & 2){
        led_green_on();
    } else {
        led_green_off();
    }
}


int main(void)
{
    // Storage for status and received message buffer
    CAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};

    static enum avh_control_status AvhControlStatus = ENGINE_STOP;
    static bool AvhStatus = AVH_OFF;
    static bool AvhControl = AVH_OFF;
    static bool ParkBrake = BRAKE_ON;
    static enum status Status = PROCESSING;
    static uint8_t Gear = SHIFT_P;
    static uint16_t PreviousCanId = CAN_ID_AVH_CONTROL;
    static uint8_t Retry = 0;
    static float Speed = 0;
    static float PrevSpeed = 0;
    static float Brake = 0;
    static float PrevBrake = 0;
    static float Accel = 0;

    // Initialize peripherals
    system_init();
    can_init();
    led_init();
#ifdef DEBUG_MODE
    usb_init();
#endif

    can_enable();
    led_blink((AvhStatus << 1) + AvhControl);

    while(1){
#ifdef DEBUG_MODE
        cdc_process();
#endif

        // If CAN message receive is pending, process the message
        if(is_can_msg_pending(CAN_RX_FIFO0)){
            can_rx(&rx_msg_header, rx_msg_data);

            if(DebugMode == CANDUMP || (DebugMode == DEBUG && (rx_msg_header.StdId == CAN_ID_SHIFT || rx_msg_header.StdId == CAN_ID_AVH_STATUS || rx_msg_header.StdId == CAN_ID_AVH_CONTROL || rx_msg_header.StdId == CAN_ID_SPEED || rx_msg_header.StdId == CAN_ID_ACCEL))){
                print_rx_frame(&rx_msg_header, rx_msg_data);
            }
            
            if(rx_msg_header.RTR != CAN_RTR_DATA || rx_msg_header.DLC != 8){
                continue;
            }

            if(DebugMode != CANDUMP){
                switch (rx_msg_header.StdId){
                    case CAN_ID_SPEED:
                        PrevSpeed = Speed;
                        PrevBrake = Brake;
                        Speed = (rx_msg_data[2] + ((rx_msg_data[3] & 0x1f) << 8)) * 0.05625;
                        Brake = rx_msg_data[5] / 0.7;
                        if(100 < Brake){
                            Brake = 100;
                        }
                        if((rx_msg_data[7] & 0xf0) == 0x50){
                            ParkBrake = BRAKE_ON;
                        } else {
                            ParkBrake = BRAKE_OFF;
                        }                            

                        switch (AvhStatus){
                            case AVH_ON:
                                if(AvhControl == AVH_ON){
                                    // if(Gear != SHIFT_D || ParkBrake == BRAKE_ON || Speed != 0.0 || Accel != 0.0 || (PrevBrake == 0.0 && Brake != 0.0 && Brake < BRAKE_HIGH)){
                                    if((Gear != SHIFT_D && Brake != 0.0) || ParkBrake == BRAKE_ON || Accel != 0.0 || (PrevBrake == 0.0 && Brake != 0.0 && Brake < BRAKE_HIGH)){
                                        AvhControl = AVH_OFF;
                                        if(Status == SUCCEEDED){
                                            Status = PROCESSING;
                                            // led_blink(Status);
                                        }
                                        led_blink((AvhStatus << 1) + AvhControl);
                                    }
                                }
                                break;

                            case AVH_OFF:
                                if(AvhControl == AVH_OFF){
                                    if(Gear == SHIFT_D && ParkBrake == BRAKE_OFF && Speed == 0.0 && Accel == 0.0 && PrevSpeed == 0.0 && PrevBrake < BRAKE_HIGH && BRAKE_HIGH <= Brake){
                                        AvhControl = AVH_ON;
                                        if(Status == SUCCEEDED){
                                            Status = PROCESSING;
                                            // led_blink(Status);
                                        }
                                        led_blink((AvhStatus << 1) + AvhControl);
                                    }
                                }
                                break;
                        }

                        dprintf_("# Speed: %d.%02d(%d.%02d)km/h\n", (int)Speed, (int)(Speed * 100) % 100, (int)PrevSpeed, (int)(PrevSpeed * 100) % 100);
                        dprintf_("# Accel: %d.%02d%%\n", (int)Accel, (int)(Accel * 100) % 100);
                        dprintf_("# Brake: %d.%02d(%d.%02d)%%\n", (int)Brake, (int)(Brake * 100) % 100, (int)PrevBrake, (int)(PrevBrake * 100) % 100);
                        dprintf_("# Gear: %d(SHIFT_D:D,SHIFT_N:N,SHIFT_R:R,SHIFT_P:P)\n", Gear);
                        dprintf_("# ParkBrake : %d(0:OFF,1:ON)\n", ParkBrake);
                        dprintf_("# AVH: %d(0:OFF,1:ON)=>%d\n", AvhStatus, AvhControl);

                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    case CAN_ID_SHIFT:
                        if(Gear != (rx_msg_data[3] & 0x07)){
                            // 1027 dprintf_("# Information: Change Gear %n to %n.\n", Gear, rx_msg_data[3] & 0x07);
                            Gear = (rx_msg_data[3] & 0x07);
                        }
                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    case CAN_ID_ACCEL:
                        Accel = rx_msg_data[4] / 2.55;
                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    case CAN_ID_AVH_STATUS:
                        if(rx_msg_data[5] & 0x20){
                            AvhStatus = AVH_ON;
                            // Output Information message
                            // 1027 dprintf_("# Information: Auto vehicle hold On.\n");
                            if(Retry != 0 && Status == PROCESSING && AvhControl == AVH_ON){
                                // Output Information message
                                dprintf_("# Information: Enable auto vehicle hold succeeded.\n");
                                Retry = 0;
                                Status = SUCCEEDED;
                                AvhControlStatus = READY;
                                // led_blink(Status);
                            }
                        } else {
                            AvhStatus = AVH_OFF;
                            // Output Information message
                            // 1027 dprintf_("# Information: Auto vehicle hold Off.\n");
                            if(Retry != 0 && Status == PROCESSING && AvhControl == AVH_OFF){
                                // Output Information message
                                dprintf_("# Information: Disble auto vehicle hold succeeded.\n");
                                Retry = 0;
                                Status = SUCCEEDED;
                                AvhControlStatus = READY;
                                // led_blink(Status);
                            }
                        }
                        led_blink((AvhStatus << 1) + AvhControl);

                        // PreviousCanId = rx_msg_header.StdId;
                        break;

                    case CAN_ID_AVH_CONTROL:
                        if(PreviousCanId == CAN_ID_AVH_CONTROL){ // TCU don't transmit message
                            AvhStatus = AVH_OFF;
                            AvhControl = AVH_OFF;
                            ParkBrake = BRAKE_ON;
                            AvhControlStatus = ENGINE_STOP;
                            Status = PROCESSING;
                            // led_blink(Status);
                            Retry = 0;
                            Gear = SHIFT_P;
                            Speed = 0;
                            PrevSpeed = 0;
                            Brake = 0;
                            PrevBrake = 0;
                            Accel = 0;
                            led_blink((AvhStatus << 1) + AvhControl);
                            // Output Information message
                            // 1027 dprintf_("# Information: ENGINE STOP.\n");
                        } else if(Status == PROCESSING){
                            if(AvhControlStatus == NOT_READY || AvhControlStatus == ENGINE_STOP || (AvhStatus == AvhControl)){
                                // 1027 dprintf_("# Information: Status (CCU=%d SCU=%d TCU=%d R=%d).\n", AvhControlStatus, AvhStatus, ShiftStatus, R_Gear);
                                AvhControlStatus = READY;
                                // Output Information message
                                // 1027 dprintf_("# Information: READY.\n");
                                // 1027 dprintf_("# Information: Status (CCU=%d SCU=%d TCU=%d R=%d).\n", AvhControlStatus, AvhStatus, ShiftStatus, R_Gear);
                            // } else if(AvhStatus == AVH_OFF && (! R_Gear) || (AvhStatus == AVH_ON && R_Gear)){ // Transmit message for Enable or disable auto vehicle hold
                            } else if(AvhStatus != AvhControl){ // Transmit message for Enable or disable auto vehicle hold
                                // Output Information message
                                // 1027 dprintf_("# Information: Send Frame Speed=%d R=%d.\n", (int)Speed, R_Gear);
                                if(MAX_RETRY <= Retry){ // Previous enable or disable auto vehicle hold message failed
                                    // Output Warning message
                                    dprintf_("# Warning: Enable or disable auto vehicle hold failed\n");
                                    Status = FAILED;
                                    // led_blink(Status);
                                } else {
                                    Retry++;
                                    // led_blink(Retry);
                                    /* for(int i = 0;i < 5;i++){
                                        HAL_Delay(50);
                                        transmit_can_frame(rx_msg_data, AvhControl); // Transmit can frame for introduce or remove AVH
                                    } */
                                    HAL_Delay(50);
                                    transmit_can_frame(rx_msg_data, AvhControl); // Transmit can frame for introduce or remove AVH
                                    // Discard message(s) that received during HAL_delay()
                                    while(is_can_msg_pending(CAN_RX_FIFO0)){
                                        can_rx(&rx_msg_header, rx_msg_data);
                                    }
                                    rx_msg_header.StdId = CAN_ID_SHIFT;
                                    // AvhControlStatus = NOT_READY;
                                    // led_blink(Status);
                                }
                            } else { // Unexpected case
                                // Output Warning message
                                dprintf_("# Warning: Unexpected case (Status=%d AVH=%d AVHControl=%d).\n", AvhControlStatus, AvhStatus, AvhControl);
                            }
                        }
                        PreviousCanId = rx_msg_header.StdId;
                        break;

                    default: // Unexpected can id
                        // Output Warning message
                        // 1027 dprintf_("# Warning: Unexpected can id (0x%03x).\n", rx_msg_header.StdId);
                        break;
                }
            }
        }
    }
}

