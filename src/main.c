//
// AVH(Auto Vehicle Hold) auto introduce and remove system firmware for SUBARU Levorg VN5
//

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
                                                rx_msg_header->StdId,
                                                rx_msg_header->DLC);
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

void transmit_can_frame(uint8_t* rx_msg_data, uint8_t avh){
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
#ifdef DEBUG_MODE
    print_tx_frame(&tx_msg_header, tx_msg_data);
#endif
}

void init_param(struct param* VnxParam){
    VnxParam->AvhStatus = AVH_OFF;
    VnxParam->ParkBrake = ON;
    VnxParam->SeatBelt = OPEN;
    VnxParam->Door = OPEN;
    VnxParam->EyeSight = HOLD;
    VnxParam->Gear = SHIFT_P;
    VnxParam->Speed = 0;
    VnxParam->Brake = 0;
    VnxParam->Accel = 0;
}

void print_param(struct param* VnxParam, uint8_t AvhControl, float PrevSpeed, float PrevBrake, float MaxBrake){
    dprintf_("# DEBUG Speed:%d.%02d(%d.%02d)km/h\n", (int)VnxParam->Speed, (int)(VnxParam->Speed * 100) % 100, (int)PrevSpeed, (int)(PrevSpeed * 100) % 100);
    dprintf_("# DEBUG Accel:%d.%02d%%\n", (int)VnxParam->Accel, (int)(VnxParam->Accel * 100) % 100);
    dprintf_("# DEBUG Brake:%d.%02d(%d.%02d)%% / MAX: %d.%02d%%\n", (int)VnxParam->Brake, (int)(VnxParam->Brake * 100) % 100, (int)PrevBrake, (int)(PrevBrake * 100) % 100, (int)MaxBrake, (int)(MaxBrake * 100) % 100);
    dprintf_("# DEBUG Gear:%d(1:D,2:N,3:R,4:P)\n", VnxParam->Gear);
    dprintf_("# DEBUG ParkBrake:%d(0:OFF,1:ON)\n", VnxParam->ParkBrake);
    dprintf_("# DEBUG AVH:%d(0:OFF,1:ON,3:HOLD)=>%d\n", VnxParam->AvhStatus, AvhControl);
    dprintf_("# DEBUG Door:%d(0:CLOSE,1:OPEN)\n", VnxParam->Door);
    dprintf_("# DEBUG Belt:%d(0:CLOSE,1:OPEN)\n", VnxParam->SeatBelt);
    dprintf_("# DEBUG EyeSight(HOLD):%d(0:OFF,1:ON)\n", VnxParam->EyeSight);
}

void led_blink(uint8_t Status){
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
    static enum prog_status ProgStatus = PROCESSING;
    static uint16_t PreviousCanId = CAN_ID_AVH_CONTROL;
    static uint8_t AvhControl = AVH_OFF;
    static uint8_t PrevAvhStatus = AVH_OFF;
    static uint8_t Retry = 0;
    static uint8_t Led = OFF;
    static uint8_t RepressBrake = OFF;
    static float PrevSpeed = 0;
    static float PrevBrake = 0;
    static float MaxBrake = 0;
    static struct param VnxParam;

    init_param(&VnxParam);

    // Initialize peripherals
    system_init();
    can_init();
    led_init();
#ifdef DEBUG_MODE
    usb_init();
#endif

    can_enable();
    led_blink((VnxParam.AvhStatus << 1) + AvhControl);

    while(1){
#ifdef DEBUG_MODE
        cdc_process();
#endif

        // If CAN message receive is pending, process the message
        if(is_can_msg_pending(CAN_RX_FIFO0)){
            can_rx(&rx_msg_header, rx_msg_data);
            
            if(rx_msg_header.RTR != CAN_RTR_DATA || rx_msg_header.DLC != 8){
                continue;
            }

            switch (rx_msg_header.StdId){
                case CAN_ID_SPEED:
                    PrevSpeed = VnxParam.Speed;
                    PrevBrake = VnxParam.Brake;
                    VnxParam.Speed = (rx_msg_data[2] + ((rx_msg_data[3] & 0x1f) << 8)) * 0.05625;
                    VnxParam.Brake = rx_msg_data[5] / 0.8;
                    if(100 < VnxParam.Brake){
                        VnxParam.Brake = 100;
                    }
                    if(MaxBrake < VnxParam.Brake){
                        MaxBrake = VnxParam.Brake;
                    }
                    VnxParam.ParkBrake = ((rx_msg_data[7] & 0xf0) == 0x50);

                    switch (VnxParam.AvhStatus){
                        case AVH_HOLD:
                            // if(VnxParam.Gear == SHIFT_D && PrevBrake == 0.0 && VnxParam.Brake != 0.0){
                            if(PrevBrake == 0.0 && VnxParam.Brake != 0.0){
                                RepressBrake = ON; // AVH HOLD shall be released by press brake again
                                dprintf_("# DEBUG AVH:%d(0:OFF,1:ON,3:HOLD) RepressBrake:%d(0:OFF,1:ON)\n", VnxParam.AvhStatus, RepressBrake);
                            }
                            if(AvhControl == AVH_ON){
                                if(
                                   // AVH HOLD only
                                   VnxParam.Gear != SHIFT_D && BRAKE_LOW <= VnxParam.Brake
                                  ){
                                    AvhControl = AVH_OFF;
                                    // if(ProgStatus == SUCCEEDED){
                                    //    ProgStatus = PROCESSING;
                                    // }
                                    if(ProgStatus != CANCELLED && ProgStatus != FAILED){
                                        led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                                    }
                                    print_param(&VnxParam, AvhControl, PrevSpeed, PrevBrake, MaxBrake);
                                }
                            }
                            break;

                        case AVH_OFF:
                            if(RepressBrake == ON)
                                if(VnxParam.Brake == 0.0){
                                    RepressBrake = OFF; // AVH HOLD Available
                                    dprintf_("# DEBUG AVH:%d(0:OFF,1:ON,3:HOLD) RepressBrake:%d(0:OFF,1:ON)\n", VnxParam.AvhStatus, RepressBrake);
                                }
                            }
                            if(AvhControl == AVH_OFF){
                                if(RepressBrake == OFF && VnxParam.Gear == SHIFT_D && VnxParam.ParkBrake == OFF && VnxParam.Speed == 0.0 && VnxParam.Accel == 0.0 && VnxParam.SeatBelt == CLOSE && VnxParam.Door == CLOSE && VnxParam.EyeSight == UNHOLD && PrevSpeed == 0.0 && PrevBrake < BRAKE_HIGH && BRAKE_HIGH <= VnxParam.Brake){
                                    AvhControl = AVH_ON;
                                    // if(ProgStatus == SUCCEEDED){
                                    //     ProgStatus = PROCESSING;
                                    // }
                                    if(ProgStatus != CANCELLED && ProgStatus != FAILED){
                                        led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                                    }
                                    print_param(&VnxParam, AvhControl, PrevSpeed, PrevBrake, MaxBrake);
                                }
                            }
                            break;
                        
                        case AVH_ON:
                            if(RepressBrake == ON)
                                if(VnxParam.Brake == 0.0){
                                    RepressBrake = OFF; // AVH HOLD Available
                                    dprintf_("# DEBUG AVH:%d(0:OFF,1:ON,3:HOLD) RepressBrake:%d(0:OFF,1:ON)\n", VnxParam.AvhStatus, RepressBrake);
                                }
                            }
                    }

                    PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_SHIFT:
                    VnxParam.Gear = (rx_msg_data[3] & 0x07);
                    PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_ACCEL:
                    VnxParam.Accel = rx_msg_data[4] / 2.55;
                    PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_BELT:
                    VnxParam.SeatBelt = ((rx_msg_data[6] & 0x01) == 0x01);
                    if(VnxParam.SeatBelt == OPEN && (ProgStatus == FAILED || ProgStatus == CANCELLED)){
                        switch(VnxParam.AvhStatus){
                            case AVH_ON:
                                AvhControl = AVH_OFF;
                                break;
                            
                            default: // AVH_HOLD or AVH_OFF
                                AvhControl = (VnxParam.AvhStatus & 0b01);
                                break;
                        }
                        Retry = 0;
                        ProgStatus = PROCESSING;
                        led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                        dprintf_("# INFO AVH control restarted.\n");
                    }
                    // PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_DOOR:
                    VnxParam.Door = ((rx_msg_data[4] & 0x01) == 0x01);
                    // PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_EYESIGHT:
                    VnxParam.EyeSight = ((rx_msg_data[7] & 0x10) == 0x10);
                    PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_AVH_STATUS:
                    PrevAvhStatus = VnxParam.AvhStatus;
                    VnxParam.AvhStatus = ((rx_msg_data[5] & 0x20) == 0x20) + (((rx_msg_data[5] & 0x22) == 0x22) << 1);

                    if((PrevAvhStatus & 0b1) != (VnxParam.AvhStatus & 0b01)){ // AVH_OFF <=> AVH_ON/AVH_HOLD
                        if(Retry != 0 && ProgStatus == PROCESSING && AvhControl == (VnxParam.AvhStatus & 0b1)){
                            // Output Information message
                            dprintf_("# INFO AVH:%d(0:OFF,1:ON,3:HOLD) succeeded. Retry:%d\n", VnxParam.AvhStatus, Retry);
                            Retry = 0;
                            // ProgStatus = SUCCEEDED;
                            // AvhControlStatus = READY;
                        }
                        if(ProgStatus != CANCELLED && ProgStatus != FAILED){
                            led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                        }
                    } else {
                        if((PrevAvhStatus == AVH_HOLD) && (VnxParam.AvhStatus == AVH_ON)){ // AVH_HOLD => AVH_ON
                            AvhControl = AVH_OFF;
                            // if(ProgStatus == SUCCEEDED){
                            //     ProgStatus = PROCESSING;
                            // }
                            if(ProgStatus != CANCELLED && ProgStatus != FAILED){
                                led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                            }
                            dprintf_("# INFO AVH HOLD Released. RepressBrake:%d(0:OFF,1:ON)\n", RepressBrake);
                        }
                    }

                    // PreviousCanId = rx_msg_header.StdId;
                    break;

                case CAN_ID_AVH_CONTROL:
                    if(PreviousCanId == CAN_ID_AVH_CONTROL){ // Engine is stopped
                        if(AvhControlStatus != ENGINE_STOP){
                            AvhControlStatus = ENGINE_STOP;
                            ProgStatus = PROCESSING;
                            AvhControl = AVH_OFF;
                            PrevAvhStatus = AVH_OFF;
                            Retry = 0;
                            RepressBrake = OFF;
                            PrevSpeed = 0;
                            PrevBrake = 0;
                            init_param(&VnxParam);
                            led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                            dprintf_("# INFO ENGINE STOP.\n");
                        }
                    } else {
                        if((rx_msg_data[2] & 0x03) != 0x0){
                            if(ProgStatus != CANCELLED){
                                ProgStatus = CANCELLED;
                                Led = OFF;
                                dprintf_("# INFO AVH control cancelled.\n");
                            }
                        }
                            
                        switch(ProgStatus){
                            case CANCELLED:
                            case FAILED:
                                if((rx_msg_data[2] & 0x03) == 0x0){
                                    if(Led){
                                        led_blink((!VnxParam.AvhStatus << 1) + (!AvhControl & 0x01));
                                        Led = OFF;
                                    } else {
                                        led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                                        Led = ON;
                                    }
                                }
                                break;
                            
                            case PROCESSING:
                                switch(AvhControlStatus){
                                    case READY:
                                        switch(VnxParam.AvhStatus){
                                            case AVH_HOLD:
                                                if(AvhControl == AVH_OFF){
                                                    if(VnxParam.Gear == SHIFT_D || VnxParam.Brake < BRAKE_LOW){
                                                        dprintf_("# INFO AVH OFF request cancelled. Retry:%d\n", Retry);
                                                        print_param(&VnxParam, AvhControl, PrevSpeed, PrevBrake, MaxBrake);
                                                        Retry = 0;
                                                        AvhControl = AVH_ON;
                                                        led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                                                    }
                                                }
                                                break;
                                            
                                            case AVH_ON:
                                                if(AvhControl == AVH_ON){
                                                    dprintf_("# ERROR AVH HOLD failed. RepressBrake:%d(0:OFF,1:ON)\n", RepressBrake);
                                                    print_param(&VnxParam, AvhControl, PrevSpeed, PrevBrake, MaxBrake);
                                                    // Retry = 0;
                                                    AvhControl = AVH_OFF;
                                                    led_blink((VnxParam.AvhStatus << 1) + AvhControl);                     
                                                }
                                                break;
                                            
                                            default: // AVH_OFF
                                                break;
                                  
                                        }

                                        if((VnxParam.AvhStatus & 0b01) != AvhControl){ // Transmit message for Enable or disable auto vehicle hold
                                            if(MAX_RETRY <= Retry){ // Previous enable or disable auto vehicle hold message failed
                                                // Output Warning message
                                                ProgStatus = FAILED;
                                                Led = OFF;
                                                dprintf_("# ERROR AVH:%d(0:OFF,1:ON) failed. Retry:%d\n", AvhControl, Retry);
                                            } else {
                                                Retry++;
                                                for(int i = 0;i < 2;i++){
                                                    HAL_Delay(50);
                                                    transmit_can_frame(rx_msg_data, AvhControl); // Transmit can frame for introduce or remove AVH
                                                }
                                                // Discard message(s) that received during HAL_delay()
                                                while(is_can_msg_pending(CAN_RX_FIFO0)){
                                                    can_rx(&rx_msg_header, rx_msg_data);
                                                }
                                                // rx_msg_header.StdId = CAN_ID_SHIFT
                                            }
                                        }
                                        break;
                                    
                                    case ENGINE_STOP:
                                        dprintf_("# INFO ENGINE start.\n");
                                    case WAIT:
                                        AvhControlStatus = READY;
                                        break;
                                }
                                break;

                            /*
                            case SUCCEEDED:
                                switch(AvhControlStatus){
                                    case READY:
                                        if(VnxParam.AvhStatus == AVH_ON){
                                            AvhControl = AVH_OFF;
                                            ProgStatus = PROCESSING;
                                            dprintf_("# INFO AVH postprocess of restart. ReleaseFlag:%d(0:C,1:R,2:B)\n", ReleaseFlag);
                                            led_blink((VnxParam.AvhStatus << 1) + AvhControl);
                                            
                                            Retry++;
                                            for(int i = 0;i < 2;i++){
                                                HAL_Delay(50);
                                                transmit_can_frame(rx_msg_data, AvhControl); // Transmit can frame for remove AVH
                                            }

                                            print_param(&VnxParam, AvhControl, PrevSpeed, PrevBrake, MaxBrake);
                                            // Discard message(s) that received during HAL_delay()
                                            while(is_can_msg_pending(CAN_RX_FIFO0)){
                                                can_rx(&rx_msg_header, rx_msg_data);
                                            }
                                            // rx_msg_header.StdId = CAN_ID_SHIFT;
                                        }
                                        break;
                                    
                                    default: // ENGINE_STOP or WAIT
                                        break;
                                }
                                break;
                            */
                        }
                    }
                        
                    PreviousCanId = rx_msg_header.StdId;
                    break;

                default: // Unexpected can id
                    // Output Warning message
                    // dprintf_("# Warning: Unexpected can id (0x%03x).\n", rx_msg_header.StdId);
                    break;
            }
        }
    }
}

