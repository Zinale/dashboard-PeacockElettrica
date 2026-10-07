/*
 * task_inputs.c
 *  Task_ReadInputs - 20 ms
 *  Reads physical buttons and shift register, then processes actions.
 */

#include "Tasks.h"
#include "Communication/serial.h"
#include "shift_register.h"

/* Local macro to set error in car_state */
#define SET_ERROR(e)  do { car_state.error_flag = true; car_state.error = (e); } while(0)
#define reset_hold_time_ms 1000

/* Se la richiesta R2D e' alzata ma la MCU non arma entro questo tempo
 * (es. freno non premuto all'atto della pressione), la dash riporta la
 * richiesta a 0 da sola: cosi' basta una nuova pressione - col freno -
 * per riprovare, invece di doverne fare due per rigenerare il fronte.
 * Deve superare la latenza di andata/ritorno (0x104 a 10 Hz -> ~150 ms). */
#define R2D_REQUEST_TIMEOUT_MS 600
/* =========================================================================
 * Apply_SR_Position
 * Maps physical position (1-8) to car_state variables.
 *   1-3  -> engine map (ECO / NORM / GAS)
 *   4-7  -> dynamic parameter selection (REGEN / SRC / OS / TVC)
 *   8    -> dedicated page page5 (no button action)
 *   0    -> neutral, no action
 *  -1    -> hardware error (short circuit between positions)
 * ========================================================================= */
static void Apply_SR_Position(int8_t pos)
{
    if (pos == -1) {
        if (!car_state.error_flag)
            SET_ERROR(ERR_SR_HARDWARE);
        return;
    }

    if (pos >= 1 && pos <= 3) {
        car_state.SR_map = pos;
        car_state.selected_setting = 0;

        /* Exit page4/page5 to main page with new map. */
        if (car_state.current_page == 4 || car_state.current_page == 5) {
            car_state.current_page = 1;
            Nextion_Cmd("page page1");
        }
    } else if (pos >= 4 && pos <= 7) {
        car_state.selected_setting = pos - 3;
        if (car_state.current_page != 4) {
            car_state.current_page = 4;
            Nextion_Cmd("page page4");
        }
    } else if (pos == 8) {
        /* Keep current map and open dedicated page. */
        car_state.selected_setting = 0;
        if (car_state.current_page != 5) {
            car_state.current_page = 5;
            Nextion_Cmd("page page5");
        }
    }
    /* pos == 0: no action */
}

/* =========================================================================
 * Read_Physical_Buttons
 * Reads GPIO with cooldown debouncing (BTN_COOLDOWN_MS).
 * Sets .just_pressed = true only on rising edge.
 * ========================================================================= */
static void Read_Physical_Buttons(void)
{
    uint32_t now = HAL_GetTick();

    bool raw_r2d  = (HAL_GPIO_ReadPin(R2D_btn_GPIO_Port,  R2D_btn_Pin)  == GPIO_PIN_RESET);
    bool raw_page = (HAL_GPIO_ReadPin(PAGE_btn_GPIO_Port, PAGE_btn_Pin) == GPIO_PIN_RESET);

    /* Pulsante R2D */
    if (raw_r2d && !car_state.btn_1.pressed 
        &&   (now - car_state.btn_1.last_pressed_time > BTN_COOLDOWN_MS)) {
        car_state.btn_1.pressed           = true;
        car_state.btn_1.last_pressed_time = now;
        car_state.btn_1.just_pressed      = true;
    } else if (!raw_r2d) {
        car_state.btn_1.pressed = false;
    }

    /* Pulsante PAGE */
    if (raw_page && !car_state.btn_2.pressed &&
        (now - car_state.btn_2.last_pressed_time > BTN_COOLDOWN_MS)) {
        car_state.btn_2.pressed           = true;
        car_state.btn_2.last_pressed_time = now;
        car_state.btn_2.just_pressed      = true;
    } else if (!raw_page) {
        car_state.btn_2.pressed = false;
    }
}

/* =========================================================================
 * Read_SR
 * Updates internal shift register debouncing and applies position if changed.
 * ========================================================================= */
static void Read_SR(void)
{
    SR_ProcessPeriodic();

#ifdef SR_DEBUG_PRINT
    /* Print raw value every ~500 ms to avoid serial saturation */
    static uint32_t sr_last_print_time = 0;
    uint32_t now_dbg = HAL_GetTick();
    if ((now_dbg - sr_last_print_time) >= 100U) {
        sr_last_print_time = now_dbg;
        uint8_t  raw = SR_GetRawValue();
        int8_t   pos = SR_GetStablePosition();
        /* Convert byte to binary string (MSB on the left) */
        char bin_str[9];
        for (int _b = 7; _b >= 0; _b--) {
            bin_str[7 - _b] = ((raw >> _b) & 1) ? '1' : '0';
        }
        bin_str[8] = '\0';
        Display_Message(&huart2, "=====================================\r\n");
        Display_Message(&huart2,
            "[SR] RAW=0x%02X (b%s) | POS=%d\r\n",
            raw, bin_str, (int)pos);
        Display_Message(&huart2, "=====================================\r\n");
    }
#endif /* SR_DEBUG_PRINT */

    if (SR_HasChanged()) {
        car_state.SR_raw_position = SR_GetStablePosition();
        Apply_SR_Position(SR_GetStablePosition());
    }
}

/* =========================================================================
 * Process_Input_Actions
 * - Hold both buttons > 1 s -> system_reset()
 * - R2D button -> toggle r2d (with safety conditions)
 * - PAGE button -> change page OR cycle selected parameter (1->2->3->4->1)
 * ========================================================================= */
static void Process_Input_Actions(void)
{
    static uint32_t dual_press_start = 0;
    static bool     dual_press_active = false;

    /* In page5 only reset combo is allowed; single buttons are ignored. */
    if (car_state.current_page == 5) {
        if (car_state.btn_1.pressed && car_state.btn_2.pressed) {
            if (!dual_press_active) {
                dual_press_active = true;
                dual_press_start  = HAL_GetTick();
            } else if ((HAL_GetTick() - dual_press_start) > reset_hold_time_ms) {
                system_reset();
            }
        } else {
            dual_press_active = false;
        }

        car_state.btn_1.just_pressed = false;
        car_state.btn_2.just_pressed = false;
        return;
    }

    /* --- COMBO: both buttons pressed -> reset after hold time --- */
    if (car_state.btn_1.pressed && car_state.btn_2.pressed) {
        if (!dual_press_active) {
            dual_press_active = true;
            dual_press_start  = HAL_GetTick();
        } else if ((HAL_GetTick() - dual_press_start) > reset_hold_time_ms) {
            system_reset();
        }
        /* Block just_pressed to prevent single actions on release */
        car_state.btn_1.just_pressed = false;
        car_state.btn_2.just_pressed = false;
        return;
    } else {
        dual_press_active = false;
    }

    /* --- btn_1: MINUS on page4 with selected parameter, otherwise R2D --- */
    if (car_state.btn_1.just_pressed) {
        if (car_state.current_page == 4 && car_state.selected_setting != 0) {
            /* Settings page: btn_1 = MINUS (minimum value 1) */
            switch (car_state.selected_setting) {
                case 1: if (car_state.val_regen > 1) car_state.val_regen--; break;
                case 2: if (car_state.val_src   > 1) car_state.val_src--;   break;
                case 3: if (car_state.val_tvc    > 1) car_state.val_tvc--;    break;
                case 4: if (car_state.val_event  > 1) car_state.val_event--;  break;
                default: break;
            }
        } else {
            /* Altre pagine: btn_1 = toggle della RICHIESTA R2D.
             * Il volante manda solo la richiesta: le condizioni di sicurezza
             * (freno, SDC, fault, velocita') e lo stato reale li gestisce la MCU. */
            car_state.r2d_request = !car_state.r2d_request;
        }
        car_state.btn_1.just_pressed = false;
    }

    /* Allinea la richiesta allo stato reale della MCU:
     * (a) quando la MCU si disarma (fault o stop), azzera la richiesta ->
     *     basta UNA pressione per riarmare (fronte di discesa di r2d_armed);
     * (b) se la richiesta resta alzata ma la MCU non arma ne' e' attiva
     *     entro R2D_REQUEST_TIMEOUT_MS (tipicamente: freno non premuto
     *     all'atto della pressione), azzera la richiesta cosi' la pressione
     *     successiva - col freno - rigenera un fronte valido. */
    {
        static bool     s_mcu_armed_prev = false;
        static bool     s_req_prev       = false;
        static uint32_t s_req_since      = 0;

        bool mcu_armed  = car_state.mcu.r2d_armed;
        bool mcu_active = car_state.mcu.r2d_active;

        if (car_state.r2d_request && !s_req_prev) {
            s_req_since = HAL_GetTick();          /* richiesta appena alzata */
        }

        if (s_mcu_armed_prev && !mcu_armed) {
            car_state.r2d_request = false;                       /* (a) */
        } else if (car_state.r2d_request && !mcu_armed && !mcu_active &&
                   (HAL_GetTick() - s_req_since) > R2D_REQUEST_TIMEOUT_MS) {
            car_state.r2d_request = false;                       /* (b) */
        }

        s_mcu_armed_prev = mcu_armed;
        s_req_prev       = car_state.r2d_request;
    }

    /* --- PAGE: change page or cycle selected parameter --- */
    if (car_state.btn_2.just_pressed) {
        if (car_state.selected_setting == 0) {
            car_state.current_page++;
            if (car_state.current_page > 4) car_state.current_page = 1;
            Nextion_Cmd("page page%d", car_state.current_page);
        } else {
            switch (car_state.selected_setting) {
                case 1: car_state.val_regen = (car_state.val_regen % 4) + 1; break;
                case 2: car_state.val_src   = (car_state.val_src   % 4) + 1; break;
                case 3: car_state.val_tvc   = (car_state.val_tvc   % 4) + 1; break;
                case 4: car_state.val_event = (car_state.val_event % 4) + 1; break;
                default: break;
            }
        }
        car_state.btn_2.just_pressed = false;
    }
}

/* =========================================================================
 * Task_ReadInputs  [20 ms]
 * ========================================================================= */
void Task_ReadInputs(void)
{
    Read_Physical_Buttons();
#ifdef USE_SR
    Read_SR();
#endif
    Process_Input_Actions();
}
