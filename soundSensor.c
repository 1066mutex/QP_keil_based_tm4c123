/**
 * @file soundSensor.c
 * @author Eddie Mhako (eddie.mhako@sky.com)
 * @brief  handles all the DSP math based on the CMSIS DSP library. This library is
 *   is specifically for the ARM proccerss only. 
 * @version 0.1
 * @date 2021-09-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Computer real FFT using the completed data buffer */

/* definetions 
* SAMPLE_LENGTH  this is how many samples for each window. 512 starting point.
* 
*/

#include "qpc.h"
#include "bsp.h"
#include "bsp_bp.h"
 #include "arm_math.h"

#include "logger.h"
#include "defs.h" // todo test! remove this line


/* Active object class  ############################################*/

/*..........................................................................*/
typedef struct { /* the soundSensor active object */
    /* protected: */
    QActive super;
    /* private: */
    QTimeEvt adcTimeEvt;                           // private time event generator                           
    int16_t dataInput[SAMPLE_LENGTH * 2];          // private real FFT processed data buffer
    int16_t dataOutput[SAMPLE_LENGTH];             // private magnitude of FFT complex output buffer
    int16_t micFFTbuff[SAMPLE_LENGTH * 2];         // private magnitude of FFT complex output buffer
    int16_t micFFTbuff_mag[SAMPLE_LENGTH];         // private magnitude of FFT complex output buffer
    int16_t activeBuffer[SAMPLE_LENGTH];           // private magnitude of FFT complex output buffer
    float hann[SAMPLE_LENGTH];                     // private hanning window
    //arm_rfft_instance_q15 realFFT_Inst;            // private ARM q15 instance for real FFT calculations
    //arm_cfft_radix4_instance_q15 complexFFT_Inst;  // private ARM q15 radix instance for real FFT magnitude calculations

} soundSensor;

void soundSensor_ctor(void);

/* hierarchical state machine ... */
static QState soundSensor_initial(soundSensor *const me, QEvt const *const e);
static QState soundSensor_active(soundSensor *const me, QEvt const *const e);
static QState soundSensor_processing(soundSensor *const me, QEvt const *const e);
static QState soundSensor_dsp(soundSensor *const me, QEvt const *const e);
static QState soundSensor_idle(soundSensor *const me, QEvt const *const e);

/* Local objects -----------------------------------------------------------*/
static soundSensor l_soundSensor; // the single instance of the soundSensor active object

/* Global-scope objects ----------------------------------------------------*/
QActive *const AO_soundSensor = &l_soundSensor.super; // "opaque" AO pointer 

/*..........................................................................*/
void soundSensor_ctor(void) {
    soundSensor *me = &l_soundSensor;
    QActive_ctor(&me->super, (QStateHandler)&soundSensor_initial);
    QTimeEvt_ctorX(&me->adcTimeEvt, &me->super, TRIGGER_TIMEOUT_SIG, 0U);
}

/* HSM definition ----------------------------------------------------------*/
QState soundSensor_initial(soundSensor *const me, QEvt const *const e) {
    (void)e;                           // unused parameter 
     //! start time pwm to trigger the adc sampling.
     // initialise hanning window.
    int n;
    for (n = 0; n < SAMPLE_LENGTH; n++) {
        me->hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
    }
    //Init the FFT Structures
    // arm_rfft_init_q15(&me->realFFT_Inst,
    //                   SAMPLE_LENGTH,  // data points
    //                   0,
    //                   1);  //Bit Reverse Flag enabled

   // set timeout for conversion triggering
    QTimeEvt_armX(&me->adcTimeEvt, BSP_TICKS_PER_SEC / 2U, BSP_TICKS_PER_SEC / 100U);
    return Q_TRAN(&soundSensor_dsp);
}
QState soundSensor_active(soundSensor *const me, QEvt const *const e) {
    QState status;
    switch (e->sig) {
     
        case TRIGGER_TIMEOUT_SIG: {
            // trigger adc conversion.
           // BSP_Microphone_Get();  //! (2)trigger microphone ADC convertion
            status = Q_HANDLED();
            break;
        }
        case STOP_SOUND_DATA_SIG: {
            // stop sound processing and go to idle

            status = Q_TRAN(&soundSensor_idle);
            break;
        }
        default: {
            status = Q_SUPER(&QHsm_top); // pass all events to the top state which will silently ignore them
            break;
        }
    }
    return status;
}
QState soundSensor_idle(soundSensor *const me, QEvt const *const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            // stop trigger timeout 
            //! check if the timer is stoped here? or should it be stopped in the fft state exit action
            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            // start trigger timeout
            QTimeEvt_armX(&me->adcTimeEvt, BSP_TICKS_PER_SEC / 2U, BSP_TICKS_PER_SEC / 1000U);
            status = Q_HANDLED();
            break;
        }
        case GET_SOUND_DATA_SIG: {
            status = Q_TRAN(&soundSensor_dsp);
            break;
        }
        default: {
            status = Q_SUPER(&soundSensor_active);
            break;
        }
    }
    return status;
}
QState soundSensor_dsp(soundSensor *const me, QEvt const *const e) {
    QState status;
    switch (e->sig) {
        case Q_ENTRY_SIG: {

            status = Q_HANDLED();
            break;
        }
        case Q_EXIT_SIG: {
            //! can stop trigger timeout here befor transition to idle.
            // stop timeout trigger
            // dis-arm timer
            QTimeEvt_disarm(&me->adcTimeEvt);
            status = Q_HANDLED();
            break;
        }
        case NEW_SOUND_DATA_SIG: {
            // process sound sample buffer.
        
            
            //TODO: have an if statemnt to select out the windowing process.
            /* apply a Hanning window to the active buffer before processing */
            for (int i = 0; i < SAMPLE_LENGTH; i++) {  // upcast to q31_t *hann.  >> 15 is fixed point multiply
                me->activeBuffer[i] = (q31_t)((Q_EVT_CAST(RawSoundEvt)->rawSoundBuf[i]) * me->hann[i]) >> 15;
            }
            //Compute the FFT
            // arm_rfft_q15(&me->realFFT_Inst,
            //              (q15_t *)me->activeBuffer,  //! don't use const buffer, pass buff that is not beingused
            //              (q15_t *)me->micFFTbuff);   // output buffer for complex numbers

            //Scale the input before computing magnitude
            for (int i = 0; i < (SAMPLE_LENGTH *2) ; i++) {
                me->micFFTbuff[i] <<= 6;
            }
            // FFT function returns the real / imaginary values. We need to compute the magnitude
            // the resulting magnitude is the sample length size but only the first half is used 
            // as the second half is just conjigates (mirror image of the first)
            // arm_cmplx_mag_q15((q15_t *)me->micFFTbuff,
            //                   (q15_t *)me->micFFTbuff_mag,
            //                   SAMPLE_LENGTH);
            // pack display data without conjugate part only the first half.
            DspSoundEvt *mag = Q_NEW(DspSoundEvt, DISPLAY_SOUND_MAG_SIG);
            mag->buffSize = (SAMPLE_LENGTH / 2);
            for (int i = 0; i < mag->buffSize; i++) {
                mag->soundMagBuff[i] = me->micFFTbuff_mag[i];
            }
            QACTIVE_POST(AO_UI, (QEvt *)mag, me);
            status = Q_HANDLED();
            break;
        }
        default: {
            status = Q_SUPER(&soundSensor_active); // pass all unhandled events to the superstate.
            break;
        }
    }
    return status;
}


// #define SAMPLE_LENGTH 512

// float hann[SAMPLE_LENGTH];

// // buffers for the DMA data 
// //
// int16_t data_array1[SAMPLE_LENGTH];
// int16_t data_array2[SAMPLE_LENGTH];

// // buffer for RFFT processed data
// //
// int16_t data_input[SAMPLE_LENGTH * 2];

// // magnitude of FFT complex output buffer
// //
// int16_t data_output[SAMPLE_LENGTH];

// /**
//  * TODO: 
//  * 
//  * initialise uDMA in ping pong mode. 512 samples?
//  * uDMA ISR should send complete or ready buffer to AO for processing
//  * Initialise display for plotting data
//  * initialise ADC for DMA and timer PWM trigger
//  * initialise timer or pwm module for ADC trigger, start 500ms period and 10% duty cycle
//  * Initialize Hann Window //! research more. 
//  *  
// */

// // output of arm_rfft_q15()

// //! instance of the arm real FFT
// arm_rfft_instance_q15        RealFFT_Instance;
// arm_cfft_radix4_instance_q15 MyComplexFFT_Instance;

// //Init the FFT Structures
// arm_rfft_init_q15(&RealFFT_Instance,
//                   &MyComplexFFT_Instance,
//                   128, // data points
//                   0,
//                   1);  //Bit Reverse Flag enabled

// //Apply a Hanning window if the jumper is set.
// //If not, we just keep our rectangular window
// // apply a Hanning window to the active buffer before processing
// for (i = 0; i < 128; i++) {  // upcast to q31_t *hann.  >> 15 is fixed point multiply
//     Activebuffer[i] = ((q31_t)Activebuffer[i] * Hanning[i]) >> 15;
// }

// //Compute the FFT
// arm_rfft_q15(&RealFFT_Instance,
//              (q15_t *)Activebuffer,  //! don't use cont buffer, pass buff that is not beingused
//              (q15_t *)MicFFT);       // output buffer

// //Scale the input before computing magnitude
// for (i = 0; i < 256; i++) {
//     MicFFT[i] <<= 6;
// }

// //FFT function returns the real / imaginary values.   We need to compute the magnitude
// arm_cmplx_mag_q15((q15_t *)MicFFT,
//                   (q15_t *)MicFFT_Mag,
//                   128);


// //------------------------------------------------------------------------------
// // 


// // processing the data from ISR,
// if (switch_data & 1) { 
//     // apply hann window to data buffer.
//     for (i = 0; i < 512; i++) {
//         data_array1[i] = (int16_t)(hann[i] * data_array1[i]);
//     }
//     // initialise ARM real FFT function 
//     arm_rfft_instance_q15 instance;
//     status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
//                                doBitReverse);

//     // process the data using ARM DSP function, fills the data input array with results 
//     arm_rfft_q15(&instance, data_array1, data_input);
// } else {
//     for (i = 0; i < 512; i++) {
//         data_array2[i] = (int16_t)(hann[i] * data_array2[i]);
//     }
//     arm_rfft_instance_q15 instance;
//     status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
//                                doBitReverse);

//     arm_rfft_q15(&instance, data_array2, data_input);
// }

// /* Calculate magnitude of FFT complex output */

// for (i = 0; i < 1024; i += 2) {
//     data_output[i /
//                 2] =
//         (int32_t)(sqrtf((data_input[i] *
//                          data_input[i]) +
//                         (data_input[i + 1] * data_input[i + 1])));
// }

// q15_t maxValue;
// uint32_t maxIndex = 0;
// // Computes the maximum value of an array of data.
// arm_max_q15(data_output, fftSize, &maxValue, &maxIndex);

// if (maxIndex <= 64) {
//     color = ((uint32_t)(0xFF * (maxIndex / 64.0f)) << 8) + 0xFF;
// } else if (maxIndex <= 128) {
//     color =
//         (0xFF - (uint32_t)(0xFF * ((maxIndex - 64) / 64.0f))) + 0xFF00;
// } else if (maxIndex <= 192) {
//     color =
//         ((uint32_t)(0xFF * ((maxIndex - 128) / 64.0f)) << 16) + 0xFF00;
// } else {
//     color =
//         ((0xFF -
//           (uint32_t)(0xFF *
//                      ((maxIndex - 192) / 64.0f)))
//          << 8) +
//         0xFF0000;
// }

// /* Draw frequency bin graph */
// for (i = 0; i < 256; i += 2) {
//     int x = min(100, (int)((data_output[i] + data_output[i + 1]) / 8));

//     Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
//     Graphics_drawLineV(&g_sContext, i / 2, 114 - x, 14);
//     Graphics_setForegroundColor(&g_sContext, color);
//     Graphics_drawLineV(&g_sContext, i / 2, 114, 114 - x);
// }
// //! ping pong buffer in software

// // == 1 when audio data is being captured
// volatile uint8_t CaptureActive = 0;
// // the sample that is being captured
// volatile uint8_t CurrentSample = 0;
// // buffers
// volatile int16_t MicBuffer1[128];
// volatile int16_t MicBuffer2[128];

// // pointer to active and recording
// volatile int16_t *Backbuffer;
// volatile int16_t *Activebuffer;

// // controls which buffer is currently active and back
// volatile uint8_t BufferPtr = 0;

// //Initialize The Ping-pong Buffer.   The IRQ routine will fill up the back buffer while the FFT is done on the foreground buffer
// BufferPtr = 0; //! use enum for 1 and 0. 
// // when the bufferPtr is 0, the active buffer is MicBuffer1
// // and the back buffer is  MicBuffer2
// // can use a 2 dimension array micBuffer[][];
// Activebuffer = MicBuffer1;  // initialise pointer to pointer to MicBuffer1 
// Backbuffer = MicBuffer2;

// //! usage of buffer.

// // in the foreground routine, when calculating the FFT the Active buffer ptr is used
// // to access the input data.
// //Compute the FFT
// arm_rfft_q15(&RealFFT_Instance,
//              (q15_t *)Activebuffer, // pointer to current processing buffer.
//              (q15_t *)MicFFT);


// // the back buffer is filled within the ISR 
// //ADC Recording Process
// void SysTick_Handler() {
//     if (CaptureActive) {
//         //Store the Started Sample... We are assuming that the ADC willbe done by now
//         Backbuffer[CurrentSample] = (uint16_t)ADC0_RA - (uint16_t)(2048);
//         CurrentSample++;

//         if (CurrentSample < 128) {
//             //Start The Next Sample..  We will grab it on the next SysTickIrq
//             StartADC0_SingleEnded(ADC0_SE14);
//         } else {
//             CurrentSample = 0;
//             CaptureActive = 0;
//         }
//     }
// }


// //! note the buffer ptr swap is done after processing is finised, at the end.
// //Wait for the Background capture to complete
// while (CaptureActive == 1) {
// }

// //Swap the Active/Background Buffer ---> Ping Pong!
// if (BufferPtr == 0) {
//     BufferPtr = 1;
//     Activebuffer = MicBuffer2;
//     Backbuffer = MicBuffer1;
// } else {
//     BufferPtr = 0;
//     Activebuffer = MicBuffer1;
//     Backbuffer = MicBuffer2;
// }

// //Start the background capture on the new buffer
// CaptureActive = 1;
