#include <msp430.h>
#include "include/I2C.h"
#include "include/LED.h"
#include "include/motor.h"
#include "include/main.h"
#include "include/L3GD20H.h"

//#include "include/L3GD20H.h"

#define I2C_RX_BUFFER_SIZE 6

void initClockTo16MHz(void);

uint16_t results[6];
// I2C
uint8_t RX_buffer[I2C_RX_BUFFER_SIZE] = {0};
volatile uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;

uint16_t Y_unsigned = 0;
signed int Y = 0;

int read = 0;

int16_t x;
uint16_t a;







int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    initClockTo16MHz();
    _BIS_SR(GIE);
    
    SensorReset();  
    
    //Nastaveni I2C 
    P3DIR |= BIT0 | BIT3;       // SS a SA0 jako výstupy
    P3OUT |= BIT0;              // SS = HIGH (senzor v I2C režimu)
    P3OUT &= ~BIT3;             // SA0 = LOW (adresa 0x6A)
    __delay_cycles(1600000);
    //Nastaveni snimace     
    I2C_init(L3GD20H_I2C_ADDR);
    I2C_write_byte(CTRL1, (BIT0 | BIT1 | BIT2 | BIT3));
    I2C_write_byte(CTRL2, BIT6);
    __delay_cycles(160000);
   // -TODO udelat .h/.c soubor a funkce pro nastaveni snimace a I2C

    //Inicializace Outputu
    LED_init();
    RGB_LED_init();
    Motor_init(); 
    
    
    
    Motor_setSpeed(60);     
    {
      ReceiveIndex = 0;         //nulovani indexu RX_buffer


      
      I2C_read_byte(OUT_Y_L,2);
      __delay_cycles(16000);
      int16_t x = (int16_t)(((uint16_t)RX_buffer[1] << 8) | RX_buffer[0]); //nefunguje
      
      
    };
}




#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  uint8_t rx_val = 0;

  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: break;                           // Vector  4: NACKIFG
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case USCI_I2C_UCRXIFG:
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
          RX_buffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB0CTL1 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB0IE &= ~UCRXIE;
        }
        break;                              // Interrupt Vector: I2C Mode: UCRXIFG
  case 12: break;                           // Vector 12: TXIFG
  default: break;
  }
}


#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6: break;                           // Vector  6:  ADC12IFG0
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                            // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20:                            // Vector 20:  ADC12IFG7
      RGB_RED_ON();
      ADC12CTL0 &=~ADC12SC;                // For sequence-of-Channels mode, ADC12SC must be cleared by software after each sequence to trigger another sequence
      results[0] = ADC12MEM3;                 // Move results, IFG is cleared
      results[1] = ADC12MEM4;                 // Move results, IFG is cleared
      results[2] = ADC12MEM5;                 // Move results, IFG is cleared
      results[3] = ADC12MEM6;                 // Move results, IFG is cleared
      results[4] = ADC12MEM7;                 // Move results, IFG is cleared
      RGB_RED_OFF();
      ADC12CTL0 |= ADC12SC;                   // Start convn - software trigger
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}

void initClockTo16MHz()
{
    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
    UCSCTL2 = FLLD_0 + 487;                   // Set DCO Multiplier for 16MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (487 + 1) * 32768 = 16MHz
                                              // Set FLL Div = fDCOCLK
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);//
    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                          // Clear fault flags
    }while (SFRIFG1&OFIFG);                         // Test oscillator fault flag
}



/*void SensorReset()

desc: Powers sensor off and on
*/
void SensorReset()
{
  //reset senzoru a nastaveni I2C
    SENSOR_POWER_OFF();
    _delay_cycles(1600000);    
    SENSOR_POWER_DIRPORT |= SENSOR_POWER_PIN;
    SENSOR_POWER_ON();
    _delay_cycles(1600000);
}