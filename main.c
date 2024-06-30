
#include "TM4C123.h"  // Device header
#include <stdint.h>

// System Control Registers
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))


// GPIO Port F Registers
#define GPIO_PORTF_DATA_R       (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_PUR_R        (*((volatile uint32_t *)0x40025510))
#define GPIO_PORTF_LOCK_R       (*((volatile uint32_t *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile uint32_t *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C))
#define GPIO_PORTF_IS_R         (*((volatile uint32_t *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile uint32_t *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile uint32_t *)0x4002540C))
#define GPIO_PORTF_ICR_R        (*((volatile uint32_t *)0x4002541C))
#define GPIO_PORTF_IM_R         (*((volatile uint32_t *)0x40025410))

// NVIC Registers
#define NVIC_EN0_R              (*((volatile uint32_t *)0xE000E100))
#define NVIC_PRI7_R             (*((volatile uint32_t *)0xE000E41C))

void PortF_Init(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
void  EdgeCounter_Init(void);
void GPIOF_Handler(void);
 unsigned long  FallingEdges = 0;

void EdgeCounter_Init(void){
    SYSCTL->RCGCGPIO|= 0x20;  // Activate clock for Port F

    FallingEdges=0;
    //GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock Port F
    //GPIO_PORTF_CR_R |= 0x18;        // Allow changes to PF3 and PF4
		//GPIOF->LOCK = 0;

    GPIO_PORTF_DIR_R &= ~0x10;      // Make PF4 input (built-in button)
    GPIO_PORTF_DIR_R |= 0x08;       // Make PF3 output (LED)
    //GPIO_PORTF_AFSEL_R &= ~0x18;    // Disable alt funct on PF4, PF3
    GPIO_PORTF_DEN_R |= 0x18;       // Enable digital I/O on PF4, PF3
    //GPIO_PORTF_AMSEL_R &= ~0x18;    // Disable analog functionality on PF4, PF3
    GPIO_PORTF_PUR_R |=   0x10;       // Enable weak pull-up on PF4
    //GPIO_PORTF_PCTL_R &= ~0x0000FF000; // Configure PF4 and PF3 as GPIO
    GPIO_PORTF_IS_R &= ~0x10;       // PF4 is edge-sensitive
    GPIO_PORTF_IBE_R &= ~0x10;      // PF4 is not both edges
    GPIO_PORTF_IEV_R &= ~0x10;      // PF4 falling edge event
    GPIO_PORTF_ICR_R = 0x10;        // Clear flag4
    GPIO_PORTF_IM_R |= 0x10;        // Arm interrupt on PF4

    NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // Priority 5
    NVIC_EN0_R = 0x40000000;        // Enable interrupt 30 in NVIC
    EnableInterrupts();             // Enable global Interrupt flag (I)
}


void GPIOF_Handler(void){
    GPIO_PORTF_ICR_R = 0x10;        // Acknowledge flag4
    FallingEdges = FallingEdges + 1;
    GPIO_PORTF_DATA_R ^= 0x08;      // Toggle PF3 (Green LED)
}

int main(void){
    EdgeCounter_Init(); // Initialize GPIO Port F interrupt
	  GPIO_PORTF_DATA_R=0x00;
    while(1){
        WaitForInterrupt();
    }
}

void EnableInterrupts(void){
    __asm ("    CPSIE  I\n");
}

void WaitForInterrupt(void){
    __asm ("    WFI\n");
}

