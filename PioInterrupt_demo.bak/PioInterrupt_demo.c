//C code
/* 
PIO Ep. 19 PIO IRQs
PIO Demo 19A - Demonstrating ID and clearing of IRQ 0 through 4 on one state machine
 */

 #include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 
 #include "hardware/irq.h"
 #include "hardware/clocks.h"
 #include "hardware/pio.h"
 #include "pioInterrupt_demo.pio.h"
 
 static PIO PIO_O;       // pio object
 static uint SM;         // pio state machine index
 static uint PIO_IRQ;    // NVIC ARM CPU interrupt number 
 //int x;
 //char* pointer = (char*)0x50200128; //pointer for PIO0 INTR register
 //char* pointerNVIC = (char*)(PPB_BASE + M0PLUS_NVIC_ICPR_OFFSET); //pointer for NVIC ICPR register
 
 
 void pioIRQ(){          //This is our callback function
 
     printf("[IRQ] %d - ", PIO_O->irq & 0b1111 );
       
     if(pio_interrupt_get(PIO_O,0)) // returns TRUE if IRQ 0 is set
     {	
         printf("IRQ0 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq );
         pio_interrupt_clear(PIO_O, 0);
         printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq );
     }
     else if(pio_interrupt_get(PIO_O,1)) // returns TRUE if IRQ 1 is set
     {
         printf("IRQ1 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq );
         pio_interrupt_clear(PIO_O, 1);
         printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq );
     }
     else if(pio_interrupt_get(PIO_O,2)) // returns TRUE if IRQ 2 is set
     {
         printf("IRQ2 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq );
         pio_interrupt_clear(PIO_O, 2);
         printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq );
     }    
     else if(pio_interrupt_get(PIO_O,3)) // returns TRUE if IRQ 3 is set
     {
         printf("IRQ3 - Before clearing [INTR]: %x [IRQ]: %x; ", PIO_O->intr, PIO_O->irq );
         pio_interrupt_clear(PIO_O, 3);
         printf("After clearing [INTR]: %x [IRQ]: %x \n", PIO_O->intr, PIO_O->irq );
     }
 
 }
 
 void testIRQPIO(uint pioNum) {
     PIO_O = pioNum ? pio1 : pio0; //Selects the pio instance (0 or 1 for pioNUM)
     PIO_IRQ = pioNum ? PIO1_IRQ_0 : PIO0_IRQ_0;  // Selects the NVIC PIO_IRQ to use
         
     // Our assembled program needs to be loaded into this PIO's instruction
     // memory. This SDK function will find a location (offset) in the
     // instruction memory where there is enough space for our program. We need
     // to remember this location!
     uint offset = pio_add_program(PIO_O, &pioIRQ_program); 
     
     // Assign GPIO0 as the blinking LED
     uint BLINK_LED_PIN = 0;
     
     // select the desired state machine clock frequency (2000 is about the lower limit)
     float SM_CLK_FREQ = 2000;
 
     // Find a free state machine on our chosen PIO (erroring if there are
     // none). Configure it to run our program, and start it, using the
     // helper function we included in our .pio file.
     SM = pio_claim_unused_sm(PIO_O, true);
     pioIRQ_program_init(PIO_O, SM, offset, BLINK_LED_PIN, SM_CLK_FREQ);	
     
 
 //enables IRQ for the statemachine - setting IRQ0_INTE - interrupt enable register
     //pio_set_irq0_source_enabled(PIO_O, pis_interrupt0, true); // sets IRQ0
     //pio_set_irq0_source_enabled(PIO_O, pis_interrupt1, true); // sets IRQ1
     //pio_set_irq0_source_enabled(PIO_O, pis_interrupt2, true); // sets IRQ2
     //pio_set_irq0_source_enabled(PIO_O, pis_interrupt3, true); // sets IRQ3
   //*********or************	
     pio_set_irq0_source_mask_enabled(PIO_O, 3840, true); //setting all 4 at once
     
     irq_set_exclusive_handler(PIO_IRQ, pioIRQ);  //Set the handler in the NVIC
     irq_set_enabled(PIO_IRQ, true);                    //enabling the PIO1_IRQ_0
 
 }
 
 
 int main(void)
 {
   stdio_init_all();
   sleep_ms(20000);  //gives me time to start PuTTY
   printf("start program\n");
   testIRQPIO(0);
   printf("PIO: %d, SM: %d, PIO_IRQ_num: %d \n", pio_get_index(PIO_O), SM, PIO_IRQ);
 
   while (1)
   {
    sleep_ms(200);  
    printf("."); //watchdog printout
   }
 
   return 0;
 }