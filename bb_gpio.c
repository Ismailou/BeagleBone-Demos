/**
******************************************************************************
* @file    bb_gpio.c
* @author  Ismail ZEMNI (ismailzemni@gmail.com)
* @version 1.0
* @date    22/01/2015
* @brief   Demo for blinking LEDs on the Beaglebone bord.
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

/* Private define ------------------------------------------------------------*/
#define GPIO1_START_ADDR 	0x4804C000
#define GPIO1_END_ADDR 		0x4804DFFF
#define GPIO1_SIZE 				(GPIO1_END_ADDR - GPIO1_START_ADDR)

#define GPIO_OE 					0x134
#define GPIO_SETDATAOUT 	0x194
#define GPIO_CLEARDATAOUT 0x190

#define USR0_LED (1<<21)
#define USR1_LED (1<<22)
#define USR2_LED (1<<23)
#define USR3_LED (1<<24)

#define USED_LEDS		(USR3_LED|USR2_LED)

#define REG		volatile unsigned int

/* Private typedef -----------------------------------------------------------*/
typedef struct gpio_reg {
    REG *oe_addr;
    REG *setdataout_addr;
    REG *cleardataout_addr;
} gpio_reg;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * main function
 */  
int main(int argc, char *argv[]) {
    
    // initialize GPIO1 structure
    struct gpio_reg gpio1 = { NULL, NULL, NULL};
    
    volatile void *gpio_addr = NULL;
    unsigned int tmp_reg;
    
    // -- open mem device file
    int fd = open("/dev/mem", O_RDWR);

		// -- map GPIO addressing space
    printf("Mapping 0x%X - 0x%X (size: 0x%X)\n", GPIO1_START_ADDR, GPIO1_END_ADDR, GPIO1_SIZE);
    gpio_addr = mmap(0, GPIO1_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO1_START_ADDR);

		// -- check map returned address
		if(gpio_addr == MAP_FAILED) {
        perror("Unable to map GPIO");
        exit(1);
    }
    
    // -- set gpio1's register address
    gpio1.oe_addr = gpio_addr + GPIO_OE;
    gpio1.setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
    gpio1.cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;

    printf("GPIO mapped to %p\n", gpio_addr);
    printf("GPIO OE mapped to %p\n", gpio1.oe_addr);
    printf("GPIO SETDATAOUTADDR mapped to %p\n", gpio1.setdataout_addr);
    printf("GPIO CLEARDATAOUT mapped to %p\n", gpio1.cleardataout_addr);

		// -- Get the current input/output configuration of GPIO1
    tmp_reg = *gpio1.oe_addr;
    printf("GPIO1 current output/input configuration: %X\n", tmp_reg);
    
    // -- Update th input/output configuration of GPIO1
    tmp_reg &= ~USED_LEDS; // -- bit '0' mean that the pin is configured as output
    *gpio1.oe_addr = tmp_reg;
    printf("GPIO1 updated output/input configuration: %X\n", tmp_reg);

    printf("Start blinking LEDs...\n");
    // -- loop
    while(1) {
    		// -- Turn USED_LEDS on
        printf("ON\n");
        *gpio1.setdataout_addr= USED_LEDS;
        sleep(1);
        
        // -- Turn USED_LEDS off
        printf("OFF\n");
        *gpio1.cleardataout_addr = USED_LEDS;
        sleep(1);
    }

    close(fd);
    return 0;
}
/***************************** EOF ********************************************/
