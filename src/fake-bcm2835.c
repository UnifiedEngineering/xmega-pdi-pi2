/* Fake the bcm2835 GPIO library interface just enough for xmega-pdi-pi2 to work on a Tegra X1-based system such as the Jetson Nano
   Init code heavily inspired by https://github.com/jwatte/jetson-gpio-example
*/

#include "fake-bcm2835.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <time.h>

#define GPIO_1     0x6000d000
#define GPIO_2     0x6000d100
#define PINMUX_AUX 0x70003000

//  Each GPIO controller has four ports, each port controls 8 pins, each
//  register is interleaved for the four ports, so
//  REGX: port0, port1, port2, port3
//  REGY: port0, port1, port2, port3
typedef struct {
    uint32_t CNF[4];
    uint32_t OE[4];
    uint32_t OUT[4];
    uint32_t IN[4];
    uint32_t INT_STA[4];
    uint32_t INT_ENB[4];
    uint32_t INT_LVL[4];
    uint32_t INT_CLR[4];
    uint32_t dummy[32];
} GPIO_mem;

typedef struct {
    GPIO_mem volatile ctrl[8];
} GPIOS;
GPIOS volatile *pin; // 8 controllers in total

#define MAX_GPIO 247
uint8_t gpio2pinmux[MAX_GPIO + 1] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-15
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 16-31	
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 32-47
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 48-63
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 64-79
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80-95
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 96-111
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 112-127
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 128-143
                          0, 0, 0, 0, 0, 0, 0, 0x1ec/4, 0x1f0/4, 0, 0, 0, 0, 0, 0, 0, // 144-159 (only gpio151 and 152 handled at the moment)
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 160-175
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 176-191
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 192-207
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 208-223
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 224-239
                          0, 0, 0, 0, 0, 0, 0, 0 // 240-247
}; 

typedef struct {
    uint32_t PINMUX[165];
} PINMUX_mem;
PINMUX_mem volatile *pmux;

void bcm2835_delayMicroseconds (uint32_t usecs) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	time_t secs = ts.tv_sec;
	long ns = ts.tv_nsec + (1000*usecs);
	if (ns >= 1000000000) {
		ns-=1000000000;
		secs++;
	}
	do {
		clock_gettime(CLOCK_MONOTONIC, &ts);
	} while (ts.tv_sec != secs || ts.tv_nsec < ns);
	return;
}

void bcm2835_gpio_clr (uint32_t gpio) {
    if (gpio <= MAX_GPIO) {
        uint8_t controllernum = gpio >> 5;
        uint8_t subctrlnum = (gpio >> 3) & 0x07;
        uint8_t bitval = 1 << (gpio & 0x07);
        pin->ctrl[controllernum].OUT[subctrlnum] &= ~bitval;
    }
}

void bcm2835_gpio_set (uint32_t gpio) {
    if (gpio <= MAX_GPIO) {
        uint8_t controllernum = gpio >> 5;
        uint8_t subctrlnum = (gpio >> 3) & 0x07;
        uint8_t bitval = 1 << (gpio & 0x07);
        pin->ctrl[controllernum].OUT[subctrlnum] |= bitval;
    }
}

bool bcm2835_gpio_lev (uint32_t gpio) {
    if (gpio <= MAX_GPIO) {
        uint8_t controllernum = gpio >> 5;
        uint8_t subctrlnum = (gpio >> 3) & 0x07;
        uint8_t bitval = 1 << (gpio & 0x07);
        return !!(pin->ctrl[controllernum].IN[subctrlnum] & bitval);
    }
    return false;
}

int bcm2835_init (void) {
    //  read physical memory (needs root)
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem");
        fprintf(stderr, "please run this program as root (for example with sudo)\n");
        return 0;
    }

    //  map a particular physical address into our address space
    int pagesize = getpagesize();
    int pagemask = pagesize-1;
    //  This page will actually contain all the GPIO controllers, because they are co-located
    void *base = mmap(0, pagesize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (GPIO_1 & ~pagemask));
    if (base == NULL) {
        perror("gpio mmap()");
        return 0;
    }
    uint32_t *base2 = mmap(0, pagesize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (PINMUX_AUX & ~pagemask));
    if (base2 == NULL) {
        perror("pinmux mmap()");
        return 0;
    }
    pmux = (PINMUX_mem volatile *)((char *)base2 + (PINMUX_AUX & pagemask));
    pin = (GPIOS volatile *)((char *)base + (GPIO_1 & pagemask));

    return 1;
}

void bcm2835_gpio_fsel (uint32_t gpio, uint32_t mux) {
    if (gpio <= MAX_GPIO) {
        uint8_t controllernum = gpio >> 5;
        uint8_t subctrlnum = (gpio >> 3) & 0x07;
	uint8_t bitval = 1 << (gpio & 0x07);
	if (mux) pin->ctrl[controllernum].OE[subctrlnum] |= bitval; else pin->ctrl[controllernum].OE[subctrlnum] &= ~bitval;
	pin->ctrl[controllernum].CNF[subctrlnum] |= bitval; // Force GPIO mode
	pin->ctrl[controllernum].INT_ENB[subctrlnum] &= ~bitval; // Make sure interrupt is disabled
    }
}

void bcm2835_gpio_set_pud (uint8_t gpio, uint8_t pud) {
    if (gpio <= MAX_GPIO && pud == BCM2835_GPIO_PUD_OFF) {
        // Enable input and disable pullup/down (no other operation supported at this time)
        pmux->PINMUX[gpio2pinmux[gpio]] = 0b1000000;
    }
}

