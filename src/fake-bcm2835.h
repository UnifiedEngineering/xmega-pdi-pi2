#include <stdint.h>
#include <stdbool.h>

#define BCM2835_GPIO_FSEL_OUTP 1
#define BCM2835_GPIO_FSEL_INPT 0

#define BCM2835_GPIO_PUD_OFF 0

void bcm2835_delayMicroseconds (uint32_t usecs);
void bcm2835_gpio_clr (uint32_t gpio);
void bcm2835_gpio_set (uint32_t gpio);
bool bcm2835_gpio_lev (uint32_t gpio);
int bcm2835_init (void);
void bcm2835_gpio_fsel (uint32_t gpio, uint32_t mux);
void bcm2835_gpio_set_pud (uint8_t pin, uint8_t pud);

