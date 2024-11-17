#include "stm32f4xx.h"

#define DHT11_PIN (1U << 1)
#define OLED_I2C_ADDRESS 0x3C
#define TEMP_MIN 0
#define TEMP_MAX 50
#define HUM_MIN 20
#define HUM_MAX 80
#define AIR_QUALITY_SENSOR_PIN (1U << 0)
#define AIR_QUALITY_THRESHOLD 1000

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void DHT11_Init(void);
uint16_t DHT11_Read(void);
void I2C_Init(void);
void OLED_Write_Command(uint8_t cmd);
void OLED_Init(void);
void OLED_Display_Text(const char *text);
void OLED_Display_Danger(void);
void OLED_Display_OK(void);
void ADC_Init(void);
void ADC_Start(void);
void ADC_IRQHandler(void);

void ADC_Init(void) {
    RCC->APB2ENR |= (1U << 8);
    RCC->AHB1ENR |= (1U << 0);
    GPIOA->MODER |= (3U << (2 * 0));
    ADC1->CR1 |= (1U << 23) | (1U << 5);
    ADC1->CR1 |= (1U << 9);
    ADC1->CR1 &= ~(0x1F);
    ADC1->LTR = AIR_QUALITY_THRESHOLD;
    ADC1->HTR = 0xFFF;
    ADC1->CR2 |= (1U << 0);
    ADC1->SQR3 |= (0 << 0);
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void) {
    if (ADC1->SR & (1U << 0)) {
        ADC1->SR &= ~(1U << 0);
        OLED_Display_Text("DANGER");
    }
}

void delay_us(uint32_t us) {
    us *= 16;
    while (us--) __NOP();
}

void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

void DHT11_Init() {
    RCC->AHB1ENR |= (1U << 0);
    GPIOA->MODER |= (1U << (2 * 1));
    GPIOA->OTYPER |= (1U << 1);
    GPIOA->PUPDR |= (1U << (2 * 1));
}

uint16_t DHT11_Read() {
    uint8_t data[5] = {0};
    uint8_t humidity = 0, temperature = 0;

    GPIOA->MODER |= (1U << (2 * 1));
    GPIOA->BSRR = DHT11_PIN << 16;
    delay_ms(18);
    GPIOA->BSRR = DHT11_PIN;
    delay_us(30);
    GPIOA->MODER &= ~(3U << (2 * 1));

    for (int i = 0; i < 40; i++) {
        while (!(GPIOA->IDR & DHT11_PIN));
        delay_us(40);
        if (GPIOA->IDR & DHT11_PIN) data[i / 8] |= (1 << (7 - (i % 8)));
        while (GPIOA->IDR & DHT11_PIN);
    }

    humidity = data[0];
    temperature = data[2];

    return (humidity << 8) | temperature;
}

void I2C_Init() {
    RCC->AHB1ENR |= (1U << 1);
    RCC->APB1ENR |= (1U << 21);
    GPIOB->MODER |= (2U << (2 * 6)) | (2U << (2 * 7));
    GPIOB->OTYPER |= (1U << 6) | (1U << 7);
    GPIOB->AFR[0] |= (4U << (4 * 6)) | (4U << (4 * 7));
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= (1U << 0);
}

void OLED_Write_Command(uint8_t cmd) {
    while (I2C1->SR2 & (1U << 1));
    I2C1->CR1 |= (1U << 8);
    while (!(I2C1->SR1 & (1U << 0)));
    I2C1->DR = OLED_I2C_ADDRESS << 1;
    while (!(I2C1->SR1 & (1U << 1)));
    (void)I2C1->SR2;
    I2C1->DR = 0x00;
    while (!(I2C1->SR1 & (1U << 7)));
    I2C1->DR = cmd;
    while (!(I2C1->SR1 & (1U << 2)));
    I2C1->CR1 |= (1U << 9);
}

void OLED_Init() {
    OLED_Write_Command(0xAE);
    OLED_Write_Command(0x20);
    OLED_Write_Command(0x10);
    OLED_Write_Command(0xB0);
    OLED_Write_Command(0xC8);
    OLED_Write_Command(0x00);
    OLED_Write_Command(0x10);
    OLED_Write_Command(0x40);
    OLED_Write_Command(0x81);
    OLED_Write_Command(0xFF);
    OLED_Write_Command(0xA1);
    OLED_Write_Command(0xA6);
    OLED_Write_Command(0xA8);
    OLED_Write_Command(0x3F);
    OLED_Write_Command(0xA4);
    OLED_Write_Command(0xD3);
    OLED_Write_Command(0x00);
    OLED_Write_Command(0xD5);
    OLED_Write_Command(0xF0);
    OLED_Write_Command(0xD9);
    OLED_Write_Command(0x22);
    OLED_Write_Command(0xDA);
    OLED_Write_Command(0x12);
    OLED_Write_Command(0xDB);
    OLED_Write_Command(0x20);
    OLED_Write_Command(0x8D);
    OLED_Write_Command(0x14);
    OLED_Write_Command(0xAF);
}

void OLED_Display_Text(const char *text) {
    while (*text) OLED_Write_Command(*text++);
}

void OLED_Display_Danger() {
    OLED_Display_Text("DANGER");
}

void OLED_Display_OK() {
    OLED_Display_Text("SAFE");
}

void ADC_Start(void) {
    ADC1->CR2 |= (1U << 30);
}

int main() {
    DHT11_Init();
    I2C_Init();
    OLED_Init();
    ADC_Init();
    ADC_Start();

    uint8_t humidity, temperature;
    uint16_t data;

    while (1) {
        data = DHT11_Read();
        humidity = (data >> 8) & 0xFF;
        temperature = data & 0xFF;

        if ((temperature < TEMP_MIN || temperature > TEMP_MAX) ||
            (humidity < HUM_MIN || humidity > HUM_MAX)) {
            OLED_Display_Danger();
        } else {
            OLED_Display_OK();
        }

        delay_ms(2000);
    }
}
