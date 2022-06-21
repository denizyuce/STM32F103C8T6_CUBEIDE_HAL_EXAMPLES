# STM32F103C8T6-ADXL345-UART-HAL
## Amaç
**ADXL345** ivme sensöründen alınan veriler, **UART** çevre birimi üzerinden **seriport** ekranına gönderiliyor.
## Kullanılan Donanımlar
- StlinkV2
- Stm32f103c8t6 (Bluepill)
- USB/TTL Dönüştürücü
- ADXL345 
## Kullanılan Yazılımlar
- STM32CUBEIDE
- STM32PROGRAMMER
- Arduino Serial Port
## Kullanılan Çevrebirimleri
- RCC
- GPIO
- UART
- I2C
## .ioc Ayarları
- ##### Pinout & Configuration -> System Core -> Sys -> Debug -> ``Serial Wire``
- ##### Pinout & Configuration -> System Core -> RCC -> High Speed Clock (HSE) -> ``Crystal/Ceramic Resanator``
- ##### Clock Configuration -> HCLK(Mhz) -> ``72``
- ##### Project Manager -> Code Generator -> ``Generate peripheral initialization as a pair of '.c/.h' files per peripheral``
- ##### Pinout & Configuration -> Connectivity -> I2C1 -> Mode -> ``I2C``
- ##### Pinout & Configuration -> Connectivity -> USART1 -> Mode -> ``Asynchronous``
- ##### Pinout & Configuration -> Connectivity -> USART1 -> Parameter Setting -> Basic Parameters -> Baud Rate -> ``9600 Bits/s``
- ##### Pinout View -> PC13 -> ``GPIO_Output``
## MCU Pin Yapısı
| **STM32F103C8T6** | **Pin Tanımı** |
| ------ | ---------- |
| VDD | 3.3V |
| VSS | GND |
| PD0 | RCC_OSC_IN |
| PD1 | RCC_OSC_OUT |
| PA13 | SWDIO |
| PA14 | SWCLK |
| PA9 | USART1_TX |
| PA10 | USART1_RX |
| PB7 | I2C Veri Hattı |
| PB6 | I2C Clock Hattı |
| PC13 | Led Çıkışı |
## I2C Kullanımı İçin Bağlantı
| **Bluepill** | **ADXL345** | **Pin Tanımı** |
| ------ | ------- | ---------- |
| 3V3 | VCC | Besleme Hattı |
| GND | GND | Besleme Hattı |
| PB7 | SDA | I2C Veri Hattı |
| PB6 | SCL | I2C Clock Hattı |
## Akış Diyagramı
## Kütüphane Kurulumu Ve Kod İncelenmesi
- ##### yclabs_adxl345.h -> Project Folder -> Core -> Inc -> ``Copy Paste``
- ##### yclabs_adxl345.c -> Project Folder -> Core -> Src -> ``Copy Paste``
### Başlık Dosyası (.h)
````C
/******************** YCLabs 2022 ***************************************************
* File Name          : yclabs_adxl345.h
* Author             : Mazlum Deniz Yüce
* Version            : V1.0
* Description        : Descriptor Header for yclabs_adxl345.c driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 10/06/2022  | Initial Revision                            | Mazlum Deniz Yüce
***********************************************************************************/
/*------------- Define to prevent recursive inclusion -----------------------------*/
#ifndef __YCLABS_ADXL345_H
#define __YCLABS_ADXL345_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern char buffer_acce[50];
extern uint8_t chipid;
extern uint8_t axis_data[6];
extern int16_t x_axis ,y_axis ,z_axis;
extern float xg,yg,zg;
//Register Definition
#define ADXL345_DEFAULT_ADRESS  0x53<<1
#define DEVICE_ID				0x00//Device Adress
#define DATA_FORMAT_REGISTER	0X31
#define RANGE_2G				0X01
#define POWER_CONTROL_REGISTER	0X2D
#define REGISTER_CLEAR			0X00
#define MESURE_MOD				0X08
#define DATA_X					0X32

/* Exported functions --------------------------------------------------------*/
//Sensor Configuration Functions
void adxl345_init(void);
void adxl345_read_address (uint8_t adress);
void adxl345_read(uint8_t adress);
void adxl345_write(uint8_t adress,uint8_t value);
void adxl345_Data_Collect(void);
void data_push_to_serialport(void);
void buffer_clear(char *buffer,uint8_t size);

#endif /* __YCLABS_ADXL345_H */
````

### Kaynak Dosyası (.c)
````C
/******************** YCLabs 2022 ***************************************************
* File Name          : yclabs_adxl345.c
* Author             : Mazlum Deniz Yüce
* Version            : V1.0
* Description        : yclabs_adxl345.c driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 10/06/2022  | Initial Revision                            | Mazlum Deniz Yüce
***********************************************************************************/
/* ---------------------Includes -------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "yclabs_adxl345.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name		: adxl345_init
* Description		: ADXL345 setup
* Input			: None
* Output		: None
* Return		: None
*******************************************************************************/
void adxl345_init(void){
	adxl345_read_address (DEVICE_ID); // read the DEVID
	adxl345_write(DATA_FORMAT_REGISTER, RANGE_2G); //Data format aralığı +/- 2g seçilir
	adxl345_write(POWER_CONTROL_REGISTER, REGISTER_CLEAR); //İlgili register temizlenir.
	adxl345_write(POWER_CONTROL_REGISTER, MESURE_MOD); //Ölçüm modunda ve 8Hz 'de uyanma gerçekleşir.
}


/*******************************************************************************
* Function Name  : adxl345_read_address
* Description    : Read ADXL345 Adress
* Input          : adress
* Output         : None
* Return         : None
*******************************************************************************/
void adxl345_read_address (uint8_t adress)
{
	HAL_I2C_Mem_Read (&hi2c1, ADXL345_DEFAULT_ADRESS, adress, 1, &chipid, 1, 100);
}


/*******************************************************************************
* Function Name  : adxl345_read
* Description    : Read data from ADXL345
* Input          : adress
* Output         : None
* Return         : None
*******************************************************************************/
void adxl345_read(uint8_t adress){
	HAL_I2C_Mem_Read (&hi2c1, ADXL345_DEFAULT_ADRESS, adress, 1, (uint8_t *)axis_data, 6, 100);
}


/*******************************************************************************
* Function Name  : adxl345_write
* Description    : Write command to ADXL345
* Input          : adress,value
* Output         : None
* Return         : None
*******************************************************************************/
void adxl345_write(uint8_t adress,uint8_t value){
	uint8_t data[2];
	data[0]=adress; //Çok baytlı yazmayı aktif ediyoruz.
	data[1]=value;
	HAL_I2C_Master_Transmit (&hi2c1, ADXL345_DEFAULT_ADRESS, data, 2, 100);
}


/*******************************************************************************
* Function Name  : adxl345_Data_Collect
* Description    : Read data from ADXL345 and create acceralation values
* Input          : None
* Output         : xg,yg,zg
* Return         : None
*******************************************************************************/
void adxl345_Data_Collect(void){
	 adxl345_read(DATA_X);

	 x_axis=((axis_data[1]<<8) | axis_data[0]);
	 y_axis=((axis_data[3]<<8) | axis_data[2]);
	 z_axis=((axis_data[5]<<8) | axis_data[4]);

	 xg=x_axis*.0078125;
	 yg=y_axis*.0078125;
	 zg=z_axis*.0078125;
}


/*******************************************************************************
* Function Name  : buffer_clear
* Description    : Clearing the buffer
* Input          : buffer,size
* Output         : None
* Return         : None
*******************************************************************************/
void buffer_clear(char *buffer,uint8_t size){
	for (uint8_t i=0;i<size;i++){
		buffer[i]='\0';
	}
}
````

### main.c
````C
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "yclabs_adxl345.h"
#define adxl_address 0x53<<1
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static char* intro="ADXL345 Sensöründen İvme Verilerini Okuma\r\n";
char buffer[50];
uint8_t chipid=0;
uint8_t axis_data[6];
int16_t x_axis ,y_axis ,z_axis;
float xg,yg,zg;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
void data_push_to_serialport(void){
		  sprintf(buffer,"\r\nxg :%f\r\nyg :%f\r\nzg :%f\r\n",xg,yg,zg);
		  HAL_UART_Transmit(&huart1,(uint8_t*)buffer, sizeof(buffer), 100);
		  buffer_clear(buffer,50);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(500);
}
/* USER CODE END 0 */

  /* USER CODE BEGIN 2 */
  adxl345_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, (uint8_t*)intro,42, 100);
  sprintf(buffer,"\nChip ID :%u\r\n",chipid);
  HAL_UART_Transmit(&huart1,(uint8_t*)buffer, sizeof(buffer), 100);
    while (1)
  {
	 adxl345_Data_Collect();
	 data_push_to_serialport();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
````
## Sonuç

## Geliştirme
