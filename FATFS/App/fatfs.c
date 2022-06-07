/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSBH;    /* Return value for USBH */
char USBHPath[4];   /* USBH logical drive path */
FATFS USBHFatFS;    /* File system object for USBH logical drive */
FIL USBHFile;       /* File object for USBH */
char USBFNAME[32] = "STM32.TXT";

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USBH driver ###########################*/
  retUSBH = FATFS_LinkDriver(&USBH_Driver, USBHPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
void FatFS_test_routine(void)
{
	FRESULT res;
	uint32_t byteswritten;

	char wtext[100] = "This line has been written by an STM32 device!\n";

	if(f_mount(&USBHFatFS, (TCHAR const*) USBHPath, 0) == FR_OK)
	{
		res = f_open(&USBHFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE);

		if (res != FR_OK)
		{
			return;
		}

		res = f_write(&USBHFile, wtext, strlen(wtext), (void *)&byteswritten);

		if (res != FR_OK)
		{
			return;
		}

		res = f_sync(&USBHFile);

		if (res != FR_OK)
		{
			return;
		}

		res = f_close(&USBHFile);

		if (res != FR_OK)
		{
			return;
		}
	}
}

void FatFS_close(void) {
  FRESULT res;
  res = f_sync(&USBHFile);

  if (res != FR_OK)
    {
      return;
    }

  res = f_close(&USBHFile);

  if (res != FR_OK)
    {
      return;
    }
}

void FatFS_open(const char *fname)
{
  FRESULT res;

  if(f_mount(&USBHFatFS, (TCHAR const*) USBHPath, 0) == FR_OK)
    {
      res = f_open(&USBHFile, fname, FA_CREATE_ALWAYS | FA_WRITE);

      if (res != FR_OK)
	{
	  return;
	}
    }
}


/* USER CODE END Application */
