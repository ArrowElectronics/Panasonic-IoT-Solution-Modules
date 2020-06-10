/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : SensorDemo_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 19th, 2019
 * Description        : Code demonstrating a proprietary Bluetooth Low Energy profile: the sensor profile.
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file SensorDemo_main.c
 * @brief This application contains an example which shows how implementing a proprietary
 * Bluetooth Low Energy profile: the sensor profile. It also provides a reference example about how using the
 * BLE Over-The-Air (OTA) firmware upgrade capability with the BLE Sensor Demo.
 *

 * \section ATOLLIC_project ATOLLIC project
 To use the project with ATOLLIC TrueSTUDIO for ARM, please follow the instructions below:
 -# Open the ATOLLIC TrueSTUDIO for ARM and select File->Import... Project menu.
 -# Select Existing Projects into Workspace.
 -# Select the ATOLLIC project
 -# Select desired configuration to build from Project->Manage Configurations
 -# Select Project->Rebuild Project. This will recompile and link the entire application

 * \section KEIL_project KEIL project
 To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
 -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
 -# Open the KEIL project
 <tt> ...\\Project\\BLE_Examples\\BLE_Chat\\MDK-ARM\\BlueNRG-2\\BLE_Chat.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_Chat\\EWARM\\BlueNRG-2\\BLE_Chat.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c HigherApp_OTA - Release configuration for Higher Application with OTA Service
- \c LowerApp_OTA - Release configuration for Lower Application with OTA Service
- \c Release - Release configuration
- \c Use_OTA_ServiceManager - Release configuration for Application using OTA Service Manager

 * \section Board_supported Boards supported
  - \c STEVAL-BCN002V1

 * \section Serial_IO Serial I/O
 The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
 The application jumps to the OTA service manager if "OTAServiceManager" is typed and return is pressed.
 The remote node will listen for RF messages and it will output them in the serial port.

 @table
 | Parameter name  | Value               | Unit      |
 -----------------------------------------------------
 | Baudrate        | 115200 [default]    | bit/sec   |
 | Data bits       | 8                   | bit       |
 | Parity          | None                | bit       |
 | Stop bits       | 1                   | bit       |
 @endtable


 * \section Buttons_description Buttons description
 @table

 |  Configuration |                      Server_Use_OTA_ServiceManager                         |
 -----------------------------------------------------------------------------------------------
 |   BUTTON name  |                                  SW1                                       |
 -----------------------------------------------------------------------------------------------
 |    Function    |    Press 5 sec to jump to OTA service Manager *when battery operated*      |

 @endtable

* \section Usage Usage
- OTA service support for lower or higher application is enabled, respectively,  through ST_OTA_LOWER_APPLICATION=1 or ST_OTA_HIGHER_APPLICATION=1 (preprocessor, linker) options and files: OTA_btl.
- OTA service manager support is enabled, respectively,  through ST_USE_OTA_SERVICE_MANAGER_APPLICATION (preprocessor, linker) options and files: OTA_btl.
- OTA FW upgrade feature is supported only on BlueNRG-2, BLE stack v2.x.
**/

/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 SensorDemo \see SensorDemo_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "sensor.h"
#include "SensorDemo_config.h"
#include "gatt_db.h"
#include "hw_config.h"

#include "OTA_btl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#if ENABLE_BLUEVOICE
#include "bluevoice_adpcm_bnrg1.h"	// BlueVoice for CortexM0
#endif

#if ENABLE_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
uint8_t Services_Max_Attribute_Records[NUM_APP_GATT_SERVICES] = { MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_3 };

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void) {

	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Init the Hardware platform */
	Platform_Init();

	/* BlueNRG-2 stack Init */
	BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);

	/* Sensor Device Init */
	Sensor_DeviceInit();

	/* Set device connectable (advertising) */
	Set_DeviceConnectable();

	while (1) {
		/* BLE Stack Tick */
		BTLE_StackTick();

		/* User Application Tick */
		User_AppTick();

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
		/* Check if the OTA firmware upgrade session has been completed */
		if (OTA_Tick() == 1) {
			/* Jump to the new application */
			OTA_Jump_To_New_Application();
		}
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

	}/* while (1) */
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
	if (SdkEvalComIOTxFifoNotEmpty())
		return SLEEPMODE_RUNNING;
	return sleepMode;
}

/***************************************************************************************/

#ifdef USE_FULL_ASSERT
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
