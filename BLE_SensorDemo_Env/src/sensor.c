/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : sensor.c
 * Author             : IoT-EC Application team
 * Version            : V1.0.0
 * Date               : June 26, 2018
 * Description        : STEVAL-BCN002V1 Sensor init and state machines
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "SDK_EVAL_Config.h"

#include "gatt_db.h"
#include "sensor.h"
#include "hw_config.h"
#include "sleep.h"
#include "HWAdvanceFeatures.h"
#include "SensorDemo_main.h"
#include "OTA_btl.h"


#if ENABLE_BLUEVOICE
#include "bluevoice_adpcm_bnrg1.h"	// BlueVoice for CortexM0
#endif

#if ENABLE_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/* Define default BlueNRG-2 name; must be 7 char long */

#define DEFAULT_NAME_ALLMEMS		'B','C','N','-','E','N','V'

#define DEVICE_NAME_PAGE			(125)					// Board name position
#define DEVICE_NAME_ADDR			(uint32_t)0x1007E800	// Board name address

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern ADC_InitType ADC_InitStructure;

/* Connection variables */
volatile uint32_t ConnectionBleStatus = 0;
volatile uint32_t FeatureMask;
volatile uint32_t DevStatus = 0;
volatile bool set_connectable = true;
volatile bool connected = false;
volatile bool sensorTimer_expired = false;
uint16_t connection_handle = 0;
uint8_t connInfo[20];
/* End connection variables */

/* RAW Data */
float pressure = 0.0;
static axis1bit32_t data_raw_pressure;
int16_t temperature = 0;
uint16_t humidity = 0;


volatile uint16_t StepsFromPedometer = 0;
int32_t PressToSend = 0;
uint16_t HumToSend = 0;
int16_t TempToSend = 0;
volatile uint32_t start_time = 0;
float batteryVoltage = 0.0;
volatile bool isPlugged = false;

/* End RAW data */

#if ENABLE_BLUEVOICE
/* Variables for audio */
#define PCM_SAMPLES_PER_MS			(AUDIO_SAMPLING_FREQUENCY/1000)
#define SaturaLH(N, L, H)			(((N)<(L))?(L):(((N)>(H))?(H):(N)))
volatile uint8_t ready = 0;
volatile uint8_t tx_buffer_full = 0;
const uint8_t AudioVolume = 8;
HP_FilterState_TypeDef HP_Filter;
BV_Profile_Status BlueVoiceStatus;
uint16_t num_byte_sent = 0;
extern BV_ADPCM_BNRG1_Config_t BV_ADPCM_BNRG1_Config;
volatile int16_t ADC_Buffer[PCM_BUFFER_SIZE];
/* End variables for audio */
#endif


volatile FeaturePresence xFeaturePresence;
volatile FeatureNotification xFeatureNotification;
volatile HardwareFeaturePresence xHardwareFeaturePresence;

extern volatile bool FirstConnectionConfig;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : Init_Pressure_Temperature_Sensor.
 * Description    : Init LPS22HB pressure and temperature sensor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Pressure_Temperature_Sensor(void) {

	lps22hh_i3c_interface_set(0, LPS22HH_I3C_DISABLE);
	lps22hh_block_data_update_set(0, PROPERTY_ENABLE);
	lps22hh_data_rate_set(0, LPS22HH_50_Hz_LOW_NOISE);
	lps22hh_pin_mode_set(0, LPS22HH_OPEN_DRAIN);
	lps22hh_pin_polarity_set(0, LPS22HH_ACTIVE_LOW);
	lps22hh_int_notification_set(0, LPS22HH_INT_PULSED);
	lps22hh_int_pd_set(0, LPS22HH_PULL_DOWN_DISCONNECT);

}

/*******************************************************************************
 * Function Name  : Init_Humidity_Sensor.
 * Description    : Init HTS221 temperature sensor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Humidity_Sensor(void) {

	HTS221_Set_AvgH(0, HTS221_AVGH_4);
	HTS221_Set_AvgT(0, HTS221_AVGT_2);
	HTS221_Set_BduMode(0, HTS221_ENABLE);
	HTS221_Set_Odr(0, HTS221_ODR_12_5HZ);
	HTS221_Set_HeaterState(0, HTS221_DISABLE);

	HTS221_Set_IrqOutputType(0, HTS221_OPENDRAIN);
	HTS221_Set_IrqActiveLevel(0, HTS221_LOW_LVL);

	HTS221_Set_PowerDownMode(0, HTS221_RESET);
	HTS221_Activate(0);

}

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : None
 *******************************************************************************/
void Sensor_DeviceInit() {
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	uint8_t device_name[] = { 'B', 'C', 'N', '-', 'E', 'N', 'V' };

	// Set the TX power 8 dBm
	aci_hal_set_tx_power_level(1, 0x07);

	// GATT Init
	aci_gatt_init();

	// GAP Init
	aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

	// Update device name
	aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name), device_name);

	// BLE Security v4.2 is supported: BLE stack FW version >= 2.x (new API prototype)
	aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_REQUIRED, SC_IS_NOT_SUPPORTED, KEYPRESS_IS_NOT_SUPPORTED, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, 0x00);

	//aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);

	// Advertising or notification configuration
	memset((void*) &xFeatureNotification, 0, sizeof xFeatureNotification);
	xFeatureNotification.Advertising = true;

	/* Configure I2C @ 400 kHz */
	SdkEvalI2CInit(400000);

	// Check sensor list
	SensorsScan();

	// Configure discovered sensors
	if (xFeaturePresence.PressurePresence)
		Init_Pressure_Temperature_Sensor();
	if (xFeaturePresence.HumidityTemperaturePresence)
		Init_Humidity_Sensor();

	// Configure sensors in low power mode
	SensorsLowPower();

	// Init bluetooth custom service and characteristics
	Init_BlueNRG_Custom_Services();

	ADC_DMA_Configuration(ADC_Input_BattSensor);

	for (int var = 0; var <= 10; ++var) {
		ADC_Cmd(ENABLE);
		while (ADC_GetFlagStatus(ADC_FLAG_EOC) == RESET)
			;
		batteryVoltage = ADC_ConvertBatterySensor(ADC_GetRawData(), ADC_ReferenceVoltage_0V6);
	}

	ADC_DMA_Configuration(ADC_Input_None);

	PRINTF("Battery %dmV: ", (int )(batteryVoltage * 1000));
	if (batteryVoltage > 2.0)
		PRINTF("OK\n\r");
	else
		PRINTF("FAIL\n\r");

	if (batteryVoltage > 3.25)
		isPlugged = true;

	ADC_DMA_Configuration(ADC_Input_None);

	/* Configure I/O interrupts */
	Interrupts_EXT_IO_Config();

#ifdef	ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	if (!isPlugged)
	GPIO_EXTICmd(GPIO_Pin_11, ENABLE);
#endif

}



/*******************************************************************************
 * Function Name  : Set_DeviceConnectable.
 * Description    : Puts the device in connectable mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_DeviceConnectable(void) {

	uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, DEFAULT_NAME_ALLMEMS };
	uint8_t manuf_data[26] = { 2, /* lenght*/0x0A, 0x00 /* 0 dBm */,		// Transmission Power
			8, /* Length*/0x09, DEFAULT_NAME_ALLMEMS,			// Complete Name
			13, /* Length*/0xFF, 0x01, /* SKD version */
			0x05,/* 0x05 BlueNRG-Tile Board */
#if ENABLE_BLUEVOICE
			0x48, /* ADPCM Sync + ADPCM Audio + Led + Prox */
#else
			0xO0, /* Led + Prox */
#endif
			0x1E, /* Acc + Gyro + Mag + Press + Hum + Temp + Batt*/
			0x00, /* AccEvents */
			0x00,
			0x00, /* BLE MAC start */
			0x00, 0x00, 0x00, 0x00, 0x00, /* BLE MAC stop */
	};

	uint8_t sizeBDAddr;
	uint8_t randBDAddr[6];

	aci_hal_read_config_data(0x80, &sizeBDAddr, randBDAddr);

	for (uint8_t var = 0; var < 6; ++var) {
		manuf_data[25 - var] = randBDAddr[var];
	}

	if (FLASH_ReadWord(DEVICE_NAME_ADDR) != 0xFFFFFFFF) {
		for (uint8_t var = 0; var < 7; ++var) {
			local_name[var + 1] = FLASH_ReadByte(DEVICE_NAME_ADDR + var);
			manuf_data[var + 5] = FLASH_ReadByte(DEVICE_NAME_ADDR + var);
		}
	}

	// Set the TX power -2 dBm
	aci_hal_set_tx_power_level(1, 0x04);

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	hci_le_set_scan_response_data(18, BTLServiceUUID4Scan);
#else
	hci_le_set_scan_response_data(0,NULL);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
	aci_gap_set_discoverable(ADV_IND, 0x160, 0x160, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
	PRINTF("Device '%.*s' has MAC: %02x:%02x:%02x:%02x:%02x:%02x\n\r", sizeof(local_name) - 1, (local_name + 1), manuf_data[20], manuf_data[21], manuf_data[22], manuf_data[23], manuf_data[24], manuf_data[25]);

	/* Send Advertising data */
	aci_gap_update_adv_data(26, manuf_data);

	if (!isPlugged)
		HAL_VTimerStart_ms(ADVERTISING_TIMER, ADVERTISING_TIME);

}

/*******************************************************************************
 * Function Name  : Init_BlueNRG_Custom_Services.
 * Description    : Add the BLE service.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_BlueNRG_Custom_Services(void) {
	Add_HW_SW_ServW2ST_Service();
	Add_ConfigW2ST_Service();
	Add_ConsoleW2ST_Service();
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	if (OTA_Add_Btl_Service() == BLE_STATUS_SUCCESS)
		PRINTF("OTA success added\n");
	else
		PRINTF("OTA error\n");
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

}

/*******************************************************************************
 * Function Name  : User_AppTick.
 * Description    : User app state machine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void User_AppTick(void) {

	float SensorValue;
	int32_t decPart, intPart;

	if (xFeatureNotification.EnvironmentalNotification) {
		if (sensorTimer_expired) {
			sensorTimer_expired = false;
			HAL_VTimerStart_ms(SENSOR_TIMER, ENV_SENSOR_UPDATE_RATE);
			if (xFeaturePresence.PressurePresence) {
				memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
				lps22hh_pressure_raw_get(0, data_raw_pressure.u8bit);
				pressure = LPS22HH_FROM_LSB_TO_hPa(data_raw_pressure.i32bit);
				SensorValue = pressure;
				MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
				PressToSend = intPart * 100 + decPart;
			}
			if (xFeaturePresence.HumidityTemperaturePresence) {
				HTS221_Get_Measurement(0, &humidity, &temperature);
				HumToSend = humidity;
				SensorValue = ((float) temperature) / 10;
				MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
				TempToSend = intPart * 10 + decPart;
			}
			Environmental_Update(PressToSend, HumToSend, TempToSend);

		}
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);

	} else if (xFeatureNotification.BatteryMonitorNotification) {
		if (sensorTimer_expired) {
			sensorTimer_expired = false;
			ADC_DMA_Configuration(ADC_Input_BattSensor);
			while (ADC_GetFlagStatus(ADC_FLAG_EOC) == RESET)
				;
			batteryVoltage = batteryVoltage * 0.99 + ADC_ConvertBatterySensor(ADC_GetRawData(), ADC_ReferenceVoltage_0V6) * 0.01;
			HAL_VTimerStart_ms(SENSOR_TIMER, BATTERY_UPDATE_RATE);
			if (batteryVoltage > 3.2) {
				isPlugged = true;
			} else {
				isPlugged = false;
			}
			GasGauge_Update((uint32_t) (batteryVoltage * 1000));
		}
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);

	}
		else if (xFeatureNotification.MotionNotification || xFeatureNotification.ProximityNotification) {
		if (sensorTimer_expired) {
			sensorTimer_expired = false;
			if (FirstConnectionConfig) {
				Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, 0);
				Config_Notify(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, 0);
				FirstConnectionConfig = false;
			}
			
			HAL_VTimerStart_ms(SENSOR_TIMER, MOTION_SENSOR_UPDATE_RATE);
		}



	}
#if ENABLE_BLUEVOICE
	else if (xFeatureNotification.AudioNotification) {
		BlueVoiceStatus = BluevoiceADPCM_BNRG1_GetStatus();
		switch (BlueVoiceStatus) {
		case BLUEVOICE_STATUS_UNITIALIZED:
			BluevoiceADPCM_BNRG1_Initialize();
			break;
		case BLUEVOICE_STATUS_READY:
			break;
		case BLUEVOICE_STATUS_STREAMING:
			if (ready && !tx_buffer_full) {
				if (BluevoiceADPCM_BNRG1_SendData() == BV_INSUFFICIENT_RESOURCES)
					tx_buffer_full = 1;
				else
					ready = 0;
			}
			break;
		default:
			break;
		}
		HAL_VTimer_Stop(SENSOR_TIMER);
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_CPU_HALT, 0, 0);
	}
#endif
	else if (xFeatureNotification.LedNotification) {
		HAL_VTimerStart_ms(SENSOR_TIMER, 500);
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_WAKETIMER, 0, 0);
	} else if (xFeatureNotification.AccEventNotification) {
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO13, (WAKEUP_IOx_LOW << WAKEUP_IO13_SHIFT_MASK));
	} else if (xFeatureNotification.Advertising) { 					// Deep sleep with advertising
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
	} else if (!(xFeatureNotification.Advertising) && !connected) { // Deep sleep no advertising

		if (!isPlugged) {
			if (GPIO_ReadBit(GPIO_Pin_11) == Bit_SET)
				BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO11, (WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK));
			else
				BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO11, (WAKEUP_IOx_HIGH << WAKEUP_IO11_SHIFT_MASK));
		}
	} else {
		if (!isPlugged)
			BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
	}
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Role, uint8_t Peer_Address_Type, uint8_t Peer_Address[6], uint16_t Conn_Interval, uint16_t Conn_Latency, uint16_t Supervision_Timeout, uint8_t Master_Clock_Accuracy) {

	connection_handle = Connection_Handle;
	ConnectionBleStatus = 0;
#if ENABLE_BLUEVOICE
	BluevoiceADPCM_BNRG1_ConnectionComplete_CB(Connection_Handle);
#endif

	HAL_VTimer_Stop(SENSOR_TIMER);
	HAL_VTimer_Stop(ADVERTISING_TIMER);
	HAL_VTimer_Stop(RESET_TIMER);

	aci_l2cap_connection_parameter_update_req(connection_handle, 8, 17, 0, 400);

	memset((void*) &xFeatureNotification, 0, sizeof xFeatureNotification);

	sensorTimer_expired = false;
	HAL_VTimerStart_ms(SENSOR_TIMER, ENV_SENSOR_UPDATE_RATE);
	start_time = HAL_VTimerGetCurrentTime_sysT32();
	PRINTF("Device connected\n\r");
	connected = TRUE;

}

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status, uint16_t Connection_Handle, uint8_t Reason) {

	SensorsLowPower();

	memset((void*) &xFeatureNotification, 0, sizeof xFeatureNotification);
	xFeatureNotification.Advertising = true;

	connected = false;
	connection_handle = 0;

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	OTA_terminate_connection();
#endif

	ADC_DMA_Configuration(ADC_Input_BattSensor);
	while (ADC_GetFlagStatus(ADC_FLAG_EOC) == RESET)
		;
	batteryVoltage = batteryVoltage * 0.9 + ADC_ConvertBatterySensor(ADC_GetRawData(), ADC_ReferenceVoltage_0V6) * 0.1;

	if (batteryVoltage > 3.2)
		isPlugged = true;
	else
		isPlugged = false;

	ADC_DMA_Configuration(ADC_Input_None);

	/* Set device connectable (advertising) */
	Set_DeviceConnectable();

}

/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint16_t Offset) {

	Read_Request_CB(Attribute_Handle);

}

/*******************************************************************************
 * Function Name  : HAL_VTimerTimeoutCallback.
 * Description    : This function will be called on the expiration of
 *                  a one-shot virtual timer.
 * Input          : See file bluenrg1_stack.h
 * Output         : See file bluenrg1_stack.h
 * Return         : See file bluenrg1_stack.h
 *******************************************************************************/
void HAL_VTimerTimeoutCallback(uint8_t timerNum) {

	switch (timerNum) {
	case SENSOR_TIMER:	// 50 Hz
		sensorTimer_expired = true;
		break;

	case ADVERTISING_TIMER:	// 20 seconds
		if (!connected) {
			PRINTF("Entering sleep...\n\r");
			aci_gap_set_non_discoverable();
			xFeatureNotification.Advertising = false;
		}
		break;
	case RESET_TIMER:	// 5 seconds
		aci_gap_set_non_discoverable();
		NVIC_SystemReset();
		break;
	default:
		break;
	}

}

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset, uint16_t Attr_Data_Length, uint8_t Attr_Data[]) {
	Attribute_Modified_CB(Attr_Handle, Attr_Data, Attr_Data_Length);
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	OTA_Write_Request_CB(Connection_Handle, Attr_Handle, Attr_Data_Length, Attr_Data);
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State, uint8_t Next_State, uint32_t Next_State_SysTime) {
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
	if (Next_State == 0x02) { /* 0x02: Connection event slave */
		OTA_Radio_Activity(Next_State_SysTime);
	}
#endif
}

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle, uint16_t Available_Buffers) {
	/* It allows to notify when at least 2 GATT TX buffers are available */
#if ENABLE_BLUEVOICE
	tx_buffer_full = 0;
#endif
}

void aci_gap_pairing_complete_event(uint16_t Connection_Handle, uint8_t Status, uint8_t Reason) {

}

void aci_gap_pass_key_req_event(uint16_t Connection_Handle) {
	uint8_t ret = aci_gap_pass_key_resp(Connection_Handle, 123456);
	PRINTF("Pass key req: %d", ret);

}

/*******************************************************************************
 * Function Name  : SensorsScan
 * Description    : Configure the sensors in active or low-power mode.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void SensorsScan(void) {
	uint8_t who_am_I_8 = 0x00;

	PRINTF("Scan for sensors:\n\r");

	xFeaturePresence.AccelerometerGyroscopePresence = false;
	xFeaturePresence.Pedometer = false;

	lps22hh_device_id_get(0, &who_am_I_8);
	if (who_am_I_8 == LPS22HH_ID) {
		PRINTF("- Pressure and Temperature: OK\n\r");
		xFeaturePresence.PressurePresence = true;
	} else {
		PRINTF("- Pressure and Temperature: FAIL\n\r");
		xFeaturePresence.PressurePresence = false;
	}

	HTS221_Get_DeviceID(0, &who_am_I_8);
	if (who_am_I_8 == 0xBC) {
		PRINTF("- Humidity and Temperature: OK\n\r");
		xFeaturePresence.HumidityTemperaturePresence = true;
	} else {
		PRINTF("- Humidity and Temperature: FAIL\n\r");
		xFeaturePresence.HumidityTemperaturePresence = false;
	}

	PRINTF("- Magnetometer: FAIL\n\r- Proximity Sensor: ");
	xFeaturePresence.MagnetometerPresence = false;
  
	
	xFeaturePresence.ProximityLightPresence = false;

}

/*******************************************************************************
 * Function Name  : SensorsConfiguration
 * Description    : Configure the sensors low-power mode.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void SensorsLowPower(void) {

	if (xFeaturePresence.PressurePresence) {
		lps22hh_data_rate_set(0, LPS22HH_POWER_DOWN);
	}
	if (xFeaturePresence.HumidityTemperaturePresence) {
		HTS221_Set_PowerDownMode(0, HTS221_SET);
	}
	PRINTF("Sensor in low-power: OK\n\r");
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length) {
	FeatureMask = (att_data[3]) | (att_data[2] << 8) | (att_data[1] << 16) | (att_data[0] << 24);
	uint8_t Command = att_data[4];
	uint32_t SendItBack = 1;

	switch (FeatureMask) {

	case FEATURE_MASK_ACC_EVENTS:
	case FEATURE_MASK_TEMP1:
	case FEATURE_MASK_TEMP2:
	case FEATURE_MASK_PRESS:
	case FEATURE_MASK_HUM:
	case FEATURE_MASK_ACC:
	case FEATURE_MASK_GRYO:
	case FEATURE_MASK_MAG:
		break;

	case FEATURE_MASK_SENSORFUSION_SHORT:
	case FEATURE_MASK_ECOMPASS:
		/* Sensor Fusion and e-compass */
		switch (Command) {
		case W2ST_COMMAND_CAL_STATUS: {
			/* Replay with the calibration status for the feature */
			/* Control the calibration status */
			//Config_Notify(FeatureMask, Command, mag_is_calibrated ? 100 : 0);
		}
			break;
		case W2ST_COMMAND_CAL_RESET:

			break;
		case W2ST_COMMAND_CAL_STOP:
			/* Do nothing in this case */
			break;
		}
		break;

	}

	return SendItBack;
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length) {
	uint32_t SendBackData = 1;

	/* Help Command */
	if (!strncmp("help", (char *) (att_data), 4)) {
		/* Print Legend */
		SendBackData = 0;
		BytesToWrite = sprintf((char *) BufferToWrite, "Commands available:\r\n"
				"info\r\n"
				"versionFw\r\n"
				"versionBle\r\n"
				"setName xxxxxxx\r\n"
				"powerStatus\r\n"
				"compiler\r\n"
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
				"OTAServiceManager\r\n"
#endif
		);
		Term_Update(BufferToWrite, BytesToWrite);
		/* Info Command */
	} else if (!strncmp("info", (char *) (att_data), 4)) {
		SendBackData = 0;
		BytesToWrite = sprintf((char *) BufferToWrite, "\r\nSTMicroelectronics %s:\r\n"
				"\tVersion %c.%c.%c\r\n"
				"\tBlueNRG-Tile\r\n", "STEVAL-BCN002V1 DK", '1', '2', '0');
		Term_Update(BufferToWrite, BytesToWrite);
	} else if (!strncmp("compiler", (char *) (att_data), 8)) {
		SendBackData = 0;
		BytesToWrite = sprintf((char *) BufferToWrite, "\tCompiled with"
#if defined (__IAR_SYSTEMS_ICC__)
				" IAR\r\n"
#elif defined (__GNUC__)
						" TrueSTUDIO\r\n"
#else
				" KEIL\r\n"
#endif
				"\t%s, %s\r\n", __DATE__, __TIME__);
		Term_Update(BufferToWrite, BytesToWrite);
	} else if (!strncmp("versionFw", (char *) (att_data), 9)) {
		SendBackData = 0;
		BytesToWrite = sprintf((char *) BufferToWrite, "BlueNRG-2_%s_%c.%c.%c\r\n", "STEVAL-BCN002V1-DK", '1', '2', '0');
		Term_Update(BufferToWrite, BytesToWrite);
	} else if (!strncmp("versionBle", (char *) (att_data), 10)) {
		BytesToWrite = sprintf((char *) BufferToWrite, "BlueNRG-2_1.2.0\r\n");
		Term_Update(BufferToWrite, BytesToWrite);
		SendBackData = 0;
	} else if (!strncmp("powerStatus", (char *) (att_data), 11)) {
		SendBackData = 0;
		if ((batteryVoltage > 3.25) && (batteryVoltage < 3.35)) {
			BytesToWrite = sprintf((char *) BufferToWrite, "Powered by USB at %dmV \r\n", (int) (batteryVoltage * 1000));
		} else {
			BytesToWrite = sprintf((char *) BufferToWrite, "CR2032 %d%% %dmV \r\n", (int) ((batteryVoltage - 2.0) * 100), (int) (batteryVoltage * 1000));
		}
		Term_Update(BufferToWrite, BytesToWrite);
	} else if (!strncmp("setName ", (char *) (att_data), 8)) {

		int NameLength = data_length - 1;
		if (NameLength > 8) {
			FLASH_ErasePage(DEVICE_NAME_PAGE);					// Erase page containing device name
			while (FLASH_GetFlagStatus(Flash_CMDDONE) != SET)
				;												// Wait for the end of erase operation

			uint32_t b_name[4] = { 0, 0, 0, 0 };
			b_name[1] = (((uint32_t) att_data[15]) << 24) + (((uint32_t) att_data[14]) << 16) + (((uint32_t) att_data[13]) << 8) + ((uint32_t) att_data[12]);
			b_name[0] = (((uint32_t) att_data[11]) << 24) + (((uint32_t) att_data[10]) << 16) + (((uint32_t) att_data[9]) << 8) + ((uint32_t) att_data[8]);

			FLASH_ProgramWordBurst(DEVICE_NAME_ADDR, b_name);
			while (FLASH_GetFlagStatus(Flash_CMDDONE) != SET)
				;
			BytesToWrite = sprintf((char *) BufferToWrite, "\nThe node name has been updated\r\n");
			Term_Update(BufferToWrite, BytesToWrite);
		} else {
			BytesToWrite = sprintf((char *) BufferToWrite, "\nInsert the node name\r\n");
			Term_Update(BufferToWrite, BytesToWrite);
			BytesToWrite = sprintf((char *) BufferToWrite, "Use command: setName 'xxxxxxx'\r\n");
			Term_Update(BufferToWrite, BytesToWrite);
		}

		SendBackData = 0;
	}
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	else if (!strncmp("OTAServiceManager", (char *) (att_data), 17)) {
		BytesToWrite = sprintf((char *) BufferToWrite, "Resetting HW for OTA\r\n");
		Term_Update(BufferToWrite, BytesToWrite);
		OTA_Jump_To_Service_Manager_Application();
	}
#endif
	return SendBackData;
}

#if ENABLE_BLUEVOICE
/**
 * @brief  Transfer complete callback.
 * @param  None
 * @retval None
 */
void TC_IT_Callback(void) {
	/* PCM data filtering */
	for (uint16_t i = 8 * MS_IN; i < 8 * 2 * MS_IN; i++) {
		HP_Filter.Z = ADC_Buffer[i] * AudioVolume;
		HP_Filter.oldOut = (0xFC * (HP_Filter.oldOut + HP_Filter.Z - HP_Filter.oldIn)) / 256;
		HP_Filter.oldIn = HP_Filter.Z;
		ADC_Buffer[i] = SaturaLH(HP_Filter.oldOut, -32760, 32760);
	}

	BVL_APP_PER_AudioProcess((uint16_t*) &ADC_Buffer[8 * MS_IN]);

}

/**
 * @brief  Half Transfer callback.
 * @param  None
 * @retval None
 */
void HT_IT_Callback(void) {
	/* PCM data filtering */
	for (uint16_t i = 0; i < 8 * MS_IN; i++) {
		HP_Filter.Z = ADC_Buffer[i] * AudioVolume;
		HP_Filter.oldOut = (0xFC * (HP_Filter.oldOut + HP_Filter.Z - HP_Filter.oldIn)) / 256;
		HP_Filter.oldIn = HP_Filter.Z;
		ADC_Buffer[i] = SaturaLH(HP_Filter.oldOut, -32760, 32760);
	}
	BVL_APP_PER_AudioProcess((uint16_t*) ADC_Buffer);

}

BV_BNRG1_Status status;

/**
 * @brief  Audio Process function: BlueVoice buffer filling.
 * @param  PCM_Buffer: PCM input buffer.
 * @retval None.
 */
void BVL_APP_PER_AudioProcess(uint16_t* PCM_Buffer) {

	/*BlueVoice data filling*/
	if (BluevoiceADPCM_BNRG1_IsProfileConfigured()) {
		status = BluevoiceADPCM_BNRG1_AudioIn((uint16_t*) PCM_Buffer, PCM_SAMPLES_PER_MS * MS_IN);
		if (status == BV_OUT_BUF_READY) {
			ready = 1;
		}
	}
}

#endif

