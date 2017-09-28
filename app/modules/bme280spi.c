// ***************************************************************************
// BMP280 over SPI module for ESP8266 with nodeMCU
//
// Written by John Bayly, @oobayly
// Defines taken from BMP280 by Lukas Voborsky
//
// MIT license, http://opensource.org/licenses/MIT
// ***************************************************************************

//#define NODE_DEBUG

#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "c_math.h"

#include "driver/spi.h"

#define HIGH                                  PLATFORM_GPIO_HIGH
#define LOW                                   PLATFORM_GPIO_LOW
/****************************************************/
/**\name        SPI definitions */
/***************************************************/
#define BME280_SPI_MODE                       PLATFORM_SPI_MASTER
#define BME280_SPI_CPOL                       PLATFORM_SPI_CPOL_LOW
#define BME280_SPI_CPHA                       PLATFORM_SPI_CPHA_LOW
#define BME280_SPI_DATA_BITS                  (8)
#define BME280_SPI_CLOCK_DIV                  (16)      // 5MHz
/****************************************************/
/**\name        registers definition  */
/***************************************************/
#define BME280_REGISTER_CONTROL               (0xF4)
#define BME280_REGISTER_CONTROL_HUM           (0xF2)
#define BME280_REGISTER_CONFIG                (0xF5)
#define BME280_REGISTER_CHIPID                (0xD0)
#define BME280_REGISTER_VERSION               (0xD1)
#define BME280_REGISTER_SOFTRESET             (0xE0)
#define BME280_REGISTER_CAL26                 (0xE1)
#define BME280_REGISTER_STATUS                (0xF3)
#define BME280_REGISTER_PRESS                 (0xF7)    // 0xF7-0xF9
#define BME280_REGISTER_TEMP                  (0xFA)    // 0xFA-0xFC
#define BME280_REGISTER_HUM                   (0xFD)    // 0xFD-0xFE

#define BME280_REGISTER_DIG_T                 (0x88)    // 0x88-0x8D ( 6)
#define BME280_REGISTER_DIG_P                 (0x8E)    // 0x8E-0x9F (18)
#define BME280_REGISTER_DIG_H1                (0xA1)    // 0xA1      ( 1)
#define BME280_REGISTER_DIG_H2                (0xE1)    // 0xE1-0xE7 ( 7)
/****************************************************/
/**\name        POWER MODE DEFINITIONS  */
/***************************************************/
/* Sensor Specific constants */
#define BME280_SLEEP_MODE                    (0x00)
#define BME280_FORCED_MODE                   (0x01)
#define BME280_NORMAL_MODE                   (0x03)
#define BME280_SOFT_RESET_CODE               (0xB6)
/****************************************************/
/**\name        OVER SAMPLING DEFINITIONS  */
/***************************************************/
#define BME280_OVERSAMP_1X                    (0x01)
#define BME280_OVERSAMP_2X                    (0x02)
#define BME280_OVERSAMP_4X                    (0x03)
#define BME280_OVERSAMP_8X                    (0x04)
#define BME280_OVERSAMP_16X                   (0x05)
/****************************************************/
/**\name        STANDBY TIME DEFINITIONS  */
/***************************************************/
#define BME280_STANDBY_TIME_1_MS              (0x00)
#define BME280_STANDBY_TIME_63_MS             (0x01)
#define BME280_STANDBY_TIME_125_MS            (0x02)
#define BME280_STANDBY_TIME_250_MS            (0x03)
#define BME280_STANDBY_TIME_500_MS            (0x04)
#define BME280_STANDBY_TIME_1000_MS           (0x05)
#define BME280_STANDBY_TIME_10_MS             (0x06)
#define BME280_STANDBY_TIME_20_MS             (0x07)
/****************************************************/
/**\name        FILTER DEFINITIONS  */
/***************************************************/
#define BME280_FILTER_COEFF_OFF               (0x00)
#define BME280_FILTER_COEFF_2                 (0x01)
#define BME280_FILTER_COEFF_4                 (0x02)
#define BME280_FILTER_COEFF_8                 (0x03)
#define BME280_FILTER_COEFF_16                (0x04)
/****************************************************/
/**\data type definition  */
/***************************************************/
#define BME280_S32_t int32_t
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t

#define r16uLE_buf(reg) (uint16_t)((reg[1] << 8) | reg[0])
#define r16sLE_buf(reg) (int16_t)(r16uLE_buf(reg))

typedef struct bme280_spi_data {
	uint8_t		id;
	uint8_t		cs;
  uint16_t  dig_T1;
  int16_t   dig_T2;
  int16_t   dig_T3;
  uint16_t  dig_P1;
  int16_t   dig_P2;
  int16_t   dig_P3;
  int16_t   dig_P4;
  int16_t   dig_P5;
  int16_t   dig_P6;
  int16_t   dig_P7;
  int16_t   dig_P8;
  int16_t   dig_P9;
  uint8_t   dig_H1;
  int16_t   dig_H2;
  uint8_t   dig_H3;
  int16_t   dig_H4;
  int16_t   dig_H5;
  int8_t    dig_H6;
} bme280_spi_data;

// Lua methods
static int32_t			bme280_spi_lua_baro(lua_State * L);
static uint8_t			bme280_spi_lua_getData(lua_State * L, int index, bme280_spi_data * data);
static int32_t			bme280_spi_lua_humi(lua_State * L);
static int32_t			bme280_spi_lua_read(lua_State * L);
static int32_t			bme280_spi_lua_temp(lua_State * L);

// Sensor helper methods
static uint8_t			bme280_spi_getBaro(bme280_spi_data * data, uint8_t * buf, uint32_t * P,			BME280_S32_t * t_fine);
static uint8_t			bme280_spi_getHumi(bme280_spi_data * data, uint8_t * buf, uint32_t * H,			BME280_S32_t * t_fine);
static uint8_t			bme280_spi_getTemp(bme280_spi_data * data, uint8_t * buf, BME280_S32_t * T,	BME280_S32_t * t_fine);
static uint32_t			bme280_spi_qfe2qnh(uint32_t qfe, int32_t h);

// Bosch compensation methods
static							BME280_U32_t bme280_spi_compensate_H(BME280_S32_t adc_H, bme280_spi_data * data, BME280_S32_t * t_fine);
static							BME280_U32_t bme280_spi_compensate_P(BME280_S32_t adc_P, bme280_spi_data * data, BME280_S32_t * t_fine);
static 							BME280_S32_t bme280_spi_compensate_T(BME280_S32_t adc_T, bme280_spi_data * data, BME280_S32_t * t_fine);

// Module methods
static void 				bme280_spi_delay(uint16_t delay);
static uint8_t 			bme280_spi_isReadingCalibration(uint8_t id, uint8_t cs);
static void 				bme280_spi_readCoefficients(bme280_spi_data * data);
static void 				bme280_spi_read(uint8_t id, uint8_t cs, uint8_t reg, uint8_t len, uint8_t * resp);
static uint8_t	 		bme280_spi_read8(uint8_t id, uint8_t cs, uint8_t reg);
static uint16_t 		bme280_spi_read16(uint8_t id, uint8_t cs, uint8_t reg);
static uint16_t 		bme280_spi_read16_LE(uint8_t id, uint8_t cs, uint8_t reg);
static uint32_t		 	bme280_spi_read24(uint8_t id, uint8_t cs, uint8_t reg);
static uint8_t 			bme280_spi_write8(uint8_t id, uint8_t cs, uint8_t reg, uint8_t data);

/****************************************************/
/**\name        Lua methods */
/***************************************************/

// Lua: = bme280spi.setup( id, cs, [clock_div])
// Returns a string containing the data that needs to be passed to any subsequent methods
static int bme280_spi_lua_setup(lua_State *L) {
  uint8_t id            = luaL_checkinteger(L, 1);
  uint8_t cs            = luaL_checkinteger(L, 2);
  uint32_t clock_div     = luaL_optinteger(L, 3, BME280_SPI_CLOCK_DIV);

  MOD_CHECK_ID(spi , id);

  // For backwards compatability
  if (clock_div == 0) {
    clock_div = BME280_SPI_CLOCK_DIV;
  }

  // Configure the cs pin for output and default high (inactive)
  platform_gpio_mode(cs, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_PULLUP);
  platform_gpio_write(cs, HIGH);
  platform_gpio_write(cs, LOW);
  platform_gpio_write(cs, HIGH);

  // Initialise SPI
  platform_spi_setup(id, BME280_SPI_MODE, BME280_SPI_CPOL, BME280_SPI_CPHA, clock_div);

  // Fetch the device ID
  uint8_t device_id = bme280_spi_read8(id, cs, BME280_REGISTER_CHIPID);
  if (device_id != 0x60) {
    return luaL_error( L, "Invalid response for the Chip ID" );
  }

  // Soft reset to make sure IIR is off, etc.
  bme280_spi_write8(id, cs, BME280_REGISTER_SOFTRESET, 0xb6);
  bme280_spi_delay(10000);

  // Wait until the status is ready
  while (bme280_spi_isReadingCalibration(id, cs)) {
    bme280_spi_delay(1000);
  }

	// This contains all the data that is necessary for reading a specific BME280 device
	// and will be returned to the caller
	bme280_spi_data data;
	data.id = id;
	data.cs = cs;

	// Read the calibration coefficients into the data
  bme280_spi_readCoefficients(&data);

	// TODO: Implement customisable configuration
  bme280_spi_write8(id, cs, BME280_REGISTER_CONTROL_HUM, BME280_OVERSAMP_16X);

  bme280_spi_write8(id, cs, BME280_REGISTER_CONFIG,
    BME280_STANDBY_TIME_20_MS << 5
    | BME280_FILTER_COEFF_16 << 2
  );

  bme280_spi_write8(id, cs, BME280_REGISTER_CONTROL,
    BME280_NORMAL_MODE
    | BME280_OVERSAMP_16X << 2
    | BME280_OVERSAMP_16X << 5
  );

	lua_pushlstring(L, (uint8_t *)&data, sizeof(data));

  return 1;
}

// Lua: = bme280spi.bar(data, [alt])
// Returns the pressure in 10ths of Pascals - 1010716 represents 101071.6 Pa
static int32_t bme280_spi_lua_baro(lua_State * L) {
	bme280_spi_data data;
	if (!bme280_spi_lua_getData(L, 1, &data)) {
		return luaL_error(L, "Configuration not valid");
	}

	// Altitude is an optional argument, if it's provided calculate QNH
	uint8_t calc_qnh = lua_isnumber(L, 2);

  uint8_t buf[6]; // Registers are P[3], T[3], H[2]
  bme280_spi_read(data.id, data.cs, BME280_REGISTER_PRESS, 6, buf);

	BME280_S32_t T, t_fine;
	uint32_t P;
	if (
		!bme280_spi_getTemp(&data, buf + 3,	&T, &t_fine) ||
		!bme280_spi_getBaro(&data, buf,			&P, &t_fine)
	) {
		return 0;
	}
	NODE_DBG("Temp:\t%d\t%d\n", T, t_fine);

	lua_pushinteger(L, P);

	if (calc_qnh) {
		int32_t h = luaL_checkinteger(L, 2);
		lua_pushinteger(L, bme280_spi_qfe2qnh(P, h));
		return 2;
	} else {
		return 1;
	}
}

static uint8_t bme280_spi_lua_getData(lua_State * L, int index, bme280_spi_data * data) {
	// Get the config data
	size_t len;
	const uint8_t * dataString = lua_tolstring(L, index, &len);

	if (len == sizeof(bme280_spi_data)) {
		memcpy(data, dataString, sizeof(bme280_spi_data));
		return 1;
	} else {
		return 0;
	}
}

// Lua: bme280spi.humi(data)
// Returns the relative humidity in 1000ths of % - 59087 represents 59.087 %
static int32_t bme280_spi_lua_humi(lua_State * L) {
	bme280_spi_data data;
	if (!bme280_spi_lua_getData(L, 1, &data)) {
		return luaL_error(L, "Configuration not valid");
	}

  uint8_t buf[5]; // Registers are P[3], T[3], H[2]
  bme280_spi_read(data.id, data.cs, BME280_REGISTER_TEMP, 5, buf);

	BME280_S32_t T, t_fine;
	if (!bme280_spi_getTemp(&data, buf, &T, &t_fine)) {
		return 0;
	}
	NODE_DBG("Temp:\t%d\t%d\n", T, t_fine);

	uint32_t H;
	if (!bme280_spi_getHumi(&data, buf + 3, &H, &t_fine)) {
		return 0;
	} else {
		lua_pushinteger(L, H);
		return 1;
	}
}

// Lua: bme280spi.read(data, [altitude])
// Returns: Temperature, Pressure, Humidity with the same precision as the individual temp, baro, humi methods
static int32_t bme280_spi_lua_read(lua_State * L) {
	bme280_spi_data data;
	if (!bme280_spi_lua_getData(L, 1, &data)) {
		return luaL_error(L, "Configuration not valid");
	}

	// Altitude is an optional argument, if it's provided calculate QNH
	uint8_t calc_qnh = lua_isnumber(L, 2);

  uint8_t buf[8]; // Registers are P[3], T[3], H[2]
  bme280_spi_read(data.id, data.cs, BME280_REGISTER_PRESS, 8, buf);

	BME280_S32_t T, t_fine;
	uint32_t P, H;

	// Temperature is required for other readings
	if (bme280_spi_getTemp(&data, buf + 3, &T, &t_fine)) {
		lua_pushinteger(L, T);
	} else {
		return 0;
	}

	if (bme280_spi_getBaro(&data, buf, &P, &t_fine)) {
		lua_pushinteger(L, P);
	} else {
		calc_qnh = 0;
		lua_pushnil(L);
	}

	if (bme280_spi_getHumi(&data, buf + 6,	&H, &t_fine)) {
		lua_pushinteger(L, H);
	} else {
		lua_pushnil(L);
	}

	if (calc_qnh) {
		int32_t h = luaL_checkinteger(L, 2);
		lua_pushinteger(L, bme280_spi_qfe2qnh(P, h));
		return 4;
	} else {
		return 3;
	}
}

// Lua: bme280spi.temp(data)
// Returns the temperature in 100ths of degrees Celsius - 5123 represents 51.23 degrees
static int32_t bme280_spi_lua_temp(lua_State * L) {
	bme280_spi_data data;
	if (!bme280_spi_lua_getData(L, 1, &data)) {
		return luaL_error(L, "Configuration not valid");
	}

  uint8_t buf[3]; // Registers are P[3], T[3], H[2]
  bme280_spi_read(data.id, data.cs, BME280_REGISTER_TEMP, 3, buf);

	BME280_S32_t T, t_fine;
	if (!bme280_spi_getTemp(&data, buf, &T, &t_fine)) {
		return 0;
	} else {
		lua_pushinteger(L, T);
		return 1;
	}
}

/****************************************************/
/**\name        Sensor helper methods */
/***************************************************/

static uint8_t bme280_spi_getBaro(bme280_spi_data * data, uint8_t * buf, uint32_t * P, BME280_S32_t * t_fine) {
	uint32_t adc_P = (uint32_t)(((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 4);
	NODE_DBG("adc_P:\t%08x\n", adc_P);

	if (adc_P == 0x80000 || adc_P == 0xfffff) {
		return 0;
	} else {
		*P = bme280_spi_compensate_P(adc_P, data, t_fine);
		return 1;
	}
}

static uint8_t bme280_spi_getHumi(bme280_spi_data * data, uint8_t * buf, uint32_t * H, BME280_S32_t * t_fine) {
	uint32_t adc_H = (uint32_t)((buf[0] << 8) | buf[1]);
	NODE_DBG("adc_H:\t%08x\n", adc_H);

	if (adc_H == 0x80000 || adc_H == 0xfffff) {
		return 0;
	} else {
		*H = bme280_spi_compensate_H(adc_H, data, t_fine);
		return 1;
	}
}

static uint8_t bme280_spi_getTemp(bme280_spi_data * data, uint8_t * buf, BME280_S32_t * T, BME280_S32_t * t_fine) {
  uint32_t adc_T = (uint32_t)(((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 4);
	NODE_DBG("adc_T:\t%08x\n", adc_T);

  if (adc_T == 0x80000 || adc_T == 0xfffff) {
    return 0;
	} else {
		*T = bme280_spi_compensate_T(adc_T, data, t_fine);
		return 1;
	}
}

// Returns the QNH value using the specified qfe value and height in metres
static uint32_t bme280_spi_qfe2qnh(uint32_t qfe, int32_t h) {
	double hc = pow((double)(1.0 - 2.25577e-5 * h), (double)(-5.25588));
	return (uint32_t)(((double)qfe * hc) + 0.5);
}


/****************************************************/
/**\name        Bosch compensation methods */
/***************************************************/
// Returns relative humidity, resolution .001%
// Output value of "59087" = 59.087%
static BME280_U32_t bme280_spi_compensate_H(BME280_S32_t adc_H, bme280_spi_data * data, BME280_S32_t * t_fine) {
  BME280_S32_t v_x1_u32r;

  v_x1_u32r = (*t_fine - ((BME280_S32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)data->dig_H4) << 20) - (((BME280_S32_t)data->dig_H5) * v_x1_u32r)) +
	  ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)data->dig_H6)) >> 10) * (((v_x1_u32r *
		((BME280_S32_t)data->dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
	  ((BME280_S32_t)data->dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)data->dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  v_x1_u32r = v_x1_u32r>>12;

  return (BME280_U32_t)((v_x1_u32r * 1000)>>10);
}

// Returns pressure Pa, resolution is 0.1 Pa
// Output value of “963862" = 96386.2 Pa = 963.862 hPa
static BME280_U32_t bme280_spi_compensate_P(BME280_S32_t adc_P, bme280_spi_data * data, BME280_S32_t * t_fine) {
  BME280_S64_t var1, var2, p;

  var1 = ((BME280_S64_t)*t_fine) - 128000;

  var2 = var1 * var1 * (BME280_S64_t)data->dig_P6;
  var2 = var2 + ((var1*(BME280_S64_t)data->dig_P5)<<17);
  var2 = var2 + (((BME280_S64_t)data->dig_P4)<<35);

  var1 = ((var1 * var1 * (BME280_S64_t)data->dig_P3)>>8) + ((var1 * (BME280_S64_t)data->dig_P2)<<12);
  var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)data->dig_P1)>>33;

  if (var1 == 0) {
	  return 0; // avoid exception caused by division by zero
  }

  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;

  var1 = (((BME280_S64_t)data->dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((BME280_S64_t)data->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)data->dig_P7)<<4);
  p = (p * 10) >> 8;

  return (BME280_U32_t)p;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
BME280_S32_t bme280_spi_compensate_T(BME280_S32_t adc_T, bme280_spi_data * data, BME280_S32_t * t_fine) {
	BME280_S32_t var1, var2, T;

	var1 = ((((adc_T>>3) - ((BME280_S32_t)data->dig_T1<<1))) * ((BME280_S32_t)data->dig_T2)) >> 11;

	var2 = (((((adc_T>>4) - ((BME280_S32_t)data->dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)data->dig_T1))) >> 12) *
		((BME280_S32_t)data->dig_T3)) >> 14;

	T = ((var1 + var2) * 5 + 128) >> 8;

	if (t_fine) {
		*t_fine = var1 + var2;
	}

	return T;
}

/****************************************************/
/**\name        Module methods */
/***************************************************/

static void bme280_spi_delay(uint16_t delay) {
  os_delay_us(delay);
  system_soft_wdt_feed();
}

static uint8_t bme280_spi_isReadingCalibration(uint8_t id, uint8_t cs) {
  return bme280_spi_read8(id, cs, BME280_REGISTER_STATUS) & 0x01;
}

static void bme280_spi_readCoefficients(bme280_spi_data * data) {
	uint8_t buf[18]; // Enough to hold the Pressure data
	uint8_t * reg;

	// Temperature coefficients
	reg = buf;
	bme280_spi_read(data->id, data->cs, BME280_REGISTER_DIG_T, 6, buf);
	data->dig_T1 = r16uLE_buf(reg); reg += 2;
	data->dig_T2 = r16sLE_buf(reg); reg += 2;
	data->dig_T3 = r16sLE_buf(reg);
	NODE_DBG("dig_T: %d\t%d\t%d\n", data->dig_T1, data->dig_T2, data->dig_T3);

	// Pressure coefficients
	reg = buf;
	bme280_spi_read(data->id, data->cs, BME280_REGISTER_DIG_P, 18, buf);
	data->dig_P1 = r16uLE_buf(reg); reg += 2;
	data->dig_P2 = r16sLE_buf(reg); reg += 2;
	data->dig_P3 = r16sLE_buf(reg); reg += 2;
	data->dig_P4 = r16sLE_buf(reg); reg += 2;
	data->dig_P5 = r16sLE_buf(reg); reg += 2;
	data->dig_P6 = r16sLE_buf(reg); reg += 2;
	data->dig_P7 = r16sLE_buf(reg); reg += 2;
	data->dig_P8 = r16sLE_buf(reg); reg += 2;
	data->dig_P9 = r16sLE_buf(reg);
	NODE_DBG("dig_P: %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", data->dig_P1, data->dig_P2, data->dig_P3, data->dig_P4, data->dig_P5, data->dig_P6, data->dig_P7, data->dig_P8, data->dig_P9);

  // Humidity coefficients
	data->dig_H1 = bme280_spi_read8(data->id, data->cs, BME280_REGISTER_DIG_H1);
  reg = buf;
	bme280_spi_read(data->id, data->cs, BME280_REGISTER_DIG_H2, 7, buf);
	data->dig_H2 = r16sLE_buf(reg); reg += 2;
  data->dig_H3 = reg[0]; reg += 1;
  data->dig_H4 = (int16_t)reg[0] << 4 | (reg[1] & 0x0F); reg += 1; // H4[11:4 3:0] = 0xE4[7:0] 0xE5[3:0] 12-bit signed
  data->dig_H5 = (int16_t)reg[1] << 4 | (reg[0]   >> 4); reg += 2; // H5[11:4 3:0] = 0xE6[7:0] 0xE5[7:4] 12-bit signed
  data->dig_H6 = (int8_t)reg[0];
	NODE_DBG("dig_H: %d\t%d\t%d\t%d\t%d\t%d\n", data->dig_H1, data->dig_H2, data->dig_H3, data->dig_H4, data->dig_H5, data->dig_H6);
}

static void bme280_spi_read(uint8_t id, uint8_t cs, uint8_t reg, uint8_t len, uint8_t * resp) {
  platform_gpio_write(cs, LOW);

  // When reading, the 7th bit must be high
  platform_spi_send(id, BME280_SPI_DATA_BITS, reg | 0x80);

	for (uint8_t i = 0; i < len; i++) {
		resp[i] = platform_spi_send_recv(id, BME280_SPI_DATA_BITS, 0x00) & 0xff;
	}

	platform_gpio_write(cs, HIGH);
}

static uint8_t bme280_spi_read8(uint8_t id, uint8_t cs, uint8_t reg) {
  uint8_t resp = 0;
  bme280_spi_read(id, cs, reg, 1, &resp);

  return resp;
}

static uint16_t bme280_spi_read16(uint8_t id, uint8_t cs, uint8_t reg) {
  uint8_t resp[2];
  bme280_spi_read(id, cs, reg, 2, resp);

  return (uint16_t)resp[0] << 8 | resp[1];
}

static uint16_t bme280_spi_read16_LE(uint8_t id, uint8_t cs, uint8_t reg) {
  uint16_t temp = bme280_spi_read16(id, cs, reg);
  return (temp >> 8) | (temp < 8);
}

static uint32_t bme280_spi_read24(uint8_t id, uint8_t cs, uint8_t reg) {
  uint8_t resp[3];
  bme280_spi_read(id, cs, reg, 3, resp);

  return (uint32_t)resp[0] << 16 | (uint16_t)resp[1] << 8 | resp[2];
}

static uint8_t bme280_spi_write8(uint8_t id, uint8_t cs, uint8_t reg, uint8_t data) {
  platform_gpio_write(cs, LOW);

  // When reading, the 7th bit must be low
  platform_spi_send(id, BME280_SPI_DATA_BITS, reg & ~0x80);
  platform_spi_send(id, BME280_SPI_DATA_BITS, data);

  spi_data_type resp = platform_spi_send_recv(id, BME280_SPI_DATA_BITS, 0xff);

  platform_gpio_write(cs, HIGH);

  return (uint8_t)(resp & 0xff);
}

// Module function map
static const LUA_REG_TYPE bme280spi_map[] = {
  {LSTRKEY("setup"),  LFUNCVAL(bme280_spi_lua_setup)},
  {LSTRKEY("baro"),   LFUNCVAL(bme280_spi_lua_baro)},
  {LSTRKEY("humi"),   LFUNCVAL(bme280_spi_lua_humi)},
  {LSTRKEY("read"),   LFUNCVAL(bme280_spi_lua_read)},
  {LSTRKEY("temp"),   LFUNCVAL(bme280_spi_lua_temp)},
  {LNILKEY, LNILVAL}
};

NODEMCU_MODULE(BME280SPI, "bme280spi", bme280spi_map, NULL);
