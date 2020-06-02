#include "apds9960.h"

#include <assert.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/param.h>	//for MIN
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/limits.h>

#define APDS9960_I2C_ADDR		0x39
#define APDS9960_ID				0xAB

#define APDS9960_REG_ENABLE		0x80
#define APDS9960_REG_ATIME		0x81
#define APDS9960_REG_CONTROL		0x8F
#define APDS9960_REG_CONFIG2		0x90
#define APDS9960_REG_ID			0x92
#define APDS9960_REG_STATUS		0x93
#define APDS9960_REG_CDATAL		0x94
#define APDS9960_REG_CDATAH		0x95
#define APDS9960_REG_RDATAL		0x96
#define APDS9960_REG_RDATAH		0x97
#define APDS9960_REG_GDATAL		0x98
#define APDS9960_REG_GDATAH		0x99
#define APDS9960_REG_BDATAL		0x9A
#define APDS9960_REG_BDATAH		0x9B
#define APDS9960_REG_PDATA		0x9C
#define APDS9960_REG_GPENTH		0xA0
#define APDS9960_REG_GEXTH		0xA1
#define APDS9960_REG_GCONF1		0xA2
#define APDS9960_REG_GCONF2		0xA3
#define APDS9960_REG_GOFFSET_U	0xA4
#define APDS9960_REG_GOFFSET_D	0xA5
#define APDS9960_REG_GPULSE		0xA6
#define APDS9960_REG_GOFFSET_L	0xA7
#define APDS9960_REG_GOFFSET_R	0xA9
#define APDS9960_REG_GCONF3		0xAA
#define APDS9960_REG_GCONF4		0xAB
#define APDS9960_REG_GFLVL		0xAE
#define APDS9960_REG_GSTATUS		0xAF
#define APDS9960_REG_GFIFO_U		0xFC

#define APDS9960_CMD_AICLEAR		0xE7

typedef struct _apds9960_ctx
{
	int i2c_dev_fd;

	int up_edges;
	int down_edges;
	int left_edges;
	int right_edges;
} apds9960_ctx;

static FILE* g_apds9960_log_file = NULL;

static void apds9960_log(const char* format, ...)
{
	static bool logging_setup = false;
	if (!logging_setup)
	{
		const char* log_path = getenv("APDS9960_LOG");
		if(log_path)
			g_apds9960_log_file = fopen(log_path, "w");
		logging_setup = true;
	}
	if (g_apds9960_log_file)
	{
		va_list args;
		va_start(args, format);
		fprintf(g_apds9960_log_file, "[LIBAPDS9960] ");
		vfprintf(g_apds9960_log_file, format, args);
		va_end(args);
	}
}

static bool apds9960_read_reg8(int dev_fd, uint8_t reg, uint8_t* value)
{
	int ret;
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data transaction;

	msgs[0].addr = APDS9960_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = APDS9960_I2C_ADDR;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = value;

	transaction.msgs = msgs;
	transaction.nmsgs = 2;

	ret = ioctl(dev_fd, I2C_RDWR, &transaction);
	if (ret < 0) {
		apds9960_log("Failed to read i2c register %d: %d\n", reg, errno);
		return false;
	}
	return true;
}

static bool apds9960_read_reg16(int dev_fd, uint8_t reg, uint16_t* value)
{
	int ret;
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data transaction;
	uint8_t buf[2];

	msgs[0].addr = APDS9960_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = APDS9960_I2C_ADDR;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 2;
	msgs[1].buf = buf;

	transaction.msgs = msgs;
	transaction.nmsgs = 2;

	ret = ioctl(dev_fd, I2C_RDWR, &transaction);
	if (ret < 0) {
		apds9960_log("Failed to read i2c register %d: %d\n", reg, errno);
		return false;
	}

	*value = (uint16_t)( (uint16_t)( (uint16_t)buf[1] << 8) | (uint16_t)buf[0] );

	return true;
}

static bool apds9960_read_buffer(int dev_fd, uint8_t reg, uint8_t* buffer, uint16_t buffer_len)
{
	int ret;
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data transaction;

	msgs[0].addr = APDS9960_I2C_ADDR;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = APDS9960_I2C_ADDR;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = buffer_len;
	msgs[1].buf = buffer;

	transaction.msgs = msgs;
	transaction.nmsgs = 2;

	ret = ioctl(dev_fd, I2C_RDWR, &transaction);
	if (ret < 0) {
		apds9960_log("Failed to read i2c register %d buffer (addr %p len %u): %d\n", reg, buffer, buffer_len, errno);
		return false;
	}

	return true;
}

static bool apds9960_write_reg(int dev_fd, const uint8_t reg, const uint8_t value)
{
	ssize_t ret;
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;

	ret = write(dev_fd, buf, 2);
	if (ret != 2)
	{
		apds9960_log("Failed to write i2c reg %d: %d\n", reg, errno);
		return false;
	}

	return true;
}

static bool apds9960_write_cmd(int dev_fd, const uint8_t cmd)
{
	ssize_t ret = write(dev_fd, &cmd, 1);
	if (ret != 1)
	{
		apds9960_log("Failed to write i2c command %d: %d\n", cmd, errno);
		return false;
	}

	return true;
}

static bool apds9960_set_bit(int dev_fd, uint8_t reg, uint8_t bit, bool value)
{
	uint8_t tmp_val;
	bool ret = apds9960_read_reg8(dev_fd, reg, &tmp_val);
	if (!ret) return ret;

	if (value)
		tmp_val |= (uint8_t)(value << bit);
	else
		tmp_val &= (uint8_t)~(value << bit);

	return apds9960_write_reg(dev_fd, reg, tmp_val);
}

static bool apds9960_get_bit(int dev_fd, uint8_t reg, uint8_t bit, bool* value)
{
	uint8_t tmp_val;
	bool ret = apds9960_read_reg8(dev_fd, reg, &tmp_val);
	if (!ret) return ret;

	*value = (tmp_val >> bit) & 0x1;
	return true;
}

static bool apds9960_set_bits(int dev_fd, uint8_t reg, uint8_t lowest_bit, uint8_t num_bits, uint8_t value)
{
	uint8_t tmp_val;
	bool ret;
	
	if (value >= (uint8_t)(1 << num_bits))
	{
		apds9960_log("Value is larger than expected number of bits. Value: %d Bits: %d\n", value, num_bits);
		return false;
	}

	ret = apds9960_read_reg8(dev_fd, reg, &tmp_val);
	if (!ret) return ret;

	uint8_t mask = (uint8_t)(((uint8_t)(1 << num_bits) - 1) << lowest_bit);
	tmp_val &= (uint8_t)~mask;
	tmp_val |= (uint8_t)(value << lowest_bit);
	return apds9960_write_reg(dev_fd, reg, tmp_val);
}

static bool apds9960_get_bits(int dev_fd, uint8_t reg, uint8_t lowest_bit, uint8_t num_bits, uint8_t* value)
{
	uint8_t tmp_val;
	bool ret = apds9960_read_reg8(dev_fd, reg, &tmp_val);
	if (!ret) return ret;

	uint8_t mask = (uint8_t)(((uint8_t)(1 << num_bits) - 1) << lowest_bit);
	*value = (uint8_t)((tmp_val & mask) >> lowest_bit);
	return true;
}

static void apds9960_dump_reg(int dev_fd, uint8_t reg)
{
	uint8_t val;
	if (apds9960_read_reg8(dev_fd, reg, &val))
		apds9960_log("Reg %#X: %#X\n", reg, val);
}

void apds9960_set_log_file(FILE* log_file)
{
	g_apds9960_log_file = log_file;
}

int apds9960_new(apds9960_ctxp* new_ctx, int dev_id)
{
	char device_path[PATH_MAX];
	int dev_fd;
	int ret;

	*new_ctx = (apds9960_ctxp)malloc(sizeof(apds9960_ctx));
	if (!new_ctx)
	{
		apds9960_log("Failed to allocate memory for context\n");
		return LIBAPDS9960_ENOMEM;
	}
	memset(*new_ctx, 0, sizeof(apds9960_ctx));

	snprintf(device_path, PATH_MAX, "/dev/i2c-%d", dev_id);
	dev_fd = open(device_path, O_RDWR);
	if (dev_fd < 0)
	{
		apds9960_log("Failed to open device (%s) errno: %d\n", device_path, errno);
		free(*new_ctx);
		return LIBAPDS9960_EOPEN;
	}
	(*new_ctx)->i2c_dev_fd = dev_fd;

	if (ioctl(dev_fd, I2C_SLAVE, APDS9960_I2C_ADDR))
	{
		apds9960_log("Failed to set I2C to slave: %d\n", errno);
		ret = LIBAPDS9960_EOPEN;
		goto err_cleanup;
	}

	{
		uint8_t id;
		if (!apds9960_read_reg8(dev_fd, APDS9960_REG_ID, &id))
		{
			ret = LIBAPDS9960_EINTERNAL;
			goto err_cleanup;
		}

		if (id != APDS9960_ID)
		{
			ret = LIBAPDS9960_EINVALIDID;
			goto err_cleanup;
		}
	}

	if (!apds9960_set_gesture_enabled(*new_ctx, false) |
		!apds9960_set_proximity_enabled(*new_ctx, false) |
		!apds9960_set_color_enabled(*new_ctx, false) |
		!apds9960_set_proximity_interrupt_enabled(*new_ctx, false))
	{
		ret = LIBAPDS9960_EINITCHIP;
		goto err_cleanup;
	}

	if (!apds9960_clear_interrupts(*new_ctx))
	{
		ret = LIBAPDS9960_EINITCHIP;
		goto err_cleanup;
	}

	//if (!apds9960_set_chip_enabled(*new_ctx, false))
	if (!apds9960_write_reg(dev_fd, APDS9960_REG_ENABLE, 0))
	{
		ret = LIBAPDS9960_EINITCHIP;
		goto err_cleanup;
	}

	struct timespec sleep_time = {0, 10000000};
	while (nanosleep(&sleep_time, &sleep_time))
	{
		if (errno != EINTR)
		{
			apds9960_log("Failed to sleep: %d\n", errno);
			ret = LIBAPDS9960_EINTERNAL;
			goto err_cleanup;
		}
	}

	if (!apds9960_set_chip_enabled(*new_ctx, true))
		return LIBAPDS9960_EINITCHIP;

	sleep_time.tv_sec = 0;
	sleep_time.tv_nsec = 10000000;
	while (nanosleep(&sleep_time, &sleep_time))
	{
		if (errno != EINTR)
		{
			apds9960_log("Failed to sleep: %d\n", errno);
			ret = LIBAPDS9960_EINTERNAL;
			goto err_cleanup;
		}
	}

	apds9960_dump_reg(dev_fd, APDS9960_REG_ENABLE);

	if (!apds9960_set_color_gain(*new_ctx, 0x1) |
		!apds9960_set_adc_integration_time(*new_ctx, 0x1) |
		!apds9960_set_gesture_dimensions(*new_ctx, 0x0) |
		!apds9960_set_gesture_fifo_threshold(*new_ctx, 0x1) |
		!apds9960_set_gesture_gain(*new_ctx, 0x1) |
		!apds9960_set_gesture_proximity_threshold(*new_ctx, 50) |
		// gesture pulse length = 0x2 pulse count = 0x3
		!apds9960_write_reg(dev_fd, APDS9960_REG_GPULSE, (0x2 << 6) | 0x3))
	{
		ret = LIBAPDS9960_EINITCHIP;
		goto err_cleanup;
	}

	return LIBAPDS9960_OK;

err_cleanup:
	close(dev_fd);
	free(*new_ctx);
	return ret;
}

void apds9960_free(apds9960_ctxp ctx)
{
	close(ctx->i2c_dev_fd);
	free(ctx);
}

bool apds9960_set_chip_enabled(apds9960_ctxp ctx, bool enabled)
{
	return apds9960_set_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 0, enabled);
}

bool apds9960_get_chip_enabled(apds9960_ctxp ctx, bool* enabled)
{
	return apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 0, enabled);
}

bool apds9960_set_gesture_enabled(apds9960_ctxp ctx, bool enabled)
{
	return apds9960_set_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 6, enabled);
}

bool apds9960_get_gesture_enabled(apds9960_ctxp ctx, bool* enabled)
{
	return apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 6, enabled);
}

bool apds9960_set_proximity_enabled(apds9960_ctxp ctx, bool enabled)
{
	return apds9960_set_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 2, enabled);
}

bool apds9960_get_proximity_enabled(apds9960_ctxp ctx, bool* enabled)
{
	uint8_t value;
	if(apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, &value))
		printf("Enabled reg: %#X\n", value);
	return apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 2, enabled);
}

bool apds9960_set_color_enabled(apds9960_ctxp ctx, bool enabled)
{
	return apds9960_set_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 1, enabled);
}

bool apds9960_get_color_enabled(apds9960_ctxp ctx, bool * enabled)
{
	return apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 1, enabled);
}

bool apds9960_set_proximity_interrupt_enabled(apds9960_ctxp ctx, bool enabled)
{
	return apds9960_set_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 5, enabled);
}

bool apds9960_get_proximity_interrupt_enabled(apds9960_ctxp ctx, bool * enabled)
{
	return apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_ENABLE, 5, enabled);
}

bool apds9960_set_color_gain(apds9960_ctxp ctx, uint8_t gain)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_CONTROL, 0, 2, gain);
}

bool apds9960_get_color_gain(apds9960_ctxp ctx, uint8_t* gain)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_CONTROL, 0, 2, gain);
}

bool apds9960_set_proximity_gain(apds9960_ctxp ctx, uint8_t gain)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_CONTROL, 2, 2, gain);
}

bool apds9960_get_proximity_gain(apds9960_ctxp ctx, uint8_t * gain)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_CONTROL, 2, 2, gain);
}

bool apds9960_set_gesture_gain(apds9960_ctxp ctx, uint8_t gain)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_GCONF2, 5, 2, gain);
}

bool apds9960_get_gesture_gain(apds9960_ctxp ctx, uint8_t * gain)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_GCONF2, 5, 2, gain);
}

bool apds9960_set_gesture_fifo_threshold(apds9960_ctxp ctx, uint8_t threshold)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_GCONF1, 6, 2, threshold);
}

bool apds9960_get_gesture_fifo_threshold(apds9960_ctxp ctx, uint8_t* threshold)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_GCONF1, 6, 2, threshold);
}

bool apds9960_set_gesture_proximity_threshold(apds9960_ctxp ctx, uint8_t threshold)
{
	return apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GPENTH, threshold);
}

bool apds9960_get_gesture_proximity_threshold(apds9960_ctxp ctx, uint8_t * threshold)
{
	return apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GPENTH, threshold);
}

bool apds9960_set_gesture_exit_threshold(apds9960_ctxp ctx, uint8_t threshold)
{
	return apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GEXTH, threshold);
}

bool apds9960_get_gesture_exit_threshold(apds9960_ctxp ctx, uint8_t * threshold)
{
	return apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GEXTH, threshold);
}

bool apds9960_set_gesture_dimensions(apds9960_ctxp ctx, uint8_t dimensions)
{
	return apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GCONF3, dimensions & 0x3);
}

bool apds9960_get_gesture_dimensions(apds9960_ctxp ctx, uint8_t * dimensions)
{
	return apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GCONF3, dimensions);
}

bool apds9960_set_gesture_offsets(apds9960_ctxp ctx, int8_t off_u, int8_t off_d, int8_t off_l, int8_t off_r)
{
	if (!apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_U, (uint8_t)off_u))
		return false;

	if(!apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_D, (uint8_t)off_d))
		return false;

	if(!apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_L, (uint8_t)off_l))
		return false;

	if (!apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_R, (uint8_t)off_r))
		return false;

	return true;
}

bool apds9960_get_gesture_offsets(apds9960_ctxp ctx, int8_t* off_u, int8_t* off_d, int8_t* off_l, int8_t* off_r)
{
	if (!apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_U, (uint8_t*)off_u))
		return false;

	if (!apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_D, (uint8_t*)off_d))
		return false;

	if (!apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_L, (uint8_t*)off_l))
		return false;

	if (!apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GOFFSET_R, (uint8_t*)off_r))
		return false;

	return true;
}

bool apds9960_set_adc_integration_time(apds9960_ctxp ctx, uint8_t value)
{
	return apds9960_write_reg(ctx->i2c_dev_fd, APDS9960_REG_ATIME, value);
}

bool apds9960_get_adc_integration_time(apds9960_ctxp ctx, uint8_t * value)
{
	return apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_ATIME, value);
}

bool apds9960_set_proximity_led_strength(apds9960_ctxp ctx, uint8_t value)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_CONTROL, 6, 2, value);
}

bool apds9960_get_proximity_led_strength(apds9960_ctxp ctx, uint8_t* value)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_CONTROL, 6, 2, value);
}

bool apds9960_set_gesture_led_strength(apds9960_ctxp ctx, uint8_t value)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_GCONF2, 3, 2, value);
}

bool apds9960_get_gesture_led_strength(apds9960_ctxp ctx, uint8_t * value)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_GCONF2, 3, 2, value);
}

bool apds9960_set_led_boost(apds9960_ctxp ctx, uint8_t value)
{
	return apds9960_set_bits(ctx->i2c_dev_fd, APDS9960_REG_CONFIG2, 4, 2, value);
}

bool apds9960_get_led_boost(apds9960_ctxp ctx, uint8_t * value)
{
	return apds9960_get_bits(ctx->i2c_dev_fd, APDS9960_REG_CONFIG2, 4, 2, value);
}

bool apds9960_set_gmode(apds9960_ctxp ctx, bool enabled)
{
	return apds9960_set_bit(ctx->i2c_dev_fd, APDS9960_REG_GCONF4, 0, enabled);
}

bool apds9960_get_gmode(apds9960_ctxp ctx, bool * enabled)
{
	return apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_GCONF4, 0, enabled);
}

bool apds9960_clear_interrupts(apds9960_ctxp ctx)
{
	return apds9960_write_cmd(ctx->i2c_dev_fd, APDS9960_CMD_AICLEAR);
}

bool apds9960_read_clear_light_level(apds9960_ctxp ctx, uint16_t* level)
{
	bool avalid;
	if (!apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_STATUS, 0, &avalid))
		return false;

	if (!avalid)
		return false;

	return apds9960_read_reg16(ctx->i2c_dev_fd, APDS9960_REG_CDATAL, level);
}

bool apds9960_read_proximity(apds9960_ctxp ctx, uint8_t* proximity)
{
	bool pvalid;
	bool pgsat;
	
	if (!apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_STATUS, 1, &pvalid))
		return false;

	if (!pvalid)
	{
		if (!apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_STATUS, 6, &pgsat))
			return false;

		if (pgsat)
		{
			printf("proximity saturated\n");
			apds9960_clear_interrupts(ctx);
		}

		printf("no proximity\n");
		return false;
	}

	return apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_PDATA, proximity);
}

bool apds9960_read_gesture_fifo(apds9960_ctxp ctx, apds9960_gesture_point* point_buffer, uint8_t point_buffer_size, uint8_t* num_points_recv)
{
	bool gvalid;
	uint8_t nrecs;
	uint16_t bytes_to_get;

	if (!apds9960_get_bit(ctx->i2c_dev_fd, APDS9960_REG_GSTATUS, 0, &gvalid))
		return false;

	if (!gvalid)
		return false;

	if (!apds9960_read_reg8(ctx->i2c_dev_fd, APDS9960_REG_GFLVL, &nrecs))
		return false;

	if (nrecs == 0)
		return false;

	//nrecs max is 32, so max we might read is 128 bytes
	*num_points_recv = MIN(point_buffer_size, nrecs);
	bytes_to_get = (uint16_t)(*num_points_recv * 4);

	if (!apds9960_read_buffer(ctx->i2c_dev_fd, APDS9960_REG_GFIFO_U, (uint8_t*)point_buffer, bytes_to_get))
		return false;

	return true;
}

static void apds9960_clear_gesture_history(apds9960_ctxp ctx)
{
	ctx->up_edges = 0;
	ctx->down_edges = 0;
	ctx->left_edges = 0;
	ctx->right_edges = 0;

	apds9960_set_gmode(ctx, false);
}

#define APDS9960_DIFF_THRESHOLD 13

apds9960_gesture apds9960_guess_gesture(apds9960_ctxp ctx)
{
	apds9960_gesture_point points[APDS9960_MAX_GESTURE_POINTS];
	uint8_t num_points;

	if (!apds9960_read_gesture_fifo(ctx, points, APDS9960_MAX_GESTURE_POINTS, &num_points))
	{
		apds9960_clear_gesture_history(ctx);
		return kGestureNone;
	}

	for (int i = 0; i < num_points; ++i)
	{
		int v = points[i].up - points[i].down;
		int h = points[i].left - points[i].right;

		if (v > APDS9960_DIFF_THRESHOLD)
		{
			if (ctx->down_edges > 0)
			{
				apds9960_clear_gesture_history(ctx);
				return kGestureSwipeDown;
			}
			ctx->up_edges++;
		}

		if (v < -APDS9960_DIFF_THRESHOLD)
		{
			if (ctx->up_edges > 0)
			{
				apds9960_clear_gesture_history(ctx);
				return kGestureSwipeUp;
			}
			ctx->down_edges++;
		}

		if (h > APDS9960_DIFF_THRESHOLD)
		{
			if (ctx->right_edges > 0)
			{
				apds9960_clear_gesture_history(ctx);
				return kGestureSwipeRight;
			}
			ctx->left_edges++;
		}

		if (h < -APDS9960_DIFF_THRESHOLD)
		{
			if (ctx->left_edges > 0)
			{
				apds9960_clear_gesture_history(ctx);
				return kGestureSwipeLeft;
			}
			ctx->right_edges++;
		}
	}

	return kGestureNone;
}

