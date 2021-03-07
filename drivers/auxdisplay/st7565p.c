/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/st7565.h>

#include <linux/uaccess.h>

#define ST7565R_REG_CMD 0x00
#define ST7565R_REG_DAT 0x01

//! \name Command defines
//@{
#define ST7565R_CMD_DISPLAY_ON                     0xAF
#define ST7565R_CMD_DISPLAY_OFF                    0xAE
#define ST7565R_CMD_START_LINE_SET(line)           (0x40 | (line))
#define ST7565R_CMD_PAGE_ADDRESS_SET(page)         (0xB0 | (page))
#define ST7565R_CMD_COLUMN_ADDRESS_SET_MSB(column) (0x10 | (column))
#define ST7565R_CMD_COLUMN_ADDRESS_SET_LSB(column) (0x00 | (column))
#define ST7565R_CMD_ADC_NORMAL                     0xA0
#define ST7565R_CMD_ADC_REVERSE                    0xA1
#define ST7565R_CMD_DISPLAY_NORMAL                 0xA6
#define ST7565R_CMD_DISPLAY_REVERSE                0xA7
#define ST7565R_CMD_DISPLAY_ALL_POINTS_OFF         0xA4
#define ST7565R_CMD_DISPLAY_ALL_POINTS_ON          0xA5
#define ST7565R_CMD_LCD_BIAS_1_DIV_5_DUTY33        0xA1
#define ST7565R_CMD_LCD_BIAS_1_DIV_6_DUTY33        0xA2
#define ST7565R_CMD_NORMAL_SCAN_DIRECTION          0xC0
#define ST7565R_CMD_REVERSE_SCAN_DIRECTION         0xC8
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_0       0x20
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_1       0x21
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_2       0x22
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_3       0x23
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_4       0x24
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_5       0x25
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_6       0x26
#define ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_7       0x27
#define ST7565R_CMD_POWER_CTRL_ALL_ON              0x2F
#define ST7565R_CMD_SLEEP_MODE                     0xAC
#define ST7565R_CMD_NORMAL_MODE                    0xAD
#define ST7565R_CMD_RESET                          0xE2
#define ST7565R_CMD_NOP                            0xE3
#define ST7565R_CMD_ELECTRONIC_VOLUME_MODE_SET     0x81
#define ST7565R_CMD_ELECTRONIC_VOLUME(volume)      (0x3F & (~volume))
#define ST7565R_CMD_BOOSTER_RATIO_SET              0xF8
#define ST7565R_CMD_BOOSTER_RATIO_2X_3X_4X         0x00
#define ST7565R_CMD_BOOSTER_RATIO_5X               0x01
#define ST7565R_CMD_BOOSTER_RATIO_6X               0x03
#define ST7565R_CMD_STATUS_READ                    0x00
#define ST7565R_CMD_END                            0xEE
#define ST7565R_CMD_READ_MODIFY_WRITE              0xE0
//@}

// Settings
#define CONFIG_ST7565R_FRAMEBUFFER 1 // serial interface has to have a local buffer
#define ST7565R_DISPLAY_CONTRAST_MAX 40
#define ST7565R_DISPLAY_CONTRAST_MIN 30
#define ST7565R_DISPLAY_LCD_WIDTH    128
#define ST7565R_DISPLAY_LCD_HEIGHT   64
#define ST7565R_DISPLAY_LCD_PIXELS_PER_BYTE  8
#define ST7565R_DISPLAY_LCD_PAGES (ST7565R_DISPLAY_LCD_HEIGHT / ST7565R_DISPLAY_LCD_PIXELS_PER_BYTE)
#define ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE ((ST7565R_DISPLAY_LCD_WIDTH * ST7565R_DISPLAY_LCD_HEIGHT) / ST7565R_DISPLAY_LCD_PIXELS_PER_BYTE)

#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80
#define SPI_TX_DUAL		0x100
#define SPI_TX_QUAD		0x200
#define SPI_RX_DUAL		0x400
#define SPI_RX_QUAD		0x800


#define N_SPI_MINORS			32	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
				| SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct st7565_desc {
	dev_t devt;
	spinlock_t lock;
	struct spi_device *spi;
	struct list_head device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8 contrast;

	//gpios
	int reset_gpio;
	int addr_gpio;

	// frame buffer
	u8 *frame_buffer;
} st7565_desc_t;

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);


/*-------------------------------------------------------------------------*/
/* Local Functions */
static void st7565p_hard_reset(struct st7565_desc *dev)
{

	gpio_set_value(dev->reset_gpio, 0);
	mdelay(1);
	gpio_set_value(dev->reset_gpio, 1);
	mdelay(1);
}

static int st7565_setup_spi(struct st7565_desc *dev)
{
	struct spi_device *spi = dev->spi;
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 100000;
	return spi_setup(spi);
}

static void st7565p_complete(void *arg)
{
	complete(arg);
}

static ssize_t st7565p_sync(struct st7565_desc *dev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = st7565p_complete;
	message->context = &done;

	spin_lock_irq(&dev->lock);
	if (dev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(dev->spi, message);
	spin_unlock_irq(&dev->lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static ssize_t st7565p_write_command(struct st7565_desc *dev, u8 command)
{
	struct spi_message	m;
	struct spi_transfer	t = {
				.tx_buf		= &command,
				.len		= 1,
			};

	// clear A0 pin
	gpio_set_value(dev->addr_gpio, 0);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return st7565p_sync(dev, &m);
}

static ssize_t st7565p_write_data(struct st7565_desc *dev, u8 *data, size_t length )
{
	struct spi_message	m;
	struct spi_transfer	t = {
				.tx_buf		= data,
				.len		= length,
			};

	// set A0 pin
	gpio_set_value(dev->addr_gpio, 1);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return st7565p_sync(dev, &m);
}


/*-------------------------------------------------------------------------*/
/* LCD specific functions */
static u8 st7565p_set_contrast( struct st7565_desc *dev, u8 contrast )
{
	if (contrast < ST7565R_DISPLAY_CONTRAST_MIN) {
		contrast = ST7565R_DISPLAY_CONTRAST_MIN;
	}
	if (contrast > ST7565R_DISPLAY_CONTRAST_MAX) {
		contrast = ST7565R_DISPLAY_CONTRAST_MAX;
	}
	st7565p_write_command(dev, ST7565R_CMD_ELECTRONIC_VOLUME_MODE_SET);
	st7565p_write_command(dev, ST7565R_CMD_ELECTRONIC_VOLUME(contrast));
	return contrast;
}

static void st7565p_set_page_address( struct st7565_desc *dev, u8 address )
{
	// Make sure that the address is 4 bits (only 8 pages)
	address &= 0x0F;
	st7565p_write_command(dev, ST7565R_CMD_PAGE_ADDRESS_SET(address));
}

static void st7565p_set_column_address( struct st7565_desc *dev, u8 address )
{
	// Make sure the address is 7 bit
	address &= 0x7F;
	// reverse ADC comp
	address += 4;
	st7565p_write_command(dev, ST7565R_CMD_COLUMN_ADDRESS_SET_MSB(address >> 4));
	st7565p_write_command(dev, ST7565R_CMD_COLUMN_ADDRESS_SET_LSB(address & 0x0F));
}

static void st7565p_set_display_start_line_address(struct st7565_desc *dev, u8 address)
{
	// Make sure address is 6 bits
	address &= 0x3F;
	st7565p_write_command(dev, ST7565R_CMD_START_LINE_SET(address));
}

static void st7565p_framebuffer_clear( struct st7565_desc *dev  )
{
	memset(dev->frame_buffer, 0x00, ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE);
}

static ssize_t st7565p_framebuffer_sync( struct st7565_desc *dev )
{
	u8 page;
	ssize_t status = 0;

	for (page = 0; (page < ST7565R_DISPLAY_LCD_PAGES) && (status >= 0); page++) {
		st7565p_set_page_address(dev, page);
		st7565p_set_column_address(dev, 0);
		status = st7565p_write_data(dev, &dev->frame_buffer[page * ST7565R_DISPLAY_LCD_WIDTH], ST7565R_DISPLAY_LCD_WIDTH);
	}

	return status;
}

static ssize_t st7565p_framebuffer_sync_partial( struct st7565_desc *dev, int pos, size_t count )
{
	u8 page = pos / ST7565R_DISPLAY_LCD_WIDTH;
	u8 col = pos % ST7565R_DISPLAY_LCD_WIDTH;
	size_t len;
	ssize_t status = 0;

	for(; (page < ST7565R_DISPLAY_LCD_PAGES) && (status >= 0) && (count > 0); page++) {
		len = count;
		if((len + col) > ST7565R_DISPLAY_LCD_WIDTH) {
			len = (ST7565R_DISPLAY_LCD_WIDTH - col);
		}
		st7565p_set_page_address(dev, page);
		st7565p_set_column_address(dev, col);
		status = st7565p_write_data(dev, &dev->frame_buffer[page * ST7565R_DISPLAY_LCD_WIDTH + col], len);
		count -= len;
		col = 0;
	}

	if(status < 0) {
		dev_err(&dev->spi->dev, "st7565p_framebuffer_sync_partial: %ld\n", (u32)status);
	}

	//dev_err(&dev->spi->dev, "st7565p_framebuffer_sync_partial OK\n");

	return status;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t st7565_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct st7565_desc	*dev;
	ssize_t status = 0;

	if(count > ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE)
		return -EMSGSIZE;
	if(*f_pos >= ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE)
		return -EFAULT;
	if((*f_pos + count) > ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE)
		return -ESPIPE;

	dev = filp->private_data;

	mutex_lock(&dev->buf_lock);
	status = copy_to_user(buf, &dev->frame_buffer[*f_pos], count);
	*f_pos += count;
	mutex_unlock(&dev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t st7565_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct st7565_desc	*dev;
	ssize_t status = 0;

	if(count > ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE)
		return -EMSGSIZE;
	if(*f_pos >= ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE)
		return -EFAULT;
	if((*f_pos + count) > ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE)
		return -ESPIPE;

	dev = filp->private_data;

	//dev_err(&dev->spi->dev, "st7565_write OK\n");

	mutex_lock(&dev->buf_lock);
	status = copy_from_user(&dev->frame_buffer[*f_pos], buf, count);
	st7565p_framebuffer_sync_partial(dev, *f_pos, count);
	//st7565p_framebuffer_sync(dev);
	*f_pos += count;
	mutex_unlock(&dev->buf_lock);

	return status;
}



static long st7565_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct st7565_desc	*dev;
	struct spi_device	*spi;
	u32			tmp;
//	unsigned		n_ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != ST7565_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	dev = filp->private_data;
	spin_lock_irq(&dev->lock);
	spi = spi_dev_get(dev->spi);
	spin_unlock_irq(&dev->lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	mutex_lock(&dev->buf_lock);

	switch (cmd) {

	/* commands */
	case ST7565_IOC_RESET:
		st7565p_framebuffer_clear(dev);
		st7565p_framebuffer_sync(dev);
		break;

	/* read requests */
	case ST7565_IOC_GET_WIDTH:
		tmp = ST7565R_DISPLAY_LCD_WIDTH;
		__put_user(tmp, (u32 __user *)arg);
		break;

	case ST7565_IOC_GET_HEIGHT:
		tmp = ST7565R_DISPLAY_LCD_HEIGHT;
		__put_user(tmp, (u32 __user *)arg);
		break;

	case ST7565_IOC_GET_PIXELPERBYTE:
		tmp = ST7565R_DISPLAY_LCD_PIXELS_PER_BYTE;
		__put_user(tmp, (u32 __user *)arg);
		break;

	case ST7565_IOC_GET_FRAMEBUFSZ:
		tmp = ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE;
		__put_user(tmp, (u32 __user *)arg);
		break;

	case ST7565_IOC_GET_CONTRAST:
		tmp = dev->contrast;
		__put_user(tmp, (u32 __user *)arg);
		break;

	case ST7565_IOC_SET_CONTRAST:
			__get_user(tmp, (u32 __user *)arg);
			dev->contrast = tmp;
			st7565p_set_contrast(dev, dev->contrast);
			break;
	
	default:
		retval = -EINVAL;
		break;
	}

	mutex_unlock(&dev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long st7565_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return st7565_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define st7565_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int st7565_open(struct inode *inode, struct file *filp)
{
	struct st7565_desc	*dev;
	int			status = -ENXIO;

	//printk("st7565_open\r\n");

	mutex_lock(&device_list_lock);

	list_for_each_entry(dev, &device_list, device_entry) {
		if (dev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("st7565: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}


	if(!dev->frame_buffer) {
		dev->frame_buffer = kmalloc(ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE, GFP_KERNEL);
		if (!dev->frame_buffer) {
				dev_dbg(&dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
				goto err_find_dev;
		}
	}


	if(dev->users == 0) {

			// setup master
			status = st7565_setup_spi(dev);
			if(status < 0) {
				dev_err(&dev->spi->dev, "open failed, setup on master failed\n");
				goto err_spi_master;
			}

			st7565p_hard_reset(dev);
			st7565p_set_display_start_line_address(dev, 0);

			// The column address is set to increasing
			st7565p_write_command(dev, ST7565R_CMD_ADC_REVERSE);
			//st7565p_write_command(ST7565R_CMD_ADC_NORMAL);

			// Non-inverted display
			st7565p_write_command(dev, ST7565R_CMD_DISPLAY_NORMAL);

			// The common mode scan direction is reversed COM31->COM0
			//st7565p_write_command(ST7565R_CMD_REVERSE_SCAN_DIRECTION);
			st7565p_write_command(dev, ST7565R_CMD_NORMAL_SCAN_DIRECTION);

			// Set the voltage bias ratio to 1/6
			st7565p_write_command(dev, ST7565R_CMD_LCD_BIAS_1_DIV_6_DUTY33);

			// Set booster circuit, voltage regulator and voltage follower all to on
			st7565p_write_command(dev, ST7565R_CMD_POWER_CTRL_ALL_ON);

			// Set the booster ratio to 2X,3X,4X
			st7565p_write_command(dev, ST7565R_CMD_BOOSTER_RATIO_SET);
			st7565p_write_command(dev, ST7565R_CMD_BOOSTER_RATIO_2X_3X_4X);

			// Set voltage resistor ratio to 1
			st7565p_write_command(dev, ST7565R_CMD_VOLTAGE_RESISTOR_RATIO_1);

			/* Set contrast to min value, no need to check return value as the contrast
			is set to the defined min*/
			//st7565p_set_contrast(ST7565R_DISPLAY_CONTRAST_MIN);
			st7565p_set_contrast(dev, ST7565R_DISPLAY_CONTRAST_MIN);

			// Turn on the display
			st7565p_write_command(dev, ST7565R_CMD_DISPLAY_ON);

			// clear buffer and sync with display
			st7565p_framebuffer_clear(dev);
			st7565p_framebuffer_sync(dev);

			dev->users++;
	}


	//printk("open done\n");
	filp->private_data = dev;
	//nonseekable_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_spi_master:
	kfree(dev->frame_buffer);
	dev->frame_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int st7565_release(struct inode *inode, struct file *filp)
{
	struct st7565_desc	*dev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	dev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	dev->users--;
	if (!dev->users) {
		int		dofree;

		kfree(dev->frame_buffer);
		dev->frame_buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&dev->lock);
		dofree = (dev->spi == NULL);
		spin_unlock_irq(&dev->lock);

		if (dofree)
			kfree(dev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static loff_t st7565_llseek(struct file *filp, loff_t off, int whence)
{
	switch (whence) {
		case 0: // SEEK_SET
			if (off > ST7565R_DISPLAY_LCD_FRAMEBUFFER_SIZE || off < 0) {
				printk(KERN_ERR "unsupported SEEK_SET offset %llx\n", off);
				return -EINVAL;
			}
			break;
		case 1: // SEEK_CUR
		case 2: // SEEK_END (not supported, hence fall though)
		default:
			// how did we get here !
			printk(KERN_ERR "unsupported seek operation\n");
			return -EINVAL;
	}
	filp->f_pos = off;
	return off;
}


static const struct file_operations st7565_fops = {
	.owner =	THIS_MODULE,
	.write =	st7565_write,
	.read =		st7565_read,
	.unlocked_ioctl = st7565_ioctl,
	.compat_ioctl = st7565_compat_ioctl,
	.open =		st7565_open,
	.release =	st7565_release,
	.llseek =	st7565_llseek,
};

/*-------------------------------------------------------------------------*/
static int st7576_major;
static struct class *st7565_class;

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static int st7565_of_setup_device(struct st7565_desc *dev)
{
	struct device_node *np;
	int  nb;

	dev_err(&dev->spi->dev, "OF setup_device\n");

	np = dev->spi->dev.of_node;
	nb = of_gpio_named_count(np, "reset-gpio");

	if(nb < 1) {
		dev_err(&dev->spi->dev, "No reset-gpio!\n");
	}

	nb = of_gpio_named_count(np, "addr-gpio");
	if(nb < 1) {
		dev_err(&dev->spi->dev, "No addr-gpio!\n");
	}


	dev->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	dev->addr_gpio = of_get_named_gpio(np, "addr-gpio", 0);

	dev_err(&dev->spi->dev, "OF setup_device OK\n");

	nb = gpio_request(dev->reset_gpio, dev_name(&dev->spi->dev));
	if(nb) {
		dev_err(&dev->spi->dev, "Failed to get reset GPIO\n");
		return nb;
	}

	nb = gpio_request(dev->addr_gpio, dev_name(&dev->spi->dev));
	if(nb) {
		dev_err(&dev->spi->dev, "Failed to get ADDRESS GPIO\n");
		return nb;
	}


	gpio_direction_output(dev->addr_gpio,1);
	gpio_direction_output(dev->reset_gpio,1);

	return 0;
}
#else
static int st7565_of_setup_device(struct st7565_desc *dev)
{
	return 0;
}
#endif


/*-------------------------------------------------------------------------*/

static int st7565_probe(struct spi_device *spi)
{
	struct st7565_desc	*dev;
	int			status;
	unsigned long		minor;

	dev_dbg(&spi->dev, "st7565_probe\n");

	/* Allocate driver data */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* Initialize the driver data */
	dev->spi = spi;
	spin_lock_init(&dev->lock);
	mutex_init(&dev->buf_lock);
	dev->users = 0;
	dev->frame_buffer = NULL;

	INIT_LIST_HEAD(&dev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *cdev;

		dev->devt = MKDEV(st7576_major, minor);
		cdev = device_create(st7565_class, &spi->dev, dev->devt,
				    dev, "st7565lcd%ld", minor);
		status = PTR_ERR_OR_ZERO(cdev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, dev);
	else
		kfree(dev);

	st7565_of_setup_device(dev);

	return status;
}

static int st7565_remove(struct spi_device *spi)
{
	struct st7565_desc *dev = spi_get_drvdata(spi);

	dev_dbg(&spi->dev, "st7565_remove\n");

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&dev->lock);
	dev->spi = NULL;
	spin_unlock_irq(&dev->lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&dev->device_entry);
	device_destroy(st7565_class, dev->devt);
	clear_bit(MINOR(dev->devt), minors);
	if (dev->users == 0)
		kfree(dev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct of_device_id st7565_dt_ids[] = {
	{ .compatible = "radical,rbt210lcd" },
	{},
};

MODULE_DEVICE_TABLE(of, st7565_dt_ids);

static struct spi_driver st7565_spi_driver = {
	.driver = {
		.name =		"st7565",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(st7565_dt_ids),
	},
	.probe =	st7565_probe,
	.remove =	st7565_remove,

	
};

/*-------------------------------------------------------------------------*/

static int __init st7565_init(void)
{
	int status;

	st7576_major = register_chrdev(0, "lcd", &st7565_fops);
	if (st7576_major < 0)
		return status;
 
	st7565_class = class_create(THIS_MODULE, "st7565lcd");
	if (IS_ERR(st7565_class)) {
		unregister_chrdev(st7576_major, st7565_spi_driver.driver.name);
		return PTR_ERR(st7565_class);
	}

	status = spi_register_driver(&st7565_spi_driver);
	if (status < 0) {
		class_destroy(st7565_class);
		unregister_chrdev(st7576_major, st7565_spi_driver.driver.name);
	}
	return status;
}

static void __exit st7565_exit(void)
{
	spi_unregister_driver(&st7565_spi_driver);
	class_destroy(st7565_class);
	unregister_chrdev(st7576_major, st7565_spi_driver.driver.name);
}

module_init(st7565_init);
module_exit(st7565_exit);

MODULE_AUTHOR("Jan Zwiegers, <jan@radicalsystems.co.za>");
MODULE_DESCRIPTION("ST7565 LCD Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:st7565");
