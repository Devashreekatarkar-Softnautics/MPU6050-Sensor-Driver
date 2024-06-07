// SPDX-License-Identifier: MIT
/*The header files for the Sensor Driver
 *======================================
 *module.h	- Required when writing any kernel module
 *		Internally includes version.h (Includes def. of _GPL() Function Macro)
 *uaccess.h	- Included for userspace communication (copy_to_user() & copy_from_user())
 *err.h		- Included for error handling macros
 *i2c.h		- Included for i2c protocol implementation
 *init.h	- Required for module_init and module_exit macros
 *ioctl.h	- Required for implementing the ioctl interface
 *kernel.h	- Required for printing information in kernel log buffers
 *		- Includes types.h header - Contains data types char dev etc
 *sysfs.h & kobject.h	- Required for creating and initializing sysfs interface
 *proc_fs	- Required for creating and initializing Procfs interface
 *gpio.h	- Required for configuring the GPIO's of the PI
 *fs.h		- Includes structure of file operations
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/fs.h>

unsigned int Irq_Num;
int etx_value;
struct kobject *kobj_ref;
static struct work_struct mpu6050_work;

/*_adapter represents a bus and _client represents the slave*/
static struct i2c_adapter *mpu6050_adapter;
static struct i2c_client  *mpu6050_sensor;
static struct proc_dir_entry *pd_entry;

#define I2C_BUS_AVAILABLE 1
#define SLAVE_DEVICE_NAME "MPU6050"
#define MPU6050_SLAVE_ADDR 0x68
#define MPU6050_PWR1_REG 0x6B
#define MPU6050_SMP_RATE 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_INT_EN 0x38
#define MPU6050_INT_PIN 0x37
#define MPU6050_INT_STATS 0x3A
#define GPIO_25 596
#define PROCFS_NAME "MPU6050_SENSOR"
#define FAILURE -1
#define SUCCESS  0
#define DEVICE_NAME "MPU6050"
#define NUM_DEVICES 1
#define BUF_LEN 24
#define RD_VAL _IOR('a', 'b', int32_t *)

/* Acceleration data address reg */
#define X_ACC 0x3B
#define Y_ACC 0x3D
#define Z_ACC 0x3F


static const struct i2c_device_id mpu6050_id[] = {
	{SLAVE_DEVICE_NAME, 0},
	{ }
};

// This function is used to expose the driver along with its I2C Device table IDs to userspace
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

/* Initializes the required field of this structure and use that information for initializing the dev */
static struct i2c_board_info I2C_MPU6050 = {
	I2C_BOARD_INFO(SLAVE_DEVICE_NAME, MPU6050_SLAVE_ADDR),
};

/*************************** Function Prototypes *******************************/

static void Mpu6050_Work_Func(struct work_struct *work);
static int Mpu6050_Write(struct i2c_client *client, u8 reg, u8 val);
static int16_t Mpu6050_Read(struct i2c_client *client, u8 reg);
static irqreturn_t Mpu_Irq_Handler(int irq, void *dev_id);

static int chardev_open(struct inode *, struct file *);
static int chardev_close(struct inode *, struct file *);
static long Sensor_Ioctl(struct file *, unsigned int, unsigned long);

/******************************************************************************/

/*************************** Sysfs functions **********************************/

static ssize_t  sysfs_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
static ssize_t  sysfs_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count);

struct kobj_attribute etx_attr = __ATTR(etx_value, 0660, sysfs_show, sysfs_store);

/******************************************************************************/

static dev_t dev;
static struct cdev sensor_dev;
static struct class *sensor_class;
int32_t value[3];

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = chardev_open,
	.unlocked_ioctl = Sensor_Ioctl,
	.release = chardev_close,

};

static ssize_t Mpu6050_Procfs_Read(struct file *filp, char *buf, size_t count, loff_t *offp)
{

	int ret = 0;
	char tmp[1000] = { 0 };

	pr_info("PROFS READ FUNCTION CALLED %s\n", PROCFS_NAME);

	if (*offp > 0)
		return 0;

	sprintf(tmp, "MPU6050 ACCELEROMETER SENSOR PROBED\n"
			"Operating voltage: 2.375 - 3.46 v\n"
			"Full scale range: [-2g,+2g]\n"
			"Sensitivity scale factor: 16384 LSB/g\n"
			"Sample rate: 1KHZ\n"
			"Data fetched in the form of X-Y-Z axis: Static acceleration (with respect to gravity)\n");

	if (copy_to_user(buf, tmp, strlen(tmp))) {
		pr_info("Error in copy to user\n");
		return -EFAULT;
	}

	ret = *offp = strlen(tmp);
	return ret;
}

static struct proc_ops proc_fops = {
	.proc_read = Mpu6050_Procfs_Read
};

/* SYSFS Interface - Read write operations corresponding to _show & _store respectively */
static ssize_t sysfs_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	pr_info("SYSFS READ FUNCTION\n");
	s16 accel_x = 0, accel_y = 0, accel_z = 0;

	accel_x = Mpu6050_Read(mpu6050_sensor, X_ACC);
	accel_y = Mpu6050_Read(mpu6050_sensor, Y_ACC);
	accel_z = Mpu6050_Read(mpu6050_sensor, Z_ACC);

	return sprintf(buf, "X-axis: %d g Y-axis: %d g Z-axis: %d g\n", accel_x, accel_y, accel_z);

}

/* This function will be called when we write the sysfs file */
static ssize_t sysfs_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	pr_info("SYSFS WRITE FUNCTION\n");
	int ret_val = sscanf(buf, "%d", &etx_value);

	if (!ret_val) {
		pr_info("Reading Sensor data into the buffer failed\n");
		return FAILURE;
	}
	return count;
}

static void Mpu6050_Work_Func(struct work_struct *work)
{
	s16 accel_x, accel_y, accel_z;

	accel_x = Mpu6050_Read(mpu6050_sensor, X_ACC);
	accel_y = Mpu6050_Read(mpu6050_sensor, Y_ACC);
	accel_z = Mpu6050_Read(mpu6050_sensor, Z_ACC);

	//		accel_x /= 16384;
	//		accel_y /= 16384;
	//		accel_z /= 16384;

	pr_info("Accel X: %d, Y: %d, Z: %d\n", accel_x,  accel_y, accel_z);
//	msleep(0.001);
}

static int Mpu6050_Write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int16_t Mpu6050_Read(struct i2c_client *client, u8 reg)
{
	s16 a_x = 0;
	u8 a_h = 0, a_l = 0;

	a_h = i2c_smbus_read_byte_data(client, reg);
	a_l = i2c_smbus_read_byte_data(client, reg + 1);

	a_x = (s16)(a_h << 8 | a_l);
	return a_x;
}

static irqreturn_t Mpu_Irq_Handler(int irq, void *dev_id)
{
	/* pr_info("INTERRUPT HANDLER CALLED\n"); */
	// This function puts the job on the kernel global workqueue
	schedule_work(&mpu6050_work);
	return IRQ_HANDLED;
}

static int mpu6050_probe(struct i2c_client *client)
{
	/*Configuring & Initializing the PWR_MNG REG*/
	if (Mpu6050_Write(client, MPU6050_PWR1_REG, 0x00) < 0) {
		dev_err(&client->dev, "Failed to wake up MPU6050\n");
		return FAILURE;
	}

	/*Configure interrupt pin to be active high, push-pull, and enable interrupt
	 *LATCH_INT_EN - set to 1, int pin held high until the interrupt is cleared
	 *INT_RD_CLEAR - set to 1, INT_STATUS bits are cleared on any read operation
	 */
	if (Mpu6050_Write(client, MPU6050_INT_PIN, 0x30) < 0) {
		dev_err(&client->dev, "Failed to configure interrupt pin\n");
		return FAILURE;
	}

	/*Enable DATA_READY_INTERRUPT */
	if (Mpu6050_Write(client, MPU6050_INT_EN, 0x01) < 0) {
		dev_err(&client->dev, "Failed to enable interrupt\n");
		return FAILURE;
	}

	/*Configure sample rate divider */
	if (Mpu6050_Write(client, MPU6050_SMP_RATE, 0x0F) < 0) {
		dev_err(&client->dev, "Failed to set sample rate\n");
		return FAILURE;
	}

	/*Configure accelerometer range and bandwidth
	 *DLPF BITS CONFIGURED - B.W: 260 HZ & FREQ: 1 KHZ
	 */
	if (Mpu6050_Write(client, MPU6050_CONFIG, 0x00) < 0) {
		dev_err(&client->dev, "Failed to configure accelerometer\n");
		return FAILURE;
	}

	/* Check and request for the gpio 25 of raspberry pi, connected to INT pin of the sensor */
	if (!gpio_is_valid(GPIO_25)) {
		pr_err("GPIO PIN INVALID\n");
		return FAILURE;
	}

	//pr_info("Interrupt GPIO PIN VAL:%d\n", gpio_get_value(GPIO_25));

	/* Request and configure GPIO for interrupt */

	if (gpio_request(GPIO_25, "GPIO_25") < 0) {
		pr_err("GPIO PIN REQUEST FAILED\n");
		return FAILURE;
	}

	/* Direction set to input to read the data interrupt triggers from the sensor */
	gpio_direction_input(GPIO_25);

	/* _WORK : Creates the workqueue in the linux w name mpu6050_work and 2 nd arg is the function to be scheduled
	 *		in the workqueue
	 */
	INIT_WORK(&mpu6050_work, Mpu6050_Work_Func);

	Irq_Num = gpio_to_irq(GPIO_25);
	if (!Irq_Num) {
		dev_err(&mpu6050_sensor->dev, "Failed to get IRQ number for GPIO 25\n");
		return FAILURE;
	}

	/* Attaching the interrupt handler to GPIO PIN 25 ( In raspberry pi PIN 25 --> 596) */

	if (request_irq(Irq_Num, Mpu_Irq_Handler, IRQF_TRIGGER_HIGH, "mpu6050_irq", NULL)) {
		dev_err(&mpu6050_sensor->dev, "Failed to request IRQ\n");
		return FAILURE;
	}

	pr_info("MPU6050 SENSOR PROBED\n");
	return SUCCESS;
}

static struct i2c_driver mpu6050_driver = {

	.driver = {
		.name   = SLAVE_DEVICE_NAME,
	},
	.probe = mpu6050_probe,
	.id_table = mpu6050_id,
};

/* Module Init function */

static int __init sensor_driver_init(void)
{

	/*Allocates a range of character device numbers.
	 * - Major number is allocated dynamically
	 * - Minor number is returned with first minor number(requested).
	 */

	if (alloc_chrdev_region(&dev, 0, NUM_DEVICES, DEVICE_NAME) < 0) {
		pr_info("CHAR DEV ALLOCATION FAILED %d\n", FAILURE);
		return FAILURE;
	}

	/* Creating cdev structure & Initializing the sturct with struct class and file operations */

	cdev_init(&sensor_dev, &fops);
	if (!sensor_dev.ops)
		pr_err("Character driver initialization failed\n");

	/*Adding character device to the system*/
	if (cdev_add(&sensor_dev, dev, 1) < 0) {
		pr_err("Cannot add the device to the system\n");
		goto r_del;
	}

	/*Creating struct class*/
	sensor_class = class_create(DEVICE_NAME);
	if (IS_ERR(sensor_class)) {
		pr_err("Cannot create the struct class\n");
		goto r_class;
	}

	/* Creating device
	 * _create() - Creates the dev and registers it sysfsI
	 * IS_ERR() - Converts the const void * to an integer value
	 */
	if (IS_ERR(device_create(sensor_class, NULL, dev, NULL, DEVICE_NAME))) {
		pr_err("Cannot create the Device\n");
		goto r_device;
	}

	kobj_ref = kobject_create_and_add("SENSOR_SYSFS", kernel_kobj);
	if (kobj_ref == NULL) {
		pr_err("kernel Object Creation failed\n");
		kobject_put(kobj_ref);
		return FAILURE;
	}

	/*Creating sysfs file for etx_value*/

	if (sysfs_create_file(kobj_ref, &etx_attr.attr)) {
		pr_err("Cannot create sysfs file\n");
		goto r_sysfs;
	}

	pd_entry = proc_create(PROCFS_NAME, 0, NULL, &proc_fops);

	if (pd_entry == NULL) {
		remove_proc_entry(PROCFS_NAME, NULL);
		pr_info("Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	//The adapter structure is used to identify a physical i2c bus
	mpu6050_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

	if (mpu6050_adapter != NULL) {
		/* The below functions instantiates the device from I2C bus */
		mpu6050_sensor = i2c_new_client_device(mpu6050_adapter, &I2C_MPU6050);
		if (mpu6050_sensor != NULL) {
			/* _driver Structure is added to i2c subsystem */
			if (i2c_add_driver(&mpu6050_driver) < 0) {
				pr_err("Unable to add driver\n");
				i2c_unregister_device(mpu6050_sensor);
				return FAILURE;
			}
		}
		i2c_put_adapter(mpu6050_adapter);
	}

	pr_info("Adapter timeout: %d Adapter retries : %d Adapter bus number: %d\n", mpu6050_adapter->timeout,
			mpu6050_adapter->retries, mpu6050_adapter->nr);

	pr_info("Driver Added\n");

	return SUCCESS;

r_sysfs:
	kobject_put(kobj_ref);
	sysfs_remove_file(kernel_kobj, &etx_attr.attr);
r_device:
	device_destroy(sensor_class, dev);
r_class:
	class_destroy(sensor_class);
r_del:
	cdev_del(&sensor_dev);
	return FAILURE;
}

/* Module Exit function */
static void __exit sensor_driver_exit(void)
{
	i2c_unregister_device(mpu6050_sensor);
	i2c_del_driver(&mpu6050_driver);

	gpio_free(GPIO_25);
	free_irq(Irq_Num, NULL);
	flush_work(&mpu6050_work);

	kobject_put(kobj_ref);
	sysfs_remove_file(kernel_kobj, &etx_attr.attr);
	remove_proc_entry(PROCFS_NAME, NULL);
	device_destroy(sensor_class, dev);
	class_destroy(sensor_class);
	cdev_del(&sensor_dev);
	unregister_chrdev_region(dev, NUM_DEVICES);

	pr_info("Driver Removed\n");
}

static int chardev_open(struct inode *inode, struct file *file)
{
	pr_info("Device File open\n");
	return SUCCESS;
}

static int chardev_close(struct inode *inode, struct file *file)
{
	pr_info("%s Device Released\n", DEVICE_NAME);
	return SUCCESS;
}

static long Sensor_Ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

	s16 accel[3] = {0};

	accel[0] = Mpu6050_Read(mpu6050_sensor, X_ACC);
	accel[1] = Mpu6050_Read(mpu6050_sensor, Y_ACC);
	accel[2] = Mpu6050_Read(mpu6050_sensor, Z_ACC);

	pr_info("IOCTL FUNCTION CALLED\n");
	for (int i = 0; i < 3; i++) {
		value[i] = accel[i];
		if (copy_to_user((int32_t *) arg, &value, sizeof(value)) != 0)
			pr_info("Writing data to user failed\n");
	}
	return SUCCESS;
}

module_init(sensor_driver_init);
module_exit(sensor_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devashree Katarkar");
MODULE_DESCRIPTION("I2C Driver for ACCELEROMETER SENSOR");
MODULE_VERSION("1.34");
