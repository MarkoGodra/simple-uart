#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/amba/serial.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>

#define BAUDRATE (115200)
#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER 1
#define BUF_SIZE 32

// Functions
static ssize_t feserial_read(struct file *file, char __user *buf, size_t sz, loff_t *ppos);
static ssize_t feserial_write(struct file *file, const char __user *buf, size_t sz, loff_t *ppos);
static long feserial_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

// Vars
struct feserial_dev {

                struct miscdevice miscdev;
                spinlock_t lock;
                wait_queue_head_t feserial_wait;
                void* __iomem regs;
                char buf[BUF_SIZE];
                u32 clock_freq;
                u32 tx_counter;
                int irq;              
                u8 rx_char;
                u8 condition;
};

// Fops
static const struct file_operations feserial_fops = {
        .owner = THIS_MODULE,
        .read = feserial_read,
        .write = feserial_write,
        .unlocked_ioctl = feserial_ioctl,
};

static unsigned int reg_read(struct feserial_dev *dev, int offset)
{
        return readl(dev->regs + offset);
}

static void reg_write(struct feserial_dev *dev, int value, int offset)
{
        writel(value, dev->regs + offset);
}

static void serial_tx(struct feserial_dev *dev, u32 value)
{
        // Status register will be read here
        u32 reg_val;

        do
        {
                // Read flag register
                reg_val = reg_read(dev, UART01x_FR);
                cpu_relax();

        }while(reg_val & UART01x_FR_BUSY); // While uart is busy

        // When uart is not busy anymore transmit
        reg_write(dev, value, UART01x_DR);

        dev->tx_counter++;
}

static irqreturn_t feserial_irq(int irq, void *dev_id)
{
        unsigned long args;
        u32 reg_val;
        struct feserial_dev* dev = (struct feserial_dev*)dev_id;

        // Read data reg to clear interupts
        reg_val = reg_read(dev, UART01x_DR);

        // Acqure spinlock and disable interupts on local cpu
        spin_lock_irqsave(&dev->lock, args);

        // Save receieved char
        dev->rx_char = (u8)reg_val;
        dev->condition = 1;

        // Release spinlock and enable interupts on local cpu
        spin_unlock_irqrestore(&dev->lock, args);

        // Wakeup
        wake_up_interruptible(&dev->feserial_wait);

        return IRQ_HANDLED;
}

static long feserial_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        struct feserial_dev *dev = container_of(file->private_data, struct feserial_dev, miscdev);
       
        // Passed arg is actually address from user space
        void __user *argp = (void __user *) arg;
         
        switch(cmd)
        {
                case SERIAL_RESET_COUNTER:
                        //Reset tx counter
                        dev->tx_counter = 0;
                break;

                case SERIAL_GET_COUNTER:
                        // Cpy tx_counter to userspace
                        if(copy_to_user((u32 *)argp, &dev->tx_counter, sizeof(u32)))
                        {
                                return -EFAULT;
                        }
                break;
        }

        /* Success. */
        return 0;
}

static ssize_t feserial_read(struct file *file, char __user *buf, size_t sz, loff_t *ppos)
{
        int ret;
        unsigned long args;
        ssize_t chars_read = sizeof(char);
        struct feserial_dev *dev = container_of(file->private_data, struct feserial_dev, miscdev);

        // Set wait
        wait_event_interruptible(dev->feserial_wait, dev->condition);

        // Lock the lock
        spin_lock_irqsave(&dev->lock, args);

        // Copy recieved char to user space 
        ret = copy_to_user(buf, &dev->rx_char, sizeof(char));
        if(ret != 0)
        {
                pr_alert("Failed copy_to_user()\n");
                return -ENOTTY;
        }

        // If carrige return is caught add newline
        if(dev->rx_char == '\r')
        {
                u8 new_line_char = '\n';
                ret = copy_to_user(buf, &new_line_char, sizeof(char));
                if(ret != 0)
                {
                        pr_alert("Failed copy_to_user()\n");
                        return -ENOTTY;
                }

                // Set return to 2 chars in case of carrige return
                chars_read++;
        }

        // Reset condition
        dev->condition = 0;

        spin_unlock_irqrestore(&dev->lock, args);

        // Because read will block until there is char available
        return chars_read;
}

// This sys call will block untile whole passed buffer is sent via serial device
static ssize_t feserial_write(struct file *file, const char __user *buf, size_t sz, loff_t *ppos)
{
        unsigned i;
        struct feserial_dev *dev = container_of(file->private_data, struct feserial_dev, miscdev);

        for(i = 0; i < sz; i++)
        {
                char user_char;

                // Copy single char from user
                if(copy_from_user(&user_char, buf + i, 1))
                {
                        return -EFAULT;
                }

                // If oncoming char is newline pass carrige return
                if(user_char == 0x0a) // '\n'
                {
                        serial_tx(dev, (u32)0x0d); // Add '\r'
                }

                // Send actual char
                serial_tx(dev, (u32)user_char);
        }

        // Since this sys call will block, it will process all char passed from user space
        return sz;
}

static int feserial_probe(struct platform_device *pdev)
{
        u32 reg_val;
        int rv;
        unsigned baud_divisor;
        unsigned baud_divisor_frac;
        struct resource *res;
        struct feserial_dev* dev; 
        struct device_node *clk_node;

	pr_alert("Called feserial_probe\n");

        dev = (struct feserial_dev*)devm_kzalloc(&pdev->dev, sizeof(struct feserial_dev), GFP_KERNEL);

        platform_set_drvdata(pdev, dev);

        // Extract data from dts
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);   
        if(res == NULL)
        {
                pr_alert("Failed to get paltform resources");
                return -ENOTTY;
        }

        pr_alert("Acquired address is: %x\n", res->start);

        // Remap device regs
        dev->regs = devm_ioremap_resource(&pdev->dev, res);
        if (!dev->regs) 
        {
                dev_err(&pdev->dev, "Cannot remap registers\n");
                return -ENOMEM;
        }

        // Read clocks[0]
        clk_node = of_parse_phandle(pdev->dev.of_node, "clocks", 0); 
        if (!clk_node) {
                dev_err(&pdev->dev, "Cannot get clk node\n");
                return -EBUSY;
        }

        /* read timer freq */
        of_property_read_u32(clk_node, "clock-frequency", &dev->clock_freq);

        // Initialize tx counter
        dev->tx_counter = 0;

        // Initialzie rx char
        dev->rx_char = 0;

        // Read condition
        dev->condition = 0;

        // Init waitqueu head
        init_waitqueue_head(&dev->feserial_wait);

        // Init spinlock
        spin_lock_init(&dev->lock);

        pr_alert("Clock frequency: %u\n", dev->clock_freq);

        //Calculate divisor
        baud_divisor = (dev->clock_freq) / (16 * BAUDRATE);

        // Move it 1st to free 6 lower bits
        baud_divisor_frac = ((dev->clock_freq << 6) / (16 * BAUDRATE)) & (0x3F); // Mask with 0x3f to get 6 lower bits

        pr_alert("Baud divisor: %u\n", baud_divisor);
        pr_alert("Baud divisor frac: %u\n", baud_divisor_frac);

        /* Enable power management */
        pm_runtime_enable(&pdev->dev);
        pm_runtime_get_sync(&pdev->dev);

        // Turn of uart
        reg_val = reg_read(dev, UART011_CR);
        reg_val &= ~UART01x_CR_UARTEN; // Set lowest bit to 0
        reg_write(dev, reg_val, UART011_CR); 

        // Set baud divisors
        reg_write(dev, baud_divisor, UART011_IBRD);
        reg_write(dev, baud_divisor_frac, UART011_FBRD);

        // Disable FIFOs
        reg_val = reg_read(dev, UART011_LCRH);
        reg_val &= ~UART01x_LCRH_FEN;
        reg_write(dev, reg_val, UART011_LCRH);

        // Turn on uart
        reg_val = reg_read(dev, UART011_CR);
        reg_val |= UART01x_CR_UARTEN; // Set lowest bit to 1 (Enable uart)
        reg_val |= UART011_CR_TXE; // Enable TX (8th bit)
        reg_val |= UART011_CR_RXE; // Enable RX (9th bit)
        reg_write(dev, reg_val, UART011_CR);

        // Enable RX interrupt by writing 1 to 4th bit of IMSC reg
        reg_val = reg_read(dev, UART011_IMSC);
        reg_val |= UART011_RXIM; // Set 4th bit
        reg_write(dev, reg_val, UART011_IMSC);

        /* Declare misc device */
        dev->miscdev.minor = MISC_DYNAMIC_MINOR;
        dev->miscdev.name = "serial";
        dev->miscdev.fops = &feserial_fops;

        // Get irq no
        dev->irq = platform_get_irq(pdev, 0);

        pr_alert("Acquired interupt number: %d\n", dev->irq);

        /* Register interrupt handler */
        rv = devm_request_irq(&pdev->dev, dev->irq, feserial_irq, 0, "feserial", (void*)dev);
        if(rv != 0)
        {
                pr_alert("Failed to request irq\n");
                return -ENOTTY;
        }

        // Register misc device to get fs entry
        rv = misc_register(&dev->miscdev);
        if (rv < 0)
        {
                pr_alert("Timer_Driver: cannot obtain major number, ERROR %d\n", rv);
                return rv;
        }

	return 0;
}

static int feserial_remove(struct platform_device *pdev)
{
        u32 reg_val;
        struct feserial_dev* dev;

        // Retrieve private data
        dev = (struct feserial_dev*)platform_get_drvdata(pdev);

        pr_alert("Called feserial_remove\n");

        // Disable RX and TX
        reg_val = reg_read(dev, UART011_CR);
        reg_val &= ~UART011_CR_TXE; // Disable TX (8th bit)
        reg_val &= ~UART011_CR_RXE; // Disable RX (9th bit)
        reg_write(dev, reg_val, UART011_CR);

        // Turn off uart
        reg_val = reg_read(dev, UART011_CR);
        reg_val &= ~UART01x_CR_UARTEN; // Set lowest bit to 0
        reg_write(dev, reg_val, UART011_CR); 

        // Disable power management
        pm_runtime_disable(&pdev->dev);

        /* Freeing the major number. */
        misc_deregister(&dev->miscdev);

        return 0;
}

static struct of_device_id feserial_dt_match[] = {
        { .compatible = "rtrk,serial" },
        { },
};

static struct platform_driver feserial_driver = {
        .driver = {
                .name = "feserial",
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(feserial_dt_match),
        },
        .probe = feserial_probe,
        .remove = feserial_remove,
};

module_platform_driver(feserial_driver);
MODULE_LICENSE("GPL");
