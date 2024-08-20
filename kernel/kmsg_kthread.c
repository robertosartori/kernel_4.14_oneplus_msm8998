#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#define BUFFER_SIZE 16384 // Buffer size for reading
#define MAX_LOG_SIZE (50 * 1024 * 1024)  // 50 MB

static struct task_struct *kmsg_to_log_thread;
static char *log_file_path = "/cache/log.txt";

static int kmsg_to_log_thread_fn(void *data)
{
	struct file *src_file, *dest_file;
	mm_segment_t old_fs;
	char *buffer;
	ssize_t bytes_read, bytes_written;
	static loff_t current_file_size = 0;

	// Allocate memory for the buffer
	buffer = kmalloc(BUFFER_SIZE, GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to allocate buffer\n");
		return -ENOMEM;
	}

	// Open /dev/kmsg for reading
	src_file = filp_open("/dev/kmsg", O_RDONLY, 0);
	if (IS_ERR(src_file)) {
		pr_err("Failed to open /dev/kmsg\n");
		kfree(buffer);
		return PTR_ERR(src_file);
	}

	// Open the log file for writing, creating the file if it doesn't exist
	dest_file =
		filp_open(log_file_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
	if (IS_ERR(dest_file)) {
		pr_err("Failed to create %s\n", log_file_path);
		filp_close(src_file, NULL);
		kfree(buffer);
		return PTR_ERR(dest_file);
	}

	// Save the current address limit
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	// Read and write data in a loop
	while (!kthread_should_stop()) {
		bytes_read = kernel_read(src_file, buffer, BUFFER_SIZE,
					 &src_file->f_pos);
		if (bytes_read <= 0) continue;

		bytes_written = kernel_write(dest_file, buffer, bytes_read,
					     &dest_file->f_pos);

		if (bytes_written < 0) {
			pr_err("Failed to write to %s\n", log_file_path);
			break;
		}
		current_file_size = vfs_llseek(dest_file, 0, SEEK_END);
		// Check if the file is about to exceed 50 MB
		if (current_file_size + bytes_read > MAX_LOG_SIZE) {
			pr_info("Log file will exceed 50 MB, resetting...\n");
			filp_close(dest_file, NULL);
			dest_file = filp_open(log_file_path, O_WRONLY | O_TRUNC | O_CREAT, 0644);
			if (IS_ERR(dest_file)) {
				pr_err("Failed to reset log file: %ld\n", PTR_ERR(dest_file));
				break;
			}
	        }
	}

	// Restore the previous address limit
	set_fs(old_fs);

	// Close the files and free the buffer memory
	filp_close(src_file, NULL);
	filp_close(dest_file, NULL);
	kfree(buffer);

	return 0;
}

struct delayed_work my_work;

void real_init(struct work_struct *work)
{
	kmsg_to_log_thread =
                kthread_run(kmsg_to_log_thread_fn, NULL, "kmsg_to_log");
	if (IS_ERR(kmsg_to_log_thread)) {
		pr_err("Failed to create kernel thread\n");
	}
}

static int __init kmsg_to_log_init(void)
{
	INIT_DELAYED_WORK(&my_work, real_init);
	schedule_delayed_work(&my_work, msecs_to_jiffies(30000));
	return 0;
}

static void __exit kmsg_to_log_exit(void)
{
	kthread_stop(kmsg_to_log_thread);
	pr_info("Module unloaded\n");
}

module_init(kmsg_to_log_init);
module_exit(kmsg_to_log_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("anonymous");
MODULE_DESCRIPTION("Kernel module to read /dev/kmsg and write to a specified log file");
