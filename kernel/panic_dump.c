/*
 *  linux/arch/arm/kernel/kpanic.c
 *
 *
 * This file implements the mechanism to dump the printk buffer
 * to the dedicated kpanic flash partition.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/mtd/mtd.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/string.h>

#define MAX_KPANIC_PAGE_SIZE 2048	/* max flash page size */

static int kpanic_in_progress = 0;
static char kpanic_part_name[64] = "Local-disk";//CONFIG_PANIC_DUMP_PARTITION;
static struct mtd_info *kpanic_partition = NULL;

extern char **kpanic_log_buf;
extern int *kpanic_log_buf_len;
extern unsigned long *kpanic_logged_chars;
extern unsigned long *kpanic_log_end;
//extern int kpanic_meminfo_read_proc(char *page, char **start, off_t off,\
                                int count, int *eof, void *data);
int is_kpanic_running(void)
{
	return kpanic_in_progress;
}
EXPORT_SYMBOL(is_kpanic_running);

static int kpanic_initialize(void)
{
	kpanic_partition = get_mtd_device_nm(kpanic_part_name);
	if (kpanic_partition < 0) {
		printk(KERN_EMERG
		       "Couldn't find the kpanic flash partition.\n");
		return -1;
	}
	return 0;
}

static int get_kpanic_partition_size(void)
{
	if (kpanic_partition <= 0)
		return 0;
	return kpanic_partition->size;
}

static int get_kpanic_partition_page_size(void)
{
	if (kpanic_partition <= 0)
		return 0;
	return kpanic_partition->writesize;
}

static int kpanic_erase(void)
{
	struct erase_info erase;
	uint32_t panic_erasesize, erase_number=0;
	if (kpanic_partition <= 0)
		return -1;
	/* set up the erase structure */
	memset(&erase, 0, sizeof(struct erase_info));
	erase.mtd = kpanic_partition;
	erase.addr = 0;
	//erase.len = kpanic_partition->size;
	erase.callback = NULL;
	panic_erasesize = kpanic_partition->erasesize;
	erase.len = panic_erasesize;
	for (; erase_number<kpanic_partition->size;){
	if (kpanic_partition->erase(kpanic_partition, &erase)) {
		printk(KERN_EMERG
		       "Couldn't erase the kpanic flash partition.\n");
	//	return -1;
	}
	erase.addr += panic_erasesize;
	erase_number += panic_erasesize;
	//continue;
	}
	return 0;
}

static int kpanic_read_page(loff_t from, char * buf)
{
	size_t retlen;
	int ret;
	ret = kpanic_partition->read(kpanic_partition, from, kpanic_partition->writesize, 
					&retlen,buf);

	if (ret) {
		printk(KERN_EMERG
		       "Couldn't read the kpanic flash partition.\n");
		return -1;
	}

	return retlen;
}

static int kpanic_read(char *buf, loff_t from, int size)
{
	size_t retlen;
	int ret;
	ret = kpanic_partition->read(kpanic_partition, from, size, &retlen,buf);

	if (ret) {
		printk(KERN_EMERG
		       "Couldn't read the kpanic flash partition.\n");
		return -1;
	}

	return retlen;
}

static int kpanic_write_page(loff_t to, const u_char * buf)
{

	size_t retlen;
	int ret;
	ret = kpanic_partition->write(kpanic_partition, to, kpanic_partition->writesize, 
					&retlen,buf);

	if (ret) {
		printk(KERN_EMERG
		       "Couldn't write to the kpanic flash partition.\n");
		return 1;
	}

	return 0;
}

static void dump_kpanic(char *str)
{

	char *_log_buf = *kpanic_log_buf;
	int _log_buf_len = *kpanic_log_buf_len;
	unsigned long _log_end = *kpanic_log_end;
	unsigned long _logged_chars = *kpanic_logged_chars;
	char tmp_buf[MAX_KPANIC_PAGE_SIZE];
	loff_t flash_write_offset = 0;
	int quotient, remainder, i, j;
	int kpanic_page_size;
	int truncate_flag = 0;
	int orig_log_buf_len = _log_buf_len;
	int kpanic_partition_size;

#define LOG_BUF_MASK (_log_buf_len-1)	/* duplicated from printk.c */
#define LOG_BUF(idx) (_log_buf[(idx) & LOG_BUF_MASK])	/* duplicated from printk.c */

	/* initialize the kpanic partition */
	if (kpanic_initialize() != 0) {
		return;
	}

	/* try to erase the kpanic partition and return if an error occurs */
	if (kpanic_erase() != 0)
		return;

	/* if str != NULL, try to write the passed in string to flash and return */
	if (str) {
		/* purposely ignoring the return value */
		kpanic_write_page(flash_write_offset, str);
		return;
	}

	/* get the size of the kpanic partition */
	kpanic_partition_size = get_kpanic_partition_size();

	/* truncate the printk buffer if it is larger than the kpanic partition */
	if (_log_buf_len > kpanic_partition_size) {
		if (_logged_chars > kpanic_partition_size) {
			_logged_chars = kpanic_partition_size;
			truncate_flag = 1;
		}
		_log_buf_len = kpanic_partition_size;
	}

	/* initialize the temporary buffer */
	memset(tmp_buf, 0, sizeof(tmp_buf));

	/* required for flash alignment */
	kpanic_page_size = get_kpanic_partition_page_size();

	/* the printk circular buffer has not yet wrapped around */
	if (_logged_chars < _log_buf_len) {

		quotient = _logged_chars / kpanic_page_size;
		remainder = _logged_chars % kpanic_page_size;

		for (i = 0; i < quotient; i++) {
			if (kpanic_write_page(flash_write_offset, _log_buf))
				return;
			_log_buf += kpanic_page_size;
			flash_write_offset += kpanic_page_size;
		}

		if (remainder) {
			for (i = 0; i < remainder; i++)
				tmp_buf[i] = *_log_buf++;

			/* pad the buffer with 0xff, to signify un-used flash bytes */
			while (i < kpanic_page_size) {
				tmp_buf[i] = 0xff;
				i++;
			}
			if (kpanic_write_page(flash_write_offset, tmp_buf))
				return;
			/* no need to update flash_write_offset since we are done writing to the flash partition */
		}
	}
	/* once logged_chars equals log_buf_len, it is not incremented further */
	else {

		quotient = _log_buf_len / kpanic_page_size;

		/* 
		 * the number of messages inside the printk buffer (before truncating above) is equal to
		 * the size of the kpanic partition
		 */
		if (!truncate_flag) {
			for (i = 0; i < quotient; i++) {
				for (j = 0; j < kpanic_page_size; j++) {
					tmp_buf[j] = LOG_BUF(_log_end);
					_log_end++;
				}
				if (kpanic_write_page(flash_write_offset, tmp_buf))
					return;
				flash_write_offset += kpanic_page_size;
			}
		}
		/* 
		 * since the number of messages inside the printk buffer is greater than the size of the kpanic
		 * partition, we will dump everything inside the printk buffer beginning with 
		 * (log_end - kpanic_partition_size) and going to (log_end - 1).
		 */
		else {

			/* will be writing pages to flash backwards */
			flash_write_offset = _log_buf_len - kpanic_page_size;
			_log_end--;

			/* needed b/c of macro expansions being used below */
			_log_buf_len = orig_log_buf_len;

			for (i = 0; i < quotient; i++) {
				for (j = kpanic_page_size - 1; j >= 0; j--) {
					tmp_buf[j] = LOG_BUF(_log_end);
					_log_end--;
				}
				if (kpanic_write_page(flash_write_offset, tmp_buf))
					return;
				flash_write_offset -= kpanic_page_size;
			}
		}
	}
}

static int
kpanic_dump_notifier(struct notifier_block *nb, unsigned long l, void *p)
{
	char buf[1024];
	char *panic_msg = (char *)p;
	char *start = buf;
	int buf_len,eof;
	struct timeval rtc_timeval;
	struct rtc_time rtc_timestamp;
	struct timespec uptime;

	/* do not update the kpanic flash partition if a double panic occurs, i.e. 
	   a panic from within a panic context */
	if (!kpanic_in_progress) {
		kpanic_in_progress = 1;

		memset(buf, 0, sizeof(buf));

		do_gettimeofday(&rtc_timeval);
		rtc_time_to_tm((unsigned long)rtc_timeval.tv_sec,
			       &rtc_timestamp);
		do_posix_clock_monotonic_gettime(&uptime);
		/* displays current time (in ISO 8601 format) and uptime (in seconds) */
		buf_len = snprintf(buf, sizeof(buf), "Current Time = "
				   "%d-%02d-%02d %02d:%02d:%02d, Uptime = %lu.%03lu seconds\n",
				   rtc_timestamp.tm_year + 1900,
				   rtc_timestamp.tm_mon + 1,
				   rtc_timestamp.tm_mday, rtc_timestamp.tm_hour,
				   rtc_timestamp.tm_min, rtc_timestamp.tm_sec,
				   (unsigned long)uptime.tv_sec,
				   (unsigned long)(uptime.tv_nsec /
						   USEC_PER_SEC));
		printk(KERN_EMERG "%s", buf);

		/* dump the entire printk buffer to flash */
		if (1) {
			/* dump stack */
			printk(KERN_ERR "stack:\n");
			dump_stack();
			/* displays current memory statistics */
			memset(buf, 0, sizeof(buf));
			//kpanic_meminfo_read_proc(buf, &start, 0, 0, &eof, NULL);
			printk(KERN_EMERG "memory statistic:\n%s\n", buf);

			/* dump the printk log buffer to flash */
			local_irq_disable();
			dump_kpanic(NULL);
			local_irq_enable();
		}
		/* only dump the timestamp and panic string to flash */
		else {
			snprintf(buf + buf_len, sizeof(buf) - buf_len,
				 "Kernel panic - not syncing: %s\n", panic_msg);
			/* dump the panic string to flash */
			local_irq_disable();
			dump_kpanic(buf);
			local_irq_enable();
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block paniced = {
	.notifier_call = kpanic_dump_notifier
};


#ifdef CONFIG_PROC_FS

#define KPANIC_PROC_FILE	"kpanic"
static struct proc_dir_entry *kpanic_proc_file = NULL;

static ssize_t kpanic_proc_read (struct file *filp, char *buf, size_t len, loff_t *off)
{
	int kpanic_partition_size;
	int ret;
	char *read_buf;

	/* initialize the kpanic partition */
	if (kpanic_initialize() != 0) {
		return 0;
	}
	/* get the size of the kpanic partition */
	kpanic_partition_size = get_kpanic_partition_size();

	if (*off >= kpanic_partition_size) {
		return 0;
	}
	if (*off+len > kpanic_partition_size) {
		len = kpanic_partition_size - *off;
	}
	
	read_buf=(char*)kzalloc(len,GFP_KERNEL);
	if (!read_buf) {
		printk(KERN_ERR"%s: Can not allocate memory.\n",__FUNCTION__);
		return -1;
	}

	ret = kpanic_read(read_buf,*off,len);
	if (ret >= 0) {
		if (*read_buf == 0xFF && *(read_buf+1) == 0xFF)
			return 0;
		if (copy_to_user(buf, read_buf, ret))
			printk("kpanic_proc_read: copy_to_user failed\n");
		*off += ret;
	} else
		printk(KERN_NOTICE"%s: read panic info error\n",__FUNCTION__);

	kfree(read_buf);
	return ret;
}

static int kpanic_proc_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	char messages[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buf, len))
		return -EFAULT;

	if (!strncmp(messages, "--clean", 7)) {
		kpanic_erase();
	} else if (!strncmp(messages, "--test", 6)) {
		panic(messages+7);
	} else 
		printk(KERN_WARNING "bad parameter.\n");

	return len;
}

static struct file_operations kpanic_proc_ops = {
	.read  = kpanic_proc_read,
	.write = kpanic_proc_write,
};

static void create_kpanic_proc_file(void)
{
	kpanic_proc_file = create_proc_entry(KPANIC_PROC_FILE, 0644, NULL);
	if (kpanic_proc_file) {
		//kpanic_proc_file->owner = THIS_MODULE;
		kpanic_proc_file->proc_fops = &kpanic_proc_ops;
	} else
		printk(KERN_INFO "kpanic: proc file create failed!\n");
}
static void remove_kpanic_proc_file(void)
{
	remove_proc_entry(KPANIC_PROC_FILE, NULL);
}
#endif

static void __exit kpanic_exit(void)
{
#ifdef	CONFIG_PROC_FS
	remove_kpanic_proc_file();
#endif
	return;
}

static int __init kpanic_init(void)
{

	int ret = 0;

	ret = atomic_notifier_chain_register(&panic_notifier_list, &paniced);
	if (ret) {
		printk(KERN_ERR "kpanic: module register failed\n");
		return ret;
	}

	/* register proc fs for debug */
#ifdef	CONFIG_PROC_FS
	create_kpanic_proc_file();
#endif
	return ret;
}

module_init(kpanic_init);
module_exit(kpanic_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Dump kernel panic information to MTD device.");
