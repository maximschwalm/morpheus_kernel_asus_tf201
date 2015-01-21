#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>
#include <asm/types.h>
#include <asm/page.h>

struct buffer {
	size_t size;
	char data[];
};

static struct buffer* atags_buffer = NULL;

static ssize_t atags_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	// These are introduced in kernel 3.10. I don't want to backport
	// the whole chunk, and other things (ram_console) use static
	// variable to keep data too, so I guess it's okay.
	//struct buffer *b = PDE_DATA(file_inode(file));
	struct buffer *b = atags_buffer;
	return simple_read_from_buffer(buf, count, ppos, b->data, b->size);
}

static const struct file_operations atags_fops = {
	.read = atags_read,
	.llseek = default_llseek,
};

#define BOOT_PARAMS_SIZE 1536
static char __initdata atags_copy[BOOT_PARAMS_SIZE];

void __init save_atags(const struct tag *tags)
{
	memcpy(atags_copy, tags, sizeof(atags_copy));
}

static int __init init_atags_procfs(void)
{
	/*
	 * This cannot go into save_atags() because kmalloc and proc don't work
	 * yet when it is called.
	 */
	struct proc_dir_entry *tags_entry;
	struct tag *tag = (struct tag *)atags_copy;
	struct buffer *b;
	size_t size;

	if (tag->hdr.tag != ATAG_CORE) {
		printk(KERN_INFO "No ATAGs?");
		return -EINVAL;
	}

	for (; tag->hdr.size; tag = tag_next(tag))
		;

	/* include the terminating ATAG_NONE */
	size = (char *)tag - atags_copy + sizeof(struct tag_header);

	WARN_ON(tag->hdr.tag != ATAG_NONE);

	b = kmalloc(sizeof(*b) + size, GFP_KERNEL);
	if (!b)
		goto nomem;

	b->size = size;
	memcpy(b->data, atags_copy, size);

	tags_entry = proc_create_data("atags", 0400, NULL, &atags_fops, b);
	if (!tags_entry)
		goto nomem;

	atags_buffer = b;

	return 0;

nomem:
	kfree(b);
	printk(KERN_ERR "Exporting ATAGs: not enough memory\n");

	return -ENOMEM;
}
arch_initcall(init_atags_procfs);
