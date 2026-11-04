// SPDX-License-Identifier: GPL-2.0
// Single-frame VPR3 prototype driver. Owns AXI DMA MMIO and coherent buffers.
// Do not bind the generic xilinx_dma driver to the same hardware instance.
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/version.h>

#define VPR_TAG 0x56505233
#define VPR_BUF (1920U * 1080U * 4U)
#define VPR_DMA_ERRORS 0x770
struct vpr_request {
    __u32 iw, ih, ow, oh, bypass, core_status, core_errors, input_count;
    __u32 output_count, tx_status, rx_status, rx_bytes;
    __u32 content_size, pad_value;
    __u64 dma_submit_wait_ns;
};
#define VPR_RUN _IOWR('V', 3, struct vpr_request)
struct vpr_device {
    struct device *dev;
    void __iomem *core, *dma;
    void *tx, *rx;
    dma_addr_t tx_dma, rx_dma;
    struct miscdevice misc;
    struct mutex lock;
    atomic_t opened;
};
static int vpr_reset(struct vpr_device *v)
{
    u32 value;
    writel(4, v->dma);
    return readl_poll_timeout(v->dma, value, !(value & 4), 10, 100000);
}
static int vpr_open(struct inode *inode, struct file *file)
{
    struct vpr_device *v = container_of(file->private_data, struct vpr_device, misc);
    if (atomic_cmpxchg(&v->opened, 0, 1)) return -EBUSY;
    file->private_data = v;
    return 0;
}
static int vpr_release(struct inode *inode, struct file *file)
{
    struct vpr_device *v = file->private_data;
    mutex_lock(&v->lock);
    vpr_reset(v); writel(2, v->core);
    mutex_unlock(&v->lock);
    atomic_set(&v->opened, 0);
    return 0;
}
static int vpr_mmap(struct file *file, struct vm_area_struct *area)
{
    struct vpr_device *v = file->private_data;
    unsigned long offset = area->vm_pgoff << PAGE_SHIFT;
    if (area->vm_end - area->vm_start > VPR_BUF) return -EINVAL;
    if (offset != 0 && offset != VPR_BUF) return -EINVAL;
    if (!(area->vm_flags & VM_SHARED)) return -EINVAL;
    area->vm_pgoff = 0;
    vm_flags_set(area, VM_DONTEXPAND | VM_DONTDUMP);
    return dma_mmap_coherent(v->dev, area, offset ? v->rx : v->tx,
                             offset ? v->rx_dma : v->tx_dma, VPR_BUF);
}
static long vpr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct vpr_device *v = file->private_data;
    struct vpr_request r;
    u32 value, input_bytes, output_bytes;
    u64 begin;
    int ret;
    if (cmd != VPR_RUN) return -ENOTTY;
    if (copy_from_user(&r, (void __user *)arg, sizeof(r))) return -EFAULT;
    if (r.iw<16 || r.ih<16 || !r.ow || !r.oh || r.iw > 1920 || r.ih > 1080 ||
        r.ow>1920 || r.oh>1080 || r.bypass > 1 ||
        (r.bypass && (r.ow != r.iw || r.oh != r.ih))) return -EINVAL;
    if (!r.bypass && (r.iw<16 || r.ih<16 || r.ow<16 || r.oh<16 || r.ow>640 || r.oh>640 ||
        (r.content_size & 0xffff)<2 || (r.content_size >> 16)<2 ||
        (r.content_size & 0xffff)>r.ow || (r.content_size >> 16)>r.oh ||
        r.iw>4*(r.content_size & 0xffff) || r.ih>4*(r.content_size >> 16) || r.pad_value>255)) return -EINVAL;
    input_bytes = r.iw * r.ih * 4;
    output_bytes = r.ow * r.oh * (r.bypass ? 4 : 3);
    if (mutex_lock_interruptible(&v->lock)) return -ERESTARTSYS;
    begin = ktime_get_ns();
    ret = vpr_reset(v);
    if (ret) goto out;
    writel(2, v->core); // abort previous frame; new start clears errors
    writel((r.ih << 16) | r.iw, v->core + 0x08);
    writel((r.oh << 16) | r.ow, v->core + 0x0c);
    writel(r.bypass, v->core + 0x10);
    writel(r.content_size, v->core + 0x28);
    writel(r.pad_value, v->core + 0x2c);
    writel(1, v->core);
    // Coherent mappings require ordering, not userspace physical-address guesses.
    dma_wmb();
    // Arm receive before transmit. Interrupts remain disabled: bounded polling.
    writel(1, v->dma + 0x30);
    writel(lower_32_bits(v->rx_dma), v->dma + 0x48);
    writel(upper_32_bits(v->rx_dma), v->dma + 0x4c);
    writel(output_bytes, v->dma + 0x58);
    writel(1, v->dma + 0x00);
    writel(lower_32_bits(v->tx_dma), v->dma + 0x18);
    writel(upper_32_bits(v->tx_dma), v->dma + 0x1c);
    writel(input_bytes, v->dma + 0x28);
    // First establish that the core has consumed/produced the frame. Polling
    // DMA idle before any stream activity could observe the pre-start idle bit.
    ret = readl_poll_timeout(v->core + 0x04, value,
                            (value & 3) == 2, 10, 2000000);
    if (!ret) ret = readl_poll_timeout(v->dma + 0x34, value,
                             (value & 2) || (value & VPR_DMA_ERRORS), 10, 2000000);
    if (!ret) ret = readl_poll_timeout(v->dma + 0x04, value,
                             (value & 2) || (value & VPR_DMA_ERRORS), 10, 2000000);
    dma_rmb();
    r.core_status = readl(v->core + 0x04);
    r.core_errors = readl(v->core + 0x14);
    r.input_count = readl(v->core + 0x18);
    r.output_count = readl(v->core + 0x1c);
    r.tx_status = readl(v->dma + 0x04);
    r.rx_status = readl(v->dma + 0x34);
    r.rx_bytes = readl(v->dma + 0x58);
    if (!ret && (r.core_errors || (r.tx_status & VPR_DMA_ERRORS) ||
        (r.rx_status & VPR_DMA_ERRORS) || r.input_count != r.iw*r.ih ||
        r.output_count != r.ow*r.oh || r.rx_bytes != output_bytes)) ret = -EIO;
    if (ret) { vpr_reset(v); writel(2, v->core); }
    r.dma_submit_wait_ns = ktime_get_ns() - begin;
    if (copy_to_user((void __user *)arg, &r, sizeof(r))) ret = -EFAULT;
out:
    mutex_unlock(&v->lock);
    return ret;
}
static const struct file_operations vpr_fops = {
    .owner=THIS_MODULE, .open=vpr_open, .release=vpr_release,
    /* NULL llseek returns -ESPIPE, including kernels where no_llseek was removed. */
    .mmap=vpr_mmap, .unlocked_ioctl=vpr_ioctl,
};
static int vpr_probe(struct platform_device *pdev)
{
    struct vpr_device *v;
    int ret;
    BUILD_BUG_ON(sizeof(struct vpr_request) != 64);
    v=devm_kzalloc(&pdev->dev,sizeof(*v),GFP_KERNEL);
    if (!v) return -ENOMEM;
    v->dev=&pdev->dev;
    v->core=devm_platform_ioremap_resource_byname(pdev,"core");
    if (IS_ERR(v->core)) return PTR_ERR(v->core);
    v->dma=devm_platform_ioremap_resource_byname(pdev,"dma");
    if (IS_ERR(v->dma)) return PTR_ERR(v->dma);
    if (readl(v->core+0x24)!=VPR_TAG || readl(v->core+0x20)!=0x04380780 || readl(v->core+0x30)!=0x02800280 || readl(v->core+0x34)!=1) return -ENODEV;
    ret=dma_set_mask_and_coherent(v->dev,DMA_BIT_MASK(40));
    if (ret) return ret;
    v->tx=dma_alloc_coherent(v->dev,VPR_BUF,&v->tx_dma,GFP_KERNEL);
    if (!v->tx) return -ENOMEM;
    v->rx=dma_alloc_coherent(v->dev,VPR_BUF,&v->rx_dma,GFP_KERNEL);
    if (!v->rx) {ret=-ENOMEM;goto free_tx;}
    mutex_init(&v->lock);atomic_set(&v->opened,0);
    v->misc.minor=MISC_DYNAMIC_MINOR;v->misc.name="vpr3dma";v->misc.fops=&vpr_fops;
    v->misc.parent=v->dev;v->misc.mode=0600;
    platform_set_drvdata(pdev,v);
    ret=misc_register(&v->misc);
    if (ret) goto free_rx;
    dev_info(v->dev,"VPR3 coherent buffers ready, max input 1920x1080, packed RGB888 output, polling DMA\n");
    return 0;
free_rx:
    dma_free_coherent(v->dev,VPR_BUF,v->rx,v->rx_dma);
free_tx:
    dma_free_coherent(v->dev,VPR_BUF,v->tx,v->tx_dma);
    return ret;
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
static void vpr_remove(struct platform_device *pdev)
#else
static int vpr_remove(struct platform_device *pdev)
#endif
{
    struct vpr_device *v=platform_get_drvdata(pdev);
    misc_deregister(&v->misc);vpr_reset(v);writel(2,v->core);
    dma_free_coherent(v->dev,VPR_BUF,v->rx,v->rx_dma);
    dma_free_coherent(v->dev,VPR_BUF,v->tx,v->tx_dma);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}
static const struct of_device_id vpr_match[]={ {.compatible="prototype,vpr3-dma"},{} };
MODULE_DEVICE_TABLE(of,vpr_match);
static struct platform_driver vpr_driver={.probe=vpr_probe,.remove=vpr_remove,
    .driver={.name="vpr3_dma",.of_match_table=vpr_match,.suppress_bind_attrs=true}};
module_platform_driver(vpr_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KR260 VPR3 bounded single-frame AXI DMA prototype");
