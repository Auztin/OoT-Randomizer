#include "usb.h"
#include "z64.h"
#include <n64.h>
#include <stdbool.h>

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t

#define s8 int8_t
#define s16 int16_t
#define s32 int32_t
#define s64 int64_t

#define vu8 volatile uint8_t
#define vu16 volatile uint16_t
#define vu32 volatile uint32_t
#define vu64 volatile uint64_t

#define vs8 volatile int8_t
#define vs16 volatile int16_t
#define vs32 volatile int32_t
#define vs64 volatile int64_t

#define f32 float
#define f64 double

#define MI_MASK_CLR_SP      0x0001
#define MI_MASK_CLR_SI      0x0004
#define MI_MASK_CLR_AI      0x0010
#define MI_MASK_CLR_VI      0x0040
#define MI_MASK_CLR_PI      0x0100
#define MI_MASK_CLR_DP      0x0400
#define C0_STATUS_IE        0x00000001    ///< Status: interrupt enable
#define C0_INTERRUPT_RCP    0x00000400  ///< Status/Cause: HW interrupt 2 (RCP)
#define PI_BASE_REG         0x04600000
#define PI_STATUS_REG       (PI_BASE_REG+0x10)
#define MEMORY_BARRIER()    asm volatile ("" : : : "memory")
#define TICKS_READ() ({ \
  uint32_t x; \
  asm volatile("mfc0 %0,$9":"=r"(x)); \
  x; \
})
#define C0_STATUS() ({ \
  uint32_t x; \
  asm volatile("mfc0 %0,$12":"=r"(x)); \
  x; \
})
#define C0_WRITE_STATUS(x) ({ \
  asm volatile("mtc0 %0,$12"::"r"(x)); \
})
#define cache_op(op, linesize) ({ \
  if (length) { \
    void *cur = (void*)((unsigned long)addr & ~(linesize-1)); \
    int count = (int)length + (addr-cur); \
    for (int i = 0; i < count; i += linesize) \
      asm ("\tcache %0,(%1)\n"::"i" (op), "r" (cur+i)); \
  } \
})

typedef struct MI_regs_s {
  /** @brief Mode register */
  uint32_t mode;
  /** @brief Version register */
  uint32_t version;
  /** @brief Current interrupts on the system */
  uint32_t intr;
  /** @brief Interrupt mask */
  uint32_t mask;
} MI_regs_t;

static int __interrupt_depth = -1;
static int __interrupt_sr = 0;
uint32_t interrupt_disabled_tick = 0;
static volatile struct MI_regs_s * const MI_regs = (struct MI_regs_s *)0xa4300000;

void data_cache_hit_writeback(volatile const void * addr, unsigned long length) {
  cache_op(0x19, 16);
}

void data_cache_hit_writeback_invalidate(volatile void * addr, unsigned long length) {
  cache_op(0x15, 16);
}

__attribute__((constructor)) void __init_interrupts()
{
  /* Make sure that we aren't initializing interrupts when they are already enabled */
  if( __interrupt_depth < 0 )
  {
    /* Clear and mask all interrupts on the system so we start with a clean slate */
    MI_regs->mask=MI_MASK_CLR_SP|MI_MASK_CLR_SI|MI_MASK_CLR_AI|MI_MASK_CLR_VI|MI_MASK_CLR_PI|MI_MASK_CLR_DP;

    /* Set that we are enabled */
    __interrupt_depth = 0;

    /* Enable interrupts systemwide. We set the global interrupt enable,
       and then specifically enable RCP interrupts. */
    uint32_t sr = C0_STATUS();
    C0_WRITE_STATUS(sr | C0_STATUS_IE | C0_INTERRUPT_RCP);
  }
}

void disable_interrupts()
{
  /* Don't do anything if we haven't initialized */
  if( __interrupt_depth < 0 ) { return; }

  if( __interrupt_depth == 0 )
  {
    /* We must disable the interrupts now. */
    uint32_t sr = C0_STATUS();
    C0_WRITE_STATUS(sr & ~C0_STATUS_IE);

    /* Save the original SR value away, so that we now if
       interrupts were enabled and whether to restore them.
       NOTE: this memory write must happen now that interrupts
       are disabled, otherwise it could cause a race condition
       because an interrupt could trigger and overwrite it.
       So put an explicit barrier. */
    MEMORY_BARRIER();
    __interrupt_sr = sr;

    interrupt_disabled_tick = TICKS_READ();
  }

  /* Ensure that we remember nesting levels */
  __interrupt_depth++;
}

void enable_interrupts()
{
  /* Don't do anything if we've hosed up or aren't initialized */
  if( __interrupt_depth < 0 ) { return; }

  /* Check that we're not calling enable_interrupts() more than expected */
  // assertf(__interrupt_depth > 0, "unbalanced enable_interrupts() call");

  /* Decrement the nesting level now that we are enabling interrupts */
  __interrupt_depth--;

  if( __interrupt_depth == 0 )
  {
    /* Restore the interrupt state that was active when interrupts got
       disabled. This is important because, within an interrupt handler,
       we don't want here to force-enable interrupts, or we would allow
       reentrant interrupts which are not supported. */
    C0_WRITE_STATUS(C0_STATUS() | (__interrupt_sr & C0_STATUS_IE));
  }
}

static const char _itoa_lower_digits[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 'a'+0x64, 'b'+0x64, 'c'+0x64, 'd'+0x64, 'e'+0x64, 'f'+0x64};
static const char _itoa_upper_digits[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 'A'+0x6A, 'B'+0x6A, 'C'+0x6A, 'D'+0x6A, 'E'+0x6A, 'F'+0x6A};

static inline char* __attribute__ ((unused, always_inline)) _itoa(u32 value, char* buflim, u8 base, u8 upper_case) {
  const char* digits = (upper_case ? _itoa_upper_digits : _itoa_lower_digits);
  do *--buflim = digits[value % base];
  while ((value /= base) != 0);
  return buflim;
}

#define USB_CAN_READ  0x1
#define USB_CAN_WRITE 0x2

/*********************************
          64Drive macros
*********************************/

// How many cycles for the 64Drive to wait for data.
// Lowering this might improve performance slightly faster at the expense of USB reading accuracy
#define D64_POLLTIME       2000

// Cartridge Interface definitions. Obtained from 64Drive's Spec Sheet
#define D64_BASE_ADDRESS   0xB0000000
#define D64_CIREG_ADDRESS  0x08000000
#define D64_CIBASE_ADDRESS 0xB8000000

#define D64_DEBUG_AREA_SIZE 1024
#define D64_DEBUG_AREA      (0x04000000-D64_DEBUG_AREA_SIZE)

#define D64_REGISTER_STATUS  0x00000200
#define D64_REGISTER_COMMAND 0x00000208
#define D64_REGISTER_LBA     0x00000210
#define D64_REGISTER_LENGTH  0x00000218
#define D64_REGISTER_RESULT  0x00000220

#define D64_REGISTER_MAGIC    0x000002EC
#define D64_REGISTER_VARIANT  0x000002F0
#define D64_REGISTER_BUTTON   0x000002F8
#define D64_REGISTER_REVISION 0x000002FC

#define D64_REGISTER_USBCOMSTAT 0x00000400
#define D64_REGISTER_USBP0R0    0x00000404
#define D64_REGISTER_USBP1R1    0x00000408

#define D64_ENABLE_ROMWR    0xF0
#define D64_DISABLE_ROMWR   0xF1
#define D64_COMMAND_WRITE   0x08
#define D64_ENABLE_SAVE     0xD2

// Cartridge Interface return values
#define D64_MAGIC    0x55444556

#define D64_USB_IDLE        0x00
#define D64_USB_IDLEUNARMED 0x00
#define D64_USB_ARMED       0x01
#define D64_USB_DATA        0x02
#define D64_USB_ARM         0x0A
#define D64_USB_BUSY        0x0F
#define D64_USB_DISARM      0x0F
#define D64_USB_ARMING      0x0F

#define D64_CI_IDLE  0x00
#define D64_CI_BUSY  0x10
#define D64_CI_WRITE 0x20


/*********************************
         EverDrive macros
*********************************/

#define ED_BASE           0x10000000
#define ED_BASE_ADDRESS   0x1F800000
#define ED_GET_REGADD(reg)   (0xA0000000 | ED_BASE_ADDRESS | (reg))

#define ED_REG_USBCFG  0x0004
#define ED_REG_VERSION 0x0014
#define ED_REG_USBDAT  0x0400
#define ED_REG_SYSCFG  0x8000
#define ED_REG_KEY     0x8004

#define ED_USBMODE_RDNOP 0xC400
#define ED_USBMODE_RD    0xC600
#define ED_USBMODE_WRNOP 0xC000
#define ED_USBMODE_WR    0xC200

#define ED_USBSTAT_ACT   0x0200
#define ED_USBSTAT_RXF   0x0400
#define ED_USBSTAT_TXE   0x0800
#define ED_USBSTAT_POWER 0x1000
#define ED_USBSTAT_BUSY  0x2000

#define ED_REGKEY  0xAA55

#define ED3_VERSION 0xED640008
#define ED7_VERSION 0xED640013


/*********************************
       SummerCart64 macros
*********************************/

#define SC64_SDRAM_BASE             0x10000000

#define SC64_BANK_ROM               1

#define SC64_REGS_BASE              0x1E000000
#define SC64_REG_SCR                (SC64_REGS_BASE + 0x00)
#define SC64_REG_VERSION            (SC64_REGS_BASE + 0x08)
#define SC64_REG_USB_SCR            (SC64_REGS_BASE + 0x10)
#define SC64_REG_USB_DMA_ADDR       (SC64_REGS_BASE + 0x14)
#define SC64_REG_USB_DMA_LEN        (SC64_REGS_BASE + 0x18)

#define SC64_MEM_BASE               (SC64_REGS_BASE + 0x1000)
#define SC64_MEM_USB_FIFO_BASE      (SC64_MEM_BASE + 0x0000)
#define SC64_MEM_USB_FIFO_LEN       (4 * 1024)

#define SC64_SCR_SDRAM_WRITE_EN     (1 << 0)

#define SC64_VERSION_A              0x53363461

#define SC64_USB_STATUS_BUSY        (1 << 0)
#define SC64_USB_STATUS_READY       (1 << 1)
#define SC64_USB_CONTROL_START      (1 << 0)
#define SC64_USB_CONTROL_FIFO_FLUSH (1 << 2)

#define SC64_USB_BANK_ADDR(b, a)    ((((b) & 0xF) << 28) | ((a) & 0x3FFFFFF))
#define SC64_USB_LENGTH(l)          (ALIGN((l), 4) / 4)
#define SC64_USB_DMA_MAX_LEN        (2 * 1024 * 1024)
#define SC64_USB_FIFO_ITEMS(s)      (((s) >> 3) & 0x7FF)

#define CART_NONE      0
#define CART_64DRIVE   1
#define CART_EVERDRIVE 2
#define CART_SC64      3

u8 USB_INITIALIZED = 0;
u8 USB_BUFFER[512];
u32 USB_SIZE = 0;

void dma_wait() {
  while (pi_regs.status & (PI_STATUS_DMA_BUSY | PI_STATUS_IO_BUSY));
}

void dma_read(void* ram, u32 pi_address, u32 len) {
  pi_address &= 0x1FFFFFFF;
  data_cache_hit_writeback_invalidate(ram, len);
  disable_interrupts();
  dma_wait();
  MEMORY_BARRIER();
  pi_regs.dram_addr = (u32)ram;
  MEMORY_BARRIER();
  pi_regs.cart_addr = pi_address;
  MEMORY_BARRIER();
  pi_regs.wr_len = len - 1;
  MEMORY_BARRIER();
  dma_wait();
  MEMORY_BARRIER();
  pi_regs.status = PI_STATUS_CLR_INTR;
  MEMORY_BARRIER();
  enable_interrupts();
}

void dma_write(void* ram, u32 pi_address, u32 len) {
  pi_address &= 0x1FFFFFFF;
  data_cache_hit_writeback(ram, len);
  disable_interrupts();
  dma_wait();
  MEMORY_BARRIER();
  pi_regs.dram_addr = (u32)ram;
  MEMORY_BARRIER();
  pi_regs.cart_addr = pi_address;
  MEMORY_BARRIER();
  pi_regs.rd_len = len - 1;
  MEMORY_BARRIER();
  dma_wait();
  MEMORY_BARRIER();
  pi_regs.status = PI_STATUS_CLR_INTR;
  MEMORY_BARRIER();
  enable_interrupts();
}

u32 io_read(u32 pi_address) {
  u32 retval;
  data_cache_hit_writeback_invalidate(&retval, 4);
  disable_interrupts();
  pi_address |= 0xA0000000;
  dma_wait();
  MEMORY_BARRIER();
  retval = *(vu32*)pi_address;
  enable_interrupts();
  return retval;
}

void io_write(u32 pi_address, u32 data) {
  vu32 *uncached_address = (u32*)(pi_address | 0xA0000000);
  data_cache_hit_writeback(&data, 4);
  disable_interrupts();
  dma_wait();
  MEMORY_BARRIER();
  *uncached_address = data;
  MEMORY_BARRIER();
  enable_interrupts();
}

static u8 usb_64drive_ci_status() {
  return (io_read(D64_CIBASE_ADDRESS + D64_REGISTER_STATUS) >> 8) & 0x10;
}

static void usb_64drive_arm();

static u8 usb_64drive_can_read() {
  u32 status = io_read(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT) & 0xf;
  if (status == D64_USB_DATA) {
    USB_INITIALIZED = 1;
    return 1;
  }
  if (USB_INITIALIZED && status == D64_USB_IDLEUNARMED) usb_64drive_arm();
  return 0;
}

static u8 usb_64drive_can_write() {
  // if (!USB_INITIALIZED) return 0;
  if (((io_read(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT) >> 4) & 0xf) == D64_USB_IDLE) return 1;
  return 0;
}

static void usb_64drive_arm() {
	if (!usb_64drive_can_write()) return;
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBP0R0, (D64_DEBUG_AREA) >> 1);
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBP1R1, (D64_DEBUG_AREA_SIZE & 0xffffff));
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT, D64_USB_ARM);
}

static void usb_64drive_disarm() {
  u32 status = io_read(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT) & 0xf;
  if (status != D64_USB_ARMED && status != D64_USB_ARMING) return;
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT, D64_USB_DISARM);
  u32 timeout = 0;
  while (timeout++ != 8192) {
    status = io_read(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT) & 0xf;
    if (status != D64_USB_ARMED && status != D64_USB_ARMING) break;
  }
}

static void usb_64drive_writable(u8 enable) {
  while (usb_64drive_ci_status());
  io_write(D64_CIBASE_ADDRESS + D64_REGISTER_COMMAND, enable ? D64_ENABLE_ROMWR : D64_DISABLE_ROMWR);
  while (usb_64drive_ci_status());
}

static u8 usb_64drive_write(void* data, u32 len) {
  if (!len) return 0;
  // len = len + (16 - len % 16);
  usb_64drive_disarm();
  usb_64drive_writable(1);
	dma_write(data, D64_BASE_ADDRESS + D64_DEBUG_AREA, len);
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBP0R0, (D64_DEBUG_AREA) >> 1);
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBP1R1, (len & 0xffffff) | (0x1 << 24));
	io_write(D64_CIBASE_ADDRESS + D64_REGISTER_USBCOMSTAT, D64_COMMAND_WRITE);
  usb_64drive_writable(0);
  u32 timeout = 0;
  while (!usb_64drive_can_write()) {
    if (timeout++ != 8192) continue;
    return 1;
  }
  return 0;
}

static u8 usb_64drive_read() {
  USB_SIZE = io_read(D64_CIBASE_ADDRESS + D64_REGISTER_USBP0R0) & 0xffffff;
  if (!USB_SIZE) {
    usb_64drive_arm();
    return 0;
  }
  if (USB_SIZE > 512) {
    usb_64drive_arm();
    return 1;
  }
  dma_read(USB_BUFFER, D64_BASE_ADDRESS + D64_DEBUG_AREA, USB_SIZE);
  usb_64drive_arm();
  return 0;
}

static u8 usb_64drive_poll() {
  u8 ret = 0;
  if (usb_64drive_ci_status()) return ret;
  if (usb_64drive_can_read()) ret |= USB_CAN_READ;
  if (usb_64drive_can_write()) ret |= USB_CAN_WRITE;
  return ret;
}

//

static u8 usb_everdrive_can_read() {
  if ((io_read(ED_GET_REGADD(ED_REG_USBCFG)) & (ED_USBSTAT_POWER | ED_USBSTAT_RXF)) == ED_USBSTAT_POWER) return 1;
  return 0;
}

static u8 usb_everdrive_can_write() {
  // if (!USB_INITIALIZED) return 0;
  if ((io_read(ED_GET_REGADD(ED_REG_USBCFG)) & (ED_USBSTAT_POWER | ED_USBSTAT_TXE)) == ED_USBSTAT_POWER) return 1;
  return 0;
}

static u8 usb_everdrive_busy() {
  u32 timeout = 0;
  while ((io_read(ED_GET_REGADD(ED_REG_USBCFG)) & ED_USBSTAT_ACT) != 0) {
    if (timeout++ != 8192) continue;
    io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_RDNOP);
    return 1;
  }
  return 0;
}

// static u8 usb_everdrive_write(void* data, u32 len) {
//   u16 blen, baddr;
//   io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_WRNOP);
//   while (len) {
//     blen = 512;
//     if (blen > len) blen = len;
//     baddr = 512 - blen;
//     dma_write(data, ED_GET_REGADD(ED_REG_USBDAT + baddr), blen);
//     data += 512;
//     io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_WR | baddr);
//     if (usb_everdrive_busy()) return 1;
//     len -= blen;
//   }
//   return 0;
// }

// static u8 usb_everdrive_read(void* data, u32 len) {
//   u16 blen, baddr;
//   while (len) {
//     blen = 512;
//     if (blen > len) blen = len;
//     baddr = 512 - blen;
//     io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_RD | baddr);
//     if (usb_everdrive_busy()) return 1;
//     dma_read(data, ED_GET_REGADD(ED_REG_USBDAT + baddr), blen);
//     data += blen;
//     len -= blen;
//   }
//   return 0;
// }

static u8 usb_everdrive_write_data(void* data, u32 len) {
  u16 blen, baddr;
  io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_WRNOP);
  while (len) {
    blen = 512;
    if (blen > len)blen = len;
    baddr = 512 - blen;
    dma_write(data, ED_GET_REGADD(ED_REG_USBDAT + baddr), blen);
    data += 512;
    io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_WR | baddr);
    if (usb_everdrive_busy()) {
      return 1;
    }
    len -= blen;
  }
  return 0;
}

static u8 usb_everdrive_write(void* data, u32 len) {
  if (!len) return 0;
  len &= 0xFFFFFF;
  u32 bsize = 8+len+4;
  u8 data2[bsize];
  data2[0] = 'D';
  data2[1] = 'M';
  data2[2] = 'A';
  data2[3] = '@';
  data2[4] = 0x01;
  data2[5] = (len >> 16) & 0xFF;
  data2[6] = (len >> 8)  & 0xFF;
  data2[7] = len & 0xFF;
  for (u32 i = 0; i < len; i++) data2[i+8] = ((u8*)data)[i];
  data2[bsize-4] = 'C';
  data2[bsize-3] = 'M';
  data2[bsize-2] = 'P';
  data2[bsize-1] = 'H';
  return usb_everdrive_write_data(data2, bsize);
}

static u8 usb_everdrive_read_data(void* data, u32 len) {
  u16 blen, baddr;
  while (len) {
    blen = 512;
    if (blen > len) blen = len;
    baddr = 512 - blen;
    io_write(ED_GET_REGADD(ED_REG_USBCFG), ED_USBMODE_RD | baddr);
    if (usb_everdrive_busy()) return 1;
    dma_read(data, ED_GET_REGADD(ED_REG_USBDAT + baddr), blen);
    data += blen;
    len -= blen;
  }
  return 0;
}

static u8 usb_everdrive_read() {
  u8 header[16];
  if (usb_everdrive_read_data(header, 16)) return 1;
  if (header[0] != '@' || header[1] != 'C' || header[2] != 'M' || header[3] != 'D') return 1;
  USB_SIZE = (header[5] << 16) | (header[6] << 8) | header[7];
  if (USB_SIZE > 8 && (USB_SIZE > 512 || usb_everdrive_read_data(USB_BUFFER + 8, USB_SIZE - 8))) return 1;
  for (u8 i = 0; i < 8; i++) USB_BUFFER[i] = header[i+8];
  return 0;
}

static u8 usb_everdrive_poll() {
  u8 ret = 0;
  if (usb_everdrive_can_read()) ret |= USB_CAN_READ;
  if (usb_everdrive_can_write()) ret |= USB_CAN_WRITE;
  return ret;
}

static s8 usb_cart = CART_NONE;
u8 (*usb_writePointer)(void*, u32);
u8 (*usb_readPointer)();
u8 (*usb_pollPointer)();

u8 usb_write(void* data, u32 len) {
  if (usb_cart == CART_NONE) return 1;
  return usb_writePointer(data, len);
}

u8 usb_read() {
  if (usb_cart == CART_NONE) return 1;
  return usb_readPointer();
}

u8 usb_poll() {
  if (usb_cart == CART_NONE) return 0;
  return usb_pollPointer();
}

static void usb_findcart() {
  if (!io_read(0xA4100010)) return;
  u32 buff __attribute__((aligned(8)));
  buff = io_read(D64_CIBASE_ADDRESS + D64_REGISTER_MAGIC);
  if (buff == D64_MAGIC) {
    usb_cart = CART_64DRIVE;
    return;
  }
  // buff = io_read(SC64_REG_VERSION);
  // if (buff == SC64_VERSION_A) {
  //   usb_cart = CART_SC64;
  //   return;
  // }
  io_write(ED_GET_REGADD(ED_REG_KEY), ED_REGKEY);
  buff = io_read(ED_GET_REGADD(ED_REG_VERSION));
  if (buff == ED7_VERSION || buff == ED3_VERSION) {
    usb_cart = CART_EVERDRIVE;
    return;
  }
}

struct {
  u32 initialized : 1;
  u32 tracker_initialized : 1;
  u32 vars_init : 1;
  u32 packet_received : 1;
  u32 pause_writes : 3;
  u32 received_item : 5;
  u32 usb_timer : 5;
  u32 tracker_timeout : 5;
  u32 settings : 8;
} usb_vars = {0, };

enum USB_COMMANDS {
  USB_CMD_NONE,
  USB_CMD_INIT,
  USB_CMD_OUTGOING,
  USB_CMD_INCOMING,
  USB_CMD_PLAYER,
  USB_CMD_SETTINGS,
  USB_CMD_TRACKER,
  USB_CMD_PAUSE_WRITES,
  USB_CMD_PACKET_RECEIVED,
};

enum SETTINGS {
  SETTING_ANTIALIAS = 0x01,
};

#define AA1 (*(vu32*)0x8000646C)
#define AA2 (*(vu32*)0x8000649C)
#define CRC1 (*(vu32*)0xB0000010)
#define CRC2 (*(vu32*)0xB0000014)
#define MAGIC_CRC(array)  array[0] = (CRC1 & 0xFF000000) >> 24; \
                          array[1] = (CRC1 & 0x00FF0000) >> 16; \
                          array[2] = (CRC1 & 0x0000FF00) >> 8;  \
                          array[3] = (CRC1 & 0x000000FF);       \
                          array[4] = (CRC2 & 0xFF000000) >> 24; \
                          array[5] = (CRC2 & 0x00FF0000) >> 16; \
                          array[6] = (CRC2 & 0x0000FF00) >> 8;  \
                          array[7] = (CRC2 & 0x000000FF);
#define MAGIC(array)  {0, };MAGIC_CRC(array)
#define INTERNAL_COUNT (*(vu16*)(z64_file_addr + 0x90))
#define USB_VERSION 1;
extern u8 PLAYER_ID;
extern u16 INCOMING_PLAYER;
extern u16 INCOMING_ITEM;
extern u32 OUTGOING_KEY;
extern u16 OUTGOING_ITEM;
extern u16 OUTGOING_PLAYER;
extern u8 PLAYER_NAMES[];

void usb_initialize() {
  usb_vars.initialized = true;
  __init_interrupts();
  usb_findcart();
  // usb_cart = CART_EVERDRIVE;
  switch (usb_cart) {
    case CART_64DRIVE:
      usb_writePointer = usb_64drive_write;
      usb_pollPointer  = usb_64drive_poll;
      usb_readPointer  = usb_64drive_read;
      break;
    case CART_EVERDRIVE:
      usb_writePointer = usb_everdrive_write;
      usb_pollPointer = usb_everdrive_poll;
      usb_readPointer = usb_everdrive_read;
      break;
    // case CART_SC64:
    //   funcPointer_write = usb_sc64_write;
    //   funcPointer_poll  = usb_sc64_poll;
    //   funcPointer_read  = usb_sc64_read;
    //   break;
  }
  u8 num[4] = {0, };
  u8 digit = 0;
  for (u16 i = 0; i < 256; i++) {
    PLAYER_NAMES[i*8] = 'P'+0x6A;
    _itoa(i, num+4, 10, 0);
    for (digit = 0; digit < 4; digit++) if (num[digit]) break;
    for (u8 c = 1; c < 8; c++) {
      if (digit < 4) PLAYER_NAMES[i*8+c] = num[digit++]-1;
      else PLAYER_NAMES[i*8+c] = 0xDF;
    }
  }
}

struct tracker_t {
  bool mountain_c_biggoron : 1;
  bool gerudo_c_guard_2 : 1;
  bool gerudo_c_guard_1 : 1;
  bool gerudo_c_guard_3 : 1;
  bool gerudo_c_guard_4 : 1;
  bool hyrule_field_c_hyrule_deku_salesman_grotto : 1;
  bool river_c_zora_grotto_salesman_1 : 1;
  bool river_c_zora_grotto_salesman_2 : 1;

  bool woods_c_deku_salesman_meadow_grotto_1 : 1;
  bool woods_c_deku_salesman_meadow_grotto_2 : 1;
  bool lake_c_grave_grotto_salesman_1 : 1;
  bool lake_c_grave_grotto_salesman_2 : 1;
  bool lake_c_grave_grotto_salesman_3 : 1;
  bool gerudo_valley_c_gerudo_behind_carpenter_tent_grotto_salesman_1 : 1;
  bool gerudo_valley_c_gerudo_behind_carpenter_tent_grotto_salesman_2 : 1;
  bool woods_c_deku_salesman_grotto : 1;

  bool woods_c_deku_salesman_grotto_2 : 1;
  bool mountain_c_crater_grotto_salesman_1 : 1;
  bool mountain_c_crater_grotto_salesman_2 : 1;
  bool mountain_c_crater_grotto_salesman_3 : 1;
  bool goron_c_grotto_salesman_1 : 1;
  bool goron_c_grotto_salesman_2 : 1;
  bool goron_c_grotto_salesman_3 : 1;
  bool lonlon_c_grotto_deku_salesman_1 : 1;

  bool lonlon_c_grotto_deku_salesman_2 : 1;
  bool lonlon_c_grotto_deku_salesman_3 : 1;
  bool desert_c_colossus_grotto_salesman_1 : 1;
  bool desert_c_colossus_grotto_salesman_2 : 1;
  bool kokiri_c_midos_house_1 : 1;
  bool kokiri_c_midos_house_2 : 1;
  bool kokiri_c_midos_house_3 : 1;
  bool kokiri_c_midos_house_4 : 1;

  bool kokiri_c_links_house_cow : 1;
  bool lonlon_c_cow_stables_1 : 1;
  bool lonlon_c_cow_stables_2 : 1;
  bool kakariko_c_impas_cow : 1;
  bool kakariko_c_cow_hp : 1;
  bool mountain_c_summit_fairy : 1;
  bool mountain_c_crater_fairy : 1;
  bool castle_town_c_ganon_fairy : 1;

  bool mountain_c_crater_grotto : 1;
  bool woods_c_wolfos_grotto : 1;
  bool woods_c_bomb_grotto : 1;
  bool mountain_c_outside_goron_grotto : 1;
  bool kakariko_c_grotto : 1;
  bool river_c_zora_grotto : 1;
  bool kakariko_c_redead_grotto : 1;
  bool kokiri_c_storm_grotto : 1;

  bool hyrule_field_c_hyrule_north_grotto : 1;
  bool hyrule_field_c_hyrule_forest_grotto : 1;
  bool hyrule_field_c_hyrule_south_grotto : 1;
  bool mountain_c_grotto_cow : 1;
  bool hyrule_field_c_hyrule_west_grotto_cow : 1;
  bool hyrule_field_c_diving_heart_piece_grotto : 1;
  bool kakariko_c_redead_grave : 1;
  bool kakariko_c_shield_grave : 1;

  bool kakariko_c_sun_song_chest : 1;
  bool kakariko_c_race_1 : 1;
  bool kakariko_c_windmill_hp : 1;
  bool kakariko_c_race_2 : 1;
  bool lonlon_c_cow_tower_2 : 1;
  bool lonlon_c_cow_tower_1 : 1;
  bool lonlon_c_hp : 1;
  bool kakariko_c_digging : 1;

  bool kakariko_c_bean_hp : 1;
  bool river_c_zora_hp_2 : 1;
  bool river_c_zora_bean_salesman : 1;
  bool river_c_zora_hp_1 : 1;
  bool kokiri_c_sword : 1;
  bool lake_c_sun : 1;
  bool lake_c_lab_roof : 1;
  bool zoras_c_torch : 1;

  bool zoras_c_underwater : 1;
  bool zoras_c_ice : 1;
  bool gerudo_valley_c_gerudo_hammer_rock : 1;
  bool gerudo_valley_c_gerudo_river_cow : 1;
  bool gerudo_valley_c_gerudo_waterfall : 1;
  bool gerudo_valley_c_gerudo_crate : 1;
  bool woods_c_deku_salesman : 1;
  bool woods_c_deku_salesman_theater_2 : 1;

  bool woods_c_deku_salesman_theater_1 : 1;
  bool temple_spirit_c_shield : 1;
  bool temple_spirit_mq_c_mirror_shield : 1;
  bool temple_spirit_c_gauntlets : 1;
  bool temple_spirit_mq_c_gauntlets : 1;
  bool desert_c_colossus_hp : 1;
  bool gerudo_c_rooftop : 1;
  bool desert_c_chest : 1;

  bool desert_c_bombchu_wasteland : 1;
  bool mountain_c_outside_goron_chest : 1;
  bool mountain_c_above_dodongo : 1;
  bool mountain_c_magic_bean : 1;
  bool mountain_c_crater_wall : 1;
  bool mountain_c_crater_ladder_salesman : 1;
  bool goron_c_maze_left : 1;
  bool goron_c_maze_right : 1;

  bool goron_c_maze_center : 1;
  bool goron_c_spinning_pot_hp : 1;
  bool goron_c_medigoron : 1;
  bool lonlon_s_wall_child_night : 1;
  bool lonlon_s_paddock_fence_child_night : 1;
  bool lonlon_s_window_child_night : 1;
  bool lonlon_s_tree_child_night : 1;
  bool mountain_s_crater_beanspot_child : 1;

  bool mountain_s_cave_entrance_beanspot_child : 1;
  bool mountain_s_bombable : 1;
  bool mountain_s_before_goron_hammer_rock_adult_night : 1;
  bool mountain_s_wall_hammer_rock_adult_night : 1;
  bool goron_s_center_platform_adult : 1;
  bool goron_s_maze_center_child : 1;
  bool mountain_s_crater_box_child : 1;
  bool woods_s_deku_salesman_beanspot_child : 1;

  bool woods_s_mask_theater_beanspot_child : 1;
  bool woods_s_deku_salesman_magic_bean_adult : 1;
  bool woods_s_labyrinth_adult_night : 1;
  bool kokiri_s_beanspot_near_shop_child : 1;
  bool kokiri_s_know_it_all_brothers_child_night : 1;
  bool kokiri_s_twins_adult_night : 1;
  bool gerudo_valley_s_gerudo_river_beanspot_child : 1;
  bool gerudo_valley_s_gerudo_wood_plank_child_night : 1;

  bool gerudo_valley_s_gerudo_rock_near_carpenter_tent_adult_night : 1;
  bool gerudo_valley_s_gerudo_behind_carpenter_tent_adult_night : 1;
  bool lake_s_beanspot_child : 1;
  bool lake_s_near_firearrows_child_night : 1;
  bool lake_s_behind_lab_child_night : 1;
  bool lake_s_lab_box_adult : 1;
  bool lake_s_dead_tree_adult_night : 1;
  bool kakariko_s_graveyard_beanspot_child_night : 1;

  bool kakariko_s_near_gate_child_night : 1;
  bool kakariko_s_tower_child_night : 1;
  bool kakariko_s_construction_site_child_night : 1;
  bool kakariko_s_skulltula_house_child_night : 1;
  bool kakariko_s_tree_child : 1;
  bool kakariko_s_impas_house_roof_adult_night : 1;
  bool kakariko_s_graveyard_royal_grave_child_night : 1;
  bool gerudo_s_horseback_archery_target_adult_night : 1;

  bool gerudo_s_rooftop_adult_night : 1;
  bool zoras_c_diving : 1;
  bool lake_c_bottle_zora : 1;
  bool goron_c_darunia : 1;
  bool hyrule_field_c_ocarina_of_time : 1;
  bool woods_c_warp : 1;
  bool mountain_c_warp : 1;
  bool ice_cavern_c_sheik : 1;

  bool ice_cavern_mq_c_sheik : 1;
  bool castle_town_c_warp : 1;
  bool woods_c_saria : 1;
  bool gerudo_c_fortress_clear : 1;
  bool kokiri_c_saria_ocarina : 1;
  bool castle_town_c_light_arrows : 1;
  bool kakariko_c_skulltula_10 : 1;
  bool kakariko_c_skulltula_20 : 1;

  bool kakariko_c_skulltula_30 : 1;
  bool kakariko_c_skulltula_40 : 1;
  bool kakariko_c_skulltula_50 : 1;
  bool zoras_c_fairy : 1;
  bool castle_town_c_hyrule_fairy : 1;
  bool desert_c_colossus_fairy : 1;
  bool castle_town_c_chest_game : 1;
  bool woods_c_shooting : 1;

  bool woods_c_deku_theater_1 : 1;
  bool woods_c_deku_theater_2 : 1;
  bool lake_c_lab_dive : 1;
  bool castle_town_c_bowling_1 : 1;
  bool castle_town_c_bowling_2 : 1;
  bool kakariko_c_man_roof : 1;
  bool woods_c_skull_kid : 1;
  bool woods_c_memory_game : 1;

  bool kakariko_c_anju_adult : 1;
  bool goron_c_little_link : 1;
  bool goron_c_hot_rodder : 1;
  bool hyrule_field_s_hyrule_west_grotto : 1;
  bool hyrule_field_s_hyrule_north_east_grotto : 1;
  bool castle_town_s_near_ganon_fairy_adult : 1;
  bool castle_town_s_storm_grotto_child : 1;
  bool castle_town_s_tree_child : 1;

  bool castle_town_s_pot_room_child : 1;
  bool river_s_zora_ladder_child_night : 1;
  bool river_s_zora_tree_child : 1;
  bool zoras_s_wall_child_night : 1;
  bool river_s_zora_wall2_adult_night : 1;
  bool river_s_zora_wall1_adult_night : 1;
  bool zoras_s_fairy_hidden_cave_adult_night : 1;
  bool zoras_s_wall_ice_adult_night : 1;

  bool zoras_s_tree_child : 1;
  bool desert_s_beanspot_child : 1;
  bool desert_s_fortress_adult : 1;
  bool desert_s_magic_bean_adult_night : 1;
  bool desert_s_palm_tree_adult_night : 1;
  bool castle_town_c_poes : 1;
  bool lake_c_fishing_child : 1;
  bool lake_c_fishing_adult : 1;

  bool castle_town_c_maron : 1;
  bool lonlon_c_maron_song : 1;
  bool castle_town_c_zelda : 1;
  bool kakariko_c_sun_song : 1;
  bool kakariko_c_song_storm : 1;
  bool hyrule_field_c_song_of_time : 1;
  bool kakariko_c_warp : 1;
  bool desert_c_warp_spirit : 1;

  bool river_c_frog_game : 1;
  bool river_c_frog_rain : 1;
  bool kakariko_c_anju_chickens : 1;
  bool castle_town_c_child_shooting : 1;
  bool kakariko_c_adult_shooting : 1;
  bool gerudo_c_archer_2 : 1;
  bool lonlon_c_talon_chicken : 1;
  bool zoras_c_king_zora : 1;

  bool gerudo_c_archer_1 : 1;
  bool castle_town_c_dog : 1;
  bool deku_c_gohma : 1;
  bool deku_mq_c_gohma : 1;
  bool dodongo_c_dodongo : 1;
  bool dodongo_mq_c_dodongo : 1;
  bool jabujabu_c_barinade : 1;
  bool jabujabu_mq_c_barinade : 1;

  bool temple_forest_c_phantomganon : 1;
  bool temple_forest_mq_c_phantomganon : 1;
  bool temple_fire_c_volvagia : 1;
  bool temple_fire_mq_c_volvagia : 1;
  bool temple_water_c_morpha : 1;
  bool temple_water_mq_c_morpha : 1;
  bool temple_spirit_c_twinrova : 1;
  bool temple_spirit_mq_c_twinrova : 1;

  bool temple_shadow_c_bongobongo : 1;
  bool temple_shadow_mq_c_bongobongo : 1;
  bool deku_c_slingshot : 1;
  bool deku_c_compass : 1;
  bool deku_c_lobby : 1;
  bool deku_c_basement : 1;
  bool deku_c_slingshot_side : 1;
  bool deku_c_compass_side : 1;

  bool deku_s_bombable_behind_web_child : 1;
  bool deku_s_basement_metal_gate_child : 1;
  bool deku_s_basement_vines_child : 1;
  bool deku_s_3f_compass_side_child : 1;
  bool deku_mq_c_after_log : 1;
  bool deku_mq_c_compass : 1;
  bool deku_mq_c_slingshot_back : 1;
  bool deku_mq_c_lobby : 1;

  bool deku_mq_c_basement : 1;
  bool deku_mq_c_before_log : 1;
  bool deku_mq_c_slingshot : 1;
  bool deku_mq_c_basement_salesman : 1;
  bool deku_mq_s_basement_back_room_child : 1;
  bool deku_mq_s_crate_near_map_child : 1;
  bool deku_mq_s_basement_grave_ceiling_child : 1;
  bool deku_mq_s_2f_compass_side_child : 1;

  bool dodongo_c_map : 1;
  bool dodongo_c_end_bridge : 1;
  bool dodongo_c_bomb_bag : 1;
  bool dodongo_c_compass : 1;
  bool dodongo_c_bomb_flower_platform : 1;
  bool dodongo_c_near_bomb_bag_salesman_1 : 1;
  bool dodongo_c_east_corridor_salesman : 1;
  bool dodongo_c_near_bomb_bag_salesman_2 : 1;

  bool dodongo_c_lobby_salesman : 1;
  bool dodongo_c_chest_above_dodongo : 1;
  bool dodongo_s_big_staircase_vines : 1;
  bool dodongo_s_east_corridor_scarecrow_adult : 1;
  bool dodongo_s_above_big_staircase_adult : 1;
  bool dodongo_s_near_boss_bombable : 1;
  bool dodongo_s_east_corridor_bombable : 1;
  bool dodongo_mq_c_map : 1;

  bool dodongo_mq_c_under_grave : 1;
  bool dodongo_mq_c_gohma_larva_room : 1;
  bool dodongo_mq_c_torch_puzzle_room : 1;
  bool dodongo_mq_c_bomb_bag : 1;
  bool dodongo_mq_c_compass : 1;
  bool dodongo_mq_c_1f_right : 1;
  bool dodongo_mq_c_main_room_salesman_1 : 1;
  bool dodongo_mq_c_main_room_salesman_2 : 1;

  bool dodongo_mq_c_above_stairs : 1;
  bool dodongo_mq_c_chest_above_dodongo : 1;
  bool dodongo_mq_s_back_area : 1;
  bool dodongo_mq_s_scrub_poe_room : 1;
  bool dodongo_mq_s_upper_lizalfos_room : 1;
  bool dodongo_mq_s_time_block_room : 1;
  bool dodongo_mq_s_gohma_larva_skulltula : 1;
  bool jabujabu_c_boomerang : 1;

  bool jabujabu_c_map : 1;
  bool jabujabu_c_compass : 1;
  bool jabujabu_c_elevator_dive_salesman : 1;
  bool jabujabu_s_b1_ruto1_child : 1;
  bool jabujabu_s_b1_ruto2_child : 1;
  bool jabujabu_s_1f_near_boss_child : 1;
  bool jabujabu_s_b1_rising_water_child : 1;
  bool jabujabu_mq_c_basement_north_chest : 1;

  bool jabujabu_mq_c_like_like_chest : 1;
  bool jabujabu_mq_c_near_boss : 1;
  bool jabujabu_mq_c_compass : 1;
  bool jabujabu_mq_c_boomerang_room_small_chest : 1;
  bool jabujabu_mq_c_second_room_lower_chest : 1;
  bool jabujabu_mq_c_map : 1;
  bool jabujabu_mq_c_basement_south_chest : 1;
  bool jabujabu_mq_c_entry_side_chest : 1;

  bool jabujabu_mq_c_boomerang_chest : 1;
  bool jabujabu_mq_c_second_room_upper : 1;
  bool jabujabu_mq_c_cow : 1;
  bool jabujabu_mq_s_boomerang_room : 1;
  bool jabujabu_mq_s_near_boss_gs : 1;
  bool jabujabu_mq_s_electric_worm_room : 1;
  bool jabujabu_mq_s_invisible_enemies_room : 1;
  bool temple_forest_c_well : 1;

  bool temple_forest_c_near_boss : 1;
  bool temple_forest_c_bow : 1;
  bool temple_forest_c_poe_red : 1;
  bool temple_forest_c_boss_key : 1;
  bool temple_forest_c_poe_blue : 1;
  bool temple_forest_c_behind_lobby : 1;
  bool temple_forest_c_map : 1;
  bool temple_forest_c_floormaster : 1;

  bool temple_forest_c_entrance : 1;
  bool temple_forest_c_block_push : 1;
  bool temple_forest_c_outside_hookshot : 1;
  bool temple_forest_c_falling_room : 1;
  bool temple_forest_s_outside_hookshot_adult : 1;
  bool temple_forest_s_entrance_adult : 1;
  bool temple_forest_s_pillars_adult : 1;
  bool temple_forest_s_lobby_adult : 1;

  bool temple_forest_s_near_boss_adult : 1;
  bool temple_forest_mq_c_well_chest : 1;
  bool temple_forest_mq_c_near_boss : 1;
  bool temple_forest_mq_c_bow : 1;
  bool temple_forest_mq_c_map : 1;
  bool temple_forest_mq_c_boss_key : 1;
  bool temple_forest_mq_c_compass : 1;
  bool temple_forest_mq_c_behind_lobby : 1;

  bool temple_forest_mq_c_ne_outdoors_lower : 1;
  bool temple_forest_mq_c_redead_chest : 1;
  bool temple_forest_mq_c_entrance : 1;
  bool temple_forest_mq_c_ne_outdoors_upper : 1;
  bool temple_forest_mq_c_falling_room : 1;
  bool temple_forest_mq_s_outdoor_east_adult : 1;
  bool temple_forest_mq_s_first_hallway_adult : 1;
  bool temple_forest_mq_s_outdoor_west_adult : 1;

  bool temple_forest_mq_s_well_adult : 1;
  bool temple_forest_mq_s_block_push_room_adult : 1;
  bool temple_fire_c_boulder_maze_side : 1;
  bool temple_fire_c_highest_goron : 1;
  bool temple_fire_c_map : 1;
  bool temple_fire_c_boulder_maze_bomb : 1;
  bool temple_fire_c_boss_key : 1;
  bool temple_fire_c_scarecrow : 1;

  bool temple_fire_c_dancer : 1;
  bool temple_fire_c_near_boss : 1;
  bool temple_fire_c_big_lava_bomb : 1;
  bool temple_fire_c_boulder_maze_lower : 1;
  bool temple_fire_c_big_lava_open : 1;
  bool temple_fire_c_hammer : 1;
  bool temple_fire_c_boulder_maze_upper : 1;
  bool temple_fire_c_compass : 1;

  bool temple_fire_s_big_lava_time_block_adult : 1;
  bool temple_fire_s_near_boss_key_adult : 1;
  bool temple_fire_s_boulder_maze_bombable_adult : 1;
  bool temple_fire_s_boulder_maze_scarecrow2_adult : 1;
  bool temple_fire_s_boulder_maze_scarecrow1_adult : 1;
  bool temple_fire_mq_c_boulder_maze_side : 1;
  bool temple_fire_mq_c_compass : 1;
  bool temple_fire_mq_c_map : 1;

  bool temple_fire_mq_c_megaton_hammer : 1;
  bool temple_fire_mq_c_big_lava_bomb : 1;
  bool temple_fire_mq_c_map_room_small_chest : 1;
  bool temple_fire_mq_c_boulder_maze_lower : 1;
  bool temple_fire_mq_c_boss_key : 1;
  bool temple_fire_mq_c_west_tower_top : 1;
  bool temple_fire_mq_c_boulder_maze_upper : 1;
  bool temple_fire_mq_c_chest_near_boss : 1;

  bool temple_fire_mq_c_freestanding_key : 1;
  bool temple_fire_mq_s_big_lava_adult : 1;
  bool temple_fire_mq_s_above_fire_wall_maze_adult : 1;
  bool temple_fire_mq_s_east_tower_top_adult : 1;
  bool temple_fire_mq_s_fire_wall_maze_center_adult : 1;
  bool temple_fire_mq_s_fire_wall_maze_side_adult : 1;
  bool ice_cavern_c_map : 1;
  bool ice_cavern_c_compass : 1;

  bool ice_cavern_c_boots : 1;
  bool ice_cavern_c_hp : 1;
  bool ice_cavern_s_block_puzzle_adult : 1;
  bool ice_cavern_s_ice_blades_adult : 1;
  bool ice_cavern_s_compass_room_adult : 1;
  bool ice_cavern_mq_c_compass : 1;
  bool ice_cavern_mq_c_map : 1;
  bool ice_cavern_mq_c_boots : 1;

  bool ice_cavern_mq_c_hp : 1;
  bool ice_cavern_mq_s_scarecrow : 1;
  bool ice_cavern_mq_s_hp_room : 1;
  bool ice_cavern_mq_s_under_ice_block : 1;
  bool temple_water_c_central_bow : 1;
  bool temple_water_c_compass : 1;
  bool temple_water_c_chest_dragon : 1;
  bool temple_water_c_cracked_wall : 1;

  bool temple_water_c_torches : 1;
  bool temple_water_c_map : 1;
  bool temple_water_c_river : 1;
  bool temple_water_c_boss_key : 1;
  bool temple_water_c_center_pillar : 1;
  bool temple_water_c_dark_link : 1;
  bool temple_water_s_lowest_level_south_adult : 1;
  bool temple_water_s_big_waterfall_adult : 1;

  bool temple_water_s_center_pillar_adult : 1;
  bool temple_water_s_near_boss_key_adult : 1;
  bool temple_water_s_river_adult : 1;
  bool temple_water_mq_c_longshot : 1;
  bool temple_water_mq_c_compass : 1;
  bool temple_water_mq_c_map : 1;
  bool temple_water_mq_c_boss_key : 1;
  bool temple_water_mq_c_central_pillar : 1;

  bool temple_water_mq_c_freestanding_key : 1;
  bool temple_water_mq_s_lizalfos_hallway_adult : 1;
  bool temple_water_mq_s_serpent_river_adult : 1;
  bool temple_water_mq_s_before_upper_water_switch : 1;
  bool temple_water_mq_s_north_basement_adult : 1;
  bool temple_water_mq_s_south_basement_adult : 1;
  bool temple_spirit_c_topmost : 1;
  bool temple_spirit_c_hallway_right_invisible : 1;

  bool temple_spirit_c_hallway_left_invisible : 1;
  bool temple_spirit_c_child_left : 1;
  bool temple_spirit_c_boss_key : 1;
  bool temple_spirit_c_child_climb_east : 1;
  bool temple_spirit_c_first_mirror_left : 1;
  bool temple_spirit_c_first_mirror_right : 1;
  bool temple_spirit_c_main_room_NE : 1;
  bool temple_spirit_c_child_right : 1;

  bool temple_spirit_c_sun_block : 1;
  bool temple_spirit_c_statue_hand : 1;
  bool temple_spirit_c_map : 1;
  bool temple_spirit_c_compass : 1;
  bool temple_spirit_c_near_four_armos : 1;
  bool temple_spirit_c_child_climb_north : 1;
  bool temple_spirit_c_early_adult_right : 1;
  bool temple_spirit_s_childsegment_before_iron_knight : 1;

  bool temple_spirit_s_rolling_boulders_time_block_adult : 1;
  bool temple_spirit_s_coloss_room_adult : 1;
  bool temple_spirit_s_childsegment_sun_room : 1;
  bool temple_spirit_s_childsegment_left : 1;
  bool temple_spirit_mq_c_dinolfos_ice : 1;
  bool temple_spirit_mq_c_beamos_room : 1;
  bool temple_spirit_mq_c_entrance_front_left : 1;
  bool temple_spirit_mq_c_entrance_front_right : 1;

  bool temple_spirit_mq_c_silver_block_hallway : 1;
  bool temple_spirit_mq_c_child_center : 1;
  bool temple_spirit_mq_c_entrance_back_left : 1;
  bool temple_spirit_mq_c_entrance_back_right : 1;
  bool temple_spirit_mq_c_mirror_puzzle_invisible : 1;
  bool temple_spirit_mq_c_child_left : 1;
  bool temple_spirit_mq_c_child_climb_south : 1;
  bool temple_spirit_mq_c_lower_ne_main_room : 1;

  bool temple_spirit_mq_c_map : 1;
  bool temple_spirit_mq_c_sun_block_room : 1;
  bool temple_spirit_mq_c_upper_ne_main_room : 1;
  bool temple_spirit_mq_c_compass_chest : 1;
  bool temple_spirit_mq_c_lower_adult_left : 1;
  bool temple_spirit_mq_c_boss_key : 1;
  bool temple_spirit_mq_c_child_climb_north : 1;
  bool temple_spirit_mq_c_lower_adult_right : 1;

  bool temple_spirit_mq_s_sun_block_room : 1;
  bool temple_spirit_mq_s_lower_adult_left : 1;
  bool temple_spirit_mq_s_iron_knuckles_west : 1;
  bool temple_spirit_mq_s_lower_adult_right : 1;
  bool temple_spirit_mq_s_iron_knuckles_north : 1;
  bool well_c_underwater_front : 1;
  bool well_c_invisible : 1;
  bool well_c_front_left_hidden_wall : 1;

  bool well_c_underwater_left : 1;
  bool well_c_locked_pits : 1;
  bool well_c_behind_right_grate : 1;
  bool well_c_center_small : 1;
  bool well_c__center_large : 1;
  bool well_c_front_center_bombable : 1;
  bool well_c_defeat_boss : 1;
  bool well_c_back_left_bombable : 1;

  bool well_c_right_bottom_hidden_wall : 1;
  bool well_c_basement_ches : 1;
  bool well_c_coffin_key : 1;
  bool well_s_locked_pits_child : 1;
  bool well_s_east_inner_child : 1;
  bool well_s_west_inner_child : 1;
  bool well_mq_c_lens_chest : 1;
  bool well_mq_c_compass_chest : 1;

  bool well_mq_c_map_chest : 1;
  bool well_mq_c_east_inner_freestanding_key : 1;
  bool well_mq_c_deadhand_freestanding_key : 1;
  bool well_mq_s_basement : 1;
  bool well_mq_s_west_inner : 1;
  bool well_mq_s_coffin_room : 1;
  bool temple_shadow_c_after_wind_hidden : 1;
  bool temple_shadow_c_wind_hint : 1;

  bool temple_shadow_c_invisible_blades_invisible : 1;
  bool temple_shadow_c_after_wind_enemy : 1;
  bool temple_shadow_c_invisible_spikes : 1;
  bool temple_shadow_c_spike_walls : 1;
  bool temple_shadow_c_boss_key : 1;
  bool temple_shadow_c_invisible_blades_visible : 1;
  bool temple_shadow_c_hidden_floormaster : 1;
  bool temple_shadow_c_map : 1;

  bool temple_shadow_c_early_silver : 1;
  bool temple_shadow_c_compass : 1;
  bool temple_shadow_c_falling_spikes_switch : 1;
  bool temple_shadow_c_falling_spikes_lower : 1;
  bool temple_shadow_c_falling_spikes_upper : 1;
  bool temple_shadow_c_boots : 1;
  bool temple_shadow_c_giant_pot : 1;
  bool temple_shadow_s_behind_giant_pot_adult : 1;

  bool temple_shadow_s_falling_spikes_adult : 1;
  bool temple_shadow_s_behind_giant_pots_adult : 1;
  bool temple_shadow_s_invisible_blade_adult : 1;
  bool temple_shadow_s_boat_adult : 1;
  bool temple_shadow_mq_c_stalfos_room : 1;
  bool temple_shadow_mq_c_after_wind_hidden : 1;
  bool temple_shadow_mq_c_wind_hint : 1;
  bool temple_shadow_mq_c_invisible_blades_invisible : 1;

  bool temple_shadow_mq_c_after_wind_enemy : 1;
  bool temple_shadow_mq_c_invisible_spikes : 1;
  bool temple_shadow_mq_c_spike_walls_left : 1;
  bool temple_shadow_mq_c_boss_key : 1;
  bool temple_shadow_mq_c_invisible_blades_visible : 1;
  bool temple_shadow_mq_c_bomb_flower_chest : 1;
  bool temple_shadow_mq_c_near_ship_invisible : 1;
  bool temple_shadow_mq_c_beamos_silver : 1;

  bool temple_shadow_mq_c_compass : 1;
  bool temple_shadow_mq_c_map : 1;
  bool temple_shadow_mq_c_early_gibdos : 1;
  bool temple_shadow_mq_c_falling_spikes_switch : 1;
  bool temple_shadow_mq_c_falling_spikes_lower : 1;
  bool temple_shadow_mq_c_falling_spikes_upper : 1;
  bool temple_shadow_mq_c_boots : 1;
  bool temple_shadow_mq_c_freestanding_key : 1;

  bool temple_shadow_mq_s_wind_hint_adult : 1;
  bool temple_shadow_mq_s_crusher_room_adult : 1;
  bool temple_shadow_mq_s_near_boss_adult : 1;
  bool temple_shadow_mq_s_after_wind_adult : 1;
  bool temple_shadow_mq_s_after_ship_adult : 1;
  bool training_grounds_c_hammer_room_switch : 1;
  bool training_grounds_c_heavy_block_before : 1;
  bool training_grounds_c_hammer_room_clear : 1;

  bool training_grounds_c_lobby_left : 1;
  bool training_grounds_c_heavy_block_3 : 1;
  bool training_grounds_c_maze_right : 1;
  bool training_grounds_c_maze_3 : 1;
  bool training_grounds_c_maze_2 : 1;
  bool training_grounds_c_hidden_ceiling : 1;
  bool training_grounds_c_maze_final : 1;
  bool training_grounds_c_underwater_silver : 1;

  bool training_grounds_c_heavy_block_2 : 1;
  bool training_grounds_c_heavy_block_1 : 1;
  bool training_grounds_c_stalfos : 1;
  bool training_grounds_c_beamos : 1;
  bool training_grounds_c_heavy_block_4 : 1;
  bool training_grounds_c_eye_statue : 1;
  bool training_grounds_c_scarecrow : 1;
  bool training_grounds_c_maze_right_central : 1;

  bool training_grounds_c_maze_1 : 1;
  bool training_grounds_c_lobby_right : 1;
  bool training_grounds_c_maze_right_key : 1;
  bool training_grounds_mq_c_before_heavy_block : 1;
  bool training_grounds_mq_c_second_iron_knuckle : 1;
  bool training_grounds_mq_c_lobby_left : 1;
  bool training_grounds_mq_c_maze_right_side : 1;
  bool training_grounds_mq_c_maze_3 : 1;

  bool training_grounds_mq_c_maze_2 : 1;
  bool training_grounds_mq_c_hidden_ceiling : 1;
  bool training_grounds_mq_c_underwater : 1;
  bool training_grounds_mq_c_flame_circle : 1;
  bool training_grounds_mq_c_first_iron_knuckle : 1;
  bool training_grounds_mq_c_dinolfos : 1;
  bool training_grounds_mq_c_heavy_block : 1;
  bool training_grounds_mq_c_eye_statue : 1;

  bool training_grounds_mq_c_ice_arrows : 1;
  bool training_grounds_mq_c_maze_right_central : 1;
  bool training_grounds_mq_c_maze_1 : 1;
  bool training_grounds_mq_c_lobby_right : 1;
  bool castle_ganon_c_boss_key : 1;
  bool castle_ganon_c_light_trail_invisible_enemies : 1;
  bool castle_ganon_c_light_trial_lullaby : 1;
  bool castle_ganon_c_spirit_trial_first : 1;

  bool castle_ganon_c_spirit_trial_second : 1;
  bool castle_ganon_c_shadow_trial_first : 1;
  bool castle_ganon_c_forest_trial : 1;
  bool castle_ganon_c_light_trial_second_right : 1;
  bool castle_ganon_c_light_trial_second_left : 1;
  bool castle_ganon_c_light_trial_first_left : 1;
  bool castle_ganon_c_light_trial_third_left : 1;
  bool castle_ganon_c_light_trial_first_right : 1;

  bool castle_ganon_c_light_trial_third_right : 1;
  bool castle_ganon_c_shadow_trial_second : 1;
  bool castle_ganon_c_water_trial_right : 1;
  bool castle_ganon_c_water_trial_left : 1;
  bool castle_ganon_c_under_bridge_salesman_4 : 1;
  bool castle_ganon_c_under_bridge_salesman_1 : 1;
  bool castle_ganon_c_under_bridge_salesman_3 : 1;
  bool castle_ganon_c_under_bridge_salesman_2 : 1;

  bool castle_ganon_mq_c_boss_key : 1;
  bool castle_ganon_mq_c_spirit_trial_second : 1;
  bool castle_ganon_mq_c_spirit_sun_back_left : 1;
  bool castle_ganon_mq_c_spirit_sun_back_right : 1;
  bool castle_ganon_mq_c_spirit_trial_first : 1;
  bool castle_ganon_mq_c_shadow_trial_first : 1;
  bool castle_ganon_mq_c_water_trial : 1;
  bool castle_ganon_mq_c_forest_trial_first : 1;

  bool castle_ganon_mq_c_forest_trial_second : 1;
  bool castle_ganon_mq_c_light_trial_lullaby : 1;
  bool castle_ganon_mq_c_shadow_trial_second : 1;
  bool castle_ganon_mq_c_spirit_golden : 1;
  bool castle_ganon_mq_c_spirit_sun_front_left : 1;
  bool castle_ganon_mq_c_forest_trial_key : 1;
  bool castle_ganon_mq_c_under_bridge_salesman_4 : 1;
  bool castle_ganon_mq_c_under_bridge_salesman_1 : 1;

  bool castle_ganon_mq_c_under_bridge_salesman_5 : 1;
  bool castle_ganon_mq_c_under_bridge_salesman_3 : 1;
  bool castle_ganon_mq_c_under_bridge_salesman_2 : 1;
  bool item_arrow_fire : 1;
  bool item_magic_din : 1;
  bool item_bombchu : 1;
  u8 item_magic_power : 2;

  u8 item_bow : 2;
  u8 item_bombs : 2;
  u8 item_glove : 2;
  u8 item_scale : 2;

  u8 item_wallet : 2;
  u8 item_slingshot : 2;
  u8 item_stick : 2;
  u8 item_nut : 2;

  u8 item_ocarina : 2;
  u8 item_hookshot : 2;
  bool item_arrow_ice : 1;
  bool item_magic_farore : 1;
  bool item_boomerang : 1;
  bool item_lens : 1;

  u8 item_bean : 4;
  bool item_hammer : 1;
  bool item_arrow_light : 1;
  bool item_zora_letter : 1;
  bool item_magic_nayru : 1;

  bool ganon_stabbed : 1;
  u8 item_bottle : 3;
  u8 item_poe_big : 4;

  u8 item_cojiro : 4;
  u8 item_egg : 4;

  bool item_sword_kokiri : 1;
  bool item_sword_master : 1;
  bool item_sword_biggoron : 1;
  bool item_shield_kokiri : 1;
  bool item_shield_hylia : 1;
  bool item_shield_mirror : 1;
  bool item_tunic_fire : 1;
  bool item_tunic_water : 1;

  bool item_boots_iron : 1;
  bool item_boots_hover : 1;
  bool item_heart_double : 1;
  u8 item_heart_container : 4;
  bool item_medallion_forest : 1;

  u8 item_heart_piece : 6;
  bool item_medallion_fire : 1;
  bool item_medallion_water : 1;

  u8 item_skulltula : 7;
  bool item_medallion_spirit : 1;

  bool item_medallion_shadow : 1;
  bool item_medallion_light : 1;
  bool item_warp_forest : 1;
  bool item_warp_fire : 1;
  bool item_warp_water : 1;
  bool item_warp_spirit : 1;
  bool item_warp_shadow : 1;
  bool item_warp_light : 1;

  bool item_song_zelda : 1;
  bool item_song_epona : 1;
  bool item_song_saria : 1;
  bool item_song_sun : 1;
  bool item_song_time : 1;
  bool item_song_storm : 1;
  bool item_song_scarecrow : 1;
  bool item_stone_forest : 1;

  bool item_stone_fire : 1;
  bool item_stone_water : 1;
  bool item_stone_of_agony : 1;
  bool item_membership : 1;
  bool mq_deku : 1;
  bool compass_deku : 1;
  bool dungeon_map_deku : 1;
  bool mq_dodongo : 1;

  bool compass_dodongo : 1;
  bool dungeon_map_dodongo : 1;
  bool mq_jabujabu : 1;
  bool compass_jabujabu : 1;
  bool dungeon_map_jabujabu : 1;
  bool mq_temple_forest : 1;
  bool key_boss_forest : 1;
  bool compass_temple_forest : 1;

  bool dungeon_map_temple_forest : 1;
  bool mq_temple_fire : 1;
  bool key_boss_fire : 1;
  bool compass_temple_fire : 1;
  bool dungeon_map_temple_fire : 1;
  bool mq_ice_cavern : 1;
  bool compass_ice : 1;
  bool dungeon_map_ice : 1;

  bool mq_temple_water : 1;
  bool key_boss_water : 1;
  bool compass_temple_water : 1;
  bool dungeon_map_temple_water : 1;
  bool mq_temple_spirit : 1;
  bool key_boss_spirit : 1;
  bool compass_temple_spirit : 1;
  bool dungeon_map_temple_spirit : 1;

  bool mq_well : 1;
  bool compass_well : 1;
  bool dungeon_map_well : 1;
  bool mq_temple_shadow : 1;
  bool key_boss_shadow : 1;
  bool compass_temple_shadow : 1;
  bool dungeon_map_temple_shadow : 1;
  bool mq_training_grounds : 1;

  bool mq_castle_ganon : 1;
  bool key_boss_ganon : 1;
  bool is_child : 1;
  u8 key_small_forest : 4;

  u8 key_small_fire : 4;
  u8 key_small_water : 4;

  u8 key_small_spirit : 4;
  u8 key_small_well : 4;

  u8 key_small_shadow : 4;
  u8 key_small_training : 4;

  u8 key_small_gerudo : 4;
  u8 key_small_ganon : 4;

  u8 location;

  u16 child_spawn;
  u16 adult_spawn;
  u16 dmt_owl;
  u16 lh_owl;
  u16 minuet_warp;
  u16 bolero_warp;
  u16 serenade_warp;
  u16 requiem_warp;
  u16 nocturne_warp;
  u16 prelude_warp;

  u16 last_exit;
  u8 grotto_id;
  u16 exits[16];
  u16 grotto_ids[8];
  u8 grotto_types[8];

  u8 shop_players[8];
  u16 shop_items[8];
  u16 shop_bought_prices[8];
} old_tracker = {0, }, tracker = {0, };

struct {
  u32 key;
  u16 item;
  u16 player;
} outgoing_buffer = {0, };

struct {
  u16 item;
  u16 count;
} incoming_buffer = {0, };

bool usb_send_init() {
  while (usb_poll() & USB_CAN_READ) usb_read();
  u8 data[16] = MAGIC(data);
  data[ 8] = USB_CMD_INIT;
  data[ 9] = USB_VERSION;
  data[10] = PLAYER_ID;
  data[11] = (INTERNAL_COUNT & 0xFF00) >> 8;
  data[12] = (INTERNAL_COUNT & 0x00FF);
  data[13] = 'M';
  data[14] = 'g';
  data[15] = 'c';
  usb_write(&data, 16);
  return true;
}

void usb_send_outgoing() {
  u8 data[16] = MAGIC(data);
  data[ 8] = USB_CMD_OUTGOING;
  data[ 9] = (outgoing_buffer.key & 0x00FF0000) >> 16;
  data[10] = (outgoing_buffer.key & 0x0000FF00) >> 8;
  data[11] = (outgoing_buffer.key & 0x000000FF);
  data[12] = (outgoing_buffer.item & 0xFF00) >> 8;
  data[13] = (outgoing_buffer.item & 0x00FF);
  data[14] = (outgoing_buffer.player & 0xFF00) >> 8;
  data[15] = (outgoing_buffer.player & 0x00FF);
  usb_write(&data, 16);
}

void initTrackerVars();
void updateTracker();

void usb_process() {
  if (usb_vars.settings & SETTING_ANTIALIAS) {
    AA1 = 0x00013016;
    AA2 = 0x00013016;
  }
  else {
    AA1 = 0x00003216;
    AA2 = 0x00003216;
  }
  if (usb_vars.initialized && usb_cart != CART_NONE && (z64_file.game_mode == 1 || z64_file.game_mode == 2)) {
    if (usb_poll() & USB_CAN_WRITE) {
      while (usb_poll() & USB_CAN_READ) usb_read();
      u8 data[16] = {0, };
      usb_write(&data, 16);
    }
    usb_vars.initialized = false;
    usb_vars.tracker_initialized = false;
    outgoing_buffer.key = 0;
    outgoing_buffer.item = 0;
    outgoing_buffer.player = 0;
    incoming_buffer.item = 0;
    incoming_buffer.count = 0;
    tracker.ganon_stabbed = false;
    usb_vars.received_item = 0;
    usb_vars.usb_timer = 0;
    usb_vars.vars_init = false;
    usb_vars.packet_received = false;
    usb_vars.pause_writes = 0;
  }
  if ((z64_file.game_mode == 1 || z64_file.game_mode == 2) || usb_cart == CART_NONE) return;
  if (usb_vars.usb_timer++ > 20) usb_vars.usb_timer = 0;
  if (!usb_vars.initialized) {
    if (usb_vars.vars_init && (*(vu16*)(0x801DA2BA)) != (*(vu16*)(0x8011A5D2))) usb_vars.tracker_timeout = 20;
    else if (usb_vars.tracker_timeout) usb_vars.tracker_timeout--;
    else if (usb_poll() & USB_CAN_WRITE && usb_send_init()) {
      usb_vars.initialized = true;
      initTrackerVars();
      usb_vars.tracker_timeout = 20;
    }
    return;
  }
  if (!tracker.ganon_stabbed && (*(vu16*)0x801C8544) == 0x004F) {
    u32 ganon = (*(vu32*)0x801CA11C);
    if (
         (*(vu32*)(ganon + 0x144)) == 0x06003B1C
      && (*(vu32*)(ganon + 0x14C)) == 0x428E0000
      && (*(vu32*)(ganon + 0x150)) == 0x42900000
      && (*(vu32*)(ganon + 0x154)) == 0x428E0000
    ) {
      tracker.ganon_stabbed = true;
      // outgoing_buffer.key = 0xFFFFFFFF;
      // outgoing_buffer.item = 0xFFFF;
      // outgoing_buffer.player = 0x00;
      // if (usb_poll() == USB_CAN_WRITE) usb_send_outgoing();
    }
  }
  if (usb_vars.pause_writes) {
    if (usb_poll() & USB_CAN_WRITE) {
      switch (usb_vars.pause_writes) {
        case 1: {
          u8 data[16] = MAGIC(data);
          data[ 8] = USB_CMD_PAUSE_WRITES;
          data[ 9] = 1;
          usb_write(&data, 16);
          usb_vars.pause_writes = 2;
          break;
        }
        case 3: {
          u8 data[16] = MAGIC(data);
          data[ 8] = USB_CMD_PAUSE_WRITES;
          data[ 9] = 0;
          usb_write(&data, 16);
          usb_vars.pause_writes = 0;
          break;
        }
      }
    }
  }
  else if (OUTGOING_KEY && !outgoing_buffer.key) {
    outgoing_buffer.key = OUTGOING_KEY;
    outgoing_buffer.item = OUTGOING_ITEM;
    outgoing_buffer.player = OUTGOING_PLAYER;
    OUTGOING_KEY = 0;
    OUTGOING_ITEM = 0;
    OUTGOING_PLAYER = 0;
    // if (usb_poll() == USB_CAN_WRITE) usb_send_outgoing();
    usb_vars.usb_timer = 0;
    usb_vars.tracker_timeout = 20;
  }
  else if (!usb_vars.usb_timer && outgoing_buffer.key && usb_poll() == USB_CAN_WRITE) usb_send_outgoing();
  else if (usb_poll() == USB_CAN_WRITE) {
    if (incoming_buffer.item && !INCOMING_ITEM && !usb_vars.received_item) {
      switch (z64_game.scene_index) {
        case 0x2C:
        case 0x2D:
        case 0x2E:
        case 0x2F:
        case 0x30:
        case 0x31:
        case 0x32:
        case 0x33:
        case 0x42:
        case 0x4B:
          break;
        default:
          if (incoming_buffer.count < INTERNAL_COUNT || incoming_buffer.item == 0xFFFF) INTERNAL_COUNT = incoming_buffer.count;
          if (incoming_buffer.item != 0xFFFF) {
            INCOMING_PLAYER = PLAYER_ID;
            INCOMING_ITEM   = incoming_buffer.item;
          }
          usb_vars.received_item = 20;
          incoming_buffer.item = 0;
          incoming_buffer.count = 0;
      }
    }
    else if (usb_vars.received_item) {
      if (!INCOMING_ITEM) {
        usb_vars.received_item--;
        if (!usb_vars.received_item) {
          u8 data[16] = MAGIC(data);
          data[ 8] = USB_CMD_INCOMING;
          data[ 9] = ((INTERNAL_COUNT) & 0xFF00) >> 8;
          data[10] = ((INTERNAL_COUNT) & 0x00FF);
          usb_write(&data, 16);
          usb_vars.tracker_timeout = 20;
        }
      }
    }
    else if (usb_vars.packet_received) {
      u8 data[16] = MAGIC(data);
      data[ 8] = USB_CMD_PACKET_RECEIVED;
      usb_write(&data, 16);
      usb_vars.packet_received = false;
    }
    else {
      if ((*(vu16*)(0x801DA2BA)) != (*(vu16*)(0x8011A5D2))) usb_vars.tracker_timeout = 20;
      else if (usb_vars.tracker_timeout) usb_vars.tracker_timeout--;
      else updateTracker();
    }
  }
  while (usb_poll() & USB_CAN_READ) {
    if (usb_read() || USB_SIZE < 16) continue;
    usb_vars.packet_received = true;
    u32 crc1  = USB_BUFFER[0] << 24;
        crc1 |= USB_BUFFER[1] << 16;
        crc1 |= USB_BUFFER[2] << 8;
        crc1 |= USB_BUFFER[3];
    u32 crc2  = USB_BUFFER[4] << 24;
        crc2 |= USB_BUFFER[5] << 16;
        crc2 |= USB_BUFFER[6] << 8;
        crc2 |= USB_BUFFER[7];
    if (USB_BUFFER[8] != USB_CMD_INIT && (crc1 != CRC1 || crc2 != CRC2)) continue;
    switch (USB_BUFFER[8]) {
      case USB_CMD_INIT:
        usb_vars.initialized = false;
        return;
        break;
      case USB_CMD_OUTGOING:
        outgoing_buffer.key = 0;
        outgoing_buffer.item = 0;
        outgoing_buffer.player = 0;
        break;
      case USB_CMD_INCOMING:
        if (!usb_vars.received_item) {
          incoming_buffer.item   = (USB_BUFFER[ 9] << 8) | USB_BUFFER[10];
          incoming_buffer.count  = (USB_BUFFER[11] << 8) | USB_BUFFER[12];
        }
        break;
      case USB_CMD_PLAYER:
        if (USB_SIZE < 18) continue;
        u16 id = 8*USB_BUFFER[9];
        for (u8 i = 0; i < 8; i++) PLAYER_NAMES[id+i] = USB_BUFFER[i+10];
        break;
      case USB_CMD_SETTINGS:
        usb_vars.settings = USB_BUFFER[9];
        break;
      case USB_CMD_PAUSE_WRITES:
        usb_vars.pause_writes = USB_BUFFER[9] ? 1 : 3;
        break;
    }
  }
}

// void usb_process() {
  // disable_interrupts();
  // dma_wait();
  // u32 dram = pi_regs.dram_addr;
  // u32 cart = pi_regs.cart_addr;
  // _usb_process();
  // pi_regs.status = 3;
  // pi_regs.dram_addr = dram;
  // pi_regs.cart_addr = cart;
  // dma_wait();
  // enable_interrupts();
// }

#include "get_items.h"
#define ITEM_LOCATION(i, addr) ({                        \
  key.all = addr;                                        \
  item_locations[i] = lookup_override_by_key(key).value; \
})
#define SAVE 0x8011A5D0
#define BASE 0x80110000
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
extern u8 cfg_dungeon_is_mq[];
override_value_t item_locations[33] = {0, };

void initTrackerVars() {
  if (!usb_vars.vars_init) {
    usb_vars.vars_init = true;
    (*(vu16*)(0x801DA2BA)) = (*(vu16*)(0x8011A5D2));
    override_key_t key;
    ITEM_LOCATION( 0, 0x002D0034);
    ITEM_LOCATION( 1, 0x002D0035);
    ITEM_LOCATION( 2, 0x002D0036);
    ITEM_LOCATION( 3, 0x002D0037);
    ITEM_LOCATION( 4, 0x00300034);
    ITEM_LOCATION( 5, 0x00300035);
    ITEM_LOCATION( 6, 0x00300036);
    ITEM_LOCATION( 7, 0x00300037);
    ITEM_LOCATION( 8, 0x002C003D);
    ITEM_LOCATION( 9, 0x002C003E);
    ITEM_LOCATION(10, 0x002C003F);
    ITEM_LOCATION(11, 0x002C0040);
    ITEM_LOCATION(12, 0x002C0034);
    ITEM_LOCATION(13, 0x002C0035);
    ITEM_LOCATION(14, 0x002C0036);
    ITEM_LOCATION(15, 0x002C0037);
    ITEM_LOCATION(16, 0x002E0034);
    ITEM_LOCATION(17, 0x002E0035);
    ITEM_LOCATION(18, 0x002E0036);
    ITEM_LOCATION(19, 0x002E0037);
    ITEM_LOCATION(20, 0x00310034);
    ITEM_LOCATION(21, 0x00310035);
    ITEM_LOCATION(22, 0x00310036);
    ITEM_LOCATION(23, 0x00310037);
    ITEM_LOCATION(24, 0x00320034);
    ITEM_LOCATION(25, 0x00320035);
    ITEM_LOCATION(26, 0x00320036);
    ITEM_LOCATION(27, 0x00320037);
    ITEM_LOCATION(28, 0x002F0034);
    ITEM_LOCATION(29, 0x002F0035);
    ITEM_LOCATION(30, 0x002F0036);
    ITEM_LOCATION(31, 0x002F0037);
    ITEM_LOCATION(32, 0x00FF0529);
    tracker.mq_deku             = (cfg_dungeon_is_mq[ 0]) ? true : false;
    tracker.mq_dodongo          = (cfg_dungeon_is_mq[ 1]) ? true : false;
    tracker.mq_jabujabu         = (cfg_dungeon_is_mq[ 2]) ? true : false;
    tracker.mq_temple_forest    = (cfg_dungeon_is_mq[ 3]) ? true : false;
    tracker.mq_temple_fire      = (cfg_dungeon_is_mq[ 4]) ? true : false;
    tracker.mq_temple_water     = (cfg_dungeon_is_mq[ 5]) ? true : false;
    tracker.mq_temple_spirit    = (cfg_dungeon_is_mq[ 6]) ? true : false;
    tracker.mq_temple_shadow    = (cfg_dungeon_is_mq[ 7]) ? true : false;
    tracker.mq_well             = (cfg_dungeon_is_mq[ 8]) ? true : false;
    tracker.mq_ice_cavern       = (cfg_dungeon_is_mq[ 9]) ? true : false;
    tracker.mq_training_grounds = (cfg_dungeon_is_mq[11]) ? true : false;
    tracker.mq_castle_ganon     = (cfg_dungeon_is_mq[13]) ? true : false;

    tracker.child_spawn = (*(vu16*)(0x800903E2));
    tracker.adult_spawn = (*(vu16*)(0x800903D2));
    tracker.lh_owl = (*(vu16*)(0x80053FC6));
    tracker.dmt_owl = (*(vu16*)(0x80053F92));

    tracker.minuet_warp   = (*(vu16*)(0x803AB22C +  0));
    tracker.bolero_warp   = (*(vu16*)(0x803AB22C +  2));
    tracker.serenade_warp = (*(vu16*)(0x803AB22C +  4));
    tracker.requiem_warp  = (*(vu16*)(0x803AB22C +  6));
    tracker.nocturne_warp = (*(vu16*)(0x803AB22C +  8));
    tracker.prelude_warp  = (*(vu16*)(0x803AB22C + 10));
  }
}

void updateTracker() {
  u8 mem;
  mem = (*(vu8*)(BASE + 0xA642));
  tracker.mountain_c_biggoron = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xA802));
  tracker.gerudo_c_guard_2 = (mem & BIT2) ? true : false;
  tracker.gerudo_c_guard_1 = (mem & BIT4) ? true : false;
  tracker.gerudo_c_guard_3 = (mem & BIT6) ? true : false;
  tracker.gerudo_c_guard_4 = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA877));
  tracker.hyrule_field_c_hyrule_deku_salesman_grotto = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xA902));
  tracker.river_c_zora_grotto_salesman_1 = (mem & BIT0) ? true : false;
  tracker.river_c_zora_grotto_salesman_2 = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xA956));
  tracker.woods_c_deku_salesman_meadow_grotto_1 = (mem & BIT0) ? true : false;
  tracker.woods_c_deku_salesman_meadow_grotto_2 = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xA973));
  tracker.lake_c_grave_grotto_salesman_1 = (mem & BIT1) ? true : false;
  tracker.lake_c_grave_grotto_salesman_2 = (mem & BIT4) ? true : false;
  tracker.lake_c_grave_grotto_salesman_3 = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xA98E));
  tracker.gerudo_valley_c_gerudo_behind_carpenter_tent_grotto_salesman_1 = (mem & BIT0) ? true : false;
  tracker.gerudo_valley_c_gerudo_behind_carpenter_tent_grotto_salesman_2 = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAA1A));
  tracker.woods_c_deku_salesman_grotto = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xAA1B));
  tracker.woods_c_deku_salesman_grotto_2 = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xAA8B));
  tracker.mountain_c_crater_grotto_salesman_1 = (mem & BIT1) ? true : false;
  tracker.mountain_c_crater_grotto_salesman_2 = (mem & BIT4) ? true : false;
  tracker.mountain_c_crater_grotto_salesman_3 = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xAAC3));
  tracker.goron_c_grotto_salesman_1 = (mem & BIT1) ? true : false;
  tracker.goron_c_grotto_salesman_2 = (mem & BIT4) ? true : false;
  tracker.goron_c_grotto_salesman_3 = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xAADF));
  tracker.lonlon_c_grotto_deku_salesman_1 = (mem & BIT1) ? true : false;
  tracker.lonlon_c_grotto_deku_salesman_2 = (mem & BIT4) ? true : false;
  tracker.lonlon_c_grotto_deku_salesman_3 = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xAAFA));
  tracker.desert_c_colossus_grotto_salesman_1 = (mem & BIT0) ? true : false;
  tracker.desert_c_colossus_grotto_salesman_2 = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAB07));
  tracker.kokiri_c_midos_house_1 = (mem & BIT0) ? true : false;
  tracker.kokiri_c_midos_house_2 = (mem & BIT1) ? true : false;
  tracker.kokiri_c_midos_house_3 = (mem & BIT2) ? true : false;
  tracker.kokiri_c_midos_house_4 = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xAC60));
  tracker.kokiri_c_links_house_cow = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAC98));
  tracker.lonlon_c_cow_stables_1 = (mem & BIT0) ? true : false;
  tracker.lonlon_c_cow_stables_2 = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xACB4));
  tracker.kakariko_c_impas_cow = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xACB7));
  tracker.kakariko_c_cow_hp = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD1C));
  tracker.mountain_c_summit_fairy = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD1D));
  tracker.mountain_c_crater_fairy = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD1E));
  tracker.castle_town_c_ganon_fairy = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD6C));
  tracker.mountain_c_crater_grotto = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD6D));
  tracker.woods_c_wolfos_grotto = (mem & BIT1) ? true : false;
  tracker.woods_c_bomb_grotto = (mem & BIT4) ? true : false;
  tracker.mountain_c_outside_goron_grotto = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD6E));
  tracker.kakariko_c_grotto = (mem & BIT0) ? true : false;
  tracker.river_c_zora_grotto = (mem & BIT1) ? true : false;
  tracker.kakariko_c_redead_grotto = (mem & BIT2) ? true : false;
  tracker.kokiri_c_storm_grotto = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD6F));
  tracker.hyrule_field_c_hyrule_north_grotto = (mem & BIT0) ? true : false;
  tracker.hyrule_field_c_hyrule_forest_grotto = (mem & BIT2) ? true : false;
  tracker.hyrule_field_c_hyrule_south_grotto = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD78));
  tracker.mountain_c_grotto_cow = (mem & BIT0) ? true : false;
  tracker.hyrule_field_c_hyrule_west_grotto_cow = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD7B));
  tracker.hyrule_field_c_diving_heart_piece_grotto = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAD8B));
  tracker.kakariko_c_redead_grave = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xADA7));
  tracker.kakariko_c_shield_grave = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xADC3));
  tracker.kakariko_c_sun_song_chest = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAE87));
  tracker.kakariko_c_race_1 = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAE93));
  tracker.kakariko_c_windmill_hp = (mem & BIT1) ? true : false;
  tracker.kakariko_c_race_2 = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xAF00));
  tracker.lonlon_c_cow_tower_2 = (mem & BIT0) ? true : false;
  tracker.lonlon_c_cow_tower_1 = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAF03));
  tracker.lonlon_c_hp = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xAFC6));
  tracker.kakariko_c_digging = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xAFC7));
  tracker.kakariko_c_bean_hp = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xAFE2));
  tracker.river_c_zora_hp_2 = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xAFE3));
  tracker.river_c_zora_bean_salesman = (mem & BIT1) ? true : false;
  tracker.river_c_zora_hp_1 = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xAFF3));
  tracker.kokiri_c_sword = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB02B));
  tracker.lake_c_sun = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB034));
  tracker.lake_c_lab_roof = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB047));
  tracker.zoras_c_torch = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB06D));
  tracker.zoras_c_underwater = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xB06F));
  tracker.zoras_c_ice = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB07F));
  tracker.gerudo_valley_c_gerudo_hammer_rock = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB088));
  tracker.gerudo_valley_c_gerudo_river_cow = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB08B));
  tracker.gerudo_valley_c_gerudo_waterfall = (mem & BIT1) ? true : false;
  tracker.gerudo_valley_c_gerudo_crate = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0AA));
  tracker.woods_c_deku_salesman = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0AB));
  tracker.woods_c_deku_salesman_theater_2 = (mem & BIT1) ? true : false;
  tracker.woods_c_deku_salesman_theater_1 = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0B6));
  tracker.temple_spirit_c_shield = (mem & BIT1) ? true : false;
  tracker.temple_spirit_mq_c_mirror_shield = (mem & BIT1) ? true : false;
  tracker.temple_spirit_c_gauntlets = (mem & BIT3) ? true : false;
  tracker.temple_spirit_mq_c_gauntlets = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0C2));
  tracker.desert_c_colossus_hp = (mem & BIT5) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0D3));
  tracker.gerudo_c_rooftop = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0EF));
  tracker.desert_c_chest = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB0FB));
  tracker.desert_c_bombchu_wasteland = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB127));
  tracker.mountain_c_outside_goron_chest = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB130));
  tracker.mountain_c_above_dodongo = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB14E));
  tracker.mountain_c_magic_bean = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB14F));
  tracker.mountain_c_crater_wall = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB153));
  tracker.mountain_c_crater_ladder_salesman = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB15F));
  tracker.goron_c_maze_left = (mem & BIT0) ? true : false;
  tracker.goron_c_maze_right = (mem & BIT1) ? true : false;
  tracker.goron_c_maze_center = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB168));
  tracker.goron_c_spinning_pot_hp = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB16B));
  tracker.goron_c_medigoron = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB474));
  tracker.lonlon_s_wall_child_night = (mem & BIT0) ? true : false;
  tracker.lonlon_s_paddock_fence_child_night = (mem & BIT1) ? true : false;
  tracker.lonlon_s_window_child_night = (mem & BIT2) ? true : false;
  tracker.lonlon_s_tree_child_night = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB478));
  tracker.mountain_s_crater_beanspot_child = (mem & BIT0) ? true : false;
  tracker.mountain_s_cave_entrance_beanspot_child = (mem & BIT1) ? true : false;
  tracker.mountain_s_bombable = (mem & BIT2) ? true : false;
  tracker.mountain_s_before_goron_hammer_rock_adult_night = (mem & BIT3) ? true : false;
  tracker.mountain_s_wall_hammer_rock_adult_night = (mem & BIT4) ? true : false;
  tracker.goron_s_center_platform_adult = (mem & BIT5) ? true : false;
  tracker.goron_s_maze_center_child = (mem & BIT6) ? true : false;
  tracker.mountain_s_crater_box_child = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB47A));
  tracker.woods_s_deku_salesman_beanspot_child = (mem & BIT0) ? true : false;
  tracker.woods_s_mask_theater_beanspot_child = (mem & BIT1) ? true : false;
  tracker.woods_s_deku_salesman_magic_bean_adult = (mem & BIT2) ? true : false;
  tracker.woods_s_labyrinth_adult_night = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB47B));
  tracker.kokiri_s_beanspot_near_shop_child = (mem & BIT0) ? true : false;
  tracker.kokiri_s_know_it_all_brothers_child_night = (mem & BIT1) ? true : false;
  tracker.kokiri_s_twins_adult_night = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB47C));
  tracker.gerudo_valley_s_gerudo_river_beanspot_child = (mem & BIT0) ? true : false;
  tracker.gerudo_valley_s_gerudo_wood_plank_child_night = (mem & BIT1) ? true : false;
  tracker.gerudo_valley_s_gerudo_rock_near_carpenter_tent_adult_night = (mem & BIT2) ? true : false;
  tracker.gerudo_valley_s_gerudo_behind_carpenter_tent_adult_night = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB47D));
  tracker.lake_s_beanspot_child = (mem & BIT0) ? true : false;
  tracker.lake_s_near_firearrows_child_night = (mem & BIT1) ? true : false;
  tracker.lake_s_behind_lab_child_night = (mem & BIT2) ? true : false;
  tracker.lake_s_lab_box_adult = (mem & BIT3) ? true : false;
  tracker.lake_s_dead_tree_adult_night = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xB47F));
  tracker.kakariko_s_graveyard_beanspot_child_night = (mem & BIT0) ? true : false;
  tracker.kakariko_s_near_gate_child_night = (mem & BIT1) ? true : false;
  tracker.kakariko_s_tower_child_night = (mem & BIT2) ? true : false;
  tracker.kakariko_s_construction_site_child_night = (mem & BIT3) ? true : false;
  tracker.kakariko_s_skulltula_house_child_night = (mem & BIT4) ? true : false;
  tracker.kakariko_s_tree_child = (mem & BIT5) ? true : false;
  tracker.kakariko_s_impas_house_roof_adult_night = (mem & BIT6) ? true : false;
  tracker.kakariko_s_graveyard_royal_grave_child_night = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB483));
  tracker.gerudo_s_horseback_archery_target_adult_night = (mem & BIT0) ? true : false;
  tracker.gerudo_s_rooftop_adult_night = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4AA));
  tracker.zoras_c_diving = (mem & BIT0) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4AB));
  tracker.lake_c_bottle_zora = (mem & BIT1) ? true : false;
  tracker.item_zora_letter = (mem & BIT3) ? true : false;
  tracker.goron_c_darunia = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4AD));
  tracker.hyrule_field_c_ocarina_of_time = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4AF));
  tracker.woods_c_warp = (mem & BIT0) ? true : false;
  tracker.mountain_c_warp = (mem & BIT1) ? true : false;
  tracker.ice_cavern_c_sheik = (mem & BIT2) ? true : false;
  tracker.ice_cavern_mq_c_sheik = (mem & BIT2) ? true : false;
  tracker.castle_town_c_warp = (mem & BIT5) ? true : false;
  tracker.woods_c_saria = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4B7));
  tracker.gerudo_c_fortress_clear = (mem & (BIT0 | BIT1 | BIT2 | BIT3) == 0xF) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4BD));
  tracker.kokiri_c_saria_ocarina = (mem & BIT1) ? true : false;
  tracker.castle_town_c_light_arrows = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4BE));
  tracker.kakariko_c_skulltula_10 = (mem & BIT2) ? true : false;
  tracker.kakariko_c_skulltula_20 = (mem & BIT3) ? true : false;
  tracker.kakariko_c_skulltula_30 = (mem & BIT4) ? true : false;
  tracker.kakariko_c_skulltula_40 = (mem & BIT5) ? true : false;
  tracker.kakariko_c_skulltula_50 = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4C2));
  tracker.zoras_c_fairy = (mem & BIT0) ? true : false;
  tracker.castle_town_c_hyrule_fairy = (mem & BIT1) ? true : false;
  tracker.desert_c_colossus_fairy = (mem & BIT2) ? true : false;
  tracker.castle_town_c_chest_game = (mem & BIT3) ? true : false;
  tracker.woods_c_shooting = (mem & BIT5) ? true : false;
  tracker.woods_c_deku_theater_1 = (mem & BIT6) ? true : false;
  tracker.woods_c_deku_theater_2 = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4C3));
  tracker.lake_c_lab_dive = (mem & BIT0) ? true : false;
  tracker.castle_town_c_bowling_1 = (mem & BIT1) ? true : false;
  tracker.castle_town_c_bowling_2 = (mem & BIT2) ? true : false;
  tracker.kakariko_c_man_roof = (mem & BIT5) ? true : false;
  tracker.woods_c_skull_kid = (mem & BIT6) ? true : false;
  tracker.woods_c_memory_game = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4C4));
  tracker.kakariko_c_anju_adult = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4E8));
  tracker.goron_c_little_link = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4EA));
  tracker.goron_c_hot_rodder = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB475));
  tracker.hyrule_field_s_hyrule_west_grotto = (mem & BIT0) ? true : false;
  tracker.hyrule_field_s_hyrule_north_east_grotto = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB479));
  tracker.castle_town_s_near_ganon_fairy_adult = (mem & BIT0) ? true : false;
  tracker.castle_town_s_storm_grotto_child = (mem & BIT1) ? true : false;
  tracker.castle_town_s_tree_child = (mem & BIT2) ? true : false;
  tracker.castle_town_s_pot_room_child = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB47E));
  tracker.river_s_zora_ladder_child_night = (mem & BIT0) ? true : false;
  tracker.river_s_zora_tree_child = (mem & BIT1) ? true : false;
  tracker.zoras_s_wall_child_night = (mem & BIT2) ? true : false;
  tracker.river_s_zora_wall2_adult_night = (mem & BIT3) ? true : false;
  tracker.river_s_zora_wall1_adult_night = (mem & BIT4) ? true : false;
  tracker.zoras_s_fairy_hidden_cave_adult_night = (mem & BIT5) ? true : false;
  tracker.zoras_s_wall_ice_adult_night = (mem & BIT6) ? true : false;
  tracker.zoras_s_tree_child = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB482));
  tracker.desert_s_beanspot_child = (mem & BIT0) ? true : false;
  tracker.desert_s_fortress_adult = (mem & BIT1) ? true : false;
  tracker.desert_s_magic_bean_adult_night = (mem & BIT2) ? true : false;
  tracker.desert_s_palm_tree_adult_night = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB48F));
  tracker.castle_town_c_poes = (mem >= 0x64) ? true : false;
  mem = (*(vu8*)(BASE + 0xB492));
  tracker.lake_c_fishing_child = (mem & BIT2) ? true : false;
  tracker.lake_c_fishing_adult = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4A7));
  tracker.castle_town_c_maron = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4AE));
  tracker.lonlon_c_maron_song = (mem & BIT0) ? true : false;
  tracker.castle_town_c_zelda = (mem & BIT1) ? true : false;
  switch (item_locations[32].item_id) {
    case 0xBB:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA4)) & BIT6) ? true : false;
      break;
    case 0xBC:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA4)) & BIT7) ? true : false;
      break;
    case 0xBD:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT0) ? true : false;
      break;
    case 0xBE:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT1) ? true : false;
      break;
    case 0xBF:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT2) ? true : false;
      break;
    case 0xC0:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT3) ? true : false;
      break;
    case 0xC1:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT4) ? true : false;
      break;
    case 0xC2:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT5) ? true : false;
      break;
    case 0xC3:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT6) ? true : false;
      break;
    case 0xC4:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA5)) & BIT7) ? true : false;
      break;
    case 0xC5:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA6)) & BIT0) ? true : false;
      break;
    case 0xC6:
      tracker.kakariko_c_sun_song = ((*(vu8*)(SAVE + 0xA6)) & BIT1) ? true : false;
      break;
    default:
      tracker.kakariko_c_sun_song = (mem & BIT2) ? true : false;
  }
  tracker.kakariko_c_song_storm = (mem & BIT3) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4B8));
  tracker.hyrule_field_c_song_of_time = (mem & BIT1) ? true : false;
  tracker.kakariko_c_warp = (mem & BIT2) ? true : false;
  tracker.desert_c_warp_spirit = (mem & BIT4) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4BF));
  tracker.river_c_frog_game = (mem & BIT0) ? true : false;
  tracker.river_c_frog_rain = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4C0));
  tracker.kakariko_c_anju_chickens = (mem & BIT4) ? true : false;
  tracker.castle_town_c_child_shooting = (mem & BIT5) ? true : false;
  tracker.kakariko_c_adult_shooting = (mem & BIT6) ? true : false;
  tracker.gerudo_c_archer_2 = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4C1));
  tracker.lonlon_c_talon_chicken = (mem & BIT2) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4EE));
  tracker.zoras_c_king_zora = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xB4FB));
  tracker.gerudo_c_archer_1 = (mem & BIT0) ? true : false;
  tracker.castle_town_c_dog = (mem & BIT1) ? true : false;
  mem = (*(vu8*)(BASE + 0xA88C));
  tracker.deku_c_gohma = (mem & BIT7) ? true : false;
  tracker.deku_mq_c_gohma = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA8A8));
  tracker.dodongo_c_dodongo = (mem & BIT7) ? true : false;
  tracker.dodongo_mq_c_dodongo = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA8C4));
  tracker.jabujabu_c_barinade = (mem & BIT7) ? true : false;
  tracker.jabujabu_mq_c_barinade = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA8E0));
  tracker.temple_forest_c_phantomganon = (mem & BIT7) ? true : false;
  tracker.temple_forest_mq_c_phantomganon = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA8FC));
  tracker.temple_fire_c_volvagia = (mem & BIT7) ? true : false;
  tracker.temple_fire_mq_c_volvagia = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA918));
  tracker.temple_water_c_morpha = (mem & BIT7) ? true : false;
  tracker.temple_water_mq_c_morpha = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA934));
  tracker.temple_spirit_c_twinrova = (mem & BIT7) ? true : false;
  tracker.temple_spirit_mq_c_twinrova = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(BASE + 0xA950));
  tracker.temple_shadow_c_bongobongo = (mem & BIT7) ? true : false;
  tracker.temple_shadow_mq_c_bongobongo = (mem & BIT7) ? true : false;
  if (!tracker.mq_deku) {
    mem = (*(vu8*)(BASE + 0xA6A7));
    tracker.deku_c_slingshot = (mem & BIT1) ? true : false;
    tracker.deku_c_compass = (mem & BIT2) ? true : false;
    tracker.deku_c_lobby = (mem & BIT3) ? true : false;
    tracker.deku_c_basement = (mem & BIT4) ? true : false;
    tracker.deku_c_slingshot_side = (mem & BIT5) ? true : false;
    tracker.deku_c_compass_side = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46F));
    tracker.deku_s_bombable_behind_web_child = (mem & BIT0) ? true : false;
    tracker.deku_s_basement_metal_gate_child = (mem & BIT1) ? true : false;
    tracker.deku_s_basement_vines_child = (mem & BIT2) ? true : false;
    tracker.deku_s_3f_compass_side_child = (mem & BIT3) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA6A7));
    tracker.deku_mq_c_after_log = (mem & BIT0) ? true : false;
    tracker.deku_mq_c_compass = (mem & BIT1) ? true : false;
    tracker.deku_mq_c_slingshot_back = (mem & BIT2) ? true : false;
    tracker.deku_mq_c_lobby = (mem & BIT3) ? true : false;
    tracker.deku_mq_c_basement = (mem & BIT4) ? true : false;
    tracker.deku_mq_c_before_log = (mem & BIT5) ? true : false;
    tracker.deku_mq_c_slingshot = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6B7));
    tracker.deku_mq_c_basement_salesman = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46F));
    tracker.deku_mq_s_basement_back_room_child = (mem & BIT0) ? true : false;
    tracker.deku_mq_s_crate_near_map_child = (mem & BIT1) ? true : false;
    tracker.deku_mq_s_basement_grave_ceiling_child = (mem & BIT2) ? true : false;
    tracker.deku_mq_s_2f_compass_side_child = (mem & BIT3) ? true : false;
  }
  if (!tracker.mq_dodongo) {
    mem = (*(vu8*)(BASE + 0xA6C2));
    tracker.dodongo_c_map = (mem & BIT0) ? true : false;
    tracker.dodongo_c_end_bridge = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6C3));
    tracker.dodongo_c_bomb_bag = (mem & BIT4) ? true : false;
    tracker.dodongo_c_compass = (mem & BIT5) ? true : false;
    tracker.dodongo_c_bomb_flower_platform = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6D3));
    tracker.dodongo_c_near_bomb_bag_salesman_1 = (mem & BIT1) ? true : false;
    tracker.dodongo_c_east_corridor_salesman = (mem & BIT2) ? true : false;
    tracker.dodongo_c_near_bomb_bag_salesman_2 = (mem & BIT4) ? true : false;
    tracker.dodongo_c_lobby_salesman = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xA89F));
    tracker.dodongo_c_chest_above_dodongo = (mem & BIT0) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46E));
    tracker.dodongo_s_big_staircase_vines = (mem & BIT0) ? true : false;
    tracker.dodongo_s_east_corridor_scarecrow_adult = (mem & BIT1) ? true : false;
    tracker.dodongo_s_above_big_staircase_adult = (mem & BIT2) ? true : false;
    tracker.dodongo_s_near_boss_bombable = (mem & BIT3) ? true : false;
    tracker.dodongo_s_east_corridor_bombable = (mem & BIT4) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA6C3));
    tracker.dodongo_mq_c_map = (mem & BIT0) ? true : false;
    tracker.dodongo_mq_c_under_grave = (mem & BIT1) ? true : false;
    tracker.dodongo_mq_c_gohma_larva_room = (mem & BIT2) ? true : false;
    tracker.dodongo_mq_c_torch_puzzle_room = (mem & BIT3) ? true : false;
    tracker.dodongo_mq_c_bomb_bag = (mem & BIT4) ? true : false;
    tracker.dodongo_mq_c_compass = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6D2));
    tracker.dodongo_mq_c_1f_right = (mem & BIT0) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6D3));
    tracker.dodongo_mq_c_main_room_salesman_1 = (mem & BIT2) ? true : false;
    tracker.dodongo_mq_c_main_room_salesman_2 = (mem & BIT4) ? true : false;
    tracker.dodongo_mq_c_above_stairs = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xA89F));
    tracker.dodongo_mq_c_chest_above_dodongo = (mem & BIT0) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46E));
    tracker.dodongo_mq_s_back_area = (mem & BIT0) ? true : false;
    tracker.dodongo_mq_s_scrub_poe_room = (mem & BIT1) ? true : false;
    tracker.dodongo_mq_s_upper_lizalfos_room = (mem & BIT2) ? true : false;
    tracker.dodongo_mq_s_time_block_room = (mem & BIT3) ? true : false;
    tracker.dodongo_mq_s_gohma_larva_skulltula = (mem & BIT4) ? true : false;
  }
  if (!tracker.mq_jabujabu) {
    mem = (*(vu8*)(BASE + 0xA6DF));
    tracker.jabujabu_c_boomerang = (mem & BIT1) ? true : false;
    tracker.jabujabu_c_map = (mem & BIT2) ? true : false;
    tracker.jabujabu_c_compass = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6EF));
    tracker.jabujabu_c_elevator_dive_salesman = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46D));
    tracker.jabujabu_s_b1_ruto1_child = (mem & BIT0) ? true : false;
    tracker.jabujabu_s_b1_ruto2_child = (mem & BIT1) ? true : false;
    tracker.jabujabu_s_1f_near_boss_child = (mem & BIT2) ? true : false;
    tracker.jabujabu_s_b1_rising_water_child = (mem & BIT3) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA6DE));
    tracker.jabujabu_mq_c_basement_north_chest = (mem & BIT0) ? true : false;
    tracker.jabujabu_mq_c_like_like_chest = (mem & BIT1) ? true : false;
    tracker.jabujabu_mq_c_near_boss = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6DF));
    tracker.jabujabu_mq_c_compass = (mem & BIT0) ? true : false;
    tracker.jabujabu_mq_c_boomerang_room_small_chest = (mem & BIT1) ? true : false;
    tracker.jabujabu_mq_c_second_room_lower_chest = (mem & BIT2) ? true : false;
    tracker.jabujabu_mq_c_map = (mem & BIT3) ? true : false;
    tracker.jabujabu_mq_c_basement_south_chest = (mem & BIT4) ? true : false;
    tracker.jabujabu_mq_c_entry_side_chest = (mem & BIT5) ? true : false;
    tracker.jabujabu_mq_c_boomerang_chest = (mem & BIT6) ? true : false;
    tracker.jabujabu_mq_c_second_room_upper = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6E8));
    tracker.jabujabu_mq_c_cow = (mem & BIT0) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46D));
    tracker.jabujabu_mq_s_boomerang_room = (mem & BIT0) ? true : false;
    tracker.jabujabu_mq_s_near_boss_gs = (mem & BIT1) ? true : false;
    tracker.jabujabu_mq_s_electric_worm_room = (mem & BIT2) ? true : false;
    tracker.jabujabu_mq_s_invisible_enemies_room = (mem & BIT3) ? true : false;
  }
  if (!tracker.mq_temple_forest) {
    mem = (*(vu8*)(BASE + 0xA6FA));
    tracker.temple_forest_c_well = (mem & BIT1) ? true : false;
    tracker.temple_forest_c_near_boss = (mem & BIT3) ? true : false;
    tracker.temple_forest_c_bow = (mem & BIT4) ? true : false;
    tracker.temple_forest_c_poe_red = (mem & BIT5) ? true : false;
    tracker.temple_forest_c_boss_key = (mem & BIT6) ? true : false;
    tracker.temple_forest_c_poe_blue = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6FB));
    tracker.temple_forest_c_behind_lobby = (mem & BIT0) ? true : false;
    tracker.temple_forest_c_map = (mem & BIT1) ? true : false;
    tracker.temple_forest_c_floormaster = (mem & BIT2) ? true : false;
    tracker.temple_forest_c_entrance = (mem & BIT3) ? true : false;
    tracker.temple_forest_c_block_push = (mem & BIT4) ? true : false;
    tracker.temple_forest_c_outside_hookshot = (mem & BIT5) ? true : false;
    tracker.temple_forest_c_falling_room = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46C));
    tracker.temple_forest_s_outside_hookshot_adult = (mem & BIT0) ? true : false;
    tracker.temple_forest_s_entrance_adult = (mem & BIT1) ? true : false;
    tracker.temple_forest_s_pillars_adult = (mem & BIT2) ? true : false;
    tracker.temple_forest_s_lobby_adult = (mem & BIT3) ? true : false;
    tracker.temple_forest_s_near_boss_adult = (mem & BIT4) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA6FA));
    tracker.temple_forest_mq_c_well_chest = (mem & BIT1) ? true : false;
    tracker.temple_forest_mq_c_near_boss = (mem & BIT3) ? true : false;
    tracker.temple_forest_mq_c_bow = (mem & BIT4) ? true : false;
    tracker.temple_forest_mq_c_map = (mem & BIT5) ? true : false;
    tracker.temple_forest_mq_c_boss_key = (mem & BIT6) ? true : false;
    tracker.temple_forest_mq_c_compass = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA6FB));
    tracker.temple_forest_mq_c_behind_lobby = (mem & BIT0) ? true : false;
    tracker.temple_forest_mq_c_ne_outdoors_lower = (mem & BIT1) ? true : false;
    tracker.temple_forest_mq_c_redead_chest = (mem & BIT2) ? true : false;
    tracker.temple_forest_mq_c_entrance = (mem & BIT3) ? true : false;
    tracker.temple_forest_mq_c_ne_outdoors_upper = (mem & BIT5) ? true : false;
    tracker.temple_forest_mq_c_falling_room = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xB46C));
    tracker.temple_forest_mq_s_outdoor_east_adult = (mem & BIT0) ? true : false;
    tracker.temple_forest_mq_s_first_hallway_adult = (mem & BIT1) ? true : false;
    tracker.temple_forest_mq_s_outdoor_west_adult = (mem & BIT2) ? true : false;
    tracker.temple_forest_mq_s_well_adult = (mem & BIT3) ? true : false;
    tracker.temple_forest_mq_s_block_push_room_adult = (mem & BIT4) ? true : false;
  }
  if (!tracker.mq_temple_fire) {
    mem = (*(vu8*)(BASE + 0xA716));
    tracker.temple_fire_c_boulder_maze_side = (mem & BIT0) ? true : false;
    tracker.temple_fire_c_highest_goron = (mem & BIT1) ? true : false;
    tracker.temple_fire_c_map = (mem & BIT2) ? true : false;
    tracker.temple_fire_c_boulder_maze_bomb = (mem & BIT3) ? true : false;
    tracker.temple_fire_c_boss_key = (mem & BIT4) ? true : false;
    tracker.temple_fire_c_scarecrow = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xA717));
    tracker.temple_fire_c_dancer = (mem & BIT0) ? true : false;
    tracker.temple_fire_c_near_boss = (mem & BIT1) ? true : false;
    tracker.temple_fire_c_big_lava_bomb = (mem & BIT2) ? true : false;
    tracker.temple_fire_c_boulder_maze_lower = (mem & BIT3) ? true : false;
    tracker.temple_fire_c_big_lava_open = (mem & BIT4) ? true : false;
    tracker.temple_fire_c_hammer = (mem & BIT5) ? true : false;
    tracker.temple_fire_c_boulder_maze_upper = (mem & BIT6) ? true : false;
    tracker.temple_fire_c_compass = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xB473));
    tracker.temple_fire_s_big_lava_time_block_adult = (mem & BIT0) ? true : false;
    tracker.temple_fire_s_near_boss_key_adult = (mem & BIT1) ? true : false;
    tracker.temple_fire_s_boulder_maze_bombable_adult = (mem & BIT2) ? true : false;
    tracker.temple_fire_s_boulder_maze_scarecrow2_adult = (mem & BIT3) ? true : false;
    tracker.temple_fire_s_boulder_maze_scarecrow1_adult = (mem & BIT4) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA716));
    tracker.temple_fire_mq_c_boulder_maze_side = (mem & BIT0) ? true : false;
    tracker.temple_fire_mq_c_compass = (mem & BIT3) ? true : false;
    tracker.temple_fire_mq_c_map = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xA717));
    tracker.temple_fire_mq_c_megaton_hammer = (mem & BIT0) ? true : false;
    tracker.temple_fire_mq_c_big_lava_bomb = (mem & BIT1) ? true : false;
    tracker.temple_fire_mq_c_map_room_small_chest = (mem & BIT2) ? true : false;
    tracker.temple_fire_mq_c_boulder_maze_lower = (mem & BIT3) ? true : false;
    tracker.temple_fire_mq_c_boss_key = (mem & BIT4) ? true : false;
    tracker.temple_fire_mq_c_west_tower_top = (mem & BIT5) ? true : false;
    tracker.temple_fire_mq_c_boulder_maze_upper = (mem & BIT6) ? true : false;
    tracker.temple_fire_mq_c_chest_near_boss = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA720));
    tracker.temple_fire_mq_c_freestanding_key = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xB473));
    tracker.temple_fire_mq_s_big_lava_adult = (mem & BIT0) ? true : false;
    tracker.temple_fire_mq_s_above_fire_wall_maze_adult = (mem & BIT1) ? true : false;
    tracker.temple_fire_mq_s_east_tower_top_adult = (mem & BIT2) ? true : false;
    tracker.temple_fire_mq_s_fire_wall_maze_center_adult = (mem & BIT3) ? true : false;
    tracker.temple_fire_mq_s_fire_wall_maze_side_adult = (mem & BIT4) ? true : false;
  }
  if (!tracker.mq_ice_cavern) {
    mem = (*(vu8*)(BASE + 0xA7A3));
    tracker.ice_cavern_c_map = (mem & BIT0) ? true : false;
    tracker.ice_cavern_c_compass = (mem & BIT1) ? true : false;
    tracker.ice_cavern_c_boots = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7AF));
    tracker.ice_cavern_c_hp = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xB476));
    tracker.ice_cavern_s_block_puzzle_adult = (mem & BIT0) ? true : false;
    tracker.ice_cavern_s_ice_blades_adult = (mem & BIT1) ? true : false;
    tracker.ice_cavern_s_compass_room_adult = (mem & BIT2) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA7A3));
    tracker.ice_cavern_mq_c_compass = (mem & BIT0) ? true : false;
    tracker.ice_cavern_mq_c_map = (mem & BIT1) ? true : false;
    tracker.ice_cavern_mq_c_boots = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7AF));
    tracker.ice_cavern_mq_c_hp = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xB476));
    tracker.ice_cavern_mq_s_scarecrow = (mem & BIT0) ? true : false;
    tracker.ice_cavern_mq_s_hp_room = (mem & BIT1) ? true : false;
    tracker.ice_cavern_mq_s_under_ice_block = (mem & BIT2) ? true : false;
  }
  if (!tracker.mq_temple_water) {
    mem = (*(vu8*)(BASE + 0xA732));
    tracker.temple_water_c_central_bow = (mem & BIT0) ? true : false;
    tracker.temple_water_c_compass = (mem & BIT1) ? true : false;
    tracker.temple_water_c_chest_dragon = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA733));
    tracker.temple_water_c_cracked_wall = (mem & BIT0) ? true : false;
    tracker.temple_water_c_torches = (mem & BIT1) ? true : false;
    tracker.temple_water_c_map = (mem & BIT2) ? true : false;
    tracker.temple_water_c_river = (mem & BIT3) ? true : false;
    tracker.temple_water_c_boss_key = (mem & BIT5) ? true : false;
    tracker.temple_water_c_center_pillar = (mem & BIT6) ? true : false;
    tracker.temple_water_c_dark_link = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xB472));
    tracker.temple_water_s_lowest_level_south_adult = (mem & BIT0) ? true : false;
    tracker.temple_water_s_big_waterfall_adult = (mem & BIT1) ? true : false;
    tracker.temple_water_s_center_pillar_adult = (mem & BIT2) ? true : false;
    tracker.temple_water_s_near_boss_key_adult = (mem & BIT3) ? true : false;
    tracker.temple_water_s_river_adult = (mem & BIT4) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA733));
    tracker.temple_water_mq_c_longshot = (mem & BIT0) ? true : false;
    tracker.temple_water_mq_c_compass = (mem & BIT1) ? true : false;
    tracker.temple_water_mq_c_map = (mem & BIT2) ? true : false;
    tracker.temple_water_mq_c_boss_key = (mem & BIT5) ? true : false;
    tracker.temple_water_mq_c_central_pillar = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA73F));
    tracker.temple_water_mq_c_freestanding_key = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xB472));
    tracker.temple_water_mq_s_lizalfos_hallway_adult = (mem & BIT0) ? true : false;
    tracker.temple_water_mq_s_serpent_river_adult = (mem & BIT1) ? true : false;
    tracker.temple_water_mq_s_before_upper_water_switch = (mem & BIT2) ? true : false;
    tracker.temple_water_mq_s_north_basement_adult = (mem & BIT3) ? true : false;
    tracker.temple_water_mq_s_south_basement_adult = (mem & BIT4) ? true : false;
  }
  if (!tracker.mq_temple_spirit) {
    mem = (*(vu8*)(BASE + 0xA74D));
    tracker.temple_spirit_c_topmost = (mem & BIT2) ? true : false;
    tracker.temple_spirit_c_hallway_right_invisible = (mem & BIT4) ? true : false;
    tracker.temple_spirit_c_hallway_left_invisible = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xA74E));
    tracker.temple_spirit_c_child_left = (mem & BIT0) ? true : false;
    tracker.temple_spirit_c_boss_key = (mem & BIT2) ? true : false;
    tracker.temple_spirit_c_child_climb_east = (mem & BIT4) ? true : false;
    tracker.temple_spirit_c_first_mirror_left = (mem & BIT5) ? true : false;
    tracker.temple_spirit_c_first_mirror_right = (mem & BIT6) ? true : false;
    tracker.temple_spirit_c_main_room_NE = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA74F));
    tracker.temple_spirit_c_child_right = (mem & BIT0) ? true : false;
    tracker.temple_spirit_c_sun_block = (mem & BIT1) ? true : false;
    tracker.temple_spirit_c_statue_hand = (mem & BIT2) ? true : false;
    tracker.temple_spirit_c_map = (mem & BIT3) ? true : false;
    tracker.temple_spirit_c_compass = (mem & BIT4) ? true : false;
    tracker.temple_spirit_c_near_four_armos = (mem & BIT5) ? true : false;
    tracker.temple_spirit_c_child_climb_north = (mem & BIT6) ? true : false;
    tracker.temple_spirit_c_early_adult_right = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xB471));
    tracker.temple_spirit_s_childsegment_before_iron_knight = (mem & BIT0) ? true : false;
    tracker.temple_spirit_s_rolling_boulders_time_block_adult = (mem & BIT1) ? true : false;
    tracker.temple_spirit_s_coloss_room_adult = (mem & BIT2) ? true : false;
    tracker.temple_spirit_s_childsegment_sun_room = (mem & BIT3) ? true : false;
    tracker.temple_spirit_s_childsegment_left = (mem & BIT4) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA74C));
    tracker.temple_spirit_mq_c_dinolfos_ice = (mem & BIT0) ? true : false;
    tracker.temple_spirit_mq_c_beamos_room = (mem & BIT1) ? true : false;
    tracker.temple_spirit_mq_c_entrance_front_left = (mem & BIT2) ? true : false;
    tracker.temple_spirit_mq_c_entrance_front_right = (mem & BIT3) ? true : false;
    tracker.temple_spirit_mq_c_silver_block_hallway = (mem & BIT4) ? true : false;
    tracker.temple_spirit_mq_c_child_center = (mem & BIT5) ? true : false;
    tracker.temple_spirit_mq_c_entrance_back_left = (mem & BIT6) ? true : false;
    tracker.temple_spirit_mq_c_entrance_back_right = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA74D));
    tracker.temple_spirit_mq_c_mirror_puzzle_invisible = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA74E));
    tracker.temple_spirit_mq_c_child_left = (mem & BIT0) ? true : false;
    tracker.temple_spirit_mq_c_child_climb_south = (mem & BIT4) ? true : false;
    tracker.temple_spirit_mq_c_lower_ne_main_room = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA74F));
    tracker.temple_spirit_mq_c_map = (mem & BIT0) ? true : false;
    tracker.temple_spirit_mq_c_sun_block_room = (mem & BIT1) ? true : false;
    tracker.temple_spirit_mq_c_upper_ne_main_room = (mem & BIT2) ? true : false;
    tracker.temple_spirit_mq_c_compass_chest = (mem & BIT3) ? true : false;
    tracker.temple_spirit_mq_c_lower_adult_left = (mem & BIT4) ? true : false;
    tracker.temple_spirit_mq_c_boss_key = (mem & BIT5) ? true : false;
    tracker.temple_spirit_mq_c_child_climb_north = (mem & BIT6) ? true : false;
    tracker.temple_spirit_mq_c_lower_adult_right = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xB471));
    tracker.temple_spirit_mq_s_sun_block_room = (mem & BIT0) ? true : false;
    tracker.temple_spirit_mq_s_lower_adult_left = (mem & BIT1) ? true : false;
    tracker.temple_spirit_mq_s_iron_knuckles_west = (mem & BIT2) ? true : false;
    tracker.temple_spirit_mq_s_lower_adult_right = (mem & BIT3) ? true : false;
    tracker.temple_spirit_mq_s_iron_knuckles_north = (mem & BIT4) ? true : false;
  }
  if (!tracker.mq_well) {
    mem = (*(vu8*)(BASE + 0xA785));
    tracker.well_c_underwater_front = (mem & BIT0) ? true : false;
    tracker.well_c_invisible = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xA786));
    tracker.well_c_front_left_hidden_wall = (mem & BIT0) ? true : false;
    tracker.well_c_underwater_left = (mem & BIT1) ? true : false;
    tracker.well_c_locked_pits = (mem & BIT2) ? true : false;
    tracker.well_c_behind_right_grate = (mem & BIT4) ? true : false;
    tracker.well_c_center_small = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA787));
    tracker.well_c__center_large = (mem & BIT1) ? true : false;
    tracker.well_c_front_center_bombable = (mem & BIT2) ? true : false;
    tracker.well_c_defeat_boss = (mem & BIT3) ? true : false;
    tracker.well_c_back_left_bombable = (mem & BIT4) ? true : false;
    tracker.well_c_right_bottom_hidden_wall = (mem & BIT5) ? true : false;
    tracker.well_c_basement_ches = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA793));
    tracker.well_c_coffin_key = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xB477));
    tracker.well_s_locked_pits_child = (mem & BIT0) ? true : false;
    tracker.well_s_east_inner_child = (mem & BIT1) ? true : false;
    tracker.well_s_west_inner_child = (mem & BIT2) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA787));
    tracker.well_mq_c_lens_chest = (mem & BIT1) ? true : false;
    tracker.well_mq_c_compass_chest = (mem & BIT2) ? true : false;
    tracker.well_mq_c_map_chest = (mem & BIT3) ? true : false;
    mem = (*(vu8*)(BASE + 0xA793));
    tracker.well_mq_c_east_inner_freestanding_key = (mem & BIT1) ? true : false;
    tracker.well_mq_c_deadhand_freestanding_key = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xB477));
    tracker.well_mq_s_basement = (mem & BIT0) ? true : false;
    tracker.well_mq_s_west_inner = (mem & BIT1) ? true : false;
    tracker.well_mq_s_coffin_room = (mem & BIT2) ? true : false;
  }
  if (!tracker.mq_temple_shadow) {
    mem = (*(vu8*)(BASE + 0xA769));
    tracker.temple_shadow_c_after_wind_hidden = (mem & BIT4) ? true : false;
    tracker.temple_shadow_c_wind_hint = (mem & BIT5) ? true : false;
    tracker.temple_shadow_c_invisible_blades_invisible = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA76A));
    tracker.temple_shadow_c_after_wind_enemy = (mem & BIT0) ? true : false;
    tracker.temple_shadow_c_invisible_spikes = (mem & BIT1) ? true : false;
    tracker.temple_shadow_c_spike_walls = (mem & BIT2) ? true : false;
    tracker.temple_shadow_c_boss_key = (mem & BIT3) ? true : false;
    tracker.temple_shadow_c_invisible_blades_visible = (mem & BIT4) ? true : false;
    tracker.temple_shadow_c_hidden_floormaster = (mem & BIT5) ? true : false;
    mem = (*(vu8*)(BASE + 0xA76B));
    tracker.temple_shadow_c_map = (mem & BIT1) ? true : false;
    tracker.temple_shadow_c_early_silver = (mem & BIT2) ? true : false;
    tracker.temple_shadow_c_compass = (mem & BIT3) ? true : false;
    tracker.temple_shadow_c_falling_spikes_switch = (mem & BIT4) ? true : false;
    tracker.temple_shadow_c_falling_spikes_lower = (mem & BIT5) ? true : false;
    tracker.temple_shadow_c_falling_spikes_upper = (mem & BIT6) ? true : false;
    tracker.temple_shadow_c_boots = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA777));
    tracker.temple_shadow_c_giant_pot = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xB470));
    tracker.temple_shadow_s_behind_giant_pot_adult = (mem & BIT0) ? true : false;
    tracker.temple_shadow_s_falling_spikes_adult = (mem & BIT1) ? true : false;
    tracker.temple_shadow_s_behind_giant_pots_adult = (mem & BIT2) ? true : false;
    tracker.temple_shadow_s_invisible_blade_adult = (mem & BIT3) ? true : false;
    tracker.temple_shadow_s_boat_adult = (mem & BIT4) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA769));
    tracker.temple_shadow_mq_c_stalfos_room = (mem & BIT0) ? true : false;
    tracker.temple_shadow_mq_c_after_wind_hidden = (mem & BIT4) ? true : false;
    tracker.temple_shadow_mq_c_wind_hint = (mem & BIT5) ? true : false;
    tracker.temple_shadow_mq_c_invisible_blades_invisible = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA76A));
    tracker.temple_shadow_mq_c_after_wind_enemy = (mem & BIT0) ? true : false;
    tracker.temple_shadow_mq_c_invisible_spikes = (mem & BIT1) ? true : false;
    tracker.temple_shadow_mq_c_spike_walls_left = (mem & BIT2) ? true : false;
    tracker.temple_shadow_mq_c_boss_key = (mem & BIT3) ? true : false;
    tracker.temple_shadow_mq_c_invisible_blades_visible = (mem & BIT4) ? true : false;
    tracker.temple_shadow_mq_c_bomb_flower_chest = (mem & BIT5) ? true : false;
    tracker.temple_shadow_mq_c_near_ship_invisible = (mem & BIT6) ? true : false;
    tracker.temple_shadow_mq_c_beamos_silver = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA76B));
    tracker.temple_shadow_mq_c_compass = (mem & BIT1) ? true : false;
    tracker.temple_shadow_mq_c_map = (mem & BIT2) ? true : false;
    tracker.temple_shadow_mq_c_early_gibdos = (mem & BIT3) ? true : false;
    tracker.temple_shadow_mq_c_falling_spikes_switch = (mem & BIT4) ? true : false;
    tracker.temple_shadow_mq_c_falling_spikes_lower = (mem & BIT5) ? true : false;
    tracker.temple_shadow_mq_c_falling_spikes_upper = (mem & BIT6) ? true : false;
    tracker.temple_shadow_mq_c_boots = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA777));
    tracker.temple_shadow_mq_c_freestanding_key = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xB470));
    tracker.temple_shadow_mq_s_wind_hint_adult = (mem & BIT0) ? true : false;
    tracker.temple_shadow_mq_s_crusher_room_adult = (mem & BIT1) ? true : false;
    tracker.temple_shadow_mq_s_near_boss_adult = (mem & BIT2) ? true : false;
    tracker.temple_shadow_mq_s_after_wind_adult = (mem & BIT3) ? true : false;
    tracker.temple_shadow_mq_s_after_ship_adult = (mem & BIT4) ? true : false;
  }
  if (!tracker.mq_training_grounds) {
    mem = (*(vu8*)(BASE + 0xA7D9));
    tracker.training_grounds_c_hammer_room_switch = (mem & BIT0) ? true : false;
    tracker.training_grounds_c_heavy_block_before = (mem & BIT1) ? true : false;
    tracker.training_grounds_c_hammer_room_clear = (mem & BIT2) ? true : false;
    tracker.training_grounds_c_lobby_left = (mem & BIT3) ? true : false;
    tracker.training_grounds_c_heavy_block_3 = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7DA));
    tracker.training_grounds_c_maze_right = (mem & BIT0) ? true : false;
    tracker.training_grounds_c_maze_3 = (mem & BIT1) ? true : false;
    tracker.training_grounds_c_maze_2 = (mem & BIT2) ? true : false;
    tracker.training_grounds_c_hidden_ceiling = (mem & BIT3) ? true : false;
    tracker.training_grounds_c_maze_final = (mem & BIT4) ? true : false;
    tracker.training_grounds_c_underwater_silver = (mem & BIT5) ? true : false;
    tracker.training_grounds_c_heavy_block_2 = (mem & BIT6) ? true : false;
    tracker.training_grounds_c_heavy_block_1 = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7DB));
    tracker.training_grounds_c_stalfos = (mem & BIT0) ? true : false;
    tracker.training_grounds_c_beamos = (mem & BIT1) ? true : false;
    tracker.training_grounds_c_heavy_block_4 = (mem & BIT2) ? true : false;
    tracker.training_grounds_c_eye_statue = (mem & BIT3) ? true : false;
    tracker.training_grounds_c_scarecrow = (mem & BIT4) ? true : false;
    tracker.training_grounds_c_maze_right_central = (mem & BIT5) ? true : false;
    tracker.training_grounds_c_maze_1 = (mem & BIT6) ? true : false;
    tracker.training_grounds_c_lobby_right = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7E7));
    tracker.training_grounds_c_maze_right_key = (mem & BIT1) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA7D9));
    tracker.training_grounds_mq_c_before_heavy_block = (mem & BIT1) ? true : false;
    tracker.training_grounds_mq_c_second_iron_knuckle = (mem & BIT2) ? true : false;
    tracker.training_grounds_mq_c_lobby_left = (mem & BIT3) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7DA));
    tracker.training_grounds_mq_c_maze_right_side = (mem & BIT0) ? true : false;
    tracker.training_grounds_mq_c_maze_3 = (mem & BIT1) ? true : false;
    tracker.training_grounds_mq_c_maze_2 = (mem & BIT2) ? true : false;
    tracker.training_grounds_mq_c_hidden_ceiling = (mem & BIT3) ? true : false;
    tracker.training_grounds_mq_c_underwater = (mem & BIT5) ? true : false;
    tracker.training_grounds_mq_c_flame_circle = (mem & BIT6) ? true : false;
    mem = (*(vu8*)(BASE + 0xA7DB));
    tracker.training_grounds_mq_c_first_iron_knuckle = (mem & BIT0) ? true : false;
    tracker.training_grounds_mq_c_dinolfos = (mem & BIT1) ? true : false;
    tracker.training_grounds_mq_c_heavy_block = (mem & BIT2) ? true : false;
    tracker.training_grounds_mq_c_eye_statue = (mem & BIT3) ? true : false;
    tracker.training_grounds_mq_c_ice_arrows = (mem & BIT4) ? true : false;
    tracker.training_grounds_mq_c_maze_right_central = (mem & BIT5) ? true : false;
    tracker.training_grounds_mq_c_maze_1 = (mem & BIT6) ? true : false;
    tracker.training_grounds_mq_c_lobby_right = (mem & BIT7) ? true : false;
  }
  if (!tracker.mq_castle_ganon) {
    mem = (*(vu8*)(BASE + 0xA7BE));
    tracker.castle_ganon_c_boss_key = (mem & BIT3) ? true : false;
    mem = (*(vu8*)(BASE + 0xA811));
    tracker.castle_ganon_c_light_trail_invisible_enemies = (mem & BIT0) ? true : false;
    tracker.castle_ganon_c_light_trial_lullaby = (mem & BIT1) ? true : false;
    tracker.castle_ganon_c_spirit_trial_first = (mem & BIT2) ? true : false;
    tracker.castle_ganon_c_spirit_trial_second = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xA812));
    tracker.castle_ganon_c_shadow_trial_first = (mem & BIT0) ? true : false;
    tracker.castle_ganon_c_forest_trial = (mem & BIT1) ? true : false;
    tracker.castle_ganon_c_light_trial_second_right = (mem & BIT2) ? true : false;
    tracker.castle_ganon_c_light_trial_second_left = (mem & BIT3) ? true : false;
    tracker.castle_ganon_c_light_trial_first_left = (mem & BIT4) ? true : false;
    tracker.castle_ganon_c_light_trial_third_left = (mem & BIT5) ? true : false;
    tracker.castle_ganon_c_light_trial_first_right = (mem & BIT6) ? true : false;
    tracker.castle_ganon_c_light_trial_third_right = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA813));
    tracker.castle_ganon_c_shadow_trial_second = (mem & BIT5) ? true : false;
    tracker.castle_ganon_c_water_trial_right = (mem & BIT6) ? true : false;
    tracker.castle_ganon_c_water_trial_left = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA822));
    tracker.castle_ganon_c_under_bridge_salesman_4 = (mem & BIT0) ? true : false;
    tracker.castle_ganon_c_under_bridge_salesman_1 = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xA823));
    tracker.castle_ganon_c_under_bridge_salesman_3 = (mem & BIT4) ? true : false;
    tracker.castle_ganon_c_under_bridge_salesman_2 = (mem & BIT6) ? true : false;
  }
  else {
    mem = (*(vu8*)(BASE + 0xA7BE));
    tracker.castle_ganon_mq_c_boss_key = (mem & BIT3) ? true : false;
    mem = (*(vu8*)(BASE + 0xA811));
    tracker.castle_ganon_mq_c_spirit_trial_second = (mem & BIT4) ? true : false;
    mem = (*(vu8*)(BASE + 0xA812));
    tracker.castle_ganon_mq_c_spirit_sun_back_left = (mem & BIT0) ? true : false;
    tracker.castle_ganon_mq_c_spirit_sun_back_right = (mem & BIT1) ? true : false;
    tracker.castle_ganon_mq_c_spirit_trial_first = (mem & BIT2) ? true : false;
    mem = (*(vu8*)(BASE + 0xA813));
    tracker.castle_ganon_mq_c_shadow_trial_first = (mem & BIT0) ? true : false;
    tracker.castle_ganon_mq_c_water_trial = (mem & BIT1) ? true : false;
    tracker.castle_ganon_mq_c_forest_trial_first = (mem & BIT2) ? true : false;
    tracker.castle_ganon_mq_c_forest_trial_second = (mem & BIT3) ? true : false;
    tracker.castle_ganon_mq_c_light_trial_lullaby = (mem & BIT4) ? true : false;
    tracker.castle_ganon_mq_c_shadow_trial_second = (mem & BIT5) ? true : false;
    tracker.castle_ganon_mq_c_spirit_golden = (mem & BIT6) ? true : false;
    tracker.castle_ganon_mq_c_spirit_sun_front_left = (mem & BIT7) ? true : false;
    mem = (*(vu8*)(BASE + 0xA81F));
    tracker.castle_ganon_mq_c_forest_trial_key = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xA822));
    tracker.castle_ganon_mq_c_under_bridge_salesman_4 = (mem & BIT0) ? true : false;
    tracker.castle_ganon_mq_c_under_bridge_salesman_1 = (mem & BIT1) ? true : false;
    mem = (*(vu8*)(BASE + 0xA823));
    tracker.castle_ganon_mq_c_under_bridge_salesman_5 = (mem & BIT1) ? true : false;
    tracker.castle_ganon_mq_c_under_bridge_salesman_3 = (mem & BIT4) ? true : false;
    tracker.castle_ganon_mq_c_under_bridge_salesman_2 = (mem & BIT6) ? true : false;
  }

  tracker.item_magic_power = (*(vu8*)(SAVE + 0x32));

  mem = (*(vu8*)(SAVE + 0xA3));
  tracker.item_bow = mem & (BIT0 | BIT1);
  tracker.item_bombs = (mem & (BIT3 | BIT4)) >> 3;
  tracker.item_glove = (mem & (BIT6 | BIT7)) >> 6;

  mem = (*(vu8*)(SAVE + 0xA2));
  tracker.item_scale = (mem & (BIT1 | BIT2)) >> 1;
  tracker.item_wallet = (mem & (BIT4 | BIT5)) >> 4;
  tracker.item_slingshot = (mem & (BIT6 | BIT7)) >> 6;

  mem = ((*(vu8*)(SAVE + 0xA1)) & (BIT1 | BIT2 | BIT3)) >> 1;
  if (mem == 5) mem = 2;
  tracker.item_stick = mem;
  tracker.item_nut = ((*(vu8*)(SAVE + 0xA1)) & (BIT4 | BIT5)) >> 4;
  tracker.item_arrow_fire = ((*(vu8*)(SAVE + 0x74 + 4)) == 0x04) ? true : false;
  tracker.item_magic_din = ((*(vu8*)(SAVE + 0x74 + 5)) == 0x05) ? true : false;
  switch ((*(vu8*)(SAVE + 0x74 + 7))) {
    case 0x07:
      tracker.item_ocarina = 1;
      break;
    case 0x08:
      tracker.item_ocarina = 2;
      break;
  }
  tracker.item_bombchu = ((*(vu8*)(SAVE + 0x74 + 8)) == 0x09) ? true : false;
  switch ((*(vu8*)(SAVE + 0x74 + 9))) {
    case 0x0A:
      tracker.item_hookshot = 1;
      break;
    case 0x0B:
      tracker.item_hookshot = 2;
      break;
  }
  tracker.item_arrow_ice = ((*(vu8*)(SAVE + 0x74 + 10)) == 0x0C) ? true : false;
  tracker.item_magic_farore = ((*(vu8*)(SAVE + 0x74 + 11)) == 0x0D) ? true : false;
  tracker.item_boomerang = ((*(vu8*)(SAVE + 0x74 + 12)) == 0x0E) ? true : false;
  tracker.item_lens = ((*(vu8*)(SAVE + 0x74 + 13)) == 0x0F) ? true : false;
  mem = 0;
  if ((*(vu8*)(SAVE + 0x74 + 14)) == 0x10) {
    mem = (*(vu8*)(SAVE + 0x8C + 14));
    if ((*(vu8*)(BASE + 0xAFBF)) & BIT3) mem++;
    if ((*(vu8*)(BASE + 0xAFDB)) & BIT3) mem++;
    if ((*(vu8*)(BASE + 0xAFF6)) & BIT1) mem++;
    if ((*(vu8*)(BASE + 0xB02F)) & BIT1) mem++;
    if ((*(vu8*)(BASE + 0xB083)) & BIT3) mem++;
    if ((*(vu8*)(BASE + 0xB09D)) & BIT2) mem++;
    if ((*(vu8*)(BASE + 0xB09F)) & BIT4) mem++;
    if ((*(vu8*)(BASE + 0xB0B8)) & BIT0) mem++;
    if ((*(vu8*)(BASE + 0xB12B)) & BIT6) mem++;
    if ((*(vu8*)(BASE + 0xB147)) & BIT3) mem++;
  }
  tracker.item_bean = mem;
  tracker.item_hammer = ((*(vu8*)(SAVE + 0x74 + 15)) == 0x11) ? true : false;
  tracker.item_arrow_light = ((*(vu8*)(SAVE + 0x74 + 16)) == 0x12) ? true : false;
  tracker.item_magic_nayru = ((*(vu8*)(SAVE + 0x74 + 17)) == 0x13) ? true : false;
  tracker.item_poe_big = (*(vu32*)(SAVE + 0xEBC)) / 10;
  tracker.item_bottle = 0;
  for (u8 i = 18; i < 22; i++) {
    mem = (*(vu8*)(SAVE + 0x74 + i));
    if (mem == 0x1B) tracker.item_zora_letter = true;
    else if (mem == 0x1E) tracker.item_poe_big++;
    else if (mem != 0xFF) tracker.item_bottle++;
  }
  mem = (*(vu8*)(SAVE + 0x74 + 22));
  if (mem >= 0x2D && mem <= 0x37) mem -= 0x2C;
  else mem = 0;
  tracker.item_cojiro = mem;
  if (tracker.castle_town_c_zelda) {
    if ((mem = (*(vu8*)(BASE + 0xB4C6))) & BIT3) mem = 8;
    else if (mem & BIT2) mem = 7;
    else if (mem & BIT1) mem = 6;
    else if ((mem = (*(vu8*)(BASE + 0xB4D7))) & BIT7) mem = 5;
    else if (mem & BIT6) mem = 4;
    else mem = 3;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x74 + 23));
    if (mem >= 0x21 && mem <= 0x23) mem -= 0x20;
    else mem = 0;
  }
  tracker.item_egg = mem;
  mem = (*(vu8*)(SAVE + 0x9D));
  tracker.item_sword_kokiri = (mem & BIT0) ? true : false;
  tracker.item_sword_master = (mem & BIT1) ? true : false;
  tracker.item_sword_biggoron = (*(vu8*)(BASE + 0xA60E)) ? ((mem & BIT2) ? true : false) : false;
  tracker.item_shield_kokiri = (mem & BIT4) ? true : false;
  tracker.item_shield_hylia = (mem & BIT5) ? true : false;
  tracker.item_shield_mirror = (mem & BIT6) ? true : false;
  mem = (*(vu8*)(SAVE + 0x9C));
  tracker.item_tunic_fire = (mem & BIT1) ? true : false;
  tracker.item_tunic_water = (mem & BIT2) ? true : false;
  tracker.item_boots_iron = (mem & BIT5) ? true : false;
  tracker.item_boots_hover = (mem & BIT6) ? true : false;
  tracker.item_skulltula = (*(vu8*)(SAVE + 0xD1));
  tracker.item_heart_piece = (*(vu8*)(SAVE + 0xA4)) / 0x10;
  mem = (*(vu16*)(SAVE + 0x2E)) / 0x10 - 3;
  if (mem > 8) {
    tracker.item_heart_piece += (mem-8)*4;
    tracker.item_heart_container = 8;
  }
  else tracker.item_heart_container = mem;
  tracker.item_heart_double = (*(vu8*)(SAVE + 0xCF)) ? true : false;
  mem = (*(vu8*)(SAVE + 0xA7));
  tracker.item_medallion_forest = (mem & BIT0) ? true : false;
  tracker.item_medallion_fire = (mem & BIT1) ? true : false;
  tracker.item_medallion_water = (mem & BIT2) ? true : false;
  tracker.item_medallion_spirit = (mem & BIT3) ? true : false;
  tracker.item_medallion_shadow = (mem & BIT4) ? true : false;
  tracker.item_medallion_light = (mem & BIT5) ? true : false;
  tracker.item_warp_forest = (mem & BIT6) ? true : false;
  tracker.item_warp_fire = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(SAVE + 0xA6));
  tracker.item_warp_water = (mem & BIT0) ? true : false;
  tracker.item_warp_spirit = (mem & BIT1) ? true : false;
  tracker.item_warp_shadow = (mem & BIT2) ? true : false;
  tracker.item_warp_light = (mem & BIT3) ? true : false;
  tracker.item_song_zelda = (mem & BIT4) ? true : false;
  tracker.item_song_epona = (mem & BIT5) ? true : false;
  tracker.item_song_saria = (mem & BIT6) ? true : false;
  tracker.item_song_sun = (mem & BIT7) ? true : false;
  mem = (*(vu8*)(SAVE + 0xA5));
  tracker.item_song_time = (mem & BIT0) ? true : false;
  tracker.item_song_storm = (mem & BIT1) ? true : false;
  tracker.item_stone_forest = (mem & BIT2) ? true : false;
  tracker.item_stone_fire = (mem & BIT3) ? true : false;
  tracker.item_stone_water = (mem & BIT4) ? true : false;
  tracker.item_stone_of_agony = (mem & BIT5) ? true : false;
  tracker.item_membership = (mem & BIT6) ? true : false;
  tracker.item_song_scarecrow = ((*(vu8*)(BASE + 0xB4B6)) & BIT4) ? true : false;

  tracker.compass_deku              = (z64_file.dungeon_items[ 0].compass ) ? true : false;
  tracker.compass_dodongo           = (z64_file.dungeon_items[ 1].compass ) ? true : false;
  tracker.compass_jabujabu          = (z64_file.dungeon_items[ 2].compass ) ? true : false;
  tracker.compass_temple_forest     = (z64_file.dungeon_items[ 3].compass ) ? true : false;
  tracker.compass_temple_fire       = (z64_file.dungeon_items[ 4].compass ) ? true : false;
  tracker.compass_temple_water      = (z64_file.dungeon_items[ 5].compass ) ? true : false;
  tracker.compass_temple_spirit     = (z64_file.dungeon_items[ 6].compass ) ? true : false;
  tracker.compass_temple_shadow     = (z64_file.dungeon_items[ 7].compass ) ? true : false;
  tracker.compass_well              = (z64_file.dungeon_items[ 8].compass ) ? true : false;
  tracker.compass_ice               = (z64_file.dungeon_items[ 9].compass ) ? true : false;
  tracker.dungeon_map_deku          = (z64_file.dungeon_items[ 0].map     ) ? true : false;
  tracker.dungeon_map_dodongo       = (z64_file.dungeon_items[ 1].map     ) ? true : false;
  tracker.dungeon_map_jabujabu      = (z64_file.dungeon_items[ 2].map     ) ? true : false;
  tracker.dungeon_map_temple_forest = (z64_file.dungeon_items[ 3].map     ) ? true : false;
  tracker.dungeon_map_temple_fire   = (z64_file.dungeon_items[ 4].map     ) ? true : false;
  tracker.dungeon_map_temple_water  = (z64_file.dungeon_items[ 5].map     ) ? true : false;
  tracker.dungeon_map_temple_spirit = (z64_file.dungeon_items[ 6].map     ) ? true : false;
  tracker.dungeon_map_temple_shadow = (z64_file.dungeon_items[ 7].map     ) ? true : false;
  tracker.dungeon_map_well          = (z64_file.dungeon_items[ 8].map     ) ? true : false;
  tracker.dungeon_map_ice           = (z64_file.dungeon_items[ 9].map     ) ? true : false;
  tracker.key_boss_forest           = (z64_file.dungeon_items[ 3].boss_key) ? true : false;
  tracker.key_boss_fire             = (z64_file.dungeon_items[ 4].boss_key) ? true : false;
  tracker.key_boss_water            = (z64_file.dungeon_items[ 5].boss_key) ? true : false;
  tracker.key_boss_spirit           = (z64_file.dungeon_items[ 6].boss_key) ? true : false;
  tracker.key_boss_shadow           = (z64_file.dungeon_items[ 7].boss_key) ? true : false;
  tracker.key_boss_ganon            = (z64_file.dungeon_items[10].boss_key) ? true : false;

  tracker.key_small_forest = z64_file.dungeon_keys[3];
  if (tracker.key_small_forest == 0xF) tracker.key_small_forest = 0;
  if (tracker.mq_temple_forest) {
    mem = (*(vu8*)(SAVE + 0x12F));
    if (mem & BIT0) tracker.key_small_forest++;
    if (mem & BIT1) tracker.key_small_forest++;
    if (mem & BIT2) tracker.key_small_forest++;
    if (mem & BIT3) tracker.key_small_forest++;
    if (mem & BIT4) tracker.key_small_forest++;
    if (mem & BIT6) tracker.key_small_forest++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x12F));
    if (mem & BIT0) tracker.key_small_forest++;
    if (mem & BIT1) tracker.key_small_forest++;
    if (mem & BIT2) tracker.key_small_forest++;
    if (mem & BIT3) tracker.key_small_forest++;
    if (mem & BIT4) tracker.key_small_forest++;
  }
  tracker.key_small_fire = z64_file.dungeon_keys[4];
  if (tracker.key_small_fire == 0xF) tracker.key_small_fire = 0;
  if (tracker.mq_temple_fire) {
    mem = (*(vu8*)(SAVE + 0x148));
    if (mem & BIT0) tracker.key_small_fire++;
    if (mem & BIT2) tracker.key_small_fire++;
    if (mem & BIT3) tracker.key_small_fire++;
    if (mem & BIT6) tracker.key_small_fire++;
    mem = (*(vu8*)(SAVE + 0x149));
    if (mem & BIT7) tracker.key_small_fire++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x148));
    if (mem & BIT0) tracker.key_small_fire++;
    if (mem & BIT1) tracker.key_small_fire++;
    if (mem & BIT2) tracker.key_small_fire++;
    if (mem & BIT3) tracker.key_small_fire++;
    if (mem & BIT5) tracker.key_small_fire++;
    if (mem & BIT6) tracker.key_small_fire++;
    if (mem & BIT7) tracker.key_small_fire++;
  }
  tracker.key_small_water = z64_file.dungeon_keys[5];
  if (tracker.key_small_water == 0xF) tracker.key_small_water = 0;
  if (tracker.mq_temple_water) {
    mem = (*(vu8*)(SAVE + 0x165));
    if (mem & BIT5) tracker.key_small_water++;
    mem = (*(vu8*)(SAVE + 0x167));
    if (mem & BIT2) tracker.key_small_water++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x166));
    if (mem & BIT1) tracker.key_small_water++;
    mem = (*(vu8*)(SAVE + 0x167));
    if (mem & BIT1) tracker.key_small_water++;
    if (mem & BIT2) tracker.key_small_water++;
    if (mem & BIT5) tracker.key_small_water++;
    if (mem & BIT6) tracker.key_small_water++;
  }
  tracker.key_small_spirit = z64_file.dungeon_keys[6];
  if (tracker.key_small_spirit == 0xF) tracker.key_small_spirit = 0;
  if (tracker.mq_temple_spirit) {
    mem = (*(vu8*)(SAVE + 0x180));
    if (mem & BIT3) tracker.key_small_spirit++;
    if (mem & BIT4) tracker.key_small_spirit++;
    if (mem & BIT6) tracker.key_small_spirit++;
    mem = (*(vu8*)(SAVE + 0x181));
    if (mem & BIT2) tracker.key_small_spirit++;
    if (mem & BIT5) tracker.key_small_spirit++;
    mem = (*(vu8*)(SAVE + 0x183));
    if (mem & BIT1) tracker.key_small_spirit++;
    if (mem & BIT3) tracker.key_small_spirit++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x180));
    if (mem & BIT3) tracker.key_small_spirit++;
    if (mem & BIT4) tracker.key_small_spirit++;
    if (mem & BIT6) tracker.key_small_spirit++;
    mem = (*(vu8*)(SAVE + 0x181));
    if (mem & BIT5) tracker.key_small_spirit++;
    mem = (*(vu8*)(SAVE + 0x182));
    if (mem & BIT5) tracker.key_small_spirit++;
  }
  tracker.key_small_well = z64_file.dungeon_keys[8];
  if (tracker.key_small_well == 0xF) tracker.key_small_well = 0;
  if (tracker.mq_well) {
    mem = (*(vu8*)(SAVE + 0x1B9));
    if (mem & BIT4) tracker.key_small_well++;
    if (mem & BIT5) tracker.key_small_well++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x1B8));
    if (mem & BIT3) tracker.key_small_well++;
    if (mem & BIT4) tracker.key_small_well++;
    if (mem & BIT5) tracker.key_small_well++;
  }
  tracker.key_small_shadow = z64_file.dungeon_keys[7];
  if (tracker.key_small_shadow == 0xF) tracker.key_small_shadow = 0;
  if (tracker.mq_temple_shadow) {
    mem = (*(vu8*)(SAVE + 0x19C));
    if (mem & BIT0) tracker.key_small_shadow++;
    if (mem & BIT1) tracker.key_small_shadow++;
    if (mem & BIT3) tracker.key_small_shadow++;
    mem = (*(vu8*)(SAVE + 0x19D));
    if (mem & BIT5) tracker.key_small_shadow++;
    if (mem & BIT6) tracker.key_small_shadow++;
    if (mem & BIT7) tracker.key_small_shadow++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x19C));
    if (mem & BIT0) tracker.key_small_shadow++;
    if (mem & BIT1) tracker.key_small_shadow++;
    mem = (*(vu8*)(SAVE + 0x19D));
    if (mem & BIT5) tracker.key_small_shadow++;
    if (mem & BIT6) tracker.key_small_shadow++;
    if (mem & BIT7) tracker.key_small_shadow++;
  }
  tracker.key_small_training = z64_file.dungeon_keys[11];
  if (tracker.key_small_training == 0xF) tracker.key_small_training = 0;
  if (tracker.mq_training_grounds) {
    mem = (*(vu8*)(SAVE + 0x20C));
    if (mem & BIT5) tracker.key_small_training++;
    mem = (*(vu8*)(SAVE + 0x20D));
    if (mem & BIT4) tracker.key_small_training++;
    if (mem & BIT7) tracker.key_small_training++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x20D));
    if (mem & BIT7) tracker.key_small_training++;
    mem = (*(vu8*)(SAVE + 0x20E));
    if (mem & BIT1) tracker.key_small_training++;
    if (mem & BIT2) tracker.key_small_training++;
    mem = (*(vu8*)(SAVE + 0x20F));
    if (mem & BIT1) tracker.key_small_training++;
    if (mem & BIT3) tracker.key_small_training++;
    if (mem & BIT4) tracker.key_small_training++;
    if (mem & BIT5) tracker.key_small_training++;
    if (mem & BIT6) tracker.key_small_training++;
    if (mem & BIT7) tracker.key_small_training++;
  }
  tracker.key_small_gerudo = z64_file.dungeon_keys[12];
  if (tracker.key_small_gerudo == 0xF) tracker.key_small_gerudo = 0;
  mem = (*(vu8*)(SAVE + 0x22B));
  if (mem & BIT1) tracker.key_small_gerudo++;
  if (mem & BIT2) tracker.key_small_gerudo++;
  if (mem & BIT3) tracker.key_small_gerudo++;
  if (mem & BIT4) tracker.key_small_gerudo++;
  tracker.key_small_ganon = z64_file.dungeon_keys[13];
  if (tracker.key_small_ganon == 0xF) tracker.key_small_ganon = 0;
  if (tracker.mq_castle_ganon) {
    mem = (*(vu8*)(SAVE + 0x245));
    if (mem & BIT4) tracker.key_small_ganon++;
    if (mem & BIT5) tracker.key_small_ganon++;
    if (mem & BIT6) tracker.key_small_ganon++;
  }
  else {
    mem = (*(vu8*)(SAVE + 0x244));
    if (mem & BIT5) tracker.key_small_ganon++;
    if (mem & BIT6) tracker.key_small_ganon++;
  }

  tracker.is_child = z64_file.link_age ? true : false;

  if (!usb_vars.initialized || ((*(vu8*)(0x801CA20C)) == 0 && (*(vu32*)(0x801C853C)) >= 20)) {
    tracker.location = z64_game.scene_index;

    {
      u32 list = 0;
      u32 inventory = 0x801E7B00;
      u8 bought = 0;
      u8 item_location = 0;
      switch (tracker.location) {
        case 0x2C:
          if (z64_file.link_age) {
            list = 0x801F510C;
            inventory = 0x801E7F40 + 0x100;
            bought = (*(vu8*)(SAVE + 0x5B5)) & 0x0F;
            item_location = 12;
          }
          else {
            list = 0x801F143C;
            inventory = 0x801E7F40 + 0x140;
            bought = (*(vu8*)(SAVE + 0x5B4)) >> 4;
            item_location = 8;
          }
          break;
        case 0x2D:
          for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA104)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
            if (*actor_id == 0x0004) {
              list = (*(vu32*)((u32)(actor_id) + 0x134));
              if (list) {
                list += 0xDC;
                break;
              }
            }
          }
          inventory += 0x00;
          bought = (*(vu8*)(SAVE + 0x5B4)) & 0x0F;
          item_location = 0;
          break;
        case 0x2E:
          list = 0x801EAE8C;
          inventory += 0x200;
          bought = (*(vu8*)(SAVE + 0x5B5)) >> 4;
          item_location = 16;
          break;
        case 0x2F:
          list = 0x801EDB5C;
          inventory += 0x1C0;
          bought = (*(vu8*)(SAVE + 0x5B6)) & 0x0F;
          item_location = 28;
          break;
        case 0x30:
          if (!z64_file.link_age) {
            list = 0x801EC54C;
            inventory += 0x40;
            bought = (*(vu8*)(SAVE + 0x5B6)) >> 4;
            item_location = 4;
          }
          break;
        case 0x31:
          list = 0x801EEAFC;
          inventory += 0xC0;
          bought = (*(vu8*)(SAVE + 0x5B7)) & 0x0F;
          item_location = 20;
          break;
        case 0x32:
          list = 0x801EAE2C;
          inventory += 0x80;
          bought = (*(vu8*)(SAVE + 0x5B7)) >> 4;
          item_location = 24;
          break;
      }
      for (u8 i = 0; i < 8; i++) {
        tracker.shop_items[i] = 0;
        tracker.shop_players[i] = 0;
        tracker.shop_bought_prices[i] = 0;
      }
      if (list) {
        override_value_t item;
        tracker.shop_items[6] = (*(vu16*)(inventory + 0x08 * 0));
        tracker.shop_bought_prices[6] = (*(vu16*)(list + tracker.shop_items[6] * 0x20 + 0x08));
        tracker.shop_items[2] = (*(vu16*)(inventory + 0x08 * 1));
        tracker.shop_bought_prices[2] = (*(vu16*)(list + tracker.shop_items[2] * 0x20 + 0x08));
        tracker.shop_items[7] = (*(vu16*)(inventory + 0x08 * 2));
        tracker.shop_bought_prices[7] = (*(vu16*)(list + tracker.shop_items[7] * 0x20 + 0x08));
        tracker.shop_items[3] = (*(vu16*)(inventory + 0x08 * 3));
        tracker.shop_bought_prices[3] = (*(vu16*)(list + tracker.shop_items[3] * 0x20 + 0x08));
        tracker.shop_items[5] = (*(vu16*)(inventory + 0x08 * 4));
        tracker.shop_bought_prices[5] = (*(vu16*)(list + tracker.shop_items[5] * 0x20 + 0x08));
        item.all = item_locations[item_location + 0].all;
        if (item.all) {
          tracker.shop_items[5] = item.item_id;
          tracker.shop_players[5] = item.player;
          tracker.shop_bought_prices[5] |= (bought & BIT0) ? 0x8000 : 0;
        }
        tracker.shop_items[1] = (*(vu16*)(inventory + 0x08 * 5));
        tracker.shop_bought_prices[1] = (*(vu16*)(list + tracker.shop_items[1] * 0x20 + 0x08));
        item.all = item_locations[item_location + 1].all;
        if (item.all) {
          tracker.shop_items[1] = item.item_id;
          tracker.shop_players[1] = item.player;
          tracker.shop_bought_prices[1] |= (bought & BIT1) ? 0x8000 : 0;
        }
        tracker.shop_items[4] = (*(vu16*)(inventory + 0x08 * 6));
        tracker.shop_bought_prices[4] = (*(vu16*)(list + tracker.shop_items[4] * 0x20 + 0x08));
        item.all = item_locations[item_location + 2].all;
        if (item.all) {
          tracker.shop_items[4] = item.item_id;
          tracker.shop_players[4] = item.player;
          tracker.shop_bought_prices[4] |= (bought & BIT2) ? 0x8000 : 0;
        }
        tracker.shop_items[0] = (*(vu16*)(inventory + 0x08 * 7));
        tracker.shop_bought_prices[0] = (*(vu16*)(list + tracker.shop_items[0] * 0x20 + 0x08));
        item.all = item_locations[item_location + 3].all;
        if (item.all) {
          tracker.shop_items[0] = item.item_id;
          tracker.shop_players[0] = item.player;
          tracker.shop_bought_prices[0] |= (bought & BIT3) ? 0x8000 : 0;
        }
      }
    }

    tracker.last_exit = (*(vu16*)(0x8011A5D2));
    tracker.grotto_id = (*(vu8*)(0x80402E36));

    u32 exit_list = (*(vu32*)(0x801DA2A4));
    for (u8 i = 0; i < 16; i++) {
      if (
          (tracker.location == 0x55 && i > 10)
        || (tracker.location == 0x5B && i >  9)
        || (tracker.location == 0x56 && i >  1)
        || (tracker.location == 0x54 && i >  4)
        || (tracker.location == 0x58 && i >  3)
        || (tracker.location == 0x59 && i >  4)
        || (tracker.location == 0x52 && i > 13)
        || (tracker.location == 0x53 && i >  6)
        || (tracker.location == 0x60 && i >  4)
        || (tracker.location == 0x62 && i >  3)
        || (tracker.location == 0x61 && i >  3)
        || (tracker.location == 0x22 && i >  2)
        || (tracker.location == 0x1E && i >  3)
        || (tracker.location == 0x5F && i >  2)
        || (tracker.location == 0x64 && i >  2)
        || (tracker.location == 0x51 && i >  7)
        || (tracker.location == 0x63 && i >  6)
        || (tracker.location == 0x5A && i >  4)
        || (tracker.location == 0x5E && i >  1)
        || (tracker.location == 0x5C && i >  4)
        || ((tracker.location == 0x1B || tracker.location == 0x1C || tracker.location == 0x1D) && i > 2)
        || ((tracker.location == 0x20 || tracker.location == 0x21) && i > 10)
        || ((tracker.location == 0x23 || tracker.location == 0x24 || tracker.location == 0x25) && i > 1)
      ) tracker.exits[i] = 0;
      else tracker.exits[i] = (*(vu16*)(exit_list + i*2));
      if (i < 8) {
        tracker.grotto_ids[i] = 0xFFFF;
        tracker.grotto_types[i] = 0xFF;
      }
    }
    switch (tracker.location) {
      case 0x55:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC4000000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x43BE0000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC4990000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[0]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[0] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x5B:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x4464C000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC4674000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x42A00000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC1A00000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC4C80000
            ) i = 1;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x44278000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC51D8000
            ) i = 2;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x56:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC3430000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x44ED8000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x42340000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x435C0000
            ) i = 1;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x439B0000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x43F00000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC50FC000
            ) i = 2;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x54:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x44278000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x440E8000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC3B68000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC4CBC000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x42C80000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC3020000
            ) i = 1;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x43B40000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x440E8000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x43020000
            ) i = 2;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x58:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC4570000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x41600000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC3EB0000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x52:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC3C80000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x43C80000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x44570000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x42A00000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC3820000
            ) i = 1;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x60:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC3BF8000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x44AD4000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC496C000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC42C0000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x44F34000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC38E8000
            ) i = 1;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x62:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x44898000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x44110000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC494C000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x61:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC4D46000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x44348000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC3EC0000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x42200000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x449A2000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x44DD4000
            ) i = 1;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x5F:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x44790000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x44C46000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x44530000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x51:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC58B1000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC3960000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC3D48000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC59BF000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC42F0000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x4657F000
            ) i = 1;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x4500C000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x41A00000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC32A0000
            ) i = 2;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC5F5F000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC3960000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x45D84000
            ) i = 3;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC59A8800
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC3960000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x45313000
            ) i = 4;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC3870000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC3FA0000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x4640F800
            ) i = 5;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC57BE000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC42F0000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x46589000
            ) i = 6;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC4B22000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x444A8000
            ) i = 7;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x63:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x44E10000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x00000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x44BB8000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x57:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC53E0000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC4812000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x45BDD800
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x5A:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0xC4A56000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x41700000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC4724000
            ) i = 0;
            else if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x438C0000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC40AC000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0x44B7C000
            ) i = 1;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x5D:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x43BC0000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0x43A68000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC4C38000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
      case 0x5C:
        for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA10C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
          if (*actor_id == 0x009B) {
            s8 i = -1;
            if (
                (*(vu32*)((u32)(actor_id) + 0x08)) == 0x42700000
              && (*(vu32*)((u32)(actor_id) + 0x0C)) == 0xC2000000
              && (*(vu32*)((u32)(actor_id) + 0x10)) == 0xC4A28000
            ) i = 0;
            if (i >= 0) {
              tracker.grotto_ids[i]   = (*(vu16*)((u32)(actor_id) + 0x18));
              tracker.grotto_types[i] = (*(vu8 *)((u32)(actor_id) + 0x1C)) >> 4;
            }
          }
        }
        break;
    }
  }
  u8* prev_tracker = (u8*)((u32)(&old_tracker));
  u8* new_tracker = (u8*)((u32)(&tracker));
  u8 data[sizeof(struct tracker_t)*3+16];
  MAGIC_CRC(data);
  data[8] = USB_CMD_TRACKER;
  u16 idx = 11;
  for (u16 i = 0; i < sizeof(struct tracker_t); i++) {
    if (!usb_vars.tracker_initialized || prev_tracker[i] != new_tracker[i]) {
      data[idx++] = ((i) & 0xFF00) >> 8;
      data[idx++] = ((i) & 0x00FF);
      data[idx++] = new_tracker[i];
    }
    prev_tracker[i] = new_tracker[i];
  }
  if (idx > 11) {
    data[ 9] = ((idx-11) & 0xFF00) >> 8;
    data[10] = ((idx-11) & 0x00FF);
    while (idx < 16 || idx % 4) data[idx++] = 0;
    // if (idx % 2) data[idx++] = 0;
    usb_write(&data, idx);
  }
  usb_vars.tracker_initialized = true;

  // u8* prev_tracker = (u8*)((u32)(&old_tracker));
  // u8* new_tracker = (u8*)((u32)(&tracker));
  // u8 data[16] = MAGIC(data);
  // for (u16 i = 0; i < sizeof(struct tracker_t); i++) {
  //   if (!usb_vars.initialized || prev_tracker[i] != new_tracker[i]) {
  //     if (data[8] == USB_CMD_TRACKER) usb_write(&data, 16);
  //     data[ 8] = USB_CMD_TRACKER;
  //     data[ 9] = ((i) & 0xFF00) >> 8;
  //     data[10] = ((i) & 0x00FF);
  //     data[11] = new_tracker[i];
  //   }
  //   prev_tracker[i] = new_tracker[i];
  // }
  // if (data[8] == USB_CMD_TRACKER) {
  //   data[13] = 'M';
  //   data[14] = 'g';
  //   data[15] = 'c';
  //   usb_write(&data, 16);
  // }
}
