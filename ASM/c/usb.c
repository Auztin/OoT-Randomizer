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
u8 USB_READ_BUFFER[512];
u8 USB_WRITE_BUFFER[512];
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
  dma_read(USB_READ_BUFFER, D64_BASE_ADDRESS + D64_DEBUG_AREA, USB_SIZE);
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
  if (USB_SIZE > 8 && (USB_SIZE > 512 || usb_everdrive_read_data(USB_READ_BUFFER + 8, USB_SIZE - 8))) return 1;
  for (u8 i = 0; i < 8; i++) USB_READ_BUFFER[i] = header[i+8];
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

#define USB_POINTER_ADDRS_SIZE 8
#define USB_POINTER_MEMORY_SIZE 1024
#define USB_MONITOR_SIZE 256
#define USB_MEMORY_SIZE 512

struct {
  // u32 tracker_initialized : 1;
  // u32 vars_init : 1;
  // u32 received_item : 5;
  // u32 usb_timer : 5;
  // u32 lastPtr;
  // u32 tracker_timeout : 5;
  u32 range;
  u32 mem32;
  u32 mem16;
  u32 mem8;
  u32 outgoing_key;
  u32 inventory;

  u32 outgoing_item : 16;
  u32 outgoing_player : 16;

  u32 incoming_item : 16;
  u32 incoming_count : 16;

  u32 actor_offset : 16;
  u32 actor_id : 16;

  u32 rangeBytes : 16;
  u32 lastEntranceIndex : 16;

  u32 range_sid : 8;
  u32 mem32_sid : 8;
  u32 mem16_sid : 8;
  u32 mem8_sid : 8;

  u32 settings : 8;
  u32 actor_sent : 8;
  u32 actor_num : 8;
  u32 actor_sid : 8;

  u32 outgoing_timer : 5;
  u32 pause_writes : 3;
  u32 initialized : 1;
  u32 ready : 1;
  u32 defeatedGanon : 1;
  u32 sendSpawn : 1;
  u32 sendDone : 1;
  u32 sendRangeDone : 1;
  u32 packet_received : 1;
  u32 packet_sent : 1;
  u32 was_in_game : 1;

  u32 pointerAddrs[USB_POINTER_ADDRS_SIZE];
  u32 pointerValue[USB_POINTER_ADDRS_SIZE];
  u16 pointerBytes[USB_POINTER_ADDRS_SIZE];
  u16 pointerMemory[USB_POINTER_MEMORY_SIZE];
  u32 monitorAddrs[USB_MONITOR_SIZE];
  u8 monitorBytes[USB_MONITOR_SIZE];
  u16 memory[USB_MEMORY_SIZE];
} usb_vars = {0, };

enum USB_COMMANDS {
  USB_CMD_NONE,
  USB_CMD_RESET_VARS,
  USB_CMD_UNREADY,
  USB_CMD_READY,
  USB_CMD_OUTGOING,
  USB_CMD_INCOMING,
  USB_CMD_PLAYER,
  USB_CMD_INVENTORY,
  USB_CMD_SETTINGS,
  USB_CMD_PAUSE_WRITES,
  USB_CMD_PACKET_RECEIVED,
  USB_CMD_MEM,
  // USB_CMD_PTR,
  // USB_CMD_PTR_MEM,
  USB_CMD_DONE,
  USB_CMD_SPAWN,
  USB_CMD_GANON_DEFEATED,
  USB_CMD_MONITOR,
  USB_CMD_MONITOR_POINTER,
  USB_CMD_U8,
  USB_CMD_U16,
  USB_CMD_U32,
  USB_CMD_RANGE,
  USB_CMD_RANGE_DONE,
  USB_CMD_ACTOR,
};

enum SETTINGS {
  SETTING_ANTIALIAS = 0x01,
  SETTING_MW_SEND_OWN_ITEMS = 0x02,
  SETTING_MW_PROGRESSIVE_ITEMS = 0x04,
};

#define AA1 (*(vu32*)0x8000646C)
#define AA2 (*(vu32*)0x8000649C)
#define CRC1 (*(vu32*)0xB0000010)
#define CRC2 (*(vu32*)0xB0000014)
#define FILENAME1 (*(vu32*)(0x8011A5D0 + 0x24))
#define FILENAME2 (*(vu32*)(0x8011A5D0 + 0x28))
// #define MAGIC_CRC(array)  array[0] = (CRC1 & 0xFF000000) >> 24; \
//                           array[1] = (CRC1 & 0x00FF0000) >> 16; \
//                           array[2] = (CRC1 & 0x0000FF00) >> 8;  \
//                           array[3] = (CRC1 & 0x000000FF);       \
//                           array[4] = (CRC2 & 0xFF000000) >> 24; \
//                           array[5] = (CRC2 & 0x00FF0000) >> 16; \
//                           array[6] = (CRC2 & 0x0000FF00) >> 8;  \
//                           array[7] = (CRC2 & 0x000000FF);
// #define MAGIC(array)  {0, };MAGIC2(array);
#define MAGIC(array) array[0]='O';array[1]='o';array[2]='T';array[3]='R';
#define INTERNAL_COUNT (*(vu16*)(z64_file_addr + 0x90))
#define USB_VERSION 3;
extern u8 PLAYER_ID;
extern u16 INCOMING_PLAYER;
extern u16 INCOMING_ITEM;
extern u32 OUTGOING_KEY;
extern u16 OUTGOING_ITEM;
extern u16 OUTGOING_PLAYER;
extern u8 PLAYER_NAMES[];
extern u8 MW_SEND_OWN_ITEMS;
extern u8 MW_PROGRESSIVE_ITEMS_ENABLE;
extern u8 MW_PROGRESSIVE_ITEMS_STATE[];
extern u8 VERSION_STRING_TXT[];
extern u8 WORLD_STRING_TXT[];
extern u8 CFG_FILE_SELECT_HASH[];

void usb_initialize() {
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
  AA1 = 0x00003216;
  AA2 = 0x00003216;
}

void usb_reset_vars() {
  usb_vars.mem8 = 0;
  usb_vars.mem16 = 0;
  usb_vars.mem32 = 0;
  usb_vars.range = 0;
  usb_vars.mem8_sid = 0;
  usb_vars.mem16_sid = 0;
  usb_vars.mem32_sid = 0;
  usb_vars.range_sid = 0;
  usb_vars.actor_sid = 0;
  usb_vars.rangeBytes = 0;
  usb_vars.actor_id = 0;
  usb_vars.actor_num = 0;
  usb_vars.actor_offset = 0;
  usb_vars.defeatedGanon = false;
  usb_vars.sendSpawn = false;
  usb_vars.sendDone = false;
  usb_vars.sendRangeDone = false;
  usb_vars.packet_sent = false;
  for (int i = 0; i < USB_POINTER_ADDRS_SIZE; i++) {
    usb_vars.pointerAddrs[i] = 0;
    usb_vars.pointerValue[i] = 0;
    usb_vars.pointerBytes[i] = 0;
  }
  for (int i = 0; i < USB_POINTER_MEMORY_SIZE; i++) usb_vars.pointerMemory[i] = 0x100;
  for (int i = 0; i < USB_MONITOR_SIZE; i++) {
    usb_vars.monitorAddrs[i] = 0;
    usb_vars.monitorBytes[i] = 0;
  }
  for (int i = 0; i < USB_MEMORY_SIZE; i++) usb_vars.memory[i] = 0x100;
}

bool usb_process_in_out(bool in_game, bool transitioning, u32 inventory) {
  bool loop = true;
  if (usb_poll() & USB_CAN_READ) {
    if (
      !usb_read() && USB_SIZE >= 16 &&
      in_game && usb_vars.ready &&
      USB_READ_BUFFER[0] == 'O' &&
      USB_READ_BUFFER[1] == 'o' &&
      USB_READ_BUFFER[2] == 'T' &&
      USB_READ_BUFFER[3] == 'R'
    ) {
      bool packet_received = true;
      switch (USB_READ_BUFFER[4]) {
        case USB_CMD_PACKET_RECEIVED: {
          packet_received = false;
          usb_vars.packet_sent = false;
          return true;
        }
        case USB_CMD_RESET_VARS: {
          usb_reset_vars();
          break;
        }
        case USB_CMD_UNREADY: {
          usb_reset_vars();
          usb_vars.ready = false;
          break;
        }
        case USB_CMD_OUTGOING: {
          usb_vars.outgoing_key = 0;
          usb_vars.outgoing_item = 0;
          usb_vars.outgoing_player = 0;
          break;
        }
        case USB_CMD_INCOMING: {
          if (!usb_vars.incoming_item) {
            usb_vars.incoming_item   = (USB_READ_BUFFER[5] << 8) | USB_READ_BUFFER[6];
            usb_vars.incoming_count  = (USB_READ_BUFFER[7] << 8) | USB_READ_BUFFER[8];
          }
          break;
        }
        case USB_CMD_PLAYER: {
          if (USB_SIZE >= 14) {
            u16 id = 8*USB_READ_BUFFER[5];
            for (u8 i = 0; i < 8; i++) PLAYER_NAMES[id+i] = USB_READ_BUFFER[i+6];
          }
          break;
        }
        case USB_CMD_INVENTORY: {
          if (USB_SIZE >= 10) {
            u16 id = 4*USB_READ_BUFFER[5];
            for (u8 i = 0; i < 4; i++) MW_PROGRESSIVE_ITEMS_STATE[id+i] = USB_READ_BUFFER[i+6];
          }
          break;
        }
        case USB_CMD_SETTINGS: {
          usb_vars.settings = USB_READ_BUFFER[5];
          MW_SEND_OWN_ITEMS = (usb_vars.settings & SETTING_MW_SEND_OWN_ITEMS) ? 1 : 0;
          MW_PROGRESSIVE_ITEMS_ENABLE = (usb_vars.settings & SETTING_MW_PROGRESSIVE_ITEMS) ? 1 : 0;
          break;
        }
        case USB_CMD_PAUSE_WRITES: {
          usb_vars.pause_writes = USB_READ_BUFFER[5] ? 1 : 3;
          break;
        }
        case USB_CMD_MONITOR: {
          u32 newAddr  = USB_READ_BUFFER[5] << 24;
              newAddr |= USB_READ_BUFFER[6] << 16;
              newAddr |= USB_READ_BUFFER[7] << 8;
              newAddr |= USB_READ_BUFFER[8];
          if (newAddr < 0x80000000 || newAddr > 0x80800000) break;
          u8 newBytes = USB_READ_BUFFER[9];
          u32 n = 0;
          for (; n < USB_MONITOR_SIZE; n++) {
            u32 addr = usb_vars.monitorAddrs[n];
            if (addr == 0) break;
            u8 bytes = usb_vars.monitorBytes[n];
            if (addr == newAddr) {
              if (bytes < newBytes) {
                usb_vars.monitorBytes[n] = newBytes;
              }
              newAddr = 0;
              break;
            }
          }
          if (newAddr != 0) {
            usb_vars.monitorAddrs[n] = newAddr;
            usb_vars.monitorBytes[n] = newBytes;
          }
          break;
        }
        case USB_CMD_MONITOR_POINTER: {
          u32 newAddr  = USB_READ_BUFFER[5] << 24;
              newAddr |= USB_READ_BUFFER[6] << 16;
              newAddr |= USB_READ_BUFFER[7] << 8;
              newAddr |= USB_READ_BUFFER[8];
          if (newAddr < 0x80000000 || newAddr > 0x80800000) break;
          u16 newBytes = (USB_READ_BUFFER[9] << 8) | USB_READ_BUFFER[10];
          u32 n = 0;
          for (; n < USB_POINTER_ADDRS_SIZE; n++) {
            u32 addr = usb_vars.pointerAddrs[n];
            if (addr == 0) break;
            u16 bytes = usb_vars.pointerBytes[n];
            if (addr == newAddr) {
              if (bytes < newBytes) {
                usb_vars.pointerBytes[n] = newBytes;
              }
              newAddr = 0;
              break;
            }
          }
          if (newAddr != 0) {
            usb_vars.pointerAddrs[n] = newAddr;
            usb_vars.pointerBytes[n] = newBytes;
          }
          break;
        }
        case USB_CMD_U8: {
          usb_vars.mem8_sid = USB_READ_BUFFER[5];
          usb_vars.mem8  = USB_READ_BUFFER[6] << 24;
          usb_vars.mem8 |= USB_READ_BUFFER[7] << 16;
          usb_vars.mem8 |= USB_READ_BUFFER[8] << 8;
          usb_vars.mem8 |= USB_READ_BUFFER[9];
          if (usb_vars.mem8 < 0x80000000 || usb_vars.mem8 > 0x80800000) usb_vars.mem8 = 0;
          break;
        }
        case USB_CMD_U16: {
          usb_vars.mem16_sid = USB_READ_BUFFER[5];
          usb_vars.mem16  = USB_READ_BUFFER[6] << 24;
          usb_vars.mem16 |= USB_READ_BUFFER[7] << 16;
          usb_vars.mem16 |= USB_READ_BUFFER[8] << 8;
          usb_vars.mem16 |= USB_READ_BUFFER[9];
          if (usb_vars.mem16 < 0x80000000 || usb_vars.mem16 > 0x80800000) usb_vars.mem16 = 0;
          break;
        }
        case USB_CMD_U32: {
          usb_vars.mem32_sid = USB_READ_BUFFER[5];
          usb_vars.mem32  = USB_READ_BUFFER[6] << 24;
          usb_vars.mem32 |= USB_READ_BUFFER[7] << 16;
          usb_vars.mem32 |= USB_READ_BUFFER[8] << 8;
          usb_vars.mem32 |= USB_READ_BUFFER[9];
          if (usb_vars.mem32 < 0x80000000 || usb_vars.mem32 > 0x80800000) usb_vars.mem32 = 0;
          break;
        }
        case USB_CMD_RANGE: {
          usb_vars.range_sid = USB_READ_BUFFER[5];
          usb_vars.range  = USB_READ_BUFFER[6] << 24;
          usb_vars.range |= USB_READ_BUFFER[7] << 16;
          usb_vars.range |= USB_READ_BUFFER[8] << 8;
          usb_vars.range |= USB_READ_BUFFER[9];
          if (usb_vars.range < 0x80000000 || usb_vars.range > 0x80800000) usb_vars.range = 0;
          else usb_vars.rangeBytes = (USB_READ_BUFFER[10] << 8) | USB_READ_BUFFER[11];
          break;
        }
        case USB_CMD_ACTOR: {
          usb_vars.actor_sid = USB_READ_BUFFER[5];
          usb_vars.actor_id  = USB_READ_BUFFER[6] << 8;
          usb_vars.actor_id |= USB_READ_BUFFER[7];
          usb_vars.actor_num = USB_READ_BUFFER[8];
          usb_vars.actor_offset  = USB_READ_BUFFER[9] << 8;
          usb_vars.actor_offset |= USB_READ_BUFFER[10];
          break;
        }
      }
      if (packet_received) usb_vars.packet_received = true;
    }
    else loop = false;
  }
  else if (usb_poll() & USB_CAN_WRITE) {
    // u8 data[512];
    // usb_write(&data, 512);
    if (usb_vars.pause_writes) {
      switch (usb_vars.pause_writes) {
        case 1: {
          MAGIC(USB_WRITE_BUFFER);
          USB_WRITE_BUFFER[4] = USB_CMD_PAUSE_WRITES;
          USB_WRITE_BUFFER[5] = 1;
          usb_write(&USB_WRITE_BUFFER, 16);
          usb_vars.pause_writes = 2;
          break;
        }
        case 3: {
          MAGIC(USB_WRITE_BUFFER);
          USB_WRITE_BUFFER[4] = USB_CMD_PAUSE_WRITES;
          USB_WRITE_BUFFER[5] = 0;
          usb_write(&USB_WRITE_BUFFER, 16);
          usb_vars.pause_writes = 0;
          break;
        }
        default: loop = false;
      }
    }
    else if (!usb_vars.ready) {
      if (!usb_vars.initialized) {
        usb_reset_vars();
        usb_vars.initialized = true;
        MAGIC(USB_WRITE_BUFFER);
        USB_WRITE_BUFFER[4] = USB_CMD_UNREADY;
        usb_write(&USB_WRITE_BUFFER, 16);
      }
      else if (in_game) {
        usb_vars.ready = true;
        MAGIC(USB_WRITE_BUFFER);
        USB_WRITE_BUFFER[ 4] = USB_CMD_READY;
        USB_WRITE_BUFFER[ 5] = USB_VERSION;
        USB_WRITE_BUFFER[ 6] = PLAYER_ID;
        USB_WRITE_BUFFER[ 7] = (INTERNAL_COUNT & 0xFF00) >> 8;
        USB_WRITE_BUFFER[ 8] = (INTERNAL_COUNT & 0x00FF);
        USB_WRITE_BUFFER[ 9] = (CRC1 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[10] = (CRC1 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[11] = (CRC1 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[12] = (CRC1 & 0x000000FF);
        USB_WRITE_BUFFER[13] = (CRC2 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[14] = (CRC2 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[15] = (CRC2 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[16] = (CRC2 & 0x000000FF);
        USB_WRITE_BUFFER[17] = (FILENAME1 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[18] = (FILENAME1 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[19] = (FILENAME1 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[20] = (FILENAME1 & 0x000000FF);
        USB_WRITE_BUFFER[21] = (FILENAME2 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[22] = (FILENAME2 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[23] = (FILENAME2 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[24] = (FILENAME2 & 0x000000FF);
        for (int i = 0; i < 0x24; i++) USB_WRITE_BUFFER[25+i] = VERSION_STRING_TXT[i];
        for (int i = 0; i < 0x10; i++) USB_WRITE_BUFFER[61+i] = WORLD_STRING_TXT[i];
        for (int i = 0; i < 0x05; i++) USB_WRITE_BUFFER[77+i] = CFG_FILE_SELECT_HASH[i];
        USB_WRITE_BUFFER[82] = (usb_vars.inventory & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[83] = (usb_vars.inventory & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[84] = (usb_vars.inventory & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[85] = (usb_vars.inventory & 0x000000FF);
        usb_write(&USB_WRITE_BUFFER, 88);
        if (!usb_vars.was_in_game) usb_vars.sendSpawn = true;
      }
      else loop = false;
    }
    else if (!in_game) {
      usb_reset_vars();
      usb_vars.ready = false;
      usb_vars.outgoing_key = 0;
      usb_vars.outgoing_item = 0;
      usb_vars.outgoing_player = 0;
      usb_vars.incoming_item = 0;
      usb_vars.incoming_count = 0;
      MAGIC(USB_WRITE_BUFFER);
      USB_WRITE_BUFFER[4] = USB_CMD_UNREADY;
      usb_write(&USB_WRITE_BUFFER, 16);
    }
    else if (usb_vars.outgoing_key && usb_vars.outgoing_timer == 0) {
      MAGIC(USB_WRITE_BUFFER);
      USB_WRITE_BUFFER[ 4] = USB_CMD_OUTGOING;
      USB_WRITE_BUFFER[ 5] = (usb_vars.outgoing_key & 0xFF000000) >> 24;
      USB_WRITE_BUFFER[ 6] = (usb_vars.outgoing_key & 0x00FF0000) >> 16;
      USB_WRITE_BUFFER[ 7] = (usb_vars.outgoing_key & 0x0000FF00) >> 8;
      USB_WRITE_BUFFER[ 8] = (usb_vars.outgoing_key & 0x000000FF);
      USB_WRITE_BUFFER[ 9] = (usb_vars.outgoing_item & 0xFF00) >> 8;
      USB_WRITE_BUFFER[10] = (usb_vars.outgoing_item & 0x00FF);
      USB_WRITE_BUFFER[11] = (usb_vars.outgoing_player & 0xFF00) >> 8;
      USB_WRITE_BUFFER[12] = (usb_vars.outgoing_player & 0x00FF);
      usb_write(&USB_WRITE_BUFFER, 16);
      usb_vars.outgoing_timer++;
    }
    else if (!transitioning) {
      MAGIC(USB_WRITE_BUFFER);
      if (usb_vars.inventory != inventory) {
        usb_vars.inventory = inventory;
        USB_WRITE_BUFFER[ 4] = USB_CMD_INVENTORY;
        USB_WRITE_BUFFER[ 5] = (inventory & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[ 6] = (inventory & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[ 7] = (inventory & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[ 8] = (inventory & 0x000000FF);
        usb_write(&USB_WRITE_BUFFER, 16);
      }
      else if (usb_vars.mem8) {
        USB_WRITE_BUFFER[ 4] = USB_CMD_U8;
        USB_WRITE_BUFFER[ 5] = usb_vars.mem8_sid;
        USB_WRITE_BUFFER[ 6] = (usb_vars.mem8 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[ 7] = (usb_vars.mem8 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[ 8] = (usb_vars.mem8 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[ 9] = (usb_vars.mem8 & 0x000000FF);
        USB_WRITE_BUFFER[10] = (*(vu8*)(usb_vars.mem8));
        usb_write(&USB_WRITE_BUFFER, 16);
        usb_vars.mem8 = 0;
      }
      else if (usb_vars.mem16) {
        USB_WRITE_BUFFER[ 4] = USB_CMD_U16;
        USB_WRITE_BUFFER[ 5] = usb_vars.mem16_sid;
        USB_WRITE_BUFFER[ 6] = (usb_vars.mem16 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[ 7] = (usb_vars.mem16 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[ 8] = (usb_vars.mem16 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[ 9] = (usb_vars.mem16 & 0x000000FF);
        USB_WRITE_BUFFER[10] = (*(vu8*)(usb_vars.mem16));
        USB_WRITE_BUFFER[11] = (*(vu8*)(usb_vars.mem16+1));
        usb_write(&USB_WRITE_BUFFER, 16);
        usb_vars.mem16 = 0;
      }
      else if (usb_vars.mem32) {
        USB_WRITE_BUFFER[ 4] = USB_CMD_U32;
        USB_WRITE_BUFFER[ 5] = usb_vars.mem32_sid;
        USB_WRITE_BUFFER[ 6] = (usb_vars.mem32 & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[ 7] = (usb_vars.mem32 & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[ 8] = (usb_vars.mem32 & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[ 9] = (usb_vars.mem32 & 0x000000FF);
        USB_WRITE_BUFFER[10] = (*(vu8*)(usb_vars.mem32));
        USB_WRITE_BUFFER[11] = (*(vu8*)(usb_vars.mem32+1));
        USB_WRITE_BUFFER[12] = (*(vu8*)(usb_vars.mem32+2));
        USB_WRITE_BUFFER[13] = (*(vu8*)(usb_vars.mem32+3));
        usb_write(&USB_WRITE_BUFFER, 16);
        usb_vars.mem32 = 0;
      }
      else if (usb_vars.actor_num) {
        USB_WRITE_BUFFER[4] = USB_CMD_ACTOR;
        USB_WRITE_BUFFER[5] = usb_vars.actor_sid;
        USB_WRITE_BUFFER[6] = 0;
        u16 bufferOffset = 7;
        u8 n = 0xFF;
        for (u8 i = 0; i < 12 && usb_vars.actor_num; i++) {
          for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA0D4 + 0x08 * i)); actor_id && usb_vars.actor_num; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
            if (*actor_id == usb_vars.actor_id) {
              n++;
              if (n < usb_vars.actor_sent) continue;
              if (bufferOffset < 500-40) {
                usb_vars.actor_sent++;
                USB_WRITE_BUFFER[6]++;
                usb_vars.actor_num--;
                u32 addr = (u32)(actor_id);
                USB_WRITE_BUFFER[bufferOffset+0] = (addr & 0xFF000000) >> 24;
                USB_WRITE_BUFFER[bufferOffset+1] = (addr & 0x00FF0000) >> 16;
                USB_WRITE_BUFFER[bufferOffset+2] = (addr & 0x0000FF00) >> 8;
                USB_WRITE_BUFFER[bufferOffset+3] = (addr & 0x000000FF);
                bufferOffset += 4;
                for (u8 n = 0; n < 32; n++) {
                  USB_WRITE_BUFFER[bufferOffset] = (*(vu8*)(addr+n));
                  bufferOffset++;
                }
                for (u8 n = 0; n < 4; n++) {
                  USB_WRITE_BUFFER[bufferOffset] = usb_vars.actor_offset ? (*(vu8*)(addr+usb_vars.actor_offset+n)) : 0;
                  bufferOffset++;
                }
              }
            }
          }
        }
        if (USB_WRITE_BUFFER[6]) {
          while (bufferOffset < 16 || bufferOffset % 4) bufferOffset++;
          usb_write(&USB_WRITE_BUFFER, bufferOffset);
          if (usb_vars.actor_num == 0) {
            usb_vars.actor_id = 0;
            usb_vars.actor_offset = 0;
            usb_vars.actor_sent = 0;
          }
        }
        else {
          usb_vars.actor_id = 0;
          usb_vars.actor_offset = 0;
          usb_vars.actor_sent = 0;
          usb_vars.actor_num = 0;
          loop = false;
        }
      }
      else if (usb_vars.range) {
        USB_WRITE_BUFFER[ 4] = USB_CMD_RANGE;
        USB_WRITE_BUFFER[ 5] = usb_vars.range_sid;
        USB_WRITE_BUFFER[ 6] = (usb_vars.range & 0xFF000000) >> 24;
        USB_WRITE_BUFFER[ 7] = (usb_vars.range & 0x00FF0000) >> 16;
        USB_WRITE_BUFFER[ 8] = (usb_vars.range & 0x0000FF00) >> 8;
        USB_WRITE_BUFFER[ 9] = (usb_vars.range & 0x000000FF);
        USB_WRITE_BUFFER[10] = 0;
        USB_WRITE_BUFFER[11] = 0;
        u16 bufferOffset = 12;
        while (usb_vars.rangeBytes && bufferOffset < 500-5) {
          USB_WRITE_BUFFER[bufferOffset] = (*(vu8*)(usb_vars.range));
          bufferOffset++;
          usb_vars.range++;
          usb_vars.rangeBytes--;
        }
        if (usb_vars.rangeBytes == 0) usb_vars.range = 0;
        if (bufferOffset > 12) {
          USB_WRITE_BUFFER[10] = ((bufferOffset-12) & 0x0000FF00) >> 8;
          USB_WRITE_BUFFER[11] = ((bufferOffset-12) & 0x000000FF);
          while (bufferOffset < 16 || bufferOffset % 4) bufferOffset++;
          usb_write(&USB_WRITE_BUFFER, bufferOffset);
          usb_vars.sendRangeDone = true;
        }
        else loop = false;
      }
      else if (usb_vars.sendRangeDone) {
        USB_WRITE_BUFFER[4] = USB_CMD_RANGE_DONE;
        usb_write(&USB_WRITE_BUFFER, 16);
        usb_vars.sendRangeDone = false;
      }
      else if (usb_vars.packet_received) {
        MAGIC(USB_WRITE_BUFFER);
        USB_WRITE_BUFFER[4] = USB_CMD_PACKET_RECEIVED;
        usb_write(&USB_WRITE_BUFFER, 16);
        usb_vars.packet_received = false;
        return true;
      }
      else if (!usb_vars.packet_sent) {
        USB_WRITE_BUFFER[4] = USB_CMD_MEM;
        USB_WRITE_BUFFER[5] = 0;
        u16 bufferOffset = 6;
        u32 i = 0;
        for (u32 n = 0; n < USB_MONITOR_SIZE && bufferOffset < 500-5; n++) {
          u32 addr = usb_vars.monitorAddrs[n];
          if (addr < 0x80000000 || addr > 0x80800000) break;
          s16 bytes = usb_vars.monitorBytes[n];
          while (bytes >= 0 && bufferOffset < 500-5) {
            u8 value = (*(vu8*)(addr));
            if (usb_vars.memory[i] != value) {
              usb_vars.memory[i] = value;
              USB_WRITE_BUFFER[bufferOffset+0] = (addr & 0xFF000000) >> 24;
              USB_WRITE_BUFFER[bufferOffset+1] = (addr & 0x00FF0000) >> 16;
              USB_WRITE_BUFFER[bufferOffset+2] = (addr & 0x0000FF00) >> 8;
              USB_WRITE_BUFFER[bufferOffset+3] = (addr & 0x000000FF);
              USB_WRITE_BUFFER[bufferOffset+4] = value;
              bufferOffset += 5;
              USB_WRITE_BUFFER[5]++;
            }
            addr++;
            bytes--;
            i++;
          }
        }
        i = 0;
        for (u32 n = 0; n < USB_POINTER_ADDRS_SIZE && bufferOffset < 500-20; n++) {
          u32 ptr = usb_vars.pointerAddrs[n];
          if (ptr < 0x80000000 || ptr > 0x80800000) break;
          u32 addr = (*(vu32*)(ptr));
          if (addr < 0x80000000 || addr > 0x80800000) addr = 0;
          u8 pointerChanged = 0;
          if (usb_vars.pointerValue[n] != addr) {
            pointerChanged |= 1;
            usb_vars.pointerValue[n] = addr;
          }
          s32 bytes = usb_vars.pointerBytes[n];
          while (bytes >= 0) {
            if (pointerChanged & 1) usb_vars.pointerMemory[i] = 0x100;
            if (addr != 0 && bufferOffset < 500-25) {
              u8 value = (*(vu8*)(addr));
              if (usb_vars.pointerMemory[i] != value) {
                usb_vars.pointerMemory[i] = value;
                USB_WRITE_BUFFER[bufferOffset+0] = (addr & 0xFF000000) >> 24;
                USB_WRITE_BUFFER[bufferOffset+1] = (addr & 0x00FF0000) >> 16;
                USB_WRITE_BUFFER[bufferOffset+2] = (addr & 0x0000FF00) >> 8;
                USB_WRITE_BUFFER[bufferOffset+3] = (addr & 0x000000FF);
                USB_WRITE_BUFFER[bufferOffset+4] = value;
                bufferOffset += 5;
                USB_WRITE_BUFFER[5]++;
                pointerChanged |= 2;
              }
              addr++;
            }
            bytes--;
            i++;
          }
          if (pointerChanged) {
            for (u8 i = 0; i < 4; i++) {
              u32 ptrAddr = ptr+i;
              USB_WRITE_BUFFER[bufferOffset+0] = (ptrAddr & 0xFF000000) >> 24;
              USB_WRITE_BUFFER[bufferOffset+1] = (ptrAddr & 0x00FF0000) >> 16;
              USB_WRITE_BUFFER[bufferOffset+2] = (ptrAddr & 0x0000FF00) >> 8;
              USB_WRITE_BUFFER[bufferOffset+3] = (ptrAddr & 0x000000FF);
              USB_WRITE_BUFFER[bufferOffset+4] = (*(vu8*)(ptrAddr));
              bufferOffset += 5;
              USB_WRITE_BUFFER[5]++;
            }
          }
        }
        if (bufferOffset > 6) {
          while (bufferOffset < 16 || bufferOffset % 4) bufferOffset++;
          usb_write(&USB_WRITE_BUFFER, bufferOffset);
          usb_vars.sendDone = true;
        }
        else if (usb_vars.sendSpawn && usb_vars.sendDone) {
          MAGIC(USB_WRITE_BUFFER);
          USB_WRITE_BUFFER[4] = USB_CMD_SPAWN;
          usb_write(&USB_WRITE_BUFFER, 16);
          usb_vars.sendSpawn = false;
        }
        else if (usb_vars.sendDone) {
          USB_WRITE_BUFFER[4] = USB_CMD_DONE;
          usb_write(&USB_WRITE_BUFFER, 16);
          usb_vars.sendDone = false;
        }
        else {
          loop = false;
          if (z64_game.scene_index == 0x004F && !usb_vars.defeatedGanon) {
            for (vu16* actor_id = (vu16*)(*(vu32*)(0x801CA11C)); actor_id; actor_id = (vu16*)(*(vu32*)((u32)(actor_id) + 0x124))) {
              if (*actor_id == 0x017A) {
                if ((*(vs8*)((u32)(actor_id) + 0xAF)) <= 0) {
                  USB_WRITE_BUFFER[4] = USB_CMD_GANON_DEFEATED;
                  usb_write(&USB_WRITE_BUFFER, 16);
                  usb_vars.defeatedGanon = true;
                  loop = true;
                }
                break;
              }
            }
          }
        }
      }
      else loop = false;
    }
    else loop = false;
    if (loop == false && usb_vars.incoming_item && !INCOMING_ITEM) {
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
          if (usb_vars.incoming_count < INTERNAL_COUNT || usb_vars.incoming_item == 0xFFFF) INTERNAL_COUNT = usb_vars.incoming_count;
          if (usb_vars.incoming_item != 0xFFFF) {
            INCOMING_PLAYER = PLAYER_ID;
            INCOMING_ITEM   = usb_vars.incoming_item;
          }
          usb_vars.incoming_item = 0;
          usb_vars.incoming_count = 0;
          MAGIC(USB_WRITE_BUFFER);
          USB_WRITE_BUFFER[4] = USB_CMD_INCOMING;
          USB_WRITE_BUFFER[5] = ((INTERNAL_COUNT) & 0xFF00) >> 8;
          USB_WRITE_BUFFER[6] = ((INTERNAL_COUNT) & 0x00FF);
          usb_write(&USB_WRITE_BUFFER, 16);
          loop = true;
      }
    }
  }
  else loop = false;
  if (loop && !usb_vars.packet_received) usb_vars.packet_sent = true;
  return loop;
}

void usb_process() {
  if (usb_cart == CART_NONE) return;
  if (usb_vars.settings & SETTING_ANTIALIAS) {
    AA1 = 0x00013016;
    AA2 = 0x00013016;
  }
  else {
    AA1 = 0x00003216;
    AA2 = 0x00003216;
  }
  bool in_game = (z64_file.game_mode != 1 && z64_file.game_mode != 2);
  u16 nextEntranceIndex = (*(vu16*)(0x801DA2BA));
  u32 stateFrames = (*(vu32*)(0x801C853C));
  if (stateFrames < 20 && nextEntranceIndex != usb_vars.lastEntranceIndex) usb_vars.lastEntranceIndex = nextEntranceIndex;
  u32 inventory = 0;
  if (in_game) {
    if (usb_vars.outgoing_key) {
      if (usb_vars.outgoing_timer++ > 20) usb_vars.outgoing_timer = 0;
    }
    else if (OUTGOING_KEY) {
      usb_vars.outgoing_key = OUTGOING_KEY;
      usb_vars.outgoing_item = OUTGOING_ITEM;
      usb_vars.outgoing_player = OUTGOING_PLAYER;
      OUTGOING_KEY = 0;
      OUTGOING_ITEM = 0;
      OUTGOING_PLAYER = 0;
      usb_vars.outgoing_timer = 0;
    }
    u8 val = (*(vu8*)(0x8011A64D));
    inventory |= (val == 0x0A ? 1 : (val == 0x0B ? 2 : 0)) <<  0; // hookshot
    val = (*(vu8*)(0x8011A673));
    inventory |=                       ((val & 0xC0) >> 6) <<  2; // strength
    inventory |=                       ((val & 0x18) >> 3) <<  4; // bomb bag
    inventory |=                       ((val & 0x03) >> 0) <<  6; // bow
    val = (*(vu8*)(0x8011A672));
    inventory |=                       ((val & 0xC0) >> 6) <<  8; // slingshot
    inventory |=                       ((val & 0x30) >> 4) << 10; // wallet
    inventory |=                       ((val & 0x06) >> 1) << 12; // scale
    val = (*(vu8*)(0x8011A671));
    inventory |=                       ((val & 0x30) >> 4) << 14; // nuts
    val = (val & 0x0E) >> 1;
    if (val == 5) val = 2;
    inventory |=                                       val << 16; // sticks
    inventory |=                     (*(vu8*)(0x8011A602)) << 18; // magic
    val = (*(vu8*)(0x8011A64B));
    inventory |= (val == 0x07 ? 1 : (val == 0x08 ? 2 : 0)) << 20; // ocarina
    val = (*(vu8*)(0x8011A64C));
    inventory |=                   ((val == 0x09) ? 1 : 0) << 22; // bombchu bag
  }
  u32 timeout = 0;
  bool transitioning = stateFrames < 20 || nextEntranceIndex != usb_vars.lastEntranceIndex;
  while (true) {
    if (usb_process_in_out(in_game, transitioning, inventory)) timeout = 0;
    else if (!usb_vars.packet_sent || timeout++ >= 8192) {
      usb_vars.packet_sent = false;
      break;
    }
  }
  usb_vars.was_in_game = in_game;
}
