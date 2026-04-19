/****************************************************************************
 * boards/spike-prime-hub/src/stm32_w25q256.c
 *
 * Winbond W25Q256 32 MB SPI NOR flash driver for SPIKE Prime Hub.
 *
 * Hardware:
 *   SPI2: SCK=PB13, MISO=PC2, MOSI=PC3 (AF5)
 *   /CS:  PB12 (software NSS, idle HIGH)
 *   DMA:  DMA1 Stream3 Ch0 (RX), DMA1 Stream4 Ch0 (TX)
 *
 * The W25Q256 is 32 MB and requires 4-byte addressing.  This driver always
 * uses dedicated 4-byte commands (Fast Read 0x0C, Page Program 0x12,
 * Sector Erase 0x21) so it does not depend on the device's address-mode
 * register state across resets (matches pybricks driver behavior).
 *
 * Partition layout:
 *   0x000000-0x0FFFFF  (1 MB)   reserved (LEGO bootloader area)
 *   0x100000-0x1FFFFFF (31 MB)  exposed as /dev/mtdblock0 → LittleFS
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_SPI2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI configuration */

#define W25Q256_SPI_FREQUENCY  24000000   /* 24 MHz (APB1 48 MHz / 2) */
#define W25Q256_SPI_MODE       SPIDEV_MODE0
#define W25Q256_SPI_NBITS      8

/* W25Q256 instructions (4-byte addressing variants) */

#define W25Q256_CMD_WREN       0x06   /* Write Enable                       */
#define W25Q256_CMD_RDSR1      0x05   /* Read Status Register-1             */
#define W25Q256_CMD_RDSR2      0x35   /* Read Status Register-2             */
#define W25Q256_CMD_JEDEC_ID   0x9f   /* Read JEDEC ID                      */
#define W25Q256_CMD_FAST_READ4 0x0c   /* Fast Read with 4-byte address      */
#define W25Q256_CMD_PP4        0x12   /* Page Program with 4-byte address   */
#define W25Q256_CMD_SE4        0x21   /* Sector Erase 4 KB w/ 4-byte addr   */
#define W25Q256_CMD_BE4        0xdc   /* Block Erase 64 KB w/ 4-byte addr   */
#define W25Q256_CMD_CE         0xc7   /* Chip Erase                         */
#define W25Q256_CMD_RDP        0xab   /* Release Power Down                 */

/* Status register bits */

#define W25Q256_SR_BUSY        (1 << 0)
#define W25Q256_SR_WEL         (1 << 1)

/* JEDEC identification */

#define W25Q256_JEDEC_ID       0x00ef4019  /* MFR=Winbond, Type=0x40, Cap=0x19 */
#define W25Q256_JEDEC_MFR      0xef
#define W25Q256_JEDEC_TYPE     0x40
#define W25Q256_JEDEC_CAP      0x19

/* Geometry (chip-wide) */

#define W25Q256_PAGE_SIZE      256u
#define W25Q256_PAGE_SHIFT     8
#define W25Q256_SECTOR_SIZE    4096u                /* 4 KB */
#define W25Q256_SECTOR_SHIFT   12
#define W25Q256_BLOCK_SIZE     65536u               /* 64 KB */
#define W25Q256_BLOCK_SHIFT    16
#define W25Q256_CHIP_SIZE      (32u * 1024u * 1024u) /* 32 MB */
#define W25Q256_NSECTORS       (W25Q256_CHIP_SIZE / W25Q256_SECTOR_SIZE) /* 8192 */

#define W25Q256_ERASED_STATE   0xff

/* Mount layout */

#define W25Q256_RESERVED_BYTES 0x00100000u  /* 1 MB reserved (LEGO area) */
#define W25Q256_FS_START_SECTOR (W25Q256_RESERVED_BYTES / W25Q256_SECTOR_SIZE)
#define W25Q256_FS_NSECTORS     (W25Q256_NSECTORS - W25Q256_FS_START_SECTOR)

#define W25Q256_MTDBLOCK_PATH  "/dev/mtdblock0"
#define W25Q256_MOUNT_POINT    "/mnt/flash"
#define W25Q256_FS_TYPE        "littlefs"

/* Polling timeouts (microseconds).  Sector erase ~45 ms typ, 400 ms max.
 * Chip erase ~80 s max.  Use generous bounds.
 */

#define W25Q256_BUSY_POLL_USEC      50   /* per-iteration sleep        */
#define W25Q256_PROGRAM_TIMEOUT_US  100000   /* page program 100 ms    */
#define W25Q256_ERASE_TIMEOUT_US    1000000  /* sector erase 1 s       */
#define W25Q256_BULK_TIMEOUT_US     120000000 /* chip erase 120 s      */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct w25q256_dev_s
{
  struct mtd_dev_s      mtd;     /* Must be first */
  FAR struct spi_dev_s *spi;     /* SPI bus instance */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI helpers */

static void    w25q256_lock(FAR struct w25q256_dev_s *priv);
static void    w25q256_unlock(FAR struct w25q256_dev_s *priv);
static void    w25q256_send_addr4(FAR struct w25q256_dev_s *priv,
                                  uint32_t address);
static void    w25q256_write_enable(FAR struct w25q256_dev_s *priv);
static int     w25q256_wait_busy(FAR struct w25q256_dev_s *priv,
                                 uint32_t timeout_us);
static int     w25q256_read_jedec(FAR struct w25q256_dev_s *priv,
                                  FAR uint32_t *jedec);

/* Flash operations */

static int     w25q256_erase_sector(FAR struct w25q256_dev_s *priv,
                                    uint32_t address);
static int     w25q256_chip_erase(FAR struct w25q256_dev_s *priv);
static int     w25q256_program_page(FAR struct w25q256_dev_s *priv,
                                    uint32_t address,
                                    FAR const uint8_t *buffer,
                                    size_t nbytes);
static void    w25q256_read_data(FAR struct w25q256_dev_s *priv,
                                 uint32_t address,
                                 FAR uint8_t *buffer,
                                 size_t nbytes);

/* MTD interface */

static int     w25q256_mtd_erase(FAR struct mtd_dev_s *dev,
                                 off_t startblock, size_t nblocks);
static ssize_t w25q256_mtd_bread(FAR struct mtd_dev_s *dev,
                                 off_t startblock, size_t nblocks,
                                 FAR uint8_t *buffer);
static ssize_t w25q256_mtd_bwrite(FAR struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks,
                                  FAR const uint8_t *buffer);
static ssize_t w25q256_mtd_read(FAR struct mtd_dev_s *dev,
                                off_t offset, size_t nbytes,
                                FAR uint8_t *buffer);
static int     w25q256_mtd_ioctl(FAR struct mtd_dev_s *dev,
                                 int cmd, unsigned long arg);

/* Public board init helper (forward decl, declared in spike_prime_hub.h) */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25q256_lock
 *
 * Take exclusive access to the SPI bus and configure it for the W25Q256.
 * Required when the bus is shared (defensive — currently SPI2 has only one
 * device, but follow standard NuttX MTD driver convention).
 ****************************************************************************/

static void w25q256_lock(FAR struct w25q256_dev_s *priv)
{
  SPI_LOCK(priv->spi, true);
  SPI_SETMODE(priv->spi, W25Q256_SPI_MODE);
  SPI_SETBITS(priv->spi, W25Q256_SPI_NBITS);
  SPI_HWFEATURES(priv->spi, 0);
  SPI_SETFREQUENCY(priv->spi, W25Q256_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: w25q256_unlock
 ****************************************************************************/

static void w25q256_unlock(FAR struct w25q256_dev_s *priv)
{
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: w25q256_send_addr4
 *
 * Transmit a 4-byte address (MSB first) immediately after a command byte.
 ****************************************************************************/

static void w25q256_send_addr4(FAR struct w25q256_dev_s *priv,
                               uint32_t address)
{
  SPI_SEND(priv->spi, (address >> 24) & 0xff);
  SPI_SEND(priv->spi, (address >> 16) & 0xff);
  SPI_SEND(priv->spi, (address >>  8) & 0xff);
  SPI_SEND(priv->spi,  address        & 0xff);
}

/****************************************************************************
 * Name: w25q256_write_enable
 ****************************************************************************/

static void w25q256_write_enable(FAR struct w25q256_dev_s *priv)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_WREN);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
}

/****************************************************************************
 * Name: w25q256_wait_busy
 *
 * Poll Status Register-1 until BUSY clears or until timeout_us elapses.
 * Returns OK on success, -ETIMEDOUT on timeout.
 ****************************************************************************/

static int w25q256_wait_busy(FAR struct w25q256_dev_s *priv,
                             uint32_t timeout_us)
{
  uint32_t elapsed = 0;
  uint8_t  status;

  for (;;)
    {
      SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
      SPI_SEND(priv->spi, W25Q256_CMD_RDSR1);
      status = SPI_SEND(priv->spi, 0xff);
      SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

      if ((status & W25Q256_SR_BUSY) == 0)
        {
          return OK;
        }

      if (elapsed >= timeout_us)
        {
          ferr("ERROR: W25Q256 BUSY timeout (status=0x%02x)\n", status);
          return -ETIMEDOUT;
        }

      nxsig_usleep(W25Q256_BUSY_POLL_USEC);
      elapsed += W25Q256_BUSY_POLL_USEC;
    }
}

/****************************************************************************
 * Name: w25q256_read_jedec
 ****************************************************************************/

static int w25q256_read_jedec(FAR struct w25q256_dev_s *priv,
                              FAR uint32_t *jedec)
{
  uint8_t mfr;
  uint8_t type;
  uint8_t cap;

  /* Make sure the device is awake (Release Power-Down).  No-op if already
   * powered up; harmless either way.
   */

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_RDP);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
  nxsig_usleep(50);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_JEDEC_ID);
  mfr  = SPI_SEND(priv->spi, 0xff);
  type = SPI_SEND(priv->spi, 0xff);
  cap  = SPI_SEND(priv->spi, 0xff);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  *jedec = ((uint32_t)mfr << 16) | ((uint32_t)type << 8) | cap;
  return OK;
}

/****************************************************************************
 * Name: w25q256_erase_sector
 ****************************************************************************/

static int w25q256_erase_sector(FAR struct w25q256_dev_s *priv,
                                uint32_t address)
{
  int ret;

  ret = w25q256_wait_busy(priv, W25Q256_ERASE_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  w25q256_write_enable(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_SE4);
  w25q256_send_addr4(priv, address);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  return w25q256_wait_busy(priv, W25Q256_ERASE_TIMEOUT_US);
}

/****************************************************************************
 * Name: w25q256_chip_erase
 ****************************************************************************/

static int w25q256_chip_erase(FAR struct w25q256_dev_s *priv)
{
  int ret;

  ret = w25q256_wait_busy(priv, W25Q256_ERASE_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  w25q256_write_enable(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_CE);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  return w25q256_wait_busy(priv, W25Q256_BULK_TIMEOUT_US);
}

/****************************************************************************
 * Name: w25q256_program_page
 *
 * Program 1..256 bytes within a single page.  Caller must ensure that
 * (address & 0xff) + nbytes <= 256 so the program does not wrap.
 ****************************************************************************/

static int w25q256_program_page(FAR struct w25q256_dev_s *priv,
                                uint32_t address,
                                FAR const uint8_t *buffer,
                                size_t nbytes)
{
  int ret;

  DEBUGASSERT(nbytes > 0 && nbytes <= W25Q256_PAGE_SIZE);
  DEBUGASSERT(((address & (W25Q256_PAGE_SIZE - 1)) + nbytes)
              <= W25Q256_PAGE_SIZE);

  ret = w25q256_wait_busy(priv, W25Q256_PROGRAM_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  w25q256_write_enable(priv);

  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_PP4);
  w25q256_send_addr4(priv, address);
  SPI_SNDBLOCK(priv->spi, buffer, nbytes);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);

  return w25q256_wait_busy(priv, W25Q256_PROGRAM_TIMEOUT_US);
}

/****************************************************************************
 * Name: w25q256_read_data
 *
 * Issue Fast Read 4-byte (0x0C) and stream nbytes into buffer.  The Fast
 * Read command auto-increments the internal address counter and crosses
 * page/sector/block boundaries transparently.
 ****************************************************************************/

static void w25q256_read_data(FAR struct w25q256_dev_s *priv,
                              uint32_t address,
                              FAR uint8_t *buffer,
                              size_t nbytes)
{
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), true);
  SPI_SEND(priv->spi, W25Q256_CMD_FAST_READ4);
  w25q256_send_addr4(priv, address);
  SPI_SEND(priv->spi, 0xff);   /* dummy byte */
  SPI_RECVBLOCK(priv->spi, buffer, nbytes);
  SPI_SELECT(priv->spi, SPIDEV_FLASH(0), false);
}

/****************************************************************************
 * MTD methods
 ****************************************************************************/

/****************************************************************************
 * Name: w25q256_mtd_erase
 ****************************************************************************/

static int w25q256_mtd_erase(FAR struct mtd_dev_s *dev,
                             off_t startblock, size_t nblocks)
{
  FAR struct w25q256_dev_s *priv = (FAR struct w25q256_dev_s *)dev;
  size_t i;
  int    ret = OK;

  finfo("startblock=%jd nblocks=%zu\n", (intmax_t)startblock, nblocks);

  if ((size_t)startblock + nblocks > W25Q256_NSECTORS)
    {
      return -EINVAL;
    }

  w25q256_lock(priv);

  for (i = 0; i < nblocks; i++)
    {
      uint32_t addr = (uint32_t)(startblock + i) << W25Q256_SECTOR_SHIFT;

      ret = w25q256_erase_sector(priv, addr);
      if (ret < 0)
        {
          ferr("ERROR: erase_sector @0x%08" PRIx32 " failed: %d\n",
               addr, ret);
          break;
        }
    }

  w25q256_unlock(priv);

  return (ret < 0) ? ret : (int)nblocks;
}

/****************************************************************************
 * Name: w25q256_mtd_bread
 ****************************************************************************/

static ssize_t w25q256_mtd_bread(FAR struct mtd_dev_s *dev,
                                 off_t startblock, size_t nblocks,
                                 FAR uint8_t *buffer)
{
  FAR struct w25q256_dev_s *priv = (FAR struct w25q256_dev_s *)dev;
  uint32_t address;
  size_t   nbytes;

  /* Block size for bread/bwrite == page size (256 B) so that the MTD
   * partition layer aligns reads/writes on programmable page boundaries.
   * However LittleFS uses MTDIOC_GEOMETRY to learn the geometry and works
   * in those units, so we keep this consistent.
   */

  if ((size_t)startblock + nblocks
      > (W25Q256_CHIP_SIZE / W25Q256_PAGE_SIZE))
    {
      return -EINVAL;
    }

  address = (uint32_t)startblock << W25Q256_PAGE_SHIFT;
  nbytes  = nblocks << W25Q256_PAGE_SHIFT;

  w25q256_lock(priv);
  w25q256_read_data(priv, address, buffer, nbytes);
  w25q256_unlock(priv);

  return (ssize_t)nblocks;
}

/****************************************************************************
 * Name: w25q256_mtd_bwrite
 ****************************************************************************/

static ssize_t w25q256_mtd_bwrite(FAR struct mtd_dev_s *dev,
                                  off_t startblock, size_t nblocks,
                                  FAR const uint8_t *buffer)
{
  FAR struct w25q256_dev_s *priv = (FAR struct w25q256_dev_s *)dev;
  uint32_t address;
  size_t   i;
  int      ret = OK;

  if ((size_t)startblock + nblocks
      > (W25Q256_CHIP_SIZE / W25Q256_PAGE_SIZE))
    {
      return -EINVAL;
    }

  w25q256_lock(priv);

  for (i = 0; i < nblocks; i++)
    {
      address = (uint32_t)(startblock + i) << W25Q256_PAGE_SHIFT;

      ret = w25q256_program_page(priv, address,
                                 buffer + (i << W25Q256_PAGE_SHIFT),
                                 W25Q256_PAGE_SIZE);
      if (ret < 0)
        {
          ferr("ERROR: program_page @0x%08" PRIx32 " failed: %d\n",
               address, ret);
          break;
        }
    }

  w25q256_unlock(priv);

  return (ret < 0) ? ret : (ssize_t)nblocks;
}

/****************************************************************************
 * Name: w25q256_mtd_read
 ****************************************************************************/

static ssize_t w25q256_mtd_read(FAR struct mtd_dev_s *dev,
                                off_t offset, size_t nbytes,
                                FAR uint8_t *buffer)
{
  FAR struct w25q256_dev_s *priv = (FAR struct w25q256_dev_s *)dev;

  if ((uint64_t)offset + nbytes > W25Q256_CHIP_SIZE)
    {
      return -EINVAL;
    }

  w25q256_lock(priv);
  w25q256_read_data(priv, (uint32_t)offset, buffer, nbytes);
  w25q256_unlock(priv);

  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: w25q256_mtd_ioctl
 ****************************************************************************/

static int w25q256_mtd_ioctl(FAR struct mtd_dev_s *dev,
                             int cmd, unsigned long arg)
{
  FAR struct w25q256_dev_s *priv = (FAR struct w25q256_dev_s *)dev;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
              (FAR struct mtd_geometry_s *)arg;

          if (geo == NULL)
            {
              ret = -EINVAL;
              break;
            }

          memset(geo, 0, sizeof(*geo));
          geo->blocksize    = W25Q256_PAGE_SIZE;     /* 256 B */
          geo->erasesize    = W25Q256_SECTOR_SIZE;   /* 4 KB  */
          geo->neraseblocks = W25Q256_NSECTORS;      /* 8192 (chip)  */
          strlcpy(geo->model, "W25Q256", sizeof(geo->model));

          ret = OK;
        }
        break;

      case BIOC_PARTINFO:
        {
          /* Reported when partitioning code or LittleFS asks. */

          FAR struct partition_info_s *info =
              (FAR struct partition_info_s *)arg;

          if (info == NULL)
            {
              ret = -EINVAL;
              break;
            }

          memset(info, 0, sizeof(*info));
          info->numsectors  = W25Q256_NSECTORS;
          info->sectorsize  = W25Q256_SECTOR_SIZE;
          info->startsector = 0;
          strlcpy(info->parent, "", sizeof(info->parent));

          ret = OK;
        }
        break;

      case MTDIOC_ERASESECTORS:
        {
          FAR struct mtd_erase_s *erase =
              (FAR struct mtd_erase_s *)arg;

          if (erase == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = w25q256_mtd_erase(dev, (off_t)erase->startblock,
                                       erase->nblocks);
          if (ret > 0)
            {
              /* MTDIOC_ERASESECTORS returns 0 on success */

              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          w25q256_lock(priv);
          ret = w25q256_chip_erase(priv);
          w25q256_unlock(priv);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;

          if (result == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *result = W25Q256_ERASED_STATE;
          ret = OK;
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: w25q256_initialize
 *
 * Allocate and initialize a W25Q256 MTD device on the given SPI bus.
 * Verifies the JEDEC ID; returns NULL if the chip is not present or not
 * the expected W25Q256JV.
 ****************************************************************************/

static FAR struct mtd_dev_s *
w25q256_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct w25q256_dev_s *priv;
  uint32_t jedec = 0;
  int      ret;

  priv = kmm_zalloc(sizeof(struct w25q256_dev_s));
  if (priv == NULL)
    {
      ferr("ERROR: kmm_zalloc failed\n");
      return NULL;
    }

  priv->spi = spi;

  priv->mtd.erase  = w25q256_mtd_erase;
  priv->mtd.bread  = w25q256_mtd_bread;
  priv->mtd.bwrite = w25q256_mtd_bwrite;
  priv->mtd.read   = w25q256_mtd_read;
  priv->mtd.ioctl  = w25q256_mtd_ioctl;
  priv->mtd.name   = "w25q256";

  /* Probe the chip */

  w25q256_lock(priv);
  ret = w25q256_read_jedec(priv, &jedec);
  w25q256_unlock(priv);

  if (ret < 0)
    {
      ferr("ERROR: JEDEC read failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  if (jedec != W25Q256_JEDEC_ID)
    {
      ferr("ERROR: Unexpected JEDEC ID 0x%06" PRIx32
           " (expected 0x%06x)\n", jedec, W25Q256_JEDEC_ID);
      kmm_free(priv);
      return NULL;
    }

  syslog(LOG_INFO, "W25Q256: JEDEC ID 0x%06" PRIx32 ", 32MB detected\n",
         jedec);

  return &priv->mtd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi2select / stm32_spi2status
 *
 * Board-supplied SPI2 chip-select and status callbacks.  Software NSS for
 * the W25Q256 only — toggle PB12 active-low when the SPI core asks to
 * select SPIDEV_FLASH(0).  Other devids are no-op (currently none).
 ****************************************************************************/

void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  if (devid == SPIDEV_FLASH(0))
    {
      stm32_gpiowrite(GPIO_W25_CS, !selected);
    }
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_w25q256_initialize
 *
 * Bring up the W25Q256: configure /CS, initialize SPI2, probe the chip,
 * carve a partition that excludes the LEGO-reserved 1 MB at the start,
 * register /dev/mtdblock0, and try to mount LittleFS at /mnt/flash.
 *
 * Failure is non-fatal — bringup continues with /mnt/flash absent.  The
 * full-chip raw device is not registered to keep the LEGO bootloader area
 * out of reach of user space.
 ****************************************************************************/

int stm32_w25q256_initialize(void)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd_full;
  FAR struct mtd_dev_s *mtd_part;
  int ret;

  /* /CS = PB12, idle HIGH */

  stm32_configgpio(GPIO_W25_CS);

  spi = stm32_spibus_initialize(2);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "W25Q256: stm32_spibus_initialize(2) failed\n");
      return -ENODEV;
    }

  mtd_full = w25q256_initialize(spi);
  if (mtd_full == NULL)
    {
      syslog(LOG_ERR, "W25Q256: chip not detected\n");
      return -ENODEV;
    }

  /* Carve out the FS partition (skip the first 1 MB).  The partition
   * layer takes block units measured in the parent MTD's blocksize
   * (which we report as 256 B page).
   */

  mtd_part = mtd_partition(mtd_full,
                           (off_t)W25Q256_FS_START_SECTOR
                               * (W25Q256_SECTOR_SIZE / W25Q256_PAGE_SIZE),
                           (off_t)W25Q256_FS_NSECTORS
                               * (W25Q256_SECTOR_SIZE / W25Q256_PAGE_SIZE));
  if (mtd_part == NULL)
    {
      syslog(LOG_ERR, "W25Q256: mtd_partition failed\n");
      return -ENOMEM;
    }

  ret = register_mtddriver(W25Q256_MTDBLOCK_PATH, mtd_part, 0644, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "W25Q256: register_mtddriver(%s) failed: %d\n",
             W25Q256_MTDBLOCK_PATH, ret);
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS
  /* Try to mount.  NuttX's "autoformat" only triggers on -EFAULT (CORRUPT),
   * but a freshly erased chip with no superblock at all returns -EINVAL
   * (LFS_ERR_INVAL from lfs_rawmount when the directory scan completes
   * without finding any superblock — see littlefs/lfs.c:4259).  Treat both
   * as "absent or unrecognized FS" and format on first boot.  Any other
   * error (notably -EIO from SPI) propagates without touching the chip.
   */

  ret = nx_mount(W25Q256_MTDBLOCK_PATH, W25Q256_MOUNT_POINT,
                 W25Q256_FS_TYPE, 0, NULL);
  if (ret == -EFAULT || ret == -EINVAL)
    {
      syslog(LOG_INFO,
             "W25Q256: no LittleFS detected (mount=%d), formatting...\n",
             ret);
      ret = nx_mount(W25Q256_MTDBLOCK_PATH, W25Q256_MOUNT_POINT,
                     W25Q256_FS_TYPE, 0, "forceformat");
    }

  if (ret < 0)
    {
      syslog(LOG_ERR,
             "W25Q256: LittleFS mount %s failed: %d (use "
             "'mount -t littlefs -o autoformat %s %s' to recover)\n",
             W25Q256_MOUNT_POINT, ret,
             W25Q256_MTDBLOCK_PATH, W25Q256_MOUNT_POINT);
      return ret;
    }

  syslog(LOG_INFO, "W25Q256: LittleFS mounted at %s\n",
         W25Q256_MOUNT_POINT);
#endif

  return OK;
}

#endif /* CONFIG_STM32_SPI2 */
