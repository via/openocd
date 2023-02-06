/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "imp.h"
#include <target/algorithm.h>
#include <target/mips32.h>
#include <target/mips_m4k.h>

#define PIC32MZ_MANUF_ID	0x029

/* pic32mz memory locations */

#define PIC32MZ_PHYS_RAM			0x00000000
#define PIC32MZ_PHYS_PGM_FLASH		0x1D000000
#define PIC32MZ_PHYS_PERIPHERALS	0x1F800000
#define PIC32MZ_PHYS_BOOT_FLASH		0x1FC00000

/*
 * Translate Virtual and Physical addresses.
 * Note: These macros only work for KSEG0/KSEG1 addresses.
 */

#define Virt2Phys(v)	((v) & 0x1FFFFFFF)

/* pic32mz configuration register locations */

#define PIC32MZ_DEVCFG0		0xBFC0FFCC

/* pic32mz flash controller register locations */

#define PIC32MZ_NVMCON		0xBF800600
#define PIC32MZ_NVMCONCLR	0xBF800604
#define PIC32MZ_NVMCONSET	0xBF800608
#define PIC32MZ_NVMCONINV	0xBF80060C
#define NVMCON_WR		(1 << 15)
#define NVMCON_WREN		(1 << 14)
#define NVMCON_WRERR		(1 << 13)
#define NVMCON_LVDERR		(1 << 12)
#define NVMCON_OP_PFM_ERASE		0x7
#define NVMCON_OP_UPPER_PFM_ERASE		0x6
#define NVMCON_OP_LOWER_PFM_ERASE		0x6
#define NVMCON_OP_PAGE_ERASE	0x4
#define NVMCON_OP_ROW_PROG		0x3
#define NVMCON_OP_QUAD_PROG		0x2
#define NVMCON_OP_WORD_PROG		0x1
#define NVMCON_OP_NOP			0x0

#define PIC32MZ_NVMKEY		0xBF800610
#define PIC32MZ_NVMADDR		0xBF800620
#define PIC32MZ_NVMADDRCLR	0xBF800624
#define PIC32MZ_NVMADDRSET	0xBF800628
#define PIC32MZ_NVMADDRINV	0xBF80062C
#define PIC32MZ_NVMDATA0		0xBF800630
#define PIC32MZ_NVMDATA1		0xBF800640
#define PIC32MZ_NVMDATA2		0xBF800650
#define PIC32MZ_NVMDATA3		0xBF800660
#define PIC32MZ_NVMSRCADDR	0xBF800670

#define PIC32MZ_NVMPWP	0xBF800680
#define PIC32MZ_NVMBWP	0xBF800690

/* flash unlock keys */

#define NVMKEY1			0xAA996655
#define NVMKEY2			0x556699AA

struct pic32mz_flash_bank {
	bool probed;
};

/*
 * DEVID values as per PIC32MZ Flash Programming Specification Rev N
 */

static const struct pic32mz_devs_s {
	uint32_t devid;
	const char *name;
} pic32mz_devs[] = {
	{0x7213053, "2048EFH100"},
	{0x00000000, NULL}
};

/* flash bank pic32mz <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pic32mz_flash_bank_command)
{
	struct pic32mz_flash_bank *pic32mz_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pic32mz_info = malloc(sizeof(struct pic32mz_flash_bank));
	bank->driver_priv = pic32mz_info;

	pic32mz_info->probed = false;

	return ERROR_OK;
}

static uint32_t pic32mz_get_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, PIC32MZ_NVMCON, &status);

	return status;
}

static uint32_t pic32mz_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;

	/* wait for busy to clear */
	while (((status = pic32mz_get_flash_status(bank)) & NVMCON_WR) && (timeout-- > 0)) {
		LOG_DEBUG("status: 0x%" PRIx32, status);
		alive_sleep(1);
	}
	if (timeout <= 0)
		LOG_DEBUG("timeout: status: 0x%" PRIx32, status);

	return status;
}

static void pic32mz_unlock(struct target *target) {
	/* unlock flash registers */
	target_write_u32(target, PIC32MZ_NVMKEY, 0);
	target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY1);
	target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY2);
}

static int pic32mz_nvm_exec(struct flash_bank *bank, uint32_t op, uint32_t timeout)
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MZ_NVMCON, NVMCON_WREN | op);

  pic32mz_unlock(target);
	/* start operation */
	target_write_u32(target, PIC32MZ_NVMCONSET, NVMCON_WR);

	pic32mz_wait_status_busy(bank, timeout);

	/* lock flash registers */
	target_write_u32(target, PIC32MZ_NVMCONCLR, NVMCON_WREN);

  uint32_t status = pic32mz_get_flash_status(bank);

  /* WRERR will not clear itself unless an explicit NOP is sent */
  if (status & NVMCON_WRERR) {
    target_write_u32(target, PIC32MZ_NVMCON, NVMCON_WREN | NVMCON_OP_NOP);
    pic32mz_unlock(target);
    target_write_u32(target, PIC32MZ_NVMCONSET, NVMCON_WR);
    pic32mz_wait_status_busy(bank, timeout);
  }

	return status;
}

/* Use unlock sequence to write to NVMPWP */
static int pic32mz_nvm_write_nvmpwp(struct target *target, uint32_t nvmpwp)
{
	/* unlock flash registers */
  pic32mz_unlock(target);

	/* Unlock access to NVMPWP */
	target_write_u32(target, PIC32MZ_NVMPWP, nvmpwp);
	return ERROR_OK;
}

/* Use unlock sequence to write to NVMBWP */
static int pic32mz_nvm_write_nvmbwp(struct target *target, uint32_t nvmbwp)
{
	/* unlock flash registers */
  pic32mz_unlock(target);

	/* Unlock access to NVMPWP */
	target_write_u32(target, PIC32MZ_NVMBWP, nvmbwp);
	return ERROR_OK;
}

static int pic32mz_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;

	unsigned int s, num_pages;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}


	if (Virt2Phys(bank->base) == PIC32MZ_PHYS_BOOT_FLASH) {
		/* pgm flash */
    uint32_t nvmbwp;
    target_read_u32(target, PIC32MZ_NVMBWP, &nvmbwp);
    for (s = 0; s < bank->num_sectors; s++)
      bank->sectors[s].is_protected = 1;

    /* Upper program area is sectors 0-4 and NVMBWP bits 0-4 */
    for (int b = 0; b <= 4; b++)
      bank->sectors[b].is_protected = (nvmbwp & (1 << b)) ? 1 : 0;

    /* Lower program area is sectors 8-12 and NVMBWP bits 8-12 */
    for (int b = 8; b <= 12; b++)
      bank->sectors[b].is_protected = (nvmbwp & (1 << b)) ? 1 : 0;

	} else {
    uint32_t nvmpwp;
    target_read_u32(target, PIC32MZ_NVMPWP, &nvmpwp);
    num_pages = nvmpwp & 0xffffff;
    /* TODO improve this logic to account for other page sizes */
    if (nvmpwp & 0x3fff)
      num_pages += 1;
    for (s = 0; s < bank->num_sectors && s < num_pages; s++)
      bank->sectors[s].is_protected = 1;
    for (; s < bank->num_sectors; s++)
      bank->sectors[s].is_protected = 0;
	}

	return ERROR_OK;
}

static int pic32mz_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))
		&& (Virt2Phys(bank->base) == PIC32MZ_PHYS_PGM_FLASH)) {
		/* this will only erase the Program Flash (PFM), not the Boot Flash (BFM)
		 * we need to use the MTAP to perform a full erase */
		LOG_DEBUG("Erasing entire program flash");
		status = pic32mz_nvm_exec(bank, NVMCON_OP_PFM_ERASE, 50);
		if (status & NVMCON_WRERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		return ERROR_OK;
	}

	for (unsigned int i = first; i <= last; i++) {
		target_write_u32(target, PIC32MZ_NVMADDR, Virt2Phys(bank->base + bank->sectors[i].offset));

		status = pic32mz_nvm_exec(bank, NVMCON_OP_PAGE_ERASE, 10);

		if (status & NVMCON_WRERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pic32mz_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (Virt2Phys(bank->base) == PIC32MZ_PHYS_BOOT_FLASH) {
		/* pgm flash */
    uint32_t nvmbwp;
    target_read_u32(target, PIC32MZ_NVMBWP, &nvmbwp);
    
    for (unsigned int b = first; b <= last; b++) {
      /* Valid areas to set are the lower and upper areas */
      if ((b <= 4) ||
          ((b >= 8) && (b <= 12))) {
        if (set) {
          nvmbwp |= (1 << b);
        } else {
          nvmbwp &= ~(1 << b);
        }
      }
    }

    pic32mz_nvm_write_nvmbwp(target, nvmbwp);

	} else {
    uint32_t nvmpwp;
    target_read_u32(target, PIC32MZ_NVMPWP, &nvmpwp);
    /* MZ uses a water mark approach for protection.  Addresses below PWP are
     * protected.  We will always try to honor the full request, understanding
     * that it will probably protect/unprotect more than desired */
    pic32mz_nvm_write_nvmpwp(target, nvmpwp);
	}


	return ERROR_OK;
}

static int pic32mz_write_word(struct flash_bank *bank, uint32_t address, uint32_t word)
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MZ_NVMADDR, Virt2Phys(address));
	target_write_u32(target, PIC32MZ_NVMDATA0, word);

	return pic32mz_nvm_exec(bank, NVMCON_OP_WORD_PROG, 5);
}

static int pic32mz_write_quadword(struct flash_bank *bank, uint32_t address,
uint32_t values[4])
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MZ_NVMADDR, Virt2Phys(address));
	target_write_u32(target, PIC32MZ_NVMDATA0, values[3]);
	target_write_u32(target, PIC32MZ_NVMDATA1, values[2]);
	target_write_u32(target, PIC32MZ_NVMDATA2, values[1]);
	target_write_u32(target, PIC32MZ_NVMDATA3, values[0]);

	return pic32mz_nvm_exec(bank, NVMCON_OP_QUAD_PROG, 5);
}

static int pic32mz_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("writing to flash at address " TARGET_ADDR_FMT " at offset 0x%8.8" PRIx32
			" count: 0x%8.8" PRIx32 "", bank->base, offset, count);

	if (offset & 0x3) {
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

  bool quadword_aligned = (offset & 0xf) == 0;
  while (quadword_aligned && words_remaining > 4) {
    /* Use quad-word writes */
		uint32_t values[4];
		memcpy(values, buffer + bytes_written, sizeof(values));

		status = pic32mz_write_quadword(bank, address, values);
		LOG_INFO("write quadword at 0x%" PRIx32 " (status = 0x%08" PRIx32 ")", address, status);

		if (status & NVMCON_WRERR) {
			LOG_ERROR("Flash write error WRERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bytes_written += 16;
		words_remaining -= 4;
		address += 16;
  }

	while (words_remaining > 0) {
		uint32_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint32_t));

		status = pic32mz_write_word(bank, address, value);

		if (status & NVMCON_WRERR) {
			LOG_ERROR("Flash write error WRERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bytes_written += 4;
		words_remaining--;
		address += 4;
	}

	if (bytes_remaining) {
		uint32_t value = 0xffffffff;
		memcpy(&value, buffer + bytes_written, bytes_remaining);

		status = pic32mz_write_word(bank, address, value);

		if (status & NVMCON_WRERR) {
			LOG_ERROR("Flash write error WRERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int pic32mz_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32mz_flash_bank *pic32mz_info = bank->driver_priv;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int i;
	uint32_t num_pages = 0;
	uint32_t device_id;
	int page_size;

	pic32mz_info->probed = false;

	device_id = ejtag_info->idcode;
	LOG_INFO("device id = 0x%08" PRIx32 " (manuf 0x%03x dev 0x%04x, ver 0x%02x)",
			  device_id,
			  (unsigned)((device_id >> 1) & 0x7ff),
			  (unsigned)((device_id >> 12) & 0xffff),
			  (unsigned)((device_id >> 28) & 0xf));

	if (((device_id >> 1) & 0x7ff) != PIC32MZ_MANUF_ID) {
		LOG_WARNING("Cannot identify target as a PIC32MZ family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	page_size = 16384;

	if (Virt2Phys(bank->base) == PIC32MZ_PHYS_BOOT_FLASH) {
		/* 0x1FC00000: Boot flash size */
		/* fixed 12k boot bank - see comments above */
		num_pages = 0x74000;
	} else {
    num_pages = (2048 * 1024);
	}

	LOG_INFO("flash size = %" PRIu32 "kbytes", num_pages / 1024);

	free(bank->sectors);

	/* calculate numbers of pages */
	num_pages /= page_size;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < (int)num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	pic32mz_info->probed = true;

	return ERROR_OK;
}

static int pic32mz_auto_probe(struct flash_bank *bank)
{
	struct pic32mz_flash_bank *pic32mz_info = bank->driver_priv;
	if (pic32mz_info->probed)
		return ERROR_OK;
	return pic32mz_probe(bank);
}

static int pic32mz_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t device_id;
	int printed = 0, i;

	device_id = ejtag_info->idcode;

	if (((device_id >> 1) & 0x7ff) != PIC32MZ_MANUF_ID) {
		snprintf(buf, buf_size,
				 "Cannot identify target as a PIC32MZ family (manufacturer 0x%03x != 0x%03x)\n",
				 (unsigned)((device_id >> 1) & 0x7ff),
				 PIC32MZ_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (i = 0; pic32mz_devs[i].name != NULL; i++) {
		if (pic32mz_devs[i].devid == (device_id & 0x0fffffff)) {
			printed = snprintf(buf, buf_size, "PIC32MZ%s", pic32mz_devs[i].name);
			break;
		}
	}

	if (pic32mz_devs[i].name == NULL)
		printed = snprintf(buf, buf_size, "Unknown");

	buf += printed;
	buf_size -= printed;
	snprintf(buf, buf_size, " Ver: 0x%02x",
			(unsigned)((device_id >> 28) & 0xf));

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mz_handle_pgm_word_command)
{
	uint32_t address, value;
	int status, res;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 2, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (address < bank->base || address >= (bank->base + bank->size)) {
		command_print(CMD, "flash address '%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	res = ERROR_OK;
	status = pic32mz_write_word(bank, address, value);
	if (status & NVMCON_WRERR)
		res = ERROR_FLASH_OPERATION_FAILED;
	if (status & NVMCON_LVDERR)
		res = ERROR_FLASH_OPERATION_FAILED;

	if (res == ERROR_OK)
		command_print(CMD, "pic32mz pgm word complete");
	else
		command_print(CMD, "pic32mz pgm word failed (status = 0x%x)", status);

	return ERROR_OK;
}


static const struct command_registration pic32mz_exec_command_handlers[] = {
	{
		.name = "pgm_word",
		.usage = "<addr> <value> <bank>",
		.handler = pic32mz_handle_pgm_word_command,
		.mode = COMMAND_EXEC,
		.help = "program a word",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pic32mz_command_handlers[] = {
	{
		.name = "pic32mz",
		.mode = COMMAND_ANY,
		.help = "pic32mz flash command group",
		.usage = "",
		.chain = pic32mz_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pic32mz_flash = {
	.name = "pic32mz",
	.commands = pic32mz_command_handlers,
	.flash_bank_command = pic32mz_flash_bank_command,
	.erase = pic32mz_erase,
	.protect = pic32mz_protect,
	.write = pic32mz_write,
	.read = default_flash_read,
	.probe = pic32mz_probe,
	.auto_probe = pic32mz_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pic32mz_protect_check,
	.info = pic32mz_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
