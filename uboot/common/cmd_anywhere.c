/*
 * Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * (C) Copyright 2001
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * DS2460 Erpyto
 */
#include <common.h>
#include <command.h>

extern int write_anywhere(void);
extern void clear_anywhere(void);
static int do_anywhere(cmd_tbl_t *cmdtp, int flag, int argc,
		      char * const argv[])
{
  int ret;
  const char *cmd;

  if (argc < 2) {
    goto usage;
  }

  cmd = argv[1];

  if (strcmp(cmd, "write") == 0) {
    if (write_anywhere() == -1) {
      printf("you had encrypto this board\n");
    }
  } else if (strcmp(cmd, "erase") == 0) {
    clear_anywhere();
  } else {
    goto usage;
  }

	return CMD_RET_SUCCESS;

usage:
  return CMD_RET_USAGE;
}

U_BOOT_CMD(
	anywhere,	2,	1,	do_anywhere,
	"anywhere mode:",
	"write - write mac address to ds2460\n"
  "anywhere erase - erase mac address on ds2460"
);

