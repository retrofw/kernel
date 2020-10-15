/*
 * Copyright (C) 2010 zhiping zhong <zpzhong@ingenic.cn>, JZ
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef SSTEP_H
#define SSTEP_H
struct pt_regs;

/*
 * Only "eret" cann't single be execute?
 */
#define IS_SYNC(instr)  	(((instr) & 0xfc00003f) == 0x0f)
#define IS_ERET(instr)  	( (instr) == 0x42000018)

/* Emulate instructions that cause a transfer of control. */
extern int emulate_step(struct pt_regs *regs, unsigned int instr);
extern int test_step(unsigned int instr);
extern int insn_is_jmpbra(unsigned int instr);


#define __TEST_EMULATE_STEP

#ifdef __TEST_EMULATE_STEP
extern int test_emulate_step(void);
#else
static inline int test_emulate_step(void) {
	return 0;
}
#endif

#endif
