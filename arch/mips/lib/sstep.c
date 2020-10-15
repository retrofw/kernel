/*
 * Single-step support.
 *
 * Copyright (C) 2010 zhiping zhong <zpzhong@ingenic.cn>, JZ
 * Base on a copy of powerpc/lib/sstep.c 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/ptrace.h>
#include <asm/inst.h>
#include <asm/sstep.h>
#include <asm/processor.h>

extern char system_call_common[];


/*
 * Emulate instructions that cause a transfer of control.
 * Returns -1 if the instruction is one that should not be stepped,
 * returns 0 if the step was not emulated, and need to single stepped,
 * returns 1 if the step was emulated, and need delayslot single stepped,
 * returns 2 if do not need to execute delayslot instruction,
 * such as a likely instruction but no branch.
 * such as break, eret and all cp0/float instructions.
 * Here only jmp/branch instruction was emulated.
 */
int __kprobes emulate_step(struct pt_regs *regs, unsigned int instr)
{
	unsigned int opcode, rs, rt, fun;
	unsigned long int imm;
	long int *gpr = (long int *)regs->regs;
	unsigned long *ugpr = regs->regs;
	union mips_instruction inst;

	int branch = 0, likely = 0;
	inst.word = instr;
	opcode = inst.r_format.opcode;
	rs  = inst.r_format.rs;
	rt  = inst.r_format.rt;
	switch (opcode) {
	case 0:
		fun = inst.r_format.func;
		/*
		 * jr rs , but mips16e instruction
		 * (jr.hb rs) not support 
		 */
		if(fun == 8) {
			if(inst.r_format.re == 16)
				return -1;
			regs->cp0_epc = ugpr[rs];
			return 1;
		}
		/*
		 * jalr rs , but mips16e instruction
		 * (jalr.hb rs) not support 
		 */
		if(fun == 9) {
			int rd = inst.r_format.rd;
			if(inst.r_format.re == 16)
				return -1;
			ugpr[rd] = regs->cp0_epc + 8;
			regs->cp0_epc = ugpr[rs];
			return 1;
		}
		/* syscall, break, sdbbp not support */
		if(fun > 11 && fun < 16)
			return -1;
		break;
	case 1:	/* b */
		fun = (instr >> 16) & 0x1f;
		switch(fun) {
		case 0: /* bltz */
			if(gpr[rs] < 0) branch = 1;
			goto normal;
		case 1: /* bgez */
			if(gpr[rs] >= 0) branch = 1;
			goto normal;
		case 2: /* bltzl */
			if(gpr[rs] < 0) branch = 1;
			goto likely;
		case 3: /* bgezl */
			if(gpr[rs] >= 0) branch = 1;
			goto likely;
		case 16: /* bltzal */
			if(gpr[rs] < 0) branch = 1;
			goto link;
		case 17: /* bgezal */
			if(gpr[rs] >= 0) branch = 1;
			goto link;
		case 18: /* bltzall */
			if(gpr[rs] < 0) branch = 1;
			goto likely_link;
		case 19: /* bgezall */
			if(gpr[rs] >= 0) branch = 1;
			goto likely_link;
		default: /* trap inst*/ 
			if(fun == 31) return -1; /* synci */
			return 0;
		}
	case 3: /* jal */
		regs->regs[31] = regs->cp0_epc + 8;
	case 2: /* j */
		imm = (instr << 2) & 0x0ffffffc;
		regs->cp0_epc &= 0xf0000000;
		regs->cp0_epc |= imm;
		return 1;
	case 4: /* beq */
		if(gpr[rs] == gpr[rt]) branch = 1;
		goto normal;
	case 5: /* bne */
		if(gpr[rs] != gpr[rt]) branch = 1;
		goto normal;
	case 6: /* blez */
		if(gpr[rs] <= 0) branch = 1;
		goto normal;
	case 7: /* bgtz */
		if(gpr[rs] >  0) branch = 1;
		goto normal;
	case 16: /* cp0 inst only support mfc0/mtc0 */
		if(rs == 0 || rs == 4)
			return 0;
	case 17: /* float inst not support */
		return -1;
		
	case 20: /* beql */
		if(gpr[rs] == gpr[rt]) branch = 1;
		goto likely;
	case 21: /* bnel */
		if(gpr[rs] != gpr[rt]) branch = 1;
		goto likely;
	case 22: /* blezl */
		if(gpr[rs] <= 0) branch = 1;
		goto likely;
	case 23: /* bgtzl */
		if(gpr[rs] >  0) branch = 1;
		goto likely;
	case 29: /* jalx */
		return -1;
	default:
		return 0;
	}
	return 0;

likely:
	likely = 1;
	goto normal;
likely_link:
	likely = 1;
link:
	if(branch)
		regs->regs[31] = regs->cp0_epc + 8;
normal:
	if(branch) {
		long int offset = (long int)((short)(instr & 0xffff));
		offset <<= 2;
		regs->cp0_epc += offset + 4;
		likely = 0;
	} else {
		regs->cp0_epc += 4;
		/* if likely and not branch, invalid delayslot */
		if (likely) regs->cp0_epc += 4;
	}
	
	/* emulate delayslot if branch */
	return 2 - branch;
}

int __kprobes test_step(unsigned int instr)
{
	struct pt_regs regs; 
	return emulate_step(&regs , instr);
}

/*
 * jmp instruction return 1, branch instruction return 2
 * other branch instruction return 3, not jmp/branch return 0
 */
int __kprobes insn_is_jmpbra(unsigned int instr)
{
	unsigned int opcode, rs, rt, fun;

	int type = 0;
	opcode = instr >> 26;
	rs  = (instr >> 21) & 0x1f;
	rt  = (instr >> 16) & 0x1f;
	switch (opcode) {
	case 0:
		fun = instr & 0x3f;
		if(fun == 9 || fun == 8)
			type = 1;
		break;
	case 1:	/* b */
		fun = (instr >> 16) & 0x1f;
		switch(fun) {
		case 0: /* bltz */
		case 1: /* bgez */
		case 2: /* bltzl */
		case 3: /* bgezl */
		case 16: /* bltzal */
		case 17: /* bgezal */
		case 18: /* bltzall */
		case 19: /* bgezall */
			type = 2;
			break;
		default: /* trap inst*/ 
			break;
		}
	case 3: /* jal */
	case 2: /* j */
		type = 1;
	case 4: /* beq */
	case 5: /* bne */
	case 6: /* blez */
	case 7: /* bgtz */
		type = 2;
		break;
	case 16: /* cp0 inst */
		fun = (instr >> 16) & 0x1f;
		if(fun == 8)  /* bc0f/t[l] */
			type = 3;
		break;
	case 17: /* float inst */
		fun = (instr >> 16) & 0x1f;
		/* bc1f/t[l] bc1any2/4 f/t*/
		if(fun == 8 || fun == 9 || fun == 10)  
			type = 3;
		break;
	case 18: /* cp2 inst */
		fun = (instr >> 16) & 0x1f;
		if(fun == 8)  /* bc2f/t[l] */
			type = 3;
		break;
		
	case 20: /* beql */
	case 21: /* bnel */
	case 22: /* blezl */
	case 23: /* bgtzl */
		type = 2;
		break;
	default:
		break;
	}
	return type;

}

#ifdef __TEST_EMULATE_STEP
enum {
	ltz = 8 + 1,
	lez = 8 + 3,
	eqz = 8 + 2,
	gez = 8 + 6,
	gtz = 8 + 4,
	eq = 2,
	ne = 5,
	any = 7,
};
struct test_st {
	int cmp_type;
	unsigned int inst;
	int link, likely;
	char *name;
};
#define B1(x) ((1<<26) | ((x)<<16))
#define B2(x) ( (x)<<26 )
struct test_st __initdata testb[] = {
	{ ltz, B1(0), 0, 0, "bltz"}, /* bltz */
	{ gez, B1(1), 0, 0, "bgez"}, /* bgez */
	{ ltz, B1(2), 0, 1, "bltzl"}, /* bltzl */
	{ gez, B1(3), 0, 1, "bgezl"}, /* bgezl */

	{ ltz, B1(16), 1, 0, "bltzal"}, /* bltzal */
	{ gez, B1(17), 1, 0, "bgezal"}, /* bgezal */
	{ ltz, B1(18), 1, 1, "bltzall"}, /* bltzall */
	{ gez, B1(19), 1, 1, "bgezall"}, /* bgezall */

	{ eq,  B2(4), 0, 0, "beq"}, /* beq */
	{ ne,  B2(5), 0, 0, "bne"}, /* bne */
	{ lez, B2(6), 0, 0, "blez"}, /* blez */
	{ gtz, B2(7), 0, 0, "bgtz"}, /* bgtz */

	{ eq,  B2(20), 0, 1, "beql"}, /* beql */
	{ ne,  B2(21), 0, 1, "bnel"}, /* bnel */
	{ lez, B2(22), 0, 1, "blezl"}, /* blezl */
	{ gtz, B2(23), 0, 1, "bgtzl"}, /* bgtzl */

	{ 0, 0, 0, 0 }
};
#define J1(x) (x)
#define J2(x) ( (x) << 26 )
struct test_st __initdata testj[] = {
	{ any,  J1(8), 0, 0, "jr"}, /* jr */
	{ any,  J1(9), 1, 0, "jalr"}, /* jalr */
	{ any,  J2(2), 0, 0, "j"}, /* j */
	{ any,  J2(3), 1, 0, "jal"}, /* jal */

	{ 0, 0, 0, 0 }
};
static inline int __init judge_cmp(int cmp_type,int vs,int vt)
{
	int cmp = 0;
	cmp += vs <  vt ? 1: 0;
	cmp += vs == vt ? 2: 0;
	cmp += vs >  vt ? 4: 0;
	return (cmp_type & 7) & cmp;
}

#define EPC (0x80001000)
int __init test_emulate_step(void)
{
	struct test_st *ptest = &testb[0];
	struct pt_regs regs; 
	union mips_instruction inst;
	int vals[3] = { -1, 0, 1};
	int rs = 5, rt = 7;
	int i, ret, jmp;
	
again:
	inst.word = ptest->inst;
	for(i=0;i<3;i++) {
		int vrs,vrt;
		vrs = vals[i];
		vrt = 0;
		regs.regs[0] = 0;
		regs.regs[31] = 0;
		regs.regs[rs] = (unsigned int)vrs;
		regs.regs[rt] = (unsigned int)vrt;
		regs.cp0_epc = EPC;

		if(inst.i_format.opcode != 1)
			inst.i_format.rt = rt;
		inst.i_format.rs = rs;
		inst.i_format.simmediate = vals[i];
		
		ret = emulate_step(&regs,inst.word);
		jmp = judge_cmp(ptest->cmp_type,vrs,vrt);
	
		if((jmp && ret != 1) || (!jmp && ret != 2)) {
			printk("[SSTEP]" "ret(%d) jmp(%d) err at %s %d\n"
			       ,ret,jmp,ptest->name,i);
			return -1;
		}
		if(jmp) {
			if(regs.cp0_epc != EPC +4 +vals[i]*4) {
				printk("[SSTEP]" "epc jmp err at %s %d\n"
				       ,ptest->name,i);
				return -1;
			}
		} else {
			int off = ptest->likely? 8: 4;
			if(regs.cp0_epc != EPC + off) {
				printk("[SSTEP]" "epc nojmp err at %s %d\n"
				       ,ptest->name,i);
				return -1;
			}
		}
		if((( ptest->link &&  jmp) && regs.regs[31] != EPC+8) || 
		   ((!ptest->link || !jmp) && regs.regs[31] != 0) )
		{
			printk("[SSTEP]" "ra link err at %s %d\n"
			       ,ptest->name,i);
			return -1;
		}

	}
	ptest ++;
	if(ptest->inst != 0)
		goto again;

#define TARGET (0x80020004)
	ptest = &testj[0];
	do {
		inst.word = ptest->inst;
		regs.regs[0] = 0;
		regs.regs[31] = 0;
		regs.cp0_epc = EPC;
		if(inst.j_format.opcode == 0) {
			inst.r_format.rs = rs;
			regs.regs[rs] = TARGET;
			if(inst.r_format.func == 9)
				inst.r_format.rd = 31;
		} else {
			inst.j_format.target = (TARGET & 0x03ffffff)>>2;
		}
		
		ret = emulate_step(&regs, inst.word);
		
		if(ret != 1) {
			printk("[SSTEP]" "ret err at %s\n",ptest->name);
			return -1;
		}
		if(regs.cp0_epc != TARGET) {
			printk("[SSTEP]" "epc jmp err at %s\n",ptest->name);
			return -1;
		}
		if((ptest->link && regs.regs[31] != EPC+8) || 
		   (!ptest->link && regs.regs[31] != 0) )
		{
			printk("[SSTEP]" "ra link err at %s\n",ptest->name);
			return -1;
		}
		ptest ++;
	} while(ptest->inst != 0);
	
	printk("[SSTEP]emulate_step() test passed.\n");
	return 0;
}

#endif
