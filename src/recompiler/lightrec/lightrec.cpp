#include <lightrec.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "cdrom.h"
#include "gte.h"
#include "mdec.h"
#include "psxdma.h"
#include "psxhw.h"
#include "psxmem.h"
#include "r3000a.h"

//#include "frontend/main.h"

#define ARRAY_SIZE(x) (sizeof(x) ? sizeof(x) / sizeof((x)[0]) : 0)

#ifdef __GNUC__
#	define likely(x)       __builtin_expect(!!(x),1)
#	define unlikely(x)     __builtin_expect(!!(x),0)
#else
#	define likely(x)       (x)
#	define unlikely(x)     (x)
#endif

extern int lightrec_init_mmap();
extern void lightrec_free_mmap();

static struct lightrec_state *lightrec_state;

static char cache_control[512];
static char parallel_port[0x10000];

static char *name = "pcsx";

/* Unused for now */
u32 event_cycles[PSXINT_COUNT];
u32 next_interupt;
u32 cycle_multiplier;

bool use_lightrec_interpreter = false;
bool lightrec_debug = false;
u32 lightrec_begin_cycles = 0;

int lightrec_very_debug = 0;
int stop = 0;

enum my_cp2_opcodes {
	OP_CP2_RTPS		= 0x01,
	OP_CP2_NCLIP		= 0x06,
	OP_CP2_OP		= 0x0c,
	OP_CP2_DPCS		= 0x10,
	OP_CP2_INTPL		= 0x11,
	OP_CP2_MVMVA		= 0x12,
	OP_CP2_NCDS		= 0x13,
	OP_CP2_CDP		= 0x14,
	OP_CP2_NCDT		= 0x16,
	OP_CP2_NCCS		= 0x1b,
	OP_CP2_CC		= 0x1c,
	OP_CP2_NCS		= 0x1e,
	OP_CP2_NCT		= 0x20,
	OP_CP2_SQR		= 0x28,
	OP_CP2_DCPL		= 0x29,
	OP_CP2_DPCT		= 0x2a,
	OP_CP2_AVSZ3		= 0x2d,
	OP_CP2_AVSZ4		= 0x2e,
	OP_CP2_RTPT		= 0x30,
	OP_CP2_GPF		= 0x3d,
	OP_CP2_GPL		= 0x3e,
	OP_CP2_NCCT		= 0x3f,
	OP_LAST			= 0x40,
};

typedef void (*cp2_fn)(u32);
static cp2_fn cp2_ops[OP_LAST];

static void init_cp2_ops()
{
	cp2_ops[OP_CP2_RTPS] = (cp2_fn)gteRTPS;
	cp2_ops[OP_CP2_RTPS] = (cp2_fn)gteRTPS;
	cp2_ops[OP_CP2_NCLIP] = (cp2_fn)gteNCLIP;
	cp2_ops[OP_CP2_OP] = (cp2_fn)gteOP;
	cp2_ops[OP_CP2_DPCS] = (cp2_fn)gteDPCS;
	cp2_ops[OP_CP2_INTPL] = (cp2_fn)gteINTPL;
	cp2_ops[OP_CP2_MVMVA] = (cp2_fn)gteMVMVA;
	cp2_ops[OP_CP2_NCDS] = (cp2_fn)gteNCDS;
	cp2_ops[OP_CP2_CDP] = (cp2_fn)gteCDP;
	cp2_ops[OP_CP2_NCDT] = (cp2_fn)gteNCDT;
	cp2_ops[OP_CP2_NCCS] = (cp2_fn)gteNCCS;
	cp2_ops[OP_CP2_CC] = (cp2_fn)gteCC;
	cp2_ops[OP_CP2_NCS] = (cp2_fn)gteNCS;
	cp2_ops[OP_CP2_NCT] = (cp2_fn)gteNCT;
	cp2_ops[OP_CP2_SQR] = (cp2_fn)gteSQR;
	cp2_ops[OP_CP2_DCPL] = (cp2_fn)gteDCPL;
	cp2_ops[OP_CP2_DPCT] = (cp2_fn)gteDPCT;
	cp2_ops[OP_CP2_AVSZ3] = (cp2_fn)gteAVSZ3;
	cp2_ops[OP_CP2_AVSZ4] = (cp2_fn)gteAVSZ4;
	cp2_ops[OP_CP2_RTPT] = (cp2_fn)gteRTPT;
	cp2_ops[OP_CP2_GPF] = (cp2_fn)gteGPF;
	cp2_ops[OP_CP2_GPL] = (cp2_fn)gteGPL;
	cp2_ops[OP_CP2_NCCT] = (cp2_fn)gteNCCT;
}

static u32 cop0_mfc(struct lightrec_state *state, u8 reg)
{
	return psxRegs.CP0.r[reg];
}

static u32 cop2_mfc_cfc(struct lightrec_state *state, u8 reg, bool cfc)
{
	if (cfc)
		return psxRegs.CP2C.r[reg];
	else
		return gtecalcMFC2(reg);
}

static u32 cop2_mfc(struct lightrec_state *state, u8 reg)
{
	return cop2_mfc_cfc(state, reg, false);
}

static u32 cop2_cfc(struct lightrec_state *state, u8 reg)
{
	return cop2_mfc_cfc(state, reg, true);
}

static void cop0_mtc_ctc(struct lightrec_state *state,
			 u8 reg, u32 value, bool ctc)
{
	switch (reg) {
	case 1:
	case 4:
	case 8:
	case 14:
	case 15:
		/* Those registers are read-only */
		break;
	case 12: /* Status */
		psxRegs.CP0.n.Status = value;
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);
		break;
	case 13: /* Cause */
		psxRegs.CP0.n.Cause &= ~0x0300;
		psxRegs.CP0.n.Cause |= value & 0x0300;
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);
		break;
	default:
		psxRegs.CP0.r[reg] = value;
		break;
	}
}

static void cop2_mtc_ctc(struct lightrec_state *state,
			 u8 reg, u32 value, bool ctc)
{
	if (ctc)
		gtecalcCTC2(value, reg);
	else
		gtecalcMTC2(value, reg);
}

static void cop0_mtc(struct lightrec_state *state, u8 reg, u32 value)
{
	cop0_mtc_ctc(state, reg, value, false);
}

static void cop0_ctc(struct lightrec_state *state, u8 reg, u32 value)
{
	cop0_mtc_ctc(state, reg, value, true);
}

static void cop2_mtc(struct lightrec_state *state, u8 reg, u32 value)
{
	cop2_mtc_ctc(state, reg, value, false);
}

static void cop2_ctc(struct lightrec_state *state, u8 reg, u32 value)
{
	cop2_mtc_ctc(state, reg, value, true);
}

static void cop0_op(struct lightrec_state *state, u32 func)
{
	fprintf(stderr, "Invalid access to COP0\n");
}

static void cop2_op(struct lightrec_state *state, u32 func)
{
	psxRegs.code = func;

	if (unlikely(!cp2_ops[func & 0x3f]))
		fprintf(stderr, "Invalid CP2 function %u\n", func);
	else
		cp2_ops[func & 0x3f](func >> 10);
}

static void hw_write_byte(struct lightrec_state *state,
		const struct opcode *op, u32 mem, u8 val)
{
	psxRegs.cycle = lightrec_current_cycle_count(state);

	psxHwWrite8(mem, val);

	if (!psxRegs.io_cycle_counter)
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);
}

static void hw_write_half(struct lightrec_state *state,
		const struct opcode *op, u32 mem, u16 val)
{
	psxRegs.cycle = lightrec_current_cycle_count(state);

	psxHwWrite16(mem, val);

	if (!psxRegs.io_cycle_counter)
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);
}

static void hw_write_word(struct lightrec_state *state,
		const struct opcode *op, u32 mem, u32 val)
{
	u32 old_cycles = lightrec_current_cycle_count(state);

	psxRegs.cycle = old_cycles;

	psxHwWrite32(mem, val);

	if (!psxRegs.io_cycle_counter)
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);

	/* Calling psxHwWrite32 might update psxRegs.cycle - Make sure
	 * here that state->current_cycle stays in sync. */
	lightrec_reset_cycle_count(state, psxRegs.cycle);
}

static u8 hw_read_byte(struct lightrec_state *state,
		const struct opcode *op, u32 mem)
{
	u8 val;

	psxRegs.cycle = lightrec_current_cycle_count(state);

	val = psxHwRead8(mem);

	if (!psxRegs.io_cycle_counter)
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);

	return val;
}

static u16 hw_read_half(struct lightrec_state *state,
		const struct opcode *op, u32 mem)
{
	u16 val;

	psxRegs.cycle = lightrec_current_cycle_count(state);

	val = psxHwRead16(mem);

	if (!psxRegs.io_cycle_counter)
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);

	return val;
}

static u32 hw_read_word(struct lightrec_state *state,
		const struct opcode *op, u32 mem)
{
	u32 val;

	psxRegs.cycle = lightrec_current_cycle_count(state);

	val = psxHwRead32(mem);

	if (!psxRegs.io_cycle_counter)
		lightrec_set_exit_flags(state, LIGHTREC_EXIT_CHECK_INTERRUPT);

	return val;
}

static struct lightrec_mem_map_ops hw_regs_ops = {
	.sb = hw_write_byte,
	.sh = hw_write_half,
	.sw = hw_write_word,
	.lb = hw_read_byte,
	.lh = hw_read_half,
	.lw = hw_read_word,
};

static struct lightrec_mem_map lightrec_map[] = {
	[PSX_MAP_KERNEL_USER_RAM] = {
		/* Kernel and user memory */
		.pc = 0x00000000,
		.length = 0x200000,
	},
	[PSX_MAP_BIOS] = {
		/* BIOS */
		.pc = 0x1fc00000,
		.length = 0x80000,
	},
	[PSX_MAP_SCRATCH_PAD] = {
		/* Scratch pad */
		.pc = 0x1f800000,
		.length = 0x400,
	},
	[PSX_MAP_PARALLEL_PORT] = {
		/* Parallel port */
		.pc = 0x1f000000,
		.length = sizeof(parallel_port),
		.address = &parallel_port,
	},
	[PSX_MAP_HW_REGISTERS] = {
		/* Hardware registers */
		.pc = 0x1f801000,
		.length = 0x2000,
		.ops = &hw_regs_ops,
	},
	[PSX_MAP_CACHE_CONTROL] = {
		/* Cache control */
		.pc = 0x5ffe0000,
		.length = sizeof(cache_control),
		.address = &cache_control,
	},

	/* Mirrors of the kernel/user memory */
	{
		.pc = 0x00200000,
		.length = 0x200000,
		.mirror_of = &lightrec_map[PSX_MAP_KERNEL_USER_RAM],
	},
	{
		.pc = 0x00400000,
		.length = 0x200000,
		.mirror_of = &lightrec_map[PSX_MAP_KERNEL_USER_RAM],
	},
	{
		.pc = 0x00600000,
		.length = 0x200000,
		.mirror_of = &lightrec_map[PSX_MAP_KERNEL_USER_RAM],
	},
};

static const struct lightrec_ops lightrec_ops = {
	.cop0_ops = {
		.mfc = cop0_mfc,
		.cfc = cop0_mfc,
		.mtc = cop0_mtc,
		.ctc = cop0_ctc,
		.op = cop0_op,
	},
	.cop2_ops = {
		.mfc = cop2_mfc,
		.cfc = cop2_cfc,
		.mtc = cop2_mtc,
		.ctc = cop2_ctc,
		.op = cop2_op,
	},
};

static int lightrec_plugin_init(void)
{
	int ret = lightrec_init_mmap();
	if (ret)
		return ret;

	psxP = (s8 *)parallel_port;

	psxM_allocated = true;
	psxP_allocated = true;
	psxH_allocated = true;
	psxR_allocated = true;

	init_cp2_ops();

	lightrec_map[PSX_MAP_KERNEL_USER_RAM].address = psxM;
	lightrec_map[PSX_MAP_BIOS].address = psxR;
	lightrec_map[PSX_MAP_SCRATCH_PAD].address = psxH;

	lightrec_state = lightrec_init(name,
			lightrec_map, ARRAY_SIZE(lightrec_map),
			&lightrec_ops);

	fprintf(stderr, "M=0x%lx, P=0x%lx, R=0x%lx, H=0x%lx\n",
			(uintptr_t) psxM,
			(uintptr_t) psxP,
			(uintptr_t) psxR,
			(uintptr_t) psxH);

	signal(SIGPIPE, exit);
	return 0;
}

static u32 hash_calculate(const void *buffer, u32 count)
{
	unsigned int i;
	u32 *data = (u32 *) buffer;
	u32 hash = 0xffffffff;

	count /= 4;
	for(i = 0; i < count; ++i) {
		hash += data[i];
		hash += (hash << 10);
		hash ^= (hash >> 6);
	}

	hash += (hash << 3);
	hash ^= (hash >> 11);
	hash += (hash << 15);
	return hash;
}

static void print_for_big_ass_debugger(void)
{
	unsigned int i;
	extern int lightrec_very_debug;

	printf("CYCLE 0x%08x PC 0x%08x", psxRegs.cycle, psxRegs.pc);

	if (lightrec_very_debug)
		printf(" RAM 0x%08x SCRATCH 0x%08x HW 0x%08x",
				hash_calculate(psxM, 0x200000),
				hash_calculate(psxH, 0x400),
				hash_calculate(psxH + 0x1000, 0x2000));

	printf(" CP0 0x%08x CP2D 0x%08x CP2C 0x%08x INT 0x%04x INTCYCLE 0x%08x",
			hash_calculate(&psxRegs.CP0.r,
				sizeof(psxRegs.CP0.r)),
			hash_calculate(&psxRegs.CP2D.r,
				sizeof(psxRegs.CP2D.r)),
			hash_calculate(&psxRegs.CP2C.r,
				sizeof(psxRegs.CP2C.r)),
			psxRegs.interrupt,
			hash_calculate(psxRegs.intCycle,
				sizeof(psxRegs.intCycle)));

	if (lightrec_very_debug)
		for (i = 0; i < 34; i++)
			printf(" GPR[%i] 0x%08x", i, psxRegs.GPR.r[i]);
	else
		printf(" GPR 0x%08x", hash_calculate(&psxRegs.GPR.r,
					sizeof(psxRegs.GPR.r)));
	printf("\n");
}

extern void intExecuteBlock(unsigned int);

static void lightrec_plugin_execute_block(unsigned int target_pc)
{
	u32 old_pc = psxRegs.pc;
	u32 flags;
	unsigned int target_cycle;

	if (target_pc || !psxRegs.io_cycle_counter)
		target_cycle = psxRegs.cycle;
	else
		target_cycle = psxRegs.io_cycle_counter;

	do {
		lightrec_restore_registers(lightrec_state, psxRegs.GPR.r);
		lightrec_reset_cycle_count(lightrec_state, psxRegs.cycle);

		if (use_lightrec_interpreter) {
			psxRegs.pc = lightrec_run_interpreter(lightrec_state,
							      psxRegs.pc);
		} else {
			psxRegs.pc = lightrec_execute(lightrec_state,
						      psxRegs.pc, target_cycle);
		}

		psxRegs.cycle = lightrec_current_cycle_count(lightrec_state);
		lightrec_dump_registers(lightrec_state, psxRegs.GPR.r);

		flags = lightrec_exit_flags(lightrec_state);

		if (flags & LIGHTREC_EXIT_CHECK_INTERRUPT)
			psxBranchTest();

		if (flags & LIGHTREC_EXIT_SYSCALL)
			psxException(0x20, 0);

		if (flags & LIGHTREC_EXIT_SEGFAULT) {
			fprintf(stderr, "Exiting at cycle 0x%08x\n",
					psxRegs.cycle);
			exit(1);
		}

	} while (psxRegs.cycle < target_cycle);

	psxBranchTest();

	if (lightrec_debug && psxRegs.cycle >= lightrec_begin_cycles
			&& psxRegs.pc != old_pc)
		print_for_big_ass_debugger();

	if ((psxRegs.CP0.n.Cause & psxRegs.CP0.n.Status & 0x300) &&
			(psxRegs.CP0.n.Status & 0x1)) {
		/* Handle software interrupts */
		psxRegs.CP0.n.Cause &= ~0x7c;
		psxException(psxRegs.CP0.n.Cause, 0);
	}
}

static void lightrec_plugin_execute(void)
{
	extern int stop;

	while (!stop)
		lightrec_plugin_execute_block(0);
}

static void lightrec_plugin_clear(u32 addr, u32 size)
{
	/* size * 4: PCSX uses DMA units */
	lightrec_invalidate(lightrec_state, addr, size * 4);
}

static void lightrec_plugin_shutdown(void)
{
	lightrec_destroy(lightrec_state);
}

static void lightrec_plugin_notify(int note, void *data)
{
}

static void lightrec_plugin_reset(void)
{
	/* At some point, the BIOS disables the writes to the RAM - every
	 * SB/SH/SW/etc pointing to the RAM won't have any effect.
	 * Since Lightrec does not emulate that, we just hack the BIOS here to
	 * jump above that code. */
	memset(psxR + 0x250, 0, 0x28);
	memset(psxR + 0x2a0, 0, 0x88);
	*(u32 *) (psxR + 0x320) = 0x240a1000;
	*(u32 *) (psxR + 0x324) = 0x240b0f80;

	memset(psxR + 0x1960, 0, 0x28);
	memset(psxR + 0x19b0, 0, 0x88);
	*(u32 *) (psxR + 0x1a30) = 0x240a1000;
	*(u32 *) (psxR + 0x1a34) = 0x240b0f80;
}

R3000Acpu psxRec =
{
	lightrec_plugin_init,
	lightrec_plugin_reset,
	lightrec_plugin_execute,
	lightrec_plugin_execute_block,
	lightrec_plugin_clear,
	lightrec_plugin_notify,
	lightrec_plugin_shutdown,
};
