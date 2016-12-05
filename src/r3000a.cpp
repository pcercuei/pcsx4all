/***************************************************************************
 *   Copyright (C) 2007 Ryan Schultz, PCSX-df Team, PCSX team              *
 *   schultz.ryan@gmail.com, http://rschultz.ath.cx/code.php               *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307 USA.            *
 ***************************************************************************/

/*
* R3000A CPU functions.
*/

//senquack - May 22 2016 NOTE: 
// These have all been updated to use new PSXINT_* interrupts enum and intCycle
// struct from PCSX Reloaded/Rearmed (much cleaner, no magic numbers)

#include "r3000a.h"
#include "cdrom.h"
#include "mdec.h"
#include "gte.h"
#include "psxevents.h"

PcsxConfig Config;
R3000Acpu *psxCpu=NULL;
psxRegisters psxRegs;
u32 BIAS=3; /* 2 */
int autobias=0;
u32 PSXCLK=33868800; /* 33.8688 Mhz */

//senquack - Adapted pcsxReARMed SPU to PCSX4ALL:
#ifdef spu_pcsxrearmed
//senquack - Added so spu_pcsxrearmed can read current cycle value. Some of its SPU*() functions
//           take a new "cycles" parameter, and because of circuluar header dependency problems
//           I must provide a simple pointer to the current cycle value, so the wrapped
//           functions can be passed it from anywhere in code.
const u32 * const psxRegs_cycle_valptr = &psxRegs.cycle;
#endif //spu_pcsxrearmed

int psxInit() {
	printf("Running PCSX Version %s (%s).\n", PACKAGE_VERSION, __DATE__);

#ifdef PSXREC
	#ifndef interpreter_none
	if (Config.Cpu == CPU_INTERPRETER) {
		psxCpu = &psxInt;
	} else
	#endif
	psxCpu = &psxRec;
#else
	psxCpu = &psxInt;
#endif

	if (psxMemInit() == -1) return -1;

	return psxCpu->Init();
}

void psxReset() {

	psxCpu->Reset();

	psxMemReset();

	memset(&psxRegs, 0, sizeof(psxRegs));

	psxRegs.pc = 0xbfc00000; // Start in bootstrap

	psxRegs.psxM = psxM;	// PSX Memory
	psxRegs.psxP = psxP;	// PSX Memory
	psxRegs.psxR = psxR;	// PSX Memory
	psxRegs.psxH = psxH;	// PSX Memory

	psxRegs.CP0.r[12] = 0x10900000; // COP0 enabled | BEV = 1 | TS = 1
	psxRegs.CP0.r[15] = 0x00000002; // PRevID = Revision ID, same as R3000A

	// Queue up any events that always get scheduled (e.g., SPU update)
	psxEventQueue.clear_internal_queue();
	psxEventQueue.schedule_persistent_events();

	psxHwReset();
	psxBiosInit();

	if (!Config.HLE)
		psxExecuteBios();
}

void psxShutdown() {
	psxMemShutdown();
	psxBiosShutdown();

	psxCpu->Shutdown();
}

void psxException(u32 code, u32 bd) {
	// Set the Cause, preserving R/W 'interrupt pending field' bits 8,9
	// (From Notaz's PCSX Rearmed)
	psxRegs.CP0.n.Cause = (psxRegs.CP0.n.Cause & 0x300) | code;

	// Set the EPC & PC
	if (bd) {
		psxRegs.CP0.n.Cause|= 0x80000000;
		psxRegs.CP0.n.EPC = (psxRegs.pc - 4);
	} else
		psxRegs.CP0.n.EPC = (psxRegs.pc);

	if (psxRegs.CP0.n.Status & 0x400000)
		psxRegs.pc = 0xbfc00180;
	else
		psxRegs.pc = 0x80000080;

	// Set the Status
	psxRegs.CP0.n.Status = (psxRegs.CP0.n.Status &~0x3f) |
						  ((psxRegs.CP0.n.Status & 0xf) << 2);

	if (!Config.HLE && (((PSXMu32(psxRegs.CP0.n.EPC) >> 24) & 0xfe) == 0x4a)) {
		// "hokuto no ken" / "Crash Bandicot 2" ... fix
		PSXMu32ref(psxRegs.CP0.n.EPC)&= SWAPu32(~0x02000000);
	}

	if (autobias)
		psxRegs.cycle += 11;
	if (Config.HLE) {
		psxBiosException();
	}
}

void psxBranchTest()
{
	//senquack - Do not rearrange the math here! Events' sCycle val can end up
	// negative (very large unsigned int) when a PSXINT_RESET_CYCLE_VAL event
	// resets psxRegs.cycle to 0 and subtracts the previous psxRegs.cycle value
	// from each event's sCycle value. If you were instead to test like this:
	// 'while ((psxRegs.cycle >= (psxRegs.intCycle[X].sCycle + psxRegs.intCycle[X].cycle)',
	// it could fail for events that were past-due at the moment of adjustment.
	while ((psxRegs.cycle - psxRegs.intCycle[PSXINT_NEXT_EVENT].sCycle) >=
			psxRegs.intCycle[PSXINT_NEXT_EVENT].cycle) {
		// After dispatching the most-imminent event, this will update
		//  the intCycle[PSXINT_NEXT_EVENT] element.
		psxEventQueue.dispatch_and_remove_front();
	}

	psxRegs.io_cycle_counter = psxRegs.intCycle[PSXINT_NEXT_EVENT].sCycle +
	                           psxRegs.intCycle[PSXINT_NEXT_EVENT].cycle;

	// Are one or more HW IRQ bits set in both their status and mask registers?
	if (psxHu32(0x1070) & psxHu32(0x1074)) {
		// Are both HW IRQ mask bit and IRQ master-enable bit set in CP0 status reg?
		if ((psxRegs.CP0.n.Status & 0x401) == 0x401) {
			psxException(0x400, 0);
		}

		// If CP0 SR value didn't allow a HW IRQ exception here, it is likely
		//  because a game is currently inside an exception handler.
		//  It is therefore important that the RFE 'return-from-exception'
		//  instruction resets psxRegs.io_cycle_counter to 0. This ensures that
		//  psxBranchTest() is called again as soon as possible so that any
		//  pending HW IRQs are handled.
	}
}

void psxExecuteBios() {
	while (psxRegs.pc != 0x80030000)
		psxCpu->ExecuteBlock(0x80030000);
}
