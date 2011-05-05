/* $Id$ */

/* VMTG driver */

#include <epicsExport.h>
#include <devLib.h>
#include <vme64x.h>
#include <drvSup.h>
#include <devBusMapped.h>
#include <errlog.h>
#include <inttypes.h>

#define  VMTG_MAN_ID	(('S'<<16) | ('L' << 8) | 'A')
#define  VMTG_BRD_ID    0x14407802

#define  VMTG_FUN       0 /* VMTG is function 0 */

unsigned   vmtgVect = 0;
int        vmtgSlot = 0;
VME64_Addr vmtgCSRB = (VME64_Addr)0;
VME64_Addr vmtgBase = (VME64_Addr)0;

static long
vmtgReport(int interest_level)
{
	epicsPrintf("VMTG Driver, Release $Name:$\n");
	if ( vmtgSlot < 1 ) {
		epicsPrintf("No VMTG card detected\n");
	} else {
		epicsPrintf("VMTG in slot #%i, @0x%"PRIxVME64A" (CSR @0x%"PRIxVME64A"), IRQ Vector: %u\n",
		            vmtgSlot, vmtgBase, vmtgCSRB, vmtgVect);
	}
	return 0;
}

static long
vmtgInit(void)
{
volatile void *vme_csr_base, *vme_24_base, *tmp;
uint32_t  vme_24_bus;
long      rval = -1;
uint32_t  adem;
int       lsb;

	/* Look-up CPU address of VME CSR space */
	if ( devBusToLocalAddr( atVMECSR, 0, &vme_csr_base) ) {
		epicsPrintf("vmtgInit FAILED: Unable to find VME CSR space of this board; cannot initialize VMTG driver\n");
		goto bail;
	}

	/* Scan VME64 for a board */
	vmtgSlot = vme64FindBoard( (VME64_Addr)vme_csr_base, 0, VMTG_MAN_ID, VMTG_BRD_ID );
	if ( vmtgSlot < 0 ) {
		epicsPrintf("vmtgInit FAILED: No VMTG Card found!\n");
		goto bail;
	}

	vmtgCSRB = (VME64_Addr)vme_csr_base + vmtgSlot * VME64_CR_SPACING;

	/* How much space do we need ? */
	adem = vme64ReadADEM( vmtgCSRB, VMTG_FUN );

	for ( lsb=8, adem>>=8; 0 == (adem & 1); lsb++ )
		adem >>= 1;

	/* Allocate a chunk of A24 addresses for this card */
	if ( devAllocAddress( "VMTG", atVMEA24, (1<<lsb), lsb, &vme_24_base ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to map %u bytes of VME A24 space\n", (1<<lsb));
		goto bail;
	}

	vmtgBase = (VME64_Addr)vme_24_base;

	/* Hmm - there is no devLocalToBusAddr() but we need the *bus* address
	 * for programming the ADER.
	 */

	if ( devBusToLocalAddr( atVMEA24, 0, &tmp ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to map VME A24(0)\n");
		goto bail;
	}

	vme_24_bus = vme_24_base - tmp;

	/* Make sure module is disabled */
	vme64_out08( vmtgCSRB, VME64_CSR_OFF_BCLR, VME64_CSR_BIT_MODENBL );

	/* Program ADER                 */
	vme64SetupADER( vmtgCSRB, VMTG_FUN, (uint32_t)vme_24_bus, VME_AM_STD_SUP_DATA );
	
	rval     = 0;

bail:

	return rval;
}


static drvet drvVMTG = {
	number:    2,
	report:    vmtgReport,
	init:      vmtgInit
};

epicsExportAddress( drvet, drvVMTG );
