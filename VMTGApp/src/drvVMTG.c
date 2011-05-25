/* $Id: drvVMTG.c,v 1.3 2011/05/19 15:01:10 strauman Exp $ */

/* VMTG driver */

#include <epicsExport.h>
#include <epicsInterrupt.h>
#include <devLib.h>
#include <vme64x.h>
#include <drvSup.h>
#include <devBusMapped.h>
#include <errlog.h>
#include <inttypes.h>

#include <basicIoOps.h>

#define  VMTG_MAN_ID	(('S'<<16) | ('L' << 8) | 'A')
#define  VMTG_BRD_ID    0x14407802

#define  VMTG_FUN       0 /* VMTG is function 0 */

#define  R32_HI_OFF     0
#define  R32_LO_OFF     2

uint16_t   vmtgLfsr = 0xace1;

unsigned       vmtgVect = 0;
int            vmtgSlot = 0;
VME64_Addr     vmtgCSRB = (VME64_Addr)0;
volatile void *vmtgBase = (volatile void*)0;

static int
vmtg32Rd(DevBusMappedPvt pvt, epicsUInt32 *pvalue, dbCommon *prec)
{
	*pvalue  = 0;
 	*pvalue |= (in_be16( pvt->addr + R32_HI_OFF ) << 16);
	*pvalue |= (in_be16( pvt->addr + R32_LO_OFF ) & 0xffff);
	return 0;
}

static DevBusMappedAccessRec vmtg32IO = {
	rd: vmtg32Rd,
	wr: 0
};

static int
vmtgCSR2Rd(DevBusMappedPvt pvt, epicsUInt32 *pvalue, dbCommon *prec)
{
	*pvalue = in_be16( pvt->addr );
	return 0;
}

static int
vmtgCSR2WrLo(DevBusMappedPvt pvt, epicsUInt32 value, dbCommon *prec)
{
epicsUInt16 oval;
	/* When writing low bits (write-1-to-clear) then we don't want to
	 * touch the high bits.
	 */
	oval  = in_be16( pvt->addr );
	oval  = (oval & 0xff00) | ( value & 0xff);
	out_be16( pvt->addr, oval );
	return 0;
}

static int
vmtgCSR2WrHi(DevBusMappedPvt pvt, epicsUInt32 value, dbCommon *prec)
{
	/* When writing hi bits ('normal' bits) then we don't want to
	 * set any of the low bits.
	 */
	out_be16( pvt->addr, (value & 0xff00) );
	return 0;
}

static DevBusMappedAccessRec vmtgCSR2Lo = {
	rd: vmtg32Rd,
	wr: vmtgCSR2WrLo
};

static DevBusMappedAccessRec vmtgCSR2Hi = {
	rd: vmtg32Rd,
	wr: vmtgCSR2WrHi
};

static uint16_t
vmtgRnd()
{
/* Primitive Galois LFSR (Wikipedia, 2011) */
uint16_t rval;
int      key;
	key = epicsInterruptLock();
		rval = vmtgLfsr = (vmtgLfsr>>1) ^ ( (- (vmtgLfsr&1)) & 0xb400);
	epicsInterruptUnlock( key );
	return rval;
}

long
vmtgInstallISR(void (*isr)(void* parm), void *parm)
{
int      i;
unsigned v;
long     rval;

	i = 0;
	do {
		v    = vmtgRnd() & 0xff;
		rval = devConnectInterruptVME( v, isr, parm );
	} while ( S_dev_vectorInUse == rval && i++ < 10 );
	
	return rval;
}


static long
vmtgReport(int interest_level)
{
	epicsPrintf("VMTG Driver, Release $Name:  $\n");
	if ( vmtgSlot < 1 ) {
		epicsPrintf("No VMTG card detected\n");
	} else {
		epicsPrintf("VMTG in slot #%i, @0x%"PRIxVME64A" (CSR @0x%"PRIxPTR"), IRQ Vector: %u\n",
		            vmtgSlot, (uintptr_t)vmtgBase, vmtgCSRB, vmtgVect);
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
printf("LSB: %u, base: 0x%p\n", lsb, vme_24_base);

	vmtgBase = vme_24_base;

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
	if ( vme64SetupADER( vmtgCSRB, VMTG_FUN, (uint32_t)vme_24_bus, VME_AM_STD_SUP_DATA ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to program address decoders for function 0\n");
		goto bail;
	}

	if ( 0 == devBusMappedRegister( "VMTG0", vmtgBase ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to register base address with devBusMapped\n");
		goto bail;
	}

	if ( 0 != devBusMappedRegisterIO( "vmtg32IO", &vmtg32IO ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to register vmtg32IO with devBusMapped\n");
		goto bail;
	}

	if ( 0 != devBusMappedRegisterIO( "vmtgCSR2Lo", &vmtgCSR2Lo ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to register vmtgCSR2Lo with devBusMapped\n");
		goto bail;
	}

	if ( 0 != devBusMappedRegisterIO( "vmtgCSR2Hi", &vmtgCSR2Hi ) ) {
		epicsPrintf("vmtgInit FAILED: Unable to register vmtgCSR2Hi with devBusMapped\n");
		goto bail;
	}

	vme64_out08( vmtgCSRB, VME64_CSR_OFF_BSET, VME64_CSR_BIT_MODENBL );
	
	rval     = 0;

#warning "Remove this test"
	*(volatile uint16_t*)vmtgBase = 2;


bail:

	return rval;
}


static drvet drvVMTG = {
	number:    2,
	report:    vmtgReport,
	init:      vmtgInit
};

epicsExportAddress( drvet, drvVMTG );
