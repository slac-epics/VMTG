#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include <vme64x.h>

#define DEBUG

/* read a 24-bit register composed of 3 bytes spaced by 4 */
uint32_t vme64CSRegRead24(VME64_Addr base, uint32_t off)
{
	return __vme64CSRegRead24(base,off);
}


/* read a 32-bit register composed of 4 bytes spaced by 4 */
uint32_t vme64CSRegRead32(VME64_Addr base, uint32_t off)
{
	return __vme64CSRegRead32(base,off);
}

/* read a 64-bit register composed of 8 bytes spaced by 4 */
uint64_t vme64CSRegRead64(VME64_Addr base, uint32_t off)
{
	return __vme64CSRegRead64(base,off);
}

/* write a 32-bit register composed of 4 bytes spaced by 4 */
void vme64CSRegWrite32(VME64_Addr base, uint32_t off, uint32_t val)
{
	__vme64CSRegWrite32(base, off, val);
}

uint32_t
vme64ReadADER(VME64_Addr base, uint32_t idx)
{
	return vme64CSRegRead32(base, VME64_CSR_OFF_ADER(idx));
}

void
vme64WriteADER(VME64_Addr base, uint32_t idx, uint32_t val)
{
	return vme64CSRegWrite32(base, VME64_CSR_OFF_ADER(idx), val);
}

uint32_t
vme64ReadADEM(VME64_Addr base, uint32_t idx)
{
	return vme64CSRegRead32(base, VME64_CR_OFF_ADEM(idx));
}

uint64_t
vme64ReadAMCAP(VME64_Addr base, uint32_t idx)
{
	return vme64CSRegRead64(base, VME64_CR_OFF_AMCAP(idx));
}

#ifdef DEBUG
int
vme64PrintAMCAP(VME64_Addr base, uint32_t idx)
{
	printf("AMCAP(%"PRIu32"): 0x%016"PRIx64"\n", idx, vme64ReadAMCAP(base,idx));
	return 0;
}
#endif

int
vme64SetupADER(VME64_Addr base, uint32_t idx, uint64_t addr, uint32_t am)
{
uint64_t cap;
uint64_t msk;
uint32_t adem;

	if (idx < 0 || idx > 7 ) {
		fprintf(stderr,"vme64SetupADER(): index %"PRIu32" out of range\n", idx);
		return -1;
	}

	if ( am > 63 ) {
		fprintf(stderr,"vme64SetupADER(): invalid AM requested\n");
		return -1;
	}

	cap = vme64ReadAMCAP(base,idx);
	if ( ! (cap & (1ULL<<am)) ) {
		fprintf(stderr,"vme64SetupADER(): requested AM 0x%02"PRIx32" not supported by function %"PRIu32"\n", am, idx);
		return -1;
	}

	adem = vme64ReadADEM(base,idx);
	msk  = adem & ~0xff;

	if ( adem & VME64_ADEM_EFM ) {
		fprintf(stderr,"vme64SetupADER(): Extra function mask not supported\n");
		return -1;
	}
	if ( adem & VME64_ADEM_EFD ) {
		fprintf(stderr,"vme64SetupADER(): Extra function decoder not supported\n");
		return -1;
	}
	if ( adem & VME64_ADEM_DFS ) {
		fprintf(stderr,"vme64SetupADER(): Dynamic function sizing not supported\n");
		return -1;
	}

	if ( ~msk & addr ) {
		fprintf(stderr,"vme64SetupADER(): requested address (0x%016"PRIx64")\n", addr);
		fprintf(stderr,"                  not on MASK       (0x%016"PRIx64") boundary\n", msk);
		return -1;
	}
	if ( adem & VME64_ADEM_FAF ) {
		if ( (vme64ReadADER(base, idx) & msk) != addr ) {
			fprintf(stderr,"vme64SetupADER(): Dynamic function sizing not supported\n");
			return -1;
		} else {
			return 0;
		}
	}

	vme64WriteADER(base, idx, addr | (am<<2));

	return 0;
}

uint32_t
vme64CRChecksum(VME64_Addr base, int quiet)
{
uint32_t off;
uint8_t  sum;
uint32_t len = 0;
uint8_t  w,sp;

	if (   'C' != vme64_in08(base, VME64_CR_OFF_C_ID)
		|| 'R' != vme64_in08(base, VME64_CR_OFF_R_ID) ) {
		if ( !quiet )
			fprintf(stderr,"'CR' ID not found -- is there a VME64 module at 0x%08"myPRIxPTR" ?\n", base);
		return -1;
	}

	len = vme64_in08(base, VME64_CR_OFF_LEN2);
	if ( len > 0x7 ) {
		if ( !quiet )
			fprintf(stderr,"CR Length MSB 0x%02"PRIx32" > 0x07", len);
		return -1;
	}
	len = (len<<8) | vme64_in08(base, VME64_CR_OFF_LEN1);
	len = (len<<8) | vme64_in08(base, VME64_CR_OFF_LEN0);

	for ( sum=0, off = VME64_CR_OFF_LEN2; len && off <= 0x7f ; off+=4, len--)
		sum += vme64_in08(base, off);

	/* Hmm - the standard says that e.g., for D32 access width every
	 * byte of the ROM is used. I would interpret that literally and
	 * count the (zero byte-0..byte2) locations into the ROM length
	 * but at least the VME digitizer does not.
	 */
	switch ( ( w = vme64_in08(base, VME64_CR_OFF_CRWD) ) ) {
		case VME64_CR_CRWD_D8_O:
			sp  = 4;
		break;

		case VME64_CR_CRWD_D8_EO:
			/* see above sp  = 2; */
			sp = 4;
		break;

		case VME64_CR_CRWD_D16:
		case VME64_CR_CRWD_D32:
			/* see above sp  = 1; */
			sp  = 4;
		break;

		default:
			fprintf(stderr,"Unsupported CR data access width 0x%02x"PRIx8"\n",w);
			return -1;
	}
	off = 0x7f + sp;
	while ( len ) {
		sum += vme64_in08(base, off);
		off += sp;
		len--;
	}
	return sum - vme64_in08(base,3);
}

/* Read serial-number info */
uint32_t
vme64ReadSN(VME64_Addr base)
{
VME64_Addr b,e;
int        sp   = -1;
uint32_t   rval = 0;
	b = base + __vme64CSRegRead24(base, VME64_CR_OFF_BEG_SN);
	e = base + __vme64CSRegRead24(base, VME64_CR_OFF_END_SN);
	/* Standard says that S/N should be on D08(O) - every fourth
	 * byte (address ends on 3,7,B or F).
	 * Use some heuristics if this rule is violated.
	 */
	if ( 3 == ( b & 3 ) && 3 == ( e & 3 ) && (int)(e-b) <= 3*4 ) {
		/* Ordinary case with S/N <= 4 bytes */
		sp = 4;
	} else if ( e-b <= 3 ) {
		/* They violate the rule but S/N is <= 4 bytes
		 * sitting there...
		 */
		sp = 1;
	}

	if ( -1 == sp ) {
		return (uint32_t)-1;
	}

	while ( b <= e ) {
		rval = (rval << 8) + vme64_in08(b, 0);
		b+=sp;
	}

	return rval;
}


int
vme64FindBoard(VME64_Addr base, int start_slot, uint32_t man_id, uint32_t brd_id)
{

	if ( start_slot < 1 )
		start_slot = 1;

	base += start_slot * VME64_CR_SPACING;

	while ( start_slot <= 21 ) {
		if (   'C' == vme64_in08(base, VME64_CR_OFF_C_ID)
			&& 'R' == vme64_in08(base, VME64_CR_OFF_R_ID) ) {
			if (    man_id == __vme64CSRegRead24(base, VME64_CR_OFF_MANID)
				 && brd_id == __vme64CSRegRead32(base, VME64_CR_OFF_BRDID) )
				return start_slot;
		}
		base += VME64_CR_SPACING;
		start_slot++;
	}

	return -1;
}
